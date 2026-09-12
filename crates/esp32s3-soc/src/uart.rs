//! ESP32-S3 UART (0/1/2) peripheral model.
//!
//! Register layout per QEMU `include/hw/char/esp32_uart.h` +
//! `esp32s3_uart.h` (identical to the TRM UART chapter).
//!
//! Modeled behavior (all that P2 firmware needs):
//! - `UART_FIFO` write → byte appended to the TX output buffer (console).
//! - `UART_STATUS.TXFIFO_CNT` always 0 (FIFO drains instantly) so IDF's
//!   polling TX path never blocks.
//! - `UART_INT_RAW.TXFIFO_EMPTY` set after each TX (FIFO empty); `INT_CLR`
//!   write clears the corresponding RAW bit; `INT_ST = RAW & ENA`.
//! - RX: `inject_rx` pushes a byte into the RX FIFO; `INT_RXFIFO_FULL`
//!   latches when the count reaches CONF1.rxfifo_full_thrhd (level-style,
//!   re-armed by lowering the threshold under a pending count);
//!   `UART_FIFO` reads pop one byte (RAW drops when the FIFO empties);
//!   `UART_STATUS.rxfifo_cnt` (bits [9:0], the S3 layout per uart_struct.h —
//!   NOT classic-ESP32 [29:24]) mirrors the FIFO length for polling.
//!   `INT_RXFIFO_TOUT` fires via `tick` once the RX line has been idle for
//!   `rx_tout_thrhd` bit-times with data pending and `rx_tout_en` set
//!   (CONF1 bit 23, MEM_CONF bits [26:17]).
//! - Remaining registers are latched (writes stored, reads return stored
//!   value or reset value) so firmware configuration writes are harmless.
//! - RS485: `RS485_CONF` rs485_en + rs485tx_rx_en echoes TX into RX
//!   (half-duplex loopback); en-only mutes during TX (already the default).
//!   No LIN hardware exists on S3 (no `lin_*` in `uart_struct.h`; LIN is
//!   software over break detect) — nothing to model.

use alloc::collections::VecDeque;
use alloc::vec::Vec;

/// UART register offsets (TRM UART chapter / QEMU esp32_uart.h).
pub const UART_FIFO: u32 = 0x00;
pub const UART_INT_RAW: u32 = 0x04;
pub const UART_INT_ST: u32 = 0x08;
pub const UART_INT_ENA: u32 = 0x0C;
pub const UART_INT_CLR: u32 = 0x10;
pub const UART_CLKDIV: u32 = 0x14;
pub const UART_AUTOBAUD: u32 = 0x18;
pub const UART_STATUS: u32 = 0x1C;
pub const UART_CONF0: u32 = 0x20;
pub const UART_CONF1: u32 = 0x24;
pub const UART_LOWPULSE: u32 = 0x28;
pub const UART_HIGHPULSE: u32 = 0x2C;
pub const UART_RXD_CNT: u32 = 0x30;
pub const UART_MEM_CONF: u32 = 0x60;
pub const UART_MEM_RX_STATUS: u32 = 0x64;
pub const UART_DATE: u32 = 0x78;
// RS485 mode (TRM RS485_CONF_REG @ 0x4C / uart_struct.h rs485_conf_reg_t):
// rs485_en[0] selects RS485 mode, rs485tx_rx_en[3] lets the receiver hear
// the transmitter (echo); dl0/dl1 stop-bit delays and rx/tx_dly_num are
// timing-only here.
pub const UART_RS485_CONF: u32 = 0x4C;
const RS485_EN: u32 = 1 << 0;
const RS485_TX_RX_EN: u32 = 1 << 3;

// INT_RAW/ST/ENA/CLR bit positions (QEMU esp32_uart.h).
pub const INT_RXFIFO_FULL: u32 = 1 << 0;
pub const INT_TXFIFO_EMPTY: u32 = 1 << 1;
pub const INT_RXFIFO_OVF: u32 = 1 << 4;
pub const INT_RXFIFO_TOUT: u32 = 1 << 8;
pub const INT_TX_DONE: u32 = 1 << 14;

// UART_STATUS fields (S3 TRM UART_STATUS_REG @ 0x1C / uart_struct.h —
// NOTE this is NOT the classic-ESP32 layout: rxfifo_cnt = bits [9:0],
// txfifo_cnt = bits [25:16]; dsrn/ctsn/rxd/dtrn are modem-line levels).
// The esp-idf UART HAL reads the RX count with `extui status, 0, 10`, so
// these low bits must carry the count — a ST_UTX_OUT-style constant here
// (as on classic ESP32) poisons every length computation with 0x300.
const STATUS_RXFIFO_CNT_MASK: u32 = 0x3FF;
// Modem-line idle levels (TRM defaults): ctsn/rxd/dtrn/rtsn/txd high,
// dsrn low.
const STATUS_MODEM_IDLE: u32 = (1 << 14) | (1 << 15) | (1 << 29) | (1 << 30) | (1 << 31);
const STATUS_CTSN: u32 = 1 << 14;
const STATUS_RTSN: u32 = 1 << 30;
const STATUS_TXFIFO_CNT_SHIFT: u32 = 16;
const STATUS_TXFIFO_CNT_MASK: u32 = 0x3FF;
// UART_CONF0 flow-control bits (uart_reg.h): TX_FLOW_EN[15] gates the
// transmitter on CTSn; RX_FLOW_EN[22] drives RTSn from the RX level.
// LOOPBACK[14] (uart_reg.h loopback test mode) feeds every transmitted
// byte back into the receiver, like the RS485 echo but unconditional.
const CONF0_TX_FLOW_EN: u32 = 1 << 15;
const CONF0_RX_FLOW_EN: u32 = 1 << 22;
const CONF0_LOOPBACK: u32 = 1 << 14;
// UART_MEM_CONF RX_FLOW_THRHD[16:7]: RX level asserting RTSn (stop).
const MEM_CONF_RX_FLOW_THRHD_SHIFT: u32 = 7;
const MEM_CONF_RX_FLOW_THRHD_MASK: u32 = 0x3FF;
// CTS-edge interrupt (uart_reg.h CTS_CHG_INT_RAW bit 6).
pub const INT_CTS_CHG: u32 = 1 << 6;
// GPIO-matrix modem signals (gpio_sig_map.h): UnCTS_IN/RTS_OUT, n = 0..2.
pub const UART_CTS_SIG: [u32; 3] = [13, 16, 19];
pub const UART_RTS_SIG: [u32; 3] = [13, 16, 19];
// Hardware FIFO depth (uart_struct.h TX/RX size): held TX bytes cap here.
const FIFO_DEPTH: usize = 128;

// UART_CONF1 fields (TRM UART_CONF1_REG @ 0x24).
const CONF1_RX_TOUT_EN: u32 = 1 << 23;
const CONF1_FULL_THRHD_MASK: u32 = 0x3FF;
// UART_MEM_CONF fields (TRM UART_MEM_CONF_REG @ 0x60): rx_tout_thrhd [26:17].
const MEM_CONF_RX_TOUT_THRHD_SHIFT: u32 = 17;
const MEM_CONF_RX_TOUT_THRHD_MASK: u32 = 0x3FF;
// UART_CLKDIV fields (TRM UART_CLKDIV_REG @ 0x14): clkdiv [11:0] (+frag
// [23:20]/16); the APB (80 MHz) bit time in model ticks is ~clkdiv.
const CLKDIV_DIV_MASK: u32 = 0xFFF;
const CLKDIV_FRAG_SHIFT: u32 = 20;
const CLKDIV_FRAG_MASK: u32 = 0xF;

const REG_COUNT: usize = 0x80 / 4;

/// One UART instance.
pub struct Uart {
    /// Generic register bank; indexed by (offset/4).
    regs: [u32; REG_COUNT],
    /// Bytes emitted on the TX line (read by the host console).
    tx_out: Vec<u8>,
    /// Received-but-unread bytes (the RX FIFO).
    rx: VecDeque<u8>,
    /// Sticky RX-edge flag for light-sleep UART wakeup: silicon wakes on
    /// the RX start-bit edge, not the FIFO level, and the sleep-entry path
    /// resets the FIFO — so a byte injected before entry must still wake
    /// after the reset drains it. Set on every injected byte, consumed
    /// (taken) by the sleep evaluation.
    rx_edge: bool,
    /// TX bytes held by hardware flow control (CTSn high with TX_FLOW_EN);
    /// flushed to `tx_out` when CTS drops. Empty unless flow-controlled.
    tx_hold: VecDeque<u8>,
    /// CTSn input level (1 = stop TX when TX_FLOW_EN; reset pull-high).
    cts: u32,
    /// RX-idle ticks since the last received byte (RXFIFO_TOUT counter).
    tout_idle: u64,
}

impl Uart {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        // Modem lines idle (ctsn/rxd/dtrn high); TXFIFO_CNT = 0.
        regs[(UART_STATUS / 4) as usize] = STATUS_MODEM_IDLE;
        // TXFIFO_EMPTY/TX_DONE are LEVEL-style latches on real silicon: the
        // FIFO is empty and the transmitter idle from reset, so both RAW
        // bits read 1 until the first byte is written (TRM UART_INT_RAW).
        // IDF's uart driver relies on this: uart_enable_tx_intr() enables
        // TXFIFO_EMPTY and the ISR fires immediately to drain the driver's
        // TX ringbuffer (uart_tx_all -> xRingbufferSend -> enable -> ISR).
        regs[(UART_INT_RAW / 4) as usize] = INT_TXFIFO_EMPTY | INT_TX_DONE;
        // MEM_CONF reset default carries rx_tout_thrhd = 10 (TRM).
        regs[(UART_MEM_CONF / 4) as usize] = 10 << MEM_CONF_RX_TOUT_THRHD_SHIFT;
        // CONF1 reset defaults: rxfifo_full_thrhd = txfifo_empty_thrhd = 96.
        regs[(UART_CONF1 / 4) as usize] = 96 | (96 << 10);
        Self {
            regs,
            tx_out: Vec::new(),
            rx: VecDeque::new(),
            rx_edge: false,
            tx_hold: VecDeque::new(),
            cts: 1,
            tout_idle: 0,
        }
    }

    /// Transmitter may shift: flow control off, or CTSn low (go).
    fn can_tx(&self) -> bool {
        self.regs[(UART_CONF0 / 4) as usize] & CONF0_TX_FLOW_EN == 0 || self.cts == 0
    }

    /// Drive the CTSn input level (matrix U_CTSn, unrouted = pull-high).
    /// A change latches CTS_CHG; a drop to go flushes held TX bytes.
    pub fn set_cts(&mut self, level: u32) {
        let level = level & 1;
        if level != self.cts {
            self.cts = level;
            self.regs[(UART_INT_RAW / 4) as usize] |= INT_CTS_CHG;
            if level == 0 {
                self.flush_hold();
            }
        }
    }

    /// Move held TX bytes to the line (transmitter runs).
    fn flush_hold(&mut self) {
        if self.tx_hold.is_empty() {
            return;
        }
        // RS485 echo applies to flushed bytes exactly like direct writes.
        let rs485 = self.regs[(UART_RS485_CONF / 4) as usize];
        let echo = rs485 & (RS485_EN | RS485_TX_RX_EN) == (RS485_EN | RS485_TX_RX_EN)
            && self.regs[(UART_CONF0 / 4) as usize] & (1 << 8) == 0;
        let loopback = self.regs[(UART_CONF0 / 4) as usize] & CONF0_LOOPBACK != 0;
        while let Some(b) = self.tx_hold.pop_front() {
            self.tx_out.push(b);
            if echo {
                self.inject_rx(b);
            }
            if loopback {
                self.inject_rx(b);
            }
        }
        self.regs[(UART_INT_RAW / 4) as usize] |= INT_TXFIFO_EMPTY | INT_TX_DONE;
    }

    /// RTSn output level (0 = ready): asserted while RX sits below the
    /// MEM_CONF flow threshold when RX_FLOW_EN is set; idle high otherwise.
    /// Routed to the U_RTSn matrix signal by the SoC.
    pub fn rts_level(&self) -> u32 {
        if self.regs[(UART_CONF0 / 4) as usize] & CONF0_RX_FLOW_EN == 0 {
            return 1;
        }
        let thrhd = ((self.regs[(UART_MEM_CONF / 4) as usize] >> MEM_CONF_RX_FLOW_THRHD_SHIFT)
            & MEM_CONF_RX_FLOW_THRHD_MASK)
            .max(1);
        u32::from(self.rx.len() as u32 >= thrhd)
    }

    /// Live STATUS: stored register with the TX count, CTSn, RTSn, TXD
    /// overlaid (RX count is maintained in the stored word on inject/pop).
    fn status_live(&self) -> u32 {
        let mut v = self.regs[(UART_STATUS / 4) as usize];
        v = (v & !(STATUS_TXFIFO_CNT_MASK << STATUS_TXFIFO_CNT_SHIFT))
            | ((self.tx_hold.len() as u32).min(STATUS_TXFIFO_CNT_MASK) << STATUS_TXFIFO_CNT_SHIFT);
        if self.cts == 0 {
            v &= !STATUS_CTSN;
        } else {
            v |= STATUS_CTSN;
        }
        if self.rts_level() == 0 {
            v &= !STATUS_RTSN;
        } else {
            v |= STATUS_RTSN;
        }
        v | (1 << 31) // TXD idle high (instant shift, never mid-bit)
    }

    /// Drain the bytes this UART emitted (host console output).
    pub fn take_tx(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.tx_out)
    }

    /// Bytes queued (debug probe). Inline: hot console-drain fast path.
    #[inline]
    pub fn tx_len(&self) -> usize {
        self.tx_out.len()
    }

    /// Append host-generated console bytes (printf mailbox path).
    pub fn push_tx(&mut self, bytes: &[u8]) {
        self.tx_out.extend_from_slice(bytes);
    }

    /// Push one received byte into the RX FIFO (host console input).    /// Caps at the 128-byte hardware depth (`SOC_UART_FIFO_LEN`): the REPL
    /// ISR reads `rxfifo_cnt` bytes into a fixed 128B stack buffer, so the
    /// count must never exceed depth — a 150B burst smashed the ISR stack
    /// (input text landed in a length field -> runaway copy -> Guru).
    /// Real silicon drops overrun bytes the same way.
    pub fn inject_rx(&mut self, byte: u8) {
        // Any arrival is an RX edge (the only wire activity in the model).
        self.rx_edge = true;
        if self.rx.len() >= 128 {
            return;
        }
        self.rx.push_back(byte);
        self.tout_idle = 0;
        let regs = &mut self.regs;
        regs[(UART_STATUS / 4) as usize] = (regs[(UART_STATUS / 4) as usize]
            & !STATUS_RXFIFO_CNT_MASK)
            | ((self.rx.len() as u32) & STATUS_RXFIFO_CNT_MASK);
        regs[(UART_RXD_CNT / 4) as usize] = regs[(UART_RXD_CNT / 4) as usize].wrapping_add(1);
        // RXFIFO_FULL is level-gated on the CONF1 threshold (TRM: fires when
        // the FIFO count reaches rxfifo_full_thrhd); short bursts rely on
        // RXFIFO_TOUT instead.
        let thrhd = regs[(UART_CONF1 / 4) as usize] & CONF1_FULL_THRHD_MASK;
        if self.rx.len() as u32 >= thrhd.max(1) {
            regs[(UART_INT_RAW / 4) as usize] |= INT_RXFIFO_FULL;
        }
    }

    /// True while unread RX bytes sit in the FIFO (UART light-sleep
    /// wakeup samples this at sleep entry).
    pub fn rx_pending(&self) -> bool {
        !self.rx.is_empty()
    }

    /// Take the sticky RX-edge flag (light-sleep UART wakeup consumes the
    /// edge that arrived before the entry FIFO reset).
    pub fn take_rx_edge(&mut self) -> bool {
        core::mem::take(&mut self.rx_edge)
    }

    /// Pop up to `n` bytes from the RX FIFO (GDMA/UHCI IN path), keeping
    /// the STATUS count in sync like register-FIFO reads do.
    pub(crate) fn take_rx(&mut self, n: usize) -> Vec<u8> {
        let mut v = Vec::new();
        while v.len() < n {
            if let Some(b) = self.rx.pop_front() {
                v.push(b);
            } else {
                break;
            }
        }
        let regs = &mut self.regs;
        regs[(UART_STATUS / 4) as usize] = (regs[(UART_STATUS / 4) as usize]
            & !STATUS_RXFIFO_CNT_MASK)
            | ((self.rx.len() as u32) & STATUS_RXFIFO_CNT_MASK);
        v
    }

    /// APB bit time in model ticks from CLKDIV (≈ clkdiv at 80 MHz APB;
    /// falls back to the 115200-baud divisor when unprogrammed).
    fn bit_ticks(&self) -> u64 {
        let clkdiv = self.regs[(UART_CLKDIV / 4) as usize];
        let div = u64::from(clkdiv & CLKDIV_DIV_MASK)
            + u64::from((clkdiv >> CLKDIV_FRAG_SHIFT) & CLKDIV_FRAG_MASK) / 16;
        div.max(1)
    }

    /// Advance `cycles` ticks; latch INT_RXFIFO_TOUT once the RX line has
    /// been idle for rx_tout_thrhd bit-times with data pending and
    /// rx_tout_en set (TRM UART RXFIFO_TOUT). Fast path: nothing pending or
    /// the timeout disabled.
    pub fn tick(&mut self, cycles: u64) {
        if !self.tx_hold.is_empty() && self.can_tx() {
            self.flush_hold();
        }
        if self.rx.is_empty() {
            return;
        }
        if self.regs[(UART_CONF1 / 4) as usize] & CONF1_RX_TOUT_EN == 0 {
            return;
        }
        self.tout_idle += cycles;
        let thrhd = u64::from(
            (self.regs[(UART_MEM_CONF / 4) as usize] >> MEM_CONF_RX_TOUT_THRHD_SHIFT)
                & MEM_CONF_RX_TOUT_THRHD_MASK,
        );
        if self.tout_idle >= thrhd.max(1) * self.bit_ticks() {
            self.regs[(UART_INT_RAW / 4) as usize] |= INT_RXFIFO_TOUT;
            // Re-arm: with data still pending the timeout recurs each period.
            self.tout_idle = 0;
        }
    }

    /// Interrupt status = RAW & ENA (TRM UART_INT_ST); the peripheral
    /// interrupt line into the matrix asserts while this is non-zero.
    /// `TXFIFO_EMPTY` is overlaid level-style: it reads 1 while the TX
    /// FIFO sits below the CONF1 empty threshold (our TX drains instantly
    /// so it is always empty). Real silicon re-asserts it after an INT_CLR
    /// while the FIFO is empty; without this the IDF TX pump ISR never
    /// fires after driver init clears the reset latch and ringbuffered
    /// bytes (e.g. MicroPython `UART.write`) never reach the FIFO.
    pub fn int_raw_live(&self) -> u32 {
        let mut v = self.regs[(UART_INT_RAW / 4) as usize];
        let thrhd = (self.regs[(UART_CONF1 / 4) as usize] >> 10) & 0x3FF;
        if thrhd != 0 && self.tx_hold.is_empty() {
            v |= INT_TXFIFO_EMPTY;
        }
        v
    }

    pub fn int_st(&self) -> u32 {
        self.int_raw_live() & self.regs[(UART_INT_ENA / 4) as usize]
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            UART_FIFO => {
                // Pop one RX byte; the FIFO-full line drops once empty.
                let Some(b) = self.rx.pop_front() else {
                    return 0;
                };
                self.regs[(UART_STATUS / 4) as usize] = (self.regs[(UART_STATUS / 4) as usize]
                    & !STATUS_RXFIFO_CNT_MASK)
                    | ((self.rx.len() as u32) & STATUS_RXFIFO_CNT_MASK);
                if self.rx.is_empty() {
                    self.regs[(UART_INT_RAW / 4) as usize] &= !INT_RXFIFO_FULL;
                }
                b as u32
            }
            // Live STATUS (TX count, CTSn, RTSn, TXD overlaid).
            UART_STATUS => self.status_live(),
            // Interrupt status = RAW & ENA (TRM UART_INT_ST).
            UART_INT_ST => self.int_st(),
            // RAW shows the live level for TXFIFO_EMPTY (see int_raw_live).
            UART_INT_RAW => self.int_raw_live(),
            _ => self.regs[(offset / 4) as usize],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            UART_FIFO => {
                // TXD_BRK (CONF0 bit 8) sends a break — ignore for now.
                if self.regs[(UART_CONF0 / 4) as usize] & (1 << 8) == 0 {
                    if !self.can_tx() {
                        // Flow-controlled stop: hold in the TX FIFO (capped
                        // at hardware depth; silicon overrun drops the same
                        // way). EMPTY/DONE stay low until the flush.
                        if self.tx_hold.len() < FIFO_DEPTH {
                            self.tx_hold.push_back(value as u8);
                        }
                    } else {
                        self.tx_out.push(value as u8);
                        // RS485 echo (TRM RS485_CONF rs485tx_rx_en, bit 3): in
                        // RS485 mode with the echo bit set the receiver hears
                        // the transmitter (half-duplex loopback). Without the
                        // bit the receiver is muted during TX — which the model
                        // already satisfies (TX never echoes by default). Stop-
                        // bit delays (dl0/dl1) and signal delays (rx/tx_dly_num)
                        // are timing-only at instant-drain granularity (no-op);
                        // clash/parity/frm error interrupts need a real bus.
                        let rs485 = self.regs[(UART_RS485_CONF / 4) as usize];
                        if rs485 & (RS485_EN | RS485_TX_RX_EN) == (RS485_EN | RS485_TX_RX_EN) {
                            self.inject_rx(value as u8);
                        }
                        if self.regs[(UART_CONF0 / 4) as usize] & CONF0_LOOPBACK != 0 {
                            self.inject_rx(value as u8);
                        }
                        // Emitted bytes drain instantly: EMPTY + DONE latch.
                        // Held bytes leave both low until the CTS flush.
                        let raw = &mut self.regs[(UART_INT_RAW / 4) as usize];
                        *raw |= INT_TXFIFO_EMPTY | INT_TX_DONE;
                    }
                }
            }
            UART_INT_CLR => {
                // Writing 1 clears the corresponding RAW bit (TRM UART_INT_CLR).
                self.regs[(UART_INT_RAW / 4) as usize] &= !value;
            }
            UART_CONF1 => {
                self.regs[(UART_CONF1 / 4) as usize] = value;
                // Lowering the FULL threshold under a pending count latches
                // FULL (level-style, TRM UART_INT_RAW).
                let thrhd = value & CONF1_FULL_THRHD_MASK;
                if self.rx.len() as u32 >= thrhd.max(1) {
                    self.regs[(UART_INT_RAW / 4) as usize] |= INT_RXFIFO_FULL;
                }
            }
            // Auto-baud: EN=0 at reset and on write of 0 → clear RXD_CNT.
            UART_AUTOBAUD => {
                if value & 1 == 0 {
                    self.regs[(UART_RXD_CNT / 4) as usize] = 0;
                }
                self.regs[(UART_AUTOBAUD / 4) as usize] = value;
            }
            _ if offset.is_multiple_of(4) && offset < (REG_COUNT * 4) as u32 => {
                self.regs[(offset / 4) as usize] = value;
            }
            _ => {}
        }
    }
}

impl Default for Uart {
    fn default() -> Self {
        Self::new()
    }
}
