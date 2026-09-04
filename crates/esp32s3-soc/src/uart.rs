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
//! - RX: `inject_rx` pushes a byte into the RX FIFO and latches
//!   `INT_RXFIFO_FULL`; `UART_FIFO` reads pop one byte (RAW drops when the
//!   FIFO empties, level-style); `UART_STATUS[29:24]` mirrors the FIFO
//!   length for polling. `INT_RXFIFO_TOUT` fires via `tick` once the RX line
//!   has been idle for `rx_tout_thrhd` bit-times with data pending and
//!   `rx_tout_en` set (CONF1 bit 23, MEM_CONF bits [26:17]).
//! - Remaining registers are latched (writes stored, reads return stored
//!   value or reset value) so firmware configuration writes are harmless.

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

// INT_RAW/ST/ENA/CLR bit positions (QEMU esp32_uart.h).
pub const INT_RXFIFO_FULL: u32 = 1 << 0;
pub const INT_TXFIFO_EMPTY: u32 = 1 << 1;
pub const INT_RXFIFO_OVF: u32 = 1 << 4;
pub const INT_RXFIFO_TOUT: u32 = 1 << 8;
pub const INT_TX_DONE: u32 = 1 << 14;

// UART_STATUS fields (TRM UART chapter): FIFO_CNT (RX bytes) = bits
// [29:24], TXFIFO_CNT = bits [16:13], ST_UTX_OUT = bits [9:8] (idle high).
const STATUS_FIFO_CNT_MASK: u32 = 0x3F << 24;
const STATUS_ST_UTX_OUT: u32 = 0x3 << 8;

// UART_CONF1 fields (TRM UART_CONF1_REG @ 0x24).
const CONF1_RX_TOUT_EN: u32 = 1 << 23;
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
    /// RX-idle ticks since the last received byte (RXFIFO_TOUT counter).
    tout_idle: u64,
}

impl Uart {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        // ST_UTX_OUT = idle high (UART_STATUS[9:8]); TXFIFO_CNT = 0.
        regs[(UART_STATUS / 4) as usize] = STATUS_ST_UTX_OUT;
        // TXFIFO_EMPTY/TX_DONE are LEVEL-style latches on real silicon: the
        // FIFO is empty and the transmitter idle from reset, so both RAW
        // bits read 1 until the first byte is written (TRM UART_INT_RAW).
        // IDF's uart driver relies on this: uart_enable_tx_intr() enables
        // TXFIFO_EMPTY and the ISR fires immediately to drain the driver's
        // TX ringbuffer (uart_tx_all -> xRingbufferSend -> enable -> ISR).
        regs[(UART_INT_RAW / 4) as usize] = INT_TXFIFO_EMPTY | INT_TX_DONE;
        // MEM_CONF reset default carries rx_tout_thrhd = 10 (TRM).
        regs[(UART_MEM_CONF / 4) as usize] = 10 << MEM_CONF_RX_TOUT_THRHD_SHIFT;
        Self {
            regs,
            tx_out: Vec::new(),
            rx: VecDeque::new(),
            tout_idle: 0,
        }
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

    /// Push one received byte into the RX FIFO (host console input).
    pub fn inject_rx(&mut self, byte: u8) {
        self.rx.push_back(byte);
        self.tout_idle = 0;
        let regs = &mut self.regs;
        regs[(UART_STATUS / 4) as usize] = (regs[(UART_STATUS / 4) as usize]
            & !STATUS_FIFO_CNT_MASK)
            | ((self.rx.len() as u32 & 0x3F) << 24);
        regs[(UART_RXD_CNT / 4) as usize] = regs[(UART_RXD_CNT / 4) as usize].wrapping_add(1);
        regs[(UART_INT_RAW / 4) as usize] |= INT_RXFIFO_FULL;
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
    pub fn int_st(&self) -> u32 {
        self.regs[(UART_INT_RAW / 4) as usize] & self.regs[(UART_INT_ENA / 4) as usize]
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            UART_FIFO => {
                // Pop one RX byte; the FIFO-full line drops once empty.
                let Some(b) = self.rx.pop_front() else {
                    return 0;
                };
                self.regs[(UART_STATUS / 4) as usize] = (self.regs[(UART_STATUS / 4) as usize]
                    & !STATUS_FIFO_CNT_MASK)
                    | ((self.rx.len() as u32 & 0x3F) << 24);
                if self.rx.is_empty() {
                    self.regs[(UART_INT_RAW / 4) as usize] &= !INT_RXFIFO_FULL;
                }
                b as u32
            }
            // Interrupt status = RAW & ENA (TRM UART_INT_ST).
            UART_INT_ST => self.int_st(),
            _ => self.regs[(offset / 4) as usize],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            UART_FIFO => {
                // TXD_BRK (CONF0 bit 8) sends a break — ignore for now.
                if self.regs[(UART_CONF0 / 4) as usize] & (1 << 8) == 0 {
                    self.tx_out.push(value as u8);
                }
                // FIFO drains instantly: TXFIFO_EMPTY + TX_DONE latch high.
                let raw = &mut self.regs[(UART_INT_RAW / 4) as usize];
                *raw |= INT_TXFIFO_EMPTY | INT_TX_DONE;
            }
            UART_INT_CLR => {
                // Writing 1 clears the corresponding RAW bit (TRM UART_INT_CLR).
                self.regs[(UART_INT_RAW / 4) as usize] &= !value;
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
