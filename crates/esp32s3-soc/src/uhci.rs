//! ESP32-S3 UHCI0 (Universal Host Controller Interface) DMA bridge.
//!
//! Base `DR_REG_UHCI0_BASE = 0x6001_4000`. UHCI bridges UART0/1/2 to GDMA
//! (`SOC_GDMA_TRIG_PERIPH_UHCI0` = peri_sel 2): the esp-idf UART driver
//! moves TX/RX through GDMA descriptors instead of the FIFO registers.
//!
//! Register layout per `soc/uhci_reg.h`: CONF0 @ 0x00 (UART0/1/2_CE bits
//! 2/3/4 select the UART, TX_RST bit 0, RX_RST bit 1, CLK_EN bit 11, plus
//! SLIP-framing enable bits), INT_RAW @ 0x04 / INT_ST @ 0x08 / INT_ENA @
//! 0x0C / INT_CLR @ 0x10 (RX_START bit 0, TX_START bit 1, RX_HUNG bit 2).
//!
//! SLIP framing (esptool-compatible, classic-UHCI parity): with SEPER_EN
//! (CONF0 bit 5) packets are wrapped in SEPER_CHAR separators (default
//! 0xC0) with data bytes escaped (SEPER -> [ESC0, ESC1], ESC0 ->
//! [ESC0, 0xDD]; ESCAPE_CONF @ 0x24 carries SEPER_CHAR[7:0] (dflt 0xC0),
//! ESC_CHAR0[15:8] (dflt 0xDB), ESC_CHAR1[23:16] (dflt 0xDC) per
//! uhci_reg.h). Raw mode (SEPER_EN clear — what the UART-GDMA driver
//! programs) passes bytes through untouched. HEAD_EN (bit 6) is accepted
//! but has no effect (its 2-byte head content is unverifiable offline).
//! GDMA-OUT bytes land in the selected UART's TX FIFO (framed when
//! SEPER_EN is set); GDMA-IN drains its RX FIFO (deframed, separator
//! bytes dropped, split escape pairs reassembled across chunk
//! boundaries). Transfers gate on CLK_EN plus the SYSTEM UHCI0 clock
//! (EN0 bit 8, applied at the soc GDMA-walk sites — dropped/zeros when
//! either clock is off, but still complete, no hangs). TX/RX_RST are
//! stored with no staged state to clear (the pipe is stateless).
//! RX_HUNG is never raised (no hung detection). Validated by unit tests +
//! the `esp32s3_uhci` poke sketch (GDMA-OUT to UART1 TX captured by the
//! harness, UART1 RX injected by the harness back to DRAM via GDMA-IN,
//! INT_ST asserts for both directions).

pub const UHCI0_BASE: u32 = 0x6001_4000;
/// UHCI0 interrupt matrix source (`interrupts.h` recount).
pub const UHCI0_INTR_SOURCE: u32 = 14;

// CONF0 bits (uhci_reg.h).
const CONF_UART0_CE: u32 = 1 << 2;
const CONF_UART1_CE: u32 = 1 << 3;
const CONF_UART2_CE: u32 = 1 << 4;
const CONF_CLK_EN: u32 = 1 << 11;
const CONF_HEAD_EN: u32 = 1 << 6;
// CONF1 @ 0x18 SAVE_HEAD bit (uhci_reg.h): capture the head bytes.
const CONF1_SAVE_HEAD: u32 = 1 << 3;
// Received-packet head register (uhci_reg.h RX_HEAD_REG @ 0x30).
const RX_HEAD_OFF: u32 = 0x30;
const CONF1_OFF: u32 = 0x18;
const CONF_SEPER_EN: u32 = 1 << 5;
// NOTE: no CONF_HEAD_EN const (bit 6): HEAD framing is accepted but has no
// effect (its 2-byte head content is unverifiable offline), so the bit is
// never read — see the module docs.
// ESCAPE_CONF @ 0x24 fields (uhci_reg.h).
const ESCAPE_CONF_OFF: u32 = 0x24;
const ESC_SEPER_SHIFT: u32 = 0;
const ESC_ESC0_SHIFT: u32 = 8;
const ESC_ESC1_SHIFT: u32 = 16;
const ESC_MASK: u32 = 0xFF;
// Classic-UHCI self-escape second byte (esptool SLIP: 0xDB -> DB DD).
const ESC_SELF_SECOND: u8 = 0xDD;
// INT bits.
const INT_RX_START: u32 = 1 << 0;
const INT_TX_START: u32 = 1 << 1;
const INT_RX_HUNG: u32 = 1 << 2;
// RX_HUNG idle threshold (ticks with RX_START latched and no RX data).
// Chosen constant (no TRM figure, no driver flow consumes it): the bit
// exists so firmware polling it observes documented behavior.
const RX_HUNG_TICKS: u32 = 1024;

const INT_RAW_OFF: u32 = 0x04;
const INT_ST_OFF: u32 = 0x08;
const INT_ENA_OFF: u32 = 0x0C;
const INT_CLR_OFF: u32 = 0x10;

pub struct Uhci {
    conf0: u32,
    int_raw: u32,
    int_ena: u32,
    esc_conf: u32,
    conf1: u32,
    /// Received-packet head bytes (first 2 payload bytes when HEAD_EN +
    /// SAVE_HEAD capture them instead of DRAM).
    rx_head: u32,
    /// RX_HUNG idle counter (ticks with RX_START latched and dry RX).
    hung_count: u32,
    /// Split escape pair carry (IN deframer saw a trailing ESC0).
    dec_pend: bool,
}

impl Uhci {
    pub fn new() -> Self {
        Self {
            conf0: 0,
            int_raw: 0,
            int_ena: 0,
            esc_conf: (0xDC << 16) | (0xDB << 8) | 0xC0,
            conf1: 0,
            rx_head: 0,
            hung_count: 0,
            dec_pend: false,
        }
    }

    /// Clock enabled (transfer gate).
    pub fn clk_on(&self) -> bool {
        self.conf0 & CONF_CLK_EN != 0
    }

    /// Selected UART (lowest set UARTn_CE: 0, 1, 2; None if none set).
    pub fn uart_sel(&self) -> Option<usize> {
        if self.conf0 & CONF_UART0_CE != 0 {
            Some(0)
        } else if self.conf0 & CONF_UART1_CE != 0 {
            Some(1)
        } else if self.conf0 & CONF_UART2_CE != 0 {
            Some(2)
        } else {
            None
        }
    }

    /// Done-interrupt status (`RAW & ENA`) for matrix source 14.
    pub fn int_st(&self) -> bool {
        self.int_raw & self.int_ena != 0
    }

    /// Escape characters (SEPER, ESC0, ESC1) from ESCAPE_CONF.
    fn esc_chars(&self) -> (u8, u8, u8) {
        (
            ((self.esc_conf >> ESC_SEPER_SHIFT) & ESC_MASK) as u8,
            ((self.esc_conf >> ESC_ESC0_SHIFT) & ESC_MASK) as u8,
            ((self.esc_conf >> ESC_ESC1_SHIFT) & ESC_MASK) as u8,
        )
    }

    /// SLIP-frame `data` for the UART line (OUT direction). Raw mode
    /// passes through; SEPER_EN wraps in separators with escaping.
    pub fn slip_encode(&self, data: &[u8]) -> alloc::vec::Vec<u8> {
        if self.conf0 & CONF_SEPER_EN == 0 {
            return data.into();
        }
        let (seper, esc0, esc1) = self.esc_chars();
        let mut out = alloc::vec::Vec::with_capacity(data.len() + 2);
        out.push(seper);
        for &b in data {
            if b == seper {
                out.push(esc0);
                out.push(esc1);
            } else if b == esc0 {
                out.push(esc0);
                out.push(ESC_SELF_SECOND);
            } else {
                out.push(b);
            }
        }
        out.push(seper);
        out
    }

    /// Deframe one UART-RX chunk (IN direction), dropping separators and
    /// reassembling escape pairs (a trailing ESC0 carries into the next
    /// chunk via `dec_pend`). Raw mode passes through.
    pub fn slip_decode(&mut self, chunk: &[u8]) -> alloc::vec::Vec<u8> {
        if self.conf0 & CONF_SEPER_EN == 0 {
            return chunk.into();
        }
        let (seper, esc0, esc1) = self.esc_chars();
        let mut out = alloc::vec::Vec::with_capacity(chunk.len());
        for &b in chunk {
            if self.dec_pend {
                self.dec_pend = false;
                if b == esc1 {
                    out.push(seper);
                } else {
                    // Any other second byte (incl. ESC_SELF_SECOND)
                    // yields ESC0; a lone trailing ESC0 is dropped.
                    out.push(esc0);
                    if b != esc0 {
                        // Not an escape at all: reprocess this byte (it
                        // may itself be a separator or plain data).
                        if b == seper {
                            continue;
                        }
                        out.push(b);
                    }
                }
            } else if b == esc0 {
                self.dec_pend = true;
            } else if b == seper {
                continue; // framing separator, not data
            } else {
                out.push(b);
            }
        }
        out
    }

    /// Latch TX_START (GDMA-OUT link ran through UHCI).
    pub fn latch_tx_start(&mut self) {
        self.int_raw |= INT_TX_START;
    }

    /// Latch RX_START (GDMA-IN link ran through UHCI).
    pub fn latch_rx_start(&mut self) {
        self.int_raw |= INT_RX_START;
        self.hung_count = 0;
    }

    /// Receive watchdog tick: with RX_START latched and a dry receiver,
    /// count idle ticks and raise RX_HUNG at the threshold; any data (or
    /// no latched start) resets the count. Called per step while clocked
    /// (see the SoC tick wiring).
    pub fn tick_rx_idle(&mut self, rx_empty: bool) {
        if self.int_raw & INT_RX_START == 0 || !rx_empty {
            self.hung_count = 0;
            return;
        }
        self.hung_count += 1;
        if self.hung_count >= RX_HUNG_TICKS {
            self.int_raw |= INT_RX_HUNG;
        }
    }

    /// Head capture armed (HEAD_EN + SAVE_HEAD): the transfer's first 2
    /// payload bytes land in RX_HEAD instead of DRAM (uhci_reg.h
    /// RX_HEAD_REG; TX heads are driver-prepended data, already handled
    /// by the passthrough).
    pub fn head_capture(&self) -> bool {
        self.conf0 & CONF_HEAD_EN != 0 && self.conf1 & CONF1_SAVE_HEAD != 0
    }

    /// Store the captured head (first payload byte in [7:0], LE-consistent).
    pub fn set_rx_head(&mut self, lo: u8, hi: u8) {
        self.rx_head = (hi as u32) << 8 | lo as u32;
    }

    pub fn read32(&self, off: u32) -> u32 {
        match off {
            0x00 => self.conf0,
            CONF1_OFF => self.conf1,
            ESCAPE_CONF_OFF => self.esc_conf,
            RX_HEAD_OFF => self.rx_head,
            INT_RAW_OFF => self.int_raw,
            INT_ST_OFF => self.int_raw & self.int_ena,
            INT_ENA_OFF => self.int_ena,
            _ => 0,
        }
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        match off {
            // RX_RST clears the deframer carry (fresh packet boundary).
            0x00 => {
                self.conf0 = value;
                if value & (1 << 1) != 0 {
                    self.dec_pend = false;
                }
            }
            CONF1_OFF => self.conf1 = value,
            ESCAPE_CONF_OFF => self.esc_conf = value,
            INT_ENA_OFF => self.int_ena = value,
            INT_CLR_OFF => self.int_raw &= !value,
            _ => {}
        }
    }
}

impl Default for Uhci {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conf_round_trip_and_uart_select() {
        let mut u = Uhci::new();
        assert_eq!(u.uart_sel(), None);
        assert!(!u.clk_on());
        u.write32(0x00, CONF_CLK_EN | CONF_UART1_CE);
        assert_eq!(
            u.read32(0x00) & (CONF_CLK_EN | CONF_UART1_CE),
            CONF_CLK_EN | CONF_UART1_CE
        );
        assert_eq!(u.uart_sel(), Some(1));
        assert!(u.clk_on());
        assert!(!u.int_st(), "quiet before any transfer");
    }

    #[test]
    fn int_latch_enable_and_clear() {
        let mut u = Uhci::new();
        u.latch_tx_start();
        assert!(!u.int_st(), "masked while ENA is clear");
        u.write32(INT_ENA_OFF, INT_TX_START);
        assert!(u.int_st(), "level asserts");
        u.write32(INT_CLR_OFF, INT_TX_START);
        assert!(!u.int_st(), "W1C clears");
        assert_eq!(u.read32(INT_RAW_OFF), 0);
    }
}

#[cfg(test)]
mod slip_tests {
    use super::*;
    use alloc::vec::Vec;

    fn seper_dev() -> Uhci {
        let mut u = Uhci::new();
        u.write32(0x00, CONF_CLK_EN | (1 << 5)); // CLK_EN + SEPER_EN
        u
    }

    #[test]
    fn raw_mode_passes_through() {
        let u = Uhci::new();
        let data = [0xC0u8, 0xDB, 0x41];
        assert_eq!(u.slip_encode(&data), Vec::from(data));
        let mut u = u;
        assert_eq!(u.slip_decode(&data), Vec::from(data));
    }

    #[test]
    fn seper_frames_and_escapes() {
        let u = seper_dev();
        // 0x41 plain, 0xC0 -> DB DC, 0xDB -> DB DD, wrapped in C0..C0.
        assert_eq!(
            u.slip_encode(&[0x41, 0xC0, 0xDB, 0x42]),
            Vec::from([0xC0, 0x41, 0xDB, 0xDC, 0xDB, 0xDD, 0x42, 0xC0])
        );
    }

    #[test]
    fn deframer_strips_and_reassembles_across_chunks() {
        let mut u = seper_dev();
        // Split escape pair across chunks: [C0 41 DB] + [DC 42 C0].
        assert_eq!(u.slip_decode(&[0xC0, 0x41, 0xDB]), Vec::from([0x41]));
        assert_eq!(u.slip_decode(&[0xDC, 0x42, 0xC0]), Vec::from([0xC0, 0x42]));
    }

    #[test]
    fn custom_escape_chars_apply() {
        let mut u = Uhci::new();
        // SEPER=0x7E, ESC0=0x7D, ESC1=0x5D.
        u.write32(0x24, (0x5D << 16) | (0x7D << 8) | 0x7E);
        u.write32(0x00, CONF_CLK_EN | (1 << 5));
        assert_eq!(
            u.slip_encode(&[0x7E, 0x41]),
            Vec::from([0x7E, 0x7D, 0x5D, 0x41, 0x7E])
        );
    }
}

#[cfg(test)]
mod hung_tests {
    use super::*;

    #[test]
    fn rx_hung_raises_after_idle_threshold_with_start_latched() {
        let mut u = Uhci::new();
        u.write32(0x00, CONF_CLK_EN | CONF_UART1_CE);
        // No START latched: idle ticks never raise.
        for _ in 0..2048 {
            u.tick_rx_idle(true);
        }
        assert_eq!(u.read32(INT_RAW_OFF) & INT_RX_HUNG, 0);
        // Latch RX_START with a dry receiver: raises at 1024 idle ticks.
        u.latch_rx_start();
        for _ in 0..1023 {
            u.tick_rx_idle(true);
        }
        assert_eq!(u.read32(INT_RAW_OFF) & INT_RX_HUNG, 0, "not yet");
        u.tick_rx_idle(true);
        assert_ne!(u.read32(INT_RAW_OFF) & INT_RX_HUNG, 0, "hung raised");
        assert_eq!(
            u.read32(INT_ST_OFF) & INT_RX_HUNG,
            0,
            "masked until enabled"
        );
        u.write32(INT_ENA_OFF, INT_RX_HUNG);
        assert_ne!(u.read32(INT_ST_OFF) & INT_RX_HUNG, 0, "ST asserts");
        // Data arrival resets the watchdog.
        u.write32(INT_CLR_OFF, INT_RX_HUNG);
        u.tick_rx_idle(false);
        for _ in 0..2048 {
            u.tick_rx_idle(true);
        }
        // START still latched from before, so it raises again (documented:
        // the count resets but the latch persists until CLR).
        assert_ne!(u.read32(INT_RAW_OFF) & INT_RX_HUNG, 0);
    }
}
