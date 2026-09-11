//! ESP32-S3 USB-Serial-JTAG (CDC-ACM console) peripheral model.
//!
//! Register layout per `usb_serial_jtag_struct.h` (base `0x6003_8000`).
//! Functional model of the Serial (CDC) data path:
//! - TX: firmware writes bytes to `EP1` (`rdwr_byte`, 0x00) while
//!   `EP1_CONF.serial_in_ep_data_free` (bit 1) is set; after the last byte it
//!   writes `EP1_CONF.wr_done` (bit 0) = 1, which latches `serial_in_empty_int`
//!   (`INT_RAW` bit 3) so the driver's TX-done ISR/poll proceeds. Each byte is
//!   emitted to the host console immediately (the USB host is modeled as always
//!   present, so there is no enumeration/IN-token backpressure).
//! - RX: host-injected bytes sit in an RX FIFO; `EP1_CONF.serial_out_ep_data_avail`
//!   (bit 2) = 1 and `serial_out_recv_pkt_int` (`INT_RAW` bit 2) is raised;
//!   firmware reads `EP1` (0x00) to pop bytes (up to `OUT_EP1_ST.rec_data_cnt`).
//! - Remaining registers are latched so firmware configuration writes are
//!   harmless.

use alloc::collections::VecDeque;
use alloc::vec::Vec;

/// Interrupt-matrix source (esp32s3 `interrupts.h` `ETS_USB_SERIAL_JTAG_INTR_SOURCE`).
pub const USB_SERIAL_JTAG_INTR_SOURCE: u32 = 96;

// Register offsets (`usb_serial_jtag_reg.h`).
pub const EP1: u32 = 0x00;
pub const EP1_CONF: u32 = 0x04;
pub const INT_RAW: u32 = 0x08;
pub const INT_ST: u32 = 0x0C;
pub const INT_ENA: u32 = 0x10;
pub const INT_CLR: u32 = 0x14;
pub const OUT_EP1_ST: u32 = 0x3C;

// EP1_CONF bit positions.
const CONF_WR_DONE: u32 = 1 << 0;
const CONF_SERIAL_IN_EP_DATA_FREE: u32 = 1 << 1;
const CONF_SERIAL_OUT_EP_DATA_AVAIL: u32 = 1 << 2;

// INT_RAW/ST/ENA/CLR bit positions.
const INT_SERIAL_OUT_RECV_PKT: u32 = 1 << 2;
const INT_SERIAL_IN_EMPTY: u32 = 1 << 3;

// OUT_EP1_ST field positions.
const OUT_EP1_WR_ADDR: u32 = 0x7F << 1;
const OUT_EP1_RD_ADDR: u32 = 0x7F << 9;
const OUT_EP1_REC_DATA_CNT: u32 = 0x7F << 17;

const REG_COUNT: usize = 0x84 / 4;

/// USB-Serial-JTAG controller.
pub struct UsbSerialJtag {
    /// Latched register bank (INT_RAW/INT_ENA + config registers).
    regs: [u32; REG_COUNT],
    /// Bytes emitted on the USB-CDC TX line (host console output).
    tx_out: Vec<u8>,
    /// Received-but-unread bytes (the RX FIFO).
    rx: VecDeque<u8>,
    /// True when `serial_in_empty_int` has been cleared by the ISR but
    /// the TX FIFO is still empty — the host poll timer should
    /// re-assert it.  Models the level-triggered `serial_in_empty_int`
    /// on real USB-Serial-JTAG hardware (the host periodically polls
    /// the device via IN tokens, and the interrupt stays asserted as
    /// long as the TX FIFO has space).
    need_reassert: bool,
    /// TX bytes staged while the USB_DEVICE clock (EN1 bit 10) is off
    /// (silicon holds them in the endpoint FIFO instead of shifting);
    /// flushed on the next tick once the clock returns. UART holds in its
    /// TXFIFO the same way (see uart.rs flow control); the console drain
    /// (`take_tx`) only ever sees shifted bytes.
    hold: VecDeque<u8>,
    /// SYSTEM USB_DEVICE clock shadow (set by the SoC on EN1 writes).
    sys_clk: bool,
    /// Countdown (in tick() calls) until the next re-assertion.
    /// Models the USB host polling interval (~1 ms ≈ 1000 APB cycles).
    /// This prevents the ISR from firing every step, which would kill
    /// performance.
    reassert_countdown: u32,
}

impl UsbSerialJtag {
    pub fn new() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
            tx_out: Vec::new(),
            rx: VecDeque::new(),
            hold: VecDeque::new(),
            sys_clk: true,
            need_reassert: false,
            reassert_countdown: 0,
        }
    }

    /// SYSTEM USB_DEVICE clock (EN1 bit 10) shadow for the TX hold.
    pub fn set_sys_clk(&mut self, on: bool) {
        self.sys_clk = on;
    }

    /// Drain the bytes this device emitted (host console output).
    pub fn take_tx(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.tx_out)
    }

    /// Periodic tick — models the USB host polling the device (IN tokens).
    /// On real hardware the `serial_in_empty_int` is level-triggered:
    /// asserted whenever the TX FIFO is empty and the device is
    /// configured.  The host sends IN tokens at the USB polling interval
    /// (e.g. 1 ms for full-speed).  Our edge-triggered `wr_done` model
    /// only fires once, so the CDC TX FreeRTOS task stalls after its
    /// first drain.  This method re-asserts the interrupt periodically
    /// when the ISR has cleared it but the FIFO is still empty, keeping
    /// the TX task cycling so it can drain subsequent `Serial.write()`
    /// batches.
    pub fn tick(&mut self) {
        // The SoC only ticks while the USB clock is on, so a non-empty
        // hold flushes exactly on clock return.
        if !self.hold.is_empty() {
            self.tx_out.extend(self.hold.drain(..));
        }
        if self.need_reassert {
            if self.reassert_countdown > 0 {
                self.reassert_countdown -= 1;
            } else {
                self.need_reassert = false;
                // Re-assert serial_in_empty_int only if ENA is set.
                if self.regs[(INT_ENA / 4) as usize] & INT_SERIAL_IN_EMPTY != 0 {
                    self.regs[(INT_RAW / 4) as usize] |= INT_SERIAL_IN_EMPTY;
                    // After re-asserting, the ISR will fire and clear
                    // again.  Re-arm the countdown so we don't re-assert
                    // immediately on the next tick (avoid ISR storm).
                    self.reassert_countdown = 100;
                }
            }
        }
    }

    /// Bytes queued (debug probe). Inline: hot console-drain fast path.
    #[inline]
    pub fn tx_len(&self) -> usize {
        self.tx_out.len()
    }

    /// Push one received byte into the RX FIFO (host console input).
    pub fn inject_rx(&mut self, byte: u8) {
        self.rx.push_back(byte);
        self.regs[(INT_RAW / 4) as usize] |= INT_SERIAL_OUT_RECV_PKT;
    }

    /// Interrupt status = RAW & ENA.
    pub fn int_pending(&self) -> bool {
        (self.regs[(INT_RAW / 4) as usize] & self.regs[(INT_ENA / 4) as usize]) != 0
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            EP1 => {
                // Pop one RX byte; the recv-pkt line drops once empty.
                let Some(b) = self.rx.pop_front() else {
                    return 0;
                };
                if self.rx.is_empty() {
                    self.regs[(INT_RAW / 4) as usize] &= !INT_SERIAL_OUT_RECV_PKT;
                }
                b as u32
            }
            EP1_CONF => {
                let mut v = CONF_SERIAL_IN_EP_DATA_FREE; // host always ready
                if !self.rx.is_empty() {
                    v |= CONF_SERIAL_OUT_EP_DATA_AVAIL;
                }
                v
            }
            INT_RAW => self.regs[(INT_RAW / 4) as usize],
            INT_ST => self.regs[(INT_RAW / 4) as usize] & self.regs[(INT_ENA / 4) as usize],
            INT_ENA => self.regs[(INT_ENA / 4) as usize],
            OUT_EP1_ST => {
                let n = (self.rx.len() as u32) & 0x7F;
                (n << 1 & OUT_EP1_WR_ADDR)
                    | (n << 9 & OUT_EP1_RD_ADDR)
                    | (n << 17 & OUT_EP1_REC_DATA_CNT)
            }
            _ if offset.is_multiple_of(4) && offset < (REG_COUNT * 4) as u32 => {
                self.regs[(offset / 4) as usize]
            }
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            EP1 => {
                // TX byte -> host console (captured immediately when the
                // controller clock runs; held in the endpoint FIFO while
                // gated, capped like the 128B UART FIFO for safety).
                if self.sys_clk {
                    self.tx_out.push((value & 0xFF) as u8);
                } else if self.hold.len() < 1024 {
                    self.hold.push_back((value & 0xFF) as u8);
                }
            }
            EP1_CONF => {
                // wr_done: latch serial_in_empty_int so the driver's TX-done
                // ISR/poll proceeds (host "read" the IN packet already).
                if value & CONF_WR_DONE != 0 {
                    self.regs[(INT_RAW / 4) as usize] |= INT_SERIAL_IN_EMPTY;
                }
            }
            INT_CLR => {
                let cleared = self.regs[(INT_RAW / 4) as usize] & value;
                self.regs[(INT_RAW / 4) as usize] &= !value;
                // When the ISR clears serial_in_empty_int, remember to
                // re-assert it on the next tick if the FIFO is still
                // empty (level-triggered behavior).
                if cleared & INT_SERIAL_IN_EMPTY != 0 {
                    self.need_reassert = true;
                }
            }
            INT_ENA => {
                self.regs[(INT_ENA / 4) as usize] = value;
            }
            _ if offset.is_multiple_of(4) && offset < (REG_COUNT * 4) as u32 => {
                self.regs[(offset / 4) as usize] = value;
            }
            _ => {}
        }
    }
}

impl Default for UsbSerialJtag {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_byte_emitted_and_wr_done_raises_empty_int() {
        let mut d = UsbSerialJtag::new();
        d.write32(INT_ENA, INT_SERIAL_IN_EMPTY);
        d.write32(EP1, b'H' as u32);
        d.write32(EP1, b'i' as u32);
        d.write32(EP1_CONF, CONF_WR_DONE);
        assert!(d.int_pending());
        assert_eq!(d.read32(INT_RAW) & INT_SERIAL_IN_EMPTY, INT_SERIAL_IN_EMPTY);
        let out = d.take_tx();
        assert_eq!(out, b"Hi");
    }

    #[test]
    fn tx_byte_captured_immediately() {
        let mut d = UsbSerialJtag::new();
        d.write32(EP1, 0x41);
        assert_eq!(d.take_tx(), b"A");
    }

    #[test]
    fn ep1_conf_signals_writable_and_rx_avail() {
        let mut d = UsbSerialJtag::new();
        assert_eq!(
            d.read32(EP1_CONF) & CONF_SERIAL_IN_EP_DATA_FREE,
            CONF_SERIAL_IN_EP_DATA_FREE
        );
        assert_eq!(d.read32(EP1_CONF) & CONF_SERIAL_OUT_EP_DATA_AVAIL, 0);
        d.inject_rx(b'X');
        assert_eq!(
            d.read32(EP1_CONF) & CONF_SERIAL_OUT_EP_DATA_AVAIL,
            CONF_SERIAL_OUT_EP_DATA_AVAIL
        );
        assert_eq!(
            d.read32(INT_RAW) & INT_SERIAL_OUT_RECV_PKT,
            INT_SERIAL_OUT_RECV_PKT
        );
    }

    #[test]
    fn rx_fifo_pop_and_recv_int_clears_when_empty() {
        let mut d = UsbSerialJtag::new();
        d.inject_rx(0x55);
        d.inject_rx(0x56);
        assert_eq!(d.read32(EP1), 0x55);
        assert_eq!(d.read32(EP1), 0x56);
        assert_eq!(d.read32(EP1), 0); // empty
        assert_eq!(d.read32(INT_RAW) & INT_SERIAL_OUT_RECV_PKT, 0);
    }

    #[test]
    fn int_clr_clears_raw() {
        let mut d = UsbSerialJtag::new();
        d.write32(INT_ENA, INT_SERIAL_IN_EMPTY);
        d.write32(EP1_CONF, CONF_WR_DONE);
        assert!(d.int_pending());
        d.write32(INT_CLR, INT_SERIAL_IN_EMPTY);
        assert_eq!(d.read32(INT_RAW) & INT_SERIAL_IN_EMPTY, 0);
    }
}
