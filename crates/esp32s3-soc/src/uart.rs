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
//! - Remaining registers are latched (writes stored, reads return stored
//!   value or reset value) so firmware configuration writes are harmless.

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

const REG_COUNT: usize = 0x80 / 4;

/// One UART instance.
pub struct Uart {
    /// Generic register bank; indexed by (offset/4).
    regs: [u32; REG_COUNT],
    /// Bytes emitted on the TX line (read by the host console).
    tx_out: Vec<u8>,
}

impl Uart {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        // ST_UTX_OUT = idle high (UART_STATUS[27:24]); TXFIFO_CNT = 0.
        regs[(UART_STATUS / 4) as usize] = 1 << 24;
        Self {
            regs,
            tx_out: Vec::new(),
        }
    }

    /// Drain the bytes this UART emitted (host console output).
    pub fn take_tx(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.tx_out)
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            // RX FIFO empty → 0.
            UART_FIFO => 0,
            // Interrupt status = RAW & ENA (TRM UART_INT_ST).
            UART_INT_ST => {
                self.regs[(UART_INT_RAW / 4) as usize] & self.regs[(UART_INT_ENA / 4) as usize]
            }
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
