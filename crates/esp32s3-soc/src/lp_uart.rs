//! ESP32-S3 LP_UART (low-power UART) controller.
//!
//! Base `0x6002_5400` (TRM memory map; sits just past the GPSPI3 block at
//! `0x6002_5000` in the same 4KB page). The LP_UART is the UART instance wired
//! to the LP/RTC domain, usable from deep-sleep wake stubs. It is a UART
//! register block (`FIFO` 0x00, `INT_RAW` 0x04, `CLKDIV` 0x14, `CONF0` 0x20,
//! ... up to ~0x100). The TX/RX FSM is **NOT modeled** (it needs a real serial
//! line); the block is a register store so firmware can configure it, matching
//! the P5 direct-poke validation pattern.

pub const LP_UART_BASE: u32 = 0x6002_5400;

// Cover the LP_UART register window (0x100 bytes).
const REG_COUNT: usize = 0x100 / 4;

pub struct LpUart {
    regs: [u32; REG_COUNT],
}

impl Default for LpUart {
    fn default() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
        }
    }
}

impl LpUart {
    pub fn new() -> Self {
        Self::default()
    }

    fn idx(&self, offset: u32) -> usize {
        ((offset & 0xFF) / 4) as usize
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let i = self.idx(offset);
        if i < REG_COUNT { self.regs[i] } else { 0 }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let i = self.idx(offset);
        if i < REG_COUNT {
            self.regs[i] = value;
        }
    }
}
