//! ESP32-S3 LP/I2C (a.k.a. `RTC_I2C`) controller.
//!
//! Base `DR_REG_RTC_I2C_BASE = 0x6000_8C00` (offset `0xC00` of the
//! `0x6000_8000` page; `soc/reg_base.h` defines `DR_REG_RTC_I2C_BASE`). This is
//! the low-power I2C master used by the RTC/ULP subsystem (e.g. to talk to a
//! sensor from deep-sleep wake stubs). It is a register block (`I2C_SCL_LOW`
//! 0x00, `I2C_SCL_HIGH` 0x04, `I2C_MS_DELAY` 0x08, `I2C_CTRL` 0x0C, ... up to
//! ~0x100). The bus FSM is **NOT modeled** (it drives a real I2C device); the
//! block is a register store so firmware can configure it, matching the P5
//! direct-poke validation pattern used for SDMMC/RTC_IO.

pub const RTC_I2C_BASE: u32 = 0x6000_8C00;

// Cover the RTC_I2C register window (0x100 bytes).
const REG_COUNT: usize = 0x100 / 4;

pub struct RtcI2c {
    regs: [u32; REG_COUNT],
}

impl Default for RtcI2c {
    fn default() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
        }
    }
}

impl RtcI2c {
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
