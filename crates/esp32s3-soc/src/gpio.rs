//! ESP32-S3 GPIO peripheral model.
//!
//! Register layout per QEMU `include/hw/gpio/esp32_gpio.h` (STRAP at 0x38) and
//! the TRM GPIO chapter. Implemented registers: OUT, OUT_W1TS, OUT_W1TC,
//! ENABLE, ENABLE_W1TS, ENABLE_W1TC, STRAP (read-only, flash boot mode),
//! IN (reads strap value; no input devices yet), STATUS/W1TS/W1TC.
//! All other offsets are latched (P2 scope: register-level model only).

// GPIO register offsets (TRM GPIO chapter).
pub const GPIO_OUT: u32 = 0x04;
pub const GPIO_OUT_W1TS: u32 = 0x08;
pub const GPIO_OUT_W1TC: u32 = 0x0C;
pub const GPIO_ENABLE: u32 = 0x20;
pub const GPIO_ENABLE_W1TS: u32 = 0x24;
pub const GPIO_ENABLE_W1TC: u32 = 0x28;
pub const GPIO_STRAP: u32 = 0x38;
pub const GPIO_IN: u32 = 0x3C;
pub const GPIO_STATUS: u32 = 0x44;
pub const GPIO_STATUS_W1TS: u32 = 0x48;
pub const GPIO_STATUS_W1TC: u32 = 0x4C;

/// Strap pin encoding for SPI flash boot mode (ESP32S3_STRAP_MODE_FLASH_BOOT
/// in QEMU esp32s3_gpio.h).
const STRAP_FLASH_BOOT: u32 = 0x4;

const PIN_COUNT: usize = 46;
const REG_COUNT: usize = 0x180 / 4;

pub struct Gpio {
    regs: [u32; REG_COUNT],
}

impl Gpio {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        regs[(GPIO_STRAP / 4) as usize] = STRAP_FLASH_BOOT;
        regs[(GPIO_IN / 4) as usize] = STRAP_FLASH_BOOT;
        Self { regs }
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        self.regs[(offset / 4) as usize]
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            GPIO_OUT_W1TS => self.regs[(GPIO_OUT / 4) as usize] |= value,
            GPIO_OUT_W1TC => self.regs[(GPIO_OUT / 4) as usize] &= !value,
            GPIO_ENABLE_W1TS => self.regs[(GPIO_ENABLE / 4) as usize] |= value,
            GPIO_ENABLE_W1TC => self.regs[(GPIO_ENABLE / 4) as usize] &= !value,
            GPIO_STATUS_W1TS => self.regs[(GPIO_STATUS / 4) as usize] |= value,
            GPIO_STATUS_W1TC => self.regs[(GPIO_STATUS / 4) as usize] &= !value,
            // OUT/ENABLE/STATUS and all other offsets: plain latch.
            _ => self.regs[(offset / 4) as usize] = value,
        }
    }

    /// Snapshot of the output-pin state (host LED visualization later).
    pub fn output(&self) -> u32 {
        self.regs[(GPIO_OUT / 4) as usize] & self.regs[(GPIO_ENABLE / 4) as usize]
    }

    pub fn pin_count(&self) -> usize {
        PIN_COUNT
    }
}

impl Default for Gpio {
    fn default() -> Self {
        Self::new()
    }
}
