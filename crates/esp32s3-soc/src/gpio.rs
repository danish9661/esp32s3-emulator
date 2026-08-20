//! ESP32-S3 GPIO peripheral model.
//!
//! Register layout per the S3 `soc/esp32s3/register/soc/gpio_struct.h`
//! (NOT the classic-ESP32 layout): OUT at 0x04, ENABLE at 0x20, STRAP at
//! 0x38, IN at 0x3C, STATUS at 0x44, pin[54] at 0x74, status_next at
//! 0x14C, func_in_sel_cfg[256] at 0x154 (the UART/USB/SPI signal-input
//! matrix — the app's GPIO init reads entries for its signal routing),
//! func_out_sel_cfg[54] at 0x554 (peripheral signal output select; 128 =
//! default GPIO_OUT drive), clock_gate at 0x62C, date at 0x700. The
//! window must cover 0x704 or the app's FUNC_OUT_SEL writes for pins
//! (e.g. pinMode(2) → 0x55C) panic the dispatch.

/// First FUNC_OUT_SEL_CFG register (GPIO_FUNC_OUT_SEL_CFG_REG, one per
/// pin, 4-byte stride; value 128 = default GPIO_OUT drive).
pub const GPIO_FUNC_OUT_SEL_0: u32 = 0x554;

// GPIO register offsets (S3 gpio_struct.h member order).
pub const GPIO_OUT: u32 = 0x04;
pub const GPIO_OUT_W1TS: u32 = 0x08;
pub const GPIO_OUT_W1TC: u32 = 0x0C;
pub const GPIO_ENABLE: u32 = 0x20;
pub const GPIO_ENABLE_W1TS: u32 = 0x24;
pub const GPIO_ENABLE_W1TC: u32 = 0x28;
pub const GPIO_STRAP: u32 = 0x38;
pub const GPIO_PIN_0: u32 = 0x74;
pub const GPIO_FUNC_IN_SEL_0: u32 = 0x154;
pub const GPIO_IN: u32 = 0x3C;
pub const GPIO_STATUS: u32 = 0x44;
pub const GPIO_STATUS_W1TS: u32 = 0x48;
pub const GPIO_STATUS_W1TC: u32 = 0x4C;

/// Strap pin encoding for SPI flash boot mode (ESP32S3_STRAP_MODE_FLASH_BOOT
/// in QEMU esp32s3_gpio.h).
const STRAP_FLASH_BOOT: u32 = 0x4;

const PIN_COUNT: usize = 46;
// GPIO register space: 0x000-0x148 core + pin config (pin[54]), then the
// per-signal FUNC_IN_SEL_CFG block (0x154..0x553, 256 signals), then
// FUNC_OUT_SEL_CFG (0x554..0x627, 54 signals), clock_gate 0x62C, date
// 0x700. The app's GPIO init reads/writes FUNC_IN_SEL entries for the
// UART/USB/SPI signal routing and FUNC_OUT_SEL for pinMode/digitalWrite
// (e.g. pin 2 → 0x55C); the array must cover the full 0x704 window or
// the dispatch panics.
const REG_COUNT: usize = 0x704 / 4;

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
        if offset == GPIO_IN {
            // Pad loopback: an output-enabled pin's input path reads the
            // driven value (the pad level), so digitalRead() of an OUTPUT
            // pin returns GPIO_OUT like real silicon; non-enabled pins keep
            // their input state (strap bits for the boot ROM check).
            let mut v = self.regs[(GPIO_IN / 4) as usize] as u64;
            let out = self.regs[(GPIO_OUT / 4) as usize] as u64;
            let en = self.regs[(GPIO_ENABLE / 4) as usize] as u64;
            v = (v & !en) | (out & en);
            v as u32
        } else {
            self.regs[(offset / 4) as usize]
        }
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

    /// Is pin `i`'s output driver enabled (GPIO_ENABLE)?
    pub fn enabled(&self, i: usize) -> bool {
        // u64 shift: GPIO bits live above 31 (46 pins).
        self.regs[(GPIO_ENABLE / 4) as usize] as u64 & (1u64 << i) != 0
    }

    /// Pin `i` GPIO_OUT register bit.
    pub fn out_bit(&self, i: usize) -> u32 {
        // u64 shift: GPIO bits live above 31 (46 pins).
        ((self.regs[(GPIO_OUT / 4) as usize] as u64 >> i) & 1) as u32
    }

    /// GPIO matrix output signal selected for pin `i` (FUNC_OUT_SEL [7:0];
    /// 128 = the pin follows GPIO_OUT instead of a peripheral signal).
    pub fn out_sel(&self, i: usize) -> u32 {
        let reg = self.regs[(GPIO_FUNC_OUT_SEL_0 / 4) as usize + i];
        reg & 0x7F
    }

    /// Snapshot of the output-pin state (host LED visualization later).
    pub fn output(&self) -> u32 {
        // u64 shifts: GPIO bits live above 31 (46 pins).
        let out = self.regs[(GPIO_OUT / 4) as usize] as u64;
        let en = self.regs[(GPIO_ENABLE / 4) as usize] as u64;
        (out & en) as u32
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
