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
/// Second status bank (pins 32..45): mirrors STATUS, cleared the same way.
/// Offsets from gpio_struct.h member order (status trio, then pcpu_int at
/// 0x5C, pcpu_int1 at 0x68, then pin[54] at 0x74).
pub const GPIO_STATUS1: u32 = 0x50;
pub const GPIO_STATUS1_W1TS: u32 = 0x54;
pub const GPIO_STATUS1_W1TC: u32 = 0x58;
/// Per-CPU interrupt status (what the GPIO ISR reads): status bits of
/// interrupt-enabled pins, low bank then high bank.
pub const GPIO_PCPU_INT: u32 = 0x5C;
pub const GPIO_PCPU_INT1: u32 = 0x68;

/// GPIO interrupt source for the matrix (ETS_GPIO_INTR_SOURCE).
pub const GPIO_INTR_SOURCE: u32 = 16;

// PIN register fields (gpio_struct.h pin[]: int_type[9:7], int_ena[17:13]).
// The driver enables with BIT(0) (GPIO_LL_INTR_ENA); per-CPU routing goes
// through the interrupt matrix, not the remaining ena bits.
const PIN_INT_TYPE_SHIFT: u32 = 7;
const PIN_INT_TYPE_MASK: u32 = 0x7;
const PIN_INT_ENA_BIT: u32 = 13;

// int_type values (hal gpio_int_type_t).
const INT_DISABLE: u32 = 0;
const INT_RISING: u32 = 1;
const INT_FALLING: u32 = 2;
const INT_ANYEDGE: u32 = 3;
const INT_LOW: u32 = 4;
const INT_HIGH: u32 = 5;

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
    /// Last sampled pad levels (for edge detection).
    prev: u64,
    /// Pins with the interrupt enable bit set (poll + matrix fast path).
    armed: u64,
}

impl Gpio {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        // TRM GPIO_FUNC_OUT_SEL_CFG: reset default 0x80 selects the GPIO
        // output function (vs a peripheral signal). Our window initializes
        // to 0, which would make every pin follow signal 0 instead of
        // GPIO_OUT — so output-enabled pins would never reflect their
        // driven level (breaks GPIO LED visualization and digitalRead of an
        // OUTPUT pin). Seed the default.
        for i in 0..PIN_COUNT {
            regs[(GPIO_FUNC_OUT_SEL_0 / 4) as usize + i] = 0x80;
        }
        regs[(GPIO_STRAP / 4) as usize] = STRAP_FLASH_BOOT;
        regs[(GPIO_IN / 4) as usize] = STRAP_FLASH_BOOT;
        Self {
            regs,
            prev: 0,
            armed: 0,
        }
    }

    /// PIN config word for `i` (int_type/int_ena live here).
    fn pin_reg(&self, i: usize) -> u32 {
        self.regs[(GPIO_PIN_0 / 4) as usize + i]
    }

    /// Refresh the armed bit for pin `i` from its PIN register, seeding the
    /// edge-detector baseline at the current pad level (like the silicon
    /// input synchronizer, so enabling on an already-high pin does not
    /// fire a spurious RISING edge).
    fn refresh_armed(&mut self, i: usize) {
        let ena = self.pin_reg(i) & (1 << PIN_INT_ENA_BIT) != 0;
        if ena {
            self.armed |= 1u64 << i;
            if self.pin_level(i as u32) != 0 {
                self.prev |= 1u64 << i;
            } else {
                self.prev &= !(1u64 << i);
            }
        } else {
            self.armed &= !(1u64 << i);
        }
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
        } else if offset == GPIO_PCPU_INT {
            // CPU interrupt status: latched status of enabled pins (low bank).
            (self.regs[(GPIO_STATUS / 4) as usize] as u64 & self.armed) as u32
        } else if offset == GPIO_PCPU_INT1 {
            // High bank (pins 32..45).
            (self.regs[(GPIO_STATUS1 / 4) as usize] as u64 & (self.armed >> 32)) as u32
        } else {
            self.regs[(offset / 4) as usize]
        }
    }

    /// Raw GPIO_IN register value (strap / external input state) WITHOUT the
    /// output loopback. `Soc::gpio_in_readback` overlays the loopback using
    /// the actual driven level (which for a peripheral-matrix-routed pin is the
    /// peripheral signal, not GPIO_OUT).
    pub fn raw_in(&self) -> u32 {
        self.regs[(GPIO_IN / 4) as usize]
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            GPIO_OUT_W1TS => self.regs[(GPIO_OUT / 4) as usize] |= value,
            GPIO_OUT_W1TC => self.regs[(GPIO_OUT / 4) as usize] &= !value,
            GPIO_ENABLE_W1TS => self.regs[(GPIO_ENABLE / 4) as usize] |= value,
            GPIO_ENABLE_W1TC => self.regs[(GPIO_ENABLE / 4) as usize] &= !value,
            GPIO_STATUS_W1TS => self.regs[(GPIO_STATUS / 4) as usize] |= value,
            GPIO_STATUS_W1TC => self.regs[(GPIO_STATUS / 4) as usize] &= !value,
            GPIO_STATUS1_W1TS => self.regs[(GPIO_STATUS1 / 4) as usize] |= value,
            GPIO_STATUS1_W1TC => self.regs[(GPIO_STATUS1 / 4) as usize] &= !value,
            // OUT/ENABLE/STATUS and all other offsets: plain latch.
            _ => {
                self.regs[(offset / 4) as usize] = value;
                // A PIN config write may arm/disarm that pin's interrupt.
                if (GPIO_PIN_0..GPIO_PIN_0 + 54 * 4).contains(&offset) {
                    self.refresh_armed(((offset - GPIO_PIN_0) / 4) as usize);
                }
            }
        }
    }

    /// Any pin with the interrupt enable bit set (tick fast path).
    pub fn irq_armed(&self) -> bool {
        self.armed != 0
    }

    /// Interrupt source asserted: latched status on an enabled pin.
    pub fn int_pending(&self) -> bool {
        let st = self.regs[(GPIO_STATUS / 4) as usize] as u64
            | ((self.regs[(GPIO_STATUS1 / 4) as usize] as u64) << 32);
        st & self.armed != 0
    }

    /// Sample pad `levels` (the readback word: GPIO_OUT loopback overlaid
    /// with peripheral-driven levels by the SoC) against the previous
    /// sample and latch STATUS bits per each armed pin's int_type
    /// (gpio_int_type_t: 1 rising, 2 falling, 3 either edge, 4 low level,
    /// 5 high level). Level types re-latch while the level holds, so the
    /// ISR refires after INT_CLR until the condition clears, like silicon.
    pub fn poll_interrupts(&mut self, levels: u32) {
        if self.armed == 0 {
            return;
        }
        let cur = levels as u64;
        let mut armed = self.armed;
        while armed != 0 {
            let i = armed.trailing_zeros() as usize;
            armed &= armed - 1;
            let ty = (self.pin_reg(i) >> PIN_INT_TYPE_SHIFT) & PIN_INT_TYPE_MASK;
            let was = (self.prev >> i) & 1 != 0;
            let is = (cur >> i) & 1 != 0;
            let fire = match ty {
                INT_DISABLE => false,
                INT_RISING => !was && is,
                INT_FALLING => was && !is,
                INT_ANYEDGE => was != is,
                INT_LOW => !is,
                INT_HIGH => is,
                _ => false,
            };
            if fire {
                if i < 32 {
                    self.regs[(GPIO_STATUS / 4) as usize] |= 1u32 << i;
                } else {
                    self.regs[(GPIO_STATUS1 / 4) as usize] |= 1u32 << (i - 32);
                }
            }
        }
        self.prev = cur;
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
    /// 128 = the pin follows GPIO_OUT instead of a peripheral signal). Bit 7
    /// is the GPIO-drive sentinel, so the low byte is returned intact — a
    /// `& 0x7F` mask would strip the 0x80 sentinel and force every pin onto
    /// signal 0.
    pub fn out_sel(&self, i: usize) -> u32 {
        let reg = self.regs[(GPIO_FUNC_OUT_SEL_0 / 4) as usize + i];
        reg & 0xFF
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

    /// Resolve a GPIO-matrix INPUT signal index (`sig`) to the GPIO pin it
    /// sources from, per `FUNC_IN_SEL_CFG[sig]` (bits [5:0] = gpio, bit 6 =
    /// invert). Returns `None` if `sig` is out of range. The firmware calls
    /// `gpio_matrix_in(pin, sig, inv)` which writes exactly this register; it
    /// lets a peripheral (PCNT, UART RX, I2C SDA/SCL, …) read a GPIO's level.
    pub fn in_sel(&self, sig: u32) -> Option<(u32, bool)> {
        if (sig as usize) >= 256 {
            return None;
        }
        let reg = self.regs[(GPIO_FUNC_IN_SEL_0 / 4) as usize + sig as usize];
        let pin = reg & 0x3F;
        let inv = (reg >> 6) & 1 != 0;
        Some((pin, inv))
    }

    /// Current logical level (0/1) of GPIO `pin`: the driven output if the pin
    /// is output-enabled, else its input/strap state (GPIO_IN loopback).
    pub fn pin_level(&self, pin: u32) -> u32 {
        let inp = self.regs[(GPIO_IN / 4) as usize] as u64;
        let out = self.regs[(GPIO_OUT / 4) as usize] as u64;
        let en = self.regs[(GPIO_ENABLE / 4) as usize] as u64;
        let v = (inp & !en) | (out & en);
        ((v >> pin) & 1) as u32
    }
}

impl Default for Gpio {
    fn default() -> Self {
        Self::new()
    }
}
