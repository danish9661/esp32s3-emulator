//! ESP32-S3 Timer Group (TIMG0/TIMG1) peripheral model.
//!
//! Register layout per QEMU `include/hw/timer/esp32_timg.h` (TRM Timer Group
//! chapter). Two 64-bit timers per group plus WDT registers (latched only).
//!
//! Time model: the frontend drives time via `tick(cycles)` (JS frame loop);
//! each enabled timer increments its counter by `cycles >> divider`. Counter
//! reads follow TRM semantics: T0LO returns the current low word, T0HI
//! returns the high word latched by a T0UPDATE write; T0LOAD loads the
//! counter from T0LOADLO/HI. Alarm match sets TIMG_INT_RAW.T0/T1.

const REG_COUNT: usize = 0xA8 / 4;

// Timer 0 register offsets (timer 1 is +0x24).
const T0CONFIG: u32 = 0x00;
const T0LO: u32 = 0x04;
const T0HI: u32 = 0x08;
const T0UPDATE: u32 = 0x0C;
const T0ALARMLO: u32 = 0x10;
const T0ALARMHI: u32 = 0x14;
const T0LOADLO: u32 = 0x18;
const T0LOADHI: u32 = 0x1C;
const T0LOAD: u32 = 0x20;

const INT_ENA: u32 = 0x98;
const INT_RAW: u32 = 0x9C;
const INT_ST: u32 = 0xA0;
const INT_CLR: u32 = 0xA4;

/// INT_RAW/ENA/ST/CLR bit for timer T (QEMU esp32_timg.h).
pub const INT_T0: u32 = 1 << 0;
pub const INT_T1: u32 = 1 << 1;
pub const INT_WDT: u32 = 1 << 2;

/// CONFIG field bits (QEMU esp32_timg.h).
const CFG_EN: u32 = 1 << 31;
const CFG_INCREASE: u32 = 1 << 30;
const CFG_AUTORELOAD: u32 = 1 << 29;
const CFG_DIVIDER: u32 = 0xFFFF << 13;
const CFG_ALARM: u32 = 1 << 10;

#[derive(Clone, Copy, Default)]
struct TimerState {
    counter: u64,
    hi_latched: u32,
}

pub struct Timg {
    regs: [u32; REG_COUNT],
    t0: TimerState,
    t1: TimerState,
}

impl Timg {
    pub fn new() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
            t0: TimerState::default(),
            t1: TimerState::default(),
        }
    }

    /// Advance time by `cycles` (1 cycle = 1 CPU clock; dividers per CONFIG).
    pub fn tick(&mut self, cycles: u64) {
        for _ in 0..cycles {
            Self::tick_timer(&mut self.regs, &mut self.t0, 0);
            Self::tick_timer(&mut self.regs, &mut self.t1, 1);
        }
    }

    fn tick_timer(regs: &mut [u32; REG_COUNT], t: &mut TimerState, which: usize) {
        let t1_off = if which == 0 { 0 } else { 0x24 };
        let cfg = regs[((T0CONFIG + t1_off) / 4) as usize];
        if cfg & CFG_EN == 0 {
            return;
        }
        let div = ((cfg & CFG_DIVIDER) >> 13) as u64;
        let step: u64 = if div == 0 { 1 } else { div };
        let mut new = if cfg & CFG_INCREASE != 0 {
            t.counter.wrapping_add(step)
        } else {
            t.counter.wrapping_sub(step)
        };
        let alarm_lo = regs[((T0ALARMLO + t1_off) / 4) as usize];
        let alarm_hi = regs[((T0ALARMHI + t1_off) / 4) as usize];
        let alarm = ((alarm_hi as u64) << 32) | alarm_lo as u64;
        if cfg & CFG_ALARM != 0 && new == alarm {
            if cfg & CFG_AUTORELOAD != 0 {
                let load_lo = regs[((T0LOADLO + t1_off) / 4) as usize];
                let load_hi = regs[((T0LOADHI + t1_off) / 4) as usize];
                new = (load_lo as u64) | ((load_hi as u64) << 32);
            }
            // Level-triggered alarm: RAW stays set until INT_CLR.
            regs[(INT_RAW / 4) as usize] |= if which == 0 { INT_T0 } else { INT_T1 };
        }
        t.counter = new;
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            INT_ST => self.int_st(),
            INT_RAW => self.regs[(INT_RAW / 4) as usize],
            T0LO => self.t0.counter as u32,
            T0HI => self.t0.hi_latched,
            // Timer 1 register block starts at +0x24 (QEMU esp32_timg.h).
            0x28 => self.t1.counter as u32, // T1LO
            0x2C => self.t1.hi_latched,     // T1HI
            _ => self.regs[(offset / 4) as usize],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            T0UPDATE => {
                self.t0.hi_latched = (self.t0.counter >> 32) as u32;
                self.regs[(T0UPDATE / 4) as usize] = value;
            }
            0x30 => {
                // T1UPDATE.
                self.t1.hi_latched = (self.t1.counter >> 32) as u32;
                self.regs[(0x30 / 4) as usize] = value;
            }
            T0LOAD => {
                self.t0.counter = (self.regs[(T0LOADLO / 4) as usize] as u64)
                    | ((self.regs[(T0LOADHI / 4) as usize] as u64) << 32);
                self.regs[(T0LOAD / 4) as usize] = value;
            }
            0x44 => {
                // T1LOAD.
                self.t1.counter = (self.regs[(0x3C / 4) as usize] as u64)
                    | ((self.regs[(0x40 / 4) as usize] as u64) << 32);
                self.regs[(0x44 / 4) as usize] = value;
            }
            INT_CLR => self.regs[(INT_RAW / 4) as usize] &= !value,
            _ => self.regs[(offset / 4) as usize] = value,
        }
    }

    /// INT_ST = RAW & ENA (TRM TIMG_INT_ST).
    pub fn int_st(&self) -> u32 {
        self.regs[(INT_RAW / 4) as usize] & self.regs[(INT_ENA / 4) as usize]
    }
}
impl Default for Timg {
    fn default() -> Self {
        Self::new()
    }
}
