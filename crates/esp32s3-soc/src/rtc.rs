//! ESP32-S3 RTC_CNTL slow-clock timer.
//!
//! Register layout per the TRM (RTC_CNTL chapter): TIME_UPDATE_REG at
//! 0x6000800C (TIME_UPDATE bit 31 — write 1 to latch the running count into
//! the TIME_VALUE registers), TIME0_REG at 0x60008010 and TIME1_REG at
//! 0x60008014 (the latched low/high 32 bits; IDF `rtc_time_get()` reads
//! them, `rtc_time_us_to_slowclk()` converts, and `rtc_init`'s analog-LDO
//! wait loop plus `rtc_clk_cal_internal` spin on the count).
//!
//! Time model: the counter advances at the RC_SLOW oscillator rate
//! (nominally 32.5 kHz, TRM §27); the frontend drives CPU time via
//! `tick(cycles)` (240 MHz / 32.5 kHz = 7385 CPU cycles per slow tick).
//! Without this model `rtc_time_get()` returns 0 forever and every
//! slow-clock timeout loop spins indefinitely (observed: rtc_init stuck at
//! 0x42020a74 in the Arduino core boot).

/// RTC_CNTL register block base (TRM RTC_CNTL memory map).
pub const RTC_CNTL_BASE: u32 = 0x6000_8000;

// TIME_UPDATE_REG bit 31 = TIME_UPDATE: write 1 to latch the count.
const TIME_UPDATE_OFF: u32 = 0x0C;
const TIME_UPDATE_BIT: u32 = 1 << 31;
const TIME_VALUE_LO_OFF: u32 = 0x10;
const TIME_VALUE_HI_OFF: u32 = 0x14;

// RC_SLOW nominal frequency (TRM §27): 32.5 kHz; CPU clock 240 MHz.
const SLOW_CLK_DIV: u64 = 240_000_000 / 32_500;

#[derive(Default)]
pub struct Rtc {
    /// Free-running slow-clock counter (RTC time, TRM RTC_CNTL_TIME0/1).
    count: u64,
    /// Count latched by the last TIME_UPDATE write.
    latched: u64,
    /// CPU-cycle accumulator for the slow-clock divider.
    acc: u64,
}

impl Rtc {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn tick(&mut self, cycles: u64) {
        self.acc += cycles;
        while self.acc >= SLOW_CLK_DIV {
            self.acc -= SLOW_CLK_DIV;
            self.count += 1;
        }
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            TIME_VALUE_LO_OFF => self.latched as u32,
            TIME_VALUE_HI_OFF => (self.latched >> 32) as u32,
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        if offset == TIME_UPDATE_OFF && value & TIME_UPDATE_BIT != 0 {
            self.latched = self.count;
        }
    }
}
