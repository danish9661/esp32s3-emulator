//! ESP32-S3 Timer Group (TIMG0/TIMG1) peripheral model.
//!
//! Register layout per ESP-IDF `timer_group_struct.h` (esp32s3): two 64-bit
//! timers (hw_timer[2]) at 0x00/0x24, WDT block 0x48..0x64 (latched only),
//! RTC calibration block (RTCCALICFG/1 at 0x68/0x6C), interrupt block at
//! 0x70..0x7C and RTCCALICFG2 at 0x80 (S3 layout differs from classic ESP32
//! where INT sat at 0x68..0x74 and no RTCCALI block).
//!
//! Time model: the frontend drives time via `tick(cycles)` (JS frame loop);
//! each enabled timer increments its counter by `cycles >> divider`. Counter
//! reads follow TRM semantics: TnLO returns the current low word, TnHI
//! returns the high word latched by a TnUPDATE write; TnLOAD loads the
//! counter from TnLOADLO/HI. Alarm match sets TIMG_INT_RAW.Tn.
//!
//! RTC calibration: writing RTCCALICFG.rtc_cali_start (bit 31) arms a
//! one-off count of XTAL cycles over rtc_cali_max slow-clock cycles; the
//! count completes immediately in the model (real hardware takes
//! max/32.768k..150k seconds), setting rtc_cali_rdy (bit 15) and loading
//! RTCCALICFG1.rtc_cali_value ([31:7], IDF rtc_clk_cal_internal polls both).
//! With rtc_cali_start_cycling (bit 12, default 1) set and max != 0 the
//! cycling-data-valid bit (RTCCALICFG1 bit 0) is also raised.  The timeout
//! counter (RTCCALICFG2.rtc_cali_timeout, bit 0) fires after
//! rtc_cali_timeout_thres ticks when a cycling calibration never completes.

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

// RTC calibration block (S3 timer_group_struct.h).
const RTCCALICFG: u32 = 0x68;
const RTCCALICFG1: u32 = 0x6C;
const RTCCALICFG2: u32 = 0x80;

// RTCCALICFG field bits (timg_rtccalicfg_reg_t).
const CALI_START_CYCLING: u32 = 1 << 12;
const CALI_CLK_SEL: u32 = 0x3 << 13;
const CALI_RDY: u32 = 1 << 15;
const CALI_MAX: u32 = 0x7FFF << 16;
const CALI_START: u32 = 1 << 31;

const INT_ENA: u32 = 0x70;
const INT_RAW: u32 = 0x74;
const INT_ST: u32 = 0x78;
const INT_CLR: u32 = 0x7C;

/// INT_RAW/ENA/ST/CLR bit for timer T (timg_int_raw_timers_reg_t).
pub const INT_T0: u32 = 1 << 0;
pub const INT_T1: u32 = 1 << 1;
pub const INT_WDT: u32 = 1 << 2;

/// CONFIG field bits (timg_tnconfig_reg_t).
const CFG_EN: u32 = 1 << 31;
const CFG_INCREASE: u32 = 1 << 30;
const CFG_AUTORELOAD: u32 = 1 << 29;
const CFG_DIVIDER: u32 = 0xFFFF << 13;
const CFG_ALARM: u32 = 1 << 10;

/// XTAL cycles per slow-clock cycle for RTCCALICFG.clk_sel (0 = rc_slow
/// 150 kHz, 1 = rc_fast_div 8MD256 31.25 kHz, 2 = xtal_32k 32768 Hz).
const CALI_RATIO: [u64; 3] = [267, 1280, 1221];

#[derive(Clone, Copy, Default)]
struct TimerState {
    counter: u64,
    hi_latched: u32,
}

pub struct Timg {
    regs: [u32; REG_COUNT],
    t0: TimerState,
    t1: TimerState,
    /// One-off calibration completed (rdy + cycling_data_vld raised).
    cali_done: bool,
    /// XTAL-cycle countdown until the cycling timeout fires.
    cali_timeout: u64,
}

impl Timg {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        // Reset defaults (timg_rtccalicfg_reg_t / rtccalicfg2_reg_t):
        // start_cycling = 1, clk_sel = 1, max = 1; timeout_rst_cnt = 3,
        // timeout_thres = 0x1FFFFFF.
        regs[(RTCCALICFG / 4) as usize] = CALI_START_CYCLING | (1 << 13) | (1 << 16);
        regs[(RTCCALICFG2 / 4) as usize] = (3 << 3) | (0x1FFFFFF << 7);
        Self {
            regs,
            t0: TimerState::default(),
            t1: TimerState::default(),
            cali_done: false,
            cali_timeout: 0,
        }
    }

    /// Advance time by `cycles` (1 cycle = 1 CPU clock; dividers per CONFIG).
    pub fn tick(&mut self, cycles: u64) {
        for _ in 0..cycles {
            Self::tick_timer(&mut self.regs, &mut self.t0, 0);
            Self::tick_timer(&mut self.regs, &mut self.t1, 1);
            self.tick_cali();
        }
    }

    /// RTC calibration timeout counter: fires when the count exceeds
    /// rtc_cali_timeout_thres while a cycling calibration is armed.
    fn tick_cali(&mut self) {
        if self.cali_done {
            return;
        }
        if self.regs[(RTCCALICFG / 4) as usize] & CALI_START_CYCLING != 0 {
            self.cali_timeout = self.cali_timeout.wrapping_add(1);
            let thres = (self.regs[(RTCCALICFG2 / 4) as usize] >> 7) as u64;
            if self.cali_timeout > thres {
                self.regs[(RTCCALICFG2 / 4) as usize] |= 1; // rtc_cali_timeout
            }
        } else {
            self.cali_timeout = 0;
        }
    }

    /// Run a calibration: value = max * (XTAL / slow-clock) cycles.
    fn run_cali(&mut self, cfg: u32) {
        let max = ((cfg & CALI_MAX) >> 16) as u64;
        let sel = ((cfg & CALI_CLK_SEL) >> 13) as usize;
        let count = max.saturating_mul(CALI_RATIO[sel.min(2)]);
        let count = count.min(0x1FFFFFF) as u32; // 25-bit CALI_VALUE field
        self.regs[(RTCCALICFG1 / 4) as usize] = (count << 7) | 1; // vld
        self.cali_done = true;
        self.regs[(RTCCALICFG2 / 4) as usize] &= !1; // clear timeout
        self.cali_timeout = 0;
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
            RTCCALICFG => {
                self.regs[(RTCCALICFG / 4) as usize] | if self.cali_done { CALI_RDY } else { 0 }
            }
            T0LO => self.t0.counter as u32,
            T0HI => self.t0.hi_latched,
            // Timer 1 register block starts at +0x24.
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
            RTCCALICFG => {
                // rdy is RO; store the rest, then (re)arm the count when a
                // start or a cycling run with a nonzero max is written.
                self.regs[(RTCCALICFG / 4) as usize] = value & !CALI_RDY;
                let max = ((value & CALI_MAX) >> 16) != 0;
                if value & CALI_START != 0 || (value & CALI_START_CYCLING != 0 && max) {
                    self.run_cali(value);
                }
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
