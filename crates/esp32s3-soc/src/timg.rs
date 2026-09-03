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

/// MWDT (Main Watchdog Timer) register block, per ESP-IDF `timer_group_struct.h`
/// (the WDT sits at 0x48..0x64 within each TIMG, after the two hw_timer blocks).
const WDT_CONFIG0: u32 = 0x48;
const WDT_CONFIG1: u32 = 0x4C;
const WDT_CONFIG2: u32 = 0x50;
const WDT_CONFIG3: u32 = 0x54;
const WDT_CONFIG4: u32 = 0x58;
const WDT_CONFIG5: u32 = 0x5C;
const WDT_FEED: u32 = 0x60;
const WDT_WPROTECT: u32 = 0x64;
/// Write-protect key: WDT config registers are only writable while
/// WDT_WPROTECT == this value (TRM timg_wdtwprotect_reg_t). The reset default
/// already equals the key, so the first config writes succeed.
const WDT_WKEY: u32 = 0x50D8_3AA1;
/// Feed token written to WDT_FEED to reset the watchdog counter (any value
/// works on real silicon; we accept any write to the feed register).
const _WDT_FEED_KEY: u32 = 0xABAD_1DEA;
/// WDT enable bit in WDT_CONFIG0 (timg_wdtconfig0_reg_t.wdt_en).
const WDT_EN: u32 = 1 << 31;
/// Stage action field shifts in WDT_CONFIG0: stg0 [30:29], stg1 [28:27],
/// stg2 [26:25], stg3 [24:23]. Action codes: 0 = disabled, 1 = interrupt,
/// 2 = reset CPU, 3 = reset system.
const WDT_STG_SHIFT: [u32; 4] = [29, 27, 25, 23];

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
    /// APB-cycle accumulator for the clock divider: the 64-bit counter
    /// advances once per DIVIDER APB cycles (TRM: timer clock =
    /// APB_CLK / DIVIDER).
    div_acc: u64,
}

pub struct Timg {
    regs: [u32; REG_COUNT],
    t0: TimerState,
    t1: TimerState,
    /// One-off calibration completed (rdy + cycling_data_vld raised).
    cali_done: bool,
    /// XTAL-cycle countdown until the cycling timeout fires.
    cali_timeout: u64,
    /// MWDT write-protect key currently latched in WDT_WPROTECT.
    wdt_wkey: u32,
    /// Watchdog counter (MWDT clock ticks since last feed / enable).
    wdt_count: u64,
    /// Fractional remainder of the prescaler accumulator (1/N of a tick).
    wdt_clk_acc: u64,
    /// True while the WDT is enabled and counting (latched on the enable edge).
    wdt_running: bool,
    /// Each stage's action has been taken (so it fires once until a feed).
    wdt_stage_fired: [bool; 4],
    /// A reset-action stage has elapsed; the machine consumes this to reboot.
    wdt_reset: bool,
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
            wdt_wkey: WDT_WKEY,
            wdt_count: 0,
            wdt_clk_acc: 0,
            wdt_running: false,
            wdt_stage_fired: [false; 4],
            wdt_reset: false,
        }
    }

    /// Advance time by `cycles` (1 cycle = 1 CPU clock; dividers per CONFIG).
    pub fn tick(&mut self, cycles: u64) {
        // Fast path: both timers disabled AND WDT disabled AND cali done → nothing to tick.
        let t0_en = self.regs[((T0CONFIG) / 4) as usize] & CFG_EN != 0;
        let t1_en = self.regs[((T0CONFIG + 0x24) / 4) as usize] & CFG_EN != 0;
        let wdt_en = self.regs[(WDT_CONFIG0 / 4) as usize] & WDT_EN != 0;
        if !t0_en && !t1_en && !wdt_en && self.cali_done {
            return;
        }
        for _ in 0..cycles {
            Self::tick_timer(&mut self.regs, &mut self.t0, 0);
            Self::tick_timer(&mut self.regs, &mut self.t1, 1);
            self.tick_cali();
            if wdt_en {
                self.tick_wdt();
            }
        }
    }

    /// Advance the Main Watchdog Timer. The MWDT clock is the APB clock divided
    /// by WDT_CLK_PRESCALE (timg_wdtconfig1_reg_t, default 1). The WDT counts
    /// up; on reaching each stage's cumulative hold it runs that stage's action
    /// (interrupt / reset). Feeding (WDT_FEED write) or disabling restarts it.
    fn tick_wdt(&mut self) {
        let cfg0 = self.regs[(WDT_CONFIG0 / 4) as usize];
        if cfg0 & WDT_EN == 0 {
            // Disabled: keep the counter clear so (re-)enabling starts fresh.
            self.wdt_count = 0;
            self.wdt_clk_acc = 0;
            self.wdt_stage_fired = [false; 4];
            self.wdt_running = false;
            return;
        }
        if !self.wdt_running {
            self.wdt_count = 0;
            self.wdt_clk_acc = 0;
            self.wdt_stage_fired = [false; 4];
            self.wdt_running = true;
        }
        let prescale = ((self.regs[(WDT_CONFIG1 / 4) as usize] >> 16) & 0xFFFF).max(1) as u64;
        self.wdt_clk_acc += 1;
        if self.wdt_clk_acc < prescale {
            return;
        }
        self.wdt_clk_acc -= prescale;
        self.wdt_count = self.wdt_count.wrapping_add(1);
        // Cumulative stage thresholds (TRM: stage N fires at sum of holds[0..=N]).
        let mut thr: u64 = 0;
        for (i, &shift) in WDT_STG_SHIFT.iter().enumerate() {
            let hold = self.regs[(WDT_CONFIG2 as usize / 4) + i] as u64;
            thr = thr.wrapping_add(hold);
            if thr == 0 {
                continue; // no timeout at this cumulative point
            }
            if !self.wdt_stage_fired[i] && self.wdt_count >= thr {
                self.wdt_stage_fired[i] = true;
                let action = (cfg0 >> shift) & 0x3;
                match action {
                    0 => {} // disabled / no action
                    1 => {
                        // Interrupt: raise the WDT bit in INT_RAW (level, cleared
                        // by INT_CLR like the other TIMG interrupts).
                        self.regs[(INT_RAW / 4) as usize] |= INT_WDT;
                    }
                    2 | 3 => {
                        // CPU / system reset request consumed by the machine.
                        self.wdt_reset = true;
                    }
                    _ => {}
                }
            }
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
        // Clock divider (TRM TIMG clock source): the prescaler counts APB
        // cycles and the counter advances once per DIVIDER of them.
        // DIVIDER=0 keeps the old every-cycle behavior. (The previous code
        // used DIVIDER as the counter STEP, racing 80x too fast and skipping
        // past alarm values so INT_RAW never latched — the multi_irq
        // sketch's 100/200-count alarms at divider 80 were never hit.)
        t.div_acc = t.div_acc.wrapping_add(1);
        if div != 0 && t.div_acc < div {
            return;
        }
        t.div_acc = 0;
        let mut new = if cfg & CFG_INCREASE != 0 {
            t.counter.wrapping_add(1)
        } else {
            t.counter.wrapping_sub(1)
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
            WDT_CONFIG0 | WDT_CONFIG1 | WDT_CONFIG2 | WDT_CONFIG3 | WDT_CONFIG4 | WDT_CONFIG5 => {
                // Write-protected: only honored while WDT_WPROTECT holds the key
                // (the reset default already has the key set).
                if self.wdt_wkey == WDT_WKEY {
                    self.regs[(offset / 4) as usize] = value;
                }
            }
            WDT_WPROTECT => {
                self.wdt_wkey = value;
                self.regs[(WDT_WPROTECT / 4) as usize] = value;
            }
            WDT_FEED => {
                // Feeding resets the counter and all latched stage actions.
                self.wdt_count = 0;
                self.wdt_clk_acc = 0;
                self.wdt_stage_fired = [false; 4];
            }
            _ => self.regs[(offset / 4) as usize] = value,
        }
    }

    /// INT_ST = RAW & ENA (TRM TIMG_INT_ST).
    pub fn int_st(&self) -> u32 {
        self.regs[(INT_RAW / 4) as usize] & self.regs[(INT_ENA / 4) as usize]
    }

    /// Consume (clear) a pending WDT reset request. The machine polls this each
    /// step and reboots when it returns true. Returns false once cleared.
    pub fn consume_reset(&mut self) -> bool {
        let r = self.wdt_reset;
        self.wdt_reset = false;
        r
    }
}
impl Default for Timg {
    fn default() -> Self {
        Self::new()
    }
}
