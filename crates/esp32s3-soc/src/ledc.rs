//! ESP32-S3 LEDC (LED PWM) controller.
//!
//! Register layout follows `ledc_struct.h` (S3): the `channel_group` array
//! comes first — 8 channels, each 5 words (`conf0`, `hpoint`, `duty`, `conf1`,
//! `duty_rd`) = 0x14 stride — then `timer_group` with 4 timers, each 2 words
//! (`conf`, `value`) at 0xA0, then `int_raw`/`int_st`/`int_ena`/`int_clr` at
//! 0xC0..0xCC and `conf` at 0xD0.
//!
//! TRM §23: the timer counts `1 << duty_resolution` and the output is high
//! while `count < duty`. The esp-idf driver stores `duty = user_duty << 4`
//! (4 fractional bits, see `ledc_ll_set_duty`), so the comparator value is
//! `duty_reg >> 4`. Output signal `LEDC_CHn` = GPIO-matrix signal 73+n
//! (`gpio_sig_map.h`); `sig_out_en` (conf0 bit 2) gates the pad.

/// LEDC interrupt source for the matrix (ETS_LEDC_INTR_SOURCE = 35).
pub const LEDC_INTR_SOURCE: u32 = 35;

const TIMER_COUNT: usize = 4;
const CHANNEL_COUNT: usize = 8;

// Word (4-byte) indices within the register file.
const CH_W: usize = 5; // conf0, hpoint, duty, conf1, duty_rd
const CH_CONF0: usize = 0;
const CH_HPOINT: usize = 1;
const CH_DUTY: usize = 2;
const CH_CONF1: usize = 3;
const CH_DUTY_RD: usize = 4;

const TIMER_BASE_W: usize = 0xA0 / 4; // 40
const TIMER_W: usize = 2; // conf, value
const TN_CONF: usize = 0;
const TN_VALUE: usize = 1;

const INT_RAW_W: usize = 0xC0 / 4; // 48
const INT_ST_W: usize = 49;
const INT_ENA_W: usize = 50;
const CONF_W: usize = 0xD0 / 4; // 52

// Cover up to reserved_d4 @ 0xD4.
const REG_COUNT: usize = 54;

// channel conf0
const CH_TIMER_SEL: u32 = 0x3; // bits [0:1]
const CH_SIG_OUT_EN: u32 = 1 << 2;
const CH_IDLE_LV: u32 = 1 << 3;

// channel duty: 19-bit field, esp-idf stores (user_duty << 4)
const LEDC_DUTY_FRAC: u32 = 4;
// channel conf1 fade fields (ledc_struct.h ch.conf1).
const CH_DUTY_SCALE: u32 = 0x3FF; // bits [9:0]
const CH_DUTY_CYCLE_SHIFT: u32 = 10; // bits [19:10]
const CH_DUTY_NUM_SHIFT: u32 = 20; // bits [29:20]
const CH_DUTY_INC: u32 = 1 << 30;
const CH_DUTY_START: u32 = 1 << 31;
// Duty field width (19 bits) and fade-done interrupt base bit
// (duty_chng_end_lsch0..7 = INT bits 4..11).
const DUTY_MAX: u32 = 0x7_FFFF;
const FADE_DONE_BIT: u32 = 4;

// timer conf
const TIMER_DUTY_RES: u32 = 0xF; // bits [3:0], resolution in bits
const TIMER_CLOCK_DIV: u32 = 0x3FFFF << 4; // bits [21:4]
const TIMER_PAUSE: u32 = 1 << 22;
const TIMER_RST: u32 = 1 << 23;

#[derive(Clone, Copy, Default)]
struct FadeCh {
    /// Fade running (latched by a duty_start pulse).
    active: bool,
    /// Timer wraps remaining before the fade completes.
    remaining: u32,
    /// Wraps counted toward the current duty step.
    cycles: u32,
}

#[derive(Clone)]
pub struct Lcdc {
    regs: [u32; REG_COUNT],
    counters: [u32; TIMER_COUNT],
    frac: [u32; TIMER_COUNT],
    fade: [FadeCh; 8],
    /// Any fade running (tick fast path skips the channel loop otherwise).
    fade_any: bool,
}

impl Lcdc {
    pub fn new() -> Self {
        Self {
            // Reset: all channel/timer regs 0; signal routing is undefined
            // but sig_out_en=0 leaves every channel at its idle level.
            regs: [0; REG_COUNT],
            counters: [0; TIMER_COUNT],
            frac: [0; TIMER_COUNT],
            fade: [FadeCh::default(); 8],
            fade_any: false,
        }
    }

    #[inline]
    fn ch_word(c: usize, sub: usize) -> usize {
        c * CH_W + sub
    }

    #[inline]
    fn tm_word(t: usize, sub: usize) -> usize {
        TIMER_BASE_W + t * TIMER_W + sub
    }

    #[inline]
    fn channel_conf0(&self, c: usize) -> u32 {
        self.regs[Self::ch_word(c, CH_CONF0)]
    }

    #[inline]
    fn channel_timer(&self, c: usize) -> u32 {
        self.regs[Self::ch_word(c, CH_CONF0)] & CH_TIMER_SEL
    }

    #[inline]
    fn channel_idle(&self, c: usize) -> u32 {
        ((self.channel_conf0(c) & CH_IDLE_LV) != 0) as u32
    }

    #[inline]
    fn channel_duty(&self, c: usize) -> u32 {
        self.regs[Self::ch_word(c, CH_DUTY)]
    }

    #[inline]
    fn timer_conf(&self, t: usize) -> u32 {
        self.regs[Self::tm_word(t, TN_CONF)]
    }

    #[inline]
    fn timer_div(&self, t: usize) -> u32 {
        (self.timer_conf(t) & TIMER_CLOCK_DIV) >> 4
    }

    /// Timer resolution in bits (0..15).
    #[inline]
    fn timer_res(&self, t: usize) -> u32 {
        self.timer_conf(t) & TIMER_DUTY_RES
    }

    #[inline]
    fn timer_running(&self, t: usize) -> bool {
        let conf = self.timer_conf(t);
        if (conf & TIMER_PAUSE) != 0 {
            return false;
        }
        if (conf & TIMER_RST) != 0 {
            return false;
        }
        self.timer_div(t) != 0
    }

    #[inline]
    fn timer_value(&self, t: usize) -> u32 {
        let res = self.timer_res(t);
        if res == 0 {
            return 0;
        }
        self.counters[t] % (1u32 << res)
    }

    /// Logical level (0/1) of LEDC channel output `c`.
    fn channel_level(&self, c: usize) -> u32 {
        let conf0 = self.channel_conf0(c);
        if (conf0 & CH_SIG_OUT_EN) == 0 {
            return self.channel_idle(c);
        }
        let t = self.channel_timer(c) as usize;
        if !self.timer_running(t) {
            return self.channel_idle(c);
        }
        let res = self.timer_res(t);
        if res == 0 {
            return self.channel_idle(c);
        }
        let duty_user = (self.channel_duty(c) >> LEDC_DUTY_FRAC) & 0x7FFFF;
        let cnt = self.counters[t] % (1u32 << res);
        let active = cnt < duty_user;
        if active {
            1 - self.channel_idle(c)
        } else {
            self.channel_idle(c)
        }
    }

    /// Interrupt status = RAW & ENA.
    pub fn int_st(&self) -> u32 {
        self.regs[INT_RAW_W] & self.regs[INT_ENA_W]
    }

    /// Level for a GPIO-matrix LEDC signal (73..80 → channel 0..7).
    pub fn signal_level(&self, sig: u32) -> u32 {
        if !(73..=80).contains(&sig) {
            return 0;
        }
        let c = (sig - 73) as usize;
        if c >= CHANNEL_COUNT {
            return 0;
        }
        self.channel_level(c)
    }

    pub fn read32(&self, off: u32) -> u32 {
        let o = off as usize;
        if o >= REG_COUNT * 4 {
            return 0;
        }
        let w = o / 4;
        if w < TIMER_BASE_W {
            // Channel register. DUTY_RD is a live view of the current duty
            // (TRM: read-only readback): the fade ISR reads progress through
            // it and chains rounds toward the target, so a stale zero would
            // restart every round from scratch forever.
            if w % CH_W == CH_DUTY_RD {
                return self.regs[w - (CH_DUTY_RD - CH_DUTY)];
            }
            return self.regs[w];
        }
        if w < INT_RAW_W {
            let local = w - TIMER_BASE_W;
            if local % TIMER_W == TN_VALUE {
                return self.timer_value(local / TIMER_W);
            }
            return self.regs[w];
        }
        self.regs[w]
    }

    pub fn write32(&mut self, off: u32, val: u32) {
        let o = off as usize;
        if o >= REG_COUNT * 4 {
            return;
        }
        let w = o / 4;
        if w < TIMER_BASE_W {
            let sub = w % CH_W;
            match sub {
                CH_DUTY => self.regs[w] = val,
                CH_CONF1 => {
                    // duty_start is a self-clearing start pulse (reads back
                    // 0): every write with the bit set latches a fade run
                    // (scale/cycle/num/inc from this same write). The real
                    // driver rewrites it on every round (the fade ISR chains
                    // rounds until the target), so edge-only latching would
                    // miss re-arms against the sticky bit.
                    self.regs[w] = val & !CH_DUTY_START;
                    if val & CH_DUTY_START != 0 {
                        let c = w / CH_W;
                        self.fade[c].active = (val >> CH_DUTY_NUM_SHIFT) & 0x3FF != 0;
                        self.fade[c].remaining = (val >> CH_DUTY_NUM_SHIFT) & 0x3FF;
                        self.fade[c].cycles = 0;
                        if self.fade[c].active {
                            self.fade_any = true;
                        }
                    }
                }
                CH_CONF0 | CH_HPOINT => self.regs[w] = val,
                CH_DUTY_RD => {} // read-only
                _ => {}
            }
            return;
        }
        if w < INT_RAW_W {
            self.regs[w] = val;
            return;
        }
        match o {
            0xC8 => self.regs[INT_ENA_W] = val,
            0xCC => {
                let clr = val;
                let raw = self.regs[INT_RAW_W] & !clr;
                self.regs[INT_RAW_W] = raw;
                self.regs[INT_ST_W] = raw & self.regs[INT_ENA_W];
            }
            0xD0 => self.regs[CONF_W] = val,
            _ => {} // int_raw read-only, reserved_d4
        }
    }

    /// Advance all timers by one emulator step. The counter increments every
    /// `div` steps (fractional accumulator; TRM clock divider is 1/256 APB
    /// sub-steps per counter tick).
    pub fn tick(&mut self) {
        for t in 0..TIMER_COUNT {
            let conf = self.timer_conf(t);
            if (conf & TIMER_RST) != 0 {
                self.counters[t] = 0;
                self.frac[t] = 0;
                continue;
            }
            if (conf & TIMER_PAUSE) != 0 {
                continue;
            }
            let div = self.timer_div(t);
            if div == 0 {
                continue;
            }
            let res = self.timer_res(t);
            if res == 0 {
                continue;
            }
            let period = 1u32 << res;
            self.frac[t] += 256;
            while self.frac[t] >= div {
                self.frac[t] -= div;
                let next = self.counters[t] + 1;
                self.counters[t] = if next >= period { 0 } else { next };
                self.fade_clock(t);
            }
        }
    }

    /// One timer-clock step on timer `t`: advance fades bound to it. Every
    /// `duty_cycle` clocks the duty steps by `duty_scale` (up/down by
    /// `duty_inc`), `duty_num` times, then the fade-done interrupt fires
    /// (the driver ISR chains further rounds toward the target itself).
    fn fade_clock(&mut self, t: usize) {
        if !self.fade_any {
            return;
        }
        let mut any = false;
        for c in 0..CHANNEL_COUNT {
            if self.channel_timer(c) as usize != t || !self.fade[c].active {
                continue;
            }
            any = true;
            let conf1 = self.regs[Self::ch_word(c, CH_CONF1)];
            let need = ((conf1 >> CH_DUTY_CYCLE_SHIFT) & 0x3FF).max(1);
            self.fade[c].cycles += 1;
            if self.fade[c].cycles < need {
                continue;
            }
            self.fade[c].cycles = 0;
            // The fade scale steps the duty in *user* units: the driver
            // computes rounds as `steps = delta_user / scale_user` and
            // programs the scale value verbatim, so each hardware step must
            // cover scale<<4 register units for the composition to converge
            // (verified: with a plain `scale` step the ISR reads a truncated
            // user duty that never advances, reprogramming the same round
            // forever while CH_DUTY oscillates in place).
            let scale = (conf1 & CH_DUTY_SCALE) << LEDC_DUTY_FRAC;
            let duty = self.regs[Self::ch_word(c, CH_DUTY)];
            let next = if conf1 & CH_DUTY_INC != 0 {
                duty.saturating_add(scale).min(DUTY_MAX)
            } else {
                duty.saturating_sub(scale)
            };
            self.regs[Self::ch_word(c, CH_DUTY)] = next;
            if self.fade[c].remaining > 0 {
                self.fade[c].remaining -= 1;
            }
            if self.fade[c].remaining == 0 {
                self.fade[c].active = false;
                self.regs[INT_RAW_W] |= 1 << (FADE_DONE_BIT + c as u32);
                self.regs[INT_ST_W] = self.regs[INT_RAW_W] & self.regs[INT_ENA_W];
            }
        }
        self.fade_any = any;
    }
}

impl Default for Lcdc {
    fn default() -> Self {
        Self::new()
    }
}
