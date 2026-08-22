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

// timer conf
const TIMER_DUTY_RES: u32 = 0xF; // bits [3:0], resolution in bits
const TIMER_CLOCK_DIV: u32 = 0x3FFFF << 4; // bits [21:4]
const TIMER_PAUSE: u32 = 1 << 22;
const TIMER_RST: u32 = 1 << 23;

#[derive(Clone)]
pub struct Lcdc {
    regs: [u32; REG_COUNT],
    counters: [u32; TIMER_COUNT],
    frac: [u32; TIMER_COUNT],
}

impl Lcdc {
    pub fn new() -> Self {
        Self {
            // Reset: all channel/timer regs 0; signal routing is undefined
            // but sig_out_en=0 leaves every channel at its idle level.
            regs: [0; REG_COUNT],
            counters: [0; TIMER_COUNT],
            frac: [0; TIMER_COUNT],
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
            // channel register
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
                    // duty_start is a software pulse that (re)loads the duty
                    // into the comparator; the PWM runs whenever the timer is
                    // running and sig_out_en is set.
                    self.regs[w] = val;
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
                self.counters[t] = (self.counters[t] + 1) % period;
            }
        }
    }
}

impl Default for Lcdc {
    fn default() -> Self {
        Self::new()
    }
}
