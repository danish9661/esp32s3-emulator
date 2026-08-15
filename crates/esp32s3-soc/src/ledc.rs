//! ESP32-S3 LEDC (LED PWM controller) model.
//!
//! Register layout per the TRM LEDC chapter (single module: 4 timers, 8
//! channels; channel stride 0x10 starting at CH0_CONF0 = 0x20).
//!
//! Timing model follows ESP-IDF `ledc_calculate_divisor`
//! (components/esp_driver_ledc/src/ledc.c): the driver programs
//!   div_param = (src_clk << 8) / (freq_hz * 2^duty_resolution)
//! into the 18-bit register field {clock_divider[17:8], [7:0] fractional},
//! so the hardware timer tick = src_clk * 256 / div_param APB cycles.
//! The timer counter wraps at 2^resolution (resolution = duty_resolution
//! field + 1, TRM) and a channel output is high while
//! counter < (CHx_DUTY >> (18 - resolution)).
//!
//! Modeled control: TIMER_CONF.pause/rst (rst resets the counter), CH_CONF0
//! duty_start (rising edge resets the counter) / sig_out_en / timer_sel.
//! Fade/hpoint/intr are latched but not modeled yet.  The output signal
//! indices used by the GPIO matrix are LEDC_CH0..CH7 = 73..80
//! (ESP32-S3 gpio_sig_map.h LEDC_LS_SIG_OUT0..7).

/// LEDC register offsets (TRM LEDC chapter).
pub const LEDC_TIMER_CONF_0: u32 = 0x00;
pub const LEDC_TIMER_VALUE_0: u32 = 0x04;
pub const LEDC_CH0_CONF0: u32 = 0x20;
pub const LEDC_CH0_DUTY: u32 = 0x28;
pub const LEDC_INT_RAW: u32 = 0x80;
pub const LEDC_INT_ST: u32 = 0x84;
pub const LEDC_INT_ENA: u32 = 0x88;
pub const LEDC_INT_CLR: u32 = 0x8C;

// TIMER_CONF field positions (TRM LEDC_TIMERx_CONF).
const TIMER_PAUSE: u32 = 1 << 0;
const TIMER_RST: u32 = 1 << 1;
const TIMER_CLOCK_DIV_SHIFT: u32 = 8; // 10-bit integer part [17:8]
const TIMER_DUTY_RES_SHIFT: u32 = 18; // 6-bit resolution-minus-one [23:18]
const TIMER_DUTY_RES_MASK: u32 = 0x3F << TIMER_DUTY_RES_SHIFT;

// CH_CONF0 field positions (TRM LEDC_CHx_CONF0).
const CH_DUTY_START: u32 = 1 << 2;
const CH_SIG_OUT_EN: u32 = 1 << 3;
const CH_TIMER_SEL_SHIFT: u32 = 4;

// Channel duty is 18 bits (TRM LEDC_CHx_DUTY).
const DUTY_BITS: u32 = 18;
const DUTY_MASK: u32 = (1 << DUTY_BITS) - 1;

// GPIO matrix signal indices for the 8 channels (ESP32-S3 signal table:
// LEDC_LS_SIG_OUT0..7 = 73..80, gpio_sig_map.h).
pub const LEDC_CH0_SIGNAL: u32 = 73;
pub const LEDC_CH_LAST_SIGNAL: u32 = 80;

const TIMER_COUNT: usize = 4;
const CHANNEL_COUNT: usize = 8;
const REG_COUNT: usize = 0x94 / 4;

/// The LEDC module.  `regs` holds every register (latched on write);
/// the PWM state machine derives timer/channel behavior from it.
pub struct Lcdc {
    regs: [u32; REG_COUNT],
    /// Timer phase counters (wrap at 2^resolution).
    counters: [u32; TIMER_COUNT],
    /// Fractional accumulators: each APB cycle adds 256; a timer tick
    /// fires once the accumulator reaches the 18-bit divider.
    frac: [u32; TIMER_COUNT],
}

impl Lcdc {
    pub fn new() -> Self {
        Self {
            regs: [0; REG_COUNT],
            counters: [0; TIMER_COUNT],
            frac: [0; TIMER_COUNT],
        }
    }

    fn timer_conf(&self, t: usize) -> u32 {
        self.regs[LEDC_TIMER_CONF_0 as usize / 4 + t * 2]
    }

    fn channel_conf0(&self, c: usize) -> u32 {
        self.regs[LEDC_CH0_CONF0 as usize / 4 + c * 4]
    }

    /// Advance the PWM state machines by `cycles` APB cycles.
    pub fn tick(&mut self, cycles: u64) {
        for _ in 0..cycles {
            for t in 0..TIMER_COUNT {
                let conf = self.timer_conf(t);
                if conf & TIMER_PAUSE != 0 {
                    continue;
                }
                // The timer runs while at least one channel with timer_sel
                // == t has duty_start set.
                let mut running = false;
                for c in 0..CHANNEL_COUNT {
                    let cc = self.channel_conf0(c);
                    if ((cc >> CH_TIMER_SEL_SHIFT) & 0x3) as usize == t && cc & CH_DUTY_START != 0 {
                        running = true;
                        break;
                    }
                }
                if !running {
                    continue;
                }
                let div = ((conf >> TIMER_CLOCK_DIV_SHIFT) & 0x3FF) << 8 | (conf & 0xFF);
                if div == 0 {
                    continue;
                }
                let res = ((conf & TIMER_DUTY_RES_MASK) >> TIMER_DUTY_RES_SHIFT) + 1;
                let period = 1u32 << res;
                let mut f = self.frac[t].wrapping_add(256);
                while f >= div {
                    f -= div;
                    self.counters[t] = (self.counters[t] + 1) & (period - 1);
                }
                self.frac[t] = f;
            }
        }
    }

    /// Current output level of LEDC channel `c` (0 or 1).
    pub fn channel_level(&self, c: usize) -> u32 {
        let cc = self.channel_conf0(c);
        if cc & CH_DUTY_START == 0 || cc & CH_SIG_OUT_EN == 0 {
            return 0;
        }
        let t = ((cc >> CH_TIMER_SEL_SHIFT) & 0x3) as usize;
        let conf = self.timer_conf(t);
        let res = ((conf & TIMER_DUTY_RES_MASK) >> TIMER_DUTY_RES_SHIFT) + 1;
        let duty = self.regs[LEDC_CH0_DUTY as usize / 4 + c * 4] & DUTY_MASK;
        let effective = duty >> (DUTY_BITS - res);
        u32::from(self.counters[t] < effective)
    }

    /// Level of a GPIO-matrix output signal (only LEDC signals 96..103).
    pub fn signal_level(&self, sig: u32) -> u32 {
        if (LEDC_CH0_SIGNAL..=LEDC_CH_LAST_SIGNAL).contains(&sig) {
            self.channel_level((sig - LEDC_CH0_SIGNAL) as usize)
        } else {
            0
        }
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        // TIMERx_VALUE reflects the live counter (TRM LEDC_TIMERx_VALUE).
        if (LEDC_TIMER_VALUE_0..LEDC_CH0_CONF0).contains(&offset) {
            let t = ((offset - LEDC_TIMER_VALUE_0) / 8) as usize;
            if t < TIMER_COUNT && (offset - LEDC_TIMER_VALUE_0).is_multiple_of(8) {
                return self.counters[t];
            }
        }
        if offset >= (REG_COUNT * 4) as u32 {
            return 0;
        }
        self.regs[(offset / 4) as usize]
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        // TIMER_CONF.rst resets the phase counter; duty_start rising edge
        // on a channel restarts its timer from zero.
        if offset < LEDC_CH0_CONF0 && offset.is_multiple_of(8) {
            let t = (offset / 8) as usize;
            if t < TIMER_COUNT {
                let old = self.timer_conf(t);
                self.regs[(offset / 4) as usize] = value;
                if value & TIMER_RST != 0 && old & TIMER_RST == 0 {
                    self.counters[t] = 0;
                    self.frac[t] = 0;
                }
                return;
            }
        }
        if offset >= LEDC_CH0_CONF0 && offset.is_multiple_of(0x10) {
            let c = ((offset - LEDC_CH0_CONF0) / 0x10) as usize;
            if c < CHANNEL_COUNT {
                let old = self.channel_conf0(c);
                self.regs[(offset / 4) as usize] = value;
                if value & CH_DUTY_START != 0 && old & CH_DUTY_START == 0 {
                    let t = ((value >> CH_TIMER_SEL_SHIFT) & 0x3) as usize;
                    self.counters[t] = 0;
                    self.frac[t] = 0;
                }
                return;
            }
        }
        if offset.is_multiple_of(4) && offset < (REG_COUNT * 4) as u32 {
            self.regs[(offset / 4) as usize] = value;
        }
    }
}

impl Default for Lcdc {
    fn default() -> Self {
        Self::new()
    }
}
