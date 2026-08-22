//! ESP32-S3 PCNT (Pulse Counter) peripheral model.
//!
//! Register layout per the S3 `pcnt_struct.h` (4 units, 2 channels each, one
//! shared counter per unit): CONF0/CONF1/CONF2 per unit at 0x0C stride, CNT at
//! 0x30 + n*4, INT_RAW/ST/ENA/CLR at 0x40..0x4C, STATUS at 0x50, CTRL at 0x60.
//! PCNT_BASE (esp32s3 memmap) = 0x6008_6000.
//!
//! Edge counting: each channel's signal (PCNT_SIG_CHx_INn_IDX = 33 + u*4 + c)
//! is sampled against its previous level; on an edge the increment/decrement
//! behavior comes from the channel's pos/neg mode, optionally inverted or
//! inhibited by the control signal (PCNT_CTRL_CHx = 33 + u*4 + 2 + c) per the
//! hctrl/lctrl mode. The control level is read at edge time via the GPIO-matrix
//! input routing (soc.rs resolves the signal index to the GPIO pin's level).

/// ESP32-S3 PCNT base address (esp32s3 memmap DR_REG_PCNT_BASE).
pub const PCNT_BASE: u32 = 0x6008_6000;

/// GPIO-matrix input signal indices for PCNT (esp32s3 gpio_sig_map.h):
/// unit u channel c signal = 33 + u*4 + c; unit u channel c control = 33 + u*4 + 2 + c.
const PCNT_SIG_CH0_IN0_IDX: u32 = 33;

/// PCNT interrupt source (ETS_PCNT_INTR_SOURCE = 41 in esp32s3 interrupts.h).
pub const PCNT_INTR_SOURCE: u32 = 41;

const REG_COUNT: usize = 0x100 / 4;

// Per-unit register indices (in 32-bit words).
const CONF0_OFF: [usize; 4] = [0, 0x03, 0x06, 0x09];
const CONF1_OFF: [usize; 4] = [0x01, 0x04, 0x07, 0x0A];
const CONF2_OFF: [usize; 4] = [0x02, 0x05, 0x08, 0x0B];
const STATUS_OFF: [usize; 4] = [0x50 / 4, 0x54 / 4, 0x58 / 4, 0x5C / 4];
const INT_ENA_OFF: usize = 0x48 / 4;
const CTRL_OFF: usize = 0x60 / 4;

pub struct Pcnt {
    regs: [u32; REG_COUNT],
    /// Previous sampled level of each unit/channel signal (edge detection).
    prev_sig: [[u32; 2]; 4],
    /// Per-unit pulse count (shared by the unit's two channels).
    count: [i32; 4],
    int_raw: u32,
    init: bool,
}

impl Pcnt {
    pub fn new() -> Self {
        // TRM PCNT_CTRL default: pulse_cnt_rst_u0..3 = 1 (counters held in
        // reset until the driver clears them). We mirror that so a unit only
        // counts once its reset bit is cleared via the CTRL register.
        let mut regs = [0u32; REG_COUNT];
        regs[CTRL_OFF] = 0x55; // bits 0,2,4,6 = pulse_cnt_rst (default 1)
        Self {
            regs,
            prev_sig: [[0; 2]; 4],
            count: [0; 4],
            int_raw: 0,
            init: false,
        }
    }

    fn sig_ch(unit: usize, ch: usize) -> u32 {
        PCNT_SIG_CH0_IN0_IDX + (unit * 4 + ch) as u32
    }
    fn sig_ctrl(unit: usize, ch: usize) -> u32 {
        PCNT_SIG_CH0_IN0_IDX + (unit * 4 + 2 + ch) as u32
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let off = offset & 0xFF;
        let idx = (off / 4) as usize;
        match off {
            0x30 | 0x34 | 0x38 | 0x3C => {
                let u = (off - 0x30) as usize / 4;
                (self.count[u] as u16) as u32
            }
            0x40 => self.int_raw,
            0x44 => self.int_st(),
            0x50 | 0x54 | 0x58 | 0x5C => self.regs[idx], // STATUS (latched on write)
            _ => self.regs[idx],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let off = offset & 0xFF;
        let idx = (off / 4) as usize;
        match off {
            0x30 | 0x34 | 0x38 | 0x3C => { /* CNT is read-only */ }
            0x40 => { /* INT_RAW is hardware-set only */ }
            0x44 => { /* INT_ST is read-only */ }
            0x48 => self.regs[INT_ENA_OFF] = value,
            0x4C => self.int_raw &= !value,
            0x60 => {
                self.regs[CTRL_OFF] = value;
                // pulse_cnt_rst_uX (bits 0,2,4,6) clears the unit counter.
                for u in 0..4 {
                    if (value >> (u * 2)) & 1 != 0 {
                        self.count[u] = 0;
                    }
                }
            }
            _ => self.regs[idx] = value,
        }
    }

    pub fn int_st(&self) -> u32 {
        self.int_raw & self.regs[INT_ENA_OFF]
    }

    /// Advance the counters by sampling the unit/channel signal levels. `input`
    /// resolves a GPIO-matrix input-signal index to its current logical level.
    pub fn tick<F: Fn(u32) -> u32>(&mut self, input: &F) {
        if !self.init {
            for u in 0..4 {
                for c in 0..2 {
                    self.prev_sig[u][c] = input(Self::sig_ch(u, c));
                }
            }
            self.init = true;
            return;
        }
        let ctrl = self.regs[CTRL_OFF];
        for u in 0..4 {
            // pulse_cnt_rst_uX (bit u*2) holds the counter in reset.
            if (ctrl >> (u * 2)) & 1 != 0 {
                self.count[u] = 0;
                continue;
            }
            // cnt_pause_uX (bit u*2+1) freezes counting.
            if (ctrl >> (u * 2 + 1)) & 1 != 0 {
                continue;
            }
            let conf0 = self.regs[CONF0_OFF[u]];
            let conf1 = self.regs[CONF1_OFF[u]];
            let conf2 = self.regs[CONF2_OFF[u]];
            // Signed 16-bit limits (used only for threshold interrupts; the
            // counter itself free-runs / wraps like real silicon).
            let hlim = (conf2 & 0xFFFF) as i16 as i32;
            let llim = ((conf2 >> 16) & 0xFFFF) as i16 as i32;
            for c in 0..2 {
                let cur = input(Self::sig_ch(u, c)) & 1;
                if cur == self.prev_sig[u][c] {
                    continue;
                }
                let positive = cur == 1;
                // Channel mode fields within CONF0.
                let (pos_mode, neg_mode, hctrl, lctrl) = if c == 0 {
                    ((conf0 >> 18) & 3, (conf0 >> 16) & 3, (conf0 >> 20) & 3, (conf0 >> 22) & 3)
                } else {
                    ((conf0 >> 26) & 3, (conf0 >> 24) & 3, (conf0 >> 28) & 3, (conf0 >> 30) & 3)
                };
                let base = if positive { pos_mode } else { neg_mode };
                let ctrl_cur = input(Self::sig_ctrl(u, c)) & 1;
                let ctrl_mode = if ctrl_cur == 1 { hctrl } else { lctrl };
                let mut action = base;
                match ctrl_mode {
                    1 => {
                        // Invert: increment <-> decrement.
                        action = if base == 1 { 2 } else if base == 2 { 1 } else { base };
                    }
                    2 | 3 => action = 0, // inhibit
                    _ => {}
                }
                match action {
                    1 => self.count[u] = self.count[u].wrapping_add(1),
                    2 => self.count[u] = self.count[u].wrapping_sub(1),
                    _ => {}
                }
                self.prev_sig[u][c] = cur;
            }
            self.update_threshold(u, conf0, conf1, hlim, llim);
        }
    }

    fn update_threshold(&mut self, u: usize, conf0: u32, conf1: u32, hlim: i32, llim: i32) {
        let thres0 = (conf1 & 0xFFFF) as i16 as i32;
        let thres1 = ((conf1 >> 16) & 0xFFFF) as i16 as i32;
        let mut thr = 0u32;
        if (conf0 >> 12) & 1 != 0 && self.count[u] == hlim {
            thr = 1;
        }
        if (conf0 >> 13) & 1 != 0 && self.count[u] == llim {
            thr = 1;
        }
        if (conf0 >> 11) & 1 != 0 && self.count[u] == 0 {
            thr = 1;
        }
        if (conf0 >> 14) & 1 != 0 && self.count[u] == thres0 {
            thr = 1;
        }
        if (conf0 >> 15) & 1 != 0 && self.count[u] == thres1 {
            thr = 1;
        }
        if thr != 0 {
            self.int_raw |= 1 << u;
        } else {
            self.int_raw &= !(1 << u);
        }
        // STATUS: zero-event mode bits (2=negative, 3=positive) + threshold latches.
        let mut st = if self.count[u] < 0 { 2 } else { 3 };
        if thr != 0 {
            st |= 0x7C; // latched threshold flags
        }
        self.regs[STATUS_OFF[u]] = st;
    }
}

impl Default for Pcnt {
    fn default() -> Self {
        Self::new()
    }
}
