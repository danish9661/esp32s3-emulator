//! ESP32-S3 MCPWM (motor-control PWM) model — group 0 only.
//!
//! Register block base `0x6001E000` (`DR_REG_PWM0_BASE`, esp-idf
//! `reg_base.h`; `0x6000B000` is HINF, NOT MCPWM — see the note in
//! AGENTS.md). Layout per `soc/mcpwm_struct.h` (v5.3, GPLv2): a flat
//! `mcpwm_dev_t` with `timer[3]` (16 B each) and `operators[3]` (56 B each).
//!
//! Functional model: each timer runs a 16-bit time base (count, wrap at
//! `period`) whose rate is divided by `timer_prescale + 1`; the count drives
//! one or more operators (selected via `operator_timersel`). Each operator has
//! two comparators (A/B) and two generators (A/B). On the timer events TEZ
//! (count == 0), TEA (count == cmprA), TEB (count == cmprB) the generator's
//! action table (`generator0`/`generator1`, 2-bit selectors per event:
//! 0 = no change, 1 = force high, 2 = force low, 3 = toggle) updates the
//! output level. The level is exposed to the GPIO matrix via the PWM0A_OUT..
//! PWM2B_OUT signals (160..165, `gpio_sig_map.h`).
//!
//! This models the core PWM path (up / down / up-down counting + action-based
//! generators) enough for a real firmware to produce a correct duty cycle on a
//! GPIO, plus capture and software-sync reload. Dead-time, carrier and the
//! update-shadow machinery are latched but not simulated (S3 silicon has no
//! Trip-Zone/fault submodule, so there is nothing to model there).

/// MCPWM group-0 register block base (esp-idf `DR_REG_PWM0_BASE`).
pub const MCPWM_BASE: u32 = 0x6001_E000;
/// MCPWM group-1 register block base (esp-idf `DR_REG_PWM1_BASE`): an
/// independent copy of the group-0 block (3 timers + 3 operators).
pub const MCPWM1_BASE: u32 = 0x6002_C000;
/// Peripheral interrupt source (#31 = `ETS_PWM0_INTR_SOURCE`, `interrupts.h`).
pub const MCPWM_INTR_SOURCE: u32 = 31;
/// Group-1 interrupt source (`ETS_PWM1_INTR_SOURCE` = 32).
pub const MCPWM1_INTR_SOURCE: u32 = 32;

// Timer array: 3 timers, 16 B (0x10) stride.
const NTIMER: usize = 3;
const TIMER_STRIDE: usize = 0x10;
// Operator array: 3 operators, 56 B (0x38) stride.
const NOPER: usize = 3;
const OPER_STRIDE: usize = 0x38;

// Per-timer register byte offsets within the block (timer[i] at i*0x10).
const TIMER_CFG0: u32 = 0x04;
const TIMER_CFG1: u32 = 0x08;
const TIMER_STATUS: u32 = 0x10;
// Timer sync (mcpwm_timer_sync_reg_t): SYNCI_EN[0], SYNC_SW[1] (toggle to
// trigger), PHASE[19:4] reload value, at timer stride + 0x0C.
const TIMER_SYNC: u32 = 0x0C;
const SYNC_SW: u32 = 1 << 1;
const PHASE_SHIFT: u32 = 4;
// Operator block base (operator[k] at OPER_BASE0 + k*0x38) and the
// per-operator field offsets LOCAL to that block (mcpwm_operator_reg_t).
const OPER_BASE0: u32 = 0x3C;
#[allow(dead_code)]
const GEN_STMP_CFG: u32 = 0x00;
const GEN_TSTMP_A: u32 = 0x04;
const GEN_TSTMP_B: u32 = 0x08;
#[allow(dead_code)]
const GEN_CFG0: u32 = 0x0C;
const GEN_FORCE: u32 = 0x10;
const GENERATOR0: u32 = 0x14;
const GENERATOR1: u32 = 0x18;

// Top-level registers.
const OPER_TIMERSEL: u32 = 0x38;
const INT_ENA: u32 = 0x110;
const INT_RAW: u32 = 0x114;
const INT_ST: u32 = 0x118;
const INT_CLR: u32 = 0x11C;

// Capture submodule (mcpwm_cap_*_reg_t): timer cfg @0xE8, phase @0xEC,
// channel cfg @0xF0+4n, channel value @0xFC+4n, edge status @0x108.
const CAP_TIMER_CFG: u32 = 0xE8;
const CAP_CHN_CFG_BASE: u32 = 0xF0;
const CAP_CHN_BASE: u32 = 0xFC;
const CAP_STATUS: u32 = 0x108;
// CAP_TIMER_CFG bits / cap_chn_cfg fields.
const CAP_TIMER_EN: u32 = 1 << 0;
const CAPN_EN: u32 = 1 << 0;
const CAPN_MODE_SHIFT: u32 = 1; // bit0(1) = negedge, bit1(2) = posedge
const CAPN_PRESCALE_SHIFT: u32 = 3; // [10:3], divide by prescale+1
const CAPN_INVERT: u32 = 1 << 11;
// Capture-channel interrupt bits (INT_ENA/RAW/ST/CLR).
const CAP_INT_BASE: u32 = 27;

const REG_WORDS: usize = 0x128 / 4;

// timer_cfg0 field positions (mcpwm_timer_cfg0_reg_t).
const TIMER_PRESCALE_SHIFT: u32 = 0; // [7:0]
const TIMER_PERIOD_SHIFT: u32 = 8; // [23:8]
// timer_cfg1 field positions (mcpwm_timer_cfg1_reg_t).
const TIMER_START_SHIFT: u32 = 0; // [2:0]
const TIMER_MOD_SHIFT: u32 = 3; // [4:3] 0=freeze,1=inc,2=dec,3=up-down

// generator action-selector bit positions (mcpwm_gen_reg_t), same for A/B.
const GEN_UTEZ: u32 = 0; // [1:0]
const GEN_UTEA: u32 = 4; // [5:4]
const GEN_UTEB: u32 = 6; // [7:6]
const GEN_DTEZ: u32 = 12; // [13:12]
const GEN_DTEA: u32 = 16; // [17:16]
const GEN_DTEB: u32 = 18; // [19:18]

// Action codes (2 bits).
const ACT_HIGH: u32 = 1;
const ACT_LOW: u32 = 2;
const ACT_TOGGLE: u32 = 3;

// GPIO-matrix output signal indices for MCPWM0 (gpio_sig_map.h: PWM0_OUTy*_IDX).
const PWM0_OUT0A_IDX: u32 = 160;
const PWM0_OUT2B_IDX: u32 = 165;
// Dead-time submodule (mcpwm_dt_reg_t): DT[k] base stride 0x38 with FED
// (falling-edge delay) @ +0x04 and RED (rising-edge delay) @ +0x08,
// both [15:0] in (emulator) tick units. INSEL routing is not modeled;
// delays apply to both generator outputs symmetrically.
const DT_BASE0: u32 = 0x58;
const DT_STRIDE: u32 = 0x38;
const DT_FED: u32 = 0x04;
const DT_RED: u32 = 0x08;

pub struct Mcpwm {
    regs: [u32; REG_WORDS],
    /// Live timer counter for each timer (TRM timer_status).
    timer_count: [u32; NTIMER],
    /// Per-timer prescale accumulator (advance the counter every prescale+1 ticks).
    timer_prescale_cnt: [u32; NTIMER],
    /// Up-down direction: 0 = counting up, 1 = counting down.
    timer_dir: [u8; NTIMER],
    /// Current generator output level: [operator][0=A, 1=B].
    gen_level: [[u32; 2]; NOPER],
    /// Dead-time delayed output level (what the pads actually drive).
    dt_out: [[u32; 2]; NOPER],
    /// Pending dead-time edge: tick it was armed + target level + armed.
    dt_tick: [[u64; 2]; NOPER],
    dt_level: [[u32; 2]; NOPER],
    dt_pend: [[bool; 2]; NOPER],
    /// Free-running emulator-tick counter (dead-time delay time base).
    now: u64,
    /// Latched interrupt raw bits.
    int_raw: u32,
    /// Capture-timer free-running counter (APB ticks while enabled).
    cap_timer: u32,
    /// Per-channel prescale edge counters.
    cap_edge_cnt: [u32; 3],
    /// Previous sampled input level per capture channel.
    prev_cap: [u32; 3],
    /// Per-channel first-sample seeding (a channel latches no edge on the
    /// tick its sampling starts, like the PCNT first-sample gate).
    cap_init: [bool; 3],
}

impl Mcpwm {
    pub fn new() -> Self {
        Self {
            regs: [0; REG_WORDS],
            timer_count: [0; NTIMER],
            timer_prescale_cnt: [0; NTIMER],
            timer_dir: [0; NTIMER],
            gen_level: [[0; 2]; NOPER],
            dt_out: [[0; 2]; NOPER],
            dt_tick: [[0; 2]; NOPER],
            dt_level: [[0; 2]; NOPER],
            dt_pend: [[false; 2]; NOPER],
            now: 0,
            int_raw: 0,
            cap_timer: 0,
            cap_edge_cnt: [0; 3],
            prev_cap: [0; 3],
            cap_init: [false; 3],
        }
    }

    fn timer_cfg0(&self, t: usize) -> u32 {
        self.regs[(TIMER_CFG0 as usize + t * TIMER_STRIDE) / 4]
    }
    fn timer_cfg1(&self, t: usize) -> u32 {
        self.regs[(TIMER_CFG1 as usize + t * TIMER_STRIDE) / 4]
    }
    fn gen_reg(&self, op: usize, off: u32) -> u32 {
        self.regs[(OPER_BASE0 as usize + op * OPER_STRIDE + off as usize) / 4]
    }
    fn op_timer_sel(&self, op: usize) -> usize {
        ((self.regs[OPER_TIMERSEL as usize / 4] >> (2 * op)) & 0x3) as usize
    }

    /// Apply a generator action to both A/B outputs of an operator.
    fn apply_action(&mut self, op: usize, shift: u32) {
        let g0 = self.gen_reg(op, GENERATOR0);
        let g1 = self.gen_reg(op, GENERATOR1);
        let a = (g0 >> shift) & 3;
        let b = (g1 >> shift) & 3;
        self.gen_level[op][0] = Self::do_action(self.gen_level[op][0], a);
        self.gen_level[op][1] = Self::do_action(self.gen_level[op][1], b);
    }

    fn do_action(cur: u32, act: u32) -> u32 {
        match act {
            ACT_HIGH => 1,
            ACT_LOW => 0,
            ACT_TOGGLE => cur ^ 1,
            _ => cur,
        }
    }

    /// Advance the PWM state machines by one SoC step (called from `tick_timers`).
    /// True when any timer is running (`start >= 2`). The SoC skips `tick()`
    /// otherwise — `tick` would `continue` for every stopped timer
    /// identically, so gating is behavior-preserving.
    pub fn is_active(&self) -> bool {
        for t in 0..NTIMER {
            if (self.timer_cfg1(t) >> TIMER_START_SHIFT) & 0x7 >= 2 {
                return true;
            }
        }
        false
    }

    pub fn tick(&mut self) {
        self.now = self.now.wrapping_add(1);
        for t in 0..NTIMER {
            let cfg1 = self.timer_cfg1(t);
            let start = (cfg1 >> TIMER_START_SHIFT) & 0x7;
            let mode = (cfg1 >> TIMER_MOD_SHIFT) & 0x3;
            // start == 2 "run on" (and 3/4 run until a stop condition).
            if start < 2 {
                continue;
            }
            let cfg0 = self.timer_cfg0(t);
            let prescale = (cfg0 >> TIMER_PRESCALE_SHIFT) & 0xFF;
            let mut period = (cfg0 >> TIMER_PERIOD_SHIFT) & 0xFFFF;
            if period == 0 {
                period = 1;
            }
            self.timer_prescale_cnt[t] += 1;
            if self.timer_prescale_cnt[t] < prescale + 1 {
                continue;
            }
            self.timer_prescale_cnt[t] = 0;

            let old = self.timer_count[t];
            let (new, tez) = match mode {
                1 => {
                    // Up / increment: wrap period-1 -> 0.
                    let n = old + 1;
                    if n >= period { (0, true) } else { (n, false) }
                }
                2 => {
                    // Down / decrement.
                    if old == 0 {
                        (period - 1, false)
                    } else {
                        (old - 1, false)
                    }
                }
                3 => {
                    // Up-down (triangle): up to period-1 then down to 0.
                    if self.timer_dir[t] == 0 {
                        if old + 1 >= period {
                            self.timer_dir[t] = 1;
                            (period - 1, false)
                        } else {
                            (old + 1, false)
                        }
                    } else if old == 0 {
                        self.timer_dir[t] = 0;
                        (1, false)
                    } else {
                        (old - 1, false)
                    }
                }
                _ => (old, false),
            };
            self.timer_count[t] = new;
            if tez {
                // TEZ (count == 0): apply the zero-event action selectors.
                for op in 0..NOPER {
                    if self.op_timer_sel(op) == t {
                        self.apply_action(op, GEN_UTEZ);
                        self.apply_action(op, GEN_DTEZ);
                    }
                }
            } else {
                // Comparator matches (once per period at the matching count).
                for op in 0..NOPER {
                    if self.op_timer_sel(op) != t {
                        continue;
                    }
                    let cmpa = self.gen_reg(op, GEN_TSTMP_A) & 0xFFFF;
                    let cmpb = self.gen_reg(op, GEN_TSTMP_B) & 0xFFFF;
                    if new == cmpa {
                        self.apply_action(op, GEN_UTEA);
                        self.apply_action(op, GEN_DTEA);
                    }
                    if new == cmpb {
                        self.apply_action(op, GEN_UTEB);
                        self.apply_action(op, GEN_DTEB);
                    }
                }
            }
        }
        self.tick_dead_time();
    }

    /// Inertial dead-time edge delay (FED on falling, RED on rising edges,
    /// in emulator ticks). Zero delays pass through untouched so existing
    /// PWM behavior is bit-identical when DT is unprogrammed.
    fn tick_dead_time(&mut self) {
        for op in 0..NOPER {
            let fed = self.regs[(DT_BASE0 + op as u32 * DT_STRIDE + DT_FED) as usize / 4] & 0xFFFF;
            let red = self.regs[(DT_BASE0 + op as u32 * DT_STRIDE + DT_RED) as usize / 4] & 0xFFFF;
            for g in 0..2 {
                let raw = self.gen_level[op][g];
                if fed == 0 && red == 0 {
                    self.dt_out[op][g] = raw;
                    self.dt_pend[op][g] = false;
                    continue;
                }
                if raw == self.dt_out[op][g] {
                    // Back at the driven level: cancel any pending edge.
                    self.dt_pend[op][g] = false;
                } else if !self.dt_pend[op][g] || self.dt_level[op][g] != raw {
                    // New (or changed) edge: arm with latest-wins.
                    self.dt_pend[op][g] = true;
                    self.dt_tick[op][g] = self.now;
                    self.dt_level[op][g] = raw;
                }
                if self.dt_pend[op][g] {
                    let wait = if self.dt_level[op][g] == 1 { red } else { fed };
                    if self.now.wrapping_sub(self.dt_tick[op][g]) >= wait as u64 {
                        self.dt_out[op][g] = self.dt_level[op][g];
                        self.dt_pend[op][g] = false;
                    }
                }
            }
        }
    }

    /// True while the capture timer runs (gates `tick_capture`).
    pub fn cap_timer_enabled(&self) -> bool {
        self.regs[CAP_TIMER_CFG as usize / 4] & CAP_TIMER_EN != 0
    }

    /// Sampled input level of capture channel `n` (post-invert).
    fn cap_level<F: Fn(u32) -> u32>(&self, cap_base: u32, input: &F, n: usize) -> u32 {
        let cfg = self.regs[(CAP_CHN_CFG_BASE as usize + 4 * n) / 4];
        input(cap_base + n as u32) ^ ((cfg & CAPN_INVERT) >> 11)
    }

    /// Advance the capture submodule by one SoC step: the free-running timer
    /// plus per-channel edge capture through the GPIO-matrix input routing
    /// (`input` resolves a capture signal index to its level, PCNT-style).
    /// On a configured edge (divided by prescale+1) the timer latches into
    /// CAP_CHN, the edge records in CAP_STATUS, and the CAPn interrupt
    /// latches. Called regardless of the PWM timers (capture is independent).
    pub fn tick_capture<F: Fn(u32) -> u32>(&mut self, cap_base: u32, input: &F) {
        if !self.cap_timer_enabled() {
            return;
        }
        self.cap_timer = self.cap_timer.wrapping_add(1);
        for n in 0..3 {
            let cfg = self.regs[(CAP_CHN_CFG_BASE as usize + 4 * n) / 4];
            if cfg & CAPN_EN == 0 {
                continue;
            }
            if !self.cap_init[n] {
                self.prev_cap[n] = self.cap_level(cap_base, input, n);
                self.cap_init[n] = true;
                continue;
            }
            let cur = self.cap_level(cap_base, input, n);
            let prev = self.prev_cap[n];
            self.prev_cap[n] = cur;
            let pos = cur == 1 && prev == 0;
            let neg = cur == 0 && prev == 1;
            let mode = (cfg >> CAPN_MODE_SHIFT) & 3;
            if !(mode & 2 != 0 && pos) && !(mode & 1 != 0 && neg) {
                continue;
            }
            let pre = ((cfg >> CAPN_PRESCALE_SHIFT) & 0xFF) + 1;
            self.cap_edge_cnt[n] += 1;
            if self.cap_edge_cnt[n] < pre {
                continue;
            }
            self.cap_edge_cnt[n] = 0;
            self.regs[(CAP_CHN_BASE as usize + 4 * n) / 4] = self.cap_timer;
            let st = &mut self.regs[CAP_STATUS as usize / 4];
            if pos {
                *st &= !(1 << n);
            } else {
                *st |= 1 << n;
            }
            self.int_raw |= 1 << (CAP_INT_BASE + n as u32);
        }
    }

    /// Output level of a GPIO-matrix signal (PWM0 OUT0A..OUT2B = 160..165).
    pub fn signal_level(&self, sig: u32) -> u32 {
        if (PWM0_OUT0A_IDX..=PWM0_OUT2B_IDX).contains(&sig) {
            let s = (sig - PWM0_OUT0A_IDX) as usize;
            let op = s / 2;
            // Unprogrammed dead-time (FED=RED=0) reads the generator level
            // directly (combinatorial passthrough, exactly the old path);
            // programmed delays come from the ticked inertial state.
            let fed = self.regs[(DT_BASE0 + op as u32 * DT_STRIDE + DT_FED) as usize / 4] & 0xFFFF;
            let red = self.regs[(DT_BASE0 + op as u32 * DT_STRIDE + DT_RED) as usize / 4] & 0xFFFF;
            if fed == 0 && red == 0 {
                self.gen_level[op][s % 2]
            } else {
                self.dt_out[op][s % 2]
            }
        } else {
            0
        }
    }

    /// Interrupt pending = raw & enabled (capture channels latch; timer
    /// event interrupts are not modeled).
    pub fn int_pending(&self) -> bool {
        (self.int_raw & self.regs[INT_ENA as usize / 4]) != 0
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let idx = (offset / 4) as usize;
        match offset {
            TIMER_STATUS | 0x20 | 0x30 => {
                // timer[i].timer_status: live counter.
                let t = ((offset - TIMER_STATUS) / TIMER_STRIDE as u32) as usize;
                self.timer_count[t] & 0xFFFF
            }
            INT_RAW => self.int_raw,
            INT_ST => self.int_raw & self.regs[INT_ENA as usize / 4],
            _ => {
                if idx < REG_WORDS {
                    self.regs[idx]
                } else {
                    0
                }
            }
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let idx = (offset / 4) as usize;
        match offset {
            // timer_status and int_raw are read-only.
            TIMER_STATUS | 0x20 | 0x30 | INT_RAW => {}
            INT_CLR => {
                self.int_raw &= !value;
            }
            INT_ENA => {
                self.regs[INT_ENA as usize / 4] = value;
            }
            INT_ST => {}
            _ => {
                if idx < REG_WORDS {
                    self.regs[idx] = value;
                    // Timer sync: a SYNC_SW write reloads the counter with
                    // PHASE (SYNCI_EN external input is not modeled; the
                    // level-triggered write matches what the driver emits).
                    if offset >= TIMER_SYNC
                        && offset < TIMER_SYNC + NTIMER as u32 * TIMER_STRIDE as u32
                        && (offset - TIMER_SYNC) % TIMER_STRIDE as u32 == 0
                        && value & SYNC_SW != 0
                    {
                        let t = ((offset - TIMER_SYNC) / TIMER_STRIDE as u32) as usize;
                        self.timer_count[t] = (value >> PHASE_SHIFT) & 0xFFFF;
                    }
                    // gen_force: direct force of generator A/B output level.
                    if offset >= OPER_BASE0
                        && offset < OPER_BASE0 + NOPER as u32 * OPER_STRIDE as u32
                    {
                        let local = offset - OPER_BASE0;
                        if local % OPER_STRIDE as u32 == GEN_FORCE {
                            let op = (local / OPER_STRIDE as u32) as usize;
                            let fa = value & 0x3;
                            let fb = (value >> 2) & 0x3;
                            if fa == 1 {
                                self.gen_level[op][0] = 1;
                            } else if fa == 2 {
                                self.gen_level[op][0] = 0;
                            }
                            if fb == 1 {
                                self.gen_level[op][1] = 1;
                            } else if fb == 2 {
                                self.gen_level[op][1] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
}

impl Default for Mcpwm {
    fn default() -> Self {
        Self::new()
    }
}
