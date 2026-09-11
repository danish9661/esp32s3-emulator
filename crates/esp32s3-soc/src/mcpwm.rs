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
//! GPIO, plus capture, software-sync reload, dead-time and the fault/trip
//! submodule (FAULT0..2 inputs force generator outputs via CBC/one-shot
//! actions with enter/exit interrupts). The carrier submodule is simulated
//! (8-slice wave chopping the generator output post-dead-time, with
//! first-pulse one-shot + in/out invert); the update-shadow machinery is
//! latched but applies immediately (matching the reset all-immediate
//! update methods).

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
// Timer/operator event interrupt bits (mcpwm_reg.h INT_ENA: TIMERt_STOP
// 0-2, TIMERt_TEZ 3-5, TIMERt_TEP 6-8, OPn_TEA 15-17, OPn_TEB 18-20).
const TEZ_INT_BASE: u32 = 3;
const TEP_INT_BASE: u32 = 6;
const OP_TEA_INT_BASE: u32 = 15;
const OP_TEB_INT_BASE: u32 = 18;
// Timer sync input (mcpwm_timer_sync_reg_t @ timer stride + 0x0C):
// SYNCI_EN[0] arms the external SYNCt_IN reload (timer t listens to
// SYNCt, group 0 = 160..162, group 1 = 169..171, gpio_sig_map.h
// PWMx_SYNCn_IN_IDX). SYNC_SW[1]/PHASE[19:4] already modeled.
// Dead-time output swap (mcpwm_dt_cfg_reg_t @ DT_BASE0 + op*0x38):
// A_OUTSWAP[9] / B_OUTSWAP[10] (S6/S7) swap the generator outputs
// post-dead-time (applied pre-carrier, same documented ordering class
// as the fault force). INSEL/DEB_MODE need the TRM S1-S8 switch table
// figure and stay unmodeled (reset = symmetric bypass, as modeled).
// Timer sync input arm (mcpwm_timer_sync_reg_t @ timer stride + 0x0C):
// SYNCI_EN[0] arms the external SYNCt_IN reload (timer t listens to
// SYNCt, group 0 = 160..162, group 1 = 169..171, gpio_sig_map.h
// PWMx_SYNCn_IN_IDX). SYNC_SW[1]/PHASE[19:4] already modeled.
const SYNCI_EN: u32 = 1 << 0;
const DT_CFG: u32 = 0x00;
const DT_A_OUTSWAP: u32 = 1 << 9;
const DT_B_OUTSWAP: u32 = 1 << 10;

// Fault submodule (mcpwm_fault_detect_reg_t @0xE4 + per-operator FH regs):
// FAULT_DETECT: F0/1/2_EN[2:0], F0/1/2_POLE[5:3] (1 = high-active),
// EVENT_F0/1/2[8:6] (RO, live). FHk_CFG0 @0x68/0xA0/0xD8 (stride 0x38):
// SW_CBC[0], F2_CBC[1], F1_CBC[2], F0_CBC[3], SW_OST[4], F2_OST[5],
// F1_OST[6], F0_OST[7], A_CBC_D[9:8], A_CBC_U[11:10], A_OST_D[13:12],
// A_OST_U[15:14], B_CBC_D[17:16], B_CBC_U[19:18], B_OST_D[21:20],
// B_OST_U[23:22] (action codes: 0 = keep, 1 = high, 2 = low, 3 = toggle).
// FHk_CFG1 (+0x04): CLR_OST[0] (rising edge clears OST), CBCPULSE[2:1],
// FORCE_CBC[3] + FORCE_OST[4] (any toggle triggers when SW_* enabled).
// FHk_STATUS (+0x08, RO): CBC_ON[0], OST_ON[1]. Fault enter interrupts =
// INT bits 9/10/11, exit ("CLR") = 12/13/14 (mcpwm_ll EVENT_FAULT_*).
// Fault input signals: group 0 FAULT0..2 = 163..165, group 1 = 172..174
// (gpio_sig_map.h PWMx_Fn_IN_IDX).
const FAULT_DETECT: u32 = 0xE4;
const FH_CFG0_BASE: u32 = 0x68;
const FH_STRIDE: u32 = 0x38;
const FH_CFG1_OFF: u32 = 0x04;
const FH_STATUS_OFF: u32 = 0x08;
const FAULT_INT_ENTER_BASE: u32 = 9;
const FAULT_INT_EXIT_BASE: u32 = 12;

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
// Carrier submodule (mcpwm_carrier_cfg_reg_t, per operator at OPER_BASE0 +
// op*0x38 + 0x28 = 0x64/0x9C/0xD4): carrier_en[0], carrier_prescale[4:1]
// (PC_clk period = PWM_clk x (prescale+1)), carrier_duty[7:5] (duty/8 of
// the 8-slice carrier period), carrier_oshtwth[11:8] (first-pulse one-shot
// width in carrier periods), carrier_out_invert[12], carrier_in_invert[13].
const CARRIER_CFG_OFF: u32 = 0x28;
const CARRIER_EN: u32 = 1 << 0;
const CARRIER_PRESCALE_SHIFT: u32 = 1;
const CARRIER_DUTY_SHIFT: u32 = 5;
const CARRIER_OSHTWTH_SHIFT: u32 = 8;
const CARRIER_OUT_INVERT: u32 = 1 << 12;
const CARRIER_IN_INVERT: u32 = 1 << 13;

pub struct Mcpwm {
    regs: [u32; REG_WORDS],
    /// Live timer counter for each timer (TRM timer_status).
    timer_count: [u32; NTIMER],
    /// Previous SYNC0..2 input levels (rising-edge reload).
    prev_sync: [u32; 3],
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
    /// Live fault-event bitmap (EVENT_F0..2, recomputed every fault tick).
    fault_events: u32,
    /// Per-operator CBC (cycle-by-cycle) action ongoing.
    fault_cbc_on: [bool; NOPER],
    /// Per-operator OST (one-shot) action latched.
    fault_ost_on: [bool; NOPER],
    /// Final forced output level per operator/generator (None = no force).
    /// Applied at `signal_level`, after dead-time (the trip override wins).
    fault_force: [[Option<u32>; 2]; NOPER],
    /// Last FHk_CFG1 value per operator (FORCE_CBC/FORCE_OST edge detect).
    fault_cfg1: [u32; NOPER],
    /// Carrier clock phase per operator (emulator steps; the carrier wave
    /// period is 8 x (prescale+1) steps, so the minimum period is 8 steps
    /// and step-granular sampling can never alias it, unlike faster TIE
    /// carriers). Freezes while the timers are stopped (tick is gated).
    car_phase: [u32; NOPER],
    /// Carrier one-shot: remaining forced-HIGH steps of the first pulse
    /// after a rising edge of the (in-inverted) generator input, per
    /// operator/generator. Zero disables (oshtwth = 0).
    car_osht: [[u32; 2]; NOPER],
    /// Previous (in-inverted) generator input per operator/generator, for
    /// the one-shot rising-edge detect.
    car_prev: [[u32; 2]; NOPER],
}

impl Mcpwm {
    pub fn new() -> Self {
        Self {
            regs: [0; REG_WORDS],
            timer_count: [0; NTIMER],
            prev_sync: [0; 3],
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
            fault_events: 0,
            fault_cbc_on: [false; NOPER],
            fault_ost_on: [false; NOPER],
            fault_force: [[None; 2]; NOPER],
            fault_cfg1: [0; NOPER],
            car_phase: [0; NOPER],
            car_osht: [[0; 2]; NOPER],
            car_prev: [[0; 2]; NOPER],
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
    /// Carrier configuration word for operator `op`.
    fn carrier_cfg(&self, op: usize) -> u32 {
        self.gen_reg(op, CARRIER_CFG_OFF)
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
                    // Up / increment: wrap period-1 -> 0. The period
                    // boundary is both TEP (count == period) and TEZ.
                    let n = old + 1;
                    if n >= period {
                        self.int_raw |= 1 << (TEP_INT_BASE + t as u32);
                        (0, true)
                    } else {
                        (n, false)
                    }
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
                            // Peak (count == period): TEP, not TEZ.
                            self.int_raw |= 1 << (TEP_INT_BASE + t as u32);
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
                // TEZ (count == 0): latch TIMERt_TEZ, apply the zero-event
                // action selectors.
                self.int_raw |= 1 << (TEZ_INT_BASE + t as u32);
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
                        self.int_raw |= 1 << (OP_TEA_INT_BASE + op as u32);
                        self.apply_action(op, GEN_UTEA);
                        self.apply_action(op, GEN_DTEA);
                    }
                    if new == cmpb {
                        self.int_raw |= 1 << (OP_TEB_INT_BASE + op as u32);
                        self.apply_action(op, GEN_UTEB);
                        self.apply_action(op, GEN_DTEB);
                    }
                }
            }
        }
        self.tick_dead_time();
        self.tick_carrier();
    }

    /// Advance the carrier submodule by one step: the per-operator phase
    /// always runs, and each generator's (in-inverted) input edge detector
    /// reloads the first-pulse one-shot (`oshtwth` carrier periods).
    fn tick_carrier(&mut self) {
        for op in 0..NOPER {
            let cfg = self.carrier_cfg(op);
            self.car_phase[op] = self.car_phase[op].wrapping_add(1);
            if cfg & CARRIER_EN == 0 {
                continue;
            }
            let pre = ((cfg >> CARRIER_PRESCALE_SHIFT) & 0xF) + 1;
            let period = 8 * pre;
            let oshtwth = (cfg >> CARRIER_OSHTWTH_SHIFT) & 0xF;
            let in_inv = (cfg & CARRIER_IN_INVERT) >> 13;
            for g in 0..2 {
                if self.car_osht[op][g] > 0 {
                    self.car_osht[op][g] -= 1;
                }
                let x = self.gen_level[op][g] ^ in_inv;
                if x == 1 && self.car_prev[op][g] == 0 && oshtwth != 0 {
                    self.car_osht[op][g] = oshtwth * period;
                }
                self.car_prev[op][g] = x;
            }
        }
    }

    /// Carrier-modulated output for operator `op`, generator `g`: the
    /// (in-inverted) generator level gates an 8-slice carrier wave
    /// (duty/8, period 8 x (prescale+1) steps), widened by the one-shot on
    /// the first pulse, then out-inverted. Applied post-dead-time at
    /// `signal_level` (same documented ordering assumption as the fault
    /// force: applying it pre-DT would let a programmed FED/RED swallow
    /// the carrier whole, hiding it from any firmware sampling the pad).
    fn carrier_out(&self, op: usize, g: usize, base: u32) -> u32 {
        let cfg = self.carrier_cfg(op);
        if cfg & CARRIER_EN == 0 {
            return base;
        }
        let pre = ((cfg >> CARRIER_PRESCALE_SHIFT) & 0xF) + 1;
        let duty = (cfg >> CARRIER_DUTY_SHIFT) & 0x7;
        let period = 8 * pre;
        let x = base ^ ((cfg & CARRIER_IN_INVERT) >> 13);
        let w = if x == 0 {
            0
        } else if self.car_osht[op][g] > 0 {
            1
        } else {
            u32::from(self.car_phase[op] % period < duty * pre)
        };
        w ^ ((cfg & CARRIER_OUT_INVERT) >> 12)
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

    fn fh_cfg0(&self, op: usize) -> u32 {
        self.regs[(FH_CFG0_BASE as usize + op * FH_STRIDE as usize) / 4]
    }
    /// Paired with `fh_cfg0` (fault-CFG1 fields are currently unread by the
    /// model, which consumes the combined fault state elsewhere).
    #[allow(dead_code)]
    fn fh_cfg1(&self, op: usize) -> u32 {
        self.regs[(FH_CFG0_BASE as usize + op * FH_STRIDE as usize) / 4 + 1]
    }

    /// True while any fault detector is enabled (gates `tick_fault`).
    /// Fault detection is level-driven and independent of the PWM timers,
    /// so it samples even with every timer stopped.
    pub fn fault_active(&self) -> bool {
        self.regs[FAULT_DETECT as usize / 4] & 0x7 != 0
    }

    /// Apply a fault action (CBC or OST selector pair) to an operator's
    /// forced levels. Direction picks the _D/_U selector pair from the
    /// operator's timer count direction (up-down uses the live direction;
    /// up mode always takes _U, down mode always _D).
    fn apply_fault_action(&mut self, op: usize, d_shift: u32, u_shift: u32) {
        let t = self.op_timer_sel(op);
        let down = self.timer_dir[t] != 0;
        let cfg0 = self.fh_cfg0(op);
        let a = (cfg0 >> if down { d_shift } else { u_shift }) & 3;
        let b = (cfg0 >> if down { d_shift + 8 } else { u_shift + 8 }) & 3;
        if a != 0 {
            let cur = self.fault_force[op][0].unwrap_or(self.gen_level[op][0]);
            self.fault_force[op][0] = Some(Self::do_action(cur, a));
        }
        if b != 0 {
            let cur = self.fault_force[op][1].unwrap_or(self.gen_level[op][1]);
            self.fault_force[op][1] = Some(Self::do_action(cur, b));
        }
    }

    /// Recompute an operator's final force from its latched CBC/OST state
    /// (cleared latches drop their contribution; OST re-applies first so
    /// CBC wins while both hold, matching trigger order). Used on release
    /// paths; trip-enter applies incrementally so toggle chains correctly.
    fn recompute_fault_force(&mut self, op: usize) {
        self.fault_force[op] = [None, None];
        if self.fault_ost_on[op] {
            self.apply_fault_action(op, 12, 14);
        }
        if self.fault_cbc_on[op] {
            self.apply_fault_action(op, 8, 10);
        }
    }

    /// True when any timer arms the external sync input (tick gate).
    pub fn sync_armed(&self) -> bool {
        (0..NTIMER).any(|t| self.regs[(t * TIMER_STRIDE) + TIMER_SYNC as usize / 4] & SYNCI_EN != 0)
    }

    /// Advance the timer-sync submodule by one SoC step: sample the SYNC0..2
    /// matrix inputs (`sync_base` + t, group 0 = 160, group 1 = 169,
    /// gpio_sig_map.h PWMx_SYNCn_IN_IDX); a rising edge with SYNCI_EN
    /// reloads timer t with PHASE (same load as SYNC_SW). Sampled even with
    /// the timers stopped, like the fault inputs.
    pub fn tick_sync<F: Fn(u32) -> u32>(&mut self, sync_base: u32, input: &F) {
        for t in 0..NTIMER {
            let lvl = input(sync_base + t as u32) & 1;
            let rising = lvl == 1 && self.prev_sync[t] == 0;
            self.prev_sync[t] = lvl;
            if rising && self.regs[(t * TIMER_STRIDE) + TIMER_SYNC as usize / 4] & SYNCI_EN != 0 {
                let sync = self.regs[(t * TIMER_STRIDE) + TIMER_SYNC as usize / 4];
                self.timer_count[t] = (sync >> PHASE_SHIFT) & 0xFFFF;
            }
        }
    }

    /// Advance the fault submodule by one SoC step: sample the FAULT0..2
    /// matrix inputs (`fault_base` + k, group 0 = 163, group 1 = 172),
    /// latch enter/exit interrupts, and drive CBC/OST trip actions.
    /// CBC forces while its event is ongoing (CBCPULSE refresh-moment
    /// selection is not modeled: the force applies immediately on trigger,
    /// matching the reset CBCPULSE = immediate behavior); OST latches
    /// until a CLR_OST rising edge. Called regardless of the PWM timers.
    pub fn tick_fault<F: Fn(u32) -> u32>(&mut self, fault_base: u32, input: &F) {
        let det = self.regs[FAULT_DETECT as usize / 4];
        let mut events = 0u32;
        for k in 0..3 {
            if det & (1 << k) == 0 {
                continue;
            }
            let pole = (det >> (3 + k)) & 1;
            if input(fault_base + k) == pole {
                events |= 1 << k;
            }
        }
        let entered = events & !self.fault_events;
        let exited = self.fault_events & !events;
        for k in 0..3 {
            if entered & (1 << k) != 0 {
                self.int_raw |= 1 << (FAULT_INT_ENTER_BASE + k);
            }
            if exited & (1 << k) != 0 {
                self.int_raw |= 1 << (FAULT_INT_EXIT_BASE + k);
            }
        }
        self.fault_events = events;
        for op in 0..NOPER {
            let cfg0 = self.fh_cfg0(op);
            // CBC/OST source bitmaps aligned to the event bits: bit k =
            // fault k (CFG0 stores F2/F1/F0 at [1]/[2]/[3] and [5]/[6]/[7]).
            let cbc_src = ((cfg0 >> 3) & 1) | ((cfg0 >> 1) & 2) | ((cfg0 << 1) & 4);
            let ost_src = ((cfg0 >> 7) & 1) | ((cfg0 >> 5) & 2) | ((cfg0 >> 3) & 4);
            if entered & cbc_src != 0 {
                self.fault_cbc_on[op] = true;
                // CBC A pair at shifts 8 (D) / 10 (U), B pair +8.
                self.apply_fault_action(op, 8, 10);
            }
            if entered & ost_src != 0 {
                self.fault_ost_on[op] = true;
                // OST A pair at shifts 12 (D) / 14 (U), B pair +8.
                self.apply_fault_action(op, 12, 14);
            }
            if exited != 0 && self.fault_cbc_on[op] {
                // Cycle-by-cycle ends with its event (no event left that
                // this operator listens to keeps it alive: re-check).
                if cbc_src & events == 0 {
                    self.fault_cbc_on[op] = false;
                    self.recompute_fault_force(op);
                }
            }
        }
    }

    /// Output level of a GPIO-matrix signal (PWM0 OUT0A..OUT2B = 160..165).
    /// A latched fault trip overrides everything (post-dead-time force).
    pub fn signal_level(&self, sig: u32) -> u32 {
        if (PWM0_OUT0A_IDX..=PWM0_OUT2B_IDX).contains(&sig) {
            let s = (sig - PWM0_OUT0A_IDX) as usize;
            let op = s / 2;
            let g = s % 2;
            if let Some(force) = self.fault_force[op][g] {
                return force;
            }
            // Unprogrammed dead-time (FED=RED=0) reads the generator level
            // directly (combinatorial passthrough, exactly the old path);
            // programmed delays come from the ticked inertial state.
            let fed = self.regs[(DT_BASE0 + op as u32 * DT_STRIDE + DT_FED) as usize / 4] & 0xFFFF;
            let red = self.regs[(DT_BASE0 + op as u32 * DT_STRIDE + DT_RED) as usize / 4] & 0xFFFF;
            let base = if fed == 0 && red == 0 {
                self.gen_level[op][g]
            } else {
                self.dt_out[op][g]
            };
            // S6/S7 output swap (post-dead-time, pre-carrier ordering).
            let cfg = self.regs[(DT_BASE0 + op as u32 * DT_STRIDE + DT_CFG) as usize / 4];
            let swapped = if cfg & DT_A_OUTSWAP != 0 && cfg & DT_B_OUTSWAP != 0 {
                1 - g
            } else if cfg & DT_A_OUTSWAP != 0 && g == 0 {
                1
            } else if cfg & DT_B_OUTSWAP != 0 && g == 1 {
                0
            } else {
                g
            };
            let base = if swapped == g {
                base
            } else if fed == 0 && red == 0 {
                self.gen_level[op][swapped]
            } else {
                self.dt_out[op][swapped]
            };
            self.carrier_out(op, g, base)
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
            FAULT_DETECT => {
                // EN/POLE stored; EVENT_F0..2 are the live bitmap.
                (self.regs[FAULT_DETECT as usize / 4] & 0x3F) | (self.fault_events << 6)
            }
            _ => {
                // FHk_STATUS: CBC_ON/OST_ON live latch bits (RO).
                for op in 0..NOPER {
                    if offset == FH_CFG0_BASE + op as u32 * FH_STRIDE + FH_STATUS_OFF {
                        return (self.fault_cbc_on[op] as u32)
                            | ((self.fault_ost_on[op] as u32) << 1);
                    }
                }
                if idx < REG_WORDS { self.regs[idx] } else { 0 }
            }
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let idx = (offset / 4) as usize;
        match offset {
            // timer_status, int_raw and FHk_STATUS are read-only.
            TIMER_STATUS | 0x20 | 0x30 | INT_RAW => {}
            INT_CLR => {
                self.int_raw &= !value;
            }
            INT_ENA => {
                self.regs[INT_ENA as usize / 4] = value;
            }
            INT_ST => {}
            FAULT_DETECT => {
                // EVENT_F0..2 are RO (live); only EN/POLE are stored.
                self.regs[FAULT_DETECT as usize / 4] = value & 0x3F;
            }
            _ => {
                // FHk_STATUS is RO (live latch bits).
                for op in 0..NOPER {
                    if offset == FH_CFG0_BASE + op as u32 * FH_STRIDE + FH_STATUS_OFF {
                        return;
                    }
                }
                // FHk_CFG1: CLR_OST rising edge clears the OST latch;
                // FORCE_CBC/FORCE_OST toggles trigger software trip actions
                // (gated on SW_CBC/SW_OST).
                for op in 0..NOPER {
                    if offset == FH_CFG0_BASE + op as u32 * FH_STRIDE + FH_CFG1_OFF {
                        let prev = self.fault_cfg1[op];
                        self.regs[idx] = value;
                        self.fault_cfg1[op] = value;
                        if value & 0x1 != 0 && prev & 0x1 == 0 && self.fault_ost_on[op] {
                            self.fault_ost_on[op] = false;
                            self.recompute_fault_force(op);
                        }
                        let cfg0 = self.fh_cfg0(op);
                        if (value ^ prev) & (1 << 3) != 0 && cfg0 & 0x1 != 0 {
                            self.fault_cbc_on[op] = true;
                            self.apply_fault_action(op, 8, 10);
                        }
                        if (value ^ prev) & (1 << 4) != 0 && cfg0 & (1 << 4) != 0 {
                            self.fault_ost_on[op] = true;
                            self.apply_fault_action(op, 12, 14);
                        }
                        return;
                    }
                }
                if idx < REG_WORDS {
                    self.regs[idx] = value;
                    // Timer sync: a SYNC_SW write reloads the counter with
                    // PHASE (external SYNCI_EN reloads via tick_sync; the
                    // level-triggered write matches what the driver emits).
                    if offset >= TIMER_SYNC
                        && offset < TIMER_SYNC + NTIMER as u32 * TIMER_STRIDE as u32
                        && (offset - TIMER_SYNC).is_multiple_of(TIMER_STRIDE as u32)
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
