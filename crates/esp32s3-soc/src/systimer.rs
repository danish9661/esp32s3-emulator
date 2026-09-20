//! ESP32-S3 SYSTIMER peripheral model (base 0x60023000).
//!
//! Register layout per ESP-IDF `systimer_struct.h` (esp32s3): CONF (0x000),
//! UNIT0/1_OP (0x004/0x008), UNITn_LOAD_HI/LO (0x00C..0x018), TARGETn_HI/LO
//! (0x01C..0x030), TARGET_CONFn (0x034..0x03C), UNITn_VALUE_HI/LO
//! (0x040..0x04C, the counter snapshot latches), COMP_LOADn (0x050..0x058),
//! UNITn_LOAD apply (0x05C/0x060), INT_ENA/RAW/CLR/ST (0x064..0x070),
//! REAL_TARGETn (0x074..0x07C), DATE (0x0FC).
//!
//! Register order is VALUE_HI then VALUE_LO: HI at 0x040+8n, LO at 0x044+8n
//! (id0: 0x40/0x44, id1: 0x48/0x4C).  Confirmed by esp32s3-hal ("unit0 value
//! low register @ 0x44") and the firmware disasm of
//! `systimer_hal_get_counter_value` (`l32i a2,a9,68` = LO, `l32i.n a3,a8,0`
//! = HI).  The HI word holds 20 significant bits (bits [51:32] of the 52-bit
//! counter) — the driver ends with `slli/srli 12` on the HI word.
//!
//! Snapshot handshake (TRM SYSTIMER_UNITn_OP / esp_timer_impl:
//! `systimer_hal_get_counter_value`): writing bit 30 (timer_unit_update)
//! copies the live 52-bit counter into the VALUE latches and raises bit 29
//! (timer_unit_value_valid); the driver spins on bit 29 before reading the
//! latches.
//!
//! Alarm path (esp_timer): TARGETn_HI/LO + TARGET_CONFn.target_timer_unit_sel
//! select the unit, COMP_LOADn arms the compare, CONF.targetn_work_en + a
//! matching counter set INT_RAW.targetn (level until INT_CLR).  Interrupt
//! sources: ETS_SYSTIMER_TARGET0/1/2 = 57/58/59 (esp32s3 interrupts.h; 56 is
//! ETS_CACHE_IA_INTR_SOURCE).
//!
//! Period mode (TARGET_CONFn.target_period_mode = 1): the FreeRTOS tick
//! (esp-idf systimer_ll_enable_alarm_period / vSystimerSetup) arms NO target
//! value and NO COMP_LOAD — the alarm is active on CONF.targetn_work_en +
//! INT_ENA and the effective target advances by target_period after every
//! fire (the alarm re-arms itself; the tick ISR only clears INT_CLR).  The
//! S3 uses TARGET0 for the PRO tick and TARGET1 for the APP tick, both on
//! unit 1; the esp_timer uses TARGET2 (oneshot) on unit 0.
//!
//! Time model: `tick(cycles)` increments each work-enabled unit counter by
//! 1 per cycle (no prescaler — the 1 MHz tick is external on silicon; the
//! frontend drives cycles, one per CPU step, like TIMG).

const REG_COUNT: usize = 0x100 / 4;

// Register offsets (systimer_reg.h).
const CONF: u32 = 0x000;
const OP0: u32 = 0x004;
const OP1: u32 = 0x008;
const LOAD_HI0: u32 = 0x00C;
const LOAD_LO0: u32 = 0x010;
const LOAD_HI1: u32 = 0x014;
const LOAD_LO1: u32 = 0x018;
const TARGET_HI: u32 = 0x01C; // + n * 8
const TARGET_LO: u32 = 0x020; // + n * 8
const TARGET_CONF: u32 = 0x034; // + n * 4
const VALUE_HI0: u32 = 0x040;
const VALUE_LO0: u32 = 0x044;
const VALUE_HI1: u32 = 0x048;
const VALUE_LO1: u32 = 0x04C;
const COMP_LOAD: u32 = 0x050; // + n * 4
const LOAD_APPLY0: u32 = 0x05C;
const LOAD_APPLY1: u32 = 0x060;
const INT_ENA: u32 = 0x064;
const INT_RAW: u32 = 0x068;
const INT_CLR: u32 = 0x06C;
const INT_ST: u32 = 0x070;
const REAL_TARGET: u32 = 0x074; // + n * 4
const DATE: u32 = 0x0FC;

// Field bits (systimer_unit_op_reg_t).
const OP_UPDATE: u32 = 1 << 30;
const OP_VALUE_VALID: u32 = 1 << 29;

// CONF bits (systimer_conf_reg_t): targetn_work_en [24:22],
// unit0_work_en = 30 (default 1), unit1_work_en = 29 (default 0).
const TARGET_WORK_EN: [u32; 3] = [1 << 24, 1 << 23, 1 << 22];
const UNIT0_WORK_EN: u32 = 1 << 30;
const UNIT1_WORK_EN: u32 = 1 << 29;

/// SYSTIMER reset value (systimer_date_reg_t: 33628753 = 0x02010F31).
const DATE_VALUE: u32 = 0x0201_0F31;

/// ESP-IDF hal/systimer_types.h: the esp_timer clock runs at 1 MHz on the
/// S3; the counter is 52 bits (LO masked to 20).
const COUNTER_MASK: u64 = (1 << 52) - 1;

#[derive(Clone, Copy, Default)]
struct Unit {
    counter: u64,
    value_lo: u32,
    value_hi: u32,
    op_valid: bool,
}

pub struct Systimer {
    regs: [u32; REG_COUNT],
    units: [Unit; 2],
    /// Oneshot (COMP_LOAD-armed) alarm per target.  The arm is CONSUMED when
    /// the alarm fires (single-shot: `armed[n] = false` in `check_alarms` on
    /// fire) — firmware re-arms with a fresh COMP_LOAD write for every new
    /// alarm (`systimer_hal_set_alarm_target`: disable → set target →
    /// COMP_LOAD apply → enable).  Without the consume, a stale target
    /// (counter already past it because the servicing task hasn't run yet)
    /// refires on EVERY step right after the ISR clears INT_RAW, producing
    /// an interrupt storm that starves the very task that would reprogram
    /// the target (proven live 2026-09-20 on the wifi-scan image: TARGET2
    /// RAW bit re-asserted on the step after every clear, ISR every ~183
    /// steps, wifi thread + s_timer_task starved, loopTask parked on the
    /// coex take; forcing TARGET2 future unblocked give #5 + setup progress).
    /// On silicon the same stale-target level would re-assert, but real
    /// concurrency lets the servicing task win the race; in the serialized
    /// emulator the storm can never lose, so the consume models the
    /// single-shot intent (fire once per COMP_LOAD apply).
    armed: [bool; 3],
    /// Period-mode (tick) alarms: whether active and the next effective
    /// boundary per target.
    period_active: [bool; 3],
    period_boundary: [u64; 3],
}

impl Systimer {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        regs[(CONF / 4) as usize] = UNIT0_WORK_EN;
        regs[(DATE / 4) as usize] = DATE_VALUE;
        Self {
            regs,
            units: [Unit::default(), Unit::default()],
            armed: [false; 3],
            period_active: [false; 3],
            period_boundary: [0; 3],
        }
    }

    /// Advance time: each work-enabled unit counter ticks 1 per cycle, then
    /// armed alarms are checked against the counters.
    pub fn tick(&mut self, cycles: u64) {
        let conf = self.regs[(CONF / 4) as usize];
        // Fast path: skip when no unit is counting and no alarm is armed.
        let any_unit = conf & (UNIT0_WORK_EN | UNIT1_WORK_EN);
        let any_alarm = self.armed.iter().any(|&a| a);
        if any_unit == 0 && !any_alarm {
            return;
        }
        for _ in 0..cycles {
            if conf & UNIT0_WORK_EN != 0 {
                self.units[0].counter = (self.units[0].counter + 1) & COUNTER_MASK;
            }
            if conf & UNIT1_WORK_EN != 0 {
                self.units[1].counter = (self.units[1].counter + 1) & COUNTER_MASK;
            }
            self.check_alarms();
        }
    }

    #[allow(clippy::needless_range_loop)] // n indexes parallel per-unit arrays
    fn check_alarms(&mut self) {
        let conf = self.regs[(CONF / 4) as usize];
        let ena = self.regs[(INT_ENA / 4) as usize];
        for n in 0..3 {
            let raw_off = (INT_RAW / 4) as usize;
            let tconf = self.regs[((TARGET_CONF + n as u32 * 4) / 4) as usize];
            let sel = (tconf >> 31) as usize & 1;
            if tconf & (1 << 30) != 0 {
                // Period mode (the FreeRTOS systick): active on work_en +
                // INT_ENA, fires at each period boundary, then the effective
                // target advances by target_period (self-re-arming; the ISR
                // only clears INT_CLR).  Without the advance, the level
                // stays asserted and the tick ISR spins forever.
                if conf & TARGET_WORK_EN[n] != 0 && ena & (1 << n) != 0 {
                    if !self.period_active[n] {
                        self.period_active[n] = true;
                        self.period_boundary[n] = self.units[sel].counter;
                    }
                    let counter = self.units[sel].counter;
                    if self.regs[raw_off] & (1 << n) == 0 && counter >= self.period_boundary[n] {
                        self.regs[raw_off] |= 1 << n;
                        let p = (tconf & ((1 << 26) - 1)).max(1) as u64;
                        self.period_boundary[n] +=
                            ((counter - self.period_boundary[n]) / p + 1) * p;
                    }
                } else {
                    self.period_active[n] = false;
                }
                continue;
            }
            self.period_active[n] = false;
            if !self.armed[n] || conf & TARGET_WORK_EN[n] == 0 || self.regs[raw_off] & (1 << n) != 0
            {
                continue;
            }
            let target = ((self.regs[((TARGET_HI + n as u32 * 8) / 4) as usize] as u64) << 32)
                | self.regs[((TARGET_LO + n as u32 * 8) / 4) as usize] as u64;
            if self.units[sel].counter >= target {
                // Single-shot: consume the COMP_LOAD arm (see `armed` docs).
                self.armed[n] = false;
                self.regs[raw_off] |= 1 << n;
            }
        }
    }

    fn snapshot(&mut self, n: usize) {
        self.units[n].value_lo = self.units[n].counter as u32;
        // HI field is 20 significant bits (bits [51:32] of the 52-bit counter).
        self.units[n].value_hi = ((self.units[n].counter >> 32) & 0xFFFFF) as u32;
        self.units[n].op_valid = true;
    }

    fn load_counter(&mut self, n: usize) {
        let base = if n == 0 { LOAD_HI0 } else { LOAD_HI1 };
        let lo = if n == 0 { LOAD_LO0 } else { LOAD_LO1 };
        self.units[n].counter =
            ((self.regs[(base / 4) as usize] as u64) << 32) | self.regs[(lo / 4) as usize] as u64;
    }

    fn op_read(&self, n: usize) -> u32 {
        if self.units[n].op_valid {
            OP_VALUE_VALID
        } else {
            0
        }
    }

    /// INT_ST = RAW & ENA (TRM SYSTIMER_INT_ST).
    pub fn int_st(&self) -> u32 {
        self.regs[(INT_RAW / 4) as usize] & self.regs[(INT_ENA / 4) as usize]
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            OP0 => self.op_read(0),
            OP1 => self.op_read(1),
            VALUE_LO0 => self.units[0].value_lo,
            VALUE_HI0 => self.units[0].value_hi,
            VALUE_LO1 => self.units[1].value_lo,
            VALUE_HI1 => self.units[1].value_hi,
            INT_ST => self.int_st(),
            off if (REAL_TARGET..=REAL_TARGET + 8).contains(&off) => {
                let n = ((off - REAL_TARGET) / 4) as usize;
                let sel = (self.regs[((TARGET_CONF + n as u32 * 4) / 4) as usize] >> 31) as usize;
                self.units[sel].counter as u32
            }
            _ => self.regs[(offset / 4) as usize],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            OP0 | OP1 => {
                let n = if offset == OP0 { 0 } else { 1 };
                if value & OP_UPDATE != 0 {
                    self.snapshot(n);
                }
                // update/valid are not stored (both are side effects).
            }
            off if (COMP_LOAD..=COMP_LOAD + 8).contains(&off) => {
                let n = ((off - COMP_LOAD) / 4) as usize;
                if value != 0 {
                    self.armed[n] = true;
                    self.check_alarms();
                } else {
                    self.armed[n] = false;
                }
            }
            LOAD_APPLY0 | LOAD_APPLY1 => {
                self.load_counter(if offset == LOAD_APPLY0 { 0 } else { 1 });
            }
            INT_CLR => self.regs[(INT_RAW / 4) as usize] &= !value,
            _ => self.regs[(offset / 4) as usize] = value,
        }
    }
}
impl Default for Systimer {
    fn default() -> Self {
        Self::new()
    }
}
