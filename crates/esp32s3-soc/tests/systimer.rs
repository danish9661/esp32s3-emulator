//! SYSTIMER peripheral tests (systimer.rs).
//!
//! Register offsets follow esp32s3 systimer_reg.h; the snapshot handshake
//! and alarm path mirror esp_timer_impl on the S3 (see the module docs).

#![allow(clippy::identity_op)]

use esp32s3_soc::soc::Soc;
use xtensa_core::Bus;

const OP0: u32 = 0x004;
const CONF: u32 = 0x000;
const TARGET0_WORK_EN: u32 = 1 << 24;
const LOAD_HI0: u32 = 0x00C;
const LOAD_LO0: u32 = 0x010;
const TARGET_HI0: u32 = 0x01C;
const TARGET_LO0: u32 = 0x020;
const TARGET_CONF0: u32 = 0x034;
const VALUE_HI0: u32 = 0x040;
const VALUE_LO0: u32 = 0x044;
const COMP_LOAD0: u32 = 0x050;
const LOAD_APPLY0: u32 = 0x05C;
const INT_ENA: u32 = 0x064;
const INT_RAW: u32 = 0x068;
const INT_CLR: u32 = 0x06C;
const INT_ST: u32 = 0x070;

const SYSTIMER_BASE: u32 = 0x6002_3000;

fn st() -> Soc {
    Soc::new()
}

#[test]
fn snapshot_handshake_sets_valid_and_latches_counter() {
    let mut s = st();
    // Write the OP register (bit 30 = timer_unit_update): the driver's
    // systimer_hal_get_counter_value spins on bit 29 (value_valid) after
    // this write, then reads the VALUE latches.
    s.write32(SYSTIMER_BASE + OP0, 1 << 30);
    // Counter default = 0; valid must be set immediately (poll exits).
    assert_eq!(s.read32(SYSTIMER_BASE + OP0), 1 << 29);
    assert_eq!(s.read32(SYSTIMER_BASE + VALUE_LO0), 0);
    assert_eq!(s.read32(SYSTIMER_BASE + VALUE_HI0), 0);
    // A read of the OP register before any snapshot shows valid = 0.
    let mut s2 = st();
    assert_eq!(s2.read32(SYSTIMER_BASE + OP0), 0);
}

#[test]
fn counter_ticks_and_load_apply_sets_it() {
    let mut s = st();
    // unit0_work_en defaults to 1: the counter free-runs at 1/cycle.
    s.tick_timers(5);
    s.write32(SYSTIMER_BASE + OP0, 1 << 30);
    assert_eq!(s.read32(SYSTIMER_BASE + VALUE_LO0), 5);
    // Load a value (hi first, then lo — reg order) and apply it.
    s.write32(SYSTIMER_BASE + LOAD_HI0, 0x1);
    s.write32(SYSTIMER_BASE + LOAD_LO0, 0xABCDE);
    s.write32(SYSTIMER_BASE + LOAD_APPLY0, 1);
    s.write32(SYSTIMER_BASE + OP0, 1 << 30);
    assert_eq!(s.read32(SYSTIMER_BASE + VALUE_LO0), 0xABCDE);
    assert_eq!(s.read32(SYSTIMER_BASE + VALUE_HI0), 0x1);
}

#[test]
fn alarm_target_fires_int_st_and_clears() {
    let mut s = st();
    // esp_timer flow: set target (hi/lo), select unit 0, enable the alarm
    // work bit, arm with comp_load, enable int, then the counter reaches it.
    s.write32(SYSTIMER_BASE + TARGET_HI0, 0);
    s.write32(SYSTIMER_BASE + TARGET_LO0, 10);
    s.write32(SYSTIMER_BASE + TARGET_CONF0, 0); // target_timer_unit_sel = 0
    let conf = s.read32(SYSTIMER_BASE + CONF);
    s.write32(SYSTIMER_BASE + CONF, conf | TARGET0_WORK_EN);
    s.write32(SYSTIMER_BASE + COMP_LOAD0, 1);
    s.write32(SYSTIMER_BASE + INT_ENA, 1 << 0);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 0);
    s.tick_timers(9);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 0);
    s.tick_timers(1);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 1 << 0);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_ST), 1 << 0);
    // INT_CLR write clears RAW (level-triggered).
    s.write32(SYSTIMER_BASE + INT_CLR, 1 << 0);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 0);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_ST), 0);
}

#[test]
fn alarm_with_past_target_fires_on_arm() {
    let mut s = st();
    s.tick_timers(100);
    // Target already behind the counter: arming must fire immediately.
    s.write32(SYSTIMER_BASE + TARGET_HI0, 0);
    s.write32(SYSTIMER_BASE + TARGET_LO0, 10);
    s.write32(SYSTIMER_BASE + TARGET_CONF0, 0);
    let conf = s.read32(SYSTIMER_BASE + CONF);
    s.write32(SYSTIMER_BASE + CONF, conf | TARGET0_WORK_EN);
    s.write32(SYSTIMER_BASE + COMP_LOAD0, 1);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 1 << 0);
}

#[test]
fn systimer_int_source_reaches_intc() {
    let mut s = st();
    // Program the matrix: source 57 (SYSTIMER_TARGET0; 56 is CACHE_IA on
    // the S3) -> line 1, CPU 0.
    s.write32(0x600C_2000 + 57 * 4, 1);
    s.write32(SYSTIMER_BASE + TARGET_HI0, 0);
    s.write32(SYSTIMER_BASE + TARGET_LO0, 3);
    s.write32(SYSTIMER_BASE + TARGET_CONF0, 0);
    let conf = s.read32(SYSTIMER_BASE + CONF);
    s.write32(SYSTIMER_BASE + CONF, conf | TARGET0_WORK_EN);
    s.write32(SYSTIMER_BASE + COMP_LOAD0, 1);
    s.write32(SYSTIMER_BASE + INT_ENA, 1 << 0);
    assert_eq!(s.int_pending(0), 0);
    s.tick_timers(3);
    assert_eq!(s.int_pending(0), 1 << 1);
}

#[test]
fn period_mode_rearms_itself_after_int_clr() {
    let mut s = st();
    // FreeRTOS tick semantics: period mode, no target/COMP_LOAD.  Period 100
    // (target_period [25:0]); the alarm is active on work_en + INT_ENA.
    s.write32(SYSTIMER_BASE + TARGET_CONF0, (1 << 30) | 100);
    let conf = s.read32(SYSTIMER_BASE + CONF);
    s.write32(SYSTIMER_BASE + CONF, conf | TARGET0_WORK_EN);
    s.write32(SYSTIMER_BASE + INT_ENA, 1 << 0);
    s.tick_timers(5);
    // Fires at the very first boundary (the counter sits at ~0).
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 1 << 0);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_ST), 1 << 0);
    // Clearing must NOT re-fire: the boundary has advanced past the counter.
    s.write32(SYSTIMER_BASE + INT_CLR, 1 << 0);
    s.tick_timers(90);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 0);
    // The next fire arrives at the next period boundary (counter ~100).
    s.tick_timers(10);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 1 << 0);
}

#[test]
fn period_mode_without_work_en_stays_quiet() {
    let mut s = st();
    s.write32(SYSTIMER_BASE + TARGET_CONF0, (1 << 30) | 10);
    s.write32(SYSTIMER_BASE + INT_ENA, 1 << 0);
    s.tick_timers(50);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 0);
}

#[test]
fn oneshot_arm_consumed_on_fire_no_refire_after_clr() {
    // Single-shot intent (fire once per COMP_LOAD apply): after the alarm
    // fires and the ISR clears INT_RAW, the alarm must NOT re-fire on the
    // next tick — otherwise a stale target (counter already past it because
    // the servicing task hasn't run yet) storms every step and starves the
    // task that would reprogram it (wifi-scan TARGET2 livelock, 2026-09-20).
    // A fresh COMP_LOAD write re-arms for the next alarm.
    let mut s = st();
    s.write32(SYSTIMER_BASE + TARGET_HI0, 0);
    s.write32(SYSTIMER_BASE + TARGET_LO0, 10);
    s.write32(SYSTIMER_BASE + TARGET_CONF0, 0);
    let conf = s.read32(SYSTIMER_BASE + CONF);
    s.write32(SYSTIMER_BASE + CONF, conf | TARGET0_WORK_EN);
    s.write32(SYSTIMER_BASE + COMP_LOAD0, 1);
    s.write32(SYSTIMER_BASE + INT_ENA, 1 << 0);
    s.tick_timers(10);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 1 << 0);
    // ISR clears...
    s.write32(SYSTIMER_BASE + INT_CLR, 1 << 0);
    // ...and the stale target must stay quiet (arm consumed).
    s.tick_timers(100);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 0);
    // Fresh COMP_LOAD with a future target re-arms and fires again.
    s.write32(SYSTIMER_BASE + TARGET_LO0, 200);
    s.write32(SYSTIMER_BASE + COMP_LOAD0, 1);
    s.tick_timers(100);
    assert_eq!(s.read32(SYSTIMER_BASE + INT_RAW), 1 << 0);
}
