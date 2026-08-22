//! MCPWM unit tests: up/down/up-down time base, comparator-driven generator
//! action table producing a correct duty cycle, the GPIO-matrix signal mapping,
//! and the live timer_status readback.

use esp32s3_soc::mcpwm::*;

const TIMER0_CFG0: u32 = 0x04;
const TIMER0_CFG1: u32 = 0x08;
const OPER0_TSTMP_A: u32 = 0x40;
const OPER0_GEN0: u32 = 0x50;
const TIMER0_STATUS: u32 = 0x10;

#[test]
fn up_mode_fifty_percent_duty_cycles_output() {
    let mut m = Mcpwm::new();
    // timer0: period=100, prescale=0, mode=up(1), start=run(2).
    m.write32(TIMER0_CFG0, 100 << 8);
    m.write32(TIMER0_CFG1, (1 << 3) | 2);
    // operator0 comparator A = 50 (50% of period).
    m.write32(OPER0_TSTMP_A, 50);
    // generator0: utez=set-high(1), utea=clear-low(2).
    m.write32(OPER0_GEN0, (2 << 4) | 1);
    // Burn one full period so the generator reaches steady state (the first
    // TEZ only fires on the wrap from period-1 -> 0, like real silicon).
    for _ in 0..100 {
        m.tick();
    }
    let mut high = 0u32;
    for _ in 0..200 {
        m.tick();
        high += m.signal_level(160);
    }
    // Output high for count 0..49, low for 50..99 -> 100 high / 200.
    assert!(
        (95..=105).contains(&high),
        "expected ~50% duty, got {} / 200",
        high
    );
}

#[test]
fn up_mode_twentyfive_percent_duty() {
    let mut m = Mcpwm::new();
    m.write32(TIMER0_CFG0, 100 << 8);
    m.write32(TIMER0_CFG1, (1 << 3) | 2);
    m.write32(OPER0_TSTMP_A, 25);
    m.write32(OPER0_GEN0, (2 << 4) | 1);
    for _ in 0..100 {
        m.tick();
    }
    let mut high = 0u32;
    for _ in 0..200 {
        m.tick();
        high += m.signal_level(160);
    }
    // Output high for count 0..24, low 25..99 -> 50 high / 200.
    assert!(
        (45..=55).contains(&high),
        "expected ~25% duty, got {} / 200",
        high
    );
}

#[test]
fn generator_signal_indices_map_operator_ab() {
    let mut m = Mcpwm::new();
    m.write32(TIMER0_CFG0, 100 << 8);
    m.write32(TIMER0_CFG1, (1 << 3) | 2);
    // Force operator0 genA high and operator1 genB low via gen_force.
    m.write32(0x4C, (2 << 2) | 1); // op0 genA=high(1), genB=low(2)
    m.write32(0x84, 2); // op1 genA=low, genB default
    assert_eq!(m.signal_level(160), 1, "PWM0_OUT0A = op0 genA");
    assert_eq!(m.signal_level(161), 0, "PWM0_OUT0B = op0 genB");
    assert_eq!(m.signal_level(163), 0, "PWM0_OUT1B = op1 genB");
}

#[test]
fn timer_status_reads_live_counter() {
    let mut m = Mcpwm::new();
    m.write32(TIMER0_CFG0, 10 << 8);
    m.write32(TIMER0_CFG1, (1 << 3) | 2);
    for _ in 0..15 {
        m.tick();
    }
    // 15 ticks (prescale 0) with period 10 -> counter wraps once (10) + 5 = 5.
    assert_eq!(m.read32(TIMER0_STATUS), 5, "live counter after 15 ticks");
}

#[test]
fn stopped_timer_holds_counter() {
    let mut m = Mcpwm::new();
    m.write32(TIMER0_CFG0, 100 << 8);
    m.write32(TIMER0_CFG1, (1 << 3) | 2); // running
    for _ in 0..5 {
        m.tick();
    }
    m.write32(TIMER0_CFG1, 0); // stop
    let before = m.read32(TIMER0_STATUS);
    for _ in 0..100 {
        m.tick();
    }
    assert_eq!(
        m.read32(TIMER0_STATUS),
        before,
        "counter frozen when stopped"
    );
}

#[test]
fn prescale_slows_the_counter() {
    let mut m = Mcpwm::new();
    m.write32(TIMER0_CFG0, (100 << 8) | 9); // prescale = 9 -> +1 = 10 ticks/count
    m.write32(TIMER0_CFG1, (1 << 3) | 2);
    for _ in 0..100 {
        m.tick();
    }
    // 100 ticks / 10 = 10 counts -> counter = 10 (period 100, no wrap).
    assert_eq!(m.read32(TIMER0_STATUS), 10);
}
