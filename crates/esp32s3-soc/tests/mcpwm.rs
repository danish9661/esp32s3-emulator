//! MCPWM unit tests: up/down/up-down time base, comparator-driven generator
//! action table producing a correct duty cycle, the GPIO-matrix signal mapping,
//! and the live timer_status readback.

use esp32s3_soc::mcpwm::*;
use std::cell::Cell;

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

// Capture register offsets (mcpwm_cap_*_reg_t).
const CAP_TIMER_CFG: u32 = 0xE8;
const CAP_CHN_CFG0: u32 = 0xF0;
const CAP_CHN0: u32 = 0xFC;
const CAP_STATUS: u32 = 0x108;
const CAP_INT_RAW: u32 = 0x114;
const CAP_INT_CLR: u32 = 0x11C;

/// Drives channel 0 with a scripted level (Cell for the Fn closure).
struct Script {
    levels: Vec<bool>,
    pos: Cell<usize>,
}

impl Script {
    fn input(&self, _sig: u32) -> u32 {
        let p = self.pos.get().min(self.levels.len() - 1);
        self.pos.set(p + 1);
        u32::from(self.levels[p])
    }
}

/// A rising edge latches the free-running timer, records posedge status,
/// and raises the CAP0 interrupt.
#[test]
fn capture_posedge_latches_timer_and_raises_int() {
    let mut m = Mcpwm::new();
    m.write32(CAP_TIMER_CFG, 1); // timer enable
    m.write32(CAP_CHN_CFG0, 1 | (2 << 1)); // ch0 en + posedge
    // idle low, then high: the seed consumes idx0, three setup ticks stay
    // low, the next tick sees the edge (timer reads 5).
    let s = Script {
        levels: vec![false, false, false, false, true, true, true],
        pos: Cell::new(0),
    };
    for _ in 0..4 {
        m.tick_capture(166, &|sig| s.input(sig));
    }
    assert_eq!(m.read32(CAP_INT_RAW) & (1 << 27), 0, "no int yet");
    m.tick_capture(166, &|sig| s.input(sig));
    assert_eq!(m.read32(CAP_CHN0), 5, "timer latched (5 ticks)");
    assert_eq!(m.read32(CAP_STATUS) & 1, 0, "posedge status");
    assert_eq!(m.read32(CAP_INT_RAW) & (1 << 27), 1 << 27, "CAP0 int");
    m.write32(CAP_INT_CLR, 1 << 27);
    assert_eq!(m.read32(CAP_INT_RAW) & (1 << 27), 0, "cleared");
}

/// Prescale divides the input: prescale=1 captures every 2nd edge.
#[test]
fn capture_prescale_divides_edges() {
    let mut m = Mcpwm::new();
    m.write32(CAP_TIMER_CFG, 1);
    m.write32(CAP_CHN_CFG0, 1 | (2 << 1) | (1 << 3)); // en + pos + prescale 1
    // Edges at idx1 (divided out) and idx3 (captured, timer reads 4).
    let s = Script {
        levels: vec![false, true, false, true, true],
        pos: Cell::new(0),
    };
    for _ in 0..3 {
        m.tick_capture(166, &|sig| s.input(sig));
    }
    assert_eq!(
        m.read32(CAP_INT_RAW) & (1 << 27),
        0,
        "first edge divided out"
    );
    m.tick_capture(166, &|sig| s.input(sig));
    assert_eq!(m.read32(CAP_CHN0), 4, "second edge latched");
}

/// Negative-edge mode latches with negedge status; disabled channels idle.
#[test]
fn capture_negedge_and_disabled_channel() {
    let mut m = Mcpwm::new();
    m.write32(CAP_TIMER_CFG, 1);
    m.write32(CAP_CHN_CFG0, 1 | (1 << 1)); // en + negedge
    let s = Script {
        levels: vec![true, true, false, false],
        pos: Cell::new(0),
    };
    m.tick_capture(166, &|sig| s.input(sig));
    m.tick_capture(166, &|sig| s.input(sig));
    assert_eq!(m.read32(CAP_INT_RAW) & (1 << 27), 0, "no edge yet");
    m.tick_capture(166, &|sig| s.input(sig));
    assert_eq!(m.read32(CAP_CHN0), 3, "timer latched");
    assert_eq!(m.read32(CAP_STATUS) & 1, 1, "negedge status");
    // Channel 1 (never enabled) stays quiet.
    assert_eq!(m.read32(CAP_INT_RAW) & (1 << 28), 0);
}
