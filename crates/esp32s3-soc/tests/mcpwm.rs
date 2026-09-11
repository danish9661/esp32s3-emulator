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

/// Software sync reloads the stopped timer with PHASE.
#[test]
fn sync_sw_reloads_timer_with_phase() {
    let mut m = Mcpwm::new();
    // Timer0 SYNC @ 0x0C: PHASE=500 in [19:4] + SYNC_SW (bit 1).
    m.write32(0x0C, (500 << 4) | (1 << 1));
    assert_eq!(m.read32(0x10), 500, "timer0 reloaded with phase");
    // A write without SYNC_SW leaves the counter alone.
    m.write32(0x0C, 700 << 4);
    assert_eq!(m.read32(0x10), 500, "no reload without SYNC_SW");
}

/// Dead-time delays rising edges (RED) while FED=0 passes falling edges
/// through immediately.
#[test]
fn dead_time_delays_rising_edge_only() {
    let mut m = Mcpwm::new();
    // DT0 RED = 3 ticks (FED stays 0).
    m.write32(0x60, 3);
    // Force generator A high via GEN_FORCE (oper0 + 0x10, fa=1).
    m.write32(0x4C, 1);
    // Rising edge arms; output stays low for RED ticks.
    m.tick();
    m.tick();
    assert_eq!(m.signal_level(160), 0, "rise held during RED");
    m.tick();
    m.tick();
    assert_eq!(m.signal_level(160), 1, "rise released after RED");
    // Falling edge with FED=0 applies immediately.
    m.write32(0x4C, 2);
    m.tick();
    assert_eq!(m.signal_level(160), 0, "fall immediate with FED=0");
}

// Fault submodule register offsets (mcpwm_reg.h).
const FH0_CFG0: u32 = 0x68;
const FH0_CFG1: u32 = 0x6C;
const FH0_STATUS: u32 = 0x70;
const FAULT_DETECT: u32 = 0xE4;
const INT_ENA: u32 = 0x110;
const INT_RAW: u32 = 0x114;
const INT_CLR: u32 = 0x11C;

/// Run timer0 up-mode (period 100) to steady state with generator0 driving
/// high (utez=set), so a fault trip has a live high level to force low.
fn pwm_high(m: &mut Mcpwm) {
    m.write32(TIMER0_CFG0, 100 << 8);
    m.write32(TIMER0_CFG1, (1 << 3) | 2);
    m.write32(OPER0_GEN0, 1); // utez=set-high
    for _ in 0..100 {
        m.tick();
    }
    assert_eq!(m.signal_level(160), 1, "precondition: output high");
}

/// CBC trip forces the output low while the fault input is active and
/// releases when it clears; enter/exit interrupts latch (bits 9 and 12).
#[test]
fn fault_cbc_trip_forces_low_then_releases() {
    let mut m = Mcpwm::new();
    pwm_high(&mut m);
    // FH0: F0_CBC (bit 3) + A_CBC_U force-low (2 at [11:10]).
    m.write32(FH0_CFG0, (1 << 3) | (2 << 10));
    // FAULT_DETECT: F0_EN + high-active pole.
    m.write32(FAULT_DETECT, (1 << 0) | (1 << 3));
    assert!(m.fault_active(), "detector enabled");
    m.tick_fault(163, &|_| 1);
    assert_eq!(m.signal_level(160), 0, "CBC trip forces low");
    assert_eq!(m.read32(FAULT_DETECT) & (1 << 6), 1 << 6, "EVENT_F0 live");
    assert_eq!(m.read32(INT_RAW) & (1 << 9), 1 << 9, "fault-enter latched");
    // Input clears: CBC ends, output resumes, exit latches.
    m.tick_fault(163, &|_| 0);
    assert_eq!(m.signal_level(160), 1, "CBC releases with event");
    assert_eq!(m.read32(FAULT_DETECT) & (1 << 6), 0, "EVENT_F0 cleared");
    assert_eq!(m.read32(INT_RAW) & (1 << 12), 1 << 12, "fault-exit latched");
}

/// One-shot trip latches the forced level past the end of the event until
/// a CLR_OST rising edge; FH0_STATUS reports OST_ON.
#[test]
fn fault_ost_latches_until_clr_ost() {
    let mut m = Mcpwm::new();
    pwm_high(&mut m);
    // FH0: F0_OST (bit 7) + A_OST_U force-low (2 at [15:14]).
    m.write32(FH0_CFG0, (1 << 7) | (2 << 14));
    m.write32(FAULT_DETECT, (1 << 0) | (1 << 3));
    m.tick_fault(163, &|_| 1);
    assert_eq!(m.signal_level(160), 0, "OST trip forces low");
    assert_eq!(m.read32(FH0_STATUS) & 2, 2, "OST_ON latched");
    // Event ends but the force persists.
    m.tick_fault(163, &|_| 0);
    assert_eq!(m.signal_level(160), 0, "OST holds past the event");
    assert_eq!(m.read32(FH0_STATUS) & 2, 2, "OST_ON still latched");
    // CLR_OST rising edge releases.
    m.write32(FH0_CFG1, 1);
    assert_eq!(m.signal_level(160), 1, "CLR_OST releases");
    assert_eq!(m.read32(FH0_STATUS) & 3, 0, "no trip ongoing");
}

/// Software FORCE_CBC toggle triggers a CBC trip when SW_CBC is enabled.
#[test]
fn fault_software_force_cbc_triggers_trip() {
    let mut m = Mcpwm::new();
    pwm_high(&mut m);
    // SW_CBC (bit 0) + A_CBC_U force-low; no fault input needed.
    m.write32(FH0_CFG0, (1 << 0) | (2 << 10));
    assert_eq!(m.signal_level(160), 1, "no trip before force");
    m.write32(FH0_CFG1, 1 << 3); // FORCE_CBC 0->1 toggle
    assert_eq!(m.signal_level(160), 0, "software CBC forces low");
    assert_eq!(m.read32(FH0_STATUS) & 1, 1, "CBC_ON latched");
}

/// Fault interrupts flow through INT_ENA into int_pending and clear via
/// INT_CLR.
#[test]
fn fault_enter_interrupt_pending_and_clear() {
    let mut m = Mcpwm::new();
    pwm_high(&mut m);
    m.write32(FH0_CFG0, (1 << 3) | (2 << 10));
    m.write32(FAULT_DETECT, (1 << 0) | (1 << 3));
    m.write32(INT_ENA, 1 << 9); // fault0-enter
    assert!(!m.int_pending(), "nothing pending before trip");
    m.tick_fault(163, &|_| 1);
    assert!(m.int_pending(), "enter pending through ENA");
    m.write32(INT_CLR, 1 << 9);
    assert!(!m.int_pending(), "CLR drops the line");
}

const OPER0_CARRIER: u32 = 0x64;
const OPER0_FORCE: u32 = 0x4C;

fn pwm50() -> Mcpwm {
    // timer0 up, period=100, comparator A=50, utez=set / utea=clear.
    let mut m = Mcpwm::new();
    m.write32(TIMER0_CFG0, 100 << 8);
    m.write32(TIMER0_CFG1, (1 << 3) | 2);
    m.write32(OPER0_TSTMP_A, 50);
    m.write32(OPER0_GEN0, (2 << 4) | 1);
    for _ in 0..100 {
        m.tick();
    }
    m
}

#[test]
fn carrier_chops_generator_at_programmed_duty() {
    let mut m = pwm50();
    // Carrier: en + prescale 0 (period 8 steps) + duty 4/8 (50%).
    m.write32(OPER0_CARRIER, 1 | (4 << 5));
    let mut high = 0u32;
    let mut edges = 0u32;
    let mut prev = m.signal_level(160);
    for _ in 0..800 {
        m.tick();
        let lv = m.signal_level(160);
        high += lv;
        edges += u32::from(lv != prev);
        prev = lv;
    }
    // 50% PWM x 50% carrier = 25% average (200/800); unchopped would be 400.
    assert!(
        (180..=220).contains(&high),
        "carrier duty wrong: {high}/800"
    );
    // Chopped: ~100 carrier toggles + 16 PWM edges over 8 periods.
    assert!(edges >= 60, "carrier did not chop: {edges} edges");
}

#[test]
fn carrier_out_invert_flips_wave() {
    let mut m = pwm50();
    // duty 2/8 (25%): normal ~25%, out-inverted ~75% over 800 ticks.
    m.write32(OPER0_CARRIER, 1 | (2 << 5));
    let mut high = 0u32;
    for _ in 0..800 {
        m.tick();
        high += m.signal_level(160);
    }
    assert!((80..=120).contains(&high), "duty 2/8 wrong: {high}/800");
    m.write32(OPER0_CARRIER, 1 | (2 << 5) | (1 << 12));
    let mut high_i = 0u32;
    for _ in 0..800 {
        m.tick();
        high_i += m.signal_level(160);
    }
    assert!(
        (680..=720).contains(&high_i),
        "inverted duty wrong: {high_i}/800"
    );
}

#[test]
fn carrier_in_invert_modulates_low_input() {
    let mut m = Mcpwm::new();
    // Generator A forced LOW (steady), carrier duty 4/8.
    m.write32(OPER0_FORCE, 2);
    for _ in 0..4 {
        m.tick();
    }
    m.write32(OPER0_CARRIER, 1 | (4 << 5));
    let mut high = 0u32;
    for _ in 0..200 {
        m.tick();
        high += m.signal_level(160);
    }
    assert_eq!(high, 0, "LOW input must gate the carrier off");
    // In-invert: the LOW input now reads HIGH and chops at 50%.
    m.write32(OPER0_CARRIER, 1 | (4 << 5) | (1 << 13));
    let mut high_i = 0u32;
    for _ in 0..200 {
        m.tick();
        high_i += m.signal_level(160);
    }
    assert!(
        (80..=120).contains(&high_i),
        "in-inverted wave wrong: {high_i}/200"
    );
}

#[test]
fn carrier_oshtwth_widens_first_pulse() {
    let mut m = Mcpwm::new();
    // duty 0/8 (wave always LOW) + oshtwth 2: the first pulse after the
    // rising edge is forced HIGH for 2 carrier periods (16 steps).
    m.write32(OPER0_CARRIER, 1 | (2 << 8));
    m.write32(OPER0_FORCE, 2);
    for _ in 0..10 {
        m.tick();
    }
    m.write32(OPER0_FORCE, 1);
    for i in 0..16 {
        m.tick();
        assert_eq!(m.signal_level(160), 1, "one-shot must hold HIGH (step {i})");
    }
    // After the one-shot expires the duty-0 wave reads steady LOW.
    for _ in 0..16 {
        m.tick();
    }
    assert_eq!(m.signal_level(160), 0, "wave must be LOW after one-shot");
}

// Timer/operator event interrupts (mcpwm_reg.h INT bits: TIMER0_TEZ=3,
// TIMER0_TEP=6, OP0_TEA=15, OP0_TEB=18).

#[test]
fn timer_tez_and_tep_latch_on_up_wrap() {
    let mut m = Mcpwm::new();
    m.write32(TIMER0_CFG0, 100 << 8);
    m.write32(TIMER0_CFG1, (1 << 3) | 2);
    for _ in 0..100 {
        m.tick();
    }
    let raw = m.read32(INT_RAW);
    assert_ne!(raw & (1 << 3), 0, "TIMER0_TEZ latched, raw={raw:#x}");
    assert_ne!(raw & (1 << 6), 0, "TIMER0_TEP latched, raw={raw:#x}");
}

#[test]
fn timer_tep_latches_at_updown_peak_without_tez() {
    let mut m = Mcpwm::new();
    // Up-down mode (mode=3): bit pattern per TIMER0_CFG1 test above.
    m.write32(TIMER0_CFG0, 100 << 8);
    m.write32(TIMER0_CFG1, (3 << 3) | 2);
    // Peak hits after 100 prescale-1 steps (counter 0 -> 99).
    for _ in 0..100 {
        m.tick();
    }
    let raw = m.read32(INT_RAW);
    assert_ne!(
        raw & (1 << 6),
        0,
        "TIMER0_TEP latched at peak, raw={raw:#x}"
    );
    assert_eq!(raw & (1 << 3), 0, "no TEZ at peak, raw={raw:#x}");
}

#[test]
fn operator_tea_teb_latch_on_compare_match() {
    let mut m = pwm50();
    // Comparator B = 75 (OPER0_TSTMP_B @ 0x44; default 0 would hide
    // behind the TEZ branch like real silicon's zero match).
    m.write32(0x44, 75);
    // One more period crosses both comparator matches.
    for _ in 0..100 {
        m.tick();
    }
    let raw = m.read32(INT_RAW);
    assert_ne!(raw & (1 << 15), 0, "OP0_TEA latched, raw={raw:#x}");
    assert_ne!(raw & (1 << 18), 0, "OP0_TEB latched, raw={raw:#x}");
}

#[test]
fn external_sync_reloads_timer_on_rising_edge() {
    use std::cell::Cell;
    let mut m = Mcpwm::new();
    // Timer0 SYNC @ 0x0C: SYNCI_EN (bit 0) + PHASE=500 in [19:4].
    m.write32(0x0C, (500 << 4) | 1);
    let lvl = Cell::new(0u32);
    let input = |_sig: u32| lvl.get();
    // Low level: no reload (counter advances from 0).
    m.tick_sync(160, &input);
    // Rising edge: reload with PHASE.
    lvl.set(1);
    m.tick_sync(160, &input);
    assert_eq!(m.read32(TIMER0_STATUS), 500, "sync reload with phase");
    // Held high: no second reload (edge-triggered).
    m.tick_sync(160, &input);
    assert_eq!(m.read32(TIMER0_STATUS), 500, "no reload while held");
}

#[test]
fn dead_time_outswap_exchanges_ab_outputs() {
    let mut m = pwm50();
    // DT0_CFG @ 0x58: A_OUTSWAP (bit 9) + B_OUTSWAP (bit 10) —
    // full A/B exchange (a lone A_OUTSWAP leaves B on B).
    m.write32(0x58, (1 << 9) | (1 << 10));
    let mut a_high = 0u32;
    let mut b_high = 0u32;
    for _ in 0..200 {
        m.tick();
        a_high += m.signal_level(160);
        b_high += m.signal_level(161);
    }
    assert_eq!(a_high, 0, "swapped A reads B (low), got {a_high}");
    assert!(
        (95..=105).contains(&b_high),
        "swapped B reads A (~50%), got {b_high}/200"
    );
}
