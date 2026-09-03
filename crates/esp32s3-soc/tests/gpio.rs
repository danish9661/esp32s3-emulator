//! GPIO edge/level interrupt unit tests: per-pin int_type/int_ena config,
//! STATUS latch + w1tc clear, pcpu masking, and the matrix source number.
//!
//! Live edge sampling covers pins 0..32 (the sample word is u32, matching
//! the `gpio_output()` LED path); the STATUS1/pcpu_int1 high-bank registers
//! round-trip for map completeness but no live edges are sampled there.

use esp32s3_soc::gpio::{
    GPIO_IN, GPIO_INTR_SOURCE, GPIO_PCPU_INT, GPIO_PCPU_INT1, GPIO_PIN_0, GPIO_STATUS,
    GPIO_STATUS_W1TC, GPIO_STATUS1, GPIO_STATUS1_W1TC, GPIO_STATUS1_W1TS, Gpio,
};

fn arm(g: &mut Gpio, pin: usize, ty: u32) {
    g.write32(GPIO_PIN_0 + pin as u32 * 4, (ty << 7) | (1 << 13));
}

#[test]
fn intr_source_number_matches_s3_table() {
    // ETS_GPIO_INTR_SOURCE (interrupts.h).
    assert_eq!(GPIO_INTR_SOURCE, 16);
}

#[test]
fn rising_edge_latches_status_and_clears() {
    let mut g = Gpio::new();
    arm(&mut g, 2, 1); // RISING (strap holds bit 2 high; arming seeds high)
    g.poll_interrupts(0); // fall first so the rise below is a real edge
    assert!(!g.int_pending());
    g.poll_interrupts(1 << 2);
    assert!(g.int_pending(), "rising edge fires");
    assert_eq!(g.read32(GPIO_STATUS) & (1 << 2), 1 << 2);
    assert_eq!(g.read32(GPIO_PCPU_INT) & (1 << 2), 1 << 2);
    g.write32(GPIO_STATUS_W1TC, 1 << 2);
    assert!(!g.int_pending(), "w1tc clears");
    // Still high: no re-fire without a new edge.
    g.poll_interrupts(1 << 2);
    assert!(!g.int_pending());
}

#[test]
fn falling_anyedge_and_levels() {
    let mut g = Gpio::new();
    arm(&mut g, 0, 2); // FALLING
    arm(&mut g, 1, 3); // ANYEDGE
    arm(&mut g, 3, 4); // LOW level
    arm(&mut g, 4, 5); // HIGH level
    // Phase A: pin 0 high, pin 3 low. Only the LOW level latches.
    g.poll_interrupts(0x01);
    assert_eq!(g.read32(GPIO_STATUS), 1 << 3, "only low level latches");
    assert!(g.int_pending());
    g.write32(GPIO_STATUS_W1TC, 0xFFFF_FFFF);
    assert!(!g.int_pending());
    // Phase B: raise pins 1 and 4. ANYEDGE (rise) and HIGH level fire;
    // FALLING on the steady-high pin 0 does not.
    g.poll_interrupts(0x1B);
    assert_eq!(
        g.read32(GPIO_STATUS),
        (1 << 1) | (1 << 4),
        "anyedge rise + high level"
    );
    g.write32(GPIO_STATUS_W1TC, 0xFFFF_FFFF);
    // Phase C: drop pins 0 and 3 (FALLING fires on 0, LOW latches
    // on 3), keep pin 4 high and pin 1 steady. No spurious edges.
    g.poll_interrupts(0x12);
    let st = g.read32(GPIO_STATUS);
    assert_ne!(st & (1 << 0), 0, "falling edge fires");
    assert_ne!(st & (1 << 3), 0, "low level re-latches");
    assert_ne!(st & (1 << 4), 0, "high level re-latches");
    g.write32(GPIO_STATUS_W1TC, 0xFFFF_FFFF);
    // Phase D: same levels again — only the levels re-latch, no edges.
    g.poll_interrupts(0x12);
    let st = g.read32(GPIO_STATUS);
    assert_eq!(st & (1 << 0), 0, "no repeated falling edge");
    assert_eq!(st & (1 << 1), 0, "no repeated anyedge");
    assert_ne!(st & (1 << 3), 0, "low level persists");
    assert_ne!(st & (1 << 4), 0, "high level persists");
}

#[test]
fn disabled_or_unarmed_pins_never_fire() {
    let mut g = Gpio::new();
    arm(&mut g, 5, 0); // type DISABLE with ena set
    g.poll_interrupts(0);
    g.poll_interrupts(1 << 5);
    g.poll_interrupts(0);
    assert!(!g.int_pending(), "disabled type never fires");
    assert_eq!(g.read32(GPIO_STATUS), 0);
    // No ena bit at all (plain input pin toggling).
    g.poll_interrupts(1 << 6);
    g.poll_interrupts(0);
    assert!(!g.int_pending(), "unenabled pin never fires");
}

#[test]
fn enable_on_high_pin_does_not_spurious_fire() {
    let mut g = Gpio::new();
    // Drive the input high through the IN register, then arm RISING: the
    // synchronizer seeds high, so no spurious edge fires.
    g.write32(GPIO_IN, 1 << 7);
    arm(&mut g, 7, 1);
    g.poll_interrupts(1 << 7);
    assert!(!g.int_pending(), "no spurious edge on enable");
    // A real low->high transition still fires.
    g.write32(GPIO_IN, 0);
    g.poll_interrupts(0);
    g.write32(GPIO_IN, 1 << 7);
    g.poll_interrupts(1 << 7);
    assert!(g.int_pending());
}

#[test]
fn high_bank_registers_round_trip_and_mask() {
    let mut g = Gpio::new();
    // STATUS1 bank round-trips for map completeness (live sampling covers
    // pins 0..32; see module docs).
    assert_eq!(g.read32(GPIO_STATUS1), 0);
    // A manually latched high-bank bit shows in pcpu_int1 only when armed.
    arm(&mut g, 33, 1);
    g.write32(GPIO_STATUS1_W1TS, 1 << 1); // w1ts bit 1 = pin 33
    assert_ne!(g.read32(GPIO_STATUS1) & 2, 0, "status1 latches");
    assert_ne!(g.read32(GPIO_PCPU_INT1) & 2, 0, "pcpu1 shows armed bit");
    assert!(g.int_pending(), "armed high-bank bit asserts source");
    g.write32(GPIO_STATUS1_W1TC, 2);
    assert!(!g.int_pending(), "w1tc clears high bank");
    assert_eq!(g.read32(GPIO_PCPU_INT), 0, "low pcpu unaffected");
}
