//! PCNT unit tests: edge counting, control modes, threshold interrupts.

use esp32s3_soc::pcnt::*;
use std::cell::Cell;

// PCNT_SIG_CH0_IN0_IDX = 33, PCNT_CTRL_CH0_IN0_IDX = 35.
const SIG_CH0: u32 = 33;

fn configure_inc_on_pos() -> Pcnt {
    let mut p = Pcnt::new();
    // unit0 ch0: increment on positive edge, disable on negative edge; control
    // mode KEEP (no modification) on both high/low control levels.
    let conf0 = 1 << 18;
    p.write32(0x00, conf0); // CONF0(0)
    p.write32(0x04, 0); // CONF1(0) thresholds unused
    p.write32(0x08, 0x0000_FFFF); // CONF2 h_lim=0xFFFF, l_lim=0 (no clamp)
    p.write32(0x60, 0); // CTRL: clear resets + unpause
    p
}

#[test]
fn counts_rising_edges_on_ch0() {
    let mut p = configure_inc_on_pos();
    let sig = Cell::new(0u32);
    let input = |s: u32| -> u32 {
        if s == SIG_CH0 {
            sig.get()
        } else {
            1 // control (35) high -> KEEP mode
        }
    };
    p.tick(&input); // initialize previous levels
    for _ in 0..10 {
        sig.set(1);
        p.tick(&input); // rising edge -> +1
        sig.set(0);
        p.tick(&input); // falling edge -> no change
    }
    assert_eq!(p.read32(0x30), 10, "10 rising edges should count to 10");
}

#[test]
fn counts_falling_edges_on_ch0() {
    let mut p = Pcnt::new();
    // decrement on negative edge, disable on positive edge.
    let conf0 = 2 << 16;
    p.write32(0x00, conf0);
    p.write32(0x08, 0xFFFF_0000); // l_lim = -1 (0xFFFF), so it can go negative
    p.write32(0x60, 0);
    let sig = Cell::new(0u32);
    let input = |s: u32| -> u32 { if s == SIG_CH0 { sig.get() } else { 1 } };
    p.tick(&input);
    for _ in 0..5 {
        sig.set(1);
        p.tick(&input);
        sig.set(0);
        p.tick(&input); // falling edge -> -1
    }
    // -5 as u16 = 0xFFFB.
    assert_eq!(p.read32(0x30), 0xFFFB, "5 falling edges should count to -5");
}

#[test]
fn threshold_interrupt_fires_and_clears() {
    let mut p = Pcnt::new();
    // increment on pos edge, enable high-limit threshold at 5.
    let conf0 = (1 << 18) | (1 << 12);
    p.write32(0x00, conf0);
    p.write32(0x08, 0x0000_0005); // h_lim = 5 (bits [15:0])
    p.write32(0x48, 1); // INT_ENA unit0
    p.write32(0x60, 0);
    let sig = Cell::new(0u32);
    let input = |s: u32| -> u32 { if s == SIG_CH0 { sig.get() } else { 1 } };
    p.tick(&input);
    for _ in 0..5 {
        sig.set(1);
        p.tick(&input);
        sig.set(0);
        p.tick(&input);
    }
    assert_ne!(
        p.int_st(),
        0,
        "reaching h_lim=5 should assert the interrupt"
    );
    p.write32(0x4C, 1); // INT_CLR unit0
    assert_eq!(p.int_st(), 0, "int_clr must clear the threshold interrupt");
}
