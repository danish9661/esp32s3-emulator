//! Sigma-Delta modulator model unit tests (duty ratio, signal routing,
//! prescale, register readback).

use esp32s3_soc::sigmadelta::Sdm;

const SIG0: u32 = 93; // GPIO_SD0_OUT_IDX
const SIG7: u32 = 100; // GPIO_SD7_OUT_IDX

/// Configure channel `ch` duty/prescale via the channel register (duty[7:0],
/// prescale[15:8]) and return the fraction of `n` ticks the channel's signal
/// is high.
fn high_fraction(sdm: &mut Sdm, ch: usize, n: u32) -> u32 {
    let mut high = 0u32;
    for _ in 0..n {
        high += sdm.signal_level(SIG0 + ch as u32);
        sdm.tick();
    }
    high
}

#[test]
fn channel_duty_50_percent_is_half_high() {
    let mut s = Sdm::new();
    // esp-idf writes the SIGNED duty into the 8-bit register; 0 (reg 0x00) is
    // the 50% point (threshold = 0 + 128 = 128 of 256).
    s.write32(0, 0);
    assert_eq!(high_fraction(&mut s, 0, 256), 128);
}

#[test]
fn channel_duty_25_percent_is_quarter_high() {
    let mut s = Sdm::new();
    // 25% -> threshold 64 -> signed duty = -64 -> reg 0xC0 = 192.
    s.write32(0, 192);
    assert_eq!(high_fraction(&mut s, 0, 256), 64);
}

#[test]
fn prescale_slows_the_period_by_prescale_plus_one() {
    let mut s = Sdm::new();
    s.write32(0, 7 << 8); // duty=0 (50%), prescale=7 -> period 256*8
    // Over one period (2048 ticks) exactly half (1024) are high.
    assert_eq!(high_fraction(&mut s, 0, 2048), 1024);
}

#[test]
fn each_channel_routes_to_its_own_signal() {
    let mut s = Sdm::new();
    s.write32(0, 0); // ch0 50% -> 128 high
    s.write32(4, 138); // ch1 signed -118 -> threshold 10 -> 10 high
    assert_eq!(high_fraction(&mut s, 0, 256), 128);
    let mut s2 = Sdm::new();
    s2.write32(4, 138);
    assert_eq!(high_fraction(&mut s2, 1, 256), 10);
    // Out-of-range signal reads 0.
    assert_eq!(s2.signal_level(SIG7 + 1), 0);
}

#[test]
fn control_registers_read_back() {
    let mut s = Sdm::new();
    s.write32(0x20, 1 << 31); // cg.clk_en
    s.write32(0x24, 1 << 30); // misc.function_clk_en
    s.write32(0x28, 0x1234_5678); // version.date
    assert_eq!(s.read32(0x20), 1 << 31);
    assert_eq!(s.read32(0x24), 1 << 30);
    assert_eq!(s.read32(0x28), 0x1234_5678);
    // Reserved bits of a channel register are dropped (only low 16 bits).
    s.write32(0, 0xFFFF_ABCD);
    assert_eq!(s.read32(0), 0xABCD);
}
