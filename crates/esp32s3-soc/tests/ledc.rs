//! LEDC PWM model unit tests (divider + duty-resolution timing).

use esp32s3_soc::ledc::Lcdc;

// S3 ledc_struct.h layout: channel 0 at 0x00 (conf0/duty/conf1), timer 0 at
// 0xA0 (conf/value). esp-idf stores duty = user_duty << 4.
const CH0_CONF0: u32 = 0x00;
const CH0_DUTY: u32 = 0x08;
const TMR0_CONF: u32 = 0xA0;
const TMR0_VALUE: u32 = 0xA4;
const SIG_CH0: u32 = 73;
const SIG_CH7: u32 = 80;

/// TIMER0 divider 256 (1 tick/step), 10-bit resolution, channel 0 duty 50%:
/// the output must alternate 512 steps high / 512 low.
#[test]
fn channel_toggles_50_percent_at_10_bit_resolution() {
    let mut l = Lcdc::new();
    l.write32(TMR0_CONF, 0x100A); // div 256, res 10
    l.write32(CH0_DUTY, 0x2000); // user 512 << 4 = 50% of 1024
    l.write32(CH0_CONF0, 4); // sig_out_en
    let mut runs = Vec::new();
    let mut prev = l.signal_level(73);
    let mut len = 0u32;
    for _ in 0..2048 {
        let v = l.signal_level(73);
        if v != prev {
            runs.push((prev, len));
            prev = v;
            len = 1;
        } else {
            len += 1;
        }
        l.tick();
    }
    runs.push((prev, len));
    assert!(runs.len() >= 4);
    assert_eq!(&runs[0..4], &[(1, 512), (0, 512), (1, 512), (0, 512)]);
}

/// A paused timer holds the channel at its idle level (no oscillation).
#[test]
fn paused_timer_holds_idle() {
    let mut l = Lcdc::new();
    l.write32(TMR0_CONF, 0x100A | (1 << 22)); // div 256, res 10, pause
    l.write32(CH0_DUTY, 0x2000);
    l.write32(CH0_CONF0, 4); // sig_out_en
    for _ in 0..64 {
        assert_eq!(l.signal_level(73), 0);
        l.tick();
    }
}

/// TIMERx_VALUE register reflects the live phase counter (1 tick / step at
/// div 256).
#[test]
fn timer_value_tracks_counter() {
    let mut l = Lcdc::new();
    l.write32(TMR0_CONF, 0x100A);
    l.write32(CH0_DUTY, 0x2000);
    l.write32(CH0_CONF0, 4);
    l.tick();
    l.tick();
    assert_eq!(l.read32(TMR0_VALUE), 2);
    l.tick();
    l.tick();
    assert_eq!(l.read32(TMR0_VALUE), 4);
}

/// Signal indices 73..80 map to channels 0..7.
#[test]
fn signal_level_maps_channels() {
    let mut l = Lcdc::new();
    assert_eq!(l.signal_level(72), 0);
    assert_eq!(l.signal_level(81), 0);
    l.write32(TMR0_CONF, 0x100A);
    l.write32(CH0_DUTY, 0x2000);
    l.write32(CH0_CONF0, 4);
    assert_eq!(l.signal_level(SIG_CH0), 1);
    assert_eq!(l.signal_level(SIG_CH7), 0);
}
