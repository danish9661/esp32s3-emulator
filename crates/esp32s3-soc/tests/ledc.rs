//! LEDC PWM model unit tests (divider + duty-resolution timing).

use esp32s3_soc::ledc::*;

/// TIMER0 divider 1.0, 10-bit resolution, channel 0 duty 50%:
/// the output must alternate 512 cycles high / 512 low.
#[test]
fn channel_toggles_50_percent_at_10_bit_resolution() {
    let mut l = Lcdc::new();
    l.write32(LEDC_TIMER_CONF_0, 0x0024_0100);
    l.write32(LEDC_CH0_DUTY, 0x2_0000);
    l.write32(LEDC_CH0_CONF0, 12);
    let mut runs = Vec::new();
    let mut prev = l.channel_level(0);
    let mut len = 0u32;
    for _ in 0..2048 {
        let v = l.channel_level(0);
        if v != prev {
            runs.push((prev, len));
            prev = v;
            len = 1;
        } else {
            len += 1;
        }
        l.tick(1);
    }
    runs.push((prev, len));
    assert_eq!(runs, [(1, 512), (0, 512), (1, 512), (0, 512)]);
}

/// duty_start=0 must hold the output low even with sig_out_en set
/// (TRM: the PWM only oscillates once the duty cycle is started).
#[test]
fn output_low_before_duty_start() {
    let mut l = Lcdc::new();
    l.write32(LEDC_TIMER_CONF_0, 0x0024_0100);
    l.write32(LEDC_CH0_DUTY, 0x2_0000);
    l.write32(LEDC_CH0_CONF0, 8); // sig_out_en only
    for _ in 0..64 {
        assert_eq!(l.channel_level(0), 0);
        l.tick(1);
    }
}

/// TIMERx_VALUE register reflects the live phase counter.
#[test]
fn timer_value_tracks_counter() {
    let mut l = Lcdc::new();
    l.write32(LEDC_TIMER_CONF_0, 0x0024_0100);
    l.write32(LEDC_CH0_DUTY, 0x2_0000);
    l.write32(LEDC_CH0_CONF0, 12);
    l.tick(100);
    assert_eq!(l.read32(LEDC_TIMER_VALUE_0), 100);
}

/// Signal indices 96..103 map to channels 0..7 (TRM GPIO matrix table).
#[test]
fn signal_level_maps_channels() {
    let mut l = Lcdc::new();
    assert_eq!(l.signal_level(95), 0);
    assert_eq!(l.signal_level(104), 0);
    l.write32(LEDC_TIMER_CONF_0, 0x0024_0100);
    l.write32(LEDC_CH0_DUTY, 0x2_0000);
    l.write32(LEDC_CH0_CONF0, 12);
    assert_eq!(l.signal_level(LEDC_CH0_SIGNAL), 1);
    assert_eq!(l.signal_level(LEDC_CH0_SIGNAL + 7), 0);
}
