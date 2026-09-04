//! SAR ADC model unit tests: RTC oneshot start/done/data handshake,
//! attenuation scaling, data invert, meas_status busy, and the APB_SARADC
//! digital pattern-table / timer paths.

// `(0 << n)` markers document field positions in register words (mirrors
// the generated.rs allow header).
#![allow(clippy::identity_op)]

use esp32s3_soc::adc::*;

/// Starts a SW-driven ADC1 oneshot conversion on `channel` after
/// `adc` is in RTC mode (the adc_oneshot_ll_start 0-then-1 write
/// sequence).
fn oneshot_start(adc: &mut Adc, unit: usize, channel: u32) {
    let off = if unit == 0 {
        SENS_SAR_MEAS1_CTRL2
    } else {
        SENS_SAR_MEAS2_CTRL2
    };
    let base = (1 << 31) | (1 << 18) | (1 << (19 + channel));
    adc.sens_write32(off, base); // start = 0
    adc.sens_write32(off, base | (1 << 17)); // start = 1
}

#[test]
fn oneshot_converts_injected_voltage() {
    let mut adc = Adc::new();
    adc.inject_voltage(0, 3, 825);
    // 825 mV at 0 dB (full-scale 1.1 V, default sar_atten1 is all-ones so
    // explicitly clear channel 3) -> 825 * 4095 / 1100 = 3071.
    adc.sens_write32(SENS_SAR_ATTEN1, 0);
    oneshot_start(&mut adc, 0, 3);
    assert_ne!(
        adc.sens_read32(SENS_SAR_SLAVE_ADDR1) & (0xFF << 22),
        0,
        "meas_status busy while converting"
    );
    adc.tick(7);
    assert_eq!(
        adc.sens_read32(SENS_SAR_MEAS1_CTRL2) & 0xFFFF,
        0,
        "not done yet"
    );
    adc.tick(1);
    let v = adc.sens_read32(SENS_SAR_MEAS1_CTRL2);
    assert_ne!(v & (1 << 16), 0, "meas1_done_sar set");
    assert_eq!(v & 0xFFFF, 3071, "meas1_data_sar = raw");
    assert_eq!(
        adc.sens_read32(SENS_SAR_SLAVE_ADDR1) & (0xFF << 22),
        0,
        "meas_status idle after conversion"
    );
}

#[test]
fn oneshot_honors_attenuation_full_scale() {
    let mut adc = Adc::new();
    adc.inject_voltage(0, 3, 3900);
    // sar_atten1 default 0xFFFFFFFF = attenuation 3 (11 dB, 3.9 V).
    oneshot_start(&mut adc, 0, 3);
    adc.tick(8);
    let v = adc.sens_read32(SENS_SAR_MEAS1_CTRL2);
    assert_eq!(
        v & 0xFFFF,
        4095,
        "3900 mV at 11 dB saturates the 12-bit raw"
    );
    adc.inject_voltage(0, 3, 0);
    adc.sens_write32(SENS_SAR_ATTEN1, 0); // channel 3 -> 0 dB
    oneshot_start(&mut adc, 0, 3);
    adc.tick(8);
    assert_eq!(
        adc.sens_read32(SENS_SAR_MEAS1_CTRL2) & 0xFFFF,
        0,
        "0 mV reads 0"
    );
}

#[test]
fn oneshot_data_invert_flips_result() {
    let mut adc = Adc::new();
    adc.inject_voltage(0, 2, 550);
    adc.sens_write32(SENS_SAR_ATTEN1, 0);
    oneshot_start(&mut adc, 0, 2);
    adc.tick(8);
    let plain = adc.sens_read32(SENS_SAR_MEAS1_CTRL2) & 0xFFFF;
    // 550 mV at 0 dB -> 2047.  Re-read with sar1_data_inv set.
    assert_eq!(plain, 550 * 4095 / 1100);
    adc.sens_write32(SENS_SAR_READER1_CTRL, 1 << 28); // sar1_data_inv
    oneshot_start(&mut adc, 0, 2);
    adc.tick(8);
    assert_eq!(
        adc.sens_read32(SENS_SAR_MEAS1_CTRL2) & 0xFFFF,
        plain ^ 0xFFF,
        "data_inv bitwise-inverts the 12-bit result"
    );
}

#[test]
fn oneshot_next_start_clears_done() {
    let mut adc = Adc::new();
    adc.inject_voltage(0, 0, 1100);
    adc.sens_write32(SENS_SAR_ATTEN1, 0);
    oneshot_start(&mut adc, 0, 0);
    adc.tick(8);
    assert_ne!(
        adc.sens_read32(SENS_SAR_MEAS1_CTRL2) & (1 << 16),
        0,
        "done set"
    );
    // start = 0 write (adc_oneshot_ll_start) clears the latched done bit.
    adc.sens_write32(SENS_SAR_MEAS1_CTRL2, (1 << 31) | (1 << 18) | (1 << 19));
    assert_eq!(
        adc.sens_read32(SENS_SAR_MEAS1_CTRL2) & (1 << 16),
        0,
        "start write clears done_sar"
    );
}

#[test]
fn oneshot_requires_rtc_controller() {
    let mut adc = Adc::new();
    adc.inject_voltage(0, 1, 1100);
    adc.sens_write32(SENS_SAR_ATTEN1, 0);
    adc.sens_write32(SENS_SAR_MEAS1_MUX, 1 << 31); // sar1_dig_force = 1
    oneshot_start(&mut adc, 0, 1);
    adc.tick(8);
    assert_eq!(
        adc.sens_read32(SENS_SAR_MEAS1_CTRL2) & (1 << 16),
        0,
        "dig_force=1 ignores the RTC oneshot start"
    );
    adc.sens_write32(SENS_SAR_MEAS1_MUX, 0); // back to RTC
    oneshot_start(&mut adc, 0, 1);
    adc.tick(8);
    assert_ne!(
        adc.sens_read32(SENS_SAR_MEAS1_CTRL2) & (1 << 16),
        0,
        "converts"
    );
}

#[test]
fn digital_single_converts_pattern_table() {
    let mut adc = Adc::new();
    adc.inject_voltage(0, 3, 825);
    // One pattern entry: channel 3, attenuation 0 -> byte 0x0C.
    adc.apb_write32(APB_SAR1_PATT_TAB, 0x0C);
    // ctrl: sar_clk_gated, single mode SAR1, sar1_patt_len = 0, plus the
    // start_force|start pulse (start self-clears).
    adc.apb_write32(
        APB_CTRL,
        (1 << 6) | (0 << 3) | (0 << 5) | (0 << 15) | (1 << 1) | (1 << 0),
    );
    assert_eq!(adc.apb_read32(APB_CTRL) & (1 << 1), 0, "start is a pulse");
    assert_ne!(
        adc.apb_read32(APB_INT_RAW) & APB_ADC1_DONE,
        0,
        "adc1_done raw raised"
    );
    assert_eq!(
        adc.apb_read32(APB_SARADC1_DATA_STATUS) & 0xFFFF,
        825 * 4095 / 1100,
        "adc1_data = last pattern conversion"
    );
    assert_eq!(
        adc.apb_read32(APB_INT_ST) & APB_ADC1_DONE,
        0,
        "masked by int_ena"
    );
    adc.apb_write32(APB_INT_CLR, APB_ADC1_DONE);
    assert_eq!(
        adc.apb_read32(APB_INT_RAW) & APB_ADC1_DONE,
        0,
        "clr clears raw"
    );
}

#[test]
fn digital_timer_triggers_periodic_conversion() {
    let mut adc = Adc::new();
    adc.inject_voltage(0, 2, 825);
    adc.inject_voltage(1, 2, 550);
    // Pattern byte = atten[1:0] | channel[6:2] -> ch2 atten 0 = 0x08.
    adc.apb_write32(APB_SAR1_PATT_TAB, 2 << 2);
    adc.apb_write32(APB_SAR2_PATT_TAB, 2 << 2);
    // ctrl: clk gated, double mode, patt_len 0 both.
    adc.apb_write32(APB_CTRL, (1 << 6) | (1 << 3) | (0 << 15) | (0 << 19));
    // ctrl2: timer_en + timer_sel, timer_target = 4 (period 5 cycles).
    adc.apb_write32(APB_CTRL2, (1 << 24) | (1 << 11) | (4 << 12));
    adc.tick(4);
    assert_eq!(
        adc.apb_read32(APB_INT_RAW) & (APB_ADC1_DONE | APB_ADC2_DONE),
        0,
        "not yet"
    );
    adc.tick(1);
    assert_eq!(
        adc.apb_read32(APB_SARADC1_DATA_STATUS) & 0xFFFF,
        825 * 4095 / 1100,
        "SAR1 pass: 825 mV at 0 dB"
    );
    assert_eq!(
        adc.apb_read32(APB_SARADC2_DATA_STATUS) & 0xFFFF,
        550 * 4095 / 1100,
        "SAR2 pass: 550 mV at 0 dB"
    );
    assert_ne!(
        adc.apb_read32(APB_INT_RAW) & (APB_ADC1_DONE | APB_ADC2_DONE),
        0,
        "both done raw set"
    );
    adc.tick(5);
    assert_ne!(
        adc.apb_read32(APB_INT_RAW) & APB_ADC1_DONE,
        0,
        "periodic: still converting on later ticks"
    );
}

#[test]
fn adc2_oneshot_converts_without_arbiter() {
    let mut adc = Adc::new();
    adc.inject_voltage(1, 5, 550);
    adc.sens_write32(SENS_SAR_ATTEN2, 0);
    oneshot_start(&mut adc, 1, 5);
    adc.tick(8);
    assert_eq!(
        adc.sens_read32(SENS_SAR_MEAS2_CTRL2) & 0xFFFF,
        550 * 4095 / 1100,
        "ADC2 raw at 0 dB"
    );
    assert_ne!(
        adc.sens_read32(SENS_SAR_MEAS2_CTRL2) & (1 << 16),
        0,
        "meas2_done_sar set"
    );
}

/// Digital conversions stage results for the GDMA `in` walk: each timer
/// pass pushes its data_status word, drained in order via `dma_pop`.
#[test]
fn digital_results_stage_for_gdma() {
    let mut adc = Adc::new();
    adc.inject_voltage(0, 2, 825);
    adc.apb_write32(APB_SAR1_PATT_TAB, 2 << 2);
    // ctrl: clk gated, single mode unit 0, patt_len 0.
    adc.apb_write32(APB_CTRL, (1 << 6) | (0 << 3) | (0 << 15));
    adc.apb_write32(APB_CTRL2, (1 << 24) | (1 << 11) | (4 << 12));
    assert_eq!(adc.dma_pop(), None, "nothing staged yet");
    adc.tick(5);
    assert_eq!(adc.dma_pop(), Some(825 * 4095 / 1100));
    adc.tick(5);
    assert_eq!(adc.dma_pop(), Some(825 * 4095 / 1100));
    assert_eq!(adc.dma_pop(), None, "queue drained");
}
