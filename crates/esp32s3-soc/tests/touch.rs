//! Touch sensor controller unit tests (offsets vs `sens_reg.h`).

use esp32s3_soc::touch::{TOUCH_CHN_ST_OFF, TOUCH_CONF_OFF, TOUCH_OFF_START, Touch};

const STATUS1: u32 = 0xA4;
const THRES1: u32 = 0x64;

#[test]
fn status_returns_injected_counter() {
    let mut t = Touch::new();
    t.inject(1, 2500);
    assert_eq!(t.read32(STATUS1), 2500);
}

#[test]
fn meas_done_always_set() {
    let mut t = Touch::new();
    assert_eq!(t.read32(TOUCH_CHN_ST_OFF) & (1 << 31), 1 << 31);
}

#[test]
fn active_follows_threshold_compare() {
    let mut t = Touch::new();
    t.write32(THRES1, 3000);
    t.inject(1, 2500); // touched (counter below threshold)
    assert_eq!(t.read32(TOUCH_CHN_ST_OFF) & 1, 1);
    t.inject(1, 3500); // released
    assert_eq!(t.read32(TOUCH_CHN_ST_OFF) & 1, 0);
}

#[test]
fn plain_registers_round_trip() {
    let mut t = Touch::new();
    t.write32(TOUCH_CONF_OFF, 0x7FFF);
    assert_eq!(t.read32(TOUCH_CONF_OFF), 0x7FFF);
    t.write32(THRES1, 0x2AAAAA);
    assert_eq!(t.read32(THRES1), 0x2AAAAA);
}

#[test]
fn below_window_reads_zero() {
    let mut t = Touch::new();
    assert_eq!(t.read32(TOUCH_OFF_START - 4), 0);
}
