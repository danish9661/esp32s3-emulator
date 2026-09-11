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

/// Proximity mode: an armed approach channel (CONF.approach_pad0 = pad)
/// saturates its APPR_STATUS counter while the pad reads active and
/// clears the moment it releases.
#[test]
fn approach_counter_tracks_pad_activity() {
    use esp32s3_soc::touch::Touch;
    let mut t = Touch::new();
    // THRES3 = 2000, approach_pad0 = 3.
    t.write32(0x64 + 4 * 2, 2000);
    t.write32(0x5C, 3 << 28);
    t.inject(3, 1877);
    t.tick(300);
    assert_eq!(t.read32(0xE0) & 0xFF00, 0xFF00, "pad0_cnt saturates");
    // Release (counter above threshold): clears.
    t.inject(3, 2500);
    t.tick(1);
    assert_eq!(t.read32(0xE0) & 0xFF00, 0, "clears on release");
    // Unarmed channels stay zero.
    assert_eq!(t.read32(0xE0) & 0xFFFF_0000, 0);
}

/// SLP_STATUS latches the sleep-entry triggering pad's counter.
#[test]
fn sleep_status_latches_triggering_pad() {
    use esp32s3_soc::touch::Touch;
    let mut t = Touch::new();
    t.write32(0x64 + 4 * 2, 2000);
    t.inject(3, 1877);
    t.set_sleep_data();
    assert_eq!(t.read32(0xDC), 1877);
}

/// STATUS0 denoise_data reads the pad-0 injection.
#[test]
fn denoise_status_reads_pad0_inject() {
    use esp32s3_soc::touch::Touch;
    let mut t = Touch::new();
    t.inject(0, 1234);
    assert_eq!(t.read32(0xA0) & 0x3F_FFFF, 1234);
    assert_eq!(t.read32(0xA4) & 0x3F_FFFF, 0, "pad1 untouched");
}
