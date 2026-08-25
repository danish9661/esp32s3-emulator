//! RTC_IO register-store model tests.

use esp32s3_soc::rtc_io::RtcIo;

#[test]
fn out_round_trips() {
    let mut d = RtcIo::new();
    d.write32(0x6000_8400, 0x55);
    assert_eq!(d.read32(0x6000_8400), 0x55);
}

#[test]
fn out_w1ts_sets_and_w1tc_clears() {
    let mut d = RtcIo::new();
    d.write32(0x6000_8400, 0x55);
    // out_w1ts (0x04) sets bits.
    d.write32(0x6000_8404, 0xAA);
    assert_eq!(d.read32(0x6000_8400), 0xFF);
    // out_w1tc (0x08) clears bits.
    d.write32(0x6000_8408, 0x0F);
    assert_eq!(d.read32(0x6000_8400), 0xF0);
}

#[test]
fn enable_w1ts_w1tc() {
    let mut d = RtcIo::new();
    d.write32(0x6000_840C, 0x1); // enable
    d.write32(0x6000_8410, 0x4); // enable_w1ts -> set bit 2
    assert_eq!(d.read32(0x6000_840C), 0x5);
    d.write32(0x6000_8414, 0x5); // enable_w1tc -> clear bits 0,2
    assert_eq!(d.read32(0x6000_840C), 0x0);
}

#[test]
fn status_w1ts_w1tc() {
    let mut d = RtcIo::new();
    d.write32(0x6000_8418, 0x0); // status
    d.write32(0x6000_841C, 0x10); // status_w1ts
    assert_eq!(d.read32(0x6000_8418), 0x10);
    d.write32(0x6000_8420, 0x10); // status_w1tc
    assert_eq!(d.read32(0x6000_8418), 0x0);
}

#[test]
fn w1ts_w1tc_reads_return_zero() {
    let mut d = RtcIo::new();
    d.write32(0x6000_8404, 0xFF);
    assert_eq!(d.read32(0x6000_8404), 0);
}

#[test]
fn pad_config_register_stored() {
    let mut d = RtcIo::new();
    // rtc_pad19 is at 0x6000_8400 + 0x1BC in the struct; write + read back.
    d.write32(0x6000_85BC, 0x1234_5678);
    assert_eq!(d.read32(0x6000_85BC), 0x1234_5678);
}
