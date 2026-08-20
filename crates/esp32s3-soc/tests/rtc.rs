//! Rtc (RTC_CNTL slow-clock timer) unit tests: TIME_UPDATE latch semantics
//! and the RC_SLOW divider (240 MHz CPU / 32.5 kHz slow clock).

use esp32s3_soc::rtc::Rtc;
use xtensa_core::Bus;

const TIME_UPDATE_OFF: u32 = 0x0C;
const TIME_VALUE_LO_OFF: u32 = 0x10;
const TIME_VALUE_HI_OFF: u32 = 0x14;

// 240_000_000 / 32_500 = 7384 (integer division; 0.6 cycles of error per
// slow tick, same as the device model).
const SLOW_CLK_DIV: u64 = 7384;

#[test]
fn update_latches_count_on_bit31_write() {
    let mut r = Rtc::new();
    r.tick(SLOW_CLK_DIV);
    assert_eq!(r.read32(TIME_VALUE_LO_OFF), 0, "no latch yet");
    r.write32(TIME_UPDATE_OFF, 1 << 31);
    assert_eq!(r.read32(TIME_VALUE_LO_OFF), 1, "latched low word");
    assert_eq!(r.read32(TIME_VALUE_HI_OFF), 0, "latched high word");
    // The count keeps running; a new latch shows it.
    r.tick(SLOW_CLK_DIV * 5);
    r.write32(TIME_UPDATE_OFF, 1 << 31);
    assert_eq!(r.read32(TIME_VALUE_LO_OFF), 6, "count advanced");
}

#[test]
fn writes_without_bit31_do_not_latch() {
    let mut r = Rtc::new();
    r.tick(SLOW_CLK_DIV * 3);
    r.write32(TIME_UPDATE_OFF, 0);
    r.write32(TIME_UPDATE_OFF, 0x1000);
    assert_eq!(r.read32(TIME_VALUE_LO_OFF), 0);
    r.write32(TIME_UPDATE_OFF, 1 << 31);
    assert_eq!(r.read32(TIME_VALUE_LO_OFF), 3);
}

#[test]
fn counter_wraps_low_word_at_32_bits() {
    let mut r = Rtc::new();
    r.tick(SLOW_CLK_DIV * (u64::from(u32::MAX) + 2));
    r.write32(TIME_UPDATE_OFF, 1 << 31);
    assert_eq!(r.read32(TIME_VALUE_LO_OFF), 1, "low word wrapped");
    assert_eq!(r.read32(TIME_VALUE_HI_OFF), 1, "high word carried");
}

#[test]
fn soc_page_dispatch_routes_rtc_cntl_time_regs() {
    // The 0x6000_8000 page: RTC_CNTL time regs are read/write, RTC_IO
    // (0x400) and SENS (0x800) stay out of the RTC path.
    let mut soc = esp32s3_soc::Soc::new();
    soc.write32(0x6000_8010, 0xDEAD);
    assert_eq!(soc.read32(0x6000_8010), 0, "TIME_VALUE is read-only");
    soc.tick_timers(SLOW_CLK_DIV * 4);
    soc.write32(0x6000_800C, 1 << 31);
    assert_eq!(soc.read32(0x6000_8010), 4);
    assert_eq!(soc.read32(0x6000_8014), 0);
    assert_eq!(soc.read32(0x6000_8400), 0, "RTC_IO unmapped");
}
