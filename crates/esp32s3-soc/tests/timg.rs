//! Timg (ESP32-S3 timer group) unit tests: RTC calibration block, interrupt
//! block at S3 offsets (0x70..0x7C), timer count/load/update semantics.

#![allow(clippy::identity_op)]

use esp32s3_soc::timg::{INT_T0, INT_T1, INT_WDT, Timg};

const T0CONFIG: u32 = 0x00;
const WDT_CONFIG0: u32 = 0x48;
const WDT_CONFIG1: u32 = 0x4C;
const WDT_CONFIG2: u32 = 0x50;
const WDT_FEED: u32 = 0x60;
const WDT_WPROTECT: u32 = 0x64;
const WDT_WKEY: u32 = 0x50D8_3AA1;
const T0LO: u32 = 0x04;
const T0HI: u32 = 0x08;
const T0UPDATE: u32 = 0x0C;
const T0ALARMLO: u32 = 0x10;
const T0LOADLO: u32 = 0x18;
const T0LOADHI: u32 = 0x1C;
const T0LOAD: u32 = 0x20;
const RTCCALICFG: u32 = 0x68;
const RTCCALICFG1: u32 = 0x6C;
const RTCCALICFG2: u32 = 0x80;
const INT_ENA: u32 = 0x70;
const INT_RAW: u32 = 0x74;
const INT_ST: u32 = 0x78;
const INT_CLR: u32 = 0x7C;

const CALI_START: u32 = 1 << 31;
const CALI_RDY: u32 = 1 << 15;

#[test]
fn reset_defaults_match_s3_struct() {
    let mut t = Timg::new();
    // start_cycling=1, clk_sel=1, max=1 (timg_rtccalicfg_reg_t defaults).
    assert_eq!(
        t.read32(RTCCALICFG) & !CALI_RDY,
        (1 << 12) | (1 << 13) | (1 << 16)
    );
    // rdy is clear at reset.
    assert_eq!(t.read32(RTCCALICFG) & CALI_RDY, 0);
    // timeout_rst_cnt=3, timeout_thres=0x1FFFFFF.
    assert_eq!(t.read32(RTCCALICFG2), (3 << 3) | (0x1FFFFFF << 7));
    assert_eq!(t.read32(RTCCALICFG1), 0);
}

#[test]
fn oneoff_calibration_sets_rdy_and_value() {
    let mut t = Timg::new();
    // max = 1024 slow-clock cycles (CONFIG_RTC_CLK_CAL_CYCLES), 8MD256
    // (clk_sel = 1): value = 1024 * 1280 = 1310720.
    let cfg = CALI_START | (1 << 13) | (1024 << 16);
    t.write32(RTCCALICFG, cfg);
    assert_ne!(t.read32(RTCCALICFG) & CALI_RDY, 0, "rdy raised");
    assert_eq!(t.read32(RTCCALICFG1) >> 7, 1310720, "cali value");
    assert_eq!(t.read32(RTCCALICFG1) & 1, 1, "cycling data valid");
    // A new start re-runs the count: max 3, rc_slow (clk_sel 0) -> 3*267.
    let cfg = CALI_START | (0 << 13) | (3 << 16);
    t.write32(RTCCALICFG, cfg);
    assert_eq!(t.read32(RTCCALICFG1) >> 7, 801);
}

#[test]
fn cycling_mode_raises_data_valid_on_config() {
    let mut t = Timg::new();
    // start_cycling (default 1) with max != 0 arms a cycling run: vld set.
    t.write32(RTCCALICFG, (1 << 12) | (1 << 13) | (16 << 16));
    assert_eq!(t.read32(RTCCALICFG1) & 1, 1, "cycling data valid");
    assert_eq!(t.read32(RTCCALICFG1) >> 7, 16 * 1280);
}

#[test]
fn idle_cycling_timeout_fires_after_threshold() {
    let mut t = Timg::new();
    // start_cycling armed from reset, no start written; the calibration
    // timer overflows a 1-cycle threshold -> timeout (IDF rtc_time.c sets
    // a tiny threshold to accelerate the wait-for-previous-cal path).
    t.write32(RTCCALICFG2, (1 << 7) | (3 << 3)); // timeout_thres = 1
    t.tick(2);
    assert_ne!(t.read32(RTCCALICFG2) & 1, 0, "timeout fired");
    // A start write clears the timeout and raises rdy.
    t.write32(RTCCALICFG, CALI_START | (1 << 13) | (1 << 16));
    assert_eq!(t.read32(RTCCALICFG2) & 1, 0, "timeout cleared");
    assert_ne!(t.read32(RTCCALICFG) & CALI_RDY, 0);
}

#[test]
fn interrupt_block_at_s3_offsets() {
    let mut t = Timg::new();
    // Count up from 0 with an alarm at 64: RAW.T0 set, ST = RAW & ENA.
    t.write32(T0CONFIG, 0xE000_0400); // EN|INCREASE|AUTORELOAD|ALARM
    t.write32(T0ALARMLO, 64);
    t.tick(64);
    assert_ne!(t.read32(INT_RAW) & INT_T0, 0, "alarm raw");
    assert_eq!(t.read32(INT_ST) & INT_T0, 0, "masked while disabled");
    t.write32(INT_ENA, INT_T0);
    assert_ne!(t.read32(INT_ST) & INT_T0, 0, "masked status");
    // INT_CLR write clears RAW (level-triggered alarm stays pending).
    t.write32(INT_CLR, INT_T0);
    assert_eq!(t.read32(INT_RAW) & INT_T0, 0, "raw cleared");
    t.write32(INT_CLR, INT_T1);
}

#[test]
fn divider_slows_counter_and_hits_small_alarms() {
    let mut t = Timg::new();
    // EN|INCREASE|ALARM, divider = 80 (multi_irq sketch's prescaler):
    // the counter advances once per 80 ticks and hits alarm=2 exactly
    // (the old code stepped by the divider and skipped past it, so RAW
    // never latched).
    t.write32(T0CONFIG, 0xC000_0000 | (80 << 13) | 0x400);
    t.write32(T0ALARMLO, 2);
    t.tick(79);
    assert_eq!(t.read32(T0LO), 0, "no count before a full divider window");
    t.tick(80);
    assert_eq!(t.read32(T0LO), 1, "one count per 80 ticks");
    assert_eq!(t.read32(INT_RAW) & INT_T0, 0, "no alarm yet");
    t.tick(80);
    assert_eq!(t.read32(T0LO), 2, "alarm value reached");
    assert_ne!(t.read32(INT_RAW) & INT_T0, 0, "alarm raw latches");
}

#[test]
fn timer_load_update_semantics() {
    let mut t = Timg::new();
    t.write32(T0LOADLO, 0xFFFF_FFFB);
    t.write32(T0LOADHI, 0);
    t.write32(T0LOAD, 0);
    t.write32(T0CONFIG, 0x8000_0000); // EN, decrease
    t.tick(1);
    assert_eq!(t.read32(T0LO), 0xFFFF_FFFA);
    // UPDATE latches the hi word (count is low here).
    t.write32(T0UPDATE, 1);
    assert_eq!(t.read32(T0HI), 0);
    // Timer 1 block at +0x24.
    t.write32(0x24, 0xC000_0000); // T1CONFIG EN|INCREASE
    t.tick(3);
    assert_eq!(t.read32(0x28), 3, "T1LO counts");
}

#[test]
fn wdt_disabled_does_not_fire() {
    let mut t = Timg::new();
    // Never enabled: ticking forever must not assert an interrupt or reset.
    t.tick(100_000);
    assert_eq!(
        t.read32(INT_RAW) & INT_WDT,
        0,
        "no WDT interrupt when disabled"
    );
    assert!(!t.consume_reset(), "no WDT reset when disabled");
}

#[test]
fn wdt_feed_resets_counter() {
    let mut t = Timg::new();
    t.write32(WDT_WPROTECT, WDT_WKEY);
    t.write32(WDT_CONFIG1, 1 << 16); // prescale = 1
    t.write32(WDT_CONFIG2, 10); // stage0 hold = 10
    // enable + stage0 action = interrupt (bits [30:29] = 0b01)
    t.write32(WDT_CONFIG0, (1 << 31) | (1 << 29));
    t.tick(9);
    assert_eq!(t.read32(INT_RAW) & INT_WDT, 0, "no fire before threshold");
    t.write32(WDT_FEED, 0xABAD_1DEA); // feed -> counter reset
    t.tick(9);
    assert_eq!(t.read32(INT_RAW) & INT_WDT, 0, "no fire after feed");
    t.tick(1); // reach threshold 10
    assert_ne!(t.read32(INT_RAW) & INT_WDT, 0, "interrupt on timeout");
    // INT_CLR clears the level interrupt.
    t.write32(INT_CLR, INT_WDT);
    assert_eq!(t.read32(INT_RAW) & INT_WDT, 0, "raw cleared");
}

#[test]
fn wdt_reset_action_requests_reset() {
    let mut t = Timg::new();
    t.write32(WDT_WPROTECT, WDT_WKEY);
    t.write32(WDT_CONFIG2, 5);
    // enable + stage0 action = reset (bits [30:29] = 0b11 = 3)
    t.write32(WDT_CONFIG0, (1 << 31) | (3 << 29));
    t.tick(5); // reach threshold
    assert!(t.consume_reset(), "reset requested on timeout");
    assert!(!t.consume_reset(), "reset latched once until cleared");
}

#[test]
fn wdt_write_protect_blocks_config() {
    let mut t = Timg::new();
    // Wrong key blocks config writes.
    t.write32(WDT_WPROTECT, 0);
    t.write32(WDT_CONFIG1, 1 << 16);
    t.write32(WDT_CONFIG2, 5);
    t.write32(WDT_CONFIG0, (1 << 31) | (1 << 29)); // ignored
    t.tick(10_000);
    assert_eq!(
        t.read32(INT_RAW) & INT_WDT,
        0,
        "config ignored under write-protect"
    );
    assert!(!t.consume_reset());
    // Re-enable the key, then write config -> now honored.
    t.write32(WDT_WPROTECT, WDT_WKEY);
    t.write32(WDT_CONFIG2, 5);
    t.write32(WDT_CONFIG0, (1 << 31) | (1 << 29));
    t.tick(5);
    assert_ne!(
        t.read32(INT_RAW) & INT_WDT,
        0,
        "config honored once key set"
    );
}
