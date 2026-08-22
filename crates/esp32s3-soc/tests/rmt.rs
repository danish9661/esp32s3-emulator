//! RMT TX model unit tests (item-memory FSM + tx_end interrupt).

use esp32s3_soc::rmt::*;

// chnconf0[0] = div_cnt=2, idle_out_en, idle_out_lv=1  (idle level 1).
const CONF: u32 = (2 << 8) | (1 << 5) | (1 << 6);
const ITEM0: u32 = 100 | (1 << 15) | (50 << 16); // pulse0=1/100t, pulse1=0/50t
const ITEM1: u32 = 100 | (1 << 15) | (7 << 16); // pulse0=1/100t, pulse1=0/7t

fn setup() -> Rmt {
    let mut r = Rmt::new();
    // Channel 0 memory: item0 @ 0x800, item1 @ 0x804.
    r.write32(0x800, ITEM0);
    r.write32(0x804, ITEM1);
    // tx_lim = 2 items.
    r.write32(0xA0, 2);
    // Enable tx_end interrupt for channel 0.
    r.write32(0x78, 1);
    // Configure + start.
    r.write32(0x20, CONF | 1);
    r
}

#[test]
fn tx_drives_gpio_signal_through_items() {
    let mut r = setup();
    // Initial pulse0 of item0: output high (level0=1).
    assert_eq!(r.signal_level(RMT_TX_SIGNAL_BASE), 1);
    r.tick(); // +32 ticks: still in pulse0 (100)
    assert_eq!(r.signal_level(RMT_TX_SIGNAL_BASE), 1);
    for _ in 0..3 {
        r.tick();
    } // +96 -> 128 ticks: pulse0 done at 100, pulse1 (50) 28 in -> low
    assert_eq!(r.signal_level(RMT_TX_SIGNAL_BASE), 0);
    r.tick(); // 160: pulse1 done at 150, item1 pulse0 (100) 10 in -> high
    assert_eq!(r.signal_level(RMT_TX_SIGNAL_BASE), 1);
    for _ in 0..3 {
        r.tick();
    } // 256: item1 pulse0 done at 250, pulse1 (7) 6 in -> low
    assert_eq!(r.signal_level(RMT_TX_SIGNAL_BASE), 0);
    r.tick(); // 288: pulse1 done at 257 -> tx_end
    assert_eq!(r.signal_level(RMT_TX_SIGNAL_BASE), 1); // idle level 1
    assert_ne!(r.int_st(), 0, "tx_end interrupt must be asserted");
}

#[test]
fn tx_end_clears_on_int_clr() {
    let mut r = setup();
    for _ in 0..10 {
        r.tick();
    }
    assert_ne!(r.int_st(), 0);
    r.write32(0x7C, 1); // INT_CLR channel 0
    assert_eq!(r.int_st(), 0, "int_clr must clear tx_end");
}

#[test]
fn signal_outside_tx_range_is_zero() {
    let r = Rmt::new();
    assert_eq!(r.signal_level(RMT_TX_SIGNAL_BASE - 1), 0);
    assert_eq!(r.signal_level(RMT_TX_SIGNAL_BASE + 4), 0);
}
