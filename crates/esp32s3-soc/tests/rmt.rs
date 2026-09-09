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

// chmconf1[0] @ 0x34 (rx_en = bit 0), chmconf0[0] @ 0x30 (idle_thres [22:8]).
const CHMCONF1_0: u32 = 0x34;
const CHMCONF0_0: u32 = 0x30;
const RX_MEM0: u32 = 0xC00; // RMTMEM block of HW channel 4
const RX_END_BIT: u32 = 1 << 16;
const RX_EN: u32 = 1;

use std::cell::Cell;

/// Scripted pad level: `levels[n]` for sample n (then holds the last).
fn scripted(levels: &[u32]) -> impl Fn(u32) -> u32 + '_ {
    let n = Cell::new(0);
    move |_| {
        let i = n.get().min(levels.len() - 1);
        n.set(n.get() + 1);
        levels[i]
    }
}

#[test]
fn rx_captures_edges_and_raises_rx_end() {
    let mut r = Rmt::new();
    // Idle timeout after 256 channel ticks (~8 samples).
    r.write32(CHMCONF0_0, 256 << 8);
    r.write32(CHMCONF1_0, RX_EN);
    // low x2, high x6, low x18 with idle timeout 256: pulses 64 / 160 /
    // 256 (truncated by the timeout). Quantum is 32/step: the baseline
    // sample counts, so each run measures (samples × 32).
    let input = scripted(&[0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    for _ in 0..18 {
        r.tick_rx(&input);
    }
    // Item 0: low 64, high 160. Item 1: low 256 (idle-timeout truncation).
    let w0 = r.read32(RX_MEM0);
    assert_eq!(w0 & 0x7FFF, 64, "first pulse width");
    assert_eq!((w0 >> 15) & 1, 0, "first pulse low");
    assert_eq!((w0 >> 16) & 0x7FFF, 160, "second pulse width");
    assert_eq!((w0 >> 31) & 1, 1, "second pulse high");
    let w1 = r.read32(RX_MEM0 + 4);
    assert_eq!(w1 & 0x7FFF, 256, "third pulse width");
    assert_eq!((w1 >> 15) & 1, 0, "third pulse low");
    assert_ne!(r.read32(0x70) & RX_END_BIT, 0, "rx_end latched");
    assert_eq!(r.int_st() & RX_END_BIT, 0, "masked off without ena");
    r.write32(0x78, RX_END_BIT); // INT_ENA bit 16
    assert_ne!(r.int_st() & RX_END_BIT, 0, "rx_end now visible");
    r.write32(0x7C, RX_END_BIT); // INT_CLR
    assert_eq!(r.read32(0x70) & RX_END_BIT, 0, "rx_end clears");
}

#[test]
fn rx_idle_line_ends_capture_immediately() {
    let mut r = Rmt::new();
    r.write32(CHMCONF0_0, 64 << 8);
    r.write32(CHMCONF1_0, RX_EN);
    let input = scripted(&[1]);
    for _ in 0..4 {
        r.tick_rx(&input);
    }
    assert_ne!(r.read32(0x70) & RX_END_BIT, 0, "idle timeout ends capture");
    assert!(!r.rx_pending(), "channel disarms after end");
    // The open pulse is flushed like the hardware writer offset advancing
    // past a partial item: 3 samples x 32 at HIGH.
    assert_eq!(r.read32(RX_MEM0), 96 | (1 << 15), "partial pulse flushed");
}

#[test]
fn rx_filter_absorbs_short_glitch() {
    let mut r = Rmt::new();
    // Idle timeout far beyond the script (the capture must survive to the
    // real edge); only the glitch filter shapes this run.
    r.write32(CHMCONF0_0, 2048 << 8);
    // rx_en + filter_en + threshold 64 channel ticks.
    r.write32(CHMCONF1_0, RX_EN | (1 << 4) | (64 << 5));
    // HIGH x7, LOW x1 (glitch, 32 < 64), HIGH x7, then LOW x7 (real edge).
    let input = scripted(&[
        1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    ]);
    for _ in 0..22 {
        r.tick_rx(&input);
    }
    // The glitch must not split the HIGH pulse: the first completed item
    // is HIGH (long) with an empty second half, not a short LOW item.
    let w0 = r.read32(RX_MEM0);
    assert_eq!((w0 >> 15) & 1, 1, "first pulse is the long HIGH");
    assert!((w0 & 0x7FFF) > 200, "glitch absorbed into HIGH width");
    assert_eq!((w0 >> 16) & 0x7FFF, 0, "no short LOW half written");
}
