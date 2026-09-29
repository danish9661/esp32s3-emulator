//! Ethernet bridge tap unit tests: TX capture (board→host) + RX injection
//! (host→board) FIFO semantics on the `Soc` (see `net_capture_tx` /
//! `net_take_tx` / `net_inject_rx` / `net_take_rx` / `net_rx_stage`).
//!
//! Ground truth is the tap contract itself (machine.rs hooks + run_flash
//! NET_PCAP/NET_GW legs): TX captures firmware bytes at the
//! `esp_netif_transmit` entry; RX stages gateway reply bytes for the
//! `esp_netif_receive` entry. These tests pin the queue mechanics without
//! booting firmware (the live path is proven by the `test_worker_net`
//! battery entry: 2 frames to pcap + gateway feed).

use esp32s3_soc::soc::{EVT_NET_FRAME, Soc};
use xtensa_core::Bus;

/// TX path: capture stages bytes, take drains them, event carries kind 6 /
/// direction 0 / length.
#[test]
fn net_tx_capture_take_round_trip() {
    let mut s = Soc::new();
    // A 14-byte Ethernet header (dst/src/type) is enough to exercise the
    // copy path (lengths are not validated — the tap is L2).
    let frame: Vec<u8> = (0..42u8).collect();
    // Stage the bytes at a scratch DRAM address first (the hook reads
    // them off the bus via read8).
    let base = 0x3FC8_1000u32;
    for (k, b) in frame.iter().enumerate() {
        s.write8(base + k as u32, *b as u32);
    }
    s.net_capture_tx(base, frame.len() as u32);
    let evts = s.drain_events();
    let net: Vec<_> = evts.iter().filter(|e| e.kind == EVT_NET_FRAME).collect();
    assert_eq!(net.len(), 1, "one NET event per capture");
    assert_eq!(net[0].a, 0, "direction 0 = board→host TX");
    assert_eq!(net[0].b, 42, "length carried");
    let got = s.net_take_tx();
    assert_eq!(got, frame, "captured bytes round-trip");
    assert!(s.net_take_tx().is_empty(), "drain clears");
}

/// TX length caps at 1600 (1500-MTU + headroom class).
#[test]
fn net_tx_capture_caps_at_1600() {
    let mut s = Soc::new();
    let base = 0x3FC8_1000u32;
    s.net_capture_tx(base, 4000);
    let got = s.net_take_tx();
    assert_eq!(got.len(), 1600, "capped");
}

/// RX path: inject queues FIFO, take pops in order, event carries
/// direction 1; empty injects are ignored; the 9th frame drops with a
/// counter bump.
#[test]
fn net_rx_inject_take_fifo_and_bound() {
    let mut s = Soc::new();
    assert!(s.net_take_rx().is_none(), "empty FIFO");
    s.net_inject_rx(&[]);
    assert!(s.net_take_rx().is_none(), "empty frame ignored");
    s.net_inject_rx(&[0xAA, 0xBB]);
    s.net_inject_rx(&[0xCC]);
    let evts = s.drain_events();
    let rx: Vec<_> = evts
        .iter()
        .filter(|e| e.kind == EVT_NET_FRAME && e.a == 1)
        .collect();
    assert_eq!(rx.len(), 2, "one RX event per inject");
    assert_eq!(rx[0].b, 2);
    assert_eq!(rx[1].b, 1);
    assert_eq!(s.net_take_rx().unwrap(), vec![0xAA, 0xBB], "FIFO order");
    assert_eq!(s.net_take_rx().unwrap(), vec![0xCC]);
    assert!(s.net_take_rx().is_none(), "drained");
    // Bound: 8 queued, 9th drops.
    for k in 0..8 {
        s.net_inject_rx(&[k]);
    }
    assert_eq!(s.net_rx_dropped(), 0);
    s.net_inject_rx(&[0xFF]);
    assert_eq!(s.net_rx_dropped(), 1, "9th frame drops with counter");
    // The 8 queued frames are intact (drop didn't clobber).
    for k in 0..8 {
        assert_eq!(s.net_take_rx().unwrap(), vec![k]);
    }
}

/// RX stage: bytes land in WIFI_SCRATCH past the event-payload cursor,
/// last-buf/len accessors track them, empty stages return 0.
#[test]
fn net_rx_stage_scratch_and_accessors() {
    let mut s = Soc::new();
    assert_eq!(s.net_rx_stage(&[]), 0, "empty stage is a no-op");
    let frame = vec![0xDE, 0xAD, 0xBE, 0xEF];
    let buf = s.net_rx_stage(&frame);
    assert_ne!(buf, 0);
    assert_eq!(s.net_rx_last_buf(), buf);
    assert_eq!(s.net_rx_last_len(), 4);
    for (k, b) in frame.iter().enumerate() {
        assert_eq!(s.read8(buf + k as u32) as u8, *b, "byte {k}");
    }
    // Past the event-payload bump cursor (0x858) + ESP-NOW window.
    assert!(buf >= Soc::WIFI_SCRATCH + 0xC00, "past payload cursor");
}
