//! Full-802.11-LL-MAC slice-1 unit tests: raw TX capture
//! (`esp_wifi_80211_tx` tap) + virtual-AP beacon staging (promiscuous
//! sniffer path) on the `Soc` (see `llmac_capture_tx` / `llmac_take_tx` /
//! `llmac_stage_beacon`).
//!
//! Ground truth is the tap contract itself (machine.rs LLMAC arms +
//! run_flash `WIFI_LLMAC` leg): TX captures the firmware's raw 802.11
//! frame at the callee entry; RX stages a `wifi_promiscuous_pkt_t`
//! (48-byte S3 rx_ctrl + beacon payload) the host runs the registered
//! callback over. These tests pin the queue/staging mechanics without
//! booting firmware (the live path is proven by the `llmac` battery
//! entry: probe-request capture + 3 beacons through the real sniffer).

use esp32s3_soc::Soc;
use xtensa_core::Bus;

/// TX path: capture stages bytes, take drains them, bounded at 8.
#[test]
fn llmac_tx_capture_take_round_trip() {
    let mut s = Soc::new();
    s.llmac_arm();
    let frame: Vec<u8> = (0..41u8).collect();
    let base = 0x3FC8_1000u32;
    for (k, b) in frame.iter().enumerate() {
        s.write8(base + k as u32, *b as u32);
    }
    s.llmac_capture_tx(base, frame.len() as u32);
    assert_eq!(s.llmac_tx_pending(), 1, "one frame staged");
    let got = s.llmac_take_tx().expect("frame present");
    assert_eq!(got, frame, "captured bytes round-trip");
    assert_eq!(s.llmac_tx_pending(), 0, "drain clears");
    assert!(s.llmac_take_tx().is_none(), "empty take is None");
}

/// Beacon staging: 48-byte rx_ctrl (channel 6, sig_len 56) + beacon with
/// SSID "EmuAP" at the sketch-asserted offsets; beacons decrement.
#[test]
fn llmac_beacon_stages_rx_ctrl_and_ssid() {
    let mut s = Soc::new();
    s.llmac_arm();
    assert!(s.llmac_stage_beacon().is_none(), "unarmed air is quiet");
    s.llmac_arm_beacons(2);
    let (buf, ty) = s.llmac_stage_beacon().expect("first beacon");
    assert_eq!(ty, 0, "type WIFI_PKT_MGMT");
    assert_eq!(s.llmac_beacons_left(), 1, "one consumed");
    // rx_ctrl: channel 6 at byte 10 low nibble, sig_len 56 at bytes 44-45.
    assert_eq!(s.read8(buf + 10) & 0xF, 6, "channel 6");
    let sig_len = s.read8(buf + 44) | (s.read8(buf + 45) << 8);
    assert_eq!(sig_len & 0xFFF, 56, "sig_len = beacon length");
    // Beacon: FC 0x80, broadcast DA, BSSID, SSID IE at +36.
    assert_eq!(s.read8(buf + 48), 0x80, "beacon frame control");
    for (k, b) in [0xFFu8; 6].iter().enumerate() {
        assert_eq!(s.read8(buf + 48 + 4 + k as u32), *b as u32, "broadcast DA");
    }
    for (k, b) in [0x02u8, 0x11, 0x22, 0x33, 0x44, 0x55].iter().enumerate() {
        assert_eq!(s.read8(buf + 48 + 10 + k as u32), *b as u32, "BSSID SA");
    }
    assert_eq!(s.read8(buf + 48 + 36), 0, "SSID IE id");
    assert_eq!(s.read8(buf + 48 + 37), 5, "SSID len");
    let ssid: Vec<u8> = (0..5).map(|k| s.read8(buf + 48 + 38 + k) as u8).collect();
    assert_eq!(ssid, b"EmuAP", "SSID bytes");
    assert_eq!(s.read8(buf + 48 + 53), 3, "DS IE id");
    assert_eq!(s.read8(buf + 48 + 55), 6, "DS channel 6");
    // Second beacon consumes the last arming; third finds quiet air.
    assert!(s.llmac_stage_beacon().is_some(), "second beacon");
    assert_eq!(s.llmac_beacons_left(), 0, "arming spent");
    assert!(s.llmac_stage_beacon().is_none(), "air quiet after");
}

/// Promiscuous-callback cell: 0 until registered, then the pointer.
#[test]
fn llmac_promisc_cb_cell_tracks_registration() {
    let mut s = Soc::new();
    assert_eq!(s.llmac_promisc_cb(), 0, "unregistered");
    s.llmac_set_promisc_cb(0x4200_1234);
    assert_eq!(s.llmac_promisc_cb(), 0x4200_1234, "registered pointer");
}

/// TX FIFO bound: the queue holds at most 8 frames — the 9th capture
/// evicts the oldest (same drop-oldest discipline as every other RX
/// path here), so an un-drained host never grows memory and the newest
/// verdict-relevant frame is always present.
#[test]
fn llmac_tx_fifo_bound_drops_oldest() {
    let mut s = Soc::new();
    s.llmac_arm();
    let base = 0x3FC8_1000u32;
    for i in 0..10u8 {
        s.write8(base, i as u32);
        s.llmac_capture_tx(base, 1);
    }
    assert_eq!(s.llmac_tx_pending(), 8, "bounded at 8");
    // Oldest two (0, 1) evicted; oldest remaining is 2, newest is 9.
    let first = s.llmac_take_tx().expect("frame present");
    assert_eq!(first, vec![2u8], "oldest evicted first");
    for _ in 0..6 {
        s.llmac_take_tx().expect("frame present");
    }
    let last = s.llmac_take_tx().expect("frame present");
    assert_eq!(last, vec![9u8], "newest retained");
    assert_eq!(s.llmac_tx_pending(), 0, "drain clears");
}
