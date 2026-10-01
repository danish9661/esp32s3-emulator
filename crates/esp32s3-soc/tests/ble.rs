//! BLE VHCI tap unit tests: TX capture (firmware→controller) + RX
//! injection (controller→firmware) FIFO semantics on the `Soc` (see
//! `bt_hci_capture_tx` / `bt_hci_take_tx` / `bt_hci_inject_rx` /
//! `bt_hci_take_rx`).
//!
//! Ground truth is the tap contract itself (machine.rs VHCI hooks +
//! run_flash BLE_GW leg): TX captures firmware HCI bytes at the
//! `esp_vhci_host_send_packet` entry; RX stages Bumble bridge replies
//! for the firmware VHCI-recv path. These tests pin the queue mechanics
//! without booting firmware (the live path is proven by the
//! `esp32s3_ble` battery entry through the Bumble bridge).

use esp32s3_soc::soc::{EVT_BLE_HCI, Soc};
use xtensa_core::Bus;

/// TX path: capture stages bytes, take drains them, event carries kind 7 /
/// direction 0 / length.
#[test]
fn ble_tx_capture_take_round_trip() {
    let mut s = Soc::new();
    // HCI Reset command (H4 type 0x01 + opcode 0x0C03, no params).
    let frame: Vec<u8> = vec![0x01, 0x03, 0x0C, 0x00];
    let base = 0x3FC8_1000u32;
    for (k, b) in frame.iter().enumerate() {
        s.write8(base + k as u32, *b as u32);
    }
    s.bt_hci_capture_tx(base, frame.len() as u32);
    let evts = s.drain_events();
    let ble: Vec<_> = evts.iter().filter(|e| e.kind == EVT_BLE_HCI).collect();
    assert_eq!(ble.len(), 1, "one BLE event per capture");
    assert_eq!(ble[0].a, 0, "direction 0 = host→controller TX");
    assert_eq!(ble[0].b, 4, "length carried");
    let got = s.bt_hci_take_tx();
    assert_eq!(got, frame, "captured bytes round-trip");
    assert!(s.bt_hci_take_tx().is_empty(), "drain clears");
}

/// TX length caps at 4096 (longest single HCI packet class).
#[test]
fn ble_tx_capture_caps_at_4096() {
    let mut s = Soc::new();
    let base = 0x3FC8_1000u32;
    s.bt_hci_capture_tx(base, 9000);
    let got = s.bt_hci_take_tx();
    assert_eq!(got.len(), 4096, "capped");
}

/// RX path: inject queues FIFO, take pops in order, event carries
/// direction 1; empty injects are ignored; the 9th packet drops with a
/// counter bump.
#[test]
fn ble_rx_inject_take_fifo_and_bound() {
    let mut s = Soc::new();
    assert!(s.bt_hci_take_rx().is_none(), "empty FIFO");
    s.bt_hci_inject_rx(&[]);
    assert!(s.bt_hci_take_rx().is_none(), "empty packet ignored");
    // Command Complete for HCI Reset (H4 EVT + opcode echo + status 0).
    s.bt_hci_inject_rx(&[0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00]);
    s.bt_hci_inject_rx(&[0x04, 0x05]);
    let evts = s.drain_events();
    let rx: Vec<_> = evts
        .iter()
        .filter(|e| e.kind == EVT_BLE_HCI && e.a == 1)
        .collect();
    assert_eq!(rx.len(), 2, "one RX event per inject");
    assert_eq!(rx[0].b, 7);
    assert_eq!(rx[1].b, 2);
    assert_eq!(
        s.bt_hci_take_rx().unwrap(),
        vec![0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00],
        "FIFO order"
    );
    assert_eq!(s.bt_hci_take_rx().unwrap(), vec![0x04, 0x05]);
    assert!(s.bt_hci_take_rx().is_none(), "drained");
    // Bound: 8 queued, 9th drops.
    for k in 0..8 {
        s.bt_hci_inject_rx(&[k]);
    }
    assert_eq!(s.bt_hci_rx_dropped(), 0);
    s.bt_hci_inject_rx(&[0xFF]);
    assert_eq!(s.bt_hci_rx_dropped(), 1, "9th packet drops with counter");
    // The 8 queued packets are intact (drop didn't clobber).
    for k in 0..8 {
        assert_eq!(s.bt_hci_take_rx().unwrap(), vec![k]);
    }
}

/// The VHCI tap registers survive closed-driver BB writes to the shared
/// BT page: `bt_bb_v2_rx_set` RMWs cell 0x008 — the same 12-bit offset as
/// the tap's INT_ENA slot (see the soc.rs `BT_BASE` arm) — so a shared
/// state lets the BB init's RMW clobber the tap's INT_ENA (proven live:
/// shared-state ENA read 0xFFFF0001 after the BB loop, wedging the RWBLE
/// line). The tap therefore owns 0x00/0x04/0x08/0x0C via the split
/// `write32_vhci`/`read32_vhci` entry points; the BB register file lives
/// in the same `regs` array at every other offset.
#[test]
fn ble_vhci_int_regs_survive_bb_rmw() {
    use esp32s3_soc::ble::BT_BASE;
    // Emulator tap window: BT-page offsets 0x00/0x04/0x08/0x0C (see the
    // soc.rs `BT_BASE` arm + ble.rs `write32_vhci`/`read32_vhci`).
    const VHCI_RAW: u32 = 0x00;
    const VHCI_ST: u32 = 0x04;
    const VHCI_ENA: u32 = 0x08;
    const INT_RX: u32 = 1;
    let mut s = Soc::new();
    // Arm the tap the way the VHCI driver does when it registers
    // `notify_host_recv` (INT_ENA = RX).
    s.write32(BT_BASE + VHCI_ENA, INT_RX);
    assert_eq!(
        s.read32(BT_BASE + VHCI_ENA),
        INT_RX,
        "tap ENA reads back before any BB traffic"
    );
    // Closed-driver BB init RMWs cell 0x008 (UART1+0x1008 — the UART1
    // +0x100 alias of THIS page's INT_ENA slot), plus the rest of the
    // BB window 0x008..0x0E0.
    for off in (0x008..0x0E8).step_by(4) {
        let v = s.read32(BT_BASE + off as u32);
        s.write32(BT_BASE + off as u32, v | 0xFFFF_0000);
    }
    // Tap state is intact: RAW/ST/ENA still read back, and a staged
    // controller reply still raises the line.
    assert_eq!(s.read32(BT_BASE + VHCI_ENA), INT_RX);
    assert_eq!(s.read32(BT_BASE + VHCI_RAW), 0);
    s.bt_hci_inject_rx(&[0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00]);
    assert_ne!(s.read32(BT_BASE + VHCI_ST) & INT_RX, 0);
}
