//! ESP32-S3 BLE VHCI tap: host <-> Bumble virtual-controller bridge.
//!
//! Base `DR_REG_BT_BASE = 0x6001_1000` (soc/reg_base.h; the RWBT/RWBLE
//! register file the closed `libbt.a` VHCI driver pokes). The ESP-IDF
//! BLE path is NimBLE host (`esp_nimble_hci_init`) over VHCI:
//!   * host→controller: `esp_vhci_host_send_packet(data, len)` — an H4
//!     HCI frame (type 0x01 CMD / 0x02 ACL + payload), sent iff
//!     `esp_vhci_host_check_send_available()` is true;
//!   * controller→host: the registered `notify_host_recv(data, len)`
//!     VHCI callback — an H4 HCI frame (type 0x04 EVT / 0x02 ACL);
//!   * `notify_host_send_available` wakes the blocked `vhci_send_sem`
//!     taker (proven in `esp_nimble_hci.c`: `ble_hci_trans_hs_acl_tx`
//!     gates every send on the semaphore + the check).
//!
//! Silicon moves these through shared memory + the RWBLE interrupt
//! (`ETS_RWBLE_INTR_SOURCE` = 8, `interrupts.h` recount); the emulator
//! moves them through the host instead, like the Ethernet tap
//! (`net_capture_tx`/`net_inject_rx` in soc.rs):
//!   * firmware VHCI sends are captured into `pending_tx` + an
//!     `EVT_BLE_HCI` event (kind 7, `a` = 0 = host→controller, `b` = len);
//!     the host (`run_flash` BLE_GW leg / browser bridge) drains via
//!     `bt_hci_take_tx` and forwards length-prefixed to `tools/ble_bridge.py`
//!     (Bumble virtual controller + GATT app, no radio, no root);
//!   * bridge replies are staged via `bt_hci_inject_rx` (FIFO, bounded 8)
//!     + an `EVT_BLE_HCI` event (`a` = 1);
//!
//!     the firmware's VHCI-recv path pops one frame per poll via
//!     `bt_hci_take_rx`.
//!
//! The BT register page itself is a plain store (the closed driver RMWs
//! config bits there; no proven poll needs a done-bit — unlike the WiFi
//! FE +0x174 case, so no overlay is added: every spin must be proven
//! from objdump + a live poll-load-address log first).
//!
//! Interrupt: `ETS_RWBLE_INTR_SOURCE` = 8, level, raised while the RX
//! FIFO is non-empty and masked by `int_ena` (driver enables it when it
//! arms `notify_host_recv`). Same INT_RAW/ST/ENA/CLR discipline as the
//! UART model (RAW&ENA = ST).
//!
//! Validated by unit tests + the `esp32s3_ble` NimBLE sketch (advertise →
//! GATT read/write/notify through the live Bumble bridge).

use alloc::collections::VecDeque;
use alloc::vec::Vec;

/// BT VHCI register page (`DR_REG_BT_BASE`, soc/reg_base.h).
pub const BT_BASE: u32 = 0x6001_1000;
/// RWBLE interrupt source (`interrupts.h` recount: WIFI_MAC=0, NMI=1,
/// WIFI_PWR=2, WIFI_BB=3, BT_MAC=4, BT_BB=5, BT_BB_NMI=6, RWBT=7,
/// RWBLE=8).
pub const BLE_INTR_SOURCE: u32 = 8;

// INT bits (UART-model discipline: RAW latched, ST = RAW & ENA).
const INT_RX: u32 = 1 << 0;
const INT_TX_DONE: u32 = 1 << 1;

// INT offsets WITHIN the VHCI alias window (the soc.rs UART1 arm
// strips the 0x100 base before calling `write32_vhci`/`read32_vhci`,
// so these are window-relative — same layout as the BT page INT block,
// different address).
const INT_RAW_OFF: u32 = 0x00;
const INT_ST_OFF: u32 = 0x04;
const INT_ENA_OFF: u32 = 0x08;
const INT_CLR_OFF: u32 = 0x0C;

/// BLE VHCI tap: register page + host-bridged HCI FIFOs.
pub struct Ble {
    /// Plain BT-page register store (config RMWs land here).
    regs: [u32; 0x1000 / 4],
    /// Benign store for the modem-EM alias window (UART2 +0x3000 =
    /// 0x60031000, see the soc.rs 0x6003_1000 arm): closed `libbt.a`
    /// pokes these cells during init (objdump labels them
    /// `UART2+0x3xxx`, but silicon has no page there — reg_base.h lists
    /// no DR_REG_* at 0x60031000). No engine runs behind them — the air
    /// interface is bridged through the VHCI FIFOs below instead — so
    /// RMW round-trip is the whole model. Seeded with the exchange-memory
    /// base the closed `r_lld_core_init` expects at cell 0x004
    /// (`0x60031004` — proven live: the BLE image dies in
    /// `BLE assert lld.c:324` when it reads 0; with the seed it proceeds
    /// past init into VHCI traffic).
    bt_mac: [u32; 0x1000 / 4],
    /// Captured host→controller HCI frame (drained by `bt_hci_take_tx`).
    pending_tx: Vec<u8>,
    /// Staged controller→host HCI frames (bounded FIFO, like net RX).
    pending_rx: VecDeque<Vec<u8>>,
    /// Dropped-RX counter (FIFO full while staging).
    rx_dropped: u32,
    int_raw: u32,
    int_ena: u32,
}

impl Default for Ble {
    fn default() -> Self {
        Self::new()
    }
}

impl Ble {
    pub fn new() -> Self {
        // Seed cell 0x004 with the exchange-memory base the closed
        // `r_lld_core_init` compares at `lld.c:324` (`beq a4,a11` with
        // a11 = 0x9001b00 from rodata — proven live: reads 0 without the
        // seed and the BLE image dies in the assert). `regs`/`bt_mac`
        // start zeroed; the seed is the only nonzero reset value (same
        // "seeded reset default" class as SYSTEM BT_LPCK DIV 255).
        let mut bt_mac = [0; 0x1000 / 4];
        bt_mac[1] = 0x0900_1b00;
        Self {
            regs: [0; 0x1000 / 4],
            bt_mac,
            pending_tx: Vec::new(),
            pending_rx: VecDeque::new(),
            rx_dropped: 0,
            int_raw: 0,
            int_ena: 0,
        }
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        let idx = (off / 4) as usize;
        match off {
            INT_RAW_OFF => self.int_raw |= value,
            INT_ENA_OFF => self.int_ena = value,
            INT_CLR_OFF => self.int_raw &= !value,
            _ => {
                if idx < self.regs.len() {
                    self.regs[idx] = value;
                }
            }
        }
    }

    pub fn read32(&mut self, off: u32) -> u32 {
        match off {
            INT_RAW_OFF => self.int_raw,
            INT_ST_OFF => self.int_raw & self.int_ena,
            INT_ENA_OFF => self.int_ena,
            // CLR reads back 0 (write-1-to-clear, like the UART model).
            INT_CLR_OFF => 0,
            _ => {
                let idx = (off / 4) as usize;
                if idx < self.regs.len() {
                    self.regs[idx]
                } else {
                    0
                }
            }
        }
    }

    /// VHCI tap register access on the BT page (see the soc.rs
    /// `BT_BASE` arm): the emulator owns offsets 0x00/0x04/0x08/0x0C
    /// (INT_RAW/ST/ENA/CLR driving the RWBLE line); the closed driver's
    /// own MAC/BB registers own every other offset (benign store in
    /// `regs`). Split ownership is REQUIRED, not hygiene: the closed BT
    /// BB init (`bt_bb_v2_rx_set` etc.) RMWs cell 0x008 — the same
    /// 12-bit offset as THIS page's INT_ENA slot — so a shared state lets
    /// the BB init's RMW clobber the tap's INT_ENA (proven live:
    /// shared-state ENA read 0xFFFF0001 after the BB loop, wedging the
    /// RWBLE line).
    ///
    /// NOTE: `write32`/`read32` are the OLD shared-state entry points
    /// (kept for the unit tests below); the live soc.rs arm calls the
    /// `_vhci` variants.
    pub fn write32_vhci(&mut self, off: u32, value: u32) {
        // BB-window guard: the closed BT BB init RMWs cell 0x008 (the
        // same 12-bit offset as INT_ENA) as part of its OWN register
        // file — never as the tap. A write whose upper 16 carry BB
        // residue (proven live: 0xFFFF0001 after the BB loop's
        // `v | 0xFFFF0000` RMW) is BB traffic, not a tap write — drop
        // the residue, keep the tap bit (tap writes are small
        // INT bitmasks; only bit 0 is ever armed).
        if off == INT_ENA_OFF && value & 0xFFFF_0000 != 0 {
            self.int_ena = value & 0x1;
            return;
        }
        match off {
            INT_RAW_OFF => self.int_raw |= value,
            INT_ENA_OFF => self.int_ena = value,
            INT_CLR_OFF => self.int_raw &= !value,
            _ => {
                let idx = (off / 4) as usize;
                if idx < self.regs.len() {
                    self.regs[idx] = value;
                }
            }
        }
    }

    pub fn read32_vhci(&mut self, off: u32) -> u32 {
        match off {
            INT_RAW_OFF => self.int_raw,
            INT_ST_OFF => self.int_raw & self.int_ena,
            INT_ENA_OFF => self.int_ena,
            INT_CLR_OFF => 0,
            _ => {
                let idx = (off / 4) as usize;
                if idx < self.regs.len() {
                    self.regs[idx]
                } else {
                    0
                }
            }
        }
    }

    /// Done-interrupt status (`RAW & ENA`) for matrix source 8.
    pub fn int_st(&self) -> u32 {
        self.int_raw & self.int_ena
    }

    /// Modem time-sync busy-bit overlays for the 0x60031000 alias window
    /// (see the soc.rs 0x6003_1000 arm):
    /// * Cell 0x00 (`0x60031000`): the closed `r_rwip_driver_init` writes
    ///   bit 31 then spins on `bltz` (`s32i a3,[a5]` / `l32i.n a3,[a5]` +
    ///   `extui a6,a3,31,1` + `bltz a3` — proven live: the BLE image parks
    ///   at 0x420363f9 with a2=0x3fcefbfc forever when the cell reads back
    ///   what was written).
    /// * Cell 0x1C (`0x6003101c`): the real ROM's `r_rwip_time_get` (via
    ///   `0x4002bd50`) writes `0x80000000` then spins on `bltz`
    ///   (`l32i.n a3,[a2]` + `bltz a3` — proven live: the BLE image parks
    ///   at 0x4002bd70 with a2=0x6003101c/a3=0x80000000 forever).
    ///
    /// `bltz` loops while NEGATIVE, so done = bit31 CLEAR (busy-bit,
    /// opposite of the WiFi FE +0x174 / TX-DC done = SET class):
    /// silicon clears the bit once the modem clock is running, and the
    /// emulator's clock is always running, so both cells read back with
    /// bit 31 CLEARED. Plain RMW-able store on write — init RMWs
    /// round-trip, the polls always observe done.
    pub fn write_bt_mac(&mut self, off: u32, value: u32) {
        let idx = (off / 4) as usize;
        if idx < self.bt_mac.len() {
            self.bt_mac[idx] = value;
        }
    }

    pub fn read_bt_mac(&mut self, off: u32) -> u32 {
        // Modem time-sync cells (0x00/0x1C — proven live, see above):
        // busy-bit overlay, INVERTED vs the WiFi FE +0x174 overlay (there
        // done = SET; here done = CLEAR because the poll is `bltz`).
        if off == 0x00 || off == 0x1C {
            let idx = (off / 4) as usize;
            return self.bt_mac[idx] & !(1 << 31);
        }
        let idx = (off / 4) as usize;
        if idx < self.bt_mac.len() {
            self.bt_mac[idx]
        } else {
            0
        }
    }

    /// Capture one host→controller HCI frame (firmware VHCI send path).
    /// Empty frames are ignored (a zero-length VHCI send is a no-op).
    pub fn capture_tx(&mut self, frame: &[u8]) {
        if frame.is_empty() {
            return;
        }
        self.pending_tx.clear();
        self.pending_tx.extend_from_slice(frame);
        self.int_raw |= INT_TX_DONE;
    }

    /// Drain the captured host→controller HCI frame (host frontend —
    /// forwards length-prefixed to the Bumble bridge).
    pub fn take_tx(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.pending_tx)
    }

    /// Non-draining snapshot of the last captured TX (the ACK-path block
    /// matcher in `Soc::ble_ack_deliver` needs the H4 echo while the
    /// host may also drain it for the bridge — both observe the same
    /// bytes).
    pub fn pending_tx_snapshot(&self) -> Vec<u8> {
        self.pending_tx.clone()
    }

    /// Stage one controller→host HCI frame (bridge reply path). FIFO,
    /// bounded at 8 — excess drops with a counter (same discipline as
    /// `net_inject_rx`). Empty frames are ignored. Raises INT_RX.
    pub fn inject_rx(&mut self, frame: &[u8]) {
        if frame.is_empty() {
            return;
        }
        if self.pending_rx.len() >= 8 {
            self.rx_dropped += 1;
            return;
        }
        let mut v = Vec::with_capacity(frame.len());
        v.extend_from_slice(frame);
        self.pending_rx.push_back(v);
        self.int_raw |= INT_RX;
    }

    /// Pop the oldest staged controller→host frame (firmware VHCI-recv
    /// path). Returns `None` while the FIFO is empty; clears INT_RX when
    /// the FIFO drains (level-style, like the UART RX model).
    pub fn take_rx(&mut self) -> Option<Vec<u8>> {
        let f = self.pending_rx.pop_front()?;
        if self.pending_rx.is_empty() {
            self.int_raw &= !INT_RX;
        }
        Some(f)
    }

    /// Peek at the oldest staged frame: (H4 type byte, total length),
    /// WITHOUT consuming it (host size-gate frontend).
    pub fn peek_rx(&self) -> Option<(u8, usize)> {
        self.pending_rx
            .front()
            .and_then(|f| Some((*f.first()?, f.len())))
    }

    /// Peek at the oldest staged frame with its event sub-code:
    /// (H4 type, second byte if present, total length), WITHOUT consuming
    /// it. Lets the host drop sync Command Complete/Status (0x04 0x0E /
    /// 0x04 0x0F — the ROM loopback owns those) without eating the async
    /// frame behind them.
    pub fn peek_rx_evt(&self) -> Option<(u8, Option<u8>, usize)> {
        let f = self.pending_rx.front()?;
        let h4 = *f.first()?;
        let sub = f.get(1).copied();
        Some((h4, sub, f.len()))
    }

    /// Queued-reply count (host frontend — the run_flash RX drain
    /// attempts `ble_ack_deliver_at` whenever a reply is queued AND the
    /// ack waiter is parked; level-triggered, no polling cost when 0).
    pub fn rx_pending(&self) -> usize {
        self.pending_rx.len()
    }

    /// Dropped-RX counter (frames lost while the RX FIFO was full).
    pub fn rx_dropped(&self) -> u32 {
        self.rx_dropped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_capture_take_round_trip() {
        let mut b = Ble::new();
        // HCI Reset command (H4 type 0x01 + opcode 0x0C03, no params).
        let frame = [0x01u8, 0x03, 0x0C, 0x00];
        b.capture_tx(&frame);
        assert_eq!(b.take_tx(), frame.to_vec());
        assert!(b.take_tx().is_empty(), "drain clears");
        // Empty captures are ignored.
        b.capture_tx(&[]);
        assert!(b.take_tx().is_empty());
    }

    #[test]
    fn rx_inject_take_fifo_and_bound() {
        let mut b = Ble::new();
        assert!(b.take_rx().is_none(), "empty FIFO");
        // INT_RX latches on stage, clears on drain.
        b.int_ena = INT_RX;
        b.inject_rx(&[0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00]);
        assert_eq!(b.int_st(), INT_RX);
        let f = b.take_rx().unwrap();
        assert_eq!(f[0], 0x04, "event packet type");
        assert!(b.take_rx().is_none());
        assert_eq!(b.int_st(), 0, "INT_RX clears on drain");
        // Bound: 8 queued, 9th drops.
        for k in 0..8 {
            b.inject_rx(&[k]);
        }
        assert_eq!(b.rx_dropped(), 0);
        b.inject_rx(&[0xFF]);
        assert_eq!(b.rx_dropped(), 1, "9th frame drops with counter");
        for k in 0..8 {
            assert_eq!(b.take_rx().unwrap(), alloc::vec![k]);
        }
    }

    #[test]
    fn int_regs_round_trip() {
        let mut b = Ble::new();
        // Plain BT-page registers round-trip (config RMWs never panic).
        b.write32(0x100, 0xDEAD_BEEF);
        assert_eq!(b.read32(0x100), 0xDEAD_BEEF);
        // RAW/ENA/ST/CLR discipline.
        b.write32(INT_ENA_OFF, INT_RX | INT_TX_DONE);
        b.capture_tx(&[0x01, 0x03, 0x0C, 0x00]);
        assert_eq!(b.read32(INT_ST_OFF), INT_TX_DONE);
        b.write32(INT_CLR_OFF, INT_TX_DONE);
        assert_eq!(b.read32(INT_ST_OFF), 0);
        assert_eq!(b.read32(INT_CLR_OFF), 0, "CLR reads 0");
    }
}
