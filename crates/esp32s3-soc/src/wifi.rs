//! WiFi radio blocks (FE/FE2/BB/NRX) — WIFI BRING-UP scaffold.
//!
//! The ESP32-S3 WiFi PHY is a closed ROM/driver blob: there are no public
//! per-register semantics for the RF front-end (FE @ `0x6000_6000`, FE2 @
//! `0x6000_5000`), baseband (BB @ `0x6001_D000`) or NRX (`0x6001_CC00`)
//! blocks (no `*_reg.h`/`*_struct.h` shipped in arduino-cli; only the
//! power-up/down bits in `fe_reg.h`/`bb_reg.h`/`nrx_reg.h` are public).
//! What IS observable is the driver's register-level behavior, verified by
//! objdump against the linked sketch ELF + live single-step forensics
//! (2026-09-19, corrected below):
//!
//! * PHY RF-cal (`ram_iq_est_enable`, wifi-scan ELF): RMWs FE `+0x140`,
//!   `+0x144`, then polls FE `+0x174` with masks 0x10000/0x1000000
//!   (bits 16+24 — unconditional overlay, the driver never writes the
//!   poll register itself).
//! * TX-DC cal (`txdc_cal_v70`): RMWs SENS2 `0x6000E04C`, polls bit 24
//!   (one-shot latch — see `txdc_read`; any TIMED arm stalls under
//!   `step_fast`, proven live).
//! * IQ-estimate inner poll (`ram_iq_est_enable` at 0x4208012b/2d):
//!   `l32i.n a3,[a10=FE+0x174]` then `bnone a3,a9(=0x10000)` — spins
//!   while bit 16 is CLEAR. The 0x6000E04C/bit-17 reading in earlier
//!   notes was WRONG (a10 holds FE+0x174 = 0x60006174, proven live by
//!   logging the poll-load address; the FE +0x174 bits-16+24 overlay
//!   already covers it, so NO separate bit-17 arm exists).
//!
//! Model: plain register stores (round-trip, like `regstore.rs`), PLUS
//! the proven done-bits. When the next spin is found, add its done-bit
//! here with objdump evidence + a live poll-load-address log — never
//! trust a static register guess (the bit-17 episode proves why).

use crate::regstore::RegStore;

/// MAC control ready bit (see module docs): `hal_init` polls bit 0 of
/// `+0xD14` via `bbci` (branch-while-clear) after setting bit 1 with an
/// `or`+store. Report bit 0 set once armed (any write to the register —
/// the driver always writes first); before the arm read back the stored
/// value (reset 0). Proven live 2026-09-19: first nonzero write is 0x3
/// (bit 1 set by the driver as described), and the stored+overlay read
/// is 0x3 (bit 0 set by this overlay) — the poll exits.
const MAC_CTRL_READY_OFF: u32 = 0xD14;
const MAC_CTRL_READY_BIT: u32 = 1 << 0;

/// WiFi radio block: six 4 KB pages of plain stores + proven done-bits.
pub struct Wifi {
    fe: RegStore,
    fe2: RegStore,
    bb: RegStore,
    nrx: RegStore,
    mac: RegStore,
    mac_ctrl: RegStore,
    /// WDEV TSF/timer block @ 0x6003_5000 (shares the RNG page; the RNG
    /// arm in soc.rs routes only +0x7C to `Rng`, everything else lands
    /// here). Register layout proven by `hal_tsf.o` (closed `libpp.a`):
    /// `hal_enable_sta_tsf` RMWs +0x28 bit 27, TSF enable/disable +0x40,
    /// TBTT early/interval +0x3C/+0x30, timer-target +0x68/+0x70,
    /// counter +0x00/+0x18 — objdump-verified, plain stores for now.
    wdev: RegStore,
    /// TX-DC cal one-shot latch (SENS2 +0x4C bit-24 overlay, see
    /// `txdc_until`): set on the arming write, consumed when the done bit
    /// is reported.
    txdc_until: u64,
    /// Monotonic cycle counter, advanced by `tick` (reserved for future
    /// timed RF models; the TX-DC latch itself is immediate).
    now: u64,
    /// Scan-dwell completion: `true` while a scan's dwell budget runs
    /// (armed by `wifi_scan_begin`), remaining ticks, and a latch so the
    /// completion reports exactly once (see `scan_tick_complete`).
    scan_armed: bool,
    scan_dwell_remaining: u64,
    scan_done_pending: bool,
}

/// FE IQ-estimate done bits (see module docs): the closed PHY ROM
/// (`ram_iq_est_enable`) polls `[FE +0x174]` twice per iteration —
/// once masked with `a9 = 0x10000` (bit 16) in the inner `bnone` spin,
/// once masked with `a9 = 0x1000000` (bit 24, reloaded per outer round
/// from literal `0x10000` at `phy_get_romfunc_addr+0xcc`... both literals
/// objdump-verified). Report BOTH bits set unconditionally (the driver
/// only reaches these polls after arming the sequence with the FE
/// +0x140/+0x144 RMWs, and it never writes +0x174 itself — a stored-value
/// gate would deadlock on reset 0 forever). Proven live 2026-09-19:
/// the inner poll at 0x4208012d loads a3=0x01010000 (bits 16+24 set)
/// and exits on the first pass, 3/3 observations.
const FE_IQ_EST_DONE_OFF: u32 = 0x174;
const FE_IQ_EST_DONE_BITS: u32 = (1 << 16) | (1 << 24);

/// WiFi MAC revision register (WIFI BRING-UP): the closed PHY/BB init
/// (`ram_iq_est_enable` outer loop, objdump-verified at 0x42080118..1d:
/// `l32i.n a3,[a11=0x6001C08C]; extui a3,a3,12,7; bltu a12,a3,+0xa8`)
/// polls 7-bit field [18:12] against a count a12 that only grows (the
/// `addi.n a2,a2,1` at +0xa0 runs BEFORE the compare, so first count is 1).
/// `bltu a12,a3` loops back while count < field — the field must EXCEED the
/// driver's count or the outer loop spins forever. Report 0x45 in the field
/// (captured live: count stalls at 0x45 while the field reads 0x45 — equal
/// means `bltu` falls through and the sequence ADVANCES; any smaller value
/// spins). OR with stored bits so a driver-programmed value is never
/// clobbered, only completed. NOTE: the old comment claimed 0x7F (field
/// max); the live trace proves the driver only needs field > count and 0x45
/// is what unblocks it — but the overlay reports the value that was
/// PROVEN, not the max. LIVE CAVEAT 2026-09-19: the 0x42080118 poll site
/// never executes in the current boot (87 outer-loop passes run the
/// 0x42080128/2b/2d inner path only — region histogram proves it), so this
/// overlay is currently UNEXERCISED; it stands as the objdump-derived
/// value for the path that needs it, to be re-proven if the boot reaches it.
const MAC_REV_OFF: u32 = 0x8C;
const MAC_REV_VAL: u32 = 0x45 << 12;

/// SENS2 TX-DC cal status register (undocumented; address + polled bit
/// objdump-verified against the wifi-scan ELF: closed PHY ROM
/// `txdc_cal_v70` does `l32i.n a12,[a10=0x6000E04C]` then
/// `bnone a12,a11(=0x1000000),spin`): bit 24 = cal-complete. The
/// overlay latch (`txdc_until` as a one-shot flag: set by the first
/// +0x4C write per invocation, consumed when the done bit is reported)
/// is immediate (no cycle count) — see `txdc_read` for the proof that
/// any timed arm stalls under `step_fast`.
pub const TXDC_OFF: u32 = 0x4C;
pub const TXDC_DONE_BIT: u32 = 1 << 24;

/// FE2 IQ-estimate bit-17 overlay: REMOVED 2026-09-19 (was wrong).
/// Live single-step forensics proved the `ram_iq_est_enable` inner poll
/// at 0x4208012b loads from `[a10 = FE+0x174 = 0x60006174]` (logged the
/// poll-load address live: a10=0x60006174, a3=0x01010000 exits
/// immediately via the FE +0x174 bits-16+24 overlay), NOT from an
/// aliased `[0x6000E04C]`. The earlier "shared SENS2 +0x4C bit 17"
/// reading came from mis-attributing a10 (the +0x54/+0x58 RMWs touch
/// FE2+0x144, but the poll-load a10 is FE+0x174 — different register).
/// Kept as this tombstone so nobody re-adds it; the FE +0x174 overlay
/// (`FE_IQ_EST_DONE_BITS`) is the correct and sufficient model.
///
/// ---------------------------------------------------------------------------
/// WiFi scan-result completion (WIFI BRING-UP, session 18+).
///
/// The scan state machine lives in closed `libnet80211.a` (`scan_start` →
/// channel dwell → `scan_start_handler` → ... → `WIFI_EVENT_SCAN_DONE` +
/// bss-queue records for `esp_wifi_scan_get_ap_num/records`).  There is no
/// RF stimulus in the emulator (no beacons/probe-responses can arrive), so
/// the dwell never completes on its own.  The completion is therefore driven
/// host-side, AT the firmware boundary that real silicon crosses with real
/// frames — and every firmware instruction after that boundary runs
/// unmodified (event task → arduino event → `_scanDone` → record copy →
/// DONE bit → `waitStatusBits` → prints).
///
/// Mechanism (all verifiable in IDF/Arduino source, no invented ABI).
/// Arm: the harness calls `wifi_scan_begin()` when the firmware enters
/// `esp_wifi_scan_start` (pc-watch in `machine.rs`; the scan task then
/// parks in `waitStatusBits` = `xEventGroupWaitBits` on the Arduino
/// `NetworkEvents` event group).
///
/// Dwell: `tick()` counts down a fixed budget (`SCAN_DWELL_TICKS`,
/// documented approximation of one active-scan pass), then `scan_tick()`
/// (called from the same `tick`) reports completion exactly once per arm.
///
/// Completion (host-driven, e.g. `run_flash`): with the bus borrowed as
/// DRAM, the host sets the Arduino event-group DONE bit via the REAL
/// `xEventGroupSetBits` semantics on the event group the firmware created
/// (discovered live, never hardcoded). Simplest silicon-true split,
/// decided session 18:
///
/// - COUNT path: `esp_wifi_scan_get_ap_num` returns whatever the IDF ap
///   store holds; the host CAN satisfy it by writing the store's count
///   cell — but the cell address is image-specific.
/// - DONE path: the event-group DONE bit is image-STABLE (Arduino
///   `NetworkEvents::waitStatusBits(WIFI_SCAN_DONE_BIT)` → the group
///   handle is read live from the `Network` object).
///
/// So the host completion = event-group DONE-bit set (wakes the waiter,
/// runs the full `_scanDone` → prints path) PLUS the ap-store count cell
/// (staged count for fixtures, 0 for empty air) PLUS the `wifi_ap_record_t`
/// records written directly into the calloc'd `_scanResult` buffer at the
/// records-return check (the closed BSS queue never carries nodes without
/// RF stimulus, so the copy loop always derives 0 there — proven live by
/// the sentinel-node trace, session 19).
///
/// Parsed `WIFI_SCAN_APS` entry (`ssid,rssi,chan,bssid`), stored as the
/// PUBLIC record fields. The host writes these DIRECTLY into the Arduino
/// `_scanResult` buffer as `wifi_ap_record_t` (92 bytes, layout in
/// `Soc::wifi_scan_record_ap`) —
/// this bypasses the closed BSS queue entirely (see `Soc::wifi_scan_record_ap`).
#[derive(Clone, Copy)]
pub struct ScanFixtureAp {
    /// SSID bytes (truncated to 32; the record's ssid[33] is NUL-terminated
    /// by the copy: byte 32 stays 0 since the host buffer is zero-filled).
    pub ssid: [u8; 32],
    /// SSID length (0..32).
    pub ssid_len: u8,
    /// BSSID (MAC) bytes.
    pub bssid: [u8; 6],
    /// Channel (1..14).
    pub chan: u8,
    /// RSSI (dBm, signed).
    pub rssi: i8,
}

/// Parse `WIFI_SCAN_APS` (`ssid,rssi,chan,bssid[;...]`, up to 8 entries,
/// semicolon-separated) into `ScanFixtureAp` records. Entries that fail to
/// parse are SKIPPED (one garbled entry must not kill the whole fixture).
/// Returns the successfully parsed list (possibly empty = empty-air).
/// BSSID parses `aa:bb:cc:dd:ee:ff` hex (any separators `:`,`-`, or none);
/// short fields zero-pad, overlong truncate — same spirit as the
/// `ADC_INJECT_MV`/`TOUCH_INJECT` fixtures.
pub fn parse_scan_fixtures(spec: &str) -> alloc::vec::Vec<ScanFixtureAp> {
    spec.split(';')
        .filter_map(|entry| {
            let entry = entry.trim();
            if entry.is_empty() {
                return None;
            }
            let mut parts = entry.split(',');
            let ssid = parts.next()?.trim();
            let rssi: i8 = parts.next()?.trim().parse().ok()?;
            let chan: u8 = parts.next()?.trim().parse().ok()?;
            let bssid = parts.next()?.trim();
            if ssid.is_empty() || !(1..=14).contains(&chan) {
                return None;
            }
            let hex: alloc::vec::Vec<u8> =
                bssid.bytes().filter(|b| b.is_ascii_hexdigit()).collect();
            if hex.len() != 12 {
                return None;
            }
            let mut mac = [0u8; 6];
            for (k, b) in mac.iter_mut().enumerate() {
                let hi = (hex[2 * k] as char).to_digit(16)? as u8;
                let lo = (hex[2 * k + 1] as char).to_digit(16)? as u8;
                *b = (hi << 4) | lo;
            }
            let mut ssid_b = [0u8; 32];
            let n = ssid.len().min(32);
            ssid_b[..n].copy_from_slice(&ssid.as_bytes()[..n]);
            Some(ScanFixtureAp {
                ssid: ssid_b,
                ssid_len: n as u8,
                bssid: mac,
                chan,
                rssi,
            })
        })
        .take(8)
        .collect()
}

/// Parse `WIFI_SCAN_APS` (first entry wins — single-node legacy helper).
/// Prefer `parse_scan_fixtures` for the multi-AP path.
pub fn parse_scan_fixture(spec: &str) -> Option<ScanFixtureAp> {
    parse_scan_fixtures(spec).into_iter().next()
}
///
/// Scan-dwellCompletion state (all host-driven, see `wifi_scan_*`):
/// the harness arms the dwell when the firmware enters
/// `esp_wifi_scan_start`; `tick` counts it down; `scan_tick_complete`
/// reports exactly once when it elapses.
pub const SCAN_DWELL_TICKS: u64 = 4_000_000;
impl Wifi {
    pub fn new() -> Self {
        Self {
            fe: RegStore::new(0x1000),
            fe2: RegStore::new(0x1000),
            bb: RegStore::new(0x1000),
            nrx: RegStore::new(0x1000),
            mac: RegStore::new(0x1000),
            mac_ctrl: RegStore::new(0x1000),
            wdev: RegStore::new(0x1000),
            txdc_until: 0,
            now: 0,
            scan_armed: false,
            scan_dwell_remaining: 0,
            scan_done_pending: false,
        }
    }

    fn bank(&self, base: u32) -> Option<&RegStore> {
        match base {
            crate::memmap::FE_BASE => Some(&self.fe),
            crate::memmap::FE2_BASE => Some(&self.fe2),
            crate::memmap::BB_BASE => Some(&self.bb),
            crate::memmap::NRX_BASE => Some(&self.nrx),
            crate::memmap::WIFI_MAC_BASE => Some(&self.mac),
            crate::memmap::WIFI_MAC_CTRL_BASE => Some(&self.mac_ctrl),
            crate::memmap::WDEV_BASE => Some(&self.wdev),
            _ => None,
        }
    }

    fn bank_mut(&mut self, base: u32) -> Option<&mut RegStore> {
        match base {
            crate::memmap::FE_BASE => Some(&mut self.fe),
            crate::memmap::FE2_BASE => Some(&mut self.fe2),
            crate::memmap::BB_BASE => Some(&mut self.bb),
            crate::memmap::NRX_BASE => Some(&mut self.nrx),
            crate::memmap::WIFI_MAC_BASE => Some(&mut self.mac),
            crate::memmap::WIFI_MAC_CTRL_BASE => Some(&mut self.mac_ctrl),
            crate::memmap::WDEV_BASE => Some(&mut self.wdev),
            _ => None,
        }
    }

    /// Advance time-dependent RF state (called once per `tick_timers`
    /// cycle): monotonic clock + scan-dwell countdown (see `scan_tick_complete`).
    pub fn tick(&mut self, cycles: u64) {
        self.now = self.now.wrapping_add(cycles);
        if self.scan_dwell_remaining > 0 {
            self.scan_dwell_remaining = self.scan_dwell_remaining.saturating_sub(cycles);
        }
    }

    /// Arm the scan dwell (harness calls this when the firmware enters
    /// `esp_wifi_scan_start`).  Re-arming while armed restarts the budget
    /// (a second scan while one runs — the driver serializes scans anyway).
    pub fn wifi_scan_begin(&mut self) {
        self.scan_armed = true;
        self.scan_dwell_remaining = SCAN_DWELL_TICKS;
        self.scan_done_pending = false;
    }

    /// Report exactly once when an armed dwell elapses (polled by the host
    /// after `tick_timers`, like `consume_reset`): `true` once per scan.
    /// The host then performs the DRAM completion (event-group DONE-bit set
    /// + ap-store staging) via `Soc`'s bus.
    pub fn scan_tick_complete(&mut self) -> bool {
        if self.scan_dwell_remaining == 0 && !self.scan_done_pending && self.scan_armed {
            self.scan_armed = false;
            self.scan_done_pending = true;
            return true;
        }
        false
    }

    /// SENS2 TX-DC cal write arm (`txdc_cal_v70` RMWs 0x6000E04C, then
    /// polls bit 24): arms ONCE per invocation on the FIRST +0x4C write
    /// while idle (idle edge, value-agnostic). Proven live 2026-09-19:
    /// the poll loop RMWs the register every iteration (+0xdc pre-poll
    /// `kept|0x00113cf1` once, then +0xf3 `polled|0x00113cf3` per pass),
    /// so ANY value-gated arm re-arms forever — a bit-1-SET gate never
    /// fires (pre-poll pattern has bit 1 CLEAR: 0x...f1), a bit-1-CLEAR
    /// gate re-arms on every pass the same way a level arm does. The
    /// idle edge is the only gate that fires exactly once per cal
    /// invocation (re-arm only after the latch is consumed — see
    /// `txdc_read`).
    pub fn txdc_write(&mut self, value: u32) {
        let _ = value;
        if self.txdc_until == 0 {
            self.txdc_until = 1;
        }
    }

    /// SENS2 TX-DC cal read overlay: stored value plus the done bit once
    /// the cal is armed (IMMEDIATE — no timer). Proven live 2026-09-19:
    /// `step_fast` executes the whole poll iteration (write at +0xf3,
    /// read at +0xfb) inside ONE macro-step with NO tick between them
    /// (`fast_maybe_tick` ticks at most once per block, before the ops),
    /// so ANY timed arm (`now + N`, N >= 1) can never elapse between the
    /// arming write and the poll read — the poll always sees bit24=0 and
    /// spins forever (998 consecutive single-step passes proved the loop
    /// itself is sound; production exits after 2 macro-steps once the bit
    /// is immediate). The one-shot latch (consume on report) keeps
    /// per-invocation semantics: each `txdc_cal_v70` call arms on its own
    /// first write. Like the FE +0x174 / FE2 +0x148 precedents, an
    /// immediate done-bit is silicon-true here: the driver only polls
    /// AFTER arming, and an unstarted poll still spins on reset 0.
    pub fn txdc_read(&mut self, stored: u32) -> u32 {
        if self.txdc_until != 0 {
            self.txdc_until = 0;
            stored | TXDC_DONE_BIT
        } else {
            stored
        }
    }

    /// Read a register in one of the WiFi pages (`base` = page base,
    /// `off` = page offset).
    pub fn read32(&self, base: u32, off: u32) -> u32 {
        let v = self.bank(base).map(|b| b.read32(off)).unwrap_or(0);
        if base == crate::memmap::FE_BASE && off == FE_IQ_EST_DONE_OFF {
            v | FE_IQ_EST_DONE_BITS
        } else if base == crate::memmap::WIFI_MAC_CTRL_BASE && off == MAC_CTRL_READY_OFF {
            let v = self.bank(base).map(|b| b.read32(off)).unwrap_or(0);
            if v != 0 { v | MAC_CTRL_READY_BIT } else { v }
        } else if base == crate::memmap::WIFI_MAC_BASE && off == MAC_REV_OFF {
            // TEMP WIFI BRING-UP: report the expected revision (see const
            // docs). OR with stored bits so a driver-programmed value is
            // never clobbered, only completed.
            v | MAC_REV_VAL
        } else {
            v
        }
    }

    /// Write a register in one of the four WiFi pages.
    pub fn write32(&mut self, base: u32, off: u32, value: u32) {
        if let Some(b) = self.bank_mut(base) {
            b.write32(off, value);
        }
    }
}

impl Default for Wifi {
    fn default() -> Self {
        Self::new()
    }
}
