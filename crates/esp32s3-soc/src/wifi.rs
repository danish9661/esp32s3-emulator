//! WiFi radio blocks (FE/FE2/BB/NRX) — TEMP WIFI BRING-UP scaffold.
//!
//! The ESP32-S3 WiFi PHY is a closed ROM/driver blob: there are no public
//! per-register semantics for the RF front-end (FE @ `0x6000_6000`, FE2 @
//! `0x6000_5000`), baseband (BB @ `0x6001_D000`) or NRX (`0x6001_CC00`)
//! blocks (no `*_reg.h`/`*_struct.h` shipped in arduino-cli; only the
//! power-up/down bits in `fe_reg.h`/`bb_reg.h`/`nrx_reg.h` are public).
//! What IS observable is the driver's register-level behavior, verified by
//! objdump against the linked sketch ELF:
//!
//! * PHY RF-cal (`ram_iq_est_enable`, wifi-scan ELF): RMWs FE `+0x140`,
//!   `+0x144`, then polls FE `+0x174` bit 0 (`l32i.n a12,[a15]`;
//!   `bnone a12,a11(=0x1000000)`,spin — wait, the mask is 0x1000000 =
//!   bit 24, not bit 0; see below).
//! * TX-DC cal (`txdc_cal_v70`): RMWs SENS `0x6000E04C`, polls bit 24.
//!
//! Model: plain register stores (round-trip, like `regstore.rs`), PLUS the
//! two proven done-bits, both gated on the driver arming the sequence
//! (any nonzero write to the same register) so an unstarted poll still
//! spins:
//!
//! * FE `+0x174` bit 24: set once armed (proven: `ram_iq_est_enable` spins
//!   with `a15 = <FE+0x174>`, `a11 = 0x1000000`; `bnone` exits iff the bit
//!   is set; without it the task watchdog fires inside `bb_init` ←
//!   `register_chipv7_phy` ← `esp_phy_load_cal_and_init`).
//! * SENS2 `+0x4C` bit 24: same treatment (see the SENS2 arm in `soc.rs`;
//!   kept there because that page already has a dedicated arm).
//!
//! Everything else in the four pages reads back what was written. When the
//! next spin is found, add its done-bit here with the same arming rule and
//! cite the objdump evidence — never invent timing.

use crate::regstore::RegStore;

/// MAC control ready bit (see module docs): `hal_init` polls bit 0 of
/// `+0xD14` via `bbci` (branch-while-clear) after setting bit 1 with an
/// `or`+store. Report bit 0 set once armed (any write to the register —
/// the driver always writes first); before the arm read back the stored
/// value (reset 0).
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
}

/// FE IQ-estimate done bits (see module docs): the closed PHY ROM
/// (`ram_iq_est_enable`) polls `[FE +0x174]` twice per iteration —
/// once masked with `a9 = 0x10000` (bit 16) in the inner `bnone` spin,
/// once masked with `a9 = 0x1000000` (bit 24, reloaded per outer round
/// from literal `0x10000` at `phy_get_romfunc_addr+0xcc`... both literals
/// objdump-verified). Report BOTH bits set unconditionally (the driver
/// only reaches these polls after arming the sequence with the FE
/// +0x140/+0x144 RMWs, and it never writes +0x174 itself — a stored-value
/// gate would deadlock on reset 0 forever).
const FE_IQ_EST_DONE_OFF: u32 = 0x174;
const FE_IQ_EST_DONE_BITS: u32 = (1 << 16) | (1 << 24);

/// WiFi MAC revision register (TEMP WIFI BRING-UP): the closed PHY/BB
/// init (`ram_iq_est_enable` outer loop) polls `[a11 = 0x6001C08C]`
/// field [18:12] (`l32i.n a3,a11,0; extui a3,a3,12,7; bltu a12,a3`)
/// against a count that only grows (a12 = 0x45 at the observed spin),
/// i.e. it waits for a 7-bit HW revision/step counter to EXCEED the
/// driver's count. Report a value above any reachable count (0x7F =
/// field max, captured live: count stalls at 0x45 while the field reads
/// 0x45 and `bltu` needs strict less-than). OR with stored bits so a
/// driver-programmed value is never clobbered, only completed.
const MAC_REV_OFF: u32 = 0x8C;
const MAC_REV_VAL: u32 = 0x7F << 12;

impl Wifi {
    pub fn new() -> Self {
        Self {
            fe: RegStore::new(0x1000),
            fe2: RegStore::new(0x1000),
            bb: RegStore::new(0x1000),
            nrx: RegStore::new(0x1000),
            mac: RegStore::new(0x1000),
            mac_ctrl: RegStore::new(0x1000),
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
            _ => None,
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
