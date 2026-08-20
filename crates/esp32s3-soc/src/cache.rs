//! ESP32-S3 cache MMU (EXTMEM peripheral).
//!
//! The data-cache window (0x3C000000) and the instruction-cache window
//! (0x42000000) are aliases over ONE shared 512-entry MMU with 64 KB pages
//! (QEMU esp32s3_cache.h: `dcache` and `icache` are both aliases of the same
//! `esp32s3_mmu-region` IOMMU; the cache I/O regs + MMU table live in the
//! EXT_MEM page). Each entry maps a virtual page to a physical flash page
//! (type=0, read-only) or PSRAM page (type=1, read-write):
//!
//! ```text
//! bit 15:14:10   [13:0]   page_number   (physical page index, 64 KB units)
//! bit 14        invalid    (virtual page not mapped)
//! bit 15        type       0 = flash, 1 = PSRAM
//! bits 31:16    reserved   must be 0
//! ```
//!
//! Register model follows QEMU `hw/misc/esp32s3_cache.c`:
//! - control regs at `EXTMEM_BASE` (0x600C4000); only the bits firmware
//!   actually reads/writes are modeled: dcache/icache enable (CTRL/CTRL1),
//!   the sync / preload / autoload "write ENA, poll DONE" handshake (the
//!   `done` flag is raised on read once `ena` was written — QEMU
//!   `check_and_reset_ena`), the freeze done bit, and CACHE_STATE (always
//!   idle, both caches).
//! - the MMU table occupies the next page (`MMU_TABLE_BASE` 0x600C5000,
//!   512 x u32). Writing an entry stores it with the reserved bits forced
//!   to 0 (QEMU `esp32s3_write_mmu_value`); QEMU's on-demand fill of its
//!   flash mirror is unnecessary here because our flash backing store is
//!   always resident.
//!
//! DELIBERATE DEVIATION from QEMU: QEMU's translate() ignores `invalid` and
//! resolves every entry through `page_number` (unset entries read page 0 of
//! a zeroed flash mirror).  We instead resolve an invalid (never mapped)
//! page as a 1:1 alias into flash, preserving this emulator's pre-MMU
//! contract (the boot ROM stub reads flash through the window without
//! programming the MMU).  Explicitly mapped pages (flash or PSRAM) always
//! take precedence.

use crate::memmap::{CACHE_PAGE_SIZE, MMU_ENTRIES, WINDOW_MASK};

/// Where a cache-window access lands after MMU translation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CacheTarget {
    /// Physical flash offset (read-only).
    Flash(u32),
    /// Physical PSRAM offset (read-write).
    Psram(u32),
}

/// MMU entry bit fields (QEMU ESP32S3MMUEntry).
const MMU_PAGE_MASK: u32 = 0x3FFF; // [13:0]
const MMU_INVALID: u32 = 1 << 14;
const MMU_TYPE: u32 = 1 << 15; // 1 = PSRAM

/// Cache / MMU controller.
pub struct Cache {
    /// EXT_MEM control registers (offsets 0..0x3FC, word-indexed).
    regs: [u32; 256],
    /// Shared MMU table (512 entries, one per 64 KB virtual page).
    mmu: [u32; MMU_ENTRIES],
    dcache_enable: u32,
    icache_enable: u32,
}

impl Cache {
    pub fn new() -> Self {
        let mut c = Self {
            regs: [0; 256],
            mmu: [MMU_INVALID; MMU_ENTRIES],
            dcache_enable: 0,
            icache_enable: 0,
        };
        // On reset the autoload and manual preload controllers are "done"
        // (ready) so firmware init polls exit immediately (QEMU
        // esp32s3_cache_reset_hold).
        c.regs[0x04C >> 2] |= 1 << 3; // DCACHE_AUTOLOAD_CTRL.AUTOLOAD_DONE
        c.regs[0x040 >> 2] |= 1 << 1; // DCACHE_PRELOAD_CTRL.PRELOAD_DONE
        c.regs[0x0A0 >> 2] |= 1 << 3; // ICACHE_AUTOLOAD_CTRL.AUTOLOAD_DONE
        c.regs[0x094 >> 2] |= 1 << 1; // ICACHE_PRELOAD_CTRL.PRELOAD_DONE
        c
    }

    /// Translate a cache-window virtual address (`vaddr` inside the data or
    /// instruction window) to its physical target.  The window offset is the
    /// low 25 bits — both bases (0x3C000000 / 0x42000000) share the same
    /// low bits, so one table serves both windows (QEMU icache alias).
    pub fn translate(&self, vaddr: u32) -> Option<CacheTarget> {
        let off = vaddr & WINDOW_MASK;
        let idx = (off / CACHE_PAGE_SIZE) as usize;
        let within = off & (CACHE_PAGE_SIZE - 1);
        let e = self.mmu[idx];
        if e & MMU_TYPE != 0 {
            Some(CacheTarget::Psram(
                (e & MMU_PAGE_MASK) * CACHE_PAGE_SIZE + within,
            ))
        } else if e & MMU_INVALID != 0 {
            // Deviation (module docs): unmapped -> 1:1 flash alias.
            Some(CacheTarget::Flash(off))
        } else {
            Some(CacheTarget::Flash(
                (e & MMU_PAGE_MASK) * CACHE_PAGE_SIZE + within,
            ))
        }
    }

    /// MMU table read (addr = offset within the MMU_TABLE_BASE page).
    pub fn mmu_read32(&self, off: u32) -> u32 {
        self.mmu[(off >> 2) as usize]
    }

    /// MMU table write; reserved bits [31:16] are forced to 0 (QEMU
    /// esp32s3_write_mmu_value).
    pub fn mmu_write32(&mut self, off: u32, val: u32) {
        self.mmu[(off >> 2) as usize] = val & 0xFFFF;
    }

    /// Cache control register read.  The sync/preload/autoload controllers
    /// report `done` on read once `ena` was written and clear `ena` (QEMU
    /// check_and_reset_ena — the S3 cache_ll code polls `*_DONE`).
    pub fn read32(&mut self, off: u32) -> u32 {
        let idx = (off >> 2) as usize;
        match off {
            0x000 | 0x004 => self.dcache_enable,
            0x060 | 0x064 => self.icache_enable,
            // SYNC_CTRL (DCACHE 0x028 / ICACHE 0x088): INVALIDATE_ENA bit 0.
            // DONE bit differs: DCACHE = bit 3, ICACHE = bit 1 (esp-idf
            // extmem_reg.h EXTMEM_DCACHE_SYNC_DONE bitpos 3 vs
            // EXTMEM_ICACHE_SYNC_DONE bitpos 1 — the ROM's cache-sync
            // routine at 0x4004E550 polls `bnone a9, 0x8` on 0x600C4028).
            // PRELOAD_CTRL (DCACHE 0x040 / ICACHE 0x094): ENA bit 0, DONE
            // bit 1 (both).
            0x028 => self.check_and_reset_ena(idx, 0x1, 0x8),
            0x088 | 0x040 | 0x094 => self.check_and_reset_ena(idx, 0x1, 0x2),
            // AUTOLOAD_CTRL (DCACHE 0x04C / ICACHE 0x0A0): AUTOLOAD_ENA
            // bit 2, AUTOLOAD_DONE bit 3.
            0x04C | 0x0A0 => self.check_and_reset_ena(idx, 0x4, 0x8),
            // CACHE_STATE: report both caches idle (QEMU read handler).
            0x130 => (1 << 0) | (1 << 12),
            // FREEZE (DCACHE 0x150 / ICACHE 0x154): stored done bit.
            0x150 | 0x154 => self.regs[idx],
            _ => 0,
        }
    }

    fn check_and_reset_ena(&mut self, idx: usize, ena_mask: u32, done_mask: u32) -> u32 {
        let v = self.regs[idx];
        if v & ena_mask != 0 {
            let nv = (v & !ena_mask) | done_mask;
            self.regs[idx] = nv;
            nv
        } else {
            v
        }
    }

    /// Cache control register write.
    pub fn write32(&mut self, off: u32, val: u32) {
        let idx = (off >> 2) as usize;
        match off {
            0x000 | 0x004 => self.dcache_enable = val & 1,
            0x060 | 0x064 => self.icache_enable = val & 1,
            0x150 | 0x154 => {
                // Freeze: write toggles only the DONE bit (QEMU).
                if val & 0x1 != 0 {
                    self.regs[idx] |= 1 << 2;
                } else {
                    self.regs[idx] &= !(1 << 2);
                }
            }
            _ => self.regs[idx] = val,
        }
    }
}

impl Default for Cache {
    fn default() -> Self {
        Self::new()
    }
}
