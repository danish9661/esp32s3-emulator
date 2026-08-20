//! Cache MMU unit tests: MMU table entries, PSRAM read/write through mapped
//! data/instruction window pages, physical-page remapping, the unmapped
//! 1:1 flash alias, and the sync/preload/autoload/freeze done handshakes.

// `(0 << n)`-style markers document field positions in register words
// (mirrors the generated.rs allow header).
#![allow(clippy::identity_op)]

use esp32s3_soc::Soc;
use esp32s3_soc::memmap::{
    CACHE_PAGE_SIZE, EXTMEM_BASE, FLASH_DATA_BASE, FLASH_INST_BASE, MMU_TABLE_BASE, PSRAM_SIZE,
};
use xtensa_core::Bus;

fn mmu_addr(idx: usize) -> u32 {
    MMU_TABLE_BASE + idx as u32 * 4
}

/// MMU entry for PSRAM physical page `p` (type=1, invalid=0).
fn psram_entry(p: u32) -> u32 {
    0x8000 | p
}

/// MMU entry for flash physical page `p` (type=0, invalid=0).
fn flash_entry(p: u32) -> u32 {
    p
}

#[test]
fn mmu_entry_write_clears_reserved() {
    let mut soc = Soc::new();
    // Reserved bits [31:16] are forced to 0 on write (QEMU
    // esp32s3_write_mmu_value).
    soc.write32(mmu_addr(7), 0xFFFF_8000);
    assert_eq!(soc.read32(mmu_addr(7)), 0x8000);
}

#[test]
fn psram_read_write_via_mapped_page() {
    let mut soc = Soc::new();
    soc.write32(mmu_addr(0), psram_entry(0));
    soc.write32(FLASH_DATA_BASE, 0xDEAD_BEEF);
    soc.write16(FLASH_DATA_BASE + 0x4, 0xCAFE);
    soc.write8(FLASH_DATA_BASE + 0x6, 0xAB);
    assert_eq!(soc.read32(FLASH_DATA_BASE), 0xDEAD_BEEF);
    assert_eq!(soc.read16(FLASH_DATA_BASE + 0x4), 0xCAFE);
    assert_eq!(soc.read8(FLASH_DATA_BASE + 0x6), 0xAB);
    // The instruction window aliases the same shared MMU table (QEMU
    // icache == dcache alias).
    assert_eq!(soc.read32(FLASH_INST_BASE), 0xDEAD_BEEF);
}

#[test]
fn psram_physical_page_selected_by_entry() {
    let mut soc = Soc::new();
    // vpage 3 -> PSRAM physical page 2.
    soc.write32(mmu_addr(3), psram_entry(2));
    let v3 = FLASH_DATA_BASE + 3 * CACHE_PAGE_SIZE;
    soc.write32(v3, 0x1234_5678);
    assert_eq!(soc.read32(v3), 0x1234_5678);
    // vpage 2 is unmapped -> 1:1 flash alias (no PSRAM content).
    assert_eq!(
        soc.read32(FLASH_DATA_BASE + 2 * CACHE_PAGE_SIZE),
        0,
        "unmapped page does not alias PSRAM page 2"
    );
}

#[test]
fn flash_page_remap_and_readonly() {
    let mut soc = Soc::new();
    let img = b"ABCDEFGH".to_vec();
    soc.load_flash_image(0x1_0000, &img);
    // vpage 2 -> flash physical page 1.
    soc.write32(mmu_addr(2), flash_entry(1));
    assert_eq!(
        soc.read32(FLASH_DATA_BASE + 2 * CACHE_PAGE_SIZE),
        u32::from_le_bytes(*b"ABCD")
    );
    // Flash pages are read-only: writes are dropped.
    soc.write32(FLASH_DATA_BASE + 2 * CACHE_PAGE_SIZE, 0);
    assert_eq!(
        soc.read32(FLASH_DATA_BASE + 2 * CACHE_PAGE_SIZE),
        u32::from_le_bytes(*b"ABCD"),
        "flash page write dropped"
    );
}

#[test]
fn unmapped_page_aliases_flash_1to1() {
    let mut soc = Soc::new();
    let img = b"XIP!XIP!".to_vec();
    soc.load_flash_image(0x1000, &img);
    // Never programmed MMU entries resolve as 1:1 flash (the pre-MMU
    // contract the boot ROM stub relies on).
    assert_eq!(
        soc.read32(FLASH_DATA_BASE + 0x1000),
        u32::from_le_bytes(*b"XIP!")
    );
    assert_eq!(
        soc.read32(FLASH_INST_BASE + 0x1004),
        u32::from_le_bytes(*b"XIP!")
    );
}

#[test]
fn psram_beyond_capacity_reads_zero() {
    let mut soc = Soc::new();
    // PSRAM physical page 200 (200 * 64 KB = 12.8 MB) is past the modeled
    // 8 MB capacity: reads 0, writes dropped.
    soc.write32(mmu_addr(0), psram_entry(200));
    soc.write32(FLASH_DATA_BASE, 0x1122_3344);
    assert_eq!(soc.read32(FLASH_DATA_BASE), 0);
    assert_eq!(PSRAM_SIZE, 0x80_0000, "8 MB default capacity");
}

#[test]
fn sync_ena_polls_to_done() {
    let mut soc = Soc::new();
    // cache_ll_sync pattern: write INVALIDATE_ENA, poll SYNC_DONE.
    soc.write32(EXTMEM_BASE + 0x88, 0x1); // ICACHE_SYNC_CTRL.INVALIDATE_ENA
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x88) & 0x2,
        0,
        "ICACHE SYNC_DONE set on read after ENA"
    );
    // DONE stays set (read does not clear it; a fresh ENA re-arms).
    assert_ne!(soc.read32(EXTMEM_BASE + 0x88) & 0x2, 0);
    // Same for the DCACHE sync controller (SYNC_DONE = bit 3, extmem_reg.h
    // EXTMEM_DCACHE_SYNC_DONE — unlike ICACHE's bit 1).
    soc.write32(EXTMEM_BASE + 0x28, 0x1); // DCACHE_SYNC_CTRL.INVALIDATE_ENA
    assert_ne!(soc.read32(EXTMEM_BASE + 0x28) & 0x8, 0);
}

#[test]
fn preload_autoload_reset_done_and_ena_handshake() {
    let mut soc = Soc::new();
    // Reset: autoload + preload controllers report done (ready) so init
    // polls exit immediately (QEMU esp32s3_cache_reset_hold).
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x04C) & (1 << 3),
        0,
        "DCACHE autoload done"
    );
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x0A0) & (1 << 3),
        0,
        "ICACHE autoload done"
    );
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x040) & (1 << 1),
        0,
        "DCACHE preload done"
    );
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x094) & (1 << 1),
        0,
        "ICACHE preload done"
    );
    // A fresh PRELOAD_ENA clears done, then done returns on read.
    soc.write32(EXTMEM_BASE + 0x094, 0x1); // ICACHE_PRELOAD_CTRL.PRELOAD_ENA
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x094) & 0x2,
        0,
        "preload done after ena"
    );
    soc.write32(EXTMEM_BASE + 0x0A0, 0x4); // ICACHE_AUTOLOAD_CTRL.AUTOLOAD_ENA
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x0A0) & 0x8,
        0,
        "autoload done after ena"
    );
}

#[test]
fn freeze_toggles_done_and_cache_state_idle() {
    let mut soc = Soc::new();
    // ICACHE_FREEZE: write ENA -> DONE set; write 0 -> DONE cleared.
    soc.write32(EXTMEM_BASE + 0x154, 0x1);
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x154) & (1 << 2),
        0,
        "ICACHE freeze done"
    );
    soc.write32(EXTMEM_BASE + 0x154, 0x0);
    assert_eq!(
        soc.read32(EXTMEM_BASE + 0x154) & (1 << 2),
        0,
        "freeze released"
    );
    // DCACHE_FREEZE same.
    soc.write32(EXTMEM_BASE + 0x150, 0x1);
    assert_ne!(
        soc.read32(EXTMEM_BASE + 0x150) & (1 << 2),
        0,
        "DCACHE freeze done"
    );
    // CACHE_STATE reports both caches idle.
    assert_eq!(
        soc.read32(EXTMEM_BASE + 0x130),
        (1 << 0) | (1 << 12),
        "CACHE_STATE idle"
    );
}

#[test]
fn cache_ctrl_enable_readback() {
    let mut soc = Soc::new();
    soc.write32(EXTMEM_BASE + 0x000, 0x1); // DCACHE_CTRL.ENABLE
    assert_eq!(soc.read32(EXTMEM_BASE + 0x000), 0x1);
    assert_eq!(
        soc.read32(EXTMEM_BASE + 0x004),
        0x1,
        "DCACHE_CTRL1 mirrors enable"
    );
    soc.write32(EXTMEM_BASE + 0x060, 0x1); // ICACHE_CTRL.ENABLE
    assert_eq!(soc.read32(EXTMEM_BASE + 0x060), 0x1);
}
