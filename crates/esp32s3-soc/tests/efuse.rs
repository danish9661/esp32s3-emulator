//! EFUSE unit tests: modeled factory MAC in the read-data registers, read_cmd
//! handshake is a no-op, STATUS reports idle.

use esp32s3_soc::efuse::Efuse;

// Factory MAC 0x112233445566 is stored byte-reversed per word (block bit[0:8)
// is a word's LSB, and the driver maps block bit[0:8) -> MAC byte[0]).
const MAC_LO: u32 = 0x4433_2211;
const MAC_HI: u32 = 0x0000_6655;

#[test]
fn modeled_mac_in_sys_part1_and_spi_sys() {
    let mut e = Efuse::new();
    // RD_SYS_PART1_DATA0..1 (block 1 factory MAC).
    assert_eq!(e.read32(0x5C), MAC_LO);
    assert_eq!(e.read32(0x60), MAC_HI);
    // RD_MAC_SPI_SYS_0..1 (identical SPI-boot MAC).
    assert_eq!(e.read32(0x44), MAC_LO);
    assert_eq!(e.read32(0x48), MAC_HI);
}

#[test]
fn status_reports_idle() {
    let mut e = Efuse::new();
    // EFUSE_STATUS_REG (0x1D0) state field reads idle (0) so the driver's
    // read-done poll exits immediately.
    assert_eq!(e.read32(0x1D0), 0);
}

#[test]
fn read_cmd_is_noop_and_preserves_mac() {
    let mut e = Efuse::new();
    // Issuing read_cmd must not clobber the modeled MAC.
    e.write32(0x1D4, 1);
    assert_eq!(e.read32(0x5C), MAC_LO);
    assert_eq!(e.read32(0x60), MAC_HI);
    // Writes to the read-only data registers are dropped.
    e.write32(0x5C, 0xDEAD_BEEF);
    assert_eq!(e.read32(0x5C), MAC_LO);
    // Random read of an unmodeled register returns 0.
    assert_eq!(e.read32(0x200), 0);
}

#[test]
fn burn_block3_ors_usr_data_and_sticks() {
    // PGM_DATA staging (0x00..0x1C) + PGM_CMD (0x1D4: PGM bit 1,
    // BLK_NUM 3 in [5:2]) ORs into RD_USR_DATA0..7 (@ 0x7C).
    // eFuse bits only set (never clear): burning twice accumulates.
    use esp32s3_soc::efuse::Efuse;
    let mut e = Efuse::new();
    assert_eq!(e.read32(0x7C), 0);
    e.write32(0x00, 0xA5A5_00FF);
    e.write32(0x1D4, (3 << 2) | 0x2);
    assert_eq!(e.read32(0x7C), 0xA5A5_00FF);
    // Second burn ORs (cannot clear).
    e.write32(0x00, 0x00FF_00FF);
    e.write32(0x1D4, (3 << 2) | 0x2);
    assert_eq!(e.read32(0x7C), 0xA5FF_00FF);
    // Other block numbers are ignored (only USR_DATA modeled).
    e.write32(0x00, 0xFFFF_FFFF);
    e.write32(0x1D4, (5 << 2) | 0x2);
    assert_eq!(e.read32(0x7C), 0xA5FF_00FF, "block 5 untouched");
}

// PGM_CMD helper: PGM bit 1 + BLK_NUM in [5:2] (efuse_reg.h).
fn pgm_cmd(e: &mut Efuse, blk: u32) {
    e.write32(0x1D4, 0x2 | (blk << 2));
}

fn stage(e: &mut Efuse, words: &[u32]) {
    for (i, &w) in words.iter().enumerate() {
        e.write32((i * 4) as u32, w);
    }
}

#[test]
fn burn_block0_sets_wr_dis_and_blocks_protected_reburn() {
    let mut e = Efuse::new();
    // Burn WR_DIS bit 3 (protects BLK3 under the bitN->BLKN mapping).
    stage(&mut e, &[1 << 3, 0, 0, 0, 0, 0, 0, 0]);
    pgm_cmd(&mut e, 0);
    assert_eq!(e.read32(0x2C), 1 << 3, "WR_DIS burned");
    // A direct PGM to the protected block is now dropped...
    stage(&mut e, &[0xFFFF_FFFF; 8]);
    pgm_cmd(&mut e, 3);
    assert_eq!(e.read32(0x7C), 0, "protected BLK3 burn blocked");
    // ...while an unprotected block still burns.
    pgm_cmd(&mut e, 10);
    assert_eq!(e.read32(0x15C), 0xFFFF_FFFF, "BLK10 burned");
}

#[test]
fn burn_key_block_via_pgm_and_rd_dis_masks_reads() {
    let mut e = Efuse::new();
    stage(&mut e, &[0xA5A5_A5A5, 0x5A5A_5A5A, 0, 0, 0, 0, 0, 0]);
    pgm_cmd(&mut e, 4);
    assert_eq!(e.read32(0x9C), 0xA5A5_A5A5, "KEY0 word 0 burned");
    assert_ne!(e.hmac_key(0), [0u8; 32], "key readable before RD_DIS");
    // Burn RD_DIS bit 0 (REPEAT_DATA0 @ 0x30, efuse_reg.h RD_DIS[6:0]):
    // KEY0 mirror + hmac_key both read zero. WR_DIS word stays clear so
    // the BLK0 burn itself is not self-blocked.
    stage(&mut e, &[0, 1, 0, 0, 0, 0, 0, 0]);
    pgm_cmd(&mut e, 0);
    assert_eq!(e.read32(0x30) & 1, 1, "RD_DIS burned");
    assert_eq!(e.read32(0x9C), 0, "KEY0 mirror masked");
    assert_eq!(e.hmac_key(0), [0u8; 32], "hmac_key masked");
}

#[test]
fn secure_boot_enable_lives_in_repeat_data4() {
    let mut e = Efuse::new();
    assert!(!e.secure_boot_enabled(), "reset disabled");
    // Burn REPEAT_DATA4 bit 20 (SECURE_BOOT_EN) through BLK0 word 5
    // (BLK0 window 0x2C + 5*4 = 0x40).
    stage(&mut e, &[0, 0, 0, 0, 0, 1 << 20, 0, 0]);
    pgm_cmd(&mut e, 0);
    assert!(e.secure_boot_enabled(), "SECURE_BOOT_EN burned");
}
