//! ESP32-S3 EFUSE (eFuse controller) peripheral model.
//!
//! Register layout per the S3 `efuse_struct.h` (DR_REG_EFUSE_BASE = 0x60007000):
//! the read-data registers mirror the eFuse array blocks. The factory MAC
//! (`ESP_EFUSE_MAC_FACTORY`, block 1) is exposed at `RD_SYS_PART1_DATA0..7`
//! (base 0x5C); the identical MAC used for SPI-flash boot lives at
//! `RD_MAC_SPI_SYS_0..5` (base 0x44). `EFUSE_CMD_REG` (0x1D4) `read_cmd` (bit 0)
//! triggers the copy of the eFuse array into the `RD_*` registers;
//! `EFUSE_STATUS_REG` (0x1D0) `state` field (bits [3:0]) is polled until idle.
//! We materialize a fixed eFuse array so the real esp-idf eFuse driver reads a
//! deterministic MAC / chip id.
//!
//! Flash encryption: the eFuse flash-crypt config reads correct-by-default
//! (RD_REPEAT fields reset 0: SPI_BOOT_CRYPT_CNT = 0, key purposes = 0,
//! so every boot takes the encryption-OFF plaintext path). The encrypted-
//! image pipeline is modeled too: host-provisioned XTS key in BLOCK_KEY0
//! plus SPI_BOOT_CRYPT_CNT, uniform XTS over flash, decrypt on fetch.
//! Validated by the `flashenc` battery entry, which boots real hello
//! firmware from an encrypted image.

use crate::memmap::EFUSE_BASE;

pub const EFUSE_BASE_REG: u32 = EFUSE_BASE;

const REG_COUNT: usize = 0x200 / 4;

// Register offsets in 32-bit words.
const RD_MAC_SPI_SYS0_OFF: usize = 0x44 / 4;
const RD_MAC_SPI_SYS1_OFF: usize = 0x48 / 4;
const RD_SYS_PART1_DATA0_OFF: usize = 0x5C / 4;
const RD_SYS_PART1_DATA1_OFF: usize = 0x60 / 4;
const EFUSE_STATUS_OFF: usize = 0x1D0 / 4;
const EFUSE_CLK_OFF: usize = 0x1C8 / 4;
const EFUSE_CMD_OFF: usize = 0x1D4 / 4;

// HMAC/Digital-Signature key blocks KEY0..KEY5.  Each block is 8 words
// (256 bits = 32 bytes); the 6 block bases are spaced 0x20 (32 bytes) apart.
// The real registers are read-only (they mirror the burned eFuse array), but
// for the emulator we make them writable so a sketch can provision a test key;
// they default to zero (a valid all-zero HMAC key).  Key byte j (0..32) is the
// big-endian byte of word (j/4): byte 0 = DATA0[31:24] .. byte 3 = DATA0[7:0].
const EFUSE_KEY_BASE_OFF: usize = 0x9C / 4;
const EFUSE_KEY_STRIDE_OFF: usize = 0x20 / 4;
const EFUSE_KEY_WORDS: usize = 8;
const EFUSE_KEY_COUNT: usize = 6;

// Modeled factory MAC: 0x112233445566. The esp-idf eFuse driver assembles the
// MAC with block bit[0:8) -> MAC byte[0], and bit[0:8) of a 32-bit eFuse word is
// its LSB — so each word stores the MAC in reversed byte order. Block 1 words
// (DATA0 = bits[0:32), DATA1 = bits[32:64)): DATA0 = 0x44332211, DATA1 =
// 0x00006655.
const MAC_LO: u32 = 0x4433_2211;
const MAC_HI: u32 = 0x0000_6655;
// eFuse block RD-mirror bases (efuse_reg.h, all 8-word contiguous windows):
// BLK0 (WR_DIS + REPEAT_DATA0..4 + MAC_SPI_SYS_0/1) @ 0x2C, BLK3 USR_DATA
// @ 0x7C, BLK4..9 KEYn @ 0x9C + n*0x20, BLK10 SYS_DATA_PART2 @ 0x15C.
// BLK1/BLK2 mirrors (MAC/SYS_PART1) are non-contiguous partial windows
// with factory content and no driver burn flow — PGM to them is ignored.
// RD_REPEAT_DATA0[6:0] = RD_DIS (efuse_reg.h): bit i masks reads of
// BLK(4+i) (KEY0..5, then BLK10 SYS_DATA_PART2); BLK0..3 stay readable.
const BLK0_MIRROR_OFF: usize = 0x2C / 4;
const BLK10_MIRROR_OFF: usize = 0x15C / 4;
const BLOCK_WORDS: usize = 8;
const REPEAT_DATA0_OFF: usize = 0x30 / 4;
const RD_DIS_MASK: u32 = 0x7F;
// RD_REPEAT_DATA4 (0x40) SECURE_BOOT_EN bit (efuse_reg.h bit 20).
const REPEAT_DATA4_OFF: usize = 0x40 / 4;
const SECURE_BOOT_EN_BIT: u32 = 1 << 20;

#[derive(Clone)]
pub struct Efuse {
    regs: [u32; REG_COUNT],
    /// Staged PGM_DATA0..7 burn words (written before PGM_CMD).
    pgm_stage: [u32; 8],
}

impl Efuse {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        // Factory MAC (block 1) — both the BLK1/SYS_PART1 mirror and the
        // SPI-boot MAC mirror carry the same value on real silicon.
        regs[RD_SYS_PART1_DATA0_OFF] = MAC_LO;
        regs[RD_SYS_PART1_DATA1_OFF] = MAC_HI;
        regs[RD_MAC_SPI_SYS0_OFF] = MAC_LO;
        regs[RD_MAC_SPI_SYS1_OFF] = MAC_HI;
        Self {
            regs,
            pgm_stage: [0; 8],
        }
    }
}

impl Default for Efuse {
    fn default() -> Self {
        Self::new()
    }
}

impl Efuse {
    pub fn read32(&mut self, off: u32) -> u32 {
        let w = (off >> 2) as usize;
        // PGM_DATA reads back the staged burn words.
        if off <= 0x1C {
            return self.pgm_stage[(off >> 2) as usize];
        }
        match w {
            // EFUSE_STATUS_REG state field reads idle (0) — the driver's
            // read-done poll exits immediately on our materialized array.
            EFUSE_STATUS_OFF => 0,
            _ if w < REG_COUNT => {
                // Read-disabled blocks (RD_DIS) read zero like blown fuses.
                if let Some(bit) = Self::rd_dis_for(w)
                    && self.regs[REPEAT_DATA0_OFF] & (1 << bit) & RD_DIS_MASK != 0
                {
                    return 0;
                }
                self.regs[w]
            }
            _ => 0,
        }
    }

    /// SPI_BOOT_CRYPT_CNT field (RD_REPEAT_DATA1 @ 0x34, bits [20:18]):
    /// odd parity enables flash encryption (`efuse_hal_flash_encryption_
    /// enabled` reads exactly this). Resets 0 (disabled).
    pub fn crypt_cnt(&self) -> u32 {
        (self.regs[0x34 / 4] >> 18) & 7
    }

    /// Read-disable bit covering mirror word `w`, if any (RD_DIS[6:0] ->
    /// BLK4..10: KEY0..5 mirrors, then BLK10 SYS_DATA_PART2).
    fn rd_dis_for(w: usize) -> Option<u32> {
        if (EFUSE_KEY_BASE_OFF..EFUSE_KEY_BASE_OFF + EFUSE_KEY_COUNT * EFUSE_KEY_STRIDE_OFF)
            .contains(&w)
        {
            Some(((w - EFUSE_KEY_BASE_OFF) / EFUSE_KEY_STRIDE_OFF) as u32)
        } else if (BLK10_MIRROR_OFF..BLK10_MIRROR_OFF + BLOCK_WORDS).contains(&w) {
            Some(6)
        } else {
            None
        }
    }

    /// Secure-boot enable (RD_REPEAT_DATA4 SECURE_BOOT_EN, efuse_reg.h).
    pub fn secure_boot_enabled(&self) -> bool {
        self.regs[REPEAT_DATA4_OFF] & SECURE_BOOT_EN_BIT != 0
    }

    /// Read the 32-byte HMAC/DS key from eFuse key block `id` (0..5), as a
    /// big-endian-per-word byte stream (key byte j = DATA(j/4) byte (3 - j%4)).
    pub fn hmac_key(&self, id: usize) -> [u8; 32] {
        let mut key = [0u8; 32];
        if id >= EFUSE_KEY_COUNT {
            return key;
        }
        // A read-disabled key block reads all zeros (same as the RD mirror).
        if self.regs[REPEAT_DATA0_OFF] & (1 << id) & RD_DIS_MASK != 0 {
            return key;
        }
        let base = EFUSE_KEY_BASE_OFF + id * EFUSE_KEY_STRIDE_OFF;
        for i in 0..EFUSE_KEY_WORDS {
            let word = self.regs[base + i];
            let j = i * 4;
            key[j] = (word >> 24) as u8;
            key[j + 1] = (word >> 16) as u8;
            key[j + 2] = (word >> 8) as u8;
            key[j + 3] = word as u8;
        }
        key
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        let w = (off >> 2) as usize;
        // PGM_DATA0..7 staging (off 0x00..0x1C): held until PGM_CMD.
        if off <= 0x1C {
            self.pgm_stage[(off >> 2) as usize] = value;
            return;
        }
        match w {
            // RD_REPEAT_DATA1 (@ 0x34): read-only on silicon, but the host
            // fixture provisions SPI_BOOT_CRYPT_CNT (bits [20:18]) here to
            // model a factory-encrypted device. One-way (OR) like a burn.
            _ if w == 0x34 / 4 => {
                self.regs[w] |= value & (7 << 18);
            }
            // read_cmd (bit 0): on real silicon this copies the eFuse array into
            // the RD_* registers. Our array is already materialized, so this is a
            // no-op (the STATUS poll sees idle immediately).
            // PGM_CMD (bit 1) with BLK_NUM[5:2]: one-way burn — OR the staged
            // PGM_DATA words into the block's RD mirror. Only BLOCK_USR_DATA
            // (block 3, RD_USR_DATA0..7 @ 0x7C) is modeled; other blocks are
            // ignored (documented). Verified: the driver prints
            // "BURN BLOCK3" for ESP_EFUSE_USER_DATA.
            EFUSE_CMD_OFF => {
                if value & 0x2 != 0 {
                    let blk = (value >> 2) & 0xF;
                    // WR_DIS (BLOCK0 word 0) blocks direct PGM burns to the
                    // named block (bitN -> BLKN approximation; the driver's
                    // own refusal reads the stored WR_DIS value directly, so
                    // driver flows are exact regardless of this mapping).
                    let wr_dis = self.regs[BLK0_MIRROR_OFF];
                    // (base mirror word, word count): BLK0 is 6 words
                    // (WR_DIS + REPEAT_DATA0..4 @ 0x2C..0x40 — the
                    // MAC_SPI_SYS mirrors past it belong to BLK1, and
                    // burning staged zeros over the seeded MAC trips the
                    // driver's read-back verify); the rest are full 8-word
                    // blocks. BLK1/BLK2 (MAC/SYS_PART1 partial mirrors) and
                    // 11..15 have no driver burn flow and are ignored.
                    let mirror = match blk {
                        0 => Some((BLK0_MIRROR_OFF, 6)),
                        3 => Some((0x7C / 4, BLOCK_WORDS)),
                        4..=9 => Some((
                            EFUSE_KEY_BASE_OFF + (blk - 4) as usize * EFUSE_KEY_STRIDE_OFF,
                            BLOCK_WORDS,
                        )),
                        10 => Some((BLK10_MIRROR_OFF, BLOCK_WORDS)),
                        _ => None,
                    };
                    // (`1 << blk` fits u32 for every burnable block.)
                    if let Some((base, n)) = mirror
                        && wr_dis & (1 << blk) == 0
                    {
                        for i in 0..n {
                            self.regs[base + i] |= self.pgm_stage[i];
                        }
                    }
                }
                let _ = value & 1;
            }
            // clk_en / power control — ignored.
            EFUSE_CLK_OFF => {}
            // HMAC/DS key-block readout registers: read-only on silicon, but
            // writable here so a sketch can provision a test key.  Spans KEY0..5
            // (0x9C .. 0x9C + 6*0x20).
            w if (EFUSE_KEY_BASE_OFF
                ..EFUSE_KEY_BASE_OFF + EFUSE_KEY_COUNT * EFUSE_KEY_STRIDE_OFF)
                .contains(&w) =>
            {
                self.regs[w] = value;
            }
            // RD_* data registers are read-only on silicon; PGM (burn)
            // registers are not modeled — firmware burn flows
            // (esp_efuse_batch_write) silently no-op instead of programming
            // fuses. Drop all other writes so the modeled MAC is preserved.
            _ => {
                let _ = w;
            }
        }
    }
}
