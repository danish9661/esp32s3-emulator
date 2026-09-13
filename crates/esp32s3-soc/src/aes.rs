//! ESP32-S3 AES acceleration peripheral (`DR_REG_AES_BASE = 0x6003_A000`).
//!
//! The esp-idf AES driver (`esp_aes_process_dma`) feeds the plaintext through
//! the GDMA `out` channel (`SOC_GDMA_TRIG_PERIPH_AES0`, `peri_sel = 6`) into the
//! message (TEXT_IN) registers at `AES_TEXT_IN_BASE` (offset 0x20); the engine
//! transforms the block and the ciphertext is read back through the GDMA `in`
//! channel from `AES_TEXT_OUT_BASE` (offset 0x30). `AES_MODE` (offset 0x40)
//! selects algorithm + direction (`esp_aes` LL: `(decrypt?4:0) + key_bytes/8 -
//! 2`, so 0/1/2 = AES-128/192/256 encrypt, +4 = decrypt); `AES_TRIGGER`(0x48)
//! starts the transform and `AES_STATE`(0x4c) reads idle (0) — the computation
//! is modeled synchronously.
//!
//! ECB is fully implemented (the validated path); CBC/CTR/CFB/OFB/XTS chaining
//! is implemented via the `chain`/`iv` state below (validated by the AES-CBC
//! and AES-XTS battery sketches) — the old "behaves as ECB" note was stale.

use alloc::vec;
use alloc::vec::Vec;

const S: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

const SINV: [u8; 256] = [
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d,
];

fn xtime(a: u8) -> u8 {
    let t = a << 1;
    if a & 0x80 != 0 { t ^ 0x1b } else { t }
}

fn gmul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0u8;
    for _ in 0..8 {
        if b & 1 != 0 {
            p ^= a;
        }
        let hi = a & 0x80;
        a <<= 1;
        if hi != 0 {
            a ^= 0x1b;
        }
        b >>= 1;
    }
    p
}

/// Expand the cipher key into the round-key words (returns `4 * (nr + 1)`
/// words, nr = nk + 6).
fn key_expansion(key: &[u8], nk: usize) -> Vec<u32> {
    let nr = nk + 6;
    let mut w = vec![0u32; 4 * (nr + 1)];
    for i in 0..nk {
        w[i] = ((key[4 * i] as u32) << 24)
            | ((key[4 * i + 1] as u32) << 16)
            | ((key[4 * i + 2] as u32) << 8)
            | (key[4 * i + 3] as u32);
    }
    let mut rcon = 1u8;
    for i in nk..(4 * (nr + 1)) {
        let mut temp = w[i - 1];
        if i % nk == 0 {
            temp = temp.rotate_left(8); // RotWord
            let b0 = S[((temp >> 24) & 0xFF) as usize];
            let b1 = S[((temp >> 16) & 0xFF) as usize];
            let b2 = S[((temp >> 8) & 0xFF) as usize];
            let b3 = S[(temp & 0xFF) as usize];
            temp = ((b0 as u32) << 24) | ((b1 as u32) << 16) | ((b2 as u32) << 8) | (b3 as u32);
            temp ^= (rcon as u32) << 24;
            rcon = xtime(rcon);
        } else if nk > 6 && i % nk == 4 {
            let b0 = S[((temp >> 24) & 0xFF) as usize];
            let b1 = S[((temp >> 16) & 0xFF) as usize];
            let b2 = S[((temp >> 8) & 0xFF) as usize];
            let b3 = S[(temp & 0xFF) as usize];
            temp = ((b0 as u32) << 24) | ((b1 as u32) << 16) | ((b2 as u32) << 8) | (b3 as u32);
        }
        w[i] = w[i - nk] ^ temp;
    }
    w
}

/// AES state is column-major: `state[r + 4*c]` (r = row, c = column).
fn add_round_key(state: &mut [u8; 16], w: &[u32], round: usize) {
    for c in 0..4 {
        let word = w[round * 4 + c];
        for r in 0..4 {
            state[r + 4 * c] ^= ((word >> (24 - 8 * r)) & 0xFF) as u8;
        }
    }
}

fn sub_bytes(state: &mut [u8; 16]) {
    for b in state.iter_mut() {
        *b = S[*b as usize];
    }
}

fn inv_sub_bytes(state: &mut [u8; 16]) {
    for b in state.iter_mut() {
        *b = SINV[*b as usize];
    }
}

fn shift_rows(state: &mut [u8; 16]) {
    let s = *state;
    for r in 0..4 {
        for c in 0..4 {
            state[r + 4 * c] = s[r + 4 * ((c + r) % 4)];
        }
    }
}

fn inv_shift_rows(state: &mut [u8; 16]) {
    let s = *state;
    for r in 0..4 {
        for c in 0..4 {
            state[r + 4 * c] = s[r + 4 * ((c + 4 - r) % 4)];
        }
    }
}

fn mix_columns(state: &mut [u8; 16]) {
    for c in 0..4 {
        let s0 = state[4 * c];
        let s1 = state[1 + 4 * c];
        let s2 = state[2 + 4 * c];
        let s3 = state[3 + 4 * c];
        state[4 * c] = gmul(s0, 2) ^ gmul(s1, 3) ^ s2 ^ s3;
        state[1 + 4 * c] = s0 ^ gmul(s1, 2) ^ gmul(s2, 3) ^ s3;
        state[2 + 4 * c] = s0 ^ s1 ^ gmul(s2, 2) ^ gmul(s3, 3);
        state[3 + 4 * c] = gmul(s0, 3) ^ s1 ^ s2 ^ gmul(s3, 2);
    }
}

fn inv_mix_columns(state: &mut [u8; 16]) {
    for c in 0..4 {
        let s0 = state[4 * c];
        let s1 = state[1 + 4 * c];
        let s2 = state[2 + 4 * c];
        let s3 = state[3 + 4 * c];
        state[4 * c] = gmul(s0, 14) ^ gmul(s1, 11) ^ gmul(s2, 13) ^ gmul(s3, 9);
        state[1 + 4 * c] = gmul(s0, 9) ^ gmul(s1, 14) ^ gmul(s2, 11) ^ gmul(s3, 13);
        state[2 + 4 * c] = gmul(s0, 13) ^ gmul(s1, 9) ^ gmul(s2, 14) ^ gmul(s3, 11);
        state[3 + 4 * c] = gmul(s0, 11) ^ gmul(s1, 13) ^ gmul(s2, 9) ^ gmul(s3, 14);
    }
}

fn aes_encrypt_block(input: &[u8; 16], w: &[u32], nr: usize) -> [u8; 16] {
    let mut state = *input;
    add_round_key(&mut state, w, 0);
    for rnd in 1..nr {
        sub_bytes(&mut state);
        shift_rows(&mut state);
        mix_columns(&mut state);
        add_round_key(&mut state, w, rnd);
    }
    sub_bytes(&mut state);
    shift_rows(&mut state);
    add_round_key(&mut state, w, nr);
    state
}

fn aes_decrypt_block(input: &[u8; 16], w: &[u32], nr: usize) -> [u8; 16] {
    let mut state = *input;
    add_round_key(&mut state, w, nr);
    for rnd in (1..nr).rev() {
        inv_shift_rows(&mut state);
        inv_sub_bytes(&mut state);
        add_round_key(&mut state, w, rnd);
        inv_mix_columns(&mut state);
    }
    inv_shift_rows(&mut state);
    inv_sub_bytes(&mut state);
    add_round_key(&mut state, w, 0);
    state
}

/// AES-256-CBC decrypt (PKCS#7-free: `data` length must be a multiple of 16).
/// Reused by the DS peripheral to decrypt the protected RSA key parameters.
pub(crate) fn aes256_cbc_decrypt(key: &[u8; 32], iv: &[u8; 16], data: &[u8]) -> Vec<u8> {
    let w = key_expansion(key, 8);
    let mut out = Vec::with_capacity(data.len());
    let mut prev = *iv;
    for chunk in data.chunks_exact(16) {
        let mut block = [0u8; 16];
        block.copy_from_slice(chunk);
        let dec = aes_decrypt_block(&block, &w, 14);
        let mut pt = [0u8; 16];
        for i in 0..16 {
            pt[i] = dec[i] ^ prev[i];
        }
        out.extend_from_slice(&pt);
        prev = block;
    }
    out
}

/// XTS-AES-128 tweakable block cipher (flash-encryption primitive).
/// Key split matches mbedtls (`mbedtls_aes_xts_setkey_enc(key, 256)`):
/// K1 = key[0..16] (data), K2 = key[16..32] (tweak). Tweak T = E_K2(IV);
/// out = E_K1(in ^ T) ^ T (encrypt) or the inverse. The flash pipeline
/// uses IV = LE128(absolute flash byte offset of the 16-byte block), so
/// every block is independently addressable (random-access, like IEEE
/// P1619 single-block units). This is a fixture convention — no firmware
/// observes it (the transparent HW hides it); IDF uses address-derived
/// IVs the same way.
pub(crate) fn xts_key(key: &[u8; 32]) -> (Vec<u32>, Vec<u32>) {
    (key_expansion(&key[..16], 4), key_expansion(&key[16..], 4))
}

/// XTS tweak for an IV: T = E_K2(IV).
pub(crate) fn xts_tweak(k2w: &[u32], iv: &[u8; 16]) -> [u8; 16] {
    let mut ivb = [0u8; 16];
    ivb.copy_from_slice(iv);
    aes_encrypt_block(&ivb, k2w, 10)
}

/// One XTS block with a ready tweak: out = E_K1(in ^ T) ^ T (or inverse).
pub(crate) fn xts_block(k1w: &[u32], t: &[u8; 16], blk: &[u8; 16], encrypt: bool) -> [u8; 16] {
    let mut x = [0u8; 16];
    for i in 0..16 {
        x[i] = blk[i] ^ t[i];
    }
    x = if encrypt {
        aes_encrypt_block(&x, k1w, 10)
    } else {
        aes_decrypt_block(&x, k1w, 10)
    };
    for i in 0..16 {
        x[i] ^= t[i];
    }
    x
}

/// `esp_gf128mul_x_ble` (mbedtls tweak chaining): multiply the LE128
/// value by x (shift left one bit; reduce an overflow out of bit 127 with
/// 0x87 folded into the low word). Test-anchored (the firmware XTS KAT
/// below chains through it); no production path needs it because flash
/// blocks use directly-computed tweaks (random access).
#[cfg(test)]
pub(crate) fn gf128_x_ble(t: &mut [u8; 16]) {
    let mut lo = u64::from_le_bytes([t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]]);
    let mut hi = u64::from_le_bytes([t[8], t[9], t[10], t[11], t[12], t[13], t[14], t[15]]);
    let carry = hi >> 63;
    hi = (hi << 1) | (lo >> 63);
    lo <<= 1;
    if carry != 0 {
        lo ^= 0x87;
    }
    t[0..8].copy_from_slice(&lo.to_le_bytes());
    t[8..16].copy_from_slice(&hi.to_le_bytes());
}

/// 128-bit LE tweak IV for the absolute flash byte offset of a 16-byte
/// block (fixture convention: full offset in the low word).
pub(crate) fn flash_xts_iv(byte_off: u32) -> [u8; 16] {
    let mut iv = [0u8; 16];
    iv[0..4].copy_from_slice(&byte_off.to_le_bytes());
    iv
}

/// Decrypt one 16-byte flash block at absolute offset `byte_off` (16-byte
/// aligned) with the eFuse XTS key. Physically-erased blocks (all 0xFF)
/// read back 0xFF: that is what firmware observes on silicon (every
/// empty-check in NVS/otadata/partition code depends on it), so the model
/// bypasses the cipher there rather than returning decrypt(FF) garbage.
pub(crate) fn flash_xts_decrypt(key: &[u8; 32], byte_off: u32, blk: &[u8; 16]) -> [u8; 16] {
    if blk == &[0xFF; 16] {
        return [0xFF; 16];
    }
    let (k1w, k2w) = xts_key(key);
    let t = xts_tweak(&k2w, &flash_xts_iv(byte_off));
    xts_block(&k1w, &t, blk, false)
}

/// Encrypt one 16-byte flash block (program path). No erased-bypass:
/// decrypt(encrypt(x)) == x round-trips regardless (the bypass only fires
/// on physically-stored 0xFF, which real ciphertext never collides with).
pub(crate) fn flash_xts_encrypt(key: &[u8; 32], byte_off: u32, blk: &[u8; 16]) -> [u8; 16] {
    let (k1w, k2w) = xts_key(key);
    let t = xts_tweak(&k2w, &flash_xts_iv(byte_off));
    xts_block(&k1w, &t, blk, true)
}

/// AES peripheral register-block base (`DR_REG_AES_BASE`, soc/reg_base.h).
pub const AES_BASE: u32 = 0x6003_A000;
/// AES ciphertext output registers (source for the GDMA `in` channel).
pub const AES_TEXT_OUT_BASE: u32 = AES_BASE + 0x30;

#[derive(Default)]
pub struct Aes {
    key: [u8; 32],
    mode: u32,
    block_mode: u32,
    text_in: [u8; 16],
    /// Set on direct TEXT_IN register writes, cleared by any transform.
    /// Distinguishes a fresh register-poked block from stale TEXT_IN bytes
    /// left over after a GDMA-fed DMA transfer (see TRIGGER below).
    text_in_fresh: bool,
    text_out: [u8; 16],
    /// IV register file (AES_IV_BASE + 0x50, 4 LE words). Written per
    /// operation by the driver.
    iv: [u8; 16],
    /// Chaining state across blocks (CBC feedback / OFB-CFB shift / CTR
    /// counter). Reset by IV-register writes (each driver operation sets
    /// its IV first), then advanced per block — including across the
    /// separate GDMA-descriptor `transform()` calls of one transfer.
    chain: [u8; 16],
    /// Staged DMA input bytes (a transfer may span descriptors).
    in_buf: Vec<u8>,
    /// This call's output bytes (served to the GDMA `in` walk / TEXT_OUT).
    out_buf: Vec<u8>,
    in_idx: usize,
    /// AES engine state as read from `AES_STATE` (0x4c): 0 = idle, 1 = busy,
    /// 2 = done. The esp-idf AES driver (`aes_hal_wait_done`) spins until the
    /// state reads DONE, so a synchronous transform must leave the state at 2.
    aes_state: u8,
    // AES DMA-done interrupt: INT_RAW (set when a transform completes), INT_ENA
    // (driver-controlled). INT_ST = INT_RAW & INT_ENA. The esp-idf AES driver
    // (`esp_aes_intr_alloc`) registers its completion ISR on the AES peripheral
    // interrupt source (`ETS_AES_INTR_SOURCE`, source 77), NOT the GDMA
    // interrupt — so we must raise INT_RAW when a block finishes so the driver's
    // `op_complete_sem` is given.
    int_raw: u32,
    int_ena: u32,
}

/// AES interrupt register offsets (`soc/hwcrypto_reg.h`).
const AES_INT_RAW_REG: u32 = 0xA4;
const AES_INT_ST_REG: u32 = 0xA8;
const AES_INT_CLR_REG: u32 = 0xAC;
const AES_INT_ENA_REG: u32 = 0xB0;
/// Bit raised in INT_RAW when a DMA (GDMA) transform completes.
const AES_DMA_DONE_INT: u32 = 1 << 0;

impl Aes {
    pub fn new() -> Self {
        Self::default()
    }

    /// AES peripheral interrupt is pending when RAW & ENA is non-zero.
    pub fn int_pending(&self) -> bool {
        self.int_raw & self.int_ena != 0
    }

    /// Debug: raw + enabled AES interrupt state.
    pub fn debug_int(&self) -> (u32, u32) {
        (self.int_raw, self.int_ena)
    }

    /// Read a single ciphertext byte from the last transform (for the GDMA
    /// `in` channel to copy into DRAM).
    pub fn text_out_byte(&self, i: usize) -> u8 {
        self.text_out[i % 16]
    }

    /// Read one output word at byte offset `k` (for the GDMA `in` walks,
    /// which stream the whole transfer buffer, possibly across repeated or
    /// overlapping walks — hence positional, not cursor-based). Past the end
    /// reads zero.
    pub fn out_word_at(&self, k: u32) -> u32 {
        let b = |j: usize| {
            if j < self.out_buf.len() {
                self.out_buf[j]
            } else {
                0
            }
        };
        let j = k as usize;
        (b(j) as u32)
            | ((b(j + 1) as u32) << 8)
            | ((b(j + 2) as u32) << 16)
            | ((b(j + 3) as u32) << 24)
    }

    /// Append one plaintext byte (GDMA `out` channel feed, LSB-first per word).
    pub fn feed_text_in_byte(&mut self, b: u8) {
        self.text_in[self.in_idx % 16] = b;
        self.in_idx += 1;
        self.in_buf.push(b);
    }

    /// XOR two blocks.
    fn xor16(a: &[u8], b: &[u8; 16]) -> [u8; 16] {
        let mut o = [0u8; 16];
        for i in 0..16 {
            o[i] = a[i] ^ b[i];
        }
        o
    }

    fn nk(&self) -> usize {
        match self.mode & 0x3 {
            0 => 4,
            1 => 6,
            _ => 8,
        }
    }

    fn decrypt(&self) -> bool {
        self.mode & 0x4 != 0
    }

    /// Run the transform on the staged input. In DMA mode the GDMA `out`
    /// walk feeds whole descriptor buffers before each call, so this
    /// processes every complete block with the configured block mode
    /// (ECB=0, CBC=1, OFB=2, CTR=3, CFB8=4, CFB128=5; GCM=6 falls back to
    /// ECB, see module docs), chaining across calls via `chain`. A
    /// non-multiple tail stays staged for the next descriptor. Direct
    /// register pokes (no staged bytes) transform the TEXT_IN block alone.
    pub fn transform(&mut self) {
        let nk = self.nk();
        let nr = nk + 6;
        let w = key_expansion(&self.key[..nk * 4], nk);
        let mut input = core::mem::take(&mut self.in_buf);
        if input.is_empty() {
            input.extend_from_slice(&self.text_in);
        }
        let nblocks = input.len() / 16;
        let mut out = Vec::with_capacity(nblocks * 16);
        self.text_in_fresh = false;
        let dec = self.decrypt();
        for b in 0..nblocks {
            let blk = &input[b * 16..b * 16 + 16];
            let o: [u8; 16] = match (self.block_mode & 0x7, dec) {
                (1, false) => {
                    // CBC encrypt: C = E(P ^ C_prev).
                    let c = aes_encrypt_block(&Self::xor16(blk, &self.chain), &w, nr);
                    self.chain.copy_from_slice(&c);
                    c
                }
                (1, true) => {
                    // CBC decrypt: P = D(C) ^ C_prev.
                    let p = aes_decrypt_block(&blk.try_into().unwrap_or([0u8; 16]), &w, nr);
                    let o = Self::xor16(&p, &self.chain);
                    self.chain.copy_from_slice(blk);
                    o
                }
                (2, _) => {
                    // OFB: S = E(chain); out = in ^ S; chain = S.
                    let s = aes_encrypt_block(&self.chain, &w, nr);
                    let o = Self::xor16(blk, &s);
                    self.chain.copy_from_slice(&s);
                    o
                }
                (3, _) => {
                    // CTR: S = E(chain); out = in ^ S; the 128-bit counter
                    // advances in its low word (bytes [12..16] big-endian,
                    // wrapping without carry — the layout standard counters
                    // use, pinned by the NIST CTR KAT below).
                    let s = aes_encrypt_block(&self.chain, &w, nr);
                    let o = Self::xor16(blk, &s);
                    let c = u32::from_be_bytes(self.chain[12..16].try_into().unwrap_or([0u8; 4]))
                        .wrapping_add(1);
                    self.chain[12..16].copy_from_slice(&c.to_be_bytes());
                    o
                }
                (4, _) => {
                    // CFB8: per byte S = E(chain); out = in ^ S[0].
                    let mut o = [0u8; 16];
                    for i in 0..16 {
                        let s = aes_encrypt_block(&self.chain, &w, nr);
                        o[i] = blk[i] ^ s[0];
                        self.chain.copy_within(1.., 0);
                        self.chain[15] = if dec { blk[i] } else { o[i] };
                    }
                    o
                }
                (5, _) => {
                    // CFB128: S = E(chain); out = in ^ S; chain shifts in C.
                    let s = aes_encrypt_block(&self.chain, &w, nr);
                    let o = Self::xor16(blk, &s);
                    if dec {
                        self.chain.copy_from_slice(blk);
                    } else {
                        self.chain.copy_from_slice(&o);
                    }
                    o
                }
                _ => {
                    if dec {
                        aes_decrypt_block(&blk.try_into().unwrap_or([0u8; 16]), &w, nr)
                    } else {
                        aes_encrypt_block(&blk.try_into().unwrap_or([0u8; 16]), &w, nr)
                    }
                }
            };
            out.extend_from_slice(&o);
        }
        // Streaming remainder stays staged for the next descriptor.
        self.in_buf = input[nblocks * 16..].to_vec();
        // Mirror the last block for direct TEXT_OUT readback compat.
        if nblocks > 0 {
            self.text_out.copy_from_slice(&out[out.len() - 16..]);
        }
        self.out_buf = out;
        self.in_idx = 0;
        self.aes_state = 2; // DONE: the driver's `aes_hal_wait_done` spins until this.
        // A DMA-driven transform has completed: raise the AES DMA-done
        // interrupt so the esp-idf AES driver's completion ISR
        // (`esp_aes_complete_isr`, registered on the AES peripheral interrupt
        // source 77) is invoked and gives `op_complete_sem`.
        self.int_raw |= AES_DMA_DONE_INT;
    }

    pub fn read32(&self, off: u32) -> u32 {
        match off {
            0x00..=0x1C => {
                // KEY readback (bytes stored LSB-first per 32-bit word).
                let i = (off / 4) as usize;
                if i < 8 {
                    (self.key[i * 4] as u32)
                        | ((self.key[i * 4 + 1] as u32) << 8)
                        | ((self.key[i * 4 + 2] as u32) << 16)
                        | ((self.key[i * 4 + 3] as u32) << 24)
                } else {
                    0
                }
            }
            0x20..=0x2C => {
                // TEXT_IN readback (unused by the driver).
                let i = ((off - 0x20) / 4) as usize;
                (self.text_in[i * 4] as u32)
                    | ((self.text_in[i * 4 + 1] as u32) << 8)
                    | ((self.text_in[i * 4 + 2] as u32) << 16)
                    | ((self.text_in[i * 4 + 3] as u32) << 24)
            }
            0x40 => self.mode,
            0x4c => (self.aes_state as u32) & 0x3, // STATE: idle/busy/done
            0x50..=0x5C => {
                // IV readback (LE words). Written per operation by the
                // driver; also the chaining reset point (see write32).
                let i = ((off - 0x50) / 4) as usize;
                (self.iv[i * 4] as u32)
                    | ((self.iv[i * 4 + 1] as u32) << 8)
                    | ((self.iv[i * 4 + 2] as u32) << 16)
                    | ((self.iv[i * 4 + 3] as u32) << 24)
            }
            0x94 => self.block_mode,
            AES_INT_RAW_REG => self.int_raw,
            AES_INT_ST_REG => self.int_raw & self.int_ena,
            AES_INT_ENA_REG => self.int_ena,
            // TEXT_OUT port (must come after the single-word regs above,
            // whose offsets it would otherwise shadow): the GDMA `in` walk
            // reads ascending words (one FIFO port on silicon — the address
            // increment is the walk's fiction). Serves this call's output.
            o if (0x30..0x90).contains(&o) => {
                let i = ((o - 0x30) / 4) as usize;
                let b = |j: usize| {
                    if j < self.out_buf.len() {
                        self.out_buf[j]
                    } else {
                        0
                    }
                };
                (b(i * 4) as u32)
                    | ((b(i * 4 + 1) as u32) << 8)
                    | ((b(i * 4 + 2) as u32) << 16)
                    | ((b(i * 4 + 3) as u32) << 24)
            }
            _ => 0,
        }
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        match off {
            0x00..=0x1C => {
                // KEY write (32-bit word -> 4 key bytes, LSB first).
                let i = (off / 4) as usize;
                if i < 8 {
                    self.key[i * 4] = value as u8;
                    self.key[i * 4 + 1] = (value >> 8) as u8;
                    self.key[i * 4 + 2] = (value >> 16) as u8;
                    self.key[i * 4 + 3] = (value >> 24) as u8;
                }
            }
            0x20..=0x2C => {
                // TEXT_IN direct write (32-bit word -> 4 bytes, LSB first).
                let i = ((off - 0x20) / 4) as usize;
                if i < 4 {
                    self.text_in[i * 4] = value as u8;
                    self.text_in[i * 4 + 1] = (value >> 8) as u8;
                    self.text_in[i * 4 + 2] = (value >> 16) as u8;
                    self.text_in[i * 4 + 3] = (value >> 24) as u8;
                    self.text_in_fresh = true;
                }
            }
            0x30..=0x3C => {
                // TEXT_OUT is read-only.
            }
            0x40 => self.mode = value & 0x7,
            0x48 => {
                // AES_TRIGGER (TRM AES_TRIGGER_REG): transform synchronously,
                // leaving DONE so `aes_hal_wait_done` exits. Transform only
                // with DMA-staged bytes or a fresh register-poked TEXT_IN
                // block; otherwise no-op. The esp-idf DMA driver writes
                // TRIGGER after starting the GDMA transfer, and re-encrypting
                // the already-consumed TEXT_IN mirror with the advanced chain
                // would hand the driver a doubly-encrypted block.
                self.aes_state = 1;
                if !self.in_buf.is_empty() || self.text_in_fresh {
                    self.transform();
                }
                self.aes_state = 2;
            }
            0x4c => {}
            0x90 => {} // AES_DMA_ENABLE: ignored (synchronous model)
            0x94 => self.block_mode = value,
            0x50..=0x5C => {
                // (merged into the IV arm below; kept for clarity)
                let i = ((off - 0x50) / 4) as usize;
                if i < 4 {
                    self.iv[i * 4] = value as u8;
                    self.iv[i * 4 + 1] = (value >> 8) as u8;
                    self.iv[i * 4 + 2] = (value >> 16) as u8;
                    self.iv[i * 4 + 3] = (value >> 24) as u8;
                    self.chain.copy_from_slice(&self.iv);
                }
            }

            AES_INT_ENA_REG => self.int_ena = value,
            AES_INT_CLR_REG => {
                // Writing any bit clears the corresponding raw interrupt.
                self.int_raw &= !value;
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn xts_block1_with_explicit_tweak() {
        let key = hex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
        let mut key32 = [0u8; 32];
        key32.copy_from_slice(&key);
        let pt1 = hex("112233445566778899aabbccddeeff00");
        let mut b = [0u8; 16];
        b.copy_from_slice(&pt1);
        let t1 = hex("da4761f21dd8a3d9007cbe60379fe7b1");
        let mut tt = [0u8; 16];
        tt.copy_from_slice(&t1);
        let (k1w, _) = xts_key(&key32);
        let ct = xts_block(&k1w, &tt, &b, true);
        assert_eq!(ct, hex("0f46d50a7bad5aa2a36c3a14bb4617d5")[..]);
    }

    #[test]
    fn gf128_x_ble_advances_tweak() {
        let t = hex("eda330f90eecd16c003e5fb09bcff358");
        let mut tt = [0u8; 16];
        tt.copy_from_slice(&t);
        gf128_x_ble(&mut tt);
        assert_eq!(tt, hex("da4761f21dd8a3d9007cbe60379fe7b1")[..]);
    }

    /// Host XTS matches the firmware-proven vector (`esp32s3_aes` sketch:
    /// `mbedtls_aes_crypt_xts`, key 00..1f, 2 blocks, zero tweak, chained
    /// via `esp_gf128mul_x_ble`). This ties the flash pipeline's cipher to
    /// the silicon tweak convention end to end.
    #[test]
    fn xts_matches_firmware_kat() {
        let key = hex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
        let pt = hex("00112233445566778899aabbccddeeff112233445566778899aabbccddeeff00");
        let want = hex("171c69724dcf733f9aa6317d795153e40f46d50a7bad5aa2a36c3a14bb4617d5");
        let mut key32 = [0u8; 32];
        key32.copy_from_slice(&key);
        let (k1w, k2w) = xts_key(&key32);
        let mut t = xts_tweak(&k2w, &[0u8; 16]);
        let mut out = Vec::new();
        for blk in pt.chunks_exact(16) {
            let mut b = [0u8; 16];
            b.copy_from_slice(blk);
            out.extend_from_slice(&xts_block(&k1w, &t, &b, true));
            gf128_x_ble(&mut t);
        }
        assert_eq!(out, want);
        // Round-trip back through decrypt.
        let mut t = xts_tweak(&k2w, &[0u8; 16]);
        let mut back = Vec::new();
        for blk in out.chunks_exact(16) {
            let mut b = [0u8; 16];
            b.copy_from_slice(blk);
            back.extend_from_slice(&xts_block(&k1w, &t, &b, false));
            gf128_x_ble(&mut t);
        }
        assert_eq!(back, pt);
    }
}
