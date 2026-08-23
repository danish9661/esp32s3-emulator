//! AES unit tests: the AES-128/192/256 ECB cipher is exercised through the
//! register-level model (key + TEXT_IN writes, `AES_MODE` select, `AES_TRIGGER`
//! transform, `AES_TEXT_OUT` readback), asserting the FIPS-197 test vectors.

use esp32s3_soc::aes::Aes;

const AES_MODE: u32 = 0x40;
const AES_TRIGGER: u32 = 0x48;
const AES_TEXT_IN_BASE: u32 = 0x20;
const AES_TEXT_OUT_BASE: u32 = 0x30;

/// Run one AES-ECB block: `key`/`pt` are 16/24/32 bytes, `decrypt` selects
/// direction. Returns the 16 ciphertext bytes as four LE words.
fn run_ecb(key: &[u8], pt: &[u8; 16], decrypt: bool) -> [u32; 4] {
    assert!(key.len() == 16 || key.len() == 24 || key.len() == 32);
    let mut a = Aes::new();
    // Key: 4/6/8 words, stored LSB-first per 32-bit word.
    for (i, chunk) in key.chunks(4).enumerate() {
        let mut w = [0u8; 4];
        w[..chunk.len()].copy_from_slice(chunk);
        let val =
            (w[0] as u32) | ((w[1] as u32) << 8) | ((w[2] as u32) << 16) | ((w[3] as u32) << 24);
        a.write32((i as u32) * 4, val);
    }
    // Mode: 0/1/2 = AES-128/192/256 encrypt, +4 = decrypt.
    let nk = (key.len() / 8) as u32 - 2; // 16->0, 24->1, 32->2
    let mode = if decrypt { nk + 4 } else { nk };
    a.write32(AES_MODE, mode);
    // Plaintext: 4 words LSB-first.
    for i in 0..4 {
        let mut w = [0u8; 4];
        w.copy_from_slice(&pt[i * 4..i * 4 + 4]);
        let val =
            (w[0] as u32) | ((w[1] as u32) << 8) | ((w[2] as u32) << 16) | ((w[3] as u32) << 24);
        a.write32(AES_TEXT_IN_BASE + (i as u32) * 4, val);
    }
    a.write32(AES_TRIGGER, 1);
    let mut out = [0u32; 4];
    for i in 0..4 {
        out[i] = a.read32(AES_TEXT_OUT_BASE + (i as u32) * 4);
    }
    out
}

/// Reverse the 4 LE words into the canonical byte-string form for comparison.
fn words_to_hex(words: [u32; 4]) -> String {
    let mut s = String::new();
    for w in words.iter() {
        for b in 0..4 {
            s.push_str(&format!("{:02x}", (w >> (b * 8)) & 0xFF));
        }
    }
    s
}

#[test]
fn aes128_ecb_encrypt_fips_vector() {
    // FIPS-197 ECB-AES128 example: key=000102..0f, pt=001122..eeff.
    let key = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
    let pt = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
        0xff,
    ];
    let ct = run_ecb(&key, &pt, false);
    assert_eq!(
        words_to_hex(ct),
        "69c4e0d86a7b0430d8cdb78070b4c55a",
        "AES-128 ECB encrypt must match FIPS-197"
    );
}

#[test]
fn aes128_ecb_decrypt_roundtrip() {
    let key = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
    let pt = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
        0xff,
    ];
    let ct = run_ecb(&key, &pt, false);
    // Decrypt the ciphertext: must recover the plaintext.
    let mut ct_bytes = [0u8; 16];
    for i in 0..4 {
        for b in 0..4 {
            ct_bytes[i * 4 + b] = ((ct[i] >> (b * 8)) & 0xFF) as u8;
        }
    }
    let pt2 = run_ecb(&key, &ct_bytes, true);
    assert_eq!(words_to_hex(pt2), "00112233445566778899aabbccddeeff");
}

#[test]
fn aes256_ecb_encrypt_fips_vector() {
    // FIPS-197 ECB-AES256 example.
    let key = [
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
        0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
        0x1e, 0x1f,
    ];
    let pt = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
        0xff,
    ];
    let ct = run_ecb(&key, &pt, false);
    assert_eq!(
        words_to_hex(ct),
        "8ea2b7ca516745bfeafc49904b496089",
        "AES-256 ECB encrypt must match FIPS-197"
    );
}

#[test]
fn aes_state_reads_done_after_transform() {
    let key = [0u8; 16];
    let mut a = Aes::new();
    for i in 0..4 {
        a.write32((i as u32) * 4, 0);
    }
    a.write32(AES_MODE, 0);
    a.write32(AES_TEXT_IN_BASE, 0);
    a.write32(AES_TRIGGER, 1);
    // AES_STATE (0x4c) reads 2 (DONE) after a synchronous transform.
    assert_eq!(a.read32(0x4c) & 0x3, 2);
}
