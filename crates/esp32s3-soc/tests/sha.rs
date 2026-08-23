//! SHA unit tests: the SHA-256/SHA-1/SHA-224 compression is exercised through
//! the register-level model (mode select + GDMA-style byte feed + DMA_START +
//! H-register readback), asserting the known digests of "hello".

use esp32s3_soc::sha::Sha;

/// Build the standard SHA-256/SHA-1/SHA-224 padded 64-byte block for `msg`
/// (single block: msg || 0x80 || zeros || 64-bit big-endian bit length).
fn padded_block(msg: &[u8]) -> [u8; 64] {
    let mut b = [0u8; 64];
    b[..msg.len()].copy_from_slice(msg);
    b[msg.len()] = 0x80;
    let bits = (msg.len() as u64) * 8;
    b[56..64].copy_from_slice(&bits.to_be_bytes());
    b
}

/// Feed a full padded block via `feed_byte`, set the mode, trigger DMA_START,
/// and return the digest words from the H registers.
fn digest(mode: u32, block: &[u8; 64], words: usize) -> Vec<u32> {
    let mut s = Sha::new();
    s.write32(0x00, mode);
    for &byte in block.iter() {
        s.feed_byte(byte);
    }
    s.write32(0x1C, 1); // SHA_DMA_START
    (0..words)
        .map(|i| s.read32(0x40 + (i as u32) * 4))
        .collect()
}

#[test]
fn sha256_hello() {
    // SHA256("hello") = 2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824
    let d = digest(2, &padded_block(b"hello"), 8);
    assert_eq!(
        d,
        vec![
            0xba4d_f22c,
            0x0e_a3_b0_5f,
            0x2a_3b_e8_26,
            0x9e_e2_b9_c5,
            0x5c_1e_16_1b,
            0x5e_42_a7_1f,
            0x62_33_04_73,
            0x24_98_8b_93,
        ]
    );
}

#[test]
fn sha1_hello() {
    // SHA1("hello") = aaf4c61ddcc5e8a2dabede0f3b482cd9aea9434d
    let d = digest(0, &padded_block(b"hello"), 5);
    assert_eq!(
        d,
        vec![
            0x1d_c6_f4_aa,
            0xa2_e8_c5_dc,
            0x0f_de_be_da,
            0xd9_2c_48_3b,
            0x4d_43_a9_ae,
        ]
    );
}

#[test]
fn sha224_hello() {
    // SHA224("hello") = ea09ae9cc6768c50fcee903ed054556e5bfc8347907f12598aa24193
    let d = digest(1, &padded_block(b"hello"), 7);
    assert_eq!(
        d,
        vec![
            0x9c_ae_09_ea,
            0x50_8c_76_c6,
            0x3e_90_ee_fc,
            0x6e_55_54_d0,
            0x47_83_fc_5b,
            0x59_12_7f_90,
            0x93_41_a2_8a,
        ]
    );
}

#[test]
fn busy_is_always_idle() {
    let mut s = Sha::new();
    s.write32(0x00, 2);
    s.feed_byte(0x80);
    s.write32(0x1C, 1);
    // SHA_BUSY (0x18) reads 0 in the synchronous model.
    assert_eq!(s.read32(0x18), 0);
    // Unmodeled register reads 0.
    assert_eq!(s.read32(0x200), 0);
}
