//! SHA unit tests: the SHA-256/SHA-1/SHA-224 compression is exercised through
//! the register-level model (mode select + GDMA-style byte feed + DMA_START +
//! H-register readback), asserting the known digests of "hello". SHA-384/512
//! use 128-byte blocks with a 128-bit big-endian bit length (FIPS 180-4) and
//! read back 12/16 H words (each 64-bit state word splits hi-half first,
//! every 32-bit word byte-swapped like the 256-bit modes).

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

/// Build the standard SHA-512-family padded 128-byte block for `msg`
/// (single block: msg || 0x80 || zeros || 128-bit big-endian bit length).
fn padded_block_128(msg: &[u8]) -> [u8; 128] {
    let mut b = [0u8; 128];
    b[..msg.len()].copy_from_slice(msg);
    b[msg.len()] = 0x80;
    let bits = (msg.len() as u128) * 8;
    b[112..128].copy_from_slice(&bits.to_be_bytes());
    b
}

/// Feed a full padded 128-byte block via `feed_byte`, set the mode, trigger
/// DMA_START, and return the digest words from the H registers.
fn digest_512(mode: u32, block: &[u8; 128], words: usize) -> Vec<u32> {
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
fn sha384_hello() {
    // SHA384("hello") = 59e1748777448c69de6b800d7a33bbfb9ff1b463e44354c3553bcdb9c666fa90125a3c79f90397bdf5f6a13de828684f
    let d = digest_512(3, &padded_block_128(b"hello"), 12);
    assert_eq!(
        d,
        vec![
            0x87_74_e1_59,
            0x69_8c_44_77,
            0x0d_80_6b_de,
            0xfb_bb_33_7a,
            0x63_b4_f1_9f,
            0xc3_54_43_e4,
            0xb9_cd_3b_55,
            0x90_fa_66_c6,
            0x79_3c_5a_12,
            0xbd_97_03_f9,
            0x3d_a1_f6_f5,
            0x4f_68_28_e8,
        ]
    );
}

#[test]
fn sha512_hello() {
    // SHA512("hello") = 9b71d224bd62f3785d96d46ad3ea3d73319bfbc2890caadae2dff72519673ca72323c3d99ba5c11d7c7acc6e14b8c5da0c4663475c2e5c3adef46f73bcdec043
    let d = digest_512(4, &padded_block_128(b"hello"), 16);
    assert_eq!(
        d,
        vec![
            0x24_d2_71_9b,
            0x78_f3_62_bd,
            0x6a_d4_96_5d,
            0x73_3d_ea_d3,
            0xc2_fb_9b_31,
            0xda_aa_0c_89,
            0x25_f7_df_e2,
            0xa7_3c_67_19,
            0xd9_c3_23_23,
            0x1d_c1_a5_9b,
            0x6e_cc_7a_7c,
            0xda_c5_b8_14,
            0x47_63_46_0c,
            0x3a_5c_2e_5c,
            0x73_6f_f4_de,
            0x43_c0_de_bc,
        ]
    );
}

/// Multi-block (200 bytes = two 128-byte SHA-512 blocks) via DMA_CONTINUE.
fn digest_512_multi(mode: u32, msg: &[u8], words: usize) -> Vec<u32> {
    let mut s = Sha::new();
    s.write32(0x00, mode);
    // First block + DMA_START, remainder + DMA_CONTINUE (driver convention).
    let mut it = msg.chunks(128);
    for &byte in it.next().unwrap_or(&[]).iter() {
        s.feed_byte(byte);
    }
    s.write32(0x1C, 1);
    for chunk in it {
        for &byte in chunk.iter() {
            s.feed_byte(byte);
        }
        s.write32(0x20, 1); // SHA_DMA_CONTINUE
    }
    (0..words)
        .map(|i| s.read32(0x40 + (i as u32) * 4))
        .collect()
}

#[test]
fn sha384_multiblock() {
    // 200 'a' bytes, padded by the test to two full 128-byte blocks.
    let mut msg = [b'a'; 200].to_vec();
    msg.push(0x80);
    while msg.len() % 128 != 112 {
        msg.push(0);
    }
    let bits = (200u128) * 8;
    msg.extend_from_slice(&bits.to_be_bytes());
    assert_eq!(msg.len(), 256);
    let d = digest_512_multi(3, &msg, 12);
    assert_eq!(
        d,
        vec![
            0xe9_b6_91_06,
            0x67_4b_61_78,
            0xb2_57_05_d6,
            0x53_dd_cd_a2,
            0x52_08_65_40,
            0xc6_21_fa_2e,
            0xa8_bf_db_24,
            0x6d_72_6e_ab,
            0x48_6b_58_5c,
            0xf2_09_7c_9c,
            0x4c_a6_09_41,
            0x48_1d_21_10,
        ]
    );
}

#[test]
fn sha512_multiblock() {
    let mut msg = [b'a'; 200].to_vec();
    msg.push(0x80);
    while msg.len() % 128 != 112 {
        msg.push(0);
    }
    let bits = (200u128) * 8;
    msg.extend_from_slice(&bits.to_be_bytes());
    assert_eq!(msg.len(), 256);
    let d = digest_512_multi(4, &msg, 16);
    assert_eq!(
        d,
        vec![
            0x9c_45_11_4b,
            0x22_2a_f5_33,
            0x78_36_82_ee,
            0x50_c1_14_27,
            0x09_c6_b2_a3,
            0xee_ac_e9_94,
            0x94_68_fe_17,
            0x89_67_3e_7a,
            0x68_76_1e_f3,
            0xda_92_45_39,
            0x7c_82_ef_7b,
            0xc4_88_ca_dd,
            0x4d_6e_f8_e6,
            0xe6_1a_ed_f7,
            0x3e_1f_a7_cb,
            0x9f_ee_fa_98,
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
