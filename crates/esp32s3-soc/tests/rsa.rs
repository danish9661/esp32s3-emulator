//! RSA peripheral model tests.
//!
//! Known-answer: 256-bit RSA encryption `c = m^e mod n` where
//! n = 16136940193400338063 * 16516496610031632579, e = 65537,
//! m = 123456789012345678901234567890, computed with Python's `pow`.
//! Operands are little-endian 32-bit limb arrays (as the esp-idf driver
//! writes them).

#![allow(clippy::identity_op)]

use esp32s3_soc::rsa::Rsa;

fn modexp(m: &[u32], x: &[u32], y: &[u32]) -> Vec<u32> {
    let mut rsa = Rsa::default();
    let n = m.len();
    // esp-idf writes LENGTH = nwords - 1.
    rsa.write32(0x804, (n as u32) - 1);
    for (i, &w) in m.iter().enumerate() {
        rsa.write32(0x000 + (i as u32) * 4, w);
    }
    for (i, &w) in x.iter().enumerate() {
        rsa.write32(0x600 + (i as u32) * 4, w);
    }
    for (i, &w) in y.iter().enumerate() {
        rsa.write32(0x400 + (i as u32) * 4, w);
    }
    rsa.write32(0x80C, 1);
    let mut z = Vec::new();
    for i in 0..n {
        z.push(rsa.read32(0x200 + (i as u32) * 4));
    }
    z
}

#[test]
fn rsa_256_bit_encrypt_known_answer() {
    let n = [
        956632813, 3019711633, 2859937330, 3364027506, 0, 0, 0, 0,
    ];
    let msg = [
        1312754386, 3279151342, 2397638646, 1, 0, 0, 0, 0,
    ];
    let exp = [65537, 0, 0, 0, 0, 0, 0, 0];
    let expected = [
        1593044933, 2628351229, 1762206883, 618803795, 0, 0, 0, 0,
    ];
    assert_eq!(modexp(&n, &msg, &exp), expected);
}

#[test]
fn rsa_small_modexp() {
    // 7^13 mod 19 = 7 (hand-computed). Two words to exercise the path.
    let m = [19, 0];
    let x = [7, 0];
    let y = [13, 0];
    assert_eq!(modexp(&m, &x, &y), [7, 0]);
}

#[test]
fn rsa_mod_mult_and_clear_interrupt() {
    let mut rsa = Rsa::default();
    // nwords = 1 -> n = 2 words.
    rsa.write32(0x804, 0);
    // modulus = 1000, base X = 123, exponent Y = 45 -> 123*45 mod 1000 = 535
    rsa.write32(0x000, 1000);
    rsa.write32(0x600, 123);
    rsa.write32(0x400, 45);
    rsa.write32(0x810, 1); // MOD_MULT_START
    assert!(rsa.int_pending());
    assert_eq!(rsa.read32(0x200), 535);
    rsa.write32(0x81C, 1); // CLEAR_INTERRUPT
    assert!(!rsa.int_pending());
}
