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
    let n = [956632813, 3019711633, 2859937330, 3364027506, 0, 0, 0, 0];
    let msg = [1312754386, 3279151342, 2397638646, 1, 0, 0, 0, 0];
    let exp = [65537, 0, 0, 0, 0, 0, 0, 0];
    let expected = [1593044933, 2628351229, 1762206883, 618803795, 0, 0, 0, 0];
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

/// 1024-bit modexp matching the POKE sketch (e = 65537).
/// This should complete in < 1 second on the host.
#[test]
fn rsa_1024_bit_modexp_completes() {
    let n_le: [u32; 32] = [
        0xA523A5D3, 0xA9786480, 0x03BF875E, 0x967550AE, 0xB3451599, 0x7DE43165, 0x399CF136,
        0x8C6CD061, 0x1CD1E074, 0xC9328241, 0x7CFAAC9C, 0xBB93F615, 0x5A1FC254, 0x33C7C3B8,
        0x1439E5B8, 0xC22E870B, 0x0627AEC4, 0x0034D704, 0x52BE0923, 0x30BBCF60, 0xAD549151,
        0x8C088E24, 0x54E1B7B6, 0x588DD86C, 0xDDC8A1DA, 0xA413A1E1, 0x54E694AD, 0x721D9DE7,
        0x1D5D8D13, 0xDB42D0E6, 0x14827DD3, 0x32B78617,
    ];
    let m_le: [u32; 32] = [
        0xB040DAED, 0x6951C441, 0x62019962, 0x68DB5A3C, 0x1A77BD67, 0x2EDC22B3, 0xB2B305D8,
        0x1BD00E7E, 0x7E6A7A24, 0x81887009, 0x36AC9D9B, 0x5F2250B5, 0xA330FC5E, 0xEAE19B6B,
        0xDD91B111, 0xF1196333, 0x4A9B4577, 0x06774650, 0x5230DBDC, 0x21464164, 0x9FE96664,
        0x98C1529D, 0x14D615DD, 0xB9411B54, 0x4551231B, 0xB98E13B9, 0x46E64F0C, 0xC5BA1E2E,
        0xA42F7B77, 0x7369A004, 0x9C506A97, 0x2F12FDD8,
    ];
    let e_le: [u32; 32] = [
        0x00010001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
    ];
    let c_le: [u32; 32] = [
        0xDE8235AA, 0x37F6A577, 0xF7B11535, 0x9DD17789, 0x48C6A611, 0x24D5A885, 0x6D827593,
        0x9DDEAF9C, 0x5ABC8FCA, 0x1718941D, 0x700FD985, 0x7FA19955, 0x1F8CCA73, 0x44F0AE9C,
        0x2AF04D37, 0xF7EA542B, 0xB6F12FD9, 0x4F17A9A6, 0xF0ACB2C2, 0x4219242E, 0xF565B6CC,
        0x2E3A821C, 0xE7024A3A, 0x27DD9C90, 0xD33FC36B, 0xB8D28BAC, 0x7DBF0DC4, 0xDC2CE3A6,
        0xF03058E9, 0x5025D6D7, 0x5954EA72, 0x07A3BD15,
    ];
    let z = modexp(&n_le, &m_le, &e_le);
    assert_eq!(z, c_le);
}
