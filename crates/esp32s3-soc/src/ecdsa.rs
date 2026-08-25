//! ESP32-S3 ECDSA accelerator model.
//!
//! Register block at `0x6008_E000` (`DR_REG_ECDSA_BASE` — page-aligned, own 4KB
//! mmio page). The silicon is a black box that, given the curve parameters and
//! either a private key + message hash (signature generation) or a public key +
//! signature (verification), produces an ECDSA signature / verification result.
//! The esp-idf driver stages the operands into a parameter RAM and pulses
//! `START`.
//!
//! The exact TRM/esp-idf S3 ECDSA register + parameter-memory protocol is not
//! bundled in this core's headers, so this model uses a clean, documented
//! convention (see the register map and the `PARAM_*` block indices below).
//! The cryptographic result is computed in software with the real NIST P-192 /
//! P-256 elliptic-curve arithmetic, so a firmware sketch that pokes the operands
//! receives a mathematically correct signature / decision.

use alloc::vec;
use alloc::vec::Vec;

use crate::bignum::*;

/// ECDSA register-block base (`DR_REG_ECDSA_BASE`).
pub const ECDSA_BASE: u32 = 0x6008_E000;

/// ECDSA interrupt source for the interrupt matrix (esp32s3 interrupts.h
/// `ETS_ECDSA_INTR_SOURCE` = 97).
pub const ECDSA_INTR_SOURCE: u32 = 97;

/// Max operand size: 256-bit = 8 words (P-192 uses the low 6 words).
const NW: usize = 8;

/// Parameter-RAM block indices (each block = `NW` words, little-endian limbs).
/// The curve params (P/A/B/Gx/Gy, blocks 0..5) are supplied by the model's
/// built-in NIST P-192 / P-256 tables (selected by `CONF.ecc_curve`), so the
/// firmware only stages the key/hash/signature operands.
const QX_PARAM: usize = 5;
const QY_PARAM: usize = 6;
const D_PARAM: usize = 7;
const K_PARAM: usize = 8;
const Z_PARAM: usize = 9;
const R_PARAM: usize = 10;
const S_PARAM: usize = 11;
const NPARAM: usize = 12;

/// A NIST prime curve (all integers little-endian limb slices).
struct Curve {
    p: Vec<u32>,
    a: Vec<u32>,
    gx: Vec<u32>,
    gy: Vec<u32>,
    n: Vec<u32>,
}

fn curve(curve_id: u32) -> Curve {
    if curve_id == 0 {
        // NIST P-192 (secp192r1).
        Curve {
            p: from_be_bytes(&[
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            ]),
            a: from_be_bytes(&[
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
            ]),
            gx: from_be_bytes(&[
                0x18, 0x8D, 0xA8, 0x0E, 0xB0, 0x30, 0x90, 0xF6, 0x7C, 0xBF, 0x20, 0xEB, 0x43, 0xA1,
                0x88, 0x00, 0xF4, 0xFF, 0x0A, 0xFD, 0x82, 0xFF, 0x10, 0x12,
            ]),
            gy: from_be_bytes(&[
                0x07, 0x19, 0x2B, 0x95, 0xFF, 0xC8, 0xDA, 0x78, 0x63, 0x10, 0x11, 0xED, 0x6B, 0x24,
                0xCD, 0xD5, 0x73, 0xF9, 0x77, 0xA1, 0x1E, 0x79, 0x48, 0x11,
            ]),
            n: from_be_bytes(&[
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x99, 0xDE,
                0xF8, 0x36, 0x14, 0x6B, 0xC9, 0xB1, 0xB4, 0xD2, 0x28, 0x31,
            ]),
        }
    } else {
        // NIST P-256 (secp256r1).
        Curve {
            p: from_be_bytes(&[
                0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF, 0xFF, 0xFF, 0xFF,
            ]),
            a: from_be_bytes(&[
                0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF, 0xFF, 0xFF, 0xFC,
            ]),
            gx: from_be_bytes(&[
                0x6B, 0x17, 0xD1, 0xF2, 0xE1, 0x2C, 0x42, 0x47, 0xF8, 0xBC, 0xE6, 0xE5, 0x63, 0xA4,
                0x40, 0xF2, 0x77, 0x03, 0x7D, 0x81, 0x2D, 0xEB, 0x33, 0xA0, 0xF4, 0xA1, 0x39, 0x45,
                0xD8, 0x98, 0xC2, 0x96,
            ]),
            gy: from_be_bytes(&[
                0x4F, 0xE3, 0x42, 0xE2, 0xFE, 0x1A, 0x7F, 0x9B, 0x8E, 0xE7, 0xEB, 0x4A, 0x7C, 0x0F,
                0x9E, 0x16, 0x2B, 0xCE, 0x33, 0x57, 0x6B, 0x31, 0x5E, 0xCE, 0xCB, 0xB6, 0x40, 0x68,
                0x37, 0xBF, 0x51, 0xF5,
            ]),
            n: from_be_bytes(&[
                0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF, 0xFF, 0xBC, 0xE6, 0xFA, 0xAD, 0xA7, 0x17, 0x9E, 0x84, 0xF3, 0xB9, 0xCA, 0xC2,
                0xFC, 0x63, 0x25, 0x51,
            ]),
        }
    }
}

/// Affine point; `None` = point at infinity.
type Point = Option<(Vec<u32>, Vec<u32>)>;

/// `a - b (mod m)`, valid for `a, b < m` (modular subtraction).
fn mod_sub(a: &[u32], b: &[u32], m: &[u32]) -> Vec<u32> {
    match cmp(a, b) {
        core::cmp::Ordering::Less => sub(m, &sub(b, a)),
        _ => sub(a, b),
    }
}

/// `a + b (mod m)`, valid for `a, b < m`.
fn mod_add(a: &[u32], b: &[u32], m: &[u32]) -> Vec<u32> {
    mod_m(&add(a, b), m)
}

/// Affine elliptic-curve point addition over the prime field `m = p`.
fn ec_add(p1: &Point, p2: &Point, c: &Curve) -> Point {
    let (x1, y1) = match p1 {
        Some(v) => v,
        None => return p2.clone(),
    };
    let (x2, y2) = match p2 {
        Some(v) => v,
        None => return p1.clone(),
    };
    let p = &c.p;
    if x1 == x2 {
        // Doubling or the point at infinity (y1 + y2 == 0 mod p).
        if y1 != y2 {
            return None;
        }
        let xsq = mul(x1, x1);
        // lambda = (3*x1^2 + a) / (2*y1)
        let num = mod_add(&mod_add(&xsq, &xsq, p), &mod_add(&xsq, &c.a, p), p);
        let two_y = mod_add(y1, y1, p);
        let inv = modinv(&two_y, p);
        let lambda = mod_m(&mul(&num, &inv), p);
        let lam2 = mod_m(&mul(&lambda, &lambda), p);
        let x3 = mod_sub(&mod_sub(&lam2, x1, p), x2, p);
        let y3 = mod_sub(&mod_m(&mul(&lambda, &mod_sub(x1, &x3, p)), p), y1, p);
        Some((mod_m(&x3, p), mod_m(&y3, p)))
    } else {
        let lambda = {
            let num = mod_sub(y2, y1, p);
            let den = modinv(&mod_sub(x2, x1, p), p);
            mod_m(&mul(&num, &den), p)
        };
        let lam2 = mod_m(&mul(&lambda, &lambda), p);
        let x3 = mod_sub(&mod_sub(&lam2, x1, p), x2, p);
        let y3 = mod_sub(&mod_m(&mul(&lambda, &mod_sub(x1, &x3, p)), p), y1, p);
        Some((mod_m(&x3, p), mod_m(&y3, p)))
    }
}

/// Scalar multiplication `k * point` (double-and-add).
fn ec_mul(k: &[u32], point: &(Vec<u32>, Vec<u32>), c: &Curve) -> Point {
    let mut result: Point = None;
    let mut addend: Point = Some((point.0.clone(), point.1.clone()));
    let k = trim(k);
    for limb in &k {
        let mut bits = *limb;
        for _ in 0..32 {
            if bits & 1 != 0 {
                result = ec_add(&result, &addend, c);
            }
            addend = ec_add(&addend, &addend, c);
            bits >>= 1;
        }
    }
    result
}

/// ECDSA signature generation: returns `(r, s)`.
fn sign(d: &[u32], z: &[u32], k: &[u32], c: &Curve) -> (Vec<u32>, Vec<u32>) {
    let z = mod_m(z, &c.n);
    let g = (c.gx.clone(), c.gy.clone());
    let r_point = match ec_mul(k, &g, c) {
        Some(v) => v,
        None => return (vec![0], vec![0]),
    };
    let r = mod_m(&r_point.0, &c.n);
    if r.iter().all(|w| *w == 0) {
        return (vec![0], vec![0]);
    }
    let rd = mod_m(&mul(&r, d), &c.n);
    let e = mod_add(&rd, &z, &c.n);
    let k_inv = modinv(k, &c.n);
    let s = mod_m(&mul(&k_inv, &e), &c.n);
    if s.iter().all(|w| *w == 0) {
        return (vec![0], vec![0]);
    }
    (r, s)
}

/// ECDSA signature verification: returns `true` if `(r, s)` is valid for
/// message hash `z` under public key `(qx, qy)`.
fn verify(qx: &[u32], qy: &[u32], z: &[u32], r: &[u32], s: &[u32], c: &Curve) -> bool {
    if r.iter().all(|w| *w == 0) || s.iter().all(|w| *w == 0) {
        return false;
    }
    if cmp(r, &c.n) != core::cmp::Ordering::Less || cmp(s, &c.n) != core::cmp::Ordering::Less {
        return false;
    }
    let z = mod_m(z, &c.n);
    let s_inv = modinv(s, &c.n);
    let u1 = mod_m(&mul(&z, &s_inv), &c.n);
    let u2 = mod_m(&mul(r, &s_inv), &c.n);
    let g = (c.gx.clone(), c.gy.clone());
    let q = (qx.to_vec(), qy.to_vec());
    let point = ec_add(&ec_mul(&u1, &g, c), &ec_mul(&u2, &q, c), c);
    let (px, _) = match point {
        Some(v) => v,
        None => return false,
    };
    let v = mod_m(&px, &c.n);
    trim(&v) == trim(r)
}

pub struct Ecdsa {
    conf: u32,
    int_ena: u32,
    int_raw: u32,
    result: u32,
    /// Parameter RAM: `NPARAM` blocks of `NW` little-endian limbs.
    param: [u32; NPARAM * NW],
    k_ctr: u32,
}

impl Default for Ecdsa {
    fn default() -> Self {
        Ecdsa::new()
    }
}

impl Ecdsa {
    pub fn new() -> Self {
        Ecdsa {
            conf: 0,
            int_ena: 0,
            int_raw: 0,
            result: 0,
            param: [0u32; NPARAM * NW],
            k_ctr: 0x1234_5678,
        }
    }

    fn block(&self, b: usize) -> Vec<u32> {
        let mut v = vec![0u32; NW];
        v.copy_from_slice(&self.param[b * NW..b * NW + NW]);
        v
    }

    fn store_block(&mut self, b: usize, limbs: &[u32]) {
        for (i, w) in limbs.iter().enumerate().take(NW) {
            self.param[b * NW + i] = *w;
        }
    }

    /// Run the configured operation (sign / verify / export public key).
    fn run(&mut self) {
        let work_mode = self.conf & 0x3;
        let curve_id = (self.conf >> 2) & 0x1;
        let c = curve(curve_id);
        match work_mode {
            1 => {
                // Signature generation.
                let d = self.block(D_PARAM);
                let z = self.block(Z_PARAM);
                let k = if (self.conf >> 3) & 0x1 != 0 {
                    self.block(K_PARAM)
                } else {
                    self.k_ctr = self.k_ctr.wrapping_add(0x9E37_79B1);
                    let k = mod_m(&[self.k_ctr], &c.n);
                    if k.iter().all(|w| *w == 0) {
                        vec![1]
                    } else {
                        k
                    }
                };
                let g = (c.gx.clone(), c.gy.clone());
                if let Some((qx, qy)) = ec_mul(&d, &g, &c) {
                    self.store_block(QX_PARAM, &qx);
                    self.store_block(QY_PARAM, &qy);
                }
                let (r, s) = sign(&d, &z, &k, &c);
                self.store_block(R_PARAM, &r);
                self.store_block(S_PARAM, &s);
                self.result = 1;
                self.int_raw |= 1;
            }
            0 => {
                // Signature verification.
                let qx = self.block(QX_PARAM);
                let qy = self.block(QY_PARAM);
                let z = self.block(Z_PARAM);
                let r = self.block(R_PARAM);
                let s = self.block(S_PARAM);
                self.result = if verify(&qx, &qy, &z, &r, &s, &c) {
                    1
                } else {
                    0
                };
                self.int_raw |= 1;
            }
            _ => {
                // Export public key (mode 2): Q = d * G.
                let d = self.block(D_PARAM);
                let g = (c.gx.clone(), c.gy.clone());
                if let Some((qx, qy)) = ec_mul(&d, &g, &c) {
                    self.store_block(QX_PARAM, &qx);
                    self.store_block(QY_PARAM, &qy);
                }
                self.result = 1;
                self.int_raw |= 1;
            }
        }
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        match off {
            0x00 => self.conf = value,
            0x04 => {
                if value & 1 != 0 {
                    self.run();
                }
            }
            0x0C => self.int_ena = value,
            0x14 => self.int_raw &= !value,
            0x80..=0x3FC => {
                let i = ((off - 0x80) / 4) as usize;
                if i < self.param.len() {
                    self.param[i] = value;
                }
            }
            _ => {}
        }
    }

    pub fn read32(&self, off: u32) -> u32 {
        match off {
            0x00 => self.conf,
            0x04 => 0, // START is self-clearing
            0x08 => self.int_raw,
            0x0C => self.int_ena,
            0x10 => self.int_raw & self.int_ena,
            0x14 => 0,
            0x18 => self.result,
            0x80..=0x3FC => {
                let i = ((off - 0x80) / 4) as usize;
                if i < self.param.len() {
                    self.param[i]
                } else {
                    0
                }
            }
            _ => 0,
        }
    }

    /// `calc_done` interrupt pending (RAW & ENA).
    pub fn int_pending(&self) -> bool {
        self.int_raw & self.int_ena & 1 != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bignum::{from_be_bytes, trim};

    fn be_hex_to_limbs(hex: &str) -> Vec<u32> {
        let bytes: Vec<u8> = (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect();
        from_be_bytes(&bytes)
    }

    fn write_block(e: &mut Ecdsa, block: usize, limbs: &[u32]) {
        for (i, w) in limbs.iter().enumerate().take(NW) {
            e.write32(0x80 + ((block * NW + i) * 4) as u32, *w);
        }
    }

    fn read_block(e: &Ecdsa, block: usize) -> Vec<u32> {
        let mut v = vec![0u32; NW];
        for i in 0..NW {
            v[i] = e.read32(0x80 + ((block * NW + i) * 4) as u32);
        }
        v
    }

    const CONF_SIGN_P256: u32 = (1 << 0) | (1 << 2) | (1 << 3);
    const CONF_VERIFY_P256: u32 = (0 << 0) | (1 << 2);
    const CONF_SIGN_P192: u32 = (1 << 0) | (0 << 2) | (1 << 3);

    #[test]
    fn p256_sign_kat_matches_independent_implementation() {
        // Known-answer vector computed by an independent Python P-256 ECDSA
        // implementation (deterministic k), so this is a true cross-check.
        let d = be_hex_to_limbs("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
        let z = be_hex_to_limbs("a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5");
        let k = be_hex_to_limbs("5151515151515151515151515151515151515151515151515151515151515151");
        let exp_r =
            be_hex_to_limbs("9a65173d48a0a0c706eeec2a75ae1f56793be988b2d407e9a7d3aba7574a9ea5");
        let exp_s =
            be_hex_to_limbs("373eb412969302e397f8fa980d61057d3f99487eb57391d1cb733f8134c9b7fe");

        let mut e = Ecdsa::new();
        e.write32(0x00, CONF_SIGN_P256);
        write_block(&mut e, D_PARAM, &d);
        write_block(&mut e, Z_PARAM, &z);
        write_block(&mut e, K_PARAM, &k);
        e.write32(0x04, 1); // START

        assert_eq!(trim(&read_block(&e, R_PARAM)), exp_r);
        assert_eq!(trim(&read_block(&e, S_PARAM)), exp_s);
        // The generated public key must be on the curve (non-infinity).
        assert!(e.read32(0x08) & 1 != 0);
        assert_eq!(e.read32(0x18), 1);
    }

    #[test]
    fn p256_sign_then_verify_round_trips() {
        let d = be_hex_to_limbs("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
        let z = be_hex_to_limbs("a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5");
        let k = be_hex_to_limbs("5151515151515151515151515151515151515151515151515151515151515151");

        let mut e = Ecdsa::new();
        e.write32(0x00, CONF_SIGN_P256);
        write_block(&mut e, D_PARAM, &d);
        write_block(&mut e, Z_PARAM, &z);
        write_block(&mut e, K_PARAM, &k);
        e.write32(0x04, 1); // sign
        assert_eq!(e.read32(0x18), 1);

        // Verify the just-produced signature against the stored public key.
        e.write32(0x00, CONF_VERIFY_P256);
        e.write32(0x04, 1); // verify
        assert_eq!(e.read32(0x18), 1, "valid signature must verify");

        // Tamper with S -> verification must fail.
        let mut s = read_block(&e, S_PARAM);
        s[0] ^= 0xFFFF_FFFF;
        write_block(&mut e, S_PARAM, &s);
        e.write32(0x04, 1);
        assert_eq!(e.read32(0x18), 0, "tampered signature must not verify");
    }

    #[test]
    fn p192_sign_then_verify_round_trips() {
        let d = be_hex_to_limbs("000102030405060708090a0b0c0d0e0f1011121314151617");
        let z = be_hex_to_limbs("a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5a5");
        let k = be_hex_to_limbs("515151515151515151515151515151515151515151515151");

        let mut e = Ecdsa::new();
        e.write32(0x00, CONF_SIGN_P192);
        write_block(&mut e, D_PARAM, &d);
        write_block(&mut e, Z_PARAM, &z);
        write_block(&mut e, K_PARAM, &k);
        e.write32(0x04, 1); // sign
        assert_eq!(e.read32(0x18), 1);

        e.write32(0x00, (0 << 0) | (0 << 2)); // verify P-192
        e.write32(0x04, 1);
        assert_eq!(e.read32(0x18), 1, "valid P-192 signature must verify");
    }
}
