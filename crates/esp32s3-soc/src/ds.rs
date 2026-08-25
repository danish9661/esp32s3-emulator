//! ESP32-S3 Digital Signature (DS) peripheral (`DR_REG_DIGITAL_SIGNATURE_BASE
//! = 0x6003_D000`).
//!
//! The DS peripheral accelerates RSA signing using a *pre-encrypted* RSA
//! private key (so the plaintext key never appears in software). From the
//! ESP-IDF docs / `configure_ds.py`, the decryption + signing flow is:
//!
//! 1. AES key = `HMAC-SHA256(efuse_key, 0xFF*32)` — the HMAC runs in
//!    "downstream" mode and feeds the result to the DS block as the AES key.
//! 2. The encrypted blob `c = C_Y(512) || C_M(512) || C_RB(512) || C_BOX(48)`
//!    is AES-256-CBC decrypted with the 16-byte IV, yielding
//!    `Y || M || Rb || md(32) || M_prime(4) || length(4) || 0x08*8`.
//! 3. The signature is a plain RSA operation `Z = X^Y mod M` (Y = the RSA
//!    private exponent, M = the modulus, Rb/M_prime are Montgomery-only
//!    helpers the hardware needs but a software RSA does not).
//! 4. `md` is an integrity check: `md == SHA256(Y || M || Rb || M_prime ||
//!    length || IV)`; `QUERY_CHECK` reports padding/digest failures.
//!
//! We model the block behaviourally (the same black box the firmware reads
//! `Z` back from). The HMAC key is taken from eFuse block `key_id` (default 0,
//! which is all-zero in the emulator — a valid key); the AES + RSA math is
//! done in software with the real algorithms so the result matches silicon.

use alloc::vec::Vec;

use crate::aes::aes256_cbc_decrypt;
use crate::bignum::*;
use crate::efuse::Efuse;
use crate::hmac::{hmac_sha256, sha256};

#[allow(clippy::identity_op)]
pub const DS_BASE: u32 = 0x6003_D000;

// Register byte offsets (TRM `hwcrypto_reg.h`); the read/write dispatch uses
// literal word ranges derived from these.
#[allow(dead_code)]
const C_Y_OFF: usize = 0x000;
#[allow(dead_code)]
const C_BOX_OFF: usize = 0x600;
#[allow(dead_code)]
const IV_OFF: usize = 0x630;
#[allow(dead_code)]
const X_OFF: usize = 0x800;
#[allow(dead_code)]
const Z_OFF: usize = 0xA00;

const C_Y_LEN: usize = 512;
const C_M_LEN: usize = 512;
const C_RB_LEN: usize = 512;
const C_BOX_LEN: usize = 48;
const IV_LEN: usize = 16;
const X_LEN: usize = 512;
const Z_LEN: usize = 512;

const CHECK_INVALID_DIGEST: u32 = 1 << 0;
const CHECK_INVALID_PADDING: u32 = 1 << 1;

#[allow(clippy::identity_op)]
pub struct Ds {
    cy: [u8; C_Y_LEN],
    cm: [u8; C_M_LEN],
    crb: [u8; C_RB_LEN],
    cbox: [u8; C_BOX_LEN],
    iv: [u8; IV_LEN],
    x: [u8; X_LEN],
    z: [u8; Z_LEN],
    set_start: u32,
    set_me: u32,
    set_finish: u32,
    query_key_wrong: u32,
    query_check: u32,
    /// eFuse key block supplying the HMAC downstream key (default 0).
    key_id: u8,
}

impl Default for Ds {
    fn default() -> Self {
        Self {
            cy: [0; C_Y_LEN],
            cm: [0; C_M_LEN],
            crb: [0; C_RB_LEN],
            cbox: [0; C_BOX_LEN],
            iv: [0; IV_LEN],
            x: [0; X_LEN],
            z: [0; Z_LEN],
            set_start: 0,
            set_me: 0,
            set_finish: 0,
            query_key_wrong: 0,
            query_check: 0,
            key_id: 0,
        }
    }
}

impl Ds {
    pub fn new() -> Self {
        Self::default()
    }

    /// Select the eFuse key block used as the HMAC downstream key source.
    pub fn set_key_id(&mut self, id: u8) {
        self.key_id = id;
    }

    fn put_word(arr: &mut [u8], byte_off: usize, v: u32) {
        arr[byte_off..byte_off + 4].copy_from_slice(&v.to_le_bytes());
    }

    fn get_word(arr: &[u8], byte_off: usize) -> u32 {
        u32::from_le_bytes([
            arr[byte_off],
            arr[byte_off + 1],
            arr[byte_off + 2],
            arr[byte_off + 3],
        ])
    }

    /// Decrypt the RSA parameters and compute `Z = X^Y mod M`.
    fn sign(&mut self, efuse: &Efuse) {
        let key = efuse.hmac_key(self.key_id as usize);
        let aes_key = hmac_sha256(&key, &[0xFFu8; 32]);
        let mut c = Vec::with_capacity(C_Y_LEN + C_M_LEN + C_RB_LEN + C_BOX_LEN);
        c.extend_from_slice(&self.cy);
        c.extend_from_slice(&self.cm);
        c.extend_from_slice(&self.crb);
        c.extend_from_slice(&self.cbox);
        let plain = aes256_cbc_decrypt(&aes_key, &self.iv, &c);

        // Integrity: md == SHA256(Y || M || Rb || M_prime || length || IV).
        let md = &plain[1536..1568];
        let tail = &plain[1568..1584];
        let mut md_in = Vec::with_capacity(1536 + 8 + IV_LEN);
        md_in.extend_from_slice(&plain[0..1536]);
        md_in.extend_from_slice(&tail[0..8]); // M_prime(4) || length(4)
        md_in.extend_from_slice(&self.iv);
        let calc = sha256(&md_in);
        let md_ok = calc == *md;
        let padding_ok = tail[8..16] == [0x08u8; 8];
        let mut qc = 0u32;
        if !md_ok {
            qc |= CHECK_INVALID_DIGEST;
        }
        if !padding_ok {
            qc |= CHECK_INVALID_PADDING;
        }
        self.query_check = qc;

        // Sign: Z = X^Y mod M, using (length+1) words of the operands.
        let length_words = u32::from_le_bytes([tail[4], tail[5], tail[6], tail[7]]) as usize;
        let nwords = length_words + 1;
        let nbytes = (nwords * 4).min(C_Y_LEN);
        let y = bytes_to_limbs_le(&plain[0..nbytes]);
        let m = bytes_to_limbs_le(&plain[C_Y_LEN..C_Y_LEN + nbytes]);
        let x = bytes_to_limbs_le(&self.x[0..nbytes]);
        let z = modexp(&x, &y, &m);
        let zbytes = limbs_to_bytes_le(&z);
        self.z = [0u8; Z_LEN];
        let n = zbytes.len().min(Z_LEN);
        self.z[0..n].copy_from_slice(&zbytes[0..n]);
    }

    pub fn read32(&self, off: u32) -> u32 {
        let w = off as usize / 4;
        match w {
            0..=127 => Ds::get_word(&self.cy, w * 4),
            128..=255 => Ds::get_word(&self.cm, (w - 128) * 4),
            256..=383 => Ds::get_word(&self.crb, (w - 256) * 4),
            384..=395 => Ds::get_word(&self.cbox, (w - 384) * 4),
            396..=399 => Ds::get_word(&self.iv, (w - 396) * 4),
            512..=639 => Ds::get_word(&self.x, (w - 512) * 4),
            640..=767 => Ds::get_word(&self.z, (w - 640) * 4),
            896 => self.set_start,
            897 => self.set_me,
            898 => self.set_finish,
            899 => 0, // QUERY_BUSY: synchronous model, never busy
            900 => self.query_key_wrong,
            901 => self.query_check,
            _ => 0,
        }
    }

    pub fn write32(&mut self, off: u32, value: u32, efuse: &Efuse) {
        let w = off as usize / 4;
        match w {
            0..=127 => Ds::put_word(&mut self.cy, w * 4, value),
            128..=255 => Ds::put_word(&mut self.cm, (w - 128) * 4, value),
            256..=383 => Ds::put_word(&mut self.crb, (w - 256) * 4, value),
            384..=395 => Ds::put_word(&mut self.cbox, (w - 384) * 4, value),
            396..=399 => Ds::put_word(&mut self.iv, (w - 396) * 4, value),
            512..=639 => Ds::put_word(&mut self.x, (w - 512) * 4, value),
            640..=767 => Ds::put_word(&mut self.z, (w - 640) * 4, value),
            896 => {
                self.set_start = value;
                if value != 0 {
                    self.sign(efuse);
                }
            }
            897 => self.set_me = value,
            898 => self.set_finish = value,
            // QUERY_* are read-only status registers.
            899..=901 => {}
            _ => {}
        }
    }
}

fn bytes_to_limbs_le(b: &[u8]) -> Vec<u32> {
    let n = b.len().div_ceil(4);
    let mut v = Vec::with_capacity(n);
    for i in 0..n {
        let mut w = 0u32;
        for j in 0..4 {
            if i * 4 + j < b.len() {
                w |= (b[i * 4 + j] as u32) << (j * 8);
            }
        }
        v.push(w);
    }
    v
}

fn limbs_to_bytes_le(l: &[u32]) -> Vec<u8> {
    let mut v = Vec::with_capacity(l.len() * 4);
    for &w in l {
        v.extend_from_slice(&w.to_le_bytes());
    }
    v
}
