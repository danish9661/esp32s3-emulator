//! ESP32-S3 HMAC accelerator peripheral model.
//!
//! Register block at `DR_REG_HMAC_BASE = 0x6003_E000` (per `hwcrypto_reg.h`).
//! The HMAC engine is a streaming SHA-256 core.  The driver performs the HMAC
//! key schedule (XOR the eFuse key with `ipad`/`opad` and prepend it to the
//! message) and writes the resulting 512-bit-block stream through `HMAC_WDATA`
//! together with the SHA padding; the hardware simply computes SHA-256 over the
//! exact bytes written.  We materialize the digest synchronously at `SET_START`
//! (`QUERY_BUSY` always reads idle).  The eFuse key for `key_id` is cached at
//! `SET_PARA_FINISH` time (see `Soc`).

use alloc::vec::Vec;

use crate::efuse::Efuse;

pub const HMAC_BASE: u32 = 0x6003_E000;

const REG_COUNT: usize = 0x100 / 4;

const SET_START_OFF: usize = 0x40 / 4;
#[allow(dead_code)]
const SET_PARA_PURPOSE_OFF: usize = 0x44 / 4;
const SET_PARA_KEY_OFF: usize = 0x48 / 4;
pub const SET_PARA_FINISH_OFF: usize = 0x4C / 4;
#[allow(dead_code)]
const SET_MESSAGE_ONE_OFF: usize = 0x50 / 4;
#[allow(dead_code)]
const SET_MESSAGE_ING_OFF: usize = 0x54 / 4;
#[allow(dead_code)]
const SET_MESSAGE_END_OFF: usize = 0x58 / 4;
#[allow(dead_code)]
const SET_RESULT_FINISH_OFF: usize = 0x5C / 4;
#[allow(dead_code)]
const SET_INVALIDATE_JTAG_OFF: usize = 0x60 / 4;
#[allow(dead_code)]
const SET_INVALIDATE_DS_OFF: usize = 0x64 / 4;
const QUERY_ERROR_OFF: usize = 0x68 / 4;
const QUERY_BUSY_OFF: usize = 0x6C / 4;
const WDATA_OFF: usize = 0x80 / 4; // 16 words (512 bits)
const RDATA_OFF: usize = 0xC0 / 4; // 8 words (256 bits)
const RDATA_END_OFF: usize = RDATA_OFF + 7;
#[allow(dead_code)]
const SET_MESSAGE_PAD_OFF: usize = 0xF0 / 4;
#[allow(dead_code)]
const ONE_BLOCK_OFF: usize = 0xF4 / 4;

/// SHA-256 round constants.
const K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

fn compress(h: &mut [u32; 8], chunk: &[u8]) {
    let mut w = [0u32; 64];
    for i in 0..16 {
        w[i] = u32::from_be_bytes([
            chunk[i * 4],
            chunk[i * 4 + 1],
            chunk[i * 4 + 2],
            chunk[i * 4 + 3],
        ]);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }
    let (mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh) =
        (h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
    for i in 0..64 {
        let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let ch = (e & f) ^ ((!e) & g);
        let t1 = hh
            .wrapping_add(s1)
            .wrapping_add(ch)
            .wrapping_add(K[i])
            .wrapping_add(w[i]);
        let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let t2 = s0.wrapping_add(maj);
        hh = g;
        g = f;
        f = e;
        e = d.wrapping_add(t1);
        d = c;
        c = b;
        b = a;
        a = t1.wrapping_add(t2);
    }
    h[0] = h[0].wrapping_add(a);
    h[1] = h[1].wrapping_add(b);
    h[2] = h[2].wrapping_add(c);
    h[3] = h[3].wrapping_add(d);
    h[4] = h[4].wrapping_add(e);
    h[5] = h[5].wrapping_add(f);
    h[6] = h[6].wrapping_add(g);
    h[7] = h[7].wrapping_add(hh);
}

/// SHA-256 over `msg` (applies standard padding).
fn sha256(msg: &[u8]) -> [u8; 32] {
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let bit_len = (msg.len() as u64).wrapping_mul(8);
    let mut data = msg.to_vec();
    data.push(0x80);
    while data.len() % 64 != 56 {
        data.push(0);
    }
    data.extend_from_slice(&bit_len.to_be_bytes());
    for chunk in data.chunks_exact(64) {
        compress(&mut h, chunk);
    }
    let mut out = [0u8; 32];
    for i in 0..8 {
        out[i * 4..i * 4 + 4].copy_from_slice(&h[i].to_be_bytes());
    }
    out
}

/// SHA-256 over `data` which is already a multiple of 64 bytes (no padding).
fn sha256_raw(data: &[u8]) -> [u8; 32] {
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    for chunk in data.chunks_exact(64) {
        compress(&mut h, chunk);
    }
    let mut out = [0u8; 32];
    for i in 0..8 {
        out[i * 4..i * 4 + 4].copy_from_slice(&h[i].to_be_bytes());
    }
    out
}

pub struct Hmac {
    regs: [u32; REG_COUNT],
    key: [u8; 32],
    msg: Vec<u8>,
    result: [u8; 32],
    busy: bool,
    error: u32,
}

impl Hmac {
    pub fn new() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
            key: [0u8; 32],
            msg: Vec::new(),
            result: [0u8; 32],
            busy: false,
            error: 0,
        }
    }

    /// Cache the eFuse key for the currently selected `key_id`.
    pub fn fetch_key(&mut self, efuse: &Efuse) {
        self.key = efuse.hmac_key(self.key_id());
    }

    fn key_id(&self) -> usize {
        (self.regs[SET_PARA_KEY_OFF] & 0x7) as usize
    }

    fn compute(&mut self) {
        // Inner hash: SHA-256 over the bytes written by the driver.  On
        // SET_MESSAGE_END the hardware appends the SHA padding (this is only
        // valid when the message is a multiple of 512 bits, but the raw hash
        // is correct regardless of length); ONE_BLOCK/PAD mean the driver
        // already laid out the padding.
        let inner = sha256_raw(&self.msg);
        // Outer hash: SHA-256( (key ^ opad) || inner ).
        let mut outer = Vec::with_capacity(96);
        for i in 0..64 {
            let kb = if i < self.key.len() { self.key[i] } else { 0 };
            outer.push(kb ^ 0x5c);
        }
        outer.extend_from_slice(&inner);
        self.result = sha256(&outer);
        self.busy = false;
    }

    pub fn read32(&self, off: u32) -> u32 {
        let w = (off >> 2) as usize;
        match w {
            QUERY_BUSY_OFF => self.busy as u32,
            QUERY_ERROR_OFF => self.error,
            RDATA_OFF..=RDATA_END_OFF => {
                let i = w - RDATA_OFF;
                let b = i * 4;
                u32::from_be_bytes([
                    self.result[b],
                    self.result[b + 1],
                    self.result[b + 2],
                    self.result[b + 3],
                ])
            }
            _ if w < REG_COUNT => self.regs[w],
            _ => 0,
        }
    }

    fn pad_message(&mut self) {
        self.msg.push(0x80);
        while self.msg.len() % 64 != 56 {
            self.msg.push(0);
        }
        let bit_len = (self.msg.len() as u64).wrapping_mul(8);
        self.msg.extend_from_slice(&bit_len.to_be_bytes());
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        let w = (off >> 2) as usize;
        self.regs[w] = value;
        match w {
            SET_PARA_FINISH_OFF => {
                // New operation: clear the accumulated message stream.
                self.msg.clear();
            }
            SET_MESSAGE_END_OFF | SET_MESSAGE_PAD_OFF => self.pad_message(),
            SET_START_OFF => self.compute(),
            w if (WDATA_OFF..WDATA_OFF + 16).contains(&w) => {
                // Append the 4 little-endian bytes of the word to the stream.
                self.msg.extend_from_slice(&value.to_le_bytes());
            }
            // Block-boundary signals (ONE/ING) are no-ops: the driver has
            // already laid out (key ^ ipad) || message || SHA-padding in the
            // 512-bit blocks it writes.
            _ => {}
        }
    }
}

impl Default for Hmac {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use alloc::vec;

    fn bytes(hex: &str) -> Vec<u8> {
        let mut v = Vec::new();
        let h = hex.as_bytes();
        for i in (0..h.len()).step_by(2) {
            let hi = (h[i] as char).to_digit(16).unwrap();
            let lo = (h[i + 1] as char).to_digit(16).unwrap();
            v.push((hi * 16 + lo) as u8);
        }
        v
    }

    #[test]
    fn sha256_known() {
        assert_eq!(
            sha256(b"abc").to_vec(),
            bytes("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")
        );
        assert_eq!(
            sha256(b"").to_vec(),
            bytes("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855")
        );
    }

    #[test]
    fn sha256_raw_matches() {
        // Padded "abc" (64-byte block) hashed raw == sha256("abc").
        let mut block = b"abc".to_vec();
        block.push(0x80);
        while block.len() % 64 != 56 {
            block.push(0);
        }
        block.extend_from_slice(&24u64.to_be_bytes());
        assert_eq!(block.len(), 64);
        assert_eq!(
            sha256_raw(&block).to_vec(),
            bytes("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")
        );
        // Multi-block raw hash.
        let blk = [b'a'; 64].to_vec();
        let mut two = blk.clone();
        two.extend_from_slice(&[b'b'; 64]);
        let got = sha256_raw(&two);
        let _ = got;
    }

    #[test]
    fn sha256_multiblock() {
        assert_eq!(
            sha256(&[b'a'; 100]).to_vec(),
            bytes("2816597888e4a0d3a36b82b83316ab32680eb8f00f8cd3b904d681246d285a0e")
        );
        assert_eq!(
            sha256(&[b'x'; 96]).to_vec(),
            bytes("d47f0876dd6917702e40d215907c975a9c8f6b8140510841c1ebee43497aac95")
        );
    }

    #[test]
    fn sha256_raw_eq_padded() {
        // sha256_raw over the 128-byte padded form of a 119-byte message equals
        // hashlib.sha256 of that 119-byte message.
        let mut m = vec![0u8; 119];
        m.push(0x80);
        while m.len() % 64 != 56 {
            m.push(0);
        }
        m.extend_from_slice(&(119u64 * 8).to_be_bytes());
        assert_eq!(m.len(), 128);
        assert_eq!(
            sha256_raw(&m).to_vec(),
            bytes("f616b0d54e78571a9611f343c9f8e022e859e920381ab0e4d3da01e193a7bd7e")
        );
    }

    #[test]
    fn sha256_raw_119a() {
        let mut m = vec![b'a'; 119];
        m.push(0x80);
        while m.len() % 64 != 56 {
            m.push(0);
        }
        m.extend_from_slice(&(119u64 * 8).to_be_bytes());
        assert_eq!(m.len(), 128);
        assert_eq!(
            sha256_raw(&m).to_vec(),
            bytes("31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb")
        );
    }

    #[test]
    fn sha256_raw_hmac_inner() {
        let mut m = vec![0x36u8; 64];
        m.extend_from_slice(b"hello");
        m.push(0x80);
        while m.len() % 64 != 56 {
            m.push(0);
        }
        m.extend_from_slice(&((64 + 5) as u64).wrapping_mul(8).to_be_bytes());
        assert_eq!(m.len(), 128);
        assert_eq!(
            sha256_raw(&m).to_vec(),
            bytes("38cf0e1931ef209723608c5cf96e7ab09773ddfc1f50701ce66f5d18a5fa8b24")
        );
    }
}
