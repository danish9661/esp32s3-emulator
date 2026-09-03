//! ESP32-S3 SHA acceleration peripheral (`DR_REG_SHA_BASE = 0x6003_B000`).
//!
//! The S3 SHA engine receives the (already padded) message blocks through the
//! GDMA controller (`SOC_GDMA_TRIG_PERIPH_SHA0`, `peri_sel = 7`); the esp-idf
//! driver assembles each 64-byte block in DRAM and the GDMA copies it into the
//! message (TEXT) registers at `SHA_TEXT_BASE` (offset 0x80). The algorithm is
//! selected by `SHA_MODE` (`esp32s3/rom/sha.h` `SHA_TYPE`: SHA1=0, SHA224=1,
//! SHA256=2, SHA384=3, SHA512=4, SHA512_t=5). `SHA_DMA_START` begins a new hash
//! (the IV is loaded internally); `SHA_DMA_CONTINUE` continues an in-progress
//! hash across blocks; after each trigger the 32-bit `SHA_BUSY` register reads
//! 0 (the computation is modeled synchronously) and the digest is available in
//! the `SHA_H` registers at offset 0x40.
//!
//! The message bytes are accumulated in arrival order (the GDMA copies the
//! DRAM buffer little-endian word-by-word, reconstructed LSB-first, which is
//! exactly the byte stream the SHA engine consumes). On each start/continue
//! trigger the accumulated complete 64-byte blocks are compressed and the
//! running hash state is copied into the digest readback registers.

use alloc::vec::Vec;

/// SHA peripheral register-block base (`DR_REG_SHA_BASE`, soc/reg_base.h).
pub const SHA_BASE: u32 = 0x6003_B000;

/// GDMA peripheral id for the SHA engine (`SOC_GDMA_TRIG_PERIPH_SHA0`).
pub const GDMA_SHA_PERIPH: u32 = 7;

const SHA256_K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

#[allow(clippy::needless_range_loop)]
fn sha256_compress(state: &mut [u32; 8], block: &[u8; 64]) {
    let mut w = [0u32; 64];
    for i in 0..16 {
        w[i] = ((block[i * 4] as u32) << 24)
            | ((block[i * 4 + 1] as u32) << 16)
            | ((block[i * 4 + 2] as u32) << 8)
            | (block[i * 4 + 3] as u32);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }
    let mut a = state[0];
    let mut b = state[1];
    let mut c = state[2];
    let mut d = state[3];
    let mut e = state[4];
    let mut f = state[5];
    let mut g = state[6];
    let mut h = state[7];
    for i in 0..64 {
        let big_s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let ch = (e & f) ^ ((!e) & g);
        let t1 = h
            .wrapping_add(big_s1)
            .wrapping_add(ch)
            .wrapping_add(SHA256_K[i])
            .wrapping_add(w[i]);
        let big_s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let t2 = big_s0.wrapping_add(maj);
        h = g;
        g = f;
        f = e;
        e = d.wrapping_add(t1);
        d = c;
        c = b;
        b = a;
        a = t1.wrapping_add(t2);
    }
    state[0] = state[0].wrapping_add(a);
    state[1] = state[1].wrapping_add(b);
    state[2] = state[2].wrapping_add(c);
    state[3] = state[3].wrapping_add(d);
    state[4] = state[4].wrapping_add(e);
    state[5] = state[5].wrapping_add(f);
    state[6] = state[6].wrapping_add(g);
    state[7] = state[7].wrapping_add(h);
}

#[allow(clippy::needless_range_loop)]
fn sha1_compress(state: &mut [u32], block: &[u8; 64]) {
    let mut w = [0u32; 80];
    for i in 0..16 {
        w[i] = ((block[i * 4] as u32) << 24)
            | ((block[i * 4 + 1] as u32) << 16)
            | ((block[i * 4 + 2] as u32) << 8)
            | (block[i * 4 + 3] as u32);
    }
    for i in 16..80 {
        w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
    }
    let mut a = state[0];
    let mut b = state[1];
    let mut c = state[2];
    let mut d = state[3];
    let mut e = state[4];
    for i in 0..80 {
        let (f, k) = if i < 20 {
            ((b & c) | ((!b) & d), 0x5a827999u32)
        } else if i < 40 {
            (b ^ c ^ d, 0x6ed9eba1)
        } else if i < 60 {
            ((b & c) | (b & d) | (c & d), 0x8f1bbcdc)
        } else {
            (b ^ c ^ d, 0xca62c1d6)
        };
        let temp = a
            .rotate_left(5)
            .wrapping_add(f)
            .wrapping_add(e)
            .wrapping_add(k)
            .wrapping_add(w[i]);
        e = d;
        d = c;
        c = b.rotate_left(30);
        b = a;
        a = temp;
    }
    state[0] = state[0].wrapping_add(a);
    state[1] = state[1].wrapping_add(b);
    state[2] = state[2].wrapping_add(c);
    state[3] = state[3].wrapping_add(d);
    state[4] = state[4].wrapping_add(e);
}

// SHA-512 round constants (FIPS 180-4, first 64 bits of the fractional parts
// of the cube roots of the first 80 primes).
const SHA512_K: [u64; 80] = [
    0x428a2f98d728ae22,
    0x7137449123ef65cd,
    0xb5c0fbcfec4d3b2f,
    0xe9b5dba58189dbbc,
    0x3956c25bf348b538,
    0x59f111f1b605d019,
    0x923f82a4af194f9b,
    0xab1c5ed5da6d8118,
    0xd807aa98a3030242,
    0x12835b0145706fbe,
    0x243185be4ee4b28c,
    0x550c7dc3d5ffb4e2,
    0x72be5d74f27b896f,
    0x80deb1fe3b1696b1,
    0x9bdc06a725c71235,
    0xc19bf174cf692694,
    0xe49b69c19ef14ad2,
    0xefbe4786384f25e3,
    0x0fc19dc68b8cd5b5,
    0x240ca1cc77ac9c65,
    0x2de92c6f592b0275,
    0x4a7484aa6ea6e483,
    0x5cb0a9dcbd41fbd4,
    0x76f988da831153b5,
    0x983e5152ee66dfab,
    0xa831c66d2db43210,
    0xb00327c898fb213f,
    0xbf597fc7beef0ee4,
    0xc6e00bf33da88fc2,
    0xd5a79147930aa725,
    0x06ca6351e003826f,
    0x142929670a0e6e70,
    0x27b70a8546d22ffc,
    0x2e1b21385c26c926,
    0x4d2c6dfc5ac42aed,
    0x53380d139d95b3df,
    0x650a73548baf63de,
    0x766a0abb3c77b2a8,
    0x81c2c92e47edaee6,
    0x92722c851482353b,
    0xa2bfe8a14cf10364,
    0xa81a664bbc423001,
    0xc24b8b70d0f89791,
    0xc76c51a30654be30,
    0xd192e819d6ef5218,
    0xd69906245565a910,
    0xf40e35855771202a,
    0x106aa07032bbd1b8,
    0x19a4c116b8d2d0c8,
    0x1e376c085141ab53,
    0x2748774cdf8eeb99,
    0x34b0bcb5e19b48a8,
    0x391c0cb3c5c95a63,
    0x4ed8aa4ae3418acb,
    0x5b9cca4f7763e373,
    0x682e6ff3d6b2b8a3,
    0x748f82ee5defb2fc,
    0x78a5636f43172f60,
    0x84c87814a1f0ab72,
    0x8cc702081a6439ec,
    0x90befffa23631e28,
    0xa4506cebde82bde9,
    0xbef9a3f7b2c67915,
    0xc67178f2e372532b,
    0xca273eceea26619c,
    0xd186b8c721c0c207,
    0xeada7dd6cde0eb1e,
    0xf57d4f7fee6ed178,
    0x06f067aa72176fba,
    0x0a637dc5a2c898a6,
    0x113f9804bef90dae,
    0x1b710b35131c471b,
    0x28db77f523047d84,
    0x32caab7b40c72493,
    0x3c9ebe0a15c9bebc,
    0x431d67c49c100d4c,
    0x4cc5d4becb3e42b6,
    0x597f299cfc657e2a,
    0x5fcb6fab3ad6faec,
    0x6c44198c4a475817,
];

#[allow(clippy::needless_range_loop)]
fn sha512_compress(state: &mut [u64; 8], block: &[u8; 128]) {
    let mut w = [0u64; 80];
    for i in 0..16 {
        w[i] = ((block[i * 8] as u64) << 56)
            | ((block[i * 8 + 1] as u64) << 48)
            | ((block[i * 8 + 2] as u64) << 40)
            | ((block[i * 8 + 3] as u64) << 32)
            | ((block[i * 8 + 4] as u64) << 24)
            | ((block[i * 8 + 5] as u64) << 16)
            | ((block[i * 8 + 6] as u64) << 8)
            | (block[i * 8 + 7] as u64);
    }
    for i in 16..80 {
        let s0 = w[i - 15].rotate_right(1) ^ w[i - 15].rotate_right(8) ^ (w[i - 15] >> 7);
        let s1 = w[i - 2].rotate_right(19) ^ w[i - 2].rotate_right(61) ^ (w[i - 2] >> 6);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }
    let mut a = state[0];
    let mut b = state[1];
    let mut c = state[2];
    let mut d = state[3];
    let mut e = state[4];
    let mut f = state[5];
    let mut g = state[6];
    let mut h = state[7];
    for i in 0..80 {
        let big_s1 = e.rotate_right(14) ^ e.rotate_right(18) ^ e.rotate_right(41);
        let ch = (e & f) ^ ((!e) & g);
        let t1 = h
            .wrapping_add(big_s1)
            .wrapping_add(ch)
            .wrapping_add(SHA512_K[i])
            .wrapping_add(w[i]);
        let big_s0 = a.rotate_right(28) ^ a.rotate_right(34) ^ a.rotate_right(39);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let t2 = big_s0.wrapping_add(maj);
        h = g;
        g = f;
        f = e;
        e = d.wrapping_add(t1);
        d = c;
        c = b;
        b = a;
        a = t1.wrapping_add(t2);
    }
    state[0] = state[0].wrapping_add(a);
    state[1] = state[1].wrapping_add(b);
    state[2] = state[2].wrapping_add(c);
    state[3] = state[3].wrapping_add(d);
    state[4] = state[4].wrapping_add(e);
    state[5] = state[5].wrapping_add(f);
    state[6] = state[6].wrapping_add(g);
    state[7] = state[7].wrapping_add(h);
}

#[derive(Default)]
pub struct Sha {
    mode: u32,
    block_num: u32,
    int_ena: u32,
    /// Accumulated message bytes for the current DMA/direct operation.
    msg: Vec<u8>,
    /// Running hash state (h0..hN), big-endian words.
    h: [u32; 8],
    /// Running hash state for the 512-bit modes (SHA-384/512).
    h64: [u64; 8],
    /// Digest readback registers (SHA_H_BASE), up to 16 words for SHA-512.
    digest: [u32; 16],
    digest_words: usize,
}

impl Sha {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append one message byte (used by the GDMA descriptor-walk copy).
    pub fn feed_byte(&mut self, b: u8) {
        self.msg.push(b);
    }

    fn reset_state(&mut self) {
        match self.mode {
            // SHA1
            0 => {
                self.h[0] = 0x67452301;
                self.h[1] = 0xEFCDAB89;
                self.h[2] = 0x98BADCFE;
                self.h[3] = 0x10325476;
                self.h[4] = 0xC3D2E1F0;
                self.digest_words = 5;
            }
            // SHA224 has its own IV (truncated SHA-256).
            1 => {
                self.h[0] = 0xc1059ed8;
                self.h[1] = 0x367cd507;
                self.h[2] = 0x3070dd17;
                self.h[3] = 0xf70e5939;
                self.h[4] = 0xffc00b31;
                self.h[5] = 0x68581511;
                self.h[6] = 0x64f98fa7;
                self.h[7] = 0xbefa4fa4;
                self.digest_words = 8;
            }
            // SHA256.
            2 => {
                self.h[0] = 0x6a09e667;
                self.h[1] = 0xbb67ae85;
                self.h[2] = 0x3c6ef372;
                self.h[3] = 0xa54ff53a;
                self.h[4] = 0x510e527f;
                self.h[5] = 0x9b05688c;
                self.h[6] = 0x1f83d9ab;
                self.h[7] = 0x5be0cd19;
                self.digest_words = 8;
            }
            // SHA384 (truncated SHA-512 with its own IV).
            3 => {
                self.h64[0] = 0xcbbb9d5dc1059ed8;
                self.h64[1] = 0x629a292a367cd507;
                self.h64[2] = 0x9159015a3070dd17;
                self.h64[3] = 0x152fecd8f70e5939;
                self.h64[4] = 0x67332667ffc00b31;
                self.h64[5] = 0x8eb44a8768581511;
                self.h64[6] = 0xdb0c2e0d64f98fa7;
                self.h64[7] = 0x47b5481dbefa4fa4;
                self.digest_words = 12;
            }
            // SHA512.
            4 => {
                self.h64[0] = 0x6a09e667f3bcc908;
                self.h64[1] = 0xbb67ae8584caa73b;
                self.h64[2] = 0x3c6ef372fe94f82b;
                self.h64[3] = 0xa54ff53a5f1d36f1;
                self.h64[4] = 0x510e527fade682d1;
                self.h64[5] = 0x9b05688c2b3e6c1f;
                self.h64[6] = 0x1f83d9abfb41bd6b;
                self.h64[7] = 0x5be0cd19137e2179;
                self.digest_words = 16;
            }
            _ => {
                // SHA512_t (mode 5) not modeled: mbedtls exposes no t-variant.
                self.digest_words = 0;
            }
        }
    }

    fn process(&mut self) {
        // SHA-384/512 compress 128-byte blocks; the 32-bit modes use 64.
        let blk = if self.mode == 3 || self.mode == 4 {
            128
        } else {
            64
        };
        let n = self.msg.len() / blk;
        for _ in 0..n {
            match self.mode {
                0 => {
                    let mut block = [0u8; 64];
                    block.copy_from_slice(&self.msg[..64]);
                    sha1_compress(&mut self.h[..5], &block);
                }
                1 | 2 => {
                    let mut block = [0u8; 64];
                    block.copy_from_slice(&self.msg[..64]);
                    sha256_compress(&mut self.h, &block);
                }
                3 | 4 => {
                    let mut block = [0u8; 128];
                    block.copy_from_slice(&self.msg[..128]);
                    sha512_compress(&mut self.h64, &block);
                }
                _ => {}
            }
            self.msg.drain(0..blk);
        }
        // The SHA H registers store each digest word in little-endian byte
        // order (the raw digest byte stream), so the driver's uint32 read
        // yields the byte-swapped big-endian word. Mirror that here. For the
        // 512-bit modes each 64-bit state word splits hi-half first.
        if self.mode == 3 || self.mode == 4 {
            for j in 0..8 {
                self.digest[2 * j] = ((self.h64[j] >> 32) as u32).swap_bytes();
                self.digest[2 * j + 1] = ((self.h64[j] & 0xFFFF_FFFF) as u32).swap_bytes();
            }
        } else {
            for i in 0..8 {
                self.digest[i] = self.h[i].swap_bytes();
            }
        }
    }

    pub fn read32(&self, off: u32) -> u32 {
        match off {
            0x00 => self.mode,
            0x0C => self.block_num,
            0x18 => 0, // BUSY: always idle (synchronous model)
            0x28 => self.int_ena,
            0x40..=0x7C => {
                let i = (off - 0x40) / 4;
                if (i as usize) < self.digest_words {
                    self.digest[i as usize]
                } else {
                    0
                }
            }
            _ => 0,
        }
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        match off {
            0x00 => self.mode = value & 0x7,
            0x0C => self.block_num = value,
            0x10 => {
                // SHA_START (direct-fill, new hash): process any buffered message.
                self.reset_state();
                self.process();
            }
            0x14 => {
                // SHA_CONTINUE (direct-fill): continue running hash.
                self.process();
            }
            0x1C => {
                // SHA_DMA_START: new hash, process GDMA-fed message.
                self.reset_state();
                self.process();
            }
            0x20 => {
                // SHA_DMA_CONTINUE: continue running hash.
                self.process();
            }
            0x28 => self.int_ena = value,
            0x80..=0xFC => {
                // TEXT (message) direct-fill: append the 4 bytes LSB-first,
                // matching how the GDMA byte stream is consumed.
                self.msg.push((value & 0xFF) as u8);
                self.msg.push(((value >> 8) & 0xFF) as u8);
                self.msg.push(((value >> 16) & 0xFF) as u8);
                self.msg.push(((value >> 24) & 0xFF) as u8);
            }
            _ => {}
        }
    }
}
