//! ESP32-S3 RSA modular-exponentiation accelerator model.
//!
//! Register block at `0x6003_C000` (`DR_REG_RSA_BASE` — page-aligned, own 4KB
//! mmio page). The peripheral is a big-integer engine: operands are stored as
//! arrays of 32-bit words in four 0x200-byte memory blocks — `M` (modulus,
//! +0x000), `X` (base/message, +0x600), `Y` (exponent, +0x400), and `Z`/`RB`
//! (result, +0x200, shared). A `MODEXP_START` (+0x80c) computes
//! `Z = X^Y mod M` and raises the done interrupt. Configuration registers:
//! `M_DASH`(+0x800, `-M^{-1} mod 2^32`), `LENGTH`(+0x804, key word count),
//! `QUERY_INTERRUPT`(+0x818)/`CLEAR_INTERRUPT`(+0x81c)/`INTERRUPT`(+0x82c).
//!
//! The esp-idf RSA driver (`components/hal/.../rsa_hal`) writes the mbedtls
//! MPI limbs **little-endian** (limb 0 = least significant word) into each
//! block, so the model treats block word `i` as bignum limb `i` (LSW first)
//! and the driver reads the result back the same way. We do the actual
//! modular exponentiation in software (schoolbook multiply + restoring-division
//! reduction) — the hardware is a black box to the firmware, which only reads
//! `Z` back, so a synchronous software modexp is behaviorally identical.

use alloc::vec;
use alloc::vec::Vec;

/// RSA register-block base (`DR_REG_RSA_BASE`).
pub const RSA_BASE: u32 = 0x6003_C000;

/// Max operand size: 4096-bit RSA = 128 words (each block is 0x200 bytes).
const NW: usize = 128;

/// RSA interrupt source for the interrupt matrix (esp32s3 interrupts.h
/// `ETS_RSA_INTR_SOURCE` = 95).
pub const RSA_INTR_SOURCE: u32 = 95;

pub struct Rsa {
    m: [u32; NW],
    x: [u32; NW],
    y: [u32; NW],
    z: [u32; NW],
    /// Key size in 32-bit words (from `LENGTH` register).
    nwords: usize,
    m_dash: u32,
    int_raw: u32,
}

impl Default for Rsa {
    fn default() -> Self {
        Rsa {
            m: [0; NW],
            x: [0; NW],
            y: [0; NW],
            z: [0; NW],
            nwords: 0,
            m_dash: 0,
            int_raw: 0,
        }
    }
}

/// Drop leading (most-significant) zero limbs, returning a trimmed owned Vec.
fn trim(s: &[u32]) -> Vec<u32> {
    let mut v = s.to_vec();
    while !v.is_empty() && v[v.len() - 1] == 0 {
        v.pop();
    }
    v
}

impl Rsa {
    // --- bignum helpers (limb 0 = LSW) ---

    /// Schoolbook multiply; result has `a.len() + b.len()` limbs.
    #[allow(clippy::needless_range_loop)]
    fn mul(a: &[u32], b: &[u32]) -> Vec<u32> {
        let mut r = vec![0u32; a.len() + b.len()];
        for i in 0..a.len() {
            let mut carry = 0u64;
            let mut k = i;
            for j in 0..b.len() {
                let cur = r[k] as u64 + (a[i] as u64) * (b[j] as u64) + carry;
                r[k] = cur as u32;
                carry = cur >> 32;
                k += 1;
            }
            while carry != 0 {
                let cur = r[k] as u64 + carry;
                r[k] = cur as u32;
                carry = cur >> 32;
                k += 1;
            }
        }
        r
    }

    /// `a div b` and `a mod b` via Knuth Algorithm D (radix 2^32). Returns
    /// `(quotient, remainder)`. `b` must be non-zero.
    fn divmod(a: &[u32], b: &[u32]) -> (Vec<u32>, Vec<u32>) {
        let b = trim(b);
        let n = b.len();
        if n == 0 {
            return (Vec::new(), a.to_vec());
        }
        let a = trim(a);
        if a.len() < n {
            return (vec![0u32; 1], a.to_vec());
        }
        // Normalize so that the top word of the divisor has its MSB set.
        let shift = b[n - 1].leading_zeros();
        let mut v = b.to_vec();
        let mut u = a.to_vec();
        u.push(0); // leading zero word so the top digit fits
        if shift > 0 {
            let mut carry = 0u32;
            for w in v.iter_mut() {
                let cur = (*w << shift) | carry;
                carry = *w >> (32 - shift);
                *w = cur;
            }
            carry = 0;
            for w in u.iter_mut() {
                let cur = (*w << shift) | carry;
                carry = *w >> (32 - shift);
                *w = cur;
            }
        }
        let m = n;
        let ndigits = u.len() - m; // number of quotient digits
        let mut q = vec![0u32; ndigits];
        // Working remainder: u, with the top digit consumed from index m..
        let mut rem = u.clone();
        for j in (0..ndigits).rev() {
            let num_hi = *rem.get(j + m).unwrap_or(&0) as u64;
            let num = (num_hi << 32) | rem[j + m - 1] as u64;
            let mut qhat = num / (v[m - 1] as u64);
            let mut rhat = num % (v[m - 1] as u64);
            let v_m2 = if m >= 2 { v[m - 2] as u64 } else { 0 };
            while qhat >= (1u64 << 32)
                || (m >= 2
                    && (qhat * v_m2)
                        > ((rhat << 32) | rem.get(j + m - 2).copied().unwrap_or(0) as u64))
            {
                qhat -= 1;
                rhat += v[m - 1] as u64;
                if rhat >= (1u64 << 32) {
                    break;
                }
            }
            // rem[j..j+m+1] -= qhat * v
            let mut carry = 0i64;
            let mut borrow = 0i64;
            for i in 0..m {
                let p = qhat * v[i] as u64 + carry as u64;
                carry = (p >> 32) as i64;
                let sub = (p as u32) as i64;
                let cur = rem[j + i] as i64 - sub - borrow;
                if cur < 0 {
                    rem[j + i] = (cur + (1i64 << 32)) as u32;
                    borrow = 1;
                } else {
                    rem[j + i] = cur as u32;
                    borrow = 0;
                }
            }
            let cur = rem[j + m] as i64 - carry - borrow;
            if cur < 0 {
                // qhat was one too large: add v back, decrement qhat.
                rem[j + m] = (cur + (1i64 << 32)) as u32;
                qhat -= 1;
                let mut c = 0u64;
                for i in 0..m {
                    let s = rem[j + i] as u64 + v[i] as u64 + c;
                    rem[j + i] = s as u32;
                    c = s >> 32;
                }
                let s = rem[j + m] as u64 + c;
                rem[j + m] = s as u32;
            } else {
                rem[j + m] = cur as u32;
            }
            q[j] = qhat as u32;
        }
        // Remainder is rem[0..m]; unnormalize (shift right by `shift`).
        let mut rem_out = rem[0..m].to_vec();
        if shift > 0 {
            let mut carry = 0u32;
            for i in (0..rem_out.len()).rev() {
                let cur = (rem_out[i] >> shift) | (carry << (32 - shift));
                carry = rem_out[i] & ((1u32 << shift) - 1);
                rem_out[i] = cur;
            }
        }
        while rem_out.len() > 1 && rem_out[rem_out.len() - 1] == 0 {
            rem_out.pop();
        }
        (q, rem_out)
    }

    /// `r mod m` via Knuth long division (O(n^2), not exponential in n).
    fn mod_m(r: &[u32], m: &[u32]) -> Vec<u32> {
        let m = trim(m);
        if m.is_empty() {
            return r.to_vec();
        }
        Rsa::divmod(r, &m).1
    }

    /// `base^exp mod modulus` (all little-endian limb slices).
    fn modexp(base: &[u32], exp: &[u32], modulus: &[u32]) -> Vec<u32> {
        let modulus = trim(modulus);
        if modulus.is_empty() {
            return Vec::new();
        }
        let mut result = vec![0u32; 1];
        result[0] = 1;
        let mut b = base.to_vec();
        let e = trim(exp);
        for i in 0..e.len() {
            let mut limb = e[i];
            for _bit in 0..32 {
                if limb & 1 != 0 {
                    let prod = Rsa::mul(&result, &b);
                    result = Rsa::mod_m(&prod, &modulus);
                }
                limb >>= 1;
                if limb == 0 && i == e.len() - 1 {
                    break;
                }
                let sq = Rsa::mul(&b, &b);
                b = Rsa::mod_m(&sq, &modulus);
            }
        }
        result
    }

    /// Run the configured modular exponentiation (`Z = X^Y mod M`).
    fn run_modexp(&mut self) {
        // esp-idf writes LENGTH = nwords - 1, so recover nwords.
        let n = if self.nwords == 0 {
            NW
        } else {
            self.nwords + 1
        };
        let n = n.min(NW);
        let x = trim(&self.x[..n]);
        let y = trim(&self.y[..n]);
        let m = trim(&self.m[..n]);
        let z = Rsa::modexp(&x, &y, &m);
        // Stage the result in a local buffer first: the host emulates the
        // operation synchronously inside the MODEXP_START write, so Z must be
        // ready when the firmware next reads it back.
        let mut buf = [0u32; NW];
        for (i, b) in buf.iter_mut().enumerate().take(n) {
            *b = z.get(i).copied().unwrap_or(0);
        }
        self.z.copy_from_slice(&buf);
        self.int_raw |= 1;
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        match off {
            0x000..=0x1FC => {
                let i = (off / 4) as usize;
                if i < NW {
                    self.m[i] = value;
                }
            }
            0x200..=0x3FC => {
                let i = ((off - 0x200) / 4) as usize;
                if i < NW {
                    self.z[i] = value;
                }
            }
            0x400..=0x5FC => {
                let i = ((off - 0x400) / 4) as usize;
                if i < NW {
                    self.y[i] = value;
                }
            }
            0x600..=0x7FC => {
                let i = ((off - 0x600) / 4) as usize;
                if i < NW {
                    self.x[i] = value;
                }
            }
            0x800 => self.m_dash = value,
            0x804 => self.nwords = (value & 0xFF) as usize,
            0x80C => {
                if value & 1 != 0 {
                    self.run_modexp();
                }
            }
            0x810 => {
                if value & 1 != 0 {
                    // MOD_MULT: Z = X*Y mod M.
                    let n = (self.nwords + 1).min(NW);
                    let x = trim(&self.x[..n]);
                    let y = trim(&self.y[..n]);
                    let m = trim(&self.m[..n]);
                    let prod = Rsa::mul(&x, &y);
                    let z = Rsa::mod_m(&prod, &m);
                    for i in 0..n {
                        self.z[i] = z.get(i).copied().unwrap_or(0);
                    }
                    self.int_raw |= 1;
                }
            }
            0x814 => {
                if value & 1 != 0 {
                    // MULT: Z = X*Y (no modular reduction).
                    let n = (self.nwords + 1).min(NW);
                    let x = trim(&self.x[..n]);
                    let y = trim(&self.y[..n]);
                    let z = Rsa::mul(&x, &y);
                    for i in 0..n * 2 {
                        self.z[i] = z.get(i).copied().unwrap_or(0);
                    }
                    self.int_raw |= 1;
                }
            }
            0x81C => self.int_raw = 0, // CLEAR_INTERRUPT
            _ => {}
        }
    }

    pub fn read32(&self, off: u32) -> u32 {
        match off {
            0x000..=0x1FC => {
                let i = (off / 4) as usize;
                if i < NW { self.m[i] } else { 0 }
            }
            0x200..=0x3FC => {
                let i = ((off - 0x200) / 4) as usize;
                if i < NW { self.z[i] } else { 0 }
            }
            0x400..=0x5FC => {
                let i = ((off - 0x400) / 4) as usize;
                if i < NW { self.y[i] } else { 0 }
            }
            0x600..=0x7FC => {
                let i = ((off - 0x600) / 4) as usize;
                if i < NW { self.x[i] } else { 0 }
            }
            0x800 => self.m_dash,
            0x804 => self.nwords as u32,
            0x818 => self.int_raw, // QUERY_INTERRUPT
            0x82C => self.int_raw, // INTERRUPT
            _ => 0,
        }
    }

    /// True if a transform has completed (raw interrupt pending).
    pub fn int_pending(&self) -> bool {
        self.int_raw & 1 != 0
    }

    /// Clear the done interrupt (called by the interrupt handler).
    pub fn int_clear(&mut self) {
        self.int_raw = 0;
    }
}
