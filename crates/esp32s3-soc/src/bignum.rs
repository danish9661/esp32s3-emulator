//! Minimal big-integer arithmetic for the crypto peripheral models.
//!
//! Numbers are little-endian limb slices (limb 0 = least significant 32-bit
//! word), radix 2^32. These routines are a reference software implementation of
//! each hardware black box (RSA / DS / ECDSA) — they are intentionally NOT
//! constant-time (the silicon does the real work; the emulator only needs to
//! reproduce the documented numerical result).

use alloc::vec;
use alloc::vec::Vec;

use core::cmp::Ordering;

/// Drop leading (most-significant) zero limbs, returning a trimmed owned Vec.
pub(crate) fn trim(s: &[u32]) -> Vec<u32> {
    let mut v = s.to_vec();
    while !v.is_empty() && v[v.len() - 1] == 0 {
        v.pop();
    }
    v
}

/// Three-way comparison of two big integers.
pub(crate) fn cmp(a: &[u32], b: &[u32]) -> Ordering {
    let a = trim(a);
    let b = trim(b);
    if a.len() != b.len() {
        return a.len().cmp(&b.len());
    }
    for i in (0..a.len()).rev() {
        if a[i] != b[i] {
            return a[i].cmp(&b[i]);
        }
    }
    Ordering::Equal
}

/// `a + b` (no modulus).
pub(crate) fn add(a: &[u32], b: &[u32]) -> Vec<u32> {
    let n = a.len().max(b.len());
    let mut r = Vec::with_capacity(n + 1);
    let mut carry = 0u64;
    let mut ai = a.iter().copied().chain(core::iter::repeat(0u32));
    let mut bi = b.iter().copied().chain(core::iter::repeat(0u32));
    for _ in 0..n {
        let cur = ai.next().unwrap() as u64 + bi.next().unwrap() as u64 + carry;
        r.push(cur as u32);
        carry = cur >> 32;
    }
    r.push(carry as u32);
    trim(&r)
}

/// `a - b` assuming `a >= b` (no borrow; results undefined otherwise).
pub(crate) fn sub(a: &[u32], b: &[u32]) -> Vec<u32> {
    let n = a.len().max(b.len());
    let mut r = Vec::with_capacity(n);
    let mut borrow = 0i64;
    let mut ai = a.iter().copied().chain(core::iter::repeat(0u32));
    let mut bi = b.iter().copied().chain(core::iter::repeat(0u32));
    for _ in 0..n {
        let cur = ai.next().unwrap() as i64 - bi.next().unwrap() as i64 - borrow;
        if cur < 0 {
            r.push((cur + (1i64 << 32)) as u32);
            borrow = 1;
        } else {
            r.push(cur as u32);
            borrow = 0;
        }
    }
    trim(&r)
}

/// Schoolbook multiply; result has `a.len() + b.len()` limbs.
#[allow(clippy::needless_range_loop)]
pub(crate) fn mul(a: &[u32], b: &[u32]) -> Vec<u32> {
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
pub(crate) fn divmod(a: &[u32], b: &[u32]) -> (Vec<u32>, Vec<u32>) {
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
                && (qhat * v_m2) > ((rhat << 32) | rem.get(j + m - 2).copied().unwrap_or(0) as u64))
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
pub(crate) fn mod_m(r: &[u32], m: &[u32]) -> Vec<u32> {
    let m = trim(m);
    if m.is_empty() {
        return r.to_vec();
    }
    divmod(r, &m).1
}

/// `base^exp mod modulus` (all little-endian limb slices).
pub(crate) fn modexp(base: &[u32], exp: &[u32], modulus: &[u32]) -> Vec<u32> {
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
                let prod = mul(&result, &b);
                result = mod_m(&prod, &modulus);
            }
            limb >>= 1;
            if limb == 0 && i == e.len() - 1 {
                break;
            }
            let sq = mul(&b, &b);
            b = mod_m(&sq, &modulus);
        }
    }
    result
}

/// `a^{-1} mod m` via Fermat's little theorem (`a^{m-2} mod m`), valid for a
/// prime modulus `m` (the elliptic-curve base fields modeled here are prime,
/// so this is exact). Returns `a` unchanged if `m` is not prime or `a` is not
/// invertible; the EC arithmetic only ever inverts non-zero field elements.
pub(crate) fn modinv(a: &[u32], m: &[u32]) -> Vec<u32> {
    let m = trim(m);
    if m.is_empty() || (m.len() == 1 && m[0] == 1) || cmp(a, &m) != Ordering::Less {
        return a.to_vec();
    }
    let mut e = m.to_vec();
    // m is odd (prime > 2), so subtracting 2 only touches the low limb.
    e[0] = e[0].wrapping_sub(2);
    modexp(a, &e, &m)
}

/// Convert a big-endian byte buffer (e.g. a hex constant) into the model's
/// little-endian limb representation (limb 0 = least significant word).
pub(crate) fn from_be_bytes(be: &[u8]) -> Vec<u32> {
    let n = be.len();
    let mut limbs = vec![0u32; n.div_ceil(4)];
    for (i, &byte) in be.iter().enumerate() {
        // `power` = position of this byte from the least-significant end.
        let power = n - 1 - i;
        let limb = power / 4;
        let shift = (power % 4) * 8;
        limbs[limb] |= (byte as u32) << shift;
    }
    trim(&limbs)
}
