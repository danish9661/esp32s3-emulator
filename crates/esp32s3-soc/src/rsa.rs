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

use crate::bignum::*;

/// RSA register-block base (`DR_REG_RSA_BASE`).
pub const RSA_BASE: u32 = 0x6003_C000;

/// Max operand size: 4096-bit RSA = 128 words (each block is 0x200 bytes).
const NW: usize = 128;

/// RSA interrupt source for the interrupt matrix (esp32s3 interrupts.h
/// `ETS_RSA_INTR_SOURCE` = 76).
pub const RSA_INTR_SOURCE: u32 = 76;

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

impl Rsa {
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
        let z = modexp(&x, &y, &m);
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
                    let prod = mul(&x, &y);
                    let z = mod_m(&prod, &m);
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
                    let z = mul(&x, &y);
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
