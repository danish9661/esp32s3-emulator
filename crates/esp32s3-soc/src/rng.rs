//! ESP32-S3 hardware random number generator (RNG / WDEV_RND).
//!
//! Base `DR_REG_RNG_BASE = 0x6003_5000`; the random data register is
//! `WDEV_RND_REG = 0x6003_507C` (TRM "WDEV_RND_REG"; esp-idf `esp_random()`
//! reads it). On S3 there is no control register required — the generator
//! runs continuously and every read returns 32 bits of entropy.
//!
//! The model is deterministic (a seeded LCG) so unit tests are reproducible,
//! while still producing varying, non-repeating values across consecutive
//! reads like real hardware.

pub const RNG_BASE: u32 = 0x6003_5000;
const DATA_OFF: u32 = 0x7C; // WDEV_RND_REG

// Numerical-Recipes LCG constants — full 32-bit period; every step differs
// from the previous for these multipliers, so consecutive reads never repeat.
const LCG_MULT: u64 = 1664525;
const LCG_INC: u64 = 1013904223;

// Cover the full 0x6003_5000 page (shared with the WiFi WDEV TSF/timer
// block at +0x00..0x70; the RNG data register is at +0x7C). Plain stores
// below +0x7C must not alias the data register (idx masks the full page).
const REG_COUNT: usize = 0x1000 / 4;

pub struct Rng {
    state: u32,
    regs: [u32; REG_COUNT],
}

impl Default for Rng {
    fn default() -> Self {
        Self {
            state: 0x1234_5678,
            regs: [0u32; REG_COUNT],
        }
    }
}

impl Rng {
    pub fn new() -> Self {
        Self::default()
    }

    /// Reseed the LCG (host frontend: `RNG_SEED=<u32>` varies the stream
    /// across runs while staying deterministic within a run — firmware
    /// RNG consumers get different streams on demand; the default seed
    /// keeps unit tests reproducible).
    pub fn reseed(&mut self, seed: u32) {
        self.state = seed;
    }

    fn idx(&self, offset: u32) -> usize {
        ((offset & 0xFFF) / 4) as usize
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        if (offset & 0xFFF) == DATA_OFF {
            // Advance the LCG and return the new state as "entropy".
            let next = (self.state as u64)
                .wrapping_mul(LCG_MULT)
                .wrapping_add(LCG_INC) as u32;
            self.state = next;
            next
        } else {
            self.regs[self.idx(offset)]
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let i = self.idx(offset);
        if i < REG_COUNT {
            self.regs[i] = value;
        }
    }
}
