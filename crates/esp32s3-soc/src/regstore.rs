//! Generic register-store peripheral model.
//!
//! Many ESP32-S3 peripherals are "configure-and-forget" blocks that real
//! firmware writes during boot/init but whose side effects we do not need to
//! emulate for the core milestone (e.g. clock gating, memory-protection
//! permission tables, world-controller TEE state, peripheral backup/retention
//! registers). A `RegStore` simply retains every 32-bit write and returns it
//! on read, so firmware poking these blocks never panics, and direct
//! register-poke tests can verify round-trips. This matches the P5 validation
//! pattern used for `rtc_io.rs` / `sdmmc.rs` / `lp_uart.rs`.
//!
//! Functional models (e.g. `lcd_cam.rs` for the parallel-I/O data path)
//! replace the plain store where the peripheral drives observable output.

/// A minimal MMIO register block covering one full 4 KB APB page
/// (1024 × 32-bit registers). Page-aligned at `base` in the APB region.
pub struct RegStore {
    regs: [u32; 0x1000 / 4],
}

impl RegStore {
    pub fn new(_size: usize) -> Self {
        Self {
            regs: [0u32; 0x1000 / 4],
        }
    }

    /// Read the 32-bit register at `offset` (offset is the low 12 bits of the
    /// address within the peripheral's 4 KB page). Out-of-range offsets read 0.
    pub fn read32(&mut self, offset: u32) -> u32 {
        let i = ((offset & 0xFFF) / 4) as usize;
        *self.regs.get(i).unwrap_or(&0)
    }

    /// Write the 32-bit register at `offset`. Out-of-range offsets are dropped.
    pub fn write32(&mut self, offset: u32, value: u32) {
        let i = ((offset & 0xFFF) / 4) as usize;
        if let Some(r) = self.regs.get_mut(i) {
            *r = value;
        }
    }
}
