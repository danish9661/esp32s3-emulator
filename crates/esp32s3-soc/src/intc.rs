//! ESP32-S3 Interrupt Matrix.
//!
//! Maps peripheral interrupt sources to CPU interrupt lines. Layout per QEMU
//! `hw/xtensa/esp32s3_intc.c`: register space at INT_MATRIX_BASE is
//! 512 sources × 2 CPUs × 4 bytes; entry (cpu, source) at
//! base + 4*(cpu*512 + source); written value & 0x1f selects the CPU
//! interrupt line; reset value = 6 (INTMATRIX_UNINT_VALUE, unmapped).
//!
//! P2 scope: the mapping table only. Peripheral sources (UART TX_DONE, timer
//! alarms, ...) can assert a line via `pending_lines()` which resolves the
//! matrix; the CPU-side dispatch is added in P3 (boot path / interrupts).

use crate::memmap::*;

pub struct Intc {
    /// irq_map[cpu][source] = CPU interrupt line.
    irq_map: [[u8; INT_MATRIX_INPUTS]; INT_MATRIX_CPUS],
}

impl Intc {
    pub fn new() -> Self {
        Self {
            irq_map: [[6; INT_MATRIX_INPUTS]; INT_MATRIX_CPUS],
        }
    }
}

impl Default for Intc {
    fn default() -> Self {
        Self::new()
    }
}

impl Intc {
    pub fn read32(&mut self, offset: u32) -> u32 {
        let entry = self.entry(offset);
        entry
            .map(|(cpu, src)| self.irq_map[cpu][src] as u32)
            .unwrap_or(0)
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        if let Some((cpu, src)) = self.entry(offset) {
            self.irq_map[cpu][src] = (value & 0x1F) as u8;
        }
    }

    fn entry(&self, offset: u32) -> Option<(usize, usize)> {
        let idx = (offset / 4) as usize;
        if idx >= INT_MATRIX_INPUTS * INT_MATRIX_CPUS {
            return None;
        }
        Some((idx / INT_MATRIX_INPUTS, idx % INT_MATRIX_INPUTS))
    }

    /// Resolve asserted peripheral sources (bitmap, bit n = matrix source
    /// n high) to the CPU interrupt lines asserted on `cpu` (QEMU
    /// esp32s3_intc.c irq_map: each source asserts the line stored in
    /// INT_MATRIX(cpu, source); 32 lines, all values 0..=31 are legal
    /// lines, 6 is merely the reset value).
    pub fn pending_lines(&self, cpu: usize, sources: u64) -> u32 {
        let mut lines = 0u32;
        let mut s = sources;
        while s != 0 {
            let i = s.trailing_zeros() as usize;
            s &= s - 1;
            lines |= 1u32 << self.irq_map[cpu][i];
        }
        lines
    }
}
