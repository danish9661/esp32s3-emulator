//! ESP32-S3 machine: CPU + SoC glue, firmware loading, step loop.
//!
//! The CPU is SoC-agnostic; the machine wires `xtensa-core::Cpu` to the
//! `esp32s3-soc::Soc` address space and exposes the host-facing API
//! (load image, step, read console output / GPIO).

use alloc::vec::Vec;
use esp32s3_soc::Soc;
use esp32s3_soc::memmap::{DRAM_BASE, IRAM_BASE, IROM_BASE, IROM_SIZE, SRAM_BYTES};
use xtensa_core::{Bus, Cpu, StepResult};

use crate::rom_stub;

pub struct Esp32S3 {
    /// Both ESP32-S3 LX7 cores.  Core 1 is gated at reset by the ROM stub
    /// (rom_stub.rs: PRID check) until core 0 releases it, mirroring the
    /// real ROM's APP CPU boot flow (QEMU esp32s3.c runs both CPUs and lets
    /// the ROM gate CPU1 — no release register is modeled).
    pub cpu: [Cpu; 2],
    pub soc: Soc,
}

impl Esp32S3 {
    pub fn new() -> Self {
        Self {
            cpu: [Cpu::new(0), Cpu::new(1)],
            soc: Soc::new(),
        }
    }

    /// Execute one instruction on each core.  Timers advance one cycle per
    /// step so they make progress in host-driven execution (refined in P5).
    /// The two cores are serialized core0-then-core1 within a step; real
    /// silicon runs them simultaneously, but the fixed order keeps timer
    /// ticks and per-core instruction counts identical to the single-core
    /// behavior the machine tests were written against.
    pub fn step(&mut self) -> StepResult {
        self.soc.tick_timers(1);
        let r = self.cpu[0].step(&mut self.soc);
        self.cpu[1].step(&mut self.soc);
        r
    }

    /// Load a raw firmware image at `addr` (DRAM, IRAM or IROM window).
    pub fn load_image(&mut self, addr: u32, bytes: &[u8]) {
        for (i, b) in bytes.iter().enumerate() {
            let a = addr + i as u32;
            if (DRAM_BASE..DRAM_BASE + SRAM_BYTES as u32).contains(&a)
                || (IRAM_BASE..IRAM_BASE + SRAM_BYTES as u32).contains(&a)
            {
                self.soc.write8(a, *b as u32);
            } else if (IROM_BASE..IROM_BASE + IROM_SIZE).contains(&a) {
                // The Bus write path treats IROM as read-only; the ROM
                // storage is written directly when hosting loads it.
                self.soc.irom_mut()[(a - IROM_BASE) as usize] = *b;
            }
        }
    }

    /// Install the boot ROM stub and a full flash image, then reset the CPU
    /// to the ROM reset vector. The ROM stub (rom_stub.rs) loads the app
    /// image from flash offset `APP_FLASH_OFFSET` and jumps to its entry.
    pub fn boot_from_flash(&mut self, flash: &[u8]) {
        self.soc.load_flash_image(0, flash);
        let rom = rom_stub::rom_image();
        self.load_image(rom_stub::ROM_BASE, &rom);
        self.cpu[0].pc = rom_stub::ROM_BASE;
    }

    /// Bytes emitted by UART `n` since the last call (console output).
    pub fn take_uart_tx(&mut self, n: usize) -> Vec<u8> {
        self.soc.take_uart_tx(n)
    }

    /// Output-pin state (host LED visualization).
    pub fn gpio_output(&self) -> u32 {
        self.soc.gpio_output()
    }
}

impl Default for Esp32S3 {
    fn default() -> Self {
        Self::new()
    }
}
