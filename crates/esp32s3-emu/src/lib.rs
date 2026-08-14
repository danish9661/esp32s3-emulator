//! ESP32-S3 emulator machine glue.
//!
//! P3 scope (see AGENTS.md roadmap): boot path — `rom_stub` (minimal boot
//! ROM at 0x40000000: flash app-image loader + UART puts), `asm` (hand
//! assembler for stubs/tests), `partition` (ESP-IDF partition table),
//! `Esp32S3::boot_from_flash`. Validated by hand-assembled firmware images
//! in `machine_tests`.

#![no_std]

#[cfg(test)]
extern crate std;

extern crate alloc;

pub mod asm;
mod machine;
#[cfg(test)]
mod machine_tests;
pub mod partition;
pub mod rom_stub;

pub use machine::Esp32S3;
