//! ESP32-S3 SoC: peripherals + full address-space `Bus` implementation.
//!
//! P2 scope (see AGENTS.md roadmap): memory map, UART0/1/2 (console TX),
//! GPIO, timer groups (TIMG0/1), interrupt matrix. Peripheral register
//! layouts are ported from QEMU's open-source ESP32-S3 models
//! (espressif/qemu, GPLv2) and the public ESP32-S3 TRM.

#![no_std]

extern crate alloc;

pub mod adc;
pub mod gpio;
pub mod i2c;
pub mod intc;
pub mod ledc;
pub mod memmap;
pub mod soc;
pub mod spi;
pub mod timg;
pub mod uart;

pub use soc::Soc;
