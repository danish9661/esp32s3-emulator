//! ESP32-S3 SoC: peripherals + full address-space `Bus` implementation.
//!
//! P2 scope (see AGENTS.md roadmap): memory map, UART0/1/2 (console TX),
//! GPIO, timer groups (TIMG0/1), interrupt matrix. Peripheral register
//! layouts are ported from QEMU's open-source ESP32-S3 models
//! (espressif/qemu, GPLv2) and the public ESP32-S3 TRM.

#![no_std]

extern crate alloc;

pub mod adc;
pub mod aes;
pub mod bignum;
pub mod cache;
pub mod ds;
pub mod ecdsa;
pub mod efuse;
pub mod gdma;
pub mod gpio;
pub mod hmac;
pub mod i2c;
pub mod i2s;
pub mod intc;
pub mod lcd_cam;
pub mod ledc;
pub mod lp_uart;
pub mod mcpwm;
pub mod memmap;
pub mod memspi;
pub mod pcnt;
pub mod regstore;
pub mod rmt;
pub mod rng;
pub mod rsa;
pub mod rtc;
pub mod rtc_i2c;
pub mod rtc_io;
pub mod sdmmc;
pub mod sha;
pub mod sigmadelta;
pub mod soc;
pub mod spi;
pub mod systimer;
pub mod timg;
pub mod touch;
pub mod twai;
pub mod uart;
pub mod uhci;
pub mod ulp;
pub mod usb_otg;
pub mod usb_serial_jtag;

pub use soc::Soc;
