//! WASM <-> JS bridge for the ESP32-S3 emulator.
//!
//! Exposes a single [`Emulator`] handle to JavaScript via `wasm-bindgen`.
//! The browser drives execution by calling [`Emulator::step`] in an animation
//! loop, drains serial output with [`Emulator::uart_read`], and reflects GPIO
//! output levels with [`Emulator::gpio_output`].

use wasm_bindgen::prelude::*;

use esp32s3_emu::Esp32S3;

#[wasm_bindgen(start)]
pub fn start() {
    // Route Rust panics to the browser console for easier debugging.
    console_error_panic_hook::set_once();
}

/// A running ESP32-S3 machine. One instance per tab.
#[wasm_bindgen]
#[derive(Default)]
pub struct Emulator {
    inner: Esp32S3,
}

#[wasm_bindgen]
impl Emulator {
    /// Create an empty machine (reset state, no firmware loaded).
    #[wasm_bindgen(constructor)]
    pub fn new() -> Emulator {
        Emulator::default()
    }

    /// Install the boot ROM stub and a full flash image (the same layout the
    /// `run_flash` example consumes: bootloader @0, partitions @0x8000, app
    /// @0x10000) and reset the CPU to the ROM reset vector.
    pub fn load_flash(&mut self, flash: &[u8]) {
        self.inner.boot_from_flash(flash);
    }

    /// Advance the machine by `n` instruction-steps (both cores + timers).
    pub fn step(&mut self, n: u32) {
        for _ in 0..n {
            self.inner.step();
        }
    }

    /// Drain all pending UART/console bytes (UTF-8 serial output) and return
    /// them. Call this once per animation frame.
    pub fn uart_read(&mut self) -> Vec<u8> {
        self.inner.take_uart_tx(0)
    }

    /// Bitmask of GPIO output levels: bit `i` is the driven level of GPIO `i`.
    /// Drive the LED / pin visualization from this.
    pub fn gpio_output(&self) -> u32 {
        self.inner.gpio_output()
    }

    /// Core-0 program counter (debugging / sanity checks).
    pub fn pc(&self) -> u32 {
        self.inner.cpu[0].pc
    }
}
