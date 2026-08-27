//! WASM <-> JS bridge for the ESP32-S3 emulator.
//!
//! Exposes a single [`Emulator`] handle to JavaScript via `wasm-bindgen`.
//! The browser drives execution by calling [`Emulator::step`] in an animation
//! loop, drains serial output with [`Emulator::uart_read`], reflects GPIO
//! output levels with [`Emulator::gpio_output`], and receives
//! protocol-level events (GPIO/SPI/I2C) via [`Emulator::drain_events`] for
//! Wokwi-style virtual peripherals.

use wasm_bindgen::prelude::*;

use esp32s3_emu::Esp32S3;

#[wasm_bindgen(start)]
pub fn start() {
    // Route Rust panics to the browser console for easier debugging.
    console_error_panic_hook::set_once();
}

/// A host-observable emulator event (GPIO/SPI/I2C), drained once per frame.
///
/// `kind` discriminates the event; `a`/`b` carry scalar payload (see
/// `esp32s3_soc::EmuEvent` for the per-kind layout).
#[wasm_bindgen]
#[derive(Clone)]
pub struct EmuEvent {
    pub kind: u8,
    pub a: u32,
    pub b: u32,
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

    /// Drain all host-observable events (GPIO edges + SPI/I2C transactions)
    /// accumulated since the last call. Dispatch each to the matching virtual
    /// peripheral. Call this once per animation frame (after `step`).
    pub fn drain_events(&mut self) -> Vec<EmuEvent> {
        self.inner
            .soc
            .drain_events()
            .into_iter()
            .map(|e| EmuEvent {
                kind: e.kind,
                a: e.a,
                b: e.b,
            })
            .collect()
    }

    /// Inject MISO bytes for the next SPI transfer on `chan`
    /// (0=GPSPI2, 1=GPSPI3). After an `EVT_SPI_XFER` event, call
    /// [`Emulator::spi_take_tx`] to read what the MCU sent, then this to feed
    /// the virtual device's response back.
    pub fn spi_inject_miso(&mut self, chan: u32, bytes: &[u8]) {
        self.inner.soc.spi_inject_miso(chan as usize, bytes);
    }

    /// Take the MOSI byte stream of the most recent SPI transfer on `chan`
    /// (0=GPSPI2, 1=GPSPI3). Call this when an `EVT_SPI_XFER` event arrives.
    pub fn spi_take_tx(&mut self, chan: u32) -> Vec<u8> {
        self.inner.soc.spi_take_tx(chan as usize)
    }

    /// Inject RX bytes for the next I2C master-read on `chan`
    /// (0=I2CEXT0, 1=I2CEXT1). Each byte is returned to the MCU on a READ; an
    /// empty supply reads back 0xFF (no device).
    pub fn i2c_inject_rx(&mut self, chan: u32, bytes: &[u8]) {
        self.inner.soc.i2c_inject_rx(chan as usize, bytes);
    }
}
