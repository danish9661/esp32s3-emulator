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

    /// Install a plaintext image as a factory-encrypted device: provision
    /// the eFuse XTS key, uniformly encrypt the image (every 16-byte block
    /// at its absolute offset, like esptool), then boot the ciphertext.
    /// `key` must hold exactly 32 bytes. Used by the gallery's
    /// flash-encryption demo (mirrors `run_flash`'s FLASHENC_KEY flow).
    pub fn load_flash_encrypted(&mut self, flash: &[u8], key: &[u8]) {
        assert_eq!(key.len(), 32, "flash-encryption key must be 32 bytes");
        assert_eq!(flash.len() % 16, 0, "image must be 16-byte aligned");
        let mut k = [0u8; 32];
        k.copy_from_slice(key);
        self.inner.soc.flashenc_provision(&k);
        self.inner.soc.load_flash_image(0, flash);
        self.inner
            .soc
            .flashenc_encrypt_region(0, flash.len() as u32);
        let enc = self.inner.soc.flash_image().to_vec();
        self.inner.boot_from_flash(&enc);
    }

    /// Advance the machine by `n` instruction-steps (both cores + timers).
    pub fn step(&mut self, n: u32) {
        for _ in 0..n {
            self.inner.step();
        }
    }

    /// Batch step with early-exit on reset/sleep. Returns the number of
    /// INSTRUCTIONS actually executed (blocks run whole straight-line runs
    /// via `step_fast`). This avoids wasting cycles after a WDT reset or
    /// deep-sleep entry — the caller can re-prime peripherals and continue.
    /// Deep-sleep is fast-forwarded inline (no JS round-trip per tick).
    /// The `n` budget is scaled ×2: one legacy step ran one instruction per
    /// core (2 total), so `2*n` instructions is the same per-frame work and
    /// emulated time as before.
    pub fn step_batch(&mut self, n: u32) -> u32 {
        let budget = n.saturating_mul(2);
        let mut done = 0u32;
        let mut calls = 0u32;
        // `calls` bounds the loop for non-advancing macro-steps (WDT reset /
        // sleep entry report 0 instructions but still consume the call, as
        // the old per-step loop did).
        while done < budget && calls < n.max(1) {
            calls += 1;
            // If deep-sleeping, fast-forward inline — the CPU is halted.
            if self.inner.is_asleep() {
                let skip = self.inner.fast_forward_sleep((budget - done) as u64);
                done = done.saturating_add(skip as u32);
                continue;
            }
            let (_, _, k) = self.inner.step_fast();
            done = done.saturating_add(k);
        }
        done
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

    /// Stage one camera frame (bytes, packed LE into words) for LCD_CAM
    /// capture. One frame per call; each `CAM_START` capture consumes the
    /// next staged frame.
    pub fn cam_inject_frame(&mut self, bytes: &[u8]) {
        self.inner.soc.cam_inject_frame(bytes);
    }
}
