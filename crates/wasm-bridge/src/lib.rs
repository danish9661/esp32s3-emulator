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
use xtensa_core::Bus;

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

    /// Push host-typed bytes into UART `n`'s RX FIFO (serial console input,
    /// `n` = 0/1/2). The firmware reads them like bytes from a serial
    /// terminal (e.g. the `uart_echo` sketch's `Serial1.read`, MicroPython's
    /// UART0 REPL). Caps at the 128-byte hardware FIFO depth — silicon drops
    /// overrun bytes the same way, so paste long text in chunks.
    pub fn uart_inject_rx(&mut self, n: u32, bytes: &[u8]) {
        for &b in bytes {
            self.inner.soc.uart_inject_rx(n as usize, b);
        }
    }

    /// Push host-typed bytes into the USB-Serial-JTAG RX FIFO (serial console
    /// input for firmware whose `Serial` is USB-CDC, the Arduino default on
    /// S3). Same 128-byte-class FIFO semantics as the UART path.
    pub fn usb_inject_rx(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.inner.soc.usb_inject_rx(b);
        }
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

    /// Provision the host fake quad-SPI device store on `chan` (persistent
    /// pattern for wide-mode USR transfers; see `Spi::quad_fake_provision`).
    pub fn spi_quad_fake_provision(&mut self, chan: u32, pattern: &[u8]) {
        self.inner
            .soc
            .spi_quad_fake_provision(chan as usize, pattern);
    }

    /// Quad/dual wire-mode level of `chan` (0 single, 1 dual, 2 quad).
    pub fn spi_quad_mode(&self, chan: u32) -> u32 {
        self.inner.soc.spi_quad_mode(chan as usize)
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

    /// Attach the SPI-mode SD card (SDSPI) on `chan` (0=GPSPI2, 1=GPSPI3),
    /// sharing the SDMMC card image (same FAT16 volume, so SPI `SD.begin`
    /// mounts what SDMMC formatted). Mirrors the `run_flash` SPI_SDSPI=1
    /// flow; idempotent within a boot (re-attaching resets card state).
    pub fn spi_sdspi_attach_sdmmc_image(&mut self, chan: u32) {
        self.inner.soc.spi_sdspi_attach_sdmmc_image(chan as usize);
    }

    /// Inject a touch counter value on pad 1..=14 (host frontend — drives
    /// what the firmware reads from the touch STATUS registers). Mirrors
    /// the `run_flash` TOUCH_INJECT=<pad>:<val> flow (e.g. `3:1877` for
    /// the gallery touch entry); call after load, before Run.
    pub fn touch_inject(&mut self, pad: u32, value: u32) {
        self.inner.soc.touch_inject(pad as usize, value);
    }

    /// Burn eFuse SECURE_BOOT_EN (fail-closed gate) through the real PGM
    /// path (BLK0 word 5 = REPEAT_DATA4 bit 20), mirroring the `run_flash`
    /// SECURE_BOOT_EN=1 fixture and the machine-test burn. Call BEFORE
    /// `load_flash` so `boot_from_flash` verifies the app region's
    /// signature sector; pair with a genuinely `espsecure.py sign-data`-
    /// signed image (see tools/sketches/esp32s3_secure_boot/). Unsigned
    /// images with the gate armed park both CPUs with no output (verified
    /// in-wasm: signed hello boots to `boot OK`, stock hello parks at 0
    /// insns / 0 UART bytes).
    pub fn secure_boot_enable(&mut self) {
        const EFUSE_BASE: u32 = 0x6000_7000;
        for (i, w) in [0u32, 0, 0, 0, 0, 1 << 20, 0, 0].iter().enumerate() {
            self.inner.soc.write32(EFUSE_BASE + (i as u32) * 4, *w);
        }
        self.inner.soc.write32(EFUSE_BASE + 0x1D4, 0x2); // PGM bit, BLK_NUM 0
    }

    /// Stage one camera frame (bytes, packed LE into words) for LCD_CAM
    /// capture. One frame per call; each `CAM_START` capture consumes the
    /// next staged frame.
    pub fn cam_inject_frame(&mut self, bytes: &[u8]) {
        self.inner.soc.cam_inject_frame(bytes);
    }

    /// Arm the Wi-Fi scan fixture (`WIFI_SCAN_APS` =
    /// `ssid,rssi,chan,bssid[;...]`, empty = empty air) for the wifi-scan
    /// image. Mirrors run_flash `WIFI_SCAN_FIXTURE=1`: the engine posts the
    /// REAL SCAN_DONE esp_event + writes fixture records into the calloc'd
    /// buffer, so `scanNetworks()` returns them unmodified. Call BEFORE
    /// load (the layout addresses are programmed into the Soc, and
    /// `boot_from_flash` → `Soc::new` inside `reset()` would wipe them —
    /// arming order is load-then-arm in main.js, so re-apply here).
    pub fn wifi_scan_fixture(&mut self, aps: &str) {
        self.inner.soc.wifi_fixture_image(false);
        self.inner.soc.wifi_fixture_scan(aps);
        self.inner.soc.wifi_fixture_layout_reapply();
    }

    /// Arm the Wi-Fi station-connect fixture (same AP list drives the
    /// association; fixed LAN 192.168.4.2/24 gw .1) for the wifi-sta image.
    /// Mirrors run_flash `WIFI_STA_CONN=1`: CONNECTED + GOT_IP posts plus
    /// the insider hooks serve `localIP()`/`SSID()`/`RSSI()` and the
    /// disconnect leg, so `waitForConnectResult` returns WL_CONNECTED.
    /// Call after load, before Run.
    pub fn wifi_sta_fixture(&mut self, aps: &str) {
        self.inner.soc.wifi_fixture_image(true);
        self.inner.soc.wifi_fixture_sta(aps);
        self.inner.soc.wifi_fixture_layout_reapply();
    }

    /// Arm the SoftAP fixture (SSID/passphrase/channel) for the wifi-ap
    /// image. Mirrors run_flash `WIFI_AP_FIXTURE=1`: the firmware posts
    /// AP_START itself; the engine stages the AP config (served back by
    /// the get/set-config hooks) + the fixed 192.168.4.1/24 LAN (served
    /// by the shared ip-info hook). Call after load, before Run.
    pub fn wifi_ap_fixture(&mut self, ssid: &str, passphrase: &str, channel: u32) {
        self.inner.soc.wifi_fixture_image_ap();
        self.inner
            .soc
            .wifi_fixture_ap(ssid, passphrase, channel as u8);
        self.inner.soc.wifi_fixture_layout_reapply();
    }

    /// Arm the ESP-NOW loopback fixture (virtual second node) for the
    /// espnow image. Mirrors run_flash `WIFI_ESPNOW_LOOPBACK=1`: once the
    /// sketch's `send()` returned (`sent 1` UART marker — the engine
    /// peeks the host console stream, same bytes run_flash greps), the
    /// engine invokes the registered TX wrapper in-firmware (`sent_ok`),
    /// then the peer `onReceive` directly (`got_rx`, `rx_byte0 = 0xA5`).
    /// Call after load, before Run.
    pub fn wifi_espnow_fixture(&mut self) {
        self.inner.soc.wifi_fixture_image_espnow();
        self.inner.soc.wifi_fixture_espnow();
        self.inner.soc.wifi_fixture_layout_reapply();
    }
}
