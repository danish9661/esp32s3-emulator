# ESP32-S3 Emulator (Rust → WASM)

A from-scratch **ESP32-S3 emulator** written in Rust, compiled to WebAssembly
(`wasm32-unknown-unknown`), and runnable entirely in the browser. The end goal
is to boot and run **real ESP-IDF / Arduino firmware** (serial output, GPIO,
peripherals) with no server-side emulation.

> Legal posture: the only references used are open source — Espressif's
> GPL-licensed QEMU Xtensa implementation (register layout / behavior), the
> public Xtensa ISA manual, and Espressif public TRMs. No closed-source
> emulator (Cirkit Designer, Wokwi, …) is decompiled or ported. Outcome license
> is GPL-compatible.

## Status

- **P0–P4 complete**: Xtensa LX7 core (decode/execute/windowed regs/exceptions),
  SoC (UART, GPIO, timers, interrupt matrix, SPI/I2C flash, PSRAM/cache MMU,
  dual-core), boot path (ROM stubs + second-stage loader + partition table),
  and a broad set of peripherals.
- **P5 (peripheral validation) essentially complete**: ~30 peripherals are
  validated end-to-end by booting **real arduino-cli firmware** and asserting
  both serial output and internal emulator state.
- **Remaining known gaps**: the Arduino `Wire` (I2C) *driver* path (the
  peripheral itself is validated via direct register pokes — see
  `AGENTS.md`); Xtensa `ee.*` DSP/TIE instructions (only needed for WiFi/FFT
  firmware); Touch peripheral (excluded by user directive).
- **Out of scope**: WiFi/BLE.

## Quickstart

### Build the emulator (CLI / tests)

```sh
cargo build --release -p esp32s3-emu
cargo test  --workspace            # all unit + machine tests
```

### Run real firmware

Build an Arduino/ESP-IDF sketch for the ESP32-S3 and run its merged image:

```sh
# 1. compile -> *.merged.bin (arduino-cli merges bootloader+app automatically)
arduino-cli compile -b esp32:esp32:esp32s3 \
    tools/sketches/esp32s3_hello --output-dir /tmp/build

# 2. boot it in the emulator
cargo run -q --release -p esp32s3-emu --example run_flash -- \
    /tmp/build/esp32s3_hello.ino.merged.bin
```

`run_flash` prints the firmware's UART output and exits when the step budget
(`STEPS = 48_000_000`) is reached. Many `tools/sketches/*` directories contain
pre-built `*.merged.bin` images you can run directly.

To inject an ADC voltage for analog sketches:

```sh
ADC_INJECT_MV=825 cargo run -q --release -p esp32s3-emu --example run_flash -- \
    tools/sketches/esp32s3_periph/esp32s3_periph.merged.bin
```

### Browser

The WASM build is wrapped by `crates/wasm-bridge` and served from `web/`:

```sh
wasm-pack build crates/wasm-bridge --target web --out-dir web/pkg
cd web && python3 -m http.server 8000   # open http://localhost:8000
```

The page has a serial console, a 40-pin GPIO LED grid, and an Examples
dropdown that loads bundled firmware from `web/firmware/`.

## Repository layout

```
crates/xtensa-core     CPU: decode, execute, windowed registers, exceptions (SoC-agnostic)
crates/esp32s3-soc     Peripherals: memory map, UART, GPIO, timers, INTC, flash,
                       SPI/I2C/RMT/GDMA/MCPWM/PCNT/TWAI/HMAC/DS/RSA/WDT/I2S/LCD_CAM/…
crates/esp32s3-emu     Machine glue: Bus impl, boot (ROM stubs), firmware loader, run_flash
crates/wasm-bridge     wasm-bindgen exports (Emulator struct) for the browser
web/                   static HTML/JS + wasm-pack build output + bundled firmware
tools/sketches/         arduino-cli sketches (one per validated peripheral) + merged bins
```

Core design: `xtensa-core` talks to memory only through a `Bus` trait
(`read8/16/32`, `write8/16/32`); the SoC implements `Bus`. Execution is
step-based: `Cpu::step(&mut bus)` runs one instruction and returns a
`StepResult`. The JS/browser driver runs N steps per frame.

## Adding / validating a peripheral

1. Model the peripheral in `crates/esp32s3-soc/src/<name>.rs` and wire it into
   `soc.rs` (mmio dispatch, `signal_level`, `int_pending`, `tick`).
2. Add unit tests in `crates/esp32s3-soc/tests/<name>.rs`.
3. Add an arduino-cli sketch under `tools/sketches/esp32s3_<name>` that exercises
   the **real driver** path, build it, and add a machine test in
   `crates/esp32s3-emu/src/machine_tests.rs` that boots the merged image and
   asserts the expected serial line / emulator state.

See `AGENTS.md` for the full per-peripheral history, conventions, and known
limitations.
