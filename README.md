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
- **P6 (browser frontend) complete**: Serial console, GPIO LED grid, firmware
  gallery with Examples dropdown, search, copy button.
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

The page has:
- **Serial console** — drains the UART buffer each frame and renders output
- **40-pin GPIO LED grid** — real-time pin state visualization
- **Examples dropdown** — 12 bundled firmware sketches (fetches + loads)
- **File input** — load your own `.merged.bin`
- **Run / Stop / Reset** — step-level control
- **Steps-per-frame slider** — tune emulation speed vs. responsiveness

## Repository Layout

```
crates/xtensa-core/     CPU: decode, execute, windowed registers, exceptions
                        (SoC-agnostic; talks to memory via a Bus trait)

crates/esp32s3-soc/     Peripherals: memory map, UART, GPIO, timers, INTC,
                        SPI/I2C/RMT/GDMA/MCPWM/PCNT/TWAI/HMAC/DS/RSA/
                        WDT/I2S/LCD_CAM/SHA/AES/ECDSA/EFUSE/RNG/SDMMC/…

crates/esp32s3-emu/     Machine glue: Bus impl, boot (ROM stubs), firmware
                        loader, run_flash example, machine tests

crates/wasm-bridge/     wasm-bindgen exports (Emulator struct) for the browser

web/                    Frontend: index.html + main.js + style.css +
                        wasm-pack output (web/pkg/) + bundled firmware
                        (web/firmware/)

tools/sketches/         Arduino-CLI sketches (one per validated peripheral)
                        with pre-built *.merged.bin images

help/                   Reference materials (gitignored)
```

## Architecture

### Core Design

```
┌─────────────────────────────────────────────────────────┐
│                     Browser (JS)                         │
│  main.js  →  wasm-bindgen  →  Emulator.step_batch(n)    │
│  Serial console │ GPIO grid │ Firmware gallery           │
└───────────────────────────┬─────────────────────────────┘
                            │ WASM boundary
┌───────────────────────────┴─────────────────────────────┐
│  wasm-bridge crate                                      │
│  Emulator { new, load_flash, step_batch, uart_read,    │
│             gpio_output, drain_events, inject_rx, … }   │
└───────────────────────────┬─────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────┐
│  esp32s3-emu crate (machine.rs)                         │
│  Esp32S3 { cpu: [Cpu; 2], soc: Soc, flash, … }         │
│  step() = tick_timers(1) + cpu[0].step() + cpu[1].step()│
│  load_image() / boot_from_flash()                       │
└──────┬──────────────────────────────────┬───────────────┘
       │ Bus trait                        │ Bus trait
┌──────┴──────────┐            ┌─────────┴──────────────┐
│  xtensa-core    │            │  esp32s3-soc            │
│  Cpu            │◄──read/write──► Soc                  │
│  - decode/exec  │            │  - UART, GPIO, timers   │
│  - windowed reg │            │  - SPI, I2C, RMT, etc.  │
│  - exceptions   │            │  - interrupt matrix     │
│  - interrupts   │            │  - flash/PSRAM backing  │
└─────────────────┘            └────────────────────────┘
```

### Execution Model

1. **Step-based**: `Cpu::step(&mut bus)` executes **one instruction** and
   returns a `StepResult` (normal / halted / exception). Tests and precise
   paths use this.
2. **Block-based (fast path)**: `Esp32S3::step_fast()` runs one cached
   straight-line block per core (≤16 instructions, branch op inclusive;
   only block lengths are cached, decode stays in the CPU cache) with
   interrupts once per core at the block end (QEMU TB granularity).
   Peripheral time advances inside the block at the exact single-step
   ratio. `run_flash` and the browser use this; it reports instructions
   executed.
2. **Dual-core serialized**: `Esp32S3::step()` runs `tick_timers(1)`, then
   core 0's instruction, then core 1's instruction — serialized per step.
3. **Peripheral ticks**: Every step, `tick_timers()` calls each peripheral's
   `tick()` method. Peripherals with nothing to do return immediately (fast
   path). I2C batches its FSM cycles when busy.
4. **Interrupt delivery**: After each instruction, `check_interrupts()` polls
   `int_pending()` (22 peripheral `int_st()` calls), builds a u128 bitmap,
   and dispatches through the interrupt matrix. A fast path skips the scan
   when `INTENABLE == 0` (early boot).
5. **JS driver**: The browser runs N steps per frame via
   `Emulator::step_batch(n)`, which also handles deep-sleep fast-forward.

### Boot Sequence

1. `load_flash(buf)` — host provides a merged binary (bootloader + partitions +
   app).
2. `boot_from_flash()` — parses the partition table, selects the active OTA
   slot, maps segments into DRAM/IRAM/IROM.
3. ROM stub at `0x40000000` — hand-assembled reset vector that copies segments
   and jumps to the app entry.
4. App runs — FreeRTOS boots, initializes drivers, prints via UART/USB-CDC.

### Windowed Register File

The Xtensa LX7 uses a windowed register file (64 registers across 8窗口).
The emulator implements this with a flat `ar: [u32; 64]` array + WINDOWBASE /
WINDOWSTART special registers. `rotate()` shifts the physical register mapping
on CALL/RETW, matching QEMU's `win_helper.c` behavior.

### Known Limitations

| Item | Status | Notes |
|------|--------|-------|
| I2C Wire driver | Peripheral validated, driver path hangs | esp-idf `cmd_link` completion ABI not modeled |
| ee.* DSP/TIE | Unimplemented | ~29 opcodes; only needed for WiFi/FFT firmware |
| Touch | Excluded | Per user directive |
| WiFi/BLE | Out of scope | Months of work; not required for core milestone |
| I2S TDM/PDM | Modeled | Master/slave clock-gen, TDM, PDM all functional |
| LCD_CAM 8080/6800 | Partial | FIFO + transfer-done functional; RGB FSM not modeled |
| ULP rv32imc | Functional | C extension supported; compressed decode working |
| Deep-sleep | Register model | esp-idf driver path hangs; direct-poke validated |
| SDMMC FAT mount | Block-level only | PIO + IDMAC + simulated card functional; SDIO/ACMD51/CMD6 negotiation for a FAT mount not modeled |
| Model clock | Approximate | 1 global tick per 2 instructions for all domains (silicon runs SYSTIMER 16MHz vs APB 80MHz); only observable in cross-domain counts, all passing |

## Performance

The emulator's per-step hot path costs approximately:

| Component | Per-Step | Notes |
|-----------|----------|-------|
| `tick_timers()` | ~17 peripheral tick calls | Most idle → return immediately |
| `cpu[0].step()` | decode + execute + check_interrupts | ~1 instruction per call |
| `cpu[1].step()` | same | serialized after core 0 |
| `check_interrupts()` | 22 peripheral `int_st()` + matrix | Fast path when INTENABLE==0 |

### Optimizations Applied

- **I2C batch tick**: When the I2C bus is busy, `tick(n)` runs all remaining
  cycles in one call instead of 1-per-step. When idle, the tick is skipped
  entirely. Saves ~3.35M function calls per boot.
- **I2C Vec elimination**: `tick()` buffers events internally; `drain_events()`
  returns an iterator. No heap allocation on the hot path.
- **INTENABLE fast path**: `check_interrupts` skips the 22-peripheral scan when
  no interrupts are globally enabled and no software bits are set.
- **Peripheral idle fast paths**: Systimer, TIMG, ADC, SPI all have early-return
  checks when disabled/idle.
- **Block-at-a-time execution**: `step_fast()` runs cached straight-line
  blocks (≤16 instructions) per core with interrupts once per block.
- **PGO (opt-in)**: `tools/pgo.sh` trains an LLVM profile over hello+periph
  boots and rebuilds — ~1.45× throughput on native x86_64 (native target
  only, not for wasm builds).

## Adding / Validating a Peripheral

1. **Model** the peripheral in `crates/esp32s3-soc/src/<name>.rs` and wire it
   into `soc.rs` (mmio dispatch, `signal_level`, `int_pending`, `tick`).
2. **Unit tests** in `crates/esp32s3-soc/tests/<name>.rs`.
3. **Arduino-CLI sketch** under `tools/sketches/esp32s3_<name>` that exercises
   the **real driver** path, build it, and add a machine test in
   `crates/esp32s3-emu/src/machine_tests.rs` that boots the merged image and
   asserts the expected serial line / emulator state.
4. **Browser validation**: add the firmware to `web/firmware/manifest.json` and
   verify it loads and runs in the browser UI.

See `AGENTS.md` for the full per-peripheral history, conventions, and known
limitations.

## Testing

```sh
# Run all tests (unit + integration + machine)
cargo test --workspace

# Run a specific crate's tests
cargo test -p esp32s3-soc --lib --tests
cargo test -p xtensa-core --lib
cargo test -p esp32s3-emu --lib

# Build for WASM (no tests — just compile check)
cargo build --target wasm32-unknown-unknown -p wasm-bridge

# Clippy (host + WASM)
cargo clippy --workspace
cargo clippy -p wasm-bridge --target wasm32-unknown-unknown
```

## Toolchain Requirements

| Tool | Version | Purpose |
|------|---------|---------|
| rustc/cargo | 1.97.1+ | Core build |
| wasm-pack | 0.14.0+ | WASM build + wasm-opt |
| arduino-cli | 1.5.1+ | Build real ESP32-S3 firmware |
| esp32 core | 3.3.10 | Arduino ESP32 board support |
| esptool.py | ~/.local/bin | `merge_bin` for flash images |
| node | 22+ | Browser tests (optional) |
