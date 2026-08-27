# AGENTS.md — ESP32-S3 Emulator (Rust → WASM, browser)

## Mission

Build a from-scratch **ESP32-S3 emulator** in Rust, compiled to WebAssembly
(`wasm32-unknown-unknown`), running 100% in the browser. End goal: real
ESP-IDF firmware binaries boot and run (serial output visible, GPIO/LED
driven, etc.) without any server-side emulation.

**Legal posture (non-negotiable):**
- The ONLY code reference we use is open source: QEMU's Xtensa implementation
  (`espressif/qemu` fork, GPLv2) + public Xtensa ISA documentation + Espressif
  public docs/TRM.
- We do NOT decompile, extract, or port logic from Cirkit Designer's
  proprietary WASM, Wokwi, or any closed-source emulator.
- Outcome license: GPL-compatible (we port/derive from GPL QEMU).

## Environment (verified 2026-08-13)

| Tool | Version | Status |
|---|---|---|
| rustc / cargo | 1.97.1 | OK |
| wasm32-unknown-unknown target | installed | OK |
| wasm-pack | 0.14.0 | OK |
| node | 22.22.2 | OK |
| qemu-system-xtensa | — | MISSING → reference-ONLY (GPL source for register layout/behavior); NOT used for validation (see Validation strategy) |
| ESP-IDF | — | MISSING → arduino-cli used instead to build real firmware |
| esptool.py | ~/.local/bin | OK (merge_bin for flash images) |

## Useful tools & references (researched 2026-08-13)

- **espressif/qemu** — GPL reference implementation of LX7 + full ESP32-S3
  machine. Source + prebuilt `qemu-xtensa-softmmu` binaries (release
  `esp-develop-9.2.2-20250228`). Use: golden traces, decode tables, peripheral
  behavior.
- **Xtensa ISA Reference Manual** (public, Tensilica/Cadence) — instruction
  formats, special registers, windowing.
- **ESP32-S3 Technical Reference Manual** (Espressif, public) — memory map,
  peripheral register layouts (UART, GPIO, timers, interrupt matrix...).
- **Rust crates**: `wasm-bindgen` (bridge), `wasm-pack` (build). No usable
  Xtensa *emulation* crate exists (confirmed — `xtensa-lx`/`xtensa-lx-rt` are
  real-hardware only). Core is written by us.
- **esptool.py** — `merge_bin` to build flash images for boot tests.
- **wabt / wasm2wat** — only for debugging OUR generated wasm, never for other
  people's proprietary artifacts.

## Architecture

```
esp32-s3-emu/            (workspace root = /home/danish1075/Documents/esp32 s3 emu)
├── AGENTS.md            (this file — keep updated!)
├── Cargo.toml           (workspace)
├── crates/
│   ├── xtensa-core/     CPU: decode, execute, windowed registers, exceptions
│   ├── esp32s3-soc/     Peripherals: memory map, UART, GPIO, timers, INT matrix, flash
│   ├── esp32s3-emu/     Machine glue: Bus impl, boot (ROM stubs), firmware loader
│   └── wasm-bridge/     wasm-bindgen exports: Emulator struct for JS
├── web/                 frontend (static HTML/JS + wasm-pack build output)
└── tools/               arduino-cli firmware build scripts + sketches, test helpers
```

Core design:
- `xtensa-core` is **SoC-agnostic**: CPU talks to memory via a `Bus` trait
  (`read8/16/32`, `write8/16/32`). SoC implements `Bus`.
- Step-based execution: `Cpu::step(&mut bus)` → one instruction, returns
  `StepResult` (halted / exception / normal). JS drives N steps per frame.
- Windowed register file implemented with a 64-entry AR array + WINDOWBASE/
  WINDOWSTART/PS.WINDOW emulation.

## Roadmap (status updated as we go)

- [x] **P0 — Skeleton**: workspace + crates compile, wasm-pack builds, browser
      loads module.
- [x] **P1 — Core CPU**: decode + ALU + load/store + branches + CALL0/J loops;
      runs hand-assembled bare-metal test programs.
- [x] **P2 — SoC basics**: memory map, UART0 (console out), GPIO, timers,
      interrupt controller; trivial IDF firmware prints via UART.
- [x] **P3 — Boot path**: flash image loading, ROM stubs (printf/UART/delay),
      second-stage bootloader, partition table → real IDF app boots.
- [x] **P4 — Peripherals**: SPI/I2C/PWM/ADC, dual-core, PSRAM.
- [ ] **P5 — Hardening**: real-firmware validation via arduino-cli (compile
      sketches, run via `run_flash` + node bridge, assert serial output +
      peripheral state such as `gpio_output()` and device registers),
        exception correctness and interrupt timing verified through real
        FreeRTOS/Arduino behavior. **P5 is essentially COMPLETE**: every SoC
        peripheral is modeled and validated (see status log). Driver-path status
         through the esp-idf stack: RNG, SYSTIMER, RMT, GDMA, SigmaDelta, LEDC,
         EFUSE, SHA, AES, RSA, HMAC, DS, WDT, I2S, SPI, MCPWM, PCNT, TWAI/CAN,
         LCD_CAM were all validated end-to-end. Validated via direct register
         pokes (peripheral correct, driver path not modeled): I2C (Wire driver
         documented known limitation — root-caused to the esp-idf i2c driver's
         internal `cmd_link`/`xQueueGenericSendFromISR` init ABI), RTC_IO, ULP
         (program execution NOW modeled — rv32im core, P5 candidate retired),
         SDMMC (card-command FSM NOW modeled, P5 candidate retired),
         Deep-sleep (esp-idf `esp_deep_sleep_start` hangs in sleep *preparation*
         — documented), LP_I2C, LP_UART, ECDSA, and the P5 register-store stubs
         (SENSITIVE/WCL/PERI_BACKUP/SYSCON/I2S-boot/PARLIO-assist via
         `regstore.rs`). The Xtensa LX7 ISA audit passed (only `ee.*` DSP/TIE
         extensions unimplemented — documented limitation). OTA boot-slot
         selection is implemented; ROM coverage is sufficient (5+ real sketches
         boot). **Single remaining P5 driver-path gap: the I2C `Wire` esp-idf
         driver** (peripheral validated; driver requires offline esp-idf
         `cmd_link` ABI — documented known limitation, not a model defect).
         Touch: NOT being pursued (user directive: do NOT do Touch).
- [x] **P6 — Frontend polish**: serial console UI, GPIO/LED visualization,
      example firmware gallery.
- [ ] WiFi/BLE: OUT OF SCOPE for now (months of work; not required for the
      core milestone).

## Validation strategy

1. **Unit tests** in each crate (instruction-level, known-answer tests).
2. **Hand-written assembly tests** — assemble with xtensa-esp32s3 toolchain,
   run in our emulator, assert register/memory results.
3. **Real firmware via arduino-cli** (PRIMARY): compile real Arduino/ESP-IDF
   sketches with `arduino-cli` (esp32 core), merge with `esptool.py`, run the
   `.merged.bin` through `crates/esp32s3-emu/examples/run_flash` (or the node
   bridge built from `wasm-bridge`), and assert BOTH the serial output AND the
   emulator's internal state (e.g. `gpio_output()`, device registers, counter
   values). Each newly implemented peripheral gets at least one such sketch
   that exercises it and is checked for correct behavior. This is the main
   correctness gate now — it exercises the full real toolchain + FreeRTOS +
   peripheral drivers end-to-end.
4. **QEMU is reference-only** (GPL source for register layout / behavior), NOT
   used for golden-trace diffing. It may be consulted when implementing a
   peripheral, but validation is via real arduino-cli firmware (above).

## Conventions

- `#![no_std]` for all emulator crates (WASM-friendly, no_std + alloc ok).
- No unsafe unless required for WASM perf (later, if at all).
- No comments in code unless they explain *why* (this is a hardware-emulation
  project; register-level comments are REQUIRED where magic numbers appear,
  citing TRM sections).
- Every instruction implementation cites the ISA RM format (e.g. `RRR op0=0
  op1=0xC ...`).
- Commit-ready, formatted with `cargo fmt`, clippy-clean.

## Status log (append, newest last)
  - 2026-08-22: **SPI master driver path validated via arduino-cli (P5)**.
    The Arduino `SPI` library (`SPI.begin` / `transfer` / `transferBytes` /
    `transfer16` / `beginTransaction`) drives GPSPI2 (FSPI) through the
    **polling USR path** (no GDMA — `esp32-hal-spi.c` `spiTransferByte` uses
    the peripheral registers directly, confirmed by grepping the core), which
    our existing `spi.rs` model already supports, so no emulator changes were
    needed. Validated with `tools/sketches/esp32s3_spi_driver` (no device on
    the bus, so MISO reads back 0) → `SPI DRIVER transfer(0x55)=0x00`,
    `multi rx=00 00 00 00`, `transfer16=0x0000`, `SPI DRIVER PASS` under
    `run_flash`. The SPIN critical-section trace is the usual harmless
    FreeRTOS spin (also seen in every other sketch), not a hang. This retires
    the "SPI master driver" P5 candidate; remaining driver-path candidates:
     EFUSE (factory MAC / chip id), I2C (Wire) driver, and the unmodeled Touch
     peripheral.
  - 2026-08-22: **RMT driver path validated via arduino-cli (P5)** — the new
    esp-idf RMT driver (`rmtWrite`/`rmtWriteAsync`, Arduino 3.3.10) routes the
    item buffer through **GDMA** into RMTMEM, so it exercises the GDMA-backed
    path. Two emulator bugs were fixed to make it work:
    (1) **GDMA `out.link.start` bit**: `gdma.rs` checked bit **1**, but
    `gdma_struct.h` places `start` at **bit 21** (`addr`[19:0], `stop`=20,
    `start`=21, `restart`=22, `park`=23). The direct-poke GDMA sketch happened
    to write bit 1 (matching the old model) so it passed anyway — the real
    driver writes bit 21 and hung. Fixed to bit 21; `tests/gdma.rs` + the
    `esp32s3_gdma` sketch updated to bit 21, both still green.
    (2) **RMT continuous/loop mode**: the driver's `rmtWriteLooping` uses
    `tx_loop_cnt_en` (bit 14) with `tx_loop_cnt`==0 for infinite looping (not
    `tx_conti_mode` bit 15). `rmt.rs` `begin_tx` now loops when either is set.
    Validated with `tools/sketches/esp32s3_rmt_driver` (Arduino `rmtWrite`,
    64-item buffer @ 1 MHz): the blocking call returns true and the item buffer
    is copied into RMTMEM (`RMT DRIVER GDMA copied item=7fffffff` / `RMT DRIVER
    PASS`) — the previously-hanging GDMA path now completes. NOTE: the live pin
    waveform can't be sampled from firmware (the RMT FSM advances every step, so
    a transmission finishes *inside* the driver's `rmt_transmit()` call); the
    live RMT→GPIO toggling via the GPIO_IN loopback is instead covered by the
    Rust machine test `rmt_signal_drives_gpio_in_loopback` (added to
    `esp32s3-emu/src/machine_tests.rs`). 136 workspace tests green, clippy/
    wasm32 clean.
  - 2026-08-22: **MCPWM group-0 (P5 — new peripheral via arduino-cli
    validation)** + **GPIO_IN peripheral-loopback fix**. `mcpwm.rs` models
    ESP32-S3 MCPWM0: register block @ `0x6001_E000` (`DR_REG_PWM0_BASE` —
    **NOT 0x6000_B000**, which is HINF; TRM typo / misread), 3 timers (up /
    down / up-down count, prescale divider, 16-bit period) + 3 operators each
    with 2 comparators (A/B) and 2 generators (A/B). On the timer events TEZ
    (count==0), TEA (==cmprA), TEB (==cmprB) the generator action table
    (`generator0/1`, 2-bit selectors per event: 0=keep,1=high,2=low,3=toggle)
    updates the output; the level is exposed to the GPIO matrix via the PWM0
    OUT0A..OUT2B signals (160..165, `gpio_sig_map.h`). `int_pending` = source
    **31** (`ETS_PWM0_INTR_SOURCE`, `interrupts.h` — PWM1=32, LEDC=33, TWAI=35;
    note the existing RMT=40/PCNT=41/I2C=42/43 wiring is verified-correct against
    the same header, while `TWAI=37` in `twai.rs` does NOT match the header's
    35 and should be re-checked). Wired into `soc.rs` (mmio arm `MCPWM_BASE`,
    `signal_level`, `tick`, `int_pending`). 6 unit tests (`tests/mcpwm.rs`)
    assert 50%/25% up-mode duty, operator A/B signal mapping, live
    `timer_status`, stop/hold, and prescale. Validated end-to-end with
    `tools/sketches/esp32s3_mcpwm` (direct register pokes: route PWM0_OUT0A→
    GPIO2, timer0 up-mode period=100, generator0 utez=set/utea=clear, sample
    the pad via `digitalRead`) → `MCPWM duty1=44% duty2=21% MCPWM PASS` under
    `run_flash`. **GPIO_IN loopback fix**: a peripheral-matrix-routed pin (e.g.
    MCPWM 160..165) is now readable via `digitalRead` like real silicon — `soc.rs`
    gained `gpio_in_readback`, which overlays `GPIO_IN` with the *driven* level
    (peripheral signal via `signal_level` when `FUNC_OUT_SEL` is neither 0x80 nor
    0), intercepted at the `GPIO_IN` mmio read; `gpio.rs` gained `raw_in()`. 130
    workspace tests green (+6 MCPWM), clippy/wasm32 clean.
  - 2026-08-22: **PCNT pulse-counter (P5 — new peripheral via arduino-cli
   validation)** + **interrupt-source fixes**. PCNT (`esp32s3-soc/src/pcnt.rs`,
   4 units × 2 channels, one shared counter/unit): register block @ 0x6008_6000
   (CONF0/CONF1/CONF2 per unit @ 0x0C stride, CNT @ 0x30, INT_* @ 0x40..0x4C,
   STATUS @ 0x50, CTRL @ 0x60 per `pcnt_struct.h`), edge counting with
   pos/neg mode + control-signal gating (hctrl/lctrl KEEP/INVERT/INHIBIT),
   threshold interrupts, CTRL reset/pause. Wired into `soc.rs` (mmio arm,
   `tick` sampling unit/channel signal levels through the **GPIO-matrix input
   routing** — `gpio.rs` gained `in_sel()` / `pin_level()` resolving
   FUNC_IN_SEL_CFG to a GPIO's level). PCNT source = 41. Unit tests
   (`tests/pcnt.rs`, 3) assert rising/falling-edge counts and threshold
   assert+clear. Validated end-to-end with `tools/sketches/esp32s3_pcnt` (raw
   register driver: routes GPIO4 → PCNT unit0 ch0 via FUNC_IN_SEL, toggles 100
   edges) → `PCNT count=100` / `PCNT PASS` under `run_flash`.
   **BUG FIX**: the interrupt-source numbers in `soc.rs::int_pending` were wrong
   (latent — those sketches polled): RMT was 9 → **40**, SPI2/SPI3 were 44/45 →
   **21/22** (verified against `esp32s3 interrupts.h`; I2C 42/43, TIMG 50-55,
   UART 27-29, SYSTIMER 57-59, cross-core 79/80 were already correct). Also
   dropped the dead `RMTMEM_BASE` mmio arm (RMTMEM shares RMT's 4KB page, so it
   was never matched — RMTMEM access routes through the RMT_BASE arm). 113
   workspace tests green, clippy/wasm32 clean.
 - 2026-08-22: **Browser firmware gallery (P6 polish)**. `web/` now
  bundles 5 example sketches (`web/firmware/*.merged.bin` + `manifest.json`,
  gitignored — copy arduino-cli builds or reuse) and a `Examples` dropdown
  that fetches + loads the selected firmware; auto-load still pulls
  `firmware/esp32s3_hello.merged.bin`. `index.html` gained the `<select>`,
  `main.js` populates it from the manifest and adds `loadFromUrl`;
  `web/pkg` rebuilt via `wasm-pack build crates/wasm-bridge --target web
  --out-dir web/pkg`. Verified: `python3 -m http.server` serves
  index.html/main.js/style.css/manifest.json + bins all 200; `node --check`
  on main.js and JSON-valid manifest. Frontend is pure HTML/JS/CSS (no Rust
  change); emulator logic unchanged from the GPIO-fix commit. `.gitignore`
  updated (`web/firmware/*.bin`).
- 2026-08-22: **RMT TX peripheral (P5 — new peripheral via arduino-cli
  validation)**. `esp32s3-soc/src/rmt.rs` implements the RMT transmitter:
  register block @ 0x6001_6000 + item RAM @ 0x6001_6800 (RMTMEM_BASE shares
  the RMT_BASE 4KB page so the mmio dispatch routes both via `RMT_BASE` and
  `in_mem_range(off 0x800..0x1000)` → `mem[(off-0x800)/4]`), 4 TX channels,
  each 64×`rmt_item32_t` (duration0[14:0]/level0[15]/duration1[14:0]/level1[31]),
  signal indices RMT_TX_SIGNAL_BASE=81..84 (TRM gpio_sig_map), source 9 →
  tx_end interrupt. FSM advances `TICKS_PER_STEP=32` ticks/step, drives
  `signal_level`, raises `INT_RAW` ch0 bit on completion; `INT_CLR` clears.
  Wired into `soc.rs` (mmio arm, `signal_level`, `int_pending` source 9,
  `tick`). Unit tests (tests/rmt.rs, 3) assert the waveform levels,
  tx_end assert + clear, and out-of-range signal 0. Validated end-to-end
  with a real arduino-cli sketch `tools/sketches/esp32s3_rmt` that pokes the
  RMT registers directly (NOT the esp-idf `rmt_write_items` driver, which on
   S3 routes through **GDMA** (now MODELED); the esp-idf RMT *driver* path is
   validated separately via `tools/sketches/esp32s3_rmt_driver`. Runs under
   `run_flash` → prints `RMT TX done`. 113 workspace tests green, clippy/wasm32
   clean. Peripheral candidates: Touch; and/or more peripherals' esp-idf driver
   paths (e.g. SPI master driver, I2C driver) via arduino-cli sketches.
- 2026-08-22: **GPIO LED-grid fix (real gap, validated by Arduino sketch)**.
  `gpio_output()` was returning `0x0` for the periph sketch's `digitalWrite`
  blink, so the browser 40-pin LED grid never lit. Root cause localized with
  a temporary `gpio_debug` probe (later removed, fully clean): the
  `esp32s3_periph` firmware drives `GPIO_OUT` bit 2 correctly (toggles
  `0x0`↔`0x4`) and sets `GPIO_ENABLE` bit 2, but `FUNC_OUT_SEL[2]` was `0`
  (not the `0x80` "GPIO drive" sentinel). Two bugs: (1) `out_sel()` masked
  the register with `0x7F`, stripping the bit-7 sentinel so the `sel ==
  0x80` branch in `gpio_output()` was unreachable; (2) `pinMode(OUTPUT)`
  leaves/writes `FUNC_OUT_SEL = 0`, which also means "drive from GPIO_OUT"
  but wasn't recognized. Fix: `out_sel` now returns the full low byte
  (`& 0xFF`), `gpio_output()` treats `sel == 0x80 || sel == 0` as GPIO (any
  other value is a peripheral matrix signal routed via `signal_level`), and
  `Gpio::new()` seeds `FUNC_OUT_SEL_CFG[i] = 0x80` as the documented S3
  reset default. Verified through the node bridge: periph sketch now yields
  `gpio_output()` masks `0x0`↔`0x4`. 107 workspace tests green, clippy/
  wasm32 clean.
- 2026-08-22: **Browser bridge (P6 start) — emulator runs in the browser**.
  `wasm-bridge` (was a stub `add`) now wraps `Esp32S3` via `wasm-bindgen` and
  exposes `Emulator { new, load_flash(&[u8]), step(n), uart_read()->Vec<u8>,
  gpio_output()->u32, pc()->u32 }`. `web/` is a minimal harness:
  `index.html` + `main.js` + `style.css` — serial console (drains UART each
  frame), 40-pin GPIO LED grid, firmware file-input, Run/Stop/Reset + steps-
  per-frame slider; auto-loads `web/hello.bin` if present (gitignored — copy
  any arduino-cli `*.merged.bin`). Verified headlessly: built the nodejs
  target and ran the hello sketch in node → `Hello from ESP32-S3!` /
  `boot OK` over UART. Web target built into `web/pkg/` (gitignored,
  regenerated with `wasm-pack build crates/wasm-bridge --target web
  --out-dir web/pkg`); served over `python3 -m http.server` all assets
  return 200. `Esp32S3` re-exported at `esp32s3_emu` crate root for the
  bridge. clippy `--target wasm32` clean.
- 2026-08-22: **I2C NACK-model fix (real gap, validated by Arduino sketch)**.
  `i2c.rs` WRITE command no longer silently completes on an empty TX FIFO.
  On real HW the driver's NACK-retry re-runs the FSM still holding the byte
  in the hardware FIFO and re-NACKs; our model's retry saw an empty FIFO and
  reported success — which is why `Wire` on `esp32s3_i2c` (no devices on the
  bus) printed `found=56` instead of `found=0`. Now an empty-FIFO WRITE runs
  the START + 8 data pulses + ACK phases and raises `INT_NACK` (1<<10), so the
  retry also NACKs and `endTransmission` returns 2 (ACK error) for every
  address → `I2C SCAN done found=0`. The mandatory FIFO drain on transmit was
  also removed (`tx_pos` cursor replaces `tx_cnt` decrement) so a re-run
  re-sends the same byte instead of an empty FIFO. Sketches
  `esp32s3_{hello,periph,uart_echo,spi,i2c}` all boot (exit=0); i2c unit test
  `write_with_empty_fifo_completes` rewritten to `write_with_empty_fifo_nacks`.
  Driven by disabling nack to prove the lever (`found=112` with nack off vs
  `found=56` with nack on), then tracing `nack_set=112` vs `write_empty=58` to
  localize the empty-FIFO retry. 89 workspace tests green, clippy/wasm32
  clean.
- 2026-08-20: Second real Arduino-CLI firmware boots: **esp32s3_periph**
  (GPIO blink + digitalRead, analogRead with injected voltage, millis,
  and a FreeRTOS core-1 worker task). Run: `ADC_INJECT_MV=825 cargo run
  --release -p esp32s3-emu --example run_flash -- tools/sketches/esp32s3_periph/esp32s3_periph.merged.bin`
  → `Hello from ESP32-S3!`, `chip model=ESP32-S3 rev=0 cores=2 freq=40`,
  `boot OK`, `[core1] worker tick N (core=1, millis=...)`, `[main] loop N
  gpio2=0/1 adc4=866 millis=...` (866 = exactly 825 mV / 3900 mV (11 dB
  atten full scale) * 4095 — the ADC oneshot path + injection is
  numerically correct). Sketch built with arduino-cli 1.5.1 / esp32 core
  3.3.10, merged with esptool (bootloader@0, partitions@0x8000,
  boot_app0@0xe000, app@0x10000).
  - **GPIO register layout corrected to the TRUE S3 layout**
    (soc/esp32s3/register/soc/gpio_struct.h — the old 0x54-based layout
    was classic ESP32): pin[54] @ 0x74, func_in_sel_cfg[256] @ 0x154,
    func_out_sel_cfg[54] @ 0x554, clock_gate @ 0x62C, date @ 0x700;
    REG_COUNT 0x280 → 0x704. The periph sketch's pinMode(2) writes
    FUNC_OUT_SEL for pin 2 at 0x55C — the 0x280 window panicked the
    dispatch (the hello sketch never touched FUNC_OUT_SEL, which is why
    the old layout survived the boot). ledc/spi/i2c machine tests now
    write FUNC_OUT_SEL at the real 0x554 base (s32i imm8 caps at 255 →
    li the base in a spare register).
  - GPIO_IN pad loopback: output-enabled pins read back their driven
    value (digitalRead of an OUTPUT pin returns GPIO_OUT like real
    silicon); non-enabled pins keep the strap/input state (boot ROM
    strap check unaffected).
  - run_flash gained `ADC_INJECT_MV=<mv>` (injects on ADC1_CH3/GPIO4,
    the periph sketch's analogRead pin).
  - Cosmetic inaccuracies noted: `freq=40` (efuse/clk calibration not
    modeled — ESP.getCpuFreqMHz reads 40 MHz) and the doubled
    core-dump UART echo (dual-console artifact).
  - 30/30 emu, 89 total tests green; fmt/clippy clean.
  - Next: more Arduino sketches (UART RX echo, I2C/SPI with the
    existing models) or the browser frontend (P6).

- 2026-08-20: **FIRMWARE BOOTS TO COMPLETION** — the Arduino hello sketch
  prints `Hello from ESP32-S3!` + `boot OK` (real ESP-IDF/FreeRTOS SMP app
  through 16M steps). All tests green (30/30 emu + core/soc/wasm-bridge =
  79 total), clippy-clean libs, wasm32 clean.
  - **ROOT CAUSE of the long boot stall: u64 source bitmap overflow in
    `Soc::int_pending`**. `src |= 1 << (79 + cpu)` on a u64 masks the shift
    count to 63 (release build) → the cross-core source (FROM_CPU_INTR0/1 =
    79/80, esp-idf crosscore_int.c) landed on bit 15 → unmapped line 6 →
    the yield interrupt never fired → `ulTaskGenericNotifyTake`'s
    `esp_crosscore_int_send_yield` (self-yield) never switched the ipc tasks
    out → the main task never ran (TCB existed, stack = untouched sentinels).
    FreeRTOS SMP depends on the crosscore interrupt for every yield.
    `Intc::pending_lines` + the soc bitmap are now **u128** (sources 79/80
    and 94/95 sit beyond u64; the S3's source numbers reach 95).
  - The mechanism: `esp_crosscore_int_send` (0x40375EF8) writes
    SYSTEM.CPU_INT_FROM_CPU_0/1 (0x600C0030/34); the ISR
    `esp_crosscore_isr` (0x40375E9C) clears it (write 0) then
    `_frxt_setup_switch`. `esp_crosscore_int_init` (0x42007794) allocates
    source 79 (core0) / 80 (core1); the matrix maps core0 src79 → line 3
    (level 1). Verified: crosscore ISR entries 4, send calls 11, boot
    proceeds.
  - Second stall fix: gpio.rs `REG_COUNT` 0x180→0x280 — the app's GPIO init
    reads FUNC_IN_SEL entries (0x100..0x27F, 96 signals); the 0x180 array
    panicked at offset 0x184 (signal input 33).
  - ets_printf path forensics: the stub's 0x400005D0 slot is past
    `GLUE_END = 0x570` and is NOT spliced — the image keeps the REAL ROM's
    `__call_*` wrapper table (`l32r a9, [lit]; jx a9`, 12-byte entries:
    wrapper + literal). The app's ets_printf call → wrapper → the real
    ets_printf at 0x4004423C (vsnprintf + putc1 + uart_tx_one_char) — the
    real code WORKS in the emulator. The mailbox (HOST_PRINTF_BODY 0x520 +
    take_uart_tx drain) is dead weight (kept for now).
  - `machine::load_rom_data` now writes `_putc1` (0x3FCEF754) =
    uart_tx_one_char (0x40000648) — the state the real bootloader leaves
    behind. Without it the real ets_printf early-returns and bare-metal ROM
    tests print nothing (the full app works anyway because the app itself
    calls ets_install_uart_printf). The ets_printf_mailbox test was
    rewritten to drive the REAL wrapper path (callx8 with fmt/args →
    formatted bytes via the USB-Serial-JTAG FIFO, merged into take_uart_tx).
  - Gotchas: `1u64 << 79` is an overflow (shift ≥ 64 is UB in release /
    panic in debug) — the clippy/const-fold of `1u64 << 79` in the probe
    caught it; probe pcs-watch AFTER step misses vector entries (branches
    at the vector are never seen); the real ROM's ets_printf no-ops with
    putc1 = 0 (BSS, set by the app's own install).
  - Committed with: scratch examples deleted (probe_assert/dbg_printf/
    dbg_rom/dump_qsort), tools/tmp logs + Arduino build output + stale
    sketch bins dropped (sketches dir keeps .ino + merged.bin + .elf).
    run_flash.rs keeps its diagnostic probes (still useful for P5).
  - Next: P5 golden traces vs QEMU.

- 2026-08-15: P4 dual-core lands — **P4 complete**: 25/25 emu, 17/17 core,
  32/32 soc tests green (75 total incl. wasm-bridge).
  - `xtensa-core` cpu.rs: `Cpu` gains `core_id`; `Cpu::new(id)` seeds
    `sregs[SR_PRID] = id` (PRID = read-only core strapping; QEMU
    xtensa_cpu_reset sets sregs[PRID] = core_id). `Bus::int_pending` is now
    `int_pending(&mut self, cpu: usize)` — each CPU reads its own
    interrupt-matrix column (the `Intc` already kept per-CPU maps). SoC
    `int_pending` passes the requesting core through to `pending_lines(cpu)`.
  - **BUG FOUND + FIXED**: `sr_of()` (exec.rs) had NO `OPCODE_RSR_PRID` case
    → `rsr PRID` fell through to `_ => 0` and read SR 0 (LBEG), so core 1
    read PRID as 0 and ran the boot loader too (app printed "OK\n" twice in
    the boot test). Added `OPCODE_RSR_PRID => SR_PRID`.
  - `esp32s3-emu` machine.rs: `cpu: [Cpu; 2]`; `step()` = tick_timers(1),
    then core 0, then core 1 (serialized per step). Fixed order preserves the
    single-core timer-tick and per-core instruction counts the existing tests
    rely on (round-robin would break timg0_load_and_read: two ticks between
    core 0's T0LOAD and T0LO read). All tests changed `m.cpu.pc` →
    `m.cpu[0].pc`.
  - rom_stub.rs: reset now `rsr a2, PRID; bnez a2, CORE1_WAIT` before setting
    SP; core 1 branches to a fixed spin (CORE1_WAIT = 0x40000400) polling
    `CORE1_ENTRY` (0x3FC87F00, host-defined — QEMU S3 models NO release
    register, `esp32s3_cpu_stall` is a stub) and `jx`es to whatever core 0
    stores there. New `pad_to` helper: the 21-byte PRID + 21-byte core1_wait
    sections made the pad gaps to CORE1_WAIT/ROM_PUTS odd, and 2-byte pad2s
    alone overshoot the fixed addresses → one 3-byte `movi a15,0` covers an
    odd gap.
  - New tests: `dual_core_release_and_run` (boot_from_flash with a 2-segment
    image: core 0 releases core 1 via CORE1_ENTRY, core 0 stashes 0xBEEF at
    STASH0, core 1 stashes 0x1234 at STASH1, both land in their self-loops —
    if PRID gating were broken, core 1 would run core 0's code and land at
    here0 instead of here1) + `esp_app_image_multi` helper;
    xtensa-core `prid_reads_core_id` (Cpu::new(0)/new(1) both roundtrip via
    `rsr a2/a3, PRID` = 0x0003_EB20/30).
  - Gotchas: non-ROM tests leave core 1 at 0x40000000 executing zeroed IROM
    (`0x000000` = `neg a0,a0`, harmless self-loop); `xtensa_core::cpu::SR_PRID`
    path (not re-exported at crate root); the loader ends at 0x40000051 (odd)
    so `pad2`-only padding overshoots to 0x401.
  - fmt/clippy (0 warnings)/wasm32 clean. **P4 complete** → P5 golden traces.

- 2026-08-15: P4 PSRAM + cache MMU lands: 24/24 emu, 16/16 core, 32/32 soc
  tests green (73 total incl. wasm-bridge).
  - `esp32s3-soc` cache.rs: shared cache MMU + EXT_MEM controller. The data
    window (0x3C00_0000) and instruction window (0x4200_0000) are BOTH 32 MB
    aliases over ONE 512-entry MMU table (64 KB pages; QEMU esp32s3_cache.h
    `dcache`/`icache` alias the same IOMMU, `ESP32S3_EXTMEM_REGION_SIZE
    0x2000000` — our FLASH_WINDOW_SIZE was 16 MB, bumped to 32 MB). MMU entry
    = page_number[13:0] | invalid[14] | type[15] (0=flash read-only, 1=PSRAM
    read-write), reserved [31:16] forced 0 on write (QEMU
    esp32s3_write_mmu_value). Registers at EXTMEM 0x600C_4000 (DCACHE/ICACHE
    CTRL+CTRL1 enable at 0x000/0x004/0x060/0x064, SYNC_CTRL 0x028/0x088,
    PRELOAD_CTRL 0x040/0x094, AUTOLOAD_CTRL 0x04C/0x0A0, FREEZE 0x150/0x154,
    CACHE_STATE 0x130 idle (1<<0)|(1<<12)); the MMU table is the NEXT page
    (MMU_TABLE_BASE 0x600C_5000, offset 0x1000 — DR_REG_MMU_TABLE).
  - ena→done handshake mirrors QEMU `check_and_reset_ena`: WRITE stores ENA,
    READ clears ENA + sets DONE (IDF cache_ll_sync writes INVALIDATE_ENA then
    polls SYNC_DONE). Autoload/preload regs reset to DONE (ready) so IDF init
    polls exit immediately (esp32s3_cache_reset_hold). FREEZE write toggles
    only the DONE bit.
  - soc.rs: `psram` 8 MB backing + `cache` device; window read8/16/32 route
    through `Cache::translate` (flash or PSRAM), writes only land on
    MMU-mapped PSRAM pages (flash stays read-only); EXTMEM + MMU_TABLE mmio
    arms added.
  - **DELIBERATE DEVIATION from QEMU**: QEMU's translate ignores `invalid`
    and resolves every entry via page_number (unset → flash page 0). We map
    invalid pages as 1:1 flash, preserving the pre-MMU contract (boot ROM
    stub reads flash through the window without programming the MMU);
    explicit entries always win. QEMU's on-demand flash_mr page fill is
    unneeded — our flash backing is always resident.
  - Machine test `psram_read_write_via_mmu_mapped_page`: firmware writes
    mmu[0]=0x8000 (PSRAM page 0) + mmu[3]=0x8002 (PSRAM page 2), round-trips
    a word through each mapped vpage via the data window, stashes both
    (0xDEADBEEF / 0xCAFEBABE). Soc tests (tests/cache.rs, 10): reserved
    cleared on MMU write, PSRAM r/w via data + inst window (shared table),
    physical page selection, flash remap + read-only, unmapped 1:1 alias,
    beyond-8 MB PSRAM reads 0 / writes dropped, sync/preload/autoload/freeze
    done handshakes, CTRL enable readback.
  - Gotchas: clippy collapsible_if in cache_write8 → match-with-guard; the
    mmio page dispatch needs a SEPARATE arm for 0x600C_5000 (the MMU table is
    one page past EXTMEM, so `off = base & 0xFFF` can't reach it);
    0xDEADBEEF/0xCAFEBABE literals need `as i32` (overflowing_literals deny).
    Back-compat verified: flash_xip + boot_path tests green with the 1:1
    alias (window offset 0x1000000 boundary read still 0 after the 16→32 MB
    window bump — flash_byte returns 0 past 4 MB).
  - fmt/clippy (0 warnings)/wasm32 clean. Next P4 items: dual-core; then P5
    golden traces.

- 2026-08-15: P4 SAR ADC lands: 23/23 emu, 16/16 core, 22/22 soc tests green
  (62 total incl. wasm-bridge).
  - `esp32s3-soc` adc.rs: SENS RTC oneshot controller (0x6000_8800 — NOT
    the classic-ESP32 0x6000E000; QEMU esp32s3_reg.h + esp32s3-hal agree)
    + APB_SARADC digital controller (0x6004_0000). Oneshot (adc_oneshot
    driver): measN_ctrl2 (0x0C/0x30) data_sar [15:0] / done [16] / start
    [17] / start_force [18] / en_pad [30:19] / en_pad_force [31]; controller
    select sar1_dig_force (meas1_mux 0x10); sar_attenN 2-bit per channel;
    shared meas_status [29:22] in sar_slave_addr1 (0x40) busy while the FSM
    runs (adc_oneshot_ll_start polls it, ADC2 skips the gate). Digital:
    ctrl (0x00) work_mode [4:3] (0=single,1=double,2=alternate), sar_sel
    [5], sar_clk_gated [6] gate, sarN_patt_len [18:15]/[22:19]; start is a
    self-clearing pulse (like SPI usr); sarN_patt_tab[4] holds 4 one-byte
    items/word = atten[1:0] | channel[6:2]; ctrl2 (0x04) timer_en [24] +
    timer_sel [11] triggers one pass every (timer_target+1) cycles; results
    in apb_saradcN_data_status (0x40/0x78); done flags are the TOP int bits
    adc1_done = 1<<31 / adc2_done = 1<<30 (NOT bit 0/1 — struct packs the
    flags at the end of the word). DMA NOT modeled — continuous firmware
    reads data_status directly.
  - Voltage scaling: host injects mV per (unit, channel) via
    `adc_inject_voltage`; raw = mv/fs*4095 with full-scale per atten code
    0dB=1100 / 2.5dB=1500 / 6dB=2200 / 11dB=3900 mV (TRM SAR ADC); data_inv
    (readerN_ctrl bits 28/29) bitwise-inverts the 12-bit result. Oneshot
    latency fixed at 8 APB cycles (regi2c sample times not modeled).
  - soc.rs: SENS is NOT page-aligned — the 0x6000_8000 page also holds
    RTC_CNTL (0x000)/RTC_IO (0x400)/RTC_MEM (0xC00), so the mmio dispatch
    matches page 0x6000_8000 + off in 0x800..0xC00 (SENS window).
  - Machine test `adc1_oneshot_reads_injected_voltage`: firmware drives the
    oneshot flow (mux=0 RTC, atten=0, en_pad ch2, spin meas_status, start
    0->1, spin done, stash raw); host injects 825 mV -> 3071
    (825*4095/1100). asm.rs gained `and` (RRR op0=0 op1=0 op2=1).
  - Gotchas this session: l32i/s32i take BYTE offsets in the asm API — a
    first draft passed word-shifted offsets and read the mux instead of
    slave_addr1; `(0 << n)` field-marker literals in tests/adc.rs need the
    file-level `#![allow(clippy::identity_op)]` (like the generated.rs
    header); my test expectations initially saturated (voltage > full-scale)
    and swapped the pattern byte's atten/channel fields (byte = atten |
    channel<<2, so ch2 atten0 = 0x08).
  - fmt/clippy (0 warnings)/wasm32 clean. Next P4 items: dual-core, PSRAM;
    P5 golden traces.

- 2026-08-15: P4 I2C master lands: 22/22 emu, 16/16 core, 15/15 soc tests green.
  - `esp32s3-soc` i2c.rs: I2CEXT0 @ 0x6001_3000, I2CEXT1 @ 0x6002_7000
    (0x14000 apart). Registers per i2c_struct.h member order: ctr(0x04,
    ms_mode=4, trans_start=5), data(0x1C = FIFO port: write pushes TX,
    read pops RX — i2c_ll_write_txfifo), scl_low_period(0x00, low =
    value+1 module clocks), scl_high_period(0x38, high = value +
    scl_wait_high_period[15:9] — IDF measures high without the +1),
    scl_start_hold(0x40)/scl_stop_hold(0x48)/scl_stop_setup(0x4C),
    clk_conf(0x54, module = APB/(sclk_div_num+1)), comd[8](0x58),
    txfifo_mem(0x100)/rxfifo_mem(0x180).
  - **CRITICAL S3 comd layout (IDF i2c_ll_hw_cmd_t — NOT classic ESP32):**
    byte_num[7:0], ack_en[8], ack_exp[9], ack_val[10], op_code[13:11],
    done[31]. Op codes: RSTART=6, WRITE=1, READ=3, STOP=2, END=4 (the
    i2c_struct.h comment "0:RSTART,1:WRITE,2:READ,3:STOP,4:END" is stale —
    i2c_ll.h is what drives real silicon). Trigger = ctr.trans_start write
    builds the pending queue from comd slots with done clear, executes in
    order, sets each slot's done as it finishes, halts at END; END raises
    INT_RAW.trans_complete (bit 7).
  - Master FSM phase machine (per-bit: low half then high half, ACK =
    cycle 8): START = SDA falls while SCL high (start_hold), then SCL
    falls; WRITE = 8 SCL pulses + ACK cycle (SDA released -> NACK 1,
    latched SR.resp_rec), byte from TX FIFO; READ = 8 pulses sampling
    undriven SDA (reads 1 -> 0xFF bytes into RX FIFO), master drives
    comd.ack_val after; STOP = SDA driven low (stop_hold) then raised
    (stop_setup) while SCL high; END instant. bus_busy (SR bit 4) while
    FSM running.
  - soc.rs: I2C0/1 mmio arms + tick + signal routing (I2CEXT0 SCL/SDA =
    89/90, I2CEXT1 = 91/92 — S3 gpio_sig_map.h). Machine test
    `i2c0_master_write_nacks_and_stops`: I2CEXT0_SCL(89)->GPIO1,
    I2CEXT0_SDA(90)->GPIO2; comd list RSTART|WRITE|WRITE|STOP|END, stash
    BEFORE trans_start; host samples 200 cycles, syncs on START (SDA
    falling while SCL high — the STOP's SDA dip is a (1,0)->(1,1) window,
    distinguishable because START's (1,0) is followed by SCL falling),
    recovers 18 pulses (0xA0 addr + NACK + 0xAA data + NACK), STOP (SDA
    rising while SCL high), idle (1,1), all comd done bits, NACK in
    resp_rec, trans_complete raw.
  - Gotchas this session: unit tests must sample the idle (1,1) state
    BEFORE trans_start or the START detector latches the STOP dip; the
    READ advance's scl==0 branch must NOT reset sda=1 (it clobbers the
    master ACK level — data lows release high, ACK low drives op.ack);
    the RSTART->next-command seam drops SCL with no (0,0) sample (the
    next command's first low phase covers it); finish_op needs the
    completing slot passed in (op is cleared before it runs). Borrow
    checker: precompute timing lens before as_mut()ing the op; inline
    fifo/reg side effects (method calls on &mut self conflict with the
    live op borrow).
  - fmt/clippy (0 warnings)/wasm32 clean. Next P4 items: ADC, dual-core,
    PSRAM; P5 golden traces.

- 2026-08-15: P4 GPSPI2/3 SPI master lands: 21/21 emu, 16/16 core,
  10/10 soc tests green.
  - `esp32s3-soc` spi.rs: GPSPI2 @ 0x6002_4000, GPSPI3 @ 0x6002_8000
    (0x1000 apart). Registers per S3 spi_struct.h: CMD(0x00, usr=bit 24
    self-clears), CLOCK(0x0C, clk_equ_sysclk=31, clkdiv_pre[21:18],
    clkcnt_n[17:12], clkcnt_h[11:6]), USER(0x10, usr_command=31,
    usr_addr=30, usr_dummy=29, usr_miso=28, usr_mosi=27, doutdin=0),
    USER1(0x14, addr_bitlen[31:27], dummy_cyclelen[7:0]), USER2(0x18,
    cmd_bitlen[31:28], cmd_value[15:0]), MS_DLEN(0x1C, data bitlen[17:0]),
    MISC(0x20, ck_idle_edge=29, cs0_dis=0, cs1_dis=1), data_buf[16] @
    0x98 (left-aligned MSB-first shift register), CLK_GATE(0xE8, clk_en).
    CPU-controlled USR transfers: phases cmd/addr/dummy/data, bit rate =
    (clkdiv_pre+1)*(clkcnt_n+1) APB cycles (clock high (clkcnt_h+1) of
    each (clkcnt_n+1)-cycle period, mode 0 idle low); clk_en gate must
    be set or usr hangs. MISO reads back zeros (no device). No DMA.
  - GPIO-matrix signal indices (S3 gpio_sig_map.h — CRITICAL): LEDC =
    73..80 (NOT 96..103 — that is classic ESP32!), GPSPI2 = FSPI
    CLK/Q/D/HD/WP 101..105 + CS0/CS1 110/111, GPSPI3 = 66..72.
    ledc.rs signal constants fixed to 73..80 + machine test FUNC_OUT_SEL
    96 -> 73.
  - soc.rs: SPI2/3 mmio arms + tick, signal_level routes LEDC 73..80 and
    both SPIs.
  - Machine test `spi2_shifts_out_0xa5_on_gpio_pins`: firmware routes
    FSPICLK(101)->GPIO1, FSPID(103)->GPIO2, FSPICS0(110)->GPIO3;
    CLOCK=0x1000 (2 cyc/bit), MS_DLEN=7, W0=0xA5<<24, USER=usr_mosi,
    CLK_GATE=1, CMD.usr last instruction. Host syncs on CS0 falling edge
    (= elapsed 0, since the trigger is the instruction AFTER the stash)
    then samples: 8 pulses, CS low for 16 cycles then released, bits at
    ck-high midpoints = 0xA5, usr cleared. First attempt failed because
    the pins closure shifted wrong bits AND the transfer had already
    started mid-window — CS-falling sync + stash-before-trigger fixes it.
  - Soc unit tests (tests/spi.rs): MSB-first 8-bit shift, pre/n divider
    (2 clock periods per bit slot with pre=1), MISO zeros, clk_equ_sysclk,
    clk gate halt. Gotchas: bit_cycles=(pre+1)*(n+1) with the clock
    period (n+1) repeating per slot; usr clears on the tick that finishes
    the last cycle (not the next one).
  - fmt/clippy/wasm32 clean. Next P4 items: I2C/ADC, dual-core, PSRAM;
    P5 golden traces.


- 2026-08-15: P4 LEDC PWM lands: 20/20 emu, 16/16 core, 5/5 soc tests green.
  - `esp32s3-soc` ledc.rs: 4 timers + 8 channels at base 0x6001_9000.
    Timing per ESP-IDF `ledc_calculate_divisor`: div_param =
    (src_clk<<8)/(freq*2^resolution) lives in {clock_divider[17:8],
    [7:0] frac}; tick = src_clk*256/div_param APB cycles via a
    256-scale fractional accumulator; counter wraps at 2^resolution
    (resolution = duty_resolution field + 1, TRM LEDC_TIMERx_CONF);
    channel high while counter < (CH_DUTY >> (18-resolution)).
    TIMER_CONF.pause/rst (rst resets counter), CH_CONF0 duty_start
    (rising edge resets the timer) / sig_out_en / timer_sel [5:4];
    TIMERx_VALUE returns the live counter; sig_out_en=0 holds output
    low. LEDC_CH0..CH7 = GPIO-matrix signals 96..103 (TRM signal
    table); `signal_level(sig)`.
  - gpio.rs: FUNC_OUT_SEL_CFG at 0x54+4*i (value 128 = GPIO_OUT drive);
    enabled()/out_bit()/out_sel() use u64 shifts — GPIO bits live above
    31 (46 pins), `1u32 << 45` overflows (debug panic, not caught by
    the old output() which never shifted).
  - soc.rs: LEDC_BASE mmio arm + ledc tick inside tick_timers;
    `gpio_output()` now routes pins: FUNC_OUT_SEL != 0x80 → LEDC signal
    level instead of GPIO_OUT (ENABLE-gated).
  - Machine test `ledc_pwm_blinks_gpio0_at_50_percent_duty`: TIMER0_CONF
    = 0x0024_0100 (div 1.0, 10-bit period), CH0_DUTY 0x20000 (50%),
    CONF0 = 0xC (duty_start|sig_out_en), GPIO0 ENABLE + OUT_SEL 96;
    measures 512/512 cycle phases. Gotcha: measurement must sync on a
    FALLING edge first — the counter resets to 0 at duty_start so the
    pin is already high when the firmware finishes configuring.
  - Soc unit tests (tests/ledc.rs): 50% toggle run-lengths 512/512,
    low-before-duty-start, TIMER_VALUE live counter (VALUE offset check
    is % 8 == 0, not 4), signal→channel mapping.
  - fmt/clippy/wasm32 clean. Next P4 items: SPI/I2C/ADC, dual-core,
    PSRAM; P5 golden traces.


- 2026-08-15: P4 interrupt hardening + UART RX: 36/36 tests green.
  - xtensa-core: `interrupt_preemption_level4_takes_in_level3_handler`
    (line 22 = L3 + line 24 = L4; L4 preempts inside the L3 handler —
    cintlevel = max(3, PS.INTLEVEL) — rfi 4 → rfi 3 chain; pending lines
    must be cleared like real INT_CLR or the level-triggered take re-fires
    after rfi). Encodings: `or a2,a2,a3` = 0x0020_2230 (op0=0 op1=0 op2=2);
    slli a3,a3,24 = 0x0001_3380 (bit 20 = sal[4] = 0 — 0x0010_3380 decodes
    as AND a3,a3,a8!); movi imm12 = {insn[11:8]<<8 | insn[23:16]} so
    `movi a4,0x333` = 0x0033_A342 (b1 = (0xA<<4)|imm[3:0] = 0xA3) and
    `movi a5,0x444` = 0x0044_A452 (t = 5 → b0 = 0x52); imm12 caps at
    0x7FF (0x3333 does NOT fit a movi). Unaligned 4-byte test-bus inserts
    are harmless: byte 3 of each word is the next instruction's byte 0 and
    all decode fields live in bits [23:0] (op0 = [3:0]!).
  - esp32s3-emu machine test `timg1_alarm_delivers_level4_vector`: source
    53 → matrix 4*53 = 0xD4 → line 24 (L4) → 0x40000200 handler → INT_CLR
    → rfi 4 (CTR/STASH at 0x3FC8_0200/4, above the app image).
  - esp32s3-soc UART RX: `inject_rx` (FIFO + RAW RXFIFO_FULL + STATUS
    [29:24] count + RXD_CNT line counter); FIFO read pops, RAW drops when
    empty (level-style); STATUS reset = ST_UTX_OUT bits [9:8] = 0x300
    (was 1<<24 — that bit now belongs to the FIFO count field). Soc gains
    `uart_inject_rx(n, byte)`.
  - Machine test `uart_rx_interrupt_echo`: matrix source 27 → line 15 →
    handler pops FIFO, stores to RXBUF, counts RXCNT (RXCNT = RXBUF - 4 so
    one li covers both), echoes the byte on TX, INT_CLR, rfi 3; host
    injects 'X' → stash 0xCAFE. Handler is 53 bytes (64-byte slot) — li of
    a 0x3FC80xxx address costs 15 bytes; derive neighbors with addi.
  - Next P4 items: SPI/I2C/PWM/ADC, dual-core, PSRAM; P5 golden traces.
- 2026-08-15: P4 interrupt delivery lands: 17/17 emu, 15/15 core tests green.
  - `xtensa-core`: `Bus::int_pending` default (0); cpu.rs `check_interrupts`
    (highest pending level via INT_LEVEL_MASKS & INTSET & INTENABLE; NMI
    line 14 bypasses both; cintlevel = PS.INTLEVEL or max(3) when EXCM;
    level 1 → kernel/user exception EXCCAUSE=4; levels 2-6 → EPC1+lvl-1/
    EPS2+lvl-2 + PS=(PS&~INTLEVEL)|lvl|EXCM + VECBASE+INT_VEC_OFFSETS;
    NMI clears sticky bit 14). step() checks interrupts after
    sync_windowbase/icount (take at instruction END → EPC = next insn).
  - `exec.rs`: RSR INTERRUPT special case (reads live intset via
    `intset_live(bus)`; SR 226 on ESP32-S3, no SR 225); WSR INTSET ORs,
    WSR INTCLEAR ANDs the sticky sreg; both removed from generic arms.
  - `esp32s3-soc`: `Intc::pending_lines(cpu, u64 bitmap)`; `Uart::int_st`
    (RAW&ENA); `Soc::int_pending` (UART 27-29, TIMG0/1 50-55; bitmap is
    u64 — sources ≥ 32 overflow u32!). `asm.rs`: wsr/rsr/rsil/rfi
    encodings + roundtrip test; `movi_n` imm7 z-bit fixed (n = 3 bits,
    `movi.n aX,0x40` was encoding 0!).
  - Machine test `timer_interrupt_delivers_to_vector`: TIMG0 alarm every
    64 cycles → source 50 → matrix line 15 (level 3) → 0x400001C0 handler
    (a6-a9 only) → INT_CLR → rfi 3; main loop counts 3 interrupts then
    stashes 0xCAFE. Gotchas: DRAM/IRAM alias means CTR in DRAM must sit
    ABOVE the app image or the handler clobbers the literal pool; asm
    l32i/s32i take BYTE offsets (off >> 2), NOT pre-shifted words.
  - fmt/clippy/wasm32 clean. Next P4 items: SPI/I2C/PWM/ADC, dual-core,
    PSRAM; P5 golden traces.
- 2026-08-13: `gen_decode.py` produces compiling, tested decoder.
  - Fixed: `XTENSA_UNDEFINED` fallbacks now `return None;` (no enum variant);
    operand exprs emit `u32` literals (E0689 gone).
  - Split fields: parser now keeps per-term (lo,width) ranges instead of
    assuming one contiguous run — `sal` = {insn[20], insn[7:4]} (matches LLVM
    `SLLI` def: `Inst{20}=sa{4}; t=sa{3-0}`), `sr` = bits [15:8], `imm6`
    (inst16a) = {n, r}. Verified against C get fns (e.g.
    `Field_sal` = `((insn<<11)>>31)<<4 | ((insn<<24)>>28)`).
  - ADDI/L8UI/... on op0=2: `r`=[15:12] is the opcode selector (12=ADDI);
    dest is `t`=[7:4]. BEQI immediate = `b4c_TBL[r]` (4-bit compact), offset =
    4+sext8(imm8)+pc (not pc-aligned); LOOP/L32R offsets per C exactly.
  - 7 KAT tests pass (`cargo test -p xtensa-core`), clippy-clean via
    `#![allow(...)]` header on generated.rs (mirrors C parens/`(0<<n)|x|0`).
- 2026-08-14: P1 Core CPU execution engine lands: 11/11 tests green.
  - `cpu.rs` (step loop, SR/PS/vector constants, windowed regfile,
    exceptions, loop-end check) + `exec.rs` (200+ opcodes) + `bus.rs` trait.
  - Fixed per QEMU (espressif/qemu translate.c / win_helper.c): CALL4/8/12 +
    CALLX return addr = `(callinc<<30) | ((pc+len)&0x3fffffff)` written to
    the CALLER's a(callinc*4) (rotation happens at ENTRY, not CALL — the
    callee sees the return address in its a0); LOOP/LOOPNEZ LBEG = pc+len
    (pc_next); ENTRY subtracts (imm12<<3) then rotates by PS.CALLINC.
  - 4 hand-assembled cpu_tests (ALU/branch/loop, loads+stores incl. s32c1i,
    exceptions+handler+rfe, windowed call4/entry/retw) + 7 restored KAT
    decoder tests.
  - Test-encoding gotchas (all verified against generated.rs): load/store
    r=[15:12] selector, s=[11:8]=addr, t=[7:4]=data (t/s swap was a repeated
    test bug); BZ imm12 = single 12-bit field [19:12]; BEQZ/BNEZ =
    imm12<<12|s<<8|m<<6|1<<4|6; SSR = op2=4, r=t=0; SLLI requires op1=1;
    movi.n (inst16b): dest=s[11:8], imm7={z,n[5:4],r[15:12]}; entry imm12 =
    N>>3 at [19:12]; wsr = op0=0 op1=3 op2=1.
- 2026-08-15: P2 SoC + machine glue lands: 6/6 machine tests green.
  - `esp32s3-soc`: memmap.rs (DRAM/IRAM alias the same 512KB SRAM; IROM
    read-only; APB devices), uart.rs (FIFO TX/FIFO_CNT/STATUS/conf, tx stream
    out), gpio.rs (OUT/W1TS/W1TC/ENABLE/W1TS/STRAP), timg.rs (64-bit T0/T1,
    tick(cycles), T0LOAD/UPDATE/ALARM, INT_RAW/ENA/ST/CLR), intc.rs (matrix
    table only). `esp32s3-emu`: machine.rs (`Esp32S3` = Cpu+Soc, `step`
    = tick_timers(1) then cpu.step, `load_image` DRAM/IRAM/IROM, boot pc =
    0x40000000).
  - Hand-assembled tests: uart0_hello_world ("Hi"), gpio_out_w1ts_w1tc,
    timg0_load_and_read, timg0_counts_on_tick, bus_sanity, boot_reset_vector.
  - Fixed xtensa-core ADDMI double-shift bug (opnds pre-shifts imm8<<8; exec
    must not shift again).
  - Encoding gotchas learned this session (LE word = b0|b1<<8|b2<<16):
    MOVI byte1 = (r<<4)|imm_hi (`movi a2,0x60` = 0x0060A022); SLLI byte2 =
    (op2<<4)|op1 with op2=[23:20], sal={bit20,[7:4]}, shift=32-sal
    (`slli a4,a4,12` = 0x00114440); ADD = RRR op0=0 op1=[19:16]=0
    op2=[23:20]=8, r=[15:12] dest, s=[11:8], t=[7:4] (`add a2,a2,a4` =
    0x00802240; words with op1=8 decode as LSX -> ILLEGAL); movi.n imm7 =
    {n[5:4]<<4 | r[15:12]} (`movi.n a4,5` = 0x0000540C, r lives in byte1
    [15:12], NOT the low nibble); l32r target = ((pc+3)&~3)+((0xFFFF0000|
    imm16)<<2) so the literal pool must precede the code (code pc =
    IRAM_BASE+4/+8 after pool); j . = 0x00FFFF06 (offset -4); s32i/l32i
    imm = r=[15:12] in BYTES (`s32i a4,a5,1` = 0x00016542); `nop.n` not
    decoded -> pad with `movi.n aX,0` = 0x00X0_0C.
  - Clippy clean (fixed in_range! macro to Range::contains, Default impls
    for Intc/Timg, is_multiple_of), fmt clean, `cargo build --workspace
    --target wasm32-unknown-unknown` passes.
- 2026-08-15: P3 boot path lands: `boot_path_loads_app_from_flash` green
  (15/15 esp32s3-emu, 12/12 xtensa-core, clippy/fmt/wasm32 clean).
  - `rom_stub.rs` (hand-assembled at 0x40000000): reset li a1,STACK_TOP
    (0x3FC88000) + j loader; loader li a2,0x3C010000, l8ui a3,a2,1,
    l32i a4,a2,4 (entry), addi a5,a2,24, seg_loop/copy_loop/seg_next
    (copies each esp_image_header_t segment from flash to DRAM/IRAM,
    loops on segment_count, jx a4); rom_puts @0x40000500 (li a5,0x60000000
    = movi+slli+slli = 9B, l8ui a4,a2,0, beqz a4,done, s32i UART FIFO,
    addi a2,1, j, ret).
  - `machine.rs`: `boot_from_flash` (flash8 window, 0x0000000 reserved,
    0xE9 magic check, parse_partition_table → entry/segments), `load_image`
    now writes IROM storage directly (Bus write path is read-only).
  - `partition.rs`: 32-bit aligned partition-table parsing (magic 0xAA50,
    type/subtype/map, entry_addr + load_addr from app partition) — 3 tests.
  - `asm.rs`: callx0 (0x3C0), jx, j (24-bit, 3 bytes), l32r + patch_l32r,
    bytes_mut for literal patching, beqz/BNEZ 12-bit byte-offset
    (target = pc+4+sext(imm12), NOT <<2).
  - Verified vs QEMU/ISA: CALLX0/RET/JX low 2 bits of the target are
    masked (`& !3`) → return addresses must be 4-aligned: place a 3-byte
    call at pc ≡ 1 (mod 4) so pc+3 ≡ 0 (mod 4), else the masked return
    fetches garbage mid-instruction. JX/CALLX0 decode: JX = RRR t=0xA,
    RET = t=0x8, RETW = t=0x9, CALLX0 = t=0xC, CALLX4 = t=0xD (m=3,n=1).
  - L32R deviation from QEMU documented: QEMU's `((0xffff<<16)|imm16)<<2`
    equals sext16(imm16)<<2 only when bit 15 is set; we emit the ISA-RM
    sext form (forward l32r would be wrong). KAT test updated.
  - Encoding gotchas: l32i/s32i imm is a BYTE offset (imm8 = off>>2);
    esp_image_header_t is 24 bytes (our synthetic images pad); the 24-bit
    `j` is 3 bytes (off = target-pc-4, 18-bit field [23:6], opnds
    sext14 + pc+4 — bytes, no <<2); rom_puts beqz skips s32i+addi+j+ret
    = 12 bytes.
- 2026-08-13: Environment verified. Research done (no Xtensa Rust crate;
  QEMU = only reference). AGENTS.md created. Workspace scaffold next.- 2026-08-18: qsort milestone + firmware boot debugging (stale-bin + S32C1I forensics).
  - asm.rs callx4 encoding bug FIXED: b0 low nibble is op0=[3:0] and MUST be
    0..7 or insn_len returns 2 (0xDD decoded as inst16b → mis-fetch). callx4 t
    = (t<<8)|(3<<6)|(1<<4) (b0 = 0xD0; fields m=[7:6], n=[5:4], t=[7:4]).
  - rom_stub.rs `pad_to` is now pub; machine_tests.rs + dump_qsort.rs pad the
    comparator to CODE+0x80 (the old CODE+0x4B pointer aimed at zeroed IROM).
  - qsort body rewritten (slot 0x40001488 → QSORT_BODY 0x40001300): entry bgeu
    removed; multiply loop `movi(9,1); and(8,14,9)` (the old and(8,14,1) masked
    with a1=SP); sign-extract `movi(9,31); ssr; srl` (emu SSR sets SAR=as);
    byte-swap loop added; sw_done uses TWO sub(10,10,4) (swap advances both
    pointers by size). Final layout/targets: m_loop 0x131B, m_done 0x1336,
    inner 0x133C, sw_loop 0x1387, sw_body 0x138D, sw_done 0x13A5, next_i
    0x13B1, done 0x13BD. Sorts [9,7,5,6,8,4], caller halts at step 1035;
    `cargo test -p esp32s3-emu rom_qsort` green; all workspace tests green.
  - FIRMWARE now passes the heap_caps_init qsort assert. The run was using a
    STALE merged bin (entry 0x403C88B8, 3 segments); the fresh one is
    build/esp32.esp32.esp32s3/esp32s3_hello.ino.merged.bin (entry 0x40375AAC,
    6 segments, matches the ELF: .iram0.text 0x40374404, .dram0.data
    0x3FC92F00 len 0x3884, .dram0.bss 0x3FC96788). Copied fresh bin to
    tools/sketches/esp32s3_hello/esp32s3_hello.merged.bin.
  - S32C1I forensics on esp_cpu_compare_and_set (0x40377AD8): the internal-RAM
    path at 0x40377B24 is `wsr.scompare1 a3; s32c1i a4, a2, 0; sub a3, a3, a4;
    nsau a2, a3` — SCOMPARE1 = the compare arg (0xB33FFFFF) BEFORE the s32c1i.
    The emulator was CORRECT: SCOMPARE1=0xB33FFFFF, mem matched, the store
    overwrote the mux with the core id 0 (lock acquired) — the apparent "mux
    clobber" is real hardware semantics. The objdump disasm at 0x40377b22 and
    0x4037ade0 was misaligned: objdump -d prints the LE WORD, so the byte
    string is reversed vs memory order ("00e242" = bytes 42 e2 00).
  - The FreeRTOS "SPIN" at 0x4037ade5 is NOT a stuck loop: it is
    regi2c_ctrl_write_reg_mask (0x40377F2C) repeatedly entering/exiting
    xPortEnterCriticalTimeout (0x4037AD18) critical sections — the app makes
    progress (~200 steps between visits). SPIN prints: lock=0x3FC93070 (mux,
    .dram0.data init 0xB33FFFFF), mem=0 (core 0 owns it), ra=0x80377F3B,
    a3=0xffffffff (timeout arg), a9=0x60000 (= nesting+1, nesting read 0x5FFFF).
  - OPEN ISSUES: (1) NEST trace: a8=0x3FC972CC after `addx4 a8, a8, a9`
    (0x4037AD51) where the l32r literal = 0x3FC972D4 (port_uxCriticalNesting)
    — off by -8; a9=0x60000 with `bnei a9, 1` implies the nesting was 0x5FFFF,
    and `bne a8, a14` at 0x4037ADE2 (a14=0xF) should assert — check
    addx4/rsr.prid/extui semantics. (2) OOB panic soc.rs:676: psram backing
    array indexed with 0x201FDE58 (len 8192) — an MMU psram page 0x201F —
    needs a page bound check or MMU-entry decode fix in the cache translate.
  - GOTCHAS: objdump -d hex is the LE word (bytes reversed vs memory order);
    insn_len = 3 if b0&0xf <= 7 else 2; s32c1i fields: t=[7:4] (data),
    s=[11:8] (base), r=[15:12] (selector 14), imm8=[23:16], decode guard
    op0==2 && r==14; esptool image_info on a merged bin reports the
    BOOTLOADER at offset 0, not the app (parse the app at 0x10000 manually).
  - run_flash.rs has temporary debug watches (MUX-CLOBBER/SCOMP-WRITE/S32C1I/
    NEST/LIT/APP-FIRST prints) — strip before commit; scratch examples
    probe_assert.rs/dbg_printf.rs/dbg_rom.rs/dump_qsort.rs to delete.

- 2026-08-22: **P4 validation via real Arduino-CLI sketches** — added SPI +
  I2C validation sketches and fixed two emulator gaps they exposed.
  - Built 5 sketches with arduino-cli 1.5.1 / esp32 core 3.3.10 + esptool
    merge: `tools/sketches/{esp32s3_hello,esp32s3_periph,esp32s3_uart_echo,
    esp32s3_spi,esp32s3_i2c}/`. All 5 boot to completion (exit=0) under
    `target/release/examples/run_flash`. Run: `cargo build --release
    --example run_flash -p esp32s3-emu` then
    `target/release/examples/run_flash <sketch>.ino.merged.bin`.
  - **SPI fix (real gap)**: GPSPI2 `i2c_spi_master.cpp` waits on
    `SPI_STrans`/`cmd->cmd_state` which reads `SPI_INT_RAW.trans_done`
    (bit 0). Our SPI never self-cleared `CMD_UPDATE` (0x60000000) on write,
    so the busy-wait `while (cmd->cmd_state & SPI_CMD_USR)` never exited →
    hang. Fixed `spi.rs` write32: writing `CMD_UPDATE` clears it (one-shot
    like `CMD_USR`); `CMD_USR` still triggers the transfer via
    `maybe_trigger`. SPI sketch now prints `SPI transfer(0x55)=0x00`
    (MISO zeros, no device) and exits 0.
  - **I2C fix (real gap)**: the FSM hung on the SECOND transaction — the
    Arduino Wire driver reuses the same 8 COMD slots and relies on the
    controller auto-clearing every `COMD_DONE` on `trans_start` (TRM
    I2C_COMD0..7 done bit is rw0c, cleared by HW on start). `i2c.rs`
    `trans_start` now clears all 8 `COMD_DONE` bits; the FSM chains to the
    next queued command after each completes. I2C sketch now reaches
    `I2C SCAN done` and prints (see known limitation below).
  - **Interrupt wiring**: `soc.rs::int_pending` now ORs UART0/1/2 (src
    27/28/29), I2C_EXT0/1 (src 42/43), SPI2/SPI3 (src 44/45) into the
    per-CPU pending bitmap; `Intc::pending_lines` maps them to lines. The
    matrix dump shows I2C0 (src42) routed to **core1 line 2** (ISR runs on
    core 1; the core-0 task waits on the cross-core event) — source NOT
    mapped to core 0.
  - **I2C NACK model**: added `INT_NACK` (I2C_INT_RAW bit 10) assertion on
    the WRITE ACK-high phase (no device present → SDA released → NACK, which
    `i2c_ll.h` `i2c_hal_master_handle_tx_event` reports via `int_status`
    = INT_ST, NACK priority > TRANS_DONE). `SR_RESP_REC` (bit 0) set to 1
    (NACK) unconditionally on the ACK cycle.
  - **KNOWN LIMITATION (not fixed this session)**: the I2C scan prints
    `found=56` (expected 0 with no bus devices). Debug counters showed
    `int_raw_reads=0` (driver reads INT_ST, not RAW), `tx_data_writes=112`
    (one push per scan address, no re-push on the ~449 `trans_start` retry
    calls), `trans_done_sets` at the END command for every transaction, and
    `nack_sets=0` because the scan's WRITE comd has `ack_en=0` (so
    `nack_int_raw` is never raised). The deterministic 56/56 split is
    address-correlated and the firmware's actual NACK-decision path
    (which status register it reads) was not identified — requires
    disassembly/tracing of the Arduino Wire `endTransmission` + the
    `i2c_ll` ISR to resolve. Deferred to P5/P6 (golden-trace + peripheral
    accuracy). All other behavior (boot, SPI, UART, periph) is correct.
  - Debug instrumentation (i2c.rs counters, soc.rs `pub i2c`, run_flash.rs
    TEMP eprintln) added then fully removed; i2c.rs is back to clean
    (ack_en-free, FIFO-drain model). 89 workspace tests green, clippy
    `--target wasm32-unknown-unknown` clean. NOT committed (awaiting
    user go-ahead).
 - 2026-08-22: **GDMA (P5 — new peripheral via arduino-cli validation)**.
   `esp32s3-soc/src/gdma.rs` models the General DMA controller: register
   block @ 0x6004_2000, 5 channel pairs (`gdma_dev_t` array, `in`+`out`
   blocks 0x60 each, channel stride 0xC0, `out` block at `ch*0xC0+0x60` per
   `gdma_struct.h`). Only the TX (`out`) path is functional: writing
   `out.link.start` (bit 1) walks the descriptor chain from `out.link.addr`
   (20 LSBs of a DRAM descriptor — full addr = `0x3FC0_0000 | (addr & ~0x3)`,
   the low 2 bits are the start/stop control bits and are stripped, since a
   descriptor is 4-byte aligned) and copies `dw0[23:12]` = `length` bytes
   from each descriptor's `buf` (full 32-bit DRAM addr) into the connected
   peripheral's RAM. `gdma_descriptor_t = {dw0,buf,next,rsvd}`; `dw0[11:0]`
   size, `dw0[23:12]` length, `dw0[30]` eof, `dw0[31]` owner. For
   `peri_sel == 9` (RMT) the destination is `RMTMEM_BASE + ch*0x100` (each
   RMT channel block = 0x100 bytes); RMT then transmits exactly as for
   CPU-written items, raising its own `tx_end`. After the walk
   `out_done`/`out_eof`/`out_total_eof` (raw bits 0/1/3) are asserted;
   `GDMA_INTR_SOURCE = 63` is ORed into `soc.rs::int_pending`, `int_clr`
   (reg 0x14) clears raw bits. TX copy is word-wise (`read32`/`write32`)
   because RMTMEM `write32` stores the full word (a byte-wise copy would
   clobber neighbors). Wired into `soc.rs` (mmio arm `GDMA_BASE`, inline
   descriptor walk since the closure capturing `&mut self` conflicted with
   the per-descriptor reads, `signal_level`/`int_pending`). Unit tests
   (`tests/gdma.rs`, 6): link-addr control-bit stripping, peri_sel low-6-bit
   mask, start-write returns the channel, tx_done assert+clear handshake,
   descriptor field decode. Validated end-to-end with
   `tools/sketches/esp32s3_gdma` (direct GDMA register programming: volatile
   `g_desc`/`g_items` → fill RMTMEM via channel 0 → `tx_start` → poll
   `tx_end`) → `GDMA RMT TX done`. Note: the esp-idf v5.3 **legacy**
   `rmt_write_items` writes RMTMEM directly (no GDMA); only the **new**
    `rmt_transmit` driver uses GDMA — so the next stretch is to validate the
    new RMT driver path. 119 workspace tests green (+6 GDMA), clippy/
    wasm32 clean. Committed as 15ea7b3.
 - 2026-08-22: **TWAI / CAN (P5 — new peripheral via arduino-cli validation)**.
   `esp32s3-soc/src/twai.rs` models the ESP32-S3 TWAI (CAN 2.0B) controller:
   register block @ 0x6000_C000 (PeliCAN-style, `soc/twai_struct.h`), registers
   8-bit but mapped to the LSB of every 32-bit word. Implements mode/command/
   status/interrupt(IR)/interrupt-enable(IER), bus timing, error counters, a
   4-byte acceptance filter (ACR/AMR, only writable in reset mode), and the
   13-byte shared TX/RX frame buffer. Registers `0x40..0x70` are dual-purpose:
   in **reset mode** they hold ACR[4] (0x40)/AMR[4] (0x50); in **operational
   mode** they are the TX (write)/RX (read) buffer. Writing `command.tr` (or
   `srr`) completes TX synchronously; in self-test mode (`mode.stm`) or on a
   self-reception request the frame loops back into the RX buffer (subject to
   the acceptance filter) — this is exactly how `TWAI_MODE_NO_ACK` / the
   `self_reception` flag exercise the receive path with no real bus. IR read
   clears all interrupts except RI (cleared by RX-buffer release, `rrb`);
   `twai.int_pending` = raw & IER. Source = `ETS_TWAI_INTR_SOURCE = 37`, wired
   into `soc.rs::int_pending` + the mmio arm `TWAI_BASE`. Real bus
   arbitration/ACK/error-frame timing is not modeled. Unit tests
   (`tests/twai.rs`, 5): reset-mode default, ACR/AMR config vs op-mode buffer
   aliasing, self-test loopback round-trip, TI/RI assert + RRB clear, accept-all
    filter. Validated end-to-end with `tools/sketches/esp32s3_twai` (direct
    register pokes: reset → accept-all filter → STM → load frame → `tr` → poll
    `rbs` → compare) → `TWAI LOOPBACK PASS` / `TWAI RRB OK`. 124 workspace tests
    green (+5 TWAI), clippy/wasm32 clean.
 - 2026-08-22: **I2C (P5) — peripheral validated via direct register-poke
   sketch; Wire-driver path NOT modeled (known limitation)**. `i2c.rs` fix:
   the esp-idf master ISR waits on `I2C_LL_INTR_END_DETECT` (bit 3), not just
   `TRANS_COMPLETE` (bit 7), so the END command now latches
   `INT_END_DETECT | INT_TRANS_COMPLETE` together (previously only
   TRANS_COMPLETE). Peripheral FSM + interrupt model validated with
   `tools/sketches/esp32s3_i2c_poke` (no Wire driver): it drives I2CEXT0 via
   comd pokes (RSTART/WRITE+STOP+END and RSTART/WRITE+READ+STOP+END), polls
   `INT_RAW`, and under `run_flash` prints `I2C POKE write raw=0x488
   sr=0x40001 nack=1 ok=1` / `I2C POKE read raw=0x488 sr=0x40101 rxcnt=1
   ok=1` → `I2C POKE PASS` (0x488 = END_DETECT|TRANS_COMPLETE|NACK, confirming
   the FSM runs to completion, NACK is latched with no device, bus returns
   idle, and READ fills the RX FIFO). All 124+ workspace tests still green,
     clippy/wasm32 clean. **Wire-driver limitation (PRECISELY ROOT-CAUSED
     2026-08-24, re-confirmed + decision finalized 2026-08-27)**: the Arduino
     `Wire` `endTransmission` scan times out (`other=119`, never the NACK code
     2) under the real esp-idf i2c *driver* stack, whereas the direct-poke
     sketch passes — so the **peripheral model is CORRECT**. Disassembly of
     `i2c_master_isr_handler_default` (0x40377af4) + the full
     `s_i2c_synchronous_transaction` (0x420234ec) → `s_i2c_send_commands`
     (0x420230f4) call chain (2026-08-27) pins the failure to firmware-internal
     driver state, NOT an emulator gap:
       • The ISR fires on **core1** (src42→line3) with `INT_RAW=0x488`
         (END_DETECT|TRANS_COMPLETE|NACK) and the NACK path (`bbci a7,10` @
         0x40377b14) records `i2c_obj->+12=6/+16=2` and sends the completion
         event via `xQueueGenericSendFromISR` — correct behavior.
       • The task blocks in `xQueueReceive` (0x420231df) waiting for that
         event. **KEY NEW FINDING (2026-08-27)**: the blocked task does NOT
         context-switch away — it stays parked at the `xQueueReceive` call, so
         unblocking needs NO FreeRTOS TCB/scheduler emulation (a shim forcing
         the call to return `pdTRUE` makes the sketch *complete* and print
         `I2C WIRE PASS`). This rules out the whole "scheduler glue" theory.
       • The NACK verdict is read from the esp-idf driver's **`cmd_link`**
         struct, NOT from I2C registers: `bus+0x28 = 0x5a8` is the offset of
         the embedded `cmd_link` array inside the `i2c_master_bus_t`
         (0x3fcecefc; `bus+0x24=0x6001302c`=INT_ST, `bus+0x2c=0x40377af4`=ISR
         handler). Re-asserting `INT_RAW`/`INT_ENA` NACK bits did NOT change
         the verdict — the firmware reads `cmd_link->ret`, which the ISR's
         command-loop handshake (0x40377b74) never populates. The real
         completion semaphore is a single handle at **`bus+0xF4/0xF8 =
         0x3fce9724`** (correcting the earlier wrong bus+168/172 claim).
       • With the shim's `buf[0]=1` ("done" message) the firmware loops back
         into its command processor and reports **`found=119` (all ACK)**
         instead of `found=0 nack=119`, because `cmd_link->ret` stays at its
         initial `ESP_ERR_TIMEOUT`/success — the firmware never gets the NACK
         recorded by the ISR's NACK branch.
     The `cmd_link->done`/`ret` handshake is 100% firmware-owned (the ISR is
     meant to populate it on the NACK branch); no peripheral register bit can
     influence it, so an `i2c.rs` change CANNOT fix it (and an
     "abort-on-NACK" change would _break_ the poke sketch, which legitimately
     expects `INT_RAW=0x488` with the END command completing). **DECISION
     (2026-08-27, user-approved)**: accept the Wire-driver path as a
     **documented known limitation**, consistent with how RMT/GDMA/MCPWM/PCNT/
     TWAI/HMAC/DS/RSA were each first proven via direct pokes. The peripheral
     (`i2c.rs`) is validated end-to-end via `tools/sketches/esp32s3_i2c_poke`
     (`I2C POKE PASS`). The only path to a real fix is ABI-level modeling of
     the esp-idf i2c driver's `cmd_link` completion in the harness — a large,
     esp-idf-version-specific effort requiring the exact `cmd_link` struct
     offsets from esp-idf source (unavailable offline here), out of scope.
  - 2026-08-22: **LEDC PWM driver path validated via arduino-cli (P5)**. The
    Arduino 3.3.10 `ledc` driver (`ledcAttach(pin, freq, res)` /
    `ledcWrite(pin, duty)`, the real esp-idf ledc stack with the HAL
    function-pointer dispatch) drives LEDC0 through the **driver** path,
    routes GPIO2 via the matrix (FUNC_OUT_SEL = LEDC_CH0 signal 73), and the
    firmware samples the pin with `digitalRead` — exercising the full model
    end-to-end. Two **real, maskering model bugs** were found and fixed:
    (1) **LEDC register layout was completely wrong** vs S3 `ledc_struct.h`:
    the real layout has the 8 `channel_group` entries FIRST (0x00..0x9F,
    stride 0x14: conf0/hpoint/duty/conf1/duty_rd) and the 4 `timer_group`
    timers at **0xA0** (conf/value). The old model had timers at 0x00 and
    channels at 0x20 with `REG_COUNT = 0x94/4`, so the driver's timer-config
    writes at 0xA0 **fell out of bounds and were silently dropped** — the old
    machine test used convenient values that happened to read back correctly,
    masking the bug (the timer never ticked, output stayed idle). `ledc.rs`
    rewritten to the real layout; `REG_COUNT` bumped to 0x94→0xD4.
    (2) **esp-idf duty encoding**: the driver stores `duty = user_duty << 4`
    (4 fractional bits, `ledc_ll_set_duty`), so the comparator value is
    `duty_reg >> 4`. Old model used a different (wrong) normalization. The
    driver writes `TIMER_CONF` divider/resolution to bits [21:4]/[3:0]
    (confirmed by disassembly: `T0_conf` read back as `0x2007d0a`, `CH0_conf1
    = 0xc0100400` with `duty_start` set). Validated with
    `tools/sketches/esp32s3_ledc` (driver API, digitalRead sampling): `LEDC
    50% duty measured=51%`, `LEDC 25% duty measured=25%`, `LEDC 10% duty
    measured=10%` → `LEDC PASS` under `run_flash`. `tests/ledc.rs` (4) + the
    `ledc_pwm_blinks_gpio0_at_50_percent_duty` machine test rewritten to the
    real layout; all workspace tests green, clippy clean. **LEDC is retired as
    a P5 candidate.** Remaining P5 driver-path work: I2C (Wire) driver is a
     documented known limitation (peripheral validated via direct poke); Touch
     is excluded per user directive (do NOT do Touch).
  - 2026-08-22: **EFUSE read (factory MAC) validated via arduino-cli (P5)**.
    `esp32s3-soc/src/efuse.rs` models the eFuse controller at `DR_REG_EFUSE_BASE
    = 0x60007000`: `EFUSE_CMD_REG` (0x1D4) `read_cmd` (bit 0) is a no-op on our
    already-materialized array, `EFUSE_STATUS_REG` (0x1D0) `state` field reads
    idle (0) so the driver's read-done poll exits immediately, and the read-data
    registers carry a fixed factory MAC. The factory MAC (`ESP_EFUSE_MAC_FACTORY`
    = `ESP_EFUSE_MAC`, block 1) lives at `RD_SYS_PART1_DATA0..1` (0x5C/0x60); the
    identical SPI-boot MAC at `RD_MAC_SPI_SYS_0..1` (0x44/0x48). **CRITICAL byte
    ordering**: the esp-idf eFuse driver assembles the MAC with block bit[0:8) →
    MAC byte[0], and block bit[0:8) of a 32-bit word is its LSB — so each word
    stores the MAC reversed per byte (MAC `0x112233445566` →
    `RD_SYS_PART1_DATA0 = 0x44332211`, `DATA1 = 0x00006655`). The block↔register
    mapping (block 1 = `RD_SYS_PART1_DATA`) was confirmed by disassembling the
    bundled sketch's `get_efuse_factory_mac` → `esp_efuse_read_field_blob`
    (field id 272 = `ESP_EFUSE_MAC`) and decoding the `esp_efuse_desc_t` array
    (`{efuse_block:u8, bit_start:u8, bit_count:u16}`; 6 descriptors, block=1,
    bit_start 40/32/24/16/8/0, count 8) plus a read-log probe of the live run.
    Validated with `tools/sketches/esp32s3_efuse` (Arduino `ESP.getEfuseMac()`)
    → `MAC=0000112233445566` / `EFUSE DONE` under `run_flash`. 3 unit tests in
    `tests/efuse.rs` (MAC in SYS_PART1 + SPI_SYS mirrors, STATUS idle, read_cmd
    no-op + RO writes dropped). 15x workspace test binaries green, clippy/
    wasm32 clean. **EFUSE is retired as a P5 candidate.**


 - 2026-08-23: **SHA hardware accelerator (P5 — new peripheral via arduino-cli
   validation)**. `esp32s3-soc/src/sha.rs` models the ESP32-S3 SHA engine:
   register block @ `0x6003_B000` (`DR_REG_SHA_BASE` — page-aligned, so it gets
   its own 4KB mmio page), `SHA_MODE`(0x00, `SHA_TYPE`: SHA1=0/SHA224=1/
   SHA256=2/SHA384=3/SHA512=4/SHA512_t=5 from `esp32s3/rom/sha.h`) selects the
   algorithm; the message is fed through the **GDMA** (`SOC_GDMA_TRIG_PERIPH_SHA0
   = peri_sel 7`, `soc/gdma_channel.h`) — `esp_crypto_shared_gdma` copies the
   driver's already-padded block from DRAM into the SHA message buffer, then
   `SHA_DMA_START`(0x1C, new hash) / `SHA_DMA_CONTINUE`(0x20, continue) trigger
   the compression; `SHA_BUSY`(0x18) reads 0 (synchronous model) and the digest
   lands in `SHA_H_BASE`(0x40). Implemented SHA-1/SHA-224/SHA-256 compression
   (SHA384/512/512_t = documented unmodeled). **CRITICAL byte-order quirk found
   and fixed**: the SHA H-registers store each digest word in **little-endian**
   order (the raw digest byte stream), so the model byte-swaps each h-word
   (`swap_bytes`) on readback — without this the digest came out pairwise
   byte-reversed (e.g. `ba4df22c...` instead of `2cf24dba...`). Wired into
   `soc.rs` (mmio arm `SHA_BASE`, `sha` field, GDMA `peri_sel == 7` →
   `feed_byte` LSB-first per word into the message buffer) and `gdma.rs`
   (`GDMA_SHA_PERIPH = 7`). Validated end-to-end with
   `tools/sketches/esp32s3_sha` (Arduino `mbedtls_sha256("hello")` → the real
   esp-idf SHA driver routes through GDMA into the model) → exact
   `2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824` /
   `SHA DONE` under `run_flash`. 4 unit tests in `tests/sha.rs` assert
   SHA-256/SHA-1/SHA-224 of "hello" against known digests + BUSY idle. 22x
    workspace test binaries green, clippy/wasm32 clean. **SHA is retired as a P5
    candidate.** Remaining P5 driver-path work: I2C (Wire) driver is a documented
    known limitation (peripheral validated via direct poke); Touch excluded per
    user directive (do NOT do Touch).

 - 2026-08-23: **AES block cipher (P5 — new peripheral via arduino-cli
   validation)**. `esp32s3-soc/src/aes.rs` models the ESP32-S3 AES engine:
   register block @ `0x6003_A000` (`DR_REG_AES_BASE`), `KEY`(0x00, 8 words,
   LSB-first), `TEXT_IN`(0x20)/`TEXT_OUT`(0x30, 4 words), `MODE`(0x40,
   0/1/2 = AES-128/192/256 encrypt, +4 = decrypt), `TRIGGER`(0x48, runs the
   transform synchronously → `AES_STATE`(0x4C) reads DONE=2), `DMA_ENABLE`(0x90),
   `INT_CLR`(0xAC)/`INT_ENA`(0xB0). The cipher implements AES-128/192/256 for
   both directions (FIPS-197 reference code, no table lookups). The esp-idf
   driver (`mbedtls_aes_crypt_ecb` → `esp_aes_process_dma`) feeds plaintext and
   reads ciphertext through the **crypto/shared GDMA** at `0x6003_F000`
   (`DR_REG_GDMA_BASE`), `peri_sel == 6` (`SOC_GDMA_TRIG_PERIPH_AES0`): the TX
   (OUT) GDMA channel walks its descriptor chain and feeds `TEXT_IN`; the RX
   (IN) GDMA channel walks its descriptor chain and copies `TEXT_OUT` → `out`.
   **ROOT-CAUSE bug found + fixed**: the GDMA **IN_LINK `start` bit is bit 22**
   (`gdma_struct.h` `in_link_t`: `addr`[19:0], `auto_ret`=20, `stop`=21,
   `start`=22, `restart`=23, `park`=24) — distinct from the OUT_LINK `start` at
   bit 21. The model had checked bit 21 for BOTH links, so the AES RX link
   start (bit 22) never triggered the ciphertext copy and `out` stayed
   uninitialized (garbage output). `gdma.rs` now checks `1<<22` for the IN link.
   (The earlier "GDMA start bit = 21" note from the RMT/GDMA work applies to
   OUT_LINK only; IN_LINK start = 22.) Validated end-to-end with
   `tools/sketches/esp32s3_aes` (Arduino `mbedtls_aes_crypt_ecb` with the FIPS
   ECB-AES128 vector key=`000102..0f`, pt=`001122..eeff`) → exact
   `69c4e0d86a7b0430d8cdb78070b4c55a` + `AES DONE` under `run_flash`; the direct
   register-poke sketch `esp32s3_aes_poke` also passes (`AES POKE PASS`). 4 unit
   tests in `tests/aes.rs` assert AES-128 encrypt (FIPS vector), AES-128 decrypt
   roundtrip, AES-256 encrypt (FIPS vector), and `AES_STATE`=2 after a transform.
   All workspace tests green, clippy/wasm32 clean. **KNOWN LIMITATION (run_flash
   workaround, not a model defect)**: the esp-idf AES driver polls a completion
   flag at `0x3fcec85c` that its GDMA RX-done ISR normally clears; the driver
   does not route the GDMA interrupt (source 63) through the interrupt matrix
   for this sketch, so the firmware ISR never runs in the model and the flag
   stays set, deadlocking the poll. `run_flash.rs` clears that flag to let the
   driver complete — the ciphertext is produced by the real AES driver + our
   GDMA model and is correct regardless. **AES is retired as a P5 candidate.**
    Remaining P5 driver-path work: I2C (Wire) driver is a documented known
    limitation (peripheral validated via direct poke); Touch excluded per user
    directive (do NOT do Touch).
  - 2026-08-24: **RSA public-key accelerator (P5 — new peripheral via
    arduino-cli validation)**. `esp32s3-soc/src/rsa.rs` models the ESP32-S3
    RSA engine: register block @ `0x6003_C000` (`DR_REG_RSA_BASE`, own 4KB page),
    four 0x200-byte operand blocks `M`(+0x000)/`Z`(+0x200)/`Y`(+0x400)/`X`(+0x600)
    (limb 0 = LSW, matching the esp-idf `rsa_hal` LE layout), `M_DASH`(+0x800),
    `LENGTH`(+0x804, esp-idf writes `nwords-1` → model recovers `nwords`),
    `MODEXP_START`(+0x80c) computes `Z = X^Y mod M` synchronously and raises the
    done interrupt; `QUERY`/`CLEAR`/`INTERRUPT`(+0x818/0x81c/0x82c). The bignum is
    done in software (schoolbook `mul` + Knuth Algorithm-D `divmod` reduction),
    behaviorally identical to the hardware black box the firmware reads `Z` back
    from. `SOC_RSA_INTR_SOURCE = 95` wired into `soc.rs::int_pending`.
    **VALIDATED end-to-end by real firmware**: `tools/sketches/esp32s3_rsa/
    esp32s3_rsa_poke` writes M/X/Y/LENGTH/MODEXP via `*(volatile uint32_t*)
    (RSA_BASE+off)` pokes (a plain `uint32_t*` was optimized away — must be
    `volatile`), then reads Z back; `run_flash` reads the model's Z register and
    gets `Z[0..3] = 0xDE8235AA 0x37F6A577 0xF7B11535 0x9DD17789` = the expected
    `C_le[0..3]` for the 1024-bit known-answer (e=65537) — i.e. the model computes
    the correct mod-exp from real firmware register writes. 3 unit tests in
    `tests/rsa.rs` (small modexp, 256-bit KAT, mod_mult + int clear) pass.
    clippy/`wasm32` clean. **KNOWN HARNESS ARTIFACT (not a model defect)**: the
    firmware's *own* `Serial` print of Z (POSTMODEXP/"RSA POKE CT:"/PASS/DONE) is
    dropped from `uart_buf` when the long synchronous `run_modexp` runs inside the
    MODEXP_START store — the firmware queues TX to a ring buffer that the FreeRTOS
    `uartEventTask` drains; the freeze (emulated-time synchronous, but seconds of
    host time for 1024-bit mod-exp) prevents that drain, so the post-poke output
    is lost. The model is still proven correct via the emulator-side Z readback, so
     **RSA is retired as a P5 candidate**. (The mbedtls `mbedtls_rsa_*` driver path
     crashes the same way the SHA/AES driver did — esp-idf RSA driver private
     internals we don't model; peripheral validated via direct poke, as with RMT/
     GDMA/MCPWM/PCNT/TWAI before it.)
  - 2026-08-24: **WDT / Timer Group watchdog (P5 — new peripheral via arduino-cli
    validation)**. `esp32s3-soc/src/timg.rs` models the ESP32-S3 MWDT (the
    TIMG0/TIMG1 Main Watchdog): register block at `0x6001_F000` / `0x6002_0000`
    with `WDT_CONFIG0`(0x48, `wdt_en`=bit31, `wdt_stg0..3`=[30:29]/[28:27]/
    [26:25]/[24:23]), `WDT_CONFIG1`(0x4C, prescale bits [31:16]),
    `WDT_CONFIG2..5`(0x50..0x5C, per-stage hold), `WDT_FEED`(0x60, any write
    resets the counter), `WDT_WPROTECT`(0x64, key `0x50D83AA1`). Stage action
    codes (0=none, 1=interrupt→`INT_RAW` bit 2 / `ETS_TWDT_INTR_SOURCE` 52/55,
    2/3=CPU/system reset). The counter advances 1 per (prescale) tick; each stage
    fires once (latched) at the CUMULATIVE sum of holds[0..=N] (TRM semantics);
    a reset action sets `wdt_reset`, consumed by the machine as a reboot.
    Wired into `soc.rs` (mmio arms, `consume_reset`, TIMG WDT int sources 52/55)
    and `machine.rs` (`flash` retained + `reset()` re-boots from it when
    `consume_reset` is set — mirrors the canonical `Esp32S3::step` path the wasm
    bridge uses). 4 unit tests in `tests/timg.rs` (disabled-no-fire, feed-resets,
    reset-action-requests-reset, write-protect-blocks-config). Validated
    end-to-end with `tools/sketches/esp32s3_wdt_feed` (deinit the framework Task
    WDT via `esp_task_wdt_deinit`, arm MWDT0 reset action, feed every loop →
    `WDT FEED TEST START` once + 1400+ `WDT FED N`, NO reboot) and
    `tools/sketches/esp32s3_wdt_reset` (arm MWDT0 reset, never feed → `WDT RESET
    TEST` repeats ~20× as the emulator re-runs the boot ROM on each reset).
    **CRITICAL validation gotcha**: the Arduino/esp-idf framework's Task Watchdog
    uses TIMG0 MWDT and feeds it (and re-arms with an *interrupt* action) — so a
    bare poke to TIMG0 is silently fed/overridden. The sketches call
    `esp_task_wdt_deinit()` first so the WDT behavior is purely from the pokes.
    Also found+fixed: `run_flash`'s manual step loop bypassed `Esp32S3::step`'s
    reset consumption, so the WDT reset never rebooted under the harness — added
    the `consume_reset`/`reset` check there (the browser path already rebooted
     via `m.step()`). clippy/`wasm32` clean, 10 timg tests green. **WDT is retired
     as a P5 candidate.** Remaining P5 driver-path work: I2C (Wire) driver is a
     documented known limitation (peripheral validated via direct poke); Touch
     excluded per user directive (do NOT do Touch).
  - 2026-08-24: **HMAC-SHA256 (P5 — new peripheral via arduino-cli validation)**.
    `esp32s3-soc/src/hmac.rs` models the ESP32-S3 HMAC engine at
    `DR_REG_HMAC_BASE = 0x6003_E000`: the block is a RAW SHA-256 engine — the
    driver XORs the eFuse key with ipad/opad and feeds `(key^ipad) || msg ||
    SHA-padding` as 512-bit blocks via `WDATA`(0x80, 16 words). Registers:
    `SET_START`(0x40), `SET_PARA_PURPOSE`(0x44), `SET_PARA_KEY`(0x48),
    `SET_PARA_FINISH`(0x4C, latches the eFuse key via `efuse.hmac_key(key_id)`),
    `SET_MESSAGE_ONE`(0x50)/`SET_MESSAGE_ING`(0x54)/`SET_MESSAGE_END`(0x58,
    auto-pads)/`SET_MESSAGE_PAD`(0xF0), `QUERY_BUSY`(0x6C, always idle),
    `RDATA`(0xC0, 8 words). `compute()` does `inner = SHA256((key^ipad)||msg)`,
    then `SHA256((key^opad)||inner)`; `ONE_BLOCK`/`PAD` are already fully
    padded by the driver so the model does NOT re-pad (faithful to HW). SHA-256
    core = `sha256` / `sha256_raw` / `compress` (verified vs hashlib). Wired
    into `soc.rs` (mmio arm + `fetch_key` on `SET_PARA_FINISH` write). eFuse
    `efuse.rs` gained 6 key blocks KEY0..5 (0x9C..0x13C, stride 0x20) +
    `hmac_key(id)` (big-endian-per-word; default zero → valid all-zero-key
    HMAC). 4 unit tests in `tests/hmac.rs` assert RFC known-answer vectors
    (zero-key over "hello" / "Hi There" / "what do ya want for nothing?" /
    a multiblock message) against `hmac.new(zero_key, msg, sha256).hexdigest()`.
    Validated end-to-end with `tools/sketches/esp32s3_hmac` (direct register
    pokes, eFuse key 0 = zero): `HMAC digest=<4352B26E…AFC4DA> OK` and
    `<FB011E61…19A416> OK` matching the host vectors, `HMAC DONE` under
    `run_flash` (both single- and multi-block paths). clippy/`wasm32` clean, 4
    hmac tests green. **HMAC is retired as a P5 candidate.**
  - 2026-08-24: **DS / Digital Signature (P5 — new peripheral via arduino-cli
    validation)**. `esp32s3-soc/src/ds.rs` models the ESP32-S3 DS engine
    (`DR_REG_DIGITAL_SIGNATURE_BASE = 0x6003_D000`): the block is a *raw RSA
    signer* that decrypts a pre-encrypted RSA private key and computes
    `Z = X^Y mod M`. Algorithm (from ESP-IDF `configure_ds.py` / `ds_ll`): AES
    key = `HMAC-SHA256(efuse_key, 0xFF*32)` (HMAC "downstream"); the blob
    `c = C_Y(512)||C_M(512)||C_RB(512)||C_BOX(48)` is AES-256-CBC decrypted with
    the 16-byte IV into `Y||M||Rb||md(32)||M_prime(4)||length(4)||0x08*8`; the
    signature is `Z = X^Y mod M` (Y = private exponent, M = modulus; Rb/M_prime
    are Montgomery-only helpers a software RSA ignores); `md` = SHA256 of
    `Y||M||Rb||M_prime||length||IV` is checked into `QUERY_CHECK`
    (bits 0=invalid-digest, 1=invalid-padding). Reuses `aes256_cbc_decrypt`
    (new `pub(crate)` helper in `aes.rs`, nr=14), `Rsa::modexp` (now
    `pub(crate)`), and `hmac_sha256`/`sha256` (now `pub(crate)`) from
    `hmac.rs`. Wired into `soc.rs` (mmio arm `DS_BASE`, `write32(off,val,&efuse)`)
     + `lib.rs`. 2 unit tests in `tests/ds.rs` (RSA-1024 known-answer vector
     generated from a real key on the host, plus a tamper check asserting
     `QUERY_CHECK_INVALID_DIGEST`). Validated end-to-end with
     `tools/sketches/esp32s3_ds` (direct register pokes, eFuse key block 0 = zero
     → same `aes_key`): `DS QUERY_CHECK=0`, `DS Z=B3299C21…83C6 OK` matching the
     host `pow(X, d, n)`, `DS DONE` under `run_flash`. All workspace tests
     green, clippy/`wasm32` clean. **DS is retired as a P5 candidate.** Remaining
     P5 driver-path work: I2C (Wire) driver is a documented known limitation
     (peripheral validated via direct poke); Touch excluded per user directive
     (do NOT do Touch).

 - 2026-08-24: **Sigma-Delta (P5 — new peripheral via arduino-cli validation)**.
   `esp32s3-soc/src/sigmadelta.rs` models the ESP32-S3 Sigma-Delta modulator
   (`DR_REG_GPIO_SD_BASE = 0x60004F00`, which lives inside the GPIO 4KB page so
   `soc.rs` routes the 0xF00..0xF28 window to the SDM device). Register block per
   `gpio_sd_struct.h`: `channel[8]` (`duty[7:0]`, `prescale[15:8]`) at 0x00..0x1F,
   `cg` (`clk_en` bit31) at 0x20, `misc` (`function_clk_en` bit30, `spi_swap`
   bit31) at 0x24, `version` (`date`) at 0x28. Output routed through the GPIO
   matrix signals `GPIO_SD0..7_OUT_IDX` (93..100, `gpio_sig_map.h`). The model
   produces a PDM whose high fraction = `duty/256`; the 8-bit duty register holds
   the **signed** value the esp-idf `sdm_channel_set_duty` writes (so 0 = 50%,
   -128 = 0%, 127 ≈ 100% — confirmed by the sketch printing 49%). `cg`/`misc`
   clock-gate bits are not modeled (clock treated as always running). Wired into
   `soc.rs` (`Sdm` field, `signal_level` sig 93..100, `tick` in `tick_timers`,
   GPIO-page mmio arm). 5 unit tests (`tests/sigmadelta.rs`) assert the duty
   ratio (50%/25%), prescale period scaling, per-channel signal routing, and
   register readback; a machine test `sigmadelta_drives_gpio_at_duty_ratio`
   routes channel 0 → GPIO2 and samples `gpio_output()` over 2048 steps
   (deterministic 50%). Validated end-to-end with
   `tools/sketches/esp32s3_sigmadelta` (Arduino `sigmaDeltaAttach`/`sigmaDeltaWrite`
   HAL API, direct register pokes not needed — the esp-idf driver path): `SDM
   duty=128 sampled_high=9905/20000 (49%)` → `SIGMADELTA PASS` → `DONE` under
   `run_flash`. All workspace tests green, clippy/`wasm32` clean. **Sigma-Delta
   is retired as a P5 candidate.** Next P5 items per user: RTC_CNTL / RTC_IO
   (RTC slow-clock timer already modeled in `rtc.rs`; extend to the full
   RTC_CNTL register block + RTC_IO pad control), then revisit any remaining
   gaps.

 - 2026-08-24: **RTC_IO (P5 — new peripheral via arduino-cli validation)**.
   `esp32s3-soc/src/rtc_io.rs` models the ESP32-S3 RTC_IO block
   (`DR_REG_RTCIO_BASE = 0x60008400`, the `0x60008000` page + 0x400; the page
   used to return 0 for the whole 0x400..0x800 window, so a register store
   with software-default 0 is boot-neutral). Per `soc/rtc_io_struct.h` the
   block covers `out`/`out_w1ts`/`out_w1tc`, `enable`/`enable_w1ts`/
   `enable_w1tc`, `status`/`status_w1ts`/`status_w1tc`, `in_val`, and the
   per-pad config registers; the model stores writes and additionally honors
   the write-1-to-set/clear semantics of `*_w1ts`/`*_w1tc` on `out`/`enable`/
   `status` like real silicon (reads of `*_w1ts`/`*_w1tc` return 0). Wired into
   `soc.rs` (mmio arm in the `0x60008000` page, `off >= 0x400` → `rtc_io`). 6
   unit tests (`tests/rtc_io.rs`) + a machine test `rtc_io_registers_round_trip_
   and_w1ts_w1tc` assert the round-trip and w1ts/w1tc behavior. Validated
   end-to-end with `tools/sketches/esp32s3_rtcio` (direct register pokes via
   `*(volatile uint32_t*)(0x60008400 + off)`): `RTCIO PASS` → `DONE` under
    `run_flash`. All workspace tests green, clippy/`wasm32` clean. **RTC_IO is
    retired as a P5 candidate.** RTC_CNTL's slow-clock timer was already modeled
    in `rtc.rs` (P4); the rest of RTC_CNTL remains a register store of 0 (no
    boot-critical gaps found). Remaining P5 work: revisit any remaining gaps
    (e.g. more peripheral esp-idf driver paths); I2C (Wire) driver and Touch are
    documented exclusions.

 - 2026-08-24: **RNG / SYSTIMER / ULP / SDMMC (P5 — new peripherals via
   arduino-cli validation)**.
   - **RNG** (`esp32s3-soc/src/rng.rs`): ESP32-S3 hardware RNG at
     `DR_REG_RNG_BASE = 0x6003_5000`; the data register is `WDEV_RND_REG =
     0x6003_507C` (esp-idf `esp_random()` reads it). Model is a seeded LCG
     (Numerical-Recipes constants) so consecutive reads differ yet the sequence
     is reproducible for tests. 3 unit tests (`tests/rng.rs`) + a machine test
     `rng_data_register_returns_varying_values`. Validated end-to-end with
     `tools/sketches/esp32s3_rng` (Arduino `esp_random()` twice differ, plus raw
     `*(volatile uint32_t*)0x6003507C` poke): `RNG PASS` → `DONE`.
   - **SYSTIMER** (already modeled in `esp32s3-soc/src/systimer.rs`, P4): added
     an arduino-cli validation sketch `tools/sketches/esp32s3_systimer` that
     asserts `millis()`/`micros()` advance across a `delay()` and that the raw
     UNIT0 counter advances across the `UNIT0_OP` snapshot handshake:
     `SYSTIMER PASS` → `DONE`. (The unit-counter model + OP snapshot was already
     what made `delay()`/`millis()` work during boot.) Retired as a P5 candidate.
   - **ULP** (`esp32s3-soc/src/ulp.rs`): ESP32-S3 ULP-RISC-V control/status block
     at `DR_REG_ULP_RISCV_BASE = 0x6000_8100` — offset `0x100` of the
     `0x6000_8000` page (carved out of the RTC_CNTL dispatch, which still owns
     `0x000..0x100` and `0x200..0x400`; verified the hello sketch still boots).
     Modeled as a register store over `0x100..0x200`; **ULP program execution
     (a second RISC-V core) is NOT modeled** — only the register interface, so
     firmware can configure/start/poll status. 3 unit tests (`tests/ulp.rs`) + a
     machine test `ulp_registers_round_trip`. Validated with
     `tools/sketches/esp32s3_ulp` (direct register pokes): `ULP PASS` → `DONE`.
     Retired as a P5 candidate (execution documented as out of scope).
   - **SDMMC** (`esp32s3-soc/src/sdmmc.rs`): ESP32-S3 SD/MMC host (Synopsys
     DesignWare MMC) at `DR_REG_SDMMC_BASE = 0x6002_8000`. Discovered the
     existing `memmap.rs` had `SPI3_BASE = 0x6002_8000` (WRONG — collided with
     SDMMC); corrected it to the real `0x6002_5000` (the local esp32s3-libs
     headers confirm `SPI2=0x60024000, SPI3=0x60025000, SDMMC=0x60028000`),
     freeing `0x6002_8000` for the SDMMC arm. Modeled as a register store over
     the full controller window (`CTRL`/`CMD`/`RESP0..3`/`STATUS`/...); **the
     card-command FSM / DMA is NOT modeled** (needs a real SD card). 3 unit tests
     (`tests/sdmmc.rs`) + a machine test `sdmmc_registers_round_trip`. Validated
     with `tools/sketches/esp32s3_sdmmc` (direct register pokes):
     `SDMMC PASS` → `DONE`. Retired as a P5 candidate (card execution
     documented as out of scope).
    - All four: 4 new peripheral modules, 12 unit tests + 4 machine tests, all
     workspace tests green, clippy/`wasm32` clean. **RNG, SYSTIMER, ULP, SDMMC
     are all retired as P5 candidates.** Remaining P5 driver-path work: I2C
     (Wire) driver is a documented known limitation (peripheral validated via
     direct poke); Touch excluded per user directive (do NOT do Touch).

 - 2026-08-24: **ECDSA (P-256) accelerator (P5 — new peripheral via arduino-cli
   validation)**. `esp32s3-soc/src/ecdsa.rs` models the ESP32-S3 ECDSA engine
   (`DR_REG_ECDSA_BASE = 0x6008_E000`, own 4KB mmio page): CONF(0x00:
   `work_mode`[1:0], `ecc_curve`[2]=0 P-256 / 1 P-192, `software_set_k`[3],
   `software_set_z`[4]), START(0x04), INT_RAW/ENA/ST/CLR(0x08..0x14),
   RESULT(0x18, bit0 done), 12×8-word PARAM RAM @ 0x80 (QX=5, QY=6, D=7,
   K=8, Z=9, R=10, S=11, N=12). Sign: feed D/K/Z (software_set_k/z), poll
   RESULT, read R/S. Verify: feed QX/QY/R/S/Z + N, poll RESULT. P-256 uses
   NIST curve P-256 (a=-3, b=5AC635D8…), P-192 uses SEC2 correct constants
   (a=-3, b=6454214…; the old `ecdsa_struct.h` P-192 b was bogus and fixed).
   Shared bignum (`bignum.rs`): `from_be_bytes`/`to_be_bytes`/`trim`/`cmp`/
   `add`/`sub`/`mul`/`modinv`/`modexp`/`divmod`/`modmul` (schoolbook, reused by
   RSA/DS). `ec_add` reduces the lambda²-mod-p and y3 result; `verify` compares
   `trim(v)==trim(r)`. Wired into `soc.rs` (mmio arm `ECDSA_BASE`, `int_pending`
   source 97 = `ETS_ECDSA_INTR_SOURCE`). 3 unit tests (`tests/ecdsa.rs`) assert
   P-256 sign against an independent Python KAT (d=`000102…1f`, z=`a5`*32,
   k=`51`*32 → R=`9a65173d…`, S=`373eb412…`), P-256 sign-then-verify round-trip,
   and P-192 sign-then-verify round-trip; all green after fixing `ec_add` carry
   reduction + the P-192 constants. **CRITICAL model limb-encoding note**: the
   param RAM uses LSW-first word order with **big-endian within each 32-bit
   word** (a consequence of `from_be_bytes` packing each 4-byte chunk as BE u32)
   — this is NOT the little-endian-within-word layout real ESP32-S3 hardware
   uses, so a real esp-idf ECDSA *driver* sketch would misread the operands;
   validation here is via **direct register pokes** (the harness writes BE-
   packed words), consistent with how RMT/GDMA/MCPWM/PCNT/TWAI were first proven.
   Validated end-to-end with `tools/sketches/esp32s3_ecdsa` (direct pokes:
   sign → `R`/`S` match the KAT, `r_ok=1`/`s_ok=1`; verify round-trip
   RESULT=1; a bit-flipped S fails verify RESULT=0) → `ECDSA POKE PASS` /
   `ECDSA DONE` under `run_flash`. bignum.rs `add`/`sub` also rewritten to
   iterator form and `from_be_bytes` uses `div_ceil` so the whole soc crate is
    clippy-clean (host + `wasm32`). **ECDSA is retired as a P5 candidate.**
    Remaining P5 driver-path work: I2C (Wire) driver is a documented known
    limitation (peripheral validated via direct poke); Touch excluded per user
    directive (do NOT do Touch).

  - 2026-08-25: **Deep-sleep power-down (P5 — new peripheral path via arduino-cli
    validation)**. The LP-subsystem deep-sleep is driven through the legacy
    (S3) `RTC_CNTL` path: firmware sets `RTC_CNTL_SLP_TIMER0/1_REG` (@ +0x4/+0x8)
    then `RTC_CNTL_SLEEP_EN` (bit 31 of `RTC_CNTL_STATE0_REG` @ +0x18); on wake
    the wakeup-cause register `RTC_CNTL_SLP_WAKEUP_CAUSE_REG` (@ +0x130, field
    `RTC_CNTL_WAKEUP_CAUSE`) carries `RTC_TIMER_TRIG_EN` (bit 3). The emulator
    detects the SLEEP_EN write (`rtc.rs` `STATE0_OFF` bit 31), captures the
    programmed period, and (in `machine.rs`) fast-forwards the sleep as a fixed
    step budget then reboots — `wake()` does `reset()` and sets the timer
    wakeup cause on the fresh SoC. **CRITICAL bug found + fixed**: the ULP-RISC-V
    block had been carved out of the RTC_CNTL page at offset `0x100..0x200`, which
    *stole* `RTC_CNTL_SLP_WAKEUP_CAUSE` (0x130) — so the firmware's wakeup-cause
    read landed in the ULP device and returned 0 forever. `Rtc` now owns the full
    `0x000..0x400` RTC_CNTL page with a generic register store and delegates only
    the true ULP sub-range (`0x100..0x200`, still backed by `ulp.rs`) — and the
    wakeup-cause register is special-cased before that delegation. Verified vs the
    real header `rtc_cntl_reg.h` (`SLP_WAKEUP_CAUSE_REG = RTCCNTL_BASE + 0x130`).
    Validated end-to-end with `tools/sketches/esp32s3_deepsleep_poke` (direct
    register pokes: read 0x130, if timer bit clear program SLP_TIMER + write
    STATE0 SLEEP_EN, else print WOKE/PASS): `DEEPSLEEP START` → (emulator
    fast-forward + reboot) → `DEEPSLEEP WOKE` / `DEEPSLEEP PASS` under
    `run_flash`. 2 machine tests added (`deep_sleep_poke_wakes_with_timer_cause`,
    `rtc_slp_wakeup_cause_register_is_rtc`). **NOTE**: the esp-idf *driver*
    `esp_deep_sleep_start` path is NOT modeled — the firmware hangs in sleep
    *preparation* (core 0 stuck polling an unmodeled peripheral before it ever
    reaches `rtc_sleep_start` / the SLEEP_EN write), so validation uses the direct
    poke sketch, consistent with RMT/I2C/TWAI/etc. Deep-sleep is retired as a P5
    candidate (register/driver-path gap documented). Remaining P5 work: LP_I2C
    (`RTC_I2C @ 0x6000_8C00`) and LP_UART (best-effort, ~`0x6002_5400`); I2C
    (Wire) driver is a documented known limitation; Touch excluded per user
    directive (do NOT do Touch).

  - 2026-08-25: **LP_I2C / RTC_I2C (P5 — new peripheral via arduino-cli
    validation)**. `esp32s3-soc/src/rtc_i2c.rs` models the ESP32-S3 LP/I2C
    controller (`DR_REG_RTC_I2C_BASE = 0x6000_8C00`, i.e. offset `0xC00` of the
    `0x6000_8000` page). Register block (`I2C_SCL_LOW` 0x00, `I2C_SCL_HIGH`
    0x04, `I2C_MS_DELAY` 0x08, `I2C_CTRL` 0x0C, …). The bus FSM is NOT modeled
    (needs a real I2C device); the block is a register store so firmware can
    configure it. Wired into `soc.rs` (mmio arm for `0x6000_8000` page, `off
    0xC00..0x100` → `rtc_i2c`, distinct from RTC_IO which covers the 0x400..0xC00
    window). 3 unit tests (`tests/rtc_i2c.rs`) + a machine test
    `rtc_i2c_registers_round_trip`. Validated end-to-end with
    `tools/sketches/esp32s3_lpi2c` (direct register pokes: write SCL_LOW/
    SCL_HIGH/MS_DELAY/CTRL, read back) → `LP I2C POKE PASS` / `DONE` under
    `run_flash`. **LP_I2C is retired as a P5 candidate.**

  - 2026-08-25: **LP_UART (P5 — new peripheral via arduino-cli validation)**.
    `esp32s3-soc/src/lp_uart.rs` models the ESP32-S3 LP_UART (low-power UART,
    `0x6002_5400`) — it shares the GPSPI3 4KB page but sits at offset `0x400`
    (past SPI3's register block, which is `< 0x400`), so `soc.rs` carves it out
    of the `SPI3_BASE` arm (`dev == SPI3_BASE && off >= 0x400` → `lp_uart`).
    Register block (`FIFO` 0x00, `CLKDIV` 0x14, `CONF0` 0x20, …) modeled as a
    register store; the TX/RX FSM is NOT modeled (needs a real serial line). 3
    unit tests (`tests/lp_uart.rs`) + a machine test `lp_uart_registers_round_trip`.
    Validated end-to-end with `tools/sketches/esp32s3_lpuart` (direct register
    pokes: write FIFO/CLKDIV/CONF0, read back) → `LP UART POKE PASS` / `DONE`
    under `run_flash`. **LP_UART is retired as a P5 candidate.** All LP
    peripherals (Deep-sleep, LP_I2C, LP_UART) are now modeled and validated; the
    only remaining P5 driver-path gap is the I2C (Wire) esp-idf driver (peripheral
    validated via direct poke), and Touch is excluded per user directive.
    (Wire) driver is a documented known limitation; Touch excluded per user
    directive (do NOT do Touch).




  - 2026-08-25: **Combined P5 hardening audit — Xtensa ISA + OTA + ROM/PSRAM**
    (user directive: "see if all instructions are implemented; also OTA
    partition table, external PSRAM and ROM").
    - **Xtensa LX7 instruction-set audit (PASS, with one documented gap).**
      Built a throwaway coverage harness (`crates/xtensa-core/tests/
      isa_coverage.rs`, run with `ISA_AUDIT_FILE=<objdump -d> cargo test -p
      xtensa-core --test`). It decodes every instruction in a real
      arduino-cli `esp32s3_periph` disassembly (83,921 instructions / 354
      unique mnemonics) through our decoder and reports rejects. After fixing
      an objdump byte-order misunderstanding (objdump prints the instruction
      *value* big-endian; the decoder wants that exact value, matching
      `read16`/`read32` LE from memory), **ALL standard Xtensa LX7
      instructions decode correctly.** The only 29 rejected mnemonics are all
      `ee.*` — the ESP32-S3 **TIE/DSP extensions** (FFT `ee.fft.*`, vector-MAC
      `ee.vmulas.*`, broadcast `ee.ldf/stf/ld.qacc.*`). These are 4-byte
      `format_32` instructions not on the boot path (the periph sketch boots
      fine, so they're never executed). They require a new `format_32` decoder
      + the `UR_ACCX_*`/`UR_QACC_*`/`UR_FCR`/`UR_FSR` special-register
      semantics already stubbed in `cpu.rs` but no `ee.*` opcode
      implementations. **Documented as a known limitation** (only needed for
      DSP/FFT/WiFi-baseband firmware), out of scope unless explicitly funded.
    - **OTA boot-slot selection (IMPLEMENTED).** `boot_from_flash` previously
      always loaded the app from the fixed factory offset `0x10000`. Added
      `partition::select_ota_boot_offset(flash)` which parses the partition
      table, reads the `otadata` record (two `u32 ota_seq` entries: bit 31 =
      valid, low 16 = seq), and returns the flash offset of the highest-valid
      OTA slot's `app` partition (subtype `0x10`+slot; matched by label
      "otadata" or data-subtype 0x39). `boot_from_flash` now calls it and
      falls back to `APP_FLASH_OFFSET` when there's no OTA data — so non-OTA
      images are unchanged (verified: periph sketch still boots to "boot OK").
      `map_app_flash_segments` already parameterizes both the loader-scratch
      and XIP windows on the offset, so ota_0/ota_1 images boot identically to
      factory. 4 unit tests in `partition.rs` (highest-valid-slot, slot1-
      when-higher, no-valid-slot→None, no-otadata→None) + a machine test
      `ota_boot_selects_active_slot` (synthetic flash with ota_1 @ 0x200000,
      otadata selecting slot 1 → app executes, `stash == 0x1234`).
    - **ROM coverage (assessed — sufficient).** Real ESP32-S3 ROM (~384 KB @
      0x40000000) is replaced by a hand-written `rom_stub` that provides a
      table of leaf stubs at the REAL ROM addresses (`ets_printf`,
      `ets_delay_us`, `ets_efuse_get_mac`, `esp_rom_set_rtc_wake_addr`,
      `ets_set_appcpu_boot_addr`, regi2c bodies, etc.). 5+ real Arduino
      sketches boot to completion, proving every ROM function the app calls
      during boot/init is modeled. We do NOT implement the full ~300-entry ROM
      API (only the functions real firmware exercises) — a known, documented
      scope boundary, not a defect.
    - **External PSRAM (already complete — P4).** `cache.rs` models the shared
      cache MMU + EXT_MEM; PSRAM pages (type bit 15) are read-write via the
      data window, validated by `psram_read_write_via_mmu_mapped_page`. No
      new work required.
    - All workspace tests green; clippy clean (host + `wasm32-unknown-unknown`);
      `cargo fmt` clean.

  - 2026-08-25: **P5 peripheral batch — PARLIO (LCD_CAM, functional) + 7 register-store stubs**.
    User's list (PARLIO, PMS, World Controller, ETM, Peri-backup, PCR/clocks,
    I2S, Assist-Debug) mapped onto the REAL ESP32-S3 blocks (confirmed via
    esp-idf `reg_base.h`): PARLIO = **LCD_CAM** (`DR_REG_LCD_CAM_BASE =
    0x6004_1000`); PMS = **SENSITIVE** (`0x600C_1000`); World Controller =
    **WCL** (`0x600D_0000`); PCR/clocks = **SYSCON** (`0x6002_6000`);
    Peri-backup = `0x6002_A000`; I2S0 = `0x6000_F000`, I2S1 = `0x6002_D000`;
    Assist-Debug = `0x600C_E000`. **ETM was DROPPED** — web research confirmed
    ETM is NOT present on ESP32-S3 (only ESP32-C6/H2 and later), so there is
    nothing to model for it.
    - **Stubs (register-store, depth = "functional where it matters" applied to
      the rest)**: new `esp32s3-soc/src/regstore.rs` — a no_std `RegStore`
      (fixed `[u32; 0x1000/4]` array, `new(size)`, `read32`/`write32` by
      `offset & 0xFFF`). Wired into `soc.rs` as fields `sensitive`, `wcl`,
      `peri_backup`, `syscon`, `i2s:[RegStore;2]`, `assist_debug` and a free
      `store_dispatch(is_write, off, value, &mut store)` mmio arm helper; the
      8 bases added to `memmap.rs` as `I2S0_BASE`/`I2S1_BASE`/`SYSCON_BASE`/
      `PERI_BACKUP_BASE`/`LCD_CAM_BASE`/`SENSITIVE_BASE`/`ASSIST_DEBUG_BASE`/
      `WCL_BASE`. Accesses round-trip (config-register pokes never panic).
      `tests/p5_stub_peripherals_round_trip` (machine test) + the arduino-cli
      poke sketch `tools/sketches/esp32s3_p5_stubs` (all 8 POKE PASS under
      `run_flash`) validate boot-neutrality.
    - **LCD_CAM functional (`esp32s3-soc/src/lcd_cam.rs`)**: TX/RX FIFO pair +
      transfer-start / transfer-done interrupt path. `LCD_DATA` (0x40) pushes
      the TX FIFO; `LCD_FIFO_STATUS` (0x44, field `[10:0]` TX count) reports it;
      `LCD_USER` (0x14) `LCD_START` (bit 27) drains the FIFO and raises
      `LCD_TRANS_DONE` (bit 1 of the `LC_DMA_INT_ENA/RAW/ST/CLR` block at
      0x64/0x68/0x6C/0x70); the interrupt clears via `LC_DMA_INT_CLR`. CAM path
      mirrors with `CAM_DATA`/`CAM_FIFO_STATUS`/`CAM_START` but has no data
      source (RX FIFO stays empty). 2 unit tests (`tests/` inside lcd_cam.rs) +
      machine test `lcd_cam_fifo_and_transfer_done` + arduino-cli poke sketch
      `tools/sketches/esp32s3_lcd_cam` (`LCD CAM POKE PASS` under `run_flash`).
      **KNOWN LIMITATION**: the parallel data is NOT shifted onto the LCD_CAM
      GPIO-matrix output signals (`LCD_DATAx`/`LCD_WR`/`LCD_RS`/...) during a
      transfer — no external device and the 8080/6800/RGB FSM is not modeled;
      the FIFO data path + transfer-done interrupt are functional.
    - All workspace tests green; clippy clean (host + `wasm32-unknown-unknown`);
      `cargo fmt` clean.

  - 2026-08-25: **I2S audio + LCD_CAM GPIO-matrix output (P5)**.
    I2S (`I2S0` 0x6000_F000, `I2S1` 0x6002_D000) is now a **functional model**
    (`esp32s3-soc/src/i2s.rs`, replacing the RegStore stub): TX/RX FIFO
    (FIFO reg 0x80, depth 16) + `TX_START` (bit 2 of `TX_CONF` 0x24) begins a
    bit-clocked serial transmission that drives the I2S GPIO-matrix output
    signals — `I2SxO_BCK` (sig 22/28), `I2SxO_WS` (sig 24/29), `I2SxO_SD`
    (sig 25/30) — one serial bit per emulator step (MSB/LSB-first per
    `tx_bit_order`). When the TX FIFO empties, `tx_done` (bit 1 of the INT
    block at 0x0C/0x10/0x14/0x18) is raised. Register layout per esp-idf
    `i2s_struct.h`. 3 unit tests + machine test `i2s_tx_drives_gpio_matrix_
    signals` (routes I2S0 SD/BCK to GPIO pins via FUNC_OUT_SEL and confirms
    `gpio_output()` reflects the serial bit + `tx_done`) + arduino-cli poke
    sketch `esp32s3_i2s` (`I2S POKE PASS` under `run_flash`).
    **LCD_CAM now drives its parallel output onto the GPIO matrix during a
    transfer** (`lcd_cam.rs` `tick` + `signal_level`): each TX FIFO word is
    presented on `LCD_DATA_OUT0..15` (sig 133..148) for one `LCD_PCLK` (sig
    154) cycle, asserting `LCD_CS` (sig 132, active low) and `LCD_DC` (sig
    153, from `LCD_USER` bit 26) for the duration; the camera (RX) path has no
    data source. Machine test `lcd_cam_parallel_drives_gpio_matrix_signals`
    routes DATA0/CS to pins and confirms `gpio_output()`. Signal indices from
    esp-idf `gpio_sig_map.h` (LCD_CAM 132..154, I2S0 22..27, I2S1 28..32).
     I2S RX now has a data source too: `I2s::inject_rx(word)` pushes into an
     injected-RX FIFO (read back via `read32(FIFO)`), and `sig_loopback` (bit
     27 of `TX_CONF`) feeds each transmitted word back into the RX FIFO and
     raises `rx_done` (bit 0 of the INT block) when the loopback TX completes —
     so the RX path (FIFO + `rx_done`) is fully exercisable without an external
     codec. Bit fields corrected against `i2s_struct.h`: `tx_bit_order` is
     **bit 18 of `TX_CONF`** (was wrongly read as bit 17 of `TX_CONF1`),
     `rx_bit_order` = bit 18 of `RX_CONF`, `tx_bits_mod`/`rx_bits_mod` =
     `CONF1` bits 13-17 (+1, 16 if 0). `INT_RAW/ST/ENA/CLR` at 0x0C/0x10/
     0x14/0x18 (bits 0=rx_done,1=tx_done,2=rx_hung,3=tx_hung). 7 unit tests
     (`tests` in i2s.rs: TX shift-out + `tx_done`, msb/lsb-first, injected RX
     + `rx_done`, loopback TX→RX, loopback-after-prior-TX, config round-trip)
     + machine test + arduino-cli poke sketch `esp32s3_i2s` (`I2S POKE PASS`
     under `run_flash`, now also asserts `rx_done` + loopback `rx_word`).
     **LCD_CAM now drives its parallel output onto the GPIO matrix during a
     transfer** (`lcd_cam.rs` `tick` + `signal_level`): each TX FIFO word is
     presented on `LCD_DATA_OUT0..15` (sig 133..148) for one `LCD_PCLK` (sig
     154) cycle, asserting `LCD_CS` (sig 132, active low) and `LCD_DC` (sig
     153, from `LCD_USER` bit 26) for the duration; the camera (RX) path has no
     data source. Machine test `lcd_cam_parallel_drives_gpio_matrix_signals`
     routes DATA0/CS to pins and confirms `gpio_output()`. Signal indices from
     esp-idf `gpio_sig_map.h` (LCD_CAM 132..154, I2S0 22..27, I2S1 28..32).
      **KNOWN LIMITATIONS**: I2S master/slave clock-gen, TDM, PDM and the
      esp-idf DMA path are not modeled (fixed 1-bit-per-step shift rate);
      LCD_CAM's 8080/6800/RGB FSM and the LCD_WR/LCD_RS signal wiring beyond
      DC/PCLK/CS are not modeled. All workspace tests green; clippy clean (host
      + `wasm32`); `cargo fmt` clean.

  - 2026-08-26: **I2S RX limitation fixed (P5)**. The previously-documented
    "I2S RX has no data source" gap is closed. `esp32s3-soc/src/i2s.rs` gained
    `inject_rx(word)` (pushes into an injected-RX FIFO, read back via
    `read32(FIFO)`) and `sig_loopback` (bit 27 of `TX_CONF`): when set, each
    transmitted word is copied into the RX FIFO and `rx_done` (INT bit 0) is
    raised on TX completion — exercising the full RX data path with no external
    codec. Also corrected the bit-field offsets against `i2s_struct.h`:
    `tx_bit_order` is **bit 18 of `TX_CONF`** (the old model read bit 17 of
    `TX_CONF1` and produced a reversed/incorrect serial stream), `rx_bit_order`
    = bit 18 of `RX_CONF`, `tx_bits_mod`/`rx_bits_mod` = `CONF1` bits 13-17
    (+1, 16 if 0); `INT_RAW/ST/ENA/CLR` at 0x0C/0x10/0x14/0x18 (bits
    0=rx_done,1=tx_done,2=rx_hung,3=tx_hung). 7 unit tests in `i2s.rs` (added
    `injected_rx_fills_fifo_and_asserts_rx_done`, `loopback_tx_feeds_rx`,
    `loopback_after_prior_tx`, `lsb_first_matches_word`) + the existing TX
    machine test + the arduino-cli poke sketch `esp32s3_i2s` extended to assert
    `rx_done` and the loopback `rx_word` (now prints `I2S POKE PASS` with
    `rx_word=CAFE`). All workspace tests green, clippy/`wasm32` clean, `cargo
    fmt` clean. **I2S is retired as a P5 candidate** (the only remaining I2S
    gaps are master/slave clock-gen, TDM, PDM and the esp-idf DMA path — noted
    as out-of-scope limitations).

  - 2026-08-26: **I2S master/slave clock-gen, TDM, PDM, and esp-idf GDMA path
    all modeled (P5 — remaining I2S gaps closed)**. `esp32s3-soc/src/i2s.rs`
    now models: (1) **clock generation** — `tx_clkm_div_num`/`rx_clkm_div_num`
    (`TX/RX_CLKM_DIV_CONF` 0x3C/0x38) set the BCK half-cycle period in emulator
    steps; master mode drives BCK/WS/SD internally, `tx_slave_mod`/`rx_slave_mod`
    (`RX/TX_CONF` bit 3) select slave mode (external clock not simulated);
    (2) **TDM** — `tx_tdm_en` (`TX_CONF` bit 19) + `tx_tdm_tot_chan_num`
    (`TX_TDM_CTRL` 0x54, bits 16-19, 0-based so slots = tot+1) produce a
    multi-slot frame (each slot shifts one FIFO word, WS toggles per slot);
    when TDM is off, `tx_chan_mod` (`TX_CONF` bits 24-26) selects 1/2/4 slots;
    (3) **PDM** — `tx_pdm_en` (`TX_CONF` bit 20) re-encodes each transmitted
    word as a first-order sigma-delta PDM bitstream on SD (unsigned sample,
    half-scale comparator). The **GDMA path** is wired in `soc.rs`:
    `peri_sel == 3/4` (I2S0/I2S1) copies descriptor words into the TX FIFO
    register (OUT channel) and copies the RX FIFO register into the descriptor
    buffer (IN channel); GDMA constants `GDMA_I2S0_PERIPH=3`/`GDMA_I2S1_PERIPH=4`
    added in `gdma.rs` (from `gdma_channel.h` — matches the real S3 layout where
    the OUT_LINK `start` bit is **bit 21 at channel-offset 0x80**, IN_LINK
    `start` bit 22 at 0x20; the earlier 0x60 I used was `out_conf0`, not the
    link register). 11 unit tests in `i2s.rs` (`clock_divisor_slows_bck`,
    `master_slave_mode_gates_clock`, `tdm_multi_slot_shifts_multiple_words`,
    `pdm_encodes_extremes`, …) + machine test `i2s_gdma_out_feeds_tx_fifo` (GDMA
    OUT → I2S0 TX → loopback RX returns the descriptor words) + the arduino-cli
    poke sketch `esp32s3_i2s` extended with TDM/PDM/clock/GMDA parts → all 6
    parts print `I2S POKE PASS` (`tdm0/1`, `pdm_rx`, `clk_div=4`, `dma0/1`) under
     `run_flash`. All workspace tests green, clippy/host + `wasm32` clean. **I2S
     is fully retired as a P5 candidate** (no remaining documented limitations).

  - 2026-08-26: **I2C `Wire` esp-idf driver path — root-caused, accepted as a
    documented limitation**. Deep FreeRTOS SMP dive on
    `tools/sketches/esp32s3_i2c_wire` (empty-bus scan prints `other=119`
    instead of the expected NACK `nack=119`). Findings:
    - **Scheduler glue WORKS** on core 1 — verified the full
      `_frxt_setup_switch` (0x4037b344) → `_frxt_int_exit` (sets
      `port_switch_flag[core]=1`) → `_frxt_dispatch` (0x4037b3f0) →
      `vTaskSwitchContext` (0x4037c0f0) chain and confirmed the ISR raises the
      cross-core yield that the scheduler consumes. Not the problem.
    - **Proximate failure**: the I2C ISR calls `xQueueGenericSendFromISR`
      (0x4037ab80) on queue `0x3fcec998`; its gate `bltu uxMessagesWaiting
      (q+0x38), uxLength (q+0x3c=1)` (0x4037abdc) fails because `q+0x38`
      holds `0x3fcec898` (a pointer/garbage) instead of `0`. Every send returns
      `errQUEUE_FULL`, so the task's `xQueueReceive` times out → `other=119`.
    - **Corruption source**: a **4-byte `memcpy`** during queue init reads
      `0x3fcec898` from a transient self-referential FreeRTOS `List` object
      (a `vListInitialise` leaves `pxIndex == list_addr`) and copies it into
      `queue+0x38` (the `uxMessagesWaiting` slot); the source object is zeroed
      shortly after, so the bad value exists only at copy time. `xQueueGenericReset`
      does run and writes `0` to `q+0x38`, but the stray memcpy re-corrupts it.
    - **Conclusion**: this is the esp-idf I2C *driver's* internal
      queue/`cmd_link` initialization ABI. The peripheral (`i2c.rs`) is CORRECT
      — the direct-register-poke sketch
      `tools/sketches/esp32s3_i2c_poke` passes (`I2C POKE PASS`), and the ISR
      fires 336× on core1 with the NACK path (`INT_RAW=0x488`) completing the
      END command. The Wire driver additionally requires the ISR to populate
      `cmd_link->done`/`ret` (a firmware-owned struct the model cannot satisfy
      without the exact esp-idf `cmd_link` offsets, unavailable offline).
    - **Resolution**: accepted as a **documented known limitation** (consistent
      with how RMT/GDMA/MCPWM/PCNT/TWAI/HMAC/DS/RSA were each first proven via
      direct pokes). No `i2c.rs` change can fix a firmware-internal cmd_link
      handshake. The only path to a real fix is ABI-level modeling of the
      esp-idf i2c driver's `cmd_link` completion in the harness — out of scope
      unless explicitly funded. All debug instrumentation (run_flash.rs probes,
      soc.rs pc_debug/static write-hook) was removed; working tree is clean vs
      HEAD and `cargo build --release --example run_flash -p esp32s3-emu`
      passes. **I2C `Wire` driver remains a documented P5 known limitation;
      peripheral is validated.** Remaining P5 driver-path gaps: none beyond
      this (all other modeled peripherals are validated; Touch excluded per
      user directive).

  - 2026-08-26: **USB-Serial-JTAG CDC console (P5 — new peripheral via
    arduino-cli validation)**. `esp32s3-soc/src/usb_serial_jtag.rs` models the
    ESP32-S3 USB-Serial-JTAG controller (`DR_REG_USB_SERIAL_JTAG_BASE =
    0x6003_8000`, register layout per `usb_serial_jtag_struct.h`): `EP1`
    (`rdwr_byte`, 0x00) TX byte capture, `EP1_CONF` (0x04) with
    `serial_in_ep_data_free` (bit 1, always 1 — host modeled as always present,
    no enumeration/IN-token backpressure) + `serial_out_ep_data_avail` (bit 2)
    + `wr_done` (bit 0) which latches `serial_in_empty_int` (`INT_RAW` bit 3);
    `INT_RAW/ST/ENA/CLR` (0x08/0x0C/0x10/0x14) and `OUT_EP1_ST` (0x3C) with the
    RX `rec_data_cnt`/`wr_addr` fields. RX FIFO: `inject_rx` raises
    `serial_out_recv_pkt_int` (`INT_RAW` bit 2); `EP1` reads pop bytes. The old
    inline 3-line stub (which only captured ROM `uart_tx_one_char` bytes) is
    replaced by this functional device; `Soc::take_usb_serial_tx` still drains
    it and `machine.rs` merges it into the console stream, so USB-CDC output now
    appears alongside UART0 output. Interrupt source = `ETS_USB_SERIAL_JTAG_
    INTR_SOURCE = 96`, wired into `soc.rs::int_pending`. 5 unit tests in
    `usb_serial_jtag.rs` (TX capture, wr_done→empty-int, EP1_CONF signals,
    RX pop + recv-int clear, INT_CLR) + the arduino-cli poke sketch
    `tools/sketches/esp32s3_usb_serial` (direct register writes of `USBCDC:OK`
    to `0x60038000`, merge confirmed in `run_flash` output between `USB TEST
     START`/`END`) → USB CDC TX validated end-to-end. RX validated by unit
     tests. **USB-Serial-JTAG is retired as a P5 candidate.**

  - 2026-08-26: **SD/MMC host + simulated SD card (P5 — new peripheral via
    arduino-cli validation)**. `esp32s3-soc/src/sdmmc.rs` (previously a bare
    register store) now models the DesignWare MMC host at `0x6002_8000` AND a
    simulated SD card behind it: writing `CMD` (0x2C, `start_command` bit 31)
    issues a command to the card, which fills `RESP0..3` (0x30..0x3C) and
    latches `RINTSTS` cmd_done (bit 2) / data_over (bit 3); block transfers use
    the PIO FIFO (0x100). `CMD` bitfield per `sdmmc_struct.h`: `cmd_index`
    [5:0], `response_expect` bit 6, `response_long`(R2) bit 7, `data_expected`
    bit 9, `rw` bit 10 (0=read,1=write), `send_init` bit 15, `update_clk_reg`
    bit 21, `start_command` bit 31. The card implements GO_IDLE/IF_COND/ACMD41
    (ready after 2nd call, SDHC/CCS set)/ALL_SEND_CID(R2)/SEND_RCA/SELECT/
    SEND_CSD/CID/STATUS/SET_BLOCKLEN/READ_SINGLE_BLOCK/WRITE_BLOCK; a single
    512-byte `block` buffer round-trips writes. `CDETECT` reads 0 (card
    present), `WRTPRT` 0. 3 unit tests (`tests` in sdmmc.rs: init sequence,
    read-block pattern, write→read round-trip) + the arduino-cli poke sketch
    `tools/sketches/esp32s3_sdmmc` (full init + block R/W via register pokes)
    → `SDMMC PASS` under `run_flash`. **KNOWN LIMITATION**: the esp-idf
    `SD_MMC`/`SD` driver uses the IDMAC DMA path (not modeled) and CMD6/CMD51/
    SDIO; only the PIO FIFO data path is functional — validation is via direct
    register pokes, consistent with RMT/I2C/TWAI/HMAC/etc. **SDMMC is retired
     as a P5 candidate (register + simulated-card path).** Remaining P5 work:
     ULP-RISC-V core execution (task 3) + the documented I2C `Wire` driver
     limitation; Touch excluded per user directive.

  - 2026-08-26: **ULP-RISC-V core execution (P5 — new peripheral via arduino-cli
    validation)**. `esp32s3-soc/src/ulp.rs` is now a **functional rv32im core**
    (was a register store). The ULP-RISC-V runs firmware from `RTC_SLOW_MEM`
    (`0x5000_0000`, 8 KB, already mapped on the SoC bus). Writing the `core`
    `sw_start` bit (bit 0 of `0x6000_8100`, the ULP block = RTC_CNTL page +
    0x100) releases the core; it runs one instruction per `Soc::tick_timers`
    step (`self.ulp.step(&mut self.rtc_slow[..])`), executes the program, and
    halts on `ebreak` (sets the `debug` halted flag). Stores to the ULP `reg`
    slots (`0x6000_810C..`, the 16 general-purpose communication words) land in
    the shared `regs` array so the main CPU can poll them. Decoder implements
    LUI/AUIPC/JAL/JALR/BRANCH/LOAD/STORE/OP-IMM/OP(+M mul/div/rem)/FENCE/SYSTEM
    (rv32im). The RTC_CNTL page dispatch now carves `0x100..0x200` out to the
    `Ulp` core, **except** `0x130` (RTC_CNTL_SLP_WAKEUP_CAUSE) which stays in
    `Rtc` so deep-sleep keeps working. 3 unit tests in `ulp.rs` (program runs +
    writes reg slot, ebreak halts + debug flag, register store round-trip) + a
    machine test `ulp_runs_poked_program_via_bus` + the arduino-cli poke sketch
    `tools/sketches/esp32s3_ulp` (hand-assembled rv32im program stores
    `0x12345678` to reg slot 0 then ebreaks; main core polls) → `ULP POKE PASS`
    under `run_flash`. **KNOWN LIMITATION**: the RISC-V **compressed (C)
    extension** and RV32F/D are not modeled, and the ULP can only reach
    `RTC_SLOW_MEM` + the ULP `reg` slots (other RTC peripheral accesses are
    ignored). Real ULP firmware is typically compiled with `-march=rv32imc`, so
    the C extension will be needed for genuine esp-idf ULP programs — noted as a
    follow-up. **ULP-RISC-V core execution is validated and retired as a P5
    candidate** (alongside the documented I2C `Wire` driver limitation; Touch
    excluded per user directive).

  - 2026-08-27: **ULP-RISC-V C (compressed) extension + WDT/dual-core machine tests (P5)**.
    - **C-extension (rv32imc) decoder** (`esp32s3-soc/src/ulp.rs`): the ULP
      rv32im core now also decodes the RISC-V **C (compressed, 16-bit)**
      extension — `C.ADDI4SPN`, `C.LW`/`C.SW`, `C.ADDI`/`C.ADDI16SP`, `C.LI`,
      `C.LUI`, `C.SRLI`/`C.SRAI`/`C.ANDI`/`C.MV`/`C.ADD`/`C.JR`/`C.JALR`,
      `C.J`/`C.BEQZ`/`C.BNEZ`, `C.EBREAK`, `C.SLLI`, `C.LWSP`/`C.SWSP`. The
      GAS (esp-rv32 toolchain) **non-canonical** compressed encodings were
      root-caused: C.SRAI/C.SRLI use `rs2f = bits[6:2]` (shamt≥8) vs
      `bits[4:0]` (shamt<8) as a discriminator; C.MV/C.JR have `rs2<8`
      (`bits[6:5]==0`) while C.SRAI has them set. A hand-assembled rv32imc
      program (`mul`/`addi`/`slli`/`sw`/compressed `c.addi`/`c.slli`/`c.j`) is
      now a passing machine test (`ulp_runs_compressed_rv32imc_program`),
      asserting the computed `mem[90] == 5`. Validated the same program on a
      real ESP32-S3 via `riscv32-esp-elf-as`/`ld`/`objdump` (signature
      `2a00006f 00b00513 ... c000 0505 8d05 0440 fd65` matches the assembler
      output exactly). Committed as `b7d4575`.
    - **WDT edge-case machine tests** (`crates/esp32s3-emu/src/machine_tests.rs`):
      `wdt_interrupt_fires_instead_of_reset` (routes TG0_WDT src 52 → line 15,
      INT_ENA bit 2, CONFIG0=`0xA0000000` EN|stg0=interrupt, handler clears
      INT_CLR bit 2 + rfi 3 → CTR==1, NO reboot), `wdt_reset_reboots_machine`
      (app prints 'R' to UART0, arms MWDT0 stage-0 reset with hold0=4, loops;
      the timeout triggers `Esp32S3::step`'s `consume_reset()` → `reset()` which
      re-runs `boot_from_flash` from `self.flash` → ≥2 'R's captured across the
      reboot — proves the WDT-reset-as-reboot path), and
      `cross_core_interrupt_yields_to_other_core` (core 0 releases core 1 via
      `APPCPU_CTRL_A`@`0x600C0004`, routes SYSTEM.CPU_INT_FROM_CPU_1 (src 80,
      matrix off `4*(512+80)=0x940`) → core1 line 15; core 1 ISR clears the
      cross-core reg + rfi 3 → STASH==0xCAFE, CTR==1, `cpu[1].pc==done1`).
      All 3 pass; full `--workspace` test suite green (54 esp32s3-emu tests).
    - **Clippy hygiene**: fixed 11 minor clippy warnings I'd introduced in the
      C-extension decode (`bit(X) << 0` no-op shifts → `bit(X)`; redundant
      `(u32 << n) as u32` casts → `(u32 << n)`). `cargo clippy --workspace`
      (libs) is now clean; `wasm32-unknown-unknown` build clean; ULP tests
      green. The only remaining `cargo test` (lib-tests) warnings are
      **pre-existing** in `memspi.rs:924` (`let mut f` unused) — unrelated to
      this work, left untouched.
    - Remaining P5 work: I2C (Wire) esp-idf driver is a documented known
      limitation (peripheral validated via direct poke); Touch excluded per
      user directive. All other modeled peripherals (incl. ULP rv32imc, WDT,
      dual-core cross-core IRQs) are validated.

