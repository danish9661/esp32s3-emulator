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
       FreeRTOS/Arduino behavior. NEXT: keep implementing missing peripherals
       (RMT ✓, PCNT ✓, TWAI/CAN ✓, MCPWM, Touch, …) and validate each with a
       sketch. NOTE: the esp-idf RMT *driver* uses GDMA (unmodeled) — RMT was
       validated via direct register pokes; modeling GDMA would also enable the
       driver path.
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
  S3 routes through **GDMA — UNMODELED** → would hang waiting for a DMA
  tx_end) and polls `INT_RAW`; runs under `run_flash` → prints `RMT TX done`.
  113 workspace tests green, clippy/wasm32 clean. NEXT peripheral candidates:
  TWAI/CAN, PCNT, MCPWM, Touch; and/or model GDMA so the esp-idf RMT driver
  path works too.
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
