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
| qemu-system-xtensa | — | MISSING → install espressif prebuilt later (golden validation) |
| ESP-IDF | — | MISSING → needed later to build real firmware |
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
└── tools/               QEMU golden-trace scripts, test firmware build scripts
```

Core design:
- `xtensa-core` is **SoC-agnostic**: CPU talks to memory via a `Bus` trait
  (`read8/16/32`, `write8/16/32`). SoC implements `Bus`.
- Step-based execution: `Cpu::step(&mut bus)` → one instruction, returns
  `StepResult` (halted / exception / normal). JS drives N steps per frame.
- Windowed register file implemented with a 64-entry AR array + WINDOWBASE/
  WINDOWSTART/PS.WINDOW emulation.

## Roadmap (status updated as we go)

- [ ] **P0 — Skeleton**: workspace + crates compile, wasm-pack builds, browser
      loads module. (in progress)
- [ ] **P1 — Core CPU**: decode + ALU + load/store + branches + CALL0/J loops;
      runs hand-assembled bare-metal test programs.
- [ ] **P2 — SoC basics**: memory map, UART0 (console out), GPIO, timers,
      interrupt controller; trivial IDF firmware prints via UART.
- [ ] **P3 — Boot path**: flash image loading, ROM stubs (printf/UART/delay),
      second-stage bootloader, partition table → real IDF app boots.
- [ ] **P4 — Peripherals**: SPI/I2C/PWM/ADC, dual-core, PSRAM.
- [ ] **P5 — Hardening**: golden-trace validation vs QEMU (PC/register diffs),
      exception correctness, interrupt timing.
- [ ] **P6 — Frontend polish**: serial console UI, GPIO/LED visualization,
      example firmware gallery.
- [ ] WiFi/BLE: OUT OF SCOPE for now (months of work; not required for the
      core milestone).

## Validation strategy

1. **Unit tests** in each crate (instruction-level, known-answer tests).
2. **Hand-written assembly tests** — assemble with xtensa-esp32s3 toolchain,
   run in our emulator, assert register/memory results.
3. **Golden traces vs QEMU**: run same firmware in QEMU with
   `-d in_asm,cpu` / gdb, record PC+reg trace, diff against ours.
4. **Real firmware**: ESP-IDF hello_world over UART as the first real target.

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
  QEMU = only reference). AGENTS.md created. Workspace scaffold next.