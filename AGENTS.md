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
  QEMU = only reference). AGENTS.md created. Workspace scaffold next.