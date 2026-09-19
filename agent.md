# agent.md — Handover / Working Notes (WiFi bring-up)

Last updated: 2026-09-19 (session 7: TXDC bit-1-edge arm fix; steady-state
  park PROVEN as txdc_cal_v70 BNONE poll on SENS2 bit24 — part-2 uncommitted).
Keep this file current: append session log entries, update status + next
steps on every handover.

## 1. Project snapshot

- From-scratch ESP32-S3 emulator in Rust → WASM, runs in browser. See
  `AGENTS.md` (authoritative: mission, architecture, conventions, status log).
- HEAD: `312d903` "feat: WiFi RF bring-up part 1 (PHY spins fixed, scan task
  still parked)" (2026-09-17). Plus `1a6cdb0` vendoring `tools/gateway`
  (Go SLIRP/NAT + multiplayer gateway, port **5050**, for WiFi backhaul).
- P5/P6 done per AGENTS.md: battery **106/0/0**, gallery **37 entries**,
  Playwright E2E ALL PASS. WiFi/BLE were historically OUT OF SCOPE —
  **WiFi bring-up is now the active workstream** (this file tracks it).

## 2. WiFi workstream status

- Goal: real Arduino `WiFi.scanNetworks()` returns results in-emulator;
  longer-term backhaul via `tools/gateway` (`ws://127.0.0.1:5050`).
- Part 1 LANDED (HEAD commit): `crates/esp32s3-soc/src/wifi.rs` scaffold —
  FE/FE2/BB/NRX/MAC/MAC-CTRL pages as plain stores + objdump-proven
  done-bits (FE `+0x174` bits 16+24, MAC-CTRL `+0xD14` bit 0 once armed,
  MAC `+0x8C` field `0x7F`; SENS2 `+0x4C` bit 24 in the soc SENS2 arm,
  `+0x50 = 0x07000000`). Gets closed PHY ROM through RF-cal
  (`ram_iq_est_enable`, `txdc_cal_v70`) and driver through `hal_init`
  into `esp_wifi_start`.
- Part 2 STAGED (uncommitted, sessions 6–7 — commit as "WiFi RF bring-up
  part 2"): `Wifi::txdc_write/txdc_read` + `tick` (SENS2 `+0x4C` bit 24
  now timed 48000 cycles, armed by FIRST write with bit 1 set); WDEV
  TSF/timer page split from RNG (`WDEV_BASE`, RNG arm routes only `+0x7C`
  to `Rng`); atomic `Bus::cas32` + `Soc::cas32` fast paths + `s32c1i`
  routed through it. See session-6/7 logs for the forensics behind each.
- Current behavior (reproduced 2026-09-19, STEPS=60M): UART shows only
  `WIFI SCAN START`. The steady-state park is PROVEN (session 7,
  artifact-free executed-insn histogram on core0): the ROM `ets_delay_us`
  CCOUNT loop (`0x40041a76`, entered via `wifi_station_start` ←
  `_do_wifi_start` ← `wifi_mode_set` ← `esp_wifi_start`) PLUS the
  `txdc_cal_v70` BNONE poll (`0x420819fd`, `l32i.n`+`bnone` on SENS2
  bit 24, 15429 hits / 0 exits). Earlier "89% in delay loop" region
  traces were block-sampling artifacts (mid-instruction pcs decode as
  `EE_UNIMPLEMENTED`); the executed-insn histogram (START-pc
  attribution) shows the delay loop at 68% + txdc poll at ~1% + IdleHook
  on core1 — both spins live, the txdc one is the load-bearing park.
  Next: fix the TXDC arming (below), then trace past it into `scan_start`
  (0x4206ce18) → `esp_wifi_scan_start` (0x42063b18) → SCAN_DONE event →
  `_scanDone` → results path.

## 3. Key files

| Path | Role |
|---|---|
| `crates/esp32s3-soc/src/wifi.rs` | WiFi radio scaffold (done-bits live here) |
| `crates/esp32s3-soc/src/soc.rs` | MMIO dispatch: WiFi arms (~line 2949), SENS2 arm (~2575), `tick_timers` (~1279), `scan_peripheral_sources` (~3551, systimer src 57/58/59), `int_pending` (~3685) |
| `crates/esp32s3-soc/src/systimer.rs` | SYSTIMER model: period-mode tick (TARGET0 PRO / TARGET1 APP on unit 1, TARGET2 esp_timer oneshot unit 0) |
| `crates/esp32s3-soc/src/memmap.rs` | `FE_BASE` 0x60006000, `FE2_BASE` 0x60005000, `BB_BASE` 0x6001D000, `NRX_BASE` 0x6001CC00, `WIFI_MAC_BASE` 0x6001C000, `WIFI_MAC_CTRL_BASE` 0x60033000 |
| `crates/esp32s3-emu/examples/run_flash.rs` | Harness: STEPS budget, UART drains, IDLE/STUCK detection, env fixtures (see §5) |
| `crates/esp32s3-emu/examples/probe.rs` | Scratch single-step probe pattern (delete scratch examples before commit) |
| `crates/esp32s3-emu/examples/wifi*.rs` | TEMP probes (delete before commit): wifidis (static OUR-decoder walk), wifideep (pc-range logger), wifitrace (call-site CALL/RET), wifisub/wififlag/wifipark/wifiprobe (older watches); `crates/xtensa-core/examples/decword.rs` (single-word decode) |
| `crates/esp32s3-emu/src/machine.rs` | `step_fast` (1 global tick per 2 ops), `step`, sleep/reset plumbing |
| `tools/sketches/esp32s3_wifi_scan/` | `.ino` + `build/*.elf` + committed `.merged.bin` |
| `tools/gateway/` | Vendored Go gateway, port 5050 (NAT backhaul, future) |
| `tools/run_battery.sh` | 106-case battery; wifi_scan NOT yet an entry (park unresolved — add it once SCAN DONE prints) |

## 4b. ESP32-S3 wireless protocols (all S3-supported; ground truth =
`~/esp-idf-v5.5.4/components/soc/esp32s3/include/soc/soc_caps.h` +
`esp_wifi/include/esp_wifi_types_generic.h` + Arduino `dio_qspi`
`sdkconfig.h`; S3 = 2.4 GHz ONLY, no 5G, no 802.15.4/Zigbee/Thread)

WiFi (closed `libpp.a`/`libnet80211.a`; open API `esp_wifi.h`):
modes STA/AP/APSTA (+NAN enum exists, unsupported on S3 — no
`SOC_WIFI_NAN_SUPPORT` cap); ifs STA/AP; protocols 11B/11G/11N/LR
(default bitmap 11B|11G|11N; 11A/11AC/11AX enum values exist but are
5G-gated — S3 has no `SOC_WIFI_HE_SUPPORT`/`SOC_WIFI_SUPPORT_5G`;
note: `ghz_2g` field doc DOES list 11ax as valid on 2.4G — enum-valid
but unset by default; S3 has no HE cap so treat 11AX as unsupported);
bandwidth HT20/HT40; auth OPEN/WEP/WPA_PSK/WPA2_PSK/WPA_WPA2_PSK/
Enterprise/WPA3_PSK/WPA2_WPA3_PSK/WAPI_PSK/OWE/WPA3_ENT_192/DPP/
WPA3-Enterprise variants; ciphers NONE/WEP40/WEP104/TKIP/CCMP/
TKIP_CCMP/AES_CMAC128/SMS4/GCMP/GCMP256/AES_GMAC128/AES_GMAC256;
power-save NONE/MIN_MODEM/MAX_MODEM; caps FTM (init+resp, enabled in
sdkconfig), GCMP/GMAC, WAPI (`libwapi.a`), TXOP, CSI (enabled), MESH
(`libmesh.a`, `SOC_WIFI_MESH_SUPPORT`), variable-beacon-window,
USB-PHY workaround; scan active/passive (`esp_wifi_scan_start` +
`SCAN_DONE` event → `_scanDone` → records); SoftAP (beacon/broadcast);
features ESP-NOW (`libespnow.a`), SmartConfig (`libsmartconfig.a`),
WPA3-SAE/OWE-STA/Enterprise, AMPDU TX/RX, NVS cal storage, HW TSF.
BT: BLE-only (NimBLE stack, controller enabled; roles central/
peripheral/broadcaster/observer; GATT client+server; SMP legacy+SC;
LE encryption/privacy/whitelist/BLE-mesh/BLE-5.0/BLUFI; coex enabled,
combo WiFi+BLE module) — NO Classic/BR-EDR/A2DP/SPP (no cap, no
Bluedroid — `SOC_BT_SUPPORTED` is the LE controller only).
No 802.15.4 radio on S3 (external-coex lines only) — Zigbee/Thread are
OpenThread-SPINE L-only sdkconfig stubs, out of scope with WiFi/BLE.

## 4. Commands

```bash
cargo build --release -p esp32s3-emu --example run_flash
STEPS=60000000 ./target/release/examples/run_flash tools/sketches/esp32s3_wifi_scan/esp32s3_wifi_scan.merged.bin
tools/run_battery.sh --build wifi_scan        # once a battery entry exists
cargo test --workspace                         # 39 suites expected green
cargo clippy --workspace --all-targets -- -D warnings   # must be 0 warnings
cargo fmt && cargo build --target wasm32-unknown-unknown --workspace
export PATH="$HOME/.arduino15/packages/esp32/tools/esp-x32/2601/bin:$PATH"  # objdump/nm
```

## 5. run_flash env fixtures (relevant)

`STEPS` (insn budget, default 96M), `IDLE_STEPS`, `UART0_INJECT` /
`UART0_MARKER`, `USB_INJECT`, `PANIC_CONTINUE`, `SYSCALL_CONTINUE`,
`FLASHENC_KEY`, `SECURE_BOOT_EN`, `TOUCH_INJECT`, `ADC_INJECT_MV`.
Feature `wifi-trace` (`esp32s3-soc/wifi-trace`) enables `WIFI_TRACE R/W`
unmapped-APB logging in soc.rs — **TEMP, remove before commit**.

## 6. Process gotchas (from AGENTS.md history — obey)

- Committed `.merged.bin` files are cache; `--build` is truth. "Fails
  identically with/without my change" ≠ not-rot: always `--build` the
  failing case before declaring pre-existing rot.
- objdump linear sweep desyncs on dense Xtensa — our own decoder is ground
  truth for live code; objdump OK for static function lookup.
- Rebuild run_flash after soc edits (stale harness mimics model failure).
- `crates/*` are `#![no_std]` (alloc ok); register comments REQUIRED with
  TRM citations; cite ISA RM format per instruction; `cargo fmt`,
  clippy-clean; delete scratch examples/probes before commit.
- Never invent HW timing — done-bits only with objdump evidence + arming
  rule; document approximations in-code.

## 7. Open questions

- What posts the scan command (which queue/event unparks the scan task)?
- MAC RX DMA register layout (no public headers — derive from driver ELF
  disassembly like part 1).
- Scan-done event path (interrupt source? task notification?).

## 8b. FreeRTOS struct offsets (DERIVED from local source — use these!)

Source: `~/esp-idf-v5.5.4/components/freertos/FreeRTOS-Kernel-SMP/`
(`tasks.c:362`, `queue.c:109`, `list.h:148/176`), sketch sdkconfig
(`CONFIG_FREERTOS_MAX_TASK_NAME_LEN=16`, `NUMBER_OF_CORES=2`,
`USE_TRACE_FACILITY=y`). All u32 words, Xtensa LE:

```text
ListItem_t (20B): +0 xItemValue, +4 pxNext, +8 pxPrevious, +12 pvOwner,
  +16 pxContainer. List_t (20B): +0 uxNumberOfItems, +4 pxIndex,
  +8 xListEnd(value=-1), +12 end.pxNext, +16 end.pxPrevious.
  (MiniListItem == ListItem_t here; no integrity bytes. Earlier "cursor"
  confusion resolved: pxIndex is just a cursor; walk from xListEnd.pxNext.)
TCB (PROVEN LIVE 2026-09-18, wifi cur0 dump — affinity word ABSENT):
  +0 pxTopOfStack, +4 xStateListItem (20B: +4..+23),
  +24 xEventListItem (20B: +24..+43), +44 uxPriority, +48 pxStack,
  +52 pcTaskName[16] (+52..+67), +68 xTaskRunState, +72 uxTaskAttributes,
  +76 pxEndOfStack?, +80 uxCriticalNesting?, +84 uxTCBNumber,
  +88 uxTaskNumber?, +92 uxBasePriority?, +96 uxMutexesHeld?, ...
  Evidence: wifi cur0 = +4:v=619, +8/+12:0x3fc9bac8 (pxReadyTasksLists
  prio23), +16:SELF(owner), +20:0x3fc9bac0(container); event +24:v=2,
  +28/+32:0x3fcaa18c(heap queue), +36:SELF(owner), +40:0(container NULL =
  wifi NOT event-blocked at hang); name "wifi" at +52; prio 0x17=23 at
  +44; stack 0x3fcaa890 at +48. (SMP affinity mask NOT present despite
  configUSE_CORE_AFFINITY=1 — the running-core field is xTaskRunState
  only. The +4-shift vs source means one u32 (affinity? MPU?) is compiled
  out in this build; TRUST THE LIVE DUMP, not the struct arithmetic.)
  Notification anchor: timer ISR does l32i [a2,0x158] — with this layout
  +0x158 is far past the first 112B; ulNotifiedValue NOT yet located.
  It does not matter for the park: the wifi task is READY (on the prio23
  ready list), not notification-blocked. If ever needed, find it by the
  incrementing-word scan across two timer-ISR hits.
Queue_t: +0 pcHead, +4 pcWriteTo, +8 pcTail/union.u.xSemaphore.xMutexHolder,
  +12 pcReadFrom/uxRecursiveCallCount, +16 xTasksWaitingToSend (List 20B),
  +36 xTasksWaitingToReceive (List 20B), +56 uxMessagesWaiting,
  +60 uxLength, +64 uxItemSize, +68/+69 cRxLock/cTxLock (int8),
  +70 ucStaticallyAllocated?, +72 uxQueueNumber?, +73 ucQueueType? (trace on).
  (union: queue pcTail/pcReadFrom OVERLAP mutex holder/recursive-count —
  check pcHead==NULL for mutex vs queue.)
```

Corrections to earlier notes (SUPERSEDED 2026-09-18 by live dump):
name is at **+52**, prio at **+44**, state item at **+4**, event item at
**+24** — the §8b source-arithmetic (+64/+48/+8/+28) was WRONG for this
build (one u32 compiled out). The session-2/3 findings that used
+52-name/+44-prio/+4-state/+24-event (suspended-list container 0x3fc9b870,
event container 0x3fcac4ac, prio23 wifi CURRENT) STAND as measured.
The pxContainer field of the state item = +4+16 = +20; event = +24+16 =
+40 (matches the NULL-container observation for wifi).

## 8. Session log (newest last)

- 2026-09-19 (session 7 — TXDC STALL NAILED + FIXED; steady-state park =
  `txdc_cal_v70` BNONE poll, not the delay loop):
  - Artifact-free histogram (per-core, START-pc attribution via
    `cpu.step()` + pre-step pc — kills the mid-instruction `EE_*
    misdecode` class): core0 = delay loop 68% (`0x40041a76/79/7c`,
    `RSR_CCOUNT/SUB/BLTU`) + `txdc_cal_v70` BNONE poll `0x420819fd`
    15429 hits / 0.3% (`L32I_N`+`MEMW`+`BNONE` triple at
    `0x420819fb/f8/fd`); core1 = IdleHook 55% + IDLE/`waiti`. The
    `0x420819fb/fd` "L32I_N/BNONE" pcs from the OLD sampler were
    mid-instruction artifacts of the `L32R` at `0x420819fb` — confirmed
    by walking OUR decoder from the `ENTRY` at `0x420819f8`.
  - TXDC stall root cause (proven live): the poll loop RMWs SENS2 every
    ~620 steps (`...f1`/`...f3` alternation in the sens2 trace =
    continuous RMW writes), and the old `txdc_write` level arm
    (`value != 0`) reset the 48k timer on EVERY poll RMW. Under
    `step_fast` (1 tick per 2 insns) the timer can never elapse between
    two same-block iterations → sens2 frozen (`0x00113cf3`, bit24=0),
    15429 bnone hits, zero exits. Single-stepping core0 with NO ticks
    never sets bit24 (timer correctly needs ticks) — the arm, not the
    clock, was wrong.
  - Fix (in tree, validated): `txdc_write` arms ONLY on the FIRST write
    with bit 1 set (`value & 0x2 != 0 && txdc_until == 0`) — bit 1 is the
    driver's arming bit (pre-cal RMW at `0x420819ca` writes
    `0x00113cf1` with bit 1 SET; the poll RMW writes back the polled
    value with bit 1 CLEAR — objdump + live sens2 trace agree). 574/574
    tests green, clippy/fmt/wasm32 clean, battery 6/6 subset green.
  - TXDC still parked after the fix (same hang pcs): the sens2 trace
    shows the ...f1/...f3 RMW alternation CONTINUING (the poll loop
    still runs) — the bit-1 arm fires on the pre-cal RMW but bit24 still
    reads 0 at 60M. Open: whether the 48k timer elapses under the real
    `step_fast` interleaving (to verify with the txdc probe), or the
    poll needs more (sibling poll? second arming bit?). NEXT: re-run the
    txdc probe against the fixed arm; if bit24 sets but the poll still
    spins, decode the poll exit condition fully (BNONE mask vs bit24).
  - Cautionary tales: (a) NEVER trust block-sampled pcs for decode —
    always attribute executed insns by START-pc (`cpu.step()` +
    pre-step pc); mid-instruction samples decode as phantom `ee.*`
    words; (b) `xQueueReceive` a4-at-ENTRY is the ONLY valid ticks read
    — post-entry a4 is mutex scratch; (c) objdump desyncs on closed
    `libpp.a` regions — OUR decoder + live single-step is ground truth.

- 2026-09-19 (session 6 — PARK CHAIN SOLVED, part-2 staged; NO model bug):
  - Faithful single-step forensics (`m.step()` both cores, pre-step pc so
    windowed args are caller-valid) REPLACED every session-5 conclusion:
    loopTask runs `setup()` → `esp_wifi_init` (core1 @32295120) →
    `wifi_create_queue` → ppTask ENTERs on core0 (@32443184, waits
    `xQueueReceive q=0x3fcaa160 ticks=INF` = healthy idle) →
    `ieee80211_ioctl` ×5 → `pp_post(q=0x3fcaa824)` + `xQueueGenericSend
    →WIFIQ` + `xcore_send(core0,reason0)` yields (all live-logged). The
    §8-session-5 "wifi task never runs / loopTask never posts" chain is
    SUPERSEDED — it was a block-sampling artifact (post-ENTRY sampling
    reads the rotated window; single-step pre-ENTRY reads are correct).
  - The 60M-hang pcs are a TIMED wait, not a deadlock: core0
    `0x4037ce77` = `xPortEnterCriticalTimeout` CAS-retry (proven live:
    single-step CAS trace shows the lock word cycling owner↔FREE at
    full speed — contention, not corruption) and core1 `0x420965b4` =
    `esp_vApplicationIdleHook`. Region trace: core0 settles 89% in ROM
    `0x40041000` = `ets_delay_us` CCOUNT loop (`rsr.ccount; sub; bltu`
    at `0x40041a76`, tight-loop proven by 256-pc window ≤64 B), entered
    from `wifi_station_start` (@43798434) via `_do_wifi_start` ←
    `wifi_mode_set` ← `esp_wifi_start` (single-stepped through
    `wpa_attach`/`wpa_sm_init`/heap-calloc with ROM-loop skips).
  - CCOUNT semantics verified, NOT the bug: ROM loop `delay = us *
    g_cpu_ticks_per_us` (`ets_get_cpu_frequency` → 240 → `mull`,
    objdump-verified), CCOUNT +1/insn (cpu.rs:649). A 200 µs-class wait
    needs ~100M insns; the 60M budget simply expires mid-wait. The
    uncommitted `cas32` change is behavior-preserving here (default
    trait impl = same read-compare-write; `Soc::cas32` fast paths only
    remove the interleaving the serialized `step()` never produces —
    proven by the lock-word trace: no dual-ownership in 5956
    transitions). Keep it anyway (QEMU-parity + documents the hazard).
  - Part-2 diff (staged, all validated): txdc 48k-cycle timer
    (`Wifi::tick/txdc_write/txdc_read`, soc SENS2 arm + `tick_timers`
    hook); WDEV split (`WDEV_BASE`, RNG full-page mask, RNG arm routes
    only `+0x7C`); `Bus::cas32` + `Soc::cas32` + `s32c1i` via `cas32`.
    Probes removed (21 `wifi*.rs` TEMP examples deleted); TEMP
    `wifi-trace` feature deleted from both Cargo.tomls + the soc
    catch-all arm (footprint fully captured: only `0x60009168/9160/
    11098/12090/34xxx` config RMWs, all round-trip).
  - Validation at stage: 574/574 tests green (39 suites), clippy
    `-D warnings` clean, `cargo fmt --check` clean, wasm32 clean,
    battery hello/periph/rng/systimer/hello_opi 5/5 green, wifi_scan
    reproduces the known SCAN-START-only hang (unchanged — the wait was
    always timed, not stuck).
  - NEXT: (1) commit part-2 (this diff + agent.md); (2) run wifi_scan
    with a bigger budget (200M+) and trace past the delay into
    `scan_start` (0x4206ce18) → `esp_wifi_scan_start` (0x42063b18) →
    SCAN_DONE → `_scanDone` → `WIFI SCAN found N`; (3) model whatever
    the scan path polls (MAC RX DMA / scan-done event); (4) add the
    `wifi_scan` battery entry once SCAN DONE prints.
  - Cautionary tales: (a) block-sampled pc==ENTRY reads the CALLEE
    window (args look like garbage/zeros) — always single-step to
    pre-ENTRY pcs for arg forensics; (b) `xQueueReceive` a4 is ticks
    ONLY at true ENTRY — post-entry a4 holds the mutex-layer scratch
    (`0x60023`-class values); (c) `0x3fcaa824`-class "garbage" queue
    addrs from post-entry reads are not corruption; (d) objdump on the
    CLOSED `libpp.a`/`libnet80211.a` regions desyncs — our decoder +
    live single-step is ground truth (again).

- 2026-09-18 (session 5 cont. — LAYOUT SETTLED + DECODER CLEARED;
  REAL PARK = loopTask event-wait, wifi task READY-but-never-scheduled):
  - TCB layout SETTLED by live dump (wifi cur0 @0x3fcac314, all fields
    cross-checked against pxReadyTasksLists/heap/nm symbols): +0 stack-top,
    +4 state item, +24 event item, +44 prio, +48 stack, +52 name[16],
    +68 runstate, ... (affinity word ABSENT — §8b source arithmetic was
    +4 off; corrected in §8b, old notes stand as measured).
  - Live facts at hang: wifi TCB state=(v=619, container=0x3fc9bac0 =
    prio23 ready list) → wifi is READY/RUNNABLE; event=(v=2,
    next/prev=0x3fcaa18c heap queue, owner=SELF, container=NULL) → wifi
    is NOT event-blocked. loopTask is event-blocked (infinite timeout,
    container 0x3fcac4ac). Core0 idles (waiti) DESPITE a runnable prio23
    task; core1 idles with loopTask parked. So the scheduler never
    switches TO the wifi task on core0 — the missing link is a YIELD /
    ready-priority update, not a WiFi register.
  - Queue census with CORRECT offsets still pending: dump 0x3fcaa148
    (the ONE wifi_create_queue product captured), the two pp_post queues
    (0x3fcaa824/0x3fcb1c8c), and the heap queue 0x3fcaa18c (wifi's event
    next/prev!) with +56/+60/+64 and +16/+36 wait lists.
  - wifiexec (new TEMP probe: logs every EXECUTED pc/len/mnemonic on
    core1 in a range) PROVES execution matches the wifidis static walk
    exactly through `ieee80211_ioctl` (0x42060611→…→0x420606fd
    `callx8` → 0x42060700 `bnei` → … → 0x42060724 `retw_n`, twice).
    NO desync, NO hijack — the `e417026f ee_cmul` word at 0x420606e4
    is real TIE code (or never executed on this path; either way the
    executed stream is sane). Decoder suspect CLOSED.
  - The `callx8` at 0x420606fd targets `pp_post` (0x400056e8, ROM
    code). `pp_post` is the wifi-task message queue post; the wifi task
    (`ppTask`) dequeues and runs the MAC. At hang, core0 (which should
    run the wifi task, prio 23, CURRENT) sits in `waiti`
    (`esp_cpu_wait_for_intr`) — i.e. the wifi task was never scheduled
    / never woke. loopTask (core1) is parked in `xQueueReceive` with
    infinite timeout on the pp_post queue side (event value 24).
  - So the chain is: loopTask → esp_wifi_start → … → ieee80211_ioctl →
    pp_post → (wifi task should wake on core0) → wifi task never runs
    → loopTask waits forever. The missing link is core0 scheduling,
    NOT a WiFi register: WHY does core0 idle instead of running the
    prio-23 wifi task after pp_post? Candidates: (a) the pp_post queue
    post never marks the wifi task readied (our xQueueGenericSend-from-
    ROM path?); (b) the cross-core yield (esp_crosscore_int_send) never
    preempts core0's IDLE; (c) the wifi task is itself blocked on an
    unmodeled primitive (MAC interrupt?).
  - NEXT: (1) identify the pp_post queue + the wifi task's wait primitive
    (watch xQueueGenericSend args + wifi-task TCB state/event container
    at hang — same TCB-census method as session 3); (2) check whether
    core0 takes the cross-core yield interrupt after pp_post (FROM_CPU
    regs + int_pending at hang were 0 — verify live around the post);
    (3) if the wifi task waits on a MAC event/queue, model THAT
    (MAC RX DMA ready / scan-done event path — the original §7 question,
    now precisely scoped).
  - wifidis walk of `ieee80211_ioctl` tail (0x420606c5..0x42060726,
    OUR-decoder ground truth) shows at 0x420606de: `100000 and` then
    `a52011 l32r [lit 0x42049aec = 0xc2b23adc]` (garbage literal!),
    `bc0271 l32r [lit 0x4204f674 = 0x3c0b6d50]` (garbage!),
    `922a add_n`, `0c0002 l8ui`, `b218 l32i_n`, `fec9 s32i_n`,
    `0a0c movi_n`, `83a8b0 moveqz`, `74b0a0 extui`, `0aec bnez_n`,
    `e6c992 addi`, `938b90 movnez`, `88dc bnez_n`, `201110 or`,
    `0274e5 call8 -> 0x42062ddc` (wifi_api_unlock — plausible),
    `bec931 l32r [lit 0x420501b8 = 0x3fcef940]` (sane!),
    then `callx8`, `l32r [0x4206056c = 0x3012]`, `j`, and at
    0x420606de: `100000 and`, `a52011 l32r [0x42049aec]`,
    `e417026f ee_cmul_s16_st_incp`, `030c06 j`, `000cc6 j`, ...
    The `e417026f` (b0=0x6f, op0=15 → len 4) is a REAL 4-byte word per
    insn_len — but is it a REAL ee instruction or a MISALIGNED read of
    two 2-byte + one 3-byte insn? `ee_cmul_s16_st_incp` needs
    b3==0xE4 + narrow guards (b0&0x0E==0x0E: 0x6f&0x0E==0x0E ✓).
    If EITHER this or an earlier len is wrong, every downstream pc is
    off by bytes and the "park" is the emulator executing garbage.
  - Why suspect: live wifideep trace shows core1 reaching 0x420606fd
    then NOTHING (no further pcs, ends IDLE) — consistent with EITHER a
    legitimate block (xQueueReceive with infinite timeout — the
    `callx8` at 0x420606fd's target unknown) OR executing into garbage
    after a misdecode. The garbage literals (0xc2b23adc as a POINTER)
    smell like desync, but could ALSO be rodata floats/ints the code
    never derefs on this path.
  - NEXT (do first): execution trace — single-step core1 through
    0x420606de..0x42060710 logging (pc, len, mnemonic) per EXECUTED
    insn (use cpu.last_len() or own insn_len; log from the live Cpu, not
    a static walk). If executed pcs match the wifidis walk AND the walk
    is sane arid the `callx8` target at 0x420606fd, read ITS body — the
    park is wherever execution stops advancing (pc frozen + interrupt
    storm? or waiti?). If executed pcs DIVERGE from the walk, the
    decoder has a hijack — fix the guard (likely an ee rule missing a
    length or overlap check).
  - Probes available (all TEMP, delete before commit):
    wifidis (static walk), wifideep (pc-in-range logger),
    wifitrace (call-site CALL/RET), wifisub/wififlag/wifipark/wifiprobe
    (older range/flag watches), decword (single-word decode).

- 2026-09-18 (session 4 cont. — wifidis probe works; `wifi_init_completed`
  RETURNS, `ieee80211_ioctl` next):
  - New TEMP probe `wifidis.rs` (delete before commit): disassembles LINKED
    fn bodies with OUR decoder (insn_len walk from fn symbol; call8 imm is
    already absolute in opnds — earlier `+4+` EDN math was wrong and is now
    fixed in the probe; l32r imm is the absolute literal addr + value
    dumped). Usage: `wifidis <bin> [lo hi]...`; defaults cover
    esp_wifi_start + _do_wifi_start. Verified: linked bodies decode sane
    (entry/or/call8/l32r/beqz.n/...) — objdump's garbage was linear-sweep
    desync, our decoder is ground truth.
  - `esp_wifi_start` linked body: `entry; or; call8 wifi_init_completed
    (0x42062e00); l32r 0x3001; beqz.n; ...; call8 0x4206057c; mov_n; retw`.
    `wifi_init_completed` (nm only — body at 0x42062e00, but nm -n shows
    NO gap before next sym, i.e. linker merged/folded it; disassembling
    0x42062e00 shows SIBLING code, not the fn — folded tail-call?). Its
    disasm region contains `call8 wifi_nvs_get (0x4205b854)` at 0x42062e3e
    — a second path to nvs. `0x4206057c` (second call8): entry/movi/beqz/
    .../`l32r [0x420501b8 = 0x3fcef940]`/`callx8` — needs symbol ID.
  - NEXT: identify 0x4206057c + 0x42062e00 via map (.o provenance), then
    extend the wifitrace call-site trace INTO esp_wifi_start's callees
    (call sites 0x420637ca + 0x420637f3 with site+3 RET) to find which
    callee never returns.

- 2026-09-18 (session 4 —RESOLVED: objdump artifact, emulator decodes
  correctly; REAL park is `wifi_station_start` ← `_do_wifi_start`):
  - The "b0=0x10 misdecode" was MY misread: objdump prints the LE WORD
    (`10004136`), so `esp_wifi_start` = `entry` + `or` + `call8` + ...,
    all decoding correctly in our decoder (verified word-by-word via
    decword + correct-alignment walk: entry/or/call8/l32r/beqz.n/...).
    The emulator executes it fine — and indeed the wifitrace CALL/RET
    log shows `start` never returns for a DIFFERENT reason (below).
  - Relaxation lesson stands: `.o` disasm is STALE for linked images
    (section shrank 0x4a→0x3a); always disassemble the linked ELF, and
    read objdump words as LE (b0 = LAST pair).
  - wifitrace CALL/RET (exact call8 site+3 semantics) + `_do_wifi_start`
    source disasm (from `libnet80211.a ieee80211_ioctl.o`, which IS
    readable): `_do_wifi_start` gates on `g_ic+0x100[234]==1`, then
    calls `wifi_nvs_get` → `wifi_pmk_is_valid` → **`wifi_station_start`**
    (reloc order). The wififlag watch (entry-RANGE based) showed core1
    reaching `esp_wifi_start` but none of the sub-fns — consistent with
    the park being INSIDE `wifi_station_start` or earlier in
    `_do_wifi_start` (entry-pc equality missed compressed words; ranges
    were also off because linked pcs differ from .o offsets).
  - NEXT: instrument `_do_wifi_start`'s linked sub-calls
    (`wifi_nvs_get` 0x4205b854, `wifi_station_start` 0x42068904,
    `wifi_hw_start` 0x4205e824, `wifi_mode_set` 0x4206bcb4 — LINKED
    addrs from nm, ranges to next sym) with the call-site CALL/RET
    method, and watch the g_ic+0x100 gate byte live.
  - `hal_tsf.o` disasm (closed `libpp.a`) PROVES the WDEV block layout:
    `hal_enable_sta_tsf` RMWs `[0x60035028]` bit 27; TSF enable/disable
    `0x60035040`; TBTT early/interval `0x6003503C/0x60035030`;
    timer-target `0x60035068/0x60035070`; counter `0x60035000/0x60035018`.
    These share the RNG page (RNG data = +0x7C only).
  - Tree change (UNCOMMITTED): `WDEV_BASE = 0x60035000` in memmap.rs;
    soc.rs RNG arm routes +0x7C to Rng, rest of page to Wifi store;
    `Rng` widened to full-page mask + 4KB regs (was 0x3FF/0x400 —
    +0x400..0xFFF aliased +0x000..0x3FF before). rng tests green.
  - Result: NO progress — identical hang (mode-tail max still
    0x42003bb1, same pcs). So the park is NOT a WDEV RMW spin.
  - Call-site trace (wifitrace, exact `call8` site+3 RET semantics):
    `start` CALL at t645231 never returns; everything before it
    returns OK. Next: disassemble `_do_wifi_start` (0x4205d248, from
    `libnet80211.a ieee80211_ioctl.o` — extract + objdump the .o like
    hal_tsf.o) and trace ITS sub-calls (`wifi_station_start` etc.).
- 2026-09-18 (session 3 — hang site NARROWED to loopTask event-wait with
  INFINITE timeout; vTaskDelay ruled OUT):
  - Hang-state dump (wifiprobe @60M steps): core0 =
    `timer_alarm_handler+0x18` return path (`vTaskGenericNotifyGiveFromISR
    +0x29`, notifying the esp_timer task — HEALTHY periodic activity, a
    red herring); core1 = IDLE (`esp_vApplicationIdleHook+0x16`).
  - loopTask TCB (0x3fcebcc0, prio 1): STATE item container =
    `xSuspendedTaskList` (0x3fc9b870) — correct for a blocked task — but
    the EVENT item (value 24 = 0x18) sits ALONE in a heap list at
    0x3fcac4ac (n=1, endval -1, next=prev=own item). That heap list is
    NOT any known queue's wait list (swept 5 known queues' wait words —
    no hit) and the ONLY DRAM reference to it is the task's own
    container pointer — i.e. an event-group / semaphore / mutex wait
    list, or a queue WE never discovered (wifi-driver-private).
  - `prvAddCurrentTaskToDelayedList` disasm PROVES the park is
    `vTaskPlaceOnEventList`-with-`xTicksToWait=portMAX_DELAY`
    (a2 = -1 → `bnei …,+0x9e` skips the delayed insert; a3 = 0 →
    `beqz …,+0x9e` skips the suspend branch — the task lands ONLY on
    the event list, no timeout). So loopTask waits FOREVER on whatever
    owns 0x3fcac4ac.
  - Delayed/overflow/timer lists all EMPTY (delayedN=0, ovfN=0,
    xActiveTimerList1 n=0); timeout CANNOT wake it. Tick healthy
    (tick=11462, systimer TARGET1 RAW set, both int_pending live).
  - NEXT: identify the owner of heap list 0x3fcac4ac. Options: (a) trace
    loopTask from SCAN START and catch the `vTaskPlaceOnEventList` /
    `xQueueReceive`/`xEventGroupWaitBits` call WITH its list/queue arg
    (arg = a2 at entry; resolve which call creates 0x3fcac4ac by
    watching queue/event-group alloc sites); (b) sweep heap for queue/
    event-group structs whose wait-list words point into 0x3fcac4ac
    (need exact Queue_t/EventGroup_t layouts — derive from
    `prvCopyDataFromQueue`/xQueueGenericSend disasm, NOT guesses).

- 2026-09-18 (session 2 — scheduler forensics, no tree change):
  Scratch probe `crates/esp32s3-emu/examples/wifiprobe.rs` (TEMP, delete
  before commit) + `/tmp/syms.txt` (ELF symbol dump). Findings:
  - Tick path HEALTHY: SYSTIMER TARGET_CONF0/1 = 0xc0003e80 (period mode,
    both on unit 1), CONF unit1 on, unit1 counter advances, tick ISR runs
    on both cores (xTaskIncrementTick in pc samples, xTickCount 735→2798
    over 12M steps), TARGET2 used by esp_timer (TARGET2 regs nonzero),
    RAW always 0x4 (ISR clears TARGET1 each tick; TARGET0/PRO quiet).
    Matrix src57/58/59 → lines (5,6,2)/(6,1,6). Prior "OP_VALID red
    herring" note confirmed: OP latches read stale until the OP_UPDATE
    handshake — firmware always handshakes, no bug.
  - **Real state: NOT a tick problem — the loop task never runs.**
    `loopTaskHandle` TCB (0x3fcebcc0, name "loop") sits in
    `xSuspendedTaskList` (container 0x3fc9b870), NOT in any ready list.
    Ready prio0 = {IDLE0, IDLE1} only; prio1 EMPTY; uxTopReadyPriority=23
    = "wifi" task (pxCurrentTCBs[0], contains core0). So `setup()` /
    `scanNetworks()` / `loopTask` never execute on either core — the
    earlier "delay(100) entered once, task parks" note is superseded:
    the Arduino task never starts at all.
  - Core0 IS making progress: it runs the wifi task (PHY cal: rfpll,
    txiq, chan-freq, rx-gain fns — the part-1 done-bits working) plus
    kernel critical-section spinning. Core1 runs IDLE
    (`esp_vApplicationIdleHook` 0x420965b4 + `waiti`).
  - Corrupt-looking ready counts at prio ≥25 are FreeRTOS lazy-FP-owner
    slots, not tasks (owner addrs OOB like 0x4037aa3c = IRAM code) —
    do NOT chase.
  - TCBListItem layout used: item +0 value, +4 pxNext, +8 pxPrevious,
    +12 pvOwner, +16 pvContainer; list +0 count, +4 pxIndex, +8 xListEnd
    (end marker value 0xFFFFFFFF). NOTE: pxIndex is only a cursor — walk
    raw chains from xListEnd.pxNext (+12), NOT from pxIndex.
  - Next step: find WHO suspends the loop task / why it never gets
    readied — watch `vTaskSuspendAll`/`vTaskSuspend`/`prvAddNewTaskToReadyList`
    (0x4037dc00) callers, or check whether `xTaskCreateUniversal` for
    loopTask (from `app_main` 0x42007a54 via 0x420079fc) even runs:
    instrument task-create + suspend points and log callers (return
    addresses off the stacks).
- 2026-09-18 (session 2 cont. — app_main traced, loopTask create RETURNS):
  - `main_task` (0x42096bb8) RUNS on core0 (step ~4116302), calls
    `app_main` (0x42007a54, step ~4123235).
  - `app_main` completes its full body: `setCpuFrequencyMhz` returns,
    `initArduino` returns, `xTaskCreateUniversal` (0x420064a4, the
    loopTask create) RETURNS (step ~4224035, back at 0x42007a85), then
    `main_task` calls `vTaskDelete` (0x4037ec5c, step ~4224039 — self-
    delete, normal IDF startup). So the loop task IS created.
  - Yet the loop TCB ends up in `xSuspendedTaskList` with empty ready
    prio1, and SCAN START (printed by `setup()` = loopTask body) DID
    appear on UART (17 bytes). Contradiction: setup() ran (it printed!)
    but the loop TCB is suspended and loopTask/​setup()/scanNetworks
    never appear in later pc surveys. Two hypotheses: (a) loopTask ran
    setup() then SUSPENDED ITSELF waiting on something (WiFi driver?
    event group?); (b) the "loop" TCB in the suspended list is a
    different incarnation. The delayed lists are EMPTY and tick advances,
    so it is NOT a vTaskDelay park — it is a suspend or event-wait with
    no timeout... but event-wait would leave it in a wait list, and the
    only populated wait list is the delayed list (empty).
  - NEXT: find WHERE the loop task parks AFTER printing SCAN START.
    setup() runs WiFi.mode→disconnect→delay(100)→scanNetworks. The
    delay(100) = vTaskDelay should put it in the delayed list — but the
    delayed list is empty at hang time, so either the delay EXPIRED and
    the task then blocked elsewhere (scanNetworks → waitStatusBits 60s
    timeout → xEventGroupWaitBits), or it never reached the delay.
    Instrument: watch loopTask's post-SCAN-START pcs (setup range
    0x42002be4..) + xEventGroupWaitBits entries (0x4037cb34) + dump the
    loop TCB's event-list item (TCB+8/+12: state list item) to identify
    the container list.
- 2026-09-18 (session 2 cont. — PARK SITE FOUND, single-core scheduling
  defect proven):
  - Phase 5 (post-SCAN-START trace on core1 = loop task's core): loop
    task runs setup() → `WiFi.mode` → `getMode` → `wifiLowLevelInit` →
    `NetworkManager::begin` → **`esp_wifi_init` (0x4203c684, closed
    blob) at step ~15530 — and NEVER RETURNS**. No delay, no
    disconnect, no scanNetworks, no xEventGroupWaitBits. The vTaskDelay
    hits at step ~36928 are OTHER tasks (core0 timers), not the loop
    task. The wifi task (core0) meanwhile parks on its msg queue
    `xQueueReceive q=0x3fcaa160` with `mw=0` (empty) + infinite timeout
    (a4=0xFFFFFFFF) — i.e. the wifi task waits for a message NOBODY
    SENDS (phase 7: zero `xQueueGenericSend` in 5M steps at hang).
  - Queue census: 0x3fcaa160 = wifi-task msg queue (uxLength 0xC8,
    itemsize 8, storage in heap); 0x3fc9af24 = arduino-events queue;
    0x3fc9c22c = tcpip mbox (pcOutputCallback=tcpip_init_done);
    0x3fcec7d0 = arduino-event data queue. (Queue field guess
    +56=mw/+16=wr was WRONG — real layout differs; the +56/+60/+64
    words at 0x3fcaa160 read 0/0xC8/8 = length/itemsize/count-ish.
    Don't cite offsets without re-deriving from prvCopyDataFromQueue.)
  - xQueueReceive arg caution: at ENTRY (entry a1,64, before body
    spills) callee a2/a3/a4 = queue/buf/ticks directly — the logged
    q values resolved to real queues, but `ticks`/`buf` columns were
    mislabeled (a4 vs a5); re-derive if reused. Garbage q values
    (0x3fcec7d0 mw=1G) = sampling mid-spill, not corruption.
  - ROOT CAUSE (scheduling, not WiFi): the loop task runs on CORE1, but
    `esp_wifi_init`'s blob does cross-core work that needs CORE0's
    cooperation while core0 runs the wifi task at prio 23. uxTopReady-
    Priority=23 with the wifi task CURRENT on core0 = core0 never
    yields to anything below prio 23 on its own core; the loop task
    (prio 1) on core1 waits inside the blob for an event/post that
    requires wifi-task progress, but the wifi task itself waits on its
    empty msg queue — classic single-core-starvation deadlock IN THE
    EMULATOR's serialized step order (core0 wifi task never gets the
    message because the sender (loop task on core1) is stuck in the
    blob). On silicon both cores truly run in parallel and the IPC
    (`esp_ipc`) pumps the message through.
  - NEXT (the actual fix): trace what `esp_wifi_init` waits on — dump
    core1's stack at hang (backtrace through windowed frames is hard;
    easier: watch `esp_ipc_*` / `esp_crosscore_int_send` calls and the
    FROM_CPU regs, or single-step core1 inside the blob watching for
    event/queue/IPC primitives). The fix will be a done-bit or message
    pump at whatever primitive the blob parks on — same playbook as
    part 1 (objdump evidence + arming rule, never invented timing).
