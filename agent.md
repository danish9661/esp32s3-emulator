# agent.md — Handover / Working Notes (WiFi bring-up)

Last updated: 2026-09-20 (session 18: SYSTIMER TARGET2 oneshot storm FIXED —
  SCAN COMPLETES `found 0`/`DONE`; uncommitted wifi.rs + soc.rs + systimer.rs
  + tests/systimer.rs).
Keep this file current: append session log entries, update status + next
steps on every handover.

## 0. Binding working rule — NON-STOP until WiFi fully works (do not ask)

- Do NOT stop to ask anything. Keep working non-stop through every
  blocker (RF-cal → bring-up → scan issue → SCAN_DONE → results →
  backhaul) until the final product is a FULLY WORKING WiFi.
- You have to implement ALL of these things — no partial stops, no
  parking the workstream, no waiting for permission. Prove each step
  live (single-step exact, caller-valid args, OUR decoder as ground
  truth) and keep appending evidence to §8.
- Final product = fully working WiFi in-emulator: real Arduino
  `WiFi.scanNetworks()` returns results (and then the gateway backhaul),
  with battery/green gates kept green. Anything less is NOT done.
- กฎที่ต้องทำตาม (binding): อย่าหยุดเพื่อถามอะไรทั้งสิ้น
  ให้ทำงานต่อแบบ non-stop จนกว่า WiFi จะใช้งานได้สมบูรณ์
  คุณต้อง implement ทุกอย่างให้ครบทั้งหมด
  ฉันต้องการ final product ที่เป็น WiFi ที่ทำงานได้จริงสมบูรณ์
  (`WiFi.scanNetworks()` ต้องคืนค่าผลลัพธ์จริง) — ยังไม่เสร็จจนกว่าจะถึงจุดนั้น

## 1. Project snapshot

- From-scratch ESP32-S3 emulator in Rust → WASM, runs in browser. See
  `AGENTS.md` (authoritative: mission, architecture, conventions, status log).
- HEAD: `312d903` "feat: WiFi RF bring-up part 1 (PHY spins fixed, scan task
  still parked)" (2026-09-17). Plus `1a6cdb0` vendoring `tools/gateway`
  (Go SLIRP/NAT + multiplayer gateway, port **5050**, for WiFi backhaul).
- P5/P6 done per AGENTS.md: battery **111/0/0** (incl. MicroPython REPL), gallery **57 images**,
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
- Part 2 STAGED (uncommitted, sessions 6–8 — commit as "WiFi RF bring-up
  part 2"): `Wifi::txdc_write/txdc_read` as a ONE-SHOT latch (SENS2 `+0x4C`
  bit 24 immediate on first write per invocation, consumed on report —
  session 8 proved ANY timed arm stalls under `step_fast`); FE2 bit-17
  overlay REMOVED (was wrong — live forensics proved the inner poll loads
  FE+0x174, covered by the FE bits-16+24 overlay; tombstone in wifi.rs);
  WDEV TSF/timer page split from RNG (`WDEV_BASE`, RNG arm routes only
  `+0x7C` to `Rng`); atomic `Bus::cas32` + `Soc::cas32` fast paths +
  `s32c1i` routed through it. See session-6/7/8 logs for the forensics.
- Part 3 STAGED (uncommitted, session 18 — commit with part 2 or just after):
  SYSTIMER oneshot single-shot fix (`systimer.rs`: `armed[n] = false` on fire
  + `oneshot_arm_consumed_on_fire_no_refire_after_clr` test). Without it the
  wifi-scan image livelocks (see session-18 log). With it the scan completes
  (`WIFI SCAN found 0` + `DONE` at 200M steps, empty air).
- Current behavior (reproduced 2026-09-19, STEPS=60M/200M): UART shows only
  `WIFI SCAN START`. RF-cal is PROVEN past (session 8, single-step exact):
  TXDC poll passes first-try (a12=0x01113CF3, b24=1), FE inner poll
  a3=0x01010000 exits first pass, MAC-CTRL reads 0x3 (bit 0 set). The
  post-SCAN-START chain is now MAPPED (session 9, single-step exact):
  loopTask (core1) runs setup() → `WiFi.mode` → `getMode` →
  `wifiLowLevelInit` → `NetworkManager::begin` → `esp_netif_init`
  (which creates the tcpip sys_sem + posts init) → `esp_wifi_init`
  (core1) → ppTask ENTERs on core0 → 5× `ieee80211_ioctl` + `pp_post`
  with pp-queue mw 0→1→0 consumed each time (pp pump HEALTHY) →
  `esp_wifi_start` (core1) → `_do_wifi_start` + `wifi_station_start`
  (core0, tick 588-629, full wpa_attach/wpa_sm_init chain) — i.e. the
  whole WiFi bring-up RUNS. scanNetworks/esp_wifi_scan_start/scan_start
  NEVER ENTER in 80M post-SCAN-START macro-steps; the steady state is
  loopTask event-blocked (evC=0x3fcac4ac, evV=24) with core0 in the
  timer/notify CAS loop + core1 IDLE. The earlier session-8 "sys_sem
  wait/signal pairing" block (struct 0x3fceb418 → queue 0x3fcec798) is
  RESOLVED as a HEALTHY transient: the wait (ticks=0) is consumed by the
  tick-232 signal (mw 0→1, loopTask readied, evC cleared) and execution
  continues — it is NOT the steady-state park. The session-7 "delay loop
  + txdc poll" park is SUPERSEDED (60M-budget snapshot mid-RF-cal).
  Next: find WHERE loopTask parks AFTER `esp_wifi_start` returns
  (delay(100)? disconnect? scanNetworks entry? waitStatusBits?) — the
  loop-ev transition watch + PLACE list/ticks log pin it; then model
  whatever the scan path polls (MAC RX DMA / SCAN_DONE event).

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

- 2026-09-20 (session 18 — TARGET2 ONESHOT STORM NAILED + FIXED; SCAN
  COMPLETES `found 0` + `DONE`):
  - Symptom: 60M/200M runs print only `WIFI SCAN START`; FINAL park =
    loopTask 6th coex take (evC=0x3fcac4ac, SUSPENDED) + core0 kernel-lock
    churn + core1 IDLE. Coex accounting: 6 takes / 5 pp_posts / 4 gives —
    take #6 blocks with no give #5 because the wifi thread (ppTask loop)
    never runs at FINAL.
  - Root cause (proven live, single-step exact): SYSTIMER TARGET2
    (esp_timer oneshot on unit 0) with a STALE target refires on the very
    step after every INT_CLR (`wifirace` probe: RAW 0x4→0x0 at
    `timer_alarm_isr+0x1b` then 0x0→0x4 at `+0x1e`, ISR every ~183 steps,
    252 ISR/50k). The old oneshot model kept `armed[n]=true` forever after
    the first COMP_LOAD, so any stale target (counter already past it
    because s_timer_task hasn't run yet) storms: each ISR does
    notify-give + lock churn + cross-core yield, starving the wifi thread
    and s_timer_task (the task that would reprogram the target) — a
    serialized-execution livelock (on silicon real concurrency lets the
    servicing task win the race). Forcing TARGET2 far-future UNBLOCKED
    everything live: give #5 at +3688 steps, s_timer_task RUNNING at +6420,
    loopTask through ioctl→esp_wifi_start→mode→disconnect→delay→
    scanNetworks→SCAN_DONE.
  - Fix (in tree, validated): oneshot arm CONSUMED on fire
    (`systimer.rs` `armed[n] = false` in `check_alarms` oneshot branch +
    doc comment with the forensics; firmware re-arms per alarm via
    COMP_LOAD per `systimer_hal_set_alarm_target`: disable→set→apply→
    enable — verified in IDF source). Period-mode (tick) path untouched.
    New test `oneshot_arm_consumed_on_fire_no_refire_after_clr`.
  - Result: 200M-step run prints `WIFI SCAN START` / `WIFI SCAN found 0` /
    `WIFI SCAN DONE` (empty air, no APs — CORRECT with no RF stimulus;
    results path needs an AP model or gateway backhaul, next).
    `wifirace` after fix: 6 RAW transitions (healthy alarms) + 0 ISR/50k
    in the window (was 508 + 252). Probes removed (all TEMP wifi*.rs
    examples deleted).
  - Validation at stage: 39 suites green (incl. new systimer test),
    clippy `-D warnings` clean, fmt clean, wasm32 clean, battery subset
    green (hello/hello_opi/systimer/periph/uart_echo/gpio_uart_timer/
    multi_irq/timer_alarm/uhci/tempdev/adc_dma/rng/deepsleep_touch/touch/
    touch_denoise/temp 16/16).
  - NEXT: (1) commit parts 2+3 + logs; (2) AP/results path — model scan
    results (predefined AP list? gateway backhaul?) so `found N` with N>0
    and SSIDs print; (3) add the `wifi_scan` battery entry once the
    results path is decided.
  - Cautionary tales: (a) RAW=0x4 is bit 2 = TARGET2 (esp_timer), NOT
    TARGET1 — check bit numbering before theorizing (TARGETn = bit n);
    (b) `wifidec`-after-boot reads the LOADER MMU mapping for app addrs —
    only valid post-cache-init (execution view); (c) objdump linear sweep
    desyncs in DSP-dense fns (esp_wifi_start) — OUR decoder + live
    single-step is ground truth (again); (d) shadow-stack RET pairing by
    (core,ret-pc) lies under FreeRTOS (recycled return pcs) — single-step
    pc traces, not call/ret pairing, for park proofs.

- 2026-09-20 (session 17 — unified take/send timeline; FINAL core0 mix
  classified; timer-notify pump identified):
  - Site-method take timeline (prev-pc + site-a2, caller-valid for sure;
    ENTRY-a2 == site-a2 VERIFIED equal on all 30+ takes, so ENTRY reads
    are valid here): loopTask takes run UART/mutex/queue traffic
    (0x3fc95814, 0x2, 0x3fc9be38, 0x3fc9ca50×4, sys_sem 0x3fceb418,
    0x3fc98a4c/0x3fc9bdd4/0x3fc9b4f0 bursts, 0x3fcaa824) then coex takes
    at ticks 240/259/265/268/269 (site 0x4203c4d6 =
    `esp_coex_common_semphr_take_wrapper` call site, verified). Unified
    DECIDE/SEND pairing on coex: DECIDE loopTask mw=0 (block) →
    SEND wifi mw=0 (post) → DECIDE loopTask mw=1 (consume), at ticks
    258/259/268 (+269 block with no post). Four complete rounds, then
    the FINAL take blocks with evC=0x3fcac4ac. pp-queue mw is 1 exactly
    at each coex DECIDE (pp pump guarda the rounds); after tick 269 both
    queues sit at 0 forever.
  - FINAL core0 200k-classification (single-step exact): CAS 11%
    (`esp_cpu_compare_and_set` 0x40379858..), crit 38%
    (`xPortEnter/ExitCritical` 0x4037ce00..0x4037d100), lowint 17%
    (`_xt_lowint1` dispatch 0x40377900..), rom-vec 5%
    (`_xtos_set_intlevel` + ROM vector), other 28% = the TIMER-NOTIFY
    pump: `timer_alarm_isr` (0x403771a0) → `timer_alarm_handler`
    (0x40377d70) → `vTaskGenericNotifyGiveFromISR` (0x4037f49c), each
    ~1000/200k, i.e. one full notify-give per tick. So core0 at FINAL is
    the wifi task servicing the esp_timer tick notify-give (kernel-lock
    churn is its enter/exit, NOT a stuck spin — session-16 framing
    stands, now with the notify path named).
  - NEXT: (1) name the notify TARGET (which TCB does the timer give wake?
    `s_timer_task`/esp_timer? — log a2 at `timer_alarm_handler` entry);
    (2) decide whether the notify pump is the wifi task's ONLY remaining
    work (then the missing link is whatever should post coex #5 — likely
    a wifi-driver timer/callback that never fires because its timer regs
    are unmodeled); (3) model it; (4) delete `trace_probe.rs` before
    commit; (5) validate then commit part-2 + logs.

- 2026-09-19 (session 16 — kernel-lock CAS is HEALTHY spinning (not stuck);
  park restated as two-sided idle):
  - Lock forensics at FINAL (200k single-steps): `xKernelLock`
    (0x3fc95a88) cycles owner↔FREE at full speed — b33fffff→0xcdcd at
    `s32c1i` (0x403798aa, inside `esp_cpu_compare_and_set`) then
    0xcdcd→b33fffff at the `j` back-edge of `vPortExitCritical`
    (0x4037cffa), both on core0 with cur0=wifi throughout. So the wifi
    task ACQUIRES and RELEASES the lock every iteration (not spinning on
    a held lock): the CAS loop is the normal kernel enter/exit churn of
    a task doing periodic work (the timer notify-give path at
    `vTaskGenericNotifyGiveFromISR+0x32`), NOT a stuck spin. The
    session-15 "spins acquiring" reading is SUPERSEDED — acquire+release
    both fire; the wifi task makes progress through the lock, it just
    never does VISIBLE work (no pp-queue recv, no coex post).
  - Restated park (no model gap proven anywhere): core0 runs the wifi
    task doing lock-churn ISR bookkeeping; core1 runs IDLE1 + healthy
    no-switch decisions; loopTask suspended+event-waiting on the coex
    queue (evC=0x3fcac4ac). Both sides are "running but idle": the wifi
    task's real work source (pp-queue? RF events? MAC interrupts?) is
    dry, and loopTask's wakeup (coex post) never comes because the wifi
    task has nothing to post about. The missing link is UPSTREAM of both
    — what should feed the wifi task its next unit of work (a pp message?
    an RF/MAC interrupt? a timer event?) — i.e. back to the RF/MAC
    peripheral modeling (RX DMA? scan command issue?), NOT the scheduler.
  - NEXT: (1) find what feeds the wifi task post-bring-up (pp-queue
    producers? MAC interrupt source? — watch pp-queue posts + INT matrix
    at FINAL vs during the healthy pump rounds); (2) trace what SHOULD
    happen after `wifi_station_start` returns (the bring-up chain went
    quiet after tick 629 with no scan issued — who issues the scan?
    does `scanNetworks` need an event-bit first?); (3) model it;
    (4) delete `trace_probe.rs` before commit; (5) validate then commit
    part-2 + logs.

- 2026-09-19 (session 15 — scheduler HEALTHY at FINAL; wifi ready but
  never scheduled — the switch path is the missing link):
  - At FINAL (evC=0x3fcac4ac, loopTask suspended+event-wait): the
    scheduler inputs are all HEALTHY — tick ISR runs on core0 (62
    `xTaskIncrementTick` entries/1M steps), `vTaskSwitchContext` runs on
    core1 (63/1M), `vPortYieldFromInt` runs on core1 (63/1M),
    `xYieldPending`=[0,0], `uxSchedulerSuspended`=0, tick advances.
    The switch DECIDES no-switch every time (`beqz a9` at 0x4037e24f
    with a9=0, 3/3 sampled) — correct, since nothing pends a yield.
  - Ready state at FINAL: p0={IDLE0,IDLE1}, p18={tiT}, p22={esp_timer},
    p23={wifi}, p1 EMPTY (loopTask suspended, not ready),
    uxTopReadyPriority=23, cur0=wifi, cur1=IDLE1. So core0 CURRENTLY
    RUNS the wifi task (prio 23, highest) — 100% of core0's 1M-step
    histogram is the wifi task's `esp_cpu_compare_and_set` CAS loop on
    `xKernelLock` (0x3fc95a88, sc=mem=0xb33fffff) via
    `vTaskGenericNotifyGiveFromISR+0x32` (return a0=0x8037f4ce) — i.e.
    the wifi task spins acquiring the kernel lock inside a notify-give
    from the timer ISR path, while the timer ISR itself (`timer_alarm_
    handler` 0x40377d70 → notify-give) fires every tick. Core1 runs
    IDLE1 + the per-tick switch that keeps IDLE1 (nothing readies a
    higher task on core1: loopTask is suspended, not ready; tiT/esp_timer
    waits are future-dated).
  - So the machine is NOT deadlocked on a peripheral: it is a SCHEDULER
    liveness question — why does the wifi task (READY, CURRENT, prio 23
    on core0) never progress past the kernel-lock CAS into real work
    (pp-queue recv? coex post?), and why does no post ever land on the
    coex queue to wake loopTask? The two halves (wifi spinning on the
    lock, loop suspended on the queue) form the visible park; the missing
    link is whatever should break EITHER side (a lock release? a coex
    post from ppTask/ISR?).
  - NEXT: (1) identify the CAS loop's lock competition (who HOLDS
    xKernelLock while wifi spins? — watch lock-word writers + holder);
    (2) find what the wifi task is trying to do past the lock (pp-queue
    recv? notify-take? — single-step wifi past the CAS with the switch
    path); (3) model the missing wakeup; (4) delete `trace_probe.rs`
    before commit; (5) validate then commit part-2 + logs.

- 2026-09-19 (session 14 — FINAL block = SUSPENDED + event-wait (no
  timeout); loopTask take census COMPLETE; evC=0x3fcac4ac):
  - Site-validated PLACE log (prev-pc + site-a2, the ONLY valid method —
    ENTRY-a2 is caller-window garbage for call8, proven by the garbage
    B+56 reads): loopTask PLACEs 0x3fcec798 (sys_sem, tick 231) →
    0x3fcaa824 (tick 238) → 0x3fcac488 coex ×5 (ticks 240/259/265/268/
    269), all ticks=-1, all via the take path at 0x4037c99c
    (`xQueueSemaphoreTake` → `vTaskPlaceOnEventList`, NOT the mutex
    wrapper — the take wrapper only selects the path). Every PLACE
    carries ticks=-1 = portMAX_DELAY.
  - FINAL state (stable past tick 1261, production 150M macro-steps):
    loopTask evV=24, evC=0x3fcac4ac (n=1, sole member), stV=239,
    stC=0x3fc9b870 = `xSuspendedTaskList` (NOT delayed — D1/D2 n=0).
    So the take went down the SUSPEND branch (`prvAddCurrentTaskToDelayedList`
    with ticks=-1 suspends when INCLUDE_vTaskSuspend, verified by the
    ST-CONT trace: ready(0x3fc9b908) → 0 → SUSPENDED at the block,
    woken (SUSPENDED → ready) only by the paired signals). A suspended +
    event-waiting task wakes ONLY via `xTaskRemoveFromEventList` on its
    event list — i.e. a post to 0x3fcac488 (the coex queue) — which never
    comes because the wifi task idles once loopTask stops driving it.
  - loopTask take census post-SCAN-START (60M single-steps, loopTask
    takes only): UART/mutex/queue takes (0x3fc95814, 0x2, 0x3fc9be38,
    0x3fc9ca50, 0x3fceb418/sys_sem, 0x3fc98a4c, 0x3fc9bdd4, 0x3fc9b4f0,
    0x3fcaa824, coex ×5) — then SILENCE: no further takes after the
    5th coex take, core1 IDLE forever. So loopTask never reaches
    delay(100)/disconnect/scanNetworks AFTER the coex region — it parks
    INSIDE the 5th coex take's event-wait. scanNetworks (0x42003e8c)
    never enters because the sketch never gets past the coex-gated
    `esp_wifi_start` return path (the start wrapper never returns: the
    coex take inside it never completes).
  - NEXT: (1) the coex take needs ONE more wifi post (mw 0→1) to
    complete the 5th round — find what makes the wifi task post it
    (does the wifi task need another pp message? is ppTask parked?
    watch pp-queue + wifi-task state at the FINAL block); (2) if the
    wifi task waits on loopTask (circular), find the cycle's missing
    link (the session-6 IPC chain: pp_post → WIFIQ → yield — verify each
    link live at FINAL); (3) model the missing post/wakeup; (4) delete
    `trace_probe.rs` before commit; (5) validate then commit part-2 +
    logs.

- 2026-09-19 (session 13 — wifi-task GIVE found; coex pump FULLY paired;
  block is purely loopTask-side):
  - The session-12 "never a give" verdict was WRONG (wrong-probe fallacy:
    it watched `xQueueGiveMutexRecursive` entries, but the wifi task
    gives via the COMMON wrapper `esp_coex_common_semphr_give_wrapper`
    (0x4203c4cc), which posts DIRECTLY to the queue (verified: wrapper
    body is entry → `xQueueGenericSend` with a2 = QUEUE, not struct —
    disasm-walked live bytes; at SEND-ENTRY a2 == 0x3fcac488). Queue
    histogram over 40M steps PROVES it: coex appears 9× at DECIDE and
    4× at SEND (post-deref pc 0x4037c525, callee-valid) — all four SENDs
    on core0 by the wifi task (a0=0x8203c4f1 = inside the give wrapper),
    at ticks 258/259/268/269, each mw 0→1. Paired loopTask DECIDEs at
    the same ticks show mw=1 then mw=0 (consumed). FULL pump pairing,
    no model gap anywhere in the take/send path.
  - So the FINAL block is PURELY loopTask-side: after its 5th take
    (tick 269, mw=0), loopTask event-waits (PLACE 0x3fcac488 ticks=-1 →
    evC=0x3fcac4ac) and the wifi task never posts again — because
    loopTask never ASKS again (no 6th take: the sketch moved past the
    coex-gated region into delay(100)/disconnect/scan prep, and the
    FINAL wait is a DIFFERENT primitive). The coex question is CLOSED
    (healthy, fully paired, 4 rounds + final wait).
  - NEXT: (1) identify the FINAL wait's primitive (PLACE site-validated
    list/ticks — the FINAL PLACE list read was garbage because ENTRY-a2
    is caller-window garbage for call8; use the SITE method: prev-pc +
    site-a2, proven: FIRST PLACE site-a2=0x3fcec798 correct); (2) trace
    what SHOULD post it (scan-done? event group?); (3) model it;
    (4) delete `trace_probe.rs` before commit; (5) validate then commit
    part-2 + logs.

- 2026-09-19 (session 12 — coex "missing give" CLOSED as healthy handoff;
  loopTask DOES progress past every coex take; FINAL park identified):
  - The session-11 "missing give" verdict was WRONG (single-cause
    fallacy: it tracked ONE take but FIVE takes exist). Live take census
    on the coex queue (take-ENTRY filter q==0x3fcac488, taker task +
    ticks): loopTask takes at ticks 240/259/266/268/269 (ticks=-1 each)
    — and the queue mw timeline shows 0→1 (create-copier at
    `prvCopyDataToQueue` 0x4037c1b8) then 1→0 CONSUMED four times at
    ticks 258/259/268/269 by the wifi task (core0, `pxCurrentTCBs[0] ==
    wifi`), each consume inside the cross-core yield / list-remove path
    (not a take ENTRY — the take happens via the scheduler handoff, so
    the ENTRY filter misses it; the mw transition + consumer task prove
    it). I.e. the coex mutex is a HEALTHY wifi↔loopTask handoff pump:
    every loopTask take is satisfied by the wifi task's progress, four
    consecutive rounds. The "no give in 100M steps" probe watched only
    `xQueueGiveMutexRecursive` (wrong primitive — the pump moves via
    take/consume + create-copier posts, never a give).
  - The FINAL take (5th, tick 269) then blocks (mw=0, PLACE 0x3fcac488
    ticks=-1 → evC=0x3fcac4ac) because the wifi task STOPS producing —
    and the wifi task stops because loopTask stops driving it: after
    `esp_wifi_start` returns, loopTask runs delay(100)/disconnect temper
    traffic (mutexes 0x3fced4d0/0x3fcec7d0, `wifi_api_lock`/`unlock`
    pairs for init/set_mode/get_protocol/start all completing) and then
    parks in the FINAL event-wait (evC=0x3fcac4ac, evV=24, stable past
    tick 1261). So the causality is REVERSED from session-11: not "wifi
    never gives → loop stuck", but "loop stops asking → wifi idles".
  - NEXT: (1) identify the FINAL wait (PLACE list/ticks at the FINAL
    block — same sys_sem playbook: is it the scan-done event? the
    `waitStatusBits` event group?); (2) find what SHOULD wake it (scan
    task? ppTask post? — the scan path: does `scanNetworks` even get
    called? the sketch calls it after delay(100)); (3) model the
    scan-done/event path; (4) delete `trace_probe.rs` before commit;
    (5) validate then commit part-2 + logs.

- 2026-09-19 (session 11 — coex-mutex take/take-back resolved; holder==self
  SUPERSEDED; coex enable path MAPPED, give still missing):
  - Holder semantics RESOLVED via `xQueueTakeMutexRecursive` disasm
    (objdump-verified) + live BNE ground truth: the take loads holder
    (`l32i a2,[a7,8]` at 0x4037c9f2), gets current task
    (`xTaskGetCurrentTaskHandle`), and branches at 0x4037c9f7 —
    holder==current → RECURSE-PATH (count++, return 1), else TAKE-PATH
    (`xQueueSemaphoreTake`). The "holder==mutex-addr (SELF)" reading was
    a WINDOW artifact: at TAKE-E the regs are still caller-window, so
    `m.soc.read32(q+8)` was read with a STALE q (the earlier probe read
    holder with q=0x3fcac488 before the deref was proven). Live at the
    BNE (callee window, caller-valid per session-6 rule): holder=0x0 vs
    cur=loopTask → TAKE-PATH (same as the wifi_api mutex). The session-10
    "SELF" note is SUPERSEDED — the mutex is FREE (holder 0), the take
    correctly proceeds to the underlying semaphore take.
  - The coex take then BLOCKS at DECIDE (mw=0, ticks=-1 → PLACE list
    0x3fcac488 → evC=0x3fcac4ac): i.e. the mutex's COUNT is 0 — a
    counting-semaphore-created mutex (`xQueueCreateCountingSemaphore`
    path via LEN-WRITE at 0x4037c44f, len(a7)=1, create-a0=0x82028215 =
    `sys_sem_new+0x29`!) starts EMPTY and needs an initial GIVE that
    never comes in-emulator. Creator chain: `sys_sem_new` ←
    `esp_netif_init` creates the counting sem (max=1, init=0); the block
    queue B IS that sem's queue (LEN-WRITE pairing proven). So the
    missing piece is the INITIAL `sys_sem_signal` (or the coex-init
    give) for the coex sem — same class as the session-9 sys_sem
    wait/signal pairing, but for the COEX sem this time.
  - coex enable path MAPPED (core0, tick 586): `coex_enable_wrapper` →
    `coex_enable` → `coex_core_enable` → `coex_register_start_cb` →
    `esp_coex_is_in_isr_wrapper` + `esp_coex_internal_semphr_take_wrapper`
    (take) ... → `esp_coex_internal_semphr_give_wrapper` (give) →
    `wifi_reset_mac` → `periph_module_reset`. The INTERNAL take/give
    pair use a DIFFERENT sem (`coex_env`-adjacent 0x3fc98b14, NOT the
    0x3fcac488 mutex — verified live: INT-TAKE/GIVE args are the
    `coex_env` ptr / wrapper addrs, never the mutex). So coex_enable
    does NOT give the 0x3fcac488 mutex. `coex_core_init` fires on core1
    at tick 232 (before the block) — its give, if any, was not observed;
    no `xQueueGiveMutexRecursive` on 0x3fcac488 in 100M macro-steps
    (ticks to 19714).
  - NEXT: (1) find WHO gives the coex counting-sem on silicon
    (`coex_core_init` internals? `esp_coex_init`? ppTask post-enable?
    — trace `coex_core_init` (0x4208db38) body + watch gives on
    0x3fcac488 from boot with the give-entry probe); (2) model the give
    (one-shot init give, same class as TXDC/FE done-bits but for a
    semaphore: post mw 0→1 once the init path runs); (3) then scan path;
    (4) delete `trace_probe.rs` before commit; (5) validate then commit
    part-2 + logs.

- 2026-09-19 (session 10 — FINAL block = coex mutex 0x3fcac488, HELD, never
  given; bring-up chain COMPLETE through esp_wifi_start return):
  - FINAL block identified (single-step exact): loopTask event-waits
    (ticks=-1) on the COEX mutex 0x3fcac488 (`esp_coex_common_semphr_take_
    wrapper` ← `esp_wifi_start` path, take ticks=-1), blocking in
    `vListInsert` at 0x4037d9c5 with evC=0x3fcac4ac/evV=24 (stable past
    tick 1261, production 150M macro-steps). Queue B=evc-36 PROVEN as the
    mutex itself: struct[0] check N/A (mutex IS the Queue_t — verified
    via `xQueueCreateMutex` call-site args len=1/item + returned queue
    matching B, same playbook as the sys_sem resolution).
  - Mutex state at block: mw=0, holder=0x3fcac488 (SELF — the mutex addr
    in its own holder field), count=1070253192, sendw=0, recvw=1
    (loopTask the sole waiter). It is NEVER given: zero
    `xQueueGiveMutexRecursive` on 0x3fcac488 in 6M post-`esp_wifi_start`
    steps (only unrelated mutexes 0x3fced4d0/0x3fcec7d0 cycle). So the
    park is a MISSING-GIVE, not a missed-wakeup: some core should give
    the coex mutex (ppTask? coex task? ISR?) but never does in-emulator.
  - HOLDER-SELF reading (holder == mutex addr) is the live-observed value
    at the take (verified at TAKE-E, holder-field = 0x3fcac488) — NOT yet
    interpreted (recursive-mutex owner-vs-count layout per §8b needs the
    `xQueueTakeMutexRecursive` disasm: holder-vs-current compare at
    0x4037c9f2/0x4037c9f7 takes the TAKE-PATH when holder != current,
    proven live holder=0x0 vs cur=loopTask on the wifi_api mutex; the
    coex take's holder==self case needs the same disasm read to say
    whether SELF means "free", "recursed", or a model-visible stuck bit).
  - Bring-up chain COMPLETE (entry logs, caller-valid): `esp_wifi_start`
    on loopTask runs `wifi_init_completed` → `wifi_api_lock` →
    `current_task_is_wifi_task` → mutex wrappers → `wifi_api_unlock` →
    `wifi_zalloc_wrapper`/`calloc` → ... → coex-sem take (above) — i.e.
    loopTask gets PAST start-entry INTO the coex-gated region and parks
    there. scanNetworks/`esp_wifi_scan_start`/`scan_start` still never
    enter (they are past the coex gate). The session-9 "post-start park,
    unidentified" is now IDENTIFIED (coex mutex); the remaining question
    is only the missing give.
  - NEXT: (1) disasm `xQueueTakeMutexRecursive` holder semantics
    (is holder==mutex-addr "free"? what SHOULD the take do — succeed or
    block?); (2) find WHO gives the coex mutex on silicon (ppTask?
    `coex_enable`? timer/ISR? — watch gives on 0x3fcac488 across the
    full boot, and trace what CREATES/inits it: `coex_core_init`?
    `esp_coex_init`?); (3) model the give (or the init that pre-gives);
    (4) then scan path; (5) delete `trace_probe.rs` before commit;
    (6) validate then commit part-2 + logs.

- 2026-09-19 (session 8 — TXDC ONE-SHOT latch; FE-17 REMOVED as wrong;
  loopTask block = tcpip sys_sem wait/signal pairing):
  - TXDC final form (in tree, validated): `txdc_write` arms on the FIRST
    +0x4C write per invocation (idle edge, value-agnostic);
    `txdc_read` reports bit 24 IMMEDIATELY and consumes the latch
    (one-shot, re-arms per invocation). Proven live by single-step
    forensics: a12=0x01113CF3 (b24=1) on the FIRST poll pass, and
    production (`step_fast`) exits the poll after 2 macro-steps. ANY
    timed arm (`now + N`, N >= 1) stalls: `step_fast` runs the whole
    poll iteration (write +0xf3, read +0xfb) in ONE macro-step with NO
    tick between, so the timer can never elapse (sens frozen at
    0x00113cf3, 998 single-step passes OK vs 0 production exits —
    same loop, different driver). Session-7 bit-1-SET gate was
    BACKWARDS (pre-poll pattern is 0x...f1, bit 1 CLEAR); bit-1-CLEAR
    re-arms every pass like a level arm. The 48k-cycle value is gone.
  - FE bit-17 overlay REMOVED (was added this session, proven wrong
    within the session): live poll-load-address log shows the
    `ram_iq_est_enable` inner poll at 0x4208012b loads from
    `[a10 = FE+0x174 = 0x60006174]` (a3=0x01010000 exits first pass via
    the FE bits-16+24 overlay), NOT an aliased `[0x6000E04C]`. The
    +0x54/+0x58 RMWs touch FE2+0x144 but the poll-load a10 is FE+0x174
    — different register. Tombstone comment kept in wifi.rs so nobody
    re-adds it. Cautionary tale: objdump register guesses need a live
    poll-load-address log before modeling (the static disassembly
    misled twice: +0x50 vs +0x4C earlier, bit-17 now).
  - FE/MAC-CTRL arms PROVEN live: FE inner poll 3/3 first-pass exits
    (a3=0x01010000); MAC-CTRL first nonzero write is 0x3 (driver sets
    bit 1, overlay sets bit 0 → poll exits). MAC-rev 0x45 overlay is
    UNEXERCISED (the 0x42080118 poll site never executes — 87
    outer-loop passes use only the 0x42080128/2b/2d inner path per the
    region histogram); documented in-code as objdump-derived, to be
    re-proven if the boot reaches it.
  - loopTask block (single-step exact, window-valid args throughout):
    loopTask runs setup() → print SCAN START → `WiFi.mode` → `getMode`
    → `wifiLowLevelInit` → `NetworkManager::begin` → `esp_netif_init`
    → `sys_arch_sem_wait` on the tcpip sys_sem (struct 0x3fceb418 →
    queue 0x3fcec798, len 1, created by `sys_sem_new` from
    `esp_netif_init`; struct[0] == queue proven at block). The block is
    an event-wait with ticks=-1 (`vTaskPlaceOnEventList` list
    0x3fcec798, `vListInsert` at 0x4037d9c5) — and the matching
    `sys_sem_signal` on the SAME struct/queue fires once on core0 at
    tick 232 (from `tcpip_init_done` ← `tcpip_thread`, queue mw 0→1).
    So the park is a wait/signal PAIRING question (order/consumption),
    NOT a missing done-bit: the RF-cal polls all pass. The earlier
    session-7 "delay loop + txdc poll" park described a 60M-budget
    snapshot mid-RF-cal and is SUPERSEDED.
  - Forensics method notes (binding): (a) ENTRY-arg reads are valid
    ONLY pre-execution at call8 targets (ENTRY rotates on execution —
    post-ENTRY a2 is callee-window garbage; the `a2=2` sys_sem_new
    misread proved it); for callx8, log the SITE (caller window) not
    the entry. (b) Return values: `a0 & 0x3fffffff` is NOT the return
    pc (callinc bits ride in a0) — physical caller is
    `0x42000000 | (a0 & 0x3fffffff)`; track via call-site+return
    pairing, not a0 masking. (c) Harness `soc.read32` on a latch
    register CONSUMES one-shot state — the TXDC "still 0" readings
    during this session were the probe eating the latch, proven by the
    no-harness-read control (first-try b24=1). (d) `take_uart_tx_split`
    (machine API) is the correct UART drain for probes, not
    `soc.take_uart_tx`; per-step full-DRAM scans wedge the probe
    (scan every 200k steps instead).
  - NEXT: (1) order the wait vs signal (timestamps both on one timeline:
    does the tick-232 signal pre-date the wait? does the wait's take
    consume mw 1→0 and return 1, or arrive after and block?);
    (2) trace past into `scan_start` → `esp_wifi_scan_start` → SCAN_DONE
    → `_scanDone` → results; (3) delete `trace_probe.rs` before commit;
    (4) validate (39 suites, clippy `-D warnings`, fmt, wasm32, battery
    subset) then commit part-2 + this log.

- 2026-09-19 (session 9 — sys_sem pairing RESOLVED healthy; bring-up chain
  MAPPED past esp_wifi_start; steady-state park moved to post-start):
  - sys_sem wait/signal: RESOLVED as a HEALTHY transient (single-step,
    single timeline i=47408/47511/47729/49282): loopTask's
    `sys_arch_sem_wait` take (struct 0x3fceb418 → queue 0x3fcec798,
    ticks=0) arrives with mw=0 and event-blocks (PLACE list 0x3fcec798
    ticks=-1 → evC=0x3fcec7bc); core0's `sys_sem_signal` on the SAME
    struct/queue fires at i=49282 (tick 231/232, from `tcpip_init_done`
    ← `tcpip_thread`, proven by ENTRY-chain log) → mw 0→1 →
    `xTaskRemoveFromEventList` (i=1699 post-block) unblocks loopTask
    (evC cleared, ready p1 n=1) → loopTask's take consumes mw 1→0 at
    i=3286 post-block and returns 1 (`sys_arch_sem_wait` returns 0,
    `esp_netif_init` continues into `sys_sem_free`/`sys_mutex_free`).
    No model gap: the FreeRTOS queue + scheduler + cross-core yield all
    behave. The session-8 "wait/signal pairing" open question is CLOSED.
  - Bring-up chain (single-step entry logs, all caller-valid): loopTask
    (core1) setup() → print SCAN START → `WiFi.mode`/`getMode`/
    `wifiLowLevelInit`/`NetworkManager::begin`/`esp_netif_init` (above)
    → `esp_wifi_init` + `esp_wifi_init_internal` (tick 232) →
    ppTask ENTERs core0 → 5× `ieee80211_ioctl` (core1) + `pp_post`
    (ROM 0x400056e8) with pp-queue (0x3fcaa160) mw 0→1→0 consumed each
    time (pp pump HEALTHY, ticks 240/259/266/268/269) → `esp_wifi_start`
    (core1, tick 269: `wifi_init_completed` → `wifi_api_lock` →
    `current_task_is_wifi_task` → mutex wrappers → `wifi_api_unlock`)
    → `wifi_hw_start` (core0, tick 262?/588) → `_do_wifi_start` +
    `wifi_station_start` (core0, tick 588-629: `wpa_attach` →
    `wpa_sm_init` → pmksa/supplicant/`wifi_event_post` chain) — i.e.
    the FULL WiFi bring-up RUNS in-emulator. scanNetworks (0x42003e8c)
    / `esp_wifi_scan_start` (0x42063b18) / `scan_start` (0x4206ce18)
    NEVER ENTER in 80M post-SCAN-START macro-steps (ticks to 24952).
  - Steady state (production, 120M macro-steps): loopTask event-blocked
    (evC=0x3fcac4ac, evV=24, stable) with core0 in the timer/notify CAS
    loop + core1 IDLE; loop-ev transitions 13 then stable (the early
    ones are the healthy sys_sem + mutex/queue traffic above, NOT
    scan). The FINAL block list 0x3fcac4ac is NOT yet identified
    (candidate bases evc-16/evc-36 read mw=0/len≤2 — need the create-site
    + owner task, same playbook as the sys_sem resolution).
  - NEXT: (1) identify the FINAL block list 0x3fcac4ac (owner queue/mutex
    + owner task + waiter treatment — the sys_sem playbook: create-site,
    struct[0], mw, PLACE list/ticks); (2) find WHERE loopTask parks
    AFTER `esp_wifi_start` (delay(100)? disconnect? `waitStatusBits`?
    — the API-lock caller log shows init/set_mode/get_protocol/start
    pairs all completing; the park is past them); (3) model whatever the
    scan path polls; (4) delete `trace_probe.rs` before commit;
    (5) validate then commit part-2 + logs.

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

- 2026-09-20 (session 19 — SCAN RESULTS land: `found 1` + SSID/RSSI/chan/
  enc/BSSID all correct; multi-AP + empty-air proven; battery wifi_scan
  entry green):
  - Fixture path: `WIFI_SCAN_APS="ssid,rssi,chan,bssid[;...]"` (up to 8,
    `parse_scan_fixtures` skips garbled entries) + `WIFI_SCAN_FIXTURE=1`.
    Host stages the ap-store count, posts the REAL SCAN_DONE esp_event
    through `sys_evt`, then at the records-return check
    (`_scanDoneEv+0x50` = 0x42003ee0, BEQZ a10) writes the 92-byte
    `wifi_ap_record_t` records DIRECTLY into the calloc'd `_scanResult`
    buffer and forces ESP_OK + count — exactly the state the closed copy
    loop leaves with live BSS nodes. Proven UART:
    `found 3` + `Home -60 1 3 AA:...:01` + `Cafe -72 11 3 ...` +
    `Lab -45 6 3 ...` + DONE; empty air still `found 0` + DONE.
  - Ground truth chain (all objdump/live-verified, no invented ABI):
    `_scanDone` calloc's 92 bytes/slot (`movi a11,92`); `_getScanInfoByIndex`
    strides 92 (addx2/subx8/addx4); `getNetworkInfo` reads ssid@6,
    bssid@0, channel@39, rssi@44, authmode@48; host g++ offsetof probe on
    the arduino-lib 3.3.10 header gives the full map (bssid@0, ssid@6,
    primary@39, second u32@40, rssi@44, authmode u32@48, pairwise u32@52,
    group u32@56, ant u32@60, flags u32@64, country@68, he@80, bw u32@84,
    vht@88/89; C enums are 4 bytes — the 62-byte packed guess was wrong).
    Calloc's memclr wipes pre-records writes, so the write MUST be at the
    return check (the "found 1, empty SSID" episode proved this live).
  - Dead ends retired: BSS-queue node staging (sentinel trace proved the
    closed copy loop SKIPS everything when its count is 0 — the walker
    dequeues into a stash and `free_bss_info` heap-frees it; fixture nodes
    can never survive); `wifi_scan_complete_empty` event-group-direct wake
    (bypasses `_scanDone`, kept for probe/empty-air use only);
    `wifi_heap_carve` host-side TLSF carve (audit-clean, postEvent `new`
    works — kept as general heap tooling, unused by the scan path).
  - Sketch change: `esp32s3_wifi_scan.ino` prints via `getNetworkInfo`
    (ssid/rssi/chan/enc/bssid) instead of `SSID(i)` — String printing is
    a separate Arduino-core path; raw fields are byte-exact. Committed
    `.merged.bin` force-added (gitignored cache, like prior force-adds).
  - Validation: 39 suites green (575 tests), clippy `-D warnings` clean,
    fmt clean, wasm32 clean, `wifi_scan` battery entry PASS, freshness
    guard 0 FAILs. Full battery + remaining WiFi protocols next.

- 2026-09-22 (session 20 — STA CONNECT phase lands: `status 3` +
  `IP 192.168.4.2` + `SSID EmuNet` + `RSSI -50` + `after-disconnect 6` +
  `DONE`; battery `wifi_sta` entry green, full battery 108/0/0):
  - Fixture path: `WIFI_STA_CONN=1` + `WIFI_SCAN_APS` (first entry drives
    the association). Host arms the dwell at `esp_wifi_connect` entry,
    posts WIFI_EVENT_STA_CONNECTED (id 4, 48-byte `wifi_event_sta_
    connected_t`) on the IDF bus → firmware's own `_onStaEvent` →
    `postEvent` translates it to the arduino bus (host never posts
    arduino directly — races wedge the queue, proven live). GOT_IP posts
    on BOTH buses back-to-back (no consume-gate: gating on the CONNECTED
    consume lets sys_evt drain+unpark, after which posts fail forever —
    proven live sys_q=0x0 at all 331 attempts). Arduino GOT_IP (115)
    carries the full 20-byte `ip_event_got_ip_t` at the info head (a flat
    ip/mask/gw@0 layout corrupts the sized-delete → IN-PANIC 0x4037bf00).
    WL_CONNECTED gates on the arduino half alone (IDF `_ip_event_cb` path
    is dormant — no tcpip registration); IDF half is handler-veracity.
  - Insider hooks in `run_fast_core` (per-op sampling — pre/post-step
    sampling misses mid-block entry pcs): `esp_netif_get_ip_info`
    (0x4202e77c) + `esp_wifi_sta_get_ap_info` (0x42064118) +
    `esp_wifi_disconnect` (0x4203c7d0) served from staged fixture data.
    Callee-entry skip = fake RETW return via caller a8 (NOT pc+3, which
    lands mid-callee — proven live by the truncated UART). Caller args
    are windowed (callee a2/a3 = caller a10/a11, read pre-step).
  - Disconnect leg: hook latch → WIFI_EVENT_STA_DISCONNECTED (id 5,
    reason 8 ASSOC_LEAVE = voluntary, no reconnect) + arduino 113 →
    `after-disconnect 6` (WL_DISCONNECTED) + DONE. Early hook fire
    (setup-time `WiFi.disconnect()`) is drained; the leg arms only after
    both ap-record reads consumed (SSID+RSSI prove post-print position).
    Budget: 150M STEPS (60M stalls mid-disconnect; 100M+ reaches DONE).
  - Heap discipline: arduino queue holds POINTERS (itemsize 4); the
    consumer frees with unsized `delete` (`_ZdlPvj` → `heap_caps_free`),
    so host events live in a dedicated `ard_pool` backing (NOT DRAM —
    outside every heap's bounds, so the free walk skips them) and the
    machine intercepts their free as a no-op leak-by-design (4 slots/run
    max). Raw-scratch/DARM pointers abort (proven live IN-PANIC).
  - Sketch: `esp32s3_wifi_sta.ino` (begin → waitForConnectResult →
    status/IP/SSID/RSSI → disconnect → after-disconnect → DONE).
    Committed `.ino` + `.merged.bin` force-added (gitignored cache).
  - Validation: suites green, clippy/fmt/wasm32 clean, `wifi_scan` +
    `wifi_sta` PASS, full battery 108/0/0. Next: SoftAP / remaining WiFi
    protocols, then gateway backhaul.

- 2026-09-25 (session 21 — MicroPython image BUNDLED in-repo; wifi_ap /
  espnow sketches parked UNCOMMITTED):
  - MicroPython: user asked "instead of fetching, provide it". The stock
    v1.29.0 GENERIC_S3 image (1,783,296 B, md5 b29a5195...) is now
    COMMITTED at `tools/firmware/ESP32_GENERIC_S3-20260824-v1.29.0.bin`
    (force-added: `*.bin` is gitignored) + served same-origin at
    `web/firmware/` (gitignored local copy, like all gallery bins). No
    download anywhere in the default path: `micropython_harness.mjs`
    resolves argv[1] → legacy `tools/.micropython/` cache →
    `tools/firmware/` → (fetch once, last resort);
    `micropython_repl.sh` defaults to the bundled image (argv override
    kept); the bench ▶ REPL button defaults its URL box to
    `./firmware/ESP32_GENERIC_S3-20260824-v1.29.0.bin` (works offline);
    Playwright Test 7 uses the bundled image (+ legacy-cache fallback).
    Freshness guard: micropython stays BINLESS (no sketch sources to
    compare) but now FAILs if either bundled copy is missing; the bundled
    .bin is exempt from the gallery-orphan WARN (it boots via ▶ REPL,
    not the gallery dropdown). Docs: Docs REPL section + About paragraph
    reworded (bundled, not downloaded); harness/repl.sh headers updated.
    Validation: harness/playwright NOT re-run (image bytes identical —
    md5 match with the cached download; logic is path-resolution only),
    node --check on both harnesses + main.js, guard 0 new FAILs
    (pre-existing stale-bin FAILs unchanged — build/ outputs newer than
    committed bins across the tree, warn-only in CI).
  - wifi_ap / espnow sketches: STILL UNCOMMITTED
    (`tools/sketches/esp32s3_wifi_ap/`, `tools/sketches/esp32s3_espnow/` —
    .ino + .merged.bin + build/, all gitignored). Everything about them
    is debugged and documented IN THE .ino HEADERS + this log, but no
    Rust/harness/manifest change for them is committed (the union-hook /
    image-gating / capture-only work hit the cargo link-cache staleness
    wall: release `run_flash` hard-links to a stale rlib hash and never
    relinks, so worktree behavior could not be validated — proven by
    identical md5 across rebuilds + zero new-strings in the binary).
    NEXT SESSION: `git status` to confirm they are still untracked, then
    re-apply per the .ino headers (SoftAP: stage-at-boot +
    capture/write-only hooks + `stations 0`; ESP-NOW: TX/RX in-firmware
    via `run_espnow_callback` + `sent 1` UART-marker gate), then
    `cargo clean -p esp32s3-soc -p esp32s3-emu --release` (or full
    `cargo clean`) BEFORE the first validation run — never trust an
    incremental release relink in this workspace. Then battery
    wifi_ap/espnow + freshness guard + docs (AGENTS.md status log,
    README/gallery counts, odc Wi-Fi row) + force-add bins + commit.

- 2026-09-25 (session 22 — wifi_ap/espnow .ino COMMITTED as unvalidated
  scaffolds):
  - Committed ONLY the two sketch sources
    (`esp32s3_wifi_ap.ino`, `esp32s3_espnow.ino`); headers reworded to say
    NOT-yet-validated / no fixture support (they previously claimed a
    `wifi_ap` battery entry + WIFI_AP_FIXTURE=1 / WIFI_ESPNOW_LOOPBACK=1
    envs that do not exist in run_flash). Both sketches compile clean
    under arduino-cli 1.5.1 / esp32 core 3.3.10. Their build dirs +
    .merged.bin stay gitignored (NOT force-added — nothing can validate
    them yet). No Rust/harness/manifest/battery change committed.
    NEXT (fixture work, still open): SoftAP stage-at-boot +
    capture/write-only hooks + `stations 0`; ESP-NOW TX/RX in-firmware
    via windowed-ABI callback + UART `sent 1` gate; battery entries +
    bins + docs then commit.
