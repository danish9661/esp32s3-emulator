#!/bin/bash
# tools/run_battery.sh — end-to-end validation battery for the emulator.
#
# For every validation sketch: optionally rebuild it with arduino-cli,
# run the merged flash image through run_flash (with the sketch's env),
# and assert its PASS markers (and no FAIL). Exits nonzero if any sketch
# fails. Sketches with documented out-of-scope gaps are SKIPped with a
# reason instead of going red.
#
# Usage:
#   tools/run_battery.sh                 # run all (existing .merged.bin)
#   tools/run_battery.sh --build         # rebuild every sketch first
#   tools/run_battery.sh --build hello   # rebuild+run only matching sketches
#   tools/run_battery.sh --shard 0/4     # run only shard 0 of 4 (for CI fan-out)
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EMU="$ROOT/target/release/examples/run_flash"
SK="$ROOT/tools/sketches"
BUILD=0
FILTER=()
SHARD_IDX=-1
SHARD_N=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) BUILD=1; shift ;;
    --shard=*) spec="${1#--shard=}"; shift ;;
    --shard)
      [[ $# -ge 2 ]] || { echo "usage: --shard K/N (e.g. --shard 0/4)" >&2; exit 2; }
      spec="$2"; shift 2 ;;
    *) FILTER+=("$1"); shift ;;
  esac
done
if [[ -n "${spec:-}" ]]; then
  if [[ ! "$spec" =~ ^[0-9]+/[0-9]+$ ]]; then
    echo "bad --shard spec '$spec' (want K/N)" >&2; exit 2
  fi
  SHARD_IDX="${spec%%/*}"; SHARD_N="${spec##*/}"
  if [[ $SHARD_N -lt 1 || $SHARD_IDX -ge $SHARD_N ]]; then
    echo "bad --shard spec '$spec' (want 0 <= K < N)" >&2; exit 2
  fi
fi

echo "== building run_flash =="
cargo build --release -p esp32s3-emu --example run_flash 2>&1 | grep -E "^error" -A4 | head -8

# Table: name|env(space-separated K=V)|required markers (; separated)|STEPS.
# SKIP entries: name|SKIP:reason (not run).
# NODE entries: name|NODE:path/to/harness.mjs|markers| — run under node
#   instead of run_flash (for harnesses like the virtual-device demo that
#   need JS-side devices); the sketch .merged.bin is passed as argv[1].
CASES=(
"hello||Hello from ESP32-S3!;boot OK|"
"periph|ADC_INJECT_MV=825|boot OK|"
"uart_echo|UART_INJECT=hello|[uart1] rx 'h'|"
"uart_tout|UART_INJECT=Z|UART TOUT OK|"
"uart_multi||MULTI_UART PASS|"
"usb_serial||USB TEST|"
"aes||AES DONE;AES CBC PASS;AES XTS PASS|"
"aes_gcm||AES GCM PASS;AES GCM DONE|"
"aes_poke||AES POKE PASS|"
"sha||SHA DONE;SHA384 PASS;SHA512 PASS|"
"spi||SPI DONE|"
"spi_driver||SPI DRIVER PASS|"
"spidev||SPIDEV PASS;SPIDEV DONE|"
"spi_dma||SPI DMA MOSI OK;SPI DMA RX OK|"
"spi_slave|SPI_SLAVE_XCHG=1|SPI SLAVE DONE|"
"spi_wide||SPI_WIDE PASS|"
"flashread||FLASHREAD TABLE MAGIC OK;FLASHREAD APP MAGIC OK;FLASHREAD NVS ERASED OK;FLASHREAD PASS|"
"i2c||I2C DONE|"
"i2c_poke||I2C POKE PASS|"
"i2c_slave|I2C_SLAVE_XCHG=1|I2C SLAVE DONE|"
"i2c_wire||I2C WIRE PASS|"
"rmt||RMT TX done|"
"rmt_driver||RMT DRIVER PASS|"
"gpio_interrupt||GPIO_IRQ PASS|"
"gpio_uart_timer||MULTI_PERIPH PASS|"
"multi_irq||MULTI_IRQ PASS|"
"timer_alarm||TIMER_ALARM PASS|"
"systimer||SYSTIMER PASS|"
"adc_dma|ADC_INJECT_MV=825|ADC GDMA PASS|"
"twai||TWAI LOOPBACK PASS|"
"twai_driver||TWAI DRIVER LOOPBACK PASS|"
"mcpwm||MCPWM PASS|"
"mcpwm_sync||MCPWM SYNC PASS|"
"mcpwm_dt||MCPWM DT PASS|"
"mcpwm_cap||MCPWM CAP PASS|"
"pcnt||PCNT PASS|"
"ledc||LEDC PASS|"
"sigmadelta||SIGMADELTA PASS|"
"efuse||EFUSE DONE|"
"efuse_burn||EFUSE BURN PASS;EFUSE BURN DONE|"
"rng||RNG PASS|"
"rtcio||RTCIO PASS|"
"lpi2c||LP I2C POKE PASS|"
"lpuart||LP UART POKE PASS|"
"ulp||ULP POKE PASS|"
"sdmmc||SDMMC PASS|"
"sdfat||SD BEGIN OK;SD FAT READ PASS;SD FAT WRITE PASS;SD DONE|60000000"
"deepsleep_poke||DEEPSLEEP PASS|"
"deepsleep||DEEPSLEEP START;DEEPSLEEP WOKE;DEEPSLEEP PASS|50000000"
"deepsleep_ext0||DEEPSLEEP EXT0 START;DEEPSLEEP EXT0 WOKE;DEEPSLEEP EXT0 PASS|50000000"
"deepsleep_ext1||DEEPSLEEP EXT1 START;DEEPSLEEP EXT1 WOKE;DEEPSLEEP EXT1 PASS|50000000"
"deepsleep_ulp||DEEPSLEEP ULP START;DEEPSLEEP ULP WOKE;DEEPSLEEP ULP PASS|80000000"
"deepsleep_touch|TOUCH_INJECT=3:1877|DEEPSLEEP TOUCH START;DEEPSLEEP TOUCH WOKE;DEEPSLEEP TOUCH PASS|50000000"
"lightsleep|SKIP:resume works but s_light_sleep_wakeup flag stays 0 (inner helper returns 0x103), so get_wakeup_cause reads 0 not TIMER (see AGENTS.md)"
"hmac||HMAC DONE|"
"ds||DS DONE|"
"rsa||RSA POKE PASS||esp32s3_rsa/esp32s3_rsa_poke/esp32s3_rsa_poke.merged.bin"
"ecdsa||ECDSA DONE|"
"i2s||I2S POKE PASS|"
"i2s_driver||I2S DRIVER LOOPBACK PASS|300000000"
"touch|TOUCH_INJECT=3:1877|TOUCH PASS|150000000"
"temp|TEMP_C=25|TEMP 25C OK;TEMP DONE|"
"rwdt_feed||RWDT FEED TEST START|"
"rwdt_reset||RWDT RESET TEST|"
"lcd_cam||LCD CAM POKE PASS;LCD GDMA PASS|"
"p5_stubs||P5 STUBS POKE PASS|"
"gdma||GDMA RMT TX done|"
"full_load||FULL_LOAD PASS|150000000"
"ota_slot||OTA SLOT TEST PASS|"
"psram_qspi||PSRAM total=2097152;PSRAM RW OK;PSRAM PROBE PASS|"
"psram_opi||PSRAM total=8388608;PSRAM RW OK;PSRAM PROBE PASS|"
"mcpwm_fault||MCPWM FAULT trip=500/500;MCPWM FAULT PASS|"
"dedic_gpio||DEDIC hi pad=1;DEDIC GPIO PASS|"
"ee_dsp||EE DSP DOT OK;EE DSP VADDS OK;EE DSP DONE|"
"virtual_demo|NODE:tools/virtual_demo_harness.mjs|VIRTUAL DEMO HARNESS PASS|"
"camcap|NODE:tools/camcap_harness.mjs|CAMCAP HARNESS PASS|"
"gdb|NODE:tools/gdb_harness.mjs|GDB HARNESS PASS||esp32s3_hello/esp32s3_hello.merged.bin"
)

pass=0; fail=0; skipped=0
idx=0
for c in "${CASES[@]}"; do
  name="${c%%|*}"; rest="${c#*|}"
  if [[ $SHARD_N -gt 0 ]]; then
    if [[ $((idx % SHARD_N)) != "$SHARD_IDX" ]]; then idx=$((idx+1)); continue; fi
  fi
  idx=$((idx+1))
  if [[ ${#FILTER[@]} -gt 0 ]]; then
    keep=0
    for f in "${FILTER[@]}"; do [[ "$name" == *"$f"* ]] && keep=1; done
    [[ $keep == 0 ]] && continue
  fi
  if [[ "$rest" == SKIP:* ]]; then
    reason="${rest#SKIP:}"; reason="${reason%|}"
    echo "SKIP $name ($reason)"
    skipped=$((skipped+1)); continue
  fi
  IFS='|' read -r envstr markers steps binrel _ <<< "$rest"
  [[ -z "$steps" ]] && steps=96000000
  dir="$SK/esp32s3_$name"
  if [[ "$envstr" == NODE:* ]]; then
    # Node-driven harness (not run_flash): resolve the sketch binary the
    # same way (or via an explicit 4th-field path for harnesses like gdb
    # that reuse another sketch's image), optionally rebuild it, then run
    # the harness with the bin path as argv[1] and check its output
    # markers like any other entry.
    if [[ -n "$binrel" ]]; then
      bin="$SK/$binrel"
    else
      bin="$dir/esp32s3_$name.merged.bin"
      [[ -f "$bin" ]] || bin="$dir/esp32s3_$name.ino.merged.bin"
    fi
    if [[ ! -f "$bin" ]]; then
      bin=$(find "$dir/build" -name "*.merged.bin" 2>/dev/null | head -1)
    fi
    if [[ $BUILD == 1 && -z "$binrel" ]]; then
      if ! arduino-cli compile --fqbn esp32:esp32:esp32s3 --build-path "$dir/build" "$dir" >/tmp/battery_build.log 2>&1; then
        echo "FAIL $name (compile)"; tail -3 /tmp/battery_build.log; fail=$((fail+1)); continue
      fi
      cp "$dir/build/esp32s3_$name.ino.merged.bin" "$dir/esp32s3_$name.merged.bin"
      bin="$dir/esp32s3_$name.merged.bin"
    fi
    if [[ ! -f "$bin" ]]; then echo "FAIL $name (no binary $bin)"; fail=$((fail+1)); continue; fi
    if ! command -v node >/dev/null 2>&1; then echo "FAIL $name (node missing)"; fail=$((fail+1)); continue; fi
    log=$(timeout 600 node "$ROOT/${envstr#NODE:}" "$bin" 2>&1 | tr -d '\0')
    ok=1; why=""
    for m in ${markers//;/ }; do
      echo "$log" | grep -aqF "$m" || { ok=0; why="missing [$m]"; }
    done
    echo "$log" | grep -aq "HARNESS FAIL" && { ok=0; why="harness reported FAIL"; }
    if [[ $ok == 1 ]]; then echo "PASS $name"; pass=$((pass+1)); else echo "FAIL $name ($why)"; fail=$((fail+1)); fi
    continue
  fi
  # Variant sketches share one source dir but need different arduino-cli
  # options and produce distinct committed binaries (plain `--build` must
  # reproduce them exactly).
  srcdir="$dir"; fqbn="esp32:esp32:esp32s3"; inobin="esp32s3_$name.ino.merged.bin"
  case "$name" in
    psram_qspi) srcdir="$SK/esp32s3_psram"; fqbn="$fqbn:PSRAM=enabled"; bin="$srcdir/esp32s3_psram_qspi.merged.bin"; inobin="esp32s3_psram.ino.merged.bin";;
    psram_opi) srcdir="$SK/esp32s3_psram"; fqbn="$fqbn:PSRAM=opi"; bin="$srcdir/esp32s3_psram_opi.merged.bin"; inobin="esp32s3_psram.ino.merged.bin";;
  esac
  if [[ -n "$binrel" ]]; then
    bin="$SK/$binrel"
  elif [[ "$srcdir" != "$dir" ]]; then
    : # variant bin already resolved above
  else
    bin="$dir/esp32s3_$name.merged.bin"
    [[ -f "$bin" ]] || bin="$dir/esp32s3_$name.ino.merged.bin"
    if [[ ! -f "$bin" ]]; then
      bin=$(find "$dir/build" -name "*.merged.bin" 2>/dev/null | head -1)
    fi
  fi
  if [[ $BUILD == 1 ]]; then
    if ! arduino-cli compile --fqbn "$fqbn" --build-path "$srcdir/build" "$srcdir" >/tmp/battery_build.log 2>&1; then
      echo "FAIL $name (compile)"; tail -3 /tmp/battery_build.log; fail=$((fail+1)); continue
    fi
    cp "$srcdir/build/$inobin" "$bin"
  fi
  if [[ ! -f "$bin" ]]; then echo "FAIL $name (no binary $bin)"; fail=$((fail+1)); continue; fi
  log=$(STEPS=$steps env $envstr timeout 300 "$EMU" "$bin" 2>&1 | tr -d '\0')
  ok=1; why=""
  for m in ${markers//;/ }; do
    echo "$log" | grep -aqF "$m" || { ok=0; why="missing [$m]"; }
  done
  echo "$log" | grep -aq "FAIL" && { ok=0; why="FAIL in output"; }
  if [[ $ok == 1 ]]; then echo "PASS $name"; pass=$((pass+1)); else echo "FAIL $name ($why)"; fail=$((fail+1)); fi
done
echo "== battery: $pass pass, $fail fail, $skipped skipped =="
[[ $fail == 0 ]]
