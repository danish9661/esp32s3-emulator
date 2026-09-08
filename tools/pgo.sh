#!/bin/sh
# Profile-guided optimization for native run_flash builds.
#
# Measured 2026-09-03: +45% throughput (29.6 -> 43.0M insns/s, interleaved
# A/B medians, pinned P-cores) on hello-boot trained over hello + periph.
#
# Profiles are TARGET-SPECIFIC: native x86_64 only. Do NOT apply to wasm
# builds (different backend). Requires `llvm-profdata` in PATH. Retrain
# after changing hot paths (xtensa-core decode/exec, soc tick dispatch);
# a stale profile still builds and runs correctly, just less optimally.
#
# Usage: ./tools/pgo.sh   (leaves an optimized target/ + tools/pgo/merged.profdata)
set -eu
cd "$(dirname "$0")/.."

PROFDIR=tools/pgo
mkdir -p "$PROFDIR/raw"
# Path rules (both load-bearing, learned Sep 2026):
#  1. Registry deps (e.g. libm, added for the FPU) compile with rustc CWD
#     outside the workspace, so profile paths in RUSTFLAGS must be ABSOLUTE.
#  2. RUSTFLAGS is whitespace-split and the workspace path contains a
#     space, so the absolute path must ALSO be space-free -> stage outside
#     the tree. The committed artifact stays $PROFDIR/merged.profdata.
PROFSTAGE="${TMPDIR:-/tmp}/esp32s3-pgo"
mkdir -p "$PROFSTAGE/raw"

echo "[pgo] instrumented build..."
RUSTFLAGS="-Cprofile-generate=$PROFSTAGE/raw" cargo build --release -p esp32s3-emu --example run_flash

echo "[pgo] training (hello boot)..."
LLVM_PROFILE_FILE="$PROFSTAGE/raw/hello-%m.profraw" \
  target/release/examples/run_flash tools/sketches/esp32s3_hello/esp32s3_hello.merged.bin \
  > /tmp/pgo_train_hello.log 2>&1

echo "[pgo] training (periph: GPIO/ADC/FreeRTOS SMP)..."
ADC_INJECT_MV=825 LLVM_PROFILE_FILE="$PROFSTAGE/raw/periph-%m.profraw" \
  target/release/examples/run_flash tools/sketches/esp32s3_periph/esp32s3_periph.merged.bin \
  > /tmp/pgo_train_periph.log 2>&1

echo "[pgo] merging profiles..."
llvm-profdata merge -o "$PROFDIR/merged.profdata" "$PROFSTAGE"/raw/*.profraw
cp "$PROFDIR/merged.profdata" "$PROFSTAGE/merged.profdata"

echo "[pgo] optimized build..."
RUSTFLAGS="-Cprofile-use=$PROFSTAGE/merged.profdata" cargo build --release -p esp32s3-emu --example run_flash

echo "[pgo] verifying (hello must print boot OK)..."
target/release/examples/run_flash tools/sketches/esp32s3_hello/esp32s3_hello.merged.bin 2>&1 \
  | grep -q "boot OK" && echo "[pgo] verify OK"
