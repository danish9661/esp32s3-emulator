#!/bin/sh
# Optimized web-bundle build (speed item 5).
#
# Applies wasm-friendly codegen flags, then builds the web target into
# web/pkg/ (gitignored, local-only — CI rebuilds into a temp dir instead,
# see tools/web_bundle_check.mjs).
#
# Flags:
#   -C target-feature=+bulk-memory,+mutable-globals: lets LLVM emit bulk
#   memory ops for the filling/copying the emulator does (SRAM/flash/PSRAM
#   backing stores, console drains). Recent rustc enables bulk-memory by
#   default for wasm32-unknown-unknown; passing it explicitly pins the
#   contract if the default ever changes.
#   wasm-opt -O3 (binaryen, if installed): post-link peephole pass.
#   Absent here (no wasm-opt on PATH) the build still proceeds — wasm-pack
#   + --release (opt-level=3, lto, codegen-units=1 per workspace Cargo.toml)
#   is already ~all of the gain; wasm-opt adds the last ~10%.
#
# PGO stays native-only (profiles don't carry to wasm — browser keeps this
# default build); see tools/pgo.sh.
set -e
ROOT="$(dirname "$0")/.."
cd "$ROOT"
export RUSTFLAGS="${RUSTFLAGS-} -C target-feature=+bulk-memory,+mutable-globals"
wasm-pack build crates/wasm-bridge --target web --out-dir ../../web/pkg
if command -v wasm-opt >/dev/null 2>&1; then
  wasm-opt -O3 --closed-world web/pkg/wasm_bridge_bg.wasm -o web/pkg/wasm_bridge_bg.wasm
  echo "wasm-opt -O3 applied"
else
  echo "wasm-opt not on PATH — skipped (install binaryen for the last ~10%)"
fi
