#!/bin/bash
# Per-func single-build + sim + vehicle for funcs that have oracles.
TS="/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike"
for pair in "4004e974:/tmp/opencode/funcE974.json" "400559cc:/tmp/opencode/func559cc.json"; do
  f="${pair%%:*}"; o="${pair##*:}"
  python3 "$TS/static_func.py" "/tmp/opencode/sf_$f.json" "$o" 2>&1 | grep -v '^wrote' | head -1
  python3 "$TS/sim.py" /tmp/opencode/spike_static.wasm 2>&1 | tail -1
  timeout 60 node "$TS/static_run.mjs" 2>&1 | grep -E 'AUTO STATE|MISMATCH' | head -4
  echo "--- $f done"
done
echo PERFUNC_DONE
