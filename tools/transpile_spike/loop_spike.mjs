// SPIKE Stage 2 runner: correctness + effective MIPS (S2 gate >= 2x
// vs the measured 13 MIPS wasm-interpreter rate).
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_loop.wasm');
const { instance } = await WebAssembly.instantiate(bytes, {});
const run = instance.exports.run;
let got = run(); // warmup (V8 compile included in first call separately)
const t0 = performance.now();
got = run();
const t1 = performance.now();
const t2 = performance.now();
run(); run();
const t3 = performance.now();
const wall = (t3 - t2) / 2 / 1000; // best-of warmed runs
const ops = 3_000_000; // ~3 wasm ops/iter x 1M iters
const mips = ops / wall / 1e6;
const ok = got === 1000000n;
console.log(`LOOPSPIKE run=${got} wall=${wall.toFixed(3)}s eff=${mips.toFixed(0)} MIPS ${ok ? 'LOOPSPIKE PASS' : 'LOOPSPIKE FAIL'}`);
console.log(`S2 gate (>=2x of 13 MIPS interp): ${mips >= 26 ? 'HOLD (proceed S3)' : 'MISS (stop)'}`);
if (!ok) process.exit(1);
