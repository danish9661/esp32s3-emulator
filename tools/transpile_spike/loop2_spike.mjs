// SPIKE Stage 2b runner: JS-seeded count (opaque), correctness + timing.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_loop2.wasm');
const { instance } = await WebAssembly.instantiate(bytes, {});
const mem = new Uint32Array(instance.exports.mem.buffer, 0, 4);
const SEED = 999983;
let got = 0n;
for (let r = 0; r < 5; r++) {
  mem[0] = SEED;
  const t0 = performance.now();
  got = instance.exports.run();
  const dt = (performance.now() - t0) / 1000;
  if (r === 4) {
    const ops = 4 * SEED; // load+add+add+br per iter (4 wasm ops)
    console.log(`LOOP2 run=${got} wall=${dt.toFixed(3)}s eff=${(ops / dt / 1e6).toFixed(0)} MIPS ${got === BigInt(SEED) ? 'LOOP2 PASS' : 'LOOP2 FAIL'}`);
    if (got !== BigInt(SEED)) process.exit(1);
  }
}
