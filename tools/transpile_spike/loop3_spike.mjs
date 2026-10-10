// SPIKE Stage 2c runner: JS-seeded streaming sum (unfoldable) + timing.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_loop3.wasm');
const { instance } = await WebAssembly.instantiate(bytes, {});
const mem = new Uint32Array(instance.exports.mem.buffer, 0, 16384);
let seed = 0x12345678, want = 0n;
for (let i = 0; i < 16384; i++) {
  seed = (Math.imul(seed, 1103515245) + 12345) >>> 0;
  mem[i] = seed >>> 0;
  want = (want + BigInt(seed >>> 0)) & 0xffffffffn; // i32 wraparound
}
for (let r = 0; r < 5; r++) {
  const t0 = performance.now();
  const got = instance.exports.run();
  const dt = (performance.now() - t0) / 1000;
  if (r === 4) {
    const mb = (16384 * 4) / 1048576;
    console.log(`LOOP3 sum=${got} want=${want} ${(mb / dt).toFixed(0)} MB/s ${got === want ? 'LOOP3 PASS' : 'LOOP3 FAIL'}`);
    if (got !== want) process.exit(1);
  }
}
