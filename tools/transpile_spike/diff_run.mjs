// S4c-1 runner: transpiled real-firmware block vs interpreter state.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_diff.wasm');
const exp = JSON.parse(readFileSync('/tmp/opencode/spike_diff.json', 'utf8'));
const { instance } = await WebAssembly.instantiate(bytes, {});
instance.exports.run();
const mem = new Uint32Array(instance.exports.mem.buffer, 0, 16);
let ok = true;
for (let r = 0; r < 16; r++) {
  const want = exp.regs1[r] >>> 0;
  if (mem[r] !== want) {
    console.log(`REG MISMATCH a${r}: got ${mem[r].toString(16)} want ${want.toString(16)}`);
    ok = false;
  }
}
for (const [a, before, after] of exp.writes) {
  const off = (a - 0x3FC80000) >>> 2;
  if (mem[off] !== (after >>> 0)) {
    console.log(`MEM MISMATCH ${a.toString(16)}: got ${mem[off].toString(16)} want ${(after >>> 0).toString(16)}`);
    ok = false;
  }
}
console.log(ok ? 'DIFF PASS (regs + mem identical to interpreter)' : 'DIFF FAIL');
if (!ok) process.exit(1);
