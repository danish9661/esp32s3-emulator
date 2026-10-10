// Windowed differ runner: windowed regs + DRAM words + wb/ws/ps.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_windiff.wasm');
const exp = JSON.parse(readFileSync('/tmp/opencode/spike_windiff.json', 'utf8'));
const { instance } = await WebAssembly.instantiate(bytes, {});
instance.exports.run();
const mem = new Uint32Array(instance.exports.mem.buffer);
let ok = true;
const wb = instance.exports.wb.value;
if (wb !== exp.wb1) { console.log(`WB MISMATCH: got ${wb} want ${exp.wb1}`); ok = false; }
const laws = (r) => ((wb * 4 + r) & 63);
for (let r = 0; r < 16; r++) {
  const want = exp.regs1[r] >>> 0, got = mem[laws(r)];
  if (got !== want) { console.log(`REG MISMATCH a${r}: got ${got.toString(16)} want ${want.toString(16)}`); ok = false; }
}
for (const [a, before, after] of exp.writes) {
  const off = ((a - 0x3FC80000) >>> 2) + 64; // mirror base 256 bytes
  if (mem[off] !== (after >>> 0)) { console.log(`MEM MISMATCH ${a.toString(16)}`); ok = false; }
}
for (const [nm, want] of [['ws', exp.ws1], ['ps', exp.ps1]]) {
  const got = instance.exports[nm].value >>> 0;
  if (got !== (want >>> 0)) { console.log(`${nm.toUpperCase()} MISMATCH: got ${got.toString(16)} want ${(want >>> 0).toString(16)}`); ok = false; }
}
console.log(ok ? 'WINDIFF PASS' : 'WINDIFF FAIL');
if (!ok) process.exit(1);
