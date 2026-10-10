// Auto-driver runner: trap/mmio report + conditional full-state compare.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_static.wasm');
const rep = JSON.parse(readFileSync('/tmp/opencode/spike_static.json', 'utf8'));
const traps = [];
const { instance } = await WebAssembly.instantiate(bytes, {
  env: {
    trap: (pc) => { traps.push(pc >>> 0); },
    soc_read32: () => 0,
    soc_write32: () => {},
  },
});
instance.exports.run();
console.log(`AUTO covered=${rep.covered} mmio=${rep.mmio} runtime-traps=${traps.length}`);
if (rep.traps.length) console.log('build-traps:', rep.traps.slice(0, 12).map(x => x).join(' '));
if (traps.length) { console.log('RUNTIME TRAP (unexpected)'); process.exit(1); }
if (rep.mmio === 0 && rep.traps.length === 0) {
  const mem = new Uint32Array(instance.exports.mem.buffer);
  let ok = true;
  const wb = instance.exports.wb.value;
  if (wb !== rep.wb1) { console.log(`WB MISMATCH: got ${wb} want ${rep.wb1}`); ok = false; }
  const laws = (r) => ((wb * 4 + r) & 63);
  for (let r = 0; r < 16; r++) {
    const want = rep.regs1[r] >>> 0, got = mem[laws(r)];
    if (got !== want) { console.log(`REG MISMATCH a${r}`); ok = false; }
  }
  for (const [a, before, after] of rep.writes) {
    const off = ((a - 0x3FC80000) >>> 2) + 64;
    if (mem[off] !== (after >>> 0)) { console.log(`MEM MISMATCH ${a.toString(16)}`); ok = false; }
  }
  for (const [nm, want] of [['ws', rep.ws1], ['ps', rep.ps1]]) {
    if ((instance.exports[nm].value >>> 0) !== (want >>> 0)) { console.log(`${nm} MISMATCH`); ok = false; }
  }
  console.log(ok ? 'AUTO STATE PASS' : 'AUTO STATE FAIL');
  if (!ok) process.exit(1);
} else {
  console.log('AUTO STATE SKIPPED (mmio/traps present)');
}
