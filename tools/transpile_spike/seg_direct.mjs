// Direct-seg driver v2: runs ONE seg from oracle entry state and compares
// against leaf_end (first oracle step OUTSIDE the transcribed set) — the
// same expectation static_func.py bakes into the report — plus MB_RET.
// Usage: node seg_direct.mjs <module.wasm> <sfunc.json> <oracle.json> <segid>
import { readFileSync } from 'node:fs';
const [modPath, sfuncPath, oraclePath, segArg] = process.argv.slice(2);
const segid = parseInt(segArg, 10);
const bytes = readFileSync(modPath);
const sfunc = JSON.parse(readFileSync(sfuncPath, 'utf8')).sfunc;
const oracle = JSON.parse(readFileSync(oraclePath, 'utf8'));
const staticPcs = new Set(sfunc.insns.map((s) => s.pc));
const leaf = oracle.steps.find((s) => !staticPcs.has(s.pc));
if (!leaf) { console.log('NO LEAF (window too small?)'); process.exit(1); }
console.log(`leaf_end: pc=0x${leaf.pc.toString(16)} wb=${leaf.wb} ws=0x${leaf.ws.toString(16)} ps=0x${leaf.ps.toString(16)}`);
const traps = [];
const { instance } = await WebAssembly.instantiate(bytes, {
  env: {
    trap: (pc) => { traps.push(pc >>> 0); },
    soc_read32: () => 0,
    soc_write32: () => {},
    soc_cas32: () => 0,
  },
});
const M_DRAM = 0x10000, M_IRAM = 0x90000, M_ROM = 0x110000;
{
  const m32 = new Uint32Array(instance.exports.mem.buffer);
  for (let i = 0; i < 64; i++) m32[i] = oracle.phys0[i] >>> 0;
  m32[0x300 / 4] = 0; m32[0x304 / 4] = 0; m32[0x308 / 4] = 0;
  for (const [off, arr] of [[M_DRAM, oracle.fulldram], [M_IRAM, oracle.fulliram], [M_ROM, oracle.fullrom]]) {
    for (let i = 0; i < arr.length; i++) m32[(off >>> 2) + i] = arr[i] >>> 0;
  }
  instance.exports.wb.value = oracle.wb0;
  instance.exports.ws.value = oracle.ws0;
  instance.exports.ps.value = oracle.ps0;
  m32[0x200000 / 4] = 0xFFFFFFFF;
  instance.exports.ssp.value = 4;
  instance.exports.start.value = segid;
  instance.exports.resume();
  let guard = 0;
  while (m32[0x310 / 4] === 2 && traps.length === 0 && guard++ < 1000) {
    const nxt = m32[0x31c / 4];
    if (nxt === 0xFFFFFFFF) break;
    instance.exports.start.value = nxt;
    instance.exports.resume();
  }
}
const m32 = new Uint32Array(instance.exports.mem.buffer);
const reason = m32[0x310 / 4];
const wb = instance.exports.wb.value;
let ok = true;
const chk = (nm, got, want) => {
  if ((got >>> 0) !== (want >>> 0)) { console.log(`${nm} MISMATCH: got 0x${(got >>> 0).toString(16)} want 0x${(want >>> 0).toString(16)}`); ok = false; }
};
console.log(`reason=${reason} traps=${traps.length}`);
chk('wb', wb, leaf.wb);
chk('ws', instance.exports.ws.value, leaf.ws);
chk('ps', instance.exports.ps.value, leaf.ps);
const laws = (r) => ((wb * 4 + r) & 63);
for (let r = 0; r < 16; r++) {
  const got = m32[laws(r)], want = leaf.regs[r] >>> 0;
  if (got !== want) { console.log(`REG MISMATCH a${r}: got 0x${(got >>> 0).toString(16)} want 0x${want.toString(16)}`); ok = false; }
}
// MB_RET must equal the leaf pc (interpreter-exact retw target).
chk('MB_RET', m32[0x30c / 4], leaf.pc);
if (traps.length) { console.log('RUNTIME TRAPS: ' + traps.map((t) => '0x' + t.toString(16)).join(' ')); ok = false; }
console.log(ok ? 'SEG DIRECT PASS' : 'SEG DIRECT FAIL');
process.exit(ok ? 0 : 1);
