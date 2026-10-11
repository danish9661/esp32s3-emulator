// Run-then-resume on the SAME instance: proves whether resume path diverges.
// Usage: node run_then_resume.mjs <module.wasm> <oracle.json> <segid>
import { readFileSync } from 'node:fs';
const [modPath, oraclePath, segArg] = process.argv.slice(2);
const segid = parseInt(segArg, 10);
const bytes = readFileSync(modPath);
const oracle = JSON.parse(readFileSync(oraclePath, 'utf8'));
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
// Phase 1: baked run().
instance.exports.run();
{
  const m = new Uint32Array(instance.exports.mem.buffer);
  console.log(`after run: reason=${m[0x310 / 4]} wb=${instance.exports.wb.value} (want ${oracle.wb1})`);
}
// Phase 2: manual presets + resume(segid).
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
  // Verify presets took.
  const chk = new Uint32Array(instance.exports.mem.buffer);
  console.log(`preset check: phys0=${(chk[0] >>> 0).toString(16)} wb=${instance.exports.wb.value} start=${instance.exports.start.value}`);
  instance.exports.resume();
  const m2 = new Uint32Array(instance.exports.mem.buffer);
  console.log(`after resume: reason=${m2[0x310 / 4]} traps=${traps.length} wb=${instance.exports.wb.value} (want ${oracle.wb1})`);
}
