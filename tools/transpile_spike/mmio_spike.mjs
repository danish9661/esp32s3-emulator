// SPIKE Stage 4a runner: MMIO write log + canned read + return pack.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_mmio.wasm');
const log = [];
const { instance } = await WebAssembly.instantiate(bytes, {
  env: {
    soc_write32: (addr, val) => { log.push([addr >>> 0, val]); },
    soc_read32: (addr) => { log.push(['r', addr >>> 0]); return 0x20; },
  },
});
const got = instance.exports.run();
const mem = new Uint32Array(instance.exports.mem.buffer, 0, 4);
const ok = got === 0x2000000048n && mem[2] === 0x48 && mem[3] === 0x20
  && log.length === 2 && log[0][0] === 0x60000000 && log[0][1] === 0x48 && log[1][1] === 0x60000004;
console.log(`MMIOSPIKE run=${got} mem2=${mem[2]} mem3=${mem[3]} log=${JSON.stringify(log)} ${ok ? 'MMIOSPIKE PASS' : 'MMIOSPIKE FAIL'}`);
if (!ok) process.exit(1);
