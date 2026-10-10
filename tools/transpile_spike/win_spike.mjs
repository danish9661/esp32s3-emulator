// SPIKE Stage 3b runner: window state + AR file + jump scratch.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_win.wasm');
const { instance } = await WebAssembly.instantiate(bytes, {});
const got = instance.exports.run();
const mem = new Uint32Array(instance.exports.mem.buffer, 0, 257);
const wb = instance.exports.wb.value, ws = instance.exports.ws.value, ps = instance.exports.ps.value;
const want = (0xE4n << 32n) | 17n;
const ok = got === want && mem[4] === 0x40000103 && mem[5] === 0xE4 && mem[6] === 17
  && mem[256] === 0x40000103 && wb === 0 && ws === 1 && ps === 0x50000;
console.log(`WINSPIKE run=${got} m4=${mem[4].toString(16)} m5=${mem[5].toString(16)} m6=${mem[6]} m256=${mem[256].toString(16)} wb=${wb} ws=${ws} ps=${ps.toString(16)} ${ok ? 'WINSPIKE PASS' : 'WINSPIKE FAIL'}`);
if (!ok) process.exit(1);
