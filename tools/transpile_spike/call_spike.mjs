// SPIKE Stage 3a runner: link write + shared-AR call + return value.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_call.wasm');
const { instance } = await WebAssembly.instantiate(bytes, {});
const got = instance.exports.run();
const mem = new Uint32Array(instance.exports.mem.buffer, 0, 4);
const ok = got === 4294967313n && mem[0] === 0xbeef00 && mem[2] === 17 && mem[3] === 1;
console.log(`CALLSPIKE run=${got} a0=${mem[0].toString(16)} a2=${mem[2]} a3=${mem[3]} ${ok ? 'CALLSPIKE PASS' : 'CALLSPIKE FAIL'}`);
if (!ok) process.exit(1);
