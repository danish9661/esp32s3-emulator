// SPIKE Stage 1 runner: return value + landed store word.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_mem.wasm');
const { instance } = await WebAssembly.instantiate(bytes, {});
const got = instance.exports.run();
const memw = new Uint32Array(instance.exports.mem.buffer, 0, 1)[0];
const ok = got === 2559n && memw === 0x8ff;
console.log(`MEMSPIKE run=${got} mem[0]=${memw.toString(16)} ${ok ? 'MEMSPIKE PASS' : 'MEMSPIKE FAIL'}`);
if (!ok) process.exit(1);
