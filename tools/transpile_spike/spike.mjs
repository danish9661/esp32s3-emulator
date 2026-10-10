// SPIKE runner: instantiate the transpiled block, compare bit-exact.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_block.wasm');
const { instance } = await WebAssembly.instantiate(bytes, {});
const got = instance.exports.run();
const want = (96n << 32n) | 101n; // a5=96, a4=101
console.log(`SPIKE got=${got} want=${want} ${got === want ? 'SPIKE PASS' : 'SPIKE FAIL'}`);
if (got !== want) process.exit(1);
