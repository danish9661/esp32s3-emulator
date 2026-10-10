// SPIKE Stage 4b runner: poll cadence both directions + trap signaling.
import { readFileSync } from 'node:fs';
const bytes = readFileSync('/tmp/opencode/spike_poll.wasm');
const { instance } = await WebAssembly.instantiate(bytes, {
  env: {
    poll_irq: () => pollCanned(),
    trap: (pc) => { trapLog.push(pc >>> 0); },
  },
});
let polls = 0, trapLog = [];
let canned = () => 0;
const pollCanned = () => { polls++; return canned(); };
// Run 1: polls always 0 -> completes 512 iters, 32 polls.
let got = instance.exports.run();
const want1 = 512n;
const ok1 = got === want1 && polls === 32 && trapLog.length === 0;
console.log(`POLL run1=${got} polls=${polls} traps=${trapLog.length} ${ok1 ? 'ok' : 'FAIL'}`);
// Run 2: nonzero at 3rd poll -> trap at iter 48.
polls = 0; trapLog = [];
let n = 0;
canned = () => (++n === 3 ? 1 : 0);
got = instance.exports.run();
const want2 = (0xBADn << 32n) | 48n;
const ok2 = got === want2 && polls === 3 && trapLog.length === 1 && trapLog[0] === 0x40000200;
console.log(`POLL run2=${got} polls=${polls} trap=${trapLog.map(x => x.toString(16))} ${ok2 ? 'ok' : 'FAIL'}`);
console.log(ok1 && ok2 ? 'POLLSPIKE PASS' : 'POLLSPIKE FAIL');
if (!(ok1 && ok2)) process.exit(1);
