// GDB-stub validation harness (battery NODE entry, mirrors the
// virtual-device/camcap harnesses): builds gdbstub if stale, boots the
// hello sketch under it, and drives the RSP protocol directly over TCP:
// init stop, qSupported/threads, g/p/P regs, m mem, Z0+c breakpoint,
// single-step advance, z0, Ctrl-C interrupt, qXfer target.xml.
// Prints GDB HARNESS PASS on success (exit 0), else FAIL (exit 1).
import { spawnSync, spawn } from "node:child_process";
import net from "node:net";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const BIN = path.join(ROOT, "target/release/examples/gdbstub");
const FW = path.resolve(ROOT, process.argv[2] || "tools/sketches/esp32s3_hello/esp32s3_hello.merged.bin");
const PORT = 3339;

function needBuild() {
  try {
    const b = fs.statSync(BIN).mtimeMs;
    for (const f of ["crates/esp32s3-emu/examples/gdbstub.rs", "crates/esp32s3-emu/src/machine.rs"]) {
      if (fs.statSync(path.join(ROOT, f)).mtimeMs > b) return true;
    }
    return false;
  } catch {
    return true;
  }
}
if (needBuild()) {
  console.log("[gdb] (re)building gdbstub...");
  const r = spawnSync("cargo", ["build", "--release", "--example", "gdbstub", "-p", "esp32s3-emu"], { cwd: ROOT, stdio: "inherit" });
  if (r.status !== 0) {
    console.log("GDB HARNESS FAIL (build)");
    process.exit(1);
  }
}

const child = spawn(BIN, [FW, String(PORT)], { stdio: "ignore" });
const fail = (why) => {
  console.log(`GDB HARNESS FAIL (${why})`);
  try { child.kill(); } catch {}
  process.exit(1);
};
process.on("exit", () => { try { child.kill(); } catch {} });

const sock = new net.Socket();
await new Promise((res, rej) => {
  const t = setTimeout(() => rej(new Error("connect timeout")), 10000);
  const tryConnect = () => {
    sock.connect(PORT, "127.0.0.1");
    sock.once("connect", () => { clearTimeout(t); res(); });
    sock.once("error", () => setTimeout(tryConnect, 200));
  };
  tryConnect();
}).catch((e) => fail(e.message));

let rx = Buffer.alloc(0);
sock.on("data", (d) => { rx = Buffer.concat([rx, d]); });
function csum(b) {
  let s = 0;
  for (const x of b) s = (s + x) & 0xff;
  return s;
}
const hx = (v) => (v < 10 ? 48 + v : 87 + v);
async function recvPacket() {
  for (;;) {
    const i = rx.indexOf(0x24); // '$'
    if (i >= 0) {
      const j = rx.indexOf(0x23, i); // '#'
      if (j >= 0 && rx.length >= j + 3) {
        const body = rx.subarray(i + 1, j);
        rx = rx.subarray(j + 3);
        sock.write("+"); // ack every reply (proper RSP)
        return body;
      }
    }
    await new Promise((r) => setTimeout(r, 10));
  }
}
async function cmd(body) {
  const b = Buffer.from(body);
  const c = csum(b);
  sock.write(Buffer.concat([Buffer.from("$"), b, Buffer.from("#"), Buffer.from([hx(c >> 4), hx(c & 15)])]));
  // Consume the ack ('+'); tolerate a pipelined packet start instead.
  return recvPacket();
}
// Drain acks interleaved with packets: our recvPacket skips non-'$' bytes,
// and the stub tolerates missing acks, so just proceed.
const init = await recvPacket();
if (init.toString() !== "S05") fail(`init ${init}`);

const q = await cmd("qSupported:multiprocess+");
if (!q.includes("PacketSize")) fail("qSupported");
if ((await cmd("Hc-1")).toString() !== "OK") fail("Hc");
if ((await cmd("Hg0")).toString() !== "OK") fail("Hg");
if ((await cmd("qfThreadInfo")).toString() !== "m1") fail("threads");
const g = await cmd("g");
if (g.length !== 19 * 8) fail(`g len ${g.length}`);
const pc = parseInt(g.subarray(16 * 8, 17 * 8).toString(), 16);
const mem = await cmd(`m${pc.toString(16)},10`);
if (mem.length !== 32) fail(`m len ${mem.length}`);
if ((await cmd(`Z0,${pc.toString(16)},4`)).toString() !== "OK") fail("Z0");
if ((await cmd("c")).toString() !== "S05") fail("break");
const p2 = await cmd("p2");
if (p2.length !== 8) fail("p2");
if ((await cmd("P2=12345678")).toString() !== "OK") fail("P2");
if ((await cmd("p2")).toString() !== "12345678") fail("p2 roundtrip");
const pc0 = parseInt((await cmd("p10")).toString(), 16);
if ((await cmd("s")).toString() !== "S05") fail("s");
const pc1 = parseInt((await cmd("p10")).toString(), 16);
if (pc1 === pc0) fail("step didn't advance");
if ((await cmd(`z0,${pc.toString(16)},4`)).toString() !== "OK") fail("z0");
// Free-run then Ctrl-C interrupt.
{
  const b = Buffer.from("c");
  const c = csum(b);
  sock.write(Buffer.concat([Buffer.from("$"), b, Buffer.from("#"), Buffer.from([hx(c >> 4), hx(c & 15)])]));
}
await new Promise((r) => setTimeout(r, 1000));
sock.write(Buffer.from([0x03]));
const intr = await recvPacket();
if (intr.toString() !== "S02" && intr.toString() !== "S05") fail(`intr ${intr}`);
const xfer = await cmd("qXfer:features:read:target.xml:0,400");
if (!xfer.includes("a15") || !xfer.includes("windowbase")) fail("target.xml");
console.log("GDB HARNESS PASS");
try { child.kill(); } catch {}
process.exit(0);
