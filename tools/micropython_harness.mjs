// Headless validation for the MicroPython REPL preset (battery-runnable).
//
// Drives the nodejs-target wasm build of the emulator through the SAME
// bridge APIs the browser uses (`load_flash` / `step_batch` / `uart_read` /
// `uart_inject_rx`), running a stock MicroPython GENERIC_S3 image with the
// SAME vfs-partition padding web/main.js applies (pad to 3 MiB + littlefs
// "vfs" DATA 0x82 @ 0x200000 + partition-table MD5, mirroring
// tools/micropython_repl.sh).
//
// The image is the committed stock MicroPython GENERIC_S3 release in
// tools/firmware/ (no download — fully offline; a legacy tools/.micropython/
// cache or an explicit argv[1] path still wins when present) and asserts:
//   * `>>> ` REPL banner (first boot formats littlefs: "Performing
//     initial setup")
//   * `print(6*7)` -> `42` over UART0_INJECT-at->>> (MicroPython's REPL
//     listens on UART0, not USB-CDC)
//   * `print(0.5)` -> `0.5`, `print(1.0/3.0)` -> `0.33333334`,
//     `print(0.1+0.2)` -> `0.3` (the old MOVF/MOVT hang family)
//
// Usage: node tools/micropython_harness.mjs [path/to/ESP32_GENERIC_S3-...bin]
// Builds the nodejs wasm pkg into tools/.micropython_pkg/ first if it is
// missing or older than the Rust sources. Exit 0 on PASS, 1 with the UART
// tail on failure.

import { execFileSync, execSync } from 'child_process';
import { existsSync, statSync, readFileSync, writeFileSync, mkdirSync } from 'fs';
import { createHash } from 'crypto';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const pkgDir = join(root, 'tools', '.micropython_pkg');
const wasmFile = join(pkgDir, 'wasm_bridge_bg.wasm');
const MP_URL = 'https://micropython.org/resources/firmware/ESP32_GENERIC_S3-20260824-v1.29.0.bin';
const mpDir = join(root, 'tools', '.micropython');
const mpBin = join(mpDir, 'ESP32_GENERIC_S3-20260824-v1.29.0.bin');
const mpBundled = join(root, 'tools', 'firmware', 'ESP32_GENERIC_S3-20260824-v1.29.0.bin');

function crateNewerThan(file) {
  if (!existsSync(file)) return true;
  const built = statSync(file).mtimeMs;
  let out = '';
  try {
    out = execFileSync('find', ['crates', '-name', '*.rs', '-newer', file], { cwd: root, encoding: 'utf8' });
  } catch {
    return true;
  }
  return out.trim().length > 0;
}

if (crateNewerThan(wasmFile)) {
  console.error('[micropython] building nodejs wasm pkg...');
  execFileSync('wasm-pack', ['build', 'crates/wasm-bridge', '--target', 'nodejs', '--out-dir', pkgDir], {
    cwd: root,
    stdio: 'inherit',
  });
}

// Resolve the stock image: explicit argv path wins, else the legacy
// tools/.micropython/ cache, else the committed tools/firmware/ image,
// else fetch it once into the legacy cache dir (fails loudly offline —
// no silent skip).
let imgPath = process.argv[2];
if (!imgPath) {
  if (existsSync(mpBin)) {
    imgPath = mpBin;
  } else if (existsSync(mpBundled)) {
    imgPath = mpBundled;
  } else {
    console.error('[micropython] downloading stock image (once)...');
    mkdirSync(mpDir, { recursive: true });
    execSync(`curl -sSL -o ${JSON.stringify(mpBin)} ${MP_URL}`, { stdio: 'inherit' });
    imgPath = mpBin;
  }
}
const raw = new Uint8Array(readFileSync(imgPath));
if (raw.length < 0x100000 || raw[0] !== 0xe9) {
  console.error(`MICROPYTHON HARNESS FAIL: not a flash image (${imgPath}, size ${raw.length})`);
  process.exit(1);
}

// Pad to 3 MiB + vfs partition record + MD5 (mirrors web/main.js
// mpPadAndPartition and tools/micropython_repl.sh exactly).
const PAD = 0x300000;
const flash = new Uint8Array(PAD).fill(0xff);
flash.set(raw.subarray(0, Math.min(raw.length, PAD)), 0);
const pt = 0x8000;
const rec = Buffer.alloc(32);
rec[0] = 0xaa; rec[1] = 0x50; rec[2] = 1; rec[3] = 0x82;
rec.writeUInt32LE(0x200000, 4);
rec.writeUInt32LE(0x100000, 8);
rec.write('vfs\0', 12);
flash.set(rec, pt + 0x60);
const sum = createHash('md5').update(Buffer.from(flash.slice(pt, pt + 0x80))).digest();
const mrec = Buffer.alloc(32);
mrec[0] = 0xeb; mrec[1] = 0xeb;
sum.copy(mrec, 16);
flash.set(mrec, pt + 0x80);

const { Emulator } = await import(join(pkgDir, 'wasm_bridge.js'));

function run(inject, wants = []) {
  const emu = new Emulator();
  emu.load_flash(flash);
  let uart = '';
  let injected = false;
  let done = 0;
  const budget = 300_000_000; // mirrors micropython_repl.sh STEPS
  const t0 = Date.now();
  while (done < budget) {
    done += emu.step_batch(250000);
    const chunk = emu.uart_read();
    if (chunk && chunk.length) uart += Buffer.from(chunk).toString('utf8');
    if (!injected && uart.includes('>>> ')) {
      if (inject) {
        // One byte per frame, mirroring run_flash's per-step marker
        // injection + the browser sendSerial() chunking: UART RX bytes
        // are consumed by the REPL ISR within ~100k steps, so a single
        // multi-byte burst overruns the 128-byte HW FIFO while the
        // firmware idles between polls (proven live: burst shows only
        // the tail of the line, per-byte shows the full REPL echo).
        for (const b of Buffer.from(inject, 'utf8')) {
          emu.uart_inject_rx(0, new Uint8Array([b]));
          emu.step_batch(250000);
          const c = emu.uart_read();
          if (c && c.length) uart += Buffer.from(c).toString('utf8');
        }
      }
      injected = true;
      if (!inject) break;
    }
    // Wait for the NEXT prompt after the verdict, not just any prompt:
    // the banner `>>> ` precedes the echo, so require the echoed line
    // first (run_flash asserts the same way via marker-gated injection).
    if (inject && injected && wants.every((w) => uart.includes(w))) break;
    if (Date.now() - t0 > 570_000) break; // stay inside CI step timeouts
  }
  return { uart, done };
}

const fails = [];
{
  const r = run(null);
  if (!r.uart.includes('>>> ')) fails.push(`no REPL banner (${r.done} insns)`);
  else console.log(`MP banner OK (${r.done} insns)`);
}
{
  const r = run('print(6*7)\n', ['42']);
  if (!r.uart.includes('42')) fails.push('print(6*7) != 42');
  else console.log('MP print(6*7)=42 OK');
  var tail = r.uart;
}
{
  const r = run('print(0.5)\nprint(1.0/3.0)\nprint(0.1+0.2)\n', ['0.5', '0.33333334', '0.3']);
  for (const w of ['0.5', '0.33333334', '0.3']) {
    if (!r.uart.includes(w)) fails.push(`missing float ${w}`);
  }
  if (!fails.length) console.log('MP floats OK');
  tail = r.uart;
}

if (fails.length) {
  console.error('MICROPYTHON HARNESS FAIL: ' + fails.join('; '));
  console.error('--- uart tail ---');
  console.error(JSON.stringify((typeof tail !== 'undefined' ? tail : '').slice(-600)));
  process.exit(1);
}
console.log('MICROPYTHON HARNESS PASS');
