// Headless validation for the LCD_CAM camera capture (battery-runnable).
//
// Stages three identical 8-word frames (the `esp32s3_camcap` sketch captures
// twice: plain, then byte-swapped) into the emulator before stepping, then
// runs to `CAMCAP DONE` and asserts every printed word plus `CAMCAP PASS`.
// Mirrors tools/virtual_demo_harness.mjs (nodejs wasm build, rebuilt when
// Rust sources are newer).
//
// Usage: node tools/camcap_harness.mjs [path/to/esp32s3_camcap.merged.bin]
// Exit 0 on PASS, 1 with the UART tail on failure.

import { execFileSync } from 'child_process';
import { existsSync, statSync, readFileSync } from 'fs';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const pkgDir = join(root, 'tools', '.virtual_demo_pkg');
const wasmFile = join(pkgDir, 'wasm_bridge_bg.wasm');

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
  console.error('[camcap] building nodejs wasm pkg...');
  execFileSync('wasm-pack', ['build', 'crates/wasm-bridge', '--target', 'nodejs', '--out-dir', pkgDir], {
    cwd: root,
    stdio: 'inherit',
  });
}

const { Emulator } = await import(join(pkgDir, 'wasm_bridge.js'));
const { VirtualCamera } = await import(join(root, 'web', 'virtual_devices.js'));

const FRAME = [0x01020304, 0x11223344, 0xa5a5a5a5, 0xdeadbeef, 0x12345678, 0x00000000, 0xffffffff, 0x5a5a5a5a];
const swap32 = (w) => (((w & 0xff) << 24) | (((w >>> 8) & 0xff) << 16) | (((w >>> 16) & 0xff) << 8) | ((w >>> 24) & 0xff)) >>> 0;
const hex8 = (w) => (w >>> 0).toString(16).padStart(8, '0').toUpperCase();

const binPath = process.argv[2] ?? join(root, 'tools', 'sketches', 'esp32s3_camcap', 'esp32s3_camcap.merged.bin');
const flash = new Uint8Array(readFileSync(binPath));

const emu = new Emulator();
emu.load_flash(flash);

const cam = new VirtualCamera(FRAME);
emu.cam_inject_frame(cam.takeFrame());
emu.cam_inject_frame(cam.takeFrame());
emu.cam_inject_frame(cam.takeFrame());

let uart = '';
const FRAMES = 600;
const STEPS_PER_FRAME = 250000;
let done = false;
for (let f = 0; f < FRAMES && !done; f++) {
  emu.step(STEPS_PER_FRAME);
  const chunk = emu.uart_read();
  if (chunk && chunk.length) {
    uart += Buffer.from(chunk).toString('utf8');
    done = uart.includes('CAMCAP DONE');
  }
}

const fails = [];
const has = (s) => uart.includes(s);
for (let i = 0; i < 8; i++) {
  if (!has(`CAMCAP GDMA W${i}=${hex8(FRAME[i])}`)) fails.push(`missing GDMA W${i}`);
  if (!has(`CAMCAP PLAIN W${i}=${hex8(FRAME[i])}`)) fails.push(`missing PLAIN W${i}`);
  if (!has(`CAMCAP SWAP W${i}=${hex8(swap32(FRAME[i]))}`)) fails.push(`missing SWAP W${i}`);
}
if (!has('CAMCAP GDMA PASS')) fails.push('missing GDMA PASS');
if (!has('CAMCAP PLAIN PASS')) fails.push('missing PLAIN PASS');
if (!has('CAMCAP SWAP PASS')) fails.push('missing SWAP PASS');
if (!has('CAMCAP PASS')) fails.push('missing CAMCAP PASS');
if (cam.framesProvided !== 3) fails.push(`framesProvided=${cam.framesProvided} want 3`);

if (fails.length) {
  console.error('CAMCAP HARNESS FAIL: ' + fails.join('; '));
  console.error('--- uart tail ---');
  console.error(JSON.stringify(uart.slice(-600)));
  process.exit(1);
}
console.log('CAMCAP HARNESS PASS');
