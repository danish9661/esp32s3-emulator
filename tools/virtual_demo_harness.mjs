// Headless validation for the virtual-device demo (battery-runnable).
//
// Drives the SAME JS virtual devices the browser uses
// (web/virtual_devices.js) through the SAME PeripheralBridge
// (web/emu_api.js) against the nodejs-target wasm build of the emulator,
// running tools/sketches/esp32s3_virtual_demo. Mirrors web/main.js tick():
// pre-prime injected bytes, step, drain UART, dispatch events.
//
// Asserts, beyond the UART markers:
//   * sensor.lastWrite === 0x99  (firmware->JS I2C command arrived)
//   * adc.mosi deep-equals [0x55] (firmware->JS SPI MOSI arrived)
//   * UART has "I2C read from virtual device: 0x57" (JS->firmware I2C)
//   * UART has "SPI transfer(0x55) -> MISO 0xAA"     (JS->firmware SPI)
//   * UART has "VIRTUAL DEMO PASS"
//
// Usage: node tools/virtual_demo_harness.mjs [path/to/esp32s3_virtual_demo.merged.bin]
// Builds the nodejs wasm pkg into tools/.virtual_demo_pkg/ first if it is
// missing or older than the Rust sources. Exit 0 on PASS, 1 with the UART
// tail on failure.

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
    // Any .rs newer than the build triggers a rebuild (cheap find).
    out = execFileSync('find', ['crates', '-name', '*.rs', '-newer', file], { cwd: root, encoding: 'utf8' });
  } catch {
    return true;
  }
  return out.trim().length > 0;
}

if (crateNewerThan(wasmFile)) {
  console.error('[virtual-demo] building nodejs wasm pkg...');
  execFileSync('wasm-pack', ['build', 'crates/wasm-bridge', '--target', 'nodejs', '--out-dir', pkgDir], {
    cwd: root,
    stdio: 'inherit',
  });
}

const { Emulator } = await import(join(pkgDir, 'wasm_bridge.js'));
const { PeripheralBridge } = await import(join(root, 'web', 'emu_api.js'));
const { VirtualI2CSensor, VirtualSpiAdc } = await import(join(root, 'web', 'virtual_devices.js'));

const binPath = process.argv[2] ?? join(root, 'tools', 'sketches', 'esp32s3_virtual_demo', 'esp32s3_virtual_demo.merged.bin');
const flash = new Uint8Array(readFileSync(binPath));

const emu = new Emulator();
emu.load_flash(flash);

const bridge = new PeripheralBridge(emu);
const sensor = new VirtualI2CSensor(0x42);
const adc = new VirtualSpiAdc(0xaa);
bridge.i2c.onRead((chan) => sensor.handleRead(chan));
bridge.i2c.onWrite((chan, byte) => sensor.handleWrite(chan, byte));
bridge.spi.onTransfer((chan, tx) => adc.handleTransfer(chan, tx));

let uart = '';
const FRAMES = 600;
const STEPS_PER_FRAME = 250000;
let done = false;
for (let f = 0; f < FRAMES && !done; f++) {
  // Pre-prime exactly like web/main.js tick() (one-frame latency model).
  emu.i2c_inject_rx(0, new Uint8Array([sensor.reg]));
  emu.spi_inject_miso(0, new Uint8Array([adc.value]));
  emu.step(STEPS_PER_FRAME);
  bridge.dispatch();
  const chunk = emu.uart_read();
  if (chunk && chunk.length) {
    uart += Buffer.from(chunk).toString('utf8');
    done = uart.includes('VIRTUAL DEMO DONE');
  }
}

const fails = [];
const has = (s) => uart.includes(s);
if (!has('I2C read from virtual device: 0x57')) fails.push('missing I2C 0x57 readback');
if (!has('SPI transfer(0x55) -> MISO 0xaa') && !has('SPI transfer(0x55) -> MISO 0xAA')) fails.push('missing SPI 0xAA readback');
if (!has('VIRTUAL DEMO PASS')) fails.push('missing VIRTUAL DEMO PASS');
if (sensor.lastWrite !== 0x99) fails.push(`sensor.lastWrite=${sensor.lastWrite} want 0x99`);
const mosi = adc.mosi ? Array.from(adc.mosi) : null;
if (!mosi || mosi.length !== 1 || mosi[0] !== 0x55) fails.push(`adc.mosi=${JSON.stringify(mosi)} want [0x55]`);

if (fails.length) {
  console.error('VIRTUAL DEMO HARNESS FAIL: ' + fails.join('; '));
  console.error('--- uart tail ---');
  console.error(JSON.stringify(uart.slice(-600)));
  process.exit(1);
}
console.log('VIRTUAL DEMO HARNESS PASS');
