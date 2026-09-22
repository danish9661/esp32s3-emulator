// Headless validation for the Wi-Fi gallery fixtures (battery-runnable).
//
// Drives the nodejs-target wasm build of the emulator through the SAME
// bridge APIs the browser uses (`wifi_scan_fixture` / `wifi_sta_fixture`),
// running tools/sketches/esp32s3_wifi_scan and esp32s3_wifi_sta. Mirrors
// web/main.js load order: load_flash, arm fixture, step, drain UART.
//
// Asserts:
//   * scan: "WIFI SCAN found 1", "WIFI NET 0 EmuNet", "WIFI SCAN DONE"
//   * sta:  "WIFI STA status 3", "WIFI STA IP 192.168.4.2",
//           "WIFI STA SSID EmuNet", "WIFI STA after-disconnect 6",
//           "WIFI STA DONE"
//
// Usage: node tools/wifi_harness.mjs [scan|sta]
// Builds the nodejs wasm pkg into tools/.wifi_pkg/ first if it is missing
// or older than the Rust sources. Exit 0 on PASS, 1 with the UART tail on
// failure.

import { execFileSync } from 'child_process';
import { existsSync, statSync, readFileSync } from 'fs';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const pkgDir = join(root, 'tools', '.wifi_pkg');
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
  console.error('[wifi] building nodejs wasm pkg...');
  execFileSync('wasm-pack', ['build', 'crates/wasm-bridge', '--target', 'nodejs', '--out-dir', pkgDir], {
    cwd: root,
    stdio: 'inherit',
  });
}

const { Emulator } = await import(join(pkgDir, 'wasm_bridge.js'));

const APS = 'EmuNet,-50,6,02:11:22:33:44:55';

function run(binRel, arm, wants) {
  // Battery passes an absolute bin path (binrel convention); direct runs
  // pass a repo-relative path. Accept both (like virtual_demo_harness).
  const binPath = binRel.startsWith('/') ? binRel : join(root, binRel);
  const flash = new Uint8Array(readFileSync(binPath));
  const emu = new Emulator();
  emu.load_flash(flash);
  arm(emu);
  let uart = '';
  // Budgets mirror the battery STEPS (instructions): scan 60M, sta 150M.
  const budget = binRel.includes('wifi_sta') ? 150_000_000 : 60_000_000;
  let done = 0;
  const t0 = Date.now();
  while (done < budget) {
    done += emu.step_batch(250000);
    const chunk = emu.uart_read();
    if (chunk && chunk.length) uart += Buffer.from(chunk).toString('utf8');
    if (wants.every((w) => uart.includes(w))) break;
    if (Date.now() - t0 > 570_000) break; // stay inside CI step timeouts
  }
  const fails = wants.filter((w) => !uart.includes(w));
  return { uart, fails, done };
}

let fails = [];
// Battery passes the sketch bin as argv[2] (binrel convention, like the
// gdb entry reusing the hello image); default to scan when run by hand.
const argBin = process.argv[2];
function isStaBin() {
  return (argBin ?? '').includes('wifi_sta');
}
{
  const r = run(
    argBin ?? 'tools/sketches/esp32s3_wifi_scan/esp32s3_wifi_scan.merged.bin',
    (emu) => (isStaBin() ? emu.wifi_sta_fixture(APS) : emu.wifi_scan_fixture(APS)),
    isStaBin()
      ? ['WIFI STA status 3', 'WIFI STA IP 192.168.4.2', 'WIFI STA SSID EmuNet', 'WIFI STA after-disconnect 6', 'WIFI STA DONE']
      : ['WIFI SCAN found 1', 'WIFI NET 0 EmuNet', 'WIFI SCAN DONE'],
  );
  const tag = isStaBin() ? 'WIFI STA' : 'WIFI SCAN';
  if (r.fails.length) {
    console.error(`${tag} HARNESS FAIL: missing ` + JSON.stringify(r.fails));
    console.error('--- uart tail ---');
    console.error(JSON.stringify(r.uart.slice(-600)));
    fails.push(...r.fails);
  } else {
    console.log(`${tag} HARNESS PASS (${r.done} insns)`);
  }
}

if (fails.length) process.exit(1);
// NOTE: the battery greps each entry's markers as separate words
// (markers are `;`-split), so every PASS line repeats the full marker
// text verbatim — keep these lines in sync with run_battery.sh.
console.log('WIFI HARNESS PASS');
