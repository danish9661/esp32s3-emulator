// Headless validation for the Wi-Fi gallery fixtures (battery-runnable).
//
// Drives the nodejs-target wasm build of the emulator through the SAME
// bridge APIs the browser uses (`wifi_scan_fixture` / `wifi_sta_fixture` /
// `wifi_ap_fixture` / `wifi_espnow_fixture`), running
// tools/sketches/esp32s3_wifi_scan, esp32s3_wifi_sta, esp32s3_wifi_ap and
// esp32s3_espnow. Mirrors web/main.js load order: load_flash, arm fixture,
// step, drain UART.
//
// Asserts:
//   * scan: "WIFI SCAN found 1", "WIFI NET 0 EmuNet", "WIFI SCAN DONE"
//   * sta:  "WIFI STA status 3", "WIFI STA IP 192.168.4.2",
//           "WIFI STA SSID EmuNet", "WIFI STA after-disconnect 6",
//           "WIFI STA DONE"
//   * ap:   "WIFI AP softAP 1", "WIFI AP IP 192.168.4.1",
//           "WIFI AP stations 0", "WIFI AP clients 0", "WIFI AP DONE"
//   * espnow: "WIFI ESPNOW init 1", "WIFI ESPNOW addpeer 1",
//           "WIFI ESPNOW sent 1", "WIFI ESPNOW sendcb 1",
//           "WIFI ESPNOW rxcb 1", "WIFI ESPNOW rx0 A5", "WIFI ESPNOW DONE"
//
// Usage: node tools/wifi_harness.mjs [scan|sta|ap|espnow]
// (battery passes the sketch bin as argv[2]; the mode follows the bin
// name, like the scan/sta split). Builds the nodejs wasm pkg into
// tools/.wifi_pkg/ first if it is missing or older than the Rust
// sources. Exit 0 on PASS, 1 with the UART tail on failure.

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

const MODES = {
  wifi_scan: {
    bin: 'tools/sketches/esp32s3_wifi_scan/esp32s3_wifi_scan.merged.bin',
    budget: 60_000_000,
    arm: (emu) => emu.wifi_scan_fixture(APS),
    wants: ['WIFI SCAN found 1', 'WIFI NET 0 EmuNet', 'WIFI SCAN DONE'],
  },
  wifi_sta: {
    bin: 'tools/sketches/esp32s3_wifi_sta/esp32s3_wifi_sta.merged.bin',
    budget: 150_000_000,
    arm: (emu) => emu.wifi_sta_fixture(APS),
    wants: ['WIFI STA status 3', 'WIFI STA IP 192.168.4.2', 'WIFI STA SSID EmuNet', 'WIFI STA after-disconnect 6', 'WIFI STA DONE'],
  },
  wifi_ap: {
    bin: 'tools/sketches/esp32s3_wifi_ap/esp32s3_wifi_ap.merged.bin',
    budget: 60_000_000,
    arm: (emu) => emu.wifi_ap_fixture('EmuAP', 'password', 6),
    wants: ['WIFI AP softAP 1', 'WIFI AP IP 192.168.4.1', 'WIFI AP stations 0', 'WIFI AP clients 0', 'WIFI AP DONE'],
  },
  espnow: {
    bin: 'tools/sketches/esp32s3_espnow/esp32s3_espnow.merged.bin',
    budget: 60_000_000,
    arm: (emu) => emu.wifi_espnow_fixture(),
    wants: ['WIFI ESPNOW init 1', 'WIFI ESPNOW addpeer 1', 'WIFI ESPNOW sent 1', 'WIFI ESPNOW sendcb 1', 'WIFI ESPNOW rxcb 1', 'WIFI ESPNOW rx0 A5', 'WIFI ESPNOW DONE'],
  },
};

function run(binRel, arm, wants, budget) {
  // Battery passes an absolute bin path (binrel convention); direct runs
  // pass a repo-relative path. Accept both (like virtual_demo_harness).
  const binPath = binRel.startsWith('/') ? binRel : join(root, binRel);
  const flash = new Uint8Array(readFileSync(binPath));
  const emu = new Emulator();
  emu.load_flash(flash);
  arm(emu);
  let uart = '';
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

function modeForBin(argBin) {
  // Battery passes the sketch bin as argv[2] (binrel convention, like the
  // gdb entry reusing the hello image); match the wifi_ap/espnow/scan/sta
  // sketch names. `wifi_sta` must win over the `wifi_scan` prefix test —
  // match exact sketch stems, not substrings.
  const b = argBin ?? '';
  if (b.includes('esp32s3_wifi_ap')) return 'wifi_ap';
  if (b.includes('esp32s3_espnow')) return 'espnow';
  if (b.includes('esp32s3_wifi_sta')) return 'wifi_sta';
  return 'wifi_scan';
}

let fails = [];
// Direct runs take an optional mode word (`scan|sta|ap|espnow`,
// `wifi_scan`/`wifi_sta`/`wifi_ap`/`espnow` spellings too) or a bin
// path; default to scan when run by hand.
const arg = process.argv[2];
const MODE_ALIAS = { scan: 'wifi_scan', sta: 'wifi_sta', ap: 'wifi_ap', espnow: 'espnow' };
const explicit = (arg && MODES[arg]) ? arg : (arg && MODE_ALIAS[arg] ? MODE_ALIAS[arg] : null);
const mode = explicit ?? modeForBin(arg);
const m = MODES[mode];
{
  const r = run(explicit ? m.bin : (arg ?? m.bin), m.arm, m.wants, m.budget);
  const tag = `WIFI ${mode === 'espnow' ? 'ESPNOW' : mode.split('_')[1].toUpperCase()}`;
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
