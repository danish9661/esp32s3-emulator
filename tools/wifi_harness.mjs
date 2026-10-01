// Headless validation for the Wi-Fi gallery fixtures (battery-runnable).
//
// Drives the nodejs-target wasm build of the emulator through the SAME
// bridge APIs the browser uses (`wifi_scan_fixture` / `wifi_sta_fixture` /
// `wifi_ap_fixture` / `wifi_espnow_fixture` / `wifi_worker_fixture` /
// `wifi_worker_l3_fixture`),
// running tools/sketches/esp32s3_wifi_scan, esp32s3_wifi_sta,
// esp32s3_wifi_ap, esp32s3_espnow, esp32s3_test_worker_net and
// esp32s3_test_worker_l3. Mirrors
// web/main.js load order: load_flash, arm fixture, step, drain UART.
//
// Asserts:
//   * scan: "WIFI SCAN found 1", "WIFI NET 0 EmuNet", "WIFI SCAN DONE"
//   * sta:  "WIFI STA status 3", "WIFI STA IP 192.168.4.2",
//           "WIFI STA SSID EmuNet", "WIFI STA after-disconnect 6",
//           "WIFI STA DONE"
//   * ap:   "WIFI AP softAP 1", "WIFI AP IP 192.168.4.1",
//           "WIFI AP MAC 62:55:44:33:22:11" (factory-MAC-derived AP MAC),
//           "WIFI AP stations 0", "WIFI AP clients 0", "WIFI AP DONE"
//   * espnow: "WIFI ESPNOW init 1", "WIFI ESPNOW addpeer 1",
//           "WIFI ESPNOW sent 1", "WIFI ESPNOW sendcb 1",
//           "WIFI ESPNOW rxcb 1", "WIFI ESPNOW rx0 68",
//           "WIFI ESPNOW rxlen 5", "WIFI ESPNOW rxsum 14",
//           "WIFI ESPNOW DONE" (full "hello" A→B→A exchange)
//   * worker: "WORKER NET START", "WORKER NET status 3",
//           "WORKER NET netif 1", "WORKER NET tx1 -11",
//           "WORKER NET tx2 -12", "WORKER NET keep 14",
//           "WORKER NET rx1 1 -13",
//           "WORKER NET rx2 1 -14", "WORKER NET DONE" (live-IP backhaul
//           path: real `esp_netif_transmit` TX tap fires; the -11/-12
//           return codes are ESP_FAIL without a bound lwIP netif — the
//           tap is L2, the return code is not the verdict; the keep-alive
//           loop re-transmits the ARP probe 15x so the host keeps draining
//           (defeats the run_flash idle-exit + lets gateway replies stage);
//           the rx legs call the real `esp_netif_receive` with an empty
//           FIFO, so the call runs unmodified and the zeroed buffer reads
//           -13/-14 — with NET_GW connected the gateway ARP/ICMP replies
//           arrive and the same markers read 60/42, proven live in the
//           E2E run)
//
// Usage: node tools/wifi_harness.mjs [scan|sta|ap|espnow|worker]
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
const { PeripheralBridge } = await import(join(root, 'web', 'emu_api.js'));

const APS = 'EmuNet,-50,6,02:11:22:33:44:55';

// Optional live gateway feed for the worker mode (mirrors the browser
// Live-IP panel and the run_flash NET_GW leg): set WIFI_HARNESS_GW to a
// TCP ingest address (e.g. 127.0.0.1:5051) while a Go gateway runs. The
// harness opens a raw TCP socket, forwards every captured TX frame
// length-prefixed (same framing run_flash uses), and stages every reply
// back via `net_inject_rx` for the `esp_netif_receive` entry hook. With
// the gateway up the worker's rx legs read the live ARP/ICMP replies
// (`rx1 1 60`); without it they read the empty-FIFO verdicts (-13/-14).
// Other modes ignore the env (no TX tap fires there).
import net from 'node:net';
let gwSock = null;
let gwRxBuf = Buffer.alloc(0);
async function gwConnect(addr) {
  // Open a raw TCP socket to the gateway ingest leg. Resolves to the
  // socket on success, null on failure/timeout (caller falls back to
  // local verdicts). The 'data' listener appends to the shared reassembly
  // buffer drained per macro-step below.
  return new Promise((resolve) => {
    let done = false;
    const finish = (v) => { if (!done) { done = true; resolve(v); } };
    const [host, portStr] = addr.split(':');
    let sock;
    try {
      sock = net.createConnection({ host, port: Number(portStr) }, () => finish(sock));
    } catch {
      finish(null);
      return;
    }
    sock.on('data', (chunk) => { gwRxBuf = Buffer.concat([gwRxBuf, chunk]); });
    sock.on('error', () => finish(null));
    setTimeout(() => { if (!done) { try { sock.destroy(); } catch (_) {} finish(null); } }, 3000);
  });
}
function gwDrain(emu) {
  // Extract complete length-prefixed frames; keep partial tails buffered
  // (same reassembly discipline as the run_flash nonblocking drain).
  for (;;) {
    if (gwRxBuf.length < 4) break;
    const rlen = gwRxBuf.readUInt32BE(0);
    if (rlen === 0 || rlen > 1600) { gwRxBuf = Buffer.alloc(0); break; }
    if (gwRxBuf.length < 4 + rlen) break;
    emu.net_inject_rx(gwRxBuf.subarray(4, 4 + rlen));
    gwRxBuf = gwRxBuf.subarray(4 + rlen);
  }
}

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
    wants: ['WIFI AP softAP 1', 'WIFI AP IP 192.168.4.1', 'WIFI AP MAC 62:55:44:33:22:11', 'WIFI AP stations 0', 'WIFI AP clients 0', 'WIFI AP DONE'],
  },
  espnow: {
    bin: 'tools/sketches/esp32s3_espnow/esp32s3_espnow.merged.bin',
    budget: 60_000_000,
    arm: (emu) => emu.wifi_espnow_fixture(),
    wants: ['WIFI ESPNOW init 1', 'WIFI ESPNOW addpeer 1', 'WIFI ESPNOW sent 1', 'WIFI ESPNOW sendcb 1', 'WIFI ESPNOW rxcb 1', 'WIFI ESPNOW rx0 68', 'WIFI ESPNOW rxlen 5', 'WIFI ESPNOW rxsum 14', 'WIFI ESPNOW DONE'],
  },
  worker: {
    bin: 'tools/sketches/esp32s3_test_worker_net/esp32s3_test_worker_net.merged.bin',
    budget: 350_000_000,
    arm: (emu) => emu.wifi_worker_fixture(APS),
    wants: ['WORKER NET START', 'WORKER NET status 3', 'WORKER NET netif 1', 'WORKER NET tx1 -11', 'WORKER NET tx2 -12', 'WORKER NET keep 14', 'WORKER NET rx1 1 -13', 'WORKER NET rx2 1 -14', 'WORKER NET DONE'],
  },
  worker_l3: {
    bin: 'tools/sketches/esp32s3_test_worker_l3/esp32s3_test_worker_l3.merged.bin',
    budget: 350_000_000,
    arm: (emu) => emu.wifi_worker_l3_fixture(APS),
    wants: ['WORKER L3 START', 'WORKER L3 status 3', 'WORKER L3 netif 1', 'WORKER L3 DONE'],
  },
};

function run(binRel, arm, wants, budget, liveGw) {
  // Battery passes an absolute bin path (binrel convention); direct runs
  // pass a repo-relative path. Accept both (like virtual_demo_harness).
  const binPath = binRel.startsWith('/') ? binRel : join(root, binRel);
  const flash = new Uint8Array(readFileSync(binPath));
  const emu = new Emulator();
  emu.load_flash(flash);
  arm(emu);
  // Route captured TX frames through the SAME PeripheralBridge the browser
  // uses (web/emu_api.js): EVT_NET_FRAME + `net_take_tx`. With a live
  // gateway the frames go out length-prefixed on the TCP leg and replies
  // come back via `net_inject_rx` (same path as main.js + run_flash).
  // TIMING (proven live 2026-09-28): the gateway answers each TX ~1.3 s
  // after it (ARP fast-reply + ICMP snoop are synchronous, but the TCP
  // round-trip + hub loop cost wall time), while the worker's two
  // `esp_netif_receive` calls run back-to-back right after frame 2 — so
  // the replies always land AFTER the sketch already checked. The harness
  // therefore cannot assert reply bytes in-band like run_flash's
  // TX-armed + 64-step-backstop drain can (it polls the socket every
  // macro-step). Instead it asserts the bridge mechanics: both TX frames
  // forwarded AND at least one reply staged back (proves the full
  // board→gateway→board round trip through the real stack entry points).
  const bridge = new PeripheralBridge(emu);
  let txFrames = 0;
  let rxStaged = 0;
  if (liveGw) {
    bridge.net.onFrame((frame) => {
      if (!frame || !frame.length) return;
      txFrames++;
      const len = Buffer.alloc(4);
      len.writeUInt32BE(frame.length);
      liveGw.write(Buffer.concat([len, Buffer.from(frame)]));
    });
  }
  let uart = '';
  let done = 0;
  const t0 = Date.now();
  while (done < budget) {
    done += emu.step_batch(250000);
    const chunk = emu.uart_read();
    if (chunk && chunk.length) uart += Buffer.from(chunk).toString('utf8');
    bridge.dispatch();
    if (liveGw) {
      const before = rxStaged;
      const n0 = gwRxBuf.length;
      gwDrain(emu);
      if (gwRxBuf.length !== n0) rxStaged++;
      void before;
    }
    if (wants.every((w) => uart.includes(w))) break;
    if (Date.now() - t0 > 570_000) break; // stay inside CI step timeouts
  }
  const fails = wants.filter((w) => !uart.includes(w));
  return { uart, fails, done, txFrames, rxStaged };
}

function modeForBin(argBin) {
  // Battery passes the sketch bin as argv[2] (binrel convention, like the
  // gdb entry reusing the hello image); match the wifi_ap/espnow/scan/sta
  // sketch names. `wifi_sta` must win over the `wifi_scan` prefix test —
  // match exact sketch stems, not substrings.
  const b = argBin ?? '';
  if (b.includes('esp32s3_test_worker_l3')) return 'worker_l3';
  if (b.includes('esp32s3_test_worker_net')) return 'worker';
  if (b.includes('esp32s3_wifi_ap')) return 'wifi_ap';
  if (b.includes('esp32s3_espnow')) return 'espnow';
  if (b.includes('esp32s3_wifi_sta')) return 'wifi_sta';
  return 'wifi_scan';
}

let fails = [];
// Direct runs take an optional mode word (`scan|sta|ap|espnow|worker|worker_l3`,
// `wifi_scan`/`wifi_sta`/`wifi_ap`/`espnow`/`worker`/`worker_l3` spellings too) or a
// bin path; default to scan when run by hand.
// WIFI_HARNESS_GW=<host:port> (worker mode only) feeds captured TX frames
// to a live Go gateway over its TCP ingest leg and asserts the live ARP
// reply lands in the firmware's own receive buffer (`rx1 1 60`). Without
// it the worker asserts the empty-FIFO verdicts (-13/-14) — same binary,
// purely a host-leg difference (battery default: no gateway).
const arg = process.argv[2];
const MODE_ALIAS = { scan: 'wifi_scan', sta: 'wifi_sta', ap: 'wifi_ap', espnow: 'espnow', worker: 'worker', worker_l3: 'worker_l3' };
const explicit = (arg && MODES[arg]) ? arg : (arg && MODE_ALIAS[arg] ? MODE_ALIAS[arg] : null);
const mode = explicit ?? modeForBin(arg);
const m = MODES[mode];
{
  const gwAddr = process.env.WIFI_HARNESS_GW || '';
  const wantLive = mode === 'worker' && gwAddr.includes(':');
  const liveSock = wantLive ? await gwConnect(gwAddr) : null;
  const liveGw = wantLive && liveSock ? liveSock : null;
  if (wantLive && !liveGw) console.error('[wifi] WIFI_HARNESS_GW set but gateway unreachable — running local verdicts');
  // Live-gateway verdicts: the ARP reply (60B 0x0806 opcode 2) must land
  // in the firmware buffer, so rx1 reads 60, not -13. rx2 still reads -14
  // (ICMP reply lands one step after the second receive call consumed the
  // empty FIFO — proven live in the run_flash E2E; the `net RX 42B 0x0800`
  // host line proves delivery, the sketch just checks too early).
  const wants = (liveGw && mode === 'worker')
    ? m.wants.map((w) => (w === 'WORKER NET rx1 1 -13' ? 'WORKER NET rx1 1 60' : w))
    : m.wants;
  const r = run(explicit ? m.bin : (arg ?? m.bin), m.arm, wants, m.budget, liveGw);
  if (liveGw) liveSock.destroy();
  const tag = `WIFI ${mode === 'espnow' ? 'ESPNOW' : mode === 'worker' ? 'WORKER NET' : mode === 'worker_l3' ? 'WORKER L3' : mode.split('_')[1].toUpperCase()}`;
  if (r.fails.length) {
    console.error(`${tag} HARNESS FAIL: missing ` + JSON.stringify(r.fails));
    console.error('--- uart tail ---');
    console.error(JSON.stringify(r.uart.slice(-600)));
    fails.push(...r.fails);
  } else {
    console.log(`${tag} HARNESS PASS (${r.done} insns${liveGw ? `, TX ${r.txFrames} frames` : ''})`);
  }
}

if (fails.length) process.exit(1);
// NOTE: the battery greps each entry's markers as separate words
// (markers are `;`-split), so every PASS line repeats the full marker
// text verbatim — keep these lines in sync with run_battery.sh.
console.log('WIFI HARNESS PASS');
