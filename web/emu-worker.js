// Emulation worker: owns the ESP32-S3 machine off the main thread.
//
// The UI thread (main.js) never steps the emulator. It posts commands here
// and renders the frames this worker publishes — either as postMessage
// frames with transferable UART buffers (default, works everywhere) or via
// SAB shared-rings (see sab-rings.js; needs crossOriginIsolated + COOP/COEP,
// e.g. tools/serve.py) with zero copy per frame.
//
// Two engines:
//   - single (default): both LX7 cores stay serialized inside ONE Emulator
//     (machine.rs step_fast core0-then-core1), virtual I2C/SPI devices run
//     inline. Same semantics as the old main-thread loop, minus DOM contention.
//   - dual (experimental, needs no SAB — workers talk over a MessageChannel):
//     this worker steps + drains; io-worker.js owns the virtual devices and
//     answers over a direct port with 1-frame latency (the pre-existing
//     inject latency class). NET/BLE frames still go to the UI thread
//     (sockets live there).
//
// Fast paths (all behavior-preserving — calls skipped, never drains):
//   - emu.uart_pending() == 0  → skip uart_read() Vec handoff
//   - emu.events_pending() == 0 → skip drain_events()/dispatch()
// Adaptive batch: the effective steps/frame scale up while a frame stays
// under ~8ms and down past ~14ms (250k..1M window over the slider base).

import init, { Emulator } from './pkg/wasm_bridge.js';
import { PeripheralBridge } from './emu_api.js';
import { VirtualI2CSensor, VirtualSpiAdc, VirtualCamera } from './virtual_devices.js';
import { wrapRings, ringWriteUart, C } from './sab-rings.js';

let emu = null;
let bridge = null;
let vdevSensor = null;
let vdevAdc = null;
let timer = null;
let running = false;

// Load params retained for Reset (worker-side, no main-thread copy needed).
let savedLoad = null;

// Loop tuning (UI-driven).
let baseSteps = 40000;
let effSteps = 40000;
let clockMode = 1; // Balanced default (same 1-per-2 ratio, batched ticks).
let engine = 'single';
let useSab = false;
let rings = null;

// Dual-engine plumbing.
let ioPort = null;
let ioReady = false;
let pendingInjects = [];
let primeCache = { i2c: 0x57, spi: 0xaa };

// Frame outbox (NET/BLE frames + vdev log lines collected per frame tick).
let netOut = [];
let bleOut = [];
let vdevLogs = [];

// MIPS smoothing (same 0.5s window the old main-thread meter used).
let mipsSteps = 0;
let mipsLastT = 0;
let mipsShown = 0;
let totalSteps = 0;

function armBridges() {
  bridge.net.onFrame((frame) => {
    if (frame && frame.length) netOut.push(frame);
  });
  bridge.ble.onPacket((pkt) => {
    if (pkt && pkt.length) bleOut.push(pkt);
  });
}

function attachVdevs() {
  vdevSensor = new VirtualI2CSensor(0x42);
  vdevAdc = new VirtualSpiAdc(0xaa);
  vdevSensor.onActivity = (t) => {
    if (vdevLogs.length < 60) vdevLogs.push(t);
  };
  vdevAdc.onActivity = (t) => {
    if (vdevLogs.length < 60) vdevLogs.push(t);
  };
  bridge.i2c.onRead(() => vdevSensor.handleRead(0));
  bridge.i2c.onWrite((chan, byte) => vdevSensor.handleWrite(chan, byte));
  bridge.spi.onTransfer((chan, tx) => vdevAdc.handleTransfer(chan, tx));
  const vdevCam = new VirtualCamera([0x01020304, 0x11223344, 0xa5a5a5a5, 0xdeadbeef, 0x12345678, 0x00000000, 0xffffffff, 0x5a5a5a5a]);
  vdevCam.onActivity = (t) => {
    if (vdevLogs.length < 60) vdevLogs.push(t);
  };
  emu.cam_inject_frame(vdevCam.takeFrame());
  emu.cam_inject_frame(vdevCam.takeFrame());
  vdevLogs.push('Virtual devices attached: I2C sensor @0x42, SPI ADC');
}

function hexToBytes(hex) {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(hex.substr(2 * i, 2), 16);
  return out;
}

function doLoad(m) {
  stopLoop();
  // m.bytes arrives as a transferred ArrayBuffer — view it (no copy) so the
  // wasm &[u8] handoff always sees a proper byte array.
  const bytes = new Uint8Array(m.bytes);
  emu = new Emulator();
  bridge = new PeripheralBridge(emu);
  armBridges();
  const g = m.gallery || {};
  if (g.secureBoot === true) emu.secure_boot_enable();
  if (m.keyHex) emu.load_flash_encrypted(bytes, hexToBytes(m.keyHex));
  else emu.load_flash(bytes);
  if (g.sdspi === true) emu.spi_sdspi_attach_sdmmc_image(0);
  if (g.touch) {
    const [pad, val] = String(g.touch).split(':').map(Number);
    if (Number.isInteger(pad) && Number.isInteger(val)) emu.touch_inject(pad, val);
  }
  if (g.wifiScan !== null && g.wifiScan !== undefined) emu.wifi_scan_fixture(g.wifiScan || '');
  if (g.wifiSta !== null && g.wifiSta !== undefined) emu.wifi_sta_fixture(g.wifiSta || 'EmuNet,-50,6,02:11:22:33:44:55');
  if (g.wifiAp !== null && g.wifiAp !== undefined) {
    const ap = g.wifiAp || {};
    emu.wifi_ap_fixture(ap.ssid || 'EmuAP', ap.passphrase || 'password', ap.channel || 6);
  }
  if (g.espnow === true) emu.wifi_espnow_fixture();
  if (g.wifiWorker !== null && g.wifiWorker !== undefined) {
    emu.wifi_worker_fixture(g.wifiWorker || 'EmuNet,-50,6,02:11:22:33:44:55');
  }
  if (g.wifiWorkerL3 !== null && g.wifiWorkerL3 !== undefined) {
    emu.wifi_worker_l3_fixture(g.wifiWorkerL3 || 'EmuNet,-50,6,02:11:22:33:44:55');
  }
  emu.set_clock_mode(clockMode);
  if (engine === 'single') attachVdevs();
  else if (ioPort) ioPort.postMessage({ cmd: 'attach' });
  totalSteps = 0;
  mipsSteps = 0;
  mipsLastT = performance.now();
  mipsShown = 0;
  netOut = [];
  bleOut = [];
  vdevLogs = [];
  savedLoad = m;
  postMessage({ type: 'loaded', bytes: m.bytes.byteLength });
}

// Dual-engine partial dispatch: i2c/spi events (+MOSI payloads) go to the IO
// worker; NET/BLE frames go to the UI thread; GPIO rides the mask.
function dispatchDual() {
  const events = emu.drain_events();
  if (!events.length || !ioPort) return;
  const fwd = [];
  for (const e of events) {
    if (e.kind === 1) {
      const tx = emu.spi_take_tx(e.a);
      fwd.push({ kind: 1, a: e.a, b: e.b, payload: tx.buffer });
    } else if (e.kind === 2 || e.kind === 3 || e.kind === 4) {
      fwd.push({ kind: e.kind, a: e.a, b: e.b });
    } else if (e.kind === 6) {
      const frame = emu.net_take_tx();
      if (frame && frame.length) netOut.push(frame);
    } else if (e.kind === 7) {
      const pkt = emu.bt_hci_take_tx();
      if (pkt && pkt.length) bleOut.push(pkt);
    }
  }
  if (fwd.length) ioPort.postMessage({ type: 'vdevEvents', events: fwd }, fwd.filter((e) => e.payload).map((e) => e.payload));
}

function applyInjects() {
  if (!pendingInjects.length) return;
  const ops = pendingInjects;
  pendingInjects = [];
  for (const op of ops) {
    const bytes = new Uint8Array(op.bytes);
    if (op.op === 'i2cRx') emu.i2c_inject_rx(op.chan, bytes);
    else if (op.op === 'spiMiso') emu.spi_inject_miso(op.chan, bytes);
  }
}

function tick() {
  if (!emu || !running) return;
  const t0 = performance.now();
  const deadline = t0 + 12;
  let executed = 0;
  const uartChunks = [];
  if (engine === 'dual') applyInjects();
  for (let b = 0; b < 64; b++) {
    if (engine === 'single' && vdevSensor && vdevAdc) {
      emu.i2c_inject_rx(0, new Uint8Array([vdevSensor.reg]));
      emu.spi_inject_miso(0, new Uint8Array([vdevAdc.value]));
    } else if (engine === 'dual') {
      emu.i2c_inject_rx(0, new Uint8Array([primeCache.i2c & 0xff]));
      emu.spi_inject_miso(0, new Uint8Array([primeCache.spi & 0xff]));
    }
    executed += emu.step_batch(effSteps);
    if (emu.uart_pending() > 0) {
      const u = emu.uart_read();
      if (u && u.length) uartChunks.push(u);
    }
    if (emu.events_pending() > 0) {
      if (engine === 'single') bridge.dispatch();
      else dispatchDual();
    }
    if (performance.now() >= deadline) break;
  }
  const dt = performance.now() - t0;
  // AIMD on the effective batch: grow while frames stay cheap, shrink past
  // the 12ms budget (bounded 0.25x..25x of the slider base ≈ 250k..1M).
  if (dt < 8) effSteps = Math.min(baseSteps * 25, Math.max(effSteps + baseSteps, 250000));
  else if (dt > 14) effSteps = Math.max(baseSteps, Math.floor(effSteps / 2));
  totalSteps += executed;
  const gpio = emu.gpio_output();
  const pc = emu.pc();
  mipsSteps += executed;
  const now = performance.now();
  if (now - mipsLastT >= 500) {
    mipsShown = mipsSteps / ((now - mipsLastT) / 1000) / 1e6;
    mipsSteps = 0;
    mipsLastT = now;
  }
  // Flatten UART chunks once per frame tick (single transferable).
  let uartBytes = null;
  let total = 0;
  for (const c of uartChunks) total += c.length;
  if (total) {
    uartBytes = new Uint8Array(total);
    let o = 0;
    for (const c of uartChunks) {
      uartBytes.set(c, o);
      o += c.length;
    }
  }
  const frames = netOut;
  netOut = [];
  const pkts = bleOut;
  bleOut = [];
  const logs = vdevLogs;
  vdevLogs = [];
  if (useSab && rings) {
    if (uartBytes) ringWriteUart(rings, uartBytes);
    Atomics.store(rings.ctrl, C.GPIO, gpio >>> 0);
    Atomics.store(rings.ctrl, C.STEPS_LO, totalSteps >>> 0);
    Atomics.store(rings.ctrl, C.STEPS_HI, Math.floor(totalSteps / 0x100000000) >>> 0);
    Atomics.store(rings.ctrl, C.MIPS_X10, Math.round(mipsShown * 10));
    Atomics.store(rings.ctrl, C.PC, pc >>> 0);
    Atomics.add(rings.ctrl, C.WR, 1);
  }
  const msg = { type: 'frame', gpio, pc, steps: totalSteps, mips: mipsShown, sab: !!(useSab && rings), netTx: frames, bleTx: pkts, logs };
  const xfer = [];
  if (uartBytes && !useSab) {
    msg.uart = uartBytes.buffer;
    xfer.push(uartBytes.buffer);
  }
  for (const f of frames) xfer.push(f.buffer);
  for (const p of pkts) xfer.push(p.buffer);
  postMessage(msg, xfer);
}

function startLoop() {
  if (timer || !emu) return;
  running = true;
  mipsLastT = performance.now();
  timer = setInterval(tick, 0);
}

function stopLoop() {
  running = false;
  if (timer) {
    clearInterval(timer);
    timer = null;
  }
}

onmessage = async (ev) => {
  const m = ev.data;
  if (!m) return;
  switch (m.cmd) {
    case 'init':
      await init();
      postMessage({ type: 'ready', sab: false });
      break;
    case 'load':
      doLoad(m);
      break;
    case 'run':
      startLoop();
      postMessage({ type: 'running', on: true });
      break;
    case 'stop':
      stopLoop();
      postMessage({ type: 'running', on: false });
      break;
    case 'reset':
      if (savedLoad) {
        const keep = { cmd: 'load', bytes: savedLoad.bytes, keyHex: savedLoad.keyHex, gallery: savedLoad.gallery };
        // savedLoad.bytes was transferred in — retained worker-side.
        doLoad(keep);
      }
      break;
    case 'steps':
      baseSteps = Math.max(1000, Math.min(1000000, m.n | 0 || 40000));
      effSteps = baseSteps;
      break;
    case 'clock':
      clockMode = [0, 1, 2].includes(m.mode | 0) ? m.mode | 0 : 1;
      if (emu) emu.set_clock_mode(clockMode);
      break;
    case 'engine': {
      const next = m.which === 'dual' ? 'dual' : 'single';
      if (next !== engine) {
        engine = next;
        // Vdev ownership moves (inline vs IO worker) — reload so the
        // machine boots clean under the new topology.
        if (savedLoad) {
          const keep = { cmd: 'load', bytes: savedLoad.bytes, keyHex: savedLoad.keyHex, gallery: savedLoad.gallery };
          doLoad(keep);
        }
      }
      break;
    }
    case 'ioPort':
      ioPort = m.port;
      ioPort.onmessage = (e2) => {
        const d = e2.data;
        if (!d) return;
        if (d.type === 'ioInjects') {
          if (d.prime) primeCache = d.prime;
          for (const op of d.injects || []) pendingInjects.push(op);
          // Note: op.bytes rides transferred; wrap on apply (see applyInjects).
          for (const t of d.logs || []) {
            if (vdevLogs.length < 60) vdevLogs.push(t);
          }
        } else if (d.type === 'ioReady') {
          ioReady = true;
          if (d.prime) primeCache = d.prime;
        }
      };
      break;
    case 'transport':
      if (m.sab) {
        try {
          rings = wrapRings(m.sab);
          useSab = true;
          postMessage({ type: 'transport', sab: true });
        } catch {
          rings = null;
          useSab = false;
          postMessage({ type: 'transport', sab: false, reason: 'wrap-failed' });
        }
      } else {
        rings = null;
        useSab = false;
        postMessage({ type: 'transport', sab: false });
      }
      break;
    case 'inject': {
      if (!emu) break;
      const bytes = new Uint8Array(m.bytes);
      const CHUNK = 96;
      for (let i = 0; i < bytes.length; i += CHUNK) {
        const slice = bytes.slice(i, i + CHUNK);
        if (m.port === 'usb') emu.usb_inject_rx(slice);
        else emu.uart_inject_rx(parseInt(m.port, 10), slice);
      }
      break;
    }
    case 'netRx':
      if (emu && m.bytes) emu.net_inject_rx(new Uint8Array(m.bytes));
      break;
    case 'bleRx':
      if (emu && m.bytes) emu.bt_hci_inject_rx(new Uint8Array(m.bytes));
      break;
  }
};
