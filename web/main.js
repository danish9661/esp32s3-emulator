import init, { Emulator } from './pkg/wasm_bridge.js';
import { PeripheralBridge } from './emu_api.js';
import { VirtualI2CSensor, VirtualSpiAdc, VirtualCamera } from './virtual_devices.js';

const NUM_PINS = 40;

// ── DOM references ──
const els = {
  firmware: document.getElementById('firmware'),
  gallery: document.getElementById('gallery'),
  run: document.getElementById('run'),
  stop: document.getElementById('stop'),
  reset: document.getElementById('reset'),
  steps: document.getElementById('steps'),
  stepsVal: document.getElementById('stepsVal'),
  status: document.getElementById('status'),
  mips: document.getElementById('mips'),
  console: document.getElementById('console'),
  serialInput: document.getElementById('serialInput'),
  serialSend: document.getElementById('serialSend'),
  serialPort: document.getElementById('serialPort'),
  gpio: document.getElementById('gpio'),
  vdev: document.getElementById('vdev'),
  searchInput: document.getElementById('searchInput'),
  searchBtn: document.getElementById('searchBtn'),
  searchPrevBtn: document.getElementById('searchPrevBtn'),
  searchStatus: document.getElementById('searchStatus'),
  copyBtn: document.getElementById('copyBtn'),
  clearBtn: document.getElementById('clearBtn'),
  autoScroll: document.getElementById('autoScroll'),
  gpioExpandAll: document.getElementById('gpioExpandAll'),
  gpioCollapseAll: document.getElementById('gpioCollapseAll'),
  mpUrl: document.getElementById('mpUrl'),
  mpLoad: document.getElementById('mpLoad'),
};

// ── Serial console state ──
const MAX_CONSOLE_LINES = 5000;
let consoleLines = [];
let consoleText = '';
let searchMatches = [];
let searchIdx = -1;

function setStatus(msg) {
  els.status.textContent = msg;
}

// ── Serial console ──
function appendSerial(bytes) {
  if (!bytes || bytes.length === 0) return;
  const text = new TextDecoder().decode(bytes);
  consoleText += text;
  // Track lines for search
  const newLines = text.split('\n');
  for (const line of newLines) {
    consoleLines.push(line);
  }
  // Trim scrollback
  if (consoleLines.length > MAX_CONSOLE_LINES) {
    const excess = consoleLines.length - MAX_CONSOLE_LINES;
    consoleLines.splice(0, excess);
  }
  els.console.textContent = consoleText;
  if (els.autoScroll.checked) {
    els.console.scrollTop = els.console.scrollHeight;
  }
}

// ── Search ──
function doSearch(reverse) {
  const query = els.searchInput.value;
  if (!query) {
    clearSearchHighlights();
    els.searchStatus.textContent = '';
    return;
  }
  // Build plain-text from current console content
  const plain = consoleText;
  const lower = plain.toLowerCase();
  const qLower = query.toLowerCase();
  searchMatches = [];
  let idx = 0;
  while ((idx = lower.indexOf(qLower, idx)) !== -1) {
    searchMatches.push(idx);
    idx++;
  }
  if (searchMatches.length === 0) {
    els.searchStatus.textContent = 'No matches';
    clearSearchHighlights();
    return;
  }
  // Navigate
  if (reverse) {
    searchIdx = searchIdx <= 0 ? searchMatches.length - 1 : searchIdx - 1;
  } else {
    searchIdx = searchIdx >= searchMatches.length - 1 ? 0 : searchIdx + 1;
  }
  els.searchStatus.textContent = `${searchIdx + 1}/${searchMatches.length}`;
  // Scroll to match — we mark the current match in the text
  highlightCurrentMatch(plain, searchMatches[searchIdx], query.length);
}

function highlightCurrentMatch(plain, start, len) {
  // Re-render with a highlighted span around the current match
  const before = plain.substring(0, start);
  const match = plain.substring(start, start + len);
  const after = plain.substring(start + len);
  els.console.textContent = '';
  els.console.appendChild(document.createTextNode(before));
  const mark = document.createElement('mark');
  mark.className = 'current';
  mark.textContent = match;
  els.console.appendChild(mark);
  els.console.appendChild(document.createTextNode(after));
  // Scroll the mark into view
  mark.scrollIntoView({ behavior: 'smooth', block: 'center' });
}

function clearSearchHighlights() {
  els.console.textContent = consoleText;
}

els.searchInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') {
    e.preventDefault();
    doSearch(e.shiftKey);
  }
  if (e.key === 'Escape') {
    els.searchInput.value = '';
    clearSearchHighlights();
    els.searchStatus.textContent = '';
  }
});
els.searchBtn.addEventListener('click', () => doSearch(false));
els.searchPrevBtn.addEventListener('click', () => doSearch(true));

els.copyBtn.addEventListener('click', async () => {
  try {
    await navigator.clipboard.writeText(consoleText);
    els.copyBtn.textContent = '✓ Copied';
    setTimeout(() => { els.copyBtn.textContent = '📋 Copy'; }, 1500);
  } catch {
    // Fallback: select + execCommand
    const ta = document.createElement('textarea');
    ta.value = consoleText;
    document.body.appendChild(ta);
    ta.select();
    document.execCommand('copy');
    document.body.removeChild(ta);
    els.copyBtn.textContent = '✓ Copied';
    setTimeout(() => { els.copyBtn.textContent = '📋 Copy'; }, 1500);
  }
});

els.clearBtn.addEventListener('click', () => {
  consoleText = '';
  consoleLines = [];
  els.console.textContent = '';
  clearSearchHighlights();
  els.searchStatus.textContent = '';
  searchMatches = [];
  searchIdx = -1;
});

// Ctrl+F opens search
document.addEventListener('keydown', (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key === 'f') {
    e.preventDefault();
    els.searchInput.focus();
    els.searchInput.select();
  }
});

// ── GPIO visualization ──
const NUM_GPIO = 40;
const gpioCells = [];
let gpioMode = new Array(NUM_GPIO).fill('out'); // 'out' or 'in'
let gpioPrevMask = 0;

function initGpio() {
  els.gpio.innerHTML = '';
  for (let i = 0; i < NUM_GPIO; i++) {
    const cell = document.createElement('div');
    cell.className = 'gpio-cell out-lo';
    cell.dataset.tip = `GPIO${i}`;
    cell.innerHTML = `
      <span class="pin-num">${i}</span>
      <span class="pin-state">LO</span>
    `;
    cell.addEventListener('click', () => {
      // Toggle display mode between output and input visualization
      gpioMode[i] = gpioMode[i] === 'out' ? 'in' : 'out';
      updateGpioCell(i, gpioPrevMask);
    });
    els.gpio.appendChild(cell);
    gpioCells.push(cell);
  }
}

function updateGpioCell(i, mask) {
  const cell = gpioCells[i];
  const hi = (mask & (1 << i)) !== 0;
  const mode = gpioMode[i];
  const stateEl = cell.querySelector('.pin-state');
  if (mode === 'in') {
    cell.className = 'gpio-cell input';
    stateEl.textContent = hi ? 'IN↑' : 'IN↓';
    cell.dataset.tip = `GPIO${i} INPUT ${hi ? 'HIGH' : 'LOW'}`;
  } else {
    cell.className = hi ? 'gpio-cell out-hi' : 'gpio-cell out-lo';
    stateEl.textContent = hi ? 'HI' : 'LO';
    cell.dataset.tip = `GPIO${i} OUTPUT ${hi ? 'HIGH' : 'LOW'}`;
  }
}

let gpioFrameSkip = 0;
function renderGpio() {
  const mask = emu.gpio_output();
  if (mask === gpioPrevMask) return;
  // The grid is 40 DOM cells: repaint at most every 3rd frame (~20 Hz).
  // Pin state still converges (mask diff is cumulative), but layout work
  // drops ~3x during blink-heavy firmware. Force full repaint when the
  // mask settles after activity (so the final state is never stale).
  if ((gpioFrameSkip = (gpioFrameSkip + 1) % 3) !== 0) return;
  // Only update cells that changed
  let diff = mask ^ gpioPrevMask;
  for (let i = 0; i < NUM_GPIO && diff; i++) {
    if (diff & 1) updateGpioCell(i, mask);
    diff >>= 1;
  }
  gpioPrevMask = mask;
}

els.gpioExpandAll.addEventListener('click', () => {
  for (let i = 0; i < NUM_GPIO; i++) gpioMode[i] = 'out';
  renderGpio();
  // Force full redraw
  gpioPrevMask = ~0;
  renderGpio();
});
els.gpioCollapseAll.addEventListener('click', () => {
  for (let i = 0; i < NUM_GPIO; i++) gpioCells[i].className = 'gpio-cell out-lo';
});

// ── Virtual devices ──
let vdevLineCount = 0;
function appendVdev(text) {
  if (!els.vdev) return;
  const line = document.createElement('div');
  line.className = 'vdev-line';
  line.textContent = text;
  els.vdev.appendChild(line);
  if (++vdevLineCount > 200) els.vdev.removeChild(els.vdev.firstChild);
  els.vdev.scrollTop = els.vdev.scrollHeight;
}

// ── Emulator loop ──
let emu = null;
let bridge = null;
let vdevSensor = null;
let vdevAdc = null;
let vdevCam = null;
let flashBytes = null;
let flashKeyHex = null;
let timer = null;
let totalSteps = 0;
// ── MIPS meter state ──
let mipsSteps = 0;
let mipsLastT = performance.now();
let mipsShown = 0;

// Frame budget: one rAF tick ≈ 16 ms. At ~10 MIPS in-wasm, 40k steps
// would need only ~4 ms of emulation — but the per-call JS↔wasm boundary
// cost dominates at small batches (measured: raising the batch 40k →
// 250k/frame changes wall time per emulated second far less than 6x).
// Run up to 4 consecutive batches per frame while time remains (< 12 ms),
// so fast boots finish sooner without freezing the page on slow devices.
let lastTickMs = 0;
function tick() {
  const t0 = performance.now();
  // Deadline: leave ~4 ms of the 16 ms frame for paint/input.
  const deadline = t0 + 12;
  const n = parseInt(els.steps.value, 10);
  let executed = 0;
  for (let b = 0; b < 4; b++) {
    if (bridge && vdevSensor && vdevAdc) {
      emu.i2c_inject_rx(0, new Uint8Array([vdevSensor.reg]));
      emu.spi_inject_miso(0, new Uint8Array([vdevAdc.value]));
    }
    executed += emu.step_batch(n);
    // Drain + dispatch once per batch (cheap when empty: the Rust drain
    // fast-path returns without touching merge state, and dispatch only
    // walks queued events).
    appendSerial(emu.uart_read());
    if (bridge) bridge.dispatch();
    if (performance.now() >= deadline) break;
  }
  lastTickMs = performance.now() - t0;
  totalSteps += executed;
  renderGpio();
  // MIPS = emulated instructions per wall second, smoothed over ~0.5 s.
  mipsSteps += executed;
  const now = performance.now();
  const elapsed = now - mipsLastT;
  if (elapsed >= 500) {
    mipsShown = (mipsSteps / (elapsed / 1000)) / 1e6;
    mipsSteps = 0;
    mipsLastT = now;
    if (els.mips) els.mips.textContent = `${mipsShown.toFixed(1)} MIPS`;
  }
  setStatus(`pc=0x${emu.pc().toString(16)}  steps=${totalSteps.toLocaleString()}`);
}

function startLoop() {
  if (timer) return;
  timer = setInterval(tick, 0);
}

function stopLoop() {
  if (timer) {
    clearInterval(timer);
    timer = null;
  }
}

function hexToBytes(hex) {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i++) {
    out[i] = parseInt(hex.substr(2 * i, 2), 16);
  }
  return out;
}

async function loadFlash(bytes, keyHex) {
  stopLoop();
  emu = new Emulator();
  // Gallery entries that arm the fail-closed secure-boot gate opt in via
  // `"secure_boot": true` in manifest.json (mirrors run_flash
  // SECURE_BOOT_EN=1: burns BLK0/REPEAT_DATA4 bit 20 through the real PGM
  // path BEFORE load_flash, so boot_from_flash verifies the app region's
  // signature sector; unsigned images park both CPUs with no output).
  // The committed gallery secure_boot image is genuinely `espsecure.py
  // sign-data`-signed (see tools/sketches/esp32s3_secure_boot/ +
  // tools/build_secure_boot.sh), so it boots to Hello/boot OK.
  if (currentGalleryItem && currentGalleryItem.secureBoot === true) {
    emu.secure_boot_enable();
  }
  if (keyHex) {
    emu.load_flash_encrypted(bytes, hexToBytes(keyHex));
  } else {
    emu.load_flash(bytes);
  }
  // Gallery entries that need an attached virtual SD card opt in via
  // `"sdspi": true` in manifest.json (mirrors run_flash SPI_SDSPI=1:
  // same FAT16 volume SDMMC formatted, over the GPSPI2 SDSPI path).
  const needsSdspi = currentGalleryItem && currentGalleryItem.sdspi === true;
  if (needsSdspi) {
    emu.spi_sdspi_attach_sdmmc_image(0);
  }
  // Gallery entries that need a host fixture opt in via manifest fields
  // (mirrors the run_flash env flows): `"touch": "<pad>:<val>"` injects
  // the touch counter (TOUCH_INJECT); `"secure_boot": true` burns
  // SECURE_BOOT_EN through the real PGM path BEFORE load_flash so the
  // signed gallery image verifies (fail-closed: unsigned images park
  // both CPUs with no output — the committed gallery secure_boot image
  // is genuinely `espsecure.py sign-data`-signed, see
  // tools/sketches/esp32s3_secure_boot/ + tools/build_secure_boot.sh).
  if (currentGalleryItem && currentGalleryItem.touch) {
    const [pad, val] = currentGalleryItem.touch.split(':').map(Number);
    if (Number.isInteger(pad) && Number.isInteger(val)) {
      emu.touch_inject(pad, val);
    }
  }
  // Wi-Fi fixtures (mirror the run_flash WIFI_SCAN_APS flows): the engine
  // lives in the machine (`Soc::wifi_fixture_poll`, driven per step_fast),
  // so the bridge just arms it — no JS per-frame work needed.
  if (currentGalleryItem && currentGalleryItem.wifiScan !== null) {
    emu.wifi_scan_fixture(currentGalleryItem.wifiScan || '');
  }
  if (currentGalleryItem && currentGalleryItem.wifiSta !== null) {
    emu.wifi_sta_fixture(currentGalleryItem.wifiSta || 'EmuNet,-50,6,02:11:22:33:44:55');
  }
  // SoftAP fixture (mirrors run_flash WIFI_AP_FIXTURE=1): the firmware
  // posts AP_START itself; the engine stages the AP config + fixed LAN.
  if (currentGalleryItem && currentGalleryItem.wifiAp !== null) {
    const ap = currentGalleryItem.wifiAp || {};
    emu.wifi_ap_fixture(ap.ssid || 'EmuAP', ap.passphrase || 'password', ap.channel || 6);
  }
  // ESP-NOW loopback (mirrors run_flash WIFI_ESPNOW_LOOPBACK=1): the
  // engine invokes the TX/RX wrappers in-firmware once `send()` returned
  // (UART `sent 1` marker, peeked from the host console stream).
  if (currentGalleryItem && currentGalleryItem.espnow === true) {
    emu.wifi_espnow_fixture();
  }

  if (typeof PeripheralBridge !== 'undefined') {
    bridge = new PeripheralBridge(emu);
    vdevSensor = new VirtualI2CSensor(0x42);
    vdevAdc = new VirtualSpiAdc(0xaa);
    vdevSensor.onActivity = (t) => appendVdev(t);
    vdevAdc.onActivity = (t) => appendVdev(t);
    bridge.i2c.onRead((chan) => vdevSensor.handleRead(chan));
    bridge.i2c.onWrite((chan, byte) => vdevSensor.handleWrite(chan, byte));
    bridge.spi.onTransfer((chan, tx) => vdevAdc.handleTransfer(chan, tx));
    // Demo camera frame (two captures' worth, like the harness pre-primes).
    vdevCam = new VirtualCamera([0x01020304, 0x11223344, 0xa5a5a5a5, 0xdeadbeef, 0x12345678, 0x00000000, 0xffffffff, 0x5a5a5a5a]);
    vdevCam.onActivity = (t) => appendVdev(t);
    emu.cam_inject_frame(vdevCam.takeFrame());
    emu.cam_inject_frame(vdevCam.takeFrame());
    if (els.vdev) els.vdev.textContent = '';
    vdevLineCount = 0;
    appendVdev('Virtual devices attached: I2C sensor @0x42, SPI ADC');
  }

  flashBytes = bytes;
  flashKeyHex = keyHex || null;
  totalSteps = 0;
  consoleText = '';
  consoleLines = [];
  els.console.textContent = '';
  gpioPrevMask = 0;
  renderGpio();
  setStatus(`loaded ${bytes.length.toLocaleString()} bytes`);
  els.run.disabled = false;
  els.stop.disabled = true;
  els.reset.disabled = false;
  // Reset the MIPS meter for the fresh run.
  mipsSteps = 0;
  mipsLastT = performance.now();
  if (els.mips) els.mips.textContent = '';
}

// ── File input ──
els.firmware.addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  currentGalleryItem = null;
  const buf = await file.arrayBuffer();
  let bytes = new Uint8Array(buf);
  // Uploaded MicroPython images arrive as the raw Release .bin (1.78 MB,
  // magic E9) — without the vfs partition the firmware prints "filesystem
  // appears to be corrupted" (proven live). Pad + partition exactly like
  // the ▶ REPL preset; arduino merged bins are already full-flash size
  // (4 MB ≥ pad size) so they pass through untouched.
  if (bytes.length > 0 && bytes[0] === 0xe9 && bytes.length < MP_PAD_SIZE) {
    try {
      bytes = mpPadAndPartition(bytes);
      setStatus(`MicroPython image detected — padded to ${(bytes.length / 1048576).toFixed(1)} MiB with vfs partition`);
    } catch (err) {
      setStatus(`MicroPython pad failed: ${err.message}`);
      return;
    }
  }
  loadFlash(bytes);
});

async function loadFromUrl(url, keyHex) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`fetch ${url} -> ${res.status}`);
  const buf = await res.arrayBuffer();
  loadFlash(new Uint8Array(buf), keyHex);
}

// ── MicroPython REPL preset ──
// Boots a stock MicroPython GENERIC_S3 image (Release .bin at flash
// offset 0, like esptool `write_flash 0`), pads it to 3 MiB, appends the
// littlefs "vfs" partition (DATA 0x82 @ 0x200000, 1 MiB) the firmware
// mounts at boot, recomputes the partition-table MD5, and boots it.
// The default URL is the same-origin tools/firmware/ copy committed in
// the repo (no download — works offline); any other URL may be pasted.
// MicroPython's REPL listens on UART0 (not USB-CDC), so the send port
// flips to UART0 and the input box gets a try-it snippet. Provenance:
// tools/micropython_repl.sh asserts the same image boots to `>>> ` and
// evaluates `print(6*7)` → `42` plus the float family headlessly.
// NOTE: upstream micropython.org serves no CORS header, so fetching from
// there fails — self-host the file or use the bundled default; any
// failure lands in the status line.
const MP_VFS_OFFSET = 0x200000;
const MP_VFS_SIZE = 0x100000;
const MP_PAD_SIZE = 0x300000;

function mpPadAndPartition(raw) {
  // Pad to the 3 MiB the partition table addresses (erased flash = 0xFF),
  // then write the vfs record + MD5 exactly like tools/micropython_repl.sh.
  const out = new Uint8Array(MP_PAD_SIZE).fill(0xff);
  if (raw.length > MP_PAD_SIZE) throw new Error(`image too large: ${raw.length} > ${MP_PAD_SIZE}`);
  out.set(raw, 0);
  const pt = 0x8000;
  const rec = new Uint8Array(32);
  rec[0] = 0xaa; rec[1] = 0x50; rec[2] = 1; rec[3] = 0x82; // DATA, sub 0x82
  new DataView(rec.buffer).setUint32(4, MP_VFS_OFFSET, true);
  new DataView(rec.buffer).setUint32(8, MP_VFS_SIZE, true);
  rec.set([0x76, 0x66, 0x73, 0x00], 12); // label "vfs"
  out.set(rec, pt + 0x60);
  // MD5 covers every 32-byte record BEFORE the MD5 marker itself
  // (verified against MicroPython's pristine table: md5(records[0..4])).
  const prefix = out.slice(pt, pt + 0x80);
  const sum = mpMd5(prefix);
  const mrec = new Uint8Array(32);
  mrec[0] = 0xeb; mrec[1] = 0xeb; // end marker
  mrec.set(sum, 16);
  out.set(mrec, pt + 0x80);
  return out;
}

// MD5 (RFC 1321) over a byte array — tiny self-contained port so the page
// needs no dependency; returns the 16-byte digest. Only used for the
// 128-byte partition-table prefix above.
function mpMd5(msg) {
  const s = [7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21];
  const K = [];
  for (let i = 0; i < 64; i++) K[i] = Math.floor(Math.abs(Math.sin(i + 1)) * 0x100000000) >>> 0;
  const l = msg.length;
  const bitLen = l * 8;
  const withPad = (((l + 8) >> 6) + 1) * 64;
  const m = new Uint8Array(withPad);
  m.set(msg, 0);
  m[l] = 0x80;
  new DataView(m.buffer).setUint32(withPad - 8, bitLen >>> 0, true);
  new DataView(m.buffer).setUint32(withPad - 4, Math.floor(bitLen / 0x100000000), true);
  let [a0, b0, c0, d0] = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476];
  const w = new Uint32Array(16);
  const rotl = (x, n) => ((x << n) | (x >>> (32 - n))) >>> 0;
  for (let off = 0; off < withPad; off += 64) {
    for (let i = 0; i < 16; i++) w[i] = new DataView(m.buffer).getUint32(off + i * 4, true);
    let [a, b, c, d] = [a0, b0, c0, d0];
    for (let i = 0; i < 64; i++) {
      let f, g;
      if (i < 16) { f = (b & c) | (~b & d); g = i; }
      else if (i < 32) { f = (d & b) | (~d & c); g = (5 * i + 1) % 16; }
      else if (i < 48) { f = b ^ c ^ d; g = (3 * i + 5) % 16; }
      else { f = c ^ (b | ~d); g = (7 * i) % 16; }
      f = (f + a + K[i] + w[g]) >>> 0;
      a = d; d = c; c = b;
      b = (b + rotl(f, s[i])) >>> 0;
    }
    a0 = (a0 + a) >>> 0; b0 = (b0 + b) >>> 0; c0 = (c0 + c) >>> 0; d0 = (d0 + d) >>> 0;
  }
  const out = new Uint8Array(16);
  const dv = new DataView(out.buffer);
  dv.setUint32(0, a0, true); dv.setUint32(4, b0, true);
  dv.setUint32(8, c0, true); dv.setUint32(12, d0, true);
  return out;
}

async function loadMicroPython() {
  const url = els.mpUrl ? els.mpUrl.value.trim() : '';
  if (!url) { setStatus('MicroPython: paste an image URL first'); return; }
  try {
    setStatus('MicroPython: downloading image…');
    const res = await fetch(url);
    if (!res.ok) throw new Error(`fetch ${url} -> ${res.status}`);
    const raw = new Uint8Array(await res.arrayBuffer());
    if (raw.length < 0x100000 || raw[0] !== 0xe9) {
      throw new Error(`not a flash image (size ${raw.length}, magic 0x${(raw[0] || 0).toString(16)}) — want the Release .bin at flash offset 0`);
    }
    setStatus('MicroPython: padding flash + vfs partition…');
    const flash = mpPadAndPartition(raw);
    currentGalleryItem = null; // no fixture: uploaded/self-hosted bytes boot raw
    loadFlash(flash);
    // MicroPython's REPL listens on UART0 — flip the send port so typed
    // lines arrive where the REPL reads them, and offer a try-it snippet.
    if (els.serialPort) { els.serialPort.value = '0'; updateSerialPlaceholder(); }
    if (els.serialInput && !els.serialInput.value) els.serialInput.value = 'print(6*7)';
    setStatus(`MicroPython ready (${raw.length.toLocaleString()} B image) — press Run, wait for >>>`);
  } catch (err) {
    setStatus(`MicroPython: failed: ${err.message}`);
  }
}

if (els.mpLoad) els.mpLoad.addEventListener('click', loadMicroPython);

// ── Gallery ──
let currentGalleryItem = null;
try {
  const res = await fetch('./firmware/manifest.json');
  if (res.ok) {
    const list = await res.json();
    for (const item of list) {
      const opt = document.createElement('option');
      opt.value = `./firmware/${item.file}`;
      opt.dataset.key = item.key || '';
      opt.dataset.sdspi = item.sdspi ? '1' : '';
      opt.dataset.touch = item.touch || '';
      // Wi-Fi fixture specs ride as plain strings, set ONLY when the
      // manifest entry carries the key (absent = unarmed = dataset
      // undefined; present-but-empty = empty-air scan):
      // `wifi_scan` posts SCAN_DONE + records, `wifi_sta` completes the
      // association (CONNECTED + GOT_IP + disconnect leg), `wifi_ap` is a
      // JSON object {ssid, passphrase, channel} (firmware posts AP_START
      // itself; the engine stages config + LAN), `espnow` is a boolean
      // (virtual second node; engine invokes TX/RX in-firmware).
      if (item.wifi_scan !== undefined) opt.dataset.wifiScan = item.wifi_scan;
      if (item.wifi_sta !== undefined) opt.dataset.wifiSta = item.wifi_sta;
      if (item.wifi_ap !== undefined) opt.dataset.wifiAp = JSON.stringify(item.wifi_ap);
      if (item.espnow === true) opt.dataset.espnow = '1';
      opt.textContent = item.name;
      els.gallery.appendChild(opt);
    }
  }
} catch (_) { /* no manifest */ }

els.gallery.addEventListener('change', async (e) => {
  const sel = e.target.selectedOptions[0];
  const url = e.target.value;
  if (!url) return;
  // dataset.* is undefined when the manifest entry lacks the key
  // (unarmed) and a string — possibly empty (= empty-air scan) — when
  // present. No hasAttribute dance needed: undefined means absent.
  let wifiAp = null;
  try {
    wifiAp = sel.dataset.wifiAp !== undefined ? JSON.parse(sel.dataset.wifiAp) : null;
  } catch (_) {
    wifiAp = null;
  }
  currentGalleryItem = {
    sdspi: sel.dataset.sdspi === '1',
    touch: sel.dataset.touch || null,
    wifiScan: sel.dataset.wifiScan !== undefined ? sel.dataset.wifiScan : null,
    wifiSta: sel.dataset.wifiSta !== undefined ? sel.dataset.wifiSta : null,
    wifiAp,
    espnow: sel.dataset.espnow === '1',
  };
  try {
    setStatus(`loading ${url}…`);
    await loadFromUrl(url, sel.dataset.key || null);
  } catch (err) {
    setStatus(`failed: ${err.message}`);
  }
});

// ── Buttons ──
els.run.addEventListener('click', () => {
  if (!emu) return;
  startLoop();
  els.run.disabled = true;
  els.stop.disabled = false;
});

els.stop.addEventListener('click', () => {
  stopLoop();
  els.run.disabled = false;
  els.stop.disabled = true;
});

els.reset.addEventListener('click', () => {
  if (!flashBytes) return;
  loadFlash(flashBytes, flashKeyHex);
});

els.steps.addEventListener('input', () => {
  els.stepsVal.textContent = els.steps.value;
});

// ── Serial input (host → firmware) ──
// Sends the typed line to the selected port's RX FIFO. Enter appends the
// newline the firmware's line readers (`readStringUntil`, REPL) wait for;
// Shift+Enter sends without it. The FIFO caps at the 128-byte hardware
// depth (silicon drops overrun bytes), so long pastes are chunked.
function sendSerial() {
  if (!emu || !els.serialInput) return;
  let text = els.serialInput.value;
  if (!text) return;
  if (!/\r|\n$/.test(text)) text += '\n';
  const bytes = new TextEncoder().encode(text);
  const port = els.serialPort ? els.serialPort.value : 'usb';
  const CHUNK = 96; // stay under the 128-byte FIFO with margin
  for (let i = 0; i < bytes.length; i += CHUNK) {
    const slice = bytes.slice(i, i + CHUNK);
    if (port === 'usb') {
      emu.usb_inject_rx(slice);
    } else {
      emu.uart_inject_rx(parseInt(port, 10), slice);
    }
  }
  // Echo what we sent (terminal-style) so the user sees it even if the
  // firmware doesn't echo.
  appendSerial(new TextEncoder().encode(`» ${text}`));
  els.serialInput.value = '';
  els.serialInput.focus();
}

if (els.serialSend) els.serialSend.addEventListener('click', sendSerial);
if (els.serialInput) els.serialInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' && !e.shiftKey) {
    e.preventDefault();
    sendSerial();
  }
});

// Keep the input placeholder in sync with the selected port so it is
// always obvious where keystrokes will go.
function updateSerialPlaceholder() {
  if (!els.serialInput) return;
  const port = els.serialPort ? els.serialPort.value : 'usb';
  const name = port === 'usb' ? 'USB-CDC (Serial)' : `UART${port}`;
  els.serialInput.placeholder = `Send to ${name}, Enter = send + newline…`;
}

if (els.serialPort) els.serialPort.addEventListener('change', updateSerialPlaceholder);
updateSerialPlaceholder();

// ── Init ──
initGpio();
await init();
setStatus('wasm ready — pick an example or load a merged.bin');
try {
  const res = await fetch('./firmware/esp32s3_hello.merged.bin');
  if (res.ok) {
    const buf = await res.arrayBuffer();
    loadFlash(new Uint8Array(buf));
  }
} catch (_) {
  /* no bundled firmware */
}
