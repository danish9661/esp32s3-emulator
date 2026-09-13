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

function renderGpio() {
  const mask = emu.gpio_output();
  if (mask === gpioPrevMask) return;
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

function tick() {
  const n = parseInt(els.steps.value, 10);
  if (bridge && vdevSensor && vdevAdc) {
    emu.i2c_inject_rx(0, new Uint8Array([vdevSensor.reg]));
    emu.spi_inject_miso(0, new Uint8Array([vdevAdc.value]));
  }
  const executed = emu.step_batch(n);
  totalSteps += executed;
  appendSerial(emu.uart_read());
  renderGpio();
  if (bridge) bridge.dispatch();
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
  if (keyHex) {
    emu.load_flash_encrypted(bytes, hexToBytes(keyHex));
  } else {
    emu.load_flash(bytes);
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
  const buf = await file.arrayBuffer();
  loadFlash(new Uint8Array(buf));
});

async function loadFromUrl(url, keyHex) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`fetch ${url} -> ${res.status}`);
  const buf = await res.arrayBuffer();
  loadFlash(new Uint8Array(buf), keyHex);
}

// ── Gallery ──
try {
  const res = await fetch('./firmware/manifest.json');
  if (res.ok) {
    const list = await res.json();
    for (const item of list) {
      const opt = document.createElement('option');
      opt.value = `./firmware/${item.file}`;
      opt.dataset.key = item.key || '';
      opt.textContent = item.name;
      els.gallery.appendChild(opt);
    }
  }
} catch (_) { /* no manifest */ }

els.gallery.addEventListener('change', async (e) => {
  const sel = e.target.selectedOptions[0];
  const url = e.target.value;
  if (!url) return;
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
