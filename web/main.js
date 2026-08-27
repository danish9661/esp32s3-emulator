import init, { Emulator } from './pkg/wasm_bridge.js';
import { PeripheralBridge } from './emu_api.js';

const NUM_PINS = 40; // visualize GPIO 0..39

const els = {
  firmware: document.getElementById('firmware'),
  gallery: document.getElementById('gallery'),
  run: document.getElementById('run'),
  stop: document.getElementById('stop'),
  reset: document.getElementById('reset'),
  steps: document.getElementById('steps'),
  stepsVal: document.getElementById('stepsVal'),
  status: document.getElementById('status'),
  console: document.getElementById('console'),
  gpio: document.getElementById('gpio'),
};

const leds = [];
for (let i = 0; i < NUM_PINS; i++) {
  const d = document.createElement('div');
  d.className = 'led';
  d.textContent = i;
  els.gpio.appendChild(d);
  leds.push(d);
}

let emu = null;
let bridge = null; // PeripheralBridge (virtual devices)
let flashBytes = null; // last loaded firmware, for reset
let timer = null;
let totalSteps = 0;

function setStatus(msg) {
  els.status.textContent = msg;
}

function appendSerial(bytes) {
  if (!bytes || bytes.length === 0) return;
  const text = new TextDecoder().decode(bytes);
  els.console.textContent += text;
  els.console.scrollTop = els.console.scrollHeight;
}

function renderGpio() {
  const mask = emu.gpio_output();
  for (let i = 0; i < NUM_PINS; i++) {
    leds[i].classList.toggle('on', (mask & (1 << i)) !== 0);
  }
}

function tick() {
  const n = parseInt(els.steps.value, 10);
  emu.step(n);
  totalSteps += n;
  appendSerial(emu.uart_read());
  renderGpio();
  if (bridge) bridge.dispatch(); // route SPI/I2C/GPIO events to virtual devices
  setStatus(`pc=0x${emu.pc().toString(16)}  steps=${totalSteps}`);
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

async function loadFlash(bytes) {
  stopLoop();
  emu = new Emulator();
  emu.load_flash(bytes);

  // Virtual-peripheral bridge (rp2040js-style event API). Adjust the
  // callbacks below to model real Wokwi-style parts (SPI flash, I2C sensors,
  // GPIO buttons, ...). The demo just logs I2C activity to the console.
  if (typeof PeripheralBridge !== 'undefined') {
    bridge = new PeripheralBridge(emu);
    bridge.i2c.onStart((chan) => console.log(`[i2c${chan}] START`));
    bridge.i2c.onWrite((chan, byte) => console.log(`[i2c${chan}] WRITE 0x${byte.toString(16)}`));
    bridge.i2c.onRead((chan) => { console.log(`[i2c${chan}] READ`); return undefined; });
    bridge.i2c.onStop((chan) => console.log(`[i2c${chan}] STOP`));
  }

  flashBytes = bytes;
  totalSteps = 0;
  els.console.textContent = '';
  renderGpio();
  setStatus(`loaded ${bytes.length} bytes`);
  els.run.disabled = false;
  els.stop.disabled = true;
  els.reset.disabled = false;
}

els.firmware.addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const buf = await file.arrayBuffer();
  loadFlash(new Uint8Array(buf));
});

async function loadFromUrl(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`fetch ${url} -> ${res.status}`);
  const buf = await res.arrayBuffer();
  loadFlash(new Uint8Array(buf));
}

// Populate the example-firmware gallery from a manifest (graceful if absent).
try {
  const res = await fetch('./firmware/manifest.json');
  if (res.ok) {
    const list = await res.json();
    for (const item of list) {
      const opt = document.createElement('option');
      opt.value = `./firmware/${item.file}`;
      opt.textContent = item.name;
      els.gallery.appendChild(opt);
    }
  }
} catch (_) { /* no manifest; gallery stays empty */ }

els.gallery.addEventListener('change', async (e) => {
  const url = e.target.value;
  if (!url) return;
  try {
    setStatus(`loading ${url} …`);
    await loadFromUrl(url);
  } catch (err) {
    setStatus(`failed: ${err.message}`);
  }
});

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
  loadFlash(flashBytes);
});

els.steps.addEventListener('input', () => {
  els.stepsVal.textContent = els.steps.value;
});

// Boot the WASM module, then try to auto-load a bundled demo firmware.
await init();
setStatus('wasm ready — pick an example or load a merged.bin');
try {
  const res = await fetch('./firmware/esp32s3_hello.merged.bin');
  if (res.ok) {
    const buf = await res.arrayBuffer();
    loadFlash(new Uint8Array(buf));
  }
} catch (_) {
  /* no bundled firmware; user must upload */
}
