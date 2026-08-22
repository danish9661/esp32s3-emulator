import init, { Emulator } from './pkg/wasm_bridge.js';

const NUM_PINS = 40; // visualize GPIO 0..39

const els = {
  firmware: document.getElementById('firmware'),
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
setStatus('wasm ready — load a merged.bin firmware');
try {
  const res = await fetch('./hello.bin');
  if (res.ok) {
    const buf = await res.arrayBuffer();
    loadFlash(new Uint8Array(buf));
  }
} catch (_) {
  /* no bundled firmware; user must upload */
}
