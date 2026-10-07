// Dual-engine IO worker: owns the virtual I2C/SPI devices when the UI
// selects Engine = Dual (see index.html + emu-worker.js).
//
// Why a second worker instead of doing this inline: the emulation worker's
// hot loop is step_batch + drains. Virtual-device dispatch (event decode,
// MOSI/MISO round-trips, activity logging) is pure JS that otherwise
// interleaves every batch. Offloading it keeps the stepping thread stepping;
// the 1-frame inject latency is unchanged by design (the old main-thread
// loop already injected one frame late — see the pre-prime comment in
// main.js history).
//
// Wiring: main.js creates a MessageChannel, posts one port to the emulation
// worker (`{cmd:'ioPort', port}`) and one here (`{cmd:'emuPort', port}`).
// Sockets stay in the UI thread; NET/BLE frames never come here (the
// emulation worker forwards those to main itself).

import { VirtualI2CSensor, VirtualSpiAdc } from './virtual_devices.js';

let port = null;
let sensor = null;
let adc = null;

function prime() {
  return { i2c: sensor ? sensor.reg & 0xff : 0x57, spi: adc ? adc.value & 0xff : 0xaa };
}

onmessage = (ev) => {
  const m = ev.data;
  if (!m) return;
  if (m.cmd === 'emuPort') {
    port = m.port;
    port.onmessage = onEmu;
    return;
  }
  if (m.cmd === 'attach') {
    sensor = new VirtualI2CSensor(0x42);
    adc = new VirtualSpiAdc(0xaa);
    const logs = [];
    sensor.onActivity = (t) => logs.push(t);
    adc.onActivity = (t) => logs.push(t);
    if (port) port.postMessage({ type: 'ioReady', prime: prime(), logs });
    return;
  }
};

function onEmu(ev) {
  const m = ev.data;
  if (!m || m.type !== 'vdevEvents' || !port) return;
  const injects = [];
  const logs = [];
  const pushLog = (t) => {
    if (logs.length < 50) logs.push(t);
  };
  const sLog = sensor ? sensor.onActivity : null;
  const aLog = adc ? adc.onActivity : null;
  if (sensor) sensor.onActivity = pushLog;
  if (adc) adc.onActivity = pushLog;
  try {
    for (const e of m.events) {
      if (e.kind === 1) {
        // SPI_XFER: payload carries the MOSI bytes (transferred).
        const tx = e.payload ? new Uint8Array(e.payload) : new Uint8Array(0);
        if (adc) {
          const rx = adc.handleTransfer(e.a, tx);
          if (rx && rx.length) {
            injects.push({ op: 'spiMiso', chan: e.a, bytes: rx.buffer });
          }
        }
      } else if (e.kind === 2) {
        // I2C_START: no device state (the address header arrives as WRITE).
      } else if (e.kind === 3) {
        if (sensor) sensor.handleWrite(e.a, e.b & 0xff);
      } else if (e.kind === 4) {
        if (sensor) {
          const b = sensor.handleRead(e.a);
          if (b !== undefined && b !== null) {
            const u8 = new Uint8Array([b & 0xff]);
            injects.push({ op: 'i2cRx', chan: e.a, bytes: u8.buffer });
          }
        }
      }
      // GPIO(0)/STOP(5)/NET(6)/BLE(7) never arrive here (filtered upstream).
    }
  } finally {
    if (sensor) sensor.onActivity = sLog;
    if (adc) adc.onActivity = aLog;
  }
  port.postMessage({ type: 'ioInjects', injects, prime: prime(), logs }, injects.map((i) => i.bytes));
}
