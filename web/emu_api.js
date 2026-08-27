// rp2040js / Wokwi-style virtual-peripheral bridge for the ESP32-S3 emulator.
//
// The Rust `Emulator` (see crates/wasm-bridge) exposes a flat event stream via
// `drain_events()` plus injection hooks (`spi_inject_miso` / `spi_take_tx` /
// `i2c_inject_rx`). Those are the lowest-level building blocks: one event queue
// drained once per frame, exactly like rp2040js's per-frame pin sampling, so it
// is free even at high step rates. This module turns that stream into the
// ergonomic callback API Wokwi-style virtual devices expect, and routes the
// device's responses back into the firmware.
//
// Usage:
//   import { PeripheralBridge } from './emu_api.js';
//   const bridge = new PeripheralBridge(emu);
//   bridge.gpio.onChange((pin, level) => { /* LED matrix, button, ... */ });
//   bridge.spi.onTransfer((chan, tx) => { return rxBytes; });   // inject MISO
//   bridge.i2c.onRead((chan) => 0x57);                            // inject RX byte
//   // inside the animation loop, right after emu.step(n):
//   bridge.dispatch();

export const EVT = {
  GPIO: 0,
  SPI_XFER: 1,
  I2C_START: 2,
  I2C_WRITE: 3,
  I2C_READ: 4,
  I2C_STOP: 5,
};

export class PeripheralBridge {
  constructor(emu) {
    this.emu = emu;
    this.gpio = new GpioPeripheral();
    this.spi = new SpiPeripheral();
    this.i2c = new I2cPeripheral();
  }

  // Drain the emulator's event queue and dispatch each event to the matching
  // virtual peripheral. Call once per animation frame, after `emu.step(n)`.
  dispatch() {
    const events = this.emu.drain_events();
    for (const e of events) {
      switch (e.kind) {
        case EVT.GPIO:
          this.gpio._emit(e.a, e.b);
          break;
        case EVT.SPI_XFER: {
          // The MCU's transmitted bytes are available via spi_take_tx; a
          // virtual device returns its MISO bytes (or undefined for none).
          const tx = this.emu.spi_take_tx(e.a);
          const rx = this.spi._emit(e.a, tx);
          if (rx && rx.length) this.emu.spi_inject_miso(e.a, rx);
          break;
        }
        case EVT.I2C_START:
          this.i2c._emitStart(e.a);
          break;
        case EVT.I2C_WRITE:
          this.i2c._emitWrite(e.a, e.b);
          break;
        case EVT.I2C_READ: {
          // A virtual slave returns the byte it drives onto SDA; undefined /
          // null means "no device" (the MCU reads 0xFF).
          const byte = this.i2c._emitRead(e.a);
          if (byte !== undefined && byte !== null) {
            this.emu.i2c_inject_rx(e.a, new Uint8Array([byte & 0xff]));
          }
          break;
        }
        case EVT.I2C_STOP:
          this.i2c._emitStop(e.a);
          break;
      }
    }
  }
}

class GpioPeripheral {
  constructor() { this._cbs = []; }
  // cb(pin, level) — called on every observed GPIO output edge.
  onChange(cb) { this._cbs.push(cb); }
  _emit(pin, level) { for (const cb of this._cbs) cb(pin, level); }
}

class SpiPeripheral {
  constructor() { this._cbs = []; }
  // cb(chan, txBytes: Uint8Array) -> rxBytes: Uint8Array | undefined
  // chan 0 = GPSPI2, 1 = GPSPI3.
  onTransfer(cb) { this._cbs.push(cb); }
  _emit(chan, tx) {
    let rx;
    for (const cb of this._cbs) rx = cb(chan, tx);
    return rx;
  }
}

class I2cPeripheral {
  constructor() {
    this._start = [];
    this._write = [];
    this._read = [];
    this._stop = [];
  }
  // chan 0 = I2CEXT0, 1 = I2CEXT1.
  onStart(cb) { this._start.push(cb); }
  // cb(chan, byte) — a master WRITE data byte.
  onWrite(cb) { this._write.push(cb); }
  // cb(chan) -> byte | undefined — a master READ; return the slave byte.
  onRead(cb) { this._read.push(cb); }
  onStop(cb) { this._stop.push(cb); }
  _emitStart(chan) { for (const cb of this._start) cb(chan); }
  _emitWrite(chan, byte) { for (const cb of this._write) cb(chan, byte); }
  _emitRead(chan) {
    let out;
    for (const cb of this._read) out = cb(chan);
    return out;
  }
  _emitStop(chan) { for (const cb of this._stop) cb(chan); }
}
