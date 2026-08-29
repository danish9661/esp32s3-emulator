// Demo virtual peripherals that talk to the emulator through PeripheralBridge.
//
// These stand in for real Wokwi-style parts so the browser can exchange data
// with firmware that expects I2C/SPI slaves. They are intentionally tiny — the
// point is to show the round-trip: firmware writes land in JS, firmware reads
// come from JS, and every byte is surfaced in the UI.
//
// The I2C model here is a single addressed sensor at `addr`: the first byte of
// a transaction is the (address << 1 | rw) header and is ignored; subsequent
// WRITE bytes are commands the device stores, and every READ returns (and
// increments) a register value.

export class VirtualI2CSensor {
  constructor(addr = 0x42) {
    this.addr = addr;
    this.reg = 0x57;        // value the firmware will read back
    this.lastWrite = null;  // last command byte from firmware
    this.onActivity = null; // (text) => void
  }

  _isAddressByte(b) {
    return b === (this.addr << 1) || b === ((this.addr << 1) | 1);
  }

  handleWrite(chan, byte) {
    if (this._isAddressByte(byte)) return; // the I2C header, not real data
    this.lastWrite = byte;
    if (this.onActivity) {
      this.onActivity(`I2C${chan} ← 0x${byte.toString(16).padStart(2, '0')} (firmware→device)`);
    }
  }

  handleRead(chan) {
    const b = this.reg;
    this.reg = (this.reg + 1) & 0xff; // change so repeated reads are visible
    if (this.onActivity) {
      this.onActivity(`I2C${chan} → 0x${b.toString(16).padStart(2, '0')} (device→firmware)`);
    }
    return b;
  }

  handleStop() {}
}

export class VirtualSpiAdc {
  constructor(value = 0xaa) {
    this.value = value;     // MISO byte returned to firmware
    this.mosi = null;       // last MOSI bytes from firmware
    this.onActivity = null;
  }

  // chan: 0 = GPSPI2, 1 = GPSPI3. tx: Uint8Array of MOSI bytes.
  // Return a Uint8Array of MISO bytes (same length).
  handleTransfer(chan, tx) {
    this.mosi = tx;
    const hex = [...tx].map((b) => '0x' + b.toString(16).padStart(2, '0')).join(',');
    if (this.onActivity) {
      this.onActivity(`SPI${chan} MOSI=[${hex}] → MISO 0x${this.value.toString(16).padStart(2, '0')}`);
    }
    return new Uint8Array(tx.length).fill(this.value);
  }
}
