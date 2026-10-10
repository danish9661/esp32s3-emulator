#!/usr/bin/env python3
"""S4 bridge module: RAM direct ops + Soc imports served by a REAL Soc
(via wasmtime in transpile_run.rs — not JS stubs). Layout matches the
S4a proof; import surface frozen for S4: trap/read/write/poll.
  mem[8] = 0x48 (direct); soc_write32(0x60000000, mem[8]) (UART0 FIFO);
  mem[12] = soc_read32(0x6000001C) (UART0 STATUS); return pack."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, exports, funcbody, I32C, I32LD, I32ST, END

CALL = 0x10


def C(v):
    return bytearray(bytes([I32C]) + sleb(v))


def st(slot, val):
    return C(slot) + val + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))


def ld(slot):
    return C(slot) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))


run = bytearray()
run += st(8, C(0x48))
run += C(0x60000000) + ld(8) + bytes([CALL, 0x02])
run += C(12) + C(0x6000001C) + bytes([CALL, 0x01])
run += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
run += ld(8) + bytes([0xAD])
run += ld(12) + bytes([0xAD, 0x42]) + sleb(32) + bytes([0x86, 0x84, END])

memsec = uleb(1) + bytes([0x00, 0x01])
imps = (uleb(4)
        + bytes([0x03]) + b"env" + bytes([0x04]) + b"trap" + bytes([0x00, 0x01])
        + bytes([0x03]) + b"env" + bytes([0x0A]) + b"soc_read32" + bytes([0x00, 0x02])
        + bytes([0x03]) + b"env" + bytes([0x0B]) + b"soc_write32" + bytes([0x00, 0x03])
        + bytes([0x03]) + b"env" + bytes([0x08]) + b"poll_irq" + bytes([0x00, 0x04]))
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(5) + bytes([0x60, 0x00, 0x01, 0x7E])
              + bytes([0x60, 0x01, 0x7F, 0x00])
              + bytes([0x60, 0x01, 0x7F, 0x01, 0x7F])
              + bytes([0x60, 0x02, 0x7F, 0x7F, 0x00])
              + bytes([0x60, 0x00, 0x01, 0x7F]))
    + section(2, imps)
    + section(3, uleb(1) + bytes([0x00]))
    + section(5, memsec)
    + section(7, exports([(b"run", 0x00, 0x04), (b"mem", 0x02, 0x00)]))
    + section(10, uleb(1) + funcbody(bytes(run), 0))
)
open("/tmp/opencode/spike_s4bridge.wasm", "wb").write(mod)
print(f"bridge module: {len(mod)} bytes")
