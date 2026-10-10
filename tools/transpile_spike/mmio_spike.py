#!/usr/bin/env python3
"""SPIKE Stage 4a: MMIO import boundary (both directions).
Transpile-time rule proven here: RAM addrs -> direct mem ops, MMIO addrs
-> host import calls (static memmap proof, no runtime dispatch).
  mem[8] = 0x48 (direct store: RAM path);
  call soc_write32(0x60000000, mem[8]) (MMIO path);
  mem[12] = soc_read32(0x60000004) (canned 0x20);
  return i64 mem[8] | (mem[12] << 32).
Expect run()=0x48|(0x20<<32), JS log [(0x60000000,0x48)], read saw 0x60000004."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, I32C, I32LD, I32ST, END

CALL = 0x10
I64EX, I64SHL, I64OR = 0xAD, 0x86, 0x84


def C(v):
    return bytearray(bytes([I32C]) + sleb(v))


def st(slot, val):
    return C(slot) + val + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))


def ld(slot):
    return C(slot) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))


run = bytearray()
run += st(8, C(0x48))                                            # RAM direct
run += C(0x60000000) + ld(8) + bytes([CALL, 0x00])               # MMIO write
run += C(0x60000004) + bytes([CALL, 0x01])                      # MMIO read -> [status]
run += C(12)                                                    # [status, 12]... order!
# store needs [addr, val]: currently [status]; push addr FIRST was wrong order.
# Rebuild: addr then val. Drop the two lines above, redo:
run = bytearray()
run += st(8, C(0x48))
run += C(0x60000000) + ld(8) + bytes([CALL, 0x00])
run += C(12) + C(0x60000004) + bytes([CALL, 0x01])               # [12]; call->_ [12, st]
run += bytes([I32ST]) + uleb(2) + uleb(0)                       # mem[12] = status
run += ld(8) + bytes([0xAD])                                    # i64(0x48)
run += ld(12) + bytes([0xAD, 0x42]) + sleb(32) + bytes([0x86, 0x84, END])
run_code = uleb(len(run) + 1) + bytes([0x00]) + bytes(run)

memsec = uleb(1) + bytes([0x00, 0x01])
imps = (uleb(2)
        + bytes([0x03]) + b"env" + bytes([0x0B]) + b"soc_write32" + bytes([0x00, 0x01])
        + bytes([0x03]) + b"env" + bytes([0x0A]) + b"soc_read32" + bytes([0x00, 0x02]))
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(3) + bytes([0x60, 0x00, 0x01, 0x7E])
              + bytes([0x60, 0x02, 0x7F, 0x7F, 0x00])
              + bytes([0x60, 0x01, 0x7F, 0x01, 0x7F]))
    + section(2, imps)
    + section(3, uleb(1) + bytes([0x00]))
    + section(5, memsec)
    + section(7, uleb(2) + bytes([0x03]) + b"run" + bytes([0x00, 0x02])
              + bytes([0x03]) + b"mem" + bytes([0x02, 0x00]))
    + section(10, uleb(1) + run_code)
)
with open("/tmp/opencode/spike_mmio.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; expect run()=0x2000000048 log=[(0x60000000,72)]")
