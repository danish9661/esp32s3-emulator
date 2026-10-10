#!/usr/bin/env python3
"""SPIKE Stage 2b: honest loop (count from JS-seeded memory, opaque to V8).
  cnt = load@0; acc = 0; L: acc+=1; cnt+=-1; bnez L; return i64 acc.
Seed mem[0]=999983 -> expect 999983. Times the warmed run."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, LG, LS, I32C, I32ADD, I32LD, END

BLOCK, LOOP, BRIF = 0x02, 0x03, 0x0D
EMPTY = 0x40

body = bytearray()
body += bytes([I32C]) + sleb(0) + bytes([I32LD]) + uleb(2) + uleb(0) + bytes([LS, 2])
body += bytes([I32C]) + sleb(0) + bytes([LS, 3])
body += bytes([BLOCK, EMPTY, LOOP, EMPTY])
body += bytes([LG, 3, I32C]) + sleb(1) + bytes([I32ADD, LS, 3])
body += bytes([LG, 2, I32C]) + sleb(-1) + bytes([I32ADD, LS, 2])
body += bytes([LG, 2, BRIF, 0x00])
body += bytes([END, END])
body += bytes([LG, 3, 0xAD, END])

code = uleb(1) + uleb(len(body) + 3) + bytes([0x01, 0x10, 0x7F]) + bytes(body)
memsec = uleb(1) + bytes([0x00, 0x01])
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(1) + bytes([0x60, 0x00, 0x01, 0x7E]))
    + section(3, uleb(1) + bytes([0x00]))
    + section(5, memsec)
    + section(7, uleb(2) + bytes([0x03]) + b"run" + bytes([0x00, 0x00])
              + bytes([0x03]) + b"mem" + bytes([0x02, 0x00]))
    + section(10, code)
)
with open("/tmp/opencode/spike_loop2.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; seed mem[0], expect run()==seed")
