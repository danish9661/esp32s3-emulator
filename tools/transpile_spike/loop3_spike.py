#!/usr/bin/env python3
"""SPIKE Stage 2c: honest memory-bound loop (streaming sum, unfoldable).
  i = 0; acc = 0; L: acc += load[i*4]; i += 1; if (i != N) goto L;
N = 16384 words (1 page). JS seeds + computes expected independently."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, LG, LS, I32C, I32ADD, I32LD, END

BLOCK, LOOP, BRIF = 0x02, 0x03, 0x0D
EMPTY = 0x40
I32NE, I32SHL, I32MUL = 0x47, 0x74, 0x6C

N = 16384
body = bytearray()
body += bytes([I32C]) + sleb(0) + bytes([LS, 2])          # i = 0
body += bytes([I32C]) + sleb(0) + bytes([LS, 3])          # acc = 0
body += bytes([BLOCK, EMPTY, LOOP, EMPTY])
body += bytes([LG, 2, I32C]) + sleb(2) + bytes([I32SHL])  # i*4
body += bytes([I32LD]) + uleb(2) + uleb(0)                # load[i*4]
body += bytes([LG, 3]) + bytes([I32ADD, LS, 3])           # acc += w
body += bytes([LG, 2, I32C]) + sleb(1) + bytes([I32ADD, LS, 2])  # i += 1
body += bytes([LG, 2, I32C]) + sleb(N) + bytes([I32NE])   # i != N?
body += bytes([BRIF, 0x00])
body += bytes([END, END])
body += bytes([LG, 3, 0xAD, END])                         # return i64(acc)

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
with open("/tmp/opencode/spike_loop3.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; N={N}")
