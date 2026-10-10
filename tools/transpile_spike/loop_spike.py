#!/usr/bin/env python3
"""SPIKE Stage 2: control flow (bnez-style counted loop, 1M iterations).
Proves backward branches + structured loop nesting:
  a2 = 1000000 (cnt); a3 = 0 (acc);
  L: a3 += 1; a2 += -1; if (a2 != 0) goto L;
return i64 acc (=1000000). Measures effective MIPS for the S2 gate."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, LG, LS, I32C, I32ADD, END

BLOCK, LOOP, BRIF = 0x02, 0x03, 0x0D
EMPTY = 0x40

body = bytearray()
body += bytes([I32C]) + sleb(1_000_000) + bytes([LS, 2])  # cnt
body += bytes([I32C]) + sleb(0) + bytes([LS, 3])          # acc
body += bytes([BLOCK, EMPTY, LOOP, EMPTY])                # block$exit { loop$L {
body += bytes([LG, 3, I32C]) + sleb(1) + bytes([I32ADD, LS, 3])   # acc+=1
body += bytes([LG, 2, I32C]) + sleb(-1) + bytes([I32ADD, LS, 2])  # cnt+=-1
body += bytes([LG, 2, BRIF, 0x00])                        # bnez -> loop start
body += bytes([END, END])                                 # } }
body += bytes([LG, 3, 0xAD, END])                         # return i64(acc)

code = uleb(1) + uleb(len(body) + 3) + bytes([0x01, 0x10, 0x7F]) + bytes(body)
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(1) + bytes([0x60, 0x00, 0x01, 0x7E]))
    + section(3, uleb(1) + bytes([0x00]))
    + section(7, uleb(1) + bytes([0x03]) + b"run" + bytes([0x00, 0x00]))
    + section(10, code)
)
with open("/tmp/opencode/spike_loop.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; expect run()=1000000")
