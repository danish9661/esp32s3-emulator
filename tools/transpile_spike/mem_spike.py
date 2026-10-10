#!/usr/bin/env python3
"""SPIKE Stage 1: direct-memory access (the transpiler's core advantage).
Proves RAM loads/stores compile to inline wasm mem ops (no call-outs):
  movi a6,0x7FF; movi a7,0x100; add a6,a6,a7 -> 0x8FF; store @0;
  load @0 -> a6; add a6,a6,a7(=0x100)? No: a7 was overwritten? Keep a7.
Block: a6=0x7FF, a7=0x100, a6=a6+a7=0x8FF, store a6@0, a6=load@0,
a6=a6+a7=0x9FF=2559. Return i64 a6; node also reads mem[0]==0x8FF."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, LG, LS, I32C, I32ADD, I32LD, I32ST, END

body = bytearray()
body += bytes([I32C]) + sleb(0x7FF) + bytes([LS, 6])          # movi a6,0x7FF
body += bytes([I32C]) + sleb(0x100) + bytes([LS, 7])          # movi a7,0x100
body += bytes([LG, 6, LG, 7, I32ADD, LS, 6])                  # add a6,a6,a7=0x8FF
body += bytes([I32C]) + sleb(0) + bytes([LG, 6])              # addr 0, val a6
body += bytes([I32ST]) + uleb(2) + uleb(0)                    # i32.store align=2 off=0
body += bytes([I32C]) + sleb(0) + bytes([I32LD]) + uleb(2) + uleb(0) + bytes([LS, 6])
body += bytes([LG, 6, LG, 7, I32ADD, LS, 6])                  # a6=0x8FF+0x100=0x9FF
# return i64(a6): extend then end (single i64 on stack).
body += bytes([LG, 6, 0xAD, END])

code = uleb(1) + uleb(len(body) + 3) + bytes([0x01, 0x10, 0x7F]) + bytes(body)
memsec = uleb(1) + bytes([0x00, 0x01])  # 1 memory, limits flags=0, min=1 page
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(1) + bytes([0x60, 0x00, 0x01, 0x7E]))
    + section(3, uleb(1) + bytes([0x00]))
    + section(5, memsec)
    + section(7, uleb(2) + bytes([0x03]) + b"run" + bytes([0x00, 0x00])
              + bytes([0x03]) + b"mem" + bytes([0x02, 0x00]))
    + section(10, code)
)
with open("/tmp/opencode/spike_mem.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; expect run()=2559 mem[0]=0x8FF")
