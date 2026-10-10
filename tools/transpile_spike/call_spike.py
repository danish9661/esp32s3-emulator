#!/usr/bin/env python3
"""SPIKE Stage 3a: calls (CALL0 mechanism: link + transfer + return).
AR file = 64 i32 words at mem[0..256), shared by both functions.
Stack discipline: i32.store pops value (top), then address — push addr,
then value.
  $callee: mem[8] = mem[8] + 10; return.
  $caller: mem[8] = 7; mem[0] = LINKMARK (ra write); call $callee;
           mem[12] = 1; return i64 mem[8] | (mem[12] << 32).
Expect run()=17 | (1<<32) = 4294967313 and mem[0]==LINKMARK."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, LG, LS, I32C, I32ADD, I32LD, I32ST, END

LINKMARK = 0xBEEF00
CALL = 0x10
A2S, A0S, A3S = 8, 0, 12  # AR slots (bytes): a2, a0, a3


def store(slot: int, val_ops: bytearray) -> bytearray:
    out = bytearray(bytes([I32C]) + sleb(slot))  # address first
    out += val_ops                                # then value on top
    out += bytes([I32ST]) + uleb(2) + uleb(0)
    return out


def load(slot: int) -> bytearray:
    return bytearray(bytes([I32C]) + sleb(slot) + bytes([I32LD]) + uleb(2) + uleb(0))


def const(v: int) -> bytearray:
    return bytearray(bytes([I32C]) + sleb(v))


# $callee (func 1): mem[8] = mem[8] + 10 (addr pushed first, value on top).
cal = bytearray(bytes([I32C]) + sleb(A2S)) + load(A2S) + bytes([I32C]) + sleb(10) + bytes([I32ADD])
cal += bytes([I32ST]) + uleb(2) + uleb(0)
cal += bytes([END])
callee_code = uleb(len(cal) + 1) + bytes([0x00]) + bytes(cal)

# $caller (func 0).
run = bytearray()
run += store(A2S, const(7))
run += store(A0S, const(LINKMARK))
run += bytes([CALL, 0x01])
run += store(A3S, const(1))
run += load(A2S) + bytes([0xAD])                  # i64(a2)
run += load(A3S) + bytes([0xAD, 0x42]) + sleb(32) + bytes([0x86, 0x84, END])  # i64.const

caller_code = uleb(len(run) + 1) + bytes([0x00]) + bytes(run)
code = uleb(2) + caller_code + callee_code
memsec = uleb(1) + bytes([0x00, 0x01])
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(2) + bytes([0x60, 0x00, 0x01, 0x7E]) + bytes([0x60, 0x00, 0x00]))
    + section(3, uleb(2) + bytes([0x00, 0x01]))
    + section(5, memsec)
    + section(7, uleb(2) + bytes([0x03]) + b"run" + bytes([0x00, 0x00])
              + bytes([0x03]) + b"mem" + bytes([0x02, 0x00]))
    + section(10, code)
)
with open("/tmp/opencode/spike_call.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; expect run()=4294967313 mem[0]={hex(LINKMARK)}")
