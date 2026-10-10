#!/usr/bin/env python3
"""SPIKE Stage 4b: interrupt poll + trap-to-interpreter signaling.
  cnt@8=512, acc@12=0, pollctr@16=0;
  L: acc+=1; cnt+=-1; pollctr+=1;
     if (pollctr==16) { pollctr=0; r=poll_irq();
       if (r!=0) { trap(0x40000200); return acc|(0xBAD<<32); } }
     if (cnt!=0) goto L;
  return acc|(0<<32).
Run 1 (poll always 0): completes, 32 polls, returns 512.
Run 2 (poll nonzero at 3rd poll): traps at iter 48, acc=48, 3 polls."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import (uleb, sleb, section, I32C, I32ADD, I32LD, I32ST, I32EQ, I32NE,
                   GLG, GLS, END)

CALL, RET = 0x10, 0x0F
BLOCK, LOOP, BRIF, IF = 0x02, 0x03, 0x0D, 0x04
EMPTY = 0x40
CNT, ACC, PCTR = 8, 12, 16
K, N = 16, 512
TRAP_PC, TRAP_CODE = 0x40000200, 0xBAD


def C(v):
    return bytearray(bytes([I32C]) + sleb(v))


def st(slot, val):
    return C(slot) + val + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))


def ld(slot):
    return C(slot) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))


def addmem(slot, delta):
    return C(slot) + ld(slot) + C(delta) + bytearray([I32ADD]) + bytearray(
        bytes([I32ST]) + uleb(2) + uleb(0))


def pack(acc_slot, code):
    return (ld(acc_slot) + bytes([0xAD]) + C(code) + bytes([0xAD, 0x42])
            + sleb(32) + bytes([0x86, 0x84]))


run = bytearray()
run += st(CNT, C(N)) + st(ACC, C(0)) + st(PCTR, C(0))
run += bytes([BLOCK, EMPTY, LOOP, EMPTY])
run += addmem(ACC, 1) + addmem(CNT, -1) + addmem(PCTR, 1)
# if (pollctr == K) {
run += ld(PCTR) + C(K) + bytearray([I32EQ, IF, EMPTY])
run += st(PCTR, C(0))
run += bytes([CALL, 0x00])                              # r = poll_irq() -> [r]
run += C(0) + bytearray([I32NE, IF, EMPTY])           # if (r != 0) {
run += C(TRAP_PC) + bytes([CALL, 0x01])               #   trap(pc)
run += pack(ACC, TRAP_CODE) + bytes([RET])            #   return trap pack
run += bytes([END, END])                              # } }
# if (cnt != 0) goto L
run += ld(CNT) + bytes([BRIF, 0x00])
run += bytes([END, END])                              # } }
run += pack(ACC, 0) + bytes([END])

run_code = uleb(len(run) + 1) + bytes([0x00]) + bytes(run)
memsec = uleb(1) + bytes([0x00, 0x01])
imps = (uleb(2)
        + bytes([0x03]) + b"env" + bytes([0x08]) + b"poll_irq" + bytes([0x00, 0x01])
        + bytes([0x03]) + b"env" + bytes([0x04]) + b"trap" + bytes([0x00, 0x02]))
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(3) + bytes([0x60, 0x00, 0x01, 0x7E])
              + bytes([0x60, 0x00, 0x01, 0x7F])
              + bytes([0x60, 0x01, 0x7F, 0x00]))
    + section(2, imps)
    + section(3, uleb(1) + bytes([0x00]))
    + section(5, memsec)
    + section(7, uleb(2) + bytes([0x03]) + b"run" + bytes([0x00, 0x02])
              + bytes([0x03]) + b"mem" + bytes([0x02, 0x00]))
    + section(10, uleb(1) + run_code)
)
with open("/tmp/opencode/spike_poll.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; run1: 512/32 polls; run2: trap at iter 48")
