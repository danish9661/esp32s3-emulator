#!/usr/bin/env python3
"""SPIKE Stage 3b: windowed CALL4/ENTRY/RETW (engine-exact semantics).
Ground truth: exec.rs CALL4/ENTRY/RETW arms + P1 windowed_call_entry_retw.
Globals: wb(0)=0, ws(1)=1, ps(2)=0x40000 (WOE). AR file mem[0..256).
Fictional pcs (transfer MECHANISM proven; real pcs in S4):
  CALL4 ret const 0x40000103, RETW top const 0x40000000, SP preset 0x100.
Expect: mem[4]=0x40000103 mem[5]=0xE4 mem[6]=17 mem[256]=0x40000103
  wb=0 ws=1 ps=0x50000 run()=17|(0xE4<<32)."""

import sys
sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import (uleb, sleb, section, I32C, I32ADD, I32SUB, I32LD, I32ST,
                   I32AND, I32OR, I32XOR, I32SHL, I32SHRU, I32NE, I32EQZ,
                   GLG, GLS, UNREACH, END)

CALL = 0x10
LG, LS, MUL, SELECT, IF = 0x20, 0x21, 0x6C, 0x1B, 0x04
WOE_BIT, CI_SHIFT = 0x40000, 16


def C(v):
    return bytearray(bytes([I32C]) + sleb(v))


def phys(reg):
    """Byte addr of windowed reg: ((wb*4 + reg) & 63) * 4."""
    return (bytearray(bytes([GLG, 0])) + C(4) + bytearray([MUL]) + reg
            + bytearray([I32ADD]) + C(63) + bytearray([I32AND]) + C(2)
            + bytearray([I32SHL]))


def ld(reg):
    return phys(C(reg)) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))


def st(reg, val):
    return phys(C(reg)) + val + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))


def woe_check():
    return (bytearray(bytes([GLG, 2])) + C(WOE_BIT) + bytearray([I32AND])
            + bytearray([I32EQZ, IF, 0x40, UNREACH, END]))


def trap_if_zero(cond):
    return cond + bytearray([IF, 0x40, UNREACH, END])


# ---------------- $callee (func 1), temps t0=0,t1=1,t2=2 ----------------
cal = bytearray()
cal += woe_check()
cal += bytearray(bytes([GLG, 2])) + C(CI_SHIFT) + bytearray([I32SHRU])
cal += C(3) + bytearray([I32AND, LS, 0])          # t0 = callinc
# ENTRY: new_a1 = reg(1) - 32 -> phys((t0<<2)|1), caller window.
cal += bytes([GLG, 0]) + C(4) + bytearray([MUL])  # wb*4
cal += bytes([LG, 0]) + C(2) + bytearray([I32SHL])  # t0<<2
cal += C(1) + bytearray([I32OR, I32ADD])          # wb*4 + dest
cal += C(63) + bytearray([I32AND]) + C(2) + bytearray([I32SHL])  # addr
cal += ld(1) + C(32) + bytearray([I32SUB])        # val
cal += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
# wb_next = (wb + t0) & 15 -> t1; ws |= 1 << t1; wb = t1.
cal += bytes([GLG, 0, LG, 0, I32ADD]) + C(15) + bytearray([I32AND, LS, 1])
cal += bytearray(bytes([GLG, 1])) + C(1) + bytes([LG, 1])
cal += bytearray([I32SHL, I32OR, GLS, 1, LG, 1, GLS, 0])
# body: callee a2 += 10; callee a1 += 4.
cal += st(2, ld(2) + C(10) + bytearray([I32ADD]))
cal += st(1, ld(1) + C(4) + bytearray([I32ADD]))
# RETW: t0 = a0; t1 = n = a0 >> 30.
cal += woe_check()
cal += ld(0) + bytearray([LS, 0])
cal += bytes([LG, 0]) + C(30) + bytearray([I32SHRU, LS, 1])
# m = b0?1 : b1?2 : b2?3 : 0 -> t2; bk = (ws >> ((wb+15-k)&15)) & 1.
cal += C(1) + C(2) + C(3) + C(0)


def wbit(k):
    pos = bytearray(bytes([GLG, 0])) + C(15 - k) + bytearray([I32ADD])
    pos += C(15) + bytearray([I32AND])
    return bytearray(bytes([GLG, 1])) + pos + bytearray([I32SHRU]) + C(1) + bytearray([I32AND])


cal += wbit(2) + bytearray([SELECT]) + wbit(1) + bytearray([SELECT])
cal += wbit(0) + bytearray([SELECT, LS, 2])
cal += trap_if_zero(bytearray(bytes([LG, 1, I32EQZ])))          # n==0 -> trap
cal += bytearray(bytes([LG, 2, I32EQZ, I32EQZ]))                # m!=0
cal += bytearray(bytes([LG, 2, LG, 1, I32NE]))                  # m!=n
cal += bytearray([I32AND]) + trap_if_zero(bytearray())          # both -> trap
# underflow: ((ws >> ((wb+16-n)&15)) & 1) == 0 -> trap.
upos = (bytearray(bytes([GLG, 0])) + C(16) + bytearray([I32ADD])
        + bytes([LG, 1]) + bytearray([I32SUB]) + C(15) + bytearray([I32AND]))
cal += trap_if_zero(bytearray(bytes([GLG, 1])) + upos
                    + bytearray([I32SHRU]) + C(1) + bytearray([I32AND, I32EQZ]))
# ws &= ~(1 << wb); wb = (wb - n) & 15.
cal += (bytearray(bytes([GLG, 1])) + C(1) + bytearray(bytes([GLG, 0]))
        + bytearray([I32SHL]) + C(-1) + bytearray([I32XOR, I32AND, GLS, 1]))
cal += (bytearray(bytes([GLG, 0])) + bytes([LG, 1]) + bytearray([I32SUB])
        + C(15) + bytearray([I32AND, GLS, 0]))
# jump scratch mem[1024] = 0x40000000 | (a0 & 0x3FFFFFFF).
cal += (C(1024) + C(0x40000000) + bytes([LG, 0]) + C(0x3FFFFFFF)
        + bytearray([I32AND, I32OR]) + bytearray(bytes([I32ST]) + uleb(2) + uleb(0)))
cal += bytearray([END])
callee_code = uleb(len(cal) + 3) + bytes([0x01, 0x03, 0x7F]) + bytes(cal)

# ---------------- $caller (func 0) ----------------
run = bytearray()
run += st(1, C(0x100))                            # SP preset
run += st(6, C(7))                                # arg a6
run += st(4, C(1) + C(30) + bytearray([I32SHL]) + C(0x103) + bytearray([I32OR]))
run += (bytearray(bytes([GLG, 2])) + C(0x30000) + C(-1) + bytearray([I32XOR])
        + bytearray([I32AND]) + C(1) + C(16) + bytearray([I32SHL])
        + bytearray([I32OR, GLS, 2]))             # PS.CALLINC = 1
run += bytes([CALL, 0x01])
run += ld(6) + bytes([0xAD])                      # i64(caller a6)
run += ld(5) + bytes([0xAD, 0x42]) + sleb(32) + bytes([0x86, 0x84, END])
caller_code = uleb(len(run) + 1) + bytes([0x00]) + bytes(run)

code = uleb(2) + caller_code + callee_code
memsec = uleb(1) + bytes([0x00, 0x01])
glob = (uleb(3)
        + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0) + bytes([END])
        + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(1) + bytes([END])
        + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0x40000) + bytes([END]))
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(2) + bytes([0x60, 0x00, 0x01, 0x7E]) + bytes([0x60, 0x00, 0x00]))
    + section(3, uleb(2) + bytes([0x00, 0x01]))
    + section(5, memsec)
    + section(6, glob)
    + section(7, uleb(5) + bytes([0x03]) + b"run" + bytes([0x00, 0x00])
              + bytes([0x03]) + b"mem" + bytes([0x02, 0x00])
              + bytes([0x02]) + b"wb" + bytes([0x03, 0x00])
              + bytes([0x02]) + b"ws" + bytes([0x03, 0x01])
              + bytes([0x02]) + b"ps" + bytes([0x03, 0x02]))
    + section(10, code)
)
with open("/tmp/opencode/spike_win.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; expect mem[4]=0x40000103 mem[5]=0xE4 mem[6]=17 wb=0 ws=1 ps=0x50000")
