#!/usr/bin/env python3
"""wcond runtime KATs: every branch-family arm emits a condition matching
branch_taken_py (exec.rs mirror) on preset regs. Usage: wcond_kat.py."""
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import (uleb, sleb, section, exports, funcbody, I32C, I32ST,
                   I32EQZ, END)
from windiff import C, wcond, branch_taken_py, wphys

GLG = 0x23


def mk(*vs):
    return [{"v": v} for v in vs]


# (opc, opnds(with dummy tgt), regs preset)
REGSET = {2: 0b1100, 3: 0b1010, 4: 0xFFFFFFFF, 5: 1, 6: 0, 7: 7}
CASES = [
    ("bne", mk(2, 3, 0)), ("beq", mk(2, 3, 0)),
    ("blt", mk(4, 5, 0)), ("bge", mk(4, 5, 0)),
    ("bltu", mk(5, 4, 0)), ("bgeu", mk(4, 5, 0)),
    ("beqi", mk(3, 1, 0)), ("bnei", mk(3, 1, 0)),
    ("blti", mk(4, -3 & 0xFFFFFFFF, 0)), ("bgei", mk(5, -3 & 0xFFFFFFFF, 0)),
    ("bltui", mk(5, 1, 0)), ("bgeui", mk(5, 1, 0)),
    ("ball", mk(2, 3, 0)), ("bnall", mk(2, 3, 0)),
    ("bany", mk(2, 3, 0)), ("bnone", mk(2, 3, 0)),
    ("bbc", mk(2, 3, 0)), ("bbs", mk(2, 3, 0)),
    ("bbci", mk(2, 2, 0)), ("bbsi", mk(2, 2, 0)),
    ("bltz", mk(4, 0)), ("bgez", mk(4, 0)), ("bgez", mk(6, 0)),
    ("bnez", mk(5, 0)), ("bnez_n", mk(5, 0)),
    ("beqz", mk(6, 0)), ("beqz_n", mk(6, 0)),
]
R = [0] * 16
for k, v in REGSET.items():
    R[k] = v
expected = [1 if branch_taken_py(opc, o, R)[0] else 0 for opc, o in CASES]

body = bytearray()
for i, (opc, o) in enumerate(CASES):
    body += C(0x1000 + i * 4)  # addr
    body += wcond(opc, o)
    body += bytes([I32EQZ, I32EQZ])  # normalize to 0/1
    body += bytes([0x36, 0x02, 0x00])  # i32.store
body += bytes([END])

# AR preset via data segment (wb=0 -> phys slot r*4).
ardata = b"".join(v.to_bytes(4, "little") for v in R[:16]) + b"\x00" * (64 * 4 - 64)
segs = (uleb(2) + bytes([0x00, I32C]) + sleb(0) + bytes([END])
        + uleb(len(ardata)) + ardata
        + bytes([0x00, I32C]) + sleb(0x1000) + bytes([END]) + uleb(80) + bytes(80))
mod = (b"\x00asm\x01\x00\x00\x00"
       + section(1, uleb(1) + bytes([0x60, 0x00, 0x00]))
       + section(3, uleb(1) + bytes([0x00]))
       + section(5, uleb(1) + bytes([0x00, 0x02]))
       + section(6, uleb(1) + bytes([0x7F, 0x01, I32C]) + sleb(0) + bytes([END]))
       + section(7, exports([(b"run", 0x00, 0x00), (b"mem", 0x02, 0x00)]))
       + section(10, uleb(1) + funcbody(bytes(body), 0))
       + section(11, segs))
open("/tmp/opencode/wcond_kat.wasm", "wb").write(mod)
print("cases:", len(CASES), "expected:", "".join(map(str, expected)))
open("/tmp/opencode/wcond_kat.txt", "w").write("".join(map(str, expected)))
