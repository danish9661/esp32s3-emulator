#!/usr/bin/env python3
"""movcc KATs: moveqz/movnez/movltz/movgez taken + not-taken. Usage: movcc_kat.py."""
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, exports, funcbody, I32C, I32ST, END
from windiff import C, wld, wst, walu

mk = lambda *vs: [{"v": v} for v in vs]
# regs: r2=0xAA (src), r3=0 (dest init 0xCC), r4=0 (cond-zero), r5=1, r6=-1
CASES = [
    # (opc, (d,s,t), init-dest, expected)
    ("moveqz", (3, 2, 4), 0xCC, 0xAA),   # t==0 -> move
    ("moveqz", (3, 2, 5), 0xCC, 0xCC),   # t!=0 -> keep
    ("movnez", (3, 2, 5), 0xCC, 0xAA),
    ("movnez", (3, 2, 4), 0xCC, 0xCC),
    ("movltz", (3, 2, 6), 0xCC, 0xAA),   # -1 < 0
    ("movltz", (3, 2, 5), 0xCC, 0xCC),
    ("movgez", (3, 2, 5), 0xCC, 0xAA),
    ("movgez", (3, 2, 6), 0xCC, 0xCC),
]
body = bytearray()
for i, (opc, (d, s, t), init, want) in enumerate(CASES):
    body += wst(3, C(init))
    walu(opc, mk(d, s, t), body)
    body += C(0x3000 + i * 4) + wld(3) + bytes([I32ST]) + uleb(2) + uleb(0)
body += bytes([END])
R = [0] * 64
R[2], R[3], R[4], R[5], R[6] = 0xAA, 0, 0, 1, 0xFFFFFFFF
ardata = b"".join(v.to_bytes(4, "little") for v in R)
segs = (uleb(2) + bytes([0x00, I32C]) + sleb(0) + bytes([END])
        + uleb(len(ardata)) + ardata
        + bytes([0x00, I32C]) + sleb(0x3000) + bytes([END]) + uleb(32) + bytes(32))
mod = (b"\x00asm\x01\x00\x00\x00"
       + section(1, uleb(1) + bytes([0x60, 0x00, 0x00]))
       + section(3, uleb(1) + bytes([0x00]))
       + section(5, uleb(1) + bytes([0x00, 0x01]))
       + section(6, uleb(1) + bytes([0x7F, 0x01, I32C]) + sleb(0) + bytes([END]))
       + section(7, exports([(b"run", 0x00, 0x00), (b"mem", 0x02, 0x00)]))
       + section(10, uleb(1) + funcbody(bytes(body), 0))
       + section(11, segs))
open("/tmp/opencode/movcc_kat.wasm", "wb").write(mod)
print("want:", " ".join("%x" % w for _, _, _, w in CASES))
