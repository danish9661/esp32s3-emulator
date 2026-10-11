#!/usr/bin/env python3
"""Tail-op KATs: nsau/shifts/div-family taken/not-taken + edges. Usage: tail_kat.py."""
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, exports, funcbody, I32C, I32ST, END
from windiff import C, wld, wst, walu, SAR_P

mk = lambda *vs: [{"v": v} for v in vs]
CASES = []
# nsau: clz
CASES += [("nsau", mk(3, 2), {2: 0x00F00000}, 8)]  # clz: bits 23-20 set
CASES += [("nsau", mk(3, 2), {2: 0x00000000}, 32)]
# ssr+sll: SAR=5 -> d = s << ((32-5)&63)=27
CASES += [("ssr", mk(2), {2: 5}, None), ("sll", mk(3, 4), {4: 1}, 1 << 27)]
# ssl+srl: SAR=32-3=29 -> srl shifts by 29
CASES += [("ssl", mk(2), {2: 3}, None), ("srl", mk(3, 4), {4: 0x80000000}, 4)]
# sra sign-fill: SAR=31 -> 0x80000000>>31 = 0xFFFFFFFF
CASES += [("ssr", mk(2), {2: 31}, None), ("sra", mk(3, 4), {4: 0x80000000}, 0xFFFFFFFF)]
# src: ({s,t}>>sh): s=1,t=0,sh=33 -> ((1<<32)>>33)=0
CASES += [("ssr", mk(2), {2: 33}, None), ("src", mk(3, 4, 5), {4: 1, 5: 0}, 0x80000000)]  # ssr masks: SAR=33&31=1; (1<<32)>>1
# src: s=3,t=0xFFFFFFFF,sh=4 -> ((3<<32|FFFFFFFF)>>4) low32 = 0x3FFFFFFF
CASES += [("ssr", mk(2), {2: 4}, None), ("src", mk(3, 4, 5), {4: 3, 5: 0xFFFFFFFF}, 0x3FFFFFFF)]
# div family edges
CASES += [("quou", mk(3, 4, 5), {4: 7, 5: 2}, 3)]
CASES += [("quou", mk(3, 4, 5), {4: 7, 5: 0}, 0xFFFFFFFF)]
CASES += [("quos", mk(3, 4, 5), {4: 0xFFFFFFF9, 5: 2}, 0xFFFFFFFD)]  # -7/2 = -3 trunc-toward-zero
CASES += [("quos", mk(3, 4, 5), {4: 7, 5: 0}, 0x7FFFFFFF)]
CASES += [("quos", mk(3, 4, 5), {4: 0x80000000, 5: 0xFFFFFFFF}, 0x80000000)]
CASES += [("rems", mk(3, 4, 5), {4: 0xFFFFFFF9, 5: 2}, 0xFFFFFFFF)]  # -7%2 = -1
CASES += [("rems", mk(3, 4, 5), {4: 7, 5: 0}, 7)]
CASES += [("remu", mk(3, 4, 5), {4: 7, 5: 3}, 1)]
CASES += [("remu", mk(3, 4, 5), {4: 7, 5: 0}, 7)]
# ssai + ssa8l: ssai 6 -> srl by 6: 0xFF>>6=3
CASES += [("ssai", mk(6), {}, None), ("srl", mk(3, 4), {4: 0xFF}, 3)]
# ssa8b as=1 -> SAR=32-((1<<3)&31)=24 -> sll by (32-24)=8: 1<<8=256
CASES += [("ssa8b", mk(2), {2: 1}, None), ("sll", mk(3, 4), {4: 1}, 256)]
# ssa8l as=1 -> SAR=(1<<3)&31=8 -> srl 0xFF00>>8=0xFF
CASES += [("ssa8l", mk(2), {2: 1}, None), ("srl", mk(3, 4), {4: 0xFF00}, 0xFF)]

body = bytearray()
nres = 0
for opc, o, preset, want in CASES:
    if opc in ("ssr", "ssl", "ssai", "ssa8b", "ssa8l"):
        for r, v in preset.items():
            body += wst(r, C(v))
        walu(opc, o, body)
    else:
        for r, v in preset.items():
            body += wst(r, C(v))
        walu(opc, o, body)
        body += C(0x3000 + nres * 4) + wld(3) + bytes([I32ST]) + uleb(2) + uleb(0)
        nres += 1
body += bytes([END])
R = [0] * 64
ardata = b"".join(v.to_bytes(4, "little") for v in R)
segs = (uleb(2) + bytes([0x00, I32C]) + sleb(0) + bytes([END])
        + uleb(len(ardata)) + ardata
        + bytes([0x00, I32C]) + sleb(0x3000) + bytes([END]) + uleb(4 * nres) + bytes(4 * nres))
mod = (b"\x00asm\x01\x00\x00\x00"
       + section(1, uleb(1) + bytes([0x60, 0x00, 0x00]))
       + section(3, uleb(1) + bytes([0x00]))
       + section(5, uleb(1) + bytes([0x00, 0x01]))
       + section(6, uleb(1) + bytes([0x7F, 0x01, I32C]) + sleb(0) + bytes([END]))
       + section(7, exports([(b"run", 0x00, 0x00), (b"mem", 0x02, 0x00)]))
       + section(10, uleb(1) + funcbody(bytes(body), 3))
       + section(11, segs))
open("/tmp/opencode/tail_kat.wasm", "wb").write(mod)
want = [w for _, _, _, w in CASES if w is not None]
print("results:", len(want), "want:", " ".join("%x" % w for w in want))
open("/tmp/opencode/tail_kat.txt", "w").write(" ".join("%x" % w for w in want))
