#!/usr/bin/env python3
"""s32c1i + scompare1 KATs: CAS success/fail + old-value return, wsr/rsr
round-trip. Host provides stateful soc_cas32 over a JS map. Usage: scas_kat.py."""
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, exports, funcbody, I32C, I32ST, I32LD, END
from windiff import C, wld, wst, wphys, SCOMP_P, TRAP_I, RD_I, WR_I, CAS_I, CALL

GLG, GLS, LG, LS = 0x23, 0x24, 0x20, 0x21
mk = lambda *vs: [{"v": v} for v in vs]

LOCK = 0x3FC90000
body = bytearray()
# wsr_scompare1 a2 (a2 preset 0xB33FFFFF); rsr back to a3.
body += C(SCOMP_P) + wld(2) + bytes([I32ST]) + uleb(2) + uleb(0)
body += wst(3, C(SCOMP_P) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0)))
# CAS#1: mem==cmp -> store 0x11111111, t gets old. [t=0x11111111, eff=LOCK]
body += wphys(C(4)) + C(0x11111111) + C(LOCK)
body += bytes([LS, 2, LS, 0])
body += (bytes([LG, 2]) + C(SCOMP_P) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))
         + bytes([LG, 0, CALL, CAS_I]))
body += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
# CAS#2 (stale cmp, must NOT store; t gets current 0x11111111).
body += wphys(C(5)) + C(0x22222222) + C(LOCK)
body += bytes([LS, 2, LS, 0])
body += (bytes([LG, 2]) + C(SCOMP_P) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))
         + bytes([LG, 0, CALL, CAS_I]))
body += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
# report cells: a3 (rsr readback), a4 (cas1 old), a5 (cas2 old).
body += C(0x3000) + wld(3) + bytes([I32ST]) + uleb(2) + uleb(0)
body += C(0x3004) + wld(4) + bytes([I32ST]) + uleb(2) + uleb(0)
body += C(0x3008) + wld(5) + bytes([I32ST]) + uleb(2) + uleb(0)
body += bytes([END])
R = [0] * 64
R[2] = 0xB33FFFFF
ardata = b"".join(v.to_bytes(4, "little") for v in R)
segs = (uleb(1) + bytes([0x00, I32C]) + sleb(0) + bytes([END])
        + uleb(len(ardata)) + ardata)
mod = (b"\x00asm\x01\x00\x00\x00"
       + section(1, uleb(5) + bytes([0x60, 0x00, 0x00])
                 + bytes([0x60, 0x01, 0x7F, 0x00])
                 + bytes([0x60, 0x01, 0x7F, 0x01, 0x7F])
                 + bytes([0x60, 0x02, 0x7F, 0x7F, 0x00])
                 + bytes([0x60, 0x03, 0x7F, 0x7F, 0x7F, 0x01, 0x7F]))
       + section(2, uleb(4)
                 + bytes([0x03]) + b"env" + bytes([0x04]) + b"trap" + bytes([0x00, 0x01])
                 + bytes([0x03]) + b"env" + bytes([0x0A]) + b"soc_read32" + bytes([0x00, 0x02])
                 + bytes([0x03]) + b"env" + bytes([0x0B]) + b"soc_write32" + bytes([0x00, 0x03])
                 + bytes([0x03]) + b"env" + bytes([0x09]) + b"soc_cas32" + bytes([0x00, 0x04]))
       + section(3, uleb(1) + bytes([0x00]))
       + section(5, uleb(1) + bytes([0x00, 0x01]))
       + section(6, uleb(1) + bytes([0x7F, 0x01, I32C]) + sleb(0) + bytes([END]))
       + section(7, exports([(b"run", 0x00, 0x04), (b"mem", 0x02, 0x00)]))
       + section(10, uleb(1) + funcbody(bytes(body), 3))
       + section(11, segs))
open("/tmp/opencode/scas_kat.wasm", "wb").write(mod)
print("scas module written")
