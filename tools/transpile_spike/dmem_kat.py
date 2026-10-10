#!/usr/bin/env python3
"""dload/dstore KATs: direct DRAM/IRAM/ROM legs + MMIO import legs +
ROM-store-drop. Usage: dmem_kat.py."""
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import (uleb, sleb, section, exports, funcbody, I32C, I32ADD,
                   I32LD, I32ST, END)
from windiff import (C, dload, dstore, DRAM_BASE, IRAM_BASE, ROM_BASE,
                     M_DRAM_OFF, M_IRAM_OFF, M_ROM_OFF, M_PAGES)

CALL = 0x10
GLG, GLS, LG, LS = 0x23, 0x24, 0x20, 0x21


def st(off, val):
    return C(off) + val + bytes([I32ST]) + uleb(2) + uleb(0)


body = bytearray()
# Preset direct cells (host writes snapshot below; here just exercise).
# 1. dload DRAM word at DRAM_BASE+0x10 -> store result to 0x2000.
body += C(DRAM_BASE + 0x10) + dload(4) + bytes([LS, 0]) + C(0x2000) + bytes([LG, 0])
body += bytes([I32ST]) + uleb(2) + uleb(0)
# 2. dload IRAM byte at IRAM_BASE+0x20 -> 0x2004.
body += C(IRAM_BASE + 0x20) + dload(1) + bytes([LS, 0]) + C(0x2004) + bytes([LG, 0])
body += bytes([I32ST]) + uleb(2) + uleb(0)
# 3. dload ROM word at ROM_BASE+0x30 -> 0x2008.
body += C(ROM_BASE + 0x30) + dload(4) + bytes([LS, 0]) + C(0x2008) + bytes([LG, 0])
body += bytes([I32ST]) + uleb(2) + uleb(0)
# 4. MMIO read (0x6000001C) via import -> 0x200C (import returns 0xAB).
body += C(0x6000001C) + dload(4) + bytes([LS, 0]) + C(0x200C) + bytes([LG, 0])
body += bytes([I32ST]) + uleb(2) + uleb(0)
# 5. dstore DRAM word at DRAM_BASE+0x40 = 0xDEADBEEF; read back -> 0x2010.
body += C(0xDEADBEEF) + C(DRAM_BASE + 0x40) + dstore(4)
body += C(DRAM_BASE + 0x40) + dload(4) + bytes([LS, 0]) + C(0x2010) + bytes([LG, 0])
body += bytes([I32ST]) + uleb(2) + uleb(0)
# 6. dstore IRAM byte at IRAM_BASE+0x50 = 0x7E; read back -> 0x2014.
body += C(0x7E) + C(IRAM_BASE + 0x50) + dstore(1)
body += C(IRAM_BASE + 0x50) + dload(1) + bytes([LS, 0]) + C(0x2014) + bytes([LG, 0])
body += bytes([I32ST]) + uleb(2) + uleb(0)
# 7. ROM store (dropped) then ROM read still pristine -> 0x2018.
body += C(0x12345678) + C(ROM_BASE + 0x30) + dstore(4)
body += C(ROM_BASE + 0x30) + dload(4) + bytes([LS, 0]) + C(0x2018) + bytes([LG, 0])
body += bytes([I32ST]) + uleb(2) + uleb(0)
# 8. MMIO write via import (count it) then MMIO read.
body += C(0x55) + C(0x60000000) + dstore(4)
body += bytes([END])

dram = bytearray(0x80000)
dram[0x10:0x14] = (0xCAFEBABE).to_bytes(4, "little")
iram = bytearray(0x80000)
iram[0x20] = 0x42
rom = bytearray(0x60000)
rom[0x30:0x34] = (0x12344321).to_bytes(4, "little")
segs = (uleb(3)
        + bytes([0x00, I32C]) + sleb(M_DRAM_OFF) + bytes([END]) + uleb(len(dram)) + bytes(dram)
        + bytes([0x00, I32C]) + sleb(M_IRAM_OFF) + bytes([END]) + uleb(len(iram)) + bytes(iram)
        + bytes([0x00, I32C]) + sleb(M_ROM_OFF) + bytes([END]) + uleb(len(rom)) + bytes(rom))
mod = (b"\x00asm\x01\x00\x00\x00"
       + section(1, uleb(4) + bytes([0x60, 0x00, 0x00])
                 + bytes([0x60, 0x01, 0x7F, 0x00])
                 + bytes([0x60, 0x01, 0x7F, 0x01, 0x7F])
                 + bytes([0x60, 0x02, 0x7F, 0x7F, 0x00]))
       + section(2, uleb(3)
                 + bytes([0x03]) + b"env" + bytes([0x04]) + b"trap" + bytes([0x00, 0x01])
                 + bytes([0x03]) + b"env" + bytes([0x0A]) + b"soc_read32" + bytes([0x00, 0x02])
                 + bytes([0x03]) + b"env" + bytes([0x0B]) + b"soc_write32" + bytes([0x00, 0x03]))
       + section(3, uleb(1) + bytes([0x03]))
       + section(5, uleb(1) + bytes([0x00]) + uleb(M_PAGES))
       + section(6, uleb(1) + bytes([0x7F, 0x01, I32C]) + sleb(0) + bytes([END]))
       + section(7, exports([(b"run", 0x00, 0x03), (b"mem", 0x02, 0x00)]))
       + section(10, uleb(1) + funcbody(bytes(body), 3))
       + section(11, segs))
open("/tmp/opencode/dmem_kat.wasm", "wb").write(mod)
print("dmem_kat module written")
