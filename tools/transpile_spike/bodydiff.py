"""Diff seg bodies between single-func and multi-func modules."""
import sys


def uleb(b, i):
    s = 0
    sh = 0
    while True:
        x = b[i]
        i += 1
        s |= (x & 0x7F) << sh
        sh += 7
        if not x & 0x80:
            return s, i


def bodies(path):
    b = open(path, "rb").read()
    assert b[:4] == b"\x00asm"
    i = 8
    while i < len(b):
        sid = b[i]
        i += 1
        ln, i = uleb(b, i)
        if sid == 10:
            n, i = uleb(b, i)
            out = []
            for _ in range(n):
                sz, i = uleb(b, i)
                out.append(bytes(b[i:i + sz]))
                i += sz
            return out
        i += ln
    raise AssertionError("no code section")


single = bodies("/tmp/opencode/single559cc.wasm")
hy = bodies("/tmp/opencode/hy1.wasm")
print("single nbodies:", len(single), "hy1 nbodies:", len(hy))
s0 = single[2]
h3 = hy[5]
print("len seg0:", len(s0), "len seg3:", len(h3))
print("IDENTICAL" if s0 == h3 else "DIFFER")
if s0 != h3:
    n = min(len(s0), len(h3))
    for i in range(n):
        if s0[i] != h3[i]:
            print("first diff at byte", i, hex(s0[i]), "vs", hex(h3[i]))
            print("  single ctx:", s0[max(0, i - 8):i + 8].hex())
            print("  hy1    ctx:", h3[max(0, i - 8):i + 8].hex())
            break
    print("tail single:", s0[-12:].hex())
    print("tail hy1   :", h3[-12:].hex())
