#!/usr/bin/env python3
"""SPIKE (transpiler proof): emit a minimal wasm module executing a
hand-verified straight-line Xtensa integer block. AR file = 16 i32
locals; no windows/calls/memory (straight-line only). New files only;
zero engine impact. DELETE or promote after the verdict."""

def uleb(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            out.append(b | 0x80)
        else:
            out.append(b)
            break
    return bytes(out)


def sleb(n: int) -> bytes:
    # Signed LEB128 (i32.const/i64.const immediates): values >= 64 need
    # two bytes (caught live: 96 as `60` decodes to -32). u32 inputs >= 2**31
    # must normalize to negative first, else the encoder emits a 35-bit
    # form V8 rejects ("extra bits in varint", caught live on addi -17).
    if n >= 2 ** 31:
        n -= 2 ** 32
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7  # arithmetic shift preserves the sign
        if (n == 0 and not b & 0x40) or (n == -1 and b & 0x40):
            out.append(b)
            break
        out.append(b | 0x80)
    return bytes(out)


def section(sid: int, payload: bytes) -> bytes:
    return bytes([sid]) + uleb(len(payload)) + payload


def funcbody(body: bytes, ntemps: int) -> bytes:
    """Function body with DERIVED size (hand-added sizes desync silently;
    caught live). ntemps i32 temps."""
    loc = bytes([0x01, ntemps, 0x7F]) if ntemps else bytes([0x00])
    return uleb(len(body) + len(loc)) + loc + bytes(body)


def exports(items) -> bytes:
    """Export section payload with a DERIVED count (never hand-count:
    count-vs-payload mismatch is the classic spike killer, caught 3x live).
    items: list of (name: bytes, kind: int, index: int)."""
    out = bytearray()
    for name, kind, idx in items:
        out += uleb(len(name)) + name + bytes([kind]) + uleb(idx)
    return uleb(len(items)) + bytes(out)


# Opcodes.
LG, LS = 0x20, 0x21
I32C, I64C = 0x41, 0x42
I32ADD, I32AND = 0x6A, 0x71
I32LD, I32ST = 0x28, 0x36  # load/store with memarg (align, offset)
I32ADD, I32SUB, I32AND, I32OR, I32XOR = 0x6A, 0x6B, 0x71, 0x72, 0x73
I32SHL, I32SHRS, I32SHRU, I32EQZ, I32NE, I32EQ = 0x74, 0x75, 0x76, 0x45, 0x47, 0x46
I32LTU, I32LOAD8U = 0x49, 0x2C  # lt unsigned, load8_u
I32STORE8 = 0x3A  # store8
GLG, GLS, UNREACH = 0x23, 0x24, 0x00  # global.get/set, unreachable
I64OR, I64SHL, I64EXU = 0x84, 0x86, 0xAD
END = 0x0B

body = bytearray()
# movi a2, 0x60 (word 0x0060A022).
body += bytes([I32C]) + sleb(0x60) + bytes([LS, 2])
# movi a3, 0x04 (word 0x0004A032).
body += bytes([I32C]) + sleb(0x04) + bytes([LS, 3])
# add a4, a2, a3 (word 0x00804230).
body += bytes([LG, 2, LG, 3, I32ADD, LS, 4])
# addi a4, a4, 1 (word 0x0001C442).
body += bytes([LG, 4, I32C]) + sleb(1) + bytes([I32ADD, LS, 4])
# and a5, a4, a2 (word 0x00105420).
body += bytes([LG, 4, LG, 2, I32AND, LS, 5])
# return i64 a4 | (a5 << 32).
body += bytes([LG, 4, I64EXU, LG, 5, I64EXU, I64C]) + sleb(32) + bytes([I64SHL, I64OR, END])

code = uleb(1) + uleb(len(body) + 3) + bytes([0x01, 0x10, 0x7F]) + bytes(body)
# locals vec: 1 group x 16 i32 (3 bytes above).
mod = (
    b"\x00asm\x01\x00\x00\x00"
    + section(1, uleb(1) + bytes([0x60, 0x00, 0x01, 0x7E]))  # type: [] -> [i64]
    + section(3, uleb(1) + bytes([0x00]))  # func 0 : type 0
    + section(7, uleb(1) + bytes([0x03]) + b"run" + bytes([0x00, 0x00]))  # export run
    + section(10, code)
)
with open("/tmp/opencode/spike_block.wasm", "wb") as f:
    f.write(mod)
print(f"wrote {len(mod)} bytes; expect a4=101 a5=96 -> 412316860517")
