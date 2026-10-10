#!/usr/bin/env python3
"""S4c windowed differ: trace-directed transpile of REAL windowed firmware
(CALL/ENTRY/RETW + literals + specials), checked vs the interpreter.
Usage: windiff.py <span.json>  ->  /tmp/opencode/spike_windiff.{wasm,json}
Ground truth: exec.rs arms + P1 windowed_call_entry_retw + S3b spike.
Globals wb(0)/ws(1)/ps(2) from steps[0]; AR = 64 PHYS words mem[0..256).
Calls/branches/jumps verified against next-trace-pc (no control flow in
trace order); ENTRY/RETW do full S3b state logic. Final: windowed regs +
DRAM words + wb/ws/ps vs wb1/ws1/ps1. Loud fail on anything uncovered."""

import json
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import (uleb, sleb, section, funcbody, I32C, I32ADD, I32SUB, I32LD, I32ST,
                   I32AND, I32OR, I32XOR, I32SHL, I32SHRU, I32SHRS,
                   I32LOAD8U, I32EQ, I32NE, I32EQZ, GLG, GLS, UNREACH, END)

LG, LS, MUL, SELECT, IF = 0x20, 0x21, 0x6C, 0x1B, 0x04
EMPTY = 0x40
WOE_BIT, CI_SHIFT = 0x40000, 16
FMIRR = 0x90000
DRAM_BASE, DRAM_SIZE = 0x3FC80000, 0x80000

COVERED = {"movi", "movi_n", "mov_n", "addi", "addi_n", "addmi", "add", "add_n",
           "sub", "and", "or", "xor", "slli", "srli", "srai", "extui",
           "l8ui", "l32i", "l32i_n", "l32r", "s32i", "s32i_n", "s8i",
           "bne", "beq", "beqz", "beqz_n", "bnez", "bltu", "bgeu",
           "call4", "call8", "call12", "callx8", "entry", "retw", "retw_n",
           "jx", "j", "addx4", "blti", "wsr_ps", "rsr_ps", "rsr_prid", "rsil",
           "rsync", "memw", "isync", "esync", "dsync"}


def C(v):
    return bytearray(bytes([I32C]) + sleb(v))


def wphys(reg):
    """Byte addr of windowed reg: ((wb*4 + reg) & 63) * 4 (S3b-proven shape)."""
    return (bytearray(bytes([GLG, 0])) + C(4) + bytearray([MUL]) + reg
            + bytearray([I32ADD]) + C(63) + bytearray([I32AND])
            + C(2) + bytearray([I32SHL]))


def wld(reg):
    return wphys(C(reg)) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))


def wst(reg, val):
    return wphys(C(reg)) + val + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))


def woe_check():
    return (bytearray(bytes([GLG, 2])) + C(WOE_BIT) + bytearray([I32AND])
            + bytearray([I32EQZ, IF, EMPTY, UNREACH, END]))


def walu(opc, o, out):
    d = o[0]["v"]
    if opc == "movi":
        out += wst(d, C(o[1]["v"]))
    elif opc == "movi_n":
        assert not o[1]["r"]
        out += wst(d, C(o[1]["v"]))
    elif opc == "mov_n":
        assert o[0]["r"] and o[1]["r"]
        out += wst(d, wld(o[1]["v"]))
    elif opc in ("addi", "addi_n"):
        out += wst(d, wld(o[1]["v"]) + C(o[2]["v"]) + bytes([I32ADD]))
    elif opc == "addmi":
        out += wst(d, wld(o[1]["v"]) + C(o[2]["v"]) + bytes([I32ADD]))
    elif opc == "add":
        out += wst(d, wld(o[1]["v"]) + wld(o[2]["v"]) + bytes([I32ADD]))
    elif opc == "addx4":
        out += wst(d, wld(o[1]["v"]) + C(2) + bytes([I32SHL]) + wld(o[2]["v"])
                   + bytes([I32ADD]))
    elif opc == "add_n":
        assert o[0]["r"] and o[1]["r"] and o[2]["r"]
        out += wst(d, wld(o[1]["v"]) + wld(o[2]["v"]) + bytes([I32ADD]))
    elif opc == "sub":
        out += wst(d, wld(o[1]["v"]) + wld(o[2]["v"]) + bytes([I32SUB]))
    elif opc == "and":
        out += wst(d, wld(o[1]["v"]) + wld(o[2]["v"]) + bytes([I32AND]))
    elif opc == "or":
        out += wst(d, wld(o[1]["v"]) + wld(o[2]["v"]) + bytes([I32OR]))
    elif opc == "xor":
        out += wst(d, wld(o[1]["v"]) + wld(o[2]["v"]) + bytes([I32XOR]))
    elif opc == "slli":
        out += wst(d, wld(o[1]["v"]) + C(o[2]["v"] & 31) + bytes([I32SHL]))
    elif opc == "srli":
        out += wst(d, wld(o[1]["v"]) + C(o[2]["v"] & 31) + bytes([I32SHRU]))
    elif opc == "srai":
        out += wst(d, wld(o[1]["v"]) + C(o[2]["v"] & 31) + bytes([I32SHRS]))
    elif opc == "extui":
        st, ln = o[2]["v"] & 31, o[3]["v"] & 31
        mask = 0xFFFFFFFF if ln >= 32 else ((1 << ln) - 1) & 0xFFFFFFFF
        out += wst(d, wld(o[1]["v"]) + C(st) + bytes([I32SHRU]) + C(mask)
                   + bytes([I32AND]))
    else:
        raise AssertionError(f"unwired alu {opc}")


def emit_branch_verify(opc, o, R, nxt, pc):
    """Trace-directed branch/jump check (no state change, no emission)."""
    assert nxt is not None, "span ends on control op"
    if opc in ("bne", "beq", "beqz", "beqz_n", "bnez", "bltu", "bgeu"):
        if opc in ("bne", "beq", "bltu", "bgeu"):
            a, b, tgt = R[o[0]["v"]], R[o[1]["v"]], o[2]["v"]
        else:
            a, b, tgt = R[o[0]["v"]], 0, o[1]["v"]
        taken = {"bne": a != b, "beq": a == b, "beqz": a == 0,
                 "beqz_n": a == 0, "bnez": a != 0,
                 "bltu": (a & 0xFFFFFFFF) < (b & 0xFFFFFFFF),
                 "bgeu": (a & 0xFFFFFFFF) >= (b & 0xFFFFFFFF)}[opc]
        assert (nxt == tgt) == taken, f"branch {opc} diverged at {pc:#x}"
    elif opc == "blti":
        a, b, tgt = R[o[0]["v"]], o[1]["v"], o[2]["v"]
        sa = a - 0x100000000 if a & 0x80000000 else a
        sb = b - 0x100000000 if b & 0x80000000 else b
        assert (nxt == tgt) == (sa < sb), f"blti diverged at {pc:#x}"
    elif opc == "jx":
        assert nxt == R[o[0]["v"]], f"jx diverged at {pc:#x}"
    elif opc == "j":
        assert nxt == o[0]["v"], f"j diverged at {pc:#x}"
    else:
        raise AssertionError(f"not a branch: {opc}")


def emit_call(out, opc, o, R, nxt, pc, length):
    assert nxt is not None, "span ends on control op"
    ci = o[1]["v"] // 4
    tgt = R[o[0]["v"]] if opc == "callx8" else o[0]["v"]
    assert nxt == tgt, f"{opc} target diverged at {pc:#x}"
    # Return addr = call pc+len (NOT the target): engine writes
    # (callinc<<30)|((pc+len)&mask); the trace target is separate.
    ra = ((ci << 30) | ((pc + length) & 0x3FFFFFFF)) & 0xFFFFFFFF
    out += wst(ci * 4, C(ra))
    out += (bytearray(bytes([GLG, 2])) + C(0x30000) + C(-1)
            + bytearray([I32XOR, I32AND]) + C(ci) + C(CI_SHIFT)
            + bytearray([I32SHL, I32OR, GLS, 2]))


def emit_entry(out, o):
    s, imm = o[0]["v"], o[2]["v"]
    out += woe_check()
    out += bytearray(bytes([GLG, 2])) + C(CI_SHIFT) + bytearray([I32SHRU])
    out += C(3) + bytearray([I32AND, LS, 0])
    out += bytes([GLG, 0]) + C(4) + bytearray([MUL])
    out += bytes([LG, 0]) + C(2) + bytearray([I32SHL])
    out += C(s & 3) + bytearray([I32OR, I32ADD])
    out += C(63) + bytearray([I32AND]) + C(2) + bytearray([I32SHL])
    out += wld(s) + C(imm) + bytearray([I32SUB])
    out += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
    out += bytes([GLG, 0, LG, 0, I32ADD]) + C(15) + bytearray([I32AND, LS, 1])
    out += bytearray(bytes([GLG, 1])) + C(1) + bytes([LG, 1])
    out += bytearray([I32SHL, I32OR, GLS, 1, LG, 1, GLS, 0])


def emit_retw(out, nxt, R):
    out += woe_check()
    out += wld(0) + bytearray([LS, 0])
    out += bytes([LG, 0]) + C(30) + bytearray([I32SHRU, LS, 1])
    out += C(1) + C(2) + C(3) + C(0)
    for k in (2, 1, 0):
        pos = (bytearray(bytes([GLG, 0])) + C(15 - k) + bytearray([I32ADD])
               + C(15) + bytearray([I32AND]))
        out += bytearray(bytes([GLG, 1])) + pos + bytearray([I32SHRU])
        out += C(1) + bytearray([I32AND, SELECT])
    out += bytearray([LS, 2])
    out += bytes([LG, 1, I32EQZ, IF, EMPTY, UNREACH, END])
    out += bytes([LG, 2, I32EQZ, I32EQZ, LG, 2, LG, 1, I32NE])
    out += bytes([I32AND, IF, EMPTY, UNREACH, END])
    upos = (bytearray(bytes([GLG, 0])) + C(16) + bytearray([I32ADD])
            + bytes([LG, 1]) + bytearray([I32SUB]) + C(15) + bytearray([I32AND]))
    out += bytearray(bytes([GLG, 1])) + upos + bytearray([I32SHRU])
    out += C(1) + bytearray([I32AND, I32EQZ, IF, EMPTY, UNREACH, END])
    out += (bytearray(bytes([GLG, 1])) + C(1) + bytearray(bytes([GLG, 0]))
            + bytearray([I32SHL]) + C(-1) + bytearray([I32XOR, I32AND, GLS, 1]))
    out += (bytearray(bytes([GLG, 0])) + bytes([LG, 1]) + bytearray([I32SUB])
            + C(15) + bytearray([I32AND, GLS, 0]))
    assert nxt is not None, "span ends on control op"
    top = (nxt & 0xC0000000) & 0xFFFFFFFF
    out += C(1024) + C(top) + bytes([LG, 0]) + C(0x3FFFFFFF)
    out += bytearray([I32AND, I32OR])
    out += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
    assert ((top | (R[0] & 0x3FFFFFFF)) & 0xFFFFFFFF) == nxt,         f"retw target diverged"


def emit_special(out, opc, o):
    if opc == "wsr_ps":
        out += wld(o[0]["v"]) + bytearray([GLS, 2])
    elif opc == "rsr_ps":
        out += wst(o[0]["v"], bytearray(bytes([GLG, 2])))
    elif opc == "rsr_prid":
        out += wst(o[0]["v"], C(0))  # core0 traces only
    elif opc == "rsil":
        out += wst(o[0]["v"], bytearray(bytes([GLG, 2])))
        out += (bytearray(bytes([GLG, 2])) + C(0xFFFFFFF0) + bytearray([I32AND])
                + C(o[1]["v"] & 0xF) + bytearray([I32OR, GLS, 2]))
    else:
        raise AssertionError(f"not special: {opc}")


def main():
    span = json.load(open(sys.argv[1]))
    steps = span["steps"]
    bad = [s["opc"] for s in steps if s["opc"] not in COVERED]
    assert not bad, f"uncovered ops: {sorted(set(bad))}"
    assert span.get("psram", []) == [], "psram writes need mirror (extend)"
    out = bytearray()
    out += C(steps[0]["wb"]) + bytearray([GLS, 0])
    out += C(steps[0]["ws"]) + bytearray([GLS, 1])
    out += C(steps[0]["ps"]) + bytearray([GLS, 2])
    # Preset ALL 64 phys slots (execution reads outside the start
    # window: saved ras and spill slots from before the span).
    for p, v in enumerate(span["phys0"]):
        out += C(p * 4) + C(v) + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))

    reads = {(a, w): v for a, v, w in span["reads"]}
    fwd = {}
    baked = {}
    slot_next = [FMIRR]


    for si, s_ in enumerate(steps):
        opc, o = s_["opc"], s_["opnds"]
        nxt = steps[si + 1]["pc"] if si + 1 < len(steps) else span.get("nxt")
        R = s_["regs"]
        if opc in ("movi", "movi_n", "mov_n", "addi", "addi_n", "addmi",
                      "add", "add_n", "addx4", "sub", "and", "or", "xor",
                      "slli", "srli", "srai", "extui"):
            walu(opc, o, out)
            continue
        if opc in ("bne", "beq", "beqz", "beqz_n", "bnez", "bltu", "bgeu",
                      "blti", "jx", "j"):
            emit_branch_verify(opc, o, R, nxt, s_["pc"])
            continue
        if opc in ("call4", "call8", "call12", "callx8"):
            emit_call(out, opc, o, R, nxt, s_["pc"], s_["len"])
            continue
        if opc == "entry":
            emit_entry(out, o)
            continue
        if opc in ("retw", "retw_n"):
            emit_retw(out, nxt, R)
            continue
        if opc in ("wsr_ps", "rsr_ps", "rsr_prid", "rsil"):
            emit_special(out, opc, o)
            continue
        if opc in ("rsync", "memw", "isync", "esync", "dsync"):
            continue  # barrier no-ops (exec Seq group)
        if opc in ("l8ui", "l32i", "l32i_n"):
            t, bs, off = o[0]["v"], o[1]["v"], o[2]["v"]
            eff = (R[bs] + off) & 0xFFFFFFFF
            width = 1 if opc == "l8ui" else 4
            assert (eff, width) in reads or (eff & ~3) in fwd, f"no oracle for {eff:#x}"
            out += slot_load(t, eff, width, reads, fwd, baked, slot_next)
            continue
        if opc == "l32r":
            eff = o[1]["v"]
            assert (eff, 4) in reads or (eff & ~3) in fwd, f"no oracle for l32r {eff:#x}"
            out += slot_load(o[0]["v"], eff, 4, reads, fwd, baked, slot_next)
            continue
        if opc in ("s32i", "s32i_n", "s8i"):
            t, bs, off = o[0]["v"], o[1]["v"], o[2]["v"]
            eff = (R[bs] + off) & 0xFFFFFFFF
            width = 1 if opc == "s8i" else 4
            seq, _ = slot_store(eff, wld(t), width, fwd, span)
            out += seq
            continue
        raise AssertionError(f"unwired op {opc}")
    out += bytes([END])

    body = bytes(out)
    run_code = funcbody(body, 3)
    need = max([0x80040, 1028] + ([slot_next[0] + 4] if baked else []))
    memsec = uleb(1) + bytes([0x00]) + uleb((need + 0xFFFF) // 0x10000)
    glob = (uleb(3)
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(1) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0x40000) + bytes([END]))
    segs = bytearray()
    if baked:
        words = sorted(baked.items())
        runs, start, cur = [], words[0][0], [words[0][1]]
        for k, v in words[1:]:
            if k == start + 4 * len(cur):
                cur.append(v)
            else:
                runs.append((start, cur))
                start, cur = k, [v]
        runs.append((start, cur))
        segs += uleb(len(runs))
        for off, vals in runs:
            data = b"".join(int(x & 0xFFFFFFFF).to_bytes(4, "little") for x in vals)
            segs += bytes([0x00]) + bytes([I32C]) + sleb(off) + bytes([END])
            segs += uleb(len(data)) + data
    mod = (
        b"\x00asm\x01\x00\x00\x00"
        + section(1, uleb(1) + bytes([0x60, 0x00, 0x00]))
        + section(3, uleb(1) + bytes([0x00]))
        + section(5, memsec)
        + section(6, glob)
        + section(7, uleb(5) + bytes([0x03]) + b"run" + bytes([0x00, 0x00])
                  + bytes([0x03]) + b"mem" + bytes([0x02, 0x00])
                  + bytes([0x02]) + b"wb" + bytes([0x03, 0x00])
                  + bytes([0x02]) + b"ws" + bytes([0x03, 0x01])
                  + bytes([0x02]) + b"ps" + bytes([0x03, 0x02]))
        + section(10, uleb(1) + run_code)
        + section(11, segs if segs else uleb(0))
    )
    open("/tmp/opencode/spike_windiff.wasm", "wb").write(mod)
    exp = {"regs1": span["regs1"], "wb1": span["wb1"], "ws1": span["ws1"],
           "ps1": span["ps1"], "writes": [[a, b, c] for a, b, c in span["mem"]]}
    json.dump(exp, open("/tmp/opencode/spike_windiff.json", "w"))
    print(f"windiff: {len(mod)} bytes, {len(steps)} ops, {len(baked)} baked")


# Per-(slot, byte) oracle knowledge (merge model: bake what's known, assert
# what's needed; unobserved bytes are never read by construction).
_KNOWN = set()


def slot_load(t, eff, width, reads, fwd, baked, slot_next):
    effw = eff & ~3
    if effw in fwd and fwd[effw] < FMIRR:
        base = C(fwd[effw])
    else:
        if effw not in fwd:
            sl = slot_next[0]
            slot_next[0] += 4
            fwd[effw] = sl
            baked[sl] = 0
        sl = fwd[effw]
        w = baked[sl]
        for b in range(4):
            k1, k4 = (effw + b, 1), (effw, 4)
            if k1 in reads:
                w = (w & ~(0xFF << (8 * b))) | ((reads[k1] & 0xFF) << (8 * b))
                _KNOWN.add((sl, b))
            elif k4 in reads:
                w = (w & ~(0xFF << (8 * b))) | (((reads[k4] >> (8 * b)) & 0xFF) << (8 * b))
                _KNOWN.add((sl, b))
        baked[sl] = w & 0xFFFFFFFF
        need = range(4) if width == 4 else [(eff & 3)]
        missing = [b for b in need if (sl, b) not in _KNOWN]
        assert not missing, f"no oracle byte for {eff:#x} (missing {missing})"
        base = C(sl)
    if width == 4:
        assert (eff & 3) == 0
        return wst(t, base + bytearray(bytes([I32LD]) + uleb(2) + uleb(0)))
    sh = (eff & 3) * 8
    return wst(t, base + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))
               + C(sh) + bytearray([I32SHRU]) + C(0xFF) + bytearray([I32AND]))


def slot_store(eff, val, width, fwd, span):
    # DRAM mirror lives past the 64-word AR file ([0, 256)).
    if DRAM_BASE <= eff < DRAM_BASE + DRAM_SIZE:
        moff = eff - DRAM_BASE + 256
        if width == 4:
            fwd[eff & ~3] = moff & ~3
            return (C(moff) + val + bytearray(bytes([I32ST]) + uleb(2) + uleb(0)),
                    True)
        wm = moff & ~3
        sh = (eff & 3) * 8
        m = (0xFF << sh) & 0xFFFFFFFF
        seq = (C(wm) + C(wm) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))
               + C(m ^ 0xFFFFFFFF) + bytearray([I32AND]) + val
               + C(0xFF) + bytearray([I32AND]) + C(sh) + bytearray([I32SHL])
               + bytearray([I32OR]) + bytearray(bytes([I32ST]) + uleb(2) + uleb(0)))
        fwd[eff & ~3] = wm
        return (seq, True)
    assert span.get("psram", []) == [], f"non-DRAM store at {eff:#x} needs mirror"
    return (bytearray(), False)


if __name__ == "__main__":
    main()
