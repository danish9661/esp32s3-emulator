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
                   I32LOAD8U, I32STORE8, I32EQ, I32NE, I32EQZ,
                   I32LTS, I32LTU, I32GES, I32GEU,
                   GLG, GLS, UNREACH, END)
CALL = 0x10

LG, LS, MUL, SELECT, IF = 0x20, 0x21, 0x6C, 0x1B, 0x04
EMPTY = 0x40
WOE_BIT, CI_SHIFT = 0x40000, 16
FMIRR = 0x90000
DRAM_BASE, DRAM_SIZE = 0x3FC80000, 0x80000
IRAM_BASE, IRAM_SIZE = 0x40370000, 0x80000
ROM_BASE, ROM_SIZE = 0x40000000, 0x60000
# Static direct-memory module layout (static_func.py): AR phys [0,256),
# then identity snapshots. Everything else -> soc imports (correct, slower).
M_DRAM_OFF, M_IRAM_OFF, M_ROM_OFF = 0x10000, 0x90000, 0x110000
M_PAGES = 0x21  # 0x210000: snapshots end 0x170000, shadow at 0x200000 + slack

COVERED = {"movi", "movi_n", "mov_n", "addi", "addi_n", "addmi", "add", "add_n",
           "sub", "and", "or", "xor", "slli", "srli", "srai", "extui",
           "l8ui", "l32i", "l32i_n", "l32r", "s32i", "s32i_n", "s8i",
           "bne", "beq", "beqz", "beqz_n", "bnez", "bnez_n", "bltu", "bgeu",
           "beqi", "bnei", "blt", "bge", "bltz", "bgez", "bgei", "bltui", "bgeui",
           "ball", "bnall", "bany", "bnone", "bbc", "bbs", "bbci", "bbsi",
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


def _s32(v):
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v & 0x80000000 else v


# (regs, regs/imm) branch family: target operand index, mirroring exec.rs
# exactly (order verified against trace JSON: o[0]=s reg, o[1]=t reg-or-imm,
# o[2]=target; 2-op forms use o[1] as target). bt/bf need BR state (FPU
# compares unwired) -> loud trap.
BRANCH2 = ("beqz", "beqz_n", "bnez", "bnez_n", "bltz", "bgez", "bt", "bf")
BRANCH3 = ("bne", "beq", "blt", "bge", "bltu", "bgeu", "beqi", "bnei",
           "blti", "bgei", "bltui", "bgeui", "ball", "bnall", "bany",
           "bnone", "bbc", "bbs", "bbci", "bbsi")


def branch_taken_py(opc, o, R):
    """Python-side taken-condition (trace verify + static analysis).
    Returns (taken: bool, tgt: int). Mirrors exec.rs; regs are u32."""
    if opc in BRANCH3:
        b, tgt = o[1]["v"], o[2]["v"]
        if opc == "bne":
            return R[o[0]["v"]] != R[o[1]["v"]], tgt
        if opc == "beq":
            return R[o[0]["v"]] == R[o[1]["v"]], tgt
        if opc == "blt":
            return _s32(R[o[0]["v"]]) < _s32(R[o[1]["v"]]), tgt
        if opc == "bge":
            return _s32(R[o[0]["v"]]) >= _s32(R[o[1]["v"]]), tgt
        if opc == "bltu":
            return (R[o[0]["v"]] & 0xFFFFFFFF) < (R[o[1]["v"]] & 0xFFFFFFFF), tgt
        if opc == "bgeu":
            return (R[o[0]["v"]] & 0xFFFFFFFF) >= (R[o[1]["v"]] & 0xFFFFFFFF), tgt
        if opc == "beqi":
            return R[o[0]["v"]] == (b & 0xFFFFFFFF), tgt
        if opc == "bnei":
            return R[o[0]["v"]] != (b & 0xFFFFFFFF), tgt
        if opc == "blti":
            return _s32(R[o[0]["v"]]) < _s32(b), tgt
        if opc == "bgei":
            return _s32(R[o[0]["v"]]) >= _s32(b), tgt
        if opc == "bltui":
            return (R[o[0]["v"]] & 0xFFFFFFFF) < (b & 0xFFFFFFFF), tgt
        if opc == "bgeui":
            return (R[o[0]["v"]] & 0xFFFFFFFF) >= (b & 0xFFFFFFFF), tgt
        if opc == "ball":
            return (R[o[0]["v"]] & R[o[1]["v"]]) == (R[o[1]["v"]] & 0xFFFFFFFF), tgt
        if opc == "bnall":
            return (R[o[0]["v"]] & R[o[1]["v"]]) != (R[o[1]["v"]] & 0xFFFFFFFF), tgt
        if opc == "bany":
            return ((R[o[0]["v"]] & R[o[1]["v"]]) & 0xFFFFFFFF) != 0, tgt
        if opc == "bnone":
            return ((R[o[0]["v"]] & R[o[1]["v"]]) & 0xFFFFFFFF) == 0, tgt
        if opc == "bbc":
            return ((R[o[0]["v"]] >> (R[o[1]["v"]] & 31)) & 1) == 0, tgt
        if opc == "bbs":
            return ((R[o[0]["v"]] >> (R[o[1]["v"]] & 31)) & 1) == 1, tgt
        if opc == "bbci":
            return ((R[o[0]["v"]] >> (b & 31)) & 1) == 0, tgt
        if opc == "bbsi":
            return ((R[o[0]["v"]] >> (b & 31)) & 1) == 1, tgt
    if opc in BRANCH2:
        a, tgt = R[o[0]["v"]], o[1]["v"]
        if opc in ("beqz", "beqz_n"):
            return a == 0, tgt
        if opc in ("bnez", "bnez_n"):
            return a != 0, tgt
        if opc == "bltz":
            return _s32(a) < 0, tgt
        if opc == "bgez":
            return _s32(a) >= 0, tgt
        raise AssertionError(f"{opc} needs BR state (unwired)")
    raise AssertionError(f"not a branch: {opc}")


def emit_branch_verify(opc, o, R, nxt, pc):
    """Trace-directed branch/jump check (no state change, no emission)."""
    assert nxt is not None, "span ends on control op"
    if opc in BRANCH2 or opc in BRANCH3:
        taken, tgt = branch_taken_py(opc, o, R)
        assert (nxt == tgt) == taken, f"branch {opc} diverged at {pc:#x}"
    elif opc == "jx":
        assert nxt == R[o[0]["v"]], f"jx diverged at {pc:#x}"
    elif opc == "j":
        assert nxt == o[0]["v"], f"j diverged at {pc:#x}"
    else:
        raise AssertionError(f"not a branch: {opc}")


def wcond(opc, o):
    """Runtime branch taken-condition from live regs; leaves i32 nonzero
    iff the branch WOULD take (loop-closing + static both-ways). Mirrors
    branch_taken_py arm-for-arm (exec.rs ground truth)."""
    if opc in ("bne", "beq", "blt", "bge", "bltu", "bgeu"):
        a, b = wld(o[0]["v"]), wld(o[1]["v"])
        op = {"bne": I32NE, "beq": I32EQ, "blt": I32LTS, "bge": I32GES,
              "bltu": I32LTU, "bgeu": I32GEU}[opc]
        return a + b + bytearray([op])
    if opc in ("beqi", "bnei", "blti", "bgei", "bltui", "bgeui"):
        op = {"beqi": I32EQ, "bnei": I32NE, "blti": I32LTS, "bgei": I32GES,
              "bltui": I32LTU, "bgeui": I32GEU}[opc]
        return wld(o[0]["v"]) + C(o[1]["v"]) + bytearray([op])
    if opc in ("ball", "bnall"):
        # (s&t)==t / !=t: recompute t (loads are pure, no temps needed).
        base = wld(o[0]["v"]) + wld(o[1]["v"]) + bytearray([I32AND])
        op = I32EQ if opc == "ball" else I32NE
        return base + wld(o[1]["v"]) + bytearray([op])
    if opc == "bany":
        return wld(o[0]["v"]) + wld(o[1]["v"]) + bytearray([I32AND])
    if opc == "bnone":
        return wld(o[0]["v"]) + wld(o[1]["v"]) + bytearray([I32AND, I32EQZ])
    if opc in ("bbc", "bbs"):
        base = (wld(o[0]["v"]) + wld(o[1]["v"]) + C(31) + bytearray([I32AND])
                + bytearray([I32SHRU]) + C(1) + bytearray([I32AND]))
        return base if opc == "bbs" else base + bytearray([I32EQZ])
    if opc in ("bbci", "bbsi"):
        base = (wld(o[0]["v"]) + C(o[1]["v"]) + C(31) + bytearray([I32AND])
                + bytearray([I32SHRU]) + C(1) + bytearray([I32AND]))
        return base if opc == "bbsi" else base + bytearray([I32EQZ])
    if opc == "bltz":
        return wld(o[0]["v"]) + C(0) + bytearray([I32LTS])
    if opc == "bgez":
        return wld(o[0]["v"]) + C(0) + bytearray([I32GES])
    if opc in ("bnez", "bnez_n"):
        # Taken iff a != 0 (!!x; single EQZ inverts the loop).
        return wld(o[0]["v"]) + bytearray([I32EQZ, I32EQZ])
    if opc in ("beqz", "beqz_n"):
        return wld(o[0]["v"]) + bytearray([I32EQZ])
    raise AssertionError(f"wcond unwired (needs BR state): {opc}")



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


def emit_retw(out, nxt, R, skip_tail=False):
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
    # Static tail-call/leaf use (static_func.py): the return ADDRESS is
    # dynamic (a0) and the run ends at HALT, so only the unrotation above
    # matters; skip the reconstructed-target store + assert.
    if skip_tail:
        return
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


# Static direct-memory access (static_func.py): runtime eff -> region
# dispatch. In [eff] (dload) or [val, eff]?? Convention: caller leaves [eff]
# (dload) or computes [val] then [eff] (dstore pushes val first, stashed to
# t0). Scratch locals t2 (eff) + t0 (val); seg funcs have 3 locals, none
# used by straight-line shapes. DRAM/IRAM/ROM snapshots preloaded at the
# M_*_OFF layout; everything else (MMIO, PSRAM window, RTC, flash windows)
# goes through the soc imports (correct, slower). ROM stores route to the
# import too (soc drops them; the snapshot stays pristine = silicon-true).
TRAP_I, RD_I, WR_I = 0x00, 0x01, 0x02
ELSE = 0x05


def _region_arms(make_body):
    """Nested IF/ELSE dispatch over (DRAM, IRAM, ROM, import-else).
    make_body(base, off, is_rom) -> bytearray for one direct arm.
    Stack discipline: each arm leaves [] (stores results to t0 itself)."""
    out = bytearray()
    for i, (base, size, off, is_rom) in enumerate((
            (DRAM_BASE, DRAM_SIZE, M_DRAM_OFF, False),
            (IRAM_BASE, IRAM_SIZE, M_IRAM_OFF, False),
            (ROM_BASE, ROM_SIZE, M_ROM_OFF, True))):
        out += bytes([LG, 2]) + C(base) + bytearray([I32SUB])
        out += C(size) + bytearray([I32LTU, IF, EMPTY])
        out += make_body(base, off, is_rom)
        out += bytes([ELSE])
    out += bytes([LG, 2])  # import-else leg (also triggered for ROM stores)
    return out


def dload(width):
    """Emit direct-memory load. In: [eff]. Out: [val]."""
    ld = bytes([I32LD]) + uleb(2) + uleb(0) if width == 4 else bytes([I32LOAD8U]) + uleb(0) + uleb(0)

    def arm(base, off, is_rom):
        return (bytes([LG, 2]) + C(base) + bytearray([I32SUB])
                + C(off) + bytearray([I32ADD]) + bytearray(ld)
                + bytes([LS, 0]))
    out = bytearray(bytes([LS, 2]))  # t2 = eff
    out += _region_arms(arm)
    # import-else: val = soc_read32(eff).
    out += bytes([CALL, RD_I, LS, 0])
    out += bytes([END, END, END])
    out += bytes([LG, 0])
    return out


def dstore(width):
    """Emit direct-memory store. In: [val, eff]. Out: []."""
    st = bytes([I32ST]) + uleb(2) + uleb(0) if width == 4 else bytes([I32STORE8]) + uleb(0) + uleb(0)

    def arm(base, off, is_rom):
        if is_rom:
            # Silicon drops ROM writes: route through the import (faithful),
            # keeping the snapshot pristine for later direct reads.
            return bytes([LG, 2, LG, 0, CALL, WR_I])
        return (bytes([LG, 2]) + C(base) + bytearray([I32SUB])
                + C(off) + bytearray([I32ADD]) + bytes([LG, 0]) + bytearray(st))
    out = bytearray(bytes([LS, 2, LS, 0]))  # t2 = eff, t0 = val; []
    out += _region_arms(arm)
    # import-else: soc_write32(eff, val) ([eff] already stacked by the tail).
    out += bytes([LG, 0, CALL, WR_I])
    out += bytes([END, END, END])
    return out
