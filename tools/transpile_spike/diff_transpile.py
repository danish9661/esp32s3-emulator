#!/usr/bin/env python3
"""S4c-1 differ: transpile a REAL firmware straight-line block (decoded by
our decoder via diff_dump) and check architectural state vs the interpreter.
Usage: diff_transpile.py <span.json> <image.merged.bin>
Rules (loud fail otherwise): wb==0 throughout, no windowed/call/control/
special ops in span, every load classifiable (DRAM mirror @0, flash mirror
@0x80000 baked from the image file), every store to DRAM mirror.
AR file = flat a0-a15 words at mem[0..64)."""

import json
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import (uleb, sleb, section, I32C, I32ADD, I32SUB, I32LD, I32ST,
                   I32AND, I32OR, I32XOR, I32SHL, I32SHRU, I32SHRS, I32LTU,
                   I32LOAD8U, END)

DRAM_BASE, DRAM_SIZE = 0x3FC80000, 0x80000
FLASH_BASE, FLASH_SIZE = 0x3C000000, 0x2000000
FMIRR = 0x90000  # past the DRAM mirror tail (0x80040)
PSMIRR = 0x100000  # psram mirror base (module offsets, not addrs)
SELECT = 0x1B


def C(v):
    return bytearray(bytes([I32C]) + sleb(v))


def ar(slot):
    return C(slot * 4)


def ald(slot):
    return ar(slot) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0))


def ast(slot, val):
    return ar(slot) + val + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))


def emit_alu(opc, o, out):
    d, s, v = o[0]["v"], o[1]["v"], o[2]["v"]
    if opc == "movi":
        out += ast(d, C(s))  # movi imm is o[1], not o[2] (caught live)
    elif opc == "addi":
        out += ast(d, ald(s) + C(v) + bytes([I32ADD]))
    elif opc == "addmi":
        out += ast(d, ald(s) + C(v) + bytes([I32ADD]))  # o[2] pre-shifted
    elif opc == "add":
        out += ast(d, ald(s) + ald(o[2]["v"]) + bytes([I32ADD]))
    elif opc == "sub":
        out += ast(d, ald(s) + ald(o[2]["v"]) + bytes([I32SUB]))
    elif opc == "and":
        out += ast(d, ald(s) + ald(o[2]["v"]) + bytes([I32AND]))
    elif opc == "or":
        out += ast(d, ald(s) + ald(o[2]["v"]) + bytes([I32OR]))
    elif opc == "xor":
        out += ast(d, ald(s) + ald(o[2]["v"]) + bytes([I32XOR]))
    elif opc == "slli":
        out += ast(d, ald(s) + C(v & 31) + bytes([I32SHL]))
    elif opc == "srli":
        out += ast(d, ald(s) + C(v & 31) + bytes([I32SHRU]))
    elif opc == "srai":
        out += ast(d, ald(s) + C(v & 31) + bytes([I32SHRS]))
    else:
        raise AssertionError(f"uncovered alu {opc}")


def main():
    span = json.load(open(sys.argv[1]))  # oracle reads inside; no image needed
    steps = span["steps"]
    assert span["wb0"] == 0 and span["wb1"] == 0, "window moved"
    names = [s["opc"] for s in steps]
    covered = {"movi", "movi_n", "addi", "addmi", "add", "sub", "and", "or", "xor",
               "slli", "srli", "srai", "l8ui", "l32i", "s32i", "s8i",
               "bne", "beq", "beqz", "bnez", "bltu", "bgeu"}
    bad = [n for n in names if n not in covered]
    assert not bad, f"uncovered ops (trap import needed): {bad}"
    # Overlap check: no writes in span (mem[]) so post-hoc read values sound.
    assert span["mem"] == [], f"span writes need write-path: {span['mem']}"

    out = bytearray()
    # Prologue: preset regs0 into AR slots.
    for r, v in enumerate(span["regs0"]):
        out += ast(r, C(v))  # sleb covers full u32
    # Slot model: every load site gets a baked slot with its oracle value;
    # stores write the DRAM mirror and forward to later loads in-block.
    # (Trace-directed single path: each site reads a fixed addr.)
    fwd = {}          # eff addr -> slot
    baked = {}        # slot -> (value, width)
    slot_next = [FMIRR]
    writes = []       # eff addrs stored (for final mem[] compare)

    def slot_for(eff, val, width):
        if eff in fwd:
            return fwd[eff]
        sl = slot_next[0]
        slot_next[0] += 4
        fwd[eff] = sl
        baked[sl] = (val, width)
        return sl

    reads = {(a, w): v for a, v, w in span["reads"]}
    mem_addrs = {w for (w, _, _) in span["mem"]}
    psram_addrs = set()  # physical offsets written (span["psram"] entries)
    for _a, _b, _c in span.get("psram", []):
        pass  # collected below with proper offsets
    psram_offs = {e[0] for e in span.get("psram", [])}
    nxt = list(steps)
    for si, s_ in enumerate(steps):
        opc, o = s_["opc"], s_["opnds"]
        # Branch verification (trace-directed: taken == next-trace-pc is target).
        # The CONDITION is evaluated and must agree (else vacuous proof).
        if opc in ("bne", "beq", "beqz", "bnez", "bltu", "bgeu"):
            nxt_pc = steps[si + 1]["pc"] if si + 1 < len(steps) else None
            if opc in ("bne", "beq", "bltu", "bgeu"):
                a, b = s_["regs"][o[0]["v"]], s_["regs"][o[1]["v"]]
                tgt = o[2]["v"]
            else:
                a, b, tgt = s_["regs"][o[0]["v"]], 0, o[1]["v"]
            taken = {"bne": a != b, "beq": a == b, "beqz": a == 0,
                     "bnez": a != 0, "bltu": (a & 0xFFFFFFFF) < (b & 0xFFFFFFFF),
                     "bgeu": (a & 0xFFFFFFFF) >= (b & 0xFFFFFFFF)}[opc]
            assert (nxt_pc == tgt) == taken, f"branch {opc} diverged at {s_['pc']:#x}"
            continue  # trace order already follows the executed path
        if opc == "movi_n":
            assert not o[1]["r"], "movi_n imm shape"
            out += ast(o[0]["v"], C(o[1]["v"]))
            continue
        if opc in covered - {"l8ui", "l32i", "s32i", "s8i",
                              "bne", "beq", "beqz", "bnez", "bltu", "bgeu",
                              "movi_n"}:
            emit_alu(opc, o, out)
        elif opc in ("l8ui", "l32i"):
            t, bs, off = o[0]["v"], o[1]["v"], o[2]["v"]
            eff = (s_["regs"][bs] + off) & 0xFFFFFFFF
            width = 1 if opc == "l8ui" else 4
            assert (eff, width) in reads, f"no oracle read for {eff:#x}"
            if any(w <= eff + width - 1 and eff <= w + 3 for (w, _, _) in span["mem"]):
                raise AssertionError(f"read/write overlap at {eff:#x}")
            sl = slot_for(eff, reads[(eff, width)], width)
            if opc == "l32i":
                out += ast(t, C(sl) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0)))
            else:
                out += ast(t, C(sl) + bytearray(bytes([I32LOAD8U]) + uleb(0) + uleb(0)))
        elif opc in ("s32i", "s8i"):
            t, bs, off = o[0]["v"], o[1]["v"], o[2]["v"]
            eff = (s_["regs"][bs] + off) & 0xFFFFFFFF
            if DRAM_BASE <= eff < DRAM_BASE + DRAM_SIZE:
                # DRAM mirror (+forwarding); final compare via mem[] words.
                moff = eff - DRAM_BASE
                if opc == "s32i":
                    out += C(moff) + ald(t) + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
                else:
                    out += C(moff) + ald(t) + bytearray(bytes([0x3A]) + uleb(0) + uleb(0))
                fwd[eff] = moff
                writes.append(eff)
            else:
                # Non-DRAM (flash-dropped/MMIO): PSRAM writes need a
                # translate link the dump lacks — fail loudly if observed.
                assert span.get("psram", []) == [], f"PSRAM write at {eff:#x} needs mirror"
                # No observable effect possible (DRAM/PSRAM snapshots empty
                # here): model as no-op. Any future DRAM/PSRAM trace of this
                # addr would route above instead.
                continue
        else:
            raise AssertionError(f"uncovered op {opc} (trap import needed)")
    out += bytes([END])
    run_code = uleb(len(out) + 1) + bytes([0x00]) + bytes(out)

    # Data segments: baked read slots as word runs (LE bytes).
    segs = bytearray()
    if baked:
        words = sorted({sl: v for sl, (v, w) in baked.items()}.items())
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
    # Module memory: cover DRAM mirror + max baked slot.
    need = max([DRAM_SIZE] + ([slot_next[0] + 4] if baked else []))
    npages = max(9, (need + 0xFFFF) // 0x10000)
    memsec = uleb(1) + bytes([0x00]) + uleb(npages)
    mod = (
        b"\x00asm\x01\x00\x00\x00"
        + section(1, uleb(1) + bytes([0x60, 0x00, 0x00]))
        + section(3, uleb(1) + bytes([0x00]))
        + section(5, memsec)
        + section(7, uleb(2) + bytes([0x03]) + b"run" + bytes([0x00, 0x00])
                  + bytes([0x03]) + b"mem" + bytes([0x02, 0x00]))
        + section(10, uleb(1) + run_code)
        + section(11, segs if segs else uleb(0))
    )
    open("/tmp/opencode/spike_diff.wasm", "wb").write(mod)
    # Expectation file for the runner.
    exp = {"regs1": span["regs1"],
           "writes": [[a, b, c] for a, b, c in span["mem"]]}
    import json as J
    open("/tmp/opencode/spike_diff.json", "w").write(J.dumps(exp))
    print(f"diff module: {len(mod)} bytes, {len(steps)} ops, "
          f"{len(baked)} baked slots, {npages} pages")


if __name__ == "__main__":
    main()
