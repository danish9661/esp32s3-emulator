#!/usr/bin/env python3
"""S4c auto-coverage driver: transpile a WHOLE trace with trap fallback.
Usage: autodiff.py <span.json>  ->  /tmp/opencode/spike_auto.{wasm,json}
Covered ops emit exactly like windiff (shared helpers — single code path);
anything else (incl. MMIO loads/stores) emits an import call:
  trap(pc) for unmodeled ops, soc_read32/soc_write32 for MMIO.
Reports coverage % + trap/mmio lists. State compare only when clean
(no traps, no MMIO): then it must match regs1/mem/wb1/ws1/ps1 exactly."""

import json
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import uleb, sleb, section, exports, funcbody, I32C, END
from windiff import (C, wld, wst, wphys, woe_check, walu, slot_load, slot_store,
                     emit_branch_verify, emit_call, emit_entry, emit_retw,
                     emit_special, COVERED, DRAM_BASE, DRAM_SIZE, FMIRR,
                     LG, LS, MUL, SELECT, IF, EMPTY, WOE_BIT, CI_SHIFT,
                     I32ADD, I32SUB, I32LD, I32ST, I32AND, I32OR, I32XOR,
                     I32SHL, I32SHRU, I32SHRS, I32LOAD8U, I32EQ, I32NE,
                     I32EQZ, GLG, GLS, UNREACH)

CALL = 0x10
TRAP_I, RD_I, WR_I = 0, 1, 2  # import func indices: trap, soc_read32, soc_write32
MMIO_LO, MMIO_HI = 0x60000000, 0x60100000


def is_mmio(a):
    return MMIO_LO <= a < MMIO_HI


def main():
    span = json.load(open(sys.argv[1]))
    steps = span["steps"]
    out = bytearray()
    out += C(steps[0]["wb"]) + bytearray([GLS, 0])
    out += C(steps[0]["ws"]) + bytearray([GLS, 1])
    out += C(steps[0]["ps"]) + bytearray([GLS, 2])
    for p, v in enumerate(span["phys0"]):
        out += C(p * 4) + C(v) + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))

    reads = {(a, w): v for a, v, w in span["reads"]}
    fwd, baked, slot_next = {}, {}, [FMIRR]
    n_cov, n_mmio, traps, mmios = 0, 0, [], []

    def trap_here(pc):
        traps.append(pc)
        return C(pc) + bytes([CALL, TRAP_I])

    for si, s_ in enumerate(steps):
        opc, o = s_["opc"], s_["opnds"]
        nxt = steps[si + 1]["pc"] if si + 1 < len(steps) else span.get("nxt")
        R = s_["regs"]
        if opc in COVERED and opc not in ("l8ui", "l32i", "l32i_n", "l32r",
                                          "s32i", "s32i_n", "s8i"):
            if opc in ("bne", "beq", "beqz", "beqz_n", "bnez", "bltu", "bgeu",
                       "blti", "jx", "j"):
                emit_branch_verify(opc, o, R, nxt, s_["pc"])
            elif opc in ("call4", "call8", "call12", "callx8"):
                emit_call(out, opc, o, R, nxt, s_["pc"], s_["len"])
            elif opc == "entry":
                emit_entry(out, o)
            elif opc in ("retw", "retw_n"):
                emit_retw(out, nxt, R)
            elif opc in ("wsr_ps", "rsr_ps", "rsr_prid", "rsil"):
                emit_special(out, opc, o)
            elif opc in ("rsync", "memw", "isync", "esync", "dsync"):
                pass
            else:
                walu(opc, o, out)
            n_cov += 1
            continue
        if opc in ("l8ui", "l32i", "l32i_n", "l32r"):
            eff = o[1]["v"] if opc == "l32r" else (R[o[1]["v"]] + o[2]["v"]) & 0xFFFFFFFF
            width = 1 if opc == "l8ui" else 4
            if is_mmio(eff):
                n_mmio += 1
                mmios.append((s_["pc"], eff))
                # addr first, then host-canned value lands on top for store.
                out += wphys(C(o[0]["v"])) + C(eff) + bytes([CALL, RD_I])
                out += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
                continue
            if (eff, width) not in reads and (eff & ~3) not in fwd:
                out += trap_here(s_["pc"])
                continue
            out += slot_load(o[0]["v"], eff, width, reads, fwd, baked, slot_next)
            n_cov += 1
            continue
        if opc in ("s32i", "s32i_n", "s8i"):
            eff = (R[o[1]["v"]] + o[2]["v"]) & 0xFFFFFFFF
            width = 1 if opc == "s8i" else 4
            if is_mmio(eff):
                n_mmio += 1
                mmios.append((s_["pc"], eff))
                out += C(eff) + wld(o[0]["v"]) + bytes([CALL, WR_I])
                continue
            seq, _ = slot_store(eff, wld(o[0]["v"]), width, fwd, span)
            out += seq
            n_cov += 1
            continue
        out += trap_here(s_["pc"])
    out += bytes([END])

    body = bytes(out)
    run_code = funcbody(body, 3)
    need = max([0x80040, 1028] + ([slot_next[0] + 4] if baked else []))
    memsec = uleb(1) + bytes([0x00]) + uleb((need + 0xFFFF) // 0x10000)
    glob = (uleb(3)
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(1) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0x40000) + bytes([END]))
    imps = (uleb(3)
            + bytes([0x03]) + b"env" + bytes([0x04]) + b"trap" + bytes([0x00, 0x01])
            + bytes([0x03]) + b"env" + bytes([0x0A]) + b"soc_read32" + bytes([0x00, 0x02])
            + bytes([0x03]) + b"env" + bytes([0x0B]) + b"soc_write32" + bytes([0x00, 0x03]))
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
        + section(1, uleb(4) + bytes([0x60, 0x00, 0x00])
                  + bytes([0x60, 0x01, 0x7F, 0x00])
                  + bytes([0x60, 0x01, 0x7F, 0x01, 0x7F])
                  + bytes([0x60, 0x02, 0x7F, 0x7F, 0x00]))
        + section(2, imps)
        + section(3, uleb(1) + bytes([0x00]))
        + section(5, memsec)
        + section(6, glob)
        + section(7, exports([(b"run", 0x00, 0x03), (b"mem", 0x02, 0x00),
                                  (b"wb", 0x03, 0x00), (b"ws", 0x03, 0x01),
                                  (b"ps", 0x03, 0x02)]))
        + section(10, uleb(1) + run_code)
        + section(11, segs if segs else uleb(0))
    )
    open("/tmp/opencode/spike_auto.wasm", "wb").write(mod)
    total = len(steps)
    rep = {"covered": n_cov, "mmio": n_mmio, "traps": [hex(p) for p in traps],
           "mmio_sites": [[hex(p), hex(a)] for p, a in mmios],
           "regs1": span["regs1"], "wb1": span["wb1"], "ws1": span["ws1"],
           "ps1": span["ps1"], "writes": [[a, b, c] for a, b, c in span["mem"]]}
    json.dump(rep, open("/tmp/opencode/spike_auto.json", "w"))
    print(f"auto: {total} steps covered={n_cov} mmio={n_mmio} trapped={len(traps)}")
    if traps:
        print("trap pcs:", [hex(p) for p in traps[:10]])


if __name__ == "__main__":
    main()
