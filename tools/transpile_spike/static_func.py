#!/usr/bin/env python3
"""S3 static function transpiler v1 (leaf): instruction stream from STATIC
decode (sfunc JSON), data from DIRECT memory (snapshots preloaded, MMIO via
imports — no oracle needed for execution), validated against a paired
dynamic func-trace oracle (entry regs in, end regs + DRAM diffs out).
Usage: static_func.py <sfunc.json> <functoracle.json>
  -> /tmp/opencode/spike_static.{wasm,json} (run with static_run.mjs).
Design reuses the trampoline dispatcher + shadow stack (autodiff_func.py,
proven) with run() pushing a HALT continuation so leaf retw pops balanced.
v1 limits (loud trap otherwise): no calls (v2: static call trees), no jx/
ret/ret_n (dynamic targets), j only in-function, all branches runtime
both-ways (no trace direction needed). l32r literals bake from snapshots.
Ground truth: exec.rs arms (via windiff helpers), S2 loop-closing shapes."""

import json
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import (uleb, sleb, section, exports, funcbody, I32C, I32ADD,
                   I32SUB, I32LD, I32ST, I32AND, I32OR, I32XOR, I32SHL,
                   I32SHRU, I32SHRS, I32LOAD8U, I32EQ, I32NE, I32EQZ,
                   I32GEU, I32LTS, GLG, GLS, UNREACH, END)
from windiff import (C, wld, wst, wphys, woe_check, walu, dload, dstore,
                     emit_entry, emit_retw, emit_special, wcond,
                     branch_taken_py, BRANCH2, BRANCH3, COVERED,
                     DRAM_BASE, DRAM_SIZE, IRAM_BASE, IRAM_SIZE,
                     ROM_BASE, ROM_SIZE, M_DRAM_OFF, M_IRAM_OFF, M_ROM_OFF,
                     M_PAGES, TRAP_I, RD_I, WR_I,
                     LG, LS, MUL, SELECT, IF, EMPTY, WOE_BIT, CI_SHIFT)
from autodiff import is_mmio  # noqa: F401 (contract below)

CALL, RET, CALL_IND, BRIF = 0x10, 0x0F, 0x11, 0x0D
BLOCK, LOOP = 0x02, 0x03
HALT = 0xFFFFFFFF
SHADOW_BASE = 0x200000


def ssp_push(idval):
    return (C(SHADOW_BASE) + bytearray(bytes([GLG, 3]))
            + bytearray([I32ADD]) + idval
            + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
            + bytearray(bytes([GLG, 3])) + C(4) + bytearray([I32ADD, GLS, 3]))


def ssp_pop():
    return (bytearray(bytes([GLG, 3])) + bytearray([I32EQZ, IF, EMPTY, UNREACH, END])
            + bytearray(bytes([GLG, 3])) + C(4) + bytearray([I32SUB, GLS, 3])
            + C(SHADOW_BASE) + bytearray(bytes([GLG, 3]))
            + bytearray([I32ADD]) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0)))


def snap_val(snaps, addr, width):
    """Snapshot value for a static address (l32r baking). None if outside."""
    for base, arr in ((DRAM_BASE, snaps["dram"]), (IRAM_BASE, snaps["iram"]),
                      (ROM_BASE, snaps["rom"])):
        size = len(arr) * 4
        if base <= addr < base + size:
            if width == 4:
                assert addr % 4 == 0, f"unaligned static literal {addr:#x}"
                return arr[(addr - base) // 4] & 0xFFFFFFFF
            word = arr[(addr - base) // 4] & 0xFFFFFFFF
            return (word >> (8 * (addr & 3))) & 0xFF
    return None


def main():
    import autodiff as AD
    assert (AD.TRAP_I, AD.RD_I, AD.WR_I) == (TRAP_I, RD_I, WR_I), "import contract"
    sfunc = json.load(open(sys.argv[1]))["sfunc"]
    oracle = json.load(open(sys.argv[2]))
    insns = sfunc["insns"]
    assert insns and insns[0]["pc"] == sfunc["entry"], "static must start at entry"
    static_pcs = set(s["pc"] for s in insns)
    # Leaf-end expectation: first oracle step OUTSIDE the static pc set.
    # (The func-trace window runs past leaf return into the caller; span-end
    # regs are frames ahead. Pre-step regs/wb/ws/ps of the first outside
    # step == post-leaf state. Size the window to end near leaf return so
    # oracle mem diffs stay leaf-only.)
    leaf_end = next((st for st in oracle["steps"] if st["pc"] not in static_pcs), None)
    assert leaf_end is not None, "oracle never leaves static function (window too small?)"
    exp_regs1, exp_wb1, exp_ws1, exp_ps1 = (leaf_end["regs"], leaf_end["wb"],
                                            leaf_end["ws"], leaf_end["ps"])
    snaps = {"dram": oracle["fulldram"], "iram": oracle["fulliram"], "rom": oracle["fullrom"]}

    # Pass 1: static blocks. Starts: entry, direct-target landings,
    # post-transfer continuations. Dynamic transfers end blocks.
    starts = {0}
    pcs = [s["pc"] for s in insns]
    pcset = set(pcs)
    for i, s in enumerate(insns):
        opc = s["opc"]
        if opc in BRANCH2 or opc in BRANCH3 or opc in ("jx", "j", "call4",
                "call8", "call12", "callx8", "retw", "retw_n", "ret", "ret_n"):
            if i + 1 < len(insns):
                starts.add(i + 1)
    for i, s in enumerate(insns):
        opc = s["opc"]
        if opc in BRANCH2 or opc in BRANCH3 or opc == "j":
            o = s["opnds"]
            if opc == "j":
                tgt = o[0]["v"]
            else:
                _, tgt = branch_taken_py(opc, o, [0] * 16)
            if tgt in pcset:
                starts.add(pcs.index(tgt))
    seg_list = sorted(starts) + [len(insns)]
    nsegs = len(seg_list) - 1
    seg_id_of_pos = {}
    for k in range(nsegs):
        for i in range(seg_list[k], seg_list[k + 1]):
            seg_id_of_pos[i] = k
    pc_to_seg = {}
    for k in range(nsegs):
        pc_to_seg.setdefault(insns[seg_list[k]]["pc"], k)

    stats = {"covered": 0, "mmio_rt": 0, "traps": [], "segs": nsegs}
    bodies = []
    for sg in range(nsegs):
        lo, hi = seg_list[sg], seg_list[sg + 1]
        out = bytearray()
        for si in range(lo, hi):
            s_ = insns[si]
            opc, o = s_["opc"], s_["opnds"]
            if opc in COVERED and opc not in ("l8ui", "l32i", "l32i_n", "l32r",
                                              "s32i", "s32i_n", "s8i"):
                if opc in BRANCH2 or opc in BRANCH3:
                    assert (si + 1) in seg_id_of_pos, "branch at static end"
                    _, tgt = branch_taken_py(opc, o, [0] * 16)
                    # Static both-ways: runtime cond picks taken vs fall.
                    out += bytes([BLOCK, EMPTY])
                    out += wcond(opc, o)
                    out += bytes([BRIF, 0x00])
                    out += C(seg_id_of_pos[si + 1]) + bytes([RET])
                    out += bytes([END])
                    assert tgt in pc_to_seg, f"static target not a block: {tgt:#x}"
                    out += C(pc_to_seg[tgt]) + bytes([RET])
                    break
                if opc == "j":
                    assert o[0]["v"] in pc_to_seg, f"j out of function: {o[0]['v']:#x}"
                    out += C(pc_to_seg[o[0]["v"]]) + bytes([RET])
                    break
                if opc in ("call4", "call8", "call12", "callx8"):
                    stats["traps"].append(s_["pc"])
                    out += C(s_["pc"]) + bytes([CALL, TRAP_I])
                    out += C(HALT) + bytes([RET])
                    break
                if opc in ("jx", "ret", "ret_n"):
                    stats["traps"].append(s_["pc"])
                    out += C(s_["pc"]) + bytes([CALL, TRAP_I])
                    out += C(HALT) + bytes([RET])
                    break
                if opc == "entry":
                    emit_entry(out, o)
                elif opc in ("retw", "retw_n"):
                    emit_retw(out, None, None, skip_tail=True)
                    out += ssp_pop() + bytes([RET])
                    break
                elif opc in ("wsr_ps", "rsr_ps", "rsr_prid", "rsil"):
                    emit_special(out, opc, o)
                elif opc in ("rsync", "memw", "isync", "esync", "dsync"):
                    pass
                else:
                    walu(opc, o, out)
                stats["covered"] += 1
                continue
            if opc in ("l8ui", "l32i", "l32i_n"):
                # Direct memory: eff from LIVE regs, region-dispatched.
                width = 1 if opc == "l8ui" else 4
                out += wphys(C(o[0]["v"]))
                out += wld(o[1]["v"]) + C(o[2]["v"]) + bytearray([I32ADD])
                out += dload(width)
                out += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
                stats["covered"] += 1
                continue
            if opc == "l32r":
                # Literals are code-adjacent (static): bake from snapshots.
                lit = o[1]["v"]
                v = snap_val(snaps, lit, 4)
                if v is None:
                    out += wphys(C(o[0]["v"])) + C(lit) + bytes([CALL, RD_I])
                    out += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
                    stats["mmio_rt"] += 1
                else:
                    out += wst(o[0]["v"], C(v))
                stats["covered"] += 1
                continue
            if opc in ("s32i", "s32i_n", "s8i"):
                # dstore convention: [val, eff].
                width = 1 if opc == "s8i" else 4
                out += wld(o[0]["v"])
                out += wld(o[1]["v"]) + C(o[2]["v"]) + bytearray([I32ADD])
                out += dstore(width)
                stats["covered"] += 1
                continue
            stats["traps"].append(s_["pc"])
            out += C(s_["pc"]) + bytes([CALL, TRAP_I])
            out += C(HALT) + bytes([RET])
            break
        else:
            if hi == len(insns):
                out += C(HALT) + bytes([RET])
            else:
                out += C(sg + 1) + bytes([RET])
        out += bytes([END])
        bodies.append(bytes(out))

    # run(): presets (oracle entry state) + ssp=0 + HALT continuation +
    # dispatcher loop (same bytes as the trace vehicle, proven).
    run = bytearray()
    run += C(oracle["wb0"]) + bytearray([GLS, 0])
    run += C(oracle["ws0"]) + bytearray([GLS, 1])
    run += C(oracle["ps0"]) + bytearray([GLS, 2])
    run += C(0) + bytearray([GLS, 3])
    for pp, vv in enumerate(oracle["phys0"]):
        run += C(pp * 4) + C(vv) + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
    run += ssp_push(C(HALT))  # leaf retw pops balanced (no caller push)
    run += C(0) + bytearray([LS, 0])
    run += bytes([LOOP, EMPTY])
    run += bytes([LG, 0]) + bytes([CALL_IND, 0x04, 0x00])
    run += bytes([0x22, 0x00])  # tee t0=nid (nets zero)
    run += C(HALT) + bytes([I32NE, BRIF, 0x00])
    run += bytes([END, END])
    allbodies = [bytes(run)] + bodies
    code = uleb(len(allbodies))
    for b in allbodies:
        code += funcbody(b, 3)
    need = max([M_DRAM_OFF + DRAM_SIZE, M_IRAM_OFF + IRAM_SIZE,
                M_ROM_OFF + ROM_SIZE, SHADOW_BASE + 0x1000, 1028])
    assert need <= M_PAGES * 0x10000, f"static layout overflow {need:#x}"
    memsec = uleb(1) + bytes([0x00]) + uleb(M_PAGES)
    glob = (uleb(4)
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(1) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0x40000) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0) + bytes([END]))
    imps = (uleb(3)
            + bytes([0x03]) + b"env" + bytes([0x04]) + b"trap" + bytes([0x00, 0x01])
            + bytes([0x03]) + b"env" + bytes([0x0A]) + b"soc_read32" + bytes([0x00, 0x02])
            + bytes([0x03]) + b"env" + bytes([0x0B]) + b"soc_write32" + bytes([0x00, 0x03]))
    tabsec = uleb(1) + bytes([0x70, 0x00]) + uleb(nsegs)
    elem = uleb(1) + bytes([0x00, I32C]) + sleb(0) + bytes([END])
    elem += uleb(nsegs)
    for k in range(nsegs):
        elem += uleb(4 + k)
    segs = bytearray()
    for off, arr in ((M_DRAM_OFF, snaps["dram"]), (M_IRAM_OFF, snaps["iram"]),
                     (M_ROM_OFF, snaps["rom"])):
        data = b"".join(int(x & 0xFFFFFFFF).to_bytes(4, "little") for x in arr)
        segs += bytes([0x00]) + bytes([I32C]) + sleb(off) + bytes([END])
        segs += uleb(len(data)) + data
    mod = (
        b"\x00asm\x01\x00\x00\x00"
        + section(1, uleb(5) + bytes([0x60, 0x00, 0x00])
                  + bytes([0x60, 0x01, 0x7F, 0x00])
                  + bytes([0x60, 0x01, 0x7F, 0x01, 0x7F])
                  + bytes([0x60, 0x02, 0x7F, 0x7F, 0x00])
                  + bytes([0x60, 0x00, 0x01, 0x7F]))
        + section(2, imps)
        + section(3, uleb(len(allbodies)) + bytes([0x00]) + bytes([0x04]) * (len(allbodies) - 1))
        + section(4, tabsec)
        + section(5, memsec)
        + section(6, glob)
        + section(7, exports([(b"run", 0x00, 0x03), (b"mem", 0x02, 0x00),
                              (b"wb", 0x03, 0x00), (b"ws", 0x03, 0x01),
                              (b"ps", 0x03, 0x02)]))
        + section(9, elem)
        + section(10, code)
        + section(11, uleb(3) + segs)
    )
    open("/tmp/opencode/spike_static.wasm", "wb").write(mod)
    rep = {"entry": sfunc["entry"], "segs": nsegs, "covered": stats["covered"],
           "traps": [hex(p) for p in stats["traps"]], "mmio": 0,
           "mmio_rt": stats["mmio_rt"], "leaf_end_pc": leaf_end["pc"],
           "regs1": exp_regs1, "wb1": exp_wb1, "ws1": exp_ws1,
           "ps1": exp_ps1, "writes": oracle["mem"]}
    json.dump(rep, open("/tmp/opencode/spike_static.json", "w"))
    print(f"static func 0x{sfunc['entry']:x}: segs={nsegs} covered={stats['covered']} "
          f"trapped={len(stats['traps'])} mmio_rt={stats['mmio_rt']}")


if __name__ == "__main__":
    main()
