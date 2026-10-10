#!/usr/bin/env python3
"""S3 static function transpiler v2 (call trees): instruction streams from
STATIC decode (sfunc JSONs), data from DIRECT memory (snapshots preloaded,
MMIO via imports). v1 proved leaves; v2 links direct calls (shared
dispatcher + shadow, S3b shapes) and snapshot-literal-resolved callx8
(pattern: dominating in-block l32r into the call reg; production needs a
guard/deopt — S5/S6). v1 limits kept otherwise (jx/ret/ret_n trap loud).
Usage: static_func.py <sfunc.json> <functoracle.json> [extra-sfunc...]
  -> /tmp/opencode/spike_static.{wasm,json} (run with static_run.mjs).
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
CALL_OPS = ("call4", "call8", "call12", "callx8")
XFER_OPS = ("jx", "j") + CALL_OPS + ("retw", "retw_n", "ret", "ret_n")


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
    """Snapshot value for a static address (l32r/callx baking). None if out."""
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


def writes_reg(opc, o):
    """Regs an op may write (for callx8 literal-pattern invalidation).
    None = arbitrary (rotation/call): clear the whole map."""
    if opc in ("entry", "retw", "retw_n", "ret", "ret_n") + CALL_OPS + ("jx", "j"):
        return None
    if opc in ("l8ui", "l32i", "l32i_n", "l32r", "movi", "movi_n", "mov_n",
               "addi", "addi_n", "addmi", "add", "add_n", "sub", "and", "or",
               "xor", "slli", "srli", "srai", "extui", "addx4",
               "moveqz", "movnez", "movltz", "movgez"):
        return {o[0]["v"]}
    return set()


def split_func(insns):
    """Static block split. Returns (seg_list, seg_id_of_pos, pc_of_seg)."""
    starts = {0}
    pcs = [s["pc"] for s in insns]
    pcset = set(pcs)
    for i, s in enumerate(insns):
        opc = s["opc"]
        if opc in BRANCH2 or opc in BRANCH3 or opc in XFER_OPS:
            if i + 1 < len(insns):
                starts.add(i + 1)
    for i, s in enumerate(insns):
        opc = s["opc"]
        if opc in BRANCH2 or opc in BRANCH3 or opc == "j":
            o = s["opnds"]
            tgt = o[0]["v"] if opc == "j" else branch_taken_py(opc, o, [0] * 16)[1]
            if tgt in pcset:
                starts.add(pcs.index(tgt))
    seg_list = sorted(starts) + [len(insns)]
    seg_id_of_pos = {}
    for k in range(len(seg_list) - 1):
        for i in range(seg_list[k], seg_list[k + 1]):
            seg_id_of_pos[i] = k
    return seg_list, seg_id_of_pos


def emit_call_seq(out, opc, o, pc, length):
    """S3b call sequence: ra to caller phys + PS update (no shadow push)."""
    ci = o[1]["v"] // 4
    ra = ((ci << 30) | ((pc + length) & 0x3FFFFFFF)) & 0xFFFFFFFF
    out += wst(ci * 4, C(ra))
    out += (bytearray(bytes([GLG, 2])) + C(0x30000) + C(-1)
            + bytearray([I32XOR, I32AND]) + C(ci)
            + C(CI_SHIFT) + bytearray([I32SHL, I32OR, GLS, 2]))


def main():
    import autodiff as AD
    assert (AD.TRAP_I, AD.RD_I, AD.WR_I) == (TRAP_I, RD_I, WR_I), "import contract"
    sfunc = json.load(open(sys.argv[1]))["sfunc"]
    oracle = json.load(open(sys.argv[2]))
    extra = [json.load(open(p))["sfunc"] for p in sys.argv[3:]]
    funcs = [sfunc] + extra
    for f in funcs:
        assert f["insns"] and f["insns"][0]["pc"] == f["entry"], "static must start at entry"
    snaps = {"dram": oracle["fulldram"], "iram": oracle["fulliram"], "rom": oracle["fullrom"]}
    static_pcs = set(s["pc"] for f in funcs for s in f["insns"])
    # Root-end expectation: first oracle step OUTSIDE the transcribed union.
    leaf_end = next((st for st in oracle["steps"] if st["pc"] not in static_pcs), None)
    assert leaf_end is not None, "oracle never leaves transcribed set (window too small?)"
    exp_regs1, exp_wb1, exp_ws1, exp_ps1 = (leaf_end["regs"], leaf_end["wb"],
                                            leaf_end["ws"], leaf_end["ps"])

    # Global seg space across functions (func 0 seg 0 = dispatcher entry).
    splits, base_of, total = [], [], 0
    pc_to_seg = {}
    for f in funcs:
        seg_list, _ = split_func(f["insns"])
        splits.append(seg_list)
        base_of.append(total)
        total += len(seg_list) - 1
    nsegs = total
    for fi, f in enumerate(funcs):
        for k in range(len(splits[fi]) - 1):
            pc_to_seg.setdefault(f["insns"][splits[fi][k]]["pc"], base_of[fi] + k)
    gseg = lambda fi, k: base_of[fi] + k  # noqa: E731

    stats = {"covered": 0, "mmio_rt": 0, "traps": [], "calls": 0, "linked": 0,
             "callx_resolved": 0, "funcs": len(funcs), "segs": nsegs}
    bodies = []
    for fi, f in enumerate(funcs):
        insns = f["insns"]
        seg_list = splits[fi]
        seg_id_of_pos = {}
        for k in range(len(seg_list) - 1):
            for i in range(seg_list[k], seg_list[k + 1]):
                seg_id_of_pos[i] = k
        for sg in range(len(seg_list) - 1):
            lo, hi = seg_list[sg], seg_list[sg + 1]
            out = bytearray()
            litmap = {}  # reg -> literal addr (callx8 pattern; cleared on writes)
            for si in range(lo, hi):
                s_ = insns[si]
                opc, o = s_["opc"], s_["opnds"]
                if opc == "l32r":
                    litmap[o[0]["v"]] = o[1]["v"]
                if opc in COVERED and opc not in ("l8ui", "l32i", "l32i_n", "l32r",
                                                  "s32i", "s32i_n", "s8i"):
                    if opc in BRANCH2 or opc in BRANCH3:
                        assert (si + 1) in seg_id_of_pos, "branch at static end"
                        _, tgt = branch_taken_py(opc, o, [0] * 16)
                        out += bytes([BLOCK, EMPTY])
                        out += wcond(opc, o)
                        out += bytes([BRIF, 0x00])
                        out += C(gseg(fi, seg_id_of_pos[si + 1])) + bytes([RET])
                        out += bytes([END])
                        assert tgt in pc_to_seg, f"static target not a block: {tgt:#x}"
                        out += C(pc_to_seg[tgt]) + bytes([RET])
                        break
                    if opc == "j":
                        assert o[0]["v"] in pc_to_seg, f"j outside set: {o[0]['v']:#x}"
                        out += C(pc_to_seg[o[0]["v"]]) + bytes([RET])
                        break
                    if opc in ("call4", "call8", "call12"):
                        tgt = o[0]["v"]
                        assert tgt in pc_to_seg, f"call outside set: {tgt:#x}"
                        emit_call_seq(out, opc, o, s_["pc"], s_["len"])
                        out += ssp_push(C(gseg(fi, seg_id_of_pos[si + 1])))
                        out += C(pc_to_seg[tgt]) + bytes([RET])
                        stats["calls"] += 1
                        stats["linked"] += 1
                        litmap = {}
                        break
                    if opc == "callx8":
                        reg = o[0]["v"]
                        lit = litmap.get(reg)
                        v = snap_val(snaps, lit, 4) if lit is not None else None
                        if v is None or v not in pc_to_seg:
                            stats["traps"].append(s_["pc"])
                            out += C(s_["pc"]) + bytes([CALL, TRAP_I])
                            out += C(HALT) + bytes([RET])
                            break
                        emit_call_seq(out, opc, o, s_["pc"], s_["len"])
                        out += ssp_push(C(gseg(fi, seg_id_of_pos[si + 1])))
                        out += C(pc_to_seg[v]) + bytes([RET])
                        stats["calls"] += 1
                        stats["linked"] += 1
                        stats["callx_resolved"] += 1
                        litmap = {}
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
                    wr = writes_reg(opc, o)
                    litmap = {} if wr is None else {r: v for r, v in litmap.items()
                                                    if r not in wr}
                    continue
                if opc in ("l8ui", "l32i", "l32i_n"):
                    width = 1 if opc == "l8ui" else 4
                    out += wphys(C(o[0]["v"]))
                    out += wld(o[1]["v"]) + C(o[2]["v"]) + bytearray([I32ADD])
                    out += dload(width)
                    out += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
                    stats["covered"] += 1
                    wr = writes_reg(opc, o)
                    litmap = {r: v for r, v in litmap.items() if r not in wr}
                    continue
                if opc == "l32r":
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
                    width = 1 if opc == "s8i" else 4
                    out += wld(o[0]["v"])
                    out += wld(o[1]["v"]) + C(o[2]["v"]) + bytearray([I32ADD])
                    out += dstore(width)
                    stats["covered"] += 1
                    wr = writes_reg(opc, o)
                    litmap = {r: v for r, v in litmap.items() if r not in wr}
                    continue
                stats["traps"].append(s_["pc"])
                out += C(s_["pc"]) + bytes([CALL, TRAP_I])
                out += C(HALT) + bytes([RET])
                break
            else:
                if hi == len(insns):
                    out += C(HALT) + bytes([RET])
                else:
                    out += C(gseg(fi, sg + 1)) + bytes([RET])
            out += bytes([END])
            bodies.append(bytes(out))

    run = bytearray()
    run += C(oracle["wb0"]) + bytearray([GLS, 0])
    run += C(oracle["ws0"]) + bytearray([GLS, 1])
    run += C(oracle["ps0"]) + bytearray([GLS, 2])
    run += C(0) + bytearray([GLS, 3])
    for pp, vv in enumerate(oracle["phys0"]):
        run += C(pp * 4) + C(vv) + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
    run += ssp_push(C(HALT))
    run += C(0) + bytearray([LS, 0])
    run += bytes([LOOP, EMPTY])
    run += bytes([LG, 0]) + bytes([CALL_IND, 0x04, 0x00])
    run += bytes([0x22, 0x00])
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
           "funcs": [hex(f["entry"]) for f in funcs],
           "calls": stats["calls"], "linked": stats["linked"],
           "callx_resolved": stats["callx_resolved"],
           "regs1": exp_regs1, "wb1": exp_wb1, "ws1": exp_ws1,
           "ps1": exp_ps1, "writes": oracle["mem"]}
    json.dump(rep, open("/tmp/opencode/spike_static.json", "w"))
    print(f"static {len(funcs)} funcs: segs={nsegs} covered={stats['covered']} "
          f"calls={stats['linked']}/{stats['calls']} callx={stats['callx_resolved']} "
          f"trapped={len(stats['traps'])}")


if __name__ == "__main__":
    main()
