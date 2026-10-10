#!/usr/bin/env python3
"""S4 function-granular transpiler v2 (trampoline): segs split at landings,
post-call continuations, and trace start. Every transfer returns the next
seg id; run() loops call_indirect; nested calls use an explicit shadow
stack (push continuation on call, pop on retw). No wasm-call nesting, so
jumps/branches/returns chain correctly (v1 stranded after the first
inter-seg jump). Globals wb(0)/ws(1)/ps(2)/ssp(3); AR+mirrors+baked+shadow
in module memory. Usage: autodiff_func.py <span.json>."""

import json
import sys

sys.path.insert(0, "/home/danish1075/Documents/esp32 s3 emu/tools/transpile_spike")
from xwasm import (uleb, sleb, section, exports, funcbody, I32C, I32ADD,
                   I32SUB, I32LD, I32ST, I32AND, I32OR, I32XOR, I32SHL,
                   I32SHRU, I32SHRS, I32LOAD8U, I32EQ, I32NE, I32EQZ,
                   I32GEU, I32LTS, GLG, GLS, UNREACH, END)
from windiff import (C, wld, wst, wphys, woe_check, walu, slot_load, slot_store,
                     emit_branch_verify, emit_call, emit_entry, emit_retw,
                     emit_special, branch_taken_py, wcond, BRANCH2, BRANCH3,
                     COVERED, DRAM_BASE, DRAM_SIZE, FMIRR,
                     LG, LS, MUL, SELECT, IF, EMPTY, WOE_BIT, CI_SHIFT)
from autodiff import is_mmio, TRAP_I, RD_I, WR_I  # noqa: F401 (contract below)

CALL, RET, CALL_IND, BRIF = 0x10, 0x0F, 0x11, 0x0D
BLOCK, LOOP = 0x02, 0x03
HALT = 0xFFFFFFFF
SHADOW_BASE = 0x200000


def ssp_push(idval):
    """Push wasm-level seg funcidx; idval: bytearray leaving id on stack."""
    return (C(SHADOW_BASE) + bytearray(bytes([GLG, 3]))
            + bytearray([I32ADD]) + idval
            + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
            + bytearray(bytes([GLG, 3])) + C(4) + bytearray([I32ADD, GLS, 3]))


def ssp_pop():
    """Pop into stack (trap on empty): ssp -= 4; mem[SHADOW+ssp]."""
    return (bytearray(bytes([GLG, 3])) + bytearray([I32EQZ, IF, EMPTY, UNREACH, END])
            + bytearray(bytes([GLG, 3])) + C(4) + bytearray([I32SUB, GLS, 3])
            + C(SHADOW_BASE) + bytearray(bytes([GLG, 3]))
            + bytearray([I32ADD]) + bytearray(bytes([I32LD]) + uleb(2) + uleb(0)))


def main():
    import autodiff as AD
    assert (AD.TRAP_I, AD.RD_I, AD.WR_I) == (0, 1, 2), "import contract"
    span = json.load(open(sys.argv[1]))
    steps = span["steps"]
    starts = {0}
    for i in range(1, len(steps)):
        prev = steps[i - 1]
        if steps[i]["pc"] != (prev["pc"] + prev["len"]) & 0xFFFFFFFF:
            starts.add(i)
    for i, s in enumerate(steps):
        if s["opc"] in ("call4", "call8", "call12", "callx8") and i + 1 < len(steps):
            starts.add(i + 1)
    for i, s in enumerate(steps):
        if (s["opc"] in BRANCH2 or s["opc"] in BRANCH3
                or s["opc"] in ("jx", "j")) and i + 1 < len(steps):
            starts.add(i + 1)
    for i, s in enumerate(steps):
        if s["opc"] in ("retw", "retw_n") and i + 1 < len(steps):
            starts.add(i + 1)
    seg_list = sorted(starts) + [len(steps)]
    seg_id_of_pos = {}
    for k in range(len(seg_list) - 1):
        for i in range(seg_list[k], seg_list[k + 1]):
            seg_id_of_pos[i] = k
    pc_to_seg = {}
    for k in range(len(seg_list) - 1):
        # First wins: loop iterations duplicate head pcs; back-edges must
        # resolve to the head seg (smaller id) to classify backward.
        pc_to_seg.setdefault(steps[seg_list[k]]["pc"], k)
    nsegs = len(seg_list) - 1
    # Branch-site pre-pass: a backward branch's exit seg comes from its
    # LAST hit's fall-through (first hit's fall-through is iteration 2!).
    # All hits of one site must share the target (else irreducible: loud).
    br_hits = {}
    for i, s_ in enumerate(steps):
        if (s_["opc"] in BRANCH2 or s_["opc"] in BRANCH3
                or s_["opc"] in ("jx", "j")):
            br_hits.setdefault(s_["pc"], []).append(i)
    # Pairing pre-pass: match each call with its in-trace retw (by return
    # pc) and vice versa. Dangling calls (no return in trace) push nothing;
    # boundary returns (no call in trace) route statically. This keeps the
    # runtime shadow exactly balanced for every execution of the trace.
    match_call, match_ret = {}, {}
    open_calls = []  # (step idx, return pc)
    for i, s_ in enumerate(steps):
        opc = s_["opc"]
        if opc in ("call4", "call8", "call12", "callx8"):
            open_calls.append((i, (s_["pc"] + s_["len"]) & 0xFFFFFFFF))
        elif opc in ("retw", "retw_n") and i + 1 < len(steps):
            nxtpc = steps[i + 1]["pc"]
            for k in range(len(open_calls) - 1, -1, -1):
                if open_calls[k][1] == nxtpc:
                    ci, _ = open_calls.pop(k)
                    match_call[ci] = i
                    match_ret[i] = ci
                    break
    # funcidx: trap(0)/read(1)/write(2) imports, run(3), seg_k -> 4+k.
    segfunc = lambda k: 4 + k  # noqa: E731

    reads = {(a, w): v for a, v, w in span["reads"]}
    fwd, baked, slot_next = {}, {}, [FMIRR]
    stats = {"covered": 0, "mmio": 0, "traps": [], "mmios": [],
             "calls": 0, "linked": 0, "segs": nsegs,
             "dangling": 0, "boundary": 0}

    bodies = []
    for sg in range(nsegs):
        lo, hi = seg_list[sg], seg_list[sg + 1]
        out = bytearray()
        # Returns next-seg-id; HALT ends. Truncated only by transfer arms.
        for si in range(lo, hi):
            s_ = steps[si]
            opc, o = s_["opc"], s_["opnds"]
            nxt = steps[si + 1]["pc"] if si + 1 < len(steps) else span.get("nxt")
            R = s_["regs"]
            if opc in COVERED and opc not in ("l8ui", "l32i", "l32i_n", "l32r",
                                              "s32i", "s32i_n", "s8i"):
                if opc in BRANCH2 or opc in BRANCH3 or opc in ("jx", "j"):
                    # Trace-directed: taken-ness known statically (verified);
                    # return the executed target's seg.
                    if opc == "jx":
                        tgt = R[o[0]["v"]]
                    elif opc == "j":
                        tgt = o[0]["v"]
                    else:
                        _, tgt = branch_taken_py(opc, o, R)
                    emit_branch_verify(opc, o, R, nxt, s_["pc"])
                    assert (si + 1) in seg_id_of_pos, "fall-through must start a seg"
                    ft_seg = seg_id_of_pos[si + 1]
                    # Untaken targets never execute (no landing split); only
                    # taken branches need their target seg to exist.
                    taken_seg = pc_to_seg.get(tgt)
                    if nxt != tgt:
                        out += C(ft_seg) + bytes([RET])
                        break
                    assert taken_seg is not None, f"taken target not a seg: {tgt:#x}"
                    # Unconditional transfers are always static (no cond).
                    # Conditional backward OR SELF edges loop at runtime:
                    # a static-taken self-edge (taken_seg == sg) re-executes
                    # with live state but never tests the exit, so bounded
                    # loops (e.g. the spanLoop countdown) spin forever.
                    if taken_seg <= sg and opc not in ("jx", "j"):
                        # Multi-hit site: exit = after LAST hit; targets equal.
                        hits = br_hits.get(s_["pc"], [si])
                        last = hits[-1]
                        if len(hits) > 1:
                            for h in hits:
                                ht = steps[h]
                                if ht["opc"] in ("jx",):
                                    htg = ht["regs"][ht["opnds"][0]["v"]]
                                elif ht["opc"] == "j":
                                    htg = ht["opnds"][0]["v"]
                                else:
                                    _, htg = branch_taken_py(ht["opc"], ht["opnds"], ht["regs"])
                                assert htg == tgt, f"phase-ordered loop at {s_['pc']:#x}"
                            assert last + 1 < len(steps), "loop at trace end"
                            ft_seg = seg_id_of_pos[last + 1]
                        # Backward edge (loop): runtime cond (S2 shapes).
                        # NOTE: wcond MUST be inside the block: V8 rejects a
                        # br_if whose condition was produced outside the
                        # immediately enclosing block (proven by tst_a/tst_b).
                        out += bytes([BLOCK, EMPTY])
                        out += wcond(opc, o)
                        out += bytes([BRIF, 0x00])
                        out += C(ft_seg) + bytes([RET])
                        out += bytes([END])
                        out += C(taken_seg) + bytes([RET])
                    else:
                        out += C(taken_seg if nxt == tgt else ft_seg) + bytes([RET])
                    break
                elif opc in ("call4", "call8", "call12", "callx8"):
                    ci = o[1]["v"] // 4
                    tgt = R[o[0]["v"]] if opc == "callx8" else o[0]["v"]
                    assert nxt == tgt, f"{opc} diverged"
                    ra = ((ci << 30) | ((s_["pc"] + s_["len"]) & 0x3FFFFFFF)) & 0xFFFFFFFF
                    out += wst(ci * 4, C(ra))
                    out += (bytearray(bytes([GLG, 2])) + C(0x30000) + C(-1)
                            + bytearray([I32XOR, I32AND]) + C(ci) + C(CI_SHIFT)
                            + bytearray([I32SHL, I32OR, GLS, 2]))
                    assert tgt in pc_to_seg, f"call target not a seg: {tgt:#x}"
                    # Paired calls (matching retw in trace) push their
                    # return-site seg; dangling calls push nothing (no retw
                    # will pop it in-trace — pushing would corrupt pairing).
                    if si in match_call:
                        # Return site = step after the matching retw (its
                        # seg was split at creation); span-end retw -> HALT.
                        rseg = seg_id_of_pos.get(match_call[si] + 1, HALT)
                        out += ssp_push(C(rseg))
                    else:
                        stats["dangling"] = stats.get("dangling", 0) + 1
                    out += C(pc_to_seg[tgt]) + bytes([RET])
                    stats["calls"] += 1
                    stats["linked"] += 1
                    break
                elif opc == "entry":
                    emit_entry(out, o)
                elif opc in ("retw", "retw_n"):
                    emit_retw(out, nxt, R)
                    if si in match_ret:
                        # Paired: pop the continuation the matching call
                        # pushed; the dispatcher routes to it.
                        out += ssp_pop() + bytes([RET])
                    else:
                        # Boundary (frame predates trace): static return to
                        # the next-trace seg (post-retw split guarantees it
                        # starts a seg); shadow untouched, stays balanced.
                        assert (si + 1) in seg_id_of_pos
                        stats["boundary"] = stats.get("boundary", 0) + 1
                        out += C(seg_id_of_pos[si + 1]) + bytes([RET])
                    break
                elif opc in ("wsr_ps", "rsr_ps", "rsr_prid", "rsil"):
                    emit_special(out, opc, o)
                elif opc in ("rsync", "memw", "isync", "esync", "dsync"):
                    pass
                else:
                    walu(opc, o, out)
                stats["covered"] += 1
                continue
            if opc in ("l8ui", "l32i", "l32i_n", "l32r"):
                eff = o[1]["v"] if opc == "l32r" else (R[o[1]["v"]] + o[2]["v"]) & 0xFFFFFFFF
                width = 1 if opc == "l8ui" else 4
                if is_mmio(eff):
                    stats["mmio"] += 1
                    stats["mmios"].append((s_["pc"], eff))
                    out += wphys(C(o[0]["v"])) + C(eff) + bytes([CALL, RD_I])
                    out += bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
                    stats["covered"] += 1
                    continue
                if (eff, width) not in reads and (eff & ~3) not in fwd:
                    stats["traps"].append(s_["pc"])
                    out += C(s_["pc"]) + bytes([CALL, TRAP_I])
                    out += C(HALT) + bytes([RET])
                    break
                out += slot_load(o[0]["v"], eff, width, reads, fwd, baked, slot_next)
                stats["covered"] += 1
                continue
            if opc in ("s32i", "s32i_n", "s8i"):
                eff = (R[o[1]["v"]] + o[2]["v"]) & 0xFFFFFFFF
                width = 1 if opc == "s8i" else 4
                if is_mmio(eff):
                    stats["mmio"] += 1
                    stats["mmios"].append((s_["pc"], eff))
                    out += C(eff) + wld(o[0]["v"]) + bytes([CALL, WR_I])
                    stats["covered"] += 1
                    continue
                seq, _ = slot_store(eff, wld(o[0]["v"]), width, fwd, span)
                out += seq
                stats["covered"] += 1
                continue
            stats["traps"].append(s_["pc"])
            out += C(s_["pc"]) + bytes([CALL, TRAP_I])
            out += C(HALT) + bytes([RET])
            break
        else:
            # Fell off seg end: trace end -> HALT, else fall-through seg.
            if hi == len(steps):
                out += C(HALT) + bytes([RET])
            else:
                out += C(sg + 1) + bytes([RET])  # next seg == table slot sg+1
        out += bytes([END])  # structural terminator after every seg (RET exits)
        bodies.append(bytes(out))

    # run(): preset globals + AR phys + ssp, dispatch loop to HALT.
    run = bytearray()
    run += C(steps[0]["wb"]) + bytearray([GLS, 0])
    run += C(steps[0]["ws"]) + bytearray([GLS, 1])
    run += C(steps[0]["ps"]) + bytearray([GLS, 2])
    run += C(0) + bytearray([GLS, 3])  # ssp = 0
    for pp, vv in enumerate(span["phys0"]):
        run += C(pp * 4) + C(vv) + bytearray(bytes([I32ST]) + uleb(2) + uleb(0))
    # Dispatcher: id = 0 (table slot of seg 0); loop until HALT.
    # Per iteration stack is empty at loop top; each op below annotated.
    run += C(0) + bytearray([LS, 0])  # t0 = 0, []
    run += bytes([LOOP, EMPTY])  # []
    run += bytes([LG, 0])  # [id]
    run += bytes([CALL_IND, 0x04, 0x00])  # [nid]
    run += bytes([0x22, 0x00])  # tee t0=nid (nets zero): [nid]
    run += C(HALT) + bytes([I32NE, BRIF, 0x00])  # [cond] -> []
    run += bytes([END, END])
    allbodies = [bytes(run)] + bodies
    code = uleb(len(allbodies))
    for b in allbodies:
        code += funcbody(b, 3)
    need = max([0x80040, SHADOW_BASE + 0x1000, 1028]
                + ([slot_next[0] + 4] if baked else []))  # +TEMP-VISRING ring
    memsec = uleb(1) + bytes([0x00]) + uleb((need + 0xFFFF) // 0x10000)
    glob = (uleb(4)
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(1) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0x40000) + bytes([END])
            + bytes([0x7F, 0x01]) + bytes([I32C]) + sleb(0) + bytes([END]))
    imps = (uleb(3)
            + bytes([0x03]) + b"env" + bytes([0x04]) + b"trap" + bytes([0x00, 0x01])
            + bytes([0x03]) + b"env" + bytes([0x0A]) + b"soc_read32" + bytes([0x00, 0x02])
            + bytes([0x03]) + b"env" + bytes([0x0B]) + b"soc_write32" + bytes([0x00, 0x03]))
    # func table for call_indirect: seg funcs (funcidx 4+k).
    tabsec = uleb(1) + bytes([0x70, 0x00]) + uleb(nsegs)
    # element segment: active table 0 @ 0, seg funcidx vec.
    elem = uleb(1) + bytes([0x00, I32C]) + sleb(0) + bytes([END])
    elem += uleb(nsegs)
    for k in range(nsegs):
        elem += uleb(segfunc(k))
    # Types: 0 []->[] (run), 1 [i32]->[] (trap), 2 [i32]->[i32] (read),
    # 3 [i32,i32]->[] (write), 4 []->[i32] (segs + indirect target).
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
        + section(11, segs if segs else uleb(0))
    )
    open("/tmp/opencode/spike_auto.wasm", "wb").write(mod)
    total = len(steps)
    rep = {"covered": stats["covered"], "mmio": stats["mmio"],
           "traps": [hex(p) for p in stats["traps"]],
           "regs1": span["regs1"], "wb1": span["wb1"], "ws1": span["ws1"],
           "ps1": span["ps1"], "writes": [[a, b, c] for a, b, c in span["mem"]],
           "segs": nsegs, "linked": stats["linked"], "calls": stats["calls"],
           "dangling": stats["dangling"], "boundary": stats["boundary"]}
    json.dump(rep, open("/tmp/opencode/spike_auto.json", "w"))
    print(f"funcs: {nsegs} linked={stats['linked']}/{stats['calls']} "
          f"covered={stats['covered']} mmio={stats['mmio']} "
          f"trapped={len(stats['traps'])} dangling={stats['dangling']} "
          f"boundary={stats['boundary']}")


if __name__ == "__main__":
    main()
