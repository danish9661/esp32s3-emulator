#!/usr/bin/env python3
"""Synthetic two-function call tree (static linking smoke): caller calls
callee, value flows back, both return. Uses real E974 window state for
ENTRY/RETW guard fidelity (synthetic a0=0 is correctly rejected by the
retw callinc guard — proven live). Regenerates /tmp scratch deterministically
(the old /tmp-only copy was reaped; this one lives in-repo).
Usage: synth_tree.py  ->  /tmp/opencode/synth_{caller,callee,oracle}.json."""
import json
import os


def op(v):
    return {"v": v, "r": 0, "vis": 1}


def main():
    os.makedirs("/tmp/opencode", exist_ok=True)
    caller = {"entry": 0x1000, "insns": [
        {"pc": 0x1000, "len": 3, "raw": 0, "opc": "entry", "opnds": [op(1), op(0), op(0x20)]},
        {"pc": 0x1003, "len": 3, "raw": 0, "opc": "movi", "opnds": [op(2), op(10)]},
        {"pc": 0x1006, "len": 3, "raw": 0, "opc": "call8", "opnds": [op(0x2000), op(8)]},
        # Post-entry caller-a4 lives in phys slot 36 (wb8: 8*4+4; proven by
        # pattern-tagged probe); end-a10 (slot 34) carries the result out
        # through retw rotation (end-a2 is NOT the result — window geometry).
        {"pc": 0x1009, "len": 3, "raw": 0, "opc": "add", "opnds": [op(2), op(2), op(4)]},
        {"pc": 0x100C, "len": 3, "raw": 0, "opc": "retw", "opnds": []},
    ]}
    callee = {"entry": 0x2000, "insns": [
        {"pc": 0x2000, "len": 3, "raw": 0, "opc": "entry", "opnds": [op(1), op(0), op(0x20)]},
        {"pc": 0x2003, "len": 3, "raw": 0, "opc": "movi", "opnds": [op(3), op(7)]},
        {"pc": 0x2006, "len": 3, "raw": 0, "opc": "retw", "opnds": []},
    ]}
    json.dump({"sfunc": caller}, open("/tmp/opencode/synth_caller.json", "w"))
    json.dump({"sfunc": callee}, open("/tmp/opencode/synth_callee.json", "w"))
    e = json.load(open("/tmp/opencode/funcE974.json"))
    oracle = {
        "regs0": list(e["regs0"]), "phys0": list(e["phys0"]),
        "wb0": e["wb0"], "ws0": e["ws0"], "ps0": e["ps0"],
        "steps": [{"pc": 0xDEAD, "regs": [0] * 16, "wb": 0, "ws": 0, "ps": 0}],
        "regs1": [0] * 16, "wb1": 0, "ws1": 0, "ps1": 0, "mem": [],
        "fulldram": list(e["fulldram"]), "fulliram": list(e["fulliram"]),
        "fullrom": list(e["fullrom"]),
    }
    oracle["phys0"][36] = 100  # caller post-entry a4 (slot proven live)
    json.dump(oracle, open("/tmp/opencode/synth_oracle.json", "w"))
    print("synth inputs written (needs funcE974.json oracle first)")


if __name__ == "__main__":
    main()
