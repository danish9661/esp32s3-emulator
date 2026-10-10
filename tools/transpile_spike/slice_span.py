#!/usr/bin/env python3
"""Slice a subspan [k, j) from a full dump with exact attribution.
Rules (loud fail otherwise): no stores in slice (mem attribution), no
psram writes anywhere, end state from steps[j] (requires j < len).
Usage: slice_span.py <in.json> <out.json> <k> <j>"""
import json
import sys

d = json.load(open(sys.argv[1]))
k, j = int(sys.argv[3]), int(sys.argv[4])
S = d["steps"]
assert 0 <= k < j < len(S), "need steps[j] for end state + nxt"
sub = S[k:j]
stores = [s["opc"] for s in sub
          if s["opc"] in ("s32i", "s32i_n", "s8i", "s16i")]
assert not stores, f"stores need attribution: {stores}"
assert d.get("psram", []) == [], "psram writes present"
p = dict(d)
p["steps"] = sub
p["regs0"] = S[k]["regs"]
p["regs1"] = S[j]["regs"]
p["wb1"] = S[j]["wb"]
p["ws1"] = S[j]["ws"]
p["ps1"] = S[j]["ps"]
p["mem"] = []
p["nxt"] = S[j]["pc"]
json.dump(p, open(sys.argv[2], "w"))
print(f"sliced [{k},{j}): {[s['opc'] for s in sub]}")
