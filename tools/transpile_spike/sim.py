#!/usr/bin/env python3
"""SPIKE tooling: wasm stack-type simulator (validates emitter output).
Usage: sim.py <module.wasm>. Checks every function body; reports the
first type/length mismatch with offset. Supports the op subset the
spike emitter uses (grows with it). DELETE or promote with the spike."""

import sys

I32, I64 = "i32", "i64"


class Sim:
    def __init__(self, code, nlocals, nglobals, ret, calls=None):
        self.calls = calls or {}
        self.c = code
        self.pos = 0
        self.st = []
        self.ctrl = ['func']  # function body is an implicit frame
        self.ctrl_reach = [True]
        # Per-frame (stack snapshot, diverged): an arm that returns/branches
        # out must not corrupt fall-through accounting (the budget-IF's
        # DROP+BR left sim validating fall-through with a popped stack —
        # proven by the spanLoop dispatcher failure). On ELSE restore the
        # snapshot (both arms start from pre-IF stack); on END of a diverged
        # non-func frame restore it too. Sound for our subset: every block
        # is EMPTY-typed with []-leaving arms.
        self.ctrl_snap = [[]]
        self.ctrl_div = [False]
        self.nlocals = nlocals
        self.nglobals = nglobals
        self.ret = ret

    def uleb(self):
        n = s = 0
        while True:
            b = self.c[self.pos]
            self.pos += 1
            n |= (b & 0x7F) << s
            s += 7
            if not b & 0x80:
                return n

    def sleb(self):
        n = s = 0
        while True:
            b = self.c[self.pos]
            self.pos += 1
            n |= (b & 0x7F) << s
            s += 7
            if not b & 0x80:
                return n - (1 << s) if b & 0x40 else n

    def skip_uleb(self):
        while self.c[self.pos] & 0x80:
            self.pos += 1
        self.pos += 1

    def skip_block(self):
        depth = 1
        while depth:
            b = self.c[self.pos]
            self.pos += 1
            if b in (0x02, 0x03, 0x04):
                self.pos += 1  # blocktype
                depth += 1
            elif b == 0x0B:
                depth -= 1
            elif b in (0x0C, 0x0D, 0x10, 0x20, 0x21, 0x22, 0x23, 0x24):
                self.skip_uleb()
            elif b == 0x11:
                self.skip_uleb()
                self.skip_uleb()
            elif b in (0x28, 0x2C, 0x36, 0x3A):
                self.skip_uleb()
                self.skip_uleb()
            elif b in (0x41, 0x42):
                self.skip_uleb()  # signed LEB: same skip shape
            elif b in (0x00, 0x0F, 0x1A, 0x1B, 0x45, 0x46, 0x47, 0x48, 0x49,
                       0x4E, 0x4F, 0x67, 0x6A, 0x6B, 0x6C, 0x71, 0x72, 0x73,
                       0x74, 0x75, 0x76, 0x84, 0x86, 0x87, 0x88, 0xA7, 0xAC, 0xAD,
                       0x6D, 0x6E, 0x6F, 0x70):
                pass
            else:
                raise AssertionError(f"skip: unknown op {b:#x} at +{self.pos - 1}")

    def push(self, typ):
        self.st.append((typ, len(self.ctrl) - 1))

    def pop(self, want=None):
        assert self.st, f"stack underflow at +{self.pos}"
        got, born = self.st.pop()
        assert born == len(self.ctrl) - 1, (
            f"cross-frame use at +{self.pos} (born in frame {born}, now {len(self.ctrl) - 1})")
        if want is not None:
            assert got == want, f"type: want {want} got {got} at +{self.pos}"
        return got

    def run(self):
        # Real control validation with reachability: after `return` or
        # `unreachable`, value ops only consume immediates until the next
        # structural boundary; `end` restores the enclosing reachability.
        # (Linear scan broke on multi-return branched transfers.)
        end = len(self.c)
        reachable = [True]
        while self.pos < end:
            op = self.c[self.pos]
            self.pos += 1
            ok = reachable[-1]
            if op == 0x00:
                # Legal instruction (traps at runtime); code after it in
                # the same block is unreachable until the next boundary.
                reachable[-1] = False
                if self.ctrl:
                    self.ctrl_div[-1] = True
            elif op in (0x02, 0x03):
                assert self.c[self.pos] == 0x40, f"non-empty blocktype at +{self.pos}"
                self.pos += 1
                self.ctrl.append(op)
                self.ctrl_reach.append(reachable[-1])
                self.ctrl_snap.append(list(self.st) if ok else [])
                self.ctrl_div.append(False)
                reachable.append(reachable[-1])
            elif op == 0x0B:
                assert self.ctrl, f"stray end at +{self.pos}"
                is_func_end = (self.ctrl[-1] == 'func')
                if is_func_end:
                    # Function end: if unreachable (all paths returned),
                    # stack check is vacuous (returns already validated).
                    self.end_reachable = reachable[-1]
                elif self.ctrl_div[-1]:
                    # Diverged frame (return/br out): fall-through resumes
                    # with the pre-frame stack, not the dead arm's.
                    self.st = self.ctrl_snap[-1]
                self.ctrl.pop()
                self.ctrl_reach.pop()
                self.ctrl_snap.pop()
                self.ctrl_div.pop()
                reachable.pop()
            elif op == 0x0C:
                self.uleb()  # br depth (no type change in our shapes)
                # Unconditional branch: following ops unreachable until the
                # enclosing END (budget-exit br lives right before its END).
                reachable[-1] = False
                if self.ctrl:
                    self.ctrl_div[-1] = True
            elif op == 0x0D:
                self.uleb()
                if ok:
                    self.pop(I32)  # br_if cond
            elif op == 0x04:
                assert self.c[self.pos] == 0x40
                self.pos += 1
                if ok:
                    self.pop(I32)
                self.ctrl.append(op)
                self.ctrl_reach.append(reachable[-1])
                self.ctrl_snap.append(list(self.st) if ok else [])
                self.ctrl_div.append(False)
                reachable.append(reachable[-1])
            elif op == 0x05:
                assert self.ctrl and self.ctrl[-1] == 0x04
                # Else-arm starts from the pre-IF stack (taken-arm effects,
                # including divergence, do not leak across).
                self.st = list(self.ctrl_snap[-1])
                self.ctrl_div[-1] = False
                reachable[-1] = self.ctrl_reach[-1]
            elif op == 0x0F:
                if ok:
                    for t in self.ret:
                        self.pop(t)
                reachable[-1] = False
                if self.ctrl:
                    self.ctrl_div[-1] = True
            elif op == 0x1B:
                if ok:
                    c = self.pop(I32)
                    v2 = self.pop()
                    v1 = self.pop()
                    assert v1 == v2, f"select arms {v1}/{v2} at +{self.pos}"
                    self.push(v1)
            elif op == 0x1A:
                if ok:
                    self.pop()
            elif op == 0x10:
                idx = self.uleb()
                npar, nres = self.calls.get(idx, (0, []))
                if ok:
                    for _ in range(npar):
                        self.pop()
                    for t in nres:
                        self.push(t)
            elif op == 0x11:
                self.uleb()  # typeidx
                self.uleb()  # tableidx
                if ok:
                    self.pop(I32)  # funcidx; our segs are [] -> [i32]
                    self.push(I32)
            elif op in (0x20, 0x22):
                i = self.uleb()
                assert i < self.nlocals, f"local {i} at +{self.pos}"
                if op == 0x20:
                    if ok:
                        self.push(I32)
                else:
                    if ok:
                        assert self.st and self.st[-1][0] == I32
                        # tee nets zero (pop value, push value, set local).
            elif op == 0x21:
                i = self.uleb()
                assert i < self.nlocals
                if ok:
                    self.pop(I32)
            elif op in (0x23,):
                i = self.uleb()
                assert i < self.nglobals, f"global {i} at +{self.pos}"
                if ok:
                    self.push(I32)
            elif op == 0x24:
                i = self.uleb()
                assert i < self.nglobals
                if ok:
                    self.pop(I32)
            elif op in (0x28, 0x2C):
                self.uleb()
                self.uleb()
                if ok:
                    self.pop(I32)
                    self.push(I32)
            elif op in (0x36, 0x3A):
                self.uleb()
                self.uleb()
                if ok:
                    self.pop(I32)
                    self.pop(I32)
            elif op in (0x41, 0x42):
                self.sleb()
                if ok:
                    self.push(I32 if op == 0x41 else I64)
            elif op in (0x45,):
                if ok:
                    self.pop(I32)
                    self.push(I32)  # eqz
            elif op == 0x67:
                if ok:
                    self.pop(I32)  # clz: single operand (nsau)
                    self.push(I32)
            elif op in (0x46, 0x47, 0x48, 0x49, 0x4E, 0x4F, 0x6D, 0x6E, 0x6F, 0x70):
                if ok:
                    self.pop(I32)
                    self.pop(I32)
                    self.push(I32)
            elif op in (0x6A, 0x6B, 0x6C, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76):
                if ok:
                    self.pop(I32)
                    self.pop(I32)
                    self.push(I32)
            elif op in (0x84, 0x86, 0x87, 0x88):
                if ok:
                    self.pop(I64)
                    self.pop(I64)
                    self.push(I64)
            elif op in (0xAC, 0xAD):
                if ok:
                    self.pop(I32)
                    self.push(I64)
            elif op == 0xA7:
                if ok:
                    self.pop(I64)
                    self.push(I32)
            else:
                raise AssertionError(f"unknown op {op:#x} at +{self.pos - 1}")
        assert not self.ctrl, "unclosed block"
        if getattr(self, 'end_reachable', True):
            assert [t for t, _ in self.st] == self.ret, f"end stack {self.st} != ret {self.ret}"


CALLSIGS = {
    "spike_win.wasm": {1: (0, [])},
    "spike_call.wasm": {1: (0, [])},
    "spike_mmio.wasm": {0: (2, []), 1: (1, ["i32"])},
    "spike_poll.wasm": {0: (0, ["i32"]), 1: (1, [])},
    "spike_auto.wasm": {0: (1, []), 1: (1, ["i32"]), 2: (2, []), 3: (3, ["i32"])},
    "spike_static.wasm": {0: (1, []), 1: (1, ["i32"]), 2: (2, []), 3: (3, ["i32"])},
    "dmem_kat.wasm": {0: (1, []), 1: (1, ["i32"]), 2: (2, [])},
    "scas_kat.wasm": {0: (1, []), 1: (1, ["i32"]), 2: (2, []), 3: (3, ["i32"])},
    "spike_s4bridge.wasm": {0: (1, []), 1: (1, ["i32"]), 2: (2, []), 3: (0, ["i32"])},
}


def check(path, funcs):
    """funcs: list of (code_bytes, nlocals, nglobals, ret_types)."""
    name = path.rsplit("/", 1)[-1]
    data = open(path, "rb").read()
    assert data[:4] == b"\x00asm"
    # Minimal section walk: find code section (id 10).
    pos = 8
    bodies = []

    def rd_uleb():
        nonlocal pos
        n = s = 0
        while True:
            b = data[pos]
            pos += 1
            n |= (b & 0x7F) << s
            s += 7
            if not b & 0x80:
                return n

    while pos < len(data):
        sid = data[pos]
        pos += 1
        size = rd_uleb()
        payload = data[pos:pos + size]
        pos += size
        if sid == 10:
            q = 0

            def qleb():
                nonlocal q
                n = s = 0
                while True:
                    b = payload[q]
                    q += 1
                    n |= (b & 0x7F) << s
                    s += 7
                    if not b & 0x80:
                        return n

            nfunc = qleb()
            for _ in range(nfunc):
                fsize = qleb()
                fbody = payload[q:q + fsize]
                q += fsize
                # locals vec
                r = 0

                def rleb():
                    nonlocal r
                    n = s = 0
                    while True:
                        b = fbody[r]
                        r += 1
                        n |= (b & 0x7F) << s
                        s += 7
                        if not b & 0x80:
                            return n

                ngroups = rleb()
                nloc = 0
                for _ in range(ngroups):
                    cnt = rleb()
                    typ = fbody[r]
                    r += 1
                    assert typ == 0x7F, f"non-i32 local in spike (type {typ:#x})"
                    nloc += cnt
                bodies.append((fbody[r:], nloc))
    if isinstance(funcs, tuple):
        if name in ("spike_auto.wasm", "spike_static.wasm"):
            # run/resume return []; seg funcs return [i32] (next slot).
            funcs = [(5, []), (5, [])] + [(5, [I32])] * (len(bodies) - 2)
        else:
            # Broadcast single spec to every body (multi-func modules).
            funcs = [funcs] * len(bodies)
    assert len(bodies) == len(funcs), f"{len(bodies)} bodies vs {len(funcs)} specs"
    for i, ((code, nloc), (nglob, ret)) in enumerate(zip(bodies, funcs)):
        try:
            Sim(code, nloc, nglob, ret, CALLSIGS.get(name)).run()
        except AssertionError as e:
            raise AssertionError(f"func #{i}: {e}")
        print(f"func #{i}: sim OK ({len(code)} bytes)")


if __name__ == "__main__":
    path = sys.argv[1]
    # (nglobals, ret-stack) per function, in order.
    specs = {
        "spike_win.wasm": [(3, [I64]), (3, [])],
        "spike_call.wasm": [(0, [I64]), (0, [])],
        "spike_block.wasm": [(0, [I64])],
        "spike_mem.wasm": [(0, [I64])],
        "spike_loop.wasm": [(0, [I64])],
        "spike_loop2.wasm": [(0, [I64])],
        "spike_loop3.wasm": [(0, [I64])],
        "spike_mmio.wasm": [(0, [I64])],
        "spike_poll.wasm": [(0, [I64])],
        "spike_windiff.wasm": [(3, [])],
        "spike_auto.wasm": (4, []),
        "spike_static.wasm": (4, []),
        "wcond_kat.wasm": (1, []),
        "dmem_kat.wasm": (1, []),
        "movcc_kat.wasm": (1, []),
        "tail_kat.wasm": (1, []),
        "scas_kat.wasm": (1, []),
        "spike_s4bridge.wasm": [(0, [I64])],
        "spike_diff.wasm": [(0, [])],
    }
    name = path.rsplit("/", 1)[-1]
    check(path, specs[name])
    print("SIM ALL OK")
