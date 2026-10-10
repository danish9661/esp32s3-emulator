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
            elif b in (0x28, 0x2C, 0x36, 0x3A):
                self.skip_uleb()
                self.skip_uleb()
            elif b in (0x41, 0x42):
                self.skip_uleb()  # signed LEB: same skip shape
            elif b in (0x00, 0x0F, 0x1B, 0x45, 0x46, 0x47, 0x6A, 0x6B, 0x6C,
                       0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x84, 0x86, 0xAD):
                pass
            else:
                raise AssertionError(f"skip: unknown op {b:#x} at +{self.pos - 1}")

    def pop(self, want=None):
        assert self.st, f"stack underflow at +{self.pos}"
        got = self.st.pop()
        if want is not None:
            assert got == want, f"type: want {want} got {got} at +{self.pos}"
        return got

    def run(self):
        end = len(self.c)
        while self.pos < end:
            op = self.c[self.pos]
            self.pos += 1
            if op == 0x00:
                raise AssertionError(f"unreachable hit at +{self.pos}")
            elif op == 0x02 or op == 0x03:
                assert self.c[self.pos] == 0x40, f"non-empty blocktype at +{self.pos}"
                self.pos += 1
                self.ctrl.append(op)
            elif op == 0x0B:
                assert self.ctrl, f"stray end at +{self.pos}"
                self.ctrl.pop()
            elif op == 0x0C:
                self.uleb()  # br depth (no type change in our shapes)
            elif op == 0x0D:
                self.uleb()
                self.pop(I32)  # br_if cond
            elif op == 0x04:
                # Guards only (`if { unreachable }`, no else): validate the
                # fall-through path by skipping to the matching end. The
                # skipper decodes immediates (LEB bytes mimic opcodes).
                assert self.c[self.pos] == 0x40
                self.pos += 1
                self.pop(I32)
                self.skip_block()
            elif op == 0x10:
                idx = self.uleb()
                npar, nres = self.calls.get(idx, (0, []))
                for _ in range(npar):
                    self.pop()
                for t in nres:
                    self.st.append(t)
            elif op == 0x1B:
                c = self.pop(I32)
                v2 = self.pop()
                v1 = self.pop()
                assert v1 == v2, f"select arms {v1}/{v2} at +{self.pos}"
                self.st.append(v1)
            elif op == 0x20:
                i = self.uleb()
                assert i < self.nlocals, f"local {i} at +{self.pos}"
                self.st.append(I32)
            elif op == 0x21:
                i = self.uleb()
                assert i < self.nlocals
                self.pop(I32)
            elif op == 0x22:
                i = self.uleb()
                assert i < self.nlocals
                assert self.st and self.st[-1] == I32
            elif op == 0x23:
                i = self.uleb()
                assert i < self.nglobals, f"global {i} at +{self.pos}"
                self.st.append(I32)
            elif op == 0x24:
                i = self.uleb()
                assert i < self.nglobals
                self.pop(I32)
            elif op in (0x28, 0x2C):
                self.uleb()
                self.uleb()
                self.pop(I32)
                self.st.append(I32)
            elif op in (0x36, 0x3A):
                self.uleb()
                self.uleb()
                self.pop(I32)
                self.pop(I32)
            elif op == 0x41:
                self.sleb()
                self.st.append(I32)
            elif op == 0x42:
                self.sleb()
                self.st.append(I64)
            elif op in (0x45,):
                self.pop(I32)
                self.st.append(I32)  # eqz
            elif op in (0x46, 0x47, 0x49):
                self.pop(I32)
                self.pop(I32)
                self.st.append(I32)  # eq/ne
            elif op in (0x6A, 0x6B, 0x6C, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76):
                self.pop(I32)
                self.pop(I32)
                self.st.append(I32)
            elif op in (0x84, 0x86):
                self.pop(I64)
                self.pop(I64)
                self.st.append(I64)
            elif op == 0xAD:
                self.pop(I32)
                self.st.append(I64)
            else:
                raise AssertionError(f"unknown op {op:#x} at +{self.pos - 1}")
        assert not self.ctrl, "unclosed block"
        assert self.st == self.ret, f"end stack {self.st} != ret {self.ret}"


CALLSIGS = {
    "spike_win.wasm": {1: (0, [])},
    "spike_call.wasm": {1: (0, [])},
    "spike_mmio.wasm": {0: (2, []), 1: (1, ["i32"])},
    "spike_poll.wasm": {0: (0, ["i32"]), 1: (1, [])},
    "spike_auto.wasm": {0: (1, []), 1: (1, ["i32"]), 2: (2, [])},
    "spike_s4bridge.wasm": {0: (1, []), 1: (1, ["i32"]), 2: (2, []), 3: (0, ["i32"])},
}


def check(path, funcs):
    """funcs: list of (code_bytes, nlocals, nglobals, ret_types)."""
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
    assert len(bodies) == len(funcs), f"{len(bodies)} bodies vs {len(funcs)} specs"
    for i, ((code, nloc), (nglob, ret)) in enumerate(zip(bodies, funcs)):
        Sim(code, nloc, nglob, ret, CALLSIGS.get(name)).run()
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
        "spike_auto.wasm": [(3, [])],
        "spike_s4bridge.wasm": [(0, [I64])],
        "spike_diff.wasm": [(0, [])],
    }
    name = path.rsplit("/", 1)[-1]
    check(path, specs[name])
    print("SIM ALL OK")
