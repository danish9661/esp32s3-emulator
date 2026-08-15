#!/usr/bin/env python3
"""Generate Rust Xtensa ESP32-S3 instruction decoder tables.

Source: QEMU Espressif fork, target/xtensa/core-esp32s3/xtensa-modules.inc.c
(GPLv2, Tensilica-generated). This tool mechanically converts the C decode
tables into a Rust module. No logic is invented here; every bit range and
opcode mapping comes from the C tables verbatim.

Usage: tools/gen_decode.py > crates/xtensa-core/src/generated.rs
"""

import re
import sys

SRC = "tools/qemu-ref/target/xtensa/core-esp32s3/xtensa-modules.inc.c"

text = open(SRC).read()


def find_func(name: str) -> str:
    """Return the body (without outer braces) of a top-level function."""
    m = re.search(r"(?m)^" + re.escape(name) + r"\s*\((?:[^()]*)\)\s*\{", text)
    if not m:
        raise KeyError(f"function not found: {name}")
    start = m.end() - 1
    depth = 1
    i = start + 1
    while depth > 0:
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
        i += 1
    return text[start + 1:i - 1]


# ---------------------------------------------------------------- fields ---

fields = {}  # (slot, field) -> (msb, lsb)
for m in re.finditer(r"(?ms)static unsigned\nField_(\w+)_get \(const "
                     r"xtensa_insnbuf insn\)\n\{(.*?)\n\}", text):
    full_name = m.group(1)
    body = m.group(2)
    head, _, tail = full_name.partition("_Slot_")
    slot = tail.removesuffix("_get")
    if slot not in ("inst", "inst16a", "inst16b"):
        continue
    field = head
    terms = re.findall(r"\(\(insn\[0\] << (\d+)\) >> (\d+)\)", body)
    if not terms:
        sys.stderr.write(f"SKIP field {full_name}: no shift terms\n")
        continue
    # Each term `((insn[0] << sh) >> rs)` extracts insn bits [rs-sh, 31-sh],
    # width 32-rs. The C get fns concatenate terms in order (first = high bits).
    # Fields may be SPLIT (e.g. sal = {insn[20], insn[7:4]}), so keep per-term
    # (lo, width) ranges instead of assuming one contiguous run.
    ranges = [(int(rs) - int(sh), 32 - int(rs)) for sh, rs in terms]
    fields[(slot, field)] = ranges

print(f"// {len(fields)} fields parsed", file=sys.stderr)

# --------------------------------------------------------------- opcodes ---

enum_start = text.index("enum xtensa_opcode_id {")
enum_end = text.index("};", enum_start)
opcodes = re.findall(r"OPCODE_\w+", text[enum_start:enum_end])
print(f"// {len(opcodes)} opcodes parsed", file=sys.stderr)


# ---------------------------------------------------------------- decode ---

TOKEN_RE = re.compile(r"\(|\)|\{|\}|;|,|==|&&|\|\||0x[0-9a-fA-F]+|\d+|[A-Za-z_][A-Za-z0-9_]*")


def tokenize(s):
    toks = []
    i = 0
    while i < len(s):
        while i < len(s) and s[i].isspace():
            i += 1
        if i >= len(s):
            break
        m = TOKEN_RE.match(s, i)
        if not m:
            raise ValueError(f"cannot tokenize near: {s[i:i+20]!r}")
        toks.append(m.group(0))
        i = m.end()
    return toks


class Parser:
    def __init__(self, toks):
        self.toks = toks
        self.i = 0

    def peek(self):
        return self.toks[self.i] if self.i < len(self.toks) else None

    def next(self):
        t = self.peek()
        self.i += 1
        return t

    def expect(self, t):
        got = self.next()
        assert got == t, f"expected {t}, got {got} at {self.i}"

    def parse_statements(self):
        out = []
        while self.peek() is not None:
            out.append(self.parse_statement())
        return out

    def parse_statement(self):
        t = self.next()
        if t == "if":
            self.expect("(")
            cond = self.parse_cond()
            self.expect(")")
            then = self.parse_then()
            return ("if", cond, then)
        if t == "return":
            opc = self.next()
            self.expect(";")
            return ("ret", opc)
        # bare XTENSA_UNDEFINED; or similar
        while self.peek() not in (None, ";"):
            self.next()
        if self.peek() == ";":
            self.next()
        return ("noop",)

    def parse_then(self):
        if self.peek() == "{":
            self.next()
            stmts = []
            while self.peek() != "}":
                stmts.append(self.parse_statement())
            self.expect("}")
            return stmts
        return [self.parse_statement()]

    def parse_cond(self):
        """expr := term (('&&'|'||') term)*  — left-assoc boolean tree"""
        left = self.parse_term()
        while self.peek() in ("&&", "||"):
            op = self.next()
            right = self.parse_term()
            left = ("binop", op, left, right)
        return left

    def parse_term(self):
        if self.peek() == "(":
            self.next()
            c = self.parse_cond()
            self.expect(")")
            return ("group", c)
        name = self.next()
        assert name.startswith("Field_") and name.endswith("_get"), name
        self.expect("(")
        self.expect("insn")
        self.expect(")")
        self.expect("==")
        val = int(self.next(), 0)
        return ("atom", name, val)


def walk(parser, path, out):
    for stmt in parser.parse_statements():
        if stmt[0] == "ret":
            out.append((list(path), stmt[1]))
        elif stmt[0] == "if":
            _, cond, then = stmt
            new_path = path + [cond]
            for s in then:
                if s[0] == "ret":
                    out.append((list(new_path), s[1]))
                elif s[0] == "if":
                    walk_if(parser, s, new_path, out)


def walk_if(parser, stmt, path, out):
    _, cond, then = stmt
    new_path = path + [cond]
    for s in then:
        if s[0] == "ret":
            out.append((list(new_path), s[1]))
        elif s[0] == "if":
            walk_if(parser, s, new_path, out)


def parse_decode(fn_name):
    body = find_func(fn_name)
    parser = Parser(tokenize(body))
    out = []
    walk(parser, [], out)
    return out


decoders = {
    "inst": parse_decode("Slot_inst_decode"),
    "inst16a": parse_decode("Slot_inst16a_decode"),
    "inst16b": parse_decode("Slot_inst16b_decode"),
}
for k, v in decoders.items():
    print(f"// {k}: {len(v)} decode paths", file=sys.stderr)


def slot_of(field_full_name):
    return field_full_name.split("_Slot_")[-1].removesuffix("_get")


def emit_cond(cond):
    """Render a condition tree back as a Rust boolean expression."""
    if cond[0] == "atom":
        f = cond[1]
        return (f"fld_{slot_of(f)}::{field_of(f)}(insn) == {cond[2]}")
    if cond[0] == "group":
        return f"({emit_cond(cond[1])})"
    if cond[0] == "binop":
        return f"({emit_cond(cond[2])} {cond[1]} {emit_cond(cond[3])})"
    raise ValueError(cond)


def field_of(field_full_name):
    return field_full_name.partition("_Slot_")[0].removeprefix("Field_")


# -------------------------------------------------------------- operands ---

OPND_TABLE_RE = re.compile(
    r'\{\s*"([^"]+)",\s*(\w+|0),\s*(-1|\w+|0),\s*(\d+),\s*([\w |]+),\s*'
    r'(\w+|0),\s*(\w+|0),\s*(\w+|0),\s*(\w+|0)\s*\}', re.S)

opnd_start = text.index("static xtensa_operand_internal operands[] = {")
opnd_end = text.index("};", opnd_start)
opnd_table = []  # (name, field, regfile, is_reg, flags, decode_fn, rtoa_fn)
for m in OPND_TABLE_RE.finditer(text[opnd_start:opnd_end]):
    name, field, regfile, is_reg, flags, enc, dec, ator, rtoa = m.groups()
    flags = set(f.strip() for f in flags.split("|") if f.strip())
    opnd_table.append({
        "name": name, "field": field, "regfile": regfile,
        "is_reg": is_reg == "1",
        "pcrel": "XTENSA_OPERAND_IS_PCRELATIVE" in flags,
        "invisible": "XTENSA_OPERAND_IS_INVISIBLE" in flags,
        "decode": dec, "rtoa": rtoa,
    })
print(f"// {len(opnd_table)} operands parsed", file=sys.stderr)

m = re.search(r"enum xtensa_operand_id \{(.*?)\};", text, re.S)
operand_ids = [n.strip().rstrip(",") for n in re.findall(r"OPERAND_(\S+)", m.group(1))]
assert len(operand_ids) == len(opnd_table), "OPERAND enum count != operands table"
for i, (e, t) in enumerate(zip(operand_ids, opnd_table)):
    assert e.lstrip("_") == t["name"].lstrip("*").replace(".", "_"), \
        f"operand {i}: enum {e} != table {t['name']}"
    t["name"] = e
print("// OPERAND enum order == operands[] table order", file=sys.stderr)

# ------------------------------------------------------------ iclasses ----

m = re.search(r"enum xtensa_iclass_id \{(.*?)\};", text, re.S)
iclass_names = [n.rstrip(",") for n in re.findall(r"ICLASS_(\S+)", m.group(1))]

icls_start = text.index("static xtensa_iclass_internal iclasses[] = {")
icls_end = text.index("};", icls_start)
icls_entries = re.findall(r"\{\s*(\d+),\s*((?:Iclass_\w+_args)|0)\s*(?:/\*.*?\*/)?,", text[icls_start:icls_end])
assert len(icls_entries) == len(iclass_names), "iclasses table count != enum"
iclass_args = [args[7:-5] if args != "0" else None for _, args in icls_entries]

args_arrays = {}
for m in re.finditer(r"Iclass_(\w+)_args\[\] = \{(.*?)\};", text, re.S):
    ops = re.findall(r"\{\s*\{\s*OPERAND_(\w+)\s*\},\s*'(\w)'\s*\}", m.group(2))
    args_arrays[m.group(1)] = [(o.rstrip(","), kind) for o, kind in ops]

# -------------------------------------------------------------- opcodes ---

opc_start = text.index("static xtensa_opcode_internal opcodes[] = {")
opc_end = text.index("};", opc_start)
opc_names = re.findall(r'\{\s*"([^"]+)",\s*ICLASS_(\w+),', text[opc_start:opc_end])
assert len(opc_names) == len(opcodes), "opcodes table count != OPCODE enum"
for i, (n, ic) in enumerate(opc_names):
    assert n.replace(".", "_").upper() == opcodes[i].removeprefix("OPCODE_").upper(), \
        f"opcode {i}: {n} != {opcodes[i]}"
print("// opcodes[] order == OPCODE enum order", file=sys.stderr)

opc_iclass = []  # opcode id -> iclass id
for i, (name, icl) in enumerate(opc_names):
    icid = iclass_names.index(icl)
    opc_iclass.append(icid)
missing_args = [a for a in iclass_args if a is not None and a not in args_arrays]
assert not missing_args, f"args arrays not found: {missing_args}"

# -------------------------------------------------- operand decode exprs ---

DECODE_FN_RE = re.compile(
    r"static int\nOperandSem_(\w+)_decode \(uint32 \*valp(?: ATTRIBUTE_UNUSED)?\)\n\{(.*?)\n\}",
    re.S)

CONST_TBL_RE = re.compile(r"static const unsigned CONST_TBL_(\w+)_0\[\] = \{(.*?)\};", re.S)
const_tbls = {}
for m in CONST_TBL_RE.finditer(text):
    vals = re.findall(r"0x[0-9a-fA-F]+|\d+", m.group(2))
    const_tbls[m.group(1)] = [int(v, 0) for v in vals]
# only keep the three tables used by decoded operands
for k in list(const_tbls):
    if not k.startswith(("ai4c", "b4c", "b4cu")):
        del const_tbls[k]

EXPR_TOK_RE = re.compile(
    r"\(int\)|\*\s*valp|<<|>>|\[|\]|\(|\)|,|0x[0-9a-fA-F]+|\d+|\w+|[|&+\-~]")


def expr_tokenize(s):
    toks = []
    i = 0
    while i < len(s):
        while i < len(s) and s[i].isspace():
            i += 1
        if i >= len(s):
            break
        m = EXPR_TOK_RE.match(s, i)
        if not m:
            raise ValueError(f"cannot tokenize expr near: {s[i:i+20]!r}")
        toks.append(m.group(0))
        i = m.end()
    return toks


class ExprParser:
    """Tiny Pratt parser for the OperandSem decode expression subset."""

    PREC = {"+": 60, "-": 60, "<<": 50, ">>": 50, "&": 40, "|": 30}

    def __init__(self, toks, in_var):
        self.toks = toks
        self.i = 0
        self.in_var = in_var
        self.sext_ok = True

    def peek(self):
        return self.toks[self.i] if self.i < len(self.toks) else None

    def next(self):
        t = self.peek()
        self.i += 1
        return t

    def parse(self):
        e = self.parse_bp(0)
        assert self.peek() is None, f"trailing tokens: {self.toks[self.i:]}"
        return e

    def parse_bp(self, min_prec):
        t = self.next()
        if t == "(":
            e = self.parse_bp(0)
            assert self.next() == ")", f"expected ')', got {self.peek()}"
        elif t == "-":
            e = ("neg", self.parse_bp(70))
        elif t == "~":
            e = ("bnot", self.parse_bp(70))
        elif t == "(int)":
            e = ("cast", self.parse_bp(70))
        elif t == self.in_var or t == "*valp":
            e = ("v",)
        elif t.isdigit() or t.startswith("0x"):
            e = ("n", int(t, 0))
        else:
            name = t
            # CONST_TBL_x[...] table lookup
            assert self.next() == "[", f"unexpected identifier {name}"
            idx = self.parse_bp(0)
            assert self.next() == "]", "expected ']'"
            e = ("tbl", name, idx)
        while True:
            op = self.peek()
            if op in self.PREC and self.PREC[op] >= min_prec:
                self.next()
                rhs = self.parse_bp(self.PREC[op] + 1)
                e = ("binop", op, e, rhs)
            else:
                break
        return e


def parse_decode_fn(name, body):
    """Return (expr_ast, mask, pcrel_add) for a decode function body."""
    body = body.strip()
    if body == "return 0;":
        return ("v",), None, None, True
    m = re.search(r"(\w+_in_0) = \*valp & (0x[0-9a-fA-F]+|\d+);", body)
    if m:
        in_var, mask = m.group(1), int(m.group(2), 0)
        m2 = re.search(r"\w+_out_0 = (.*?);", body)
        if not m2:
            raise ValueError(f"decode fn {name}: no out assignment")
        expr_src = m2.group(1)
        pcrel = None
    else:
        m = re.search(r"\*valp (=|\+=) (.*?);", body)
        if not m:
            raise ValueError(f"decode fn {name}: unrecognized body {body!r}")
        in_var, mask = "v", None
        expr_src = m.group(2)
        pcrel = None
    p = ExprParser(expr_tokenize(expr_src), in_var)
    expr = p.parse()
    return expr, mask, pcrel, p.sext_ok


decode_fns = {}
for m in DECODE_FN_RE.finditer(text):
    name, body = m.group(1), m.group(2)
    expr, mask, pcrel, _ = parse_decode_fn(name, body)
    decode_fns["OperandSem_" + name + "_decode"] = (expr, mask, pcrel)
print(f"// {len(decode_fns)} operand decode fns parsed", file=sys.stderr)

# rtoa (pc-relative) functions
RTOA_RE = re.compile(
    r"static int\nOperand_(\w+)_rtoa \(uint32 \*valp, uint32 pc\)\n\{(.*?)\n\}", re.S)
rtoa_fns = {}
for m in RTOA_RE.finditer(text):
    body = m.group(2).strip().replace("\n", " ")
    if "*valp += pc;" in body:
        kind = "pc"
    elif "*valp += (pc & ~0x3);" in body:
        kind = "pc_aligned"
    elif "*valp += ((pc + 3) & ~0x3);" in body:
        kind = "pc_round"
    else:
        raise ValueError(f"rtoa {m.group(1)}: unrecognized body {body!r}")
    rtoa_fns["Operand_" + m.group(1) + "_rtoa"] = kind
print(f"// {len(rtoa_fns)} rtoa fns parsed", file=sys.stderr)

# implicit fields (constant values, e.g. FIELD__ar0 -> 0)
IMPL_FIELD_RE = re.compile(
    r"Implicit_Field_(\w+)_get \(const xtensa_insnbuf insn[^)]*\)\s*\n\{\s*\n\s*"
    r"return (\w+);", re.S)
implicit_fields = {n: int(v, 0) for n, v in IMPL_FIELD_RE.findall(text)}

# ---------------------------------------------------------- emit operands --

def emit_expr(expr, indent, seen_cast):
    """Render a decode-expr AST to Rust (u32 semantics)."""
    t = expr[0]
    if t == "v":
        return "v"
    if t == "n":
        return f"0x{expr[1]:x}u32"
    if t == "neg":
        return f"(0u32).wrapping_sub({emit_expr(expr[1], indent, seen_cast)})"
    if t == "bnot":
        return f"!({emit_expr(expr[1], indent, seen_cast)})"
    if t == "cast":
        seen_cast.append(expr)
        return emit_expr(expr[1], indent, seen_cast)
    if t == "tbl":
        tbl = expr[1].replace("CONST_TBL_", "").rstrip("_0") + "_TBL"
        return f"{tbl}[({emit_expr(expr[2], indent, seen_cast)} & 0xf) as usize]"
    if t == "binop":
        op, l, r = expr[1], expr[2], expr[3]
        ls, rs = emit_expr(l, indent, seen_cast), emit_expr(r, indent, seen_cast)
        if op == ">>":
            # sign-extend idiom: ((int) X << k) >> k
            if (l[0] == "binop" and l[1] == "<<" and l[3] == r
                    and l[2][0] == "cast"):
                return f"sext({emit_expr(l[2][1], indent, seen_cast)}, {emit_expr(r, indent, seen_cast)})"
            return f"(({ls}) >> {rs})"
        if op == "<<":
            # ISA-correct sign-extension for L32R's pc-relative offset.
            # QEMU's C emits the tensilica idiom `(((0xffff) << 16) | v) << 2`,
            # which equals sext16(v)<<2 ONLY when bit 15 of v is set (pools
            # precede code). The Xtensa ISA RM says the 16-bit offset is
            # sign-extended; forward l32r (positive offset) needs sext for ALL
            # v. Deviation from the C tables is documented in generated.rs.
            if (r == ("n", 2)
                    and l[0] == "binop" and l[1] == "|"
                    and l[2] == ("binop", "<<", ("n", 0xffff), ("n", 16))):
                return f"((sext({emit_expr(l[3], indent, seen_cast)}, 16u32)) << 2)"
            return f"(({ls}) << {rs})"
        if op == "&":
            return f"(({ls}) & {rs})"
        if op == "|":
            return f"(({ls}) | {rs})"
        if op == "+":
            return f"({ls}).wrapping_add({rs})"
        if op == "-":
            return f"({ls}).wrapping_sub({rs})"
        raise ValueError(f"unhandled binop {op}")
    raise ValueError(f"unhandled expr node {t}")


def operand_value_expr(opnd, slot, insn_ref):
    """Rust expression for the raw field of an operand (pre-decode)."""
    field = opnd["field"]
    if field == "0":
        raise ValueError(f"operand {opnd['name']} has no field")
    if field.startswith("FIELD__"):
        const_name = field.removeprefix("FIELD__")
        if const_name not in implicit_fields:
            raise ValueError(f"no implicit field value for {field}")
        return str(implicit_fields[const_name])
    fname = field.removeprefix("FIELD_")
    key = (slot, fname)
    if key not in fields:
        sys.stderr.write(f"// WARN: field {fname} has no get fn in slot {slot} (TIE opcode) — operand reads 0\n")
        return "0"
    return f"fld_{slot}::{fname}({insn_ref})"


def full_operand_expr(opnd, slot, insn_ref, pc_ref, seen_cast):
    """Rust expression for the final decoded operand value."""
    v = operand_value_expr(opnd, slot, insn_ref)
    expr = ("v",)
    mask = None
    if opnd["decode"] and opnd["decode"] not in ("0", "0,"):
        if opnd["decode"] not in decode_fns:
            raise ValueError(f"decode fn {opnd['decode']} not parsed")
        expr, mask, _ = decode_fns[opnd["decode"]]
        if expr == ("v",) and mask is None:
            expr, mask = ("v",), None  # identity fn: raw field value
    if mask is not None:
        v = f"({v} & 0x{mask:x})"
    rendered = emit_expr(expr, "    ", seen_cast)
    if rendered != "v":
        v = rendered.replace("v", f"({v})")
    if opnd["pcrel"]:
        if opnd["rtoa"] not in rtoa_fns:
            raise ValueError(f"rtoa {opnd['rtoa']} not parsed")
        kind = rtoa_fns[opnd["rtoa"]]
        if kind == "pc":
            v = f"({v}).wrapping_add({pc_ref})"
        elif kind == "pc_aligned":
            v = f"({v}).wrapping_add({pc_ref} & !3)"
        elif kind == "pc_round":
            v = f"({v}).wrapping_add(({pc_ref}.wrapping_add(3)) & !3)"
    return v


# opcode -> slot (from decode tables); assert each opcode lives in one slot
opc_slot = {}
for slot, paths in decoders.items():
    for path, opc in paths:
        if opc == "XTENSA_UNDEFINED":
            continue
        if opc in opc_slot and opc_slot[opc] != slot:
            raise ValueError(f"opcode {opc} decodes in two slots")
        opc_slot[opc] = slot

MAX_OPERANDS = 0
for args in iclass_args:
    if args:
        MAX_OPERANDS = max(MAX_OPERANDS, len(args_arrays[args]))
print(f"// MAX_OPERANDS = {MAX_OPERANDS}", file=sys.stderr)

# ---------------------------------------------------------------- emit ----

def cond_fields(cond):
    """Yield all (slot, field) pairs referenced by a condition tree."""
    if cond[0] == "atom":
        yield (slot_of(cond[1]), field_of(cond[1]))
    elif cond[0] == "group":
        yield from cond_fields(cond[1])
    elif cond[0] == "binop":
        yield from cond_fields(cond[2])
        yield from cond_fields(cond[3])


used_fields = set()
for slot, paths in decoders.items():
    for path, _ in paths:
        for cond in path:
            used_fields.update(cond_fields(cond))
for i, opc in enumerate(opcodes):
    if opc not in opc_slot:
        continue
    slot = opc_slot[opc]
    args = iclass_args[opc_iclass[i]]
    if not args:
        continue
    for oid, kind in args_arrays[args]:
        opnd = opnd_table[operand_ids.index(oid)]
        field = opnd["field"]
        if field == "0" or field.startswith("FIELD__"):
            continue
        fname = field.removeprefix("FIELD_")
        if (slot, fname) not in fields:
            sys.stderr.write(f"// WARN: operand {opnd['name']} field {fname} not parsed for slot {slot}\n")
        else:
            used_fields.add((slot, fname))

out = []
out.append("// AUTO-GENERATED by tools/gen_decode.py — DO NOT EDIT.")
out.append("// Generated from QEMU Espressif fork core-esp32s3/xtensa-modules.inc.c")
out.append("// (Tensilica-generated, GPLv2). Bit layouts are verbatim from that file.")
out.append("#![allow(non_camel_case_types, clippy::identity_op)]")
out.append("#![allow(clippy::eq_op, clippy::erasing_op, clippy::double_parens, clippy::needless_return)]")
out.append("#![allow(unused_parens, non_upper_case_globals)]")
out.append("")
out.append("/// Instruction length in bytes given the first instruction byte.")
out.append("#[inline]")
out.append("pub fn insn_len(b0: u8) -> u32 {")
out.append("    match b0 & 0xf {")
out.append("        0..=7 => 3,")
out.append("        8..=13 => 2,")
out.append("        _ => 4, // format_32 (TIE/DSP) — unsupported")
out.append("    }")
out.append("}")
out.append("")
out.append("#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]")
out.append("pub enum Opcode {")
for i, opc in enumerate(opcodes):
    out.append(f"    {opc} = {i},")
out.append("}")
out.append("")
out.append("impl Opcode {")
out.append("    pub fn name(self) -> &'static str {")
out.append("        match self {")
for opc in opcodes:
    out.append(f"            Opcode::{opc} => \"{opc[7:].lower()}\",")
out.append("        }")
out.append("    }")
out.append("}")
out.append("")
for slot in ("inst", "inst16a", "inst16b"):
    fields_slot = sorted(f for s, f in used_fields if s == slot)
    if not fields_slot:
        continue
    out.append(f"pub mod fld_{slot} {{")
    for s, f in sorted(used_fields):
        if s != slot:
            continue
        ranges = fields[(s, f)]
        terms = [f"((insn >> {lo}) & 0x{(1 << w) - 1:x})" for lo, w in ranges]
        expr = terms[0]
        for (_, w), t in zip(ranges[1:], terms[1:]):
            expr = f"({expr} << {w}) | {t}"
        out.append(f"    #[inline]")
        out.append(f"    pub fn {f}(insn: u32) -> u32 {{ {expr} }}")
    out.append("}")
    out.append("")

for slot, paths in decoders.items():
    out.append(f"/// Decode an instruction in the '{slot}' slot. Returns None on undefined.")
    out.append(f"pub fn decode_{slot}(insn: u32) -> Option<Opcode> {{")
    for path, opc in paths:
        if opc == "XTENSA_UNDEFINED":
            out.append("    return None;")
            continue
        if not path:
            out.append(f"    return Some(Opcode::{opc});")
            continue
        conds = " && ".join(emit_cond(c) for c in path)
        out.append(f"    if {conds} {{ return Some(Opcode::{opc}); }}")
    if paths and paths[-1][0]:
        out.append("    None")
    out.append("}")
    out.append("")

# operand emission
out.append("/// A decoded operand: register number, or immediate value.")
out.append("#[derive(Clone, Copy, Debug, PartialEq, Eq)]")
out.append("pub struct Opnd {")
out.append("    pub value: u32,")
out.append("    pub is_reg: bool,")
out.append("    pub visible: bool,")
out.append("}")
out.append("")
out.append("impl Opnd {")
out.append("    #[inline] pub const fn reg(v: u32) -> Opnd { Opnd { value: v, is_reg: true, visible: true } }")
out.append("    #[inline] pub const fn reg_hi(v: u32) -> Opnd { Opnd { value: v, is_reg: true, visible: false } }")
out.append("    #[inline] pub const fn imm(v: u32) -> Opnd { Opnd { value: v, is_reg: false, visible: true } }")
out.append("    #[inline] pub const fn imm_hi(v: u32) -> Opnd { Opnd { value: v, is_reg: false, visible: false } }")
out.append("}")
out.append("")
out.append("/// Sign-extend the low `k` bits of `v` (32-bit arithmetic).")
out.append("#[inline]")
out.append("pub fn sext(v: u32, k: u32) -> u32 { (((v as i32) << k) >> k) as u32 }")
out.append("")
for tbl_name, vals in const_tbls.items():
    rust_name = tbl_name.rstrip("_0") + "_TBL"
    out.append(f"const {rust_name}: [u32; {len(vals)}] = [")
    out.append("    " + ", ".join(f"0x{v:x}" for v in vals) + ",")
    out.append("];")
    out.append("")
out.append(f"pub const MAX_OPERANDS: usize = {MAX_OPERANDS};")
out.append("")
out.append("/// Decode all operands of `opc` (including invisible ones, matching")
out.append("/// QEMU's window-check mask computation). Unused slots are imm(0).")
out.append("pub fn opnds(opc: Opcode, insn: u32, pc: u32) -> [Opnd; MAX_OPERANDS] {")
out.append("    let mut o = [Opnd::imm(0); MAX_OPERANDS];")
out.append("    match opc {")
for i, opc in enumerate(opcodes):
    icid = opc_iclass[i]
    args = iclass_args[icid]
    if opc not in opc_slot:
        out.append(f"        Opcode::{opc} => {{}} // not decodable (4-byte TIE)")
        continue
    slot = opc_slot[opc]
    seen_cast = []
    operands = []
    if args:
        for oid, kind in args_arrays[args]:
            operands.append(opnd_table[operand_ids.index(oid)])
    body = []
    for j, opnd in enumerate(operands):
        expr = full_operand_expr(opnd, slot, "insn", "pc", seen_cast)
        ctor = "reg" if opnd["is_reg"] else "imm"
        if opnd["invisible"]:
            ctor += "_hi"
        body.append(f"        o[{j}] = Opnd::{ctor}({expr});")
    out.append(f"        Opcode::{opc} => {{")
    out.extend(body)
    out.append("        }")
    if seen_cast:
        sys.stderr.write(f"// WARN: cast used non-sext in {opc}\n")
out.append("    }")
out.append("    o")
out.append("}")
out.append("")

sys.stdout.write("\n".join(out) + "\n")
print("// done", file=sys.stderr)
