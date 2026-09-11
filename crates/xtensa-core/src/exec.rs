//! Instruction execution for the Xtensa LX7 core.
//!
//! Every case below mirrors the QEMU espressif/qemu `target/xtensa`
//! translate.c implementation for the corresponding opcode (GPLv2).  The
//! generated decoder produces `opnds` with immediate values already
//! pc-adjusted (branch targets, L32R addresses) and register operands
//! including invisible ones (e.g. CALLW's `reg_hi(callinc * 4)`, which the
//! generic window-overflow check in `Cpu::step` uses).
//!
//! Instruction formats are cited per the Xtensa ISA Reference Manual (e.g.
//! RRR op0=0 op1=0x8 m=0 => ADD).

use crate::bus::Bus;
use crate::cpu::{
    ALLOCA_CAUSE, Cpu, ILLEGAL_INSTRUCTION_CAUSE, PS_CALLINC, PS_CALLINC_SHIFT, PS_EXCM, PS_OWB,
    PS_WOE, SR_ACCHI, SR_ACCLO, SR_BR, SR_EPC1, SR_EPS2, SR_INTSET, SR_LBEG, SR_LCOUNT, SR_LEND,
    SR_M0, SR_PS, SR_SAR, SR_SCOMPARE1, SR_WINDOW_BASE, SR_WINDOW_START, SYSCALL_CAUSE, UR_FCR,
    UR_FSR, UR_THREADPTR,
};
use crate::generated::{Opcode, Opnd};

pub(crate) enum Outcome {
    /// Sequential continuation (loop-end check applies).
    Seq,
    /// Taken control-flow transfer to the given address (no loop check).
    Jump(u32),
    /// An exception was raised by the instruction (state already set).
    Exception(u32),
    Unimplemented,
}

fn sext16(v: u32) -> u32 {
    ((v as i16) as i32) as u32
}

/// Sign-extend the low `k` bits of `v` (k in 1..=31).
fn sext(v: u32, k: u32) -> u32 {
    let shift = 32 - k;
    ((v.wrapping_shl(shift) as i32) >> shift) as u32
}

fn clrsb(v: u32) -> u32 {
    // QEMU translate_nsa uses tcg_gen_clrsb (clrsb32 in
    // include/qemu/host-utils.h: clz(val ^ ((int32)val >> 1)) - 1, but the
    // canonical __builtin_clrsb semantics are used on GCC builds).  Count
    // of leading bits equal to the sign bit, excluding the sign bit itself.
    let t = if (v >> 31) & 1 == 1 { !v } else { v };
    t.leading_zeros() - 1
}

/// MAC16 half-select (ISA RM "MAC16 Option"; QEMU `gen_mac16_m` in
/// target/xtensa/translate.c): high half is bits [31:16] (arithmetic shift
/// down for signed, logical for unsigned), low half is bits [15:0]
/// (sign- or zero-extended).
#[inline]
fn mac16_half(v: u32, hi: bool, unsigned: bool) -> i32 {
    if hi {
        if unsigned {
            (v >> 16) as i32
        } else {
            (v as i32) >> 16
        }
    } else if unsigned {
        (v & 0xFFFF) as i32
    } else {
        ((v & 0xFFFF) as u16) as i16 as i32
    }
}

/// MUL/UMUL: overwrite the 40-bit ACC with the 16x16 product (QEMU sets
/// ACCLO to the product and ACCHI to its sign extension, or 0 for UMUL).
fn mac16_set(cpu: &mut Cpu, m1: i32, m2: i32, unsigned: bool) {
    let p = (m1 as i64).wrapping_mul(m2 as i64);
    cpu.set_sreg(SR_ACCLO, p as u32);
    cpu.set_sreg(SR_ACCHI, if unsigned { 0 } else { (p >> 32) as u32 });
}

/// MULA/MULS: ACC += (or -=) the sign-extended product, then truncate
/// ACCHI to its low 8 bits sign-extended (QEMU `ext8s`: the ACC is 40-bit,
/// ACCHI[7:0]:ACCLO[31:0]).
fn mac16_acc(cpu: &mut Cpu, m1: i32, m2: i32, add: bool) {
    let p = (m1 as i64).wrapping_mul(m2 as i64);
    let hi = cpu.sreg(SR_ACCHI) as i64;
    let lo = cpu.sreg(SR_ACCLO) as u64;
    let mut acc = (hi << 32) | lo as i64;
    if add {
        acc = acc.wrapping_add(p);
    } else {
        acc = acc.wrapping_sub(p);
    }
    cpu.set_sreg(SR_ACCLO, acc as u32);
    cpu.set_sreg(SR_ACCHI, (((acc >> 32) as u8) as i8) as i32 as u32);
}

pub(crate) fn execute<B: Bus>(
    cpu: &mut Cpu,
    bus: &mut B,
    opc: Opcode,
    o: &[Opnd; 8],
    len: u32,
) -> Outcome {
    macro_rules! rrr3 {
        ($b:expr) => {{
            // RRR op0=0: r = s OP t
            let s = cpu.reg(o[1].value);
            let t = cpu.reg(o[2].value);
            cpu.set_reg(o[0].value, $b(s, t));
            Outcome::Seq
        }};
    }

    match opc {
        // ------------------------------------------------------------------
        // ALU (RRR op0=0, op1=0x0..0x2 / 0x8..0xB, ISA RM "RRR" format)
        // ------------------------------------------------------------------
        Opcode::OPCODE_ADD => rrr3!(|s: u32, t: u32| s.wrapping_add(t)),
        Opcode::OPCODE_SUB => rrr3!(|s: u32, t: u32| s.wrapping_sub(t)),
        Opcode::OPCODE_ADDX2 => rrr3!(|s: u32, t: u32| (s << 1).wrapping_add(t)),
        Opcode::OPCODE_ADDX4 => rrr3!(|s: u32, t: u32| (s << 2).wrapping_add(t)),
        Opcode::OPCODE_ADDX8 => rrr3!(|s: u32, t: u32| (s << 3).wrapping_add(t)),
        Opcode::OPCODE_SUBX2 => rrr3!(|s: u32, t: u32| (s << 1).wrapping_sub(t)),
        Opcode::OPCODE_SUBX4 => rrr3!(|s: u32, t: u32| (s << 2).wrapping_sub(t)),
        Opcode::OPCODE_SUBX8 => rrr3!(|s: u32, t: u32| (s << 3).wrapping_sub(t)),
        Opcode::OPCODE_AND | Opcode::OPCODE_ANDB => rrr3!(|s: u32, t: u32| s & t),
        Opcode::OPCODE_OR | Opcode::OPCODE_ORB => rrr3!(|s: u32, t: u32| s | t),
        Opcode::OPCODE_XOR | Opcode::OPCODE_XORB => rrr3!(|s: u32, t: u32| s ^ t),
        Opcode::OPCODE_ANDBC => rrr3!(|s: u32, t: u32| s & !t),
        Opcode::OPCODE_ORBC => rrr3!(|s: u32, t: u32| s | !t),
        Opcode::OPCODE_MULL => rrr3!(|s: u32, t: u32| s.wrapping_mul(t)),
        Opcode::OPCODE_MULUH => rrr3!(|s: u32, t: u32| (((s as u64) * (t as u64)) >> 32) as u32),
        Opcode::OPCODE_MULSH => {
            rrr3!(|s: u32, t: u32| { ((((s as i64) * (t as i64)) >> 32) as i32) as u32 })
        }
        Opcode::OPCODE_NEG => rrr3!(|s: u32, _t: u32| s.wrapping_neg()),
        Opcode::OPCODE_ABS => rrr3!(|s: u32, _t: u32| {
            // QEMU translate_abs: max(-s, s); 0x80000000 maps to itself.
            let neg = s.wrapping_neg();
            (neg as i32).max(s as i32) as u32
        }),
        Opcode::OPCODE_MIN => rrr3!(|s: u32, t: u32| (s as i32).min(t as i32) as u32),
        Opcode::OPCODE_MAX => rrr3!(|s: u32, t: u32| (s as i32).max(t as i32) as u32),
        Opcode::OPCODE_MINU => rrr3!(|s: u32, t: u32| s.min(t)),
        Opcode::OPCODE_MAXU => rrr3!(|s: u32, t: u32| s.max(t)),
        Opcode::OPCODE_MUL16S => {
            rrr3!(|s: u32, t: u32| ((s as i16 as i32).wrapping_mul(t as i16 as i32)) as u32)
        }
        Opcode::OPCODE_MUL16U => rrr3!(|s: u32, t: u32| (s & 0xffff).wrapping_mul(t & 0xffff)),
        Opcode::OPCODE_QUOS => rrr3!(|s: u32, t: u32| {
            // QEMU translate_quos special case: 0x80000000 / -1 = 0x80000000.
            // The division is SIGNED (the old u32 `s / t` returned garbage
            // for negative operands — newlib printf's %d via div()/QUOS
            // would print wrong values).
            if s == 0x8000_0000 && t == 0xffff_ffff {
                0x8000_0000
            } else {
                // Division by zero: undefined on real silicon; saturate.
                ((s as i32).checked_div(t as i32)).unwrap_or(i32::MAX) as u32
            }
        }),
        Opcode::OPCODE_QUOU => rrr3!(|s: u32, t: u32| s.checked_div(t).unwrap_or(u32::MAX)),
        Opcode::OPCODE_REMS => rrr3!(|s: u32, t: u32| {
            if s == 0x8000_0000 && t == 0xffff_ffff {
                0
            } else {
                // Division by zero: undefined on real silicon; return s.
                // Signed remainder (see QUOS note).
                ((s as i32).checked_rem(t as i32)).unwrap_or(s as i32) as u32
            }
        }),
        Opcode::OPCODE_REMU => rrr3!(|s: u32, t: u32| s.checked_rem(t).unwrap_or(s)),
        Opcode::OPCODE_SALT => rrr3!(|s: u32, t: u32| if (s as i32) < (t as i32) { 1 } else { 0 }),
        Opcode::OPCODE_SALTU => rrr3!(|s: u32, t: u32| if s < t { 1 } else { 0 }),

        // Conditional moves (RRR op0=0, op1=0xC, ISA RM "MOV" variants;
        // QEMU translate_movcond / translate_movp).
        Opcode::OPCODE_MOVEQZ => {
            // moveqz at, as, t: if t == 0 then at = as
            if cpu.reg(o[2].value) == 0 {
                cpu.set_reg(o[0].value, cpu.reg(o[1].value));
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVNEZ => {
            if cpu.reg(o[2].value) != 0 {
                cpu.set_reg(o[0].value, cpu.reg(o[1].value));
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVLTZ => {
            if (cpu.reg(o[2].value) as i32) < 0 {
                cpu.set_reg(o[0].value, cpu.reg(o[1].value));
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVGEZ => {
            if (cpu.reg(o[2].value) as i32) >= 0 {
                cpu.set_reg(o[0].value, cpu.reg(o[1].value));
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVF | Opcode::OPCODE_MOVT => {
            // movf/movt at, as, bt: move if BR[bt] is clear/set (ISA RM
            // "MOVF/MOVT"; QEMU translate_movp tests the FP boolean bit,
            // not an AR bit).
            let cond = if opc == Opcode::OPCODE_MOVF {
                !cpu.br(o[2].value)
            } else {
                cpu.br(o[2].value)
            };
            if cond {
                cpu.set_reg(o[0].value, cpu.reg(o[1].value));
            }
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // Shifts (ISA RM "Shift Instructions"; SAR set by SSR/SSL/SSAI).
        // QEMU: SLL shifts by (32 - SAR) & 31; SRL/SRA by SAR & 31; SRC is
        // a 64-bit right shift of {s, t} by SAR.
        // ------------------------------------------------------------------
        Opcode::OPCODE_SLL => {
            // QEMU translate_sll: shift left by (32 - SAR) & 0x3f — the SAR
            // holds the RIGHT-shift amount; SSL pre-computes 32 - as.  A
            // shift of >= 32 yields 0 (TCG semantics).
            let sh = 32u32.wrapping_sub(cpu.sreg(SR_SAR)) & 0x3f;
            cpu.set_reg(o[0].value, ((cpu.reg(o[1].value) as u64) << sh) as u32);
            Outcome::Seq
        }
        Opcode::OPCODE_SRL => {
            // QEMU translate_srl: shift right by SAR; >= 32 yields 0.
            let sh = cpu.sreg(SR_SAR) & 0x3f;
            cpu.set_reg(o[0].value, ((cpu.reg(o[1].value) as u64) >> sh) as u32);
            Outcome::Seq
        }
        Opcode::OPCODE_SRA => {
            // QEMU translate_sra: arithmetic shift right by SAR; >= 32
            // sign-fills (TCG sar semantics).
            let sh = cpu.sreg(SR_SAR) & 0x3f;
            cpu.set_reg(
                o[0].value,
                ((cpu.reg(o[1].value) as i32 as i64) >> sh) as u32,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_SRC => {
            // src r, s, t: r = ({s, t} >> SAR) low 32 bits.
            let sh = cpu.sreg(SR_SAR) & 63;
            let v = ((cpu.reg(o[1].value) as u64) << 32) | (cpu.reg(o[2].value) as u64);
            cpu.set_reg(o[0].value, ((v >> sh) & 0xffff_ffff) as u32);
            Outcome::Seq
        }
        Opcode::OPCODE_SLLI => {
            cpu.set_reg(o[0].value, cpu.reg(o[1].value) << (o[2].value & 31));
            Outcome::Seq
        }
        Opcode::OPCODE_SRLI => {
            cpu.set_reg(o[0].value, cpu.reg(o[1].value) >> (o[2].value & 31));
            Outcome::Seq
        }
        Opcode::OPCODE_SRAI => {
            cpu.set_reg(
                o[0].value,
                ((cpu.reg(o[1].value) as i32) >> (o[2].value & 31)) as u32,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_SSR => {
            // ssr as: SAR = as & 31 (right-shift count).
            cpu.set_sreg(SR_SAR, cpu.reg(o[0].value) & 31);
            Outcome::Seq
        }
        Opcode::OPCODE_SSL => {
            // ssl as: SAR = 32 - (as & 31) — the SAR can legitimately hold
            // 32 (count 0), and SLL/SRC interpret it as the right-shift
            // complement (QEMU gen_left_shift_sar).
            cpu.set_sreg(SR_SAR, 32u32.wrapping_sub(cpu.reg(o[0].value) & 31));
            Outcome::Seq
        }
        Opcode::OPCODE_SSAI => {
            // ssai imm: SAR = imm & 31.
            cpu.set_sreg(SR_SAR, o[0].value & 31);
            Outcome::Seq
        }
        Opcode::OPCODE_SSA8B => {
            // ssa8b as: SAR = 32 - ((as << 3) & 31) (left byte-align; can
            // hold 32 like SSL — QEMU gen_left_shift_sar).
            cpu.set_sreg(SR_SAR, 32u32.wrapping_sub((cpu.reg(o[0].value) << 3) & 31));
            Outcome::Seq
        }
        Opcode::OPCODE_SSA8L => {
            // ssa8l as: SAR = (as << 3) & 31 (right byte-align).
            cpu.set_sreg(SR_SAR, (cpu.reg(o[0].value) << 3) & 31);
            Outcome::Seq
        }
        Opcode::OPCODE_EXTUI => {
            // extui r, t, start, len (RRR op0=0 op1=0x0 m=1; operands carry
            // sae and len-1).
            let start = o[2].value & 31;
            let len = o[3].value & 31;
            let mask = if len >= 32 {
                u32::MAX
            } else {
                (1u32 << len) - 1
            };
            cpu.set_reg(o[0].value, (cpu.reg(o[1].value) >> start) & mask);
            Outcome::Seq
        }
        Opcode::OPCODE_SEXT => {
            // sext r, s, bt: sign-extend low (bt+1) bits (QEMU
            // translate_sext: sextract(out, in, 0, imm + 1)).
            let w = (o[2].value & 31) + 1;
            cpu.set_reg(o[0].value, sext(cpu.reg(o[1].value), w));
            Outcome::Seq
        }
        Opcode::OPCODE_CLAMPS => {
            // clamps r, s, bt: clamp to [-bt, bt-1] (QEMU translate_clamps).
            let b = o[2].value & 31;
            let lo = (u32::MAX << b) as i32;
            let hi = (1i32 << b) - 1;
            let v = cpu.reg(o[1].value) as i32;
            cpu.set_reg(o[0].value, v.clamp(lo, hi) as u32);
            Outcome::Seq
        }
        Opcode::OPCODE_NSA => {
            cpu.set_reg(o[0].value, clrsb(cpu.reg(o[1].value)));
            Outcome::Seq
        }
        Opcode::OPCODE_NSAU => {
            // nsau: count leading zeros (QEMU translate_nsau: clzi(..., 32)).
            cpu.set_reg(o[0].value, cpu.reg(o[1].value).leading_zeros());
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // Immediate ALU (RI format, op0=2; ISA RM "RI16/RI8").  ADDI t, s,
        // imm8 with t = dest ([7:4]).
        // ------------------------------------------------------------------
        Opcode::OPCODE_ADDI => {
            cpu.set_reg(o[0].value, cpu.reg(o[1].value).wrapping_add(o[2].value));
            Outcome::Seq
        }
        Opcode::OPCODE_ADDMI => {
            // addmi t, s, imm8: t = s + (sext8(imm8) << 8).  opnds already
            // pre-shifts the immediate, so add it as-is (no extra << 8).
            cpu.set_reg(o[0].value, cpu.reg(o[1].value).wrapping_add(o[2].value));
            Outcome::Seq
        }
        Opcode::OPCODE_MOVI => {
            cpu.set_reg(o[0].value, o[1].value);
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // Loads/stores (RI formats; ESP32-S3 performs unaligned accesses in
        // hardware: XCHAL_UNALIGNED_LOAD_HW=1, so no alignment checks).
        // ------------------------------------------------------------------
        Opcode::OPCODE_L8UI => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            cpu.set_reg(o[0].value, bus.read8(addr));
            Outcome::Seq
        }
        Opcode::OPCODE_L16UI => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            cpu.set_reg(o[0].value, bus.read16(addr));
            Outcome::Seq
        }
        Opcode::OPCODE_L16SI => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            cpu.set_reg(o[0].value, sext16(bus.read16(addr)));
            Outcome::Seq
        }
        Opcode::OPCODE_L32I | Opcode::OPCODE_L32AI => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            cpu.set_reg(o[0].value, bus.read32(addr));
            Outcome::Seq
        }
        Opcode::OPCODE_L32R => {
            // l32r t, label: load from the pc-relative address computed by
            // the decoder (ISA RM "L32R"; QEMU translate_l32r).
            cpu.set_reg(o[0].value, bus.read32(o[1].value));
            Outcome::Seq
        }
        Opcode::OPCODE_L32E => {
            // l32e t, s, disp (privileged, ring 0; same semantics as l32i).
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            cpu.set_reg(o[0].value, bus.read32(addr));
            Outcome::Seq
        }
        Opcode::OPCODE_S8I => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            bus.write8(addr, cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_S16I => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            bus.write16(addr, cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_S32I | Opcode::OPCODE_S32NB | Opcode::OPCODE_S32E | Opcode::OPCODE_S32RI => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            bus.write32(addr, cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_S32C1I => {
            // s32c1i t, s, imm: compare-and-swap with SCOMPARE1 (ISA RM
            // "Conditional Store"; QEMU translate_s32c1i uses an atomic
            // cmpxchg; single-threaded here).
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            let old = bus.read32(addr);
            if old == cpu.sreg(SR_SCOMPARE1) {
                bus.write32(addr, cpu.reg(o[0].value));
            }
            cpu.set_reg(o[0].value, old);
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // Branches (BI/B format; targets pc-adjusted by the decoder).
        // ------------------------------------------------------------------
        Opcode::OPCODE_BEQZ | Opcode::OPCODE_BEQZ_N => branch_if(o, 1, cpu.reg(o[0].value) == 0),
        Opcode::OPCODE_BNEZ | Opcode::OPCODE_BNEZ_N => branch_if(o, 1, cpu.reg(o[0].value) != 0),
        Opcode::OPCODE_BLTZ => branch_if(o, 1, (cpu.reg(o[0].value) as i32) < 0),
        Opcode::OPCODE_BGEZ => branch_if(o, 1, (cpu.reg(o[0].value) as i32) >= 0),
        Opcode::OPCODE_BEQI => branch_if(o, 2, cpu.reg(o[0].value) == o[1].value),
        Opcode::OPCODE_BNEI => branch_if(o, 2, cpu.reg(o[0].value) != o[1].value),
        Opcode::OPCODE_BLTI => branch_if(o, 2, (cpu.reg(o[0].value) as i32) < (o[1].value as i32)),
        Opcode::OPCODE_BGEI => branch_if(o, 2, (cpu.reg(o[0].value) as i32) >= (o[1].value as i32)),
        Opcode::OPCODE_BLTUI => branch_if(o, 2, cpu.reg(o[0].value) < o[1].value),
        Opcode::OPCODE_BGEUI => branch_if(o, 2, cpu.reg(o[0].value) >= o[1].value),
        Opcode::OPCODE_BEQ => branch_if(o, 2, cpu.reg(o[0].value) == cpu.reg(o[1].value)),
        Opcode::OPCODE_BNE => branch_if(o, 2, cpu.reg(o[0].value) != cpu.reg(o[1].value)),
        Opcode::OPCODE_BLT => branch_if(
            o,
            2,
            (cpu.reg(o[0].value) as i32) < (cpu.reg(o[1].value) as i32),
        ),
        Opcode::OPCODE_BGE => branch_if(
            o,
            2,
            (cpu.reg(o[0].value) as i32) >= (cpu.reg(o[1].value) as i32),
        ),
        Opcode::OPCODE_BLTU => branch_if(o, 2, cpu.reg(o[0].value) < cpu.reg(o[1].value)),
        Opcode::OPCODE_BGEU => branch_if(o, 2, cpu.reg(o[0].value) >= cpu.reg(o[1].value)),
        Opcode::OPCODE_BALL => branch_if(
            o,
            2,
            (cpu.reg(o[0].value) & cpu.reg(o[1].value)) == cpu.reg(o[1].value),
        ),
        Opcode::OPCODE_BNALL => branch_if(
            o,
            2,
            (cpu.reg(o[0].value) & cpu.reg(o[1].value)) != cpu.reg(o[1].value),
        ),
        Opcode::OPCODE_BANY => branch_if(o, 2, (cpu.reg(o[0].value) & cpu.reg(o[1].value)) != 0),
        Opcode::OPCODE_BNONE => branch_if(o, 2, (cpu.reg(o[0].value) & cpu.reg(o[1].value)) == 0),
        Opcode::OPCODE_BBC => {
            // bbc s, t, label: branch if bit (t & 31) of s is clear (QEMU
            // translate_bb).
            branch_if(
                o,
                2,
                (cpu.reg(o[0].value) >> (cpu.reg(o[1].value) & 31)) & 1 == 0,
            )
        }
        Opcode::OPCODE_BBS => branch_if(
            o,
            2,
            (cpu.reg(o[0].value) >> (cpu.reg(o[1].value) & 31)) & 1 == 1,
        ),
        Opcode::OPCODE_BBCI => branch_if(o, 2, (cpu.reg(o[0].value) >> (o[1].value & 31)) & 1 == 0),
        Opcode::OPCODE_BBSI => branch_if(o, 2, (cpu.reg(o[0].value) >> (o[1].value & 31)) & 1 == 1),
        Opcode::OPCODE_BF => branch_if(o, 1, !cpu.br(o[0].value)),
        Opcode::OPCODE_BT => branch_if(o, 1, cpu.br(o[0].value)),
        // ALL4/ANY4/ALL8/ANY8 (ISA RM Boolean Option; QEMU translate_all):
        // BR[t] <- AND/OR of 4 (resp. 8) consecutive BR bits starting at
        // the immediate. o[0] is the dest BR index (like BT's s), o[1] the
        // start bit (s4<<2 / s8<<3, verified against the assembler).
        Opcode::OPCODE_ALL4 | Opcode::OPCODE_ALL8 => {
            let width = if opc == Opcode::OPCODE_ALL4 { 4 } else { 8 };
            let start = o[1].value;
            let brw = cpu.sreg(SR_BR);
            let mut v = (brw >> start) & ((1 << width) - 1);
            v = u32::from(v == (1 << width) - 1);
            cpu.set_br(o[0].value, v != 0);
            Outcome::Seq
        }
        Opcode::OPCODE_ANY4 | Opcode::OPCODE_ANY8 => {
            let width = if opc == Opcode::OPCODE_ANY4 { 4 } else { 8 };
            let start = o[1].value;
            let brw = cpu.sreg(SR_BR);
            let v = (brw >> start) & ((1 << width) - 1);
            cpu.set_br(o[0].value, v != 0);
            Outcome::Seq
        }
        // LDDR32.P/SDDR32.P (debug doubleword moves through the DDR state;
        // single s operand, postupdated by the 8-byte access width): LDDR
        // loads DDR from mem64[AR[s]], SDDR stores DDR to mem64[AR[s]].
        Opcode::OPCODE_LDDR32_P => {
            let addr = cpu.reg(o[0].value);
            let lo = bus.read32(addr) as u64;
            let hi = bus.read32(addr.wrapping_add(4)) as u64;
            cpu.ddr = (hi << 32) | lo;
            cpu.set_reg(o[0].value, addr.wrapping_add(8));
            Outcome::Seq
        }
        Opcode::OPCODE_SDDR32_P => {
            let addr = cpu.reg(o[0].value);
            bus.write32(addr, cpu.ddr as u32);
            bus.write32(addr.wrapping_add(4), (cpu.ddr >> 32) as u32);
            cpu.set_reg(o[0].value, addr.wrapping_add(8));
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // Jumps and calls.
        // ------------------------------------------------------------------
        Opcode::OPCODE_J => Outcome::Jump(o[0].value),
        // JX target is byte-precise: NO low-2-bit masking (QEMU
        // translate_jx = gen_jump(dc, arg[0].in)).  Real compiled code
        // returns to odd addresses (e.g. call0 at pc ≡ 3 mod 4 → return
        // pc+3 ≡ 2 mod 4); masking would fetch mid-instruction.
        Opcode::OPCODE_JX => Outcome::Jump(cpu.reg(o[0].value)),
        Opcode::OPCODE_CALL0 => {
            // call0: a0 = pc_next; PS.CALLINC is NOT modified (QEMU
            // translate_call0 = gen_jumpi only — gen_callw_slot is used
            // solely by the windowed CALL4/8/12/CALLX4/8/12).  The
            // exception vectors rely on this: the level-1 vector does
            // `call0 _xt_user_exc`, and _xt_lowint1's `rsr.ps` must still
            // see the interrupted task's CALLINC to save it into the frame.
            cpu.set_reg(0, cpu.pc.wrapping_add(len));
            Outcome::Jump(o[0].value)
        }
        Opcode::OPCODE_CALL4 | Opcode::OPCODE_CALL8 | Opcode::OPCODE_CALL12 => {
            let callinc = o[1].value / 4;
            // Return address in a(callinc*4) with window count in [31:30].
            // QEMU gen_callw_slot: return addr = pc_next = pc + actual length.
            cpu.set_reg(
                callinc * 4,
                (callinc << 30) | (cpu.pc.wrapping_add(len) & 0x3fff_ffff),
            );
            cpu.set_sreg(
                SR_PS,
                (cpu.sreg(SR_PS) & !PS_CALLINC) | (callinc << PS_CALLINC_SHIFT),
            );
            Outcome::Jump(o[0].value)
        }
        Opcode::OPCODE_CALLX0 => {
            // Target is byte-precise, no low-2-bit masking (QEMU
            // translate_callx0 = mov tmp, in; movi a0, pc_next; gen_jump).
            // PS.CALLINC is NOT modified (see OPCODE_CALL0 note).
            let target = cpu.reg(o[0].value);
            cpu.set_reg(0, cpu.pc.wrapping_add(len));
            Outcome::Jump(target)
        }
        Opcode::OPCODE_CALLX4 | Opcode::OPCODE_CALLX8 | Opcode::OPCODE_CALLX12 => {
            let callinc = o[1].value / 4;
            // The target register `as` shares its physical slot with the
            // return address (phys[wb*4 + callinc*4]): read it BEFORE the
            // write, or the jump goes to the return address (QEMU
            // gen_callw_slot materializes arg[0].in first).
            let target = cpu.reg(o[0].value);
            // QEMU gen_callw_slot: return addr = pc_next = pc + actual length.
            cpu.set_reg(
                callinc * 4,
                (callinc << 30) | (cpu.pc.wrapping_add(len) & 0x3fff_ffff),
            );
            cpu.set_sreg(
                SR_PS,
                (cpu.sreg(SR_PS) & !PS_CALLINC) | (callinc << PS_CALLINC_SHIFT),
            );
            Outcome::Jump(target)
        }

        // ------------------------------------------------------------------
        // Windowed register management (ISA RM "Window Handling
        // Instructions"; QEMU win_helper.c).
        // ------------------------------------------------------------------
        Opcode::OPCODE_ENTRY => {
            let callinc = (cpu.sreg(SR_PS) & PS_CALLINC) >> PS_CALLINC_SHIFT;
            let s = o[0].value;
            let imm = o[2].value;
            // Illegal if PS.WOE == 0 or s > 3 (QEMU test_exceptions_entry).
            if cpu.sreg(SR_PS) & PS_WOE == 0 || s > 3 {
                cpu.raise_cause(cpu.pc, ILLEGAL_INSTRUCTION_CAUSE);
                return Outcome::Exception(ILLEGAL_INSTRUCTION_CAUSE);
            }
            // Extra overflow check for the callee's window unit (QEMU
            // test_overflow_entry: mask |= 1 << (callinc * 4)).
            let wmask = 1u32 << (callinc * 4);
            if (31 - wmask.leading_zeros()) / 4 > cpu.window() {
                let cause = cpu.window_overflow(cpu.pc);
                return Outcome::Exception(cause);
            }
            // entry a(s), imm: new a(s&3) = a(s) - imm, then rotate by
            // callinc and mark the new window active (QEMU HELPER(entry)).
            let wb_next = (cpu.windowbase() + callinc) & 0xf;
            cpu.set_reg((callinc << 2) | (s & 3), cpu.reg(s).wrapping_sub(imm));
            cpu.set_sreg(
                SR_WINDOW_START,
                cpu.sreg(SR_WINDOW_START) | (1u32 << wb_next),
            );
            cpu.windowbase_next = Some(wb_next);
            Outcome::Seq
        }
        Opcode::OPCODE_RETW | Opcode::OPCODE_RETW_N => {
            let a0 = cpu.reg(0);
            let n = (a0 >> 30) & 3;
            // Illegal if PS.WOE == 0 (QEMU test_exceptions_retw).
            if cpu.sreg(SR_PS) & PS_WOE == 0 {
                cpu.raise_cause(cpu.pc, ILLEGAL_INSTRUCTION_CAUSE);
                return Outcome::Exception(ILLEGAL_INSTRUCTION_CAUSE);
            }
            let wb = cpu.windowbase();
            let ws = cpu.sreg(SR_WINDOW_START);
            // m = nearest active window below wb (within 3); must match n
            // (QEMU HELPER(test_ill_retw)).
            let m = if ws & (1 << ((wb + 15) & 15)) != 0 {
                1
            } else if ws & (1 << ((wb + 14) & 15)) != 0 {
                2
            } else if ws & (1 << ((wb + 13) & 15)) != 0 {
                3
            } else {
                0
            };
            if n == 0 || (m != 0 && m != n) {
                cpu.raise_cause(cpu.pc, ILLEGAL_INSTRUCTION_CAUSE);
                return Outcome::Exception(ILLEGAL_INSTRUCTION_CAUSE);
            }
            // Underflow: caller window not active (QEMU
            // HELPER(test_underflow_retw)).
            if ws & (1 << ((wb + 16 - n) & 15)) == 0 {
                let cause = cpu.window_underflow(cpu.pc, n);
                return Outcome::Exception(cause);
            }
            // retw: clear current window bit, rotate back, jump to a0
            // (QEMU translate_retw + HELPER(retw)).
            cpu.set_sreg(SR_WINDOW_START, ws & !(1u32 << wb));
            cpu.rotate(-(n as i32));
            Outcome::Jump((cpu.pc & 0xc000_0000) | (a0 & 0x3fff_ffff))
        }
        Opcode::OPCODE_RET | Opcode::OPCODE_RET_N => {
            // ret: jump to a0, byte-precise — NO low-2-bit masking (QEMU
            // translate_ret = gen_jump(dc, cpu_R[0])).  Real firmware
            // returns from call0 at pc ≡ 3 (mod 4) to a0 = pc+3 ≡ 2
            // (mod 4); masking would execute garbage mid-instruction.
            Outcome::Jump(cpu.reg(0))
        }
        Opcode::OPCODE_MOVSP => {
            // movsp t, s: t = s, with ALLOCA check: the three windows below
            // must contain an active window (QEMU translate_movsp +
            // HELPER(movsp)).
            let wb = cpu.windowbase();
            let ws = cpu.sreg(SR_WINDOW_START);
            let below = (1u32 << ((wb + 15) & 15))
                | (1u32 << ((wb + 14) & 15))
                | (1u32 << ((wb + 13) & 15));
            if ws & below == 0 {
                cpu.raise_cause(cpu.pc, ALLOCA_CAUSE);
                return Outcome::Exception(ALLOCA_CAUSE);
            }
            cpu.set_reg(o[0].value, cpu.reg(o[1].value));
            Outcome::Seq
        }
        Opcode::OPCODE_ROTW => {
            // rotw imm: rotate window by imm units (QEMU translate_rotw).
            cpu.windowbase_next =
                Some(((cpu.windowbase() as i32 + o[0].value as i32) & 0xf) as u32);
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // Loop instructions (ISA RM "Zero-overhead Loops"; QEMU
        // translate_loop + gen_check_loop_end).
        // ------------------------------------------------------------------
        Opcode::OPCODE_LOOP | Opcode::OPCODE_LOOPNEZ | Opcode::OPCODE_LOOPGTZ => {
            let as_ = cpu.reg(o[0].value);
            cpu.set_sreg(SR_LCOUNT, as_.wrapping_sub(1));
            // QEMU translate_loop: LBEG = dc->base.pc_next (instruction
            // after the loop, i.e. pc + actual length).
            cpu.set_sreg(SR_LBEG, cpu.pc.wrapping_add(len));
            cpu.set_sreg(SR_LEND, o[1].value);
            if (opc == Opcode::OPCODE_LOOPNEZ && as_ == 0)
                || (opc == Opcode::OPCODE_LOOPGTZ && (as_ as i32) <= 0)
            {
                Outcome::Jump(o[1].value)
            } else {
                Outcome::Seq
            }
        }

        // ------------------------------------------------------------------
        // Exceptions and control (ISA RM "Exception Handling"; QEMU
        // exc_helper.c).
        // ------------------------------------------------------------------
        Opcode::OPCODE_ILL | Opcode::OPCODE_ILL_N => {
            cpu.raise_cause(cpu.pc, ILLEGAL_INSTRUCTION_CAUSE);
            Outcome::Exception(ILLEGAL_INSTRUCTION_CAUSE)
        }
        Opcode::OPCODE_SYSCALL => {
            cpu.raise_cause(cpu.pc, SYSCALL_CAUSE);
            Outcome::Exception(SYSCALL_CAUSE)
        }
        Opcode::OPCODE_SIMCALL => {
            // simcall: semi-hosting interface; no-op for P1.
            Outcome::Seq
        }
        Opcode::OPCODE_RFE => {
            // rfe: PS.EXCM = 0; jump to EPC1 (QEMU translate_rfe).
            cpu.set_sreg(SR_PS, cpu.sreg(SR_PS) & !PS_EXCM);
            Outcome::Jump(cpu.sreg(SR_EPC1))
        }
        Opcode::OPCODE_RFDE => {
            // rfde: jump EPC1 (no NDEPC on ESP32-S3; QEMU translate_rfde).
            Outcome::Jump(cpu.sreg(SR_EPC1))
        }
        Opcode::OPCODE_RFDD | Opcode::OPCODE_RFDO => {
            // rfdd/rfdo are illegal on this core (XTENSA_OP_ILL).
            cpu.raise_cause(cpu.pc, ILLEGAL_INSTRUCTION_CAUSE);
            Outcome::Exception(ILLEGAL_INSTRUCTION_CAUSE)
        }
        Opcode::OPCODE_RFI => {
            // rfi level: PS = EPS(level); jump EPC(level) (QEMU
            // translate_rfi; EPS2 + lvl - 2, EPC1 + lvl - 1).
            let lvl = o[0].value;
            cpu.set_sreg(SR_PS, cpu.sreg(SR_EPS2 + lvl - 2));
            Outcome::Jump(cpu.sreg(SR_EPC1 + lvl - 1))
        }
        Opcode::OPCODE_RFWO | Opcode::OPCODE_RFWU => {
            // rfwo/rfwu: clear EXCM, clear/set current window bit, restore
            // OWB, jump EPC1 (QEMU translate_rfw).
            cpu.set_sreg(SR_PS, cpu.sreg(SR_PS) & !PS_EXCM);
            let wb = cpu.windowbase();
            let bit = 1u32 << wb;
            let ws = cpu.sreg(SR_WINDOW_START);
            cpu.set_sreg(
                SR_WINDOW_START,
                if opc == Opcode::OPCODE_RFWO {
                    ws & !bit
                } else {
                    ws | bit
                },
            );
            cpu.set_sreg(SR_WINDOW_BASE, (cpu.sreg(SR_PS) & PS_OWB) >> 8);
            Outcome::Jump(cpu.sreg(SR_EPC1))
        }
        Opcode::OPCODE_RSIL => {
            // rsil t, level: t = old PS; PS = (PS & ~0xf) | level.
            let old = cpu.sreg(SR_PS);
            cpu.set_sreg(SR_PS, (old & !0xf) | (o[1].value & 0xf));
            cpu.set_reg(o[0].value, old);
            Outcome::Seq
        }
        Opcode::OPCODE_WAITI => {
            // waiti level: architectural power-down hint; modeled as no-op
            // (interrupts still preempt normally on the next step).
            Outcome::Seq
        }
        Opcode::OPCODE_EXCW => {
            // excw: PS.EXCM = 0 (QEMU translate_nop; no coprocessor state
            // in P1).
            cpu.set_sreg(SR_PS, cpu.sreg(SR_PS) & !PS_EXCM);
            Outcome::Seq
        }
        Opcode::OPCODE_BREAK | Opcode::OPCODE_BREAK_N => {
            // break: only active with debug enabled; otherwise no-op (QEMU
            // translate_break checks dc->debug).
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // SR access (ISA RM "Special Registers"; QEMU translate_rsr/wsr/xsr
        // with the sr_name guard: every enumerated variant below is valid on
        // the ESP32-S3 per core-esp32s3/xtensa-modules.inc.c).
        // ------------------------------------------------------------------
        Opcode::OPCODE_RSR_INTERRUPT => {
            // rsr.interrupt: live interrupt status = sticky INTSET bits
            // ORed with the asserted SoC lines (QEMU keeps the live line
            // state in INTSET itself — xtensa_irq — and this core's
            // modules decode rsr.interrupt as SR 226).
            cpu.set_reg(o[0].value, cpu.intset_live(bus));
            Outcome::Seq
        }
        Opcode::OPCODE_RSR_LBEG
        | Opcode::OPCODE_RSR_LEND
        | Opcode::OPCODE_RSR_LCOUNT
        | Opcode::OPCODE_RSR_SAR
        | Opcode::OPCODE_RSR_BR
        | Opcode::OPCODE_RSR_LITBASE
        | Opcode::OPCODE_RSR_SCOMPARE1
        | Opcode::OPCODE_RSR_ACCLO
        | Opcode::OPCODE_RSR_ACCHI
        | Opcode::OPCODE_RSR_M0
        | Opcode::OPCODE_RSR_M1
        | Opcode::OPCODE_RSR_M2
        | Opcode::OPCODE_RSR_M3
        | Opcode::OPCODE_RSR_MISC0
        | Opcode::OPCODE_RSR_MISC1
        | Opcode::OPCODE_RSR_MISC2
        | Opcode::OPCODE_RSR_MISC3
        | Opcode::OPCODE_RSR_ERACCESS
        | Opcode::OPCODE_RSR_IBREAKENABLE
        | Opcode::OPCODE_RSR_MEMCTL
        | Opcode::OPCODE_RSR_ATOMCTL
        | Opcode::OPCODE_RSR_DDR
        | Opcode::OPCODE_RSR_IBREAKA0
        | Opcode::OPCODE_RSR_IBREAKA1
        | Opcode::OPCODE_RSR_DBREAKA0
        | Opcode::OPCODE_RSR_DBREAKA1
        | Opcode::OPCODE_RSR_DBREAKC0
        | Opcode::OPCODE_RSR_DBREAKC1
        | Opcode::OPCODE_RSR_CONFIGID0
        | Opcode::OPCODE_RSR_EPC1
        | Opcode::OPCODE_RSR_EPC2
        | Opcode::OPCODE_RSR_EPC3
        | Opcode::OPCODE_RSR_EPC4
        | Opcode::OPCODE_RSR_EPC5
        | Opcode::OPCODE_RSR_EPC6
        | Opcode::OPCODE_RSR_EPC7
        | Opcode::OPCODE_RSR_DEPC
        | Opcode::OPCODE_RSR_EPS2
        | Opcode::OPCODE_RSR_EPS3
        | Opcode::OPCODE_RSR_EPS4
        | Opcode::OPCODE_RSR_EPS5
        | Opcode::OPCODE_RSR_EPS6
        | Opcode::OPCODE_RSR_EPS7
        | Opcode::OPCODE_RSR_CONFIGID1
        | Opcode::OPCODE_RSR_EXCSAVE1
        | Opcode::OPCODE_RSR_EXCSAVE2
        | Opcode::OPCODE_RSR_EXCSAVE3
        | Opcode::OPCODE_RSR_EXCSAVE4
        | Opcode::OPCODE_RSR_EXCSAVE5
        | Opcode::OPCODE_RSR_EXCSAVE6
        | Opcode::OPCODE_RSR_EXCSAVE7
        | Opcode::OPCODE_RSR_CPENABLE
        | Opcode::OPCODE_RSR_INTENABLE
        | Opcode::OPCODE_RSR_PS
        | Opcode::OPCODE_RSR_VECBASE
        | Opcode::OPCODE_RSR_EXCCAUSE
        | Opcode::OPCODE_RSR_DEBUGCAUSE
        | Opcode::OPCODE_RSR_CCOUNT
        | Opcode::OPCODE_RSR_PRID
        | Opcode::OPCODE_RSR_ICOUNT
        | Opcode::OPCODE_RSR_ICOUNTLEVEL
        | Opcode::OPCODE_RSR_EXCVADDR
        | Opcode::OPCODE_RSR_CCOMPARE0
        | Opcode::OPCODE_RSR_CCOMPARE1
        | Opcode::OPCODE_RSR_CCOMPARE2
        | Opcode::OPCODE_RSR_WINDOWBASE
        | Opcode::OPCODE_RSR_WINDOWSTART => {
            cpu.set_reg(o[0].value, cpu.sreg(sr_of(opc)));
            Outcome::Seq
        }
        Opcode::OPCODE_WSR_MMID => {
            // MMID (memory-management ID) has no storage on the MMU-less
            // S3: the write is dropped instead of aliasing SR 0 (LBEG)
            // through sr_of's catch-all (same bug class as the old PRID /
            // WINDOWBASE gaps). Unobservable: no RSR_MMID decodes.
            Outcome::Seq
        }
        Opcode::OPCODE_WSR_LBEG
        | Opcode::OPCODE_WSR_LEND
        | Opcode::OPCODE_WSR_LCOUNT
        | Opcode::OPCODE_WSR_SAR
        | Opcode::OPCODE_WSR_BR
        | Opcode::OPCODE_WSR_LITBASE
        | Opcode::OPCODE_WSR_SCOMPARE1
        | Opcode::OPCODE_WSR_ACCLO
        | Opcode::OPCODE_WSR_ACCHI
        | Opcode::OPCODE_WSR_M0
        | Opcode::OPCODE_WSR_M1
        | Opcode::OPCODE_WSR_M2
        | Opcode::OPCODE_WSR_M3
        | Opcode::OPCODE_WSR_MISC0
        | Opcode::OPCODE_WSR_MISC1
        | Opcode::OPCODE_WSR_MISC2
        | Opcode::OPCODE_WSR_MISC3
        | Opcode::OPCODE_WSR_ERACCESS
        | Opcode::OPCODE_WSR_IBREAKENABLE
        | Opcode::OPCODE_WSR_MEMCTL
        | Opcode::OPCODE_WSR_ATOMCTL
        | Opcode::OPCODE_WSR_DDR
        | Opcode::OPCODE_WSR_IBREAKA0
        | Opcode::OPCODE_WSR_IBREAKA1
        | Opcode::OPCODE_WSR_DBREAKA0
        | Opcode::OPCODE_WSR_DBREAKA1
        | Opcode::OPCODE_WSR_DBREAKC0
        | Opcode::OPCODE_WSR_DBREAKC1
        | Opcode::OPCODE_WSR_CONFIGID0
        | Opcode::OPCODE_WSR_EPC1
        | Opcode::OPCODE_WSR_EPC2
        | Opcode::OPCODE_WSR_EPC3
        | Opcode::OPCODE_WSR_EPC4
        | Opcode::OPCODE_WSR_EPC5
        | Opcode::OPCODE_WSR_EPC6
        | Opcode::OPCODE_WSR_EPC7
        | Opcode::OPCODE_WSR_DEPC
        | Opcode::OPCODE_WSR_EPS2
        | Opcode::OPCODE_WSR_EPS3
        | Opcode::OPCODE_WSR_EPS4
        | Opcode::OPCODE_WSR_EPS5
        | Opcode::OPCODE_WSR_EPS6
        | Opcode::OPCODE_WSR_EPS7
        | Opcode::OPCODE_WSR_EXCSAVE1
        | Opcode::OPCODE_WSR_EXCSAVE2
        | Opcode::OPCODE_WSR_EXCSAVE3
        | Opcode::OPCODE_WSR_EXCSAVE4
        | Opcode::OPCODE_WSR_EXCSAVE5
        | Opcode::OPCODE_WSR_EXCSAVE6
        | Opcode::OPCODE_WSR_EXCSAVE7
        | Opcode::OPCODE_WSR_CPENABLE
        | Opcode::OPCODE_WSR_INTENABLE
        | Opcode::OPCODE_WSR_PS
        | Opcode::OPCODE_WSR_VECBASE
        | Opcode::OPCODE_WSR_EXCCAUSE
        | Opcode::OPCODE_WSR_DEBUGCAUSE
        | Opcode::OPCODE_WSR_CCOUNT
        | Opcode::OPCODE_WSR_ICOUNT
        | Opcode::OPCODE_WSR_ICOUNTLEVEL
        | Opcode::OPCODE_WSR_EXCVADDR
        | Opcode::OPCODE_WSR_CCOMPARE0
        | Opcode::OPCODE_WSR_CCOMPARE1
        | Opcode::OPCODE_WSR_CCOMPARE2 => {
            cpu.set_sreg(sr_of(opc), cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_WSR_INTSET => {
            // wsr.intset: OR the value into the sticky interrupt set
            // (QEMU HELPER(intset) — software-interrupt writes; the
            // ESP32-S3 has no software-inttype mask).
            cpu.set_sreg(SR_INTSET, cpu.sreg(SR_INTSET) | cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_WSR_INTCLEAR => {
            // wsr.intclear: AND ~value into the sticky interrupt set
            // (QEMU HELPER(intclear); the SoC line state is unaffected).
            cpu.set_sreg(SR_INTSET, cpu.sreg(SR_INTSET) & !cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_WSR_WINDOWBASE | Opcode::OPCODE_XSR_WINDOWBASE => {
            // wsr.windowbase / xsr.windowbase: rotation is deferred (QEMU
            // translate_wsr_windowbase / translate_xsr_windowbase).
            let old = cpu.sreg(SR_WINDOW_BASE);
            cpu.windowbase_next = Some(cpu.reg(o[0].value) & 0xf);
            if opc == Opcode::OPCODE_XSR_WINDOWBASE {
                cpu.set_reg(o[0].value, old);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_WSR_WINDOWSTART | Opcode::OPCODE_XSR_WINDOWSTART => {
            // wsr.windowstart / xsr.windowstart: value masked to 16 bits
            // (QEMU translate_wsr_windowstart).
            let old = cpu.sreg(SR_WINDOW_START);
            cpu.set_sreg(SR_WINDOW_START, cpu.reg(o[0].value) & 0xffff);
            if opc == Opcode::OPCODE_XSR_WINDOWSTART {
                cpu.set_reg(o[0].value, old);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_XSR_LBEG
        | Opcode::OPCODE_XSR_LEND
        | Opcode::OPCODE_XSR_LCOUNT
        | Opcode::OPCODE_XSR_SAR
        | Opcode::OPCODE_XSR_BR
        | Opcode::OPCODE_XSR_LITBASE
        | Opcode::OPCODE_XSR_SCOMPARE1
        | Opcode::OPCODE_XSR_ACCLO
        | Opcode::OPCODE_XSR_ACCHI
        | Opcode::OPCODE_XSR_M0
        | Opcode::OPCODE_XSR_M1
        | Opcode::OPCODE_XSR_M2
        | Opcode::OPCODE_XSR_M3
        | Opcode::OPCODE_XSR_MISC0
        | Opcode::OPCODE_XSR_MISC1
        | Opcode::OPCODE_XSR_MISC2
        | Opcode::OPCODE_XSR_MISC3
        | Opcode::OPCODE_XSR_ERACCESS
        | Opcode::OPCODE_XSR_IBREAKENABLE
        | Opcode::OPCODE_XSR_MEMCTL
        | Opcode::OPCODE_XSR_ATOMCTL
        | Opcode::OPCODE_XSR_DDR
        | Opcode::OPCODE_XSR_IBREAKA0
        | Opcode::OPCODE_XSR_IBREAKA1
        | Opcode::OPCODE_XSR_DBREAKA0
        | Opcode::OPCODE_XSR_DBREAKA1
        | Opcode::OPCODE_XSR_DBREAKC0
        | Opcode::OPCODE_XSR_DBREAKC1
        | Opcode::OPCODE_XSR_EPC1
        | Opcode::OPCODE_XSR_EPC2
        | Opcode::OPCODE_XSR_EPC3
        | Opcode::OPCODE_XSR_EPC4
        | Opcode::OPCODE_XSR_EPC5
        | Opcode::OPCODE_XSR_EPC6
        | Opcode::OPCODE_XSR_EPC7
        | Opcode::OPCODE_XSR_DEPC
        | Opcode::OPCODE_XSR_EPS2
        | Opcode::OPCODE_XSR_EPS3
        | Opcode::OPCODE_XSR_EPS4
        | Opcode::OPCODE_XSR_EPS5
        | Opcode::OPCODE_XSR_EPS6
        | Opcode::OPCODE_XSR_EPS7
        | Opcode::OPCODE_XSR_EXCSAVE1
        | Opcode::OPCODE_XSR_EXCSAVE2
        | Opcode::OPCODE_XSR_EXCSAVE3
        | Opcode::OPCODE_XSR_EXCSAVE4
        | Opcode::OPCODE_XSR_EXCSAVE5
        | Opcode::OPCODE_XSR_EXCSAVE6
        | Opcode::OPCODE_XSR_EXCSAVE7
        | Opcode::OPCODE_XSR_CPENABLE
        | Opcode::OPCODE_XSR_INTENABLE
        | Opcode::OPCODE_XSR_PS
        | Opcode::OPCODE_XSR_VECBASE
        | Opcode::OPCODE_XSR_EXCCAUSE
        | Opcode::OPCODE_XSR_DEBUGCAUSE
        | Opcode::OPCODE_XSR_CCOUNT
        | Opcode::OPCODE_XSR_ICOUNT
        | Opcode::OPCODE_XSR_ICOUNTLEVEL
        | Opcode::OPCODE_XSR_EXCVADDR
        | Opcode::OPCODE_XSR_CCOMPARE0
        | Opcode::OPCODE_XSR_CCOMPARE1
        | Opcode::OPCODE_XSR_CCOMPARE2 => {
            // xsr: swap register and SR (QEMU translate_xsr).
            let n = sr_of(opc);
            let old = cpu.sreg(n);
            cpu.set_sreg(n, cpu.reg(o[0].value));
            cpu.set_reg(o[0].value, old);
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // User SR access (RUR/WUR; QEMU translate_rur / translate_wur).
        // The DSP TIE registers (FFT/QACC/ACCX/UA_STATE/GPIO_OUT) have no
        // emulated hardware in P1; they are plain storage.
        // ------------------------------------------------------------------
        Opcode::OPCODE_RUR_THREADPTR => {
            cpu.set_reg(o[0].value, cpu.user_sreg(UR_THREADPTR));
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_THREADPTR => {
            cpu.set_user_sreg(UR_THREADPTR, cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_SAR_BYTE => {
            cpu.set_reg(o[0].value, cpu.sar_byte as u32);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_SAR_BYTE => {
            let v = cpu.reg(o[0].value);
            cpu.sar_byte = v as u8;
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_FCR => {
            cpu.set_reg(o[0].value, cpu.user_sreg(UR_FCR));
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_FCR => {
            cpu.set_user_sreg(UR_FCR, cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_FSR => {
            cpu.set_reg(o[0].value, cpu.user_sreg(UR_FSR));
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_FSR => {
            cpu.set_user_sreg(UR_FSR, cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_FFT_BIT_WIDTH => {
            cpu.set_reg(o[0].value, cpu.fft_width as u32);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_FFT_BIT_WIDTH => {
            let v = cpu.reg(o[0].value);
            cpu.fft_width = v as u8;
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_ACCX_0 => {
            cpu.set_reg(o[0].value, cpu.accx as u32);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_ACCX_0 => {
            let v = cpu.reg(o[0].value);
            // QEMU wur_s3 masks ACCX to 44 bits on every half write.
            cpu.accx = ((cpu.accx & -0x1_0000_0000) | v as u64 as i64) & 0xFFF_FFFF_FFFF;
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_ACCX_1 => {
            cpu.set_reg(o[0].value, (cpu.accx >> 32) as u32);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_ACCX_1 => {
            let v = cpu.reg(o[0].value);
            cpu.accx = (((v as i64) << 32) | (cpu.accx & 0xFFFF_FFFF)) & 0xFFF_FFFF_FFFF;
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_H_0 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[1], 0));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_H_1 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[1], 1));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_H_2 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[1], 2));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_H_3 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[1], 3));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_H_4 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[1], 4));
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_H_0 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[1], 0, v);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_H_1 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[1], 1, v);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_H_2 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[1], 2, v);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_H_3 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[1], 3, v);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_H_4 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[1], 4, v);
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_L_0 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[0], 0));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_L_1 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[0], 1));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_L_2 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[0], 2));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_L_3 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[0], 3));
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_QACC_L_4 => {
            cpu.set_reg(o[0].value, crate::ee::qacc_word(&cpu.accq[0], 4));
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_L_0 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[0], 0, v);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_L_1 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[0], 1, v);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_L_2 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[0], 2, v);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_L_3 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[0], 3, v);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_QACC_L_4 => {
            let v = cpu.reg(o[0].value);
            crate::ee::set_qacc_word(&mut cpu.accq[0], 4, v);
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_UA_STATE_0 => {
            cpu.set_reg(
                o[0].value,
                u32::from_le_bytes(cpu.ua_state[0..4].try_into().unwrap()),
            );
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_UA_STATE_1 => {
            cpu.set_reg(
                o[0].value,
                u32::from_le_bytes(cpu.ua_state[4..8].try_into().unwrap()),
            );
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_UA_STATE_2 => {
            cpu.set_reg(
                o[0].value,
                u32::from_le_bytes(cpu.ua_state[4 * 2..4 * 2 + 4].try_into().unwrap()),
            );
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_UA_STATE_3 => {
            cpu.set_reg(
                o[0].value,
                u32::from_le_bytes(cpu.ua_state[4 * 3..4 * 3 + 4].try_into().unwrap()),
            );
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_UA_STATE_0 => {
            let v = cpu.reg(o[0].value);
            cpu.ua_state[0..4].copy_from_slice(&v.to_le_bytes());
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_UA_STATE_1 => {
            let v = cpu.reg(o[0].value);
            cpu.ua_state[4..8].copy_from_slice(&v.to_le_bytes());
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_UA_STATE_2 => {
            let v = cpu.reg(o[0].value);
            cpu.ua_state[4 * 2..4 * 2 + 4].copy_from_slice(&v.to_le_bytes());
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_UA_STATE_3 => {
            let v = cpu.reg(o[0].value);
            cpu.ua_state[4 * 3..4 * 3 + 4].copy_from_slice(&v.to_le_bytes());
            Outcome::Seq
        }
        Opcode::OPCODE_RUR_GPIO_OUT => {
            cpu.set_reg(o[0].value, cpu.tie_gpio);
            Outcome::Seq
        }
        Opcode::OPCODE_WUR_GPIO_OUT => {
            let v = cpu.reg(o[0].value);
            cpu.tie_gpio = v;
            Outcome::Seq
        }

        // ------------------------------------------------------------------
        // 16-bit density instructions (ISA RM "16-Bit Instruction
        // Formats"; same semantics as their 24-bit counterparts, fields
        // decoded by the inst16a/inst16b slots).
        // ------------------------------------------------------------------
        Opcode::OPCODE_ADD_N => {
            // add.n at, as, at: r = s + t (inst16a QRST).
            cpu.set_reg(
                o[0].value,
                cpu.reg(o[1].value).wrapping_add(cpu.reg(o[2].value)),
            );
            Outcome::Seq
        }
        Opcode::OPCODE_ADDI_N => {
            // addi.n at, as, imm: imm from the ai4c table (inst16a QRI).
            cpu.set_reg(o[0].value, cpu.reg(o[1].value).wrapping_add(o[2].value));
            Outcome::Seq
        }
        Opcode::OPCODE_MOV_N => {
            // mov.n at, as (inst16b QRI).
            cpu.set_reg(o[0].value, cpu.reg(o[1].value));
            Outcome::Seq
        }
        Opcode::OPCODE_MOVI_N => {
            cpu.set_reg(o[0].value, o[1].value);
            Outcome::Seq
        }
        Opcode::OPCODE_L32I_N => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            cpu.set_reg(o[0].value, bus.read32(addr));
            Outcome::Seq
        }
        Opcode::OPCODE_S32I_N => {
            let addr = cpu.reg(o[1].value).wrapping_add(o[2].value);
            bus.write32(addr, cpu.reg(o[0].value));
            Outcome::Seq
        }
        Opcode::OPCODE_NOP | Opcode::OPCODE_NOP_N => Outcome::Seq,
        Opcode::OPCODE_MEMW
        | Opcode::OPCODE_EXTW
        | Opcode::OPCODE_ISYNC
        | Opcode::OPCODE_RSYNC
        | Opcode::OPCODE_ESYNC
        | Opcode::OPCODE_DSYNC
        | Opcode::OPCODE_IDTLB
        | Opcode::OPCODE_IITLB
        | Opcode::OPCODE_PDTLB
        | Opcode::OPCODE_PITLB
        | Opcode::OPCODE_RDTLB0
        | Opcode::OPCODE_RDTLB1
        | Opcode::OPCODE_RITLB0
        | Opcode::OPCODE_RITLB1
        | Opcode::OPCODE_WDTLB
        | Opcode::OPCODE_WITLB
        | Opcode::OPCODE_RER
        | Opcode::OPCODE_WER => {
            // System/TLB/cache barrier instructions: no architectural
            // effects in the P1 emulation (QEMU translates most as no-ops).
            // NOTE: OPCODE_RFR/WFR (FP moves) must NOT be listed here —
            // they have real behavior in the FPU section below.
            Outcome::Seq
        }
        Opcode::OPCODE_LDDEC => {
            // lddec t, s: t = *(s); s -= 4 (TIE ldinc/lddec family).
            let addr = cpu.reg(o[1].value);
            cpu.set_reg(o[0].value, bus.read32(addr));
            cpu.set_reg(o[1].value, addr.wrapping_sub(4));
            Outcome::Seq
        }
        Opcode::OPCODE_LDINC => {
            let addr = cpu.reg(o[1].value);
            cpu.set_reg(o[0].value, bus.read32(addr));
            cpu.set_reg(o[1].value, addr.wrapping_add(4));
            Outcome::Seq
        }
        // ------------------------------------------------------------------
        // MAC16 16-bit multiply-accumulate (ISA RM "MAC16 Option"; QEMU
        // target/xtensa/translate.c translate_mac16 plus the
        // core-esp32s3 operand tables for reference; every operand mapping
        // below was additionally verified byte-for-byte against
        // xtensa-esp32s3-elf-as output). First suffix letter selects the s
        // source (A=AR[s], D=MR[x-field]); second selects the t source
        // (A=AR[t], D=MR[2+t[2]] — the my operand decodes the y field plus
        // 2, QEMU OperandSem_opnd_sem_MR_0_decode). H/L pick the high
        // ([31:16], sign-extended down-shift) or low ([15:0],
        // sign/zero-extended) half; UMUL.AA is the unsigned variant.
        // Results accumulate into the 40-bit ACC (ACCHI[7:0]:ACCLO):
        // MUL/UMUL overwrite it (ACCHI = sign extension, 0 for UMUL);
        // MULA/MULS add/subtract the sign-extended product, then truncate
        // ACCHI to its low 8 bits (QEMU ext8s). .LDINC/.LDDEC (MULA.DA/DD
        // only) first move mem32[AR[s]+/-4] into MR[w] and postupdate AR[s]
        // by +/-4 (QEMU ld_offset; TEUL so unaligned never faults, matching
        // the S32I convention below).
        Opcode::OPCODE_MUL_AA_HH => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_AA_HL => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_AA_LH => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_AA_LL => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_AD_HH => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_AD_HL => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_AD_LH => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_AD_LL => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_DA_HH => {
            mac16_set(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_DA_HL => {
            mac16_set(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_DA_LH => {
            mac16_set(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_DA_LL => {
            mac16_set(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_DD_HH => {
            mac16_set(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_DD_HL => {
            mac16_set(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_DD_LH => {
            mac16_set(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_DD_LL => {
            mac16_set(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_AA_HH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_AA_HL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_AA_LH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_AA_LL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_AD_HH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_AD_HL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_AD_LH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_AD_LL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_HH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_HL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_LH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_LL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_HH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_HL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_LH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_LL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_AA_HH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_AA_HL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_AA_LH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_AA_LL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_AD_HH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_AD_HL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_AD_LH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_AD_LL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_DA_HH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_DA_HL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_DA_LH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_DA_LL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.reg(o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_DD_HH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_DD_HL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_DD_LH => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), true, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULS_DD_LL => {
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[0].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[1].value), false, false),
                false,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_UMUL_AA_HH => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, true),
                mac16_half(cpu.reg(o[1].value), true, true),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_UMUL_AA_HL => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), true, true),
                mac16_half(cpu.reg(o[1].value), false, true),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_UMUL_AA_LH => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, true),
                mac16_half(cpu.reg(o[1].value), true, true),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_UMUL_AA_LL => {
            mac16_set(
                cpu,
                mac16_half(cpu.reg(o[0].value), false, true),
                mac16_half(cpu.reg(o[1].value), false, true),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_HH_LDINC => {
            let addr = cpu.reg(o[1].value).wrapping_add(4u32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), true, false),
                mac16_half(cpu.reg(o[3].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_HH_LDDEC => {
            let addr = cpu.reg(o[1].value).wrapping_add(0xFFFF_FFFCu32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), true, false),
                mac16_half(cpu.reg(o[3].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_HL_LDINC => {
            let addr = cpu.reg(o[1].value).wrapping_add(4u32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), true, false),
                mac16_half(cpu.reg(o[3].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_HL_LDDEC => {
            let addr = cpu.reg(o[1].value).wrapping_add(0xFFFF_FFFCu32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), true, false),
                mac16_half(cpu.reg(o[3].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_LH_LDINC => {
            let addr = cpu.reg(o[1].value).wrapping_add(4u32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), false, false),
                mac16_half(cpu.reg(o[3].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_LH_LDDEC => {
            let addr = cpu.reg(o[1].value).wrapping_add(0xFFFF_FFFCu32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), false, false),
                mac16_half(cpu.reg(o[3].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_LL_LDINC => {
            let addr = cpu.reg(o[1].value).wrapping_add(4u32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), false, false),
                mac16_half(cpu.reg(o[3].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DA_LL_LDDEC => {
            let addr = cpu.reg(o[1].value).wrapping_add(0xFFFF_FFFCu32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), false, false),
                mac16_half(cpu.reg(o[3].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_HH_LDINC => {
            let addr = cpu.reg(o[1].value).wrapping_add(4u32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[3].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_HH_LDDEC => {
            let addr = cpu.reg(o[1].value).wrapping_add(0xFFFF_FFFCu32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[3].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_HL_LDINC => {
            let addr = cpu.reg(o[1].value).wrapping_add(4u32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[3].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_HL_LDDEC => {
            let addr = cpu.reg(o[1].value).wrapping_add(0xFFFF_FFFCu32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), true, false),
                mac16_half(cpu.sreg(SR_M0 + o[3].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_LH_LDINC => {
            let addr = cpu.reg(o[1].value).wrapping_add(4u32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[3].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_LH_LDDEC => {
            let addr = cpu.reg(o[1].value).wrapping_add(0xFFFF_FFFCu32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[3].value), true, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_LL_LDINC => {
            let addr = cpu.reg(o[1].value).wrapping_add(4u32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[3].value), false, false),
                true,
            );
            Outcome::Seq
        }
        Opcode::OPCODE_MULA_DD_LL_LDDEC => {
            let addr = cpu.reg(o[1].value).wrapping_add(0xFFFF_FFFCu32);
            let wv = bus.read32(addr);
            cpu.set_reg(o[1].value, addr);
            cpu.set_sreg(SR_M0 + o[0].value, wv);
            mac16_acc(
                cpu,
                mac16_half(cpu.sreg(SR_M0 + o[2].value), false, false),
                mac16_half(cpu.sreg(SR_M0 + o[3].value), false, false),
                true,
            );
            Outcome::Seq
        }
        // ------------------------------------------------------------------
        // Single-precision FPU (ISA RM 4.3.11 + Chapter 6 separate
        // instruction pages; QEMU target/xtensa/fpu_helper.c for the
        // arithmetic edge rules). The 16 FPRs are NOT windowed. FSR
        // exception flags and non-default rounding modes are NOT modeled
        // (round-to-nearest-even everywhere, the reset default — matches
        // every firmware-observable value; flag accumulation alone is
        // never read by in-tree firmware). NaN payload propagation follows
        // Rust f32 IEEE semantics (an acceptable approximation of the
        // use-first-NaN rule; values, infinities and NaN-ness all match).
        // ------------------------------------------------------------------
        Opcode::OPCODE_WFR => {
            // wfr fr, as: FR[r] <- AR[s] raw bits (ISA RM "WFR").
            let v = cpu.reg(o[1].value);
            cpu.set_freg(o[0].value, f32::from_bits(v));
            Outcome::Seq
        }
        Opcode::OPCODE_RFR => {
            // rfr ar, fs: AR[r] <- FR[s] raw bits (ISA RM "RFR").
            cpu.set_reg(o[0].value, cpu.freg(o[1].value).to_bits());
            Outcome::Seq
        }
        Opcode::OPCODE_MOV_S => {
            // mov.s fr, fs (ISA RM "MOV.S").
            let v = cpu.freg(o[1].value);
            cpu.set_freg(o[0].value, v);
            Outcome::Seq
        }
        Opcode::OPCODE_ABS_S => {
            // abs.s fr, fs: clear sign bit (ISA RM "ABS.S").
            let v = cpu.freg(o[1].value);
            cpu.set_freg(o[0].value, v.abs());
            Outcome::Seq
        }
        Opcode::OPCODE_NEG_S => {
            // neg.s fr, fs: flip sign bit, NaN payload preserved through
            // the bits (ISA RM "NEG.S").
            let v = cpu.freg(o[1].value);
            cpu.set_freg(o[0].value, -v);
            Outcome::Seq
        }
        Opcode::OPCODE_ADD_S => {
            // add.s fr, fs, ft (ISA RM "ADD.S").
            let v = cpu.freg(o[1].value) + cpu.freg(o[2].value);
            cpu.set_freg(o[0].value, v);
            Outcome::Seq
        }
        Opcode::OPCODE_SUB_S => {
            // sub.s fr, fs, ft (ISA RM "SUB.S").
            let v = cpu.freg(o[1].value) - cpu.freg(o[2].value);
            cpu.set_freg(o[0].value, v);
            Outcome::Seq
        }
        Opcode::OPCODE_MUL_S => {
            // mul.s fr, fs, ft (ISA RM "MUL.S").
            let v = cpu.freg(o[1].value) * cpu.freg(o[2].value);
            cpu.set_freg(o[0].value, v);
            Outcome::Seq
        }
        Opcode::OPCODE_MADD_S => {
            // madd.s fr, fs, ft: FR[r] <- FR[r] +s (FR[s] x FR[t]),
            // single rounding (QEMU float32_muladd; libm::fmaf is
            // correctly rounded like the fused HW op).
            let (a, b, c) = (
                cpu.freg(o[0].value),
                cpu.freg(o[1].value),
                cpu.freg(o[2].value),
            );
            cpu.set_freg(o[0].value, libm::fmaf(b, c, a));
            Outcome::Seq
        }
        Opcode::OPCODE_MSUB_S => {
            // msub.s fr, fs, ft: FR[r] <- FR[r] -s (FR[s] x FR[t])
            // (QEMU float_muladd_negate_product).
            let (a, b, c) = (
                cpu.freg(o[0].value),
                cpu.freg(o[1].value),
                cpu.freg(o[2].value),
            );
            cpu.set_freg(o[0].value, libm::fmaf(-b, c, a));
            Outcome::Seq
        }
        Opcode::OPCODE_MKDADJ_S => {
            // mkdadj.s fr, fs: FR[r] <- FR[s] / FR[r]_old — the divide
            // adjust step (QEMU mkdadj_s helper = div(b, a); fr is both
            // source and dest, so snapshot first).
            let (a, b) = (cpu.freg(o[0].value), cpu.freg(o[1].value));
            cpu.set_freg(o[0].value, b / a);
            Outcome::Seq
        }
        // Divide/square-root step instructions (QEMU parity, verified by
        // the libgcc-sequence tests below): QEMU implements these as NOPs
        // (commit f8c6137 "implement FPU division and square root": "most
        // of them as nops, but the results of div/sqrt sequences is
        // preserved").  The libgcc __divsf3/__ieee754_sqrtf sequences
        // collapse correctly around the NOPs because MKDADJ/MKSADJ perform
        // the true divide/sqrt and ADDEXPM moves the result into the
        // final quotient register that DIVN leaves untouched:
        //   div:  ... mkdadj.s ex,a (= a/b) ... addexpm.s q,ex (= q) ...
        //         divn.s q,r,y (NOP) -> q = correctly-rounded a/b.
        //   sqrt: ... mksadj.s y,a (= sqrt(a)) ... addexpm.s r,y (= r) ...
        //         divn.s r,a,t1 (NOP) -> r = correctly-rounded sqrt(a).
        Opcode::OPCODE_MADDN_S
        | Opcode::OPCODE_DIV0_S
        | Opcode::OPCODE_DIVN_S
        | Opcode::OPCODE_NEXP01_S
        | Opcode::OPCODE_ADDEXP_S
        | Opcode::OPCODE_SQRT0_S
        | Opcode::OPCODE_RECIP0_S
        | Opcode::OPCODE_RSQRT0_S => Outcome::Seq,
        Opcode::OPCODE_ADDEXPM_S => {
            // addexpm.s fr, fs: move (QEMU translate_mov_s).
            let v = cpu.freg(o[1].value);
            cpu.set_freg(o[0].value, v);
            Outcome::Seq
        }
        Opcode::OPCODE_MKSADJ_S => {
            // mksadj.s fr, fs: FR[r] <- sqrt(FR[s]) (QEMU mksadj_s).
            cpu.set_freg(o[0].value, libm::sqrtf(cpu.freg(o[1].value)));
            Outcome::Seq
        }
        Opcode::OPCODE_FLOAT_S => {
            // float.s fr, as, t: FR[r] <- float(AR[s]) x 2^-t, t = 0..15
            // (ISA RM "FLOAT.S"; QEMU itof_s with scale = -imm; scalbn
            // is exact, the int->float cast carries the single rounding).
            let v = cpu.reg(o[1].value) as i32 as f32;
            cpu.set_freg(o[0].value, libm::scalbnf(v, -(o[2].value as i32)));
            Outcome::Seq
        }
        Opcode::OPCODE_UFLOAT_S => {
            // ufloat.s fr, as, t: unsigned variant (ISA RM "UFLOAT.S").
            let v = cpu.reg(o[1].value) as f32;
            cpu.set_freg(o[0].value, libm::scalbnf(v, -(o[2].value as i32)));
            Outcome::Seq
        }
        Opcode::OPCODE_TRUNC_S
        | Opcode::OPCODE_FLOOR_S
        | Opcode::OPCODE_CEIL_S
        | Opcode::OPCODE_ROUND_S => {
            // trunc/floor/ceil/round.s ar, fs, t: AR[r] <- round(FR[s] x
            // 2^t) with per-op rounding; edge rules per the ISA RM TRUNC.S
            // page (positive overflow/+inf/NaN -> 0x7FFFFFFF, negative
            // overflow/-inf -> 0x80000000; the ROUND/FLOOR/CEIL pages share
            // the saturation shape with their own rounding).
            let x = libm::scalbnf(cpu.freg(o[1].value), o[2].value as i32);
            let r = if x.is_nan() || x >= 2147483648.0 {
                0x7FFF_FFFFu32
            } else if x <= -2147483648.0 {
                0x8000_0000u32
            } else {
                let y = match opc {
                    Opcode::OPCODE_FLOOR_S => libm::floorf(x),
                    Opcode::OPCODE_CEIL_S => libm::ceilf(x),
                    Opcode::OPCODE_ROUND_S => libm::roundevenf(x),
                    _ => libm::truncf(x),
                };
                y as i32 as u32
            };
            cpu.set_reg(o[0].value, r);
            Outcome::Seq
        }
        Opcode::OPCODE_UTRUNC_S => {
            // utrunc.s ar, fs, t: unsigned variant; ISA RM UTRUNC.S page:
            // positive overflow/+inf/NaN -> 0xFFFFFFFF, negative/-inf ->
            // 0x80000000 (NOT zero).
            let x = libm::scalbnf(cpu.freg(o[1].value), o[2].value as i32);
            let r = if x.is_nan() || x >= 4294967296.0 {
                0xFFFF_FFFFu32
            } else if x < 0.0 {
                0x8000_0000u32
            } else {
                libm::truncf(x) as u32
            };
            cpu.set_reg(o[0].value, r);
            Outcome::Seq
        }
        Opcode::OPCODE_CONST_S => {
            // const.s fr, imm4: 0 -> +0.0, 1 -> +1.0, 2 -> +2.0, 3 -> +0.5
            // (QEMU translate_const_s table; imm >= 4 is reserved — fold
            // mod 4 like QEMU rather than faulting).
            const CONST_S_TAB: [u32; 4] = [0x0000_0000, 0x3F80_0000, 0x4000_0000, 0x3F00_0000];
            cpu.set_freg(
                o[0].value,
                f32::from_bits(CONST_S_TAB[(o[1].value & 3) as usize]),
            );
            Outcome::Seq
        }
        Opcode::OPCODE_OEQ_S
        | Opcode::OPCODE_OLE_S
        | Opcode::OPCODE_OLT_S
        | Opcode::OPCODE_UEQ_S
        | Opcode::OPCODE_ULE_S
        | Opcode::OPCODE_ULT_S
        | Opcode::OPCODE_UN_S => {
            // FP compares write boolean bit r (ISA RM compare pages;
            // QEMU translate_compare_s): ordered ops are false on NaN,
            // unordered ops are true on NaN, UN tests NaN-ness.
            let (s, t) = (cpu.freg(o[1].value), cpu.freg(o[2].value));
            let v = match opc {
                Opcode::OPCODE_OEQ_S => s == t,
                Opcode::OPCODE_OLE_S => s <= t,
                Opcode::OPCODE_OLT_S => s < t,
                Opcode::OPCODE_UEQ_S => s.is_nan() || t.is_nan() || s == t,
                Opcode::OPCODE_ULE_S => s.is_nan() || t.is_nan() || s <= t,
                Opcode::OPCODE_ULT_S => s.is_nan() || t.is_nan() || s < t,
                _ => s.is_nan() || t.is_nan(),
            };
            cpu.set_br(o[0].value, v);
            Outcome::Seq
        }
        Opcode::OPCODE_MOVT_S => {
            // movt.s fr, fs, bt: if BR[t] then FR[r] <- FR[s] (ISA RM).
            if cpu.br(o[2].value) {
                let v = cpu.freg(o[1].value);
                cpu.set_freg(o[0].value, v);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVF_S => {
            // movf.s fr, fs, bt: if !BR[t] then FR[r] <- FR[s] (ISA RM).
            if !cpu.br(o[2].value) {
                let v = cpu.freg(o[1].value);
                cpu.set_freg(o[0].value, v);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVEQZ_S => {
            // moveqz.s fr, fs, at: if AR[t] == 0 (ISA RM "MOVEQZ.S").
            if cpu.reg(o[2].value) == 0 {
                let v = cpu.freg(o[1].value);
                cpu.set_freg(o[0].value, v);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVNEZ_S => {
            // movnez.s fr, fs, at: if AR[t] != 0 (ISA RM "MOVNEZ.S").
            if cpu.reg(o[2].value) != 0 {
                let v = cpu.freg(o[1].value);
                cpu.set_freg(o[0].value, v);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVLTZ_S => {
            // movltz.s fr, fs, at: if AR[t]31 set, i.e. signed < 0.
            if cpu.reg(o[2].value) >> 31 != 0 {
                let v = cpu.freg(o[1].value);
                cpu.set_freg(o[0].value, v);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_MOVGEZ_S => {
            // movgez.s fr, fs, at: if AR[t]31 clear, i.e. signed >= 0.
            if cpu.reg(o[2].value) >> 31 == 0 {
                let v = cpu.freg(o[1].value);
                cpu.set_freg(o[0].value, v);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_LSI | Opcode::OPCODE_LSIP => {
            // lsi ft, as, imm8 / lsip (post-increment): FR[t] <- Load32
            // (AR[s] + imm8<<2); lsip then writes AR[s] <- AR[s] + imm
            // (QEMU ldsti_s pars {false,imm,update}; ISA RM "LSI").
            // NOTE the dest is the T field (RRI8 has no r field).
            let base = cpu.reg(o[1].value);
            let v = bus.read32(base.wrapping_add(o[2].value));
            cpu.set_freg(o[0].value, f32::from_bits(v));
            if opc == Opcode::OPCODE_LSIP {
                cpu.set_reg(o[1].value, base.wrapping_add(o[2].value));
            }
            Outcome::Seq
        }
        Opcode::OPCODE_SSI | Opcode::OPCODE_SSIP => {
            // ssi/ssip: mirror of lsi/lsip (ISA RM "SSI").
            let base = cpu.reg(o[1].value);
            bus.write32(
                base.wrapping_add(o[2].value),
                cpu.freg(o[0].value).to_bits(),
            );
            if opc == Opcode::OPCODE_SSIP {
                cpu.set_reg(o[1].value, base.wrapping_add(o[2].value));
            }
            Outcome::Seq
        }
        Opcode::OPCODE_LSX | Opcode::OPCODE_LSXP => {
            // lsx fr, as, at / lsxp: FR[r] <- Load32(AR[s] + AR[t]);
            // lsxp writes AR[s] <- vAddr (ISA RM "LSX", RRR so dest = r).
            let addr = cpu.reg(o[1].value).wrapping_add(cpu.reg(o[2].value));
            cpu.set_freg(o[0].value, f32::from_bits(bus.read32(addr)));
            if opc == Opcode::OPCODE_LSXP {
                cpu.set_reg(o[1].value, addr);
            }
            Outcome::Seq
        }
        Opcode::OPCODE_SSX | Opcode::OPCODE_SSXP => {
            // ssx/ssxp: mirror of lsx/lsxp (ISA RM "SSX").
            let addr = cpu.reg(o[1].value).wrapping_add(cpu.reg(o[2].value));
            bus.write32(addr, cpu.freg(o[0].value).to_bits());
            if opc == Opcode::OPCODE_SSXP {
                cpu.set_reg(o[1].value, addr);
            }
            Outcome::Seq
        }
        o if crate::ee::is_ee_opcode(o) => {
            // TIE/DSP extension (operands derived from the raw word inside).
            if crate::ee::exec_ee(cpu, bus, o, cpu.last_raw()) {
                Outcome::Seq
            } else {
                Outcome::Unimplemented
            }
        }
        _ => Outcome::Unimplemented,
    }
}

/// Conditional branch: `idx` is the index of the (pc-adjusted) target
/// operand in `o`; the branch is taken to `o[idx].value` when `cond` holds.
fn branch_if(o: &[Opnd; 8], idx: usize, cond: bool) -> Outcome {
    if cond {
        Outcome::Jump(o[idx].value)
    } else {
        Outcome::Seq
    }
}

/// Map an enumerated RSR/WSR/XSR opcode to its SR number.  The set of
/// valid SRs mirrors core-esp32s3/xtensa-modules.inc.c (which is what makes
/// the generated decoder emit these variants at all).
fn sr_of(opc: Opcode) -> u32 {
    use crate::cpu::*;
    match opc {
        Opcode::OPCODE_RSR_LBEG | Opcode::OPCODE_WSR_LBEG | Opcode::OPCODE_XSR_LBEG => SR_LBEG,
        Opcode::OPCODE_RSR_LEND | Opcode::OPCODE_WSR_LEND | Opcode::OPCODE_XSR_LEND => SR_LEND,
        Opcode::OPCODE_RSR_LCOUNT | Opcode::OPCODE_WSR_LCOUNT | Opcode::OPCODE_XSR_LCOUNT => {
            SR_LCOUNT
        }
        Opcode::OPCODE_RSR_SAR | Opcode::OPCODE_WSR_SAR | Opcode::OPCODE_XSR_SAR => SR_SAR,
        Opcode::OPCODE_RSR_BR | Opcode::OPCODE_WSR_BR | Opcode::OPCODE_XSR_BR => SR_BR,
        Opcode::OPCODE_RSR_LITBASE | Opcode::OPCODE_WSR_LITBASE | Opcode::OPCODE_XSR_LITBASE => {
            SR_LITBASE
        }
        Opcode::OPCODE_RSR_SCOMPARE1
        | Opcode::OPCODE_WSR_SCOMPARE1
        | Opcode::OPCODE_XSR_SCOMPARE1 => SR_SCOMPARE1,
        Opcode::OPCODE_RSR_ACCLO | Opcode::OPCODE_WSR_ACCLO | Opcode::OPCODE_XSR_ACCLO => SR_ACCLO,
        Opcode::OPCODE_RSR_ACCHI | Opcode::OPCODE_WSR_ACCHI | Opcode::OPCODE_XSR_ACCHI => SR_ACCHI,
        Opcode::OPCODE_RSR_M0 | Opcode::OPCODE_WSR_M0 | Opcode::OPCODE_XSR_M0 => SR_M0,
        Opcode::OPCODE_RSR_M1 | Opcode::OPCODE_WSR_M1 | Opcode::OPCODE_XSR_M1 => SR_M1,
        Opcode::OPCODE_RSR_M2 | Opcode::OPCODE_WSR_M2 | Opcode::OPCODE_XSR_M2 => SR_M2,
        Opcode::OPCODE_RSR_M3 | Opcode::OPCODE_WSR_M3 | Opcode::OPCODE_XSR_M3 => SR_M3,
        Opcode::OPCODE_RSR_MISC0 | Opcode::OPCODE_WSR_MISC0 | Opcode::OPCODE_XSR_MISC0 => SR_MISC0,
        Opcode::OPCODE_RSR_MISC1 | Opcode::OPCODE_WSR_MISC1 | Opcode::OPCODE_XSR_MISC1 => SR_MISC1,
        Opcode::OPCODE_RSR_MISC2 | Opcode::OPCODE_WSR_MISC2 | Opcode::OPCODE_XSR_MISC2 => SR_MISC2,
        Opcode::OPCODE_RSR_MISC3 | Opcode::OPCODE_WSR_MISC3 | Opcode::OPCODE_XSR_MISC3 => SR_MISC3,
        Opcode::OPCODE_RSR_ERACCESS | Opcode::OPCODE_WSR_ERACCESS | Opcode::OPCODE_XSR_ERACCESS => {
            SR_ERACCESS
        }
        Opcode::OPCODE_RSR_IBREAKENABLE
        | Opcode::OPCODE_WSR_IBREAKENABLE
        | Opcode::OPCODE_XSR_IBREAKENABLE => SR_IBREAKENABLE,
        Opcode::OPCODE_RSR_MEMCTL | Opcode::OPCODE_WSR_MEMCTL | Opcode::OPCODE_XSR_MEMCTL => {
            SR_MEMCTL
        }
        Opcode::OPCODE_RSR_ATOMCTL | Opcode::OPCODE_WSR_ATOMCTL | Opcode::OPCODE_XSR_ATOMCTL => {
            SR_ATOMCTL
        }
        Opcode::OPCODE_RSR_DDR | Opcode::OPCODE_WSR_DDR | Opcode::OPCODE_XSR_DDR => SR_DDR,
        Opcode::OPCODE_RSR_IBREAKA0 | Opcode::OPCODE_WSR_IBREAKA0 | Opcode::OPCODE_XSR_IBREAKA0 => {
            SR_IBREAKA0
        }
        Opcode::OPCODE_RSR_IBREAKA1 | Opcode::OPCODE_WSR_IBREAKA1 | Opcode::OPCODE_XSR_IBREAKA1 => {
            SR_IBREAKA1
        }
        Opcode::OPCODE_RSR_DBREAKA0 | Opcode::OPCODE_WSR_DBREAKA0 | Opcode::OPCODE_XSR_DBREAKA0 => {
            SR_DBREAKA0
        }
        Opcode::OPCODE_RSR_DBREAKA1 | Opcode::OPCODE_WSR_DBREAKA1 | Opcode::OPCODE_XSR_DBREAKA1 => {
            SR_DBREAKA1
        }
        Opcode::OPCODE_RSR_DBREAKC0 | Opcode::OPCODE_WSR_DBREAKC0 | Opcode::OPCODE_XSR_DBREAKC0 => {
            SR_DBREAKC0
        }
        Opcode::OPCODE_RSR_DBREAKC1 | Opcode::OPCODE_WSR_DBREAKC1 | Opcode::OPCODE_XSR_DBREAKC1 => {
            SR_DBREAKC1
        }
        Opcode::OPCODE_RSR_CONFIGID0 | Opcode::OPCODE_WSR_CONFIGID0 => SR_CONFIGID0,
        Opcode::OPCODE_RSR_EPC1 | Opcode::OPCODE_WSR_EPC1 | Opcode::OPCODE_XSR_EPC1 => SR_EPC1,
        Opcode::OPCODE_RSR_EPC2 | Opcode::OPCODE_WSR_EPC2 | Opcode::OPCODE_XSR_EPC2 => SR_EPC2,
        Opcode::OPCODE_RSR_EPC3 | Opcode::OPCODE_WSR_EPC3 | Opcode::OPCODE_XSR_EPC3 => SR_EPC3,
        Opcode::OPCODE_RSR_EPC4 | Opcode::OPCODE_WSR_EPC4 | Opcode::OPCODE_XSR_EPC4 => SR_EPC4,
        Opcode::OPCODE_RSR_EPC5 | Opcode::OPCODE_WSR_EPC5 | Opcode::OPCODE_XSR_EPC5 => SR_EPC5,
        Opcode::OPCODE_RSR_EPC6 | Opcode::OPCODE_WSR_EPC6 | Opcode::OPCODE_XSR_EPC6 => SR_EPC6,
        Opcode::OPCODE_RSR_EPC7 | Opcode::OPCODE_WSR_EPC7 | Opcode::OPCODE_XSR_EPC7 => SR_EPC7,
        Opcode::OPCODE_RSR_DEPC | Opcode::OPCODE_WSR_DEPC | Opcode::OPCODE_XSR_DEPC => SR_DEPC,
        Opcode::OPCODE_RSR_EPS2 | Opcode::OPCODE_WSR_EPS2 | Opcode::OPCODE_XSR_EPS2 => SR_EPS2,
        Opcode::OPCODE_RSR_EPS3 | Opcode::OPCODE_WSR_EPS3 | Opcode::OPCODE_XSR_EPS3 => SR_EPS3,
        Opcode::OPCODE_RSR_EPS4 | Opcode::OPCODE_WSR_EPS4 | Opcode::OPCODE_XSR_EPS4 => SR_EPS4,
        Opcode::OPCODE_RSR_EPS5 | Opcode::OPCODE_WSR_EPS5 | Opcode::OPCODE_XSR_EPS5 => SR_EPS5,
        Opcode::OPCODE_RSR_EPS6 | Opcode::OPCODE_WSR_EPS6 | Opcode::OPCODE_XSR_EPS6 => SR_EPS6,
        Opcode::OPCODE_RSR_EPS7 | Opcode::OPCODE_WSR_EPS7 | Opcode::OPCODE_XSR_EPS7 => SR_EPS7,
        Opcode::OPCODE_RSR_CONFIGID1 => SR_CONFIGID1,
        Opcode::OPCODE_RSR_EXCSAVE1 | Opcode::OPCODE_WSR_EXCSAVE1 | Opcode::OPCODE_XSR_EXCSAVE1 => {
            SR_EXCSAVE1
        }
        Opcode::OPCODE_RSR_EXCSAVE2 | Opcode::OPCODE_WSR_EXCSAVE2 | Opcode::OPCODE_XSR_EXCSAVE2 => {
            SR_EXCSAVE2
        }
        Opcode::OPCODE_RSR_EXCSAVE3 | Opcode::OPCODE_WSR_EXCSAVE3 | Opcode::OPCODE_XSR_EXCSAVE3 => {
            SR_EXCSAVE3
        }
        Opcode::OPCODE_RSR_EXCSAVE4 | Opcode::OPCODE_WSR_EXCSAVE4 | Opcode::OPCODE_XSR_EXCSAVE4 => {
            SR_EXCSAVE4
        }
        Opcode::OPCODE_RSR_EXCSAVE5 | Opcode::OPCODE_WSR_EXCSAVE5 | Opcode::OPCODE_XSR_EXCSAVE5 => {
            SR_EXCSAVE5
        }
        Opcode::OPCODE_RSR_EXCSAVE6 | Opcode::OPCODE_WSR_EXCSAVE6 | Opcode::OPCODE_XSR_EXCSAVE6 => {
            SR_EXCSAVE6
        }
        Opcode::OPCODE_RSR_EXCSAVE7 | Opcode::OPCODE_WSR_EXCSAVE7 | Opcode::OPCODE_XSR_EXCSAVE7 => {
            SR_EXCSAVE7
        }
        Opcode::OPCODE_RSR_CPENABLE | Opcode::OPCODE_WSR_CPENABLE | Opcode::OPCODE_XSR_CPENABLE => {
            SR_CPENABLE
        }
        // This core's modules decode rsr.interrupt as SR 226 (INTSET) —
        // there is no SR 225 access on the ESP32-S3.
        Opcode::OPCODE_RSR_INTERRUPT => SR_INTSET,
        Opcode::OPCODE_WSR_INTCLEAR => SR_INTCLEAR,
        Opcode::OPCODE_RSR_INTENABLE
        | Opcode::OPCODE_WSR_INTENABLE
        | Opcode::OPCODE_XSR_INTENABLE => SR_INTENABLE,
        Opcode::OPCODE_WSR_INTSET => SR_INTSET,
        Opcode::OPCODE_RSR_PS | Opcode::OPCODE_WSR_PS | Opcode::OPCODE_XSR_PS => SR_PS,
        Opcode::OPCODE_RSR_VECBASE | Opcode::OPCODE_WSR_VECBASE | Opcode::OPCODE_XSR_VECBASE => {
            SR_VECBASE
        }
        Opcode::OPCODE_RSR_EXCCAUSE | Opcode::OPCODE_WSR_EXCCAUSE | Opcode::OPCODE_XSR_EXCCAUSE => {
            SR_EXCCAUSE
        }
        Opcode::OPCODE_RSR_DEBUGCAUSE
        | Opcode::OPCODE_WSR_DEBUGCAUSE
        | Opcode::OPCODE_XSR_DEBUGCAUSE => SR_DEBUGCAUSE,
        Opcode::OPCODE_RSR_CCOUNT | Opcode::OPCODE_WSR_CCOUNT | Opcode::OPCODE_XSR_CCOUNT => {
            SR_CCOUNT
        }
        // PRID is read-only strapping of the core number (no WSR/XSR form).
        Opcode::OPCODE_RSR_PRID => SR_PRID,
        Opcode::OPCODE_RSR_ICOUNT | Opcode::OPCODE_WSR_ICOUNT | Opcode::OPCODE_XSR_ICOUNT => {
            SR_ICOUNT
        }
        Opcode::OPCODE_RSR_ICOUNTLEVEL
        | Opcode::OPCODE_WSR_ICOUNTLEVEL
        | Opcode::OPCODE_XSR_ICOUNTLEVEL => SR_ICOUNTLEVEL,
        Opcode::OPCODE_RSR_EXCVADDR | Opcode::OPCODE_WSR_EXCVADDR | Opcode::OPCODE_XSR_EXCVADDR => {
            SR_EXCVADDR
        }
        Opcode::OPCODE_RSR_CCOMPARE0
        | Opcode::OPCODE_WSR_CCOMPARE0
        | Opcode::OPCODE_XSR_CCOMPARE0 => SR_CCOMPARE0,
        Opcode::OPCODE_RSR_CCOMPARE1
        | Opcode::OPCODE_WSR_CCOMPARE1
        | Opcode::OPCODE_XSR_CCOMPARE1 => SR_CCOMPARE1,
        Opcode::OPCODE_RSR_CCOMPARE2
        | Opcode::OPCODE_WSR_CCOMPARE2
        | Opcode::OPCODE_XSR_CCOMPARE2 => SR_CCOMPARE2,
        Opcode::OPCODE_RSR_WINDOWBASE
        | Opcode::OPCODE_WSR_WINDOWBASE
        | Opcode::OPCODE_XSR_WINDOWBASE => SR_WINDOW_BASE,
        Opcode::OPCODE_RSR_WINDOWSTART
        | Opcode::OPCODE_WSR_WINDOWSTART
        | Opcode::OPCODE_XSR_WINDOWSTART => SR_WINDOW_START,
        _ => 0,
    }
}
