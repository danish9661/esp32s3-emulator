//! Xtensa TIE / DSP (`ee.*`) `format_32` extension decoder.
//!
//! The ESP32-S3's LX7 core exposes a set of TIE (Tensilica Instruction
//! Extension) and DSP instructions (FFT, vector MAC, vector load/store,
//! broadcast load/store, GPIO/quarter-round accumulators) encoded as 4-byte
//! `format_32` opcodes. The primary Xtensa decoder in `generated.rs` already
//! recognizes the ~129 `ee.*` opcodes it can derive from the QEMU module
//! table; everything else in the `format_32` space is routed here.
//!
//! `decode_ee` is the designated home for the remaining `format_32` families
//! (the ~29 mnemonics rejected by the ISA-coverage audit: `ee.fft.*`,
//! `ee.vmulas.*`, `ee.ldf/stf/ld.qacc.*`, ...). Each family is selected by the
//! `format_32` primary opcode fields (`fld_inst_*` accessors in
//! `generated.rs`) and maps to one of the `OPCODE_EE_*` variants. Until a
//! family is implemented the default is `OPCODE_EE_UNIMPLEMENTED`, which the
//! executor treats as a graceful `StepResult::Unimplemented`.

use crate::cpu::Cpu;
use crate::{Bus, generated::Opcode};

/// A `format_32` TIE/DSP instruction is 4 bytes wide. Per `insn_len`, the
/// low nibble of byte 0 is 14 or 15 for the 4-byte formats (the 2-byte density
/// slots are 8..=13 and the 3-byte slots are 0..=7).
pub fn is_format32(insn: u32) -> bool {
    (insn & 0xF) >= 14
}

/// True for any `ee.*` TIE/DSP opcode variant (`OPCODE_EE_*`).
///
/// The `OPCODE_EE_*` variants occupy a contiguous discriminant block
/// (498..=753) with no other opcodes interleaved, so a range test is
/// sufficient and `no_std`-friendly.
pub fn is_ee_opcode(op: Opcode) -> bool {
    matches!(op as u32, 498..=753)
}

/// Coarse family label for an `ee.*` opcode, used for diagnostics (the
/// trap message names the TIE unit the firmware reached for). Derived from
/// the mnemonic's first component so it stays correct across all ~129
/// decoded variants without enumerating them; curated aliases normalize
/// the historic names.
pub fn ee_family(op: Opcode) -> &'static str {
    let rest = op.name().strip_prefix("ee_").unwrap_or("tie");
    let unit = rest.split('_').next().unwrap_or("tie");
    match unit {
        "vmulas" | "vmul" => "vmac",
        "wr" | "set" | "clr" | "get" => "gpio",
        _ => unit,
    }
}

/// Decode an `ee.*` / `format_32` TIE/DSP instruction.
///
/// This is the fallback for `decode_inst`: the ~129 opcodes already derived
/// from the QEMU module table are matched upstream; this function is reached
/// only for the remaining `format_32` families. Implement per-family
/// recognition here (mirroring the `fld_inst_*` conditions in `generated.rs`)
/// and return the matching `OPCODE_EE_*`; unrecognized encodings fall back to
/// `OPCODE_EE_UNIMPLEMENTED`.
pub fn decode_ee(insn: u32) -> Opcode {
    // 4-byte format_32 TIE recognizer (narrow opcode-bit rules, each pinned
    // by an assembler-captured KAT word; unknown patterns stay
    // OPCODE_EE_UNIMPLEMENTED so firmware traps loud instead of mis-
    // executing). Byte order: b0 = insn[7:0] .. b3 = insn[31:24].
    let b0 = insn & 0xFF;
    let b1 = (insn >> 8) & 0xFF;
    let b2 = (insn >> 16) & 0xFF;
    let b3 = (insn >> 24) & 0xFF;
    if b0 & 0x0F == 0x0E || b0 & 0x0F == 0x0F {
        if b3 == 0xE7 {
            return Opcode::OPCODE_EE_STXQ_32;
        }
        if (b3 & 0xFC) == 0xE0 {
            // Fused complex-multiply + load (cmul.s16.ld.incp; GAS-captured:
            // qu[0]=b2[3], qu[2:1]=b3[1:0], AR=b0[7:4], qx/qy split, sel=
            // b1[1:0]). key2==0 separates it from LDXQ (key2==7) sharing
            // nibble D, and no valu-fused entry uses key2 0. Dest rides
            // b3[1:0], so the page spans E0-E3 (unlike the E0-exact arms
            // below, which keep their exact match to preserve behavior).
            if (b0 & 0x0E) == 0x0E && (b1 & 0x0C) == 0x0C && ((b2 >> 4) & 7) == 0 {
                return Opcode::OPCODE_EE_CMUL_S16_LD_INCP;
            }
            // NOTE: mem-qu rides b3[1:0] (qu[2:1]; GAS-probed: qu=q0/q2/
            // q4/q6 -> b3=E0/E1/E2/E3, b1/b2 unchanged), so the page spans
            // E0-E3 — an exact b3 == 0xE0 match would miss every qu>=q2
            // word (caught live: real firmware `vmax.s16.ld.incp q2,...`
            // = 0xE1950D8E trapped).
            if (b3 & 0xFC) == 0xE0 {
                // NOTE: b1 low nibble D (0x0D) is the shared LDXQ/LDF/STF
                // key — but the fused-ld row also uses it ((1,1)=vmax.s16
                // etc. GAS-probed 2026-09-14). The fused entry is the one
                // whose (b1[1:0],b2[6:4]) cell is populated, so check the
                // fused table FIRST and only fall through to LDXQ/LDF/STF
                // on a miss (previously LDXQ stole the whole nibble-D row).
                // Fused vector-ALU + load (op,width) in (b1[1:0], b2[6:4]).
                if b1 & 0x0F == 0x0C || b1 & 0x0F == 0x0D || b1 & 0x0F == 0x0E || b1 & 0x0F == 0x0F
                {
                    match ((b1 & 3), ((b2 >> 4) & 7)) {
                        (0, 1) => return Opcode::OPCODE_EE_VADDS_S8_LD_INCP,
                        (1, 6) => return Opcode::OPCODE_EE_VSUBS_S8_LD_INCP,
                        (0, 4) => return Opcode::OPCODE_EE_VMUL_S8_LD_INCP,
                        (1, 2) => return Opcode::OPCODE_EE_VADDS_S16_LD_INCP,
                        (3, 2) => return Opcode::OPCODE_EE_VMIN_S8_LD_INCP,
                        (3, 1) => return Opcode::OPCODE_EE_VMAX_S8_LD_INCP,
                        (1, 3) => return Opcode::OPCODE_EE_VADDS_S32_LD_INCP,
                        (3, 3) => return Opcode::OPCODE_EE_VMUL_S16_LD_INCP,
                        (1, 4) => return Opcode::OPCODE_EE_VSUBS_S16_LD_INCP,
                        (1, 1) => return Opcode::OPCODE_EE_VMAX_S16_LD_INCP,
                        (2, 1) => return Opcode::OPCODE_EE_VMAX_S32_LD_INCP,
                        (2, 2) => return Opcode::OPCODE_EE_VMIN_S16_LD_INCP,
                        (2, 3) => return Opcode::OPCODE_EE_VMIN_S32_LD_INCP,
                        (0, 5) => return Opcode::OPCODE_EE_VMUL_U16_LD_INCP,
                        (0, 6) => return Opcode::OPCODE_EE_VMUL_U8_LD_INCP,
                        (1, 5) => return Opcode::OPCODE_EE_VSUBS_S32_LD_INCP,
                        _ => {}
                    }
                }
                if b1 & 0x0F == 0x0D {
                    return Opcode::OPCODE_EE_LDXQ_32;
                }
                if b2 & 0xE0 == 0x00
                    && (b1 & 0x0F == 0x04 || b1 & 0x0F == 0x05 || b1 & 0x0F == 0x07)
                {
                    if b1 & 0x02 == 0 {
                        return Opcode::OPCODE_EE_LDF_64_IP;
                    }
                    return Opcode::OPCODE_EE_STF_64_IP;
                }
                // Fused vector-scalar-MAC + load (s8/s16 qacc; GAS-probed
                // 2026-09-14: b1 = 0xEC const; s8: b2 = 0x20..0x37 (sel =
                // b2[4]++b2[3:0], 0..15), s16: b2 = 0x70..0x7F (sel =
                // b2[3:0], lanes mask &3). The sel field aliases the width
                // bits, so match the high nibble, not the full byte.
                // (Safe: b1 = 0xEC selects fused cell (0,2)/(0,7), which is
                // absent from the fused-ld table above, so no hijack.)
                if (b1 & 0xFF) == 0xEC
                    && ((b2 & 0xF0) == 0x20 || (b2 & 0xF0) == 0x30 || (b2 & 0xF0) == 0x70)
                {
                    return match b2 & 0xF0 {
                        0x70 => Opcode::OPCODE_EE_VSMULAS_S16_QACC_LD_INCP,
                        _ => Opcode::OPCODE_EE_VSMULAS_S8_QACC_LD_INCP,
                    };
                }
            }
        }
        // Fused complex-multiply + store (cmul.s16.st.incp; GAS-captured:
        // qu(mem src)=b2[6:4], qz(MAC dest)=b2[2:0], AR=b0[7:4],
        // qx/qy split like ld, sel=b1[1:0], b3=0xE4 const, b2[3]=0
        // const). b2[3]==0 separates it from valu-st (b2[3]==1 const
        // across 7 probes with no operand mapped there).
        if b3 == 0xE4 && (b0 & 0x0E) == 0x0E && (b1 & 0x0C) == 0x00 && (b2 & 0x08) == 0x00 {
            return Opcode::OPCODE_EE_CMUL_S16_ST_INCP;
        }
        // Fused vector-ALU + store (GAS-probed full table 2026-09-14,
        // q0,a2,q3,q4,q5 forms; the operand sweeps prove b1[7:4]
        // carries qx[1:0]/qy[2:1] and b2[6:4] carries mem-qu — NEITHER is
        // an opcode const, so the family select is the b3 row + b1[1:0]
        // cell + b2[3:0] nibble:
        // E4 row (b2 nibble 0xB const): cell 2=vadds.s8, 0=vadds.s16,
        // 1=vadds.s32, 3=vmax.s16.
        // E5 row: (cell, nibble) (3,3)=vmul.s8 (0,3)=vmax.s32
        // (0,B)=vmax.s8 (2,3)=vmin.s8 (3,B)=vmul.u16 (1,3)=vmin.s16
        // (1,B)=vmin.s32 (2,B)=vmul.s16.
        // E8 row (cell 0=vmul.u8, 1=vsubs.s16, 2=vsubs.s32, 3=vsubs.s8):
        // gated on b2[7] + nibble 0xB, which separates it from r2bf.st
        // (b2 = 0x38..0x3F, bit7 clear) and src.q.ld.xp (b2[3] clear).
        // (Cautionary tale: an early draft gated E8 on (b1&0xF0)==0x20 —
        // b1[7:4] is q-fields, so real firmware with qx=q0/qy=q1 missed
        // it; and an (op,width)=(b1[1:0],b2[6:4]) table misread mem-qu as
        // width and aliased E5 pairs through b2[3].)
        if b3 == 0xE4 && (b1 & 0x0C) == 0x00 && (b2 & 0x0F) == 0x0B && (b0 & 0x0E) == 0x0E {
            return match b1 & 3 {
                2 => Opcode::OPCODE_EE_VADDS_S8_ST_INCP,
                0 => Opcode::OPCODE_EE_VADDS_S16_ST_INCP,
                1 => Opcode::OPCODE_EE_VADDS_S32_ST_INCP,
                _ => Opcode::OPCODE_EE_VMAX_S16_ST_INCP,
            };
        }
        if b3 == 0xE5 && (b0 & 0x0E) == 0x0E {
            return match ((b1 & 3), b2 & 0x0F) {
                (3, 0x3) => Opcode::OPCODE_EE_VMUL_S8_ST_INCP,
                (0, 0x3) => Opcode::OPCODE_EE_VMAX_S32_ST_INCP,
                (0, 0xB) => Opcode::OPCODE_EE_VMAX_S8_ST_INCP,
                (1, 0x3) => Opcode::OPCODE_EE_VMIN_S16_ST_INCP,
                (1, 0xB) => Opcode::OPCODE_EE_VMIN_S32_ST_INCP,
                (2, 0x3) => Opcode::OPCODE_EE_VMIN_S8_ST_INCP,
                (2, 0xB) => Opcode::OPCODE_EE_VMUL_S16_ST_INCP,
                (3, 0xB) => Opcode::OPCODE_EE_VMUL_U16_ST_INCP,
                _ => Opcode::OPCODE_EE_UNIMPLEMENTED,
            };
        }
        if b3 == 0xE8 && (b2 & 0x80) != 0 && (b2 & 0x0F) == 0x0B && (b0 & 0x0E) == 0x0E {
            return match b1 & 3 {
                0 => Opcode::OPCODE_EE_VMUL_U8_ST_INCP,
                1 => Opcode::OPCODE_EE_VSUBS_S16_ST_INCP,
                2 => Opcode::OPCODE_EE_VSUBS_S32_ST_INCP,
                _ => Opcode::OPCODE_EE_VSUBS_S8_ST_INCP,
            };
        }
        // Fused vector-MAC + load (b3 = 0xF0|qu[2:1]; b3 = 0x20|.. is the
        // .qup suffix, still trapped). (op,width) in b2[2:0], xp in b2[4].
        if b3 & 0xFC == 0xF0 {
            let xp = (b2 >> 4) & 1 == 1;
            return match ((b2 >> 2) & 1, (b2 >> 1) & 1, b2 & 1, xp) {
                (0, 1, 0, false) => Opcode::OPCODE_EE_VMULAS_S8_ACCX_LD_IP,
                (0, 1, 0, true) => Opcode::OPCODE_EE_VMULAS_S8_ACCX_LD_XP,
                (0, 0, 0, false) => Opcode::OPCODE_EE_VMULAS_S16_ACCX_LD_IP,
                (0, 0, 0, true) => Opcode::OPCODE_EE_VMULAS_S16_ACCX_LD_XP,
                (1, 1, 0, false) => Opcode::OPCODE_EE_VMULAS_U8_ACCX_LD_IP,
                (1, 1, 0, true) => Opcode::OPCODE_EE_VMULAS_U8_ACCX_LD_XP,
                (1, 0, 0, false) => Opcode::OPCODE_EE_VMULAS_U16_ACCX_LD_IP,
                (1, 0, 0, true) => Opcode::OPCODE_EE_VMULAS_U16_ACCX_LD_XP,
                (0, 1, 1, false) => Opcode::OPCODE_EE_VMULAS_S8_QACC_LD_IP,
                (0, 1, 1, true) => Opcode::OPCODE_EE_VMULAS_S8_QACC_LD_XP,
                (0, 0, 1, false) => Opcode::OPCODE_EE_VMULAS_S16_QACC_LD_IP,
                (0, 0, 1, true) => Opcode::OPCODE_EE_VMULAS_S16_QACC_LD_XP,
                (1, 1, 1, false) => Opcode::OPCODE_EE_VMULAS_U8_QACC_LD_IP,
                (1, 1, 1, true) => Opcode::OPCODE_EE_VMULAS_U8_QACC_LD_XP,
                (1, 0, 1, false) => Opcode::OPCODE_EE_VMULAS_U16_QACC_LD_IP,
                (1, 0, 1, true) => Opcode::OPCODE_EE_VMULAS_U16_QACC_LD_XP,
                _ => Opcode::OPCODE_EE_UNIMPLEMENTED,
            };
        }
        // 128-bit FPR load/store (ldf/stf.128.ip/xp; GAS-captured FPR
        // scatter: pos1=[0]b2[3]+[3:1]b3[2:0], pos2=[0]b0[0]+[3:1]b2[2:0],
        // pos3=b1[7:4], pos4=b2[7:4], AR=b0[7:4], ip-imm=b1[3:0]<<4 else
        // ax=b1[3:0], st=b3[4], xp=b3[3]). b3 high nibble 8/9 is disjoint
        // from every qup family (high nibbles 0-7,A-C across all 16
        // GAS-probed arms) — placed before qup, which otherwise steals
        // all four forms as S16_QACC_LD_XP_QUP.
        if ((b3 & 0xF0) == 0x80 || (b3 & 0xF0) == 0x90) && (b0 & 0x0E) == 0x0E {
            return match (b3 >> 3) & 3 {
                0 => Opcode::OPCODE_EE_LDF_128_IP,
                1 => Opcode::OPCODE_EE_LDF_128_XP,
                2 => Opcode::OPCODE_EE_STF_128_IP,
                _ => Opcode::OPCODE_EE_STF_128_XP,
            };
        }
        // Fused vector-MAC + Q-slide (b3[7:6] != 11, excluding the
        // cmul.st 0xA8 and ams.st 0xA0 pages; (uns,is8,qacc) in b3[6:4],
        // xp in b3[7]).
        if (b3 & 0xC0) != 0xC0 && (b3 & 0xF8) != 0xA8 && (b3 & 0xF0) != 0xA0 {
            let xp = (b3 >> 7) & 1 == 1;
            return match ((b3 >> 6) & 1, (b3 >> 5) & 1, (b3 >> 4) & 1, xp) {
                (0, 1, 0, false) => Opcode::OPCODE_EE_VMULAS_S8_ACCX_LD_IP_QUP,
                (0, 1, 0, true) => Opcode::OPCODE_EE_VMULAS_S8_ACCX_LD_XP_QUP,
                (0, 0, 0, false) => Opcode::OPCODE_EE_VMULAS_S16_ACCX_LD_IP_QUP,
                (0, 0, 0, true) => Opcode::OPCODE_EE_VMULAS_S16_ACCX_LD_XP_QUP,
                (1, 1, 0, false) => Opcode::OPCODE_EE_VMULAS_U8_ACCX_LD_IP_QUP,
                (1, 1, 0, true) => Opcode::OPCODE_EE_VMULAS_U8_ACCX_LD_XP_QUP,
                (1, 0, 0, false) => Opcode::OPCODE_EE_VMULAS_U16_ACCX_LD_IP_QUP,
                (1, 0, 0, true) => Opcode::OPCODE_EE_VMULAS_U16_ACCX_LD_XP_QUP,
                (0, 1, 1, false) => Opcode::OPCODE_EE_VMULAS_S8_QACC_LD_IP_QUP,
                (0, 1, 1, true) => Opcode::OPCODE_EE_VMULAS_S8_QACC_LD_XP_QUP,
                (0, 0, 1, false) => Opcode::OPCODE_EE_VMULAS_S16_QACC_LD_IP_QUP,
                (0, 0, 1, true) => Opcode::OPCODE_EE_VMULAS_S16_QACC_LD_XP_QUP,
                (1, 1, 1, false) => Opcode::OPCODE_EE_VMULAS_U8_QACC_LD_IP_QUP,
                (1, 1, 1, true) => Opcode::OPCODE_EE_VMULAS_U8_QACC_LD_XP_QUP,
                (1, 0, 1, false) => Opcode::OPCODE_EE_VMULAS_U16_QACC_LD_IP_QUP,
                (1, 0, 1, true) => Opcode::OPCODE_EE_VMULAS_U16_QACC_LD_XP_QUP,
                _ => Opcode::OPCODE_EE_UNIMPLEMENTED,
            };
        }
    }
    // Radix-2 butterfly (3-byte; b1[3:0] == 4 distinguishes it from
    // vzip/src.q/slci which share b0/b2 nibbles).
    if b0 & 0x0F == 4 && b1 & 0x0F == 4 && b2 & 0x0F == 0x0C {
        return Opcode::OPCODE_EE_FFT_R2BF_S16;
    }
    // FFT AMS + load (b3 top 5 bits 11010; mode in b3[3:1]: 000 =
    // incp, 010 = incp.uaup (unaligned update), 100 = r32.decp;
    // qz1[2] rides b3[0]. Other modes trap loud. Matched explicitly
    // (not masked) so the DC/DD cmul-ld.xp words sharing the D-page
    // keep routing below.
    if ((b3 == 0xD0 || b3 == 0xD1) || (b3 == 0xD4 || b3 == 0xD5) || (b3 == 0xD8 || b3 == 0xD9))
        && (b0 & 0x0E) == 0x0E
    {
        return match (b3 >> 1) & 7 {
            0 => Opcode::OPCODE_EE_FFT_AMS_S16_LD_INCP,
            2 => Opcode::OPCODE_EE_FFT_AMS_S16_LD_INCP_UAUP,
            4 => Opcode::OPCODE_EE_FFT_AMS_S16_LD_R32_DECP,
            _ => Opcode::OPCODE_EE_UNIMPLEMENTED,
        };
    }
    // FFT AMS store (b3[7:2] == 101000; sel2 and qz1[2:1] live in
    // b3[3:0]; disjoint from cmul.st's 10101x page and qup below).
    if (b3 & 0xFC) == 0xA0 && (b0 & 0x0E) == 0x0E {
        return Opcode::OPCODE_EE_FFT_AMS_S16_ST_INCP;
    }
    // Fused complex-multiply + load (b3 top 6 bits 110111).
    if (b3 & 0xFC) == 0xDC && (b0 & 0x0E) == 0x0E {
        return Opcode::OPCODE_EE_FFT_CMUL_S16_LD_XP;
    }
    // Fused complex-multiply store (b3 top 5 bits 10101).
    if (b3 & 0xF8) == 0xA8 && (b0 & 0x0E) == 0x0E {
        return Opcode::OPCODE_EE_FFT_CMUL_S16_ST_XP;
    }
    if b3 == 0xE0 && (b0 & 0x0E) == 0x0E && (b1 & 0x0E) == 0x00 {
        return Opcode::OPCODE_EE_SRC_Q_LD_IP;
    }
    // NOTE: the fused-ST E8 row (vmul.u8/vsubs.*, b1 low nibble 2/3) is
    // checked above, so reaching here with b3 == 0xE8 means the ld.xp
    // form (b1 low nibble 0/1/2/3 with b2[3] clear, e.g. KAT word
    // 0xE820432E with b1 = 0x43); the fused-ST row always sets b2[3],
    // so testing b2[3] alone keeps the two E8 families disjoint.
    if b3 == 0xE8 && (b0 & 0x0E) == 0x0E && (b2 & 0x08) == 0x00 {
        return Opcode::OPCODE_EE_SRC_Q_LD_XP;
    }
    // Fused butterfly-store (b2[3] set).
    if b3 == 0xE8 && (b0 & 0x0E) == 0x0E && (b2 & 0x08) == 0x08 {
        return Opcode::OPCODE_EE_FFT_R2BF_S16_ST_INCP;
    }
    // FFT reversed store (b1 nibble 3/7 separates it from movi/logic).
    if b0 & 0x0F == 4 && (b1 & 0x0F == 3 || b1 & 0x0F == 7) && (b2 & 0xCF) == 0xCD {
        return Opcode::OPCODE_EE_FFT_VST_R32_DECP;
    }
    if b0 & 0x0F == 4 && b2 & 0x0F == 6 && b2 & 0xC0 != 0 {
        if b2 & 0x40 == 0 {
            return Opcode::OPCODE_EE_SLCXXP_2Q;
        }
        return Opcode::OPCODE_EE_SRCXXP_2Q;
    }
    // Vector-MAC broadcast-load (3-byte plain + 4-byte .qup tail;
    // b2[3:0] == 7 separates both from vzip which shares the b0/b1
    // nibbles; the .qup tail lives on the 0x20/0x30 b3 page with the
    // slide operands).
    if b0 & 0x0F == 4 && b1 & 0x0F == 3 && b2 & 0x0F == 7 && b2 & 0x80 == 0x80 {
        return match ((b2 >> 6) & 1, (b2 >> 5) & 1) {
            (0, 1) => Opcode::OPCODE_EE_VMULAS_S8_QACC_LDBC_INCP,
            (0, 0) => Opcode::OPCODE_EE_VMULAS_S16_QACC_LDBC_INCP,
            (1, 1) => Opcode::OPCODE_EE_VMULAS_U8_QACC_LDBC_INCP,
            _ => Opcode::OPCODE_EE_VMULAS_U16_QACC_LDBC_INCP,
        };
    }
    // Vector-MAC broadcast-load + Q-slide (4-byte .qup tail sharing
    // the fused page: same E0 b3 as the fused loads, selected by
    // b2 = 0x56 with the width in b1[1:0] (GAS-probed 2026-09-14:
    // e8=s16 e9=s8 ea=u16 eb=u8, i.e. b1[1]=0 signed / 1 unsigned,
    // b1[0]=0 16-bit / 1 8-bit), b1[3] const. Placed before the
    // generic fused-ld rule below, which would otherwise claim b1
    // low nibble 0x8/0x9 as vmulas-fused.
    // NOTE: mem-qu rides b3[1:0] here too (GAS-probed: qu=q0/q2/q4/
    // q6 -> b3=E0/E1/E2/E3), so the page spans E0-E3 like the fused
    // loads above (same trap class as the vmax.s16.ld.incp miss).
    if (b3 & 0xFC) == 0xE0 && b2 == 0x56 && (b1 & 0xF0) == 0xE0 && (b0 & 0x0E) == 0x0E {
        return match ((b1 >> 1) & 1, b1 & 1) {
            (0, 0) => Opcode::OPCODE_EE_VMULAS_S16_QACC_LDBC_INCP_QUP,
            (0, 1) => Opcode::OPCODE_EE_VMULAS_S8_QACC_LDBC_INCP_QUP,
            (1, 0) => Opcode::OPCODE_EE_VMULAS_U16_QACC_LDBC_INCP_QUP,
            _ => Opcode::OPCODE_EE_VMULAS_U8_QACC_LDBC_INCP_QUP,
        };
    }
    // All format_32 families execute (218/218 opcodes, incl. srs.accx);
    // unrecognized patterns fall through to OPCODE_EE_UNIMPLEMENTED.
    Opcode::OPCODE_EE_UNIMPLEMENTED
}

/// Format_32 recognizer hook used by `decode_inst`'s catch-all. Returns the
/// best-known `OPCODE_EE_*` for a `format_32` TIE instruction, defaulting to
/// `OPCODE_EE_UNIMPLEMENTED`.
pub fn decode_ee_or_unimplemented(insn: u32) -> Opcode {
    decode_ee(insn)
}

// ---------------------------------------------------------------------------
// TIE execution (ee.* DSP/AI units).
//
// Semantics mirror espressif/qemu
// target/xtensa/translate_tie_esp32s3.c helpers (reference only; every
// behavior below is pinned by a known-answer test with independently
// computed expectations). Operand positions were mapped empirically with
// xtensa-esp32s3-elf-as: the CAL_DOUBLE_Q triple shares one layout across
// families — qx = raw[10:8], qy = raw[14] ++ raw[12:11] (qy[2] split out),
// qz = raw[15] ++ raw[21:20] (qz[0] split out); single-Q destinations reuse
// the qz position. AR/immediate positions vary per family and are derived
// the same way (see each family's test).
// ---------------------------------------------------------------------------

/// CAL_DOUBLE_Q Q-register triple shared by the compute families.
fn cal_q(raw: u32) -> (usize, usize, usize) {
    let qx = ((raw >> 8) & 7) as usize;
    let qy = ((((raw >> 14) & 1) << 2) | ((raw >> 11) & 3)) as usize;
    let qz = (((raw >> 15) & 1) | (((raw >> 20) & 3) << 1)) as usize;
    (qz, qx, qy)
}

/// Lane width selected by the mnemonic suffix.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Width {
    S8,
    U8,
    S16,
    U16,
    S32,
    U32,
}

fn width_of(name: &str) -> Option<Width> {
    if name.contains("_s8") {
        Some(Width::S8)
    } else if name.contains("_u8") {
        Some(Width::U8)
    } else if name.contains("_s16") {
        Some(Width::S16)
    } else if name.contains("_u16") {
        Some(Width::U16)
    } else if name.contains("_s32") {
        Some(Width::S32)
    } else if name.contains("_u32") {
        Some(Width::U32)
    } else {
        None
    }
}

fn lanes(w: Width) -> usize {
    match w {
        Width::S8 | Width::U8 => 16,
        Width::S16 | Width::U16 => 8,
        Width::S32 | Width::U32 => 4,
    }
}

/// Asymmetric signed saturation (QEMU vadds/vsubs helpers clamp to
/// -0x7f/-0x7fff/-0x7fffffff, NOT the symmetric minimums).
fn sat_s(v: i64, w: Width) -> i64 {
    let (lo, hi) = match w {
        Width::S8 => (-0x7f, 0x7f),
        Width::S16 => (-0x7fff, 0x7fff),
        Width::S32 => (-0x7fffffff, 0x7fffffff),
        _ => unreachable!(),
    };
    v.clamp(lo, hi)
}

fn sat_u(v: u64, w: Width) -> u64 {
    let hi = match w {
        Width::U8 => 0xff,
        Width::U16 => 0xffff,
        Width::U32 => 0xffff_ffff,
        _ => unreachable!(),
    };
    v.min(hi)
}

/// Vector multiply-accumulate into ACCX (QEMU `vmulas_accx_s3`): dot
/// product of Q[qx], Q[qy] added to the 40-bit ACCX with saturation
/// (signed ±0x7FFFFFFFFF, unsigned [0, 0xFFFFFFFFFF]).
fn ee_vmulas_accx(cpu: &mut Cpu, name: &str, raw: u32) {
    let (_, qx, qy) = cal_q(raw);
    let w = width_of(name).unwrap();
    match w {
        Width::S8 => {
            for i in 0..16 {
                cpu.accx += q_s8(cpu, qx, i) as i64 * q_s8(cpu, qy, i) as i64;
            }
            cpu.accx = cpu.accx.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF);
        }
        Width::U8 => {
            for i in 0..16 {
                cpu.accx += q_u8(cpu, qx, i) as i64 * q_u8(cpu, qy, i) as i64;
            }
            cpu.accx = cpu.accx.clamp(0, 0x00FF_FFFF_FFFF);
        }
        Width::S16 => {
            for i in 0..8 {
                cpu.accx += q_s16(cpu, qx, i) as i64 * q_s16(cpu, qy, i) as i64;
            }
            cpu.accx = cpu.accx.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF);
        }
        Width::U16 => {
            for i in 0..8 {
                cpu.accx += q_u16(cpu, qx, i) as i64 * q_u16(cpu, qy, i) as i64;
            }
            cpu.accx = cpu.accx.clamp(0, 0x00FF_FFFF_FFFF);
        }
        _ => unreachable!(),
    }
}

/// Saturating vector add/sub (QEMU `vadds_s3`/`vsubs_s3`, asymmetric
/// signed saturation).
fn ee_valu(cpu: &mut Cpu, name: &str, raw: u32, is_min: bool, is_max: bool, is_sub: bool) {
    let (qz, qx, qy) = cal_q(raw);
    let w = width_of(name).unwrap();
    let n = lanes(w);
    for i in 0..n {
        match w {
            Width::S8 => {
                let (a, b) = (q_s8(cpu, qx, i) as i64, q_s8(cpu, qy, i) as i64);
                let r = if is_sub { a - b } else { a + b };
                let r = if is_min {
                    a.min(b)
                } else if is_max {
                    a.max(b)
                } else {
                    sat_s(r, w)
                };
                set_q_u8(cpu, qz, i, r as u8);
            }
            Width::U8 => {
                // QEMU vsubs_u8: uint16_t result = a - b; borrow wraps
                // above 0xff so it clamps to 0xff (saturates, NOT wrap).
                let (a, b) = (q_u8(cpu, qx, i) as u64, q_u8(cpu, qy, i) as u64);
                let r = if is_min {
                    a.min(b)
                } else if is_max {
                    a.max(b)
                } else if is_sub {
                    if b > a { 0xff } else { a - b }
                } else {
                    sat_u(a + b, w)
                };
                set_q_u8(cpu, qz, i, r as u8);
            }
            Width::S16 => {
                let (a, b) = (q_s16(cpu, qx, i) as i64, q_s16(cpu, qy, i) as i64);
                let r = if is_sub { a - b } else { a + b };
                let r = if is_min {
                    a.min(b)
                } else if is_max {
                    a.max(b)
                } else {
                    sat_s(r, w)
                };
                set_q_u16(cpu, qz, i, r as u16);
            }
            Width::U16 => {
                // QEMU vsubs_u16: borrow wraps above 0xffff -> clamps.
                let (a, b) = (q_u16(cpu, qx, i) as u64, q_u16(cpu, qy, i) as u64);
                let r = if is_min {
                    a.min(b)
                } else if is_max {
                    a.max(b)
                } else if is_sub {
                    if b > a { 0xffff } else { a - b }
                } else {
                    sat_u(a + b, w)
                };
                set_q_u16(cpu, qz, i, r as u16);
            }
            Width::S32 => {
                let (a, b) = (
                    q_u32(cpu, qx, i) as i32 as i64,
                    q_u32(cpu, qy, i) as i32 as i64,
                );
                let r = if is_sub { a - b } else { a + b };
                let r = if is_min {
                    a.min(b)
                } else if is_max {
                    a.max(b)
                } else {
                    sat_s(r, w)
                };
                set_q_u32(cpu, qz, i, r as u32);
            }
            Width::U32 => {
                // QEMU vsubs_u32: borrow wraps above 0xffffffff -> clamps.
                let (a, b) = (q_u32(cpu, qx, i) as u64, q_u32(cpu, qy, i) as u64);
                let r = if is_min {
                    a.min(b)
                } else if is_max {
                    a.max(b)
                } else if is_sub {
                    if b > a { 0xffff_ffff } else { a - b }
                } else {
                    sat_u(a + b, w)
                };
                set_q_u32(cpu, qz, i, r as u32);
            }
        }
    }
}

/// Little-endian u32 word `n` (0..5) of a 20-byte quarter-round
/// accumulator (QEMU `ACCQ_reg.u8`, indexed like `RUR_QACC_H/L_n`).
#[inline]
pub fn qacc_word(accq: &[u8; 20], n: usize) -> u32 {
    u32::from_le_bytes(accq[4 * n..4 * n + 4].try_into().unwrap())
}

/// Write little-endian u32 word `n` (0..5) of a quarter-round accumulator.
#[inline]
pub fn set_qacc_word(accq: &mut [u8; 20], n: usize, v: u32) {
    accq[4 * n..4 * n + 4].copy_from_slice(&v.to_le_bytes());
}

/// Q-register lane accessors (QEMU `Q_reg` union, little-endian).
#[inline]
pub fn q_u8(cpu: &Cpu, q: usize, i: usize) -> u8 {
    cpu.qregs[q & 7][i & 15]
}
#[inline]
pub fn q_s8(cpu: &Cpu, q: usize, i: usize) -> i8 {
    cpu.qregs[q & 7][i & 15] as i8
}
#[inline]
pub fn set_q_u8(cpu: &mut Cpu, q: usize, i: usize, v: u8) {
    cpu.qregs[q & 7][i & 15] = v;
}
#[inline]
pub fn q_u16(cpu: &Cpu, q: usize, i: usize) -> u16 {
    u16::from_le_bytes([
        cpu.qregs[q & 7][2 * (i & 7)],
        cpu.qregs[q & 7][2 * (i & 7) + 1],
    ])
}
#[inline]
pub fn q_s16(cpu: &Cpu, q: usize, i: usize) -> i16 {
    q_u16(cpu, q, i) as i16
}
#[inline]
pub fn set_q_u16(cpu: &mut Cpu, q: usize, i: usize, v: u16) {
    let b = v.to_le_bytes();
    cpu.qregs[q & 7][2 * (i & 7)] = b[0];
    cpu.qregs[q & 7][2 * (i & 7) + 1] = b[1];
}
#[inline]
pub fn q_u32(cpu: &Cpu, q: usize, i: usize) -> u32 {
    u32::from_le_bytes(
        cpu.qregs[q & 7][4 * (i & 3)..4 * (i & 3) + 4]
            .try_into()
            .unwrap(),
    )
}
#[inline]
pub fn set_q_u32(cpu: &mut Cpu, q: usize, i: usize, v: u32) {
    cpu.qregs[q & 7][4 * (i & 3)..4 * (i & 3) + 4].copy_from_slice(&v.to_le_bytes());
}

/// Bit-field lane accessors for the 20-byte QACC halves (QEMU
/// `load_qacc`/`save_qacc20`/`save_qacc40`: 20-bit lanes packed LE at
/// 20*i bits with sign extension from bit 19; 40-bit lanes at 40*i
/// bits with sign extension from bit 39).
fn acc_bits(accq: &[u8; 20], bit_off: usize, bits: u32) -> u64 {
    let mut v: u64 = 0;
    for i in 0..bits {
        let b = bit_off + i as usize;
        if b < 160 && (accq[b / 8] >> (b % 8)) & 1 == 1 {
            v |= 1 << i;
        }
    }
    v
}

fn set_acc_bits(accq: &mut [u8; 20], bit_off: usize, bits: u32, v: u64) {
    for i in 0..bits {
        let b = bit_off + i as usize;
        if b < 160 {
            if (v >> i) & 1 == 1 {
                accq[b / 8] |= 1 << (b % 8);
            } else {
                accq[b / 8] &= !(1 << (b % 8));
            }
        }
    }
}

pub fn acc_s20(cpu: &Cpu, acc: usize, i: usize) -> i64 {
    let v = acc_bits(&cpu.accq[acc & 1], 20 * (i & 7), 20);
    ((v << 44) as i64) >> 44
}

pub fn set_acc_s20(cpu: &mut Cpu, acc: usize, i: usize, v: i64) {
    set_acc_bits(
        &mut cpu.accq[acc & 1],
        20 * (i & 7),
        20,
        v as u64 & 0x000F_FFFF,
    );
}

pub fn acc_u20(cpu: &Cpu, acc: usize, i: usize) -> u64 {
    acc_bits(&cpu.accq[acc & 1], 20 * (i & 7), 20)
}

pub fn set_acc_u20(cpu: &mut Cpu, acc: usize, i: usize, v: u64) {
    set_acc_bits(&mut cpu.accq[acc & 1], 20 * (i & 7), 20, v & 0x000F_FFFF);
}

pub fn acc_s40(cpu: &Cpu, acc: usize, i: usize) -> i64 {
    let v = acc_bits(&cpu.accq[acc & 1], 40 * (i & 3), 40);
    ((v << 24) as i64) >> 24
}

pub fn set_acc_s40(cpu: &mut Cpu, acc: usize, i: usize, v: i64) {
    set_acc_bits(
        &mut cpu.accq[acc & 1],
        40 * (i & 3),
        40,
        v as u64 & 0xFF_FFFF_FFFF,
    );
}

pub fn acc_u40(cpu: &Cpu, acc: usize, i: usize) -> u64 {
    acc_bits(&cpu.accq[acc & 1], 40 * (i & 3), 40)
}

pub fn set_acc_u40(cpu: &mut Cpu, acc: usize, i: usize, v: u64) {
    set_acc_bits(&mut cpu.accq[acc & 1], 40 * (i & 3), 40, v & 0xFF_FFFF_FFFF);
}

/// Single-Q operand (movi/zero/vrelu/vldbc form): raw[15] ++ raw[20] ++
/// raw[21] (NOTE the bit order differs from CAL qy/qz).
fn qu(raw: u32) -> usize {
    (((raw >> 15) & 1) | (((raw >> 20) & 1) << 1) | (((raw >> 21) & 1) << 2)) as usize
}

/// AR operand at the t position (raw[7:4]).
fn ar_t(raw: u32) -> u32 {
    (raw >> 4) & 0xF
}

fn std_sar(cpu: &Cpu) -> u32 {
    cpu.sreg(crate::cpu::SR_SAR) & 0x3F
}

/// Shifted-narrow helper: (product >> sar) truncated to the lane type
/// (QEMU vmul/calu helpers shift the widened product, then truncate).
fn shr_trunc_i16(p: i32, sar: u32) -> i16 {
    (p.wrapping_shr(sar.min(31))) as i16
}

/// Vector multiply with SAR shift (QEMU `vmul_s3`, SAR = standard SAR;
/// plain truncation, NO saturation).
fn ee_vmul(cpu: &mut Cpu, name: &str, raw: u32) {
    let (qz, qx, qy) = cal_q(raw);
    let w = width_of(name).unwrap();
    let sar = std_sar(cpu);
    match w {
        Width::S8 => {
            for i in 0..16 {
                let p = q_s8(cpu, qx, i) as i32 * q_s8(cpu, qy, i) as i32;
                set_q_u8(cpu, qz, i, shr_trunc_i16(p, sar) as u8);
            }
        }
        Width::U8 => {
            for i in 0..16 {
                let p = q_u8(cpu, qx, i) as u32 * q_u8(cpu, qy, i) as u32;
                set_q_u8(cpu, qz, i, p.wrapping_shr(sar.min(31)) as u8);
            }
        }
        Width::S16 => {
            for i in 0..8 {
                let p = q_s16(cpu, qx, i) as i32 * q_s16(cpu, qy, i) as i32;
                set_q_u16(cpu, qz, i, p.wrapping_shr(sar.min(31)) as u16);
            }
        }
        Width::U16 => {
            for i in 0..8 {
                let p = q_u16(cpu, qx, i) as u32 * q_u16(cpu, qy, i) as u32;
                set_q_u16(cpu, qz, i, p.wrapping_shr(sar.min(31)) as u16);
            }
        }
        _ => unreachable!(),
    }
}

/// Vector compare (QEMU `vcmp_s3`): lane mask (all-ones / 0), eq/lt/gt.
/// Sources use the CAL positions; the destination uses the single-Q
/// position (raw[15] ++ raw[20] ++ raw[21]).
fn ee_vcmp(cpu: &mut Cpu, name: &str, raw: u32, lt: bool, gt: bool) {
    let (_, qx, qy) = cal_q(raw);
    let qz = qu(raw);
    let w = width_of(name).unwrap();
    match w {
        Width::S8 | Width::U8 => {
            for i in 0..16 {
                let (a, b) = (q_s8(cpu, qx, i), q_s8(cpu, qy, i));
                let hit = a == b || (lt && a < b) || (gt && a > b);
                set_q_u8(cpu, qz, i, if hit { 0xFF } else { 0 });
            }
        }
        Width::S16 | Width::U16 => {
            for i in 0..8 {
                let (a, b) = (q_s16(cpu, qx, i), q_s16(cpu, qy, i));
                let hit = a == b || (lt && a < b) || (gt && a > b);
                set_q_u16(cpu, qz, i, if hit { 0xFFFF } else { 0 });
            }
        }
        Width::S32 | Width::U32 => {
            for i in 0..4 {
                let (a, b) = (q_u32(cpu, qx, i) as i32, q_u32(cpu, qy, i) as i32);
                let hit = a == b || (lt && a < b) || (gt && a > b);
                set_q_u32(cpu, qz, i, if hit { 0xFFFF_FFFF } else { 0 });
            }
        }
    }
}

/// Bitwise vector logic (QEMU `bw_logic_s3`): and/or/xor/not over Q regs.
/// Rotated operand layout: qx = raw[4]++raw[6]++raw[7], qy =
/// raw[14]++raw[10]++raw[11], qz = single-Q position.
fn ee_logic(cpu: &mut Cpu, raw: u32, op: u8) {
    let qx = (((raw >> 4) & 1) | (((raw >> 6) & 1) << 1) | (((raw >> 7) & 1) << 2)) as usize;
    let qy = (((raw >> 5) & 1) | (((raw >> 10) & 1) << 1) | (((raw >> 11) & 1) << 2)) as usize;
    let qz = qu(raw);
    for i in 0..16 {
        let (a, b) = (q_u8(cpu, qx, i), q_u8(cpu, qy, i));
        let r = match op {
            0 => a | b,
            1 => a & b,
            2 => a ^ b,
            _ => !a,
        };
        set_q_u8(cpu, qz, i, r);
    }
}

/// Interleave/deinterleave Q-register pairs in place (QEMU
/// `vzip_s3`/`vunzip_s3`); width selects lane size (0 = 8-bit, 1 = 16,
/// 2 = 32).
fn ee_zip(cpu: &mut Cpu, raw: u32, width: u8, unzip: bool) {
    let qs0 = ((raw >> 12) & 7) as usize;
    let qs1 = qu(raw);
    // Byte snapshots (QEMU copies both regs first, so aliasing can never
    // tear; GAS rejects qs0 == qs1 anyway).
    let sa = cpu.qregs[qs0 & 7];
    let sb = cpu.qregs[qs1 & 7];
    let (mut d0, mut d1) = (sa, sb);
    match (width, unzip) {
        (0, false) => {
            for i in 0..8 {
                d0[2 * i] = sa[i];
                d0[2 * i + 1] = sb[i];
                d1[2 * i] = sa[i + 8];
                d1[2 * i + 1] = sb[i + 8];
            }
        }
        (1, false) => {
            for i in 0..4 {
                for k in 0..2 {
                    d0[4 * i + k] = sa[2 * i + k];
                    d0[4 * i + 2 + k] = sb[2 * i + k];
                    d1[4 * i + k] = sa[8 + 2 * i + k];
                    d1[4 * i + 2 + k] = sb[8 + 2 * i + k];
                }
            }
        }
        (2, false) => {
            for i in 0..2 {
                for k in 0..4 {
                    d0[8 * i + k] = sa[4 * i + k];
                    d0[8 * i + 4 + k] = sb[4 * i + k];
                    d1[8 * i + k] = sa[8 + 4 * i + k];
                    d1[8 * i + 4 + k] = sb[8 + 4 * i + k];
                }
            }
        }
        (0, true) => {
            for i in 0..8 {
                d0[i] = sa[2 * i];
                d0[i + 8] = sb[2 * i];
                d1[i] = sa[2 * i + 1];
                d1[i + 8] = sb[2 * i + 1];
            }
        }
        (1, true) => {
            for i in 0..4 {
                for k in 0..2 {
                    d0[2 * i + k] = sa[4 * i + k];
                    d0[8 + 2 * i + k] = sb[4 * i + k];
                    d1[2 * i + k] = sa[4 * i + 2 + k];
                    d1[8 + 2 * i + k] = sb[4 * i + 2 + k];
                }
            }
        }
        (2, true) => {
            for i in 0..2 {
                for k in 0..4 {
                    d0[4 * i + k] = sa[8 * i + k];
                    d0[8 + 4 * i + k] = sb[8 * i + k];
                    d1[4 * i + k] = sa[8 * i + 4 + k];
                    d1[8 + 4 * i + k] = sb[8 * i + 4 + k];
                }
            }
        }
        _ => unreachable!(),
    }
    cpu.qregs[qs0 & 7] = d0;
    cpu.qregs[qs1 & 7] = d1;
}

/// Complex multiply of s16 pairs, low/high half, plain/conjugate
/// (QEMU `cmul_s3` op 0..3).
fn ee_cmul(cpu: &mut Cpu, raw: u32, op: u8, sar: u32) {
    let (qz, qx, qy) = cal_q(raw);
    let base = if op & 1 == 1 { 4 } else { 0 };
    let conj = op & 2 != 0;
    for p in 0..2 {
        let (ar, ai) = (
            q_s16(cpu, qx, base + 2 * p) as i32,
            q_s16(cpu, qx, base + 2 * p + 1) as i32,
        );
        let (br, bi) = (
            q_s16(cpu, qy, base + 2 * p) as i32,
            q_s16(cpu, qy, base + 2 * p + 1) as i32,
        );
        let (re, im) = if conj {
            (ar * br + ai * bi, ar * bi - ai * br)
        } else {
            (ar * br - ai * bi, ar * bi + ai * br)
        };
        set_q_u16(cpu, qz, base + 2 * p, re.wrapping_shr(sar.min(31)) as u16);
        set_q_u16(
            cpu,
            qz,
            base + 2 * p + 1,
            im.wrapping_shr(sar.min(31)) as u16,
        );
    }
}

/// FFT complex multiply + fused 128-bit load/store with pointer
/// increment (QEMU-analogous `cmul_s3` single-pair form + incp memory op;
/// GAS-captured operand map, see decode_ee): MAC on the sel-selected s16
/// pair first (sel = quadrant 0-3, standard non-conjugate form per the
/// ESP32-P4 PIE reference `real = (a.re*b.re-a.im*b.im)>>SAR`), then
/// mem128 into qu (ld) or qu into mem128 (st), then AR += 16.
/// Both forms: qz(MAC dest)=raw[18:16], AR=raw[7:4],
/// qx=raw[14]++raw[15]<<1++raw[0]<<2, qy=raw[23]++raw[12]<<1++raw[13]<<2,
/// sel=raw[9:8]. qu (mem reg) differs: ld qu[0]=raw[19], qu[2:1]=
/// raw[25:24]; st qu=raw[22:20]. (b2[3] is 0-const in st form, which is
/// what separates it from valu-st's b2[3]==1 const.)
fn ee_cmul_fused<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, is_st: bool) {
    let qu = if is_st {
        ((raw >> 20) & 7) as usize
    } else {
        ((((raw >> 19) & 1) | (((raw >> 24) & 1) << 1) | (((raw >> 25) & 1) << 2)) & 7) as usize
    };
    let qz = ((raw >> 16) & 7) as usize;
    let qx = ((((raw >> 14) & 1) | (((raw >> 15) & 1) << 1) | ((raw & 1) << 2)) & 7) as usize;
    let qy =
        ((((raw >> 23) & 1) | (((raw >> 12) & 1) << 1) | (((raw >> 13) & 1) << 2)) & 7) as usize;
    let sel = ((raw >> 8) & 3) as usize;
    let sar = std_sar(cpu).min(31);
    let base = sel * 2;
    let (ar, ai) = (q_s16(cpu, qx, base) as i32, q_s16(cpu, qx, base + 1) as i32);
    let (br, bi) = (q_s16(cpu, qy, base) as i32, q_s16(cpu, qy, base + 1) as i32);
    let (re, im) = (ar * br - ai * bi, ar * bi + ai * br);
    set_q_u16(cpu, qz, base, re.wrapping_shr(sar) as u16);
    set_q_u16(cpu, qz, base + 1, im.wrapping_shr(sar) as u16);
    let a = ar_t(raw);
    let abase = cpu.reg(a);
    let aligned = abase & !15;
    if is_st {
        let b = cpu.qregs[qu & 7];
        ee_st64(bus, aligned, u64::from_le_bytes(b[..8].try_into().unwrap()));
        ee_st64(
            bus,
            aligned.wrapping_add(8),
            u64::from_le_bytes(b[8..].try_into().unwrap()),
        );
    } else {
        let lo = ee_ld64(bus, aligned);
        let hi = ee_ld64(bus, aligned.wrapping_add(8));
        cpu.qregs[qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
        cpu.qregs[qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    }
    cpu.set_reg(a, abase.wrapping_add(16));
}

/// Zero Q / QACC / ACCX (QEMU `zero_s3`).
fn ee_zero(cpu: &mut Cpu, raw: u32, kind: u8) {
    match kind {
        0 => {
            let q = qu(raw);
            cpu.qregs[q & 7] = [0; 16];
        }
        1 => {
            cpu.accq[0] = [0; 20];
            cpu.accq[1] = [0; 20];
        }
        _ => cpu.accx = 0,
    }
}

/// Move AR to/from Q lanes (QEMU `movi_q_s3`/`movi_a_s3`).
fn ee_movi(cpu: &mut Cpu, raw: u32, sel: usize, to_q: bool) {
    let q = qu(raw);
    let a = ar_t(raw);
    if to_q {
        set_q_u32(cpu, q, sel, cpu.reg(a));
    } else {
        cpu.set_reg(a, q_u32(cpu, q, sel));
    }
}

/// Widen Q lanes into both QACC halves (QEMU `mov_qacc_s3`); size selects
/// u8 (0) / s8 (1) / u16 (2) / s16 (3) widening into 20/40-bit lanes.
fn ee_mov_qacc(cpu: &mut Cpu, raw: u32, size: u8) {
    let q = qu(raw);
    match size {
        0 => {
            for i in 0..8 {
                set_acc_u20(cpu, 0, i, q_u8(cpu, q, i) as u64);
                set_acc_u20(cpu, 1, i, q_u8(cpu, q, i + 8) as u64);
            }
        }
        1 => {
            for i in 0..8 {
                set_acc_s20(cpu, 0, i, q_s8(cpu, q, i) as i64);
                set_acc_s20(cpu, 1, i, q_s8(cpu, q, i + 8) as i64);
            }
        }
        2 => {
            for i in 0..4 {
                set_acc_u40(cpu, 0, i, q_u16(cpu, q, i) as u64);
                set_acc_u40(cpu, 1, i, q_u16(cpu, q, i + 4) as u64);
            }
        }
        _ => {
            for i in 0..4 {
                set_acc_s40(cpu, 0, i, q_s16(cpu, q, i) as i64);
                set_acc_s40(cpu, 1, i, q_s16(cpu, q, i + 4) as i64);
            }
        }
    }
}

/// Bit-reversed FFT indices with max() fold (QEMU `bitrev_s3`); also
/// post-increments the address register by 8.
fn ee_bitrev(cpu: &mut Cpu, raw: u32, qa: usize) {
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let w = cpu.fft_width;
    for i in 0..8 {
        let v = base.wrapping_add(i as u32);
        let mut r: u16 = 0;
        for k in 0..w {
            if (v >> k) & 1 == 1 {
                r |= 1 << (w - 1 - k);
            }
        }
        set_q_u16(cpu, qa, i, v.max(r as u32) as u16);
    }
    cpu.set_reg(a, base.wrapping_add(8));
}

/// Leaky ReLU in place (QEMU `vrelu_s3`): non-positive lanes scale by
/// (AR[ax] >> AR[ay]).
fn ee_vrelu(cpu: &mut Cpu, name: &str, raw: u32) {
    let q = qu(raw);
    let ax = cpu.reg((raw >> 8) & 0xF) as i32;
    let ay = cpu.reg(ar_t(raw)) & 0x1F;
    if name.contains("_s16") {
        for i in 0..8 {
            let v = q_s16(cpu, q, i);
            if v <= 0 {
                set_q_u16(
                    cpu,
                    q,
                    i,
                    (v as i32).wrapping_mul(ax).wrapping_shr(ay) as u16,
                );
            }
        }
    } else {
        for i in 0..16 {
            let v = q_s8(cpu, q, i);
            if v <= 0 {
                set_q_u8(cpu, q, i, (v as i32 * ax).wrapping_shr(ay) as u8);
            }
        }
    }
}

/// Predicated scale (QEMU `vprelu_s3`).
fn ee_vprelu(cpu: &mut Cpu, name: &str, raw: u32) {
    let (qz, _, _) = cal_q(raw);
    let qx = (((raw >> 8) & 1) | (((raw >> 9) & 3) << 1)) as usize;
    let qy = (((raw >> 11) & 1) | (((raw >> 12) & 1) << 1) | (((raw >> 14) & 1) << 2)) as usize;
    // ay is the last AR operand at the t position.
    let ay = cpu.reg(ar_t(raw)) & 0x1F;
    if name.contains("_s16") {
        for i in 0..8 {
            let v = q_s16(cpu, qx, i);
            if v <= 0 {
                let m = v as i32 * q_s16(cpu, qy, i) as i32;
                set_q_u16(cpu, qz, i, m.wrapping_shr(ay) as u16);
            } else {
                set_q_u16(cpu, qz, i, v as u16);
            }
        }
    } else {
        for i in 0..16 {
            let v = q_s8(cpu, qx, i);
            if v <= 0 {
                let m = v as i32 * q_s8(cpu, qy, i) as i32;
                set_q_u8(cpu, qz, i, m.wrapping_shr(ay) as u8);
            } else {
                set_q_u8(cpu, qz, i, v as u8);
            }
        }
    }
}

/// 32-bit lane shift by standard SAR (QEMU `vsx32_s3`); qa is at
/// raw[6:4], qs holds only even values (raw[21:20] << 1, GAS rejects odd).
fn ee_vsx(cpu: &mut Cpu, raw: u32, left: bool) {
    let qa = ((raw >> 4) & 7) as usize;
    let qs = (((raw >> 20) & 3) << 1) as usize;
    let sar = std_sar(cpu).min(31);
    for i in 0..4 {
        let v = q_u32(cpu, qs, i);
        let r = if left {
            v.wrapping_shl(sar)
        } else {
            (v as i32).wrapping_shr(sar) as u32
        };
        set_q_u32(cpu, qa, i, r);
    }
}

/// Vector-scalar MAC into QACC lanes (QEMU `vsmulas_s3`, NO shift).
fn ee_vsmulas(cpu: &mut Cpu, name: &str, raw: u32) {
    let qx = ((raw >> 8) & 7) as usize;
    let qy = ((raw >> 11) & 3) as usize;
    let sel = (((raw >> 15) & 1) | (((raw >> 20) & 1) << 1)) as usize;
    if name.contains("_s16") {
        let s = q_s16(cpu, qy, sel) as i64;
        for i in 0..4 {
            let lane = acc_s40(cpu, 0, i) + q_s16(cpu, qx, i) as i64 * s;
            set_acc_s40(cpu, 0, i, lane.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF));
            let lane = acc_s40(cpu, 1, i) + q_s16(cpu, qx, i + 4) as i64 * s;
            set_acc_s40(cpu, 1, i, lane.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF));
        }
    } else {
        let s = q_s8(cpu, qy, sel) as i64;
        for i in 0..8 {
            let lane = acc_s20(cpu, 0, i) + q_s8(cpu, qx, i) as i64 * s;
            set_acc_s20(cpu, 0, i, lane.clamp(-0x0007_FFFF, 0x0007_FFFF));
            let lane = acc_s20(cpu, 1, i) + q_s8(cpu, qx, i + 8) as i64 * s;
            set_acc_s20(cpu, 1, i, lane.clamp(-0x0007_FFFF, 0x0007_FFFF));
        }
    }
}

/// Shift QACC right by AR, saturate into Q (QEMU `srcmb_qacc_s3`,
/// asymmetric low clamps 0x80/0x8000); writes back the shifted QACC.
fn ee_srcmb(cpu: &mut Cpu, name: &str, raw: u32) {
    let q = qu(raw);
    let sh = cpu.reg(ar_t(raw)) & 0x3F;
    if name.contains("_s16") {
        for i in 0..4 {
            let v = acc_s40(cpu, 0, i) >> sh;
            set_acc_s40(cpu, 0, i, v);
            set_q_u16(
                cpu,
                q,
                i,
                if v > 0x7FFF {
                    0x7FFF
                } else if v < -0x7FFF {
                    0x8000
                } else {
                    v as u16
                },
            );
            let v = acc_s40(cpu, 1, i) >> sh;
            set_acc_s40(cpu, 1, i, v);
            set_q_u16(
                cpu,
                q,
                i + 4,
                if v > 0x7FFF {
                    0x7FFF
                } else if v < -0x7FFF {
                    0x8000
                } else {
                    v as u16
                },
            );
        }
    } else {
        for i in 0..8 {
            let v = acc_s20(cpu, 0, i) >> sh;
            set_acc_s20(cpu, 0, i, v);
            set_q_u8(
                cpu,
                q,
                i,
                if v > 0x7F {
                    0x7F
                } else if v < -0x7F {
                    0x80
                } else {
                    v as u8
                },
            );
            let v = acc_s20(cpu, 1, i) >> sh;
            set_acc_s20(cpu, 1, i, v);
            set_q_u8(
                cpu,
                q,
                i + 8,
                if v > 0x7F {
                    0x7F
                } else if v < -0x7F {
                    0x80
                } else {
                    v as u8
                },
            );
        }
    }
}

/// Vector MAC into QACC 20/40-bit lanes (QEMU `vmulas_qacc_s3`).
fn ee_vmulas_qacc(cpu: &mut Cpu, name: &str, raw: u32) {
    let (_, qx, qy) = cal_q(raw);
    let w = width_of(name).unwrap();
    match w {
        Width::S8 => {
            for i in 0..8 {
                let lane = acc_s20(cpu, 0, i) + q_s8(cpu, qx, i) as i64 * q_s8(cpu, qy, i) as i64;
                set_acc_s20(cpu, 0, i, lane.clamp(-0x0007_FFFF, 0x0007_FFFF));
                let lane =
                    acc_s20(cpu, 1, i) + q_s8(cpu, qx, i + 8) as i64 * q_s8(cpu, qy, i + 8) as i64;
                set_acc_s20(cpu, 1, i, lane.clamp(-0x0007_FFFF, 0x0007_FFFF));
            }
        }
        Width::U8 => {
            for i in 0..8 {
                for acc in 0..2 {
                    let off = if acc == 0 { i } else { i + 8 };
                    let v = acc_u20(cpu, acc, i)
                        .wrapping_add(q_u8(cpu, qx, off) as u64 * q_u8(cpu, qy, off) as u64);
                    // QEMU assigns -0xfffff on overflow (wraps to 1 mod 2^20).
                    set_acc_u20(
                        cpu,
                        acc,
                        i,
                        if v > 0x000F_FFFF {
                            (-0x000F_FFFF_i64) as u64
                        } else {
                            v
                        },
                    );
                }
            }
        }
        Width::S16 => {
            for i in 0..4 {
                for acc in 0..2 {
                    let off = if acc == 0 { i } else { i + 4 };
                    let v = acc_s40(cpu, acc, i)
                        + q_s16(cpu, qx, off) as i64 * q_s16(cpu, qy, off) as i64;
                    set_acc_s40(cpu, acc, i, v.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF));
                }
            }
        }
        Width::U16 => {
            for i in 0..4 {
                for acc in 0..2 {
                    let off = if acc == 0 { i } else { i + 4 };
                    let v = acc_u40(cpu, acc, i)
                        + q_u16(cpu, qx, off) as u64 * q_u16(cpu, qy, off) as u64;
                    set_acc_u40(cpu, acc, i, v.min(0xFF_FFFF_FFFF));
                }
            }
        }
        _ => unreachable!(),
    }
}

/// Shift-right-saturate ACCX (QEMU `srs_accx_s3`; the ESP32-P4 PIE twin
/// documents it as `ESP.SRS.S/U.XACC`): `ee.srs.accx rd, rs, sel`
/// shifts the 40-bit ACCX right by rs[5:0], writes the 40-bit result back,
/// and writes rd the saturated 32-bit result — signed (sel=0:
/// min(max(v, -2^31), 2^31-1)) or unsigned (sel=1: min(v, 2^32-1)).
///
/// GAS-probed operand layout (all 15 ARs x sel 0/1 encode distinctly, so
/// the old "shift-AR unencodable" note was simply wrong — nothing is
/// masked): rd = bits [11:8], rs = bits [7:4], sel = bit 14, i.e. word =
/// 0x7E1004 | rd<<8 | rs<<4 | sel<<14 (verified: a1,a2,0 = 0x7E1124,
/// a6,a9,0 = 0x7E1694, a1,a2,1 = 0x7E5124). Real-firmware usage: the
/// esp-nn/esp-dl quantized dot-product epilogue
/// (`movi a9, 0; ee.srs.accx a6, a9, 0` after `ee.vmulas.*.accx`).
fn ee_srs_accx(cpu: &mut Cpu, raw: u32) {
    let rd = (raw >> 8) & 0xF;
    let rs = (raw >> 4) & 0xF;
    let sel = (raw >> 14) & 1;
    let s = cpu.reg(rs) & 63;
    if sel == 0 {
        // Signed: sign-extend the 40-bit pattern, shift, keep signed (the
        // vmulas convention, so later accumulates stay sane).
        let v = (cpu.accx << 24) >> 24;
        let shifted = v >> s;
        cpu.accx = shifted;
        cpu.set_reg(rd, shifted.clamp(-0x8000_0000, 0x7FFF_FFFF) as u32);
    } else {
        // Unsigned: zero-extend the 40-bit pattern, shift, keep unsigned.
        let v = (cpu.accx as u64) & 0x00FF_FFFF_FFFF;
        let shifted = v >> s;
        cpu.accx = shifted as i64;
        cpu.set_reg(rd, shifted.min(0xFFFF_FFFF) as u32);
    }
}

/// GPIO output latch ops (QEMU `wr_mask_gpio_out_s3`); `ee.get_gpio_in`
/// reads the SoC-resolved dedicated-input channels (GPIO-matrix
/// CORE1_GPIO_IN0..7) via the bus, NOT the output latch.
///
/// GAS-probed operand layout (objdump prints the LE word, so KAT words are
/// the printed tokens verbatim; this family does NOT use the CAL triple):
/// - `ee.wr_mask_gpio_out data_ar, mask_ar`: data = AR at bits [7:4]
///   (first operand), mask = AR at bits [11:8] (second operand). Verified
///   by token sweep (`a2,a3` -> 0x724324, `a3,a2` -> 0x724234,
///   `a5,a3` -> 0x724354, `a2,a6` -> 0x724624; bits [23:20] stay 2): the
///   esp-idf dedic driver emits `a4, a3` = 0x724344 for value/mask. (An
///   earlier data/mask swap passed vacuously on the symmetric HIGH case
///   and stuck HIGH on the LOW write — real firmware caught it.)
/// - `ee.set/clr_bit_gpio_out imm`: 4-bit immediate at bits [7:4]
///   (verified: set 3 -> 0x754034, set 5 -> 0x754054, clr 9 -> 0x764094).
/// - `ee.get_gpio_in dest_ar`: dest = AR at bits [7:4] (verified:
///   a4/a2/a9 -> tokens 0x650844/0x650824/0x650894).
pub fn ee_gpio<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, kind: u8) {
    match kind {
        0 => {
            let data = cpu.reg(ar_t(raw));
            let mask = cpu.reg((raw >> 8) & 0xF);
            cpu.tie_gpio = (cpu.tie_gpio & !mask) | (data & mask);
        }
        1 => cpu.tie_gpio |= (raw >> 4) & 0xF,
        2 => cpu.tie_gpio &= !((raw >> 4) & 0xF),
        _ => {
            let a = ar_t(raw);
            cpu.set_reg(a, bus.dedic_gpio_in());
        }
    }
}

/// Top-level TIE/DSP execution: dispatch on the decoded opcode name,
/// deriving operands from the raw instruction word with the empirically
/// mapped positions (see helpers above). Returns true when handled;
/// false (unknown ee opcode) must keep the loud unimplemented trap.
pub fn exec_ee<B: Bus>(cpu: &mut Cpu, bus: &mut B, opc: Opcode, raw: u32) -> bool {
    let name = opc.name();
    if !name.starts_with("ee_") {
        return false;
    }
    // Fused vector-ALU + mem (must precede the plain arms below, whose
    // prefixes also match these names).
    if name == "ee_fft_r2bf_s16" {
        ee_r2bf(cpu, raw);
    } else if name == "ee_vadds_s8_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 0, Width::S8, false);
    } else if name == "ee_vsubs_s8_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 1, Width::S8, false);
    } else if name == "ee_vadds_s16_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 0, Width::S16, false);
    } else if name == "ee_vsubs_s16_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 1, Width::S16, false);
    } else if name == "ee_vadds_s32_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 0, Width::S32, false);
    } else if name == "ee_vmin_s8_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 2, Width::S8, false);
    } else if name == "ee_vmin_s16_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 2, Width::S16, false);
    } else if name == "ee_vmin_s32_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 2, Width::S32, false);
    } else if name == "ee_vmax_s8_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 3, Width::S8, false);
    } else if name == "ee_vmax_s16_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 3, Width::S16, false);
    } else if name == "ee_vmax_s32_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 3, Width::S32, false);
    } else if name == "ee_vsubs_s32_ld_incp" {
        ee_fused_valu(cpu, bus, raw, 1, Width::S32, false);
    } else if name == "ee_vmul_s8_ld_incp" {
        ee_fused_vmul(cpu, bus, raw, Width::S8, false);
    } else if name == "ee_vmul_s16_ld_incp" {
        ee_fused_vmul(cpu, bus, raw, Width::S16, false);
    } else if name == "ee_vmul_u8_ld_incp" {
        ee_fused_vmul(cpu, bus, raw, Width::U8, false);
    } else if name == "ee_vmul_u16_ld_incp" {
        ee_fused_vmul(cpu, bus, raw, Width::U16, false);
    } else if name == "ee_vadds_s8_st_incp" {
        ee_fused_valu(cpu, bus, raw, 0, Width::S8, true);
    } else if name == "ee_vadds_s16_st_incp" {
        ee_fused_valu(cpu, bus, raw, 0, Width::S16, true);
    } else if name == "ee_vadds_s32_st_incp" {
        ee_fused_valu(cpu, bus, raw, 0, Width::S32, true);
    } else if name == "ee_vsubs_s8_st_incp" {
        ee_fused_valu(cpu, bus, raw, 1, Width::S8, true);
    } else if name == "ee_vsubs_s16_st_incp" {
        ee_fused_valu(cpu, bus, raw, 1, Width::S16, true);
    } else if name == "ee_vsubs_s32_st_incp" {
        ee_fused_valu(cpu, bus, raw, 1, Width::S32, true);
    } else if name == "ee_vmin_s8_st_incp" {
        ee_fused_valu(cpu, bus, raw, 2, Width::S8, true);
    } else if name == "ee_vmin_s16_st_incp" {
        ee_fused_valu(cpu, bus, raw, 2, Width::S16, true);
    } else if name == "ee_vmin_s32_st_incp" {
        ee_fused_valu(cpu, bus, raw, 2, Width::S32, true);
    } else if name == "ee_vmax_s8_st_incp" {
        ee_fused_valu(cpu, bus, raw, 3, Width::S8, true);
    } else if name == "ee_vmax_s16_st_incp" {
        ee_fused_valu(cpu, bus, raw, 3, Width::S16, true);
    } else if name == "ee_vmax_s32_st_incp" {
        ee_fused_valu(cpu, bus, raw, 3, Width::S32, true);
    } else if name == "ee_vmul_s8_st_incp" {
        ee_fused_vmul(cpu, bus, raw, Width::S8, true);
    } else if name == "ee_vmul_s16_st_incp" {
        ee_fused_vmul(cpu, bus, raw, Width::S16, true);
    } else if name == "ee_vmul_u8_st_incp" {
        ee_fused_vmul(cpu, bus, raw, Width::U8, true);
    } else if name == "ee_vmul_u16_st_incp" {
        ee_fused_vmul(cpu, bus, raw, Width::U16, true);
    } else if name == "ee_fft_ams_s16_ld_incp" {
        ee_ams_ld(cpu, bus, raw);
    } else if name == "ee_fft_ams_s16_ld_incp_uaup" {
        ee_ams_ld_uaup(cpu, bus, raw);
    } else if name == "ee_fft_ams_s16_ld_r32_decp" {
        ee_ams_ld_r32_decp(cpu, bus, raw);
    } else if name == "ee_fft_ams_s16_st_incp" {
        ee_ams_st(cpu, bus, raw);
    } else if name.starts_with("ee_vmulas_") && name.contains("_ldbc_") {
        let w = if name.contains("_s8_") {
            Width::S8
        } else if name.contains("_s16_") {
            Width::S16
        } else if name.contains("_u8_") {
            Width::U8
        } else {
            Width::U16
        };
        // The 3-byte `.ldbc.incp` form (b0 low nibble 4) carries the CAL
        // triple; the 4-byte `.qup` form (b0 low nibble E/F, b3 0x20/0x30
        // page) aliases the CAL fields with its slide operands, so the
        // MAC decodes from the post-MAC positions there.
        if name.ends_with("_qup") {
            ee_vmulas_ldbc_qup(cpu, bus, raw, w);
        } else {
            ee_vmulas_ldbc(cpu, bus, raw, w);
        }
    } else if name.starts_with("ee_vmulas_") && name.ends_with("_qup") {
        let w = if name.contains("_s8_") {
            Width::S8
        } else if name.contains("_s16_") {
            Width::S16
        } else if name.contains("_u8_") {
            Width::U8
        } else {
            Width::U16
        };
        ee_vmulas_qup_fused(
            cpu,
            bus,
            raw,
            w,
            name.contains("_qacc_"),
            name.contains("_xp_"),
        );
    } else if name.starts_with("ee_vmulas_") && (name.contains("_ld_ip") || name.contains("_ld_xp"))
    {
        let w = if name.contains("_s8_") {
            Width::S8
        } else if name.contains("_s16_") {
            Width::S16
        } else if name.contains("_u8_") {
            Width::U8
        } else {
            Width::U16
        };
        ee_vmulas_fused(
            cpu,
            bus,
            raw,
            w,
            name.contains("_qacc_"),
            name.contains("_ld_xp"),
        );
    // Vector ALU (CAL triple).
    } else if (name.starts_with("ee_vadds_") || name.starts_with("ee_vsubs_"))
        && !name.contains("incp")
    {
        ee_valu(cpu, name, raw, false, false, name.contains("vsubs"));
    } else if name.starts_with("ee_vmin_") && !name.contains("incp") {
        ee_valu(cpu, name, raw, true, false, false);
    } else if name.starts_with("ee_vmax_") && !name.contains("incp") {
        ee_valu(cpu, name, raw, false, true, false);
    } else if name.starts_with("ee_vmul_") && !name.contains("incp") {
        ee_vmul(cpu, name, raw);
    } else if name.starts_with("ee_vcmp_eq_") {
        ee_vcmp(cpu, name, raw, false, false);
    } else if name.starts_with("ee_vcmp_lt_") {
        ee_vcmp(cpu, name, raw, true, false);
    } else if name.starts_with("ee_vcmp_gt_") {
        ee_vcmp(cpu, name, raw, false, true);
    } else if name == "ee_orq" {
        ee_logic(cpu, raw, 0);
    } else if name == "ee_andq" {
        ee_logic(cpu, raw, 1);
    } else if name == "ee_xorq" {
        ee_logic(cpu, raw, 2);
    } else if name == "ee_notq" {
        ee_logic(cpu, raw, 3);
    } else if name.starts_with("ee_vzip_") {
        let w = if name.ends_with("_8") {
            0
        } else if name.ends_with("_16") {
            1
        } else {
            2
        };
        ee_zip(cpu, raw, w, false);
    } else if name.starts_with("ee_vunzip_") {
        let w = if name.ends_with("_8") {
            0
        } else if name.ends_with("_16") {
            1
        } else {
            2
        };
        ee_zip(cpu, raw, w, true);
    } else if name == "ee_cmul_s16" {
        // op/sar-imm packing resolved per-opcode at wave-2 with the fused
        // cmul forms; plain cmul uses op 0, sar-imm at t.
        ee_cmul(cpu, raw, 0, ar_t(raw));
    } else if name == "ee_zero_q" {
        ee_zero(cpu, raw, 0);
    } else if name == "ee_zero_qacc" {
        ee_zero(cpu, raw, 1);
    } else if name == "ee_zero_accx" {
        ee_zero(cpu, raw, 2);
    } else if name == "ee_srs_accx" {
        ee_srs_accx(cpu, raw);
    } else if name == "ee_movi_32_q" {
        ee_movi(cpu, raw, ((raw >> 10) & 3) as usize, true);
    } else if name == "ee_movi_32_a" {
        ee_movi(cpu, raw, ((raw >> 10) & 3) as usize, false);
    } else if name.starts_with("ee_mov_") && name.ends_with("_qacc") {
        let size = match (raw >> 4) & 7 {
            7 => 0,
            3 => 1,
            6 => 2,
            _ => 3,
        };
        ee_mov_qacc(cpu, raw, size);
    } else if name == "ee_bitrev" {
        ee_bitrev(cpu, raw, qu(raw));
    } else if name.starts_with("ee_vrelu_") {
        ee_vrelu(cpu, name, raw);
    } else if name.starts_with("ee_vprelu_") {
        ee_vprelu(cpu, name, raw);
    } else if name == "ee_vsl_32" {
        ee_vsx(cpu, raw, true);
    } else if name == "ee_vsr_32" {
        ee_vsx(cpu, raw, false);
    } else if name == "ee_wr_mask_gpio_out" {
        ee_gpio(cpu, bus, raw, 0);
    } else if name == "ee_set_bit_gpio_out" {
        ee_gpio(cpu, bus, raw, 1);
    } else if name == "ee_clr_bit_gpio_out" {
        ee_gpio(cpu, bus, raw, 2);
    } else if name == "ee_get_gpio_in" {
        ee_gpio(cpu, bus, raw, 3);
    } else if name.starts_with("ee_vsmulas_") && !name.contains("incp") {
        ee_vsmulas(cpu, name, raw);
    } else if name == "ee_vsmulas_s8_qacc_ld_incp" {
        ee_vsmulas_fused(cpu, bus, raw, Width::S8);
    } else if name == "ee_vsmulas_s16_qacc_ld_incp" {
        ee_vsmulas_fused(cpu, bus, raw, Width::S16);
    } else if name.starts_with("ee_srcmb_") && !name.contains("incp") {
        ee_srcmb(cpu, name, raw);
    } else if name.starts_with("ee_vmulas_")
        && name.ends_with("_accx")
        && !name.contains("incp")
        && !name.contains("ldbc")
    {
        ee_vmulas_accx(cpu, name, raw);
    } else if name.starts_with("ee_vmulas_")
        && name.ends_with("_qacc")
        && !name.contains("incp")
        && !name.contains("ldbc")
    {
        ee_vmulas_qacc(cpu, name, raw);
    } else if name == "ee_slci_2q" {
        ee_sxci(cpu, raw, true);
    } else if name == "ee_srci_2q" {
        ee_sxci(cpu, raw, false);
    } else if name == "ee_slcxxp_2q" || name == "ee_srcxxp_2q" {
        ee_sxcxxp(cpu, raw);
    } else if name == "ee_src_q" {
        ee_src_q(cpu, raw, false);
    } else if name == "ee_src_q_qup" {
        ee_src_q(cpu, raw, true);
    } else if name.starts_with("ee_vldbc_") {
        ee_vldbc(cpu, bus, name, raw);
    } else if name.starts_with("ee_vldhbc_") {
        ee_vldhbc(cpu, bus, raw);
    } else if name.starts_with("ee_vld_") {
        ee_vld(cpu, bus, name, raw);
    } else if name.starts_with("ee_vst_") {
        ee_vst(cpu, bus, name, raw);
    } else if name == "ee_ldxq_32" {
        ee_ldstxq(cpu, bus, raw, false);
    } else if name == "ee_stxq_32" {
        ee_ldstxq(cpu, bus, raw, true);
    } else if name == "ee_ld_accx_ip" {
        ee_accx_mem(cpu, bus, raw, false);
    } else if name == "ee_st_accx_ip" {
        ee_accx_mem(cpu, bus, raw, true);
    } else if name.starts_with("ee_ld_qacc_") {
        ee_qacc_mem(cpu, bus, name, raw, false);
    } else if name.starts_with("ee_st_qacc_") {
        ee_qacc_mem(cpu, bus, name, raw, true);
    } else if name.starts_with("ee_ldqa_") {
        ee_ldqa(cpu, bus, name, raw);
    } else if name.starts_with("ee_ldf_64_") {
        ee_ldf(cpu, bus, raw, false, name.ends_with("_ip"));
    } else if name.starts_with("ee_stf_64_") {
        ee_ldf(cpu, bus, raw, true, name.ends_with("_ip"));
    } else if name.starts_with("ee_ldf_128_") {
        ee_ldf128(cpu, bus, raw, false, name.ends_with("_ip"));
    } else if name.starts_with("ee_stf_128_") {
        ee_ldf128(cpu, bus, raw, true, name.ends_with("_ip"));
    } else if name == "ee_ld_ua_state_ip" {
        ee_ua_mem(cpu, bus, raw, false);
    } else if name == "ee_st_ua_state_ip" {
        ee_ua_mem(cpu, bus, raw, true);
    } else if name.starts_with("ee_ld_128_usar_") {
        ee_ld_usar(cpu, bus, name, raw);
    } else if name == "ee_src_q_ld_ip" || name == "ee_src_q_ld_xp" {
        ee_src_q_ld(cpu, bus, name, raw);
    } else if name == "ee_srcq_128_st_incp" {
        ee_srcq_st(cpu, bus, raw);
    } else if name == "ee_fft_r2bf_s16_st_incp" {
        ee_r2bf_st(cpu, bus, raw);
    } else if name == "ee_fft_cmul_s16_ld_xp" {
        ee_cmul_ld(cpu, bus, raw);
    } else if name == "ee_fft_cmul_s16_st_xp" {
        ee_cmul_st(cpu, bus, raw);
    } else if name == "ee_cmul_s16_ld_incp" {
        ee_cmul_fused(cpu, bus, raw, false);
    } else if name == "ee_cmul_s16_st_incp" {
        ee_cmul_fused(cpu, bus, raw, true);
    } else if name == "ee_fft_vst_r32_decp" {
        ee_vst_decp(cpu, bus, raw);
    } else {
        return false;
    }
    true
}

/// 64/128-bit memory helpers (little-endian, composed of 32-bit accesses).
fn ee_ld64<B: Bus>(bus: &mut B, addr: u32) -> u64 {
    bus.read32(addr) as u64 | ((bus.read32(addr.wrapping_add(4)) as u64) << 32)
}

fn ee_st64<B: Bus>(bus: &mut B, addr: u32, v: u64) {
    bus.write32(addr, v as u32);
    bus.write32(addr.wrapping_add(4), (v >> 32) as u32);
}

/// Byte slide over a Q-register pair (QEMU `sxci_2q_s3`); shift = sar+1
/// (sar-imm at t, 0..7). First asm operand is qs1 (q-formula, even
/// {0,2}), second is qs0 (raw[14:12], odd {1,3}).
fn ee_sxci(cpu: &mut Cpu, raw: u32, left: bool) {
    let qs1 = qu(raw) & 7;
    let qs0 = ((raw >> 12) & 7) as usize & 7;
    let shift = ar_t(raw) + 1;
    let mut tmp = [0u8; 32];
    tmp[..16].copy_from_slice(&cpu.qregs[qs0]);
    tmp[16..].copy_from_slice(&cpu.qregs[qs1]);
    if left {
        for i in 0..16 {
            cpu.qregs[qs0][i] = if (i as u32) < shift {
                0
            } else {
                tmp[i - shift as usize]
            };
            cpu.qregs[qs1][i] = tmp[16 - shift as usize + i];
        }
    } else {
        for i in 0..16 {
            cpu.qregs[qs0][i] = tmp[i + shift as usize];
            cpu.qregs[qs1][i] = if (i as u32) < 16 - shift {
                tmp[i + shift as usize + 16]
            } else {
                0
            };
        }
    }
}

/// Q-register funnel shift by SAR_BYTE (QEMU `src_q_s3`); qa at t, qs0
/// at raw[14:12], qs1 at the single-Q position. `.qup` also copies
/// qs1 over qs0.
fn ee_src_q(cpu: &mut Cpu, raw: u32, qup: bool) {
    let qa = ar_t(raw) as usize;
    let qs0 = ((raw >> 12) & 7) as usize;
    let qs1 = qu(raw);
    let sar = (cpu.sar_byte as usize).min(16);
    let (sa, sb) = (cpu.qregs[qs0 & 7], cpu.qregs[qs1 & 7]);
    let mut tmp = [0u8; 16];
    tmp[..16 - sar].copy_from_slice(&sa[sar..16]);
    tmp[16 - sar..].copy_from_slice(&sb[..sar]);
    cpu.qregs[qa & 7] = tmp;
    if qup {
        cpu.qregs[qs0 & 7] = sb;
    }
}

/// Broadcast memory to all lanes (QEMU `vldbc_s3`); vec at the single-Q
/// position, address at t. Width from raw bits (positions vary by form:
/// plain uses b1[6]/b1[2], .ip uses b2[6]/b1[3], .xp uses b1[6]/b1[4]).
/// `.ip` post-increments by b1[4]<<4, `.xp` adds AR[b1[3:0]].
fn ee_vldbc<B: Bus>(cpu: &mut Cpu, bus: &mut B, name: &str, raw: u32) {
    let vec = qu(raw);
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let is_ip = name.ends_with("_ip");
    let is_xp = name.ends_with("_xp");
    let w8 = if is_ip {
        ((raw >> 22) & 1) == 1
    } else if is_xp {
        false
    } else {
        ((raw >> 14) & 1) == 0 && ((raw >> 10) & 1) == 0
    };
    // Resolve 8/16/32 from the form-specific code bits.
    let width: u8 = if is_ip {
        if ((raw >> 22) & 1) == 1 {
            0
        } else if ((raw >> 11) & 1) == 1 {
            1
        } else {
            2
        }
    } else if is_xp {
        if ((raw >> 14) & 1) == 0 {
            2
        } else if ((raw >> 12) & 1) == 1 {
            0
        } else {
            1
        }
    } else if w8 {
        0
    } else if ((raw >> 10) & 1) == 1 {
        2
    } else {
        1
    };
    match width {
        0 => {
            let v = bus.read8(base) as u8;
            cpu.qregs[vec & 7] = [v; 16];
        }
        1 => {
            let v = bus.read16(base) as u16;
            for i in 0..8 {
                set_q_u16(cpu, vec, i, v);
            }
        }
        _ => {
            let v = bus.read32(base);
            for i in 0..4 {
                set_q_u32(cpu, vec, i, v);
            }
        }
    }
    if is_ip {
        // Postupdate bit slides with width (b1[4] for 8-bit, b1[3] for
        // 16-bit, b1[2] for 32-bit).
        let bit = if width == 0 {
            4
        } else if width == 1 {
            3
        } else {
            2
        };
        cpu.set_reg(a, base.wrapping_add(((raw >> (8 + bit as u32)) & 1) << 4));
    } else if is_xp {
        let ax = cpu.reg((raw >> 8) & 0xF);
        cpu.set_reg(a, base.wrapping_add(ax));
    }
}

/// Indexed slide with AR shift amount and postupdate (QEMU `sxcxxp_2q`
/// via `sxci_2q_s3`); qs1 at the single-Q position, qs0 at raw[14:12],
/// shift-AR at t, postupdate-AR at raw[11:8]. Direction from raw[22].
fn ee_sxcxxp(cpu: &mut Cpu, raw: u32) {
    let qs1 = qu(raw) & 7;
    let qs0 = ((raw >> 12) & 7) as usize & 7;
    let a = ar_t(raw);
    let ax = (raw >> 8) & 0xF;
    let shift = ((cpu.reg(a) & 31) + 1).min(16);
    let mut tmp = [0u8; 32];
    tmp[..16].copy_from_slice(&cpu.qregs[qs0]);
    tmp[16..].copy_from_slice(&cpu.qregs[qs1]);
    let left = ((raw >> 22) & 1) == 0;
    if left {
        for i in 0..16 {
            cpu.qregs[qs0][i] = if (i as u32) < shift {
                0
            } else {
                tmp[i - shift as usize]
            };
            cpu.qregs[qs1][i] = tmp[16 - shift as usize + i];
        }
    } else {
        for i in 0..16 {
            cpu.qregs[qs0][i] = tmp[i + shift as usize];
            cpu.qregs[qs1][i] = if (i as u32) < 16 - shift {
                tmp[i + shift as usize + 16]
            } else {
                0
            };
        }
    }
    cpu.set_reg(a, cpu.reg(a).wrapping_add(cpu.reg(ax)));
}

/// 128/64-bit vector load/store with postupdate (QEMU `vld_128_s3` and
/// friends); qu at the single-Q position, address at t. `.ip` adds
/// b1[5:0]<<4 (128-bit) or b1[0]<<3 (64-bit); `.xp` adds AR[b1[3:0]].
fn ee_vld<B: Bus>(cpu: &mut Cpu, bus: &mut B, name: &str, raw: u32) {
    let vec = qu(raw);
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let wide = name.contains("128");
    let aligned = if wide { base & !15 } else { base & !7 };
    let lo = ee_ld64(bus, aligned);
    if wide {
        let hi = ee_ld64(bus, aligned.wrapping_add(8));
        cpu.qregs[vec & 7][..8].copy_from_slice(&lo.to_le_bytes());
        cpu.qregs[vec & 7][8..].copy_from_slice(&hi.to_le_bytes());
    } else if name.contains(".h.") || name.contains("_h_") {
        cpu.qregs[vec & 7][8..].copy_from_slice(&lo.to_le_bytes());
    } else {
        cpu.qregs[vec & 7][..8].copy_from_slice(&lo.to_le_bytes());
    }
    ee_post_update(cpu, name, raw, a, base, wide);
}

/// Vector store counterpart (QEMU `vst_64_s3` / `store_qreg_to_memory`).
fn ee_vst<B: Bus>(cpu: &mut Cpu, bus: &mut B, name: &str, raw: u32) {
    let vec = qu(raw);
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let wide = name.contains("128");
    let aligned = if wide { base & !15 } else { base & !7 };
    if wide {
        let lo = u64::from_le_bytes(cpu.qregs[vec & 7][..8].try_into().unwrap());
        let hi = u64::from_le_bytes(cpu.qregs[vec & 7][8..].try_into().unwrap());
        ee_st64(bus, aligned, lo);
        ee_st64(bus, aligned.wrapping_add(8), hi);
    } else if name.contains(".h.") || name.contains("_h_") {
        let v = u64::from_le_bytes(cpu.qregs[vec & 7][8..].try_into().unwrap());
        ee_st64(bus, aligned, v);
    } else {
        let v = u64::from_le_bytes(cpu.qregs[vec & 7][..8].try_into().unwrap());
        ee_st64(bus, aligned, v);
    }
    ee_post_update(cpu, name, raw, a, base, wide);
}

/// Shared address postupdate for the vld/vst family.
fn ee_post_update(cpu: &mut Cpu, name: &str, raw: u32, a: u32, base: u32, wide: bool) {
    // Opcode names use dots (ee.vld.128.ip); decoded names use underscores.
    let ip = name.contains(".ip") || name.contains("_ip");
    let xp = name.contains(".xp") || name.contains("_xp");
    if ip {
        let inc = if wide {
            ((raw >> 8) & 0x3F) << 4
        } else {
            ((raw >> 8) & 1) << 3
        };
        cpu.set_reg(a, base.wrapping_add(inc));
    } else if xp {
        let ax = cpu.reg((raw >> 8) & 0xF);
        cpu.set_reg(a, base.wrapping_add(ax));
    }
}

/// Indexed 32-bit Q-lane load/store (QEMU `ldxq_32_s3`/`stxq_32_s3`):
/// addr = (as + sext(Q[qs].s16[sel8])*4 - 4) & ~3.
fn ee_ldstxq<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, is_st: bool) {
    // ldxq: qu at raw[18:16]; stxq: qv at raw[21:20]. Both take qs
    // at raw[14]++raw[15]. sel4 = raw[19]++raw[24],
    // sel8 = raw[23]++raw[12].
    let (qd, qs) = if is_st {
        (
            ((raw >> 20) & 3) as usize,
            (((raw >> 14) & 1) | (((raw >> 15) & 1) << 1)) as usize,
        )
    } else {
        (
            ((raw >> 16) & 7) as usize,
            (((raw >> 14) & 1) | (((raw >> 15) & 1) << 1)) as usize,
        )
    };
    let a = ar_t(raw);
    let sel4 = (((raw >> 19) & 1) | (((raw >> 24) & 1) << 1)) as usize;
    let sel8 = (((raw >> 23) & 1) | (((raw >> 12) & 1) << 1)) as usize;
    let off = q_s16(cpu, qs, sel8) as i32;
    let addr = (cpu.reg(a).wrapping_add((off * 4 - 4) as u32)) & !3;
    if is_st {
        bus.write32(addr, q_u32(cpu, qd, sel4));
    } else {
        set_q_u32(cpu, qd, sel4, bus.read32(addr));
    }
}

/// 64-bit ACCX spill/fill (QEMU `ld_accx_s3`/`st_accx_s3`, 44-bit mask);
/// as at t, post-increment b1<<3.
fn ee_accx_mem<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, is_st: bool) {
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let addr = base & !7;
    if is_st {
        ee_st64(bus, addr, (cpu.accx as u64) & 0x0FFF_FFFF_FFFF);
    } else {
        cpu.accx = (ee_ld64(bus, addr) & 0x0FFF_FFFF_FFFF) as i64;
    }
    cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 0x3F) << 3));
}

/// QACC-half spill/fill; acc H/L from the name, position/size from the
/// name (`h_32` = top word, `l_128` = low 16 bytes). as at t.
fn ee_qacc_mem<B: Bus>(cpu: &mut Cpu, bus: &mut B, name: &str, raw: u32, is_st: bool) {
    // Decoded names look like ee_ld_qacc_h_h_32_ip.
    let acc = if name.contains("_h_h_") || name.contains("_h_l_") {
        1
    } else {
        0
    };
    let top_word = name.contains("_h_32");
    let a = ar_t(raw);
    let base = cpu.reg(a);
    if top_word {
        let addr = base & !3;
        if is_st {
            bus.write32(addr, qacc_word(&cpu.accq[acc], 4));
        } else {
            set_qacc_word(&mut cpu.accq[acc], 4, bus.read32(addr));
        }
        cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 0xF) << 2));
    } else {
        let aligned = base & !15;
        if is_st {
            for i in 0..2 {
                ee_st64(
                    bus,
                    aligned.wrapping_add(8 * i as u32),
                    ee_qacc_u64(&cpu.accq[acc], i),
                );
            }
        } else {
            for i in 0..2 {
                ee_set_qacc_u64(
                    &mut cpu.accq[acc],
                    i,
                    ee_ld64(bus, aligned.wrapping_add(8 * i as u32)),
                );
            }
        }
        cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 0x3F) << 4));
    }
}

fn ee_qacc_u64(accq: &[u8; 20], i: usize) -> u64 {
    u64::from_le_bytes(accq[8 * i..8 * i + 8].try_into().unwrap())
}

fn ee_set_qacc_u64(accq: &mut [u8; 20], i: usize, v: u64) {
    accq[8 * i..8 * i + 8].copy_from_slice(&v.to_le_bytes());
}

/// 128-bit QACC fill with lane widening (QEMU `ldqa_64_s3` into L/H);
/// as at t, post-increment b1<<4 (ip) or AR[b1[3:0]] (xp).
fn ee_ldqa<B: Bus>(cpu: &mut Cpu, bus: &mut B, name: &str, raw: u32) {
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let lo = ee_ld64(bus, aligned);
    let hi = ee_ld64(bus, aligned.wrapping_add(8));
    let s16 = name.contains("_s16");
    let s8 = name.contains("_s8");
    let w16 = name.contains("16");
    for (acc, data) in [(0, lo), (1, hi)] {
        let b = data.to_le_bytes();
        if w16 {
            for i in 0..4 {
                let v = u16::from_le_bytes([b[2 * i], b[2 * i + 1]]);
                if s16 {
                    set_acc_s40(cpu, acc, i, v as i16 as i64);
                } else {
                    set_acc_u40(cpu, acc, i, v as u64);
                }
            }
        } else {
            for (i, byte) in b.iter().enumerate() {
                if s8 {
                    set_acc_s20(cpu, acc, i, *byte as i8 as i64);
                } else {
                    set_acc_u20(cpu, acc, i, *byte as u64);
                }
            }
        }
    }
    if name.contains("_xp") {
        let ax = cpu.reg((raw >> 8) & 0xF);
        cpu.set_reg(a, base.wrapping_add(ax));
    } else {
        cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 0x3F) << 4));
    }
}

/// 64-bit FPR load/store with swapped halves (QEMU `ldf_64_ip`); fu0 at
/// b1[4], fu1 at b2[4], address at t. xp adds AR[b1[3:0]], ip adds
/// b1[0]<<3 (4-byte form).
fn ee_ldf<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, is_st: bool, is_ip: bool) {
    let fu0 = (raw >> 12) & 1;
    let fu1 = (raw >> 20) & 1;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let addr = base & !7;
    if is_st {
        bus.write32(addr, cpu.freg(fu1).to_bits());
        bus.write32(addr.wrapping_add(4), cpu.freg(fu0).to_bits());
    } else {
        cpu.set_freg(fu1, f32::from_bits(bus.read32(addr)));
        cpu.set_freg(fu0, f32::from_bits(bus.read32(addr.wrapping_add(4))));
    }
    if is_ip {
        cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 1) << 3));
    } else {
        let ax = cpu.reg((raw >> 8) & 0xF);
        cpu.set_reg(a, base.wrapping_add(ax));
    }
}

/// 128-bit FPR load/store (QEMU-analogous `ldf_128`/`stf_128_s3`, GAS-
/// captured FPR scatter): four singles move as two swapped pairs (the
/// ldf64 half-swap extended: mem[0..4]<->fb, mem[4..8]<->fa,
/// mem[8..12]<->fd, mem[12..16]<->fc), so stf128-then-ldf128 round-trips
/// exactly. `.ip` adds b1[3:0]<<4, `.xp` adds AR[b1[3:0]].
fn ee_ldf128<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, is_st: bool, is_ip: bool) {
    let fa = (((raw >> 24) & 7) << 1) | ((raw >> 19) & 1);
    let fb = (((raw >> 16) & 7) << 1) | (raw & 1);
    let fc = (raw >> 12) & 0xF;
    let fd = (raw >> 20) & 0xF;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    if is_st {
        bus.write32(aligned, cpu.freg(fb).to_bits());
        bus.write32(aligned.wrapping_add(4), cpu.freg(fa).to_bits());
        bus.write32(aligned.wrapping_add(8), cpu.freg(fd).to_bits());
        bus.write32(aligned.wrapping_add(12), cpu.freg(fc).to_bits());
    } else {
        cpu.set_freg(fb, f32::from_bits(bus.read32(aligned)));
        cpu.set_freg(fa, f32::from_bits(bus.read32(aligned.wrapping_add(4))));
        cpu.set_freg(fd, f32::from_bits(bus.read32(aligned.wrapping_add(8))));
        cpu.set_freg(fc, f32::from_bits(bus.read32(aligned.wrapping_add(12))));
    }
    if is_ip {
        cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 0xF) << 4));
    } else {
        let ax = cpu.reg((raw >> 8) & 0xF);
        cpu.set_reg(a, base.wrapping_add(ax));
    }
}

/// 128-bit UA_STATE spill/fill; as at t, post-increment b1<<4.
fn ee_ua_mem<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, is_st: bool) {
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    for i in 0..2 {
        if is_st {
            ee_st64(
                bus,
                aligned.wrapping_add(8 * i as u32),
                u64::from_le_bytes(cpu.ua_state[8 * i..8 * i + 8].try_into().unwrap()),
            );
        } else {
            let v = ee_ld64(bus, aligned.wrapping_add(8 * i as u32));
            cpu.ua_state[8 * i..8 * i + 8].copy_from_slice(&v.to_le_bytes());
        }
    }
    cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 0x3F) << 4));
}

/// 128-bit load that also snapshots the address low nibble into SAR_BYTE
/// (QEMU `ld_usar_128_s3`); otherwise like vld.128.
fn ee_ld_usar<B: Bus>(cpu: &mut Cpu, bus: &mut B, name: &str, raw: u32) {
    let vec = qu(raw);
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let lo = ee_ld64(bus, aligned);
    let hi = ee_ld64(bus, aligned.wrapping_add(8));
    cpu.qregs[vec & 7][..8].copy_from_slice(&lo.to_le_bytes());
    cpu.qregs[vec & 7][8..].copy_from_slice(&hi.to_le_bytes());
    cpu.sar_byte = (base & 0xF) as u8;
    if name.contains("_xp") {
        let ax = cpu.reg((raw >> 8) & 0xF);
        cpu.set_reg(a, base.wrapping_add(ax));
    } else {
        cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 0x3F) << 4));
    }
}

/// Halfword-broadcast pair load (QEMU `vldhbc_16_s3` into qu and qu1,
/// then as += 16); qu even-only at b2[5:4]<<1, qu1 at b1[7:4].
fn ee_vldhbc<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    let vec = (((raw >> 20) & 3) << 1) as usize;
    let vec1 = (((raw >> 12) & 15) & 7) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    for (v, off) in [(vec, 0), (vec1, 8)] {
        let data = ee_ld64(bus, aligned.wrapping_add(off));
        let b = data.to_le_bytes();
        for i in 0..4 {
            let h = u16::from_le_bytes([b[2 * i], b[2 * i + 1]]);
            set_q_u16(cpu, v, 2 * i, h);
            set_q_u16(cpu, v, 2 * i + 1, h);
        }
    }
    cpu.set_reg(a, base.wrapping_add(16));
}

/// Fused vector ALU + 128-bit mem transfer (QEMU `translate_vadds_s3`
/// and siblings with addr_inc16): `.ld.incp` loads mem128 into mem-qu
/// first, `.st.incp` stores mem-qu first, then the ALU runs, then
/// AR += 16. Operand layout (4-byte): mem-qu = raw[19]++raw[25:24], AR
/// = raw[7:4], qz = raw[18:16], qx = raw[14]++raw[15]++raw[0], qy =
/// raw[23]++raw[12]++raw[13]. (op,width) selects via (b1[1:0],b2[6:4]).
fn ee_fused_valu<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, op: u8, w: Width, is_st: bool) {
    // mem-qu is split (raw[19]++raw[25:24]) for loads, contiguous
    // raw[22:20] for stores.
    let mem_qu = if is_st {
        ((raw >> 20) & 7) as usize
    } else {
        (((raw >> 19) & 1) | (((raw >> 24) & 3) << 1)) as usize
    };
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    if is_st {
        let lo = u64::from_le_bytes(cpu.qregs[mem_qu & 7][..8].try_into().unwrap());
        let hi = u64::from_le_bytes(cpu.qregs[mem_qu & 7][8..].try_into().unwrap());
        ee_st64(bus, aligned, lo);
        ee_st64(bus, aligned.wrapping_add(8), hi);
    } else {
        let lo = ee_ld64(bus, aligned);
        let hi = ee_ld64(bus, aligned.wrapping_add(8));
        cpu.qregs[mem_qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
        cpu.qregs[mem_qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    }
    let qz = ((raw >> 16) & 7) as usize;
    let qx = (((raw >> 14) & 1) | (((raw >> 15) & 1) << 1) | (((raw) & 1) << 2)) as usize;
    let qy = (((raw >> 23) & 1) | (((raw >> 12) & 1) << 1) | (((raw >> 13) & 1) << 2)) as usize;
    ee_valu_fused_lanes(cpu, qz, qx, qy, op, w);
    cpu.set_reg(a, base.wrapping_add(16));
}

/// Lane loop shared by plain and fused saturating ALU (op 0 = add,
/// 1 = sub, 2 = min, 3 = max).
fn ee_valu_fused_lanes(cpu: &mut Cpu, qz: usize, qx: usize, qy: usize, op: u8, w: Width) {
    let n = lanes(w);
    for i in 0..n {
        match w {
            Width::S8 => {
                let (a, b) = (q_s8(cpu, qx, i) as i64, q_s8(cpu, qy, i) as i64);
                let r = match op {
                    0 => sat_s(a + b, w),
                    1 => sat_s(a - b, w),
                    2 => a.min(b),
                    _ => a.max(b),
                };
                set_q_u8(cpu, qz, i, r as u8);
            }
            Width::S16 => {
                let (a, b) = (q_s16(cpu, qx, i) as i64, q_s16(cpu, qy, i) as i64);
                let r = match op {
                    0 => sat_s(a + b, w),
                    1 => sat_s(a - b, w),
                    2 => a.min(b),
                    _ => a.max(b),
                };
                set_q_u16(cpu, qz, i, r as u16);
            }
            Width::S32 => {
                let (a, b) = (
                    q_u32(cpu, qx, i) as i32 as i64,
                    q_u32(cpu, qy, i) as i32 as i64,
                );
                let r = match op {
                    0 => sat_s(a + b, w),
                    1 => sat_s(a - b, w),
                    2 => a.min(b),
                    _ => a.max(b),
                };
                set_q_u32(cpu, qz, i, r as u32);
            }
            _ => unreachable!(),
        }
    }
}

/// Fused vector multiply + load (QEMU `translate_vmul_s3` with
/// addr_inc16); same mem/ALU/postupdate shape, SAR from standard SAR.
/// U8/U16 lanes (QEMU `vmul_u8`/`vmul_u16` helpers) shift the widened
/// unsigned product and truncate like the S8/S16 arms.
fn ee_fused_vmul<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, w: Width, is_st: bool) {
    let mem_qu = if is_st {
        ((raw >> 20) & 7) as usize
    } else {
        (((raw >> 19) & 1) | (((raw >> 24) & 3) << 1)) as usize
    };
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    if is_st {
        let lo = u64::from_le_bytes(cpu.qregs[mem_qu & 7][..8].try_into().unwrap());
        let hi = u64::from_le_bytes(cpu.qregs[mem_qu & 7][8..].try_into().unwrap());
        ee_st64(bus, aligned, lo);
        ee_st64(bus, aligned.wrapping_add(8), hi);
    } else {
        let lo = ee_ld64(bus, aligned);
        let hi = ee_ld64(bus, aligned.wrapping_add(8));
        cpu.qregs[mem_qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
        cpu.qregs[mem_qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    }
    let qz = ((raw >> 16) & 7) as usize;
    let qx = (((raw >> 14) & 1) | (((raw >> 15) & 1) << 1) | (((raw) & 1) << 2)) as usize;
    let qy = (((raw >> 23) & 1) | (((raw >> 12) & 1) << 1) | (((raw >> 13) & 1) << 2)) as usize;
    let sar = std_sar(cpu);
    match w {
        Width::S8 => {
            for i in 0..16 {
                let p = q_s8(cpu, qx, i) as i32 * q_s8(cpu, qy, i) as i32;
                set_q_u8(cpu, qz, i, shr_trunc_i16(p, sar) as u8);
            }
        }
        Width::U8 => {
            for i in 0..16 {
                let p = q_u8(cpu, qx, i) as u32 * q_u8(cpu, qy, i) as u32;
                set_q_u8(cpu, qz, i, p.wrapping_shr(sar.min(31)) as u8);
            }
        }
        Width::S16 => {
            for i in 0..8 {
                let p = q_s16(cpu, qx, i) as i32 * q_s16(cpu, qy, i) as i32;
                set_q_u16(cpu, qz, i, p.wrapping_shr(sar.min(31)) as u16);
            }
        }
        Width::U16 => {
            for i in 0..8 {
                let p = q_u16(cpu, qx, i) as u32 * q_u16(cpu, qy, i) as u32;
                set_q_u16(cpu, qz, i, p.wrapping_shr(sar.min(31)) as u16);
            }
        }
        _ => unreachable!(),
    }
    cpu.set_reg(a, base.wrapping_add(16));
}

/// Fused vector-scalar-MAC + 128-bit load (QEMU
/// `translate_vsmulas_qacc_s3` with addr_inc16): MAC first (scalar lane
/// `sel` of qy widened across qx into both QACC halves), then mem128
/// into qu, then AR += 16. Operand map GAS-probed 2026-09-14 on
/// `q0,a2,q3,q4,sel` (+ full qx/qy/qu/AS sweeps): qu = raw[25:24]++
/// raw[19] (qu[1:0]=b3[1:0], qu[2]=b2[3]), AR = raw[7:4],
/// qx = raw[15:14]++raw[0] (qx[1:0]=b1[7:6], qx[2]=b0[0]),
/// qy = raw[23]++raw[13:12] (qy[0]=b2[7], qy[2:1]=b1[4:3]),
/// sel = s8: raw[20]++raw[19:16] (sel[0]=b2[4], sel[3:1]=b2[3:0]),
/// s16: raw[19:16] (sel=b2[3:0], lanes mask &3).
fn ee_vsmulas_fused<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, w: Width) {
    let qx = ((((raw >> 14) & 3) | (((raw) & 1) << 2)) & 7) as usize;
    let qy = ((((raw >> 23) & 1) | (((raw >> 12) & 3) << 1)) & 7) as usize;
    let sel = if matches!(w, Width::S16) {
        ((raw >> 16) & 0xF) as usize
    } else {
        ((((raw >> 16) & 0xF) << 1) | ((raw >> 20) & 1)) as usize & 15
    };
    ee_vsmulas_lanes(cpu, qx, qy, sel, w);
    let qu = ((((raw >> 19) & 1) | (((raw >> 24) & 3) << 1)) & 7) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let lo = ee_ld64(bus, aligned);
    let hi = ee_ld64(bus, aligned.wrapping_add(8));
    cpu.qregs[qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
    cpu.qregs[qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    cpu.set_reg(a, base.wrapping_add(16));
}

/// Scalar-lane MAC shared by plain and fused vsmulas (QEMU
/// `vsmulas_s3`: scalar qy[sel] widened across qx into both halves).
fn ee_vsmulas_lanes(cpu: &mut Cpu, qx: usize, qy: usize, sel: usize, w: Width) {
    if matches!(w, Width::S16) {
        let s = q_s16(cpu, qy, sel & 3) as i64;
        for i in 0..4 {
            let lane = acc_s40(cpu, 0, i) + q_s16(cpu, qx, i) as i64 * s;
            set_acc_s40(cpu, 0, i, lane.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF));
            let lane = acc_s40(cpu, 1, i) + q_s16(cpu, qx, i + 4) as i64 * s;
            set_acc_s40(cpu, 1, i, lane.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF));
        }
    } else {
        let s = q_s8(cpu, qy, sel & 15) as i64;
        for i in 0..8 {
            let lane = acc_s20(cpu, 0, i) + q_s8(cpu, qx, i) as i64 * s;
            set_acc_s20(cpu, 0, i, lane.clamp(-0x0007_FFFF, 0x0007_FFFF));
            let lane = acc_s20(cpu, 1, i) + q_s8(cpu, qx, i + 8) as i64 * s;
            set_acc_s20(cpu, 1, i, lane.clamp(-0x0007_FFFF, 0x0007_FFFF));
        }
    }
}

/// Radix-2 butterfly (QEMU `r2bf_s3`); qa0 at b1[7]++b2[5:4]<<1, qa1 at
/// raw[14:12], qx at raw[4]++raw[6]++raw[7], qy at
/// raw[5]++raw[10]++raw[11], sel2 at raw[8].
fn ee_r2bf(cpu: &mut Cpu, raw: u32) {
    let qa0 = (((raw >> 15) & 1) | (((raw >> 20) & 3) << 1)) as usize;
    let qa1 = ((raw >> 12) & 7) as usize;
    let qx = (((raw >> 4) & 1) | (((raw >> 6) & 1) << 1) | (((raw >> 7) & 1) << 2)) as usize;
    let qy = (((raw >> 5) & 1) | (((raw >> 10) & 1) << 1) | (((raw >> 11) & 1) << 2)) as usize;
    let sel = ((raw >> 8) & 1) as usize;
    let mut op_a = [0i16; 8];
    let mut op_b = [0i16; 8];
    if sel == 0 {
        for i in 0..4 {
            op_a[i] = q_s16(cpu, qx, i);
            op_a[i + 4] = q_s16(cpu, qy, i);
            op_b[i] = q_s16(cpu, qx, i + 4);
            op_b[i + 4] = q_s16(cpu, qy, i + 4);
        }
    } else {
        for i in 0..2 {
            op_a[i] = q_s16(cpu, qx, i);
            op_a[i + 2] = q_s16(cpu, qx, i + 4);
            op_a[i + 4] = q_s16(cpu, qy, i);
            op_a[i + 6] = q_s16(cpu, qy, i + 4);
            op_b[i] = q_s16(cpu, qx, i + 2);
            op_b[i + 2] = q_s16(cpu, qx, i + 6);
            op_b[i + 4] = q_s16(cpu, qy, i + 2);
            op_b[i + 6] = q_s16(cpu, qy, i + 6);
        }
    }
    for i in 0..4 {
        set_q_u16(cpu, qa0, i, op_a[i].wrapping_add(op_b[i]) as u16);
        set_q_u16(cpu, qa0, i + 4, op_a[i].wrapping_sub(op_b[i]) as u16);
        set_q_u16(cpu, qa1, i, op_a[i + 4].wrapping_add(op_b[i + 4]) as u16);
        set_q_u16(
            cpu,
            qa1,
            i + 4,
            op_a[i + 4].wrapping_sub(op_b[i + 4]) as u16,
        );
    }
}

/// Fused vector-MAC + 128-bit load (QEMU `translate_vmulas_*_s3` with
/// addr_ip/addr_xp): the MAC runs FIRST (on qx/qy), then mem128 loads
/// into qu, then the postupdate. qu = raw[19]++raw[25:24], AR =
/// raw[7:4], imm = raw[8]<<4 (ip) or ax = raw[11:8] (xp, marker
/// raw[20]), qx = raw[13]++raw[15]++raw[0], qy =
/// raw[23]++raw[13:12]. (op,width) in raw[18:16] (b2[2:0]):
/// [2] = unsigned, [1] = 8-bit, [0] = qacc (vs accx).
fn ee_vmulas_fused<B: Bus>(
    cpu: &mut Cpu,
    bus: &mut B,
    raw: u32,
    w: Width,
    to_qacc: bool,
    is_xp: bool,
) {
    // Fused qx/qy use their own split positions (NOT CAL).
    let qx = (((raw >> 13) & 1) | (((raw >> 15) & 1) << 1) | ((raw & 1) << 2)) as usize;
    let qy = (((raw >> 23) & 1) | (((raw >> 12) & 3) << 1)) as usize;
    if to_qacc {
        ee_vmulas_qacc_fused_lanes(cpu, qx, qy, w);
    } else {
        ee_vmulas_accx_fused_lanes(cpu, qx, qy, w);
    }
    let qu = (((raw >> 19) & 1) | (((raw >> 24) & 3) << 1)) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let lo = ee_ld64(bus, aligned);
    let hi = ee_ld64(bus, aligned.wrapping_add(8));
    cpu.qregs[qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
    cpu.qregs[qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    if is_xp {
        let ax = cpu.reg((raw >> 8) & 0xF);
        cpu.set_reg(a, base.wrapping_add(ax));
    } else {
        cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 1) << 4));
    }
}

fn ee_vmulas_accx_fused_lanes(cpu: &mut Cpu, qx: usize, qy: usize, w: Width) {
    match w {
        Width::S8 => {
            for i in 0..16 {
                cpu.accx += q_s8(cpu, qx, i) as i64 * q_s8(cpu, qy, i) as i64;
            }
            cpu.accx = cpu.accx.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF);
        }
        Width::U8 => {
            for i in 0..16 {
                cpu.accx += q_u8(cpu, qx, i) as i64 * q_u8(cpu, qy, i) as i64;
            }
            cpu.accx = cpu.accx.clamp(0, 0x00FF_FFFF_FFFF);
        }
        Width::S16 => {
            for i in 0..8 {
                cpu.accx += q_s16(cpu, qx, i) as i64 * q_s16(cpu, qy, i) as i64;
            }
            cpu.accx = cpu.accx.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF);
        }
        _ => {
            for i in 0..8 {
                cpu.accx += q_u16(cpu, qx, i) as i64 * q_u16(cpu, qy, i) as i64;
            }
            cpu.accx = cpu.accx.clamp(0, 0x00FF_FFFF_FFFF);
        }
    }
}

fn ee_vmulas_qacc_fused_lanes(cpu: &mut Cpu, qx: usize, qy: usize, w: Width) {
    match w {
        Width::S8 => {
            for i in 0..8 {
                let lane = acc_s20(cpu, 0, i) + q_s8(cpu, qx, i) as i64 * q_s8(cpu, qy, i) as i64;
                set_acc_s20(cpu, 0, i, lane.clamp(-0x0007_FFFF, 0x0007_FFFF));
                let lane =
                    acc_s20(cpu, 1, i) + q_s8(cpu, qx, i + 8) as i64 * q_s8(cpu, qy, i + 8) as i64;
                set_acc_s20(cpu, 1, i, lane.clamp(-0x0007_FFFF, 0x0007_FFFF));
            }
        }
        Width::U8 => {
            for i in 0..8 {
                for acc in 0..2 {
                    let off = if acc == 0 { i } else { i + 8 };
                    let v = acc_u20(cpu, acc, i)
                        .wrapping_add(q_u8(cpu, qx, off) as u64 * q_u8(cpu, qy, off) as u64);
                    set_acc_u20(
                        cpu,
                        acc,
                        i,
                        if v > 0x000F_FFFF {
                            (-0x000F_FFFF_i64) as u64
                        } else {
                            v
                        },
                    );
                }
            }
        }
        Width::S16 => {
            for i in 0..4 {
                for acc in 0..2 {
                    let off = if acc == 0 { i } else { i + 4 };
                    let v = acc_s40(cpu, acc, i)
                        + q_s16(cpu, qx, off) as i64 * q_s16(cpu, qy, off) as i64;
                    set_acc_s40(cpu, acc, i, v.clamp(-0x007F_FFFF_FFFF, 0x007F_FFFF_FFFF));
                }
            }
        }
        _ => {
            for i in 0..4 {
                for acc in 0..2 {
                    let off = if acc == 0 { i } else { i + 4 };
                    let v = acc_u40(cpu, acc, i)
                        + q_u16(cpu, qx, off) as u64 * q_u16(cpu, qy, off) as u64;
                    set_acc_u40(cpu, acc, i, v.min(0x00FF_FFFF_FFFF));
                }
            }
        }
    }
}

/// Fused vector-MAC + broadcast load (QEMU `translate_vmulas_qacc_s3`
/// with addr_ldbc_inc1): MAC first (CAL qx/qy), then broadcast mem
/// (8/16-bit from raw[21]) into qu, then AR += 1 (8-bit) or +2.
/// qu = raw[13]++raw[15]++raw[20].
fn ee_vmulas_ldbc<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, w: Width) {
    ee_vmulas_ldbc_mac(cpu, raw, w);
    let qu = (((raw >> 13) & 1) | (((raw >> 15) & 1) << 1) | (((raw >> 20) & 1) << 2)) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let is8 = matches!(w, Width::S8 | Width::U8);
    if is8 {
        let v = bus.read8(base) as u8;
        cpu.qregs[qu & 7] = [v; 16];
        cpu.set_reg(a, base.wrapping_add(1));
    } else {
        let v = bus.read16(base & !1) as u16;
        for i in 0..8 {
            set_q_u16(cpu, qu, i, v);
        }
        cpu.set_reg(a, base.wrapping_add(2));
    }
}

/// MAC half shared by the ldbc and ldbc.qup fused forms. The plain
/// `.ldbc.incp` word carries the CAL_DOUBLE_Q triple (true qx/qy at the
/// CAL positions — verified by single-varying GAS sweeps against the
/// in-model MAC). The `.qup` word reuses the vsmulas-fused operand
/// positions instead (qx = raw[15:14]++raw[0], qy = raw[23]++
/// raw[13:12] — GAS-probed 2026-09-14; the CAL fields alias the slide
/// operands there, so CAL decodes garbage). `qup` selects the form.
fn ee_vmulas_ldbc_mac(cpu: &mut Cpu, raw: u32, w: Width) {
    let b3 = (raw >> 24) & 0xFF;
    let (qx, qy) = if b3 == 0xE0 && (raw & 0x0F) >= 0x0E && ((raw >> 16) & 0xFF) == 0x56 {
        // .qup tail (selected by the same b2 == 0x56 rule as the
        // decoder): vsmulas-fused positions.
        (
            ((((raw >> 14) & 3) | (((raw) & 1) << 2)) & 7) as usize,
            ((((raw >> 23) & 1) | (((raw >> 12) & 3) << 1)) & 7) as usize,
        )
    } else {
        let (_, qx, qy) = cal_q(raw);
        (qx, qy)
    };
    ee_vmulas_qacc_fused_lanes(cpu, qx, qy, w);
}

/// Fused vector-MAC + broadcast load + Q-slide (QEMU
/// `translate_vmulas_qacc_s3` with addr_ldbc_inc1 + vmul_qup): MAC,
/// then broadcast, then AR += 1/2, then qup-slide (qs0 by SAR_BYTE with
/// qs1 fill). Operand map GAS-probed 2026-09-14
/// (`qu,as,qx,qy,qs0,qs1` positional): qu = raw[19]++raw[25:24]
/// (qu[0]=b2[3], qu[2:1]=b3[1:0]), AR = raw[7:4] (b0[7:4]),
/// qs0 = raw[22:20] (full 3-bit b2[6:4] — GAS accepts odd slides;
/// qu==qs0 is rejected by GAS as a duplicate), qs1 = raw[18:16].
fn ee_vmulas_ldbc_qup<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32, w: Width) {
    ee_vmulas_ldbc_mac(cpu, raw, w);
    let qu = ((((raw >> 19) & 1) | (((raw >> 24) & 3) << 1)) & 7) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let is8 = matches!(w, Width::S8 | Width::U8);
    if is8 {
        let v = bus.read8(base) as u8;
        cpu.qregs[qu & 7] = [v; 16];
        cpu.set_reg(a, base.wrapping_add(1));
    } else {
        let v = bus.read16(base & !1) as u16;
        for i in 0..8 {
            set_q_u16(cpu, qu, i, v);
        }
        cpu.set_reg(a, base.wrapping_add(2));
    }
    let qs0 = ((raw >> 20) & 7) as usize;
    let qs1 = ((raw >> 16) & 7) as usize;
    let sar = (cpu.sar_byte as usize).min(16);
    let sb = cpu.qregs[qs1 & 7];
    for i in 0..(16 - sar) {
        cpu.qregs[qs0 & 7][i] = cpu.qregs[qs0 & 7][i + sar];
    }
    for i in 16 - sar..16 {
        cpu.qregs[qs0 & 7][i] = sb[i - (16 - sar)];
    }
}

/// FFT add-multiply-subtract + load (QEMU `fft_ams_s16`): AMS runs on
/// (qz,qz1,qx,qy,qm,sel2) first (lanes 2,3 only), then mem128 loads
/// into qu, then AR += 16. qu = raw[10:8] (qu[1:0] != 10, qu != qz),
/// AR = raw[7:4], qz = raw[13:12]<<1 ([0] masked), qz1 = raw[11] ++
/// raw[24]<<2 ([1] masked, [2:1] != 00), qx = raw[15]<<1 ++ raw[0]<<2
/// ([0] masked, qx != qz), qy = laned decode ([1] = raw[21]; if set
/// [0] = raw[20],[2] = raw[22], else [2] = raw[18],[0] = raw[16]&~[2]),
/// qm = raw[18:16], sel2 = raw[25], SAR = standard SAR.
fn ee_ams_ld<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    ee_ams_math(cpu, raw);
    // Load + postupdate.
    let qu = ((raw >> 8) & 7) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let lo = ee_ld64(bus, aligned);
    let hi = ee_ld64(bus, aligned.wrapping_add(8));
    cpu.qregs[qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
    cpu.qregs[qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    cpu.set_reg(a, base.wrapping_add(16));
}

/// FFT AMS + unaligned-update load (QEMU-analogous `fft_ams_s16` math +
/// ESP32-P4 PIE reference "load with unaligned update"): same math and
/// aligned 128-bit load as `ee_ams_ld`, but the address low nibble is
/// also snapshotted into SAR_BYTE (LD.USAR precedent) so a following
/// src.q slide aligns correctly; AR += 16.
fn ee_ams_ld_uaup<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    ee_ams_math(cpu, raw);
    let qu = ((raw >> 8) & 7) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let lo = ee_ld64(bus, aligned);
    let hi = ee_ld64(bus, aligned.wrapping_add(8));
    cpu.qregs[qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
    cpu.qregs[qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    cpu.sar_byte = (base & 0xF) as u8;
    cpu.set_reg(a, base.wrapping_add(16));
}

/// FFT AMS + reversed-32 load with pointer decrement (QEMU-analogous
/// `fft_ams_s16` math + `vst_r32_decp`-mirrored lane order): qu fills
/// with u32-reversed lanes ([m3,m2,m1,m0], the inverse of the r32 store
/// arrangement), then AR -= 16 (decp precedent: `ee_vst_decp`).
fn ee_ams_ld_r32_decp<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    ee_ams_math(cpu, raw);
    let qu = ((raw >> 8) & 7) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let mut w = [0u32; 4];
    for (i, slot) in w.iter_mut().enumerate() {
        *slot = bus.read32(aligned.wrapping_add(4 * i as u32));
    }
    cpu.qregs[qu & 7][..4].copy_from_slice(&w[3].to_le_bytes());
    cpu.qregs[qu & 7][4..8].copy_from_slice(&w[2].to_le_bytes());
    cpu.qregs[qu & 7][8..12].copy_from_slice(&w[1].to_le_bytes());
    cpu.qregs[qu & 7][12..].copy_from_slice(&w[0].to_le_bytes());
    cpu.set_reg(a, base.wrapping_sub(16));
}

/// Shared AMS butterfly math (QEMU `fft_ams_s16`, lanes 2,3 only); the
/// ld/uaup/decp tails differ only in the memory op + AR update.
fn ee_ams_math(cpu: &mut Cpu, raw: u32) {
    let qz = (((raw >> 12) & 3) << 1) as usize;
    let qz1 = (((raw >> 11) & 1) | (((raw >> 24) & 1) << 2)) as usize;
    let qx = (((raw >> 15) & 1) << 1 | ((raw) & 1) << 2) as usize;
    let qy1 = ((raw >> 21) & 1) as usize;
    let qy = if qy1 == 1 {
        (((raw >> 20) & 1) | (1 << 1) | (((raw >> 22) & 1) << 2)) as usize
    } else {
        let qy2 = (raw >> 18) & 1;
        ((((raw >> 16) & 1) & !qy2) | (qy2 << 2)) as usize
    };
    let qm = ((raw >> 16) & 7) as usize;
    let sel = ((raw >> 25) & 1) as usize;
    let sar = std_sar(cpu).min(31);
    // Complex twiddle multiply + add/sub on lanes 2,3.
    let (ax2, ax3) = (q_s16(cpu, qx, 2) as i32, q_s16(cpu, qx, 3) as i32);
    let (ay2, ay3) = (q_s16(cpu, qy, 2) as i32, q_s16(cpu, qy, 3) as i32);
    let (am2, am3) = (q_s16(cpu, qm, 2) as i32, q_s16(cpu, qm, 3) as i32);
    let temp0 = (ax2 + ay2) as i16;
    let temp1 = (ax3 - ay3) as i16;
    let (temp2, temp3) = if sel == 0 {
        let t2 =
            (((ax2 - ay2) as i64 * am2 as i64 - (ax3 + ay3) as i64 * am3 as i64) >> sar) as i16;
        let t3 =
            (((ax2 - ay2) as i64 * am3 as i64 + (ax3 + ay3) as i64 * am2 as i64) >> sar) as i16;
        (t2, t3)
    } else {
        let t2 =
            (((ax3 + ay3) as i64 * am3 as i64 + (ax2 - ay2) as i64 * am2 as i64) >> sar) as i16;
        let t3 =
            (((ax3 + ay3) as i64 * am2 as i64 - (ax2 - ay2) as i64 * am3 as i64) >> sar) as i16;
        (t2, t3)
    };
    set_q_u16(cpu, qz, 2, temp0.wrapping_add(temp2) as u16);
    set_q_u16(cpu, qz, 3, temp1.wrapping_add(temp3) as u16);
    set_q_u16(cpu, qz1, 2, temp0.wrapping_sub(temp2) as u16);
    set_q_u16(cpu, qz1, 3, temp3.wrapping_sub(temp1) as u16);
}

/// Fused Q-slide + 128-bit load (QEMU `translate_src_q_s3` with
/// addr_ip/addr_xp): qup-slide (qs0,qs1) by SAR_BYTE first, then
/// mem128 into qu, then postupdate. qu = raw[18:16], AR = raw[7:4],
/// imm = raw[8]<<4 (ip) or ax = raw[11:8] (xp), qs0 = raw[14] ++
/// raw[15] ++ raw[0], qs1 = raw[22:20]. (qu != qs0.)
fn ee_src_q_ld<B: Bus>(cpu: &mut Cpu, bus: &mut B, name: &str, raw: u32) {
    let qs0 = (((raw >> 14) & 1) | (((raw >> 15) & 1) << 1) | ((raw & 1) << 2)) as usize;
    let qs1 = ((raw >> 20) & 7) as usize;
    // qup slide (in-place ascending is hazard-free: reads lead writes).
    let sar = (cpu.sar_byte as usize).min(16);
    let sb = cpu.qregs[qs1 & 7];
    for i in 0..(16 - sar) {
        cpu.qregs[qs0 & 7][i] = cpu.qregs[qs0 & 7][i + sar];
    }
    for i in 16 - sar..16 {
        cpu.qregs[qs0 & 7][i] = sb[i - (16 - sar)];
    }
    // Load.
    let qu = ((raw >> 16) & 7) as usize;
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let lo = ee_ld64(bus, aligned);
    let hi = ee_ld64(bus, aligned.wrapping_add(8));
    cpu.qregs[qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
    cpu.qregs[qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    if name.contains("_xp") {
        let ax = cpu.reg((raw >> 8) & 0xF);
        cpu.set_reg(a, base.wrapping_add(ax));
    } else {
        cpu.set_reg(a, base.wrapping_add(((raw >> 8) & 1) << 4));
    }
}

/// Fused slide-store (QEMU `srcq_128_st_s3` via `srcq_64_rd_s3`): store
/// the SAR_BYTE slide of (qs0,qs1) to mem128, then AR += 16. qs0 =
/// raw[14:12], qs1 = single-Q position, AR at t.
fn ee_srcq_st<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    let qs0 = ((raw >> 12) & 7) as usize;
    let qs1 = qu(raw);
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let sar = (cpu.sar_byte as usize).min(16);
    let (sa, sb) = (cpu.qregs[qs0 & 7], cpu.qregs[qs1 & 7]);
    for (i, off) in [(0u32, 0usize), (8u32, 8usize)] {
        // srcq_64_rd low_high selects temp[shift..+8] / temp[shift+8..+8].
        let mut w = [0u8; 8];
        for (k, slot) in w.iter_mut().enumerate() {
            let src = sar + off + k;
            *slot = if src < 16 { sa[src] } else { sb[src - 16] };
        }
        ee_st64(bus, aligned.wrapping_add(i), u64::from_le_bytes(w));
    }
    cpu.set_reg(a, base.wrapping_add(16));
}

/// Fused butterfly-store (QEMU `r2bf_st_*_s3`): qa0 = qx-qy (in
/// place), mem halves get shifted sums, then as += 16. qa0 =
/// raw[18:16], qx = raw[15:14] with raw[0]<<2, qy even/odd split (see
/// code), AR at t, sar4 = raw[12] with raw[23]<<1 (2-bit).
fn ee_r2bf_st<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    let qa0 = ((raw >> 16) & 7) as usize;
    let qx = (((raw >> 14) & 3) | (((raw) & 1) << 2)) as usize;
    // qy[0] = raw[15]; even qy fully in raw[14:12], odd qy keeps [2] in
    // raw[8] with [1] masked.
    let qy = if ((raw >> 15) & 1) == 0 {
        ((raw >> 12) & 7) as usize
    } else {
        (1 | (((raw >> 8) & 1) << 2)) as usize
    };
    let a = ar_t(raw);
    let sar = (((raw >> 12) & 1) | (((raw >> 23) & 1) << 1)) as usize;
    let base = cpu.reg(a);
    let aligned = base & !15;
    for i in 0..8 {
        let d = q_s16(cpu, qx, i).wrapping_sub(q_s16(cpu, qy, i));
        set_q_u16(cpu, qa0, i, d as u16);
    }
    for half in 0..2 {
        let mut b = [0u8; 8];
        for i in 0..4 {
            let s =
                (q_s16(cpu, qx, half * 4 + i) as i32 + q_s16(cpu, qy, half * 4 + i) as i32) >> sar;
            let w = (s as i16) as u16;
            b[2 * i] = w as u8;
            b[2 * i + 1] = (w >> 8) as u8;
        }
        ee_st64(
            bus,
            aligned.wrapping_add(8 * half as u32),
            u64::from_le_bytes(b),
        );
    }
    cpu.set_reg(a, base.wrapping_add(16));
}

/// FFT complex multiply + load (QEMU `fft_cmul_ld_s3`): cmul on the
/// sel8-selected s16 pair first (even sel8 = (ac+bd,bc-ad), odd =
/// (ac-bd,bc+ad)), then mem128 into qu, then AR += ax. qu =
/// raw[12]<<1 ++ raw[13]<<2 ([0] masked, qu != qz), AR = raw[7:4], ax
/// = raw[11:8], qz = raw[19:16], qx = raw[14] ++ raw[15]<<1 ++
/// raw[0]<<2, qy = raw[23:20], sel8 = raw[19] ++ raw[24]<<1 ++
/// raw[25]<<2. SAR = standard SAR.
fn ee_cmul_ld<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    let qz = ((raw >> 16) & 15) as usize;
    let qx = (((raw >> 14) & 1) | (((raw >> 15) & 1) << 1) | ((raw & 1) << 2)) as usize;
    let qy = ((raw >> 20) & 15) as usize;
    let sel = (((raw >> 19) & 1) | (((raw >> 24) & 1) << 1) | (((raw >> 25) & 1) << 2)) as usize;
    let sar = std_sar(cpu).min(31);
    let base = (sel >> 1) * 2;
    let (ar, ai) = (q_s16(cpu, qx, base) as i32, q_s16(cpu, qx, base + 1) as i32);
    let (br, bi) = (q_s16(cpu, qy, base) as i32, q_s16(cpu, qy, base + 1) as i32);
    let (re, im) = if sel & 1 == 0 {
        (ar * br + ai * bi, ai * br - ar * bi)
    } else {
        (ar * br - ai * bi, ar * bi + ai * br)
    };
    // NOTE: sel8 lanes 6,7 (sel 6,7) address s16[6],s16[7] (high pair).
    set_q_u16(cpu, qz, base, re.wrapping_shr(sar) as u16);
    set_q_u16(cpu, qz, base + 1, im.wrapping_shr(sar) as u16);
    let qu = (((raw >> 12) & 1) << 1 | (((raw >> 13) & 1) << 2)) as usize;
    let a = ar_t(raw);
    let abase = cpu.reg(a);
    let aligned = abase & !15;
    let lo = ee_ld64(bus, aligned);
    let hi = ee_ld64(bus, aligned.wrapping_add(8));
    cpu.qregs[qu & 7][..8].copy_from_slice(&lo.to_le_bytes());
    cpu.qregs[qu & 7][8..].copy_from_slice(&hi.to_le_bytes());
    let ax = cpu.reg((raw >> 8) & 0xF);
    cpu.set_reg(a, abase.wrapping_add(ax));
}

/// FFT complex-multiply store (QEMU `fft_cmul_st_*_s3`): mem[(as&~15)]
/// gets upd4-selected low half, mem[+8] gets [qv[4],qv[5],cmul-pair],
/// then AR += ax. qx = raw[0]<<2 ([1:0] masked), qy = raw[23:20], qv
/// = raw[19:16], AR at t, ax = raw[11:8], sel8 = raw[23] ++ raw[12]<<1
/// ++ raw[13]<<2, upd4 = raw[19], sar4 = raw[25] ++ raw[26]<<1 (2-bit).
/// SAR = standard SAR. (qy=q5 corrupts ax and is avoided.)
fn ee_cmul_st<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    let qx = (((raw) & 1) << 2) as usize;
    // qy is 3-bit (raw[22:20]); raw[23] is sel8[0], NOT qy[3].
    let qy = ((raw >> 20) & 7) as usize;
    let qv = ((raw >> 16) & 15) as usize & 7;
    let sel = (((raw >> 23) & 1) | (((raw >> 12) & 1) << 1) | (((raw >> 13) & 1) << 2)) as usize;
    let upd = ((raw >> 19) & 1) as usize;
    let sar4 = (((raw >> 25) & 1) | (((raw >> 26) & 1) << 1)) as usize;
    let sar = std_sar(cpu).min(31);
    let a = ar_t(raw);
    let base = cpu.reg(a);
    let aligned = base & !15;
    // Low half by upd4.
    let mut lo = [0u8; 8];
    for i in 0..4 {
        let v = if upd == 0 {
            q_s16(cpu, qv, i) as u16
        } else if upd == 1 || i < 2 {
            q_s16(cpu, qx, i).wrapping_shr(sar4 as u32) as u16
        } else {
            q_s16(cpu, qv, i + 2) as u16
        };
        lo[2 * i] = v as u8;
        lo[2 * i + 1] = (v >> 8) as u8;
    }
    ee_st64(bus, aligned, u64::from_le_bytes(lo));
    // High half: [qv[4],qv[5],cmul-pair] (upd4 0/1) or [qx>>sar4 (upd4 2)].
    let qbase = (sel >> 1) * 2;
    let (ar, ai) = (
        q_s16(cpu, qx, qbase) as i32,
        q_s16(cpu, qx, qbase + 1) as i32,
    );
    let (br, bi) = (
        q_s16(cpu, qy, qbase) as i32,
        q_s16(cpu, qy, qbase + 1) as i32,
    );
    let (re, im) = if sel & 1 == 0 {
        (ar * br + ai * bi, ai * br - ar * bi)
    } else {
        (ar * br - ai * bi, ar * bi + ai * br)
    };
    let (t0, t1) = (
        re.wrapping_shr(sar) as i16 as u16,
        im.wrapping_shr(sar) as i16 as u16,
    );
    let (h0, h1) = if upd == 2 {
        (
            q_s16(cpu, qx, 2).wrapping_shr(sar4 as u32) as u16,
            q_s16(cpu, qx, 3).wrapping_shr(sar4 as u32) as u16,
        )
    } else {
        (q_s16(cpu, qv, 4) as u16, q_s16(cpu, qv, 5) as u16)
    };
    let mut hi = [0u8; 8];
    for (i, v) in [h0, h1, t0, t1].iter().enumerate() {
        hi[2 * i] = *v as u8;
        hi[2 * i + 1] = (*v >> 8) as u8;
    }
    ee_st64(bus, aligned.wrapping_add(8), u64::from_le_bytes(hi));
    let ax = cpu.reg((raw >> 8) & 0xF);
    cpu.set_reg(a, base.wrapping_add(ax));
}

/// FFT reversed store + decrement (QEMU `fft_vst_64_s3`): mem[(as&~15)]
/// gets [Q.u32[3],Q.u32[2]]>>sar2, mem[+8] gets [Q.u32[1],Q.u32[0]]>>
/// sar2 (logical), then AR -= 16. qv at the single-Q position, AR at
/// t, sar2 = raw[10] (1-bit).
fn ee_vst_decp<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    let qv = qu(raw);
    let a = ar_t(raw);
    let sar2 = ((raw >> 10) & 1).min(31);
    let base = cpu.reg(a);
    let aligned = base & !15;
    let lo = (q_u32(cpu, qv, 3).wrapping_shr(sar2) as u64)
        | ((q_u32(cpu, qv, 2).wrapping_shr(sar2) as u64) << 32);
    let hi = (q_u32(cpu, qv, 1).wrapping_shr(sar2) as u64)
        | ((q_u32(cpu, qv, 0).wrapping_shr(sar2) as u64) << 32);
    ee_st64(bus, aligned, lo);
    ee_st64(bus, aligned.wrapping_add(8), hi);
    cpu.set_reg(a, base.wrapping_sub(16));
}

/// Fused vector-MAC + load + Q-slide (vmulas `.qup` suffix): MAC,
/// then load, then postupdate, then qup-slide (qs0 by SAR_BYTE with
/// qs1 fill). qs0 = raw[22]<<2 (0/4, lossy), qs1 = raw[18:16].
fn ee_vmulas_qup_fused<B: Bus>(
    cpu: &mut Cpu,
    bus: &mut B,
    raw: u32,
    w: Width,
    to_qacc: bool,
    is_xp: bool,
) {
    ee_vmulas_fused(cpu, bus, raw, w, to_qacc, is_xp);
    let qs0 = (((raw >> 22) & 1) << 2) as usize;
    let qs1 = ((raw >> 16) & 7) as usize;
    let sar = (cpu.sar_byte as usize).min(16);
    let sb = cpu.qregs[qs1 & 7];
    for i in 0..(16 - sar) {
        cpu.qregs[qs0 & 7][i] = cpu.qregs[qs0 & 7][i + sar];
    }
    for i in 16 - sar..16 {
        cpu.qregs[qs0 & 7][i] = sb[i - (16 - sar)];
    }
}

/// FFT AMS store (QEMU `fft_ams_st_0/1_s16_*`): mem[(as&~15)] gets
/// [as0>>1,qv>>1] (sel2=0) or [as0,qv] (sel2=1); mem[+8] gets
/// [qv>>1] (sel2=0) or [qv] (sel2=1); qz1[6,7] side effects; then AR
/// += 16. qv = raw[22:20], qz1 = raw[19] ++ raw[25:24]<<1, as0 =
/// raw[7:4], as = raw[11:8], qx = raw[18:16], qy = raw[15:14] ++
/// raw[0]<<2, qm = raw[12]<<1 ++ raw[13]<<2 ([0] masked), sel2 =
/// raw[26]. (temp_asm has no implemented reader; qz1 writes kept.)
fn ee_ams_st<B: Bus>(cpu: &mut Cpu, bus: &mut B, raw: u32) {
    let qv = ((raw >> 20) & 7) as usize;
    let qz1 = (((raw >> 19) & 1) | (((raw >> 24) & 3) << 1)) as usize;
    let as0 = cpu.reg(ar_t(raw));
    let qx = ((raw >> 16) & 7) as usize;
    let qy = (((raw >> 14) & 3) | (((raw) & 1) << 2)) as usize;
    let qm = (((raw >> 12) & 1) << 1 | (((raw >> 13) & 1) << 2)) as usize;
    let sel = ((raw >> 26) & 1) as usize;
    let sar = std_sar(cpu).min(31);
    let a = (raw >> 8) & 0xF;
    let base = cpu.reg(a);
    let aligned = base & !15;
    let as0_lo = (as0 & 0xFFFF) as u16 as i16;
    let as0_hi = ((as0 >> 16) & 0xFFFF) as u16 as i16;
    // Low half.
    let mut lo = [0u8; 8];
    for (i, v) in [
        if sel == 0 {
            as0_lo.wrapping_shr(1)
        } else {
            as0_lo
        },
        if sel == 0 {
            as0_hi.wrapping_shr(1)
        } else {
            as0_hi
        },
        if sel == 0 {
            q_s16(cpu, qv, 0).wrapping_shr(1)
        } else {
            q_s16(cpu, qv, 0)
        },
        if sel == 0 {
            q_s16(cpu, qv, 1).wrapping_shr(1)
        } else {
            q_s16(cpu, qv, 1)
        },
    ]
    .iter()
    .enumerate()
    {
        lo[2 * i] = *v as u8;
        lo[2 * i + 1] = (*v >> 8) as u8;
    }
    ee_st64(bus, aligned, u64::from_le_bytes(lo));
    // High half + qz1 side effects (lanes 6,7 twiddle).
    let (ax6, ax7) = (q_s16(cpu, qx, 6) as i32, q_s16(cpu, qx, 7) as i32);
    let (ay6, ay7) = (q_s16(cpu, qy, 6) as i32, q_s16(cpu, qy, 7) as i32);
    let (am6, am7) = (q_s16(cpu, qm, 6) as i32, q_s16(cpu, qm, 7) as i32);
    // sel2=0 uses (qx-qy, qx+qy) pairing; sel2=1 uses (qx+qy, qx-qy).
    // (QEMU st_0 vs st_1 high bodies differ in which lanes feed temp.)
    let (t2, t3) = if sel == 0 {
        let t2 =
            (((ax6 - ay6) as i64 * am6 as i64 - (ax7 + ay7) as i64 * am7 as i64) >> sar) as i16;
        let t3 =
            (((ax6 - ay6) as i64 * am7 as i64 + (ax7 + ay7) as i64 * am6 as i64) >> sar) as i16;
        (t2, t3)
    } else {
        let t2 =
            (((ax7 + ay7) as i64 * am7 as i64 + (ax6 - ay6) as i64 * am6 as i64) >> sar) as i16;
        let t3 =
            (((ax7 + ay7) as i64 * am6 as i64 - (ax6 - ay6) as i64 * am7 as i64) >> sar) as i16;
        (t2, t3)
    };
    let t0 = (ax6 + ay6) as i16;
    let t1 = (ax7 - ay7) as i16;
    set_q_u16(cpu, qz1, 6, t0.wrapping_sub(t2) as u16);
    set_q_u16(cpu, qz1, 7, t3.wrapping_sub(t1) as u16);
    let (h0, h1) = if sel == 0 {
        (
            q_s16(cpu, qv, 2).wrapping_shr(1),
            q_s16(cpu, qv, 3).wrapping_shr(1),
        )
    } else {
        (q_s16(cpu, qv, 2), q_s16(cpu, qv, 3))
    };
    // High stored lanes: sel2=0 [qv2,qv3,qv4,qv5]>>1; sel2=1 [qv2..qv5].
    // (temp4/temp5 go to temp_asm, unwired.)
    let (q4, q5) = if sel == 0 {
        (
            q_s16(cpu, qv, 4).wrapping_shr(1),
            q_s16(cpu, qv, 5).wrapping_shr(1),
        )
    } else {
        (q_s16(cpu, qv, 4), q_s16(cpu, qv, 5))
    };
    let mut hi = [0u8; 8];
    for (i, v) in [h0, h1, q4, q5].iter().enumerate() {
        hi[2 * i] = *v as u8;
        hi[2 * i + 1] = (*v >> 8) as u8;
    }
    ee_st64(bus, aligned.wrapping_add(8), u64::from_le_bytes(hi));
    cpu.set_reg(a, base.wrapping_add(16));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::generated::Opcode;

    #[test]
    fn format32_width_detection() {
        // b0 = 0x0E -> low nibble 14 -> 4-byte format_32 TIE instruction.
        assert!(is_format32(0x0000_000E));
        // b0 = 0x00 -> 3-byte (e.g. EE_LDF_64_XP encoding).
        assert!(!is_format32(0x0007_0000));
        // b0 = 0x0C -> 2-byte density slot.
        assert!(!is_format32(0x0000_000C));
    }

    #[test]
    fn ee_opcode_classification() {
        assert!(is_ee_opcode(Opcode::OPCODE_EE_STF_64_XP));
        assert!(is_ee_opcode(Opcode::OPCODE_EE_VMULAS_U16_ACCX));
        assert!(!is_ee_opcode(Opcode::OPCODE_LSI));
    }

    #[test]
    fn ee_family_labels() {
        assert_eq!(ee_family(Opcode::OPCODE_EE_VMULAS_U16_ACCX), "vmac");
        assert_eq!(ee_family(Opcode::OPCODE_EE_LDF_64_XP), "ldf");
        assert_eq!(ee_family(Opcode::OPCODE_EE_STF_64_XP), "stf");
        assert_eq!(ee_family(Opcode::OPCODE_EE_WR_MASK_GPIO_OUT), "gpio");
        assert_eq!(ee_family(Opcode::OPCODE_EE_ZERO_ACCX), "zero");
        assert_eq!(ee_family(Opcode::OPCODE_EE_VCMP_EQ_S16), "vcmp");
        assert_eq!(ee_family(Opcode::OPCODE_EE_UNIMPLEMENTED), "unimplemented");
    }

    #[test]
    fn decode_ee_unknown_falls_back() {
        // An unrecognized format_32 instruction (b0 = 0xEE) maps to the
        // dedicated unimplemented opcode rather than decoding as illegal.
        assert_eq!(decode_ee(0xEE00_0000), Opcode::OPCODE_EE_UNIMPLEMENTED);
        assert_eq!(
            decode_ee_or_unimplemented(0xEE00_0000),
            Opcode::OPCODE_EE_UNIMPLEMENTED
        );
    }
}
