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

use crate::generated::Opcode;

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
pub fn decode_ee(_insn: u32) -> Opcode {
    // TODO(ee): recognize the remaining format_32 families:
    //   - ee.fft.*        (FFT butterfly / complex-multiply)
    //   - ee.vmulas.*     (vector multiply-accumulate, signed/unsigned)
    //   - ee.ldf/stf      (broadcast load/store f)
    //   - ee.ld.qacc.*    (quarter-round accumulator load)
    // Each maps to an OPCODE_EE_* variant already defined in `generated.rs`.
    Opcode::OPCODE_EE_UNIMPLEMENTED
}

/// Format_32 recognizer hook used by `decode_inst`'s catch-all. Returns the
/// best-known `OPCODE_EE_*` for a `format_32` TIE instruction, defaulting to
/// `OPCODE_EE_UNIMPLEMENTED`.
pub fn decode_ee_or_unimplemented(insn: u32) -> Opcode {
    decode_ee(insn)
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
