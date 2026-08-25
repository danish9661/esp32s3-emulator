// Throwaway ISA-coverage audit: decode every instruction in an objdump -d
// disassembly and report mnemonics our decoder rejects. Run with:
//   ISA_AUDIT_FILE=/tmp/audit.s cargo test -p xtensa-core --test isa_coverage
#![cfg(test)]
use std::collections::BTreeMap;
use std::fs;

use xtensa_core::generated::{decode_inst, decode_inst16a, decode_inst16b};

#[test]
fn audit_disassembly() {
    let path = match std::env::var("ISA_AUDIT_FILE") {
        Ok(p) => p,
        Err(_) => {
            eprintln!("ISA_AUDIT_FILE not set; skipping coverage audit");
            return;
        }
    };
    let text = fs::read_to_string(&path).expect("read audit file");

    let mut failures: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut checked = 0u64;
    let mut mnemonic_count: BTreeMap<String, u64> = BTreeMap::new();

    for line in text.lines() {
        // objdump instruction line: "  40374000:	49c500        	s32e	a0, a5, -16"
        let mut parts = line.splitn(2, ':');
        let _addr = match parts.next() {
            Some(a) if !a.trim().is_empty() && a.trim().chars().all(|c| c.is_ascii_hexdigit()) => a,
            _ => continue,
        };
        let rest = match parts.next() {
            Some(r) => r,
            None => continue,
        };
        // bytes token is the first whitespace-delimited field of rest
        let mut it = rest.split_whitespace();
        let bytes_tok = match it.next() {
            Some(b) => b,
            None => continue,
        };
        let mnem = match it.next() {
            Some(m) => m,
            None => continue,
        };
        let raw = match u32::from_str_radix(bytes_tok, 16) {
            Ok(r) => r,
            Err(_) => continue,
        };
        let nhex = bytes_tok.len();
        if nhex != 4 && nhex != 6 && nhex != 8 {
            continue;
        }
        let decoded = if nhex == 4 {
            // 16-bit instruction: value matches read16 (LE from memory).
            decode_inst16a(raw).is_some() || decode_inst16b(raw).is_some()
        } else {
            // 24-bit (standard) or 32-bit (format_32 TIE/DSP) instruction.
            decode_inst(raw).is_some()
        };
        checked += 1;
        *mnemonic_count.entry(mnem.to_string()).or_insert(0) += 1;
        if !decoded {
            failures
                .entry(mnem.to_string())
                .or_default()
                .push(format!("0x{:x}", raw));
        }
    }

    eprintln!("CHECKED {} instructions", checked);
    eprintln!("UNIQUE MNEMONICS {}", mnemonic_count.len());
    if failures.is_empty() {
        eprintln!("ALL INSTRUCTIONS DECODED OK");
    } else {
        eprintln!("FAILED MNEMONICS: {}", failures.len());
        for (m, ex) in &failures {
            eprintln!(
                "  {}  (count={})  e.g. {:?}",
                m,
                mnemonic_count[m],
                ex.first()
            );
        }
    }
}
