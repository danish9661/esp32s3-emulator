//! TEMP (2026-10-03, forensics — DELETE after): length-stepped decode
//! of a byte range using our own decoder (objdump desyncs in vectors).
//! Usage: `cargo run -p esp32s3-emu --example walkvec -- <elf> <start-hex> <nbytes-hex>`
//! Reads raw file bytes — caller must pass the file offset, NOT vaddr.
//! Simpler: pass vaddr + section file offset separately is overkill; this
//! tool instead takes hex BYTE STRING (from `objdump -s`) + base vaddr.

use xtensa_core::generated::{decode_inst, decode_inst16a, decode_inst16b, insn_len, opnds};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let hexs = &args[1];
    let mut base = u32::from_str_radix(&args[2], 16).unwrap();
    let bytes: Vec<u8> = (0..hexs.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hexs[i..i + 2], 16).unwrap())
        .collect();
    let mut off = 0usize;
    while off < bytes.len() {
        let b0 = bytes[off];
        let len = insn_len(b0) as usize;
        if off + 4 > bytes.len() + 1 && len > bytes.len() - off {
            break;
        }
        let mut w = 0u32;
        for k in 0..4 {
            w |= (bytes.get(off + k).copied().unwrap_or(0) as u32) << (8 * k);
        }
        let opc = if len == 2 {
            let ww = w & 0xFFFF;
            if b0 & 0xf <= 11 {
                decode_inst16a(ww)
            } else {
                decode_inst16b(ww)
            }
        } else {
            decode_inst(w)
        };
        match opc {
            Some(op) => {
                let o = opnds(op, w, 0);
                let vals: Vec<String> = o
                    .iter()
                    .take(4)
                    .map(|x| format!("{}{}", if x.is_reg { "a" } else { "#" }, x.value as i32))
                    .collect();
                println!("{base:#010x}: {:?} {}", op, vals.join(" "));
            }
            None => println!("{base:#010x}: <illegal> raw={w:#010x}"),
        }
        base += len as u32;
        off += len;
    }
}
