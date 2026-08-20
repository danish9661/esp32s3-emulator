//! Temporary disassembler: decode a list of words from stdin/args with our
//! generated decoder (authoritative — matches QEMU's decode table).
//! Usage: `cargo run -p xtensa-core --example dis -- <hex words...>`

use xtensa_core::generated::{decode_inst, opnds};

fn main() {
    for a in std::env::args().skip(1) {
        let w = u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap();
        match decode_inst(w) {
            Some(op) => {
                let o = opnds(op, w, 0);
                let names: Vec<String> = o.iter().map(|x| format!("{:?}", x)).collect();
                println!("{w:#010x}: {:?}  opnds [{}]", op, names.join(", "));
            }
            None => println!("{w:#010x}: <illegal>"),
        }
    }
}
