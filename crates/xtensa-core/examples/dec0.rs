fn main() {
    let raw: u32 = 0;
    println!(
        "decode_inst(0) = {:?}",
        xtensa_core::generated::decode_inst(raw)
    );
    let b0 = raw as u8;
    println!(
        "insn_len({:#04x}) = {}",
        b0,
        xtensa_core::generated::insn_len(b0)
    );
}
