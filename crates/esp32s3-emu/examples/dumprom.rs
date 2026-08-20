use xtensa_core::generated::{decode_inst, opnds};
fn main() {
    let data =
        std::fs::read("/home/danish1075/Documents/esp32 s3 emu/tools/esp32s3_rom/esp32s3_rom.bin")
            .unwrap();
    let base = 0x40000000u32;
    let (start, end) = (0x40048c00u32, 0x40048d40u32);
    let mut pc = start;
    while pc < end {
        let off = (pc - base) as usize;
        let w = u32::from_le_bytes([data[off], data[off + 1], data[off + 2], 0]);
        let b0 = (w & 0xff) as u32;
        let len = if b0 <= 7 { 3 } else { 2 };
        let insn = w & 0xffffff;
        match decode_inst(insn) {
            Some(op) => {
                let o = opnds(op, insn, pc);
                let vs: Vec<String> = o.iter().take(3).map(|x| format!("{}", x.value)).collect();
                println!("0x{:08x}: {:?} [{}]", pc, op, vs.join(","));
            }
            None => println!("0x{:08x}: ???? ({:06x})", pc, insn),
        }
        pc += len as u32;
    }
}
