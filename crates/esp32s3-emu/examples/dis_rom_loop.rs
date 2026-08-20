use std::fs;

fn main() {
    let rom = fs::read("tools/esp32s3_rom/esp32s3_rom.bin").unwrap();
    let base = 0x4000_0000usize;
    let mut a = 0x41a50usize;
    let end = 0x41a90usize;
    while a < end {
        let b0 = rom[a];
        let len = xtensa_core::generated::insn_len(b0) as usize;
        let raw = u32::from_le_bytes([rom[a], rom[a + 1], rom[a + 2], rom[a + 3]]);
        let name = match xtensa_core::generated::decode_inst(raw) {
            Some(o) => format!("{o:?}"),
            None => format!("??"),
        };
        let mut extra = String::new();
        if name == "OPCODE_ENTRY" {
            let imm12 = raw >> 16 & 0xfff;
            extra = format!(" a1, {}", imm12 * 8);
        }
        if name == "OPCODE_RSR_CCOUNT" {
            extra = format!(" a{}", raw >> 4 & 0xf);
        }
        if name == "OPCODE_SUB" {
            extra = format!(
                " a{},a{},a{}",
                raw >> 12 & 0xf,
                raw >> 8 & 0xf,
                raw >> 4 & 0xf
            );
        }
        if name == "OPCODE_BLTU" {
            let imm8 = (raw >> 16 & 0xff) as i8;
            let tgt = ((base as u64 + a as u64 + 4) as i64 + imm8 as i64) as u32;
            extra = format!(" a{},a{},{:#x}", raw >> 8 & 0xf, raw >> 4 & 0xf, tgt);
        }
        if name == "OPCODE_L32R" {
            let imm16 = (raw >> 8 & 0xffff) as u32;
            let tgt = (((base as u64 + a as u64 + 3) & !3) + (imm16 << 2) as u64) as u32;
            extra = format!(" a{},{:#x}", raw >> 4 & 0xf, tgt);
        }
        if name == "OPCODE_CALL8" {
            let imm16 = (raw >> 8 & 0xffff) as u32;
            let tgt = ((base as u64 + a as u64 + 4) + (imm16 << 2) as u64) as u32;
            extra = format!(" {:#x}", tgt);
        }
        if name == "OPCODE_ADDI" || name == "OPCODE_ADDMI" {
            let imm = (raw >> 16 & 0xff) as i8;
            extra = format!(" a{},a{},{}", raw >> 12 & 0xf, raw >> 8 & 0xf, imm);
        }
        if name == "OPCODE_BNEZ" || name == "OPCODE_BEQZ" {
            let imm12 = (raw >> 16 & 0xfff) as i32;
            let tgt = ((base as u64 + a as u64 + 4) as i64 + imm12 as i64) as u32;
            extra = format!(" a{},{:#x}", raw >> 8 & 0xf, tgt);
        }
        if name == "OPCODE_MOVI" {
            let imm = ((raw >> 8 & 0xf0) | (raw >> 16 & 0xff)) as i8;
            extra = format!(" a{},{}", raw >> 12 & 0xf, imm);
        }
        if name == "OPCODE_L8UI" {
            extra = format!(
                " a{},a{},{}",
                raw >> 12 & 0xf,
                raw >> 8 & 0xf,
                raw >> 16 & 0xff
            );
        }
        if name == "OPCODE_S32I" {
            extra = format!(
                " a{},a{},{}(x4)",
                raw >> 12 & 0xf,
                raw >> 8 & 0xf,
                raw >> 16 & 0xff
            );
        }
        if name == "OPCODE_L32I" {
            extra = format!(
                " a{},a{},{}(x4)",
                raw >> 12 & 0xf,
                raw >> 8 & 0xf,
                raw >> 16 & 0xff
            );
        }
        println!(
            "{:08x}: {:08x} len={} {}{}",
            base + a,
            raw,
            len,
            name,
            extra
        );
        a += len;
    }
}
