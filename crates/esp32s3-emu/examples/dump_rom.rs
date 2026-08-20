//! Debug: disassemble the ROM stub loader region (0x40000400..0x40000480).
use esp32s3_emu::rom_stub::rom_image;

fn main() {
    let img = rom_image();
    let mut a = 0x400usize;
    while a < 0x480 {
        let b0 = img[a];
        let len = xtensa_core::generated::insn_len(b0) as usize;
        let raw = if len == 2 {
            u16::from_le_bytes([img[a], img[a + 1]]) as u32
        } else {
            u32::from_le_bytes([img[a], img[a + 1], img[a + 2], img[a + 3]])
        };
        let name = match xtensa_core::generated::decode_inst(raw) {
            Some(o) => format!("{o:?}"),
            None => format!("??"),
        };
        println!("{a:#06x}: {raw:#010x} {name}");
        a += len;
    }
}
