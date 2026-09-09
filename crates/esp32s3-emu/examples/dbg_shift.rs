//! Scratch: trace the real ROM __ashldi3 call (shift-di3 test replica).
//! Delete after debugging.
use esp32s3_emu::Esp32S3;
use esp32s3_emu::asm::Asm;
use esp32s3_emu::rom_stub;
use esp32s3_soc::memmap::IRAM_BASE;
use xtensa_core::Bus;
use xtensa_core::cpu::SR_PS;

fn main() {
    const STASH: u32 = 0x3FC8_0200;
    const CODE: u32 = IRAM_BASE + 0x8000;
    let value: u64 = 0x0123_4567_89AB_CDEF;
    let count: u32 = 0;

    let mut a = Asm::new(CODE);
    a.li(1, 0x3FC8_8000);
    a.li(3, 0x40000);
    a.wsr(SR_PS, 3);
    a.li(10, value as i32);
    a.li(11, (value >> 32) as i32);
    a.li(12, count as i32);
    a.li(8, 0x4000_21B4);
    a.callx8(8);
    a.li(2, STASH as i32);
    a.s32i(10, 2, 0);
    a.s32i(11, 2, 4);
    let halt = a.pc();
    a.j(halt);

    let mut m = Esp32S3::new();
    let rom = rom_stub::rom_image();
    m.load_image(rom_stub::ROM_BASE, &rom);
    m.load_image(CODE, a.bytes());
    m.cpu[0].pc = CODE;
    for i in 0..1000 {
        let pc = m.cpu[0].pc;
        let wb = m.cpu[0].windowbase();
        if i < 10
            || (0x4000_21B4..=0x4000_21C0).contains(&pc)
            || (0x4005_60D8..=0x4005_60F0).contains(&pc)
        {
            println!(
                "step {i}: pc={pc:#010x} wb={wb} a2={:#x} a3={:#x} a4={:#x} a8={:#x} a10={:#x} a11={:#x}",
                m.cpu[0].reg(2),
                m.cpu[0].reg(3),
                m.cpu[0].reg(4),
                m.cpu[0].reg(8),
                m.cpu[0].reg(10),
                m.cpu[0].reg(11),
            );
        }
        m.step();
        if m.cpu[0].pc == halt {
            break;
        }
    }
    let lo = m.soc.read32(STASH);
    let hi = m.soc.read32(STASH + 4);
    println!(
        "stash lo={lo:#x} hi={hi:#x} result={:#x} expected={value:#x}",
        lo as u64 | ((hi as u64) << 32)
    );
}
