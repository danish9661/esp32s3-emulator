use esp32s3_emu::Esp32S3;
use xtensa_core::Bus;

fn main() {
    let img = std::fs::read("tools/sketches/esp32s3_hello/esp32s3_hello.merged.bin").unwrap();
    let mut m = Esp32S3::new();
    m.boot_from_flash(&img);
    let mut released = 0u64;
    let mut ring: Vec<u32> = Vec::with_capacity(256);
    let mut jumped = false;
    for i in 0..80_000_000u64 {
        m.step();
        if ring.len() == 256 {
            ring.remove(0);
        }
        ring.push(m.cpu[0].pc);
        if m.cpu[0].pc == 0x4037_4300 && !jumped {
            jumped = true;
            println!("FIRST kernel vector at step {i}");
            println!("prev 256: {:x?}", ring.as_slice());
            println!(
                "ps={:#x} wb={} ws={:#x} a1={:#x} epc1={:#x} exccause={:#x}",
                m.cpu[0].sreg(230),
                m.cpu[0].windowbase(),
                m.cpu[0].sreg(73),
                m.cpu[0].reg(1),
                m.cpu[0].sreg(231),
                m.cpu[0].sreg(232),
            );
            break;
        }
        if m.cpu[1].pc >= 0x4030_0000 && released == 0 {
            released = i;
        }
    }
    println!(
        "done pc={:#x} core1pc={:#x} core1-released={released}",
        m.cpu[0].pc, m.cpu[1].pc
    );
}
