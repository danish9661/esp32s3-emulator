//! ESP32-S3 machine-level tests: load hand-assembled bare-metal programs into
//! IRAM and verify UART output, GPIO behavior, and timer registers.
//!
//! NOTE on program layout: Xtensa instructions are variable-length (2/3/4
//! bytes).  Test streams are built byte-by-byte at their true offsets with
//! `insn()`; a fixed 4-byte-per-word layout misaligns after the first 3-byte
//! instruction (verified failure mode 2026-08-15).

use esp32s3_soc::memmap::{GPIO_BASE, IRAM_BASE, TIMG0_BASE, UART0_BASE};

use xtensa_core::Bus;

use crate::Esp32S3;

type Bytes = alloc::vec::Vec<u8>;

/// Append the low `len` bytes of `word` (little-endian) to the stream.
fn insn(stream: &mut Bytes, word: u32, len: u32) {
    let b = word.to_le_bytes();
    stream.extend_from_slice(&b[..len as usize]);
}

/// Append a 32-bit literal (l32r target) to the stream.
fn lit(stream: &mut Bytes, word: u32) {
    insn(stream, word, 4);
}

#[test]
fn uart0_hello_world() {
    let mut m = Esp32S3::new();
    // movi a2,0x60; slli a2,a2,24; movi.n a3,0x48('H'); s32i a3,a2,0;
    // l32i a4,a2,7(STATUS); movi.n a3,0x69('i'); s32i a3,a2,0; j .
    let mut s = Bytes::new();
    insn(&mut s, 0x0060_A022, 3); // movi a2, 0x60
    insn(&mut s, 0x0001_2280, 3); // slli a2, a2, 24           -> a2 = 0x60000000
    insn(&mut s, 0x0000_834C, 2); // movi.n a3, 0x48           'H'
    insn(&mut s, 0x0000_6232, 3); // s32i a3, a2, 0            UART_FIFO <- 'H'
    insn(&mut s, 0x0007_2242, 3); // l32i a4, a2, 7            read UART_STATUS
    insn(&mut s, 0x0021_C332, 3); // addi a3, a3, 0x21           'i'
    insn(&mut s, 0x0000_6232, 3); // s32i a3, a2, 0            UART_FIFO <- 'i'
    insn(&mut s, 0x00FF_FF06, 3); // j .                      (self-loop)
    m.load_image(IRAM_BASE, &s);
    m.cpu.pc = IRAM_BASE;
    for _ in 0..10 {
        m.step();
    }
    let out = m.take_uart_tx(0);
    assert_eq!(out, b"Hi", "UART0 TX bytes");
    assert_eq!(m.take_uart_tx(1), Bytes::new(), "UART1 silent");
    assert_eq!(m.take_uart_tx(2), Bytes::new(), "UART2 silent");
}

#[test]
fn gpio_out_w1ts_w1tc() {
    let mut m = Esp32S3::new();
    // Literal pool at 0x40370000 (stash addrs), code at 0x40370008:
    // movi a2,0x60; slli a2,a2,24; movi.n a3,1; l32r a5,lit0; l32r a6,lit1;
    // addmi a2,a2,0x40 (GPIO); s32i a3,a2,9 (ENABLE_W1TS); s32i a3,a2,2
    // (OUT_W1TS); l32i a4,a2,1; s32i a4,a5,0; s32i a3,a2,3 (OUT_W1TC);
    // l32i a4,a2,1; s32i a4,a6,0; j .
    let mut s = Bytes::new();
    lit(&mut s, 0x3FC8_0004); // lit0 @ 0x40370000: stash addr A
    lit(&mut s, 0x3FC8_0010); // lit1 @ 0x40370004: stash addr B
    insn(&mut s, 0x0060_A022, 3); // movi a2, 0x60
    insn(&mut s, 0x0001_2280, 3); // slli a2, a2, 24
    insn(&mut s, 0x0000_130C, 2); // movi.n a3, 1
    insn(&mut s, 0x00FF_FC51, 3); // l32r a5, lit0 @ 0x40370010 -> 0x40370000
    insn(&mut s, 0x00FF_FC61, 3); // l32r a6, lit1 @ 0x40370013 -> 0x40370004
    insn(&mut s, 0x0040_D222, 3); // addmi a2, a2, 0x40          -> 0x60004000
    insn(&mut s, 0x0009_6232, 3); // s32i a3, a2, 9              GPIO_ENABLE_W1TS
    insn(&mut s, 0x0002_6232, 3); // s32i a3, a2, 2              GPIO_OUT_W1TS
    insn(&mut s, 0x0001_2242, 3); // l32i a4, a2, 1              read GPIO_OUT
    insn(&mut s, 0x0000_6542, 3); // s32i a4, a5, 0              stash -> A
    insn(&mut s, 0x0003_6232, 3); // s32i a3, a2, 3              GPIO_OUT_W1TC
    insn(&mut s, 0x0001_2242, 3); // l32i a4, a2, 1              read GPIO_OUT
    insn(&mut s, 0x0000_6642, 3); // s32i a4, a6, 0              stash -> B
    insn(&mut s, 0x00FF_FF06, 3); // j .
    m.load_image(IRAM_BASE, &s);
    m.cpu.pc = IRAM_BASE + 8; // code starts after the literal pool
    for _ in 0..16 {
        m.step();
    }
    assert_eq!(m.soc.read32(0x3FC8_0004), 1, "GPIO_OUT after W1TS");
    assert_eq!(m.soc.read32(0x3FC8_0010), 0, "GPIO_OUT after W1TC");
    assert_eq!(m.gpio_output(), 0, "output() = OUT & ENABLE, bit0 cleared");
}

#[test]
fn timg0_load_and_read() {
    let mut m = Esp32S3::new();
    // Literal at 0x40370000 (stash addr), code at 0x40370004:
    // movi a2,0x60; slli a2,a2,24; movi.n a3,16; movi.n a4,31; l32r a5,lit0;
    // slli a4,a4,12 (0x1F000); add a2,a2,a4 (TIMG0 base);
    // slli a3,a3,24; slli a3,a3,2; slli a3,a3,1 (0x80000000 EN);
    // movi.n a4,5; s32i a3,a2,0 (T0CONFIG); s32i a4,a2,6 (T0LOADLO);
    // s32i a4,a2,8 (T0LOAD); l32i a4,a2,1 (T0LO); s32i a4,a5,0;
    // s32i a3,a2,3 (T0UPDATE); l32i a4,a2,2 (T0HI); s32i a4,a5,1; j .
    let mut s = Bytes::new();
    lit(&mut s, 0x3FC8_0010); // lit0 @ 0x40370000: stash addr
    insn(&mut s, 0x0060_A022, 3); // movi a2, 0x60
    insn(&mut s, 0x0001_2280, 3); // slli a2, a2, 24
    insn(&mut s, 0x0000_031C, 2); // movi.n a3, 16
    insn(&mut s, 0x0000_F41C, 2); // movi.n a4, 31
    insn(&mut s, 0x00FF_FC51, 3); // l32r a5, lit0 @ 0x4037000E -> 0x40370000
    insn(&mut s, 0x0011_4440, 3); // slli a4, a4, 12               -> 0x1F000
    insn(&mut s, 0x0080_2240, 3); // add a2, a2, a4                -> 0x6001F000
    insn(&mut s, 0x0001_3380, 3); // slli a3, a3, 24               -> 0x10000000
    insn(&mut s, 0x0011_33E0, 3); // slli a3, a3, 2                -> 0x40000000
    insn(&mut s, 0x0011_33F0, 3); // slli a3, a3, 1                -> 0x80000000
    insn(&mut s, 0x0000_540C, 2); // movi.n a4, 5
    insn(&mut s, 0x0000_6232, 3); // s32i a3, a2, 0                T0CONFIG = EN
    insn(&mut s, 0x0006_6242, 3); // s32i a4, a2, 6                T0LOADLO <- 5
    insn(&mut s, 0x0008_6242, 3); // s32i a4, a2, 8                T0LOAD
    insn(&mut s, 0x0001_2242, 3); // l32i a4, a2, 1                read T0LO
    insn(&mut s, 0x0000_6542, 3); // s32i a4, a5, 0                stash T0LO
    insn(&mut s, 0x0003_6232, 3); // s32i a3, a2, 3                T0UPDATE latch
    insn(&mut s, 0x0002_2242, 3); // l32i a4, a2, 2                read T0HI
    insn(&mut s, 0x0001_6542, 3); // s32i a4, a5, 1                stash T0HI
    insn(&mut s, 0x00FF_FF06, 3); // j .
    m.load_image(IRAM_BASE, &s);
    m.cpu.pc = IRAM_BASE + 4; // code starts after the (single) literal
    for _ in 0..22 {
        m.step();
    }
    // T0LOAD writes 5; the machine ticks once per step before the CPU runs,
    // so the T0LO read (next step) sees 5 - 1 = 4 (default DECREASE).
    assert_eq!(m.soc.read32(0x3FC8_0010), 4, "T0LO after T0LOAD + 1 tick");
    assert_eq!(m.soc.read32(0x3FC8_0014), 0, "T0HI after T0UPDATE latch");
}

#[test]
fn timg0_counts_on_tick() {
    let mut m = Esp32S3::new();
    // Enable T0 (EN bit 31); default direction is DECREASE (INCREASE bit 30
    // clear, TRM TIMG_T0CONFIG).
    m.soc.write32(TIMG0_BASE, 0x8000_0000);
    m.soc.tick_timers(5);
    assert_eq!(
        m.soc.read32(TIMG0_BASE + 0x04),
        0xFFFF_FFFB,
        "down-count after 5 ticks"
    );
    // EN | INCREASE, load 1, tick 3 -> 4.
    m.soc.write32(TIMG0_BASE, 0xC000_0000);
    m.soc.write32(TIMG0_BASE + 0x18, 1); // T0LOADLO
    m.soc.write32(TIMG0_BASE + 0x20, 0); // T0LOAD
    m.soc.tick_timers(3);
    assert_eq!(
        m.soc.read32(TIMG0_BASE + 0x04),
        4,
        "up-count after load+3 ticks"
    );
    // INT_ST = RAW & ENA: nothing enabled yet.
    assert_eq!(m.soc.read32(TIMG0_BASE + 0xA0), 0, "INT_ST empty");
}

#[test]
fn bus_sanity() {
    let mut m = Esp32S3::new();
    // DRAM write visible through the IRAM alias (same physical SRAM).
    m.soc.write32(0x3FC8_0000, 0xDEAD_BEEF);
    assert_eq!(m.soc.read32(0x4037_0000), 0xDEAD_BEEF, "IRAM alias of DRAM");
    // Sub-word writes merge correctly.
    m.soc.write16(0x3FC8_0004, 0x1234);
    assert_eq!(m.soc.read32(0x3FC8_0004), 0x1234);
    m.soc.write8(0x3FC8_0008, 0xAB);
    assert_eq!(m.soc.read32(0x3FC8_0008), 0xAB);
    // IROM is read-only.
    m.soc.write32(0x4000_0000, 0x1234_5678);
    assert_eq!(m.soc.read32(0x4000_0000), 0, "IROM write ignored");
    // Unimplemented APB space reads 0, writes ignored.
    m.soc.write32(0x6003_0000, 0x55);
    assert_eq!(m.soc.read32(0x6003_0000), 0, "unmapped APB reads 0");
    // UART registers are latched.
    m.soc.write32(UART0_BASE + 0x14, 0x2A); // UART_CLKDIV
    assert_eq!(m.soc.read32(UART0_BASE + 0x14), 0x2A);
    // GPIO reset state: strap = flash boot, OUT/ENABLE = 0.
    assert_eq!(m.soc.read32(GPIO_BASE + 0x38), 0x4, "GPIO_STRAP flash boot");
    assert_eq!(m.soc.read32(GPIO_BASE + 0x04), 0, "GPIO_OUT reset");
}

#[test]
fn boot_reset_vector_is_irom() {
    let m = Esp32S3::new();
    assert_eq!(m.cpu.pc, 0x4000_0000, "CPU boots at the ROM reset vector");
}

/// ESP-IDF-style app image: 24-byte esp_image_header_t + one segment
/// (esp_image_format.h; the ROM stub does not pad/checksum segments).
fn esp_app_image(load_addr: u32, entry: u32, data: &[u8]) -> Bytes {
    let mut img = Bytes::from(&[0xE9, 1, 0, 0][..]); // magic, count, mode, speed
    img.extend_from_slice(&entry.to_le_bytes()); // entry_addr
    img.extend_from_slice(&[0u8; 13]); // wp_pin(1) + spi_pin_drv[3] + reserved[9]
    img.extend_from_slice(&[0u8; 3]); // pad the 21-byte tail to the 24-byte
    // header (the real struct is aligned to 24 bytes)
    img.extend_from_slice(&load_addr.to_le_bytes()); // segment load_addr
    img.extend_from_slice(&(data.len() as u32).to_le_bytes()); // data_len
    img.extend_from_slice(data);
    img
}

#[test]
fn boot_path_loads_app_from_flash() {
    use crate::asm::Asm;
    use crate::partition::parse_partition_table;
    use crate::rom_stub::{APP_FLASH_OFFSET, ROM_PUTS};
    use esp32s3_soc::memmap::IRAM_BASE;

    // App: literals first (4-aligned so l32r imm16 = 0), then code:
    // l32r a2,str; l32r a3,rom_puts; callx0 a3 (call0/j cannot reach the ROM
    // from IRAM: 0x370000 > 18-bit offset); li a4,0xCAFE; li a5,stash;
    // s32i a4,a5,0; j .
    const APP_ENTRY: u32 = IRAM_BASE + 8;
    const STASH: u32 = 0x3FC8_0000;
    let mut a = Asm::new(IRAM_BASE);
    let p_str = a.offset(); // literal pool: string address (patched below)
    a.lit(0);
    a.lit(ROM_PUTS); // ...and the ROM puts address
    let p_l2 = a.l32r(2);
    let p_l3 = a.l32r(3);
    a.patch_l32r(p_l2, IRAM_BASE);
    a.patch_l32r(p_l3, IRAM_BASE + 4);
    a.movi(15, 0); // pad: JX/RET clears the low 2 bits of the target, so the
    // callx0 must sit at pc ≡ 1 (mod 4) for pc+3 to be 4-aligned
    a.callx0(3);
    a.li(4, 0xCAFE);
    a.li(5, STASH as i32);
    a.s32i(4, 5, 0);
    let here = a.pc();
    a.j(here);
    let str_addr = a.pc(); // string lives right after the code
    a.bytes_mut().extend_from_slice(b"OK\n\0");
    a.bytes_mut()[p_str..p_str + 4].copy_from_slice(&str_addr.to_le_bytes());
    let app = a.bytes().to_vec();
    assert_eq!(a.bytes()[4..8], ROM_PUTS.to_le_bytes(), "puts addr lit");

    // Full flash image: partition table (1 nvs entry + terminator) and the
    // app at APP_FLASH_OFFSET.
    let mut flash = std::vec![0xFFu8; 0x200_000];
    flash[0x8000] = 0x50;
    flash[0x8001] = 0xAA;
    flash[0x8002] = 1; // type: nvs
    flash[0x8003] = 2; // subtype: no_keep
    flash[0x8004..0x8008].copy_from_slice(&0x9000u32.to_le_bytes());
    flash[0x8008..0x800C].copy_from_slice(&0x6000u32.to_le_bytes());
    flash[0x801C..0x8020].copy_from_slice(&0x55u32.to_le_bytes()); // flags
    flash[0x8020] = 0xEB;
    flash[0x8021] = 0xEB;
    let img = esp_app_image(IRAM_BASE, APP_ENTRY, &app);
    flash[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + img.len()].copy_from_slice(&img);

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    for _ in 0..1000 {
        if m.cpu.pc == here {
            break;
        }
        m.step();
    }
    assert_eq!(m.take_uart_tx(0), b"OK\n", "app printed via ROM rom_puts");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "app stash write");
    assert_eq!(m.cpu.pc, here, "app reached its self-loop");
    assert_eq!(parse_partition_table(&flash).unwrap().len(), 1);
}

#[test]
fn flash_xip_reads_and_readonly() {
    use esp32s3_soc::memmap::{FLASH_DATA_BASE, FLASH_INST_BASE};
    let mut m = Esp32S3::new();
    let img = b"ESP32S3!!".to_vec();
    m.soc.load_flash_image(0x1000, &img);
    assert_eq!(
        m.soc.read32(FLASH_DATA_BASE + 0x1000),
        u32::from_le_bytes(*b"ESP3"),
        "XIP data window read32"
    );
    assert_eq!(
        m.soc.read16(FLASH_INST_BASE + 0x1004),
        u16::from_le_bytes(*b"2S") as u32,
        "XIP instruction window read16"
    );
    assert_eq!(
        m.soc.read8(FLASH_INST_BASE + 0x1006),
        b'3' as u32,
        "XIP read8"
    );
    // Writes to flash are ignored (read-only backing store).
    m.soc.write32(FLASH_DATA_BASE + 0x1000, 0);
    assert_eq!(
        m.soc.read32(FLASH_DATA_BASE + 0x1000),
        u32::from_le_bytes(*b"ESP3")
    );
    // Reads beyond the 4 MB physical flash come back 0.
    assert_eq!(m.soc.read8(FLASH_DATA_BASE + 0x100_0000), 0, "beyond flash");
}
