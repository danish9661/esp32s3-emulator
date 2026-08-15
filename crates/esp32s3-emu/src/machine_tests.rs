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
fn timer_interrupt_delivers_to_vector() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::INT_MATRIX_BASE;

    // App: literal pool first (CTR/STASH/INT_MATRIX/TIMG0), then code.
    // Configures TIMG0 T0 to alarm every 64 cycles (EN|INCREASE|AUTORELOAD|
    // ALARM, alarm = 0x40), routes source 50 (TG0_T0) to CPU line 15 (level
    // 3) via the interrupt matrix, enables INTENABLE bit 15, then spins on
    // CTR until the level-3 handler has run 3 times, then stashes 0xCAFE.
    // DRAM/IRAM alias the same SRAM: keep CTR/STASH above the app
    // image (0x3FC80000..0x3FC80060) so the handler write cannot clobber
    // the app literal pool at 0x40370000.
    const CTR: u32 = 0x3FC8_0100;
    const STASH: u32 = 0x3FC8_0104;
    let mut a = Asm::new(IRAM_BASE);
    let l_ctr = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let l_mat = a.offset();
    a.lit(0);
    let l_timg = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p_l2 = a.l32r(2); // INT_MATRIX_BASE
    a.patch_l32r(p_l2, IRAM_BASE + l_mat as u32);
    a.movi_n(3, 15);
    a.s32i(3, 2, 0xC8); // INT_MATRIX_BASE + 4*50: TG0_T0 -> line 15
    let p_l2 = a.l32r(2); // TIMG0_BASE
    a.patch_l32r(p_l2, IRAM_BASE + l_timg as u32);
    a.movi_n(3, 0x40);
    a.s32i(3, 2, 0x10); // T0ALARMLO = 64
    a.li(3, 0xE000_0400u32 as i32); // T0CONFIG: EN|INCREASE|AUTORELOAD|ALARM
    a.s32i(3, 2, 0);
    a.movi_n(4, 1);
    a.s32i(4, 2, 0x98); // INT_ENA bit 0
    a.li(3, 0x8000); // INTENABLE bit 15
    a.wsr(228, 3);
    a.rsil(4, 0);
    let loop_start = a.pc();
    let p_l2 = a.l32r(2); // CTR
    a.patch_l32r(p_l2, IRAM_BASE + l_ctr as u32);
    a.l32i(3, 2, 0);
    a.addi(4, 3, -3);
    a.bnez(4, loop_start);
    a.li(4, 0xCAFE);
    let p_l5 = a.l32r(5); // STASH
    a.patch_l32r(p_l5, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    // Patch the literal pool (values must sit at their 4-aligned slots).
    a.bytes_mut()[l_ctr..l_ctr + 4].copy_from_slice(&CTR.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());
    a.bytes_mut()[l_mat..l_mat + 4].copy_from_slice(&INT_MATRIX_BASE.to_le_bytes());
    a.bytes_mut()[l_timg..l_timg + 4].copy_from_slice(&TIMG0_BASE.to_le_bytes());

    // Level-3 handler at VECBASE + 0x1C0 (64-byte slot; uses only a6-a9 so
    // the main loop's a2/a3/a4 stay live across the interrupt).
    let mut h = Asm::new(0x4000_01C0);
    h.li(6, CTR as i32);
    h.l32i(7, 6, 0);
    h.addi(7, 7, 1);
    h.s32i(7, 6, 0); // CTR += 1
    h.li(8, TIMG0_BASE as i32);
    h.movi_n(9, 1);
    h.s32i(9, 8, 0xA4); // INT_CLR
    h.rfi(3);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_01C0, h.bytes());
    m.cpu.pc = code_start;
    for _ in 0..2000 {
        if m.cpu.pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu.pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after 3 interrupts");
    assert_eq!(m.soc.read32(CTR), 3, "handler ran 3 times");
    assert_eq!(m.take_uart_tx(0), Bytes::new(), "UART0 silent");
    assert_eq!(m.soc.read32(INT_MATRIX_BASE + 4 * 50), 15, "matrix write");
}

#[test]
fn timg1_alarm_delivers_level4_vector() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::{INT_MATRIX_BASE, TIMG1_BASE};

    // Same story as the level-3 test but through TIMG1: source 53 (TG1_T0)
    // routed to CPU line 24 (level 4), handler at VECBASE + 0x200.  Proves
    // the second timer group, a different matrix entry (4*53 = 0xD4) and the
    // level-4 vector path end to end.
    const CTR: u32 = 0x3FC8_0200;
    const STASH: u32 = 0x3FC8_0204;
    let mut a = Asm::new(IRAM_BASE);
    let l_ctr = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let l_mat = a.offset();
    a.lit(0);
    let l_timg = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // INT_MATRIX_BASE
    a.patch_l32r(p, IRAM_BASE + l_mat as u32);
    a.movi_n(3, 24);
    a.s32i(3, 2, 0xD4); // INT_MATRIX_BASE + 4*53: TG1_T0 -> line 24 (L4)
    let p = a.l32r(2); // TIMG1_BASE
    a.patch_l32r(p, IRAM_BASE + l_timg as u32);
    a.movi_n(3, 0x40);
    a.s32i(3, 2, 0x10); // T0ALARMLO = 64
    a.li(3, 0xE000_0400u32 as i32); // T0CONFIG: EN|INCREASE|AUTORELOAD|ALARM
    a.s32i(3, 2, 0);
    a.movi_n(4, 1);
    a.s32i(4, 2, 0x98); // INT_ENA bit 0
    a.li(3, 0x100_0000); // INTENABLE bit 24
    a.wsr(228, 3);
    a.rsil(4, 0);
    let loop_start = a.pc();
    let p = a.l32r(2); // CTR
    a.patch_l32r(p, IRAM_BASE + l_ctr as u32);
    a.l32i(3, 2, 0);
    a.addi(4, 3, -3);
    a.bnez(4, loop_start);
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    a.bytes_mut()[l_ctr..l_ctr + 4].copy_from_slice(&CTR.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());
    a.bytes_mut()[l_mat..l_mat + 4].copy_from_slice(&INT_MATRIX_BASE.to_le_bytes());
    a.bytes_mut()[l_timg..l_timg + 4].copy_from_slice(&TIMG1_BASE.to_le_bytes());

    // Level-4 handler at VECBASE + 0x200 (64-byte slot; a6-a9 only).
    let mut h = Asm::new(0x4000_0200);
    h.li(6, CTR as i32);
    h.l32i(7, 6, 0);
    h.addi(7, 7, 1);
    h.s32i(7, 6, 0); // CTR += 1
    h.li(8, TIMG1_BASE as i32);
    h.movi_n(9, 1);
    h.s32i(9, 8, 0xA4); // INT_CLR
    h.rfi(4);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_0200, h.bytes());
    m.cpu.pc = code_start;
    for _ in 0..2000 {
        if m.cpu.pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu.pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after 3 interrupts");
    assert_eq!(m.soc.read32(CTR), 3, "handler ran 3 times");
    assert_eq!(m.soc.read32(INT_MATRIX_BASE + 4 * 53), 24, "matrix write");
}

#[test]
fn uart_rx_interrupt_echo() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::INT_MATRIX_BASE;

    // UART0 RX path end to end: a host-injected byte latches
    // INT_RXFIFO_FULL, source 27 routes to line 15 (level 3), the vector
    // handler pops the byte (RXFIFO_FULL drops with the FIFO), echoes it
    // back on TX, clears INT_CLR and rfi 3; the main loop counts 1 byte.
    const RXCNT: u32 = 0x3FC8_0100;
    const RXBUF: u32 = 0x3FC8_0104;
    const STASH: u32 = 0x3FC8_0108;
    let mut a = Asm::new(IRAM_BASE);
    let l_cnt = a.offset();
    a.lit(0);
    let l_buf = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let l_mat = a.offset();
    a.lit(0);
    let l_uart = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // INT_MATRIX_BASE
    a.patch_l32r(p, IRAM_BASE + l_mat as u32);
    a.movi_n(3, 15);
    a.s32i(3, 2, 0x6C); // INT_MATRIX_BASE + 4*27: UART0 -> line 15 (L3)
    let p = a.l32r(2); // UART0_BASE
    a.patch_l32r(p, IRAM_BASE + l_uart as u32);
    a.movi_n(3, 1);
    a.s32i(3, 2, 0x0C); // INT_ENA bit 0 (RXFIFO_FULL)
    a.li(3, 0x8000); // INTENABLE bit 15
    a.wsr(228, 3);
    a.rsil(4, 0);
    let loop_start = a.pc();
    let p = a.l32r(2); // RXCNT
    a.patch_l32r(p, IRAM_BASE + l_cnt as u32);
    a.l32i(3, 2, 0);
    a.addi(4, 3, -1);
    a.bnez(4, loop_start);
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    a.bytes_mut()[l_cnt..l_cnt + 4].copy_from_slice(&RXCNT.to_le_bytes());
    a.bytes_mut()[l_buf..l_buf + 4].copy_from_slice(&RXBUF.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());
    a.bytes_mut()[l_mat..l_mat + 4].copy_from_slice(&INT_MATRIX_BASE.to_le_bytes());
    a.bytes_mut()[l_uart..l_uart + 4].copy_from_slice(&UART0_BASE.to_le_bytes());

    // Level-3 handler (a6-a9 only): pop FIFO, save to RXBUF (RXCNT sits 4
    // bytes below it, so one li covers both), count up, echo the byte on
    // TX, clear INT_CLR, rfi 3.
    let mut h = Asm::new(0x4000_01C0);
    h.li(6, UART0_BASE as i32);
    h.l32i(7, 6, 0); // pop RX byte
    h.li(8, RXBUF as i32);
    h.s32i(7, 8, 0);
    h.addi(8, 8, -4); // RXCNT = RXBUF - 4
    h.l32i(9, 8, 0);
    h.addi(9, 9, 1);
    h.s32i(9, 8, 0);
    h.s32i(7, 6, 0); // echo on TX
    h.movi_n(9, 1);
    h.s32i(9, 6, 0x10); // INT_CLR bit 0
    h.rfi(3);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_01C0, h.bytes());
    m.cpu.pc = code_start;
    m.soc.uart_inject_rx(0, b'X');
    for _ in 0..2000 {
        if m.cpu.pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu.pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(RXCNT), 1, "one byte received");
    assert_eq!(m.soc.read32(RXBUF), b'X' as u32, "handler saved the byte");
    assert_eq!(m.take_uart_tx(0), b"X".to_vec(), "echo on TX");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after RX interrupt");
    assert_eq!(m.soc.read32(INT_MATRIX_BASE + 4 * 27), 15, "matrix write");
}

#[test]
fn ledc_pwm_blinks_gpio0_at_50_percent_duty() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::LEDC_BASE;

    // Firmware: TIMER0 with divider 1.0 and duty_resolution field 9 (10-bit
    // period = 1024 timer ticks, one tick per APB cycle), channel 0 duty =
    // 0x20000 (50% of 18 bits) with duty_start | sig_out_en, then routes
    // LEDC_CH0 (GPIO-matrix signal 96) to GPIO0 and enables the pad.  The
    // host measures the pad: 512 cycles high / 512 cycles low.
    const STASH: u32 = 0x3FC8_0100;
    let mut a = Asm::new(IRAM_BASE);
    let l_ledc = a.offset();
    a.lit(0);
    let l_gpio = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // LEDC_BASE
    a.patch_l32r(p, IRAM_BASE + l_ledc as u32);
    a.li(3, 0x0024_0100);
    a.s32i(3, 2, 0); // TIMER0_CONF: clock_divider 1.0, duty_resolution 9
    a.li(3, 0x2_0000);
    a.s32i(3, 2, 0x28); // CH0_DUTY = 50% of the 18-bit range
    a.movi_n(3, 12);
    a.s32i(3, 2, 0x20); // CH0_CONF0: duty_start | sig_out_en
    let p = a.l32r(2); // GPIO_BASE
    a.patch_l32r(p, IRAM_BASE + l_gpio as u32);
    a.movi_n(3, 1);
    a.s32i(3, 2, 0x20); // ENABLE bit 0
    a.li(3, 73);
    a.s32i(3, 2, 0x54); // GPIO0 FUNC_OUT_SEL = LEDC_CH0 (signal 73)
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    a.bytes_mut()[l_ledc..l_ledc + 4].copy_from_slice(&LEDC_BASE.to_le_bytes());
    a.bytes_mut()[l_gpio..l_gpio + 4].copy_from_slice(&GPIO_BASE.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu.pc = code_start;
    let pin = |m: &Esp32S3| (m.gpio_output() & 1) != 0;
    for _ in 0..500 {
        if m.soc.read32(STASH) == 0xCAFE {
            break;
        }
        m.step();
    }
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "firmware configured LEDC");
    for _ in 0..2 {
        // Sync on a falling edge, then a rising edge: we may already be
        // mid-phase when the firmware finished configuring.
        while pin(&m) {
            m.step();
        }
        while !pin(&m) {
            m.step();
        }
        let mut high = 0u32;
        while pin(&m) {
            m.step();
            high += 1;
        }
        let mut low = 0u32;
        while !pin(&m) {
            m.step();
            low += 1;
        }
        assert!((510..=514).contains(&high), "high phase ~512, got {high}");
        assert!((510..=514).contains(&low), "low phase ~512, got {low}");
    }
}

#[test]
fn spi2_shifts_out_0xa5_on_gpio_pins() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::{GPIO_BASE, SPI2_BASE};

    // Firmware: route FSPICLK (101) -> GPIO1, FSPID (103) -> GPIO2,
    // FSPICS0 (110) -> GPIO3, then run an 8-bit MOSI-only CPU transfer of
    // 0xA5 on GPSPI2 at (clkdiv_pre+1)*(clkcnt_n+1) = 2 APB cycles per bit
    // and stash.  The host samples the pins and recovers the bit stream.
    const STASH: u32 = 0x3FC8_0100;
    let mut a = Asm::new(IRAM_BASE);
    let l_spi = a.offset();
    a.lit(0);
    let l_gpio = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // SPI2_BASE
    a.patch_l32r(p, IRAM_BASE + l_spi as u32);
    let p = a.l32r(3); // GPIO_BASE
    a.patch_l32r(p, IRAM_BASE + l_gpio as u32);
    a.movi_n(4, 14);
    a.s32i(4, 3, 0x20); // ENABLE bits 1,2,3
    a.li(4, 101);
    a.s32i(4, 3, 0x58); // GPIO1 FUNC_OUT_SEL = FSPICLK
    a.li(4, 103);
    a.s32i(4, 3, 0x5C); // GPIO2 FUNC_OUT_SEL = FSPID
    a.li(4, 110);
    a.s32i(4, 3, 0x60); // GPIO3 FUNC_OUT_SEL = FSPICS0
    a.li(4, 0x1000);
    a.s32i(4, 2, 0x0C); // SPI_CLOCK: clkdiv_pre=0, clkcnt_n=1 -> 2 cyc/bit
    a.movi_n(4, 7);
    a.s32i(4, 2, 0x1C); // SPI_MS_DLEN: 8 data bits
    a.li(4, 0xA5 << 24);
    a.s32i(4, 2, 0x98); // SPI_W0: left-aligned MSB-first data
    a.li(4, 1 << 27);
    a.s32i(4, 2, 0x10); // SPI_USER: usr_mosi
    a.movi_n(4, 1);
    a.s32i(4, 2, 0xE8); // SPI_CLK_GATE: clk_en
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    a.li(4, 1 << 24);
    a.s32i(4, 2, 0x00); // SPI_CMD: usr -> trigger transfer (last)
    let done = a.pc();
    a.j(done);
    a.bytes_mut()[l_spi..l_spi + 4].copy_from_slice(&SPI2_BASE.to_le_bytes());
    a.bytes_mut()[l_gpio..l_gpio + 4].copy_from_slice(&GPIO_BASE.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu.pc = code_start;
    // Sample encoding per pin: bit0 = GPIO1 = SPICLK, bit1 = GPIO2 =
    // SPID (MOSI), bit2 = GPIO3 = FSPICS0.
    let pins = |m: &Esp32S3| {
        let out = m.gpio_output();
        let ck = (out >> 1) & 1;
        let d = (out >> 2) & 1;
        let cs = (out >> 3) & 1;
        ck | (d << 1) | (cs << 2)
    };
    for _ in 0..500 {
        if m.soc.read32(STASH) == 0xCAFE {
            break;
        }
        m.step();
    }
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "firmware configured SPI2");
    // The CMD write is the instruction after the stash: step until CS0
    // goes low = the first cycle of the transaction (elapsed 0).
    while pins(&m) & 4 != 0 {
        m.step();
    }
    // Sample before each step: sample j sees the bus at elapsed j.
    let mut samples = [0u32; 40];
    for s in samples.iter_mut() {
        *s = pins(&m);
        m.step();
    }
    // 8 bits at 2 APB cycles each: bit i's midpoint (clock high) is
    // sample 2i+1, MSB first.
    let mut bits = 0u32;
    for i in 0..8 {
        let s = samples[2 * i + 1];
        assert_eq!(s & 1, 1, "mid-sample must be clock-high (slot {i})");
        assert_eq!(s & 4, 0, "CS0 active low (slot {i})");
        bits = (bits << 1) | ((s >> 1) & 1);
    }
    assert_eq!(bits, 0xA5, "MOSI bit stream");
    assert_eq!(samples[16] & 4, 4, "CS0 released after 16 cycles");
    assert_eq!(samples[16] & 1, 0, "clock idles low");
    // 8 bits * 2 cycles = 16 APB cycles: usr must self-clear afterwards.
    assert_eq!(
        m.soc.read32(SPI2_BASE) & (1 << 24),
        0,
        "CMD.usr self-clears"
    );
}

#[test]
fn i2c0_master_write_nacks_and_stops() {
    use crate::asm::Asm;
    use alloc::vec::Vec;
    use esp32s3_soc::memmap::{GPIO_BASE, I2C0_BASE};

    // Firmware: route I2CEXT0_SCL (89) -> GPIO1, I2CEXT0_SDA (90) -> GPIO2,
    // configure I2C0 as master (ms_mode), push address 0xA0 (0x50<<1|W) and
    // data 0xAA into the TX FIFO, then run the command list
    // RSTART|WRITE(1)|WRITE(1)|STOP|END via ctr.trans_start and stash.  The
    // stash precedes the trigger (like the SPI test): the host syncs on the
    // START condition (SDA falling while SCL high) and recovers the 18 SCL
    // pulses with their SDA levels (8 addr bits + NACK + 8 data bits + NACK),
    // then verifies the STOP condition and bus idle.
    // Timing: scl_low_period=1 -> 2 APB cycles low, scl_high_period=1
    // (wait=0) -> 1 APB cycle high, start/stop holds = 0 -> 1 cycle.
    const STASH: u32 = 0x3FC8_0100;
    let mut a = Asm::new(IRAM_BASE);
    let l_i2c = a.offset();
    a.lit(0);
    let l_gpio = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // I2C0_BASE
    a.patch_l32r(p, IRAM_BASE + l_i2c as u32);
    let p = a.l32r(3); // GPIO_BASE
    a.patch_l32r(p, IRAM_BASE + l_gpio as u32);
    a.movi_n(4, 6);
    a.s32i(4, 3, 0x20); // ENABLE bits 1,2
    a.li(4, 89);
    a.s32i(4, 3, 0x58); // GPIO1 FUNC_OUT_SEL = I2CEXT0_SCL
    a.li(4, 90);
    a.s32i(4, 3, 0x5C); // GPIO2 FUNC_OUT_SEL = I2CEXT0_SDA
    a.li(4, 1 << 4);
    a.s32i(4, 2, 0x04); // I2C_CTR: ms_mode
    a.movi_n(4, 1);
    a.s32i(4, 2, 0x00); // scl_low_period = 1 -> 2 APB cycles low
    a.s32i(4, 2, 0x38); // scl_high_period = 1 -> 1 APB cycle high
    a.s32i(4, 2, 0x40); // scl_start_hold = 0 -> 1
    a.s32i(4, 2, 0x48); // scl_stop_hold = 0 -> 1
    a.s32i(4, 2, 0x4C); // scl_stop_setup = 0 -> 1
    a.li(4, 0xA0);
    a.s32i(4, 2, 0x1C); // I2C_DATA: TX FIFO byte 1 (address 0x50<<1|W)
    a.li(4, 0xAA);
    a.s32i(4, 2, 0x1C); // I2C_DATA: TX FIFO byte 2
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    a.li(4, 6 << 11);
    a.s32i(4, 2, 0x58); // comd0: RSTART
    a.li(4, (1 << 11) | 1);
    a.s32i(4, 2, 0x5C); // comd1: WRITE 1 byte
    a.s32i(4, 2, 0x60); // comd2: WRITE 1 byte
    a.li(4, 2 << 11);
    a.s32i(4, 2, 0x64); // comd3: STOP
    a.li(4, 4 << 11);
    a.s32i(4, 2, 0x68); // comd4: END
    a.li(4, (1 << 5) | (1 << 4));
    a.s32i(4, 2, 0x04); // I2C_CTR: trans_start -> run the command list
    let done = a.pc();
    a.j(done);
    a.bytes_mut()[l_i2c..l_i2c + 4].copy_from_slice(&I2C0_BASE.to_le_bytes());
    a.bytes_mut()[l_gpio..l_gpio + 4].copy_from_slice(&GPIO_BASE.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu.pc = code_start;
    // Sample encoding: bit0 = GPIO1 = SCL, bit1 = GPIO2 = SDA.
    let pins = |m: &Esp32S3| {
        let out = m.gpio_output();
        ((out >> 1) & 1) | (((out >> 2) & 1) << 1)
    };
    for _ in 0..600 {
        if m.soc.read32(STASH) == 0xCAFE {
            break;
        }
        m.step();
    }
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "firmware configured I2C0");
    // The stash precedes the trans_start write: sample the whole
    // transaction (57 cycles) plus margin.
    let mut samples = Vec::new();
    for _ in 0..200 {
        samples.push(pins(&m));
        m.step();
    }
    // START condition: SDA falls while SCL is high (1,1) -> (1,0).
    let start = samples
        .windows(2)
        .position(|w| w[0] == 3 && w[1] == 1)
        .expect("START condition (SDA falling while SCL high)");
    // Recover bits at each SCL rising edge: 8 addr bits + NACK + 8 data
    // bits + NACK = 18 pulses.
    let mut bits = Vec::new();
    for k in start + 1..samples.len() {
        if samples[k] & 1 == 1 && samples[k - 1] & 1 == 0 {
            bits.push(samples[k] >> 1);
        }
    }
    assert_eq!(bits.len(), 18, "18 SCL pulses (addr + NACK + data + NACK)");
    let expected = [
        1, 0, 1, 0, 0, 0, 0, 0, // 0xA0 = addr 0x50, write
        1, // NACK (no device)
        1, 0, 1, 0, 1, 0, 1, 0, // 0xAA
        1, // NACK
    ];
    for (k, e) in expected.iter().enumerate() {
        assert_eq!(bits[k], *e, "SDA level at pulse {k}");
    }
    // STOP condition: SDA rises while SCL is high after the last pulse.
    let last_edge = (start + 1..samples.len())
        .filter(|&k| samples[k] & 1 == 1 && samples[k - 1] & 1 == 0)
        .nth(17)
        .unwrap();
    let stop = samples[last_edge + 1..]
        .windows(2)
        .position(|w| w[0] == 1 && w[1] == 3)
        .expect("STOP condition (SDA rising while SCL high)");
    assert!(stop > 0, "STOP after the ACK pulse");
    // Bus idle high afterwards.
    for s in samples.iter().skip(last_edge + 1 + stop + 2) {
        assert_eq!(*s, 3, "bus idle (1,1) after STOP");
    }
    // Controller state: NACK latched, all command slots done, trans
    // complete interrupt raw raised, TX FIFO drained.
    assert_eq!(m.soc.read32(I2C0_BASE + 0x08) & 1, 1, "resp_rec = NACK");
    for off in [0x58, 0x5C, 0x60, 0x64, 0x68] {
        assert_ne!(
            m.soc.read32(I2C0_BASE + off) & (1 << 31),
            0,
            "comd {off:#x} done"
        );
    }
    assert_ne!(
        m.soc.read32(I2C0_BASE + 0x20) & (1 << 7),
        0,
        "trans_complete raw"
    );
}

#[test]
fn adc1_oneshot_reads_injected_voltage() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::SENS_BASE;

    // Firmware drives the RTC oneshot path exactly like ESP-IDF's
    // adc_oneshot driver: select the RTC controller, set the channel bitmap
    // + SW start force, spin on sar_slave_addr1.meas_status, then start
    // (start_sar 0 -> 1), spin on meas1_done_sar and stash the raw result.
    // The host injects 825 mV on ADC1 channel 2 at 0 dB (full-scale 1.1 V)
    // -> 825 * 4095 / 1100 = 3071.
    const STASH: u32 = 0x3FC8_0200;
    let mut a = Asm::new(IRAM_BASE);
    let l_sens = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // SENS_BASE
    a.patch_l32r(p, IRAM_BASE + l_sens as u32);
    let p = a.l32r(3); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.movi_n(4, 0);
    a.s32i(4, 2, 0x10); // meas1_mux = 0: RTC controller (dig_force = 0)
    a.s32i(4, 2, 0x14); // sar_atten1 = 0: channel 2 at 0 dB
    a.li(4, (1 << 31) | (1 << 18) | (1 << 21)); // en_pad_force|start_force|ch2
    a.s32i(4, 2, 0x0C); // meas1_ctrl2
    let poll = a.pc();
    a.l32i(4, 2, 0x40); // slave_addr1: meas_status
    a.li(5, 0xFF << 22);
    a.and(4, 4, 5);
    a.bnez(4, poll); // wait for the shared SAR FSM idle
    a.li(4, (1 << 31) | (1 << 18) | (1 << 21));
    a.s32i(4, 2, 0x0C); // start_sar = 0
    a.li(4, (1 << 31) | (1 << 18) | (1 << 21) | (1 << 17)); // start_sar = 1
    a.s32i(4, 2, 0x0C);
    let done = a.pc();
    a.l32i(4, 2, 0x0C); // meas1_ctrl2
    a.li(5, 1 << 16); // meas1_done_sar
    a.and(4, 4, 5);
    a.beqz(4, done); // spin until the conversion is done
    a.l32i(4, 2, 0x0C);
    a.li(5, 0xFFFF); // meas1_data_sar [15:0]
    a.and(4, 4, 5);
    a.s32i(4, 3, 0); // stash the raw result
    let halt = a.pc();
    a.j(halt);
    a.bytes_mut()[l_sens..l_sens + 4].copy_from_slice(&SENS_BASE.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());

    let mut m = Esp32S3::new();
    m.soc.adc_inject_voltage(0, 2, 825);
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu.pc = code_start;
    for _ in 0..400 {
        if m.soc.read32(STASH) != 0 {
            break;
        }
        m.step();
    }
    assert_eq!(m.soc.read32(STASH), 825 * 4095 / 1100, "ADC1 oneshot raw");
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
