//! ESP32-S3 machine-level tests: load hand-assembled bare-metal programs into
//! IRAM and verify UART output, GPIO behavior, and timer registers.
//!
//! NOTE on program layout: Xtensa instructions are variable-length (2/3/4
//! bytes).  Test streams are built byte-by-byte at their true offsets with
//! `insn()`; a fixed 4-byte-per-word layout misaligns after the first 3-byte
//! instruction (verified failure mode 2026-08-15).

use esp32s3_soc::gdma::{GDMA_BASE, GDMA_I2S0_PERIPH, GDMA_SPI2_PERIPH};
use esp32s3_soc::gpio::{GPIO_ENABLE_W1TS, GPIO_OUT_W1TC, GPIO_OUT_W1TS};
use esp32s3_soc::memmap::{
    ASSIST_DEBUG_BASE, GPIO_BASE, I2S0_BASE, I2S1_BASE, IRAM_BASE, LCD_CAM_BASE, PERI_BACKUP_BASE,
    SENSITIVE_BASE, SPI2_BASE, SYSCON_BASE, TIMG0_BASE, UART0_BASE, WCL_BASE,
};
use esp32s3_soc::sdmmc::{
    BLKSIZ, BYTCNT, CMD, CMDARG, IDMAC_CTRL, IDMAC_DBADDR, RINTSTS, SDMMC_BASE,
};
use esp32s3_soc::soc::EVT_GPIO;

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

/// Enable a SYSCON peripheral clock (PERIP_CLK_EN0 @ +0x18 / EN1 @ +0x1C),
/// required since clock gating: peripherals freeze with their bit clear
/// (firmware normally enables these via periph_module_enable; hand-written
/// tests must do it explicitly, like silicon requires).
fn syscon_clk(m: &mut Esp32S3, en1: bool, bit: u32) {
    use esp32s3_soc::memmap::SYSTEM_BASE;
    let off = if en1 { 0x1C } else { 0x18 };
    let cur = m.soc.read32(SYSTEM_BASE + off);
    m.soc.write32(SYSTEM_BASE + off, cur | (1 << bit));
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
    m.cpu[0].pc = IRAM_BASE;
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
    m.cpu[0].pc = IRAM_BASE + 8; // code starts after the literal pool
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
    m.cpu[0].pc = IRAM_BASE + 4; // code starts after the (single) literal
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
    assert_eq!(m.soc.read32(TIMG0_BASE + 0x78), 0, "INT_ST empty");
}

#[test]
fn bus_sanity() {
    let mut m = Esp32S3::new();
    // D/IRAM write visible through the instruction alias (same physical
    // SRAM1 cells: data 0x3FC88000 == instruction 0x40378000, offset
    // 0x6F0000).  SRAM0 (0x40370000) is instruction-only — NOT an alias.
    m.soc.write32(0x3FC8_8000, 0xDEAD_BEEF);
    assert_eq!(m.soc.read32(0x4037_8000), 0xDEAD_BEEF, "D/IRAM alias");
    assert_eq!(m.soc.read32(0x4037_0000), 0, "SRAM0 has no DRAM alias");
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
    assert_eq!(
        m.cpu[0].pc, 0x4000_0400,
        "CPU boots at the ROM reset vector (core-isa.h XCHAL_RESET_VECTOR_PADDR)"
    );
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
    flash[0x8000] = 0xAA;
    flash[0x8001] = 0x50;
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
        if m.cpu[0].pc == here {
            break;
        }
        m.step();
    }
    // rom_puts is a no-op (console goes through ets_printf → putc1);
    // boot correctness is proven by the stash + PC checks below.
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "app stash write");
    assert_eq!(m.cpu[0].pc, here, "app reached its self-loop");
    assert_eq!(parse_partition_table(&flash).unwrap().len(), 1);
}

/// The ROM stub loader skips flash-mapped (XIP) segments instead of
/// byte-copying them: a 3-segment app (DRAM, XIP @0x42000000, DRAM marker)
/// must load both DRAM segments. If the skip mis-advances the cursor past
/// the XIP data, the third header misparses and MARKER never lands.
#[test]
fn boot_loader_skips_flash_mapped_segments() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::IRAM_BASE;

    const APP_ENTRY: u32 = IRAM_BASE;
    const STASH: u32 = 0x3FC8_0500;
    const MARKER: u32 = 0x3FC8_0504;
    let mut a = Asm::new(IRAM_BASE);
    a.li(4, 0x1234);
    a.li(5, STASH as i32);
    a.s32i(4, 5, 0);
    let here = a.pc();
    a.j(here);
    let app = a.bytes().to_vec();

    let filler = [0xAAu8; 64];
    let img = esp_app_image_multi(
        APP_ENTRY,
        &[
            (IRAM_BASE, &app),
            (0x4200_0000, &filler),
            (MARKER, &[0x11, 0x22, 0x33, 0x44]),
        ],
    );
    // No partition table: boot falls back to the factory slot at 0x10000.
    let mut flash = std::vec![0xFFu8; 0x20000 + 1024];
    flash[0x10000..0x10000 + img.len()].copy_from_slice(&img);

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    for _ in 0..4000 {
        if m.cpu[0].pc == here {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, here, "app entry ran");
    assert_eq!(m.soc.read32(STASH), 0x1234);
    // The third segment header must parse after the skipped XIP data: the
    // marker payload lands (not XIP filler, not a misparsed header).
    assert_eq!(m.soc.read32(MARKER), 0x4433_2211);
}

#[test]
fn ota_boot_selects_active_slot() {
    use crate::asm::Asm;
    use crate::partition::select_ota_boot_offset;
    use esp32s3_soc::memmap::IRAM_BASE;

    // OTA app (slot 1) located at 0x200000, distinct from the factory slot.
    // `li` is an ALU sequence (no literal pool), so entry = IRAM_BASE (first
    // instruction), unlike the boot_path test which pads a literal pool.
    const APP_ENTRY: u32 = IRAM_BASE;
    const STASH: u32 = 0x3FC8_0400;
    let mut a = Asm::new(IRAM_BASE);
    a.li(4, 0x1234);
    a.li(5, STASH as i32);
    a.s32i(4, 5, 0);
    let here = a.pc();
    a.j(here);
    let app = a.bytes().to_vec();

    let img = esp_app_image(IRAM_BASE, APP_ENTRY, &app);
    let mut flash = std::vec![0xFFu8; 0x300_000];
    // Partition table at 0x8000: ota_0 (count filler) + ota_1 + otadata.
    flash[0x8000] = 0xAA;
    flash[0x8001] = 0x50;
    // entry 0: ota_0 (app, subtype 0x10) @ 0x100000 (never booted here)
    flash[0x8002] = 0x00;
    flash[0x8003] = 0x10;
    flash[0x8004..0x8008].copy_from_slice(&0x100000u32.to_le_bytes());
    flash[0x8008..0x800C].copy_from_slice(&0x100000u32.to_le_bytes());
    flash[0x800C..0x801C].copy_from_slice(b"ota_0\0\0\0\0\0\0\0\0\0\0\0");
    // entry 1: ota_1 (app, subtype 0x11) @ 0x200000
    flash[0x8020] = 0xAA;
    flash[0x8021] = 0x50;
    flash[0x8022] = 0x00;
    flash[0x8023] = 0x11;
    flash[0x8024..0x8028].copy_from_slice(&0x200000u32.to_le_bytes());
    flash[0x8028..0x802C].copy_from_slice(&0x100000u32.to_le_bytes());
    flash[0x802C..0x803C].copy_from_slice(b"ota_1\0\0\0\0\0\0\0\0\0\0\0");
    // entry 2: otadata (data, subtype 0x39) @ 0xe000
    flash[0x8040] = 0xAA;
    flash[0x8041] = 0x50;
    flash[0x8042] = 0x01;
    flash[0x8043] = 0x39;
    flash[0x8044..0x8048].copy_from_slice(&0xe000u32.to_le_bytes());
    flash[0x8048..0x804C].copy_from_slice(&0x2000u32.to_le_bytes());
    flash[0x804C..0x805C].copy_from_slice(b"otadata\0\0\0\0\0\0\0\0\0");
    flash[0x8060] = 0xEB;
    flash[0x8061] = 0xEB;
    // otadata (real 32-byte entries + CRC): sector 0 seq 1, sector 1 seq 2
    // -> highest seq 2 -> slot (2-1)%2 = 1 -> ota_1 @ 0x200000.
    {
        use crate::partition::ota_seq_crc;
        for (off, seq) in [(0xe000, 1u32), (0xf000, 2u32)] {
            flash[off..off + 4].copy_from_slice(&seq.to_le_bytes());
            flash[off + 28..off + 32].copy_from_slice(&ota_seq_crc(seq).to_le_bytes());
        }
    }

    assert_eq!(select_ota_boot_offset(&flash), Some(0x200000));
    flash[0x200000_usize..0x200000_usize + img.len()].copy_from_slice(&img);

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    for _ in 0..2000 {
        if m.cpu[0].pc == here {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, here, "OTA app reached its self-loop");
    assert_eq!(m.soc.read32(STASH), 0x1234, "OTA slot-1 app executed");
}

/// ESP-IDF-style app image with multiple segments: 24-byte header (byte 1 =
/// segment count) followed by `(load_addr, data)` segments read back-to-back
/// (the ROM stub does not pad/checksum segments).
fn esp_app_image_multi(entry: u32, segs: &[(u32, &[u8])]) -> Bytes {
    let mut img = Bytes::from(&[0xE9, segs.len() as u8, 0, 0][..]); // magic, count, ...
    img.extend_from_slice(&entry.to_le_bytes()); // entry_addr
    img.extend_from_slice(&[0u8; 13]); // wp_pin + spi_pin_drv[3] + reserved[9]
    img.extend_from_slice(&[0u8; 3]); // pad the 21-byte tail to the 24-byte header
    for (addr, data) in segs {
        img.extend_from_slice(&addr.to_le_bytes()); // segment load_addr
        img.extend_from_slice(&(data.len() as u32).to_le_bytes()); // data_len
        img.extend_from_slice(data);
    }
    img
}

/// Drive one MEMSPI USR transaction the way the IDF spi_flash driver does:
/// WREN latch, then command + plain 24-bit address + optional MOSI data
/// from W0.., triggered by CMD_USR. `REG_ADDR` takes the plain address
/// (esp-idf `spimem_flash_ll_set_address`: `dev->addr = addr`); the
/// model's register decoding pairs with its addr-phase streaming so plain
/// values land correctly (pinned by `usr_program_writes_flash` and the
/// SE/BE unit tests — an MSB-first streaming "fix" was tried here and
/// broke them, proving the pairing).
fn memspi_usr(m: &mut Esp32S3, cmd: u32, addr: u32, mosi: &[u8], wren_first: bool) {
    use esp32s3_soc::memmap::SPI1_BASE;
    use esp32s3_soc::memspi::{
        CMD_FLASH_WREN, CMD_USR, REG_ADDR, REG_CMD, REG_MOSI_DLEN, REG_USER, REG_USER1, REG_USER2,
        REG_W0, USER_USR_ADDR, USER_USR_COMMAND, USER_USR_MOSI,
    };
    if wren_first {
        m.soc.write32(SPI1_BASE + REG_CMD, CMD_FLASH_WREN);
    }
    let mut user = USER_USR_COMMAND | USER_USR_ADDR;
    if !mosi.is_empty() {
        user |= USER_USR_MOSI;
        m.soc
            .write32(SPI1_BASE + REG_MOSI_DLEN, mosi.len() as u32 * 8 - 1);
        for (i, chunk) in mosi.chunks(4).enumerate() {
            let mut w = [0u8; 4];
            w[..chunk.len()].copy_from_slice(chunk);
            m.soc
                .write32(SPI1_BASE + REG_W0 + i as u32 * 4, u32::from_le_bytes(w));
        }
    }
    m.soc.write32(SPI1_BASE + REG_USER, user);
    m.soc.write32(SPI1_BASE + REG_USER2, cmd | (7 << 28)); // 8-bit command
    m.soc.write32(SPI1_BASE + REG_USER1, 23 << 26); // 24-bit address
    m.soc.write32(SPI1_BASE + REG_ADDR, addr);
    m.soc.write32(SPI1_BASE + REG_CMD, CMD_USR);
}

/// NOR flash writes through the emulated SPI controller: sector erase fills
/// 0xFF, page-program ANDs bits (clears only), exactly like the m25p80 the
/// model mirrors — the mechanism `esp_ota_write` relies on.
#[test]
fn spi_flash_page_program_and_sector_erase_via_memspi() {
    use esp32s3_soc::memmap::FLASH_DATA_BASE;
    let mut m = Esp32S3::new();
    const PAGE: u32 = 0x1_0000;
    let rd = |m: &mut Esp32S3, off: u32| m.soc.read32(FLASH_DATA_BASE + off);
    // Erase the scratch sector (erase fills a 4 KB page with 0xFF).
    memspi_usr(&mut m, 0x20, PAGE, &[], true); // SE
    assert_eq!(rd(&mut m, PAGE), 0xFFFF_FFFF, "erased word reads all-ones");
    assert_eq!(
        rd(&mut m, PAGE + 0xFFC),
        0xFFFF_FFFF,
        "erase covers the whole 4 KB sector"
    );
    // Program 4 bytes (PP + W0 data).
    memspi_usr(&mut m, 0x02, PAGE, &0xDEAD_BEEFu32.to_le_bytes(), true); // PP
    assert_eq!(rd(&mut m, PAGE), 0xDEAD_BEEF, "programmed word reads back");
    // Programming without erase clears bits but never sets them.
    memspi_usr(&mut m, 0x02, PAGE, &0xFFFF_FFFFu32.to_le_bytes(), true);
    assert_eq!(
        rd(&mut m, PAGE),
        0xDEAD_BEEF,
        "PP of all-ones leaves bits unchanged"
    );
    memspi_usr(&mut m, 0x02, PAGE, &0x0000_0000u32.to_le_bytes(), true);
    assert_eq!(rd(&mut m, PAGE), 0x0000_0000, "PP of zeros clears bits");
}

/// End-to-end OTA update at the mechanism level: boot slot 0, reprogram the
/// otadata sector through the SPI controller exactly like
/// `esp_ota_set_boot_partition` does (SE + PP), then reboot from the mutated
/// flash and land in slot 1. (Slot *selection* alone is covered by
/// `ota_boot_selects_active_slot`; this covers the *update* path.)
#[test]
fn ota_update_reprograms_otadata_and_reboots_into_new_slot() {
    use crate::asm::Asm;
    use crate::partition::select_ota_boot_offset;
    use esp32s3_soc::memmap::{FLASH_DATA_BASE, IRAM_BASE};

    const STASH: u32 = 0x3FC8_0400;
    const MARK0: u32 = 0x1111_1111;
    const MARK1: u32 = 0x2222_2222;
    const OTA0_OFF: u32 = 0x11_0000;
    const OTA1_OFF: u32 = 0x21_0000;
    const OTADATA_OFF: u32 = 0xE000;
    // Minimal app: stash a slot mark, then spin on itself.
    fn slot_app(mark: u32) -> (Bytes, u32) {
        let mut a = Asm::new(IRAM_BASE);
        a.li(4, mark as i32);
        a.li(5, STASH as i32);
        a.s32i(4, 5, 0);
        let here = a.pc();
        a.j(here);
        (a.bytes().to_vec(), here)
    }
    let (app0, here0) = slot_app(MARK0);
    let (app1, here1) = slot_app(MARK1);
    let img0 = esp_app_image(IRAM_BASE, IRAM_BASE, &app0);
    let img1 = esp_app_image(IRAM_BASE, IRAM_BASE, &app1);

    // Partition table: ota_0 @ 0x110000, ota_1 @ 0x210000, otadata @ 0xE000.
    let mut flash = std::vec![0xFFu8; 0x300_000];
    let mut entry = |idx: usize, ty: u8, sub: u8, off: u32, len: u32, label: &[u8; 16]| {
        let b = 0x8000 + idx * 0x20;
        flash[b] = 0xAA;
        flash[b + 1] = 0x50;
        flash[b + 2] = ty;
        flash[b + 3] = sub;
        flash[b + 4..b + 8].copy_from_slice(&off.to_le_bytes());
        flash[b + 8..b + 12].copy_from_slice(&len.to_le_bytes());
        flash[b + 12..b + 28].copy_from_slice(label);
    };
    entry(
        0,
        0x00,
        0x10,
        OTA0_OFF,
        0x100000,
        b"ota_0\0\0\0\0\0\0\0\0\0\0\0",
    );
    entry(
        1,
        0x00,
        0x11,
        OTA1_OFF,
        0x100000,
        b"ota_1\0\0\0\0\0\0\0\0\0\0\0",
    );
    entry(
        2,
        0x01,
        0x39,
        OTADATA_OFF,
        0x2000,
        b"otadata\0\0\0\0\0\0\0\0\0",
    );
    flash[0x8060] = 0xEB;
    flash[0x8061] = 0xEB;
    // otadata: real 32-byte entry, seq 1 (slot (1-1)%2 = 0).
    {
        use crate::partition::ota_seq_crc;
        flash[OTADATA_OFF as usize..OTADATA_OFF as usize + 4].copy_from_slice(&1u32.to_le_bytes());
        flash[OTADATA_OFF as usize + 28..OTADATA_OFF as usize + 32]
            .copy_from_slice(&ota_seq_crc(1).to_le_bytes());
    }
    flash[OTA0_OFF as usize..OTA0_OFF as usize + img0.len()].copy_from_slice(&img0);
    flash[OTA1_OFF as usize..OTA1_OFF as usize + img1.len()].copy_from_slice(&img1);
    assert_eq!(select_ota_boot_offset(&flash), Some(OTA0_OFF));

    // Boot slot 0 through the real boot path.
    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    for _ in 0..4000 {
        if m.cpu[0].pc == here0 {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, here0, "slot-0 app reached its self-loop");
    assert_eq!(m.soc.read32(STASH), MARK0, "slot 0 executed");

    // Firmware-style update: erase the otadata sector, then program a real
    // 32-byte entry (seq 2, state NEW, correct CRC) at +32 through MEMSPI.
    let mut rec = [0xFFu8; 64];
    {
        use crate::partition::ota_seq_crc;
        rec[0x20..0x24].copy_from_slice(&2u32.to_le_bytes());
        rec[0x20 + 24..0x20 + 28].copy_from_slice(&0u32.to_le_bytes());
        rec[0x20 + 28..0x20 + 32].copy_from_slice(&ota_seq_crc(2).to_le_bytes());
    }
    memspi_usr(&mut m, 0x20, OTADATA_OFF, &[], true); // SE
    memspi_usr(&mut m, 0x02, OTADATA_OFF, &rec, true); // PP
    // Read the mutated sector back through the flash window and splice it
    // into an image copy (the live backing holds the update; reset() would
    // reload the pristine image, so reboot from the updated copy instead).
    let mut img2 = flash.clone();
    for (i, dst) in img2[OTADATA_OFF as usize..OTADATA_OFF as usize + 0x1000]
        .chunks_mut(4)
        .enumerate()
    {
        dst.copy_from_slice(
            &m.soc
                .read32(FLASH_DATA_BASE + OTADATA_OFF + i as u32 * 4)
                .to_le_bytes(),
        );
    }
    assert_eq!(
        u32::from_le_bytes(
            img2[OTADATA_OFF as usize..OTADATA_OFF as usize + 4]
                .try_into()
                .unwrap()
        ),
        0xFFFF_FFFF,
        "first entry stays erased after the update"
    );
    assert_eq!(
        u32::from_le_bytes(
            img2[OTADATA_OFF as usize + 0x20..OTADATA_OFF as usize + 0x24]
                .try_into()
                .unwrap()
        ),
        2,
        "second entry programmed by the update"
    );
    assert_eq!(select_ota_boot_offset(&img2), Some(OTA1_OFF));

    // Reboot from the updated image: slot 1 runs.
    let mut m1 = Esp32S3::new();
    m1.boot_from_flash(&img2);
    for _ in 0..4000 {
        if m1.cpu[0].pc == here1 {
            break;
        }
        m1.step();
    }
    assert_eq!(m1.cpu[0].pc, here1, "slot-1 app reached its self-loop");
    assert_eq!(
        m1.soc.read32(STASH),
        MARK1,
        "slot 1 executed after OTA switch"
    );
}

/// Flash-encryption pipeline: provision an eFuse XTS key, encrypt the
/// whole image host-side (uniform encryption like esptool: every 16-byte
/// block at its absolute offset), boot the ciphertext. The app jumps from
/// its copied IRAM segment into an XIP-mapped `.flash.text` function, so
/// this exercises decrypt-on-read on the stub-loader window reads, the
/// boot-time header parse (decrypted view) AND live instruction fetch.
#[test]
fn flashenc_encrypted_app_boots_and_runs() {
    use crate::asm::Asm;
    use crate::rom_stub::APP_FLASH_OFFSET;
    use esp32s3_soc::memmap::{FLASH_INST_BASE, IRAM_BASE};

    const STASH: u32 = 0x3FC8_0400;
    const STASH2: u32 = 0x3FC8_0404;
    const MARK: u32 = 0xE11C_0001;
    const BEEF: u32 = 0xE11C_BEEF;
    // Entry (copied to IRAM by the stub loader): jump to the XIP function
    // via a literal pool. NOTE: patch the Asm buffer BEFORE cloning it
    // into `main` (a post-clone patch silently applies to nothing — this
    // exact ordering bug cost an hour: the pool read 0 and execution
    // wandered through qsort into the scratch window).
    // NOTE 2: the pool lives at the segment base, so the entry must point
    // PAST it (IRAM_BASE+4) — pointing at the pool executes data as code.
    let mut a0 = Asm::new(IRAM_BASE);
    let p_pool = a0.offset();
    a0.lit(0); // placeholder for XIP_FN
    let p_j = a0.l32r(2);
    a0.jx(2);
    let main_len = a0.bytes().len();
    // The MMU maps whole 64 KB pages, so the XIP load address must carry
    // the same intra-page offset as the segment's file position.
    let xip_file_off = APP_FLASH_OFFSET as usize + 24 + 8 + main_len + 8;
    const XIP_VPAGE: u32 = 0x1_0000;
    let xip_fn = FLASH_INST_BASE + XIP_VPAGE + (xip_file_off & 0xFFFF) as u32;
    a0.bytes_mut()[p_pool..p_pool + 4].copy_from_slice(&xip_fn.to_le_bytes());
    a0.patch_l32r(p_j, IRAM_BASE + p_pool as u32);
    let main = a0.bytes().to_vec();
    // XIP function: stash BEEF, then MARK, then self-loop (all fetched
    // through the decrypting instruction window; position-independent).
    let mut x = Asm::new(xip_fn);
    x.li(6, BEEF as i32);
    x.li(7, STASH2 as i32);
    x.s32i(6, 7, 0);
    x.li(6, MARK as i32);
    x.li(7, STASH as i32);
    x.s32i(6, 7, 0);
    let here = x.pc();
    x.j(here);
    let xip = x.bytes().to_vec();

    let img = esp_app_image_multi(IRAM_BASE + 4, &[(IRAM_BASE, &main), (xip_fn, &xip)]);
    const FLASH_LEN: usize = 0x2_0000;
    let mut flash = std::vec![0xFFu8; FLASH_LEN];
    flash[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + img.len()].copy_from_slice(&img);

    // Provision the eFuse fixture key, encrypt the image in the backing,
    // snapshot the ciphertext, then boot from it.
    let key = [0x5Au8; 32];
    let mut m = Esp32S3::new();
    assert!(!m.soc.flash_enc_enabled(), "fresh device is plaintext");
    m.soc.flashenc_provision(&key);
    assert!(m.soc.flash_enc_enabled(), "provisioned device is encrypted");
    m.soc.load_flash_image(0, &flash);
    m.soc.flashenc_encrypt_region(0, FLASH_LEN as u32);
    let enc = m.soc.flash_image().to_vec();
    assert_ne!(
        &enc[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + 16],
        &flash[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + 16],
        "backing holds ciphertext, not plaintext"
    );
    m.boot_from_flash(&enc);
    assert_eq!(m.cpu[0].pc, 0x4000_0400, "boot leaves reset vector");
    assert!(m.soc.rom_boot_mode(), "boot leaves rom_boot_mode set");
    // Scratch window (loader path) must serve decrypted image bytes.
    use crate::rom_stub::LOADER_SCRATCH;
    for (k, &want) in img.iter().enumerate().take(32) {
        assert_eq!(
            m.soc.read8(LOADER_SCRATCH + k as u32),
            want as u32,
            "scratch byte {k} decrypts"
        );
    }
    // XIP window must serve decrypted segment bytes.
    for (k, &want) in xip.iter().enumerate() {
        assert_eq!(
            m.soc.read8(xip_fn + k as u32),
            want as u32,
            "XIP byte {k} decrypts"
        );
    }
    for _ in 0..6000 {
        if m.cpu[0].pc == here {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, here, "XIP function reached its self-loop");
    assert_eq!(m.soc.read32(STASH), MARK, "entry marker via XIP");
    assert_eq!(m.soc.read32(STASH2), BEEF, "XIP function executed");
}

#[test]
fn dual_core_release_and_run() {
    use crate::asm::Asm;
    use crate::rom_stub::APP_FLASH_OFFSET;
    use esp32s3_soc::memmap::IRAM_BASE;

    // Core 0's app (the ROM loader jumps here): release core 1 by writing its
    // entry point to the APP-CPU release register (SYSTEM.APPCPU_CTRL_A,
    // where our CORE1_WAIT / the real ROM fastboot polls), stash 0xBEEF,
    // self-loop.
    const CORE1_CODE: u32 = IRAM_BASE + 0x100;
    const STASH0: u32 = 0x3FC8_0200;
    const STASH1: u32 = 0x3FC8_0204;
    let mut a0 = Asm::new(IRAM_BASE);
    a0.li(6, CORE1_CODE as i32);
    a0.li(7, esp32s3_soc::memmap::SYSTEM_BASE as i32 + 4);
    a0.s32i(6, 7, 0); // release core 1
    a0.li(6, 0xBEEF);
    a0.li(7, STASH0 as i32);
    a0.s32i(6, 7, 0);
    let here0 = a0.pc();
    a0.j(here0);

    // Core 1's firmware (runs only after core 0 releases it): stash 0x1234,
    // self-loop.
    let mut a1 = Asm::new(CORE1_CODE);
    a1.li(6, 0x1234);
    a1.li(7, STASH1 as i32);
    a1.s32i(6, 7, 0);
    let here1 = a1.pc();
    a1.j(here1);

    let img = esp_app_image_multi(
        IRAM_BASE,
        &[(IRAM_BASE, a0.bytes()), (CORE1_CODE, a1.bytes())],
    );
    let mut flash = std::vec![0xFFu8; 0x200_000];
    flash[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + img.len()].copy_from_slice(&img);

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    for _ in 0..2000 {
        if m.cpu[0].pc == here0 && m.cpu[1].pc == here1 {
            break;
        }
        m.step();
    }
    // If core 1's PRID read returned 0, it would have run the loader and then
    // core 0's code too, landing at here0 instead of here1 (its release gate
    // would be dead code).
    assert_eq!(m.cpu[0].pc, here0, "core 0 self-loop");
    assert_eq!(m.cpu[1].pc, here1, "core 1 self-loop after release");
    assert_eq!(m.soc.read32(STASH0), 0xBEEF, "core 0 stash");
    assert_eq!(m.soc.read32(STASH1), 0x1234, "core 1 stash");
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
    a.s32i(4, 2, 0x70); // INT_ENA bit 0
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
    h.s32i(9, 8, 0x7C); // INT_CLR
    h.rfi(3);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_01C0, h.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..2000 {
        if m.cpu[0].pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, done, "app finished its loop");
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
    a.s32i(4, 2, 0x70); // INT_ENA bit 0
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
    h.s32i(9, 8, 0x7C); // INT_CLR
    h.rfi(4);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_0200, h.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..2000 {
        if m.cpu[0].pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after 3 interrupts");
    assert_eq!(m.soc.read32(CTR), 3, "handler ran 3 times");
    assert_eq!(m.soc.read32(INT_MATRIX_BASE + 4 * 53), 24, "matrix write");
}

#[test]
fn sha_done_interrupt_delivers_to_vector() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::INT_MATRIX_BASE;
    use esp32s3_soc::sha::SHA_BASE;

    // App: literal pool (CTR/STASH/INT_MATRIX/SHA), then code. Routes
    // source 78 (SHA done) to CPU line 15 (level 3), feeds one 64-byte
    // direct-fill block, enables the done interrupt, then spins on CTR
    // until the level-3 handler runs once and stashes 0xCAFE.
    const CTR: u32 = 0x3FC8_0300;
    const STASH: u32 = 0x3FC8_0304;
    let mut a = Asm::new(IRAM_BASE);
    let l_ctr = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let l_mat = a.offset();
    a.lit(0);
    let l_sha = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // INT_MATRIX_BASE
    a.patch_l32r(p, IRAM_BASE + l_mat as u32);
    a.movi_n(3, 15);
    a.s32i(3, 2, 4 * 78); // matrix source 78 -> line 15
    let p = a.l32r(2); // SHA_BASE
    a.patch_l32r(p, IRAM_BASE + l_sha as u32);
    a.movi_n(3, 2);
    a.s32i(3, 2, 0x00); // MODE = SHA-256
    a.movi_n(3, 1);
    a.s32i(3, 2, 0x28); // INT_ENA (done interrupt)
    a.li(4, SHA_BASE as i32 + 0x80); // TEXT fill pointer
    a.movi_n(3, 0);
    a.movi_n(5, 16);
    let fill = a.pc();
    a.s32i(3, 4, 0); // 16 words of counter pattern (one block)
    a.addi(4, 4, 4);
    a.addi(3, 3, 1);
    a.bne(3, 5, fill);
    a.movi_n(3, 1);
    a.s32i(3, 2, 0x10); // SHA_START: transform + latch done
    a.li(3, 0x8000); // INTENABLE bit 15
    a.wsr(228, 3);
    a.rsil(4, 0);
    let loop_start = a.pc();
    let p = a.l32r(2); // CTR
    a.patch_l32r(p, IRAM_BASE + l_ctr as u32);
    a.l32i(3, 2, 0);
    a.beqz(3, loop_start); // spin while CTR == 0 (handler sets 1)
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    // Patch the literal pool (values must sit at their 4-aligned slots).
    a.bytes_mut()[l_ctr..l_ctr + 4].copy_from_slice(&CTR.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());
    a.bytes_mut()[l_mat..l_mat + 4].copy_from_slice(&INT_MATRIX_BASE.to_le_bytes());
    a.bytes_mut()[l_sha..l_sha + 4].copy_from_slice(&SHA_BASE.to_le_bytes());

    // Level-3 handler at VECBASE + 0x1C0 (uses only a6-a9).
    let mut h = Asm::new(0x4000_01C0);
    h.li(6, CTR as i32);
    h.l32i(7, 6, 0);
    h.addi(7, 7, 1);
    h.s32i(7, 6, 0); // CTR += 1
    h.li(8, SHA_BASE as i32);
    h.movi_n(9, 1);
    h.s32i(9, 8, 0x24); // INT_CLR (CLEAR_IRQ)
    h.rfi(3);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 2);
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_01C0, h.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..4000 {
        if m.cpu[0].pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after SHA interrupt");
    assert_eq!(m.soc.read32(CTR), 1, "handler ran once");
    assert_eq!(m.soc.read32(INT_MATRIX_BASE + 4 * 78), 15, "matrix write");
}

#[test]
fn adc_done_interrupt_delivers_to_vector() {
    use crate::asm::Asm;
    use esp32s3_soc::adc::{APB_CTRL, APB_INT_CLR, APB_INT_ENA, APB_SAR1_PATT_TAB};
    use esp32s3_soc::memmap::{APB_SARADC_BASE, INT_MATRIX_BASE};

    // App: literal pool (CTR/STASH/INT_MATRIX/ADC), then code. Routes
    // source 65 (APB ADC done) to CPU line 15 (level 3), runs one digital
    // single-shot conversion (pattern ch3, start pulse), enables the
    // adc1_done interrupt, then spins on CTR until the level-3 handler
    // runs once and stashes 0xCAFE. The conversion completes synchronously
    // on the START write (value unimportant — delivery is what's tested).
    const CTR: u32 = 0x3FC8_0400;
    const STASH: u32 = 0x3FC8_0404;
    let mut a = Asm::new(IRAM_BASE);
    let l_ctr = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let l_mat = a.offset();
    a.lit(0);
    let l_adc = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // INT_MATRIX_BASE
    a.patch_l32r(p, IRAM_BASE + l_mat as u32);
    a.movi_n(3, 15);
    a.s32i(3, 2, 4 * 65); // matrix source 65 -> line 15
    let p = a.l32r(2); // APB_SARADC_BASE
    a.patch_l32r(p, IRAM_BASE + l_adc as u32);
    a.movi_n(3, 0x0C);
    a.s32i(3, 2, APB_SAR1_PATT_TAB); // pattern: ch3 atten 0
    a.li(3, 0x8000_0000u32 as i32);
    a.s32i(3, 2, APB_INT_ENA); // enable adc1_done interrupt
    a.li(3, 0x43u32 as i32); // clk_gated|single|start_force|start
    a.s32i(3, 2, APB_CTRL); // start pulse: converts + latches done
    a.li(3, 0x8000); // INTENABLE bit 15
    a.wsr(228, 3);
    a.rsil(4, 0);
    let loop_start = a.pc();
    let p = a.l32r(2); // CTR
    a.patch_l32r(p, IRAM_BASE + l_ctr as u32);
    a.l32i(3, 2, 0);
    a.beqz(3, loop_start); // spin while CTR == 0
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    // Patch the literal pool (values must sit at their 4-aligned slots).
    a.bytes_mut()[l_ctr..l_ctr + 4].copy_from_slice(&CTR.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());
    a.bytes_mut()[l_mat..l_mat + 4].copy_from_slice(&INT_MATRIX_BASE.to_le_bytes());
    a.bytes_mut()[l_adc..l_adc + 4].copy_from_slice(&APB_SARADC_BASE.to_le_bytes());

    // Level-3 handler at VECBASE + 0x1C0 (uses only a6-a9).
    let mut h = Asm::new(0x4000_01C0);
    h.li(6, CTR as i32);
    h.l32i(7, 6, 0);
    h.addi(7, 7, 1);
    h.s32i(7, 6, 0); // CTR += 1
    h.li(8, APB_SARADC_BASE as i32);
    h.li(9, 0x8000_0000u32 as i32);
    h.s32i(9, 8, APB_INT_CLR); // clear adc1_done
    h.rfi(3);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_01C0, h.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..4000 {
        if m.cpu[0].pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after ADC interrupt");
    assert_eq!(m.soc.read32(CTR), 1, "handler ran once");
    assert_eq!(m.soc.read32(INT_MATRIX_BASE + 4 * 65), 15, "matrix write");
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
    a.movi_n(3, 1);
    a.s32i(3, 2, 0x24); // CONF1: rxfifo_full_thrhd = 1 (a 1-byte burst fires)
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
    m.cpu[0].pc = code_start;
    m.soc.uart_inject_rx(0, b'X');
    for _ in 0..2000 {
        if m.cpu[0].pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, done, "app finished its loop");
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

    // Firmware: TIMER0 at 0xA0 (S3 ledc_struct.h: timer_group after the 8
    // channels) with clock_divider 256 (CONF [21:4]) and duty_resolution field
    // 10 ([3:0]) — 1024-tick period, one tick per APB step — channel 0 at
    // 0x00 with duty 0x2000 (esp-idf stores user_duty<<4, so 512<<4 = 50%),
    // sig_out_en (conf0 bit 2) + duty_start (conf1 bit 31), then routes
    // LEDC_CH0 (GPIO-matrix signal 73) to GPIO0 and enables the pad.  The host
    // measures the pad: 512 steps high / 512 steps low.
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
    a.li(3, 0x100A);
    a.s32i(3, 2, 0xA0); // TIMER0_CONF: clock_divider 256, duty_resolution 10
    a.li(3, 0x2000);
    a.s32i(3, 2, 0x08); // CH0_DUTY: user 512 << 4 = 50% of 1024
    a.movi_n(3, 4);
    a.s32i(3, 2, 0x00); // CH0_CONF0: sig_out_en (bit 2)
    a.li(3, 0x8000_0000u32 as i32);
    a.s32i(3, 2, 0x0C); // CH0_CONF1: duty_start (bit 31)
    let p = a.l32r(2); // GPIO_BASE
    a.patch_l32r(p, IRAM_BASE + l_gpio as u32);
    a.movi_n(3, 1);
    a.s32i(3, 2, 0x20); // ENABLE bit 0
    a.li(3, 73);
    a.li(6, (GPIO_BASE + 0x554) as i32); // FUNC_OUT_SEL base (S3 gpio_struct.h)
    a.s32i(3, 6, 0); // GPIO0 FUNC_OUT_SEL = LEDC_CH0 (signal 73)
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
    syscon_clk(&mut m, false, 11);
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu[0].pc = code_start;
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
    a.li(6, (GPIO_BASE + 0x554) as i32); // FUNC_OUT_SEL base (S3 gpio_struct.h)
    a.li(4, 101);
    a.s32i(4, 6, 4); // GPIO1 FUNC_OUT_SEL = FSPICLK
    a.li(4, 103);
    a.s32i(4, 6, 8); // GPIO2 FUNC_OUT_SEL = FSPID
    a.li(4, 110);
    a.s32i(4, 6, 0xC); // GPIO3 FUNC_OUT_SEL = FSPICS0
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
    m.cpu[0].pc = code_start;
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
    a.li(6, (GPIO_BASE + 0x554) as i32); // FUNC_OUT_SEL base (S3 gpio_struct.h)
    a.li(4, 89);
    a.s32i(4, 6, 4); // GPIO1 FUNC_OUT_SEL = I2CEXT0_SCL
    a.li(4, 90);
    a.s32i(4, 6, 8); // GPIO2 FUNC_OUT_SEL = I2CEXT0_SDA
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
    syscon_clk(&mut m, false, 7);
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu[0].pc = code_start;
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
    m.cpu[0].pc = code_start;
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

#[test]
fn psram_read_write_via_mmu_mapped_page() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::{CACHE_PAGE_SIZE, FLASH_DATA_BASE, MMU_TABLE_BASE};

    // Firmware programs the shared cache MMU (vpage 0 -> PSRAM physical
    // page 0, vpage 3 -> PSRAM physical page 2; entry = type<<15 | page),
    // then reads/writes the data window and stashes the round-trip words.
    // The physical page field selecting the backing is what a real
    // esp_rom_mmu_map does before ESP-IDF touches PSRAM.
    const STASH: u32 = 0x3FC8_0200;
    let mut a = Asm::new(IRAM_BASE);
    let l_tab = a.offset();
    a.lit(0);
    let l_win = a.offset();
    a.lit(0);
    let l_win3 = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // MMU_TABLE_BASE
    a.patch_l32r(p, IRAM_BASE + l_tab as u32);
    let p = a.l32r(3); // FLASH_DATA_BASE
    a.patch_l32r(p, IRAM_BASE + l_win as u32);
    let p = a.l32r(4); // FLASH_DATA_BASE + 3 * CACHE_PAGE_SIZE
    a.patch_l32r(p, IRAM_BASE + l_win3 as u32);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.li(6, 0x8000); // PSRAM page 0 entry
    a.s32i(6, 2, 0); // mmu[0]
    a.li(6, 0x8002); // PSRAM page 2 entry
    a.s32i(6, 2, 3 * 4); // mmu[3] (byte offset 12)
    a.li(6, 0xDEAD_BEEFu32 as i32);
    a.s32i(6, 3, 0); // PSRAM page 0 [0]
    a.l32i(6, 3, 0);
    a.s32i(6, 5, 0); // stash word 0
    a.li(6, 0xCAFE_BABEu32 as i32);
    a.s32i(6, 4, 0); // PSRAM page 2 [0]
    a.l32i(6, 4, 0);
    a.s32i(6, 5, 4); // stash word 1
    let halt = a.pc();
    a.j(halt);
    a.bytes_mut()[l_tab..l_tab + 4].copy_from_slice(&MMU_TABLE_BASE.to_le_bytes());
    a.bytes_mut()[l_win..l_win + 4].copy_from_slice(&FLASH_DATA_BASE.to_le_bytes());
    a.bytes_mut()[l_win3..l_win3 + 4]
        .copy_from_slice(&(FLASH_DATA_BASE + 3 * CACHE_PAGE_SIZE).to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..100 {
        if m.soc.read32(STASH + 4) != 0 {
            break;
        }
        m.step();
    }
    assert_eq!(m.soc.read32(STASH), 0xDEAD_BEEF, "PSRAM page 0 round-trip");
    assert_eq!(
        m.soc.read32(STASH + 4),
        0xCAFE_BABE,
        "PSRAM page 2 round-trip"
    );
}

#[test]
fn udivdi3_umoddi3_helpers_return_correct_results() {
    // The ROM __udivdi3 (0x40002544) / __umoddi3 (0x40002574) slots implement
    // the libgcc di ABI (caller view): dividend a10:a11, divisor a12:a13;
    // __udivdi3 returns the quotient in a10:a11 and the remainder in
    // a12:a13, __umoddi3 the remainder in a10:a11.  Regression cases for the
    // 2026-08-17 div-body fixes (bit63(dividend) -> r_lo, divisor word
    // order, 64-bit quotient carry, umod early-exit removal).
    use crate::asm::Asm;
    use crate::rom_stub;
    const STASH: u32 = 0x3FC8_0200;
    const CODE: u32 = IRAM_BASE + 0x8000;
    const SLOT_UDIV: u32 = 0x4000_2544;
    const SLOT_UMOD: u32 = 0x4000_2574;

    fn program(slot: u32, dividend: u64, divisor: u64) -> (Bytes, u32) {
        let mut a = Asm::new(CODE);
        a.li(1, 0x3FC8_8000); // SP: the helper spills a8-a11 below SP
        a.li(3, 0x40000); // PS.WOE (bit 18): ENTRY/RETW are illegal
        a.wsr(xtensa_core::cpu::SR_PS, 3); // without it; the ROM reset
        // leaves PS = 0 (the ROM itself never uses windowed calls)
        a.li(10, dividend as i32);
        a.li(11, (dividend >> 32) as i32);
        a.li(12, divisor as i32);
        a.li(13, (divisor >> 32) as i32);
        a.li(8, slot as i32);
        a.callx8(8);
        a.li(2, STASH as i32);
        a.s32i(10, 2, 0); // result lo  (q or r)
        a.s32i(11, 2, 4); // result hi
        a.s32i(12, 2, 8); // remainder lo (__udivdi3 only)
        a.s32i(13, 2, 12); // remainder hi
        let halt = a.pc();
        a.j(halt);
        (a.bytes().to_vec(), halt)
    }

    fn run(slot: u32, dividend: u64, divisor: u64) -> (u64, u64) {
        let mut m = Esp32S3::new();
        let rom = rom_stub::rom_image();
        m.load_image(rom_stub::ROM_BASE, &rom);
        let (code, halt) = program(slot, dividend, divisor);
        m.load_image(CODE, &code);
        m.cpu[0].pc = CODE;
        for _ in 0..10000 {
            if m.cpu[0].pc == halt {
                break;
            }
            m.step();
        }
        assert_eq!(m.cpu[0].pc, halt, "div helper must halt cleanly");
        let lo = m.soc.read32(STASH) as u64;
        let hi = m.soc.read32(STASH + 4) as u64;
        (
            lo | (hi << 32),
            (m.soc.read32(STASH + 8) as u64) | ((m.soc.read32(STASH + 12) as u64) << 32),
        )
    }

    // (dividend, divisor) — expectations computed with native u64 math
    // (independent of the emulated long division).
    let udiv_cases: &[(u64, u64)] = &[
        (100_000_000, 4000),                  // rtc_clk_cal_internal replica
        (0x0003_4260_07CF, 4000),             // rtc_clk_cal replica (boot blocker)
        (0x0123_4567_89AB_CDEF, 1),           // 64-bit quotient (qshift carry)
        (0xFFFF_FFFF_FFFF_FFFF, 0xFFFF_FFFF), // q = 0x100000001
        (5, 7),                               // dividend < divisor
        (0x0123_4567_89AB_CDEF, 0x1234_5678), // 64-bit quotient + remainder
    ];
    for &(d, v) in udiv_cases {
        // The real ROM __udivdi3 (libgcc di ABI) returns the quotient in
        // a2:a3 only — the remainder is NOT delivered in a4:a5 (the stub's
        // body used to return it there); __umoddi3 below covers remainders.
        let (q, _r) = run(SLOT_UDIV, d, v);
        assert_eq!(q, d / v, "udivdi3 q for {d:#x} / {v:#x}");
    }
    let umod_cases: &[(u64, u64)] = &[
        (0x100, 3), // old body's early exit returned 0
        (0x0003_4260_07CF, 4000),
        (0xFFFF_FFFF_FFFF_FFFF, 0xFFFF_FFFF),
        (13, 5),
    ];
    for &(d, v) in umod_cases {
        let (r, _) = run(SLOT_UMOD, d, v);
        assert_eq!(r, d % v, "umoddi3 r for {d:#x} / {v:#x}");
    }
}

#[test]
fn shift_di3_helpers_return_correct_results() {
    // __ashldi3 (0x400021B4) / __ashrdi3 (0x400021C0) / __lshrdi3
    // (0x400023D0) slots: caller view a10:a11 = value, a12 = count; result
    // in a10:a11.  Regression cases for the 2026-08-17 body fixes (double
    // `entry` removed — the slot does the windowed entry; bgeu branch
    // offset corrected for the count >= 32 path).
    use crate::asm::Asm;
    use crate::rom_stub;
    const STASH: u32 = 0x3FC8_0200;
    const CODE: u32 = IRAM_BASE + 0x8000;
    const SLOT_ASHL: u32 = 0x4000_21B4;
    const SLOT_ASHR: u32 = 0x4000_21C0;
    const SLOT_LSHR: u32 = 0x4000_23D0;

    fn program(slot: u32, value: u64, count: u32) -> (Bytes, u32) {
        let mut a = Asm::new(CODE);
        a.li(1, 0x3FC8_8000); // SP
        a.li(3, 0x40000); // PS.WOE (bit 18)
        a.wsr(xtensa_core::cpu::SR_PS, 3);
        a.li(10, value as i32);
        a.li(11, (value >> 32) as i32);
        a.li(12, count as i32);
        a.li(8, slot as i32);
        a.callx8(8);
        a.li(2, STASH as i32);
        a.s32i(10, 2, 0); // result lo
        a.s32i(11, 2, 4); // result hi
        let halt = a.pc();
        a.j(halt);
        (a.bytes().to_vec(), halt)
    }

    fn run(slot: u32, value: u64, count: u32) -> u64 {
        let mut m = Esp32S3::new();
        let rom = rom_stub::rom_image();
        m.load_image(rom_stub::ROM_BASE, &rom);
        let (code, halt) = program(slot, value, count);
        m.load_image(CODE, &code);
        m.cpu[0].pc = CODE;
        for _ in 0..10000 {
            if m.cpu[0].pc == halt {
                break;
            }
            m.step();
        }
        assert_eq!(m.cpu[0].pc, halt, "shift helper must halt cleanly");
        let lo = m.soc.read32(STASH) as u64;
        let hi = m.soc.read32(STASH + 4) as u64;
        lo | (hi << 32)
    }

    // (value, count) — expectations computed with native u64 math
    // (independent of the emulated shifts).  Counts span 0 / <32 / >=32
    // to cover the beqz, main, and bgeu-tail paths.
    let ashl_cases: &[(u64, u32)] = &[
        (0x0123_4567_89AB_CDEF, 0),  // beqz path
        (0x0123_4567_89AB_CDEF, 4),  // cross-word path
        (0x0123_4567_89AB_CDEF, 40), // count >= 32 (bgeu tail)
        (1, 63),
        (0xFFFF_FFFF_FFFF_FFFF, 33),
    ];
    for &(v, c) in ashl_cases {
        assert_eq!(run(SLOT_ASHL, v, c), v << c, "ashldi3 for {v:#x} << {c}");
    }
    let ashr_cases: &[(u64, u32)] = &[
        (0x8000_0000_0000_0000, 8), // sign-fill, < 32
        (0x8000_0000_0000_0000, 40),
        (0xFFFF_FFFF_FFFF_FFFF, 1),
        (0x0123_4567_89AB_CDEF, 12),
        (0x0123_4567_89AB_CDEF, 33),
    ];
    for &(v, c) in ashr_cases {
        assert_eq!(
            run(SLOT_ASHR, v, c),
            ((v as i64) >> c) as u64,
            "ashrdi3 for {v:#x} >> {c}"
        );
    }
    let lshr_cases: &[(u64, u32)] = &[
        (0x8000_0000_0000_0000, 8),  // logical, not sign-filled
        (0x8000_0000_0000_0000, 40), // count >= 32 (bgeu tail)
        (0xFFFF_FFFF_FFFF_FFFF, 63),
        (0, 5),
        (0x0123_4567_89AB_CDEF, 32),
    ];
    for &(v, c) in lshr_cases {
        assert_eq!(run(SLOT_LSHR, v, c), v >> c, "lshrdi3 for {v:#x} >> {c}");
    }
}

#[test]
fn ets_printf_mailbox_formats_and_prints() {
    // The app's ets_printf call resolves to the 0x400005D0 __call_ets_printf
    // wrapper (the real ROM's table — the stub's own 0x5D0 slot is past
    // GLUE_END 0x570 and is NOT spliced), which l32r+jx's to the real ROM's
    // ets_printf at 0x4004423C: vsnprintf + putc1 + uart_tx_one_char.  The
    // real code formats the test's %s/%d/%02x and emits via the
    // USB-Serial-JTAG FIFO (0x60038000), merged into UART0's stream by
    // take_uart_tx(0).  load_rom_data pre-installs putc1 = uart_tx_one_char
    // (the state the real bootloader leaves behind), so bare-metal callers
    // print without calling ets_install_uart_printf first.
    use crate::asm::Asm;
    use crate::rom_stub;
    const CODE: u32 = IRAM_BASE + 0x9000;
    const FMT: u32 = CODE + 0x200;
    const S1: u32 = CODE + 0x400;
    let mut a = Asm::new(CODE);
    a.li(1, 0x3FC8_8000); // SP
    a.li(3, 0x40000); // PS.WOE (bit 18)
    a.wsr(xtensa_core::cpu::SR_PS, 3);
    a.li(10, FMT as i32); // fmt (caller a10 = callee a2)
    a.li(11, S1 as i32); // %s
    a.li(12, 42); // %d
    a.li(13, 0x2A); // %02x
    a.li(8, 0x4000_05D0); // ets_printf
    a.callx8(8);
    let halt = a.pc();
    a.j(halt);
    let mut m = Esp32S3::new();
    let rom = rom_stub::rom_image();
    m.load_image(rom_stub::ROM_BASE, &rom);
    m.load_rom_data();
    m.load_image(CODE, a.bytes());
    m.load_image(FMT, b"%s core %d val=0x%02x!\0");
    m.load_image(S1, b"s1\0");
    m.cpu[0].pc = CODE;
    for _ in 0..10000 {
        if m.cpu[0].pc == halt {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, halt, "printf caller must halt cleanly");
    assert_eq!(m.take_uart_tx(0), b"s1 core 42 val=0x2a!");
    assert_eq!(m.soc.read32(rom_stub::HOST_PRINTF), 0, "mailbox cleared");
}

#[test]
fn rom_qsort_sorts_via_windowed_comparator() {
    // The ROM qsort symbol (0x40001488) must sort a table in place through
    // the windowed call convention: the app calls via callx8 (args in
    // a10..a13 -> callee a2..a5), the stub body calls the comparator via
    // callx4 (callinc 1: args a6/a7 -> callee a2/a3, return in the
    // callee's a2 = the caller's a6, caller a0 untouched).  heap_caps_init
    // relies on this to sort its reserved memory regions.
    use crate::asm::Asm;
    use crate::rom_stub;
    const CODE: u32 = IRAM_BASE + 0x9000;
    const ARR: u32 = CODE + 0x200;
    let mut a = Asm::new(CODE);
    a.li(1, 0x3FCB_0000); // SP (clear of code/SRC/DST)
    a.li(3, 0x40000); // PS.WOE
    a.wsr(xtensa_core::cpu::SR_PS, 3);
    a.li(10, ARR as i32); // callee a2 = base (callx8: caller a10..a13)
    a.movi(11, 6); // callee a3 = nmemb
    a.movi(12, 4); // callee a4 = size
    a.li(13, (CODE + 0x80) as i32); // callee a5 = compar
    a.li(8, 0x4000_1488); // qsort
    a.callx8(8);
    let halt = a.pc();
    a.j(halt);
    // comparator(int *a, int *b) -> *a - *b in a2 (GCC windowed ABI:
    // return value in the callee's a2, which the callx4 caller reads
    // from its own a6 — mirrors s_compare_reserved_regions' compiled
    // `sub a2,a2,a8; retw.n`).
    rom_stub::pad_to(&mut a, CODE + 0x80);
    a.entry(1, 0);
    a.l32i(2, 2, 0);
    a.l32i(9, 3, 0);
    a.sub(2, 2, 9);
    a.retw();
    let mut m = Esp32S3::new();
    let rom = rom_stub::rom_image();
    m.load_image(rom_stub::ROM_BASE, &rom);
    m.load_image(CODE, a.bytes());
    let arr: [i32; 6] = [9, 7, 5, 6, 8, 4];
    m.load_image(ARR, &arr.map(|v| v.to_le_bytes()).concat());
    m.cpu[0].pc = CODE;
    for _ in 0..20000 {
        if m.cpu[0].pc == halt {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, halt, "qsort caller must halt cleanly");
    let vals: [i32; 6] = std::array::from_fn(|i| m.soc.read32(ARR + 4 * i as u32) as i32);
    assert_eq!(vals, [4, 5, 6, 7, 8, 9]);
}

#[test]
fn rom_qsort_sorts_size8_entries() {
    // heap_caps_init sorts soc_reserved_region_t ({u32 start, u32 end} =
    // 8-byte entries) with qsort(nmemb, 8, compar).  The ROM qsort body
    // must honor the size argument from the stack (it previously
    // hard-coded 4 and corrupted the array with overlapping byte swaps,
    // which made the app abort in soc_get_available_memory_regions).
    use crate::asm::Asm;
    use crate::rom_stub;
    const CODE: u32 = IRAM_BASE + 0x9000;
    const ARR: u32 = CODE + 0x200;
    let mut a = Asm::new(CODE);
    a.li(1, 0x3FC8_9000); // SP (clear of the ROM layout struct)
    a.li(3, 0x40000); // PS.WOE
    a.wsr(xtensa_core::cpu::SR_PS, 3);
    a.li(10, ARR as i32); // callee a2 = base (callx8: caller a10..a13)
    a.movi(11, 6); // callee a3 = nmemb
    a.movi(12, 8); // callee a4 = size (8-byte {start,end} entries)
    a.li(13, (CODE + 0x80) as i32); // callee a5 = compar
    a.li(8, 0x4000_1488); // qsort
    a.callx8(8);
    let halt = a.pc();
    a.j(halt);
    // comparator: return a->start - b->start (reads word [0] of each entry)
    rom_stub::pad_to(&mut a, CODE + 0x80);
    a.entry(1, 0);
    a.l32i(2, 2, 0);
    a.l32i(9, 3, 0);
    a.sub(2, 2, 9);
    a.retw();
    let mut m = Esp32S3::new();
    let rom = rom_stub::rom_image();
    m.load_image(rom_stub::ROM_BASE, &rom);
    m.load_image(CODE, a.bytes());
    let arr: [i32; 12] = [9, 90, 7, 70, 5, 50, 6, 60, 8, 80, 4, 40];
    m.load_image(ARR, &arr.map(|v| v.to_le_bytes()).concat());
    m.cpu[0].pc = CODE;
    for _ in 0..20000 {
        if m.cpu[0].pc == halt {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, halt, "qsort caller must halt cleanly");
    let vals: [i32; 12] = std::array::from_fn(|i| m.soc.read32(ARR + 4 * i as u32) as i32);
    assert_eq!(vals, [4, 40, 5, 50, 6, 60, 7, 70, 8, 80, 9, 90]);
}

#[test]
fn rmt_signal_drives_gpio_in_loopback() {
    use esp32s3_soc::memmap::GPIO_BASE;
    use esp32s3_soc::rmt::{RMT_BASE, RMTMEM_BASE};

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 9);
    // Route GPIO2 to RMT TX signal 81 (channel 0) and enable output.
    m.soc.write32(GPIO_BASE + 0x554 + 2 * 4, 81); // FUNC_OUT_SEL_CFG[2]
    m.soc.write32(GPIO_BASE + 0x20, 1 << 2); // GPIO_ENABLE_W1TS bit2

    // Two RMT items: item0 = HIGH 100 / LOW 100, item1 = HIGH 50 / LOW 50.
    // item format: duration0[14:0], level0[15], duration1[30:16], level1[31].
    let item0: u32 = 100 | (1u32 << 15) | (100 << 16);
    let item1: u32 = 50 | (1u32 << 15) | (50 << 16);
    m.soc.write32(RMTMEM_BASE, item0);
    m.soc.write32(RMTMEM_BASE + 4, item1);

    // chnconf0[0] = tx_start | idle_out_en (mem_size default -> 64 items).
    m.soc.write32(RMT_BASE + 0x20, (1 << 0) | (1 << 6));

    let mut saw0 = false;
    let mut saw1 = false;
    for _ in 0..20000 {
        m.step();
        let g = m.soc.gpio_in_readback();
        if (g >> 2) & 1 == 1 {
            saw1 = true;
        } else {
            saw0 = true;
        }
        if saw0 && saw1 {
            break;
        }
    }
    assert!(
        saw0 && saw1,
        "RMT output must toggle GPIO2 via the GPIO_IN loopback (saw0={saw0} saw1={saw1})"
    );
}

#[test]
fn rmt_carrier_tx_demod_rx_loopback_envelope() {
    use esp32s3_soc::memmap::GPIO_BASE;
    use esp32s3_soc::rmt::{RMT_BASE, RMTMEM_BASE};

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 9);
    // Route GPIO2 to RMT TX signal 81 (channel 0) and enable output.
    m.soc.write32(GPIO_BASE + 0x554 + 2 * 4, 81); // FUNC_OUT_SEL_CFG[2]
    m.soc.write32(GPIO_BASE + 0x20, 1 << 2); // GPIO_ENABLE_W1TS bit2
    // Route GPIO2 into RMT RX signal 81 (HW channel 4).
    m.soc.write32(GPIO_BASE + 0x154 + 81 * 4, 2); // FUNC_IN_SEL_CFG[81]

    // One item: HIGH 400 / LOW 400 channel ticks.
    let item0: u32 = 400 | (1u32 << 15) | (400 << 16);
    m.soc.write32(RMTMEM_BASE, item0);
    // RX: idle timeout 2048, demod on (bit 28) bursting high (bit 29).
    m.soc
        .write32(RMT_BASE + 0x30, (2048 << 8) | (1 << 28) | (1 << 29));
    // RX block limit 1 item, then arm.
    m.soc.write32(RMT_BASE + 0xB0, 1); // CHM_RX_LIM
    m.soc.write32(RMT_BASE + 0x34, 1); // rx_en
    // TX: start + idle-low + carrier EN/EFF/LV-high + 10/10 duty.
    m.soc.write32(RMT_BASE + 0x80, (10 << 16) | 10); // carrier duty
    m.soc.write32(
        RMT_BASE + 0x20,
        (1 << 0) | (1 << 6) | (1 << 20) | (1 << 21) | (1 << 22),
    );

    // The carrier must chop the HIGH burst: far more pad edges than the
    // two a clean burst would produce.
    let mut edges = 0u32;
    let mut prev = 0u32;
    let mut rx_end = false;
    for _ in 0..400 {
        m.step();
        let g = (m.soc.gpio_in_readback() >> 2) & 1;
        if g != prev {
            edges += 1;
            prev = g;
        }
        if m.soc.read32(RMT_BASE + 0x70) & (1 << 16) != 0 {
            rx_end = true;
            break;
        }
    }
    assert!(edges >= 6, "carrier chops the burst (edges={edges})");
    assert!(rx_end, "demodulated capture raises rx_end");
    // Envelope: first item HIGH ~400 then LOW (demod release costs up to
    // DEMOD_RELEASE quanta = 256 ticks late on the falling edge, plus the
    // usual ~64-tick edge uncertainty; the LOW half runs to the idle
    // timeout by design).
    let w0 = m.soc.read32(RMTMEM_BASE + 0x400);
    assert_eq!((w0 >> 15) & 1, 1, "envelope opens HIGH");
    let d0 = w0 & 0x7FFF;
    assert!((300..=700).contains(&d0), "HIGH width ~400, got {d0}");
    assert_eq!((w0 >> 31) & 1, 0, "envelope closes LOW");
    let d1 = (w0 >> 16) & 0x7FFF;
    assert!(d1 > 300, "LOW gap captured (runs to idle), got {d1}");
}

#[test]
fn sigmadelta_drives_gpio_at_duty_ratio() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::{GPIO_BASE, IRAM_BASE};
    use esp32s3_soc::sigmadelta::GPIO_SD_BASE;

    // Firmware: route SDM channel 0 (GPIO-matrix signal 93) -> GPIO2 via
    // FUNC_OUT_SEL, enable GPIO2, and configure SDM channel 0 to duty=128
    // (50%) prescale=0. Then spin. The host samples gpio_output() bit 2 and
    // verifies the Sigma-Delta PDM averages to ~50% high.
    const STASH: u32 = 0x3FC8_0100;
    let mut a = Asm::new(IRAM_BASE);
    let l_gpio = a.offset();
    a.lit(0);
    let l_sdm = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // GPIO_BASE
    a.patch_l32r(p, IRAM_BASE + l_gpio as u32);
    let p = a.l32r(3); // GPIO_SD_BASE
    a.patch_l32r(p, IRAM_BASE + l_sdm as u32);
    a.movi_n(4, 1 << 2);
    a.s32i(4, 2, 0x20); // GPIO_ENABLE: bit 2
    a.li(6, (GPIO_BASE + 0x554) as i32); // FUNC_OUT_SEL base
    a.li(4, 93); // GPIO_SD0_OUT_IDX
    a.s32i(4, 6, 8); // GPIO2 FUNC_OUT_SEL = 93
    a.li(4, 0); // signed duty 0 = 50% (esp-idf writes signed duty to reg)
    a.s32i(4, 3, 0x00); // GPIO_SD channel 0
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    a.bytes_mut()[l_gpio..l_gpio + 4].copy_from_slice(&GPIO_BASE.to_le_bytes());
    a.bytes_mut()[l_sdm..l_sdm + 4].copy_from_slice(&GPIO_SD_BASE.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..500 {
        if m.soc.read32(STASH) == 0xCAFE {
            break;
        }
        m.step();
    }
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "firmware configured SDM");
    let pin = |m: &Esp32S3| (m.gpio_output() & (1 << 2)) != 0;
    // 2048 steps = 8 full 256-tick PDM periods; duty 128/256 -> ~1024 high.
    let mut high = 0u32;
    for _ in 0..2048 {
        if pin(&m) {
            high += 1;
        }
        m.step();
    }
    assert!(
        (1000..=1048).contains(&high),
        "SDM 50% duty ~1024 high over 2048 steps, got {high}"
    );
}

#[test]
fn rtc_io_registers_round_trip_and_w1ts_w1tc() {
    use esp32s3_soc::rtc_io::RTC_IO_BASE;

    let mut m = Esp32S3::new();

    // `out` register round-trips.
    m.soc.write32(RTC_IO_BASE, 0x55);
    assert_eq!(m.soc.read32(RTC_IO_BASE), 0x55);

    // `out_w1ts` sets bits, `out_w1tc` clears bits (real silicon semantics).
    m.soc.write32(RTC_IO_BASE + 0x04, 0xAA);
    assert_eq!(m.soc.read32(RTC_IO_BASE), 0xFF);
    m.soc.write32(RTC_IO_BASE + 0x08, 0x0F);
    assert_eq!(m.soc.read32(RTC_IO_BASE), 0xF0);

    // `enable` w1ts/w1tc.
    m.soc.write32(RTC_IO_BASE + 0x0C, 0x1);
    m.soc.write32(RTC_IO_BASE + 0x10, 0x4);
    assert_eq!(m.soc.read32(RTC_IO_BASE + 0x0C), 0x5);
    m.soc.write32(RTC_IO_BASE + 0x14, 0x5);
    assert_eq!(m.soc.read32(RTC_IO_BASE + 0x0C), 0x0);

    // Reading a `*_w1ts` register returns 0.
    m.soc.write32(RTC_IO_BASE + 0x04, 0xFF);
    assert_eq!(m.soc.read32(RTC_IO_BASE + 0x04), 0);

    // A plain pad-config register stores the written value.
    m.soc.write32(RTC_IO_BASE + 0x5BC, 0x1234_5678);
    assert_eq!(m.soc.read32(RTC_IO_BASE + 0x5BC), 0x1234_5678);
}

#[test]
fn rng_data_register_returns_varying_values() {
    use esp32s3_soc::rng::RNG_BASE;

    let mut m = Esp32S3::new();
    let a = m.soc.read32(RNG_BASE + 0x7C); // WDEV_RND_REG
    let b = m.soc.read32(RNG_BASE + 0x7C);
    let c = m.soc.read32(RNG_BASE + 0x7C);
    assert_ne!(a, b, "consecutive RNG reads must differ");
    assert_ne!(b, c, "consecutive RNG reads must differ");

    // Non-data registers store writes.
    m.soc.write32(RNG_BASE + 0x10, 0xCAFE);
    assert_eq!(m.soc.read32(RNG_BASE + 0x10), 0xCAFE);
}

#[test]
fn ulp_registers_round_trip() {
    use esp32s3_soc::ulp::ULP_BASE;

    let mut m = Esp32S3::new();
    // ULP-RISC-V block is at page 0x6000_8000 + 0x100.
    m.soc.write32(ULP_BASE, 0xDEAD_BEEF); // core
    m.soc.write32(ULP_BASE + 0x04, 0x1234_5678); // ocp
    m.soc.write32(ULP_BASE + 0x0C, 0xAB); // general reg 0
    assert_eq!(m.soc.read32(ULP_BASE), 0xDEAD_BEEF);
    assert_eq!(m.soc.read32(ULP_BASE + 0x04), 0x1234_5678);
    assert_eq!(m.soc.read32(ULP_BASE + 0x0C), 0xAB);
}

#[test]
fn sdmmc_registers_round_trip() {
    use esp32s3_soc::sdmmc::SDMMC_BASE;

    let mut m = Esp32S3::new();
    m.soc.write32(SDMMC_BASE, 0x000F_0001); // CTRL
    m.soc.write32(SDMMC_BASE + 0x2C, 0x0020_0000); // CMD (no start bit -> stores)
    m.soc.write32(SDMMC_BASE + 0x30, 0xCAFE_BEEF); // RESP0
    // CTRL bit 0 (controller_reset) self-clears like silicon.
    assert_eq!(m.soc.read32(SDMMC_BASE), 0x000F_0000);
    assert_eq!(m.soc.read32(SDMMC_BASE + 0x2C), 0x0020_0000);
    assert_eq!(m.soc.read32(SDMMC_BASE + 0x30), 0xCAFE_BEEF);
}

#[test]
fn rtc_i2c_registers_round_trip() {
    use esp32s3_soc::rtc_i2c::RTC_I2C_BASE;

    let mut m = Esp32S3::new();
    // RTC_I2C (LP/I2C) block at 0x6000_8C00.
    m.soc.write32(RTC_I2C_BASE, 0x0000_0032); // I2C_SCL_LOW
    m.soc.write32(RTC_I2C_BASE + 0x04, 0x0000_0064); // I2C_SCL_HIGH
    m.soc.write32(RTC_I2C_BASE + 0x0C, 0x00FF_00AA); // I2C_CTRL
    assert_eq!(m.soc.read32(RTC_I2C_BASE), 0x0000_0032);
    assert_eq!(m.soc.read32(RTC_I2C_BASE + 0x04), 0x0000_0064);
    assert_eq!(m.soc.read32(RTC_I2C_BASE + 0x0C), 0x00FF_00AA);
}

#[test]
fn lp_uart_registers_round_trip() {
    use esp32s3_soc::lp_uart::LP_UART_BASE;

    let mut m = Esp32S3::new();
    // LP_UART block at 0x6002_5400 (shares the GPSPI3 page).
    m.soc.write32(LP_UART_BASE, 0x0000_00AB); // FIFO
    m.soc.write32(LP_UART_BASE + 0x14, 0x00AA_00BB); // CLKDIV
    m.soc.write32(LP_UART_BASE + 0x20, 0x1234_5678); // CONF0
    assert_eq!(m.soc.read32(LP_UART_BASE), 0x0000_00AB);
    assert_eq!(m.soc.read32(LP_UART_BASE + 0x14), 0x00AA_00BB);
    assert_eq!(m.soc.read32(LP_UART_BASE + 0x20), 0x1234_5678);
}

/// Poke the TWAI/CAN controller through the bus (mirrors the `esp32s3_twai`
/// arduino-cli sketch): enter self-test (loopback) mode, load a 13-byte
/// frame, transmit, poll the RX-buffer status, and confirm the looped-back
/// frame matches plus RRB clears `rbs`.
#[test]
fn twai_loopback_transmits_and_receives() {
    use esp32s3_soc::twai::TWAI_BASE;

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 19);
    let b = TWAI_BASE;

    // Enter reset mode so the acceptance filter is writable.
    m.soc.write32(b, 1);
    // Acceptance filter: code 0, mask 0xFFFFFFFF (all don't-care) -> accept all.
    for off in [0x40u32, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C] {
        m.soc
            .write32(b + off, if off < 0x50 { 0 } else { 0xFFFF_FFFF });
    }
    // Leave reset, enter self-test mode (stm = bit 2) -> TX loops back to RX.
    m.soc.write32(b, 1 << 2);

    // Load a 13-byte frame: DLC=8 standard data frame, ID 0x123, payload 0x13..0x1C.
    let tx: [u32; 13] = [
        0x08, 0x24, 0x60, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C,
    ];
    for (i, &v) in tx.iter().enumerate() {
        m.soc.write32(b + 0x40 + (i as u32) * 4, v);
    }

    // Transmission request (command.tr = bit 0).
    m.soc.write32(b + 0x04, 1);

    // Poll the RX-buffer status bit (SR.rbs = bit 0); advance the timers in
    // case the loopback is gated on a tick.
    let mut got = false;
    for _ in 0..1000 {
        if m.soc.read32(b + 0x08) & 1 != 0 {
            got = true;
            break;
        }
        m.soc.tick_timers(1);
    }
    assert!(got, "TWAI loopback: RX buffer never filled");

    let mut matched = true;
    for (i, &v) in tx.iter().enumerate() {
        if m.soc.read32(b + 0x40 + (i as u32) * 4) != v {
            matched = false;
        }
    }
    assert!(matched, "TWAI loopback: received frame differs from sent");

    // Release the RX buffer (command.rrb = bit 2) and confirm rbs clears.
    m.soc.write32(b + 0x04, 1 << 2);
    assert_eq!(m.soc.read32(b + 0x08) & 1, 0, "TWAI RRB did not clear rbs");
}

/// Poke the legacy deep-sleep path directly (mirrors the `esp32s3_deepsleep_poke`
/// arduino-cli sketch): program a sleep period, write `RTC_CNTL_SLEEP_EN`, then
/// step until the machine fast-forwards and reboots with the timer wakeup cause.
#[test]
fn deep_sleep_poke_wakes_with_timer_cause() {
    use esp32s3_soc::rtc::{
        RTC_CNTL_BASE, SLEEP_EN_BIT, SLP_TIMER0_OFF, SLP_TIMER1_OFF, SLP_WAKEUP_CAUSE_OFF,
        STATE0_OFF,
    };
    const SLP_TIMER0: u32 = RTC_CNTL_BASE + SLP_TIMER0_OFF;
    const SLP_TIMER1: u32 = RTC_CNTL_BASE + SLP_TIMER1_OFF;
    const STATE0: u32 = RTC_CNTL_BASE + STATE0_OFF;
    const WAKEUP_CAUSE: u32 = RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF;

    let mut m = Esp32S3::new();
    m.soc.write32(SLP_TIMER0, 0x0000_1234);
    m.soc.write32(SLP_TIMER1, 0x0000_0000);
    // Trigger power-down (bit 31 of STATE0).  This requests deep-sleep.
    let prev = m.soc.read32(STATE0);
    m.soc.write32(STATE0, prev | SLEEP_EN_BIT);

    // Step until the wakeup-cause register reports a timer wakeup, or we give up.
    let timer_cause = 1 << 3; // RTC_TIMER_TRIG_EN
    let mut woke = false;
    for _ in 0..200_000 {
        m.step();
        if m.soc.read32(WAKEUP_CAUSE) & timer_cause != 0 {
            woke = true;
            break;
        }
    }
    assert!(woke, "machine did not wake from deep-sleep");
    assert_eq!(m.soc.read32(WAKEUP_CAUSE) & timer_cause, timer_cause);
}

/// `RTC_CNTL_RESET_STATE_REG` (+0x38) reports POWERON (1) on normal boot and
/// DEEPSLEEP (5) after a deep-sleep wake — the live ROM's
/// `esp_rom_get_reset_reason` returns these fields directly, and
/// `esp_sleep_get_wakeup_cause` gates on PRO == 5 (see the
/// `esp32s3_deepsleep` driver sketch: WOKE/PASS requires it).
#[test]
fn rtc_reset_cause_poweron_then_deepsleep_after_wake() {
    use esp32s3_soc::rtc::{
        RESET_CAUSE_DEEPSLEEP, RESET_STATE_OFF, RTC_CNTL_BASE, SLEEP_EN_BIT, SLP_TIMER0_OFF,
        SLP_TIMER1_OFF, STATE0_OFF,
    };
    const RESET_STATE: u32 = RTC_CNTL_BASE + RESET_STATE_OFF;

    let mut m = Esp32S3::new();
    // Normal boot: silicon-accurate POWERON.
    assert_eq!(m.soc.read32(RESET_STATE) & 0x3F, 1, "PRO cause");
    assert_eq!((m.soc.read32(RESET_STATE) >> 6) & 0x3F, 1, "APP cause");

    // Poke a deep sleep + wake like `deep_sleep_poke_wakes_with_timer_cause`:
    // mark it deep via DIG_PWC DG_WRAP_PD_EN (bit 31), as the esp-idf
    // deep-sleep driver does (SLEEP_EN alone now means light/resume).
    m.soc.write32(RTC_CNTL_BASE + SLP_TIMER0_OFF, 0x100);
    m.soc.write32(RTC_CNTL_BASE + SLP_TIMER1_OFF, 0);
    m.soc.write32(RTC_CNTL_BASE + 0x90, 1 << 31);
    let prev = m.soc.read32(RTC_CNTL_BASE + STATE0_OFF);
    m.soc
        .write32(RTC_CNTL_BASE + STATE0_OFF, prev | SLEEP_EN_BIT);
    for _ in 0..200_000 {
        m.step();
        if m.soc.read32(RESET_STATE) & 0x3F == RESET_CAUSE_DEEPSLEEP {
            break;
        }
    }
    assert_eq!(
        m.soc.read32(RESET_STATE) & 0x3F,
        RESET_CAUSE_DEEPSLEEP,
        "PRO cause"
    );
    assert_eq!(
        (m.soc.read32(RESET_STATE) >> 6) & 0x3F,
        RESET_CAUSE_DEEPSLEEP,
        "APP cause"
    );
}

/// EXT0 deep-sleep wake: with WAKEUP_ENA EXT0 armed, EXT0 pin = RTC pad 4
/// at HIGH level and GPIO4 driven HIGH, `SLEEP_EN` must reboot immediately
/// with the EXT0 cause bit (no timer involved).
#[test]
fn deep_sleep_ext0_wakes_with_ext0_cause() {
    use esp32s3_soc::gpio::{GPIO_ENABLE_W1TS, GPIO_OUT_W1TS};
    use esp32s3_soc::memmap::GPIO_BASE;
    use esp32s3_soc::rtc::{
        EXT_CONF_OFF, RTC_CNTL_BASE, SLEEP_EN_BIT, SLP_WAKEUP_CAUSE_OFF, STATE0_OFF,
        WAKEUP_STATE_OFF,
    };
    let mut m = Esp32S3::new();
    // Drive GPIO4 HIGH from GPIO_OUT (loopback-readable).
    m.soc.write32(GPIO_BASE + GPIO_ENABLE_W1TS, 1 << 4);
    m.soc.write32(GPIO_BASE + GPIO_OUT_W1TS, 1 << 4);
    // Arm EXT0 (ENA bit 15), level HIGH (EXT_CONF bit 30), RTC pad 4.
    m.soc.write32(RTC_CNTL_BASE + WAKEUP_STATE_OFF, 1 << 15);
    m.soc.write32(RTC_CNTL_BASE + EXT_CONF_OFF, 1 << 30);
    m.soc.write32(0x6000_84DC, 4 << 27); // RTC_IO EXT_WAKEUP0_SEL
    // Sleep: no timer programmed, so only the EXT0 level can wake.
    let prev = m.soc.read32(RTC_CNTL_BASE + STATE0_OFF);
    m.soc
        .write32(RTC_CNTL_BASE + STATE0_OFF, prev | SLEEP_EN_BIT);
    for _ in 0..200_000 {
        m.step();
        if m.soc.read32(RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF) & 1 != 0 {
            break;
        }
    }
    assert_eq!(
        m.soc.read32(RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF) & 1,
        1,
        "EXT0 cause after level wake"
    );
    // EXT0 with the pin LOW must not wake (level mismatch).
    let mut m = Esp32S3::new();
    m.soc.write32(GPIO_BASE + GPIO_ENABLE_W1TS, 1 << 4);
    m.soc.write32(RTC_CNTL_BASE + WAKEUP_STATE_OFF, 1 << 15);
    m.soc.write32(RTC_CNTL_BASE + EXT_CONF_OFF, 1 << 30); // want HIGH
    m.soc.write32(0x6000_84DC, 4 << 27);
    m.soc.write32(RTC_CNTL_BASE + STATE0_OFF, SLEEP_EN_BIT);
    for _ in 0..50_000 {
        m.step();
    }
    assert_eq!(
        m.soc.read32(RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF),
        0,
        "no wake while level unmet (still asleep)"
    );
    assert!(m.is_asleep(), "machine still fast-forwarding sleep");
}

/// Light sleep (SLEEP_EN with no DIG_PWC power-down bits) resumes in place:
/// no reboot (DRAM marker and POWERON reset cause survive) with the stashed
/// wakeup cause applied to the WAKEUP_CAUSE register.
#[test]
fn light_sleep_resumes_without_reboot() {
    use esp32s3_soc::memmap::DRAM_BASE;
    use esp32s3_soc::rtc::{
        CAUSE_TIMER, RESET_CAUSE_POWERON, RESET_STATE_OFF, RTC_CNTL_BASE, SLEEP_EN_BIT,
        SLP_TIMER0_OFF, SLP_TIMER1_OFF, SLP_WAKEUP_CAUSE_OFF, STATE0_OFF,
    };
    let mut m = Esp32S3::new();
    // Retained marker (deep sleep would reboot and lose plain DRAM state
    // set this way outside the image; light sleep keeps everything).
    m.soc.write32(DRAM_BASE + 0x1000, 0x1234_5678);
    m.soc.write32(RTC_CNTL_BASE + SLP_TIMER0_OFF, 0x100);
    m.soc.write32(RTC_CNTL_BASE + SLP_TIMER1_OFF, 0);
    // No DIG_PWC PD bits -> light (resume), unlike the deep tests above.
    let prev = m.soc.read32(RTC_CNTL_BASE + STATE0_OFF);
    m.soc
        .write32(RTC_CNTL_BASE + STATE0_OFF, prev | SLEEP_EN_BIT);
    for _ in 0..200_000 {
        m.step();
        if !m.is_asleep() {
            break;
        }
    }
    assert!(!m.is_asleep(), "light sleep woke");
    assert_eq!(
        m.soc.read32(RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF) & CAUSE_TIMER,
        CAUSE_TIMER,
        "timer wake cause applied"
    );
    assert_eq!(
        m.soc.read32(DRAM_BASE + 0x1000),
        0x1234_5678,
        "DRAM retained (no reboot)"
    );
    assert_eq!(
        m.soc.read32(RTC_CNTL_BASE + RESET_STATE_OFF) & 0x3F,
        RESET_CAUSE_POWERON,
        "reset cause still POWERON (no reboot)"
    );
}

/// `RTC_CNTL_SLP_WAKEUP_CAUSE` (0x130, inside the ULP sub-region) must be served
/// by RTC_CNTL, not the ULP block, so the wakeup cause survives a deep-sleep.
#[test]
fn rtc_slp_wakeup_cause_register_is_rtc() {
    use esp32s3_soc::rtc::{RTC_CNTL_BASE, SLP_WAKEUP_CAUSE_OFF};
    let mut m = Esp32S3::new();
    m.soc
        .write32(RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF, 0xABCD_1234);
    assert_eq!(
        m.soc.read32(RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF),
        0xABCD_1234
    );
}

/// P5 register-store peripherals (SENSITIVE/PMS, WCL/World-Ctrl, PERI_BACKUP,
/// SYSCON/PCR-clocks, I2S0/1, ASSIST_DEBUG): firmware pokes them during
/// boot/init. Each is a `RegStore` that retains writes and reads them back, so
/// accesses never panic and round-trip. (LCD_CAM is modeled functionally in
/// `lcd_cam.rs` and covered by `lcd_cam_fifo_and_transfer_done`.)
#[test]
fn p5_stub_peripherals_round_trip() {
    let addrs = [
        (SENSITIVE_BASE, "SENSITIVE"),
        (WCL_BASE, "WCL"),
        (PERI_BACKUP_BASE, "PERI_BACKUP"),
        (SYSCON_BASE, "SYSCON"),
        (I2S0_BASE, "I2S0"),
        (I2S1_BASE, "I2S1"),
        (ASSIST_DEBUG_BASE, "ASSIST_DEBUG"),
        (LCD_CAM_BASE, "LCD_CAM"),
    ];
    let mut m = Esp32S3::new();
    for (base, name) in addrs.iter() {
        // Use plain config registers so the round-trip holds even for
        // peripherals whose control words have side effects on write: I2S
        // RX_CONF/TX_CONF (0x20/0x24) self-clear reset/start/update bits, so
        // use CONF1 (0x28/0x2C, plain field stores) instead. 0x10 stays
        // avoided: it is a computed read-only register on some blocks
        // (e.g. I2S INT_ST).
        let w = 0x1234_5678u32;
        m.soc.write32(*base + 0x28, w);
        let r = m.soc.read32(*base + 0x28);
        assert_eq!(r, w, "{} register round-trip failed", name);
        // A second, distinct offset also round-trips.
        m.soc.write32(*base + 0x2C, 0xDEAD_BEEF);
        assert_eq!(m.soc.read32(*base + 0x2C), 0xDEAD_BEEF, "{} off 0x2C", name);
    }
}

/// LCD_CAM camera capture delivers a host-staged frame: VSYNC interrupt +
/// FIFO words in order + START self-clear, with the VSYNC input level
/// visible on a pad routed to CAM_V_SYNC (sensor loopback).
#[test]
fn lcd_cam_capture_delivers_injected_frame() {
    use esp32s3_soc::gpio::GPIO_FUNC_IN_SEL_0;
    use esp32s3_soc::lcd_cam::LCD_CAM_BASE;
    use esp32s3_soc::memmap::GPIO_BASE;
    let cam_ctrl1 = LCD_CAM_BASE + 0x08;
    let cam_data = LCD_CAM_BASE + 0x48;
    let cam_fifo_status = LCD_CAM_BASE + 0x4C;
    let lc_int_raw = LCD_CAM_BASE + 0x68;

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 8);
    // CAM_V_SYNC (152) -> GPIO7 pad (input routing only; pad is sensor-driven).
    m.soc.write32(GPIO_BASE + GPIO_FUNC_IN_SEL_0 + 152 * 4, 7);
    // Stage 4 words (LE bytes) and start the capture.
    let mut bytes = [0u8; 16];
    for (i, w) in [0x0102_0304u32, 0xA5A5_A5A5, 0xDEAD_BEEF, 0x1234_5678]
        .iter()
        .enumerate()
    {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    m.soc.cam_inject_frame(&bytes);
    m.soc.write32(cam_ctrl1, 1 << 29); // CAM_START
    // Sample mid-stream (2 of 4 words): VSYNC asserted + pad loopback live.
    m.soc.tick_timers(2);
    assert_eq!(m.soc.read32(lc_int_raw) & (1 << 2), 1 << 2, "VSYNC latched");
    assert_eq!(
        (m.soc.gpio_in_readback() >> 7) & 1,
        1,
        "VSYNC visible on routed pad"
    );
    m.soc.tick_timers(8);
    assert_eq!(m.soc.read32(cam_fifo_status) & 0x7FF, 4, "frame arrived");
    assert_eq!(m.soc.read32(cam_ctrl1) & (1 << 29), 0, "START self-cleared");
    for w in [0x0102_0304u32, 0xA5A5_A5A5, 0xDEAD_BEEF, 0x1234_5678] {
        assert_eq!(m.soc.read32(cam_data), w, "frame word");
    }
}

/// LCD_CAM (= PARLIO) functional model: TX FIFO + transfer-start / done.
/// Words pushed to `LCD_DATA` (0x40) fill the TX FIFO; `LCD_FIFO_STATUS`
/// (0x44) reports the count. Setting `LCD_START` (bit 27 of `LCD_USER` 0x14)
/// drains the FIFO and raises `LCD_TRANS_DONE` (bit 1 of the LC_DMA_INT_*
/// block). The interrupt clears via `LC_DMA_INT_CLR`.
#[test]
fn lcd_cam_fifo_and_transfer_done() {
    use esp32s3_soc::lcd_cam::LCD_CAM_BASE;
    let lcd_user = LCD_CAM_BASE + 0x14;
    let lcd_data = LCD_CAM_BASE + 0x40;
    let lcd_fifo_status = LCD_CAM_BASE + 0x44;
    let lc_int_ena = LCD_CAM_BASE + 0x64;
    let lc_int_raw = LCD_CAM_BASE + 0x68;
    let lc_int_st = LCD_CAM_BASE + 0x6C;
    let lc_int_clr = LCD_CAM_BASE + 0x70;

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 8);
    // Enable + clear the TRANS_DONE interrupt, then fill the TX FIFO.
    m.soc.write32(lc_int_ena, 1 << 1);
    m.soc.write32(lc_int_clr, 0xF);
    m.soc.write32(lcd_data, 0x1111_1111);
    m.soc.write32(lcd_data, 0x2222_2222);
    m.soc.write32(lcd_data, 0x3333_3333);
    assert_eq!(m.soc.read32(lcd_fifo_status) & 0x7FF, 3, "TX FIFO count");
    // Start a transfer; the FIFO drains one word per PCLK cycle (~2 ticks).
    let user = m.soc.read32(lcd_user);
    m.soc.write32(lcd_user, user | (1 << 27));
    assert_eq!(
        m.soc.read32(lcd_fifo_status) & 0x7FF,
        3,
        "TX still queued pre-tick"
    );
    m.soc.tick_timers(40);
    assert_eq!(m.soc.read32(lcd_fifo_status) & 0x7FF, 0, "TX drained");
    assert_eq!(m.soc.read32(lc_int_raw) & (1 << 1), 1 << 1, "RAW done set");
    assert_eq!(m.soc.read32(lc_int_st) & (1 << 1), 1 << 1, "ST done set");
    // Clear and confirm.
    m.soc.write32(lc_int_clr, 1 << 1);
    assert_eq!(m.soc.read32(lc_int_raw) & (1 << 1), 0, "RAW cleared");
}

/// I2S TX serial output is observable on GPIO pins routed to the I2S0 SD/BCK
/// matrix signals. Pushing 0x8000 (MSB-first) and starting a transfer must
/// drive SD high on the first bit, and the transfer must raise `tx_done`.
#[test]
fn i2s_tx_drives_gpio_matrix_signals() {
    use esp32s3_soc::i2s::I2S0_BASE;
    let i2s_fifo = I2S0_BASE + 0x80;
    let i2s_tx_conf = I2S0_BASE + 0x24;
    let i2s_int_raw = I2S0_BASE + 0x0C;
    let sd_pin = 5u32; // route I2S0 SD (sig 25) here
    let bck_pin = 6u32; // route I2S0 BCK (sig 22) here
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 4);
    m.soc.write32(GPIO_BASE + 0x554 + 4 * sd_pin, 25);
    m.soc.write32(GPIO_BASE + 0x554 + 4 * bck_pin, 22);
    m.soc
        .write32(GPIO_BASE + 0x24, (1u32 << sd_pin) | (1u32 << bck_pin));
    m.soc.write32(i2s_fifo, 0x8000);
    m.soc.write32(I2S0_BASE + 0x34, 1); // TX_CLKM_CONF M = 1 (reset M = 2 halves the pace)
    m.soc.write32(i2s_tx_conf, 1 << 2); // TX_START
    m.soc.tick_timers(1);
    let out = m.soc.gpio_output();
    assert_eq!(
        (out >> sd_pin) & 1,
        1,
        "I2S0 SD high (MSB first) after 1 tick"
    );
    assert_eq!((out >> bck_pin) & 1, 1, "I2S0 BCK high after 1 tick");
    m.soc.tick_timers(40);
    assert_eq!(
        m.soc.read32(i2s_int_raw) & (1 << 1),
        1 << 1,
        "I2S tx_done fired"
    );
}

/// LCD_CAM parallel output is observable on GPIO pins routed to the LCD
/// data/CS matrix signals. A transfer presenting 0x00000001 must drive
/// DATA0 high and CS (active-low) low.
#[test]
fn lcd_cam_parallel_drives_gpio_matrix_signals() {
    use esp32s3_soc::lcd_cam::LCD_CAM_BASE;
    let lcd_user = LCD_CAM_BASE + 0x14;
    let lcd_data = LCD_CAM_BASE + 0x40;
    let data_pin = 7u32; // route LCD_DATA_OUT0 (sig 133)
    let cs_pin = 8u32; // route LCD_CS (sig 132)
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 8);
    m.soc.write32(GPIO_BASE + 0x554 + 4 * data_pin, 133);
    m.soc.write32(GPIO_BASE + 0x554 + 4 * cs_pin, 132);
    m.soc
        .write32(GPIO_BASE + 0x24, (1u32 << data_pin) | (1u32 << cs_pin));
    m.soc.write32(lcd_data, 0x0000_0001);
    m.soc.write32(lcd_user, 1 << 27); // LCD_START
    m.soc.tick_timers(1);
    let out = m.soc.gpio_output();
    assert_eq!((out >> data_pin) & 1, 1, "LCD DATA0 high during transfer");
    assert_eq!(
        (out >> cs_pin) & 1,
        0,
        "LCD CS active (low) during transfer"
    );
}

/// The esp-idf GDMA path for I2S: an `out` channel with `peri_sel == 3`
/// (I2S0) copies its descriptor's words into the I2S0 TX FIFO. Starting a
/// loopback TX then shifts those words into the I2S0 RX FIFO, which a
/// subsequent `read32(FIFO)` must return in order.
#[test]
fn i2s_gdma_out_feeds_tx_fifo() {
    let i2s_fifo = I2S0_BASE + 0x80;
    let i2s_tx_conf = I2S0_BASE + 0x24;
    let i2s_int_raw = I2S0_BASE + 0x0C;
    let desc = 0x3FC8_1000;
    let buf = 0x3FC8_2000;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, false, 4);
    // Descriptor: owner=1, eof=1, length=8 bytes (2 words).
    m.soc
        .write32(desc, (1u32 << 31) | (1u32 << 30) | (8u32 << 12));
    m.soc.write32(desc + 4, buf); // buffer pointer
    m.soc.write32(desc + 8, 0); // next = end
    m.soc.write32(buf, 0x1122_3344);
    m.soc.write32(buf + 4, 0x5566_7788);
    // Wire GDMA channel 0 OUT to I2S0 and start the link. The OUT_LINK start
    // register is at channel-offset 0x80 (OUT block base 0x60 + 0x20).
    m.soc.write32(GDMA_BASE + 0xA8, GDMA_I2S0_PERIPH); // out_peri_sel[0]
    m.soc
        .write32(GDMA_BASE + 0x80, (desc & 0x000F_FFFF) | (1 << 21)); // out_link start
    // Let the streaming pump pre-fill the TX FIFO before TX_START: starting
    // with an empty FIFO would shift out a zero underflow word first (as on
    // silicon) and the loopback would return it instead of the pattern.
    m.soc.tick_timers(4);
    // Start I2S0 TX with loopback so the FIFO words arrive at RX.
    // M = 1: the reset pre-div M = 2 would halve the shift pace.
    m.soc.write32(I2S0_BASE + 0x34, 1); // TX_CLKM_CONF
    m.soc.write32(i2s_tx_conf, (1 << 27) | (1 << 2)); // SIG_LOOPBACK | TX_START
    m.soc.tick_timers(200);
    assert_eq!(
        m.soc.read32(i2s_int_raw) & (1 << 1),
        1 << 1,
        "I2S tx_done fired"
    );
    let rx0 = m.soc.read32(i2s_fifo);
    let rx1 = m.soc.read32(i2s_fifo);
    assert!(
        rx0 == 0x1122_3344 && rx1 == 0x5566_7788,
        "RX words mismatch: rx0={:#x} rx1={:#x}",
        rx0,
        rx1
    );
}

/// GDMA SPI master DMA: an OUT descriptor stages bytes for one DMA-backed
/// SPI2 transfer (trans_done + usr clear), and the IN link copies the
/// captured RX (zeros, no device) into DRAM with both done bits raised.
#[test]
fn gdma_spi_out_runs_dma_transfer_and_in_returns_rx() {
    use esp32s3_soc::spi::{SPI_CLK_GATE, SPI_CLOCK, SPI_CMD, SPI_INT_RAW, SPI_MS_DLEN, SPI_USER};
    let desc = 0x3FC8_1000;
    let buf = 0x3FC8_2000;
    let rdesc = 0x3FC8_1100;
    let rbuf = 0x3FC8_2100;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    // TX descriptor: owner=1, eof=1, length=4 bytes.
    m.soc
        .write32(desc, (1u32 << 31) | (1u32 << 30) | (4u32 << 12));
    m.soc.write32(desc + 4, buf);
    m.soc.write32(desc + 8, 0);
    m.soc.write32(buf, 0xA53C_F00F); // MSB-first bytes A5 3C F0 0F
    // SPI2: 1024 cyc/bit (pre=15, n=63), 32-bit full-duplex, clock gate on.
    m.soc
        .write32(SPI2_BASE + SPI_CLOCK, (15 << 18) | (63 << 12) | (31 << 6));
    m.soc.write32(SPI2_BASE + SPI_MS_DLEN, 31);
    m.soc
        .write32(SPI2_BASE + SPI_USER, (1 << 27) | (1 << 28) | 1);
    m.soc.write32(SPI2_BASE + SPI_CLK_GATE, 1);
    // GDMA OUT ch0 -> SPI2, start.
    m.soc.write32(GDMA_BASE + 0xA8, GDMA_SPI2_PERIPH);
    m.soc
        .write32(GDMA_BASE + 0x80, (desc & 0x000F_FFFF) | (1 << 21));
    // Run the 32-bit transfer to completion.
    for _ in 0..40 {
        m.soc.tick_timers(1000);
    }
    assert_eq!(
        m.soc.read32(SPI2_BASE + SPI_CMD) & (1 << 24),
        0,
        "CMD.usr clears"
    );
    assert_eq!(
        m.soc.read32(SPI2_BASE + SPI_INT_RAW) & (1 << 12),
        1 << 12,
        "SPI trans_done latched"
    );
    assert_eq!(
        m.soc.read32(GDMA_BASE + 0x68) & 1,
        1,
        "GDMA out_done latched"
    );
    // RX descriptor + IN link start (ch0 IN block: link @ 0x20, start = bit 22).
    m.soc
        .write32(rdesc, (1u32 << 31) | (1u32 << 30) | (4u32 << 12));
    m.soc.write32(rdesc + 4, rbuf);
    m.soc.write32(rdesc + 8, 0);
    m.soc.write32(GDMA_BASE + 0x48, GDMA_SPI2_PERIPH);
    m.soc
        .write32(GDMA_BASE + 0x20, (rdesc & 0x000F_FFFF) | (1 << 22));
    assert_eq!(m.soc.read32(rbuf), 0, "RX capture is zeros (no device)");
    assert_eq!(
        m.soc.read32(GDMA_BASE + 0x08) & 1,
        1,
        "GDMA in_done latched"
    );
}

/// UHCI UART-DMA at the SoC level: a GDMA OUT descriptor (peri_sel 2)
/// moves bytes into UART1's TX FIFO through the framing-off pipe, and a
/// GDMA IN descriptor drains injected UART1 RX bytes back to DRAM, with
/// TX_START/RX_START latched in UHCI INT_ST. (Matrix source-14 delivery
/// follows the identical pattern proven by the SHA/ADC vector tests.)
#[test]
fn uhci_gdma_moves_uart_bytes_both_directions() {
    use esp32s3_soc::gdma::{GDMA_BASE, GDMA_UHCI0_PERIPH};
    use esp32s3_soc::uhci::UHCI0_BASE;
    let desc = 0x3FC8_1000;
    let buf = 0x3FC8_2000;
    let rdesc = 0x3FC8_3000;
    let rbuf = 0x3FC8_4000;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, false, 8);
    // UHCI: clock on, UART1 selected; enable both start interrupts.
    m.soc.write32(UHCI0_BASE, (1 << 11) | (1 << 3));
    m.soc.write32(UHCI0_BASE + 0x0C, (1 << 1) | (1 << 0));
    // OUT: 8-byte message to UART1 TX.
    m.soc
        .write32(desc, (1u32 << 31) | (1u32 << 30) | (8u32 << 12));
    m.soc.write32(desc + 4, buf);
    m.soc.write32(desc + 8, 0);
    m.soc.write32(buf, 0x4943_4855); // "UHCI" LE
    m.soc.write32(buf + 4, 0x5854_2D44); // "D-TX" LE
    m.soc.write32(GDMA_BASE + 0xA8, GDMA_UHCI0_PERIPH); // out_peri_sel[0]
    m.soc
        .write32(GDMA_BASE + 0x80, (desc & 0x000F_FFFF) | (1 << 21));
    assert_eq!(
        m.take_uart_tx(1),
        alloc::vec![0x55, 0x48, 0x43, 0x49, 0x44, 0x2D, 0x54, 0x58],
        "TX bytes reach UART1"
    );
    assert_ne!(
        m.soc.read32(UHCI0_BASE + 0x08) & (1 << 1),
        0,
        "TX_START latched"
    );
    // IN: inject 8 RX bytes, drain to DRAM.
    for &b in b"UHCI-RX!" {
        m.soc.uart_inject_rx(1, b);
    }
    m.soc
        .write32(rdesc, (1u32 << 31) | (1u32 << 30) | (16u32 << 12));
    m.soc.write32(rdesc + 4, rbuf);
    m.soc.write32(rdesc + 8, 0);
    m.soc.write32(GDMA_BASE + 0x108, GDMA_UHCI0_PERIPH); // in_peri_sel[1]
    m.soc
        .write32(GDMA_BASE + 0xE0, (rdesc & 0x000F_FFFF) | (1 << 22));
    let mut got = alloc::vec::Vec::new();
    for i in 0..2 {
        got.extend_from_slice(&m.soc.read32(rbuf + 4 * i).to_le_bytes());
    }
    assert_eq!(&got[..8], b"UHCI-RX!", "RX bytes reach DRAM");
    assert_ne!(
        m.soc.read32(UHCI0_BASE + 0x08) & (1 << 0),
        0,
        "RX_START latched"
    );
    // Clear both via INT_CLR.
    m.soc.write32(UHCI0_BASE + 0x10, (1 << 1) | (1 << 0));
    assert_eq!(m.soc.read32(UHCI0_BASE + 0x08), 0, "INT_ST clears");
}

/// MCPWM capture loopback at the SoC level: timer0 PWM drives GPIO2 via the
/// output matrix, GPIO2 feeds CAP0 via input selection, and two rising
/// edges latch timer values ~10000 ticks apart with the CAP0 interrupt.
#[test]
fn mcpwm_capture_measures_pwm_period_via_loopback() {
    use esp32s3_soc::gpio::GPIO_ENABLE_W1TS;
    use esp32s3_soc::memmap::GPIO_BASE;
    let mcpwm = 0x6001_E000;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 17);
    // GPIO2 <- PWM0_OUT0A (160), output driver on.
    m.soc.write32(GPIO_BASE + 0x554 + 2 * 4, 160);
    m.soc.write32(GPIO_BASE + GPIO_ENABLE_W1TS, 1 << 2);
    // CAP0 input <- GPIO2.
    m.soc.write32(GPIO_BASE + 0x154 + 166 * 4, 2);
    // Timer0: period 100, prescale 99, up mode, run.
    m.soc.write32(mcpwm + 0x04, (100 << 8) | 99);
    m.soc.write32(mcpwm + 0x08, (1 << 3) | 2);
    m.soc.write32(mcpwm + 0x40, 50);
    m.soc.write32(mcpwm + 0x50, (2 << 4) | 1);
    // Capture timer on, ch0 rising edges.
    m.soc.write32(mcpwm + 0xE8, 1);
    m.soc.write32(mcpwm + 0xF0, 1 | (2 << 1));
    // Run past ~1.5 PWM periods (10000 ticks each): the first rising edge
    // latches ~10000 (later edges would overwrite, so don't overrun).
    for _ in 0..15 {
        m.soc.tick_timers(1000);
    }
    assert_eq!(
        m.soc.read32(mcpwm + 0x114) & (1 << 27),
        1 << 27,
        "CAP0 interrupt latched"
    );
    let c1 = m.soc.read32(mcpwm + 0xFC);
    assert!(c1 > 9000 && c1 < 11000, "first capture ~10000, got {c1}");
}

#[test]
fn mcpwm_carrier_chops_output_on_gpio() {
    use esp32s3_soc::gpio::GPIO_ENABLE_W1TS;
    use esp32s3_soc::memmap::GPIO_BASE;
    let mcpwm = 0x6001_E000;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 17);
    // GPIO2 <- PWM0_OUT0A (160), output driver on.
    m.soc.write32(GPIO_BASE + 0x554 + 2 * 4, 160);
    m.soc.write32(GPIO_BASE + GPIO_ENABLE_W1TS, 1 << 2);
    // Timer0: period 100, prescale 0, up mode, run; 50% via comparator A.
    m.soc.write32(mcpwm + 0x04, 100 << 8);
    m.soc.write32(mcpwm + 0x08, (1 << 3) | 2);
    m.soc.write32(mcpwm + 0x40, 50);
    m.soc.write32(mcpwm + 0x50, (2 << 4) | 1);
    // Carrier: en + prescale 0 (period 8 steps) + duty 4/8.
    m.soc.write32(mcpwm + 0x64, 1 | (4 << 5));
    for _ in 0..200 {
        m.step();
    }
    let mut high = 0u32;
    let mut edges = 0u32;
    let mut prev = (m.soc.gpio_output() >> 2) & 1;
    for _ in 0..800 {
        m.step();
        let lv = (m.soc.gpio_output() >> 2) & 1;
        high += lv;
        edges += u32::from(lv != prev);
        prev = lv;
    }
    // 50% PWM x 50% carrier = 25% average; unchopped would be 400/800.
    assert!(
        (180..=220).contains(&high),
        "carrier duty wrong: {high}/800"
    );
    assert!(edges >= 60, "carrier did not chop: {edges} edges");
}

#[test]
fn spi_slave_dma_routes_through_gdma_links() {
    use esp32s3_soc::gdma::GDMA_BASE;
    let spi2 = 0x6002_4000u32;
    let mut m = Esp32S3::new();
    // SPI2 slave + DMA RX/TX enabled.
    m.soc.write32(spi2 + 0xE0, 1 << 26); // slave_mode
    m.soc.write32(spi2 + 0x30, (1 << 25) | (1 << 26)); // dma_conf rx+tx ena
    // DRAM scratch: TX desc + 4-byte pattern, RX desc + 4-byte buffer.
    let txd = 0x3FC8_1000u32;
    let txb = 0x3FC8_1100u32;
    let rxd = 0x3FC8_1200u32;
    let rxb = 0x3FC8_1300u32;
    for (i, b) in [0x12u32, 0x34, 0x56, 0x78].iter().enumerate() {
        m.soc.write8(txb + i as u32, *b);
    }
    for (d, b) in [(txd, txb), (rxd, rxb)] {
        m.soc.write32(d, (4) | (4 << 12) | (1 << 30) | (1 << 31));
        m.soc.write32(d + 4, b);
        m.soc.write32(d + 8, 0);
        m.soc.write32(d + 12, 0);
    }
    // GDMA ch0 OUT (TX) + IN (RX) links wired to SPI2, started. Slave
    // links arm only: no done bits, descriptors stay owned.
    m.soc.write32(GDMA_BASE + 0xA8, 0); // out_peri_sel = SPI2
    m.soc.write32(GDMA_BASE + 0x80, (txd & 0xFFFFF) | (1 << 21));
    m.soc.write32(GDMA_BASE + 0x48, 0); // in_peri_sel = SPI2
    m.soc.write32(GDMA_BASE + 0x20, (rxd & 0xFFFFF) | (1 << 22));
    assert_eq!(m.soc.read32(spi2 + 0x3C) & 0xF00, 0, "no done yet");
    assert_ne!(m.soc.read32(txd) >> 31, 0, "TX desc still owned");
    // Host master-write: bytes land in the IN-link DRAM buffer with
    // WR_DMA_DONE (bit 9), not the CPU data buffer.
    m.soc.spi_slave_inject_write(0, &[0xDE, 0xAD, 0xBE, 0xEF]);
    for (i, b) in [0xDEu32, 0xAD, 0xBE, 0xEF].iter().enumerate() {
        assert_eq!(m.soc.read8(rxb + i as u32), *b, "RX DRAM byte {i}");
    }
    assert_eq!(m.soc.read32(spi2 + 0xE4) & 0x3FFFF, 32, "SLAVE1 bitlen");
    assert_ne!(m.soc.read32(spi2 + 0x3C) & (1 << 9), 0, "WR_DMA_DONE");
    assert_eq!(m.soc.read32(rxd) >> 31, 0, "RX desc handed back");
    // Host master-read: bytes source the OUT-link DRAM buffer with
    // RD_DMA_DONE (bit 8); the OUT descriptor is handed back.
    let got = m.soc.spi_slave_take_read(0, 4);
    assert_eq!(got, alloc::vec![0x12, 0x34, 0x56, 0x78]);
    assert_ne!(m.soc.read32(spi2 + 0x3C) & (1 << 8), 0, "RD_DMA_DONE");
    assert_eq!(m.soc.read32(txd) >> 31, 0, "TX desc handed back");
}

#[test]
fn gdma_m2m_copies_memory_to_memory() {
    use esp32s3_soc::gdma::GDMA_BASE;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    // DRAM scratch: source pattern, source/dest descriptors (8 bytes).
    let src = 0x3FC8_2000u32;
    let dst = 0x3FC8_2100u32;
    let odesc = 0x3FC8_2200u32;
    let idesc = 0x3FC8_2300u32;
    for (i, b) in [0x11u32, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
        .iter()
        .enumerate()
    {
        m.soc.write8(src + i as u32, *b);
        m.soc.write8(dst + i as u32, 0);
    }
    for d in [odesc, idesc] {
        let b = if d == odesc { src } else { dst };
        m.soc.write32(d, (8) | (8 << 12) | (1 << 30) | (1 << 31));
        m.soc.write32(d + 4, b);
        m.soc.write32(d + 8, 0);
        m.soc.write32(d + 12, 0);
    }
    // M2M mode on channel 1 (IN_CONF0 mem_trans_en, bit 4).
    m.soc.write32(GDMA_BASE + 0xC0, 1 << 4);
    // OUT start alone only arms: no copy until the IN link starts too.
    m.soc.write32(
        GDMA_BASE + 0xC0 + 0x60 + 0x20,
        (odesc & 0xFFFFF) | (1 << 21),
    );
    assert_eq!(m.soc.read8(dst), 0, "no copy before both links start");
    m.soc
        .write32(GDMA_BASE + 0xC0 + 0x20, (idesc & 0xFFFFF) | (1 << 22));
    for (i, b) in [0x11u32, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
        .iter()
        .enumerate()
    {
        assert_eq!(m.soc.read8(dst + i as u32), *b, "M2M byte {i}");
    }
    assert_eq!(m.soc.read32(odesc) >> 31, 0, "OUT desc handed back");
    assert_eq!(m.soc.read32(idesc) >> 31, 0, "IN desc handed back");
    assert_ne!(
        m.soc.read32(GDMA_BASE + 0xC0 + 0x60 + 0x08) & 1,
        0,
        "OUT done"
    );
    assert_ne!(m.soc.read32(GDMA_BASE + 0xC0 + 0x08) & 1, 0, "IN done");
}

#[test]
fn usb_otg_reset_and_fifo_page_routing() {
    use esp32s3_soc::usb_otg::{USB_OTG_BASE, USB_OTG_FIFO_PAGE};
    let mut m = Esp32S3::new();
    // Core reset handshake through the bus (both pages route).
    m.soc.write32(USB_OTG_BASE + 0x800, 0x1234_5678); // DCFG
    m.soc.write32(USB_OTG_BASE + 0x010, 1); // GRSTCTL.CSFTRST
    assert_eq!(m.soc.read32(USB_OTG_BASE + 0x010) & 1, 0, "self-clears");
    assert_ne!(m.soc.read32(USB_OTG_BASE + 0x010) & (1 << 31), 0, "AHBIDLE");
    assert_eq!(m.soc.read32(USB_OTG_BASE + 0x800), 0, "bank restored");
    // TXFIFO staging via the second page.
    m.soc.write32(USB_OTG_FIFO_PAGE, 0xA5A5_A5A5);
    m.soc.write32(USB_OTG_FIFO_PAGE, 0x5A5A_5A5A);
    assert_eq!(m.soc.read32(USB_OTG_BASE + 0x914), 256 - 2);
    assert_eq!(m.soc.read32(USB_OTG_BASE + 0x014), 0, "quiet, no host");
}

#[test]
fn ulp_runs_poked_program_via_bus() {
    use esp32s3_soc::memmap::RTC_SLOW_BASE;
    use esp32s3_soc::ulp::ULP_BASE;
    let mut m = Esp32S3::new();
    // Hand-assembled rv32im program: store 0x12345678 to ULP reg slot 0 then ebreak.
    let prog: [u32; 6] = [
        0x6000_80B7,
        0x10C0_8093,
        0x1234_5137,
        0x6781_0113,
        0x0020_A023,
        0x0010_0073,
    ];
    for (i, w) in prog.iter().enumerate() {
        m.soc.write32(RTC_SLOW_BASE + (i as u32) * 4, *w);
    }
    // Release the ULP core.
    m.soc.write32(ULP_BASE, 1);
    // Run only the ULP (via tick_timers) — no Xtensa execution needed.
    for _ in 0..200 {
        m.soc.tick_timers(1);
    }
    assert_eq!(m.soc.read32(ULP_BASE + 0x0C), 0x1234_5678);
}

/// MWDT0 stage 0 = interrupt (not reset): the WDT must raise TIMG0 INT_RAW.WDT
/// (source 52), the interrupt matrix must route it to a CPU line, and the
/// level-3 handler must run — without rebooting the machine.  This is the
/// edge case where a watched-dog timeout is handled by firmware rather than
/// triggering a system reset.
#[test]
fn wdt_interrupt_fires_instead_of_reset() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::{INT_MATRIX_BASE, TIMG0_BASE};

    // App: literal pool first (CTR/STASH/INT_MATRIX/TIMG0), then code.  Routes
    // TG0_WDT (source 52) -> CPU line 15 (level 3), enables the TIMG0 WDT
    // interrupt (INT_ENA bit 2), arms MWDT0 stage 0 with an interrupt action
    // (CONFIG0 stg0=1) and a short hold (CONFIG2 = 8), enables INTENABLE bit
    // 15, then spins until the handler has run once and stashes 0xCAFE.
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
    let p = a.l32r(2); // INT_MATRIX_BASE
    a.patch_l32r(p, IRAM_BASE + l_mat as u32);
    a.movi_n(3, 15);
    a.s32i(3, 2, 4 * 52); // INT_MATRIX_BASE + 4*52: TG0_WDT -> line 15
    let p = a.l32r(2); // TIMG0_BASE
    a.patch_l32r(p, IRAM_BASE + l_timg as u32);
    a.movi_n(3, 8);
    a.s32i(3, 2, 0x50); // WDT_CONFIG2 (hold0) = 8
    a.li(3, 0xA000_0000u32 as i32); // WDT_CONFIG0: EN | stg0=interrupt(1<<29)
    a.s32i(3, 2, 0x48);
    a.movi_n(3, 4);
    a.s32i(3, 2, 0x70); // INT_ENA bit 2 (WDT)
    a.li(3, 0x8000); // INTENABLE bit 15
    a.wsr(228, 3);
    a.rsil(4, 0);
    let loop_start = a.pc();
    let p = a.l32r(2); // CTR
    a.patch_l32r(p, IRAM_BASE + l_ctr as u32);
    a.l32i(3, 2, 0);
    a.addi(4, 3, -1);
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
    a.bytes_mut()[l_timg..l_timg + 4].copy_from_slice(&TIMG0_BASE.to_le_bytes());

    // Level-3 handler (a6-a9 only): CTR += 1, clear WDT INT (INT_CLR bit 2), rfi 3.
    let mut h = Asm::new(0x4000_01C0);
    h.li(6, CTR as i32);
    h.l32i(7, 6, 0);
    h.addi(7, 7, 1);
    h.s32i(7, 6, 0);
    h.li(8, TIMG0_BASE as i32);
    h.movi_n(9, 4);
    h.s32i(9, 8, 0x7C); // INT_CLR bit 2 (WDT)
    h.rfi(3);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_01C0, h.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..2000 {
        if m.cpu[0].pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after WDT interrupt");
    assert_eq!(
        m.soc.read32(CTR),
        1,
        "handler ran exactly once (no reset, no re-fire)"
    );
    assert_eq!(m.soc.read32(INT_MATRIX_BASE + 4 * 52), 15, "matrix write");
}

/// MWDT0 stage 0 = reset: when the watchdog times out the machine must reboot.
/// The app prints 'R' to the UART on every boot; a WDT reset re-runs the boot
/// sequence, so the UART stream accumulates multiple 'R's.  This is the
/// canonical "system reset on watchdog" edge case.
#[test]
fn wdt_reset_reboots_machine() {
    use crate::asm::Asm;
    use crate::rom_stub::APP_FLASH_OFFSET;

    // App (single IRAM segment): print 'R' (UART0 FIFO), arm MWDT0 stage 0 =
    // reset (CONFIG0 stg0=2) with a short hold (CONFIG2 = 4), then loop
    // forever.  Each WDT timeout triggers a machine reboot, re-printing 'R'.
    const APP_ENTRY: u32 = IRAM_BASE;
    let mut a = Asm::new(IRAM_BASE);
    // Print 'R' to UART0 (0x60000000) — printed once per boot.
    a.li(2, 0x6000_0000);
    a.movi_n(3, 0x52); // 'R'
    a.s32i(3, 2, 0); // UART0 FIFO <- 'R'
    // Arm MWDT0 (TIMG0_BASE) stage 0 = reset (CONFIG0 stg0=2) with a short
    // hold (CONFIG2 = 4), then loop forever; the timeout reboots.
    a.li(2, TIMG0_BASE as i32); // a2 = 0x6001F000
    a.movi_n(3, 4);
    a.s32i(3, 2, 0x50); // WDT_CONFIG2 (hold0) = 4
    a.li(3, 0xC000_0000u32 as i32); // WDT_CONFIG0: EN | stg0=reset(2<<29)
    a.s32i(3, 2, 0x48);
    let here = a.pc();
    a.j(here); // loop forever (WDT will reset)
    let app = a.bytes().to_vec();

    let img = esp_app_image(IRAM_BASE, APP_ENTRY, &app);
    let mut flash = std::vec![0xFFu8; 0x200_000];
    flash[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + img.len()].copy_from_slice(&img);

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    // Drain the UART each step and accumulate so a reboot's FIFO clear does
    // not drop the 'R' printed before it.
    let mut out = std::vec::Vec::new();
    for _ in 0..5000 {
        m.step();
        out.extend(m.take_uart_tx(0));
    }
    let r_count = out.iter().filter(|&&b| b == b'R').count();
    assert!(
        r_count >= 2,
        "WDT reset must reboot the machine (>=2 'R's), got {r_count} (out={out:?})"
    );
}

#[test]
fn bod_reset_reboots_machine() {
    use crate::asm::Asm;
    use crate::rom_stub::APP_FLASH_OFFSET;

    // App (single IRAM segment): print 'B' (UART0 FIFO), arm BOD reset
    // (BROWN_OUT ena + rst_ena, short waits), then loop forever.  Each BOD
    // timeout reboots the machine, re-printing 'B'.  The host injects the
    // low-voltage condition (nominal voltage never trips).
    const APP_ENTRY: u32 = IRAM_BASE;
    let mut a = Asm::new(IRAM_BASE);
    a.li(2, 0x6000_0000);
    a.movi_n(3, 0x42); // 'B'
    a.s32i(3, 2, 0); // UART0 FIFO <- 'B'
    a.li(2, 0x6000_8000); // RTC_CNTL page
    // BROWN_OUT = ena + rst_ena + int_wait=4 + rst_wait=4.
    a.li(3, 0x4444_0040u32 as i32);
    a.s32i(3, 2, 0xE8);
    let here = a.pc();
    a.j(here); // loop forever (BOD will reset)
    let app = a.bytes().to_vec();

    let img = esp_app_image(IRAM_BASE, APP_ENTRY, &app);
    let mut flash = std::vec![0xFFu8; 0x200_000];
    flash[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + img.len()].copy_from_slice(&img);

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    m.soc.bod_inject(true);
    let mut out = std::vec::Vec::new();
    for _ in 0..5000 {
        m.step();
        out.extend(m.take_uart_tx(0));
    }
    let b_count = out.iter().filter(|&&b| b == b'B').count();
    assert!(
        b_count >= 2,
        "BOD reset must reboot the machine (>=2 'B's), got {b_count} (out={out:?})"
    );
}

/// Cross-core interrupt (FROM_CPU_INTR1, source 80): core 0 writes
/// SYSTEM.CPU_INT_FROM_CPU_1 to assert an interrupt on core 1; the interrupt
/// matrix maps source 80 to core 1's line 15; core 1's level-3 ISR must run
/// and wake the idle core.  This is the FreeRTOS SMP yield path.
#[test]
fn cross_core_interrupt_yields_to_other_core() {
    use crate::asm::Asm;
    use crate::rom_stub::APP_FLASH_OFFSET;
    use esp32s3_soc::memmap::{INT_MATRIX_BASE, SYSTEM_BASE};

    const CORE1_CODE: u32 = IRAM_BASE + 0x200;
    const CTR: u32 = 0x3FC8_0300;
    const STASH: u32 = 0x3FC8_0304;

    // Core 0 (IRAM segment): release core 1 (APPCPU_CTRL_A), route source 80
    // (FROM_CPU_INTR1) -> core 1 line 15, assert the cross-core interrupt
    // (SYSTEM.CPU_INT_FROM_CPU_1 = 1), then loop.
    let mut a0 = Asm::new(IRAM_BASE);
    a0.li(6, CORE1_CODE as i32);
    a0.li(7, (SYSTEM_BASE + 4) as i32); // APPCPU_CTRL_A
    a0.s32i(6, 7, 0); // release core 1
    a0.li(6, (INT_MATRIX_BASE + 4 * (512 + 80)) as i32); // cpu1, src80
    a0.movi_n(7, 15);
    a0.s32i(7, 6, 0); // route FROM_CPU_INTR1 -> core1 line 15
    a0.li(6, (SYSTEM_BASE + 0x34) as i32); // CPU_INT_FROM_CPU_1
    a0.movi_n(7, 1);
    a0.s32i(7, 6, 0); // assert cross-core interrupt to core 1
    let here0 = a0.pc();
    a0.j(here0);
    let core0 = a0.bytes().to_vec();

    // Core 1 (segment at CORE1_CODE): enable INTENABLE bit 15 (level 3), then
    // idle on CTR.  When the cross-core ISR runs it bumps CTR, so the loop
    // exits and stashes 0xCAFE.
    let mut a1 = Asm::new(CORE1_CODE);
    a1.li(2, CTR as i32); // a2 = CTR addr
    a1.li(3, 0x8000); // INTENABLE bit 15
    a1.wsr(228, 3);
    a1.rsil(4, 0);
    let loop1 = a1.pc();
    a1.l32i(3, 2, 0); // a3 = CTR
    a1.addi(4, 3, -1); // a4 = CTR - 1
    a1.bnez(4, loop1); // while CTR < 1: spin
    a1.li(4, 0xCAFE);
    a1.li(5, STASH as i32);
    a1.s32i(4, 5, 0); // stash 0xCAFE once woken
    let done1 = a1.pc();
    a1.j(done1);
    let core1 = a1.bytes().to_vec();

    // Core-1 level-3 ISR (vector 0x400001C0): CTR += 1, clear the cross-core
    // register, rfi 3.
    let mut h = Asm::new(0x4000_01C0);
    h.li(6, CTR as i32);
    h.l32i(7, 6, 0);
    h.addi(7, 7, 1);
    h.s32i(7, 6, 0); // CTR += 1
    h.li(8, (SYSTEM_BASE + 0x34) as i32);
    h.movi_n(9, 0);
    h.s32i(9, 8, 0); // clear CPU_INT_FROM_CPU_1 (deassert)
    h.rfi(3);
    assert!(h.bytes().len() <= 0x40, "ISR fits the 64-byte vector slot");

    let img = esp_app_image_multi(IRAM_BASE, &[(IRAM_BASE, &core0), (CORE1_CODE, &core1)]);
    let mut flash = std::vec![0xFFu8; 0x200_000];
    flash[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + img.len()].copy_from_slice(&img);

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    m.load_image(0x4000_01C0, h.bytes()); // core-1 ISR
    for _ in 0..5000 {
        if m.soc.read32(STASH) != 0 {
            break;
        }
        m.step();
    }
    assert_eq!(
        m.soc.read32(STASH),
        0xCAFE,
        "core 1 woke via cross-core interrupt"
    );
    assert_eq!(
        m.soc.read32(CTR),
        1,
        "cross-core ISR ran exactly once (no re-fire)"
    );
    assert_eq!(m.cpu[1].pc, done1, "core 1 reached done after ISR");
}

#[test]
fn sdmmc_idmac_walks_descriptors() {
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 7);
    let sd = SDMMC_BASE;

    // Descriptor ring + data buffers in DRAM.
    let desc_w = 0x3FCE_0000; // write descriptor
    let buf_w = 0x3FCE_1000; // write data buffer
    let desc_r = 0x3FCE_2000; // read descriptor
    let buf_r = 0x3FCE_3000; // read data buffer

    // Fill the write buffer with a recognizable pattern (LE words).
    for i in 0..128u32 {
        m.soc.write32(buf_w + i * 4, i.wrapping_mul(0x0101_0101));
    }

    // Write descriptor: OWN|FS|LD, size 512, buffer = buf_w, next = 0.
    m.soc.write32(desc_w, (1 << 31) | (1 << 1) | (1 << 2));
    m.soc.write32(desc_w + 4, 512);
    m.soc.write32(desc_w + 8, buf_w);
    m.soc.write32(desc_w + 12, 0);

    // Enable IDMAC (BMOD.DE, bit 7) and point it at the descriptor.
    m.soc.write32(sd + IDMAC_CTRL, 1 << 7);
    m.soc.write32(sd + IDMAC_DBADDR, desc_w);
    m.soc.write32(sd + BLKSIZ, 512);
    m.soc.write32(sd + BYTCNT, 512);

    // Issue CMD24 (WRITE_BLOCK): data expected + RW (host->card).
    m.soc.write32(sd + CMDARG, 0);
    m.soc
        .write32(sd + CMD, 24 | (1 << 6) | (1 << 9) | (1 << 10) | (1 << 31));

    // The descriptor OWN bit is cleared (host now owns it) and DATA_OVER latched.
    assert_eq!(
        m.soc.read32(desc_w) & (1 << 31),
        0,
        "descriptor OWN cleared"
    );
    assert!(
        m.soc.read32(sd + RINTSTS) & (1 << 3) != 0,
        "DATA_OVER latched after IDMAC write"
    );

    // Read it back via IDMAC: descriptor points at buf_r.
    m.soc.write32(desc_r, (1 << 31) | (1 << 1) | (1 << 2));
    m.soc.write32(desc_r + 4, 512);
    m.soc.write32(desc_r + 8, buf_r);
    m.soc.write32(desc_r + 12, 0);
    m.soc.write32(sd + IDMAC_DBADDR, desc_r);
    m.soc.write32(sd + CMDARG, 0);
    m.soc
        .write32(sd + CMD, 17 | (1 << 6) | (1 << 9) | (1 << 31));

    // The read buffer must match the written pattern (card round-trips).
    let mut mismatch = 0;
    for i in 0..128u32 {
        if m.soc.read32(buf_r + i * 4) != i.wrapping_mul(0x0101_0101) {
            mismatch += 1;
        }
    }
    assert_eq!(mismatch, 0, "IDMAC read-back matches IDMAC write");
}

/// FATFS-visible bytes arrive intact through the IDMAC path: MBR (LBA 0)
/// carries the 0x55AA signature + FAT16 partition, and the volume boot
/// sector (LBA 64) carries the BPB signature + "FAT16" type + HELLO.TXT's
/// first cluster content at LBA 168.
#[test]
fn sdmmc_idmac_reads_fat_boot_sectors() {
    use esp32s3_soc::sdmmc::{
        BLKSIZ, BYTCNT, CMD, CMDARG, IDMAC_CTRL, IDMAC_DBADDR, RINTSTS, SDMMC_BASE,
    };

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 7);
    let sd = SDMMC_BASE;
    let desc = 0x3FCE_4000;
    let buf = 0x3FCE_5000;

    let read_lba = |m: &mut Esp32S3, lba: u32| {
        m.soc.write32(desc, (1 << 31) | (1 << 1) | (1 << 2));
        m.soc.write32(desc + 4, 512);
        m.soc.write32(desc + 8, buf);
        m.soc.write32(desc + 12, 0);
        m.soc.write32(sd + IDMAC_CTRL, 1 << 7);
        m.soc.write32(sd + IDMAC_DBADDR, desc);
        m.soc.write32(sd + BLKSIZ, 512);
        m.soc.write32(sd + BYTCNT, 512);
        m.soc.write32(sd + CMDARG, lba);
        m.soc
            .write32(sd + CMD, 17 | (1 << 6) | (1 << 9) | (1 << 31));
        assert!(
            m.soc.read32(sd + RINTSTS) & (1 << 3) != 0,
            "DATA_OVER for LBA {lba}"
        );
    };

    // MBR.
    read_lba(&mut m, 0);
    assert_eq!(m.soc.read8(buf + 0x1C2), 0x06, "partition type FAT16");
    assert_eq!(m.soc.read8(buf + 510), 0x55, "MBR signature lo");
    assert_eq!(m.soc.read8(buf + 511), 0xAA, "MBR signature hi");
    // Boot sector.
    read_lba(&mut m, 64);
    assert_eq!(m.soc.read8(buf), 0xEB, "jump boot");
    assert_eq!(m.soc.read8(buf + 510), 0x55, "BPB signature lo");
    assert_eq!(m.soc.read8(buf + 511), 0xAA, "BPB signature hi");
    for (i, b) in b"FAT16   ".iter().enumerate() {
        assert_eq!(m.soc.read8(buf + 54 + i as u32), *b as u32, "fs type");
    }
    // HELLO.TXT data cluster.
    read_lba(&mut m, 168);
    for (i, b) in b"Hello from SDMMC!\n".iter().enumerate() {
        assert_eq!(m.soc.read8(buf + i as u32), *b as u32, "file byte {i}");
    }
}

/// GPIO output edges are reported to the host via `drain_events` as
/// `EVT_GPIO` events (Wokwi-style pin observers).
#[test]
fn gpio_edge_emits_event() {
    let mut m = Esp32S3::default();
    m.soc.write32(GPIO_BASE + GPIO_ENABLE_W1TS, 1 << 2);
    m.soc.write32(GPIO_BASE + GPIO_OUT_W1TS, 1 << 2);
    m.soc.tick_timers(1);
    let evs = m.soc.drain_events();
    assert!(
        evs.iter()
            .any(|e| e.kind == EVT_GPIO && e.a == 2 && e.b == 1),
        "rising edge on pin 2"
    );
    m.soc.write32(GPIO_BASE + GPIO_OUT_W1TC, 1 << 2);
    m.soc.tick_timers(1);
    let evs = m.soc.drain_events();
    assert!(
        evs.iter()
            .any(|e| e.kind == EVT_GPIO && e.a == 2 && e.b == 0),
        "falling edge on pin 2"
    );
}

/// Unimplemented DSP/TIE extensions trap instead of hanging: executing a
/// `format_32` word with no execution model returns
/// `StepResult::Unimplemented` with the pc frozen (the fast-block runner
/// aborts the block the same way). Dynamic audit over 24 arduino-cli
/// sketches x 96M instructions each (2026-09-03) shows zero executions,
/// so the trap path is firmware-invisible today — this test pins it.
/// (0xEEEEEEEE: low nibble 0xE = 4-byte format; decode lands on an
/// `OPCODE_EE_*` with no executor, or the EE catch-all.)
#[test]
fn ee_extension_traps_unimplemented() {
    use xtensa_core::StepResult;
    let mut m = Esp32S3::new();
    let addr = 0x4000_1000u32;
    m.load_image(addr, &0xEEEE_EEEEu32.to_le_bytes());
    m.cpu[0].pc = addr;
    let r = m.cpu[0].step(&mut m.soc);
    assert!(
        matches!(r, StepResult::Unimplemented(_)),
        "ee word traps, got {r:?}"
    );
    assert_eq!(m.cpu[0].pc, addr, "pc frozen on unimplemented trap");
    // The block runner surfaces the same trap promptly (no silent hang).
    m.cpu[0].pc = addr;
    let (br, _, n) = m.step_fast();
    assert!(
        matches!(br, StepResult::Unimplemented(_)),
        "fast block traps, got {br:?}"
    );
    assert!(n >= 1, "trap counts the attempted op");
}

/// RMT waveform edge fires a GPIO interrupt through the full chain: RMT TX
/// channel 0 drives GPIO2 (matrix signal 81), GPIO2's PIN interrupt
/// (RISING, enabled) latches STATUS, source 16 (ETS_GPIO_INTR_SOURCE) routes
/// to CPU line 15, and the level-3 handler counts and clears via STATUS_W1TC.
#[test]
fn gpio_rmt_edge_fires_gpio_isr() {
    use crate::asm::Asm;
    use esp32s3_soc::memmap::{GPIO_BASE, INT_MATRIX_BASE, IRAM_BASE};
    use esp32s3_soc::rmt::{RMT_BASE, RMTMEM_BASE};

    const CTR: u32 = 0x3FC8_0300;
    const STASH: u32 = 0x3FC8_0304;
    let mut a = Asm::new(IRAM_BASE);
    let l_ctr = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let l_mat = a.offset();
    a.lit(0);
    let l_gpio = a.offset();
    a.lit(0);
    let l_gfunc = a.offset();
    a.lit(0);
    let l_pin2 = a.offset();
    a.lit(0);
    let l_rmt = a.offset();
    a.lit(0);
    let l_rmtmem = a.offset();
    a.lit(0);
    let code_start = a.pc();
    // GPIO2 <- RMT TX signal 81, output enabled.
    let p = a.l32r(5);
    a.patch_l32r(p, IRAM_BASE + l_gfunc as u32);
    a.movi_n(4, 81);
    a.s32i(4, 5, 0); // FUNC_OUT_SEL_CFG[2] = 81
    let p = a.l32r(3);
    a.patch_l32r(p, IRAM_BASE + l_gpio as u32);
    a.movi_n(4, 1 << 2);
    a.s32i(4, 3, 0x24); // GPIO_ENABLE_W1TS bit 2
    // RMT items: HIGH 2000 / LOW 2000, HIGH 1000 / LOW 1000. Pulses are
    // deliberately long: the RMT FSM advances 32 duration units per step,
    // so sub-100-unit pulses would alias past the per-step GPIO sampler.
    let p = a.l32r(5);
    a.patch_l32r(p, IRAM_BASE + l_rmtmem as u32);
    a.li(4, 2000 | (1 << 15) | (2000 << 16));
    a.s32i(4, 5, 0);
    a.li(4, 1000 | (1 << 15) | (1000 << 16));
    a.s32i(4, 5, 4);
    let p = a.l32r(5);
    a.patch_l32r(p, IRAM_BASE + l_rmt as u32);
    a.movi_n(4, (1 << 0) | (1 << 6));
    a.s32i(4, 5, 0x20); // chnconf0: tx_start | idle_out_en
    // GPIO2 PIN interrupt: RISING + enable bit 0.
    let p = a.l32r(5);
    a.patch_l32r(p, IRAM_BASE + l_pin2 as u32);
    a.li(4, (1 << 7) | (1 << 13));
    a.s32i(4, 5, 0);
    // Matrix: source 16 (GPIO) -> CPU line 15; INTENABLE; rsil 0.
    let p = a.l32r(2);
    a.patch_l32r(p, IRAM_BASE + l_mat as u32);
    a.movi_n(3, 15);
    a.s32i(3, 2, 4 * 16);
    a.li(3, 0x8000);
    a.wsr(228, 3);
    a.rsil(4, 0);
    // Spin until the handler has counted 2 edges, then stash 0xCAFE.
    let loop_start = a.pc();
    let p = a.l32r(2);
    a.patch_l32r(p, IRAM_BASE + l_ctr as u32);
    a.l32i(3, 2, 0);
    a.addi(4, 3, -2);
    a.bnez(4, loop_start);
    a.li(4, 0xCAFE);
    let p = a.l32r(5);
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    // Patch literals.
    a.bytes_mut()[l_ctr..l_ctr + 4].copy_from_slice(&CTR.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());
    a.bytes_mut()[l_mat..l_mat + 4].copy_from_slice(&INT_MATRIX_BASE.to_le_bytes());
    a.bytes_mut()[l_gpio..l_gpio + 4].copy_from_slice(&GPIO_BASE.to_le_bytes());
    a.bytes_mut()[l_gfunc..l_gfunc + 4].copy_from_slice(&(GPIO_BASE + 0x55C).to_le_bytes());
    a.bytes_mut()[l_pin2..l_pin2 + 4].copy_from_slice(&(GPIO_BASE + 0x7C).to_le_bytes());
    a.bytes_mut()[l_rmt..l_rmt + 4].copy_from_slice(&RMT_BASE.to_le_bytes());
    a.bytes_mut()[l_rmtmem..l_rmtmem + 4].copy_from_slice(&RMTMEM_BASE.to_le_bytes());

    // Level-3 handler at VECBASE + 0x1C0: CTR += 1, clear GPIO STATUS bit 2.
    let mut h = Asm::new(0x4000_01C0);
    h.li(6, CTR as i32);
    h.l32i(7, 6, 0);
    h.addi(7, 7, 1);
    h.s32i(7, 6, 0); // CTR += 1
    h.li(8, GPIO_BASE as i32);
    h.movi_n(9, 1 << 2);
    h.s32i(9, 8, 0x4C); // GPIO_STATUS_W1TC bit 2
    h.rfi(3);
    assert!(
        h.bytes().len() <= 0x40,
        "handler fits the 64-byte vector slot"
    );

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 9);
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_01C0, h.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..60000 {
        if m.cpu[0].pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after 2 GPIO interrupts");
    assert_eq!(m.soc.read32(CTR), 2, "GPIO handler ran twice");
    assert_eq!(m.soc.read32(INT_MATRIX_BASE + 4 * 16), 15, "matrix write");
}

/// RMT TX loopback into RMT RX through a GPIO pad: TX channel 0 drives
/// GPIO2 (matrix signal 81) while RX channel 4 samples matrix input 81
/// (routed from the same pad). The received items must match the
/// transmitted waveform (within the 32-tick sample quantum) and the rx_end
/// interrupt must latch.
#[test]
fn rmt_tx_loopback_into_rx_channel() {
    use esp32s3_soc::gpio::GPIO_FUNC_IN_SEL_0;
    use esp32s3_soc::memmap::GPIO_BASE;
    use esp32s3_soc::rmt::{RMT_BASE, RMTMEM_BASE};

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 9);
    // GPIO2 <- RMT TX ch0 (signal 81), output enabled.
    m.soc.write32(GPIO_BASE + 0x554 + 2 * 4, 81);
    m.soc.write32(GPIO_BASE + 0x20, 1 << 2);
    // RMT RX input 81 <- GPIO2 pad.
    m.soc.write32(GPIO_BASE + GPIO_FUNC_IN_SEL_0 + 81 * 4, 2);
    // TX items: HIGH 100 / LOW 100, HIGH 50 / LOW 50.
    m.soc.write32(RMTMEM_BASE, 100 | (1u32 << 15) | (100 << 16));
    m.soc
        .write32(RMTMEM_BASE + 4, 50 | (1u32 << 15) | (50 << 16));
    // Enable RX channel 4 first (samples the idle-low pad as baseline).
    m.soc.write32(RMT_BASE + 0x34, 1); // chmconf1[0]: rx_en
    // Let the receiver baseline a few steps on the idle pad before the
    // transmitter starts (back-to-back setup would race TX's synchronous
    // first-pulse level into the baseline, as on silicon).
    for _ in 0..5 {
        m.step();
    }
    // Start TX (default idle_thres/mem_size/rx_lim on the RX side).
    m.soc.write32(RMT_BASE + 0x20, (1 << 0) | (1 << 6));
    for _ in 0..4000 {
        m.step();
        if m.soc.read32(RMT_BASE + 0x70) & (1 << 16) != 0 {
            break;
        }
    }
    assert_ne!(
        m.soc.read32(RMT_BASE + 0x70) & (1 << 16),
        0,
        "rx_end latched after the looped-back transmission"
    );
    // Received block (HW ch 4 @ RMTMEM + 0x400): first item holds the
    // baseline-LOW remnant + the transmitted HIGH-100; levels exact,
    // widths within two sample quanta.
    let r0 = m.soc.read32(RMTMEM_BASE + 0x400);
    assert_eq!((r0 >> 15) & 1, 0, "item0 first half low (baseline)");
    assert_eq!((r0 >> 31) & 1, 1, "item0 second half high (TX pulse)");
    assert!((r0 & 0x7FFF) <= 256, "baseline remnant is short");
    let high = (r0 >> 16) & 0x7FFF;
    assert!(
        (36..=200).contains(&high),
        "captured HIGH duration, got {high}"
    );
    let r1 = m.soc.read32(RMTMEM_BASE + 0x404);
    assert_eq!((r1 >> 15) & 1, 0, "item1 first half low");
    assert_eq!((r1 >> 31) & 1, 1, "item1 second half high");
    // INT_CLR clears the RX end flag.
    m.soc.write32(RMT_BASE + 0x7C, 1 << 16);
    assert_eq!(m.soc.read32(RMT_BASE + 0x70) & (1 << 16), 0);
}

#[test]
fn rom_memcpy_matrix_lengths_and_alignments() {
    // ROM newlib memcpy (0x40056F44) over length x src/dst misalignment:
    // MicroPython's qstr interning (and heap churn generally) copies tiny
    // strings to chunk addresses whose alignment varies with content, so a
    // length/alignment-sensitive memcpy bug would manifest as
    // content-dependent corruption.  Each combo gets a fresh DST slot;
    // SRC holds a fixed pseudo-random pattern; sentinels guard both.
    use crate::asm::Asm;
    use crate::rom_stub;
    const CODE: u32 = IRAM_BASE + 0x9000;
    const SRC: u32 = 0x3FC8_1000;
    // DST must not alias the test code itself: CODE lives at IRAM
    // 0x40379000+ (= sram 0x9000+, i.e. DRAM 0x3FC89000+), so DST slots
    // (320 x 128 B) go to 0x3FCA0000, clear of code/SRC/stack.
    const DST: u32 = 0x3FCA_0000;
    const MEMCPY: u32 = 0x4005_6F44;
    const LENS: [u32; 20] = [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15, 16, 17, 24, 31, 32, 33, 48, 64,
    ];
    let mut combos = alloc::vec::Vec::new();
    for &len in &LENS {
        for sm in 0..4u32 {
            for dm in 0..4u32 {
                combos.push((len, sm, dm));
            }
        }
    }
    let mut a = Asm::new(CODE);
    a.li(1, 0x3FC8_9000); // SP
    a.li(3, 0x40000); // PS.WOE
    a.wsr(xtensa_core::cpu::SR_PS, 3);
    for (i, &(len, sm, dm)) in combos.iter().enumerate() {
        let dst = DST + i as u32 * 128;
        a.li(10, (dst + dm) as i32); // callee a2 = dst
        a.li(11, (SRC + sm) as i32); // callee a3 = src
        a.li(12, len as i32); // callee a4 = len
        a.li(8, MEMCPY as i32);
        a.callx8(8);
    }
    let halt = a.pc();
    a.j(halt);
    let mut m = Esp32S3::new();
    let rom = rom_stub::rom_image();
    m.load_image(rom_stub::ROM_BASE, &rom);
    m.load_rom_data();
    m.load_image(CODE, a.bytes());
    // SRC pattern + DST sentinels, host-side (pattern is a pure
    // function of the absolute source address, mirrored in the assert).
    let pat = |a: u32| (a.wrapping_mul(0x9E3779B9).wrapping_add(a >> 3) & 0xFF) as u8;
    for i in 0..256u32 {
        m.soc.write8(SRC + i, pat(SRC + i) as u32);
    }
    for i in 0..(combos.len() as u32 * 128) {
        m.soc.write8(DST + i, 0xCC);
    }
    m.cpu[0].pc = CODE;
    for _ in 0..2_000_000 {
        if m.cpu[0].pc == halt {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, halt, "memcpy matrix must halt cleanly");
    for (i, &(len, sm, dm)) in combos.iter().enumerate() {
        let dst = DST + i as u32 * 128;
        for k in 0..len {
            let want = pat(SRC + sm + k);
            let got = m.soc.read8(dst + dm + k) as u8;
            assert_eq!(
                got, want,
                "combo {i} len={len} src_mis={sm} dst_mis={dm} byte {k}"
            );
        }
        // Sentinels around the written span must be intact.
        if dm > 0 {
            assert_eq!(
                m.soc.read8(dst) as u8,
                0xCC,
                "combo {i} leading sentinel intact"
            );
        }
        let tail = dst + dm + len;
        let slot_end = dst + 128;
        if tail < slot_end {
            assert_eq!(
                m.soc.read8(tail) as u8,
                0xCC,
                "combo {i} trailing sentinel intact"
            );
        }
    }
}

#[test]
fn rom_memset_matrix_lengths_and_alignments() {
    // ROM newlib memset (0x400570C8) over length x misalignment, same
    // rationale as the memcpy matrix above (heap/pool zeroing paths).
    // Layout keeps code/stack/data disjoint (see memcpy test).
    use crate::asm::Asm;
    use crate::rom_stub;
    const CODE: u32 = IRAM_BASE + 0x9000;
    const DST: u32 = 0x3FCA_0000;
    const MEMSET: u32 = 0x4005_70C8;
    const LENS: [u32; 12] = [0, 1, 2, 3, 4, 5, 7, 8, 15, 16, 17, 33];
    let mut combos = alloc::vec::Vec::new();
    for &len in &LENS {
        for dm in 0..4u32 {
            combos.push((len, dm));
        }
    }
    let mut a = Asm::new(CODE);
    a.li(1, 0x3FCB_0000); // SP (clear of code/data)
    a.li(3, 0x40000); // PS.WOE
    a.wsr(xtensa_core::cpu::SR_PS, 3);
    for (i, &(len, dm)) in combos.iter().enumerate() {
        let dst = DST + i as u32 * 64;
        a.li(10, (dst + dm) as i32); // callee a2 = dst
        a.li(11, 0x5A); // callee a3 = value
        a.li(12, len as i32); // callee a4 = len
        a.li(8, MEMSET as i32);
        a.callx8(8);
    }
    let halt = a.pc();
    a.j(halt);
    let mut m = Esp32S3::new();
    let rom = rom_stub::rom_image();
    m.load_image(rom_stub::ROM_BASE, &rom);
    m.load_rom_data();
    m.load_image(CODE, a.bytes());
    for i in 0..(combos.len() as u32 * 64) {
        m.soc.write8(DST + i, 0xCC);
    }
    m.cpu[0].pc = CODE;
    for _ in 0..2_000_000 {
        if m.cpu[0].pc == halt {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, halt, "memset matrix must halt cleanly");
    for (i, &(len, dm)) in combos.iter().enumerate() {
        let dst = DST + i as u32 * 64;
        for k in 0..len {
            assert_eq!(
                m.soc.read8(dst + dm + k) as u8,
                0x5A,
                "combo {i} len={len} mis={dm} byte {k}"
            );
        }
        if dm > 0 {
            assert_eq!(m.soc.read8(dst) as u8, 0xCC, "combo {i} leading sentinel");
        }
        let tail = dst + dm + len;
        if tail < dst + 64 {
            assert_eq!(m.soc.read8(tail) as u8, 0xCC, "combo {i} trailing sentinel");
        }
    }
}

#[test]
fn psram_write16_preserves_adjacent_halfword() {
    // Regression: Soc::write16 widened cache-window stores to write32,
    // zero-clobbering the neighbor halfword on MMU-mapped PSRAM pages.
    // That broke MicroPython's u16 qstr table (written in hash order, so a
    // later pair-store cleared already-written entries -> NameError '').
    // Firmware maps vpage 1 -> PSRAM page 1, writes 6 halfwords in the
    // non-monotonic order the MP compiler used, and stashes all six.
    use crate::asm::Asm;
    use esp32s3_soc::memmap::{CACHE_PAGE_SIZE, FLASH_DATA_BASE, MMU_TABLE_BASE};
    const STASH: u32 = 0x3FC8_0200;
    let mut a = Asm::new(IRAM_BASE);
    let l_tab = a.offset();
    a.lit(0);
    let l_win = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // MMU_TABLE_BASE
    a.patch_l32r(p, IRAM_BASE + l_tab as u32);
    let p = a.l32r(3); // FLASH_DATA_BASE + 1 * CACHE_PAGE_SIZE
    a.patch_l32r(p, IRAM_BASE + l_win as u32);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.li(6, 0x8001); // PSRAM page 1 entry
    a.s32i(6, 2, 4); // mmu[1]
    // Non-monotonic u16 writes: +8, +2, +4, +10, +0, +6 (MP emit order).
    a.li(6, 0x007B);
    a.s16i(6, 3, 8);
    a.li(6, 0x0007);
    a.s16i(6, 3, 2);
    a.li(6, 0x067F);
    a.s16i(6, 3, 4);
    a.li(6, 0x067B);
    a.s16i(6, 3, 10);
    a.li(6, 0x0586);
    a.s16i(6, 3, 0);
    a.li(6, 0x0680);
    a.s16i(6, 3, 6);
    a.l16ui(6, 3, 0);
    a.s32i(6, 5, 0);
    a.l16ui(6, 3, 2);
    a.s32i(6, 5, 4);
    a.l16ui(6, 3, 4);
    a.s32i(6, 5, 8);
    a.l16ui(6, 3, 6);
    a.s32i(6, 5, 12);
    a.l16ui(6, 3, 8);
    a.s32i(6, 5, 16);
    a.l16ui(6, 3, 10);
    a.s32i(6, 5, 20);
    let halt = a.pc();
    a.j(halt);
    a.bytes_mut()[l_tab..l_tab + 4].copy_from_slice(&MMU_TABLE_BASE.to_le_bytes());
    a.bytes_mut()[l_win..l_win + 4]
        .copy_from_slice(&(FLASH_DATA_BASE + CACHE_PAGE_SIZE).to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());
    let mut m = Esp32S3::new();
    m.load_image(IRAM_BASE, a.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..2000 {
        if m.cpu[0].pc == halt {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, halt, "must halt");
    let want = [0x0586u32, 0x0007, 0x067F, 0x0680, 0x007B, 0x067B];
    for (i, &w) in want.iter().enumerate() {
        assert_eq!(m.soc.read32(STASH + i as u32 * 4), w, "halfword {i}");
    }
}
/// I2S GDMA streaming pump advances across an IN descriptor chain (EOF is
/// an event, not a stop): two 8-byte descs fill from the loopback-fed RX
/// FIFO in order.
#[test]
fn i2s_in_pump_advances_desc_chain() {
    use esp32s3_soc::gdma::{GDMA_BASE, GDMA_I2S0_PERIPH};
    use esp32s3_soc::memmap::I2S0_BASE;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, false, 4);
    // Two IN descs: 8 bytes then 8 bytes (4 words total), eof on second.
    let d0 = 0x3FC8_3000u32;
    let d1 = 0x3FC8_3100u32;
    let b0 = 0x3FC8_4000u32;
    let b1 = 0x3FC8_4100u32;
    m.soc.write32(d0, (1u32 << 31) | (8u32 << 12));
    m.soc.write32(d0 + 4, b0);
    m.soc.write32(d0 + 8, d1);
    m.soc
        .write32(d1, (1u32 << 31) | (1u32 << 30) | (8u32 << 12));
    m.soc.write32(d1 + 4, b1);
    m.soc.write32(d1 + 8, 0);
    m.soc.write32(GDMA_BASE + 0x48, GDMA_I2S0_PERIPH); // in_peri_sel[0]
    m.soc
        .write32(GDMA_BASE + 0x20, (d0 & 0x000F_FFFF) | (1 << 22)); // in_link start
    // Inject 4 words via loopback-style: push TX words with loopback on.
    m.soc.write32(I2S0_BASE + 0x34, 1); // M = 1
    m.soc.write32(I2S0_BASE + 0x3C, 1); // N = 1
    for w in [0x11111111u32, 0x22222222, 0x33333333, 0x44444444] {
        m.soc.write32(I2S0_BASE + 0x80, w);
    }
    m.soc.write32(I2S0_BASE + 0x24, (1 << 27) | (1 << 2)); // loopback + start
    m.soc.tick_timers(600);
    assert_eq!(m.soc.read32(b0), 0x11111111, "chain word0");
    assert_eq!(m.soc.read32(b0 + 4), 0x22222222, "chain word1");
    assert_eq!(m.soc.read32(b1), 0x33333333, "chain word2");
    assert_eq!(m.soc.read32(b1 + 4), 0x44444444, "chain word3");
}

#[test]
fn gdma_inlink_reset_reads_zero() {
    use esp32s3_soc::gdma::GDMA_BASE;
    let mut m = Esp32S3::new();
    assert_eq!(m.soc.read32(GDMA_BASE + 0x20), 0, "IN0 link reset");
    assert_eq!(m.soc.read32(0x6000_F020), 0, "I2S0 RX_CONF reset");
}

#[test]
fn i2s_out_pump_large_chain() {
    use esp32s3_soc::gdma::{GDMA_BASE, GDMA_I2S0_PERIPH};
    use esp32s3_soc::memmap::I2S0_BASE;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, false, 4);
    // OUT chain across uneven descriptors: 960 + 8 bytes, eof on second.
    // EOF raises the per-descriptor event but does not stop the walk.
    let d0 = 0x3FC8_5000u32;
    let d1 = 0x3FC8_5100u32;
    let b0 = 0x3FC8_6000u32;
    let b1 = 0x3FC8_7000u32;
    for i in 0..240u32 {
        m.soc.write32(b0 + 4 * i, 0x1000_0000 + i);
    }
    m.soc.write32(b1, 0x5555AAAA);
    m.soc.write32(b1 + 4, 0x5555BBBB);
    m.soc.write32(d0, (1u32 << 31) | (960u32 << 12));
    m.soc.write32(d0 + 4, b0);
    m.soc.write32(d0 + 8, d1);
    m.soc
        .write32(d1, (1u32 << 31) | (1u32 << 30) | (8u32 << 12));
    m.soc.write32(d1 + 4, b1);
    m.soc.write32(d1 + 8, 0);
    m.soc.write32(GDMA_BASE + 0xA8, GDMA_I2S0_PERIPH);
    m.soc
        .write32(GDMA_BASE + 0x80, (d0 & 0x000F_FFFF) | (1 << 21));
    m.soc.write32(I2S0_BASE + 0x34, 1);
    m.soc.write32(I2S0_BASE + 0x3C, 1);
    m.soc.write32(I2S0_BASE + 0x24, (1 << 27) | (1 << 2));
    m.soc.tick_timers(9000);
    assert_eq!(m.soc.read32(0x6000_F00C) & 2, 2, "tx_done after chain");
}

/// Touch pad counter flows from host injection through the SENS-page
/// dispatch to the STATUS register the driver reads.
#[test]
fn touch_status_reports_injected_counter() {
    use esp32s3_soc::touch::{TOUCH_CHN_ST_OFF, TOUCH_CONF_OFF};
    const SENS: u32 = 0x6000_8800;
    let mut m = Esp32S3::new();
    // Plain CONF round-trip through the carved touch window.
    m.soc.write32(SENS + TOUCH_CONF_OFF, 0x7FFF);
    assert_eq!(m.soc.read32(SENS + TOUCH_CONF_OFF), 0x7FFF);
    // Injected counter visible in STATUS3 (pad 3); meas_done set.
    m.soc.touch_inject(3, 1877);
    assert_eq!(m.soc.read32(SENS + 0xAC), 1877);
    assert_eq!(m.soc.read32(SENS + TOUCH_CHN_ST_OFF) & (1 << 31), 1 << 31);
}

#[test]
fn uart_flow_control_crosstalk_via_gpio_matrix() {
    // UART1 TX flow control driven by GPIO2 (U1CTS_IN = 16), RTS observed
    // on GPIO3 (U1RTS_OUT = 16). Hand-assembled firmware would work, but
    // MMIO pokes exercise the identical bus path with less encoding risk.
    use esp32s3_soc::memmap::{GPIO_BASE, UART1_BASE};
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 5); // UART1
    // GPIO2 output high (CTS stop); GPIO3 output-enabled for RTS readback
    // (peripheral-driven pins read back the driven signal via GPIO_IN).
    m.soc.write32(GPIO_BASE + 0x24, 1 << 2); // ENABLE_W1TS pin 2
    m.soc.write32(GPIO_BASE + 0x08, 1 << 2); // OUT_W1TS pin 2 -> high
    m.soc.write32(GPIO_BASE + 0x24, 1 << 3); // ENABLE_W1TS pin 3
    m.soc.write32(GPIO_BASE + 0x554 + 3 * 4, 16); // FUNC_OUT_SEL[3] = U1RTS
    m.soc.write32(GPIO_BASE + 0x154 + 16 * 4, 2); // FUNC_IN_SEL[16] = GPIO2
    // TX flow on: FIFO byte held (CTS high), console sees nothing.
    m.soc.write32(UART1_BASE + 0x20, 1 << 15); // CONF0 TX_FLOW_EN
    m.soc.write32(UART1_BASE, 0x41); // FIFO 'A'
    for _ in 0..10 {
        m.step();
    }
    assert!(m.soc.take_uart_tx(1).is_empty(), "held while CTS high");
    // Drop CTS (GPIO2 low): flush on the next steps.
    m.soc.write32(GPIO_BASE + 0x0C, 1 << 2); // OUT_W1TC pin 2
    for _ in 0..10 {
        m.step();
    }
    assert_eq!(
        m.soc.take_uart_tx(1),
        alloc::vec![0x41],
        "flushed on CTS go"
    );
    // RX flow on with threshold 1: RTS (GPIO3) ready-low while RX empty.
    m.soc.write32(UART1_BASE + 0x20, (1 << 15) | (1 << 22)); // TX+RX flow
    m.soc.write32(0x6001_0000 + 0x60, 1 << 7); // MEM_CONF RX_FLOW_THRHD=1
    for _ in 0..10 {
        m.step();
    }
    assert_eq!(m.soc.gpio_output() & (1 << 3), 0, "RTS ready while empty");
    // Fill RX to threshold: RTS stops (high).
    m.soc.uart_inject_rx(1, 0x5A);
    for _ in 0..10 {
        m.step();
    }
    assert_ne!(m.soc.gpio_output() & (1 << 3), 0, "RTS stop at threshold");
}

/// Inject a peer CAN frame into TWAI in normal (non-loopback) mode: the
/// virtual-second-node path fills the RX buffer subject to the acceptance
/// filter, and RRB releases it — exactly like a bus reception.
#[test]
fn twai_inject_rx_delivers_peer_frame_in_normal_mode() {
    use esp32s3_soc::twai::{FRAME_LEN, TWAI_BASE};

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 19);
    let b = TWAI_BASE;

    // Reset mode for the filter, accept-all, then normal mode (stm = 0).
    m.soc.write32(b, 1);
    for off in [0x40u32, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C] {
        m.soc
            .write32(b + off, if off < 0x50 { 0 } else { 0xFFFF_FFFF });
    }
    m.soc.write32(b, 0);

    let mut frame = [0u8; FRAME_LEN];
    frame[0] = 0x08;
    frame[1] = 0x24;
    frame[2] = 0x60;
    for (i, v) in (0x13u8..0x13 + 10).enumerate() {
        frame[3 + i] = v;
    }
    m.soc.twai_inject_rx(frame);
    assert_eq!(m.soc.read32(b + 0x08) & 1, 1, "rbs set by injected frame");
    for (i, &v) in frame.iter().enumerate() {
        assert_eq!(
            m.soc.read32(b + 0x40 + (i as u32) * 4),
            v as u32,
            "RX byte {i} differs"
        );
    }
    m.soc.write32(b + 0x04, 1 << 2); // RRB
    assert_eq!(m.soc.read32(b + 0x08) & 1, 0, "RRB did not clear rbs");
}

/// Deep-sleep digital-pad hold (RTC_CNTL_DIG_PAD_HOLD @ +0xDC): a held pad
/// keeps driving across the wake reboot; an unheld pad resets with the
/// digital core.
#[test]
fn deep_sleep_gpio_hold_preserves_driven_pins() {
    use esp32s3_soc::memmap::GPIO_BASE;
    use esp32s3_soc::rtc::{
        RTC_CNTL_BASE, SLEEP_EN_BIT, SLP_TIMER0_OFF, SLP_TIMER1_OFF, SLP_WAKEUP_CAUSE_OFF,
        STATE0_OFF,
    };
    const SLP_TIMER0: u32 = RTC_CNTL_BASE + SLP_TIMER0_OFF;
    const SLP_TIMER1: u32 = RTC_CNTL_BASE + SLP_TIMER1_OFF;
    const STATE0: u32 = RTC_CNTL_BASE + STATE0_OFF;
    const WAKEUP_CAUSE: u32 = RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF;
    const PAD_HOLD: u32 = RTC_CNTL_BASE + 0xDC;

    for held in [true, false] {
        let mut m = Esp32S3::new();
        // GPIO2 output high.
        m.soc.write32(GPIO_BASE + 0x24, 1 << 2);
        m.soc.write32(GPIO_BASE + 0x08, 1 << 2);
        // Deep sleep with a short timer.
        m.soc.write32(SLP_TIMER0, 0x100);
        m.soc.write32(SLP_TIMER1, 0);
        m.soc.write32(RTC_CNTL_BASE + 0x90, 1 << 31); // DIG_PWC deep
        if held {
            m.soc.write32(PAD_HOLD, 1 << 2);
        }
        let prev = m.soc.read32(STATE0);
        m.soc.write32(STATE0, prev | SLEEP_EN_BIT);
        let mut woke = false;
        for _ in 0..200_000 {
            m.step();
            if m.soc.read32(WAKEUP_CAUSE) & (1 << 3) != 0 {
                woke = true;
                break;
            }
        }
        assert!(woke, "held={held}: machine did not wake");
        assert_eq!(
            m.soc.gpio_output() & (1 << 2),
            if held { 1 << 2 } else { 0 },
            "held={held}: GPIO2 after wake"
        );
    }
}

/// USB-Serial-JTAG TX hold while the USB_DEVICE clock (EN1 bit 10) is off:
/// EP1 bytes stage invisibly and flush to the console on clock return.
#[test]
fn usb_serial_tx_holds_while_clock_gated() {
    const USB: u32 = 0x6003_8000;
    const EN1: u32 = 0x600C_001C;
    let mut m = Esp32S3::new();
    // Gate the USB clock off (RMW keeps the other default enables).
    let en1 = m.soc.read32(EN1);
    m.soc.write32(EN1, en1 & !(1 << 10));
    m.soc.write32(USB, 0x48); // EP1 'H'
    m.soc.write32(USB, 0x69); // EP1 'i'
    for _ in 0..10 {
        m.step();
    }
    assert!(m.soc.take_usb_serial_tx().is_empty(), "held while gated");
    // Clock back on: the tick flushes the endpoint FIFO to the console.
    m.soc.write32(EN1, en1 | (1 << 10));
    for _ in 0..10 {
        m.step();
    }
    assert_eq!(m.soc.take_usb_serial_tx(), alloc::vec![0x48, 0x69]);
}

/// Secure-boot fail-closed gate: with eFuse SECURE_BOOT_EN burned (BLK0
/// word 5 = REPEAT_DATA4 bit 20), `boot_from_flash` refuses the boot and
/// parks both CPUs instead of insecurely booting an unverified image.
/// Default eFuse boots normally.
#[test]
fn secure_boot_enabled_denies_boot_and_parks_cpus() {
    const EFUSE_BASE: u32 = 0x6000_7000;
    let mut m = Esp32S3::new();
    // Burn SECURE_BOOT_EN through the real PGM path (staged words +
    // PGM_CMD BLK_NUM 0).
    for (i, w) in [0u32, 0, 0, 0, 0, 1 << 20, 0, 0].iter().enumerate() {
        m.soc.write32(EFUSE_BASE + (i as u32) * 4, *w);
    }
    m.soc.write32(EFUSE_BASE + 0x1D4, 0x2); // PGM bit, BLK_NUM 0
    assert!(m.soc.secure_boot_enabled(), "SECURE_BOOT_EN burned");
    m.boot_from_flash(&[0xFF; 0x1000]);
    assert!(m.secure_boot_rejected(), "boot refused");
    let pc0 = m.cpu[0].pc;
    for _ in 0..100 {
        m.step();
    }
    assert_eq!(m.cpu[0].pc, pc0, "CPUs parked");
    assert!(m.soc.take_uart_tx(0).is_empty(), "no output when denied");
}

/// Light-sleep GPIO wakeup (RTCIO PINn WAKEUP_ENABLE + level): a held-high
/// RTC pin with a high-level wakeup armed resumes immediately with the
/// GPIO cause bit.
#[test]
fn light_sleep_gpio_wakeup_resumes_with_gpio_cause() {
    use esp32s3_soc::memmap::GPIO_BASE;
    use esp32s3_soc::rtc::{RTC_CNTL_BASE, SLEEP_EN_BIT, STATE0_OFF};
    let mut m = Esp32S3::new();
    // GPIO4 output high.
    m.soc.write32(GPIO_BASE + 0x24, 1 << 4);
    m.soc.write32(GPIO_BASE + 0x08, 1 << 4);
    // RTCIO PIN4_REG (page offset 0x438): WAKEUP_ENABLE + high level.
    m.soc.write32(0x6000_8438, (1 << 10) | (5 << 7));
    // WAKEUP_STATE ENA bit 2 (GPIO_TRIG).
    let ws = m.soc.read32(RTC_CNTL_BASE + 0x3C);
    m.soc.write32(RTC_CNTL_BASE + 0x3C, ws | (1 << 17));
    // Light sleep (DIG_PWC default = light).
    let prev = m.soc.read32(RTC_CNTL_BASE + STATE0_OFF);
    m.soc
        .write32(RTC_CNTL_BASE + STATE0_OFF, prev | SLEEP_EN_BIT);
    for _ in 0..1000 {
        m.step();
        if !m.is_asleep() {
            break;
        }
    }
    assert!(!m.is_asleep(), "GPIO wakeup did not resume");
    assert_eq!(
        m.soc.read32(RTC_CNTL_BASE + 0x130) & (1 << 2),
        1 << 2,
        "GPIO cause bit"
    );
}

/// Light-sleep UART0 wakeup: pending RX bytes at entry resume immediately
/// with the UART0 cause bit.
#[test]
fn light_sleep_uart_wakeup_resumes_with_uart_cause() {
    use esp32s3_soc::rtc::{RTC_CNTL_BASE, SLEEP_EN_BIT, STATE0_OFF};
    let mut m = Esp32S3::new();
    m.soc.uart_inject_rx(0, 0x5A);
    // WAKEUP_STATE ENA bit 6 (UART0_TRIG).
    let ws = m.soc.read32(RTC_CNTL_BASE + 0x3C);
    m.soc.write32(RTC_CNTL_BASE + 0x3C, ws | (1 << 21));
    // Light sleep (DIG_PWC default = light).
    let prev = m.soc.read32(RTC_CNTL_BASE + STATE0_OFF);
    m.soc
        .write32(RTC_CNTL_BASE + STATE0_OFF, prev | SLEEP_EN_BIT);
    for _ in 0..1000 {
        m.step();
        if !m.is_asleep() {
            break;
        }
    }
    assert!(!m.is_asleep(), "UART wakeup did not resume");
    assert_eq!(
        m.soc.read32(RTC_CNTL_BASE + 0x130) & (1 << 6),
        1 << 6,
        "UART0 cause bit"
    );
}

/// MCPWM timer-event interrupt delivery: TIMER0 TEZ (INT bit 3) routes
/// source 31 to line 15 (level 3); the handler clears INT_CLR and counts.
#[test]
fn mcpwm_tez_interrupt_delivers_to_vector() {
    use crate::asm::Asm;
    use esp32s3_soc::mcpwm::{MCPWM_BASE, MCPWM_INTR_SOURCE};
    use esp32s3_soc::memmap::INT_MATRIX_BASE;

    const CTR: u32 = 0x3FC8_0500;
    const STASH: u32 = 0x3FC8_0504;
    let mut a = Asm::new(IRAM_BASE);
    let l_ctr = a.offset();
    a.lit(0);
    let l_stash = a.offset();
    a.lit(0);
    let l_mat = a.offset();
    a.lit(0);
    let l_mcpwm = a.offset();
    a.lit(0);
    let code_start = a.pc();
    let p = a.l32r(2); // INT_MATRIX_BASE
    a.patch_l32r(p, IRAM_BASE + l_mat as u32);
    a.movi_n(3, 15);
    a.s32i(3, 2, 4 * MCPWM_INTR_SOURCE); // matrix source 31 -> line 15
    let p = a.l32r(2); // MCPWM_BASE
    a.patch_l32r(p, IRAM_BASE + l_mcpwm as u32);
    // TIMER0 up, period 500, run (slow enough that the handler
    // exits and the app observes CTR between TEZ refires).
    a.li(3, 500 << 8);
    a.s32i(3, 2, 0x04);
    a.li(3, (1 << 3) | 2);
    a.s32i(3, 2, 0x08);
    a.li(3, 1 << 3);
    a.s32i(3, 2, 0x110); // INT_ENA TIMER0_TEZ
    a.li(3, 0x8000); // INTENABLE bit 15
    a.wsr(228, 3);
    a.rsil(4, 0);
    let loop_start = a.pc();
    let p = a.l32r(2); // CTR
    a.patch_l32r(p, IRAM_BASE + l_ctr as u32);
    a.l32i(3, 2, 0);
    a.beqz(3, loop_start); // spin while CTR == 0
    a.li(4, 0xCAFE);
    let p = a.l32r(5); // STASH
    a.patch_l32r(p, IRAM_BASE + l_stash as u32);
    a.s32i(4, 5, 0);
    let done = a.pc();
    a.j(done);
    a.bytes_mut()[l_ctr..l_ctr + 4].copy_from_slice(&CTR.to_le_bytes());
    a.bytes_mut()[l_stash..l_stash + 4].copy_from_slice(&STASH.to_le_bytes());
    a.bytes_mut()[l_mat..l_mat + 4].copy_from_slice(&INT_MATRIX_BASE.to_le_bytes());
    a.bytes_mut()[l_mcpwm..l_mcpwm + 4].copy_from_slice(&MCPWM_BASE.to_le_bytes());

    // Level-3 handler: CTR += 1, clear TIMER0_TEZ, rfi 3.
    let mut h = Asm::new(0x4000_01C0);
    h.li(6, CTR as i32);
    h.l32i(7, 6, 0);
    h.addi(7, 7, 1);
    h.s32i(7, 6, 0);
    h.li(8, MCPWM_BASE as i32);
    h.li(9, 1 << 3);
    h.s32i(9, 8, 0x11C); // INT_CLR TIMER0_TEZ
    h.rfi(3);
    assert!(h.bytes().len() <= 0x40, "handler fits the slot");

    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 17); // MCPWM0
    m.load_image(IRAM_BASE, a.bytes());
    m.load_image(0x4000_01C0, h.bytes());
    m.cpu[0].pc = code_start;
    for _ in 0..20000 {
        if m.cpu[0].pc == done {
            break;
        }
        m.step();
    }
    assert_eq!(m.cpu[0].pc, done, "app finished its loop");
    assert_eq!(m.soc.read32(STASH), 0xCAFE, "stash after MCPWM interrupt");
    assert_eq!(m.soc.read32(CTR), 1, "handler ran once");
}

/// USB-OTG string descriptors + SOF advance + disconnect: the simulated
/// device reports string indexes, answers GET_DESCRIPTOR STRING, the
/// frame counter advances while clocked, and a disconnect drops ConnSts.
#[test]
fn usb_otg_strings_sof_and_disconnect() {
    const USB: u32 = 0x6008_0000;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 23); // USB controller clock
    // Power the port (device connects, like the usb_host sketch).
    m.soc.write32(USB + 0x440, 1 << 12); // HPRT_PWR
    assert_ne!(m.soc.read32(USB + 0x440) & 1, 0, "connected");
    // Force host mode (GUSBCFG bit 29) so DFIFO stages SETUP payloads.
    m.soc.write32(USB + 0x00C, 1 << 29);
    // GET_DESCRIPTOR STRING1 via the control path: stage the 8 SETUP
    // bytes (rt=0x80 req=6 val=0x0301 len=20) into DFIFO0, then run the
    // channel like the usb_host sketch does.
    m.soc.write32(USB + 0x1000, 0x03010680);
    m.soc.write32(USB + 0x1000, 0x00140000);
    // HCTSIZ0: xfer 20, pid SETUP(3).
    m.soc.write32(USB + 0x510, 20 | (3 << 19) | (3 << 29));
    // HCCHAR0: MPS 64, EP0, addr 0, ChEna.
    m.soc.write32(USB + 0x500, 64 | (1 << 31));
    // IN data stage (pid 2, dir bit 15): moves the staged string into RXFIFO.
    m.soc.write32(USB + 0x510, 20 | (1 << 19) | (2 << 29));
    m.soc.write32(USB + 0x500, 64 | (1 << 15) | (1 << 31));
    // Transfer ran synchronously: RXFIFO holds the string descriptor.
    let hcint = m.soc.read32(USB + 0x508);
    let w0 = m.soc.read32(USB + 0x1000);
    assert_eq!(hcint & 0x9, 0x1, "XFERCOMPL, no STALL");
    assert_eq!(w0 & 0xFFFF, 0x0314, "STR1 header (len 20, type 3)");
    assert_eq!((w0 >> 16) & 0xFFFF, 0x0045, "STR1 'E'");
    // SOF advances while clocked.
    let f0 = m.soc.read32(USB + 0x408);
    for _ in 0..10 {
        m.step();
    }
    assert!(m.soc.read32(USB + 0x408) != f0, "HFNUM advances");
    // Disconnect drops ConnSts + ENA and latches enable-change.
    m.soc.usb_otg_disconnect();
    let hprt = m.soc.read32(USB + 0x440);
    assert_eq!(hprt & 1, 0, "ConnSts dropped");
    assert_eq!(hprt & (1 << 2), 0, "port disabled");
    assert_ne!(hprt & (1 << 3), 0, "enable-change latched");
}

/// UHCI SLIP framing through GDMA: with SEPER_EN set, OUT bytes frame
/// (separators + escapes) onto the UART line, and IN bytes deframe back
/// into DRAM (split pairs reassembled).
#[test]
fn uhci_slip_frames_out_and_deframes_in() {
    use esp32s3_soc::gdma::{GDMA_BASE, GDMA_UHCI0_PERIPH};
    use esp32s3_soc::uhci::UHCI0_BASE;
    let desc = 0x3FC8_1000;
    let buf = 0x3FC8_2000;
    let rdesc = 0x3FC8_3000;
    let rbuf = 0x3FC8_4000;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, false, 8);
    // UHCI: clock on, UART1 selected, SEPER_EN framing on.
    m.soc.write32(UHCI0_BASE, (1 << 11) | (1 << 3) | (1 << 5));
    // OUT: 4 bytes incl. escapables -> framed on the UART line.
    m.soc
        .write32(desc, (1u32 << 31) | (1u32 << 30) | (4u32 << 12));
    m.soc.write32(desc + 4, buf);
    m.soc.write32(desc + 8, 0);
    m.soc.write32(buf, 0x42DB_C041); // 41 C0 DB 42 LE
    m.soc.write32(GDMA_BASE + 0xA8, GDMA_UHCI0_PERIPH);
    m.soc
        .write32(GDMA_BASE + 0x80, (desc & 0x000F_FFFF) | (1 << 21));
    assert_eq!(
        m.take_uart_tx(1),
        alloc::vec![0xC0, 0x41, 0xDB, 0xDC, 0xDB, 0xDD, 0x42, 0xC0],
        "SLIP-framed bytes reach UART1"
    );
    // IN: framed bytes in UART RX deframe into DRAM.
    for b in [0xC0u8, 0x41, 0xDB, 0xDC, 0xC0] {
        m.soc.uart_inject_rx(1, b);
    }
    m.soc
        .write32(rdesc, (1u32 << 31) | (1u32 << 30) | (16u32 << 12));
    m.soc.write32(rdesc + 4, rbuf);
    m.soc.write32(rdesc + 8, 0);
    m.soc.write32(GDMA_BASE + 0x108, GDMA_UHCI0_PERIPH);
    m.soc
        .write32(GDMA_BASE + 0xE0, (rdesc & 0x000F_FFFF) | (1 << 22));
    let w = m.soc.read32(rbuf);
    assert_eq!(w & 0xFFFF, 0xC041, "deframed bytes reach DRAM, got {w:#x}");
}

/// LCD_CAM GDMA-RX end to end at the SoC level: an IN descriptor chain
/// for the camera peri plus CAM_START streams a staged frame through the
/// DMA pump into DRAM (mirrors the `esp32s3_camcap` GDMA leg).
#[test]
fn cam_gdma_rx_streams_frame_to_descriptors() {
    const CAM: u32 = 0x6004_1000;
    const GDMA: u32 = 0x6003_F000;
    let desc = 0x3FC8_1000u32;
    let buf = 0x3FC8_2000u32;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, true, 8);
    m.soc
        .cam_inject_frame(&[0x04, 0x03, 0x02, 0x01, 0x44, 0x33, 0x22, 0x11]);
    // IN desc ch2, peri 5, len 8, eof, owned.
    m.soc
        .write32(desc, (1u32 << 31) | (1u32 << 30) | (8u32 << 12));
    m.soc.write32(desc + 4, buf);
    m.soc.write32(desc + 8, 0);
    m.soc.write32(GDMA + 0x1C8, 5);
    m.soc
        .write32(GDMA + 0x1A0, (desc & 0x000F_FFFF) | (1 << 22));
    // CAM_CTRL + START (with LINE_INT_NUM(7) + INT_ENA like the sketch).
    m.soc.write32(CAM + 0x04, (1 << 31) | (1 << 4));
    m.soc.write32(CAM + 0x64, (1 << 2) | (1 << 3));
    m.soc.write32(CAM + 0x08, (7 << 16) | (1 << 29));
    for i in 0..200 {
        m.step();
        if m.soc.read32(desc) & (1 << 31) == 0 {
            break;
        }
        if i == 199 {
            panic!("owner stuck");
        }
    }
    assert_eq!(m.soc.read32(buf), 0x01020304);
    assert_eq!(m.soc.read32(buf + 4), 0x11223344);
}

/// Deep-sleep ULP-trap wakeup (WAKEUP_STATE TRIG bit 13, RISCV_TRAP_TRIG):
/// a running ULP that halts mid-sleep wakes the chip with the trap cause.
#[test]
fn deep_sleep_ulp_trap_wakes_with_trap_cause() {
    use esp32s3_soc::memmap::RTC_SLOW_BASE;
    use esp32s3_soc::rtc::{
        RTC_CNTL_BASE, SLEEP_EN_BIT, SLP_TIMER0_OFF, SLP_TIMER1_OFF, SLP_WAKEUP_CAUSE_OFF,
        STATE0_OFF,
    };
    use esp32s3_soc::ulp::ULP_BASE;
    const WAKEUP_CAUSE: u32 = RTC_CNTL_BASE + SLP_WAKEUP_CAUSE_OFF;
    let mut m = Esp32S3::new();
    // Hand-assembled rv32im program: store to ULP reg slot 0 then ebreak
    // (halts after a few steps — mid-sleep, like a trap).
    let prog: [u32; 6] = [
        0x6000_80B7,
        0x10C0_8093,
        0x1234_5137,
        0x6781_0113,
        0x0020_A023,
        0x0010_0073,
    ];
    for (i, w) in prog.iter().enumerate() {
        m.soc.write32(RTC_SLOW_BASE + (i as u32) * 4, *w);
    }
    // Release the ULP, arm the trap trigger, enter deep sleep — all via
    // MMIO with no steps between, so the ULP is still running at entry.
    m.soc.write32(ULP_BASE, 1);
    let ws = m.soc.read32(RTC_CNTL_BASE + 0x3C);
    m.soc.write32(RTC_CNTL_BASE + 0x3C, ws | (1 << (15 + 13)));
    m.soc.write32(RTC_CNTL_BASE + SLP_TIMER0_OFF, 0x100);
    m.soc.write32(RTC_CNTL_BASE + SLP_TIMER1_OFF, 0);
    m.soc.write32(RTC_CNTL_BASE + 0x90, 1 << 31); // DIG_PWC deep
    let prev = m.soc.read32(RTC_CNTL_BASE + STATE0_OFF);
    m.soc
        .write32(RTC_CNTL_BASE + STATE0_OFF, prev | SLEEP_EN_BIT);
    let mut woke = false;
    for _ in 0..200_000 {
        m.step();
        if m.soc.read32(WAKEUP_CAUSE) & (1 << 13) != 0 {
            woke = true;
            break;
        }
    }
    assert!(woke, "ULP trap did not wake deep sleep");
}

/// MWDT0 fires with the SHARED watchdog clock (EN0 bit 3) CLEAR: only the
/// TIMG0 module clock (bit 13) gates the watchdog tick. Ground truth is
/// driver-behavioral — IDF's own Task-WDT runs MWDT0 while startup leaves
/// EN0 = 0x7100E007 (bit 3 clear), so silicon MWDT demonstrably runs
/// without it. Same 'R'-per-boot counting as `wdt_reset_reboots_machine`,
/// plus an EN0.3-clear prelude.
#[test]
fn wdt_fires_with_shared_watchdog_clock_off() {
    use crate::asm::Asm;
    use crate::rom_stub::APP_FLASH_OFFSET;

    const APP_ENTRY: u32 = IRAM_BASE;
    let mut a = Asm::new(IRAM_BASE);
    // Clear SYSTEM EN0 bit 3 (WDG_CLK) first: everything below runs gated.
    a.li(2, 0x600C_0018u32 as i32); // SYSTEM_PERIP_CLK_EN0
    a.l32i(3, 2, 0);
    a.movi(4, -9); // ~8: keep every enable except WDG
    a.and(3, 3, 4);
    a.s32i(3, 2, 0);
    // Print 'R' (UART0 FIFO), arm MWDT0 stage 0 = reset, loop forever.
    a.li(2, 0x6000_0000);
    a.movi_n(3, 0x52); // 'R'
    a.s32i(3, 2, 0);
    a.li(2, TIMG0_BASE as i32);
    a.movi_n(3, 4);
    a.s32i(3, 2, 0x50); // WDT_CONFIG2 (hold0) = 4
    a.li(3, 0xC000_0000u32 as i32); // WDT_CONFIG0: EN | stg0=reset(2<<29)
    a.s32i(3, 2, 0x48);
    let here = a.pc();
    a.j(here);
    let app = a.bytes().to_vec();

    let img = esp_app_image(IRAM_BASE, APP_ENTRY, &app);
    let mut flash = std::vec![0xFFu8; 0x200_000];
    flash[APP_FLASH_OFFSET as usize..APP_FLASH_OFFSET as usize + img.len()].copy_from_slice(&img);

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    let mut out = std::vec::Vec::new();
    for _ in 0..5000 {
        m.step();
        out.extend(m.take_uart_tx(0));
    }
    let r_count = out.iter().filter(|&&b| b == b'R').count();
    assert!(
        r_count >= 2,
        "MWDT must fire with EN0.3 clear (>=2 'R's), got {r_count}"
    );
}

/// Model-clock coherence invariant (documents the 1:1 clock-tree
/// approximation): one global tick per step advances TIMG0 (divider 0),
/// SYSTIMER unit0, and the RTC slow clock in a FIXED ratio — TIMG and
/// SYSTIMER 1:1, RTC at exactly 1/SLOW_CLK_DIV (240M/32.5k = 7384).
/// Firmware observes only self-consistent time (no wall clock exists),
/// so this ratio — not silicon's ~16:1 SYSTIMER:APB — is the contract;
/// remodeling it would only slow boots for zero observable gain.
#[test]
fn model_clock_runs_all_domains_in_lockstep() {
    use esp32s3_soc::memmap::SYSTIMER_BASE;
    use esp32s3_soc::rtc::RTC_CNTL_BASE;
    // TIMG0 up, divider 0 (every tick), from 0.
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 13); // TIMG0
    m.soc.write32(TIMG0_BASE, 0xC000_0000); // EN | INCREASE
    m.soc.write32(TIMG0_BASE + 0x18, 0); // T0LOADLO
    m.soc.write32(TIMG0_BASE + 0x20, 0); // T0LOAD
    // SYSTIMER unit0 runs from reset (WORK_EN preset); RTC slow clock too.
    m.soc.tick_timers(73840);
    assert_eq!(
        m.soc.read32(TIMG0_BASE + 0x04),
        73840,
        "TIMG0 advanced 1/step"
    );
    m.soc.write32(SYSTIMER_BASE + 0x004, 1 << 30); // UNIT0_OP update snapshot
    assert_eq!(
        m.soc.read32(SYSTIMER_BASE + 0x044),
        73840,
        "SYSTIMER advanced 1/step"
    );
    m.soc.write32(RTC_CNTL_BASE + 0x0C, 1 << 31); // TIME_UPDATE latch
    assert_eq!(
        m.soc.read32(RTC_CNTL_BASE + 0x10),
        10,
        "RTC slow clock advanced 73840/7384"
    );
}

/// UHCI HEAD capture: with HEAD_EN + SAVE_HEAD, an IN transfer's first 2
/// payload bytes land in RX_HEAD (@0x30) instead of DRAM.
#[test]
fn uhci_head_capture_diverts_first_two_bytes() {
    use esp32s3_soc::gdma::{GDMA_BASE, GDMA_UHCI0_PERIPH};
    use esp32s3_soc::uhci::UHCI0_BASE;
    let rdesc = 0x3FC8_3000;
    let rbuf = 0x3FC8_4000;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, false, 8);
    // UHCI: clock on, UART1 selected, HEAD_EN; SAVE_HEAD in CONF1.
    m.soc.write32(UHCI0_BASE, (1 << 11) | (1 << 3) | (1 << 6));
    m.soc.write32(UHCI0_BASE + 0x18, 1 << 3);
    for b in [0xAAu8, 0xBB, 0xCC, 0xDD] {
        m.soc.uart_inject_rx(1, b);
    }
    m.soc
        .write32(rdesc, (1u32 << 31) | (1u32 << 30) | (16u32 << 12));
    m.soc.write32(rdesc + 4, rbuf);
    m.soc.write32(rdesc + 8, 0);
    m.soc.write32(GDMA_BASE + 0x108, GDMA_UHCI0_PERIPH);
    m.soc
        .write32(GDMA_BASE + 0xE0, (rdesc & 0x000F_FFFF) | (1 << 22));
    assert_eq!(m.soc.read32(UHCI0_BASE + 0x30), 0xBBAA, "RX_HEAD");
    assert_eq!(m.soc.read32(rbuf), 0x0000_DDCC, "DRAM past the head");
}

/// CAM DMA-then-poll mixing on one capture: after the GDMA descriptor
/// fills and parks, later-streamed words fall back to the RX FIFO
/// (CAM_DATA polling) instead of vanishing.
#[test]
fn cam_dma_then_poll_mixing_falls_back_to_fifo() {
    const CAM: u32 = 0x6004_1000;
    const GDMA: u32 = 0x6003_F000;
    let desc = 0x3FC8_1000u32;
    let buf = 0x3FC8_2000u32;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, true, 8);
    // 4-word frame, 1-word descriptor: DMA takes word 0, the rest must
    // land in the RX FIFO for polling.
    m.soc
        .cam_inject_frame(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);
    m.soc
        .write32(desc, (1u32 << 31) | (1u32 << 30) | (4u32 << 12));
    m.soc.write32(desc + 4, buf);
    m.soc.write32(desc + 8, 0);
    m.soc.write32(GDMA + 0x1C8, 5);
    m.soc
        .write32(GDMA + 0x1A0, (desc & 0x000F_FFFF) | (1 << 22));
    m.soc.write32(CAM + 0x04, (1 << 31) | (1 << 4));
    m.soc.write32(CAM + 0x08, 1 << 29); // CAM_START, no byte limit
    for _ in 0..300 {
        m.step();
        if m.soc.read32(desc) & (1 << 31) == 0 {
            break;
        }
    }
    assert_eq!(m.soc.read32(desc) & (1 << 31), 0, "descriptor filled");
    assert_eq!(m.soc.read32(buf), 0x0403_0201, "first word via DMA");
    // Remainder arrived in the RX FIFO: poll two words.
    let mut got_empty = true;
    for _ in 0..300 {
        m.step();
        if m.soc.read32(CAM + 0x4C) & 0x7FF != 0 {
            got_empty = false;
            break;
        }
    }
    assert!(!got_empty, "RX FIFO received post-DMA words");
    assert_eq!(
        m.soc.read32(CAM + 0x48),
        0x0807_0605,
        "second word via polling"
    );
}

/// Short-frame GDMA tail: a capture shorter than the descriptor ends the
/// transfer with a partial fill (owner cleared, IN done raised) instead of
/// hanging the descriptor forever.
#[test]
fn cam_short_frame_completes_partial_descriptor() {
    const CAM: u32 = 0x6004_1000;
    const GDMA: u32 = 0x6003_F000;
    let desc = 0x3FC8_1000u32;
    let buf = 0x3FC8_2000u32;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, true, 6);
    syscon_clk(&mut m, true, 8);
    // 2-word frame against an 8-word descriptor.
    m.soc
        .cam_inject_frame(&[0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE]);
    m.soc
        .write32(desc, (1u32 << 31) | (1u32 << 30) | (32u32 << 12));
    m.soc.write32(desc + 4, buf);
    m.soc.write32(desc + 8, 0);
    m.soc.write32(GDMA + 0x1C8, 5);
    m.soc
        .write32(GDMA + 0x1A0, (desc & 0x000F_FFFF) | (1 << 22));
    m.soc.write32(CAM + 0x04, (1 << 31) | (1 << 4));
    m.soc.write32(CAM + 0x08, 1 << 29);
    for _ in 0..500 {
        m.step();
        if m.soc.read32(desc) & (1 << 31) == 0 {
            break;
        }
    }
    assert_eq!(m.soc.read32(desc) & (1 << 31), 0, "partial tail completed");
    assert_eq!(m.soc.read32(buf), 0xEFBEADDE, "first two words landed");
}

/// I2C slave stretch end to end through the bus: enable the stretch
/// function, run a dry master-read (cause 1 + SLAVE_STRETCH), clear it,
/// fill the TX FIFO, and complete the exchange.
#[test]
fn i2c_slave_stretch_holds_and_releases_exchange() {
    use esp32s3_soc::i2c::{I2C_CTR, I2C_SLAVE_ADDR, I2C_SR};
    const I2C0: u32 = 0x6001_3000;
    let mut m = Esp32S3::new();
    syscon_clk(&mut m, false, 7); // I2C0
    m.soc.write32(I2C0 + I2C_CTR, 0); // slave mode
    m.soc.write32(I2C0 + I2C_SLAVE_ADDR, 0x42);
    m.soc.write32(I2C0 + 0x84, 1 << 10); // slave_scl_stretch_en
    assert!(m.soc.i2c_slave_take_read(0, 0x42, 2).is_empty());
    assert_eq!(m.soc.read32(I2C0 + I2C_SR) >> 14 & 3, 1, "stretch cause");
    // Firmware fills the FIFO and releases the stretch.
    m.soc.write32(I2C0 + 0x1C, 0x5A); // DATA port push
    m.soc.write32(I2C0 + 0x84, (1 << 10) | (1 << 11)); // en + clr
    assert_eq!(m.soc.read32(I2C0 + I2C_SR) >> 14 & 3, 3, "released");
    assert_eq!(m.soc.i2c_slave_take_read(0, 0x42, 1), alloc::vec![0x5A]);
}

/// PSRAM mixed-size differential vs a host mirror (byte/halfword/word
/// traffic through the cache MMU window, verified periodically).
#[test]
fn psram_mixed_size_differential() {
    // Map PSRAM page 0 at vpage 0 via MMU, then hammer mixed sizes.
    let mut m = Esp32S3::new();
    // MMU table @ 0x600C5000: mmu[0] = PSRAM page 0 (0x8000+type bit).
    m.soc.write32(0x600C_5000, 0x8000);
    let base = 0x3C00_0000u32;
    let mut mirror = alloc::vec![0u8; 256];
    let mut seed = 0x12345678u32;
    let mut next = || {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        seed
    };
    for i in 0..2000 {
        let r = next();
        let off = r % 250;
        let size = 1 << (next() % 3); // 1, 2, 4
        let v = next();
        match size {
            1 => {
                m.soc.write8(base + off, v);
                mirror[off as usize] = v as u8;
            }
            2 => {
                m.soc.write16(base + off, v);
                mirror[off as usize] = v as u8;
                mirror[off as usize + 1] = (v >> 8) as u8;
            }
            _ => {
                m.soc.write32(base + off, v);
                let b = v.to_le_bytes();
                mirror[off as usize..off as usize + 4].copy_from_slice(&b);
            }
        }
        // Verify full mirror every 200 ops.
        if i % 200 == 199 {
            for (j, &b) in mirror.iter().enumerate() {
                let got = m.soc.read8(base + j as u32);
                assert_eq!(got, b as u32, "mismatch at +{j:#x} after {i} ops");
            }
        }
        let _ = r;
    }
}
