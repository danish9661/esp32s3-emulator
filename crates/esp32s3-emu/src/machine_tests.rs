//! ESP32-S3 machine-level tests: load hand-assembled bare-metal programs into
//! IRAM and verify UART output, GPIO behavior, and timer registers.
//!
//! NOTE on program layout: Xtensa instructions are variable-length (2/3/4
//! bytes).  Test streams are built byte-by-byte at their true offsets with
//! `insn()`; a fixed 4-byte-per-word layout misaligns after the first 3-byte
//! instruction (verified failure mode 2026-08-15).

use esp32s3_soc::gdma::{GDMA_BASE, GDMA_I2S0_PERIPH};
use esp32s3_soc::gpio::{GPIO_ENABLE_W1TS, GPIO_OUT_W1TC, GPIO_OUT_W1TS};
use esp32s3_soc::memmap::{
    ASSIST_DEBUG_BASE, GPIO_BASE, I2S0_BASE, I2S1_BASE, IRAM_BASE, LCD_CAM_BASE, PERI_BACKUP_BASE,
    SENSITIVE_BASE, SYSCON_BASE, TIMG0_BASE, UART0_BASE, WCL_BASE,
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
    // Partition table at 0x8000: ota_1 + otadata.
    flash[0x8000] = 0x50;
    flash[0x8001] = 0xAA;
    // entry 0: ota_1 (app, subtype 0x11) @ 0x200000
    flash[0x8002] = 0x00;
    flash[0x8003] = 0x11;
    flash[0x8004..0x8008].copy_from_slice(&0x200000u32.to_le_bytes());
    flash[0x8008..0x800C].copy_from_slice(&0x100000u32.to_le_bytes());
    flash[0x8010..0x8020].copy_from_slice(b"ota_1\0\0\0\0\0\0\0\0\0\0\0");
    // entry 1: otadata (data, subtype 0x39) @ 0xe000
    flash[0x8020] = 0x50;
    flash[0x8021] = 0xAA;
    flash[0x8022] = 0x01;
    flash[0x8023] = 0x39;
    flash[0x8024..0x8028].copy_from_slice(&0xe000u32.to_le_bytes());
    flash[0x8028..0x802C].copy_from_slice(&0x2000u32.to_le_bytes());
    flash[0x8030..0x8040].fill(0);
    flash[0x8030..0x8040].copy_from_slice(b"otadata\0\0\0\0\0\0\0\0\0");
    flash[0x8040] = 0xEB;
    flash[0x8041] = 0xEB;
    // otadata: slot0 invalid, slot1 valid (seq 1).
    flash[0xe000..0xe004].copy_from_slice(&0u32.to_le_bytes());
    flash[0xe020..0xe024].copy_from_slice(&0x8000_0001u32.to_le_bytes());

    assert_eq!(select_ota_boot_offset(&flash), Some(0x200000));
    flash[0x200000 as usize..0x200000 as usize + img.len()].copy_from_slice(&img);

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
        flash[b] = 0x50;
        flash[b + 1] = 0xAA;
        flash[b + 2] = ty;
        flash[b + 3] = sub;
        flash[b + 4..b + 8].copy_from_slice(&off.to_le_bytes());
        flash[b + 8..b + 12].copy_from_slice(&len.to_le_bytes());
        flash[b + 16..b + 32].copy_from_slice(label);
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
    // otadata: slot 0 valid (seq 1), slot 1 invalid (erased flash reads
    // 0xFF, whose bit 31 the parser treats as valid — zero it explicitly).
    flash[OTADATA_OFF as usize..OTADATA_OFF as usize + 4]
        .copy_from_slice(&0x8000_0001u32.to_le_bytes());
    flash[OTADATA_OFF as usize + 0x20..OTADATA_OFF as usize + 0x24]
        .copy_from_slice(&0u32.to_le_bytes());
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

    // Firmware-style update: erase the otadata sector, then program both
    // records (slot 0 invalid, slot 1 sequence 2) through MEMSPI.
    let mut rec = [0u8; 64];
    rec[0x20..0x24].copy_from_slice(&0x8000_0002u32.to_le_bytes());
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
        0,
        "slot-0 record cleared by the update"
    );
    assert_eq!(
        u32::from_le_bytes(
            img2[OTADATA_OFF as usize + 0x20..OTADATA_OFF as usize + 0x24]
                .try_into()
                .unwrap()
        ),
        0x8000_0002,
        "slot-1 record programmed by the update"
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
    m.load_image(CODE, &a.bytes());
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
    a.li(1, 0x3FC8_9000); // SP (clear of the ROM layout struct)
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
    m.load_image(CODE, &a.bytes());
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
    m.load_image(CODE, &a.bytes());
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
    m.soc.write32(RTC_IO_BASE + 0x00, 0x55);
    assert_eq!(m.soc.read32(RTC_IO_BASE + 0x00), 0x55);

    // `out_w1ts` sets bits, `out_w1tc` clears bits (real silicon semantics).
    m.soc.write32(RTC_IO_BASE + 0x04, 0xAA);
    assert_eq!(m.soc.read32(RTC_IO_BASE + 0x00), 0xFF);
    m.soc.write32(RTC_IO_BASE + 0x08, 0x0F);
    assert_eq!(m.soc.read32(RTC_IO_BASE + 0x00), 0xF0);

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
    m.soc.write32(ULP_BASE + 0x00, 0xDEAD_BEEF); // core
    m.soc.write32(ULP_BASE + 0x04, 0x1234_5678); // ocp
    m.soc.write32(ULP_BASE + 0x0C, 0xAB); // general reg 0
    assert_eq!(m.soc.read32(ULP_BASE + 0x00), 0xDEAD_BEEF);
    assert_eq!(m.soc.read32(ULP_BASE + 0x04), 0x1234_5678);
    assert_eq!(m.soc.read32(ULP_BASE + 0x0C), 0xAB);
}

#[test]
fn sdmmc_registers_round_trip() {
    use esp32s3_soc::sdmmc::SDMMC_BASE;

    let mut m = Esp32S3::new();
    m.soc.write32(SDMMC_BASE + 0x00, 0x000F_0001); // CTRL
    m.soc.write32(SDMMC_BASE + 0x2C, 0x0020_0000); // CMD (no start bit -> stores)
    m.soc.write32(SDMMC_BASE + 0x30, 0xCAFE_BEEF); // RESP0
    assert_eq!(m.soc.read32(SDMMC_BASE + 0x00), 0x000F_0001);
    assert_eq!(m.soc.read32(SDMMC_BASE + 0x2C), 0x0020_0000);
    assert_eq!(m.soc.read32(SDMMC_BASE + 0x30), 0xCAFE_BEEF);
}

#[test]
fn rtc_i2c_registers_round_trip() {
    use esp32s3_soc::rtc_i2c::RTC_I2C_BASE;

    let mut m = Esp32S3::new();
    // RTC_I2C (LP/I2C) block at 0x6000_8C00.
    m.soc.write32(RTC_I2C_BASE + 0x00, 0x0000_0032); // I2C_SCL_LOW
    m.soc.write32(RTC_I2C_BASE + 0x04, 0x0000_0064); // I2C_SCL_HIGH
    m.soc.write32(RTC_I2C_BASE + 0x0C, 0x00FF_00AA); // I2C_CTRL
    assert_eq!(m.soc.read32(RTC_I2C_BASE + 0x00), 0x0000_0032);
    assert_eq!(m.soc.read32(RTC_I2C_BASE + 0x04), 0x0000_0064);
    assert_eq!(m.soc.read32(RTC_I2C_BASE + 0x0C), 0x00FF_00AA);
}

#[test]
fn lp_uart_registers_round_trip() {
    use esp32s3_soc::lp_uart::LP_UART_BASE;

    let mut m = Esp32S3::new();
    // LP_UART block at 0x6002_5400 (shares the GPSPI3 page).
    m.soc.write32(LP_UART_BASE + 0x00, 0x0000_00AB); // FIFO
    m.soc.write32(LP_UART_BASE + 0x14, 0x00AA_00BB); // CLKDIV
    m.soc.write32(LP_UART_BASE + 0x20, 0x1234_5678); // CONF0
    assert_eq!(m.soc.read32(LP_UART_BASE + 0x00), 0x0000_00AB);
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
    let b = TWAI_BASE;

    // Enter reset mode so the acceptance filter is writable.
    m.soc.write32(b + 0x00, 1);
    // Acceptance filter: code 0, mask 0xFFFFFFFF (all don't-care) -> accept all.
    for off in [0x40u32, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C] {
        m.soc
            .write32(b + off, if off < 0x50 { 0 } else { 0xFFFF_FFFF });
    }
    // Leave reset, enter self-test mode (stm = bit 2) -> TX loops back to RX.
    m.soc.write32(b + 0x00, 1 << 2);

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
        // Use 0x20 (a plain config register on every modeled block) so the
        // round-trip holds even for peripherals whose 0x10 is a computed
        // read-only register (e.g. I2S INT_ST).
        let w = 0x1234_5678u32;
        m.soc.write32(*base + 0x20, w);
        let r = m.soc.read32(*base + 0x20);
        assert_eq!(r, w, "{} register round-trip failed", name);
        // A second, distinct offset also round-trips.
        m.soc.write32(*base + 0x24, 0xDEAD_BEEF);
        assert_eq!(m.soc.read32(*base + 0x24), 0xDEAD_BEEF, "{} off 0x24", name);
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
    m.soc.write32(GPIO_BASE + 0x554 + 4 * sd_pin, 25);
    m.soc.write32(GPIO_BASE + 0x554 + 4 * bck_pin, 22);
    m.soc
        .write32(GPIO_BASE + 0x24, (1u32 << sd_pin) | (1u32 << bck_pin));
    m.soc.write32(i2s_fifo, 0x8000);
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
    // Start I2S0 TX with loopback so the FIFO words arrive at RX.
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

    // Enable IDMAC and point it at the descriptor.
    m.soc.write32(sd + IDMAC_CTRL, 1);
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
