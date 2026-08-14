//! Boot ROM stub (P3): our own minimal first-stage boot code, installed at
//! the real ROM reset vector 0x40000000 (TRM memory map).
//!
//! Content (hand-assembled with `Asm`):
//! - reset: set the stack pointer in DRAM, jump to the loader
//! - loader: parse an ESP-IDF-style app image (esp_image_format.h: 24-byte
//!   esp_image_header_t at flash offset `APP_FLASH_OFFSET`, then
//!   esp_image_segment_header_t entries = load_addr u32, data_len u32, data),
//!   copy each segment into memory, jump to the image entry point. Segments
//!   are read back-to-back with no 16-byte padding/checksum (real IDF images
//!   are padded; the real 2nd-stage bootloader handles that once we can
//!   build ESP-IDF binaries).
//! - rom_puts: raw UART0 TX of a NUL-terminated string (stand-in for the ROM
//!   printf family; IDF calls these via the fixed-address ROM API table).
//!
//! The flash is read through the XIP data-cache window (0x3C000000), the
//! same address a cache-initialized CPU would use.
//!
//! Branch targets here are compile-time constants: every instruction in this
//! file is 3 bytes (no 16-bit), so pc-relative offsets are fixed regardless
//! of emission order. All encodings verified against `generated.rs` (beqz
//! 12-bit offset = target - pc - 4, j 18-bit = target - pc - 4).

use alloc::vec::Vec;

use crate::asm::Asm;

/// ROM reset vector (boot PC, TRM memory map).
pub const ROM_BASE: u32 = 0x4000_0000;
/// ROM printf stand-in: `rom_puts(a2 = string)` -> UART0.
pub const ROM_PUTS: u32 = 0x4000_0500;
/// Stack pointer set by the reset vector (top of the internal SRAM DRAM
/// region; the real ROM uses a stack near the top of internal SRAM).
pub const STACK_TOP: u32 = 0x3FC8_8000;
/// Flash offset of the app image the ROM loader boots (the factory app slot;
/// the real 2nd-stage bootloader reads the partition table at 0x8000 first).
pub const APP_FLASH_OFFSET: u32 = 0x1_0000;

/// Build the ROM stub bytes (installed at ROM_BASE).
pub fn rom_image() -> Vec<u8> {
    let mut a = Asm::new(ROM_BASE);

    // ── reset vector ─────────────────────────────────────────────────────────
    let reset = a.pc();
    debug_assert_eq!(reset, ROM_BASE);
    a.li(1, STACK_TOP as i32); // a1 = stack pointer
    let loader = a.pc() + 3; // after this 3-byte j
    a.j(loader);

    // ── loader: copy app image segments from flash, jump to entry ────────────
    let flash_win = 0x3C00_0000u32; // XIP data window (TRM memory map)
    a.li(2, (flash_win + APP_FLASH_OFFSET) as i32); // app image base
    a.l8ui(3, 2, 1); // segment_count (esp_image_header_t, offset 1)
    a.l32i(4, 2, 4); // entry_addr (offset 4)
    a.addi(5, 2, 24); // -> first segment header (24-byte image header)
    let seg_loop = a.pc();
    a.l32i(6, 5, 0); // load_addr
    a.l32i(7, 5, 4); // data_len
    a.addi(5, 5, 8); // -> segment data
    // beqz a7, seg_next; seg_next is 6 instructions further on (21 bytes).
    // (The constant is derived from the fixed 3-byte instruction widths.)
    a.beqz(7, a.pc() + 21);
    let copy_loop = a.pc();
    a.l8ui(8, 5, 0);
    a.s8i(8, 6, 0);
    a.addi(5, 5, 1);
    a.addi(6, 6, 1);
    a.addi(7, 7, -1);
    a.bnez(7, copy_loop);
    let _seg_next = a.pc();
    a.addi(3, 3, -1);
    a.bnez(3, seg_loop);
    a.jx(4); // jump to app entry point

    // ── rom_puts (fixed ROM API address) ─────────────────────────────────────
    while a.pc() < ROM_PUTS {
        a.pad2();
    }
    debug_assert_eq!(a.pc(), ROM_PUTS);
    let puts_loop = a.pc();
    a.li(5, 0x6000_0000); // UART0 (TRM UART0_BASE)
    a.l8ui(4, 2, 0);
    // beqz a4, done; done is the ret, 12 bytes on (s32i 3 + addi 3 + j 3).
    // (li a5 above is 3 instructions = 9 bytes, so the loop starts at 0x509.)
    a.beqz(4, a.pc() + 12);
    a.s32i(4, 5, 0); // UART FIFO (TXFIFO, TRM 26.3.6)
    a.addi(2, 2, 1);
    a.j(puts_loop);
    a.ret();

    a.bytes().to_vec()
}
