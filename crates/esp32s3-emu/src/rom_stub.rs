//! Boot ROM stub (P3): our own minimal first-stage boot code, installed at
//! the real ROM reset vector 0x40000000 (TRM memory map).
//!
//! Content (hand-assembled with `Asm`):
//! - reset: read PRID — core 1 branches to `CORE1_WAIT` and spins on the
//!   APP-CPU release register (SYSTEM.APPCPU_CTRL_A @ 0x600C0004 — the real
//!   ROM's release slot, see below) until core 0 writes it.  Core 0 sets the
//!   stack pointer in DRAM and jumps to the loader.
//! - loader: parse an ESP-IDF-style app image (esp_image_format.h: 24-byte
//!   esp_image_header_t at flash offset `APP_FLASH_OFFSET`, then
//!   esp_image_segment_header_t entries = load_addr u32, data_len u32, data),
//!   copy each segment into memory, jump to the image entry point. Segments
//!   are read back-to-back with no 16-byte padding/checksum (real IDF images
//!   are padded; the real 2nd-stage bootloader handles that once we can
//!   build ESP-IDF binaries).
//! - core1_wait: APP-CPU release gate, matching the real S3 ROM: the reset
//!   vector (0x400454) checks PRID == 0xABAB (core 1), reads
//!   SYSTEM.APPCPU_CTRL_A (0x600C0004) and jumps to the stored boot address
//!   (bit 31 = fastboot-valid flag, masked with 0x7FFFFFFF).  Our stub polls
//!   the register until nonzero and jumps — the app's
//!   `ets_set_appcpu_boot_addr` (real ROM 0x40043664) stores
//!   `call_start_cpu1` there with bit 31 clear.
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

/// End of the ROM image (exclusive) — the ROM-boot phase ends when core 0
/// leaves this region (`Esp32S3::step` clears `Soc::rom_boot_mode`).
pub const ROM_BASE_END_SENTINEL: u32 = 0x4000_9000;

/// Real ESP32-S3 boot ROM size: 384 KB (0x40000000-0x4005FFFF,
/// esp32s3_rev0_rom.elf — esp-rom-elfs, Apache-2.0; provenance +
/// extraction in tools/esp32s3_rom/README.md).
pub const ROM_SIZE: u32 = 0x0006_0000;
/// End of the real ROM (exclusive).
pub const ROM_END: u32 = ROM_BASE + ROM_SIZE;

/// The real ROM text (384 KB @ 0x40000000) and its data/rodata blobs —
/// pre-loaded by `Esp32S3::boot_from_flash` (rodata @ 0x3FF18000, data
/// @ 0x3FCD7E00; the ROM's window vectors + boot glue are spliced over the
/// text in `rom_image`).
pub fn rom_text_blob() -> &'static [u8] {
    include_bytes!("../../../tools/esp32s3_rom/esp32s3_rom.bin")
}
pub fn rom_rodata_blob() -> &'static [u8] {
    include_bytes!("../../../tools/esp32s3_rom/esp32s3_rom_rodata.bin")
}
pub fn rom_data_blob() -> &'static [u8] {
    include_bytes!("../../../tools/esp32s3_rom/esp32s3_rom_data.bin")
}
/// ROM printf stand-in: `rom_puts(a2 = string)` -> UART0.
pub const ROM_PUTS: u32 = 0x4000_0500;
/// ets_printf mailbox body: free space between ROM_PUTS (ends 0x51B) and
/// the rtc_get_reset_reason slot (0x57C).  4-aligned for l32r-free code.
pub const HOST_PRINTF_BODY: u32 = 0x4000_0520;
/// Stack pointer set by the reset vector.  The ROM hands the app a stack in
/// the 0x3FCE0000-0x3FCEFFFF region — the "stacks used by startup flow"
/// area at the TOP of the D/IRAM data window (ESP-IDF heap memory_layout.c
/// Level 8; bootstrap buffers end 0x3FCE9704, esp32s3-mm.pdf; real S3 boot
/// backtraces show SP 0x3FCEB4B0-0x3FCEB570).  Its instruction-side alias
/// (0x403D0000+) is above the app's text, so the stack can never overwrite
/// code — unlike 0x3FC88000, whose D-side writes hit the app's SRAM0 text.
pub const STACK_TOP: u32 = 0x3FCE_F000;
/// Fixed address of the APP-CPU release spin loop (the ROM stub pads the
/// region below the ROM API table).  Core 1 lands here on `rsr PRID` != 0.
pub const CORE1_WAIT: u32 = 0x4000_0480;
/// The S3's APP-CPU release register: the real ROM's
/// `ets_set_appcpu_boot_addr` (0x40043664) stores the core-1 entry address
/// in SYSTEM.APPCPU_CTRL_A (TRM SYSTEM_APPCPU_CTRL_A; bit 31 = boot-addr
/// valid flag), and the ROM reset vector's core-1 path jumps to
/// [0x600C0004] & 0x7FFFFFFF when bit 31 is set.  Our stub polls the
/// register for nonzero (the app stores `call_start_cpu1` with bit 31
/// clear) and jumps to it.
pub const APPCPU_CTRL_A: u32 = 0x600C_0004;
/// Host printf mailbox (host-defined; sits in DRAM below the core-0 stack
/// top so nothing collides): the `ets_printf` ROM slot stores {fmt, a3..a7}
/// here and the machine formats it on the host side, emitting the bytes via
/// UART0.  Six words: [0] = format pointer, [1..5] = the first five varargs.
pub const HOST_PRINTF: u32 = 0x3FC8_7F10;
/// qsort body: free space between the EFUSE_GET_MAC_HELPER body (ends
/// 0x400002E5) and the reset vector 0x40000400.  The real ROM's libc
/// exports start at 0x40000570, so 0x300..0x400 collides with nothing;
/// the old 0x1300 location sat MID-EXPORT (strcasestr 0x1368 / strcat
/// 0x1374) — the printf path's first ILLEGAL (EPC1 0x40001377).  4-aligned
/// for the entry's stack frame.
pub const QSORT_BODY: u32 = 0x4000_0300;
/// libc/newlib bodies: free IROM space past the last real ROM export
/// (0x4000642C).  Frame-less bodies are entered AFTER their slot's
/// `entry a1,0` (no entry of their own — a second entry would re-rotate
/// the window, PS.CALLINC is not cleared by ENTRY); frame-ful bodies
/// (strcasestr, strlcat, strtok_r, strtol, utoa, itoa) are entered via a
/// bare `j` from their slot and do their own `entry(1,16)`.  All bodies
/// may clobber a2-a7 only (a8-a15 are the caller's window: a0 RA, a1 SP).
/// 0x100-stride slots keep the addresses fixed for the slot `j`s.
pub const SBRK_BODY: u32 = 0x4000_6500;
pub const ISASCII_BODY: u32 = 0x4000_6600;
pub const ISBLANK_BODY: u32 = 0x4000_6700;
pub const ISCNTRL_BODY: u32 = 0x4000_6800;
pub const ISGRAPH_BODY: u32 = 0x4000_6900;
pub const ISPRINT_BODY: u32 = 0x4000_6A00;
pub const ISPUNCT_BODY: u32 = 0x4000_6B00;
pub const MEMCCPY_BODY: u32 = 0x4000_6C00;
pub const MEMCHR_BODY: u32 = 0x4000_6D00;
pub const MEMRCHR_BODY: u32 = 0x4000_6E00;
pub const STRCASESTR_BODY: u32 = 0x4000_6F00;
pub const STRCAT_BODY: u32 = 0x4000_7000;
pub const STRCSPN_BODY: u32 = 0x4000_7100;
pub const STRLCAT_BODY: u32 = 0x4000_7200;
pub const STRLCPY_BODY: u32 = 0x4000_7300;
pub const STRNCAT_BODY: u32 = 0x4000_7400;
pub const STRNLEN_BODY: u32 = 0x4000_7500;
pub const STRRCHR_BODY: u32 = 0x4000_7600;
pub const STRSEP_BODY: u32 = 0x4000_7700;
pub const STRSPN_BODY: u32 = 0x4000_7800;
pub const STRTOK_R_BODY: u32 = 0x4000_7900;
pub const STRUPR_BODY: u32 = 0x4000_7A00;
pub const ABS_BODY: u32 = 0x4000_7B00;
pub const DIV_BODY: u32 = 0x4000_7C00;
pub const RAND_R_BODY: u32 = 0x4000_7D00;
pub const RAND_BODY: u32 = 0x4000_7E00;
pub const SRAND_BODY: u32 = 0x4000_7F00;
pub const UTOA_BODY: u32 = 0x4000_8000;
pub const ITOA_BODY: u32 = 0x4000_8100;
pub const ATOI_BODY: u32 = 0x4000_8200;
pub const STRTOL_BODY: u32 = 0x4000_8300;
/// The ROM layout struct the app's heap init reads (ets_rom_layout_t).
/// The S3 struct starts with magic, then dram0_rtos_reserved_start/end;
/// the ROM's RTOS-reserved DRAM is the HIGH-DRAM area (PRO/APP CPU stacks
/// 0x3FCE9710-0x3FCED710 + ROM .bss/.data up to 0x3FCF0000) — NOT the
/// ESP32-classic 0x3FC88000 low area.  Stored in free DRAM below the ROM
/// data region; the pointer lives at 0x3FF1FFFC per esp32s3.rom.ld (RTC
/// fast memory).
pub const ROM_LAYOUT: u32 = 0x3FC8_7FD0;
/// Flash offset of the app image the ROM loader boots (the factory app slot;
/// the real 2nd-stage bootloader reads the partition table at 0x8000 first).
pub const APP_FLASH_OFFSET: u32 = 0x1_0000;
/// Window address where the app image is host-mapped for the ROM loader
/// (FLASH_DATA_BASE + LOADER_SCRATCH_OFF; see Soc::map_app_flash_segments).
pub const LOADER_SCRATCH: u32 = 0x3C1F_0000;

/// A fixed-address ROM stub entry: load address + the code to assemble there.
type ApiSlot = (u32, fn(&mut Asm));

/// Pad exactly to `target` with 2-byte `pad2`s, using one 3-byte `movi` if
/// the gap is odd (the code sections above are odd-length, so the 2-byte
/// pads alone would overshoot the fixed ROM API addresses).
pub fn pad_to(a: &mut Asm, target: u32) {
    let pc = a.pc();
    assert!(target >= pc, "pad_to: pc={:#x} > target={:#x}", pc, target);
    let mut gap = (target - pc) as i64;
    if gap % 2 == 1 {
        assert!(gap >= 3, "pad_to: 1-byte gap at {:#x}", pc);
        a.movi(15, 0); // 3-byte pad (movi a15, 0)
        gap -= 3;
    }
    while gap > 0 {
        a.pad2();
        gap -= 2;
    }
    debug_assert_eq!(a.pc(), target);
}

/// Build the ROM stub bytes (installed at ROM_BASE).
pub fn rom_image() -> Vec<u8> {
    // The real 384 KB boot ROM (esp-rom-elfs esp32s3_rev0_rom.elf, Apache-2.0
    // — see tools/esp32s3_rom/README.md).  The app's ROM calls resolve to the
    // real silicon code (the __call_* wrapper table @ 0x40000570+ and the
    // newlib/libgcc bodies @ 0x40020000+).  Only the reset/vector region
    // below GLUE_END is replaced by our boot glue: the window vectors here
    // are a1-based (the emulator's overflow rotation gives the handler the
    // frame SP in a1; real silicon uses a5), and the reset vector must run
    // our segment loader (the real ROM's _ResetVector waits for a SPI boot
    // flow we do not model).
    //
    // NOTE: the stub's API-table assembly past GLUE_END is dead — it is
    // assembled and then discarded by the splice (kept as reference).
    let mut rom = rom_text_blob().to_vec();
    let mut a = Asm::new(ROM_BASE);

    // ── window overflow/underflow handlers ──────────────────────────────────
    // ESP32-S3 window vectors: OF4@0x00, UF4@0x40, OF8@0x80, UF8@0xC0,
    // OF12@0x100, UF12@0x140 (VECBASE 0x40000000 + core-isa.h
    // XCHAL_WINDOW_*_VECOFS, 0x40 bytes each).  The RESET vector is NOT
    // here — it lives at 0x40000400 (XCHAL_RESET_VECTOR_PADDR), so 0x00
    // is a pure OF4 handler.  The app later relocates VECBASE to its own
    // IRAM vector table; these handlers are the standard Xtensa
    // spill/fill sequences (ISA RM "Window Handling") and must be
    // functional for pre-relocation code regardless.
    const BOOT: u32 = 0x4000_0400; // reset vector (core-isa.h)
    const OF4: u32 = 0x4000_0000;
    const UF4: u32 = 0x4000_0040;
    const OF8: u32 = 0x4000_0080;
    const UF8: u32 = 0x4000_00C0;
    const OF12: u32 = 0x4000_0100;
    const UF12: u32 = 0x4000_0140;
    let of4 = a.pc();
    debug_assert_eq!(of4, OF4);
    // OF4: spill a0-a3 to the frame's 16-byte base save area (SP-16..-1).
    // After the overflow rotation the handler's a1 is the overflowing
    // frame's SP (QEMU window_check rotates, then EPC1 = faulting pc).
    a.s32e(0, 1, -16);
    a.s32e(1, 1, -12);
    a.s32e(2, 1, -8);
    a.s32e(3, 1, -4);
    a.rfwo();
    pad_to(&mut a, UF4);
    // UF4: fill a0-a3 back from the saved frame, rfwu.
    a.l32e(0, 1, -16);
    a.l32e(1, 1, -12);
    a.l32e(2, 1, -8);
    a.l32e(3, 1, -4);
    a.rfwu();
    pad_to(&mut a, OF8);
    // OF8: spill a0-a3 at SP-16..-1 and a4-a7 at SP-32..-17.
    a.s32e(0, 1, -16);
    a.s32e(1, 1, -12);
    a.s32e(2, 1, -8);
    a.s32e(3, 1, -4);
    a.s32e(4, 1, -32);
    a.s32e(5, 1, -28);
    a.s32e(6, 1, -24);
    a.s32e(7, 1, -20);
    a.rfwo();
    pad_to(&mut a, UF8);
    a.l32e(0, 1, -16);
    a.l32e(1, 1, -12);
    a.l32e(2, 1, -8);
    a.l32e(3, 1, -4);
    a.l32e(4, 1, -32);
    a.l32e(5, 1, -28);
    a.l32e(6, 1, -24);
    a.l32e(7, 1, -20);
    a.rfwu();
    pad_to(&mut a, OF12);
    // OF12: call12 base save area (48 bytes): a0-a3 at SP-16..-1, a8-a11 at
    // SP-32..-17, a4-a7 at SP-48..-33 (ISA RM call12 spill order).
    a.s32e(0, 1, -16);
    a.s32e(1, 1, -12);
    a.s32e(2, 1, -8);
    a.s32e(3, 1, -4);
    a.s32e(8, 1, -32);
    a.s32e(9, 1, -28);
    a.s32e(10, 1, -24);
    a.s32e(11, 1, -20);
    a.s32e(4, 1, -48);
    a.s32e(5, 1, -44);
    a.s32e(6, 1, -40);
    a.s32e(7, 1, -36);
    a.rfwo();
    pad_to(&mut a, UF12);
    a.l32e(0, 1, -16);
    a.l32e(1, 1, -12);
    a.l32e(2, 1, -8);
    a.l32e(3, 1, -4);
    a.l32e(8, 1, -32);
    a.l32e(9, 1, -28);
    a.l32e(10, 1, -24);
    a.l32e(11, 1, -20);
    a.l32e(4, 1, -48);
    a.l32e(5, 1, -44);
    a.l32e(6, 1, -40);
    a.l32e(7, 1, -36);
    a.rfwu();

    // ── 64-bit shift helper bodies ───────────────────────────────────────────
    // Relocated out of the window-vector region (0x000-0x17F); they use
    // a2-a6 only (a4-a7 are caller-saved — the windowed ABI needs no
    // spills).  NO `entry` here: the libgcc slots (0x400021B4 etc.) do the
    // windowed entry and j to these bodies — a second entry would re-rotate
    // the window (PS.CALLINC is not cleared by ENTRY; QEMU HELPER(entry),
    // win_helper.c — the 2026-08-17 double-rotation bug).
    const ASHLDI3_BODY: u32 = 0x4000_0180;
    const ASHRDI3_BODY: u32 = 0x4000_0240;
    const LSHRDI3_BODY: u32 = 0x4000_0290;
    /// ets_efuse_get_mac: writes 6 zero bytes (no eFuse programmed) and
    /// returns 0 — the ESP-IDF esp_efuse driver reads the default MAC with
    /// `esp_rom_efuse_get_mac` (esp_rom_efuse.h) and expects all-zeroes.
    const EFUSE_GET_MAC_HELPER: u32 = 0x4000_02D0;
    let shift_bodies: &[ApiSlot] = &[
        // __ashldi3: a2:a3 <<= (a4 & 63).  count < 32: a3 = (a3<<c) |
        // (a2 >> (32-c)); a2 <<= c.  count >= 32: a3 = a2 << (c-32); a2 = 0.
        (ASHLDI3_BODY, |a| {
            a.movi(5, 63);
            a.and(4, 4, 5);
            a.beqz(4, a.pc() + 48);
            a.movi(5, 32);
            a.bgeu(4, 5, a.pc() + 30);
            a.sub(5, 5, 4); // 32 - c
            a.ssr(5);
            a.srl(6, 2); // a2 >> (32-c)
            a.ssl(4); // SAR = 32-c -> SLL shifts by c
            a.sll(3, 3);
            a.or(3, 3, 6);
            a.ssl(4);
            a.sll(2, 2);
            a.j(a.pc() + 15);
            a.sub(4, 4, 5); // c - 32
            a.ssl(4);
            a.sll(3, 2);
            a.movi(2, 0);
            a.retw();
        }),
        // __ashrdi3: a2:a3 >>= (a4 & 63) arithmetic (a3 sign-fills).
        (ASHRDI3_BODY, |a| {
            a.movi(5, 63);
            a.and(4, 4, 5);
            a.beqz(4, a.pc() + 51);
            a.movi(5, 32);
            a.bgeu(4, 5, a.pc() + 27);
            a.sub(5, 5, 4); // 32 - c
            a.ssl(5);
            a.sll(6, 3); // a3 << (32-c)
            a.ssr(4);
            a.srl(2, 2); // a2 >>= c
            a.or(2, 2, 6);
            a.sra(3, 3); // a3 >>= c, sign-filled
            a.j(a.pc() + 21);
            a.sub(4, 4, 5); // c - 32
            a.ssr(4);
            a.sra(2, 3); // a2 = a3 >> (c-32), sign-filled
            a.movi(5, 31);
            a.ssr(5);
            a.sra(3, 3); // a3 = all sign bits
            a.retw();
        }),
        // __lshrdi3: a2:a3 >>= (a4 & 63) logical.
        (LSHRDI3_BODY, |a| {
            a.movi(5, 63);
            a.and(4, 4, 5);
            a.beqz(4, a.pc() + 48);
            a.movi(5, 32);
            a.bgeu(4, 5, a.pc() + 30);
            a.sub(5, 5, 4); // 32 - c
            a.ssl(5);
            a.sll(6, 3); // a3 << (32-c)
            a.ssr(4);
            a.srl(2, 2); // a2 >>= c
            a.or(2, 2, 6);
            a.ssr(4);
            a.srl(3, 3);
            a.j(a.pc() + 15);
            a.sub(4, 4, 5); // c - 32
            a.ssr(4);
            a.srl(2, 3);
            a.movi(3, 0);
            a.retw();
        }),
        (EFUSE_GET_MAC_HELPER, |a| {
            a.movi_n(3, 0);
            a.s8i(3, 2, 0);
            a.s8i(3, 2, 1);
            a.s8i(3, 2, 2);
            a.s8i(3, 2, 3);
            a.s8i(3, 2, 4);
            a.s8i(3, 2, 5);
            a.movi_n(2, 0); // ESP_OK
            a.retw();
        }),
    ];
    for (addr, f) in shift_bodies {
        pad_to(&mut a, *addr);
        debug_assert_eq!(a.pc(), *addr);
        f(&mut a);
    }

    // ── qsort body (0x40000300, free space before the reset vector) ──────────
    // The real ROM's libc exports occupy 0x40000570..0x4000642C (dense,
    // ~12 bytes apart), so the body cannot live mid-ROM; 0x300..0x400 is
    // export-free and not a vector address (user 0x500, kernel 0x600,
    // NMI 0x700, window ofs 0xC00+).  The qsort SLOT (0x40001488) and the
    // per-slot j's are what the firmware calls; the body does its own
    // entry (the slot jumps without one, like the qsort slot below).
    // Branch targets below are the exact label addresses (verified by
    // measuring the assembled body): the B4cc bgeu/bltu 8-bit offsets
    // cannot reach >127 bytes, so the outer loop uses bgeu+j.
    pad_to(&mut a, QSORT_BODY);
    debug_assert_eq!(a.pc(), QSORT_BODY);
    {
        let mut body = Asm::new(QSORT_BODY);
        body.entry(1, 56); // [0]=nmemb [4]=size [8]=compar [12]=base [16]=i [20]=j [24]=cur [28]=prev
        body.s32i(3, 1, 0);
        body.s32i(4, 1, 4);
        body.s32i(5, 1, 8);
        body.s32i(2, 1, 12);
        body.movi(6, 1);
        let outer = body.pc();
        body.movi(13, 0);
        body.or(14, 6, 6);
        body.or(15, 4, 4);
        let m_loop = body.pc();
        body.beqz(14, 0x4000_0336); // -> m_done
        body.movi(9, 1);
        body.and(8, 14, 9);
        body.beqz(8, 0x4000_032A); // -> m_skip (over the add)
        body.add(13, 13, 15);
        body.slli(15, 15, 1);
        body.ssr(9); // SAR = 1 (emu SSR: SAR = as) -> srl shifts by 1
        body.srl(14, 14);
        body.j(m_loop);
        body.add(10, 2, 13); // cur = base + i*size
        body.or(7, 6, 6); // j = i
        let inner = body.pc();
        body.beqz(7, 0x4000_03B7); // j == 0 -> next_i
        body.l32i(15, 1, 4); // prev = cur - size (size from stack slot [4],
        body.sub(11, 10, 15); // NOT hard-coded: heap_caps_init qsorts 8-byte entries)
        body.s32i(2, 1, 12);
        body.s32i(4, 1, 4);
        body.s32i(5, 1, 8);
        body.s32i(6, 1, 16);
        body.s32i(7, 1, 20);
        body.s32i(10, 1, 24);
        body.s32i(11, 1, 28);
        body.or(6, 11, 11); // compar(prev, cur)
        body.or(7, 10, 10);
        body.or(8, 5, 5);
        body.callx4(8);
        body.movi(9, 31); // SAR = 31 via SSR -> srl gives the sign bit
        body.ssr(9);
        body.srl(9, 6); // a9 = sign of prev - cur (a6 = callee a2 = GCC return-value register)
        body.l32i(2, 1, 12);
        body.l32i(4, 1, 4);
        body.l32i(5, 1, 8);
        body.l32i(6, 1, 16);
        body.l32i(7, 1, 20);
        body.l32i(10, 1, 24);
        body.l32i(11, 1, 28);
        body.bnez(9, 0x4000_03B7); // prev < cur -> in order -> next_i
        body.movi(12, 0);
        let sw_loop = body.pc();
        body.bne(12, 4, 0x4000_0390); // -> sw_body
        body.j(0x4000_03A8); // -> sw_done
        body.l8ui(8, 11, 0);
        body.l8ui(9, 10, 0);
        body.s8i(9, 11, 0);
        body.s8i(8, 10, 0);
        body.addi(11, 11, 1);
        body.addi(10, 10, 1);
        body.addi(12, 12, 1);
        body.j(sw_loop);
        body.addi(7, 7, -1); // sw_done: j--
        body.l32i(15, 1, 4); // cur = prev (the swap loop advanced cur by
        body.sub(10, 10, 15); // TWO sizes: undo both)
        body.sub(10, 10, 15);
        body.j(inner);
        body.addi(6, 6, 1); // next_i
        body.l32i(3, 1, 0);
        body.bgeu(6, 3, 0x4000_03C3); // i >= nmemb -> done
        body.j(outer);
        body.retw();
        a.bytes_mut().extend_from_slice(body.bytes());
    }
    debug_assert_eq!(a.pc(), QSORT_BODY + 198);

    // ── reset vector + boot prologue (0x40000400) ───────────────────────────
    // XCHAL_RESET_VECTOR_PADDR (core-isa.h): the window vectors own
    // 0x40000000-0x17F, so the reset PC is 0x40000400 on the S3 (QEMU
    // esp32s3.c writes its boot stub there).
    pad_to(&mut a, BOOT);
    debug_assert_eq!(a.pc(), BOOT);
    // The real ROM's reset prologue runs on BOTH cores before the PRID split
    // (core 1 keeps these for the APP-CPU entry: its first app instruction
    // is `entry a1, imm`, illegal while PS.WOE == 0 — QEMU
    // test_exceptions_entry).
    a.li(3, 0x40000); // PS = WOE (ISA RM PS bit 18)
    a.wsr(xtensa_core::cpu::SR_PS, 3);
    a.rsync();
    a.li(1, STACK_TOP as i32); // a1 = stack pointer (both cores)
    a.rsr(2, xtensa_core::cpu::SR_PRID);
    a.li(3, 0xABAB); // core-1 PRID strapping (the real ROM compares
    // against 0xABAB at 0x40045D and _start selects the core-1 stack on
    // PRID == 0xCDCD); core 0 (0xCDCD) skips the spin.
    a.bne(2, 3, a.pc() + 6); // PRID != 0xABAB -> past the CORE1_WAIT j
    a.j(CORE1_WAIT); // core 1 spins on the APP-CPU release register
    // core 0 falls through: the loader starts here

    // ── loader: copy app image segments from flash, jump to entry ────────────
    // The app image is host-mapped at the loader scratch offset (data window);
    // reading it at its 1:1 flash offset would collide with the app's own
    // MMU-mapped pages (I/D windows share one MMU table).
    a.li(2, LOADER_SCRATCH as i32); // app image base (scratch mapping)
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

    // ── ROM API table: stubs at the REAL ROM addresses ───────────────────────
    // Real firmware calls ROM functions at fixed addresses (ESP-IDF
    // components/esp_rom/esp32s3/ld/esp32s3.rom.ld).  Each stub is a leaf
    // that may clobber a2-a7 (Xtensa ABI: a2-a3 args/return, a4-a7
    // caller-saved; a8-a15 must be preserved, so stubs avoid them).
    //
    // EVERY stub does `entry a1, 0` + `retw` — the windowed-call contract:
    // CALLX8 stores the return address in the CALLER's a2 physical slot
    // (phys[wb*4 + callinc*4]); only ENTRY's rotation makes it appear in
    // the callee's a0, and plain `ret` would jump to the caller's a0 (0 at
    // reset — this was the boot's first crash).  retw requires a0's
    // callinc bits (set by CALLX4/8/12); callers always use callx8.
    //
    // Dense clusters (symbols 12 bytes apart, e.g. the UART group) cannot
    // hold `li` + store + entry + retw; those use `l32r` against literals
    // emitted in the padding gaps.  `uart_tx_one_char2` is exactly 12 bytes
    // and falls through into `uart_rx_one_char`, which shares its final
    // store + retw (real ROMs share boundary bytes the same way).
    const LIT_UART0: u32 = 0x4000_0644;
    const LIT_APPCPU_CTRL: u32 = 0x4000_071C;
    const LIT_APB_FREQ: u32 = 0x4000_1A24;
    const LIT_CPU_FREQ: u32 = 0x4000_1A30;
    const LIT_XTAL_FREQ: u32 = 0x4000_1A78;
    const INTR_UNLOCK_HELPER: u32 = 0x4000_1B96;
    const LIT_PS_INTLEVEL_MASK: u32 = 0x4000_1BB4;
    const LIT_INTLEVEL_MASK: u32 = 0x4000_1BB8;
    /// _xtos handler tables (DRAM, zeroed at boot; above the core-0 stack
    /// top 0x3FC88000, below the app .data 0x3FC92F00):
    /// 0x3FC8A000 = 32 x 4B interrupt handlers, +0x80 = 32 x 4B handler args,
    /// 0x3FC8A200 = 32 x 4B exception handlers.
    // Tables in the D-only ROM-data area 0x3FC80000-0x3FC87FFF: the D/IRAM
    // window (0x3FC88000+) is the data-side alias of the app's text
    // (0x40378000+), so tables there would be clobbered by / clobber code.
    const XTOS_INT_TABLE: u32 = 0x3FC8_6000;
    const XTOS_INT_ARG_TABLE: u32 = 0x3FC8_6080;
    const XTOS_EXC_TABLE: u32 = 0x3FC8_6200;
    /// regi2c analog register file: 16 blocks x 256 regs = 4 KB at
    /// 0x3FC8A400 (host-invented; the internal analog I2C bus has no MMIO).
    const REGI2C_TABLE: u32 = 0x3FC8_6400;
    let api: &[ApiSlot] = &[
        (CORE1_WAIT, |a| {
            a.li(3, APPCPU_CTRL_A as i32);
            let core1_poll = a.pc();
            a.l32i(2, 3, 0); // poll SYSTEM.APPCPU_CTRL_A until core 0 writes it
            a.beqz(2, core1_poll);
            a.jx(2); // jump to the stored core-1 entry point
        }),
        (ROM_PUTS, |a| {
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
        }),
        // ets_printf(fmt=a2, args a3..a7): host printf mailbox — the real
        // ROM formats with vsnprintf + putc; we store fmt + first 5 varargs
        // at HOST_PRINTF and let the machine format them (Esp32S3::
        // take_uart_tx drains the mailbox into UART0).  Body sits in the
        // ROM_PUTS→0x57C gap so the entries stay in ascending address order.
        (HOST_PRINTF_BODY, |a| {
            a.li(8, HOST_PRINTF as i32);
            a.s32i(2, 8, 0); // fmt
            a.s32i(3, 8, 4); // arg0
            a.s32i(4, 8, 8); // arg1
            a.s32i(5, 8, 12); // arg2
            a.s32i(6, 8, 16); // arg3
            a.s32i(7, 8, 20); // arg4
            a.retw();
        }),
        // rtc_get_reset_reason -> POWERON_RESET (1)
        (0x4000_057C, |a| {
            a.entry(1, 0);
            a.movi(2, 1);
            a.retw();
        }),
        // rtc_get_wakeup_cause -> none
        (0x4000_05A0, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }),
        // ets_is_print_boot -> true
        (0x4000_05C4, |a| {
            a.entry(1, 0);
            a.movi(2, 1);
            a.retw();
        }),
        // ets_printf entry point: entry + jump to the mailbox body (the
        // body needs 39 bytes; the 0x5D0 slot is only 12 wide).
        (0x4000_05D0, |a| {
            a.entry(1, 0);
            a.j(HOST_PRINTF_BODY);
        }),
        // ets_install_putc1 / ets_install_uart_printf / ets_install_putc2
        (0x4000_05DC, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_05E8, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_05F4, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        // ets_delay_us: busy-wait, nothing to wait for
        (0x4000_0600, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_060C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_get_stack_info
        (0x4000_0618, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_install_lock
        (0x4000_063C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // UartRxString
        // literal for the UART cluster (4-aligned, pc of 0x648 l32r = 0x648)
        (LIT_UART0, |a| a.lit(0x6000_0000)),
        // uart_tx_one_char(a2 = char) -> UART0 FIFO
        (0x4000_0648, |a| {
            a.entry(1, 0);
            let at = a.l32r(3);
            a.patch_l32r(at, LIT_UART0);
            a.s32i(2, 3, 0);
            a.retw();
        }),
        // uart_tx_one_char2(a2 = uart, a3 = char) -> UARTn FIFO; exactly 12
        // bytes, falls through into uart_rx_one_char's shared store + retw
        (0x4000_0654, |a| {
            a.entry(1, 0);
            a.slli(4, 2, 16); // uart * 0x10000 (UARTn bases are 0x10000 apart)
            let at = a.l32r(5);
            a.patch_l32r(at, LIT_UART0);
            a.add(4, 4, 5);
        }),
        // uart_rx_one_char: shares tx2's final s32i, then retw (a direct call
        // stores garbage to an unmapped address — dropped by the SoC — and
        // returns undefined; no input is modeled so boot never reads RX)
        (0x4000_0660, |a| {
            a.s32i(3, 4, 0);
            a.retw();
        }),
        // uart_rx_one_char_block: no input modeled
        (0x4000_066C, |a| {
            a.entry(1, 0);
            a.movi(2, -1);
            a.retw();
        }),
        // uart_rx_readbuff -> 0 bytes read
        (0x4000_0678, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }),
        (0x4000_0684, |a| {
            a.entry(1, 0);
            a.retw();
        }), // uartAttach
        (0x4000_0690, |a| {
            a.entry(1, 0);
            a.retw();
        }), // uart_tx_flush
        (0x4000_069C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // uart_tx_wait_idle
        (0x4000_06A8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // uart_div_modify
        // ets_write_char_uart(a2 = char) -> UART0 FIFO
        (0x4000_06B4, |a| {
            a.entry(1, 0);
            let at = a.l32r(3);
            a.patch_l32r(at, LIT_UART0);
            a.s32i(2, 3, 0);
            a.retw();
        }),
        (0x4000_06C0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // uart_tx_switch
        // software_reset: real silicon resets; self-loop models the hang
        (0x4000_06D8, |a| {
            a.entry(1, 0);
            a.j(a.pc());
        }),
        (0x4000_06E4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // software_reset_cpu
        (0x4000_0708, |a| {
            a.entry(1, 0);
            a.retw();
        }), // clear_super_wdt_reset_flag
        (0x4000_0714, |a| {
            a.entry(1, 0);
            a.retw();
        }), // disable_default_watchdog
        // literal for the APPCPU_CTRL_A store (4-aligned; 0x720 l32r pc = 0x720)
        (LIT_APPCPU_CTRL, |a| a.lit(APPCPU_CTRL_A)),
        // ets_set_appcpu_boot_addr(a2 = addr): the S3 APP-CPU release —
        // store the core-1 entry in SYSTEM.APPCPU_CTRL_A (0x600C0004), where
        // the core-1 reset path (our CORE1_WAIT / the real ROM fastboot)
        // polls.  Matches the real ROM fn at 0x40043664.
        (0x4000_0720, |a| {
            a.entry(1, 0);
            let at = a.l32r(3);
            a.patch_l32r(at, LIT_APPCPU_CTRL);
            a.s32i(2, 3, 0);
            a.retw();
        }),
        (0x4000_072C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // esp_rom_set_rtc_wake_addr
        (0x4000_0738, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // esp_rom_get_rtc_wake_addr
        (0x4000_0774, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Uart_Init
        (0x4000_0780, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_set_user_start
        // ── newlib/_xtos bodies (free pad gap: reachable only via our j's) ──
        // Windowed ABI: after the slot's `entry a1,0`, callee a2/a3/a4 =
        // caller's a4/a5/a6 (CALLX8), so `memset(dst,val,n)` arrives as
        // dst=a2, val=a3, n=a4.  Bodies clobber a2..a8 only (caller-saved).
        // memset(dst=a2, val=a3, n=a4) -> dst (newlib memset semantics).
        (0x4000_078C, |a| {
            a.or(5, 2, 2); // a5 = dst
            let loop_ = a.pc();
            a.beqz(4, loop_ + 15); // done: retw at loop+15
            a.s8i(3, 5, 0);
            a.addi(5, 5, 1);
            a.addi(4, 4, -1);
            a.bnez(4, loop_);
            a.retw();
        }),
        // memcpy(dst=a2, src=a3, n=a4) -> dst.
        (0x4000_07A4, |a| {
            a.or(5, 2, 2);
            let loop_ = a.pc();
            a.beqz(4, loop_ + 21);
            a.l8ui(6, 3, 0);
            a.s8i(6, 5, 0);
            a.addi(5, 5, 1);
            a.addi(3, 3, 1);
            a.addi(4, 4, -1);
            a.bnez(4, loop_);
            a.retw();
        }),
        // memmove -> memcpy body (overlap only matters for copies touching
        // the same page the wrong way; the app's uses are non-overlapping).
        (0x4000_07C4, |a| {
            a.j(0x4000_07A4);
        }),
        // memcmp(a2, a3, a4) -> a2 - b2 as i8 diff (0 if equal).
        (0x4000_07CC, |a| {
            let loop_ = a.pc();
            a.beqz(4, loop_ + 24); // eq: movi a2,0
            a.l8ui(5, 2, 0);
            a.l8ui(6, 3, 0);
            a.bne(5, 6, loop_ + 30); // diff: sub a2,a5,a6
            a.addi(2, 2, 1);
            a.addi(3, 3, 1);
            a.addi(4, 4, -1);
            a.bnez(4, loop_);
            a.movi(2, 0);
            a.retw();
            a.sub(2, 5, 6);
            a.retw();
        }),
        // strlen(s=a2) -> length.
        (0x4000_07F0, |a| {
            a.movi(3, 0);
            let loop_ = a.pc();
            a.l8ui(4, 2, 0);
            a.beqz(4, loop_ + 15); // done: or a2,a3,a3
            a.addi(2, 2, 1);
            a.addi(3, 3, 1);
            a.j(loop_);
            a.or(2, 3, 3);
            a.retw();
        }),
        // strcpy(dst=a2, src=a3) -> dst.
        (0x4000_080C, |a| {
            a.or(5, 2, 2);
            let loop_ = a.pc();
            a.l8ui(4, 3, 0);
            a.s8i(4, 5, 0);
            a.beqz(4, loop_ + 18);
            a.addi(5, 5, 1);
            a.addi(3, 3, 1);
            a.j(loop_);
            a.retw();
        }),
        // strncpy(dst=a2, src=a3, n=a4) -> dst; NUL-pads if src shorter.
        (0x4000_0828, |a| {
            a.or(5, 2, 2);
            let head = a.pc();
            a.beqz(4, head + 45); // done: retw at head+45
            let loop_ = a.pc();
            a.l8ui(6, 3, 0);
            a.s8i(6, 5, 0);
            a.beqz(6, head + 27); // pad loop at head+27
            a.addi(5, 5, 1);
            a.addi(3, 3, 1);
            a.addi(4, 4, -1);
            a.bnez(4, loop_);
            a.j(head + 42); // done2: retw at head+42
            let pad = a.pc();
            a.beqz(4, head + 42);
            a.addi(5, 5, 1);
            a.s8i(6, 5, 0); // a6 == 0: NUL pad
            a.addi(4, 4, -1);
            a.j(pad);
            a.retw(); // done2 (head+42)
            a.retw(); // done (head+45)
        }),
        // strcmp(a2, a3) -> i8 diff (0 if equal).
        (0x4000_0860, |a| {
            let loop_ = a.pc();
            a.l8ui(5, 2, 0);
            a.l8ui(6, 3, 0);
            a.bne(5, 6, loop_ + 21); // diff
            a.beqz(5, loop_ + 27); // eq
            a.addi(2, 2, 1);
            a.addi(3, 3, 1);
            a.j(loop_);
            a.sub(2, 5, 6);
            a.retw(); // diff (loop_+21)
            a.movi(2, 0);
            a.retw(); // eq (loop_+27)
        }),
        // strncmp(a2, a3, a4) -> i8 diff (0 if equal).
        (0x4000_0884, |a| {
            let head = a.pc();
            a.beqz(4, head + 27); // eq
            let loop_ = a.pc();
            a.l8ui(5, 2, 0);
            a.l8ui(6, 3, 0);
            a.bne(5, 6, head + 33); // diff
            a.beqz(5, head + 27);
            a.addi(2, 2, 1);
            a.addi(3, 3, 1);
            a.addi(4, 4, -1);
            a.bnez(4, loop_);
            a.movi(2, 0);
            a.retw(); // eq (head+27)
            a.sub(2, 5, 6);
            a.retw(); // diff (head+33)
        }),
        // strchr(s=a2, c=a3) -> ptr or 0.
        (0x4000_08B0, |a| {
            let loop_ = a.pc();
            a.l8ui(4, 2, 0);
            a.beqz(4, loop_ + 18); // nf: movi a2,0
            a.bne(4, 3, loop_ + 12); // next: addi a2,a2,1
            a.retw();
            a.addi(2, 2, 1);
            a.j(loop_);
            a.movi(2, 0);
            a.retw();
        }),
        // memchr(s=a2, c=a3, n=a4) -> ptr or 0.
        (0x4000_08C8, |a| {
            let head = a.pc();
            a.beqz(4, head + 21); // nf: movi a2,0 at head+21
            let loop_ = a.pc();
            a.l8ui(5, 2, 0);
            a.bne(5, 3, loop_ + 9); // next: addi a2,a2,1
            a.retw();
            a.addi(2, 2, 1);
            a.addi(4, 4, -1);
            a.bnez(4, loop_);
            a.movi(2, 0);
            a.retw();
        }),
        // _xtos_ints_off(mask=a2): old = INTENABLE; INTENABLE &= ~mask.
        (0x4000_08E8, |a| {
            a.rsr(3, xtensa_core::cpu::SR_INTENABLE);
            a.or(4, 3, 3); // old
            a.and(5, 3, 2);
            a.xor(3, 3, 5);
            a.wsr(xtensa_core::cpu::SR_INTENABLE, 3);
            a.or(2, 4, 4);
            a.retw();
        }),
        // _xtos_ints_on(mask=a2): old = INTENABLE; INTENABLE |= mask.
        (0x4000_0900, |a| {
            a.rsr(3, xtensa_core::cpu::SR_INTENABLE);
            a.or(4, 3, 3); // old
            a.or(3, 3, 2);
            a.wsr(xtensa_core::cpu::SR_INTENABLE, 3);
            a.or(2, 4, 4);
            a.retw();
        }),
        // _xtos_set_intlevel(level=a2) -> old level.  Also the shared body
        // for _xtos_restore_intlevel (a2 = level to restore).
        (0x4000_0914, |a| {
            a.rsr(3, xtensa_core::cpu::SR_PS);
            a.or(4, 3, 3); // old PS
            a.movi(6, 15);
            a.and(2, 2, 6); // new level (masked)
            a.movi(5, -16);
            a.and(3, 3, 5); // PS & ~INTLEVEL
            a.or(3, 3, 2);
            a.wsr(xtensa_core::cpu::SR_PS, 3);
            a.rsync();
            a.and(2, 4, 6); // return old level
            a.retw();
        }),
        // _xtos_set_exception_handler(cause=a2, handler=a3) -> old handler.
        // Table at XTOS_EXC_TABLE (32 entries x 4B, DRAM, zeroed at boot).
        (0x4000_0938, |a| {
            a.li(4, XTOS_EXC_TABLE as i32);
            a.slli(5, 2, 2);
            a.add(4, 4, 5);
            a.l32i(6, 4, 0);
            a.s32i(3, 4, 0);
            a.or(2, 6, 6);
            a.retw();
        }),
        // _xtos_set_interrupt_handler(intnum=a2, handler=a3) -> old handler.
        (0x4000_0960, |a| {
            a.li(4, XTOS_INT_TABLE as i32);
            a.slli(5, 2, 2);
            a.add(4, 4, 5);
            a.l32i(6, 4, 0);
            a.s32i(3, 4, 0);
            a.or(2, 6, 6);
            a.retw();
        }),
        // _xtos_set_interrupt_handler_arg(intnum=a2, handler=a3, arg=a4)
        // -> old handler (the arg slot lives 0x80 past the handler slot).
        (0x4000_0984, |a| {
            a.li(6, XTOS_INT_ARG_TABLE as i32);
            a.slli(7, 2, 2);
            a.add(6, 6, 7);
            a.l32i(8, 6, 0); // old arg
            a.s32i(4, 6, 0);
            a.li(6, XTOS_INT_TABLE as i32);
            a.add(6, 6, 7);
            a.l32i(7, 6, 0); // old handler
            a.s32i(3, 6, 0);
            a.or(2, 7, 7);
            a.retw();
        }),
        // _xtos_set_vpri(intlevel=a2, mask=a3): PS.INTLEVEL = a2,
        // INTENABLE |= a3; returns 0.
        (0x4000_09CC, |a| {
            a.rsr(4, xtensa_core::cpu::SR_PS);
            a.movi(5, 15);
            a.and(2, 2, 5);
            a.movi(6, -16);
            a.and(4, 4, 6);
            a.or(4, 4, 2);
            a.wsr(xtensa_core::cpu::SR_PS, 4);
            a.rsr(4, xtensa_core::cpu::SR_INTENABLE);
            a.or(4, 4, 3);
            a.wsr(xtensa_core::cpu::SR_INTENABLE, 4);
            a.rsync();
            a.movi(2, 0);
            a.retw();
        }),
        // ── regi2c bodies (esp_rom_regi2c_*; the internal analog I2C bus).
        // The real bus reads/writes analog-block registers (SENS).  Model:
        // a host-invented 4 KB register file in DRAM at REGI2C_TABLE,
        // indexed (block & 0xf) << 8 | reg — block 0x6D etc. (the analog
        // cal registers), all-zero default, so cal reads return 0.
        // regi2c_read(block=a2, host=a3, reg=a4) -> byte.
        (0x4000_0A00, |a| {
            a.li(6, REGI2C_TABLE as i32);
            a.movi(7, 0xf);
            a.and(2, 2, 7);
            a.slli(2, 2, 8);
            a.or(2, 2, 4);
            a.add(6, 6, 2);
            a.l8ui(2, 6, 0);
            a.retw();
        }),
        // regi2c_read_mask(block=a2, host=a3, reg=a4, msb=a5, lsb=a6)
        // -> (v >> lsb) & ((1 << (msb-lsb+1)) - 1).
        (0x4000_0A2C, |a| {
            a.li(8, REGI2C_TABLE as i32);
            a.movi(9, 0xf);
            a.and(2, 2, 9);
            a.slli(2, 2, 8);
            a.or(2, 2, 4);
            a.add(8, 8, 2);
            a.l8ui(8, 8, 0); // v
            a.ssr(6); // SAR = lsb
            a.srl(8, 8); // v >> lsb
            a.sub(9, 5, 6); // msb - lsb
            a.addi(9, 9, 1); // width
            a.movi(2, 1);
            a.ssl(9); // SAR = 32 - width
            a.sll(2, 2); // 1 << width
            a.addi(2, 2, -1); // mask
            a.and(2, 2, 8);
            a.retw();
        }),
        // regi2c_write(block=a2, host=a3, reg=a4, data=a5).
        (0x4000_0A70, |a| {
            a.li(6, REGI2C_TABLE as i32);
            a.movi(7, 0xf);
            a.and(2, 2, 7);
            a.slli(2, 2, 8);
            a.or(2, 2, 4);
            a.add(6, 6, 2);
            a.s8i(5, 6, 0);
            a.retw();
        }),
        // regi2c_write_mask(block=a2, host=a3, reg=a4, msb=a5, lsb=a6,
        // data=a7): reg = (reg & ~(mask << lsb)) | ((data & mask) << lsb).
        (0x4000_0A9C, |a| {
            a.li(10, REGI2C_TABLE as i32);
            a.movi(9, 0xf);
            a.and(2, 2, 9);
            a.slli(2, 2, 8);
            a.or(2, 2, 4);
            a.add(10, 10, 2);
            a.l8ui(8, 10, 0); // old
            a.ssr(6); // SAR = lsb
            a.srl(8, 8); // old >> lsb
            a.sub(9, 5, 6); // msb - lsb
            a.addi(9, 9, 1); // width
            a.movi(2, 1);
            a.ssl(9); // SAR = 32 - width
            a.sll(2, 2); // 1 << width
            a.addi(2, 2, -1); // mask
            a.and(8, 8, 2); // old >> lsb & mask
            a.and(9, 7, 2); // data & mask
            a.ssl(6); // SAR = 32 - lsb
            a.sll(9, 9); // (data & mask) << lsb
            a.add(8, 8, 9); // new value
            a.s8i(8, 10, 0);
            a.retw();
        }),
        // ── RTC WDT hal (esp32s3.rom.ld 0x40000DBC..0x40000E34) ──────────────
        // wdt_hal_init/config_stage/enable/write_protect_{disable,enable}/
        // feed/is_enabled.  The panic path calls these (enable_rtc_wdt,
        // feed_wdts) to arm/feed the RTC watchdog.  NO-OPs: the emulator
        // never fires a WDT reset, so the register writes (RTC_CNTL
        // WDTWPROTECT/WDTCONFIG*/WDTFEED) are meaningless; the stubs only
        // need to return cleanly.  is_enabled returns 0 so feed_wdts skips
        // its feed sequence (matches the all-zero RTC_CNTL model).
        (0x4000_0DBC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // wdt_hal_init
        (0x4000_0DD4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // wdt_hal_config_stage
        (0x4000_0DE0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // wdt_hal_write_protect_disable
        (0x4000_0DEC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // wdt_hal_write_protect_enable
        (0x4000_0DF8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // wdt_hal_enable
        (0x4000_0E04, |a| {
            a.entry(1, 0);
            a.retw();
        }), // wdt_hal_disable
        (0x4000_0E1C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // wdt_hal_feed
        (0x4000_0E34, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // wdt_hal_is_enabled -> false
        // ── newlib slots (esp32s3.rom.ld addresses; j to bodies above) ──────
        // 12-byte slots: entry(3) + j(3) + pad to next; bzero packs exactly.
        // esp_rom_newlib_init_common_mutexes: inits the ROM-side newlib
        // locks (ROM DRAM data).  NO-OP: ROM-side locks are unused here
        // (our rom_printf is a host-mailbox stub); leaving them zeroed
        // means "unlocked".  MISSING SLOT BUG: with no entry at 0x11DC the
        // app's call (esp_libc_locks_init) fell through the 12-byte pad
        // gap into the memset slot at 0x11E8 and looped on garbage args.
        (0x4000_11DC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // esp_rom_newlib_init_common_mutexes
        (0x4000_11E8, |a| {
            a.entry(1, 0);
            a.j(0x4000_078C); // memset
        }),
        (0x4000_11F4, |a| {
            a.entry(1, 0);
            a.j(0x4000_07A4); // memcpy
        }),
        (0x4000_1200, |a| {
            a.entry(1, 0);
            a.j(0x4000_07C4); // memmove
        }),
        (0x4000_120C, |a| {
            a.entry(1, 0);
            a.j(0x4000_07CC); // memcmp
        }),
        (0x4000_1218, |a| {
            a.entry(1, 0);
            a.j(0x4000_080C); // strcpy
        }),
        (0x4000_1224, |a| {
            a.entry(1, 0);
            a.j(0x4000_0828); // strncpy
        }),
        (0x4000_1230, |a| {
            a.entry(1, 0);
            a.j(0x4000_0860); // strcmp
        }),
        (0x4000_123C, |a| {
            a.entry(1, 0);
            a.j(0x4000_0884); // strncmp
        }),
        (0x4000_1248, |a| {
            a.entry(1, 0);
            a.j(0x4000_07F0); // strlen
        }),
        (0x4000_1254, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // strstr: not implemented yet -> NULL
        // bzero(addr=a2, n=a3): entry + move args to memset + j (12 bytes)
        (0x4000_1260, |a| {
            a.entry(1, 0);
            a.or(4, 3, 3); // n -> a4
            a.movi(3, 0); // val = 0
            a.j(0x4000_078C); // memset
        }),
        // ── libc/newlib slots (esp32s3.rom.libc.ld / .rom.newlib.ld) ────────
        // The printf path (the app's ROM printf wrapper) calls these by
        // their fixed addresses: atol 0x400014DC, itoa 0x400014C4, strcat
        // 0x40001374 (verified 2026-08-18 — the first ILLEGAL was the stub
        // pad at 0x40001377 executing mid-export).  Slots are `entry(1,0)`
        // + `j body` (bodies entered post-entry), or a bare `j` when the
        // body does its own entry (strcasestr, strlcat, strtok_r, strtol,
        // utoa, itoa).  After the entry, args arrive in a2-a5 = the
        // caller's a10-a13 (callx8 window rotation).
        (0x4000_1278, |a| {
            a.entry(1, 0);
            a.j(SBRK_BODY); // sbrk
        }),
        (0x4000_129C, |a| {
            a.entry(1, 0);
            a.j(ISASCII_BODY); // isascii
        }),
        (0x4000_12A8, |a| {
            a.entry(1, 0);
            a.j(ISBLANK_BODY); // isblank
        }),
        (0x4000_12B4, |a| {
            a.entry(1, 0);
            a.j(ISCNTRL_BODY); // iscntrl
        }),
        (0x4000_12D8, |a| {
            a.entry(1, 0);
            a.j(ISGRAPH_BODY); // isgraph
        }),
        (0x4000_12E4, |a| {
            a.entry(1, 0);
            a.j(ISPRINT_BODY); // isprint
        }),
        (0x4000_12F0, |a| {
            a.entry(1, 0);
            a.j(ISPUNCT_BODY); // ispunct
        }),
        // toascii(c=a2) -> c & 0x7F (12 bytes, inlined in the slot)
        (0x4000_132C, |a| {
            a.entry(1, 0);
            a.movi(3, 0x7F);
            a.and(2, 2, 3);
            a.retw();
        }),
        (0x4000_1338, |a| {
            a.entry(1, 0);
            a.j(MEMCCPY_BODY); // memccpy
        }),
        (0x4000_1344, |a| {
            a.entry(1, 0);
            a.j(MEMCHR_BODY); // memchr
        }),
        (0x4000_1350, |a| {
            a.entry(1, 0);
            a.j(MEMRCHR_BODY); // memrchr
        }),
        (0x4000_1368, |a| {
            a.j(STRCASESTR_BODY); // strcasestr (body does its own entry)
        }),
        (0x4000_1374, |a| {
            a.entry(1, 0);
            a.j(STRCAT_BODY); // strcat
        }),
        (0x4000_138C, |a| {
            a.entry(1, 0);
            a.j(0x4000_08B0); // strchr (existing body)
        }),
        (0x4000_1398, |a| {
            a.entry(1, 0);
            a.j(STRCSPN_BODY); // strcspn
        }),
        (0x4000_13B0, |a| {
            a.j(STRLCAT_BODY); // strlcat (body does its own entry)
        }),
        (0x4000_13BC, |a| {
            a.entry(1, 0);
            a.j(STRLCPY_BODY); // strlcpy
        }),
        (0x4000_13E0, |a| {
            a.entry(1, 0);
            a.j(STRNCAT_BODY); // strncat
        }),
        (0x4000_13F8, |a| {
            a.entry(1, 0);
            a.j(STRNLEN_BODY); // strnlen
        }),
        (0x4000_1404, |a| {
            a.entry(1, 0);
            a.j(STRRCHR_BODY); // strrchr
        }),
        (0x4000_1410, |a| {
            a.entry(1, 0);
            a.j(STRSEP_BODY); // strsep
        }),
        (0x4000_141C, |a| {
            a.entry(1, 0);
            a.j(STRSPN_BODY); // strspn
        }),
        (0x4000_1428, |a| {
            a.j(STRTOK_R_BODY); // strtok_r (body does its own entry)
        }),
        (0x4000_1434, |a| {
            a.entry(1, 0);
            a.j(STRUPR_BODY); // strupr
        }),
        // longjmp(buf=a2, val=a3): no-op stub — returns to longjmp's own
        // caller instead of transferring to the setjmp site (wrong, but
        // the boot path never calls it; real setjmp/longjmp needs full
        // window save/restore, revisit when a caller appears).
        (0x4000_1440, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        // setjmp(buf=a2) -> 0: saves nothing (see longjmp note).
        (0x4000_144C, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }),
        (0x4000_1458, |a| {
            a.entry(1, 0);
            a.j(ABS_BODY); // abs
        }),
        (0x4000_1464, |a| {
            a.entry(1, 0);
            a.j(DIV_BODY); // div
        }),
        (0x4000_1470, |a| {
            a.entry(1, 0);
            a.j(ABS_BODY); // labs
        }),
        (0x4000_147C, |a| {
            a.entry(1, 0);
            a.j(DIV_BODY); // ldiv
        }),
        // qsort(base=a2, nmemb=a3, size=a4, compar=a5) — the ROM's qsort
        // is a full quicksort; ours is insertion sort (correct, O(n²) —
        // fine for the boot-time tables that use it, e.g. the reserved
        // memory regions; larger sorts would be slow, not wrong).
        // The body lives at QSORT_BODY (0x40000300, before the reset
        // vector) and does its own entry; the slot jumps there without
        // one.  See the body's comment for the windowed-call convention.
        (0x4000_1488, |a| {
            a.j(QSORT_BODY);
        }),
        (0x4000_1494, |a| {
            a.entry(1, 0);
            a.j(RAND_R_BODY); // rand_r
        }),
        (0x4000_14A0, |a| {
            a.entry(1, 0);
            a.j(RAND_BODY); // rand
        }),
        (0x4000_14AC, |a| {
            a.entry(1, 0);
            a.j(SRAND_BODY); // srand
        }),
        (0x4000_14B8, |a| {
            a.j(UTOA_BODY); // utoa (body does its own entry)
        }),
        (0x4000_14C4, |a| {
            a.j(ITOA_BODY); // itoa (body does its own entry)
        }),
        (0x4000_14D0, |a| {
            a.entry(1, 0);
            a.j(ATOI_BODY); // atoi
        }),
        (0x4000_14DC, |a| {
            a.entry(1, 0);
            a.j(ATOI_BODY); // atol
        }),
        (0x4000_14E8, |a| {
            a.j(STRTOL_BODY); // strtol (body does its own entry)
        }),
        (0x4000_14F4, |a| {
            a.j(STRTOL_BODY); // strtoul (same body: unsigned semantics only
        }), // differ in overflow clamping, which we don't model)
        // Cache_Get_ICache_Line_Size / DCache -> 32 (S3 cache line)
        (0x4000_15FC, |a| {
            a.entry(1, 0);
            a.movi(2, 32);
            a.retw();
        }),
        (0x4000_1608, |a| {
            a.entry(1, 0);
            a.movi(2, 32);
            a.retw();
        }),
        (0x4000_1614, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // Cache_Get_Mode
        (0x4000_1620, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Set_ICache_Mode
        (0x4000_162C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Set_DCache_Mode
        // Cache_Address_Through_*: identity (flash window addrs are direct)
        (0x4000_1638, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1644, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1650, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Set_Default_Mode
        (0x4000_165C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Enable_Default_ICache_Mode
        (0x4000_1668, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ROM_Boot_Cache_Init
        (0x4000_1674, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Invalidate_ICache_Items
        (0x4000_1680, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Invalidate_DCache_Items
        (0x4000_168C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Clean_Items
        (0x4000_1698, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_WriteBack_Items
        (0x4000_16A4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Op_Addr
        (0x4000_16B0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Invalidate_Addr
        (0x4000_16BC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Clean_Addr
        (0x4000_16C8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // rom_Cache_WriteBack_Addr
        (0x4000_16D4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Invalidate_ICache_All
        (0x4000_16E0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Invalidate_DCache_All
        (0x4000_16EC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Clean_All
        (0x4000_16F8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_WriteBack_All
        (0x4000_1704, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Mask_All
        (0x4000_1710, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_UnMask_Dram0
        (0x4000_171C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Suspend_ICache_Autoload
        (0x4000_1728, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Resume_ICache_Autoload
        (0x4000_1734, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Suspend_DCache_Autoload
        (0x4000_1740, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Resume_DCache_Autoload
        (0x4000_174C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Start_ICache_Preload
        (0x4000_1758, |a| {
            a.entry(1, 0);
            a.movi(2, 1);
            a.retw();
        }), // Cache_ICache_Preload_Done
        (0x4000_1764, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_End_ICache_Preload
        (0x4000_1770, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Start_DCache_Preload
        (0x4000_177C, |a| {
            a.entry(1, 0);
            a.movi(2, 1);
            a.retw();
        }), // Cache_DCache_Preload_Done
        (0x4000_1788, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_End_DCache_Preload
        (0x4000_1794, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Config_ICache_Autoload
        (0x4000_17A0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Config_ICache_Region_Autoload
        (0x4000_17AC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Enable_ICache_Autoload
        (0x4000_17B8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Disable_ICache_Autoload
        (0x4000_17C4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Config_DCache_Autoload
        (0x4000_17D0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Config_DCache_Region_Autoload
        (0x4000_17DC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Enable_DCache_Autoload
        (0x4000_17E8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Disable_DCache_Autoload
        (0x4000_17F4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Enable_ICache_PreLock
        (0x4000_1800, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Disable_ICache_PreLock
        (0x4000_180C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Lock_ICache_Items
        (0x4000_1818, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Unlock_ICache_Items
        (0x4000_1824, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Enable_DCache_PreLock
        (0x4000_1830, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Disable_DCache_PreLock
        (0x4000_183C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Lock_DCache_Items
        (0x4000_1848, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Unlock_DCache_Items
        (0x4000_1854, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Lock_Addr
        (0x4000_1860, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Unlock_Addr
        (0x4000_186C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Disable_ICache (cache always on)
        (0x4000_1878, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Enable_ICache
        (0x4000_1884, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Disable_DCache
        (0x4000_1890, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Enable_DCache
        (0x4000_189C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // rom_Cache_Suspend_ICache
        (0x4000_18A8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Resume_ICache
        (0x4000_18B4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Suspend_DCache
        (0x4000_18C0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Resume_DCache
        (0x4000_18CC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Occupy_Items
        (0x4000_18D8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Occupy_Addr
        (0x4000_18E4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // rom_Cache_Freeze_ICache_Enable
        (0x4000_18F0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Freeze_ICache_Disable
        (0x4000_18FC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Freeze_DCache_Enable
        (0x4000_1908, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Freeze_DCache_Disable
        (0x4000_1914, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Set_IDROM_MMU_Size
        (0x4000_1920, |a| {
            a.entry(1, 0);
            a.retw();
        }), // flash2spiram_instruction_offset
        (0x4000_192C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // flash2spiram_rodata_offset
        (0x4000_1938, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // flash_instr_rodata_start_page
        (0x4000_1944, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // flash_instr_rodata_end_page
        (0x4000_1950, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Set_IDROM_MMU_Info
        (0x4000_195C, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // Cache_Get_IROM_MMU_End
        (0x4000_1968, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // Cache_Get_DROM_MMU_End
        (0x4000_1974, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Owner_Init
        (0x4000_1980, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Occupy_ICache_MEMORY
        (0x4000_198C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Occupy_DCache_MEMORY
        (0x4000_1998, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_MMU_Init (MMU pre-mapped host-side)
        // Cache_Ibus_MMU_Set / Cache_Dbus_MMU_Set: the app re-programs the
        // same mappings the machine pre-mapped; writes are idempotent.
        (0x4000_19A4, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_19B0, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_19BC, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // rom_Cache_Count_Flash_Pages
        (0x4000_19C8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Flash_To_SPIRAM_Copy
        (0x4000_19D4, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Travel_Tag_Memory
        (0x4000_19E0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Travel_Tag_Memory2
        (0x4000_19EC, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Get_Virtual_Addr
        (0x4000_19F8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Get_Memory_BaseAddr
        (0x4000_1A04, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Get_Memory_Addr
        (0x4000_1A10, |a| {
            a.entry(1, 0);
            a.retw();
        }), // Cache_Get_Memory_value
        (0x4000_1A1C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // rom_config_instruction_cache_mode
        // ets_get_apb_freq -> 80 MHz (S3 default APB)
        (LIT_APB_FREQ, |a| a.lit(80_000_000)),
        (0x4000_1A28, |a| {
            a.entry(1, 0);
            a.retw();
        }), // rom_config_data_cache_mode
        // ets_get_cpu_frequency -> 240 MHz (Arduino core default)
        (LIT_CPU_FREQ, |a| a.lit(240_000_000)),
        (0x4000_1A34, |a| {
            a.entry(1, 0);
            let at = a.l32r(2);
            a.patch_l32r(at, LIT_APB_FREQ);
            a.retw();
        }),
        (0x4000_1A40, |a| {
            a.entry(1, 0);
            let at = a.l32r(2);
            a.patch_l32r(at, LIT_CPU_FREQ);
            a.retw();
        }),
        (0x4000_1A4C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_update_cpu_frequency
        (0x4000_1A58, |a| {
            a.entry(1, 0);
            a.movi(2, 0);
            a.retw();
        }), // ets_get_printf_channel -> UART0
        (0x4000_1A64, |a| {
            a.entry(1, 0);
            a.movi(2, 1);
            a.retw();
        }), // ets_get_xtal_div
        (0x4000_1A70, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_set_xtal_div
        // ets_get_xtal_freq -> 40 MHz (S3 crystal)
        (LIT_XTAL_FREQ, |a| a.lit(40_000_000)),
        (0x4000_1A7C, |a| {
            a.entry(1, 0);
            let at = a.l32r(2);
            a.patch_l32r(at, LIT_XTAL_FREQ);
            a.retw();
        }),
        // rom_gpio_*: the app's GPIO driver writes registers directly
        (0x4000_1A88, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1A94, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1AA0, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1AAC, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1AB8, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1AC4, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1AD0, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1ADC, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1AE8, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1AF4, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1B00, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1B0C, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1B18, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1B24, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1B30, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1B3C, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1B48, |a| {
            a.entry(1, 0);
            a.retw();
        }),
        (0x4000_1B54, |a| {
            a.entry(1, 0);
            a.retw();
        }), // intr_matrix_set (app writes matrix directly)
        // ets_intr_lock: disable all interrupts, return old level
        (0x4000_1B60, |a| {
            a.entry(1, 0);
            a.rsil(2, 15);
            a.retw();
        }),
        // ets_intr_unlock(a2 = level): entry then jump to the helper in free
        // ROM space (the 12-byte slot cannot hold the restore sequence)
        (0x4000_1B6C, |a| {
            a.entry(1, 0);
            a.j(INTR_UNLOCK_HELPER);
        }),
        (0x4000_1B78, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_isr_attach
        (0x4000_1B84, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_isr_mask
        (0x4000_1B90, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_isr_unmask
    ];
    for (addr, f) in api {
        pad_to(&mut a, *addr);
        debug_assert_eq!(a.pc(), *addr);
        f(&mut a);
    }

    // ── ets_intr_unlock helper (free space after the API table) ──────────────
    // a2 = old PS returned by ets_intr_lock's `rsil a2, 15` (RSIL t,level
    // writes OLD PS into t; exec.rs line 646).  Restore INTLEVEL [3:0] only,
    // preserving CALLINC/WOE/OWB.  Entered AFTER the stub's entry, so it
    // ends with retw (rotates back).  NOTE: INTLEVEL lives in PS bits [3:0]
    // in this model (cpu.rs PS_INTLEVEL = 0xf), NOT the real-silicon
    // [19:16] — the level field must match check_interrupts (cpu.rs 432).
    pad_to(&mut a, INTR_UNLOCK_HELPER);
    debug_assert_eq!(a.pc(), INTR_UNLOCK_HELPER);
    a.rsr(3, xtensa_core::cpu::SR_PS);
    let at = a.l32r(4);
    a.patch_l32r(at, LIT_PS_INTLEVEL_MASK);
    a.and(3, 3, 4);
    let at = a.l32r(4);
    a.patch_l32r(at, LIT_INTLEVEL_MASK);
    a.and(2, 2, 4);
    a.or(3, 3, 2);
    a.wsr(xtensa_core::cpu::SR_PS, 3);
    a.rsync();
    a.retw();
    pad_to(&mut a, LIT_PS_INTLEVEL_MASK);
    a.lit(0xFFFF_FFF0); // ~INTLEVEL [3:0] mask
    pad_to(&mut a, LIT_INTLEVEL_MASK);
    a.lit(0x0000_000F); // INTLEVEL field mask

    // ── _xtos slots (after the helper + literals; j to bodies above) ─────────
    let xtos: &[ApiSlot] = &[
        (0x4000_1BF0, |a| {
            a.entry(1, 0);
            a.j(0x4000_08E8); // _xtos_ints_off
        }),
        (0x4000_1BFC, |a| {
            a.entry(1, 0);
            a.j(0x4000_0900); // _xtos_ints_on
        }),
        (0x4000_1C08, |a| {
            a.entry(1, 0);
            a.j(0x4000_0914); // _xtos_restore_intlevel
        }),
        (0x4000_1C14, |a| {
            a.entry(1, 0);
            a.j(0x4000_0938); // _xtos_set_exception_handler
        }),
        (0x4000_1C20, |a| {
            a.entry(1, 0);
            a.j(0x4000_0960); // _xtos_set_interrupt_handler
        }),
        (0x4000_1C2C, |a| {
            a.entry(1, 0);
            a.j(0x4000_0984); // _xtos_set_interrupt_handler_arg
        }),
        (0x4000_1C38, |a| {
            a.entry(1, 0);
            a.j(0x4000_0914); // _xtos_set_intlevel
        }),
        (0x4000_1C44, |a| {
            a.entry(1, 0);
            a.j(0x4000_09CC); // _xtos_set_vpri
        }),
    ];
    for (addr, f) in xtos {
        pad_to(&mut a, *addr);
        debug_assert_eq!(a.pc(), *addr);
        f(&mut a);
    }

    // ── 64-bit integer helpers (esp32s3.rom.ld addresses 0x400021B4.. ──────
    // 0x40002574; the firmware calls these via callx8: __udivdi3 from
    // rtc_clk_cal_internal's calibration math).  Each slot is
    // `entry a1, 0` + `j body`; after the slot's ENTRY, callee a2:a3 =
    // the caller's a10:a11 (dividend), a4:a5 = the caller's a12:a13
    // (divisor), and the result returns in a2:a3 (libgcc di ABI).
    // The bodies live in the free pad gap 0x40001D00-0x400021B4 and spill
    // a8-a11 to the 16 bytes below SP (the caller's register-save area,
    // Xtensa windowed ABI) since a8-a15 belong to the caller's window.
    // The divmod loop is the classic 64-bit long division (shift-subtract,
    // LCOUNT loop, 64 iterations): the running remainder < divisor, so
    // r < 2^63 before every left shift and the 64-bit carry is never lost.
    let helpers: &[ApiSlot] = &[
        // ── bodies (fixed addresses in the free gap) ──────────────────────────
        // __udivdi3: a2:a3 = dividend (consumed bit by bit), a4:a5 =
        // divisor (fixed), a6:a7 = quotient, a8:a9 = remainder.
        (0x4000_1D00, |a| {
            a.s32i(8, 1, (-16i32) as u32);
            a.s32i(9, 1, (-12i32) as u32);
            a.s32i(10, 1, (-8i32) as u32);
            a.s32i(11, 1, (-4i32) as u32);
            a.movi(6, 0); // q = 0
            a.movi(7, 0);
            a.movi(8, 0); // r = 0
            a.movi(9, 0);
            a.movi(10, 64);
            a.movi(11, 31);
            a.ssr(11); // SAR = 31 once: every bit extraction in the loop is
            // a bit31 (r_lo carry, bit63/bit31 of the dividend, q_lo carry)
            // and nothing else touches SAR until the register restore.
            a.loop_(10, a.pc() + 81); // LEND = the `or a2,a6,a6` — the
            // body's last instruction (`or a6,a6,a10`) ends exactly at LEND
            a.srl(11, 9); // bit31(r_lo) — extract before r_lo shifts
            a.slli(9, 9, 1);
            a.slli(8, 8, 1);
            a.or(8, 8, 11); // r_hi |= carry out of r_lo
            a.srl(11, 3); // bit63(dividend) = bit31(d_hi), before d shifts
            a.or(9, 9, 11); // r_lo |= bit63(dividend) — MUST go into r_lo
            // (a9), not r_hi (a8): a first version OR'd into a8 and every
            // div returned garbage (q = 0x3FF / 0x3FFFFF, verified 2026-08-17)
            a.srl(11, 2); // bit31(d_lo)
            a.slli(3, 3, 1);
            a.or(3, 3, 11); // d_hi |= carry
            a.slli(2, 2, 1); // d <<= 1
            // Compare r vs v (libgcc di ABI: a4 = v_lo, a5 = v_hi — the
            // original body compared against a4 as v_hi and a5 as v_lo, so
            // with a 32-bit divisor it tested r_hi < v_lo and never
            // subtracted once r_hi crossed v_lo).
            a.bltu(8, 5, a.pc() + 30); // r_hi < v_hi -> no subtract
            a.bne(8, 5, a.pc() + 6); // r_hi != v_hi -> subtract
            a.bltu(9, 4, a.pc() + 24); // equal hi, r_lo < v_lo -> skip
            a.sub(11, 9, 4); // r_lo - v_lo
            a.bgeu(9, 4, a.pc() + 6); // no borrow
            a.addi(8, 8, -1); // borrow: r_hi -= 1
            a.sub(8, 8, 5); // r_hi -= v_hi
            a.or(9, 11, 11);
            a.movi(10, 1); // flag = 1
            a.j(a.pc() + 6); // qshift
            a.movi(10, 0); // flag = 0
            // qshift: q = (q << 1) | flag, with the bit shifted out of q_lo
            // carried into q_hi (the original OR'd the flag into q_hi, which
            // is only value-correct for quotients < 2^32).  a10 is dead
            // here: LCOUNT lives in an SR and LOOP reads a10 once at setup.
            a.srl(11, 6); // bit31(q_lo) — carry into q_hi
            a.slli(7, 7, 1);
            a.or(7, 7, 11); // q_hi = (q_hi << 1) | carry
            a.slli(6, 6, 1);
            a.or(6, 6, 10); // q_lo = (q_lo << 1) | flag
            a.or(2, 6, 6); // q -> a2:a3 (lo, hi)
            a.or(3, 7, 7);
            a.or(4, 9, 9); // remainder -> a4:a5 with a4 = r_lo, a5 = r_hi
            a.or(5, 8, 8); // (Xtensa libgcc __udivdi3 ABI, caller-view
            // a12:a13; the body's internal r lives in a9 = r_lo / a8 = r_hi,
            // the opposite register order of libgcc's div.s, so the tail
            // must cross them — a first version assigned a4 = a8 and a5 =
            // a9, returning (r_hi, r_lo), caught by the 0x3426007cf /
            // 0xfa0 test (remainder 3599 came back word-swapped, verified
            // 2026-08-17))
            a.l32i(8, 1, (-16i32) as u32);
            a.l32i(9, 1, (-12i32) as u32);
            a.l32i(10, 1, (-8i32) as u32);
            a.l32i(11, 1, (-4i32) as u32);
            a.retw();
        }),
        // __umoddi3: same long division, but return the remainder.  a2:a3 =
        // r (starts 0), a4:a5 = dividend (shifted out), a6:a7 = divisor copy
        // (the libgcc di ABI hands the dividend in a2:a3 and divisor in
        // a4:a5, so the preamble copies them into the working registers).
        (0x4000_1D90, |a| {
            a.s32i(8, 1, (-16i32) as u32);
            a.s32i(9, 1, (-12i32) as u32);
            a.s32i(10, 1, (-8i32) as u32);
            a.s32i(11, 1, (-4i32) as u32);
            a.or(6, 4, 4); // divisor -> a6:a7 (fixed)
            a.or(7, 5, 5);
            a.or(4, 2, 2); // dividend -> a4:a5 (shifting)
            a.or(5, 3, 3);
            a.movi(2, 0); // r = 0
            a.movi(3, 0);
            a.movi(10, 64);
            a.loop_(10, a.pc() + 78); // LEND = the `or a3,a3,a3` — the
            // body's last instruction ends exactly at LEND; the no-sub
            // branches target LEND too (the do_sub completion `or a3,a11,a11`
            // must NOT run on the no-sub path).  The original body instead
            // branched past LEND, terminating the loop early — wrong unless
            // the remaining dividend bits are zero (e.g. umod(0x100, 3) ->
            // 0 instead of 1).
            a.movi(11, 31);
            a.ssr(11);
            a.srl(11, 3); // bit31(r_hi) — extract before r shifts
            a.slli(3, 3, 1);
            a.slli(2, 2, 1);
            a.or(2, 2, 11); // r_lo |= carry out of r_hi
            a.movi(11, 31);
            a.ssr(11);
            a.srl(11, 5); // bit63(dividend) = bit31(d_hi)
            a.or(2, 2, 11); // r_lo |= bit63(dividend)
            a.movi(11, 31);
            a.ssr(11);
            a.srl(11, 4); // bit31(d_lo)
            a.slli(5, 5, 1);
            a.or(5, 5, 11); // d_hi |= carry
            a.slli(4, 4, 1); // d <<= 1
            // r < v check, hi word first (the original compared r_lo against
            // v_lo first, skipping the subtract when r_lo < v_lo even though
            // r_hi > v_hi means r >= v).
            a.bltu(3, 7, a.pc() + 24); // r_hi < v_hi -> no subtract
            a.bne(3, 7, a.pc() + 6); // r_hi != v_hi -> subtract
            a.bltu(2, 6, a.pc() + 18); // equal hi, r_lo < v_lo -> no subtract
            a.sub(11, 3, 7); // r_hi - v_hi
            a.bgeu(3, 7, a.pc() + 6); // no borrow
            a.addi(2, 2, -1); // borrow: r_lo -= 1
            a.sub(2, 2, 6); // r_lo -= v_lo
            a.or(3, 11, 11); // r_hi = tmp (do_sub completion)
            a.or(3, 3, 3); // loop continuation (LEND)
            a.l32i(8, 1, (-16i32) as u32);
            a.l32i(9, 1, (-12i32) as u32);
            a.l32i(10, 1, (-8i32) as u32);
            a.l32i(11, 1, (-4i32) as u32);
            a.retw();
        }),
        // ── efuse family (esp32s3.rom.ld "efuse" group, 0x40001E90..0x40002028)
        // The eFuse block is unprogrammed in the emulator: reads return the
        // "no fuse set" values the ESP-IDF boot expects (esp_rom_efuse.h).
        // get_mac writes six zero bytes via EFUSE_GET_MAC_HELPER; get_wp_pad
        // returns 63 (EFUSE_WP_PAD "unset") so esp_mspi_pin_reserve falls
        // back to its default table.  All stubs are entry + retw leaves that
        // clobber a2 only (return), a3 max — the windowed caller's a2/a3.
        (0x4000_1E90, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_efuse_read (void)
        (0x4000_1E9C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_efuse_program (void)
        (0x4000_1EA8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_efuse_clear_program_registers (void)
        (0x4000_1EB4, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_write_key -> ESP_OK
        (0x4000_1EC0, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_get_read_register_address -> 0
        (0x4000_1ECC, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_get_key_purpose -> 0
        (0x4000_1ED8, |a| {
            a.entry(1, 0);
            a.movi_n(2, 1);
            a.retw();
        }), // ets_efuse_key_block_unused -> 1 (all key blocks free)
        (0x4000_1EE4, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_find_unused_key_block -> 0
        (0x4000_1EF0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_efuse_rs_calculate (void)
        (0x4000_1EFC, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_count_unused_key_blocks -> 0
        (0x4000_1F08, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_secure_boot_enabled -> 0
        (0x4000_1F14, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_secure_boot_aggressive_revoke_enabled -> 0
        (0x4000_1F20, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_cache_encryption_enabled -> 0
        (0x4000_1F2C, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_download_modes_disabled -> 0
        (0x4000_1F38, |a| {
            a.entry(1, 0);
            a.movi(2, -1);
            a.retw();
        }), // ets_efuse_find_purpose -> ESP_EFUSE_KEY_PURPOSE_MAX
        (0x4000_1F44, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_flash_opi_5pads_power_sel_vddspi -> 0
        (0x4000_1F50, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_force_send_resume -> 0
        (0x4000_1F5C, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_get_flash_delay_us -> 0
        (0x4000_1F68, |a| {
            a.entry(1, 0);
            a.j(EFUSE_GET_MAC_HELPER);
        }), // ets_efuse_get_mac (helper in the pre-CORE1_WAIT region)
        (0x4000_1F74, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_get_spiconfig -> 0 (uses s_mspi_io_num_default)
        (0x4000_1F80, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_usb_print_is_disabled -> 0
        (0x4000_1F8C, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_usb_serial_jtag_print_is_disabled -> 0
        (0x4000_1F98, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_get_uart_print_control -> 0 (UART0)
        (0x4000_1FA4, |a| {
            a.entry(1, 0);
            a.movi(2, 63);
            a.retw();
        }), // ets_efuse_get_wp_pad -> 63 (no flash WP pad)
        (0x4000_1FB0, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_legacy_spi_boot_mode_disabled -> 0
        (0x4000_1FBC, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_security_download_modes_enabled -> 0
        (0x4000_1FC8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_efuse_set_timing (void)
        (0x4000_1FD4, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_jtag_disabled -> 0
        (0x4000_1FE0, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_usb_download_mode_disabled -> 0
        (0x4000_1FEC, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_usb_module_disabled -> 0
        (0x4000_1FF8, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_usb_device_disabled -> 0
        (0x4000_2004, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_flash_octal_mode -> 0 (QIO, not octal)
        (0x4000_2010, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_ecc_en -> 0
        (0x4000_201C, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_ecc_flash_page_size -> 0
        (0x4000_2028, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_efuse_ecc_16to17_mode -> 0
        // ── ecc family (esp32s3.rom.ld "ecc" group, 0x40002034..0x40002118)
        // Flash/SRAM ECC is not modeled: getters return the "disabled" state,
        // setters are no-ops.
        (0x4000_2034, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_flash_enable -> 0
        (0x4000_2040, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_flash_enable_all -> 0
        (0x4000_204C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_ecc_flash_disable (void)
        (0x4000_2058, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_ecc_flash_disable_all (void)
        (0x4000_2064, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_get_flash_page_size -> 0
        (0x4000_2070, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_ecc_set_flash_page_size (void)
        (0x4000_207C, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_ecc_set_flash_byte_mode (void)
        (0x4000_2088, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_get_flash_byte_mode -> 0
        (0x4000_2094, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_set_flash_range (void)
        (0x4000_20A0, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_get_flash_range -> 0
        (0x4000_20AC, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_sram_enable -> 0
        (0x4000_20B8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_ecc_sram_disable (void)
        (0x4000_20C4, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_sram_enable_all -> 0
        (0x4000_20D0, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_ecc_sram_disable_all (void)
        (0x4000_20DC, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_get_sram_page_size -> 0
        (0x4000_20E8, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_ecc_set_sram_page_size (void)
        (0x4000_20F4, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_get_sram_byte_mode -> 0
        (0x4000_2100, |a| {
            a.entry(1, 0);
            a.retw();
        }), // ets_ecc_set_sram_byte_mode (void)
        (0x4000_210C, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_set_sram_range -> 0
        (0x4000_2118, |a| {
            a.entry(1, 0);
            a.movi_n(2, 0);
            a.retw();
        }), // ets_ecc_get_sram_range -> 0
        // ── slots at the real ROM addresses (entry + j to the bodies) ─────────
        // 64-bit integer helpers (bodies relocated to the pre-CORE1_WAIT
        // region) ...
        (0x4000_2184, |a| {
            a.entry(1, 0);
            a.j(0x4000_2600); // __adddf3
        }),
        (0x4000_21B4, |a| {
            a.entry(1, 0);
            a.j(ASHLDI3_BODY); // __ashldi3
        }),
        (0x4000_21C0, |a| {
            a.entry(1, 0);
            a.j(ASHRDI3_BODY); // __ashrdi3
        }),
        (0x4000_21D8, |a| {
            a.entry(1, 0);
            a.j(0x4000_2B10); // __bswapsi2
        }),
        (0x4000_2250, |a| {
            a.entry(1, 0);
            a.j(0x4000_2780); // __divdf3
        }),
        (0x4000_228C, |a| {
            a.entry(1, 0);
            a.j(0x4000_2A70); // __eqdf2
        }),
        (0x4000_22A4, |a| {
            a.entry(1, 0);
            a.j(0x4000_2914); // __extendsfdf2
        }),
        (0x4000_22D4, |a| {
            a.entry(1, 0);
            a.j(0x4000_2988); // __fixdfsi
        }),
        (0x4000_2334, |a| {
            a.entry(1, 0);
            a.j(0x4000_29FC); // __floatsidf
        }),
        (0x4000_2364, |a| {
            a.entry(1, 0);
            a.j(0x4000_2A3C); // __floatunsidf
        }),
        (0x4000_23A0, |a| {
            a.entry(1, 0);
            a.j(0x4000_2AA0); // __gtdf2
        }),
        (0x4000_23B8, |a| {
            a.entry(1, 0);
            a.j(0x4000_2ABC); // __ledf2
        }),
        (0x4000_23D0, |a| {
            a.entry(1, 0);
            a.j(LSHRDI3_BODY); // __lshrdi3
        }),
        (0x4000_23DC, |a| {
            a.entry(1, 0);
            a.j(0x4000_2AD8); // __ltdf2
        }),
        (0x4000_2418, |a| {
            a.entry(1, 0);
            a.j(0x4000_2638); // __muldf3
        }),
        (0x4000_246C, |a| {
            a.entry(1, 0);
            a.j(0x4000_2A88); // __nedf2
        }),
        (0x4000_24FC, |a| {
            a.entry(1, 0);
            a.j(0x4000_2618); // __subdf3
        }),
        (0x4000_2544, |a| {
            a.entry(1, 0);
            a.j(0x4000_1D00); // __udivdi3
        }),
        (0x4000_2574, |a| {
            a.entry(1, 0);
            a.j(0x4000_1D90); // __umoddi3
        }),
        (0x4000_258C, |a| {
            a.entry(1, 0);
            a.j(0x4000_2AF4); // __unorddf2
        }),
    ];
    for (addr, f) in helpers {
        pad_to(&mut a, *addr);
        debug_assert_eq!(a.pc(), *addr);
        f(&mut a);
    }

    // ── soft-float bodies (tools/gen_softfloat.sh → softfloat.bin) ───────────
    // Compiled IEEE-754 binary64 suite linked at 0x40002600; the slots above
    // jump here.  The blob is emitted as raw bytes (the C is integer-only and
    // references nothing outside itself — the Xtensa linker misresolves
    // direct l32rs against the absolute helper symbols, so shl64/shr64/mul64
    // keep every opcode inlined).
    pad_to(&mut a, 0x4000_2600);
    debug_assert_eq!(a.pc(), 0x4000_2600);
    a.bytes_mut()
        .extend_from_slice(include_bytes!("softfloat.bin"));

    // ── regi2c slots (0x5D48 < 0x1C44? no: 0x5D48 sorts AFTER the _xtos
    // cluster — separate loop so the main table stays ascending) ────────────
    let regi2c: &[ApiSlot] = &[
        (0x4000_5D48, |a| {
            a.entry(1, 0);
            a.j(0x4000_0A00); // esp_rom_regi2c_read
        }),
        (0x4000_5D54, |a| {
            a.entry(1, 0);
            a.j(0x4000_0A2C); // esp_rom_regi2c_read_mask
        }),
        (0x4000_5D60, |a| {
            a.entry(1, 0);
            a.j(0x4000_0A70); // esp_rom_regi2c_write
        }),
        (0x4000_5D6C, |a| {
            a.entry(1, 0);
            a.j(0x4000_0A9C); // esp_rom_regi2c_write_mask
        }),
    ];
    for (addr, f) in regi2c {
        pad_to(&mut a, *addr);
        debug_assert_eq!(a.pc(), *addr);
        f(&mut a);
    }

    // ── libc/newlib bodies (past the last real ROM export 0x4000642C) ───────
    // The printf path (the app's ROM printf wrapper, RAM 0x40380240) calls
    // atol 0x400014DC, itoa 0x400014C4 and strcat 0x40001374 at their fixed
    // addresses — the slots above j here.  Bodies are entered AFTER the
    // slot's `entry a1,0` (frame-less ones: no entry of their own, a second
    // entry would re-rotate the window — the 2026-08-17 double-rotation
    // bug), may clobber a2-a7 only (a8-a15 are the CALLER's window: a0 RA,
    // a1 SP), and retw.  Frame-ful bodies (strcasestr, strlcat, strtok_r,
    // strtol, utoa, itoa) are entered via a bare `j` from their slot and do
    // their own `entry(1,16)`; their frame lives at [sp+0..16).
    // Windowed arg mapping (callx8 from the app): after the rotation, a2-a5
    // = the caller's a10-a13 (arg 0-3), a6-a7 = the caller's a14-a15.
    // Branch targets are exact byte offsets (every instruction is 3 bytes);
    // 8-bit B4cc offsets must stay within ±127.
    pad_to(&mut a, SBRK_BODY);
    debug_assert_eq!(a.pc(), SBRK_BODY);
    // sbrk(incr=a2): bump allocator from a host-invented DRAM word
    // 0x3FC87F28 (below the ROM data region 0x3FC88000; zeroed at boot =
    // "first call" sentinel).  First call starts the heap at counter+8.
    a.li(5, 0x3FC8_7F28);
    let l = a.pc();
    a.l32i(4, 5, 0); // cur
    a.bnez(4, l + 9);
    a.addi(4, 5, 8); // cur = 0x3FC87F30 (base)
    a.or(6, 4, 4); // old
    a.addi(2, 2, 3);
    a.movi(7, -4);
    a.and(2, 2, 7); // incr = (incr + 3) & ~3
    a.add(4, 4, 2);
    a.s32i(4, 5, 0);
    a.or(2, 6, 6);
    a.retw();
    pad_to(&mut a, ISASCII_BODY);
    debug_assert_eq!(a.pc(), ISASCII_BODY);
    // isascii(c=a2) -> (c & ~0x7F) == 0 ? 1 : 0
    a.movi(3, 0x80); // ~0x7F (sext8 -128)
    a.and(2, 2, 3);
    a.beqz(2, a.pc() + 9); // -> yes (movi a2,1)
    a.movi(2, 0);
    a.retw();
    a.movi(2, 1);
    a.retw();
    pad_to(&mut a, ISBLANK_BODY);
    debug_assert_eq!(a.pc(), ISBLANK_BODY);
    // isblank(c=a2): ' ' or '\t'
    a.movi(3, 9);
    a.bne(2, 3, a.pc() + 9); // -> try space
    a.movi(2, 1);
    a.retw();
    a.movi(3, 32);
    a.bne(2, 3, a.pc() + 9); // -> no
    a.movi(2, 1);
    a.retw();
    a.movi(2, 0);
    a.retw();
    pad_to(&mut a, ISCNTRL_BODY);
    debug_assert_eq!(a.pc(), ISCNTRL_BODY);
    // iscntrl(c=a2): c < 0x20 || c == 0x7F
    a.movi(3, 32);
    a.bltu(2, 3, a.pc() + 9); // c < 0x20 -> yes (movi a2,1)
    a.movi(3, 0x7F);
    a.bne(2, 3, a.pc() + 9); // != 0x7F -> no (movi a2,0)
    a.movi(2, 1); // yes
    a.retw();
    a.movi(2, 0); // no
    a.retw();
    pad_to(&mut a, ISGRAPH_BODY);
    debug_assert_eq!(a.pc(), ISGRAPH_BODY);
    // isgraph(c=a2): 0x21 <= c <= 0x7E
    a.movi(3, 0x21);
    a.bltu(2, 3, a.pc() + 15); // c < 0x21 -> no
    a.movi(3, 0x7E);
    a.bltu(3, 2, a.pc() + 9); // 0x7E < c -> no
    a.movi(2, 1);
    a.retw();
    a.movi(2, 0); // no
    a.retw();
    pad_to(&mut a, ISPRINT_BODY);
    debug_assert_eq!(a.pc(), ISPRINT_BODY);
    // isprint(c=a2): 0x20 <= c <= 0x7E
    a.movi(3, 0x20);
    a.bltu(2, 3, a.pc() + 15); // c < 0x20 -> no
    a.movi(3, 0x7E);
    a.bltu(3, 2, a.pc() + 9); // 0x7E < c -> no
    a.movi(2, 1);
    a.retw();
    a.movi(2, 0); // no
    a.retw();
    pad_to(&mut a, ISPUNCT_BODY);
    debug_assert_eq!(a.pc(), ISPUNCT_BODY);
    // ispunct(c=a2): graph && !digit && !upper && !lower (newlib).
    // Ranges: c in [0x21,0x7E], not in [0x30,0x39]/[0x41,0x5A]/[0x61,0x7A].
    // Layout: bltu#1..#8 at P/P+6/P+12/P+18/P+27/P+33/P+42/P+48, jumps at
    // P+21/P+36/P+51; punct = P+54 (movi a2,1), no = P+60 (movi a2,0).
    a.movi(3, 0x21);
    a.bltu(2, 3, a.pc() + 60); // c < 0x21 -> no
    a.movi(3, 0x7E);
    a.bltu(3, 2, a.pc() + 54); // 0x7E < c -> no
    a.movi(3, 0x30);
    a.bltu(2, 3, a.pc() + 42); // c < '0' -> punct
    a.movi(3, 0x39);
    a.bltu(3, 2, a.pc() + 6); // '9' < c -> up_hi
    a.j(a.pc() + 39); // -> no
    a.movi(3, 0x41); // up_hi
    a.bltu(2, 3, a.pc() + 27); // c < 'A' -> punct
    a.movi(3, 0x5A);
    a.bltu(3, 2, a.pc() + 6); // 'Z' < c -> lo_hi
    a.j(a.pc() + 24); // -> no
    a.movi(3, 0x61); // lo_hi
    a.bltu(2, 3, a.pc() + 12); // c < 'a' -> punct
    a.movi(3, 0x7A);
    a.bltu(3, 2, a.pc() + 6); // 'z' < c -> punct
    a.j(a.pc() + 9); // -> no
    a.movi(2, 1); // punct
    a.retw();
    a.movi(2, 0); // no
    a.retw();
    pad_to(&mut a, MEMCCPY_BODY);
    debug_assert_eq!(a.pc(), MEMCCPY_BODY);
    // memccpy(dst=a2, src=a3, c=a4, n=a5) -> ptr after c or 0.
    a.or(6, 2, 2); // dst
    let l = a.pc();
    a.beqz(5, l + 30); // n == 0 -> nf (movi a2,0 at l+30)
    a.l8ui(7, 3, 0);
    a.s8i(7, 6, 0);
    a.addi(3, 3, 1);
    a.addi(6, 6, 1);
    a.addi(5, 5, -1);
    a.bne(7, 4, l); // char != c -> loop
    a.or(2, 6, 6); // found: ptr to next byte
    a.retw();
    a.movi(2, 0); // nf
    a.retw();
    pad_to(&mut a, MEMCHR_BODY);
    debug_assert_eq!(a.pc(), MEMCHR_BODY);
    // memchr(s=a2, c=a3, n=a4) -> ptr or 0.
    let l = a.pc();
    a.beqz(4, l + 21); // n == 0 -> nf
    a.l8ui(5, 2, 0);
    a.bne(5, 3, l + 12); // != c -> next
    a.retw(); // found (a2 = ptr)
    a.addi(2, 2, 1); // next
    a.addi(4, 4, -1);
    a.j(l);
    a.movi(2, 0); // nf
    a.retw();
    pad_to(&mut a, MEMRCHR_BODY);
    debug_assert_eq!(a.pc(), MEMRCHR_BODY);
    // memrchr(s=a2, c=a3, n=a4) -> ptr or 0 (scan backwards).
    a.add(2, 2, 4); // end = s + n
    let l = a.pc();
    a.beqz(4, l + 18); // n == 0 -> nf
    a.addi(2, 2, -1);
    a.l8ui(5, 2, 0);
    a.addi(4, 4, -1);
    a.bne(5, 3, l); // != c -> loop
    a.retw(); // found
    a.movi(2, 0); // nf
    a.retw();
    pad_to(&mut a, STRCASESTR_BODY);
    debug_assert_eq!(a.pc(), STRCASESTR_BODY);
    // strcasestr(hay=a2, needle=a3) -> ptr or 0.  Case-insensitive; both
    // chars are lowercased inline ('A'..'Z' + 32) before each compare.
    // [4] = hay start, saved BEFORE the empty-needle check so `found`
    // (l32i a2,[4]) also serves the empty-needle path.  The mismatch loop
    // reloads needle from [0] and hops back to `outer` (re-saves hay) so
    // the hay advance survives.
    a.entry(1, 16);
    a.s32i(3, 1, 0); // [0] = needle start
    let outer = a.pc();
    a.s32i(2, 1, 4); // [4] = hay start
    a.l8ui(4, 3, 0);
    a.beqz(4, a.pc() + 75); // empty needle -> found
    let inner = a.pc();
    a.l8ui(4, 2, 0); // hc
    a.l8ui(5, 3, 0); // nc
    a.movi(6, 0x41);
    a.bltu(4, 6, a.pc() + 12); // hc < 'A' -> hc_done
    a.movi(6, 0x5A);
    a.bltu(6, 4, a.pc() + 6); // 'Z' < hc -> hc_done
    a.addi(4, 4, 32);
    a.movi(6, 0x41);
    a.bltu(5, 6, a.pc() + 12); // nc < 'A' -> nc_done
    a.movi(6, 0x5A);
    a.bltu(6, 5, a.pc() + 6);
    a.addi(5, 5, 32);
    a.bne(4, 5, a.pc() + 15); // hc != nc -> mismatch
    a.beqz(4, a.pc() + 33); // both NUL -> found
    a.addi(2, 2, 1);
    a.addi(3, 3, 1);
    a.j(inner);
    a.l32i(3, 1, 0); // mismatch: reload needle start
    a.l32i(4, 1, 4);
    a.addi(4, 4, 1); // hay + 1
    a.l8ui(5, 4, 0);
    a.beqz(5, a.pc() + 13); // end of hay -> nf
    a.or(2, 4, 4);
    a.j(outer);
    a.l32i(2, 1, 4); // found
    a.retw();
    a.movi(2, 0); // nf
    a.retw();
    pad_to(&mut a, STRCAT_BODY);
    debug_assert_eq!(a.pc(), STRCAT_BODY);
    // strcat(dst=a2, src=a3) -> dst.
    a.or(4, 2, 2); // dst start
    let fwd = a.pc();
    a.l8ui(5, 2, 0);
    a.beqz(5, fwd + 12); // -> copy
    a.addi(2, 2, 1);
    a.j(fwd);
    let copy = a.pc(); // fwd + 9
    a.l8ui(5, 3, 0);
    a.s8i(5, 2, 0);
    a.beqz(5, fwd + 30); // src NUL -> done
    a.addi(2, 2, 1);
    a.addi(3, 3, 1);
    a.j(copy);
    a.or(2, 4, 4); // done
    a.retw();
    pad_to(&mut a, STRCSPN_BODY);
    debug_assert_eq!(a.pc(), STRCSPN_BODY);
    // strcspn(s=a2, reject=a3) -> length of leading chars not in reject.
    a.or(4, 2, 2); // p = s
    let outer = a.pc();
    a.l8ui(5, 4, 0); // c
    a.beqz(5, outer + 36); // s NUL -> done
    a.or(6, 3, 3); // r = reject
    let inner = a.pc(); // outer + 6
    a.l8ui(7, 6, 0); // rc
    a.beqz(7, outer + 30); // reject NUL -> next
    a.bne(7, 5, inner + 6); // != c -> inner_next
    a.sub(2, 4, 2); // found: p - s
    a.retw();
    a.addi(6, 6, 1); // inner_next
    a.j(inner);
    a.addi(4, 4, 1); // next
    a.j(outer);
    a.sub(2, 4, 2); // done
    a.retw();
    pad_to(&mut a, STRLCAT_BODY);
    debug_assert_eq!(a.pc(), STRLCAT_BODY);
    // strlcat(dst=a2, src=a3, size=a4) -> dlen + slen (truncation safe).
    a.entry(1, 16);
    a.s32i(3, 1, 0); // [0] = src
    a.s32i(4, 1, 4); // [4] = size
    a.or(5, 2, 2); // p = dst
    let fwd = a.pc();
    a.l8ui(6, 5, 0);
    a.beqz(6, fwd + 12); // -> fwd_done
    a.addi(5, 5, 1);
    a.j(fwd);
    a.sub(6, 5, 2); // fwd_done: dlen (a2 = dst start)
    a.s32i(6, 1, 8); // [8] = dlen
    a.l32i(4, 1, 4);
    a.bgeu(6, 4, a.pc() + 75); // dlen >= size -> ret_size_slen
    a.sub(7, 4, 6);
    a.addi(7, 7, -1); // space
    a.l32i(3, 1, 0);
    let cp = a.pc();
    a.beqz(7, cp + 24); // space == 0 -> nul
    a.l8ui(6, 3, 0);
    a.beqz(6, cp + 24); // src NUL -> nul
    a.s8i(6, 5, 0);
    a.addi(5, 5, 1);
    a.addi(3, 3, 1);
    a.addi(7, 7, -1);
    a.j(cp);
    a.movi(6, 0); // nul
    a.s8i(6, 5, 0);
    a.j(a.pc() + 3); // -> strl_from_dlen
    a.l32i(6, 1, 8); // strl_from_dlen: dlen
    let strl = a.pc();
    a.l32i(3, 1, 0); // src
    a.or(4, 3, 3); // q
    let sl = a.pc();
    a.l8ui(7, 4, 0);
    a.beqz(7, sl + 12); // -> sl_done
    a.addi(4, 4, 1);
    a.j(sl);
    a.sub(4, 4, 3); // sl_done: slen
    a.add(2, 6, 4); // dlen/size + slen
    a.retw();
    a.l32i(6, 1, 4); // ret_size_slen: size
    a.j(strl);
    pad_to(&mut a, STRLCPY_BODY);
    debug_assert_eq!(a.pc(), STRLCPY_BODY);
    // strlcpy(dst=a2, src=a3, size=a4) -> slen; NUL-fills the last slot.
    a.or(5, 2, 2); // dst
    a.or(6, 3, 3); // src start
    a.beqz(4, a.pc() + 45); // size == 0 -> ret_slen
    a.addi(4, 4, -1); // room
    let cp = a.pc();
    a.beqz(4, a.pc() + 33); // room exhausted -> nul
    a.l8ui(7, 3, 0);
    a.beqz(7, a.pc() + 21); // src NUL -> nul_cp
    a.s8i(7, 5, 0);
    a.addi(5, 5, 1);
    a.addi(3, 3, 1);
    a.addi(4, 4, -1);
    a.j(cp);
    a.movi(7, 0); // nul_cp
    a.s8i(7, 5, 0);
    a.j(a.pc() + 9); // -> ret_slen
    a.movi(7, 0); // nul
    a.s8i(7, 5, 0);
    a.sub(4, 3, 6); // ret_slen: slen = src - srcstart
    a.or(2, 4, 4);
    a.retw();
    pad_to(&mut a, STRNCAT_BODY);
    debug_assert_eq!(a.pc(), STRNCAT_BODY);
    // strncat(dst=a2, src=a3, n=a4) -> dst.
    a.or(5, 2, 2); // dst start
    let fwd = a.pc();
    a.l8ui(6, 2, 0);
    a.beqz(6, fwd + 12); // -> copy
    a.addi(2, 2, 1);
    a.j(fwd);
    let copy = a.pc(); // fwd + 9
    a.beqz(4, fwd + 42); // n == 0 -> done
    a.l8ui(6, 3, 0);
    a.beqz(6, fwd + 36); // src NUL -> nul
    a.s8i(6, 2, 0);
    a.addi(2, 2, 1);
    a.addi(3, 3, 1);
    a.addi(4, 4, -1);
    a.j(copy);
    a.movi(6, 0); // nul
    a.s8i(6, 2, 0);
    a.or(2, 5, 5); // done
    a.retw();
    pad_to(&mut a, STRNLEN_BODY);
    debug_assert_eq!(a.pc(), STRNLEN_BODY);
    // strnlen(s=a2, max=a3) -> min(strlen(s), max).
    a.or(4, 2, 2); // p = s
    let l = a.pc();
    a.beqz(3, l + 18); // max == 0 -> done
    a.l8ui(5, 4, 0);
    a.beqz(5, l + 18); // NUL -> done
    a.addi(4, 4, 1);
    a.addi(3, 3, -1);
    a.j(l);
    a.sub(2, 4, 2); // done
    a.retw();
    pad_to(&mut a, STRRCHR_BODY);
    debug_assert_eq!(a.pc(), STRRCHR_BODY);
    // strrchr(s=a2, c=a3) -> ptr to last c (or NUL position if c == 0).
    a.or(4, 2, 2); // last = s
    a.or(5, 2, 2); // p = s
    let sl = a.pc();
    a.l8ui(6, 5, 0);
    a.beqz(6, sl + 12);
    a.addi(5, 5, 1);
    a.j(sl);
    a.or(2, 5, 5); // strlen path: s + strlen
    a.retw();
    a.beqz(3, sl); // c == 0 -> strlen path
    let lp = a.pc();
    a.l8ui(6, 5, 0);
    a.beqz(6, lp + 18); // NUL -> done
    a.bne(6, 3, lp + 12); // != c -> next
    a.or(4, 5, 5); // last = p
    a.addi(5, 5, 1);
    a.j(lp);
    a.or(2, 4, 4); // done
    a.retw();
    pad_to(&mut a, STRSEP_BODY);
    debug_assert_eq!(a.pc(), STRSEP_BODY);
    // strsep(sp=a2, delim=a3): tok = *sp; find next delim char; NUL it;
    // *sp = tok + 1 (or NULL at end-of-string).
    a.l32i(4, 2, 0); // start = *sp
    a.beqz(4, a.pc() + 66); // *sp == NULL -> nf
    a.or(5, 4, 4); // tok
    let lp = a.pc();
    a.l8ui(6, 5, 0); // c
    a.beqz(6, a.pc() + 45); // NUL -> end_of_tok
    a.or(7, 3, 3); // d = delim
    let dl = a.pc();
    a.l8ui(7, 3, 0); // dc
    a.beqz(7, a.pc() + 30); // delim NUL -> dl_done
    a.bne(7, 6, a.pc() + 21); // != c -> dl_next
    a.movi(6, 0); // found: NUL it
    a.s8i(6, 5, 0);
    a.addi(6, 5, 1);
    a.s32i(6, 2, 0); // *sp = tok + 1
    a.or(2, 4, 4); // return start
    a.retw();
    a.addi(3, 3, 1); // dl_next
    a.j(dl);
    a.addi(5, 5, 1); // dl_done
    a.j(lp);
    a.movi(6, 0); // end_of_tok
    a.s32i(6, 2, 0); // *sp = NULL
    a.or(2, 4, 4);
    a.retw();
    a.movi(2, 0); // nf
    a.retw();
    pad_to(&mut a, STRSPN_BODY);
    debug_assert_eq!(a.pc(), STRSPN_BODY);
    // strspn(s=a2, accept=a3) -> length of leading chars in accept.
    a.or(4, 2, 2); // p = s
    let outer = a.pc();
    a.l8ui(5, 4, 0); // c
    a.beqz(5, outer + 30); // s NUL -> done
    a.or(6, 3, 3); // r = accept
    let inner = a.pc(); // outer + 6
    a.l8ui(7, 6, 0); // ac
    a.beqz(7, outer + 30); // accept NUL -> done
    a.bne(7, 5, inner + 6); // != c -> inner_next
    a.addi(4, 4, 1); // found: next s char
    a.j(outer);
    a.addi(6, 6, 1); // inner_next
    a.j(inner);
    a.sub(2, 4, 2); // done
    a.retw();
    pad_to(&mut a, STRTOK_R_BODY);
    debug_assert_eq!(a.pc(), STRTOK_R_BODY);
    // strtok_r(s=a2, delim=a3, saveptr=a4): newlib semantics — skip
    // leading delim chars, then scan to the next delim char; NUL it and
    // advance *saveptr (or set *saveptr = p at end-of-string).
    a.entry(1, 16);
    a.s32i(4, 1, 0); // [0] = saveptr
    a.bnez(2, a.pc() + 9); // -> have_s
    a.l32i(2, 4, 0); // s = *saveptr
    a.beqz(2, a.pc() + 120); // s == NULL -> nf
    a.or(5, 2, 2); // have_s: p = s
    let spo = a.pc();
    a.l8ui(6, 5, 0); // c
    a.beqz(6, a.pc() + 27); // -> sp_done
    a.or(7, 3, 3); // q = delim
    let spi = a.pc();
    a.l8ui(4, 7, 0); // dc
    a.beqz(4, a.pc() + 18); // delim NUL -> sp_done
    a.bne(4, 6, a.pc() + 9); // != c -> sp_next
    a.addi(5, 5, 1); // in delim
    a.j(spo);
    a.addi(7, 7, 1); // sp_next
    a.j(spi);
    a.l8ui(6, 5, 0); // sp_done: *p
    a.bnez(6, a.pc() + 15); // -> have_tok
    a.l32i(7, 1, 0); // end of string: *saveptr = p; NULL
    a.s32i(5, 7, 0);
    a.movi(2, 0);
    a.retw();
    a.or(4, 5, 5); // have_tok: token = p
    let cso = a.pc();
    a.l8ui(6, 5, 0); // c
    a.beqz(6, a.pc() + 48); // -> cs_end
    a.or(7, 3, 3); // q = delim
    let csi = a.pc();
    a.l8ui(2, 7, 0); // dc
    a.beqz(2, a.pc() + 33); // delim NUL -> cs_next
    a.bne(2, 6, a.pc() + 24); // != c -> cs_i_next
    a.movi(6, 0); // found
    a.s8i(6, 5, 0); // *p = 0
    a.addi(5, 5, 1);
    a.l32i(7, 1, 0);
    a.s32i(5, 7, 0); // *saveptr = p + 1
    a.or(2, 4, 4); // return token
    a.retw();
    a.addi(7, 7, 1); // cs_i_next
    a.j(csi);
    a.addi(5, 5, 1); // cs_next
    a.j(cso);
    a.l32i(7, 1, 0); // cs_end: *saveptr = p
    a.s32i(5, 7, 0);
    a.or(2, 4, 4);
    a.retw();
    a.movi(2, 0); // nf
    a.retw();
    pad_to(&mut a, STRUPR_BODY);
    debug_assert_eq!(a.pc(), STRUPR_BODY);
    // strupr(s=a2) -> s: uppercase in place.
    a.or(3, 2, 2); // p = s
    let l = a.pc();
    a.l8ui(4, 3, 0);
    a.beqz(4, l + 30); // NUL -> done
    a.movi(5, 0x61);
    a.bltu(4, 5, l + 24); // c < 'a' -> next
    a.movi(5, 0x7A);
    a.bltu(5, 4, l + 24); // c > 'z' -> next
    a.addi(4, 4, -32);
    a.s8i(4, 3, 0);
    a.addi(3, 3, 1); // next
    a.j(l);
    a.retw(); // done
    pad_to(&mut a, ABS_BODY);
    debug_assert_eq!(a.pc(), ABS_BODY);
    // abs(v=a2) / labs: |v| = (v ^ mask) - mask, mask = v >> 31 (arith).
    a.movi(3, 31);
    a.ssr(3);
    a.sra(3, 2);
    a.xor(2, 2, 3);
    a.sub(2, 2, 3);
    a.retw();
    pad_to(&mut a, DIV_BODY);
    debug_assert_eq!(a.pc(), DIV_BODY);
    // div(num=a2, den=a3) / ldiv: div_t {quot=a2, rem=a3}.
    a.rems(5, 2, 3); // rem
    a.quos(2, 2, 3); // quot
    a.or(3, 5, 5);
    a.retw();
    pad_to(&mut a, RAND_R_BODY);
    debug_assert_eq!(a.pc(), RAND_R_BODY);
    // rand_r(seedp=a2): newlib LCG *s = *s * 1103515245 + 12345, return
    // (*s & 0x7fffffff).
    a.l32i(3, 2, 0);
    a.li(4, 1103515245);
    a.mull(3, 3, 4);
    a.li(4, 12345);
    a.add(3, 3, 4);
    a.s32i(3, 2, 0);
    a.li(4, 0x7FFF_FFFF);
    a.and(3, 3, 4);
    a.or(2, 3, 3);
    a.retw();
    pad_to(&mut a, RAND_BODY);
    debug_assert_eq!(a.pc(), RAND_BODY);
    // rand(): same LCG over a host-invented DRAM seed word 0x3FC87F2C
    // (next to sbrk's counter; zeroed at boot — first value 12345).
    a.li(3, 0x3FC8_7F2C);
    a.l32i(2, 3, 0);
    a.li(4, 1103515245);
    a.mull(2, 2, 4);
    a.li(4, 12345);
    a.add(2, 2, 4);
    a.s32i(2, 3, 0);
    a.li(4, 0x7FFF_FFFF);
    a.and(2, 2, 4);
    a.retw();
    pad_to(&mut a, SRAND_BODY);
    debug_assert_eq!(a.pc(), SRAND_BODY);
    // srand(seed=a2) -> the same seed word.
    a.li(3, 0x3FC8_7F2C);
    a.s32i(2, 3, 0);
    a.retw();
    pad_to(&mut a, UTOA_BODY);
    debug_assert_eq!(a.pc(), UTOA_BODY);
    // utoa(value=a2, buf=a3, radix=a4) -> buf.  Digits are generated
    // backwards (remu/quou) then reversed in place; lowercase letters
    // ('a'+d-10) like newlib.  The entry + frame save is shared with itoa
    // (ITOA jumps to UTOA_LOOP after its own entry + save).
    a.entry(1, 16);
    a.s32i(3, 1, 0); // [0] = buf start
    a.or(5, 2, 2); // UTOA_LOOP: v
    a.or(6, 3, 3); // p
    let lp = a.pc();
    a.remu(7, 5, 4); // d = v % radix
    a.quou(5, 5, 4); // v /= radix
    a.movi(2, 10);
    a.bltu(7, 2, lp + 18); // d < 10 -> small
    a.addi(7, 7, 0x57); // 'a' - 10
    a.j(lp + 21); // -> store (s8i at lp+21)
    a.addi(7, 7, 0x30); // small
    a.s8i(7, 6, 0); // store
    a.addi(6, 6, 1);
    a.bnez(5, lp);
    a.movi(2, 0);
    a.s8i(2, 6, 0); // NUL
    a.l32i(3, 1, 0); // start
    a.addi(6, 6, -1); // end = p - 1
    let rl = a.pc();
    a.bgeu(3, 6, rl + 24); // start >= end -> rdone (l32i a2,[0])
    a.l8ui(4, 3, 0);
    a.l8ui(7, 6, 0);
    a.s8i(7, 3, 0);
    a.s8i(4, 6, 0);
    a.addi(3, 3, 1);
    a.addi(6, 6, -1);
    a.j(rl);
    a.l32i(2, 1, 0); // rdone: return buf
    a.retw();
    pad_to(&mut a, ITOA_BODY);
    debug_assert_eq!(a.pc(), ITOA_BODY);
    // itoa(value=a2, buf=a3, radix=a4) -> buf: '-' prefix for negatives,
    // then the shared utoa digit loop (UTOA_LOOP expects buf in a3).
    a.entry(1, 16);
    a.s32i(3, 1, 0); // [0] = buf
    a.li(5, 0x8000_0000u32 as i32);
    a.bltu(5, 2, a.pc() + 18); // value >= 0 -> utoa loop
    a.movi(5, 0x2D); // '-'
    a.s8i(5, 3, 0);
    a.addi(3, 3, 1);
    a.movi(5, 0);
    a.sub(2, 5, 2);
    a.j(UTOA_BODY + 6); // UTOA_LOOP
    pad_to(&mut a, ATOI_BODY);
    debug_assert_eq!(a.pc(), ATOI_BODY);
    // atoi/atol(s=a2) -> long: strtol(s, NULL, 10) subset (no endptr, no
    // overflow clamp).  v = v*10 + digit via (v<<3)+(v<<1).
    a.movi(3, 0); // result
    a.movi(4, 0); // sign flag
    let skip = a.pc();
    a.l8ui(5, 2, 0); // c
    a.movi(6, 9);
    a.bltu(5, 6, skip + 30); // c < '\t' -> not_ws
    a.movi(6, 13);
    a.bltu(6, 5, skip + 18); // c > '\r' -> try space
    a.j(skip + 24); // -> is_ws
    a.movi(6, 32); // try space
    a.bne(5, 6, skip + 30); // != ' ' -> not_ws
    a.addi(2, 2, 1); // is_ws
    a.j(skip);
    a.movi(6, 0x2B); // not_ws: '+'
    a.bne(5, 6, skip + 39); // -> try minus
    a.addi(2, 2, 1);
    a.j(skip + 54); // -> digits
    a.movi(6, 0x2D); // try minus
    a.bne(5, 6, skip + 54); // -> digits
    a.movi(4, 1);
    a.addi(2, 2, 1);
    let digits = a.pc(); // skip + 54
    a.l8ui(5, 2, 0);
    a.movi(6, 0x30);
    a.bltu(5, 6, skip + 93); // c < '0' -> done
    a.movi(6, 0x39);
    a.bltu(6, 5, skip + 93); // c > '9' -> done
    a.slli(7, 3, 3); // v*8
    a.slli(6, 3, 1); // v*2
    a.add(3, 7, 6); // v*10
    a.movi(6, 0x30);
    a.sub(5, 5, 6); // c - '0'
    a.add(3, 3, 5);
    a.addi(2, 2, 1);
    a.j(digits);
    let done = a.pc(); // skip + 93
    a.beqz(4, done + 9); // sign == 0 -> pos
    a.movi(5, 0);
    a.sub(3, 5, 3);
    a.or(2, 3, 3); // pos
    a.retw();
    pad_to(&mut a, STRTOL_BODY);
    debug_assert_eq!(a.pc(), STRTOL_BODY);
    // strtol(nptr=a2, endptr=a3, base=a4) / strtoul (unsigned: only the
    // overflow clamp differs, which we don't model): full base 0/2..36
    // parsing with the 0x/0 prefix detection, sign, and *endptr.  v*base
    // via a generic shift-add multiply (any base).  Frame: [0]=endptr,
    // [4]=base, [8]=sign, [12]=p.
    a.entry(1, 16);
    a.s32i(3, 1, 0); // endptr
    a.s32i(4, 1, 4); // base
    a.movi(3, 0); // v
    a.movi(5, 0); // sign
    let skip = a.pc();
    a.l8ui(6, 2, 0); // c
    a.movi(7, 9);
    a.bltu(6, 7, skip + 30); // -> not_ws
    a.movi(7, 13);
    a.bltu(7, 6, skip + 18); // -> try space
    a.j(skip + 24); // -> is_ws
    a.movi(7, 32); // try space
    a.bne(6, 7, skip + 30); // -> not_ws
    a.addi(2, 2, 1); // is_ws
    a.j(skip);
    a.movi(7, 0x2B); // not_ws: '+'
    a.bne(6, 7, skip + 39); // -> try minus
    a.addi(2, 2, 1);
    a.j(skip + 54); // -> after_sign
    a.movi(7, 0x2D); // try minus
    a.bne(6, 7, skip + 54); // -> after_sign
    a.movi(5, 1);
    a.addi(2, 2, 1);
    let after = a.pc(); // skip + 54
    a.l32i(4, 1, 4);
    a.bnez(4, after + 60); // base != 0 -> digits_start
    a.l8ui(6, 2, 0); // base 0: detect prefix
    a.movi(7, 0x30);
    a.bne(6, 7, after + 57); // != '0' -> base10
    a.l8ui(6, 2, 1);
    a.movi(7, 0x78); // 'x'
    a.bne(6, 7, after + 33); // -> try X
    a.addi(2, 2, 2);
    a.movi(4, 16);
    a.j(after + 60); // -> digits_start
    a.movi(7, 0x58); // try X
    a.bne(6, 7, after + 48); // -> base8
    a.addi(2, 2, 2);
    a.movi(4, 16);
    a.j(after + 60);
    a.addi(2, 2, 1); // base8
    a.movi(4, 8);
    a.j(after + 60);
    a.movi(4, 10); // base10
    let digits = a.pc(); // after + 60
    a.l8ui(6, 2, 0); // c
    a.movi(7, 0x30);
    a.bltu(6, 7, digits + 135); // c < '0' -> done
    a.movi(7, 0x39);
    a.bltu(7, 6, digits + 24); // c > '9' -> try_lo
    a.movi(7, 0x30);
    a.sub(7, 6, 7); // d = c - '0'
    a.j(digits + 69); // -> check_base
    a.movi(7, 0x61); // try_lo: 'a'
    a.bltu(6, 7, digits + 48); // -> try_hi
    a.movi(7, 0x7A); // 'z'
    a.bltu(7, 6, digits + 135); // -> done
    a.movi(7, 0x61);
    a.sub(7, 6, 7);
    a.addi(7, 7, 10);
    a.j(digits + 69); // -> check_base
    a.movi(7, 0x41); // try_hi: 'A'
    a.bltu(6, 7, digits + 135); // -> done
    a.movi(7, 0x5A); // 'Z'
    a.bltu(7, 6, digits + 135); // -> done
    a.movi(7, 0x41);
    a.sub(7, 6, 7);
    a.addi(7, 7, 10);
    a.bltu(7, 4, digits + 75); // check_base: d < base -> acc
    a.j(digits + 135); // -> done
    // acc: v = v*base + d (shift-add multiply over the base's set bits;
    // d survives in a7 — the loop only clobbers a2/a4/a5).
    a.s32i(5, 1, 8); // sign
    a.s32i(2, 1, 12); // p
    a.or(5, 4, 4); // m = base
    a.or(2, 3, 3); // v -> a2
    a.movi(3, 0); // acc
    let mul = a.pc();
    a.beqz(5, mul + 27); // -> mul_done
    a.movi(4, 1);
    a.and(4, 5, 4);
    a.beqz(4, mul + 12); // -> shift
    a.add(3, 3, 2); // acc += v
    a.movi(4, 1); // shift
    a.ssr(4);
    a.srl(5, 5); // m >>= 1
    a.slli(2, 2, 1); // v <<= 1
    a.j(mul);
    a.l32i(2, 1, 12); // mul_done: p
    a.l32i(5, 1, 8); // sign
    a.l32i(4, 1, 4); // base
    a.add(3, 3, 7); // v = v*base + d
    a.addi(2, 2, 1); // p++
    a.j(digits);
    let done = a.pc();
    a.l32i(4, 1, 0); // endptr
    a.beqz(4, done + 9); // NULL -> skip store
    a.s32i(2, 4, 0); // *endptr = p
    a.beqz(5, done + 18); // sign == 0 -> pos
    a.movi(6, 0);
    a.sub(3, 6, 3); // v = -v
    a.or(2, 3, 3); // pos
    a.retw();

    // Splice: our proven boot glue replaces the real ROM up to the first
    // real API slot (0x40000570); everything from 0x570 on stays real.
    const GLUE_END: usize = 0x570;
    debug_assert!(a.bytes().len() >= GLUE_END, "glue covers the splice region");
    rom[..GLUE_END].copy_from_slice(&a.bytes()[..GLUE_END]);
    rom
}
