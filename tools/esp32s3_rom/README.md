# ESP32-S3 boot ROM images (Apache-2.0)

Source: `esp32s3_rev0_rom.elf` from the Espressif `esp-rom-elfs` release
`20260528` (https://github.com/espressif/esp-rom-elfs/releases).

The ROM source is closed, but Espressif publishes the ROM binaries under
Apache License 2.0 (see the repo LICENSE + "Copyrights and License" section
of the README). Apache-2.0 is GPLv3-compatible, so vendoring these blobs
keeps the project's GPL-compatible outcome posture.

Extracted with the toolchain's `xtensa-esp32s3-elf-readelf` (section
table -> per-section bytes from file offsets):

- `esp32s3_rom.bin` (384 KB): the ROM text at 0x4000_0000-0x4005_FFFF
  (window/exception vectors, `__call_*` wrapper table at 0x4000_0570+,
  newlib/libgcc bodies at 0x4002_0000+). Zero-padded gaps.
- `esp32s3_rom_rodata.bin` (32 KB): the ROM's read-only tables at
  0x3FF1_8000-0x3FF2_0000 (RTC fast memory data window; includes
  ets_rom_layout_p's target at 0x3FF1_AE90). Pre-loaded into the RTC fast
  storage (writable, like silicon).
- `esp32s3_rom_data.bin` (0x28200 bytes): the ROM's `.data` init values at
  0x3FCD_7E00-0x3FCF_0000 (XTOS tables, spi_flash driver data, PRO/APP
  stacks, shared buffers; `.bss` is zeroed by the loader).

The emulator overlays its own boot glue on top of the ROM text: the window
vectors (0x4000_0000-0x4000_0170), the reset vector + segment loader
(0x4000_0400-0x4000_0451), the core-1 release spin (0x4000_0480), the
boot printf (0x4000_0500) and the host printf body (0x4000_0520) — see
`crates/esp32s3-emu/src/rom_stub.rs`.
