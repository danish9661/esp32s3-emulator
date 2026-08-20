#!/usr/bin/env bash
# Regenerate crates/esp32s3-emu/src/softfloat.bin — the ROM-stub soft-float
# double suite (IEEE-754 binary64 add/sub/mul/div + conversions + compares,
# see softfloat.c).  Compiled with the esp32s3 toolchain and linked at
# 0x40002600 (the stub's float-body region); the fixed-address ROM slots in
# rom_stub.rs (0x40002184..0x4000258C) jump into it.  The linker script
# resolves the __ashldi3/__ashrdi3/__lshrdi3/__udivdi3/__umoddi3 calls to the
# stub's integer slots; the C code currently avoids them entirely (shl64/
# shr64/mul64 on 32-bit halves) because the Xtensa linker misresolves direct
# l32rs against absolute symbols.
set -euo pipefail
TOOL=~/.arduino15/packages/esp32/tools/esp-x32/2601/bin
SRC=$(dirname "$0")
CC="$TOOL/xtensa-esp-elf-gcc"
LD="$TOOL/xtensa-esp-elf-ld"
OBJCOPY="$TOOL/xtensa-esp-elf-objcopy"
OBJDUMP="$TOOL/xtensa-esp-elf-objdump"
OUT="$(dirname "$0")/../crates/esp32s3-emu/src/softfloat"
"$CC" -O2 -ffunction-sections -fno-reorder-functions -mtext-section-literals \
    -fno-builtin -c "$SRC/softfloat.c" -o "$OUT.o"
"$LD" -T "$SRC/softfloat.ld" "$OUT.o" -o "$OUT.elf"
"$OBJCOPY" -O binary "$OUT.elf" "$OUT.bin"
"$OBJDUMP" -d "$OUT.elf" > "$OUT.dis"
echo "wrote $OUT.bin ($(stat -c%s "$OUT.bin") bytes)"
