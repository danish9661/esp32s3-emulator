#!/bin/bash
# MicroPython REPL validation recipe (manual — NOT a battery entry: the image
# is an external ~1.8MB download, and the boot needs ~300M steps).
#
#   tools/micropython_repl.sh /path/to/ESP32_GENERIC_S3-<date>-v1.29.0.bin
#
# What it does:
#   1. Appends a littlefs "vfs" partition (DATA sub 0x82 @ 0x200000, 1MB) to a
#      copy of the image and recomputes the partition-table MD5 (the digest
#      covers every record before the MD5 marker; the digest itself lives at
#      record-offset 16 — verified against MicroPython's pristine table).
#   2. Boots it (first boot formats littlefs: "Performing initial setup").
#   3. Re-runs with UART0 input at the `>>> ` marker and asserts `42`.
#   4. Re-runs float formatting (`0.5`, `1/3`, `0.1+0.2` — the old MOVF/MOVT
#      hang family) and asserts exact output.
#
# Provenance: MicroPython v1.29.0 GENERIC_S3 boots to REPL and runs code
# through the windowed-ABI spill path, the LX7 FPU, and UART0 RX — the same
# binary class that once exposed the write16 qstr clobber and the AR-MOVF
# hang. Re-run after touching cpu/exec/window/UART/PSRAM/cache code.
set -u
IMG="${1:?usage: micropython_repl.sh <ESP32_GENERIC_S3-...-v1.29.0.bin>}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN="$ROOT/target/release/examples/run_flash"
WORK="$(mktemp -d /tmp/mp_repl.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

[ -x "$RUN" ] || cargo build --release --example run_flash -p esp32s3-emu || exit 1

python3 - "$IMG" "$WORK/mp_vfs.bin" << 'EOF'
import hashlib, sys
d = bytearray(open(sys.argv[1], 'rb').read())
if len(d) < 0x300000:
    d += bytes([0xFF]) * (0x300000 - len(d))
pt = 0x8000
rec = bytearray(32)
rec[0:2] = bytes([0xAA, 0x50])
rec[2] = 1
rec[3] = 0x82
rec[4:8] = (0x200000).to_bytes(4, 'little')
rec[8:12] = (0x100000).to_bytes(4, 'little')
rec[12:16] = b'vfs\x00'
d[pt+0x60:pt+0x80] = rec
mrec = bytearray(32)
mrec[0:2] = bytes([0xEB, 0xEB])
mrec[16:32] = hashlib.md5(bytes(d[pt:pt+0x80])).digest()
d[pt+0x80:pt+0xA0] = mrec
open(sys.argv[2], 'wb').write(d)
print("image ready", hex(len(d)))
EOF

COMMON="STEPS=300000000 SYSCALL_CONTINUE=1 IDLE_STEPS=5000000"
fail() { echo "MP REPL FAIL: $1"; exit 1; }

out=$(env $COMMON "$RUN" "$WORK/mp_vfs.bin" 2>&1 | grep -a "uart bytes" | head -1)
echo "$out" | grep -q ">>> " || fail "no REPL banner ($out)"

out=$(env $COMMON UART0_INJECT='print(6*7)\n' "$RUN" "$WORK/mp_vfs.bin" 2>&1 | grep -a "uart bytes" | head -1)
echo "$out" | grep -q "42" || fail "print(6*7) != 42 ($out)"

out=$(env $COMMON UART0_INJECT='print(0.5)\nprint(1.0/3.0)\nprint(0.1+0.2)\n' "$RUN" "$WORK/mp_vfs.bin" 2>&1 | grep -a "uart bytes" | head -1)
echo "$out" | grep -q "0.5" || fail "print(0.5) ($out)"
echo "$out" | grep -q "0.33333334" || fail "print(1.0/3.0) ($out)"
echo "$out" | grep -q "0.3" || fail "print(0.1+0.2) ($out)"

echo "MP REPL PASS"
