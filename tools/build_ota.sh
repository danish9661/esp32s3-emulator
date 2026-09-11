#!/bin/bash
# Two-pass build for the end-to-end OTA test.
# Pass 1 builds the slot-1 sketch; pass 2 embeds its raw app binary into
# the updater sketch and builds the bootable (merged) test image.
# Usage: tools/build_ota.sh   (runs arduino-cli, needs esp32 core 3.x)
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SK="$ROOT/tools/sketches"
FQBN="esp32:esp32:esp32s3"
echo "== OTA pass 1: slot-1 image"
arduino-cli compile --fqbn "$FQBN" --build-path "$SK/esp32s3_ota_slot1/build" "$SK/esp32s3_ota_slot1"
echo "== OTA pass 2: embed + build updater"
python3 - "$SK/esp32s3_ota_slot1/build/esp32s3_ota_slot1.ino.bin" \
  "$SK/esp32s3_ota_update/ota_slot1_image.h" << 'EOF'
import sys
data = open(sys.argv[1], 'rb').read()
out = ['#pragma once', '#include <stdint.h>', '',
       'static const uint8_t ota_slot1_bin[] = {']
for i in range(0, len(data), 12):
    out.append('  ' + ', '.join('0x%02x' % b for b in data[i:i+12]) + ',')
out.append('};')
out.append('static const unsigned int ota_slot1_bin_len = %d;' % len(data))
open(sys.argv[2], 'w').write('\n'.join(out) + '\n')
print('embedded %d bytes' % len(data))
EOF
arduino-cli compile --fqbn "$FQBN" --build-path "$SK/esp32s3_ota_update/build" "$SK/esp32s3_ota_update"
cp "$SK/esp32s3_ota_update/build/esp32s3_ota_update.ino.merged.bin" \
  "$SK/esp32s3_ota_update/esp32s3_ota_update.merged.bin"
cp "$SK/esp32s3_ota_update/build/esp32s3_ota_update.ino.elf" \
  "$SK/esp32s3_ota_update/esp32s3_ota_update.ino.elf" 2>/dev/null || true
cp "$SK/esp32s3_ota_slot1/build/esp32s3_ota_slot1.ino.elf" \
  "$SK/esp32s3_ota_slot1/esp32s3_ota_slot1.ino.elf" 2>/dev/null || true
echo "== OTA image ready: tools/sketches/esp32s3_ota_update/esp32s3_ota_update.merged.bin"
