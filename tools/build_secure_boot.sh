#!/bin/bash
# Build + sign the Secure Boot v2 validation image.
# Signs the hello app binary with a local ECDSA-P256 dev key
# (espsecure.py sign-data --version 2), then reassembles the merged flash
# image (bootloader @0, partitions @0x8000, boot_app0 @0xe000, SIGNED app
# @0x10000). The dev key is test-only (NOT a production key) and may be
# regenerated at any time — what matters is the sign/verify round-trip
# through the emulator's ECDSA model, not the key identity.
# Usage: tools/build_secure_boot.sh
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SK="$ROOT/tools/sketches"
FQBN="esp32:esp32:esp32s3"
HELLO="$SK/esp32s3_hello"
OUT="$SK/esp32s3_secure_boot"
KEY="$OUT/secure_boot_signing_key.pem"
echo "== secure-boot: build hello app"
arduino-cli compile --fqbn "$FQBN" --build-path "$HELLO/build" "$HELLO" >/dev/null
BLD="$HELLO/build"
if [[ ! -f "$KEY" ]]; then
  echo "== secure-boot: generate dev signing key"
  espsecure generate-signing-key --version 2 --scheme ecdsa256 "$KEY" >/dev/null
fi
echo "== secure-boot: sign app binary"
espsecure sign-data --version 2 --keyfile "$KEY" \
  --output "$OUT/app_signed.bin" "$BLD/esp32s3_hello.ino.bin" >/dev/null
echo "== secure-boot: reassemble merged image"
python3 - "$BLD/esp32s3_hello.ino.bootloader.bin" "$BLD/esp32s3_hello.ino.partitions.bin" \
  "$BLD/boot_app0.bin" "$OUT/app_signed.bin" "$OUT/esp32s3_secure_boot.merged.bin" << 'EOF'
import sys
bl = open(sys.argv[1], 'rb').read()
part = open(sys.argv[2], 'rb').read()
b0 = open(sys.argv[3], 'rb').read()
app = open(sys.argv[4], 'rb').read()
img = bytearray(b'\xff' * 0x400000)
img[0x0:0x0 + len(bl)] = bl
img[0x8000:0x8000 + len(part)] = part
img[0xe000:0xe000 + len(b0)] = b0
img[0x10000:0x10000 + len(app)] = app
assert img[0x10000] == 0xE9, "app magic missing after reassemble"
open(sys.argv[5], 'wb').write(bytes(img))
print('reassembled %d bytes (signed app %d bytes at 0x10000)' % (len(img), len(app)))
EOF
espsecure signature-info-v2 "$OUT/esp32s3_secure_boot.merged.bin" | head -n 3
echo "== secure-boot image ready: $OUT/esp32s3_secure_boot.merged.bin"
