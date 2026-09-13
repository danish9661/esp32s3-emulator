// ESP32-S3 Secure Boot v2 validation for the emulator.
//
// Boots a genuinely `espsecure.py sign-data`-signed hello image with
// eFuse SECURE_BOOT_EN burned (the `run_flash` SECURE_BOOT_EN=1 fixture
// burns it through the real PGM path before boot, so `boot_from_flash`
// verifies the app region's signature sector fail-closed). The signed
// image is built by tools/build_secure_boot.sh (signs the hello app
// binary with a local ECDSA-P256 dev key, reassembles the merged flash
// image); the sketch source is the plain hello app — the SIGNING, not
// the firmware, is what this validates. Prints the standard hello
// markers so the battery asserts a real boot, not just gate passage.
//
// Unsigned images with the gate armed park both CPUs with no output
// (covered by the `secure_boot_enabled_denies_boot` machine test).

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Hello from ESP32-S3!");
  Serial.println("boot OK");
}

void loop() {
  delay(1000);
}
