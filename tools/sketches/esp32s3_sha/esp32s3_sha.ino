// ESP32-S3 SHA-256 hardware accelerator validation sketch.
// Computes SHA-256 of a known string via mbedtls (which routes through the
// real esp-idf SHA peripheral driver) and prints the hex digest so the
// emulator's SHA model can be asserted against the known value.
#include <Arduino.h>
#include <mbedtls/sha256.h>

void setup() {
  Serial.begin(115200);
  const char* msg = "hello";
  uint8_t out[32];
  // last arg 0 => SHA-256 (1 => SHA-224)
  mbedtls_sha256((const uint8_t*)msg, 5, out, 0);
  for (int i = 0; i < 32; i++) {
    Serial.printf("%02x", out[i]);
  }
  Serial.println();
  Serial.println("SHA DONE");
}

void loop() {}
