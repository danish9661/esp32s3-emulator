// ESP32-S3 SHA hardware accelerator validation sketch.
// Computes SHA-256/384/512 digests via mbedtls (which routes through the
// real esp-idf SHA peripheral driver) and prints the hex digests so the
// emulator's SHA model can be asserted against known values.
#include <Arduino.h>
#include <mbedtls/sha256.h>
#include <mbedtls/sha512.h>
#include <string.h>

static void print_hex(const uint8_t* p, int n) {
  for (int i = 0; i < n; i++) {
    Serial.printf("%02x", p[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  const char* msg = "hello";
  uint8_t out[64];

  // SHA-256 (last arg 0 => SHA-256, 1 => SHA-224).
  uint8_t out256[32];
  mbedtls_sha256((const uint8_t*)msg, 5, out256, 0);
  print_hex(out256, 32);
  Serial.println("SHA DONE");

  // SHA-384 (last arg 1) of "hello".
  mbedtls_sha512((const uint8_t*)msg, 5, out, 1);
  print_hex(out, 48);
  static const char* want384 =
      "59e1748777448c69de6b800d7a33bbfb9ff1b463e44354c3553bcdb9c666fa90125a3c79f90397bdf5f6a13de828684f";
  char got384[97];
  for (int i = 0; i < 48; i++) {
    sprintf(got384 + 2 * i, "%02x", out[i]);
  }
  got384[96] = 0;
  Serial.println(strcmp(got384, want384) == 0 ? "SHA384 PASS" : "SHA384 FAIL");

  // SHA-512 (last arg 0) of "hello".
  mbedtls_sha512((const uint8_t*)msg, 5, out, 0);
  print_hex(out, 64);
  static const char* want512 =
      "9b71d224bd62f3785d96d46ad3ea3d73319bfbc2890caadae2dff72519673ca723"
      "23c3d99ba5c11d7c7acc6e14b8c5da0c4663475c2e5c3adef46f73bcdec043";
  char got512[129];
  for (int i = 0; i < 64; i++) {
    sprintf(got512 + 2 * i, "%02x", out[i]);
  }
  got512[128] = 0;
  Serial.println(strcmp(got512, want512) == 0 ? "SHA512 PASS" : "SHA512 FAIL");
}

void loop() {}
