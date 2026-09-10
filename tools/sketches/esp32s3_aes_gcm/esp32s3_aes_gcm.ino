// AES-GCM validation (NIST zero-vector) for the ESP32-S3 emulator.
// mbedTLS GCM runs GHASH in software over the AES-ECB hardware block,
// so this exercises the HW-ECB path plus the DMA engine end to end.
// Key=0/IV=0/PT=0(16) -> CT=0388dace60b6a392f328c2b971b2fe78,
// Tag=ab6e47d42cec13bdf53a67b21257bddf (NIST Case 2, cross-checked
// against PyCryptodome).
#include "mbedtls/gcm.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  uint8_t key[16] = {0};
  uint8_t iv[12] = {0};
  uint8_t pt[16] = {0};
  uint8_t ct[16] = {0};
  uint8_t tag[16] = {0};
  mbedtls_gcm_context ctx;
  mbedtls_gcm_init(&ctx);
  int rc = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, 128);
  if (rc != 0) {
    Serial.print("AES GCM SETKEY FAIL ");
    Serial.println(rc);
    Serial.println("AES GCM FAIL");
    return;
  }
  rc = mbedtls_gcm_crypt_and_tag(&ctx, MBEDTLS_GCM_ENCRYPT, sizeof(pt), iv,
                                 sizeof(iv), NULL, 0, pt, ct, sizeof(tag), tag);
  mbedtls_gcm_free(&ctx);
  if (rc != 0) {
    Serial.print("AES GCM CRYPT FAIL ");
    Serial.println(rc);
    Serial.println("AES GCM FAIL");
    return;
  }
  const char *want_ct = "0388dace60b6a392f328c2b971b2fe78";
  const char *want_tag = "ab6e47d42cec13bdf53a67b21257bddf";
  char got_ct[33], got_tag[33];
  for (int i = 0; i < 16; i++) {
    sprintf(got_ct + 2 * i, "%02x", ct[i]);
    sprintf(got_tag + 2 * i, "%02x", tag[i]);
  }
  got_ct[32] = 0;
  got_tag[32] = 0;
  Serial.print("AES GCM CT=");
  Serial.println(got_ct);
  Serial.print("AES GCM TAG=");
  Serial.println(got_tag);
  if (strcmp(got_ct, want_ct) == 0 && strcmp(got_tag, want_tag) == 0) {
    Serial.println("AES GCM PASS");
  } else {
    Serial.println("AES GCM FAIL");
  }
  Serial.println("AES GCM DONE");
}

void loop() {
  delay(1000);
}
