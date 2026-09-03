// ESP32-S3 AES hardware accelerator validation sketch (ECB, FIPS PUB 197 vector).
// AES-128 ECB: key = 000102030405060708090a0b0c0d0e0f,
//              pt  = 00112233445566778899aabbccddeeff,
//              ct  = 69c4e0d86a7b0430d8cdb78070b4c55a
#include <Arduino.h>
#include <mbedtls/aes.h>

void setup() {
  Serial.begin(115200);
  const uint8_t key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
  const uint8_t in[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                          0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
  uint8_t out[16];
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  mbedtls_aes_setkey_enc(&ctx, key, 128);
  mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, in, out);
  for (int i = 0; i < 16; i++) {
    Serial.printf("%02x", out[i]);
  }
  Serial.println();
  Serial.println("AES DONE");

  // AES-128-CBC via the real driver (NIST SP 800-38A F.2.1, all 4 blocks):
  // exercises multi-block CBC chaining (IV + feedback across blocks) end to end.
  {
    const uint8_t cbc_key[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                                 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
    const uint8_t iv[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                            0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
    const uint8_t pt[64] = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
                            0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
                            0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
                            0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
                            0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
                            0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
                            0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
                            0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10};
    uint8_t ivcopy[16], ct[64];
    memcpy(ivcopy, iv, 16);
    mbedtls_aes_context c2;
    mbedtls_aes_init(&c2);
    mbedtls_aes_setkey_enc(&c2, cbc_key, 128);
    mbedtls_aes_crypt_cbc(&c2, MBEDTLS_AES_ENCRYPT, 64, ivcopy, pt, ct);
    mbedtls_aes_free(&c2);
    for (int i = 0; i < 64; i++) {
      Serial.printf("%02x", ct[i]);
    }
    Serial.println();
    static const char *want =
        "7649abac8119b246cee98e9b12e9197d"
        "5086cb9b507219ee95db113a917678b2"
        "73bed6b8e3c1743b7116e69e22229516"
        "3ff1caa1681fac09120eca307586e1a7";
    char got[129];
    for (int i = 0; i < 64; i++) {
      sprintf(got + 2 * i, "%02x", ct[i]);
    }
    got[128] = 0;
    Serial.println(strcmp(got, want) == 0 ? "AES CBC PASS" : "AES CBC FAIL");
  }

  // AES-128-XTS via the real driver (2 blocks, zero tweak). The esp-idf XTS
  // path is software over the HW block cipher (no XTS block mode in the
  // peripheral: esp_aes_crypt_xts calls esp_aes_crypt_ecb per block and
  // advances the tweak with esp_gf128mul_x_ble, the byte-reversed-α variant
  // — hence block 1 differs from the BE-α textbook value), so this exercises
  // the ECB block path with tweak chaining.
  {
    const uint8_t key[32] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                             0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
                             0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                             0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
    const uint8_t pt[32] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                            0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
                            0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
                            0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00};
    const uint8_t tweak[16] = {0};
    uint8_t ct[32];
    mbedtls_aes_xts_context xctx;
    mbedtls_aes_xts_init(&xctx);
    int rc = mbedtls_aes_xts_setkey_enc(&xctx, key, 256);
    if (rc == 0) {
      rc = mbedtls_aes_crypt_xts(&xctx, MBEDTLS_AES_ENCRYPT, 32, tweak, pt, ct);
    }
    mbedtls_aes_xts_free(&xctx);
    for (int i = 0; i < 32; i++) {
      Serial.printf("%02x", ct[i]);
    }
    Serial.println();
    static const char* want =
        "171c69724dcf733f9aa6317d795153e4"
        "0f46d50a7bad5aa2a36c3a14bb4617d5";
    char got[65];
    for (int i = 0; i < 32; i++) {
      sprintf(got + 2 * i, "%02x", ct[i]);
    }
    got[64] = 0;
    Serial.println(
        (rc == 0 && strcmp(got, want) == 0) ? "AES XTS PASS" : "AES XTS FAIL");
  }
}

void loop() {}
