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
}

void loop() {}
