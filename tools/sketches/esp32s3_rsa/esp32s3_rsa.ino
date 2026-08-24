// ESP32-S3 RSA hardware accelerator validation sketch (raw RSA primitive).
// Expected C = pow(M, E, N):
// 6777730cf4bd1a654bff5366905b501dff131769e26b1e20794702ba2d462f9384aef81f50589dfe2e88a40774eccc557d2291294e51e0ad68cdc1f87a0beba2fae2dddb0a459f77375367ca538abf789a1c1056e6c07c73f804502b999b592429357cce7a4eb6d1c379e5de9e260fab26c8ef73a17cad6428620d10cfa30fad

#include <Arduino.h>
#include <mbedtls/rsa.h>

static const char *N_hex = "966aa3d25836d3f0e39be1afcb5bb44afd6cf557e14ef58726c38dfd8a94a237e176fa79af54fa1db4eeba7de43cb70b39122c06f21108f8ccbbade2bf970a87d906732bcda617993b75defcbf6395420d55f2c3e614b7ce05048dbc9a1058e1c6b8746d2cd3a5415de18d5ca0c044f3bea635bc1f7b9b33028c6c0ec77fa769";
static const char *P_hex = "f637bfe52626ff63c3ad204a01908298fc631e7e885f290cd845151d809891a04ad13d9e9d57b758520c8d8297c86e54215dfb298d91f7318d1e9d2ecf4f0c9b";
static const char *Q_hex = "9c64821435b4254d11d1ea2e7af03362412d70b698f19ecdadef91edf80582b01ac1afee132847b415d89f0a9589b067720c3b1e9da79935c33b63be0a35424b";
static const char *D_hex = "932b3fa23cc15858e6b9cbf56e69095c1ddd0fa7ae40cd26311d40be036b2dd4b2faf05342e347dcecfc6ee761faadb5835f6e48556ba975950b443508f3c54e5f72e333d1f699f81c5b86cee98a849f0a06bd4420ecfcc7271a4f429bd0eb99ba8d370c4958a6d446a16a2f4f674469cc93657b944f51b4bf636942c75016e9";
static const char *E_hex = "010001";
static const char *M_hex = "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcd";

static int hex2bin(const char *hex, uint8_t *out, size_t outlen) {
  size_t len = strlen(hex);
  if (len != outlen * 2) return -1;
  for (size_t i = 0; i < outlen; i++) {
    unsigned v = 0;
    if (sscanf(hex + i * 2, "%2x", &v) != 1) return -1;
    out[i] = (uint8_t)v;
  }
  return 0;
}

static void print_hex(const char *tag, const uint8_t *buf, size_t len) {
  Serial.print(tag);
  for (size_t i = 0; i < len; i++) Serial.printf("%02x", buf[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  uint8_t N[128], P[64], Q[64], D[128], E[3], M[128];
  if (hex2bin(N_hex, N, 128) || hex2bin(P_hex, P, 64) || hex2bin(Q_hex, Q, 64) ||
      hex2bin(D_hex, D, 128) || hex2bin(E_hex, E, 3) || hex2bin(M_hex, M, 128)) {
    Serial.println("HEX PARSE FAIL");
    return;
  }
  mbedtls_rsa_context rsa;
  mbedtls_rsa_init(&rsa);
  int ret = mbedtls_rsa_import_raw(&rsa, N, 128, P, 64, Q, 64, D, 128, E, 3);
  if (ret != 0) { Serial.printf("RSA IMPORT FAIL ret=%d\n", ret); return; }
  ret = mbedtls_rsa_complete(&rsa);
  if (ret != 0) { Serial.printf("RSA COMPLETE FAIL ret=%d\n", ret); return; }
  uint8_t C[128], M2[128];
  ret = mbedtls_rsa_public(&rsa, M, C);
  if (ret != 0) { Serial.printf("RSA PUB FAIL ret=%d\n", ret); return; }
  print_hex("RSA CT=", C, 128);
  ret = mbedtls_rsa_private(&rsa, NULL, NULL, C, M2);
  if (ret != 0) { Serial.printf("RSA PRIV FAIL ret=%d\n", ret); return; }
  print_hex("RSA PT=", M2, 128);
  bool ok = (memcmp(M, M2, 128) == 0);
  Serial.println(ok ? "RSA ROUNDTRIP PASS" : "RSA ROUNDTRIP FAIL");
  Serial.println("RSA DONE");
}

void loop() {}
