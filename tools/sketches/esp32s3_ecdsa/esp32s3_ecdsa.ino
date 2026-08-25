// ESP32-S3 ECDSA (P-256) peripheral validation via direct register pokes.
// The emulator's ECDSA model implements NIST P-192/P-256 sign/verify over its
// internal curve tables. This sketch signs a fixed message hash with a fixed
// private key + nonce (software_set_k), checks the produced (R,S) against a
// known-answer vector computed by an independent Python implementation, then
// verifies the signature round-trip, and finally proves tamper detection.

#define ECDSA_BASE 0x6008E000UL
#define PARAM_BASE  (ECDSA_BASE + 0x80)
#define NW 8            // words per parameter block
#define BLK(blk) (PARAM_BASE + (blk) * NW * 4)

const uint8_t D_PRIV[32] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
};
const uint8_t Z_HASH[32] = {
  0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
  0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5,
  0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5, 0xa5
};
const uint8_t K_NONCE[32] = {
  0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51,
  0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51,
  0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51
};
const uint8_t EXP_R[32] = {
  0x9a, 0x65, 0x17, 0x3d, 0x48, 0xa0, 0xa0, 0xc7, 0x06, 0xee, 0xec, 0x2a,
  0x75, 0xae, 0x1f, 0x56, 0x79, 0x3b, 0xe9, 0x88, 0xb2, 0xd4, 0x07, 0xe9,
  0xa7, 0xd3, 0xab, 0xa7, 0x57, 0x4a, 0x9e, 0xa5
};
const uint8_t EXP_S[32] = {
  0x37, 0x3e, 0xb4, 0x12, 0x96, 0x93, 0x02, 0xe3, 0x97, 0xf8, 0xfa, 0x98,
  0x0d, 0x61, 0x05, 0x7d, 0x3f, 0x99, 0x48, 0x7e, 0xb5, 0x73, 0x91, 0xd1,
  0xcb, 0x73, 0x3f, 0x81, 0x34, 0xc9, 0xb7, 0xfe
};

static inline void wr(uint32_t a, uint32_t v) { *(volatile uint32_t*)a = v; }
static inline uint32_t rd(uint32_t a) { return *(volatile uint32_t*)a; }

// The ESP32-S3 ECDSA param memory stores a scalar with word0 = least
// significant word, and each 32-bit word holds its 4 big-endian bytes packed
// as a big-endian u32 (this is the model's limb convention, matching
// `from_be_bytes`). Convert a big-endian byte vector into that layout: word i
// gets the BE u32 of the 4 big-endian bytes at [(nwords-1-i)*4 .. ].
void write_block(uint32_t blk, const uint8_t* data, int len) {
  int nwords = len / 4;
  for (int i = 0; i < nwords; i++) {
    const uint8_t* p = &data[(nwords - 1 - i) * 4];
    uint32_t w = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                 ((uint32_t)p[2] << 8) | (uint32_t)p[3];
    wr(BLK(blk) + i * 4, w);
  }
}

// Compare the little-endian-word param block `blk` (word0 = LSW) against the
// big-endian byte array `exp`.
bool block_eq(uint32_t blk, const uint8_t* exp, int len) {
  int nwords = len / 4;
  for (int i = 0; i < nwords; i++) {
    uint32_t w = rd(BLK(blk) + i * 4);
    const uint8_t* p = &exp[(nwords - 1 - i) * 4];
    if ((uint8_t)((w >> 24) & 0xff) != p[0]) return false;
    if ((uint8_t)((w >> 16) & 0xff) != p[1]) return false;
    if ((uint8_t)((w >> 8) & 0xff) != p[2]) return false;
    if ((uint8_t)(w & 0xff) != p[3]) return false;
  }
  return true;
}

void print_block(uint32_t blk, const char* tag) {
  Serial.print(tag);
  int nwords = 8;
  for (int i = nwords - 1; i >= 0; i--) {
    uint32_t w = rd(BLK(blk) + i * 4);
    for (int b = 3; b >= 0; b--) {
      uint8_t v = (w >> (b * 8)) & 0xff;
      if (v < 0x10) Serial.print("0");
      Serial.print(v, HEX);
    }
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ECDSA VALIDATION START");

  // --- Sign: work_mode=1 (sign), ecc_curve=1 (P-256), software_set_k=1 ---
  write_block(7, D_PRIV, sizeof(D_PRIV));   // private key d
  write_block(9, Z_HASH, sizeof(Z_HASH));   // message hash z
  write_block(8, K_NONCE, sizeof(K_NONCE)); // nonce k
  wr(ECDSA_BASE + 0x00, (1 << 0) | (1 << 2) | (1 << 3));  // CONF
  wr(ECDSA_BASE + 0x04, 1);                                 // START
  uint32_t res = rd(ECDSA_BASE + 0x18);                     // RESULT
  bool r_ok = block_eq(10, EXP_R, sizeof(EXP_R));           // R block
  bool s_ok = block_eq(11, EXP_S, sizeof(EXP_S));           // S block
  Serial.print("ECDSA sign RESULT=");
  Serial.println(res);
  print_block(10, "ECDSA R=");
  print_block(11, "ECDSA S=");
  Serial.print("ECDSA sign KAT r_ok=");
  Serial.println(r_ok ? "1" : "0");
  Serial.print("ECDSA sign KAT s_ok=");
  Serial.println(s_ok ? "1" : "0");

  // --- Verify round-trip: reuse the public key (blocks 5/6) the sign stored,
  //     z (block 9), R (block 10), S (block 11); work_mode=0 verify. ---
  wr(ECDSA_BASE + 0x00, (0 << 0) | (1 << 2));  // CONF: verify, P-256
  wr(ECDSA_BASE + 0x04, 1);                     // START
  uint32_t verify_res = rd(ECDSA_BASE + 0x18); // RESULT (1 = valid)
  Serial.print("ECDSA verify round-trip RESULT=");
  Serial.println(verify_res);

  // --- Tamper: flip a bit in S, re-verify -> must fail (RESULT=0). ---
  uint32_t s0 = rd(BLK(11));
  wr(BLK(11), s0 ^ 0xFFFFFFFFu);
  wr(ECDSA_BASE + 0x00, (0 << 0) | (1 << 2));
  wr(ECDSA_BASE + 0x04, 1);
  uint32_t tamper_res = rd(ECDSA_BASE + 0x18);
  Serial.print("ECDSA tamper RESULT=");
  Serial.println(tamper_res);

  bool pass = (res == 1) && r_ok && s_ok && (verify_res == 1) && (tamper_res == 0);
  Serial.println(pass ? "ECDSA POKE PASS" : "ECDSA POKE FAIL");
  Serial.println("ECDSA DONE");
}

void loop() { delay(1000); }
