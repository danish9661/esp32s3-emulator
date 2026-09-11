// ESP32-S3 HMAC peripheral validation (direct register pokes).
// The HMAC block is a raw SHA-256 engine: the driver XORs the eFuse key with
// ipad/opad and feeds (key^ipad)||message||SHA-padding as 512-bit blocks.
// eFuse key block 0 defaults to all-zero in the emulator, so this computes
// HMAC-SHA256(zero_key, message). Expected digests are RFC-style vectors
// computed on the host.

#define HMAC_BASE 0x6003E000UL
#define SET_PARA_PURPOSE (HMAC_BASE + 0x44)
#define SET_PARA_KEY     (HMAC_BASE + 0x48)
#define SET_PARA_FINISH  (HMAC_BASE + 0x4C)
#define SET_MESSAGE_ING  (HMAC_BASE + 0x54)
#define SET_MESSAGE_ONE  (HMAC_BASE + 0x50)
#define SET_START        (HMAC_BASE + 0x40)
#define WDATA            (HMAC_BASE + 0x80)
#define RDATA            (HMAC_BASE + 0xC0)

static inline void wr(uint32_t a, uint32_t v) { *(volatile uint32_t*)a = v; }
static inline uint32_t rd(uint32_t a) { return *(volatile uint32_t*)a; }

// Compute HMAC of `msg` (len bytes) using eFuse key 0 (zero). Prints the
// 32-byte digest hex and compares against `exp` (32 bytes).
void hmac_run(const uint8_t* msg, int len, const uint8_t* exp) {
  wr(SET_PARA_PURPOSE, 8); // HMAC_KEY_PURPOSE_UP (read back to software)
  wr(SET_PARA_KEY, 0);     // eFuse key block 0
  wr(SET_PARA_FINISH, 1);  // latch key

  // Build the full inner SHA input: (key ^ ipad)[64] || msg || SHA-padding.
  // key is zero -> ipad = 0x36*64.
  uint8_t buf[256];
  int n = 0;
  for (int i = 0; i < 64; i++) buf[n++] = 0x36;
  for (int i = 0; i < len; i++) buf[n++] = msg[i];
  buf[n++] = 0x80;
  while (n % 64 != 56) buf[n++] = 0;
  uint64_t bitlen = (uint64_t)(64 + len) * 8;
  for (int i = 7; i >= 0; i--) buf[n++] = (uint8_t)((bitlen >> (i * 8)) & 0xff);

  // Write 16-word (512-bit) blocks; signal ONE_BLOCK on the last.
  int words = n / 4;
  for (int b = 0; b < words; b += 16) {
    for (int i = 0; i < 16 && (b + i) < words; i++) {
      int idx = (b + i) * 4;
      uint32_t w = (uint32_t)buf[idx] | ((uint32_t)buf[idx + 1] << 8) |
                   ((uint32_t)buf[idx + 2] << 16) | ((uint32_t)buf[idx + 3] << 24);
      wr(WDATA + i * 4, w);
    }
    if (b + 16 < words) wr(SET_MESSAGE_ING, 1);
    else wr(SET_MESSAGE_ONE, 1);
  }

  wr(SET_START, 1);

  uint8_t dig[32];
  for (int i = 0; i < 8; i++) {
    uint32_t v = rd(RDATA + i * 4);
    dig[i * 4 + 0] = (v >> 24) & 0xff;
    dig[i * 4 + 1] = (v >> 16) & 0xff;
    dig[i * 4 + 2] = (v >> 8) & 0xff;
    dig[i * 4 + 3] = v & 0xff;
  }
  Serial.print("HMAC digest=");
  bool ok = true;
  for (int i = 0; i < 32; i++) {
    if (dig[i] < 0x10) Serial.print("0");
    Serial.print(dig[i], HEX);
    if (dig[i] != exp[i]) ok = false;
  }
  Serial.println(ok ? " OK" : " MISMATCH");
}

const uint8_t MSG1[] = "hello";
const uint8_t EXP1[32] = {0x43,0x52,0xb2,0x6e,0x33,0xfe,0x0d,0x76,0x9a,0x89,0x22,0xa6,0xba,0x29,0x00,0x41,
                          0x09,0xf0,0x16,0x88,0xe2,0x6a,0xcc,0x9e,0x6c,0xb3,0x47,0xe5,0xa5,0xaf,0xc4,0xda};
const uint8_t MSG2[] = "The quick brown fox jumps over the lazy dog";
const uint8_t EXP2[32] = {0xfb,0x01,0x1e,0x61,0x54,0xa1,0x9b,0x9a,0x4c,0x76,0x73,0x73,0xc3,0x05,0x27,0x5a,
                          0x5a,0x69,0xe8,0xb6,0x8b,0x0b,0x4c,0x92,0x00,0xc3,0x83,0xdc,0xed,0x19,0xa4,0x16};

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 5);
  delay(200);
  Serial.println("HMAC VALIDATION START");
  hmac_run(MSG1, sizeof(MSG1) - 1, EXP1);
  hmac_run(MSG2, sizeof(MSG2) - 1, EXP2);
  Serial.println("HMAC DONE");
}

void loop() { delay(1000); }
