// Direct register-poke ESP32-S3 AES validation (no esp-idf driver / GDMA).
// AES-128 ECB FIPS vector:
//   key       = 000102030405060708090a0b0c0d0e0f
//   plaintext = 00112233445566778899aabbccddeeff
//   ciphertext= 69c4e0d86a7b0430d8cdb78070b4c55a

#define AES_BASE 0x6003A000
#define REG(r)   (*(volatile uint32_t*)(AES_BASE + (r)))

// AES register offsets (soc/reg_base.h + aes_struct.h)
#define AES_KEY_BASE   0x00
#define AES_TEXT_IN    0x20
#define AES_TEXT_OUT   0x30
#define AES_MODE       0x40
#define AES_TRIGGER    0x48
#define AES_STATE      0x4c

static uint32_t rd(uint32_t off) { return REG(off); }
static void wr(uint32_t off, uint32_t v) { REG(off) = v; }

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 1);
  delay(50);

  // Key bytes 00..0f stored little-endian as 32-bit words.
  const uint32_t key[4] = { 0x03020100, 0x07060504, 0x0b0a0908, 0x0f0e0d0c };
  const uint32_t pt[4]  = { 0x33221100, 0x77665544, 0xbbaa9988, 0xffeeddcc };

  // mode = 0 (AES-128 encrypt): (decrypt?4:0) + key_bytes/8 - 2 = 0
  wr(AES_MODE, 0);
  for (int i = 0; i < 4; i++) wr(AES_KEY_BASE + i * 4, key[i]);
  for (int i = 0; i < 4; i++) wr(AES_TEXT_IN + i * 4, pt[i]);
  wr(AES_TRIGGER, 1);

  uint32_t ct[4];
  for (int i = 0; i < 4; i++) ct[i] = rd(AES_TEXT_OUT + i * 4);

  Serial.print("AES POKE ct=");
  for (int i = 0; i < 4; i++) {
    Serial.printf("%08x", ct[i]);
  }
  Serial.print(" state=");
  Serial.print(rd(AES_STATE));
  Serial.println();

  // Expected little-endian words of 69c4e0d8 6a7b0430 d8cdb780 70b4c55a
  const uint32_t exp[4] = { 0xd8e0c469, 0x30047b6a, 0x80b7cdd8, 0x5ac5b470 };
  bool ok = true;
  for (int i = 0; i < 4; i++) if (ct[i] != exp[i]) ok = false;
  Serial.println(ok ? "AES POKE PASS" : "AES POKE FAIL");
}

void loop() { delay(1000); }
