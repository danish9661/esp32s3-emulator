#include <Arduino.h>

// ESP32-S3 RSA peripheral (DR_REG_RSA_BASE = 0x6003C000)
#define RSA_BASE 0x6003C000UL
#define RSA_M     0x0000   // M (and N on real HW; we use M=N)
#define RSA_Z     0x0200   // Z / RB result
#define RSA_Y     0x0400   // Y (exponent)
#define RSA_X     0x0600   // X (base/message)
#define RSA_M_DASH 0x0800
#define RSA_LENGTH 0x0804
#define RSA_MODEXP 0x080c
#define RSA_QUERY  0x0818
#define RSA_CLEAR  0x081c
#define RSA_INT    0x082c

static inline void poke32(uint32_t addr, uint32_t v) { *(volatile uint32_t*)(RSA_BASE + addr) = v; }
static inline uint32_t peek32(uint32_t addr) { return *(volatile uint32_t*)(RSA_BASE + addr); }

static const uint32_t N_le[32] = { 0xA523A5D3, 0xA9786480, 0x03BF875E, 0x967550AE, 0xB3451599, 0x7DE43165, 0x399CF136, 0x8C6CD061, 0x1CD1E074, 0xC9328241, 0x7CFAAC9C, 0xBB93F615, 0x5A1FC254, 0x33C7C3B8, 0x1439E5B8, 0xC22E870B, 0x0627AEC4, 0x0034D704, 0x52BE0923, 0x30BBCF60, 0xAD549151, 0x8C088E24, 0x54E1B7B6, 0x588DD86C, 0xDDC8A1DA, 0xA413A1E1, 0x54E694AD, 0x721D9DE7, 0x1D5D8D13, 0xDB42D0E6, 0x14827DD3, 0x32B78617 };
static const uint32_t M_le[32] = { 0xB040DAED, 0x6951C441, 0x62019962, 0x68DB5A3C, 0x1A77BD67, 0x2EDC22B3, 0xB2B305D8, 0x1BD00E7E, 0x7E6A7A24, 0x81887009, 0x36AC9D9B, 0x5F2250B5, 0xA330FC5E, 0xEAE19B6B, 0xDD91B111, 0xF1196333, 0x4A9B4577, 0x06774650, 0x5230DBDC, 0x21464164, 0x9FE96664, 0x98C1529D, 0x14D615DD, 0xB9411B54, 0x4551231B, 0xB98E13B9, 0x46E64F0C, 0xC5BA1E2E, 0xA42F7B77, 0x7369A004, 0x9C506A97, 0x2F12FDD8 };
static const uint32_t E_le[32] = { 0x00010001, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 };
static const uint32_t C_le[32] = { 0xDE8235AA, 0x37F6A577, 0xF7B11535, 0x9DD17789, 0x48C6A611, 0x24D5A885, 0x6D827593, 0x9DDEAF9C, 0x5ABC8FCA, 0x1718941D, 0x700FD985, 0x7FA19955, 0x1F8CCA73, 0x44F0AE9C, 0x2AF04D37, 0xF7EA542B, 0xB6F12FD9, 0x4F17A9A6, 0xF0ACB2C2, 0x4219242E, 0xF565B6CC, 0x2E3A821C, 0xE7024A3A, 0x27DD9C90, 0xD33FC36B, 0xB8D28BAC, 0x7DBF0DC4, 0xDC2CE3A6, 0xF03058E9, 0x5025D6D7, 0x5954EA72, 0x07A3BD15 };
volatile uint32_t Z_le[32];
volatile uint32_t MARKERv4 = 0;

void setup() __attribute__((optimize("O0")));

void setup() {
  MARKERv4 = 0x1234;
  Serial.begin(115200);
  Serial.println("S0");
  // Force the RSA registers to be touched directly, inlined in setup.
  for (volatile int i = 0; i < 32; i++) poke32(RSA_M + i*4, N_le[i]);
  for (volatile int i = 0; i < 32; i++) poke32(RSA_X + i*4, M_le[i]);
  Serial.println("S1");
  for (volatile int i = 0; i < 32; i++) poke32(RSA_Y + i*4, E_le[i]);
  poke32(RSA_LENGTH, 31);
  Serial.println("S2");
  Serial.println("PREMODEXP");
  poke32(RSA_MODEXP, 1);
  MARKERv4 = 0x5678;
  Serial.println("POSTMODEXP");
  for (volatile int i = 0; i < 32; i++) Z_le[i] = peek32(RSA_Z + i*4);
  Serial.println("RSA POKE CT:");
  for (volatile int i = 0; i < 32; i++) { Serial.print(" "); Serial.print((uint32_t)Z_le[i], HEX); }
  Serial.println();
  bool ok = true;
  for (volatile int i = 0; i < 32; i++) if (Z_le[i] != C_le[i]) ok = false;
  Serial.println(ok ? "RSA POKE PASS" : "RSA POKE FAIL");
  Serial.println("RSA DONE");
}

void loop() { delay(1000); }
