#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  uint32_t total = ESP.getPsramSize();
  uint32_t freeb = ESP.getFreePsram();
  Serial.printf("PSRAM total=%u free=%u\n", total, freeb);
  if (total == 0) { Serial.println("PSRAM PROBE NO-PSRAM"); return; }
  uint32_t *p = (uint32_t *)ps_malloc(1024);
  Serial.printf("PSRAM malloc=%p\n", p);
  if (!p) { Serial.println("PSRAM PROBE MALLOC-FAIL"); return; }
  for (int i = 0; i < 256; i++) p[i] = 0xA5000000u | (uint32_t)i;
  bool ok = true;
  for (int i = 0; i < 256; i++) if (p[i] != (0xA5000000u | (uint32_t)i)) { ok = false; break; }
  Serial.printf("PSRAM RW %s\n", ok ? "OK" : "MISMATCH");
  free(p);
  if (!ok) { Serial.println("PSRAM PROBE FAIL"); return; }
  // 16 MB parts (MR2[2:0] = 5): a 13 MB allocation must round-trip at
  // both ends, proving the upper 8 MB are really mapped (with an 8 MB
  // backing the high bytes would read back zero).
  if (total >= 16u * 1024u * 1024u) {
    const uint32_t BIG = 13u * 1024u * 1024u;
    uint8_t *big = (uint8_t *)ps_malloc(BIG);
    Serial.printf("PSRAM big=%p\n", big);
    bool hok = big != nullptr;
    if (hok) {
      for (uint32_t i = 0; i < 1024; i++) big[i] = (uint8_t)(i & 0xFF);
      for (uint32_t i = 0; i < 1024; i++) big[BIG - 1024 + i] = (uint8_t)((i + 77) & 0xFF);
      for (uint32_t i = 0; i < 1024 && hok; i++) {
        if (big[i] != (uint8_t)(i & 0xFF)) hok = false;
        if (big[BIG - 1024 + i] != (uint8_t)((i + 77) & 0xFF)) hok = false;
      }
      free(big);
    }
    Serial.println(hok ? "PSRAM HIGH OK" : "PSRAM HIGH MISMATCH");
    if (!hok) { Serial.println("PSRAM PROBE FAIL"); return; }
  }
  Serial.println("PSRAM PROBE PASS");
}
void loop() { delay(1000); }
