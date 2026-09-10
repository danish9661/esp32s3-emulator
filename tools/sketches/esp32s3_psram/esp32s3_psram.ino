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
  Serial.println(ok ? "PSRAM PROBE PASS" : "PSRAM PROBE FAIL");
}
void loop() { delay(1000); }
