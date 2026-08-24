// ESP32-S3 WDT validation (feed path): disable the framework Task Watchdog,
// then enable TIMG0 MWDT with a reset action and feed it every loop. If the
// model's feed path works, the WDT never fires and the sketch runs to
// completion ("WDT FEED TEST START" is printed exactly once). If feeding were
// broken, the WDT would reset the chip and the banner would repeat.
#include "esp_task_wdt.h"
#define TIMG0_BASE 0x6001F000
#define WKEY 0x50D83AA1

void poke(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

void setup() {
  Serial.begin(115200);
  esp_task_wdt_deinit();   // stop the framework Task WDT from feeding/panicking
  Serial.println("WDT FEED TEST START");
  poke(TIMG0_BASE + 0x64, WKEY);            // WDT_WPROTECT = key
  poke(TIMG0_BASE + 0x4C, (1u << 16));      // WDT_CONFIG1 prescale = 1
  poke(TIMG0_BASE + 0x50, 5000000);         // WDT_CONFIG2 stg0 hold = 5M
  poke(TIMG0_BASE + 0x54, 0);
  poke(TIMG0_BASE + 0x58, 0);
  poke(TIMG0_BASE + 0x5C, 0);
  // WDT_CONFIG0: wdt_en=1, stg0 action = reset (bits [30:29] = 0b11 = 3)
  poke(TIMG0_BASE + 0x48, (1u << 31) | (3u << 29));
}

void loop() {
  static int n = 0;
  poke(TIMG0_BASE + 0x60, 0xABAD1DEA);      // feed -> counter reset
  Serial.println("WDT FED " + String(n++));
  delay(2);
}
