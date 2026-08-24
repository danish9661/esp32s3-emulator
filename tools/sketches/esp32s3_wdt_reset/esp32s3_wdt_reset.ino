// ESP32-S3 WDT validation (reset path): disable the framework Task Watchdog
// so it won't touch the MWDT, then arm TIMG0's MWDT with a RESET action and
// NEVER feed it. The WDT must fire and reset the chip, so the boot banner
// "WDT RESET TEST" repeats (the emulator re-runs the boot ROM on each reset).
// If the reset action were not modeled, the banner would appear only once and
// the firmware would hang in the busy loop.
#include "esp_task_wdt.h"
#define TIMG0_BASE 0x6001F000
#define WKEY 0x50D83AA1

void poke(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

void setup() {
  Serial.begin(115200);
  esp_task_wdt_deinit();   // stop the framework Task WDT from feeding/panicking
  Serial.println("WDT RESET TEST");
  poke(TIMG0_BASE + 0x64, WKEY);            // WDT_WPROTECT = key
  poke(TIMG0_BASE + 0x4C, (1u << 16));      // WDT_CONFIG1 prescale = 1
  poke(TIMG0_BASE + 0x50, 30000);           // WDT_CONFIG2 stg0 hold = 30000
  poke(TIMG0_BASE + 0x54, 0);
  poke(TIMG0_BASE + 0x58, 0);
  poke(TIMG0_BASE + 0x5C, 0);
  // WDT_CONFIG0: wdt_en=1, stg0 action = reset (bits [30:29] = 0b11 = 3)
  poke(TIMG0_BASE + 0x48, (1u << 31) | (3u << 29));
  while (1) { /* never feed */ }
}

void loop() {}
