// ESP32-S3 RTC watchdog (RWDT) reset-path validation: enable RWDT with a
// reset action and never feed it. Each timeout reboots the chip, so
// "RWDT RESET TEST" repeats (like the MWDT reset sketch).
#define RTC_BASE 0x60008000u
#define WKEY 0x50D83AA1

void poke(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("RWDT RESET TEST");
  poke(RTC_BASE + 0xB0, WKEY);        // WDTWPROTECT = key
  poke(RTC_BASE + 0x9C, 200000);      // STG0_HOLD short so it fires fast
  // WDTCONFIG0: WDT_EN=1, STG0 = reset system (3<<28).
  poke(RTC_BASE + 0x98, (1u << 31) | (3u << 28));
}

void loop() {
  delay(1000);
}
