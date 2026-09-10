// ESP32-S3 RTC watchdog (RWDT) feed-path validation: enable RWDT with a
// reset action and feed it every loop. "RWDT FEED TEST START" prints once;
// repeats would mean an unwanted reset fired.
#define RTC_BASE 0x60008000u
#define WKEY 0x50D83AA1

void poke(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("RWDT FEED TEST START");
  poke(RTC_BASE + 0xB0, WKEY);        // WDTWPROTECT = key
  poke(RTC_BASE + 0x9C, 5000000);     // STG0_HOLD = 5M slow ticks
  // WDTCONFIG0: WDT_EN=1, STG0 = reset system (3<<28).
  poke(RTC_BASE + 0x98, (1u << 31) | (3u << 28));
}

void loop() {
  static int n = 0;
  poke(RTC_BASE + 0xAC, 0xABAD1DEA);  // feed -> counter reset
  Serial.println("RWDT FED " + String(n++));
  delay(2);
}
