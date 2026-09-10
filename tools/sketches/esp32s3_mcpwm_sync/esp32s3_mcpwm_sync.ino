// MCPWM software-sync validation for the ESP32-S3 emulator.
// Stops timer0, fires a software sync with PHASE=500, and asserts the
// live TIMER_STATUS reads back exactly 500 (sync reload path).
#define MCPWM0 0x6001E000u
#define T0CFG1 (MCPWM0 + 0x08)
#define T0SYNC (MCPWM0 + 0x0C)
#define T0STAT (MCPWM0 + 0x10)

void setup() {
  Serial.begin(115200);
  delay(200);
  volatile uint32_t *cfg1 = (volatile uint32_t *)T0CFG1;
  volatile uint32_t *sync = (volatile uint32_t *)T0SYNC;
  volatile uint32_t *stat = (volatile uint32_t *)T0STAT;
  *cfg1 = 0;  // stop timer0 (START < 2 freezes the counter)
  *sync = (500u << 4) | (1u << 1);  // PHASE=500 + SYNC_SW
  uint32_t s = *stat;
  Serial.print("MCPWM SYNC status=");
  Serial.println(s);
  if (s == 500) {
    Serial.println("MCPWM SYNC PASS");
  } else {
    Serial.println("MCPWM SYNC FAIL");
  }
  Serial.println("MCPWM SYNC DONE");
}

void loop() {
  delay(1000);
}
