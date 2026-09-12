// MCPWM software-sync + timer-to-timer SYNCO chain for the ESP32-S3 emulator.
// Stops timer0, fires a software sync with PHASE=500, and asserts the
// live TIMER_STATUS reads back exactly 500 (sync reload path). Then
// programs timer0 TEZ sync_out into timer1 (SYNCO_SEL + SYNCISEL per
// mcpwm_reg.h) and asserts timer1 reloads with its PHASE.
#define MCPWM0 0x6001E000u
#define T0CFG1 (MCPWM0 + 0x08)
#define T0SYNC (MCPWM0 + 0x0C)
#define T0STAT (MCPWM0 + 0x10)
#define T1SYNC (MCPWM0 + 0x1C)
#define T1STAT (MCPWM0 + 0x20)
#define SYNCI_CFG (MCPWM0 + 0x34)

void setup() {
  Serial.begin(115200);
  delay(200);
  *(volatile uint32_t*)(0x600C0018) |= (1u << 17);
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
  // SYNCO chain: timer0 period 10 up run with SYNCO_SEL=TEZ, timer1
  // stopped armed PHASE=7 selecting timer0 sync_out.
  volatile uint32_t *t0cfg0 = (volatile uint32_t *)(MCPWM0 + 0x04);
  volatile uint32_t *t1sync = (volatile uint32_t *)T1SYNC;
  volatile uint32_t *t1stat = (volatile uint32_t *)T1STAT;
  volatile uint32_t *syncisel = (volatile uint32_t *)SYNCI_CFG;
  *t0cfg0 = (10u << 8);
  *cfg1 = (1u << 3) | 2;
  *sync = (1u << 2) | 1;  // SYNCO_SEL=TEZ + SYNCI_EN (harmless)
  *t1sync = (7u << 4) | 1;
  *syncisel = (1u << 3);  // timer1 selects timer0 sync_out
  delay(50);
  uint32_t s1 = *t1stat;
  Serial.print("MCPWM SYNCO t1=");
  Serial.println(s1);
  if (s1 == 7) {
    Serial.println("MCPWM SYNCO PASS");
  } else {
    Serial.println("MCPWM SYNCO FAIL");
  }
  Serial.println("MCPWM SYNC DONE");
}

void loop() {
  delay(1000);
}
