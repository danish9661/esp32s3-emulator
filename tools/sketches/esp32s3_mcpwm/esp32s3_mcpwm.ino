// MCPWM (motor-control PWM) validation for the ESP32-S3 emulator.
// Pokes the MCPWM0 registers directly (no esp-idf driver) to avoid unmodeled
// paths, mirrors tools/sketches/esp32s3_{rmt,pcnt,twai,gdma}.
//
// Flow: route MCPWM0 operator0 generator A (PWM0_OUT0A = signal 160) to GPIO2,
// start timer0 in up-counting mode (period 100), and configure generator0 so
// the output goes high at TEZ (count == 0) and low at TEA (count == cmprA).
// The firmware then samples the pad via digitalRead (GPIO_IN loopback resolves
// the peripheral-driven level) and reports the measured duty cycle. A second
// pass with a smaller comparator exercises a different duty.

#define MCPWM 0x6001E000u
#define MCPWM1 0x6002C000u
#define GPIO  0x60004000u

static volatile uint32_t* M(uint32_t off) {
  return (volatile uint32_t*)(MCPWM + off);
}
static volatile uint32_t* M1(uint32_t off) {
  return (volatile uint32_t*)(MCPWM1 + off);
}
static volatile uint32_t* G(uint32_t off) {
  return (volatile uint32_t*)(GPIO + off);
}

static int measure_duty(int pin, uint32_t samples) {
  uint32_t high = 0;
  for (uint32_t i = 0; i < samples; i++) {
    if (*G(0x3C) & (1u << pin)) high++;
  }
  return (int)(high * 100u / samples);
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 17);
  *(volatile uint32_t*)(0x600C0018) |= (1u << 20);
  const int PIN = 2;

  // Route PWM0_OUT0A (signal 160) to GPIO2 and enable the output driver.
  *G(0x554 + PIN * 4) = 160;       // FUNC_OUT_SEL_CFG[2]
  *G(0x24) = (1u << PIN);           // GPIO_ENABLE_W1TS

  // timer0: period = 100, prescale = 0.
  *M(0x04) = (100u << 8);
  // timer0_cfg1: mode = up (1<<3), start = run-on (2).
  *M(0x08) = (1u << 3) | 2;
  // operator_timersel: operator0 -> timer0.
  *M(0x38) = 0;
  // generator0: utez = set-high (1), utea = clear-low (2).
  *M(0x50) = (2u << 4) | 1;

  // Pass 1: comparator A = 50 -> ~50% duty.
  *M(0x40) = 50;
  int duty1 = measure_duty(PIN, 4000);
  Serial.printf("MCPWM duty1=%d%%\n", duty1);

  // Pass 2: comparator A = 25 -> ~25% duty.
  *M(0x40) = 25;
  int duty2 = measure_duty(PIN, 4000);
  Serial.printf("MCPWM duty2=%d%%\n", duty2);

  // Pass 3: MCPWM group 1 (base 0x6002C000), same layout: route
  // PWM1_OUT0A (signal 166) to GPIO3, timer0 up period 100, utez=set,
  // utea=clear, comparator A = 50 -> ~50% duty.
  const int PIN1 = 3;
  *G(0x554 + PIN1 * 4) = 166;
  *G(0x24) = (1u << PIN1);
  *M1(0x04) = (100u << 8);
  *M1(0x08) = (1u << 3) | 2;
  *M1(0x38) = 0;
  *M1(0x50) = (2u << 4) | 1;
  *M1(0x40) = 50;
  int duty3 = measure_duty(PIN1, 4000);
  Serial.printf("MCPWM1 duty3=%d%%\n", duty3);

  // Pass 4: external timer sync. GPIO4 drives SYNC0_IN (signal 160);
  // a rising edge with SYNCI_EN reloads timer0 with PHASE.
  const int SYNC_PIN = 4;
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  *G(0x154 + 160 * 4) = SYNC_PIN;  // FUNC_IN_SEL[160] = GPIO4
  *M(0x0C) = (1u << 0) | (20u << 4);  // SYNCI_EN + PHASE=20
  // Wait until the free-running counter passes 50, then pulse SYNC.
  uint32_t t0 = millis();
  while ((*M(0x10) & 0xFFFFu) <= 50u) {
    if (millis() - t0 > 1000) break;
  }
  digitalWrite(SYNC_PIN, HIGH);
  uint32_t synced = *M(0x10) & 0xFFFFu;
  Serial.printf("MCPWM sync cnt=%lu\n", (unsigned long)synced);
  bool sync_ok = synced >= 15 && synced <= 45;

  if (duty1 >= 40 && duty1 <= 60 && duty2 >= 15 && duty2 <= 35
      && duty3 >= 40 && duty3 <= 60 && sync_ok) {
    Serial.println("MCPWM PASS");
  } else {
    Serial.println("MCPWM FAIL");
  }
}

void loop() {}
