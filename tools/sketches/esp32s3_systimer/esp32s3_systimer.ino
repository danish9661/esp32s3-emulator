// ESP32-S3 SYSTIMER validation for the emulator.
// Arduino millis()/micros() read the SYSTIMER unit counter via the
// UNITn_OP snapshot handshake; advancing time is the core SYSTIMER behavior.

void setup() {
  Serial.begin(115200);
  delay(200);

  unsigned long m0 = millis();
  unsigned long u0 = micros();
  delay(100);
  unsigned long m1 = millis();
  unsigned long u1 = micros();

  if (!(m1 > m0)) {
    Serial.println("SYSTIMER FAIL millis did not advance");
    return;
  }
  if (!(u1 > u0)) {
    Serial.println("SYSTIMER FAIL micros did not advance");
    return;
  }

  // Also confirm the raw SYSTIMER unit counter advances across a busy loop,
  // exercising the UNIT_OP snapshot handshake directly.
  volatile uint32_t* unit_op = (volatile uint32_t*)0x60023004;   // UNIT0_OP
  volatile uint32_t* unit_lo = (volatile uint32_t*)0x60023044;   // UNIT0_VALUE_LO
  volatile uint32_t* unit_hi = (volatile uint32_t*)0x60023040;   // UNIT0_VALUE_HI
  *unit_op = (1u << 30);  // timer_unit_update: latch counter into VALUE
  uint32_t lo0 = *unit_lo;
  uint32_t hi0 = *unit_hi;
  delay(50);
  *unit_op = (1u << 30);
  uint32_t lo1 = *unit_lo;
  uint32_t hi1 = *unit_hi;
  if (lo0 == lo1 && hi0 == hi1) {
    Serial.println("SYSTIMER FAIL unit counter did not advance");
    return;
  }

  Serial.println("SYSTIMER PASS");
}

void loop() {
  Serial.println("DONE");
  delay(1000);
}
