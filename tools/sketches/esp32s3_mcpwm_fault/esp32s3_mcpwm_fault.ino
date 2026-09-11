// MCPWM fault/trip validation for the ESP32-S3 emulator (direct register
// pokes, no esp-idf driver — mirrors esp32s3_mcpwm).
//
// Flow: timer0 up-mode drives PWM0_OUT0A high (utez=set, no comparator),
// routed to GPIO2. GPIO4 (driven by the firmware itself) feeds FAULT0
// (signal 163). With F0 enabled high-active and a CBC force-low trip action,
// driving GPIO4 high must force GPIO2 low (trip), and driving it low must
// release the output back high. EVENT_F0 and the fault-enter interrupt bit
// are polled too.

#define MCPWM 0x6001E000u
#define GPIO  0x60004000u

static volatile uint32_t* M(uint32_t off) {
  return (volatile uint32_t*)(MCPWM + off);
}
static volatile uint32_t* G(uint32_t off) {
  return (volatile uint32_t*)(GPIO + off);
}

void setup() {
  Serial.begin(115200);
  const int OUT = 2;
  const int FAULTPIN = 4;

  // Route PWM0_OUT0A (signal 160) to GPIO2, enable the driver.
  *G(0x554 + OUT * 4) = 160;
  *G(0x24) = (1u << OUT);
  // Route GPIO4 to FAULT0 (signal 163).
  *G(0x154 + 163 * 4) = FAULTPIN;

  // timer0: period = 100, prescale = 0, up-mode, run.
  *M(0x04) = (100u << 8);
  *M(0x08) = (1u << 3) | 2;
  *M(0x38) = 0;      // operator0 -> timer0
  *M(0x50) = 1;      // generator0: utez = set-high (stuck high)

  // FH0: F0_CBC (bit 3) + A_CBC_U force-low (2 at [11:10]).
  *M(0x68) = (1u << 3) | (2u << 10);
  // FAULT_DETECT: F0_EN + high-active pole.
  *M(0xE4) = (1u << 0) | (1u << 3);

  pinMode(FAULTPIN, OUTPUT);

  // No fault: output must read high.
  digitalWrite(FAULTPIN, LOW);
  delay(5);
  int idle_high = 0;
  for (int i = 0; i < 500; i++) idle_high += digitalRead(OUT);
  Serial.printf("MCPWM FAULT idle=%d/500\n", idle_high);

  // Trip: drive the fault input high, output must go low.
  digitalWrite(FAULTPIN, HIGH);
  delay(5);
  int trip_low = 0;
  for (int i = 0; i < 500; i++) trip_low += !digitalRead(OUT);
  uint32_t ev = *M(0xE4);
  uint32_t raw = *M(0x114);
  Serial.printf("MCPWM FAULT trip=%d/500 ev=%d enter=%d\n", trip_low,
                (int)((ev >> 6) & 1), (int)((raw >> 9) & 1));

  // Release: fault input low again, output must return high.
  digitalWrite(FAULTPIN, LOW);
  delay(5);
  int rel_high = 0;
  for (int i = 0; i < 500; i++) rel_high += digitalRead(OUT);
  Serial.printf("MCPWM FAULT release=%d/500\n", rel_high);

  bool ok = idle_high == 500 && trip_low == 500 && ((ev >> 6) & 1) == 1 &&
            ((raw >> 9) & 1) == 1 && rel_high == 500;
  Serial.println(ok ? "MCPWM FAULT PASS" : "MCPWM FAULT FAIL");
}

void loop() { delay(1000); }
