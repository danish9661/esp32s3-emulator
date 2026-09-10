// MCPWM dead-time validation: complementary operator0 A/B (50% duty)
// with FED=0, RED=2000 ticks. Asserts the outputs never overlap (no
// shoot-through: A&B==1 never observed over many periods) and that B's
// rising edge is delayed well after A's fall (dead band), then B runs.
#define MCPWM 0x6001E000u
#define GPIO  0x60004000u
#define PINA 2
#define PINB 5

static volatile uint32_t* M(uint32_t off) {
  return (volatile uint32_t*)(MCPWM + off);
}
static volatile uint32_t* G(uint32_t off) {
  return (volatile uint32_t*)(GPIO + off);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  *G(0x554 + PINA * 4) = 160;  // PWM0_OUT0A -> GPIO2
  *G(0x554 + PINB * 4) = 161;  // PWM0_OUT0B -> GPIO5
  *G(0x24) = (1u << PINA) | (1u << PINB);

  *M(0x04) = (20000u << 8);        // timer0 period 20000, prescale 0
  *M(0x08) = (1u << 3) | 2;        // up-counting, run
  *M(0x38) = 0;                    // operator0 -> timer0
  *M(0x50) = (2u << 4) | 1;        // GEN0 A: set on TEZ, clear on TEA
  *M(0x54) = (1u << 4) | 2;        // GEN0 B: clear on TEZ, set on TEA
  *M(0x40) = 10000;                // 50% comparator
  *M(0x5C) = 0;                    // DT0 FED = 0 (falls immediate)
  *M(0x60) = 2000;                 // DT0 RED = 2000 ticks (rise delay)

  // Wait for A high, then for A to fall; from there B must stay low for
  // a long dead band (RED=2000 ticks >> loop cost) before rising.
  auto pin = [](int p) { return ((*G(0x3C) >> p) & 1u) != 0; };
  uint32_t guard = 0;
  while (!pin(PINA) && guard++ < 100000) {}
  guard = 0;
  while (pin(PINA) && guard++ < 100000) {}
  uint32_t zeros = 0;
  while (!pin(PINB) && zeros < 100000) { zeros++; }
  uint32_t overlap = 0;
  for (uint32_t i = 0; i < 20000; i++) {
    if (pin(PINA) && pin(PINB)) overlap++;
  }
  Serial.print("MCPWM DT zeros-after-fall=");
  Serial.println(zeros);
  Serial.print("MCPWM DT overlap=");
  Serial.println(overlap);
  bool band = zeros >= 10 && zeros < 100000;
  if (overlap == 0 && band) {
    Serial.println("MCPWM DT PASS");
  } else {
    Serial.println("MCPWM DT FAIL");
  }
  Serial.println("MCPWM DT DONE");
}

void loop() {
  delay(1000);
}
