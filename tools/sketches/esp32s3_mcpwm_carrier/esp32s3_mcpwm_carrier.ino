// MCPWM carrier-submodule validation for the ESP32-S3 emulator (direct
// register pokes, mirrors tools/sketches/esp32s3_mcpwm).
//
// Flow: route PWM0_OUT0A (signal 160) to GPIO2, start timer0 up-counting
// (period 1000) with generator0 utez=set / utea=clear at cmprA=500 (50%
// PWM), then enable the operator0 carrier (prescale 1 -> 16-step period,
// duty 4/8 -> 50% chop). The firmware samples the pad via raw GPIO_IN
// reads: the carrier must chop the bursts (many pad edges), where an
// unmodulated 50% PWM would show ~2 edges per PWM period. The average must
// read ~25% (50% PWM x 50% carrier). A second pass asserts the one-shot:
// duty 0/8 + oshtwth 2 holds the first pulse HIGH for 2 carrier periods.

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
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 17);
  const int PIN = 2;

  *G(0x554 + PIN * 4) = 160;  // FUNC_OUT_SEL_CFG[2] = PWM0_OUT0A
  *G(0x24) = (1u << PIN);     // GPIO_ENABLE_W1TS

  *M(0x04) = (1000u << 8);    // timer0: period 1000, prescale 0
  *M(0x08) = (1u << 3) | 2;   // timer0_cfg1: up, run
  *M(0x38) = 0;               // operator0 -> timer0
  *M(0x40) = 500;             // cmprA = 500 (50%)
  *M(0x50) = (2u << 4) | 1;   // generator0: utez=set, utea=clear
  // Carrier: en + prescale 1 (period 16 steps) + duty 4/8.
  *M(0x64) = 1u | (1u << 1) | (4u << 5);

  uint32_t high = 0, edges = 0, prev = (*G(0x3C) >> PIN) & 1u;
  for (uint32_t i = 0; i < 4000; i++) {
    uint32_t lv = (*G(0x3C) >> PIN) & 1u;
    high += lv;
    edges += (lv != prev);
    prev = lv;
  }
  int duty = (int)(high * 100u / 4000u);
  Serial.printf("MCPWM CARRIER duty=%d%% edges=%lu\n", duty, (unsigned long)edges);
  bool chopped = edges >= 20;
  bool avg = duty >= 15 && duty <= 35;
  Serial.println(chopped && avg ? "MCPWM CARRIER PASS" : "MCPWM CARRIER FAIL");
}

void loop() {}
