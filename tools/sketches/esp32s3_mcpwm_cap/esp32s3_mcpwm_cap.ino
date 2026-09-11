// ESP32-S3 MCPWM0 capture validation sketch (direct register pokes).
//
// Runs MCPWM0 timer0 as a slow PWM (period ~10000 APB ticks), routes
// PWM0_OUT0A (signal 160) to GPIO2, loops GPIO2 back into CAP0 (signal 166)
// via FUNC_IN_SEL, and measures the period from two consecutive CAP0
// captures — the full peripheral loopback through the emulator's capture
// submodule (timer latch + CAP interrupt).
#include <Arduino.h>

#define GPIO 0x60004000u
#define MCPWM0 0x6001E000u
#define G(x) ((volatile uint32_t*)(GPIO + (x)))
#define M(x) ((volatile uint32_t*)(MCPWM0 + (x)))

#define PIN 2
#define SIG_OUT0A 160
#define SIG_CAP0 166

static uint32_t wait_cap() {
  for (uint32_t t = 0; t < 20000000; t++) {
    if (*M(0x114) & (1u << 27)) {
      return 1;
    }
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 17);
  delay(50);

  // Route OUT0A -> GPIO2 (output driver on, like the mcpwm sketch).
  *G(0x554 + PIN * 4) = SIG_OUT0A;
  *G(0x24) = (1u << PIN);
  // Route GPIO2 -> CAP0 input.
  *G(0x154 + SIG_CAP0 * 4) = PIN;

  // Timer0: period 100, prescale 99 (100 ticks/count), up mode, run.
  *M(0x04) = (100u << 8) | 99u;
  *M(0x08) = (1u << 3) | 2u;
  // Operator0: comparator A = 50, generator0 utez=set / utea=clear.
  *M(0x40) = 50;
  *M(0x50) = (2u << 4) | 1u;

  // Capture timer on, channel 0 enabled on rising edges.
  *M(0xE8) = 1u;
  *M(0xF0) = 1u | (2u << 1);
  *M(0x11C) = (1u << 27);  // clear stale

  if (!wait_cap()) {
    Serial.println("MCPWM CAP TIMEOUT");
    return;
  }
  uint32_t c1 = *M(0xFC);
  *M(0x11C) = (1u << 27);
  if (!wait_cap()) {
    Serial.println("MCPWM CAP TIMEOUT");
    return;
  }
  uint32_t c2 = *M(0xFC);
  uint32_t period = c2 - c1;
  Serial.printf("MCPWM CAP c1=%u c2=%u period=%u\n", c1, c2, period);
  // Expected PWM period: 100 counts * 100 ticks = 10000, +/-5%.
  Serial.println(
      (period > 9500 && period < 10500) ? "MCPWM CAP PASS" : "MCPWM CAP FAIL");
  Serial.println("MCPWM CAP DONE");
}

void loop() {}
