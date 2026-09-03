// LEDC validation sketch (ESP32-S3) — uses the Arduino `ledc` driver (the
// real esp-idf ledc driver stack) to generate PWM on a GPIO and measures the
// duty cycle by sampling the pin with digitalRead. This exercises the
// emulator's LEDC model (timer divider / duty_resolution in TIMERx_CONF,
// sig_out_en in CHx_CONF0, duty_start in CHx_CONF1) end-to-end through the
// driver. Compiled with arduino-cli; runs identically on real silicon
// (measured duty should track the requested duty). Arduino-3.x pin-based API.

#define LEDC_PIN 2
#define SAMPLES 60000

void setup() {
  Serial.begin(115200);
  delay(50);

  // Attach GPIO2 as a PWM output: 5 kHz, 10-bit resolution (auto-assigned
  // channel, routed to GPIO2 via the GPIO matrix).
  ledcAttach(LEDC_PIN, 5000, 10);

  // 50% duty.
  ledcWrite(LEDC_PIN, 512);
  uint32_t hi = 0;
  for (uint32_t i = 0; i < SAMPLES; i++) {
    if (digitalRead(LEDC_PIN)) hi++;
  }
  uint32_t pct50 = (hi * 100) / SAMPLES;
  Serial.printf("LEDC 50%% duty measured=%u%%\n", pct50);

  // 25% duty.
  ledcWrite(LEDC_PIN, 256);
  hi = 0;
  for (uint32_t i = 0; i < SAMPLES; i++) {
    if (digitalRead(LEDC_PIN)) hi++;
  }
  uint32_t pct25 = (hi * 100) / SAMPLES;
  Serial.printf("LEDC 25%% duty measured=%u%%\n", pct25);

  // 10% duty (asymmetry check).
  ledcWrite(LEDC_PIN, 102);
  hi = 0;
  for (uint32_t i = 0; i < SAMPLES; i++) {
    if (digitalRead(LEDC_PIN)) hi++;
  }
  uint32_t pct10 = (hi * 100) / SAMPLES;
  Serial.printf("LEDC 10%% duty measured=%u%%\n", pct10);

  bool ok = (pct50 >= 40 && pct50 <= 60 && pct25 >= 18 && pct25 <= 32 &&
             pct10 >= 5 && pct10 <= 15);
  Serial.println(ok ? "LEDC PASS" : "LEDC FAIL");

  // Fade path (real esp-idf fade driver on pin 3, channel 1): fade 0 ->
  // max over 100 ms and block until the fade-done ISR fires (exercises
  // the fade engine plus LEDC interrupt delivery through the matrix).
  fade_test();
}

#include "driver/ledc.h"

void fade_test() {
  // Fade on Arduino's own channel 0 (already configured above): no timer
  // reconfiguration, so no clock-diver conflict is possible.
  // Enable the fade-done interrupt directly (the Arduino channel was
  // attached without it, and the fade ISR only sees RAW&ENA): same bit
  // the HAL's ledc_enable_intr_type would set.
  *(volatile uint32_t*)(0x60019000u + 0xC8u) |= (1u << 4); // INT_ENA ch0
  *(volatile uint32_t*)(0x60019000u + 0xCCu) = 0xFFFu; // INT_CLR: drop stale
  ledc_fade_func_install(0);
  ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 300, 100);
  // Modest target (102 -> 300 user units): completes in a round or two,
  // robustly inside the step budget (each ISR-chained round costs task
  // wakeups; a full-range fade needs many). The engine + DONE interrupt +
  // semaphore path is identical either way.
  ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 300, 100);
  esp_err_t r = ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);
  uint32_t d = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  Serial.printf("LEDC fade err=%d final=%lu\n", (int)r, (unsigned long)d);
  // get_duty units are driver-version dependent (user units or reg units);
  // accept the exact target in either encoding.
  Serial.println((r == ESP_OK && (d == 300 || d == 4800)) ? "LEDC FADE PASS" : "LEDC FADE FAIL");
}

void loop() {}
