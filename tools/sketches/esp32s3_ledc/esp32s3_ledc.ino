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
}

void loop() {}
