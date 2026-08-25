// ESP32-S3 RNG (WDEV_RND) validation for the emulator.
// esp_random() reads WDEV_RND_REG (0x6003507C); two consecutive reads must
// differ (the model advances an LCG each read).

void setup() {
  Serial.begin(115200);
  delay(200);

  uint32_t a = esp_random();
  uint32_t b = esp_random();
  uint32_t c = esp_random();

  if (a == b || b == c) {
    Serial.println("RNG FAIL: consecutive reads equal");
    return;
  }
  // Also poke the raw data register directly to confirm the model responds.
  volatile uint32_t* rnd = (volatile uint32_t*)0x6003507C;
  uint32_t raw1 = *rnd;
  uint32_t raw2 = *rnd;
  if (raw1 == raw2) {
    Serial.println("RNG FAIL: raw reads equal");
    return;
  }

  Serial.println("RNG PASS");
}

void loop() {
  Serial.println("DONE");
  delay(1000);
}
