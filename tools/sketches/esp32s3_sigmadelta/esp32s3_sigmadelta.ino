// SigmaDelta peripheral validation sketch (ESP32-S3, arduino-cli).
//
// Configures SigmaDelta on GPIO2 at 50% duty and samples the pin. The
// emulator's Sigma-Delta model produces a PDM whose high fraction equals
// duty/256 (esp-idf signed duty: 128 -> 50%), so sampling should average to
// ~50%.  Prints SIGMADELTA PASS/FAIL and DONE so run_flash can assert the
// peripheral works end-to-end through the real esp-idf SigmaDelta driver.

#define SD_PIN 2

void setup() {
  Serial.begin(115200);
  // Attach SigmaDelta to GPIO2 at ~1 MHz; write 50% duty (esp-idf: 128 == 50%).
  sigmaDeltaAttach(SD_PIN, 1000000);
  sigmaDeltaWrite(SD_PIN, 128);

  const int N = 20000;
  int high = 0;
  for (int i = 0; i < N; i++) {
    if (digitalRead(SD_PIN)) {
      high++;
    }
  }
  int pct = (high * 100) / N;
  Serial.printf("SDM duty=128 sampled_high=%d/%d (%d%%)\n", high, N, pct);
  if (pct >= 40 && pct <= 60) {
    Serial.println("SIGMADELTA PASS");
  } else {
    Serial.println("SIGMADELTA FAIL");
  }
  Serial.println("DONE");
}

void loop() {}
