// Internal temperature-sensor validation for the ESP32-S3 emulator.
// Uses the Arduino temperatureRead() driver path (esp-idf tsens:
// install 10..50C range, enable, get_celsius). The emulator models the
// SENS TSENS block (power-up, READY poll, 8-bit OUT) with the host
// injecting TEMP_INJECT_C (default 25); the sketch asserts the reading
// lands within +/-3C of injected (float + curve tolerance) and prints
// TOUCH-style PASS markers for the battery.
void setup() {
  Serial.begin(115200);
  delay(200);
  float t = temperatureRead();
  Serial.print("TEMP C=");
  Serial.println(t, 2);
  // Sensor valid range on S3 is about -10..80; NAN means driver failure.
  if (t == t && t > -20.0 && t < 90.0) {
    Serial.println("TEMP SANE");
  } else {
    Serial.println("TEMP FAIL");
  }
  // Battery injects TEMP_C=25 and expects 25 +/- 3 (curve + float slop).
  if (t > 22.0 && t < 28.0) {
    Serial.println("TEMP 25C OK");
  }
  Serial.println("TEMP DONE");
}

void loop() {
  delay(1000);
}
