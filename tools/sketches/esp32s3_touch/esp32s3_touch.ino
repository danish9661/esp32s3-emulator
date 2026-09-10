// ESP32-S3 touch sensor validation for the emulator.
//
// Arduino touchRead(pin) drives the real touch stack (touch pad init, FSM
// timer mode, touch_pad_read_raw_data polling meas_done, STATUS counter
// read). The host injects the pad counter via TOUCH_INJECT=<pad>:<value>
// (run_flash env, like ADC_INJECT_MV); the sketch asserts the exact value
// comes back through the driver. Touch pad N is GPIO N on S3 (1:1).
//
// Expected run_flash output with TOUCH_INJECT=3:1877: "TOUCH READ 1877"
// then "TOUCH PASS" and "DONE".

void setup() {
  Serial.begin(115200);
  delay(200);

  touch_value_t v = touchRead(3);
  Serial.printf("TOUCH READ %u\n", (unsigned)v);
  if (v != 1877) {
    Serial.printf("TOUCH FAIL want 1877 got %u\n", (unsigned)v);
    return;
  }

  // An untouched pad reads the idle baseline (0 in the model: no charge
  // accumulated without host injection).
  touch_value_t idle = touchRead(5);
  Serial.printf("TOUCH IDLE %u\n", (unsigned)idle);
  if (idle != 0) {
    Serial.printf("TOUCH FAIL idle want 0 got %u\n", (unsigned)idle);
    return;
  }

  Serial.println("TOUCH PASS");
  Serial.println("DONE");
}

void loop() {
  delay(1000);
}
