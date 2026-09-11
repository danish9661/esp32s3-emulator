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

  volatile uint32_t *sens = (volatile uint32_t *)0x60008800u;
  // Denoise-only run (touch_denoise battery entry injects pad 0 instead
  // of pad 3): report STATUS0 and stop. No FAIL verdicts on this path.
  uint32_t denoise0 = sens[0xA0 / 4] & 0x3FFFFFu;
  if (denoise0 != 0) {
    Serial.printf("TOUCH DENOISE %u\n", (unsigned)denoise0);
    Serial.println("TOUCH DENOISE DONE");
    return;
  }

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

  // Proximity mode: arm approach_pad0 = pad 3 (CONF bits [31:28],
  // read-modify-write to keep the driver's outen). With pad 3 touched
  // (1877 < 2000) the APPR_STATUS pad0 counter must run up.
  sens[0x5C / 4] |= (3u << 28);
  sens[0x64 / 4 + 2] = 2000;  // THRES3
  delay(200);
  uint32_t appr = (sens[0xE0 / 4] >> 8) & 0xFFu;
  Serial.printf("TOUCH APPROACH %u\n", (unsigned)appr);
  if (appr == 0) {
    Serial.println("TOUCH APPROACH MISMATCH");
    return;
  }
  Serial.println("TOUCH APPROACH OK");

  Serial.println("TOUCH PASS");
  Serial.println("DONE");
}

void loop() {
  delay(1000);
}
