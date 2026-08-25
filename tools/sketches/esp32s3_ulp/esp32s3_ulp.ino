// ESP32-S3 ULP-RISC-V register-block validation for the emulator.
// The ULP peripheral is at 0x60008100 (page 0x60008000 + 0x100). We poke the
// control/status registers and read them back; program execution itself is not
// modeled, so this only validates the register path.

#define ULP_BASE 0x60008100
volatile uint32_t* ulp_core = (volatile uint32_t*)(ULP_BASE + 0x00);
volatile uint32_t* ulp_ocp  = (volatile uint32_t*)(ULP_BASE + 0x04);
volatile uint32_t* ulp_reg0 = (volatile uint32_t*)(ULP_BASE + 0x0C); // general reg

void setup() {
  Serial.begin(115200);
  delay(200);

  *ulp_core = 0xDEADBEEF;
  *ulp_ocp  = 0x12345678;
  *ulp_reg0 = 0xAB;

  if (*ulp_core != 0xDEADBEEF) { Serial.println("ULP FAIL core"); return; }
  if (*ulp_ocp  != 0x12345678) { Serial.println("ULP FAIL ocp"); return; }
  if (*ulp_reg0 != 0xAB) { Serial.println("ULP FAIL reg0"); return; }

  Serial.println("ULP PASS");
}

void loop() {
  Serial.println("DONE");
  delay(1000);
}
