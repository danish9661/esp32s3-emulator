// ESP32-S3 ULP-RISC-V coprocessor validation for the emulator.
// The ULP-RISC-V core executes firmware from RTC_SLOW_MEM (0x50000000). We
// hand-assemble a tiny rv32im program that stores 0x12345678 to ULP reg slot 0
// (0x6000810C) and then `ebreak`, load it into RTC_SLOW_MEM, release the ULP
// core (write 1 to the `core` register @ 0x60008100), and poll the reg slot
// until the value appears. This exercises the real rv32im interpreter.

#define RTC_SLOW_MEM 0x50000000
#define ULP_BASE     0x60008100
#define ULP_REG0     (ULP_BASE + 0x0C)

volatile uint32_t* prog     = (volatile uint32_t*)RTC_SLOW_MEM;
volatile uint32_t* ulp_core = (volatile uint32_t*)ULP_BASE;
volatile uint32_t* ulp_reg0 = (volatile uint32_t*)ULP_REG0;

// rv32im program (LE words):
//   lui x1, 0x60008        ; x1 = 0x60008000
//   addi x1, x1, 0x10C     ; x1 = 0x6000810C (reg slot 0)
//   lui x2, 0x12345        ; x2 = 0x12345000
//   addi x2, x2, 0x678     ; x2 = 0x12345678
//   sw x2, 0(x1)           ; store value to reg slot 0
//   ebreak                 ; halt
const uint32_t prog_words[6] = {
  0x600080B7, 0x10C08093, 0x12345137, 0x67810113, 0x0020A023, 0x00100073
};

void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < 6; i++) {
    prog[i] = prog_words[i];
  }
  // Release the ULP core.
  *ulp_core = 1;

  uint32_t v = 0;
  for (int t = 0; t < 200000; t++) {
    v = *ulp_reg0;
    if (v == 0x12345678) break;
  }

  if (v == 0x12345678) {
    Serial.println("ULP POKE PASS");
  } else {
    Serial.print("ULP POKE FAIL got=0x");
    Serial.println(v, HEX);
  }
}

void loop() {
  Serial.println("DONE");
  delay(1000);
}
