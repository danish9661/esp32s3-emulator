// ESP32-S3 SD/MMC host controller register validation for the emulator.
// The SDMMC host is at 0x60028000. We poke a few control/command/response
// registers and read them back. (Card command execution / DMA is not modeled;
// this only validates the register path.)

#define SDMMC_BASE 0x60028000
volatile uint32_t* ctrl = (volatile uint32_t*)(SDMMC_BASE + 0x00);
volatile uint32_t* cmd  = (volatile uint32_t*)(SDMMC_BASE + 0x2C);
volatile uint32_t* resp0 = (volatile uint32_t*)(SDMMC_BASE + 0x30);

void setup() {
  Serial.begin(115200);
  delay(200);

  *ctrl = 0x000F0001;
  *cmd  = 0x80200000;
  *resp0 = 0xCAFEBEEF;

  if (*ctrl != 0x000F0001) { Serial.println("SDMMC FAIL ctrl"); return; }
  if (*cmd  != 0x80200000) { Serial.println("SDMMC FAIL cmd"); return; }
  if (*resp0 != 0xCAFEBEEF) { Serial.println("SDMMC FAIL resp0"); return; }

  Serial.println("SDMMC PASS");
}

void loop() {
  Serial.println("DONE");
  delay(1000);
}
