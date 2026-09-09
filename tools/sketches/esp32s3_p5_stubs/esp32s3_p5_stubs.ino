// Direct register-poke validation for the emulator's P5 "register-store"
// peripherals: SENSITIVE (=PMS), WCL (=World Controller), PERI_BACKUP,
// SYSCON (=PCR/clock control), I2S0, I2S1, ASSIST_DEBUG, and LCD_CAM.
// Each is modeled as a RegStore that retains writes and reads them back, so
// firmware poking them during boot/init never panics. We poke two registers
// per block and verify the round-trip (P5 "poke" pattern).

#define SENSITIVE_BASE   0x600C1000
#define WCL_BASE         0x600D0000
#define PERI_BACKUP_BASE 0x6002A000
#define SYSCON_BASE      0x60026000
#define I2S0_BASE        0x6000F000
#define I2S1_BASE        0x6002D000
#define ASSIST_DEBUG_BASE 0x600CE000
#define LCD_CAM_BASE     0x60041000

bool check(const char* name, uint32_t base) {
  REG_WRITE(base + 0x10, 0x12345678);
  REG_WRITE(base + 0x24, 0xDEADBEEF);
  uint32_t a = REG_READ(base + 0x10);
  uint32_t b = REG_READ(base + 0x24);
  bool ok = (a == 0x12345678) && (b == 0xDEADBEEF);
  Serial.print(name);
  Serial.print(ok ? " OK" : " FAIL");
  Serial.print(" (0x10=");
  Serial.print(a, HEX);
  Serial.print(" 0x24=");
  Serial.println(b, HEX);
  return ok;
}

// I2S0/1 are functional models now (not RegStores): INT_ST (0x10) reads
// RAW&ENA, the FIFOs pop, and TX/RX_CONF (0x24/0x20) self-clear
// reset/start/update bits, so none round-trip. Poke two plain config regs
// instead (CONF1 0x28/0x2C, plain field stores).
bool check_i2s(const char* name, uint32_t base) {
  REG_WRITE(base + 0x2C, 0x12345678);
  REG_WRITE(base + 0x28, 0xDEADBEEB);
  uint32_t a = REG_READ(base + 0x2C);
  uint32_t b = REG_READ(base + 0x28);
  bool ok = (a == 0x12345678) && (b == 0xDEADBEEB);
  Serial.print(name);
  Serial.print(ok ? " OK" : " FAIL");
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  bool all = true;
  all &= check("SENSITIVE", SENSITIVE_BASE);
  all &= check("WCL", WCL_BASE);
  all &= check("PERI_BACKUP", PERI_BACKUP_BASE);
  all &= check("SYSCON", SYSCON_BASE);
  all &= check_i2s("I2S0", I2S0_BASE);
  all &= check_i2s("I2S1", I2S1_BASE);
  all &= check("ASSIST_DEBUG", ASSIST_DEBUG_BASE);
  all &= check("LCD_CAM", LCD_CAM_BASE);

  Serial.println(all ? "P5 STUBS POKE PASS" : "P5 STUBS POKE FAIL");
  Serial.println("DONE");
}

void loop() {}
