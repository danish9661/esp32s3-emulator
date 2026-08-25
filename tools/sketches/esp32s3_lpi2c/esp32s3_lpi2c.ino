// Direct register-poke validation for the emulator's RTC_I2C (LP/I2C) model.
// The LP/I2C block lives at 0x6000_8C00.  We just write a few timing/control
// registers and read them back; the emulator models it as a register store, so
// the values must round-trip.  This mirrors the P5 "poke" pattern used for
// SDMMC/RTC_IO/Deep-sleep.

#define RTC_I2C_BASE 0x60008C00
#define I2C_SCL_LOW_REG  (RTC_I2C_BASE + 0x00)
#define I2C_SCL_HIGH_REG (RTC_I2C_BASE + 0x04)
#define I2C_MS_DELAY_REG (RTC_I2C_BASE + 0x08)
#define I2C_CTRL_REG     (RTC_I2C_BASE + 0x0C)

void setup() {
  Serial.begin(115200);
  delay(50);

  REG_WRITE(I2C_SCL_LOW_REG, 0x00000032);
  REG_WRITE(I2C_SCL_HIGH_REG, 0x00000064);
  REG_WRITE(I2C_MS_DELAY_REG, 0x00000010);
  REG_WRITE(I2C_CTRL_REG, 0x00FF00AA);

  uint32_t low = REG_READ(I2C_SCL_LOW_REG);
  uint32_t high = REG_READ(I2C_SCL_HIGH_REG);
  uint32_t md = REG_READ(I2C_MS_DELAY_REG);
  uint32_t ctrl = REG_READ(I2C_CTRL_REG);

  if (low == 0x32 && high == 0x64 && md == 0x10 && ctrl == 0x00FF00AA) {
    Serial.println("LP I2C POKE PASS");
  } else {
    Serial.print("LP I2C POKE FAIL low=");
    Serial.print(low, HEX);
    Serial.print(" high=");
    Serial.print(high, HEX);
    Serial.print(" md=");
    Serial.print(md, HEX);
    Serial.print(" ctrl=");
    Serial.println(ctrl, HEX);
  }
  Serial.println("DONE");
}

void loop() {}
