// I2C validation sketch for the ESP32-S3 emulator.
// Exercises the real Arduino Wire library against the emulated I2CEXT0
// controller: a bus scan (NACK on every address) plus a dummy write.
// Prints markers so the emulator run can confirm the I2C FSM completes
// (START/STOP/clock + trans_complete interrupt) under real driver code.
#include <Wire.h>

void setup() {
  // Claim I2C pins first, then Serial, so UART0 ends up owning the
  // (collapsed) 43/44 pins and we can still print the result.
  Wire.begin(8, 9);
  Serial.begin(115200);
  Serial.println("I2C START");

  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    uint8_t err = Wire.endTransmission();
    if (err == 0) found++;
  }
  Serial.printf("I2C SCAN done found=%d\n", found);

  Wire.beginTransmission(0x42);
  Wire.write(0xAA);
  uint8_t e = Wire.endTransmission();
  Serial.printf("I2C WRITE err=%d\n", e);

  Serial.println("I2C DONE");
}

void loop() { delay(1000); }
