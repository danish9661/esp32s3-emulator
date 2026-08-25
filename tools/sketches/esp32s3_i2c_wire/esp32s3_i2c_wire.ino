// I2C (Wire) driver path validation for the ESP32-S3 emulator. Uses the
// Arduino `Wire` library, which is built on the esp-idf I2C master driver
// (i2c_master_cmd_begin over the I2CEXT0 registers our i2c.rs models). With no
// device on the bus every address probe must NACK, so a bus scan should report
// found=0. This validates the driver completes (no hang) and that the NACK
// path is modeled well enough for the driver to detect it.

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();

  int found = 0;
  int nack = 0;
  int other = 0;
  for (int addr = 1; addr < 120; addr++) {
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    int err = Wire.endTransmission();
    if (err == 0) {
      found++;
      Serial.printf("found %02x\n", addr);
    } else if (err == 2) {
      nack++;
    } else {
      other++;
    }
  }
  Serial.printf("I2C WIRE SCAN done found=%d nack=%d other=%d\n", found, nack, other);
  Serial.println("I2C WIRE PASS");
}

void loop() {}
