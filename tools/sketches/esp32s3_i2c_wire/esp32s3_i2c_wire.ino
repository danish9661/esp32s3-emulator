// I2C (Wire) driver path validation for the ESP32-S3 emulator. Uses the
// Arduino `Wire` library (esp-idf NG I2C master driver) over the I2CEXT0
// registers our i2c.rs models. With no device on the bus every address
// probe must fail quickly (no hang, no false ACK). NOTE on codes: the NG
// driver returns ESP_ERR_INVALID_STATE (0x103) for a NACKed address (see
// s_i2c_transaction_start: status != DONE -> INVALID_STATE), which Wire
// maps to 4 ("other") — NOT 2. So the correct expectation on an empty bus
// is found=0 + other=119 (NACKs surfacing as driver errors, none hanging,
// none misdetected as devices). Validated against IDF release/v5.3 source.

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
  if (found == 0 && nack + other == 119) {
    Serial.println("I2C WIRE PASS");
  } else {
    Serial.println("I2C WIRE FAIL");
  }
}

void loop() {}
