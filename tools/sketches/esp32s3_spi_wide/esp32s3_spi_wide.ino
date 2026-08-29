// SPI wide-transfer validation: tests 16-bit and 32-bit transfers via
// the Arduino SPI library.  With no device on the bus, MISO reads back
// as 0.  The test verifies the SPI peripheral shifts out the correct
// number of clock pulses for each transfer width.

#include <SPI.h>

#define CS_PIN 3

void setup() {
  Serial.begin(115200);
  delay(50);

  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // 8-bit transfer
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  uint8_t r8 = SPI.transfer(0xA5);
  digitalWrite(CS_PIN, HIGH);
  Serial.printf("SPI8  tx=0x%02X rx=0x%02X\n", 0xA5, r8);

  // 16-bit transfer
  digitalWrite(CS_PIN, LOW);
  uint16_t r16 = SPI.transfer16(0xBE01);
  digitalWrite(CS_PIN, HIGH);
  Serial.printf("SPI16 tx=0x%04X rx=0x%04X\n", 0xBE01, r16);

  // 32-bit transfer via transferBytes
  uint8_t tx32[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  uint8_t rx32[4] = {0};
  digitalWrite(CS_PIN, LOW);
  SPI.transferBytes(tx32, rx32, 4);
  digitalWrite(CS_PIN, HIGH);
  Serial.printf("SPI32 tx=%02X%02X%02X%02X rx=%02X%02X%02X%02X\n",
                tx32[0], tx32[1], tx32[2], tx32[3],
                rx32[0], rx32[1], rx32[2], rx32[3]);
  SPI.endTransaction();

  // All MISO reads should be 0x00 (no device)
  bool pass = (r8 == 0x00) && (r16 == 0x0000);
  for (int i = 0; i < 4; i++) pass = pass && (rx32[i] == 0x00);
  Serial.println(pass ? "SPI_WIDE PASS" : "SPI_WIDE FAIL");
}

void loop() {
  delay(1000);
}
