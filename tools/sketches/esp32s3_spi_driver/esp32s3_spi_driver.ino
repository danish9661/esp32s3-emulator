// SPI master driver (esp-idf SPI master via Arduino `SPI` library) validation
// for the ESP32-S3 emulator. The Arduino SPI library drives the peripheral with
// the polling USR path (no GDMA), so this exercises spi_bus_initialize +
// spi_device polling transmit through the modeled GPSPI registers. With no real
// device on the bus, MISO reads back 0; the goal is that the driver completes
// (does not hang) and returns a value.

#include <SPI.h>

void setup() {
  Serial.begin(115200);

  SPI.begin();  // default FSPI bus + pins

  // Single-byte transfer.
  uint8_t r = SPI.transfer(0x55);
  Serial.printf("SPI DRIVER transfer(0x55)=0x%02x\n", r);

  // Multi-byte transfer (tx -> rx buffers).
  uint8_t tx[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  uint8_t rx[4] = {0};
  SPI.transferBytes(tx, rx, 4);
  Serial.printf("SPI DRIVER multi rx=%02x %02x %02x %02x\n",
                 rx[0], rx[1], rx[2], rx[2]);

  // A transaction (begin/end) with a word-length transfer.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  uint16_t w = SPI.transfer16(0x1234);
  SPI.endTransaction();
  Serial.printf("SPI DRIVER transfer16=0x%04x\n", w);

  SPI.end();
  Serial.println("SPI DRIVER PASS");
}

void loop() {}
