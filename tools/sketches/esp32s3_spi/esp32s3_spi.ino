// SPI validation sketch for the ESP32-S3 emulator.
// Exercises the real Arduino SPI library against the emulated GPSPI2
// controller: a USR transfer (clock phases + MOSI shift) with no device
// attached (MISO reads 0). Prints markers so the emulator run can confirm
// the SPI FSM completes (usr self-clears + done interrupt) under real
// driver code.
#include <SPI.h>

static const int PIN_SCK = 10;
static const int PIN_MISO = 11;
static const int PIN_MOSI = 12;
static const int PIN_SS = 13;

void setup() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
  Serial.begin(115200);
  Serial.println("SPI START");

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  uint8_t r = SPI.transfer(0x55);
  SPI.endTransaction();
  Serial.printf("SPI transfer(0x55)=0x%02X\n", r);

  digitalWrite(PIN_SS, LOW);
  uint8_t r2 = SPI.transfer(0xAA);
  digitalWrite(PIN_SS, HIGH);
  Serial.printf("SPI transfer(0xAA)=0x%02X\n", r2);

  Serial.println("SPI DONE");
}

void loop() { delay(1000); }
