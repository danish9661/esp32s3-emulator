// ESP32-S3 GPSPI2 slave-mode validation sketch (direct register pokes).
//
// Firmware puts GPSPI2 (0x60024000) in slave mode (SPI_SLAVE.slave_mode),
// preloads the TX buffer, and polls INT_RAW.trans_done. The HOST (run_flash
// with SPI_SLAVE_XCHG=1) drives both directions of the exchange when it sees
// the UART markers: a master-write-to-slave on "SPI SLAVE READY", and a
// master-read-from-slave on "SPI SLAVE TX-REQ".
#include <Arduino.h>

#define SPI2_BASE 0x60024000u
#define SPI_SLAVE_REG (*(volatile uint32_t*)(SPI2_BASE + 0xE0))
#define SPI_SLAVE1_REG (*(volatile uint32_t*)(SPI2_BASE + 0xE4))
#define SPI_W0 (*(volatile uint32_t*)(SPI2_BASE + 0x98))
#define SPI_INT_RAW (*(volatile uint32_t*)(SPI2_BASE + 0x3C))
#define SPI_INT_CLR (*(volatile uint32_t*)(SPI2_BASE + 0x38))

static int poll_done() {
  for (uint32_t t = 0; t < 20000000; t++) {
    if (SPI_INT_RAW & 1) {
      return 1;
    }
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  SPI_SLAVE_REG = (1 << 26);  // slave_mode (spi_struct.h `slave`)
  SPI_W0 = 0xA5C30000;        // TX preload for the master-read half
  SPI_INT_CLR = 1;
  Serial.println("SPI SLAVE READY");
  if (!poll_done()) {
    Serial.println("SPI SLAVE TIMEOUT");
    return;
  }
  Serial.printf("SPI SLAVE RX=%08x\n", SPI_W0);
  Serial.printf("SPI SLAVE RXLEN=%u\n", SPI_SLAVE1_REG & 0x3FFFF);
  SPI_INT_CLR = 1;
  // Reload the TX buffer: the master-write captured into the shared data
  // buffer, so the master-read half shifts out whatever is preloaded now.
  SPI_W0 = 0xA5C30000;
  Serial.println("SPI SLAVE TX-REQ");
  if (!poll_done()) {
    Serial.println("SPI SLAVE TIMEOUT");
    return;
  }
  Serial.printf("SPI SLAVE TXLEN=%u\n", SPI_SLAVE1_REG & 0x3FFFF);
  Serial.println("SPI SLAVE DONE");
}

void loop() {}
