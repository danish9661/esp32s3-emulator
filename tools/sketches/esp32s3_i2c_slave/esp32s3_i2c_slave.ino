// ESP32-S3 I2C0 slave-mode validation sketch (direct register pokes).
//
// Firmware puts I2CEXT0 (0x60013000) in slave mode (CTR.ms_mode clear),
// programs SLAVE_ADDR, preloads the TX FIFO, and polls
// INT_RAW.trans_complete. The HOST (run_flash with I2C_SLAVE_XCHG=1) drives
// both directions when it sees the UART markers: a master-write-to-slave on
// "I2C SLAVE READY", and a master-read-from-slave on "I2C SLAVE TX-REQ".
#include <Arduino.h>

#define I2C0_BASE 0x60013000u
#define I2C_CTR (*(volatile uint32_t*)(I2C0_BASE + 0x04))
#define I2C_SR (*(volatile uint32_t*)(I2C0_BASE + 0x08))
#define I2C_SLAVE_ADDR (*(volatile uint32_t*)(I2C0_BASE + 0x10))
#define I2C_DATA (*(volatile uint32_t*)(I2C0_BASE + 0x1C))
#define I2C_INT_RAW (*(volatile uint32_t*)(I2C0_BASE + 0x20))
#define I2C_INT_CLR (*(volatile uint32_t*)(I2C0_BASE + 0x24))

static int poll_done() {
  for (uint32_t t = 0; t < 20000000; t++) {
    if (I2C_INT_RAW & (1 << 7)) {
      return 1;
    }
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  I2C_CTR = 0;           // slave mode (ms_mode clear)
  I2C_SLAVE_ADDR = 0x42;
  I2C_DATA = 0xA5;       // TX preload for the master-read half
  I2C_INT_CLR = 0xFFFFFFFF;
  Serial.println("I2C SLAVE READY");
  if (!poll_done()) {
    Serial.println("I2C SLAVE TIMEOUT");
    return;
  }
  uint32_t b0 = I2C_DATA & 0xFF;
  uint32_t b1 = I2C_DATA & 0xFF;
  Serial.printf("I2C SLAVE RX=%02x %02x\n", b0, b1);
  Serial.printf("I2C SLAVE SR=%08x\n", I2C_SR);
  I2C_INT_CLR = 0xFFFFFFFF;
  I2C_DATA = 0xA5;  // reload TX (the read half pops it)
  Serial.println("I2C SLAVE TX-REQ");
  if (!poll_done()) {
    Serial.println("I2C SLAVE TIMEOUT");
    return;
  }
  Serial.printf("I2C SLAVE SR2=%08x\n", I2C_SR);
  Serial.println("I2C SLAVE DONE");
}

void loop() {}
