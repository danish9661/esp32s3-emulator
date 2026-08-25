// I2S0 (audio) functional poke test.
// Builds with arduino-cli (esp32:esp32:esp32s3) and runs under run_flash.
// Pushes a word to the TX FIFO and starts a transmission; polls the
// tx_done interrupt (bit 1 of INT_RAW). Validates the FIFO + serial
// shift-out + interrupt path of the emulator's i2s.rs model.

#define I2S0_BASE 0x6000F000
#define I2S_FIFO     (I2S0_BASE + 0x80)
#define I2S_TX_CONF  (I2S0_BASE + 0x24)
#define I2S_INT_ENA  (I2S0_BASE + 0x14)
#define I2S_INT_RAW  (I2S0_BASE + 0x0C)
#define I2S_INT_CLR  (I2S0_BASE + 0x18)
#define TX_START_BIT (1u << 2)
#define TX_DONE      (1u << 1)

// Route a GPIO pin to the I2S0 SD output signal (sig 25) so the serial
// data is observable on the matrix.
#define GPIO_BASE         0x60004000
#define GPIO_FUNC_OUT_SEL0 (GPIO_BASE + 0x554)
#define GPIO_ENABLE_W1TS  (GPIO_BASE + 0x24)
#define SD_PIN 2

void setup() {
  Serial.begin(115200);
  delay(80);

  REG_WRITE(GPIO_FUNC_OUT_SEL0 + 4 * SD_PIN, 25); // I2S0O_SD_OUT_IDX
  REG_WRITE(GPIO_ENABLE_W1TS, (1u << SD_PIN));

  REG_WRITE(I2S_INT_ENA, TX_DONE);
  REG_WRITE(I2S_INT_CLR, 0xF);
  REG_WRITE(I2S_FIFO, 0xABCD);
  REG_WRITE(I2S_TX_CONF, TX_START_BIT);

  bool done = false;
  for (int i = 0; i < 4000; i++) {
    if (REG_READ(I2S_INT_RAW) & TX_DONE) {
      done = true;
      break;
    }
  }
  uint32_t raw = REG_READ(I2S_INT_RAW);
  REG_WRITE(I2S_INT_CLR, TX_DONE);
  uint32_t raw2 = REG_READ(I2S_INT_RAW);

  bool ok = done && (raw & TX_DONE) && !(raw2 & TX_DONE);
  Serial.print("I2S done=");
  Serial.print(done);
  Serial.print(" raw=");
  Serial.println(raw, HEX);
  Serial.println(ok ? "I2S POKE PASS" : "I2S POKE FAIL");
  Serial.println("DONE");
}

void loop() {}
