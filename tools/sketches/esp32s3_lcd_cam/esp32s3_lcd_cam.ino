// LCD_CAM (= PARLIO / parallel-I/O) functional poke test.
// Builds with arduino-cli (esp32:esp32:esp32s3) and runs under run_flash.
// Exercises the TX FIFO + transfer-start / transfer-done interrupt path of
// the emulator's lcd_cam.rs model.

#define LCD_CAM_BASE 0x60041000
#define LCD_USER       (LCD_CAM_BASE + 0x14)
#define LCD_DATA       (LCD_CAM_BASE + 0x40)
#define LCD_FIFO_STATUS (LCD_CAM_BASE + 0x44)
#define LC_INT_ENA     (LCD_CAM_BASE + 0x64)
#define LC_INT_RAW     (LCD_CAM_BASE + 0x68)
#define LC_INT_ST      (LCD_CAM_BASE + 0x6C)
#define LC_INT_CLR     (LCD_CAM_BASE + 0x70)
#define LCD_START_BIT  (1u << 27)
#define TRANS_DONE     (1u << 1)

void setup() {
  Serial.begin(115200);
  delay(80);

  REG_WRITE(LC_INT_ENA, TRANS_DONE);   // enable TRANS_DONE interrupt
  REG_WRITE(LC_INT_CLR, 0xF);          // clear any pending
  // Fill the TX FIFO with 3 words.
  REG_WRITE(LCD_DATA, 0x11111111);
  REG_WRITE(LCD_DATA, 0x22222222);
  REG_WRITE(LCD_DATA, 0x33333333);
  uint32_t cnt = REG_READ(LCD_FIFO_STATUS) & 0x7FF;

  // Start a transfer.
  uint32_t user = REG_READ(LCD_USER);
  REG_WRITE(LCD_USER, user | LCD_START_BIT);

  // Poll for the transfer-done interrupt (masked view).
  bool done = false;
  for (int i = 0; i < 1000; i++) {
    if (REG_READ(LC_INT_ST) & TRANS_DONE) { done = true; break; }
  }
  uint32_t raw = REG_READ(LC_INT_RAW);
  // Clear it and confirm.
  REG_WRITE(LC_INT_CLR, TRANS_DONE);
  uint32_t raw2 = REG_READ(LC_INT_RAW);

  bool ok = (cnt == 3) && done && (raw & TRANS_DONE) && !(raw2 & TRANS_DONE);
  Serial.print("LCD_CAM fifo=");
  Serial.print(cnt);
  Serial.print(" done=");
  Serial.print(done);
  Serial.print(" raw=");
  Serial.println(raw, HEX);
  Serial.println(ok ? "LCD CAM POKE PASS" : "LCD CAM POKE FAIL");
  Serial.println("DONE");
}

void loop() {}
