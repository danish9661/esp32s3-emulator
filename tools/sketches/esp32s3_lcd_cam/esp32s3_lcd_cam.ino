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

#define GDMA_BASE 0x6003F000UL // single GDMA controller (DR_REG_GDMA_BASE)
#define G_OUT_PERI(ch) (GDMA_BASE + (ch) * 0xC0 + 0x60 + 0x48)
#define G_OUT_LINK(ch) (GDMA_BASE + (ch) * 0xC0 + 0x60 + 0x20)

typedef struct {
  uint32_t dw0;
  uint32_t buf;
  uint32_t next;
  uint32_t rsvd;
} gdma_desc_t;

// 8-word GDMA payload (exercises multi-word FIFO streaming).
static volatile uint32_t g_lcd_words[8] = {
  0x11111111, 0x22222222, 0x33333333, 0x44444444,
  0x55555555, 0x66666666, 0x77777777, 0x88888888
};
static volatile gdma_desc_t g_lcd_desc;

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 8);
  *(volatile uint32_t*)(0x600C001C) |= (1u << 6);
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

  // GDMA-fed 8080 transfer (peri_sel = 5 = LCD): the OUT walk streams the
  // 8 descriptor words through the TX FIFO and latches TRANS_DONE.
  g_lcd_desc.dw0 = (32) | ((8 * 4) << 12) | (1u << 30) | (1u << 31);
  g_lcd_desc.buf = (uint32_t)g_lcd_words;
  g_lcd_desc.next = 0;
  g_lcd_desc.rsvd = 0;
  REG_WRITE(G_OUT_PERI(0), 5);
  uint32_t lsb = ((uint32_t)&g_lcd_desc) & 0x000FFFFFu;
  REG_WRITE(G_OUT_LINK(0), lsb);
  REG_WRITE(G_OUT_LINK(0), lsb | (1u << 21));
  bool gdone = false;
  for (int i = 0; i < 1000000; i++) {
    if (REG_READ(LC_INT_ST) & TRANS_DONE) { gdone = true; break; }
  }
  uint32_t gcnt = REG_READ(LCD_FIFO_STATUS) & 0x7FF;
  Serial.print("LCD GDMA done=");
  Serial.print(gdone);
  Serial.print(" fifo=");
  Serial.println(gcnt);
  Serial.println((gdone && gcnt == 0) ? "LCD GDMA PASS" : "LCD GDMA FAIL");
  Serial.println("DONE");
}

void loop() {}
