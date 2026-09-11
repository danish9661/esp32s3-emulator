// LCD_CAM camera-capture validation for the emulator.
//
// Drives the camera (RX) path with direct register pokes (per
// lcd_cam_reg.h): CAM_CTRL (CLK_EN + UPDATE), CAM_CTRL1 (LINE_INT_NUM,
// START), CAM_FIFO_STATUS polling, CAM_DATA reads, LC_INT_RAW VSYNC/HS
// checks. The frame bytes come from a virtual camera (see
// web/virtual_devices.js + tools/camcap_harness.mjs), which stages two
// identical 8-word frames before the run:
//   round 1 (plain)    -> expect exact words + VSYNC/HS latched
//   round 2 (BYTE_ORDER) -> expect byte-swapped words
// Run under run_flash only via the node harness (needs JS-side staging).

#define LCD_CAM_BASE 0x60041000UL
#define GDMA_BASE 0x6003F000UL
// GDMA ch2 IN link (ch stride 0xC0, IN block +0x00): link @ +0x20,
// peri_sel @ +0x48, IN start = bit 22 (see the gdma/uhci sketches).
#define G_IN_LINK    (GDMA_BASE + 2 * 0xC0 + 0x20)
#define G_IN_PERI    (GDMA_BASE + 2 * 0xC0 + 0x48)
#define GDMA_CAM_PERI 5  // LCD_CAM shares peri 5 (IN = camera RX)
#define CAM_CTRL       (LCD_CAM_BASE + 0x04)
#define CAM_CTRL1      (LCD_CAM_BASE + 0x08)
#define CAM_DATA       (LCD_CAM_BASE + 0x48)
#define CAM_FIFO_STATUS (LCD_CAM_BASE + 0x4C)
#define LC_INT_ENA     (LCD_CAM_BASE + 0x64)
#define LC_INT_RAW     (LCD_CAM_BASE + 0x68)
#define LC_INT_CLR     (LCD_CAM_BASE + 0x70)

#define CAM_CLK_EN    (1u << 31)
#define CAM_UPDATE    (1u << 4)
#define CAM_BYTE_ORDER (1u << 5)
#define CAM_START     (1u << 29)
#define LINE_INT_NUM(n) (((uint32_t)(n) & 0x3Fu) << 16)
#define VSYNC_INT     (1u << 2)
#define HS_INT        (1u << 3)

static const uint32_t FRAME[8] = {
  0x01020304, 0x11223344, 0xA5A5A5A5, 0xDEADBEEF,
  0x12345678, 0x00000000, 0xFFFFFFFF, 0x5A5A5A5A
};

static inline uint32_t swap32(uint32_t w) {
  return ((w & 0xFFu) << 24) | (((w >> 8) & 0xFFu) << 16) |
         (((w >> 16) & 0xFFu) << 8) | ((w >> 24) & 0xFFu);
}

// Capture 8 words with `ctrl` CAM_CTRL bits set; expect `want[i]`.
// Returns true on full match (prints every word either way).
static bool capture_round(uint32_t ctrl, const uint32_t *want, const char *tag) {
  *(volatile uint32_t *)CAM_CTRL = CAM_CLK_EN | CAM_UPDATE | ctrl;
  *(volatile uint32_t *)LC_INT_CLR = 0xF;
  *(volatile uint32_t *)CAM_CTRL1 = LINE_INT_NUM(7) | CAM_START;
  uint32_t n = 0;
  while (n < 200000u) {
    if (((*(volatile uint32_t *)CAM_FIFO_STATUS) & 0x7FFu) >= 8) break;
    n++;
  }
  if (n >= 200000u) {
    Serial.println("CAMCAP TIMEOUT");
    return false;
  }
  uint32_t raw = *(volatile uint32_t *)LC_INT_RAW;
  if (!(raw & VSYNC_INT) || !(raw & HS_INT)) {
    Serial.println("CAMCAP IRQ MISSING");
    return false;
  }
  bool ok = true;
  for (int i = 0; i < 8; i++) {
    uint32_t w = *(volatile uint32_t *)CAM_DATA;
    Serial.printf("CAMCAP %s W%d=%08X\n", tag, i, w);
    if (w != want[i]) ok = false;
  }
  *(volatile uint32_t *)LC_INT_CLR = VSYNC_INT | HS_INT;
  return ok;
}

static uint32_t swapped[8];

// GDMA-RX capture leg (runs FIRST, consumes one staged frame): program a
// ch2 IN descriptor (32 B, eof, owned) for the camera peri, then START the
// camera — order matters, the link must arm before streaming begins — and
// poll the descriptor owner bit for the pump's completion.
static volatile uint32_t gdma_desc[4];
static volatile uint32_t gdma_buf[8];

static bool capture_gdma(const uint32_t *want, const char *tag) {
  for (int i = 0; i < 8; i++) gdma_buf[i] = 0;
  gdma_desc[0] = (1u << 31) | (1u << 30) | (32u << 12);  // owner+eof+len
  gdma_desc[1] = (uint32_t)gdma_buf;
  gdma_desc[2] = 0;
  gdma_desc[3] = 0;
  *(volatile uint32_t *)G_IN_PERI = GDMA_CAM_PERI;
  uint32_t lsb = ((uint32_t)gdma_desc) & 0x000FFFFFu;
  *(volatile uint32_t *)G_IN_LINK = lsb | (1u << 22);  // IN start
  *(volatile uint32_t *)CAM_CTRL = CAM_CLK_EN | CAM_UPDATE;
  *(volatile uint32_t *)LC_INT_CLR = 0xF;
  *(volatile uint32_t *)CAM_CTRL1 = LINE_INT_NUM(7) | CAM_START;
  uint32_t t0 = millis();
  while (gdma_desc[0] & (1u << 31)) {
    if (millis() - t0 > 2000) {
      Serial.println("CAMCAP GDMA TIMEOUT");
      return false;
    }
  }
  bool ok = true;
  for (int i = 0; i < 8; i++) {
    Serial.printf("CAMCAP %s W%d=%08X\n", tag, i, gdma_buf[i]);
    if (gdma_buf[i] != want[i]) ok = false;
  }
  return ok;
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 8);  // LCD_CAM
  *(volatile uint32_t*)(0x600C001C) |= (1u << 6);  // GDMA (DMA clock)
  delay(80);
  Serial.println("CAMCAP START");
  *(volatile uint32_t *)LC_INT_ENA = VSYNC_INT | HS_INT;

  bool gdma = capture_gdma(FRAME, "GDMA");
  Serial.printf("CAMCAP GDMA %s\n", gdma ? "PASS" : "MISMATCH");

  bool plain = capture_round(0, FRAME, "PLAIN");
  Serial.printf("CAMCAP PLAIN %s\n", plain ? "PASS" : "MISMATCH");

  for (int i = 0; i < 8; i++) swapped[i] = swap32(FRAME[i]);
  bool swap = capture_round(CAM_BYTE_ORDER, swapped, "SWAP");
  Serial.printf("CAMCAP SWAP %s\n", swap ? "PASS" : "MISMATCH");

  if (gdma && plain && swap) Serial.println("CAMCAP PASS");
  Serial.println("CAMCAP DONE");
}

void loop() {
  delay(500);
}
