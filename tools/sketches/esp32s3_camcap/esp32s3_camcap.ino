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

void setup() {
  Serial.begin(115200);
  delay(80);
  Serial.println("CAMCAP START");
  *(volatile uint32_t *)LC_INT_ENA = VSYNC_INT | HS_INT;

  bool plain = capture_round(0, FRAME, "PLAIN");
  Serial.printf("CAMCAP PLAIN %s\n", plain ? "PASS" : "MISMATCH");

  for (int i = 0; i < 8; i++) swapped[i] = swap32(FRAME[i]);
  bool swap = capture_round(CAM_BYTE_ORDER, swapped, "SWAP");
  Serial.printf("CAMCAP SWAP %s\n", swap ? "PASS" : "MISMATCH");

  if (plain && swap) Serial.println("CAMCAP PASS");
  Serial.println("CAMCAP DONE");
}

void loop() {
  delay(500);
}
