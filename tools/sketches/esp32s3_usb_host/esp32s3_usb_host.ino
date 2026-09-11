// ESP32-S3 USB-OTG host enumeration validation for the emulator.
// The OTG block (0x60080000, DesignWare DWC2) fronts a simulated
// full-speed device. We drive host-mode enumeration with direct register
// pokes: force host mode, power the port (device connects), port reset
// (device enables), then control transfers on channel 0 — GET_DESCRIPTOR
// (device), SET_ADDRESS, GET_DESCRIPTOR (configuration), SET_CONFIGURATION
// — checking descriptors and status at each step. Markers: "USB HOST ENUM
// PASS" (any mismatch prints "USB HOST FAIL ..." with details).
// NOTE: SOF/frame timing, DMA, DATA-toggle enforcement and non-control
// endpoints are not modeled (documented approximations); the sketch uses
// PIO control transfers only, like real TinyUSB enumeration setup.

#define USB_BASE 0x60080000
#define GUSBCFG  (*(volatile uint32_t*)(USB_BASE + 0x00C))
#define GINTSTS  (*(volatile uint32_t*)(USB_BASE + 0x014))
#define GRXSTSP  (*(volatile uint32_t*)(USB_BASE + 0x020))
#define HCFG     (*(volatile uint32_t*)(USB_BASE + 0x400))
#define HAINT    (*(volatile uint32_t*)(USB_BASE + 0x414))
#define HPRT     (*(volatile uint32_t*)(USB_BASE + 0x440))
#define HCCHAR0  (*(volatile uint32_t*)(USB_BASE + 0x500))
#define HCINT0   (*(volatile uint32_t*)(USB_BASE + 0x508))
#define HCINTMSK0 (*(volatile uint32_t*)(USB_BASE + 0x50C))
#define HCTSIZ0  (*(volatile uint32_t*)(USB_BASE + 0x510))
#define DFIFO0   (*(volatile uint32_t*)(USB_BASE + 0x1000))

#define FORCE_HOST (1u << 29)
#define HPRT_PWR   (1u << 12)
#define HPRT_RST   (1u << 8)
#define HPRT_ENA   (1u << 2)
#define CHENA      (1u << 31)
#define XFERCOMPL  (1u << 0)
#define CHHALTED   (1u << 1)

static uint32_t fail = 0;

// Program channel 0 (control EP0, MPS 64) and run one transfer to `addr`.
static void ch_xfer(uint8_t addr, bool out, uint8_t pid, uint32_t xfer) {
  HCTSIZ0 = xfer | (1u << 19) | ((uint32_t)pid << 29);
  HCCHAR0 = 64 | (out ? 0u : (1u << 15)) | ((uint32_t)addr << 22) | CHENA;
  for (volatile int t = 0; t < 100000; t++) {
    if (HCINT0 & XFERCOMPL) break;
  }
  if (!(HCINT0 & XFERCOMPL)) {
    Serial.println("USB HOST XFER TIMEOUT");
    fail |= 0x8000u;
  }
  HCINT0 = XFERCOMPL | CHHALTED;  // W1C
}

// SETUP stage: stage the 8 packet bytes, then run with Pid=SETUP.
static void do_setup(uint8_t addr, uint32_t w0, uint32_t w1) {
  DFIFO0 = w0;
  DFIFO0 = w1;
  ch_xfer(addr, true, 3, 8);
}

// IN data stage: run and pop `n` bytes into buf (via GRXSTSP + DFIFO).
static void do_in(uint8_t addr, uint8_t *buf, uint32_t n) {
  ch_xfer(addr, false, 2, n);
  uint32_t st = GRXSTSP;
  uint32_t bcnt = (st >> 4) & 0x7FFu;
  if (bcnt != n) {
    Serial.print("USB HOST BCNT ");
    Serial.println(bcnt);
    fail |= 0x4000u;
  }
  for (uint32_t i = 0; i < (n + 3) / 4; i++) {
    uint32_t w = DFIFO0;
    for (uint32_t k = 0; k < 4 && 4 * i + k < n; k++) {
      buf[4 * i + k] = (uint8_t)(w >> (8 * k));
    }
  }
}

// Zero-length OUT status stage.
static void do_status_out(uint8_t addr) {
  ch_xfer(addr, true, 2, 0);
}

// Zero-length IN status stage.
static void do_status_in(uint8_t addr) {
  ch_xfer(addr, false, 2, 0);
  (void)GRXSTSP;
}

static uint8_t desc[64];

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: OTG engine frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 23); // USB-OTG
  delay(200);

  GUSBCFG |= FORCE_HOST;
  HCFG = 0x00000002;  // 48 MHz PHY clock value (readback check below)
  if ((HCFG & 3u) != 2u) {
    Serial.println("USB HOST HCFG MISMATCH");
    fail |= 1u;
  }
  // Power the port: the simulated device connects at full speed.
  HPRT = HPRT_PWR;
  for (volatile int t = 0; t < 100000 && !(HPRT & 1u); t++) {}
  if (!(HPRT & 1u) || !(HPRT & 2u)) {
    Serial.println("USB HOST NO CONNECT");
    fail |= 2u;
  }
  if (((HPRT >> 17) & 3u) != 1u) {
    Serial.println("USB HOST NOT FS");
    fail |= 4u;
  }
  // Port reset: self-clears, then enabled + enable-changed.
  HPRT = HPRT_PWR | HPRT_RST;
  for (volatile int t = 0; t < 100000 && (HPRT & HPRT_RST); t++) {}
  if ((HPRT & HPRT_RST) || !(HPRT & HPRT_ENA)) {
    Serial.println("USB HOST NO ENABLE");
    fail |= 8u;
  }
  HPRT = HPRT_PWR | (1u << 1) | (1u << 3);  // W1C connect/enable-change
  Serial.println("USB HOST PORT OK");

  HCINTMSK0 = XFERCOMPL | CHHALTED;
  // GET_DESCRIPTOR device (addr 0): 18 bytes, Espressif VID.
  // SETUP = 80 06 00 01 00 00 12 00.
  do_setup(0, 0x01000680u, 0x00120000u);
  do_in(0, desc, 18);
  Serial.print("USB HOST DEV VID=");
  Serial.println((desc[9] << 8) | desc[8], HEX);
  if (desc[0] != 18 || desc[1] != 1 ||
      desc[8] != 0x3A || desc[9] != 0x30 || desc[10] != 0x01 || desc[11] != 0x10) {
    Serial.println("USB HOST DESC MISMATCH");
    fail |= 16u;
  }
  do_status_out(0);
  Serial.println("USB HOST DESC OK");

  // SET_ADDRESS(7) + IN status applies it.
  // SETUP = 00 05 07 00 00 00 00 00.
  do_setup(0, 0x00070500u, 0x00000000u);
  do_status_in(0);
  // GET_DESCRIPTOR configuration at the new address (32 bytes).
  // SETUP = 80 06 00 02 00 00 20 00.
  do_setup(7, 0x02000680u, 0x00200000u);
  do_in(7, desc, 32);
  Serial.print("USB HOST CFG total=");
  Serial.println(desc[2]);
  if (desc[0] != 9 || desc[1] != 2 || desc[2] != 32 || desc[13] != 2) {
    Serial.println("USB HOST CFG MISMATCH");
    fail |= 32u;
  }
  do_status_out(7);
  Serial.println("USB HOST CFG OK");

  // SET_CONFIGURATION(1) + IN status.
  // SETUP = 00 09 01 00 00 00 00 00.
  do_setup(7, 0x00010900u, 0x00000000u);
  do_status_in(7);
  Serial.println("USB HOST SETCFG OK");

  // GET_DESCRIPTOR STRING idx1 (manufacturer "Espressif", UTF-16LE).
  // SETUP = 80 06 01 03 00 00 14 00.
  do_setup(7, 0x03010680u, 0x00140000u);
  do_in(7, desc, 20);
  do_status_out(7);
  if (desc[0] != 20 || desc[1] != 3 || desc[2] != 'E' || desc[3] != 0
      || desc[4] != 's') {
    Serial.println("USB HOST STR MISMATCH");
    fail |= 0x10000u;
  }
  Serial.println("USB HOST STR OK");

  // SOF frame counter advances while the controller clock runs.
  uint32_t sof0 = *(volatile uint32_t*)(USB_BASE + 0x408);
  delay(20);
  uint32_t sof1 = *(volatile uint32_t*)(USB_BASE + 0x408);
  Serial.printf("USB HOST SOF %lu -> %lu\n", (unsigned long)sof0, (unsigned long)sof1);
  if (sof1 == sof0) {
    Serial.println("USB HOST SOF MISMATCH");
    fail |= 0x20000u;
  }
  Serial.println("USB HOST SOF OK");

  if (fail == 0) {
    Serial.println("USB HOST ENUM PASS");
  } else {
    Serial.print("USB HOST FAIL bits=");
    Serial.println(fail, HEX);
  }
  Serial.println("USB HOST DONE");
}

void loop() {
  delay(1000);
}
