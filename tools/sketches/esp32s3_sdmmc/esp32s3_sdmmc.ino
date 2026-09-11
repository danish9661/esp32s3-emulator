// ESP32-S3 SD/MMC host controller validation for the emulator.
// The SDMMC host is at 0x60028000 (DesignWare MMC). We poke the command/
// response/FIFO registers to drive a modeled SD card through GO_IDLE ->
// SEND_IF_COND -> ACMD41 -> ALL_SEND_CID -> SEND_RCA -> SELECT ->
// READ_SINGLE_BLOCK, then a WRITE_BLOCK round-trip. This exercises the
// command/response FSM + PIO data path (DMA is not modeled).

#define SDMMC_BASE 0x60028000
// CMD bit positions (sdmmc_struct.h).
#define CMD_RESP_EXPECT  (1u << 6)
#define CMD_RESP_LONG    (1u << 7)
#define CMD_DATA_EXPECT  (1u << 9)
#define CMD_WRITE        (1u << 10)  // 0 = read from card, 1 = write to card
#define CMD_START        (1u << 31)

volatile uint32_t* reg_ctrl   = (volatile uint32_t*)(SDMMC_BASE + 0x00);
volatile uint32_t* reg_bytcnt = (volatile uint32_t*)(SDMMC_BASE + 0x20);
volatile uint32_t* reg_cmdarg = (volatile uint32_t*)(SDMMC_BASE + 0x28);
volatile uint32_t* reg_cmd    = (volatile uint32_t*)(SDMMC_BASE + 0x2C);
volatile uint32_t* reg_resp0  = (volatile uint32_t*)(SDMMC_BASE + 0x30);
volatile uint32_t* reg_fifo   = (volatile uint32_t*)(SDMMC_BASE + 0x100);

static inline void issue(uint8_t idx, uint32_t arg, uint32_t flags) {
  *reg_cmdarg = arg;
  *reg_cmd = (uint32_t)idx | CMD_START | flags;
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 7);
  delay(200);

  // GO_IDLE_STATE
  issue(0, 0, 0);
  // SEND_IF_COND
  issue(8, 0x000001AA, CMD_RESP_EXPECT);
  uint32_t r_if_cond = *reg_resp0;
  // ACMD41 = CMD55 + CMD41 (twice; second reports ready + SDHC).
  issue(55, 0, CMD_RESP_EXPECT);
  issue(41, 0x40FF8000, CMD_RESP_EXPECT);
  uint32_t ocr1 = *reg_resp0;
  issue(55, 0, CMD_RESP_EXPECT);
  issue(41, 0x40FF8000, CMD_RESP_EXPECT);
  uint32_t ocr2 = *reg_resp0;
  // ALL_SEND_CID (R2, long response)
  issue(2, 0, CMD_RESP_EXPECT | CMD_RESP_LONG);
  uint32_t cid0 = *reg_resp0;
  // SEND_RCA
  issue(3, 0, CMD_RESP_EXPECT);
  uint32_t rca = *reg_resp0;
  // SELECT/DESELECT
  issue(7, rca, CMD_RESP_EXPECT);

  // READ_SINGLE_BLOCK LBA 0 = MBR (FAT16 preformatted): check the
  // partition-type word (bytes 0x1C0..0x1C3 = 00 00 06 00) and the
  // 0x55AA signature word (bytes 508..511 = 00 00 55 AA).
  *reg_bytcnt = 512;
  issue(17, 0, CMD_RESP_EXPECT | CMD_DATA_EXPECT);
  uint32_t bad = 0;
  for (int i = 0; i < 128; i++) {
    uint32_t w = *reg_fifo;
    if (i == 112 && w != 0x00060000u) bad++;
    if (i == 127 && w != 0xAA550000u) bad++;
  }

  // WRITE_BLOCK: data_expect, write direction, then read back.
  *reg_bytcnt = 512;
  issue(24, 0, CMD_RESP_EXPECT | CMD_DATA_EXPECT | CMD_WRITE);
  for (int i = 0; i < 128; i++) {
    *reg_fifo = (uint32_t)i * 0x01010101u;
  }
  *reg_bytcnt = 512;
  issue(17, 0, CMD_RESP_EXPECT | CMD_DATA_EXPECT);
  uint32_t bad2 = 0;
  for (int i = 0; i < 128; i++) {
    uint32_t w = *reg_fifo;
    if (w != (uint32_t)i * 0x01010101u) bad2++;
  }

  if (bad == 0 && bad2 == 0 && (ocr2 & (1u << 31)) && cid0 == 0x12345678) {
    Serial.println("SDMMC PASS");
  } else {
    Serial.print("SDMMC FAIL bad=");
    Serial.print(bad);
    Serial.print(" bad2=");
    Serial.print(bad2);
    Serial.print(" ocr2=");
    Serial.print(ocr2, HEX);
    Serial.print(" ifcond=");
    Serial.print(r_if_cond, HEX);
    Serial.print(" cid0=");
    Serial.println(cid0, HEX);
  }
}

void loop() {
  Serial.println("DONE");
  delay(1000);
}
