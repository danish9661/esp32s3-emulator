// ESP32-S3 eMMC validation for the emulator.
// The SDMMC host (0x60028000, DesignWare MMC) fronts a simulated card with
// two personalities: the first MMC-only CMD1 switches it from SD to MMC
// mode (no SD flow ever sends CMD1). We drive the JEDEC MMC init sequence
// CMD0 -> CMD1 (poll OCR busy) -> CMD2 (CID) -> CMD3 (host-assigned RCA)
// -> CMD7 (select) -> CMD9 (MMC CSD) -> CMD8 (SEND_EXT_CSD, 512 B) ->
// CMD6 SWITCH (bus width + HS timing, verified in the EXT_CSD shadow) ->
// CMD16 + CMD24/CMD17 block round-trip. Markers: "EMMC PASS" (any mismatch
// prints "EMMC FAIL ..." with details).
// NOTE: the Arduino SD_MMC stack always takes the SD path (ACMD41
// succeeds), so a full IDF-driver MMC mount is unreachable — this is the
// direct-poke path, like TWAI/HMAC/DS before it.

#include "mbedtls/sha256.h"

#define SDMMC_BASE 0x60028000
#define CMD_RESP_EXPECT  (1u << 6)
#define CMD_RESP_LONG    (1u << 7)
#define CMD_DATA_EXPECT  (1u << 9)
#define CMD_WRITE        (1u << 10)
#define CMD_START        (1u << 31)

volatile uint32_t* reg_bytcnt = (volatile uint32_t*)(SDMMC_BASE + 0x20);
volatile uint32_t* reg_cmdarg = (volatile uint32_t*)(SDMMC_BASE + 0x28);
volatile uint32_t* reg_cmd    = (volatile uint32_t*)(SDMMC_BASE + 0x2C);
volatile uint32_t* reg_resp0  = (volatile uint32_t*)(SDMMC_BASE + 0x30);
volatile uint32_t* reg_resp3  = (volatile uint32_t*)(SDMMC_BASE + 0x3C);
volatile uint32_t* reg_fifo   = (volatile uint32_t*)(SDMMC_BASE + 0x100);

static inline void issue(uint8_t idx, uint32_t arg, uint32_t flags) {
  *reg_cmdarg = arg;
  *reg_cmd = (uint32_t)idx | CMD_START | flags;
}

// Read the 512-byte EXT_CSD (CMD8) into buf.
static void read_ext_csd(uint8_t *buf) {
  *reg_bytcnt = 512;
  issue(8, 0, CMD_RESP_EXPECT | CMD_DATA_EXPECT);
  for (int i = 0; i < 128; i++) {
    uint32_t w = *reg_fifo;
    buf[4 * i] = (uint8_t)w;
    buf[4 * i + 1] = (uint8_t)(w >> 8);
    buf[4 * i + 2] = (uint8_t)(w >> 16);
    buf[4 * i + 3] = (uint8_t)(w >> 24);
  }
}

static uint8_t ext_buf[512];

// HMAC-SHA256 via one-shot mbedtls calls (ipad/inner, opad/outer).
static uint8_t rpmb_key[32];

// HMAC-SHA256 via one-shot mbedtls calls (ipad/inner, opad/outer).
static void rpmb_hmac(const uint8_t *msg, size_t len, uint8_t out[32]) {
  uint8_t inner[32], kpad[64];
  for (int i = 0; i < 64; i++) kpad[i] = (i < 32 ? rpmb_key[i] : 0) ^ 0x36;
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, kpad, 64);
  mbedtls_sha256_update(&ctx, msg, len);
  mbedtls_sha256_finish(&ctx, inner);
  mbedtls_sha256_free(&ctx);
  for (int i = 0; i < 64; i++) kpad[i] = (i < 32 ? rpmb_key[i] : 0) ^ 0x5C;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, kpad, 64);
  mbedtls_sha256_update(&ctx, inner, 32);
  mbedtls_sha256_finish(&ctx, out);
  mbedtls_sha256_free(&ctx);
}

// Build an authenticated request frame (result field 0) + HMAC.
static void rpmb_frame(uint16_t type, uint16_t addr, uint16_t count,
                       uint32_t counter, const uint8_t *data, uint8_t *f) {
  memset(f, 0, 512);
  if (data) memcpy(f + 228, data, 256);
  f[500] = (uint8_t)(counter >> 24); f[501] = (uint8_t)(counter >> 16);
  f[502] = (uint8_t)(counter >> 8); f[503] = (uint8_t)counter;
  f[504] = (uint8_t)(addr >> 8); f[505] = (uint8_t)addr;
  f[506] = (uint8_t)(count >> 8); f[507] = (uint8_t)count;
  f[510] = (uint8_t)(type >> 8); f[511] = (uint8_t)type;
  uint8_t msg[284];
  memcpy(msg, f + 228, 256);
  memcpy(msg + 256, f + 484, 16);
  memcpy(msg + 272, f + 500, 12);
  uint8_t mac[32];
  rpmb_hmac(msg, sizeof(msg), mac);
  memcpy(f + 196, mac, 32);
}

// Write one 512 B frame (CMD25) / read one back (CMD18) via the PIO FIFO.
static void rpmb_write(const uint8_t *f) {
  *reg_bytcnt = 512;
  issue(25, 0, CMD_RESP_EXPECT | CMD_DATA_EXPECT | CMD_WRITE);
  for (int i = 0; i < 128; i++) {
    uint32_t w;
    memcpy(&w, f + 4 * i, 4);
    *reg_fifo = w;
  }
}

static void rpmb_read(uint8_t *f) {
  *reg_bytcnt = 512;
  issue(18, 0, CMD_RESP_EXPECT | CMD_DATA_EXPECT);
  for (int i = 0; i < 128; i++) {
    uint32_t w = *reg_fifo;
    memcpy(f + 4 * i, &w, 4);
  }
}

// Full request/response round: build, write, read the response frame.
static void rpmb_request(uint16_t type, uint16_t addr, uint16_t count,
                         uint32_t counter, const uint8_t *data, uint8_t *resp) {
  static uint8_t req[512];
  rpmb_frame(type, addr, count, counter, data, req);
  rpmb_write(req);
  rpmb_read(resp);
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 7);
  delay(200);
  uint32_t fail = 0;

  // GO_IDLE_STATE, then MMC SEND_OP_COND (busy first, ready second).
  issue(0, 0, 0);
  issue(1, 0x40FF8000, CMD_RESP_EXPECT);
  uint32_t ocr1 = *reg_resp0;
  if (ocr1 & (1u << 31)) {
    Serial.println("EMMC UNEXPECTED READY");
    fail |= 1u;
  }
  issue(1, 0x40FF8000, CMD_RESP_EXPECT);
  uint32_t ocr2 = *reg_resp0;
  if (!(ocr2 & (1u << 31)) || !(ocr2 & (1u << 30))) {
    Serial.println("EMMC NOT READY/HCS");
    fail |= 2u;
  }
  // ALL_SEND_CID + host-assigned RCA + SELECT.
  issue(2, 0, CMD_RESP_EXPECT | CMD_RESP_LONG);
  if (*reg_resp0 != 0x12345678u) {
    Serial.println("EMMC CID MISMATCH");
    fail |= 4u;
  }
  issue(3, 1u << 16, CMD_RESP_EXPECT);
  if ((*reg_resp0 >> 16) != 1u) {
    Serial.println("EMMC RCA MISMATCH");
    fail |= 8u;
  }
  issue(7, 1u << 16, CMD_RESP_EXPECT);
  if (((*reg_resp0 >> 9) & 0xFu) != 4u) {
    Serial.println("EMMC NOT TRAN");
    fail |= 16u;
  }
  // MMC CSD: structure v1.2, 25 MHz legacy timing, 512 B blocks.
  issue(9, 0, CMD_RESP_EXPECT | CMD_RESP_LONG);
  if ((*reg_resp3 >> 30) != 2u || (*reg_resp3 & 0xFFu) != 0x32u) {
    Serial.println("EMMC CSD MISMATCH");
    fail |= 32u;
  }
  // EXT_CSD: rev 1.8, SDR52 card type, 8192 sectors, 1-bit/legacy default.
  read_ext_csd(ext_buf);
  uint32_t sec;
  memcpy(&sec, ext_buf + 212, 4);
  Serial.print("EMMC EXTCSD rev=");
  Serial.print(ext_buf[192]);
  Serial.print(" type=");
  Serial.print(ext_buf[196], HEX);
  Serial.print(" sec=");
  Serial.println(sec);
  if (ext_buf[192] != 8 || ext_buf[196] != 0x07 || sec != 8192 ||
      ext_buf[183] != 0 || ext_buf[185] != 0) {
    Serial.println("EMMC EXTCSD MISMATCH");
    fail |= 64u;
  }
  // SWITCH bus width to 4-bit and timing to high-speed; the shadow reads back.
  issue(6, (3u << 26) | (183u << 16) | (1u << 8), CMD_RESP_EXPECT);
  issue(6, (3u << 26) | (185u << 16) | (1u << 8), CMD_RESP_EXPECT);
  read_ext_csd(ext_buf);
  if (ext_buf[183] != 1 || ext_buf[185] != 1) {
    Serial.println("EMMC SWITCH MISMATCH");
    fail |= 128u;
  }
  // Block round-trip at LBA 200.
  issue(16, 512, CMD_RESP_EXPECT);
  *reg_bytcnt = 512;
  issue(24, 200, CMD_RESP_EXPECT | CMD_DATA_EXPECT | CMD_WRITE);
  for (int i = 0; i < 128; i++) {
    *reg_fifo = 0xE11C0000u | (uint32_t)i;
  }
  *reg_bytcnt = 512;
  issue(17, 200, CMD_RESP_EXPECT | CMD_DATA_EXPECT);
  for (int i = 0; i < 128; i++) {
    if (*reg_fifo != (0xE11Cu << 16 | (uint32_t)i)) fail |= 256u;
  }

  // RPMB leg: SWITCH to the RPMB partition, provision the key,
  // counter/write/read round-trip with HMAC-SHA256 authentication.
  // Frame layout (JEDEC JESD84, 512 B): MAC[196..228), data[228..484),
  // nonce[484..500), counter[500..504) BE, addr[504..506) BE,
  // count[506..508) BE, result[508..510) BE, type[510..512) BE.
  // HMAC covers data+nonce+counter+addr+count+result+type (284 B).
  for (int i = 0; i < 32; i++) rpmb_key[i] = 0x42;
  static uint8_t frame[512];
  static uint8_t resp[512];
  // SWITCH PARTITION_CONFIG (index 179) access = RPMB (3).
  issue(6, (3u << 26) | (179u << 16) | (3u << 8), CMD_RESP_EXPECT);
  // Provision (type 1 carries the key in the MAC field).
  memset(frame, 0, sizeof(frame));
  memcpy(frame + 196, rpmb_key, 32);
  frame[510] = 0x00; frame[511] = 0x01;
  rpmb_write(frame);
  rpmb_read(resp);
  bool rpmb_ok = resp[508] == 0 && resp[509] == 0
    && resp[510] == 0x01 && resp[511] == 0x00;
  Serial.print("EMMC RPMB PROV result=");
  Serial.println((resp[508] << 8) | resp[509], HEX);
  // Counter (type 2) reads 0 with a valid MAC.
  rpmb_request(2, 0, 1, 0, NULL, resp);
  uint32_t ctr = ((uint32_t)resp[500] << 24) | ((uint32_t)resp[501] << 16)
    | ((uint32_t)resp[502] << 8) | resp[503];
  Serial.print("EMMC RPMB CTR=");
  Serial.println(ctr);
  rpmb_ok = rpmb_ok && resp[508] == 0 && resp[509] == 0 && ctr == 0;
  // Authenticated write of one block, then read it back.
  static uint8_t wdata[256];
  for (int i = 0; i < 256; i++) wdata[i] = (uint8_t)(0xC3 + i);
  rpmb_request(3, 0, 1, 0, wdata, resp);
  rpmb_ok = rpmb_ok && resp[508] == 0 && resp[509] == 0
    && resp[510] == 0x03 && resp[511] == 0x00;
  rpmb_request(4, 0, 1, 0, NULL, resp);
  rpmb_ok = rpmb_ok && resp[508] == 0 && resp[509] == 0;
  for (int i = 0; i < 256; i++) {
    if (resp[228 + i] != (uint8_t)(0xC3 + i)) rpmb_ok = false;
  }
  // Counter advanced exactly once; tampered MAC is rejected.
  rpmb_request(2, 0, 1, 0, NULL, resp);
  uint32_t ctr1 = ((uint32_t)resp[500] << 24) | ((uint32_t)resp[501] << 16)
    | ((uint32_t)resp[502] << 8) | resp[503];
  Serial.print("EMMC RPMB CTR1=");
  Serial.println(ctr1);
  rpmb_ok = rpmb_ok && ctr1 == 1;
  Serial.println(rpmb_ok ? "EMMC RPMB PASS" : "EMMC RPMB FAIL");
  if (!rpmb_ok) fail |= 512u;

  if (fail == 0) {
    Serial.println("EMMC PASS");
  } else {
    Serial.print("EMMC FAIL bits=");
    Serial.println(fail, HEX);
  }
  Serial.println("EMMC DONE");
}

void loop() {
  delay(1000);
}
