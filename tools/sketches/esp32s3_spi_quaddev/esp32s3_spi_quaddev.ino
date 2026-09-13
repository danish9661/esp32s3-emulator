// SPI fake quad-device validation for the ESP32-S3 emulator.
//
// Programs SPI_CTRL FREAD_QUAD (spi_reg.h bit 15) and drives wide-mode USR
// transfers against the host fake quad-device store (provisioned by the
// harness): a MISO read lands at the address-phase window of the pattern,
// a MOSI write commits into the store, and a second read proves the
// round-trip. Single-line mode (CTRL=0) reads zeros (no device).
//
// The harness provisions the store before boot; without provisioning the
// QUAD leg reads zeros. Markers follow the battery convention.

#define SPI2 0x60024000u
#define S(x) ((volatile uint32_t*)(SPI2 + (x)))

#define SPI_USER 0x10
#define SPI_USER1 0x14
#define SPI_ADDR 0x04
#define SPI_MS_DLEN 0x1C
#define SPI_CTRL 0x08
#define SPI_CLOCK 0x0C
#define SPI_CLK_GATE 0xE8
#define SPI_CMD 0x00
#define SPI_DATA_BUF 0x98
#define SPI_INT_RAW 0x3C
#define SPI_INT_CLR 0x38

static void xfer(uint32_t user, uint32_t ctrl) {
  *S(SPI_CTRL) = ctrl;
  *S(SPI_USER) = user;
  *S(SPI_CMD) = (1u << 24);
  for (volatile int i = 0; i < 20000; i++) {
    if ((*S(SPI_CMD) & (1u << 24)) == 0) break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  *(volatile uint32_t*)(0x600C0018) |= (1u << 6); // SPI2 clock
  *S(SPI_CLOCK) = 0x1000; // 2 cyc/bit
  *S(SPI_USER1) = (23u << 27); // 24-bit address phase
  *S(SPI_ADDR) = 0x10;
  *S(SPI_MS_DLEN) = 31; // 32 data bits
  *S(SPI_CLK_GATE) = 1;

  // Single-line baseline: MISO reads zeros (no device on single wires).
  xfer((1u << 30) | (1u << 28), 0);
  uint32_t single = *S(SPI_DATA_BUF);
  Serial.printf("SPI QUAD single=%08lX\n", (unsigned long)single);

  // Quad read at addr 0x10: harness pattern window.
  xfer((1u << 30) | (1u << 28), (1u << 15));
  uint32_t q = *S(SPI_DATA_BUF);
  Serial.printf("SPI QUAD read=%08lX\n", (unsigned long)q);

  // Quad write 0xDEADBEEF at addr 0x10, then read back.
  *S(SPI_DATA_BUF) = 0xDEADBEEF;
  xfer((1u << 30) | (1u << 27), (1u << 15));
  *S(SPI_ADDR) = 0x10;
  xfer((1u << 30) | (1u << 28), (1u << 15));
  uint32_t rb = *S(SPI_DATA_BUF);
  Serial.printf("SPI QUAD roundtrip=%08lX\n", (unsigned long)rb);

  bool ok = (single == 0) && (q == 0x10111213UL) && (rb == 0xDEADBEEFUL);
  Serial.println(ok ? "SPI QUADDEV PASS" : "SPI QUADDEV FAIL");
  Serial.println("SPI QUADDEV DONE");
}

void loop() { delay(1000); }
