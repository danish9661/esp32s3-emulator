// Fake temperature devices on real buses (no TSENS): an I2C thermometer
// and an SPI thermometer, both served by host injection (the same hooks the
// virtual-demo browser harness uses: i2c_inject_rx / spi_inject_miso).
//
// * I2C: TMP102-style — write pointer 0x00, then READ 2 bytes; the host
//   pre-injects the raw word for 25.00 C. TMP102 format is 12-bit
//   left-justified (LSB = 0.0625 C): 25 C = raw 0x190 = bytes 0x19,0x00.
// * SPI: MAX6675-style 16-bit read-only frame: bit 15..3 = temp/0.25 C
//   (25 C = 100 = 0x64 << 3 = 0x0320), bit 2 = thermocouple-input flag.
//   Two full-duplex transfers of 0x00 clock the frame out.
//
// Markers follow the battery convention. Provision via run_flash env:
// I2C_TEMP_RX=1900 (hex bytes) pre-injects the I2C read, SPI_TEMP_RX=0320
// pre-injects the next SPI transfer.

#define I2C0_BASE 0x60013000UL
volatile uint32_t* const I2C = (volatile uint32_t* const)I2C0_BASE;
#define SPI2_BASE 0x60024000UL
volatile uint32_t* const SPI = (volatile uint32_t* const)SPI2_BASE;

#define CTR(ms, st) (((ms) ? (1u << 4) : 0) | ((st) ? (1u << 5) : 0))
#define COMD(op, ackv, acke, bn) \
  (((uint32_t)((op)&7u) << 11) | ((uint32_t)((ackv)&1u) << 10) | \
   ((uint32_t)((acke)&1u) << 8) | ((uint32_t)((bn)&0xFFu)))

static inline uint32_t iraw() { return I2C[0x20 / 4]; }
static inline uint32_t isr() { return I2C[0x08 / 4]; }
static const uint32_t IDONE = (1u << 7) | (1u << 3);

// TMP102 @0x48: set pointer reg then READ 2 bytes. Returns raw word.
static uint16_t tmp102_read(uint8_t addr) {
  I2C[0x24 / 4] = 0xFFFFFFFFu;
  I2C[0x18 / 4] = (1u << 13);
  I2C[0x1C / 4] = (uint32_t)((addr << 1) & 0xFFu);  // write addr
  I2C[0x1C / 4] = 0x00;                              // pointer = temp reg
  I2C[0x58 / 4 + 0] = COMD(6, 0, 0, 0);
  I2C[0x58 / 4 + 1] = COMD(1, 0, 1, 1);
  I2C[0x58 / 4 + 2] = COMD(1, 0, 1, 1);
  I2C[0x58 / 4 + 3] = COMD(2, 0, 0, 0);
  I2C[0x58 / 4 + 4] = COMD(4, 0, 0, 0);
  I2C[0x04 / 4] = CTR(1, 1);
  uint32_t n = 0;
  while (!(iraw() & IDONE) && n < 8000u) n++;
  // Repeated-start read of 2 bytes.
  I2C[0x24 / 4] = 0xFFFFFFFFu;
  I2C[0x1C / 4] = (uint32_t)(((addr << 1) | 1) & 0xFFu);
  I2C[0x58 / 4 + 0] = COMD(6, 0, 0, 0);
  I2C[0x58 / 4 + 1] = COMD(1, 0, 1, 1);
  I2C[0x58 / 4 + 2] = COMD(3, 0, 0, 2);  // READ 2
  I2C[0x58 / 4 + 3] = COMD(2, 0, 0, 0);
  I2C[0x58 / 4 + 4] = COMD(4, 0, 0, 0);
  I2C[0x04 / 4] = CTR(1, 1);
  n = 0;
  while (!(iraw() & IDONE) && n < 8000u) n++;
  uint16_t hi = 0xFF, lo = 0xFF;
  if (((isr() >> 8) & 0x1Fu) >= 1) hi = (uint16_t)(I2C[0x1C / 4] & 0xFFu);
  if (((isr() >> 8) & 0x1Fu) >= 1) lo = (uint16_t)(I2C[0x1C / 4] & 0xFFu);
  return (uint16_t)((hi << 8) | lo);
}

// MAX6675-style SPI read: two full-duplex bytes clock out the 16-bit frame.
// NOTE: the transfer must set usr_miso (without usr_mosi there is no MISO
// window — proven by the virtual_demo register-poke path, which sets both).
static uint16_t max6675_read() {
  SPI[0xE8 / 4] = 1;
  SPI[0x1C / 4] = 15;  // 16 bits
  SPI[0x10 / 4] = (1u << 27) | (1u << 28);  // usr_mosi | usr_miso
  SPI[0x98 / 4] = 0;
  SPI[0x00 / 4] = (1u << 24);
  while (SPI[0x00 / 4] & (1u << 24)) {}
  return (uint16_t)((SPI[0x98 / 4] >> 16) & 0xFFFFu);  // MISO word, MSB word
}

void setup() {
  Serial.begin(115200);
  delay(200);
  *(volatile uint32_t*)(0x600C0018) |= (1u << 7) | (1u << 6); // I2C0 + SPI2
  Serial.println("TEMPDEV START");

  uint16_t i2c_raw = tmp102_read(0x48);
  float i2c_c = (float)(int16_t)i2c_raw / 256.0f;  // TMP102: raw/256 = C
  Serial.printf("TEMPDEV I2C raw=%04X c=%.2f\n", i2c_raw, i2c_c);

  uint16_t spi_raw = max6675_read();
  float spi_c = (float)(spi_raw >> 3) * 0.25f;  // MAX6675: bits[15:3]*0.25
  Serial.printf("TEMPDEV SPI raw=%04X c=%.2f\n", spi_raw, spi_c);

  bool ok = (i2c_raw == 0x1900) && (spi_raw == 0x0320);
  Serial.println(ok ? "TEMPDEV PASS" : "TEMPDEV FAIL");
  Serial.println("TEMPDEV DONE");
}

void loop() { delay(1000); }
