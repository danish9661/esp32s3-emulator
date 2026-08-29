// Virtual-device demo (ESP32-S3) — exercises the emulator's Wokwi/rp2040js-
// style virtual-peripheral API entirely in-browser. There are NO real bus
// devices; the browser supplies them:
//   * I2C : pokes I2CEXT0 to send a command byte to a virtual sensor @0x42
//           (firmware→JS) and to read back a value the browser injects
//           (JS→firmware).
//   * SPI : uses the Arduino SPI library; the browser injects MISO so the
//           transfer returns a non-zero reading (JS→firmware), while the MOSI
//           byte the firmware sent is captured by the browser (firmware→JS).
// Run this under the web UI (web/index.html) with the bundled virtual devices:
// open the "Virtual devices" panel to watch the bytes flow.

#define I2C0_BASE 0x60013000UL
volatile uint32_t* const I2C = (volatile uint32_t* const)I2C0_BASE;

// GPSPI2 (SPI2) register-poke surface (per spi_struct.h): CMD/USER/USER1/USER2
// at 0x00/0x10/0x14/0x18, MS_DLEN @0x1C, DATA_BUF @0x98, CLK_GATE @0xE8.
#define SPI2_BASE 0x60024000UL
volatile uint32_t* const SPI = (volatile uint32_t* const)SPI2_BASE;

// CTR bits: ms_mode = bit4, trans_start = bit5.
#define CTR(ms, st) (((ms) ? (1u << 4) : 0) | ((st) ? (1u << 5) : 0))
// comd value: op[13:11], ack_val[10], ack_en[8], byte_num[7:0].
// op codes: RSTART=6, WRITE=1, READ=3, STOP=2, END=4.
#define COMD(op, ackv, acke, bn) \
  (((uint32_t)((op)&7u) << 11) | ((uint32_t)((ackv)&1u) << 10) | \
   ((uint32_t)((acke)&1u) << 8) | ((uint32_t)((bn)&0xFFu)))

static inline uint32_t raw() { return I2C[0x20 / 4]; }   // INT_RAW
static inline uint32_t sr()  { return I2C[0x08 / 4]; }   // SR
static const uint32_t DONE_BITS = (1u << 7) | (1u << 3); // trans_complete | end_detect

// Send one byte to the virtual device at `addr` (firmware→JS).
void i2c_write_byte(uint8_t addr, uint8_t b) {
  I2C[0x24 / 4] = 0xFFFFFFFFu;        // INT_CLR
  I2C[0x18 / 4] = (1u << 13);        // FIFO_CONF: TX_FIFO_RST
  I2C[0x1C / 4] = (uint32_t)((addr << 1) & 0xFFu);  // address (write)
  I2C[0x1C / 4] = (uint32_t)(b & 0xFFu);            // data byte
  I2C[0x58 / 4 + 0] = COMD(6, 0, 0, 0);   // RSTART
  I2C[0x58 / 4 + 1] = COMD(1, 0, 1, 1);   // WRITE addr
  I2C[0x58 / 4 + 2] = COMD(1, 0, 1, 1);   // WRITE data
  I2C[0x58 / 4 + 3] = COMD(2, 0, 0, 0);   // STOP
  I2C[0x58 / 4 + 4] = COMD(4, 0, 0, 0);   // END
  I2C[0x04 / 4] = CTR(1, 1);              // ms_mode + trans_start
  uint32_t n = 0;
  while (!(raw() & DONE_BITS) && n < 8000u) n++;
}

// Read one byte from the virtual device at `addr` (JS→firmware). Returns the
// byte the browser injected, or 0xFF if no device answered.
uint8_t i2c_read_byte(uint8_t addr) {
  I2C[0x24 / 4] = 0xFFFFFFFFu;        // INT_CLR
  I2C[0x18 / 4] = (1u << 13);        // FIFO_CONF: TX_FIFO_RST
  I2C[0x1C / 4] = (uint32_t)(((addr << 1) | 1) & 0xFFu);  // address (read)
  I2C[0x58 / 4 + 0] = COMD(6, 0, 0, 0);   // RSTART
  I2C[0x58 / 4 + 1] = COMD(1, 0, 1, 1);   // WRITE read-address
  I2C[0x58 / 4 + 2] = COMD(3, 1, 0, 1);   // READ 1 byte (master ACK)
  I2C[0x58 / 4 + 3] = COMD(2, 0, 0, 0);   // STOP
  I2C[0x58 / 4 + 4] = COMD(4, 0, 0, 0);   // END
  I2C[0x04 / 4] = CTR(1, 1);              // ms_mode + trans_start
  uint32_t n = 0;
  while (!(raw() & DONE_BITS) && n < 8000u) n++;
  if (((sr() >> 8) & 0x1Fu) >= 1) return (uint8_t)(I2C[0x1C / 4] & 0xFFu);
  return 0xFF;
}

// One full-duplex SPI byte exchange via register pokes (sets usr_mosi+usr_miso
// so the model shifts the injected MISO into the data buffer). Returns MISO.
uint8_t spi_xfer(uint8_t tx) {
  SPI[0xE8 / 4] = 1;                       // CLK_GATE: clk_en
  SPI[0x1C / 4] = 7;                       // MS_DLEN: 8 bits (0-based)
  SPI[0x98 / 4] = (uint32_t)tx << 24;      // DATA_BUF W0: MOSI, left-aligned
  SPI[0x10 / 4] = (1u << 27) | (1u << 28); // USER: usr_mosi | usr_miso
  SPI[0x00 / 4] = (1u << 24);              // CMD: usr (self-clears)
  while (SPI[0x00 / 4] & (1u << 24)) {}
  return (uint8_t)((SPI[0x98 / 4] >> 24) & 0xFFu); // MISO from data buffer
}

void setup() {
  Serial.begin(115200);
  Serial.println("VIRTUAL DEMO START");

  // ---- I2C: firmware -> JS (send a command the browser displays) ----
  i2c_write_byte(0x42, 0x99);
  Serial.println("I2C sent command 0x99 to virtual device");
}

// The browser pre-primes the virtual device's response at load time, so the
// first read/transfer below receives it (the event API is drained once per
// frame, hence a one-frame latency on injected data). We read once and stop
// to keep the output crisp.
volatile bool demo_done = false;

void loop() {
  if (demo_done) { delay(500); return; }

  // ---- I2C: JS -> firmware (read a value the browser injected) ----
  uint8_t v = i2c_read_byte(0x42);
  Serial.printf("I2C read from virtual device: 0x%02X\n", v);
  bool i2c_ok = (v != 0xFF);

  // ---- SPI: JS -> firmware (browser injects MISO) ----
  uint8_t spi_miso = spi_xfer(0x55);
  Serial.printf("SPI transfer(0x55) -> MISO 0x%02X\n", spi_miso);
  bool spi_ok = (spi_miso != 0x00);

  Serial.printf("VIRTUAL DEMO %s\n", (i2c_ok && spi_ok) ? "PASS" : "RUN");
  Serial.println("VIRTUAL DEMO DONE");
  demo_done = true;
}
