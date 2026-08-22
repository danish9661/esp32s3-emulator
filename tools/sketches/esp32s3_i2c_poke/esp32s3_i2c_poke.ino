// I2C master validation sketch (ESP32-S3) — direct register pokes, NO Wire
// driver. Drives I2CEXT0 through two master transactions (WRITE and
// READ) and checks the interrupt + status the emulator models:
//   - RSTART/WRITE/STOP/END (and .../READ/...) run the FSM to completion,
//   - TRANS_COMPLETE + END_DETECT latch in INT_RAW,
//   - with no device on the bus a NACK is latched (INT_NACK + SR.resp_rec),
//   - the bus returns idle (SR.bus_busy == 0) and a READ fills the RX FIFO.
// Compiled with arduino-cli; runs identically on real silicon (NACK expected
// because there is no slave on the bus). Validates the emulator's I2C model
// independently of the esp-idf Wire driver stack.

#define I2C0_BASE 0x60013000UL
volatile uint32_t* const I2C = (volatile uint32_t* const)I2C0_BASE;

// CTR bits: ms_mode = bit4, trans_start = bit5.
#define CTR(ms, st) (((ms) ? (1u << 4) : 0) | ((st) ? (1u << 5) : 0))
// comd value: op[13:11], ack_val[10], ack_en[8], byte_num[7:0].
// op codes: RSTART=6, WRITE=1, READ=3, STOP=2, END=4.
#define COMD(op, ackv, acke, bn) \
  (((uint32_t)((op)&7u) << 11) | ((uint32_t)((ackv)&1u) << 10) | \
   ((uint32_t)((acke)&1u) << 8) | ((uint32_t)((bn)&0xFFu)))

static inline uint32_t raw() { return I2C[0x20 / 4]; }   // INT_RAW
static inline uint32_t sr()  { return I2C[0x08 / 4]; }    // SR

static const uint32_t DONE_BITS = (1u << 7) | (1u << 3);  // trans_complete | end_detect

// One master WRITE transaction of a single address byte. Expects NACK (no
// device) and END_DETECT + TRANS_COMPLETE.
bool run_write(uint8_t addr, uint32_t* out_raw, uint32_t* out_sr) {
  I2C[0x24 / 4] = 0xFFFFFFFFu;        // INT_CLR: clear latched interrupts
  I2C[0x18 / 4] = (1u << 13);         // FIFO_CONF: TX_FIFO_RST
  I2C[0x1C / 4] = (uint32_t)((addr << 1) & 0xFFu);  // DATA: write-address byte
  I2C[0x58 / 4 + 0] = COMD(6, 0, 0, 0);   // RSTART
  I2C[0x58 / 4 + 1] = COMD(1, 0, 1, 1);   // WRITE 1 byte (ack_en)
  I2C[0x58 / 4 + 2] = COMD(2, 0, 0, 0);   // STOP
  I2C[0x58 / 4 + 3] = COMD(4, 0, 0, 0);   // END
  I2C[0x04 / 4] = CTR(1, 1);          // ms_mode + trans_start
  uint32_t n = 0;
  while (!(raw() & DONE_BITS) && n < 8000u) { n++; }
  *out_raw = raw();
  *out_sr = sr();
  return ((*out_raw & DONE_BITS) != 0) && ((*out_sr & (1u << 4)) == 0);
}

// One master READ transaction: WRITE read-address, READ 1 byte, STOP, END.
// No device -> the byte read back is 0xFF and the RX FIFO holds 1 entry.
bool run_read(uint8_t addr, uint32_t* out_raw, uint32_t* out_sr) {
  I2C[0x24 / 4] = 0xFFFFFFFFu;        // INT_CLR
  I2C[0x18 / 4] = (1u << 13);         // FIFO_CONF: TX_FIFO_RST
  I2C[0x1C / 4] = (uint32_t)(((addr << 1) | 1) & 0xFFu);  // read-address byte
  I2C[0x58 / 4 + 0] = COMD(6, 0, 0, 0);   // RSTART
  I2C[0x58 / 4 + 1] = COMD(1, 0, 1, 1);   // WRITE read-address (ack_en)
  I2C[0x58 / 4 + 2] = COMD(3, 1, 0, 1);   // READ 1 byte (master ACK=1)
  I2C[0x58 / 4 + 3] = COMD(2, 0, 0, 0);   // STOP
  I2C[0x58 / 4 + 4] = COMD(4, 0, 0, 0);   // END
  I2C[0x04 / 4] = CTR(1, 1);          // ms_mode + trans_start
  uint32_t n = 0;
  while (!(raw() & DONE_BITS) && n < 8000u) { n++; }
  *out_raw = raw();
  *out_sr = sr();
  uint32_t rxcnt = (*out_sr >> 8) & 0x1Fu;
  return ((*out_raw & DONE_BITS) != 0) && (rxcnt == 1);
}

void setup() {
  Serial.begin(115200);
  delay(50);
  I2C[0x04 / 4] = (1u << 4);   // CTR: master mode (no trans_start yet)

  uint32_t r1, s1, r2, s2;
  bool w = run_write(0x42, &r1, &s1);
  bool rd = run_read(0x42, &r2, &s2);
  uint32_t rxcnt2 = (s2 >> 8) & 0x1Fu;

  Serial.printf("I2C POKE write raw=0x%08X sr=0x%08X nack=%d ok=%d\n",
                 r1, s1, (int)((r1 >> 10) & 1u), w);
  Serial.printf("I2C POKE read  raw=0x%08X sr=0x%08X rxcnt=%u ok=%d\n",
                 r2, s2, rxcnt2, rd);

  bool nack_ok = ((r1 >> 10) & 1u) != 0;     // NACK latched (no device)
  bool busy_ok = ((s1 >> 4) & 1u) == 0;      // bus idle after write
  bool pass = w && rd && nack_ok && busy_ok;
  Serial.println(pass ? "I2C POKE PASS" : "I2C POKE FAIL");
}

void loop() {}
