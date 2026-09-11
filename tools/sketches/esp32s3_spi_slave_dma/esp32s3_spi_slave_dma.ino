// ESP32-S3 GPSPI2 slave-DMA validation sketch (direct register pokes).
//
// Firmware puts GPSPI2 (0x60024000) in slave mode with DMA_CONF dma_rx_ena
// + dma_tx_ena, programs GDMA ch0 OUT (slave TX pattern) + IN (slave RX
// buffer) links wired to SPI2, and starts both. The HOST (run_flash with
// SPI_SLAVE_XCHG=1) drives both directions on the DMA markers: a 4-byte
// master-write on "SPI SLAVE DMA READY" (lands in the IN-link DRAM buffer
// with SLV_WR_DMA_DONE), then a 4-byte master-read on
// "SPI SLAVE DMA TX-REQ" (sources the OUT-link DRAM buffer with
// SLV_RD_DMA_DONE, which the host asserts byte-exact).
#include <Arduino.h>

#define SPI2_BASE 0x60024000u
#define GDMA_BASE 0x6003F000u

#define S(x) ((volatile uint32_t*)(SPI2_BASE + (x)))
#define D(x) ((volatile uint32_t*)(GDMA_BASE + (x)))

struct Desc {
  uint32_t dw0, buf, next, rsvd;
};

static Desc g_txdesc;
static uint8_t g_tx[4] = {0x12, 0x34, 0x56, 0x78};
static Desc g_rxdesc;
static uint8_t g_rx[4];

static int poll_raw(uint32_t bit) {
  for (uint32_t t = 0; t < 20000000; t++) {
    if (*S(0x3C) & bit) return 1;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) g_rx[i] = 0;

  *S(0xE0) = (1u << 26);              // slave_mode
  *S(0x30) = (1u << 25) | (1u << 26);  // dma_conf: rx_ena + tx_ena
  // GDMA ch0 OUT -> SPI2: slave TX pattern.
  g_txdesc.dw0 = (4) | (4 << 12) | (1u << 30) | (1u << 31);
  g_txdesc.buf = (uint32_t)g_tx;
  g_txdesc.next = 0;
  g_txdesc.rsvd = 0;
  *D(0x60 + 0x48) = 0;  // out_peri_sel[0] = SPI2
  uint32_t olsb = ((uint32_t)&g_txdesc) & 0x000FFFFFu;
  *D(0x60 + 0x20) = olsb | (1u << 21);  // OUT start
  // GDMA ch0 IN <- SPI2: slave RX buffer.
  g_rxdesc.dw0 = (4) | (4 << 12) | (1u << 30) | (1u << 31);
  g_rxdesc.buf = (uint32_t)g_rx;
  g_rxdesc.next = 0;
  g_rxdesc.rsvd = 0;
  *D(0x48) = 0;  // in_peri_sel[0] = SPI2
  uint32_t ilsb = ((uint32_t)&g_rxdesc) & 0x000FFFFFu;
  *D(0x20) = ilsb | (1u << 22);  // IN start

  *S(0x38) = (1u << 8) | (1u << 9);  // clear stale DMA-done bits
  Serial.println("SPI SLAVE DMA READY");
  if (!poll_raw(1u << 9)) {
    Serial.println("SPI SLAVE DMA TIMEOUT");
    return;
  }
  Serial.printf("SPI SLAVE DMA RX=%02x %02x %02x %02x\n", g_rx[0], g_rx[1], g_rx[2], g_rx[3]);
  bool rx_ok = g_rx[0] == 0xDE && g_rx[1] == 0xAD && g_rx[2] == 0xBE && g_rx[3] == 0xEF;
  Serial.println(rx_ok ? "SPI SLAVE DMA RX OK" : "SPI SLAVE DMA RX MISMATCH");

  *S(0x38) = (1u << 8) | (1u << 9);
  Serial.println("SPI SLAVE DMA TX-REQ");
  if (!poll_raw(1u << 8)) {
    Serial.println("SPI SLAVE DMA TIMEOUT");
    return;
  }
  Serial.printf("SPI SLAVE DMA TXLEN=%u\n", *S(0xE4) & 0x3FFFF);
  // The OUT descriptor must be handed back (owner cleared by the exchange).
  bool eof_ok = (g_txdesc.dw0 & (1u << 31)) == 0;
  Serial.println(eof_ok ? "SPI SLAVE DMA DONE" : "SPI SLAVE DMA STUCK");
  Serial.println(rx_ok && eof_ok ? "SPI SLAVE DMA PASS" : "SPI SLAVE DMA FAIL");
}

void loop() {}
