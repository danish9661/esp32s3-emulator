// ESP32-S3 GPSPI2 master-DMA validation sketch (direct register pokes).
//
// Programs a GDMA OUT descriptor (peri_sel = 0 = SPI2) carrying 4 bytes,
// runs a DMA-backed SPI2 transfer at a slow clock, and samples the live
// MOSI waveform back through the GPIO matrix (FSPICLK->GPIO1, FSPID->GPIO2,
// FSPICS0->GPIO3, read via GPIO_IN) to prove the GDMA-fed bytes actually
// shift out on the pins. Then runs the GDMA IN link and checks the RX
// capture (zeros, no device) plus both done bits.
#include <Arduino.h>

#define GPIO 0x60004000u
#define SPI2 0x60024000u
#define GDMA 0x60042000u
#define G(x) ((volatile uint32_t*)(GPIO + (x)))
#define S(x) ((volatile uint32_t*)(SPI2 + (x)))
#define D(x) ((volatile uint32_t*)(GDMA + (x)))

// GPSPI2 (FSPI) matrix signals (gpio_sig_map.h): CLK 101, D(MOSI) 103.
#define SIG_FSPICLK 101
#define SIG_FSPID 103
#define SIG_FSPICS0 110

#define PIN_CLK 1
#define PIN_MOSI 2
#define PIN_CS 3

typedef struct {
  uint32_t dw0;
  uint32_t buf;
  uint32_t next;
  uint32_t rsvd;
} gdma_desc_t;

static volatile uint8_t g_tx[4] = {0xA5, 0x3C, 0xF0, 0x0F};
static volatile gdma_desc_t g_txdesc;
static volatile uint8_t g_rx[4];
static volatile gdma_desc_t g_rxdesc;
// Logic-analyzer capture: GPIO_IN snapshots (bit1=CLK, bit2=MOSI, bit3=CS).
// Back-to-back reads (~6 ticks/sample vs a 512-tick clock half-phase), so
// no edge can be missed; decode happens offline below.
static volatile uint8_t g_cap[9000];

static int wait_level(int pin, int level, uint32_t budget) {
  for (uint32_t t = 0; t < budget; t++) {
    if ((int)((*G(0x3C) >> pin) & 1u) == level) {
      return 1;
    }
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // Route SPI2 CLK/MOSI/CS0 to GPIO pins (peripheral output driver).
  *G(0x554 + PIN_CLK * 4) = SIG_FSPICLK;
  *G(0x554 + PIN_MOSI * 4) = SIG_FSPID;
  *G(0x554 + PIN_CS * 4) = SIG_FSPICS0;
  *G(0x24) = (1u << PIN_CLK) | (1u << PIN_MOSI) | (1u << PIN_CS);

  // SPI2: slow clock (pre=15, n=63 -> 1024 APB cycles/bit), 32-bit
  // full-duplex data phase, clock gate on.
  *S(0x0C) = (15u << 18) | (63u << 12) | (31u << 6);
  *S(0x1C) = 31;
  *S(0x10) = (1u << 27) | (1u << 28) | 1u;
  *S(0xE8) = 1;

  // GDMA OUT ch0 -> SPI2, one 4-byte descriptor, start.
  g_txdesc.dw0 = (4) | ((4) << 12) | (1u << 30) | (1u << 31);
  g_txdesc.buf = (uint32_t)g_tx;
  g_txdesc.next = 0;
  g_txdesc.rsvd = 0;
  *D(0x60 + 0x48) = 0;  // out_peri_sel[0] = SPI2
  uint32_t lsb = ((uint32_t)&g_txdesc) & 0x000FFFFFu;
  *D(0x60 + 0x20) = lsb;
  // Start the transfer and capture back-to-back: even a short printf here
  // would eat the first bits (the transfer starts inside this write).
  *D(0x60 + 0x20) = lsb | (1u << 21);

  // Capture the framed waveform like a logic analyzer, then decode offline:
  // every rising CLK edge inside the CS window contributes one MOSI bit.
  if (!wait_level(PIN_CS, 0, 2000000)) {
    Serial.println("SPI DMA NO CS");
    return;
  }
  int nsamp = 0;
  for (int n = 0; n < 9000; n++) {
    uint32_t inw = *G(0x3C);
    g_cap[n] = (uint8_t)inw;
    nsamp++;
    if (((inw >> PIN_CS) & 1u) && n > 1000) {
      break;  // CS released: transfer over
    }
  }
  uint32_t got = 0;
  int nbits = 0;
  int prev_clk = 0;
  for (int n = 0; n < nsamp; n++) {
    int clk = (g_cap[n] >> PIN_CLK) & 1;
    if (clk && !prev_clk) {
      got = (got << 1) | ((g_cap[n] >> PIN_MOSI) & 1u);
      nbits++;
    }
    prev_clk = clk;
  }
  Serial.printf("SPI DMA BITS=%d MOSI=%08x\n", nbits, got);
  Serial.println(
      (nbits == 32 && got == 0xA53CF00F) ? "SPI DMA MOSI OK" : "SPI DMA MOSI FAIL");

  // trans_done (bit 12 of the S3 DMA_INT block) must have latched by
  // now (CS released at transfer end).
  uint32_t done = 0;
  for (uint32_t t = 0; t < 2000000; t++) {
    if (*S(0x3C) & (1u << 12)) {
      done = 1;
      break;
    }
  }
  Serial.println(done ? "SPI DMA TXDONE OK" : "SPI DMA TXDONE FAIL");

  // GDMA IN ch0 <- SPI2: capture the RX bytes (zeros, no device on MISO).
  g_rxdesc.dw0 = (4) | ((4) << 12) | (1u << 30) | (1u << 31);
  g_rxdesc.buf = (uint32_t)g_rx;
  g_rxdesc.next = 0;
  g_rxdesc.rsvd = 0;
  *D(0x48) = 0;  // in_peri_sel[0] = SPI2
  uint32_t rlsb = ((uint32_t)&g_rxdesc) & 0x000FFFFFu;
  *D(0x20) = rlsb | (1u << 22);  // in start = bit 22
  uint32_t indone = *D(0x08) & 1u;
  uint32_t rxw = *((volatile uint32_t*)g_rx);
  Serial.printf("SPI DMA RX=%08x\n", rxw);
  Serial.println((indone && rxw == 0) ? "SPI DMA RX OK" : "SPI DMA RX FAIL");
  Serial.println("SPI DMA DONE");
}

void loop() {}
