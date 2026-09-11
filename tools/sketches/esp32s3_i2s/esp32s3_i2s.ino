// I2S0 (audio) functional poke test.
// Builds with arduino-cli (esp32:esp32:esp32s3) and runs under run_flash.
// Validates the FIFO + serial shift-out + interrupt + RX (loopback) paths,
// plus the newly-modeled TDM, PDM, clock-divisor, and GDMA (esp-idf DMA) paths
// of the emulator's i2s.rs model.
//
// Part 1: TX done interrupt.
// Part 2: sig_loopback RX (no external codec).
// Part 3: TDM multi-slot frame (2 slots, words looped back in order).
// Part 4: PDM encode (loopback reconstructs the word exactly).
// Part 5: TX clock-divisor register round-trip.
// Part 6: GDMA OUT channel (peri_sel == I2S0) feeds the TX FIFO; loopback RX
//         reads the descriptor's words back.

#define I2S0_BASE 0x6000F000
#define I2S_FIFO     (I2S0_BASE + 0x80)
#define I2S_TX_CONF  (I2S0_BASE + 0x24)
#define I2S_RX_CONF  (I2S0_BASE + 0x20)
#define I2S_INT_ENA  (I2S0_BASE + 0x14)
#define I2S_INT_RAW  (I2S0_BASE + 0x0C)
#define I2S_INT_CLR  (I2S0_BASE + 0x18)
#define I2S_TX_TDM_CTRL    (I2S0_BASE + 0x54)
#define I2S_TX_CLKM_DIV   (I2S0_BASE + 0x3C)
#define I2S_TX_PCM2PDM    (I2S0_BASE + 0x40)
#define TX_START_BIT (1u << 2)
#define RX_START_BIT (1u << 2)
#define SIG_LOOPBACK (1u << 27)
#define TX_TDM_EN    (1u << 19)
#define TX_PDM_EN    (1u << 20)
#define TX_DONE      (1u << 1)
#define RX_DONE      (1u << 0)

#define GDMA_BASE 0x6003F000
#define GDMA_OUT_PERI_SEL0 (GDMA_BASE + 0xA8)
#define GDMA_OUT_LINK0     (GDMA_BASE + 0x80)

#define GPIO_BASE         0x60004000
#define GPIO_FUNC_OUT_SEL0 (GPIO_BASE + 0x554)
#define GPIO_ENABLE_W1TS  (GPIO_BASE + 0x24)
#define SD_PIN 2

#define DESC_ADDR 0x3FC81000
#define BUF_ADDR  0x3FC82000

bool wait_tx_done() {
  for (int i = 0; i < 8000; i++) {
    if (REG_READ(I2S_INT_RAW) & TX_DONE) return true;
  }
  return false;
}
bool wait_rx_done() {
  for (int i = 0; i < 8000; i++) {
    if (REG_READ(I2S_INT_RAW) & RX_DONE) return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 4);
  *(volatile uint32_t*)(0x600C001C) |= (1u << 6);
  delay(80);

  REG_WRITE(GPIO_FUNC_OUT_SEL0 + 4 * SD_PIN, 25); // I2S0O_SD_OUT_IDX
  REG_WRITE(GPIO_ENABLE_W1TS, (1u << SD_PIN));

  REG_WRITE(I2S_INT_ENA, TX_DONE | RX_DONE);
  bool ok = true;

  // ---- Part 1: TX done ----
  REG_WRITE(I2S_INT_CLR, 0xF);
  REG_WRITE(I2S_FIFO, 0xABCD);
  REG_WRITE(I2S_TX_CONF, TX_START_BIT);
  bool tx_done = wait_tx_done();
  REG_WRITE(I2S_INT_CLR, TX_DONE);
  Serial.print("tx_done=");
  Serial.println(tx_done);
  ok &= tx_done;

  // ---- Part 2: loopback RX ----
  REG_WRITE(I2S_INT_CLR, 0xF);
  REG_WRITE(I2S_FIFO, 0xCAFE);
  REG_WRITE(I2S_TX_CONF, SIG_LOOPBACK | TX_START_BIT);
  bool rx_done = wait_rx_done();
  uint32_t rx_word = REG_READ(I2S_FIFO);
  REG_WRITE(I2S_INT_CLR, RX_DONE);
  Serial.print("loopback rx_word=");
  Serial.println(rx_word, HEX);
  ok &= rx_done && (rx_word == 0xCAFE);

  // ---- Part 3: TDM 2-slot loopback ----
  REG_WRITE(I2S_INT_CLR, 0xF);
  REG_WRITE(I2S_TX_TDM_CTRL, (1u << 16)); // tot_chan_num=1 -> 2 slots
  REG_WRITE(I2S_FIFO, 0x11111111);
  REG_WRITE(I2S_FIFO, 0x22222222);
  REG_WRITE(I2S_TX_CONF, TX_TDM_EN | SIG_LOOPBACK | TX_START_BIT);
  bool tdm_done = wait_rx_done();
  uint32_t tdm0 = REG_READ(I2S_FIFO);
  uint32_t tdm1 = REG_READ(I2S_FIFO);
  REG_WRITE(I2S_INT_CLR, RX_DONE);
  Serial.print("tdm0=");
  Serial.print(tdm0, HEX);
  Serial.print(" tdm1=");
  Serial.println(tdm1, HEX);
  ok &= tdm_done && (tdm0 == 0x11111111) && (tdm1 == 0x22222222);

  // ---- Part 4: PDM encode loopback ----
  REG_WRITE(I2S_INT_CLR, 0xF);
  REG_WRITE(I2S_TX_PCM2PDM, 0x13579246);
  REG_WRITE(I2S_FIFO, 0xCAFECAFE);
  REG_WRITE(I2S_TX_CONF, TX_PDM_EN | SIG_LOOPBACK | TX_START_BIT);
  bool pdm_done = wait_rx_done();
  uint32_t pdm_rx = REG_READ(I2S_FIFO);
  REG_WRITE(I2S_INT_CLR, RX_DONE);
  Serial.print("pdm_rx=");
  Serial.println(pdm_rx, HEX);
  ok &= pdm_done && (pdm_rx == 0xCAFECAFE);

  // ---- Part 5: clock-divisor round-trip ----
  REG_WRITE(I2S_TX_CLKM_DIV, 4);
  uint32_t div_rb = REG_READ(I2S_TX_CLKM_DIV);
  Serial.print("clk_div=");
  Serial.println(div_rb, HEX);
  ok &= (div_rb == 4);
  REG_WRITE(I2S_TX_CLKM_DIV, 0);

  // ---- Part 6: GDMA OUT -> I2S0 TX, loopback RX ----
  REG_WRITE(I2S_INT_CLR, 0xF);
  // Descriptor: owner=1, eof=1, length=8 bytes (2 words).
  *(volatile uint32_t*)DESC_ADDR = (1u << 31) | (1u << 30) | (8u << 12);
  *(volatile uint32_t*)(DESC_ADDR + 4) = BUF_ADDR;
  *(volatile uint32_t*)(DESC_ADDR + 8) = 0;
  *(volatile uint32_t*)BUF_ADDR = 0x12345678;
  *(volatile uint32_t*)(BUF_ADDR + 4) = 0x9ABCDEF0;
  REG_WRITE(GDMA_OUT_PERI_SEL0, 3); // I2S0
  REG_WRITE(GDMA_OUT_LINK0, (DESC_ADDR & 0x000FFFFF) | (1u << 21));
  // GDMA filled the TX FIFO; start loopback TX.
  REG_WRITE(I2S_TX_CONF, SIG_LOOPBACK | TX_START_BIT);
  bool dma_rx = wait_rx_done();
  uint32_t dma0 = REG_READ(I2S_FIFO);
  uint32_t dma1 = REG_READ(I2S_FIFO);
  REG_WRITE(I2S_INT_CLR, RX_DONE);
  Serial.print("dma0=");
  Serial.print(dma0, HEX);
  Serial.print(" dma1=");
  Serial.println(dma1, HEX);
  ok &= dma_rx && (dma0 == 0x12345678) && (dma1 == 0x9ABCDEF0);

  Serial.println(ok ? "I2S POKE PASS" : "I2S POKE FAIL");
  Serial.println("DONE");
}

void loop() {}
