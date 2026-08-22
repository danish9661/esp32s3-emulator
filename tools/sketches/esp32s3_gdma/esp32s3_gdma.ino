// GDMA + RMT TX validation sketch (ESP32-S3) — exercises the GDMA controller
// directly (no esp-idf driver) to validate the emulator's GDMA model: a
// descriptor chain is walked by GDMA and its buffer copied into RMT channel 0's
// item RAM (RMTMEM), then RMT transmits the items and raises tx_end.
//
// Compiled with arduino-cli; runs identically on real silicon (where GDMA
// physically performs the copy). Validates: GDMA descriptor-walk, copy into
// RMTMEM, out_done interrupt, and the RMT TX path fed by GDMA.

#define RMT_BASE    0x60016000UL
#define RMTMEM_BASE 0x60016800UL
#define GDMA_BASE   0x60042000UL

// GDMA channel 0 OUT block = ch0 stride(0xC0) + out offset(0x60) = 0x60.
#define G_DMA_OUT0  0x60
// OUT-block register offsets (soc/gdma_struct.h): conf0 0x00, int_ena 0x10,
// int_clr 0x14, link 0x20, peri_sel 0x48.
#define G_OUT_CONF0   ((GDMA_BASE + G_DMA_OUT0 + 0x00))
#define G_OUT_INT_ENA ((GDMA_BASE + G_DMA_OUT0 + 0x10))
#define G_OUT_INT_CLR ((GDMA_BASE + G_DMA_OUT0 + 0x14))
#define G_OUT_LINK    ((GDMA_BASE + G_DMA_OUT0 + 0x20))
#define G_OUT_PERI    ((GDMA_BASE + G_DMA_OUT0 + 0x48))

// RMT registers (soc/rmt_struct.h): chnconf0[n] @ 0x20+4n, int_raw 0x70,
// int_ena 0x78, int_clr 0x7C, chn_tx_lim[n] @ 0xA0+4n. RMTMEM ch0 @ 0x60016800.

typedef struct {
  uint32_t dw0;   // size[11:0] | length[23:12] | eof[30] | owner[31]
  uint32_t buf;   // source buffer address
  uint32_t next;  // next descriptor (0 = end)
  uint32_t rsvd;
} gdma_desc_t;

// Item buffer (lives in DRAM) + descriptor (also DRAM).  Both MUST be volatile:
// the GDMA controller reads them, and the compiler must not reorder their
// stores after the volatile GDMA register write that starts the transfer
// (real DMA drivers use memory barriers / non-cacheable DMA memory for this).
static volatile uint32_t g_items[4];
static volatile gdma_desc_t g_desc;

volatile uint32_t* const RMT = (volatile uint32_t* const)RMT_BASE;

void setup() {
  Serial.begin(115200);
  delay(50);

  // Build 3 RMT items: (100t hi)(100t lo) / (200t hi)(50t lo) / (50t lo)(50t lo).
  g_items[0] = (100) | (1u << 15) | (100u << 16);
  g_items[1] = (200) | (1u << 15) | (50u << 16);
  g_items[2] = (50)  | (0u << 15) | (50u << 16);
  g_items[3] = 0;  // terminator

  // Build one GDMA descriptor pointing at the item buffer (length = 4 items * 4B).
  g_desc.dw0 = (16) | ((4 * 4) << 12) | (1u << 30) | (1u << 31);
  g_desc.buf = (uint32_t)g_items;
  g_desc.next = 0;
  g_desc.rsvd = 0;

  // Configure GDMA OUT channel 0 to target RMT (peri_sel = 9).
  *((volatile uint32_t*)G_OUT_PERI) = 9;
  // Enable out_done / out_eof / out_total_eof interrupts (optional, polled).
  *((volatile uint32_t*)G_OUT_INT_ENA) = (1u << 0) | (1u << 1) | (1u << 3);
  // Program descriptor address (20 LSBs) then start the transfer.
  uint32_t desc_lsb = ((uint32_t)&g_desc) & 0x000FFFFFu;
  *((volatile uint32_t*)G_OUT_LINK) = desc_lsb;
  *((volatile uint32_t*)G_OUT_LINK) = desc_lsb | (1u << 21);  // start = bit21

  // Poll GDMA out_done (raw int bit 0). Copy is performed synchronously by the
  // emulator, so this is observed immediately after the start write.
  uint32_t t0 = millis();
  uint32_t gdma_done = 0;
  while (1) {
    // int_raw mirrors out_done after the walk; read via the link-backed int_raw
    // by re-reading the OUT int status: we read out.int_raw via a small helper.
    // Simpler: read the raw reg at G_OUT + 0x08.
    uint32_t raw = *((volatile uint32_t*)(GDMA_BASE + G_DMA_OUT0 + 0x08));
    if (raw & 1u) { gdma_done = 1; break; }
    if (millis() - t0 > 2000) break;
  }
  if (!gdma_done) {
    Serial.println("GDMA TIMEOUT");
    return;
  }
  // Clear GDMA interrupt.
  *((volatile uint32_t*)G_OUT_INT_CLR) = (1u << 0) | (1u << 1) | (1u << 3);

  // Sanity: confirm RMTMEM ch0[0..2] now equals the item buffer (GDMA copied it).
  volatile uint32_t* const RMTMEM = (volatile uint32_t* const)RMTMEM_BASE;
  if (RMTMEM[0] != g_items[0] || RMTMEM[1] != g_items[1] || RMTMEM[2] != g_items[2]) {
    Serial.print("GDMA COPY MISMATCH rm0=");
    Serial.print(RMTMEM[0], HEX);
    Serial.print(" it0=");
    Serial.print(g_items[0], HEX);
    Serial.print(" rm1=");
    Serial.print(RMTMEM[1], HEX);
    Serial.print(" it1=");
    Serial.print(g_items[1], HEX);
    Serial.print(" rm2=");
    Serial.print(RMTMEM[2], HEX);
    Serial.print(" it2=");
    Serial.println(g_items[2], HEX);
    return;
  }

  // Now drive RMT TX from the GDMA-filled RMTMEM.
  RMT[0x20 / 4] = (2 << 8) | (1 << 5) | (1 << 6);  // div=2, idle_out_en, idle=1
  RMT[0xA0 / 4] = 4;                               // tx_lim = 4 items
  RMT[0x78 / 4] = 1;                               // enable tx_end (ch0)

  RMT[0x20 / 4] |= 1;                              // tx_start

  t0 = millis();
  while (!(RMT[0x70 / 4] & 1u)) {
    if (millis() - t0 > 2000) {
      Serial.println("RMT TIMEOUT");
      return;
    }
  }
  Serial.println("GDMA RMT TX done");
  RMT[0x7C / 4] = 1;  // clear
}

void loop() {}
