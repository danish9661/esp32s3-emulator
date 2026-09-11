// ESP32-S3 ADC digital-DMA validation sketch (direct register pokes).
//
// Programs the APB_SARADC digital controller (0x60040000) for single-shot
// timer-triggered conversion on ADC1_CH3 (pattern byte = 11 dB atten +
// channel 3, same pin the periph sketch reads), waits for a conversion,
// then runs the GDMA IN link (peri_sel = 8 = ADC) and checks the moved
// sample against the injected voltage (ADC_INJECT_MV env, default 825 mV:
// 825 * 4095 / 3900 = 866 at 11 dB atten).
#include <Arduino.h>

#define APB 0x60040000u
#define A(x) ((volatile uint32_t*)(APB + (x)))
#define GDMA 0x6003F000u // single GDMA controller (DR_REG_GDMA_BASE)
#define D(x) ((volatile uint32_t*)(GDMA + (x)))

typedef struct {
  uint32_t dw0;
  uint32_t buf;
  uint32_t next;
  uint32_t rsvd;
} gdma_desc_t;

static volatile uint32_t g_rx;
static volatile gdma_desc_t g_rxdesc;

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 6);
  delay(50);

  // Pattern table 0: channel 3, 11 dB atten (byte = atten | ch<<2).
  *A(0x18) = (3) | (3 << 2);
  // CTRL: SAR clock gated on (bit 6), single mode unit 0, patt_len 0.
  *A(0x00) = (1u << 6);
  // CTRL2: timer enable (24) + timer source (11), target 1000.
  *A(0x04) = (1u << 24) | (1u << 11) | (1000u << 12);

  // Wait for the first conversion (adc1_done = INT_RAW bit 31).
  bool conv = false;
  for (uint32_t t = 0; t < 2000000; t++) {
    if (*A(0x60) & (1u << 31)) {
      conv = true;
      break;
    }
  }
  if (!conv) {
    Serial.println("ADC GDMA NO CONV");
    return;
  }

  // GDMA IN ch0 <- ADC: move one staged sample to DRAM.
  g_rxdesc.dw0 = (4) | ((4) << 12) | (1u << 30) | (1u << 31);
  g_rxdesc.buf = (uint32_t)&g_rx;
  g_rxdesc.next = 0;
  g_rxdesc.rsvd = 0;
  *D(0x48) = 8;  // in_peri_sel[0] = ADC
  uint32_t rlsb = ((uint32_t)&g_rxdesc) & 0x000FFFFFu;
  *D(0x20) = rlsb | (1u << 22);  // in start = bit 22
  uint32_t indone = *D(0x08) & 1u;
  uint32_t raw = g_rx & 0xFFFu;
  Serial.printf("ADC GDMA RAW=%u\n", raw);
  Serial.println((indone && raw == 866) ? "ADC GDMA PASS" : "ADC GDMA FAIL");
  Serial.println("ADC GDMA DONE");
}

void loop() {}
