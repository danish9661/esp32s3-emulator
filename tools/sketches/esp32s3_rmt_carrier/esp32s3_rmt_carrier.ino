// RMT carrier modulation + RX demodulation validation (direct register
// pokes, mirrors esp32s3_rmt).
//
// TX channel 0 sends four HIGH/LOW items with the carrier enabled (20/20
// channel-tick duty, modulate-on-high, data-state only). The firmware
// samples the GPIO2 pad via raw GPIO_IN reads in a tight loop: the
// carrier must chop the bursts (many pad edges), where unmodulated bursts
// would read steady (2 edges). Four items give ~37 burst samples so the
// verdict is statistical, not sample-phase luck. RX HW channel 4 captures
// the same pad with demodulation enabled and must record the HIGH envelope
// plus rx_end (widths in channel ticks; the demodulator releases up to
// DEMOD_RELEASE quanta late, tolerated by the envelope bounds).

#define RMT_BASE 0x60016000UL
#define RMTMEM_BASE 0x60016800UL
#define GPIO_BASE 0x60004000UL

volatile uint32_t* const RMT = (volatile uint32_t* const)RMT_BASE;
volatile uint32_t* const RMTMEM = (volatile uint32_t* const)RMTMEM_BASE;
volatile uint32_t* const GPIO = (volatile uint32_t* const)GPIO_BASE;

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 9);
  delay(50);

  // Snapshot pre-existing RMT state (Arduino core / bootloader baggage?).
  Serial.printf("RMT CARRIER pre conf=%lx raw=%lx mem0=%lx rxmem0=%lx\n",
                (unsigned long)RMT[0x20 / 4], (unsigned long)RMT[0x70 / 4],
                (unsigned long)RMTMEM[0], (unsigned long)RMTMEM[0x400 / 4]);

  // Route TX ch0 (signal 81) to GPIO2, enable driver; loop GPIO2 back
  // into RX signal 81.
  GPIO[0x554 / 4 + 2] = 81;
  GPIO[0x24 / 4] = 1u << 2;
  GPIO[(0x154 + 81 * 4) / 4] = 2;

  // Two items: HIGH 1200 / LOW 1200 each. A single item gives only ~4
  // pad samples at the firmware's ~8-steps-per-iteration sampling rate,
  // which can alias badly enough to fake a clean burst (observed 4
  // consecutive HIGHs, edges=1); two items give ~18 burst samples so the
  // chopped verdict is statistical, not phase luck.
  RMTMEM[0] = (1200u) | (1u << 15) | (1200u << 16);
  RMTMEM[1] = (1200u) | (1u << 15) | (1200u << 16);
  RMTMEM[2] = (1200u) | (1u << 15) | (1200u << 16);
  RMTMEM[3] = (1200u) | (1u << 15) | (1200u << 16);
  RMT[0xA0 / 4] = 4;  // tx_lim = 4 items
  // RX: idle timeout 4096, 1 block, demod on bursting high.
  RMT[0x30 / 4] = (4096u << 8) | (1u << 24) | (1u << 28) | (1u << 29);
  RMT[0x34 / 4] = 1;  // rx_en
  // Carrier duty 20 high / 20 low channel ticks (period 40: sampled
  // every ~256 ticks the pad walks 16-tick phases, transitioning ~4/5
  // samples while chopped vs 2 edges clean over the whole trace).
  RMT[0x80 / 4] = (20u << 16) | 20u;
  // TX: start + idle-low + EFF_EN + CARRIER_EN + OUT_LV high.
  RMT[0x20 / 4] = 1u | (1u << 6) | (1u << 20) | (1u << 21) | (1u << 22);
  Serial.printf("RMT CARRIER cfg conf=%lx duty=%lx rxconf=%lx rxen=%lx txlim=%lx rxlim=%lx\n",
                (unsigned long)RMT[0x20 / 4], (unsigned long)RMT[0x80 / 4],
                (unsigned long)RMT[0x30 / 4], (unsigned long)RMT[0x34 / 4],
                (unsigned long)RMT[0xA0 / 4], (unsigned long)RMT[0xB0 / 4]);
  Serial.printf("RMT CARRIER txmem0=%lx\n", (unsigned long)RMTMEM[0]);

  // The config printfs above take hundreds of emulated steps, during which
  // the armed RX already captures the TX burst into the void (rx_end would
  // be set before the poll loop below samples a single pad). So clear the
  // end flag, re-arm RX and restart TX *after* the printfs, then sample
  // the live retransmission.
  RMT[0x7C / 4] = 1u << 16;  // INT_CLR rx_end
  RMT[0x34 / 4] = 0;         // rx_en falling (re-arm needs a rising edge)
  RMT[0x34 / 4] = 1;         // rx_en rising: re-arm one-shot capture
  // NOTE: no prints from here to the poll loop: even one printf spans the
  // whole ~75-step burst plus the rx_end idle window in emulated time.
  RMT[0x20 / 4] = 1u | (1u << 6) | (1u << 20) | (1u << 21) | (1u << 22);
  uint32_t rearm_raw = RMT[0x70 / 4];

  // Combined sample+poll loop: record the pad each iteration while waiting
  // for rx_end, so the pad trace covers the live transmission (a separate
  // sampling pass afterwards would only see the idle level).
  // Tight sample+poll loop: one GPIO read + one INT_RAW read per
  // iteration (~a few emulated steps). The loop body must stay tiny:
  // a millis() call per iteration costs hundreds of steps (64-bit
  // division) and the whole burst + rx_end idle window would elapse
  // between two samples (observed n=1). The iteration cap bounds a
  // missing-rx_end hang without any slow calls.
  uint32_t traw = 0, tpad[128];
  int tn = 0;
  for (uint32_t i = 0; i < 10000u; i++) {
    uint32_t pad = (GPIO[0x3C / 4] >> 2) & 1u;
    uint32_t raw = RMT[0x70 / 4];
    if (tn < 128) {
      traw = raw;
      // Pad bit only (bit 0). Do NOT pack TX_START/MEM_OWNER flags here:
      // their transitions are not pad edges and would fake the chopped
      // verdict.
      tpad[tn] = pad;
      tn++;
    }
    if (raw & (1u << 16)) break;
  }
  if (!(traw & (1u << 16))) {
    Serial.println("RMT CARRIER RX TIMEOUT");
    return;
  }
  // Count pad edges (transitions) across the trace: a clean burst has 2,
  // a carrier-chopped burst has many.
  uint32_t edges = 0, high = 0;
  for (int i = 0; i < tn; i++) {
    high += tpad[i];
    if (i > 0 && tpad[i] != tpad[i - 1]) edges++;
  }
  Serial.printf("RMT CARRIER rearm raw=%lx\n", (unsigned long)rearm_raw);
  Serial.printf("RMT CARRIER n=%d edges=%lu high=%lu txend=%lx\n", tn,
                (unsigned long)edges, (unsigned long)high,
                (unsigned long)(traw & 1u));
  bool chopped = edges >= 6;
  // HW channel 4 block starts at RMTMEM + 0x400.
  uint32_t w = RMTMEM[0x400 / 4];
  uint32_t d0 = w & 0x7FFFu, l0 = (w >> 15) & 1u;
  uint32_t d1 = (w >> 16) & 0x7FFFu, l1 = (w >> 31) & 1u;
  Serial.printf("RMT CARRIER rx item=%lu/%lu %lu/%lu\n", (unsigned long)d0,
                (unsigned long)l0, (unsigned long)d1, (unsigned long)l1);
  uint32_t w1 = RMTMEM[0x400 / 4 + 1];
  uint32_t w2 = RMTMEM[0x400 / 4 + 2];
  uint32_t txraw = RMT[0x70 / 4];
  uint32_t chnstatus = RMT[0x50 / 4];
  uint32_t txconf = RMT[0x20 / 4];
  uint32_t outsel = GPIO[0x554 / 4 + 2];
  uint32_t insel = GPIO[(0x154 + 81 * 4) / 4];
  Serial.printf("RMT CARRIER dbg txraw=%lx item1=%lx item2=%lx st=%lx conf=%lx outsel=%lx insel=%lx\n",
                (unsigned long)txraw, (unsigned long)w1, (unsigned long)w2,
                (unsigned long)chnstatus, (unsigned long)txconf,
                (unsigned long)outsel, (unsigned long)insel);
  for (int i = 0; i < 6; i++) {
    uint32_t wi = RMTMEM[0x400 / 4 + i];
    Serial.printf("RMT CARRIER blk%d=%lx\n", i, (unsigned long)wi);
  }
  Serial.printf("RMT CARRIER rxen=%lx rxlim=%lx chconf0=%lx\n",
                (unsigned long)RMT[0x34 / 4], (unsigned long)RMT[0xB0 / 4],
                (unsigned long)RMT[0x30 / 4]);
  bool envelope = l0 == 1 && d0 >= 800 && d0 <= 1600 && l1 == 0 && d1 > 300;
  Serial.println(chopped && envelope ? "RMT CARRIER PASS" : "RMT CARRIER FAIL");
}

void loop() {}
