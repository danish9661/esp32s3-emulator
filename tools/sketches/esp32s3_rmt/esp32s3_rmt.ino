// RMT TX validation sketch (ESP32-S3) — direct register驱动 (no esp-idf
// driver / GDMA, which the emulator does not model yet).
//
// Writes a short pulse train into RMT channel 0's item RAM and starts the
// transmitter by poking the real RMT registers, then polls the tx_end raw
// interrupt. Compiled with arduino-cli; runs identically on real silicon.
// Validates the emulator's RMT TX model (item memory, FSM, tx_end).

#define RMT_BASE 0x60016000UL
#define RMTMEM_BASE 0x60016800UL

volatile uint32_t* const RMT = (volatile uint32_t* const)RMT_BASE;
volatile uint32_t* const RMTMEM = (volatile uint32_t* const)RMTMEM_BASE;

// chnconf0[0] @ 0x20, chn_tx_lim[0] @ 0xA0, int_raw @ 0x70, int_ena @ 0x78,
// int_clr @ 0x7C.  RMTMEM channel 0 items start at offset 0x800.

void setup() {
  Serial.begin(115200);
  delay(50);

  // Channel 0 config: div_cnt=2, idle_out_en, idle_out_lv=1 (idle high).
  RMT[0x20 / 4] = (2 << 8) | (1 << 5) | (1 << 6);
  // 3 items (channel 0 item RAM at RMTMEM[0..2]).
  RMTMEM[0] = (100) | (1u << 15) | (100u << 16);  // pulse0=1/100t, pulse1=0/100t
  RMTMEM[1] = (200) | (1u << 15) | (50u << 16);   // pulse0=1/200t, pulse1=0/50t
  RMTMEM[2] = 0;                                  // terminator
  RMT[0xA0 / 4] = 3;                              // tx_lim = 3 items
  RMT[0x78 / 4] = 1;                              // enable tx_end (ch0)

  // Start transmit.
  RMT[0x20 / 4] |= 1;

  // Poll raw tx_end for channel 0.
  uint32_t t0 = millis();
  while (!(RMT[0x70 / 4] & 1u)) {
    if (millis() - t0 > 2000) {
      Serial.println("RMT TIMEOUT");
      return;
    }
  }
  Serial.println("RMT TX done");
  RMT[0x7C / 4] = 1;  // clear

  // RX loopback on the same pad: route TX ch0 (signal 81) to GPIO2 and
  // sample it back with RX channel 4 (input signal 81 <- GPIO2).
  volatile uint32_t* const GPIO = (volatile uint32_t* const)0x60004000UL;
  GPIO[0x554 / 4 + 2] = 81;    // FUNC_OUT_SEL_CFG[2] = RMT TX ch0
  GPIO[0x24 / 4] = 1u << 2;     // GPIO_ENABLE_W1TS bit 2
  GPIO[(0x154 + 81 * 4) / 4] = 2; // FUNC_IN_SEL[81] = GPIO2
  RMT[0x34 / 4] = 1;            // chmconf1[0]: rx_en
  // chmconf0[0]: idle timeout 2000 channel ticks, 1 memory block (also
  // proves the config registers program; default 32767 would work too but
  // leaves a saturated 15-bit trailing pulse in the dump).
  RMT[0x30 / 4] = (2000u << 8) | (1u << 24);
  RMT[0x20 / 4] |= 1;           // re-transmit (tx_start auto-cleared at end)
  t0 = millis();
  while (!(RMT[0x70 / 4] & (1u << 16))) {
    if (millis() - t0 > 2000) {
      Serial.println("RMT RX TIMEOUT");
      return;
    }
  }
  Serial.println("RMT RX done");
  // Block 4 starts at RMTMEM + 0x400. Look for the unambiguous HIGH-200
  // pulse among the first 4 received items (widths quantize to 32).
  bool found = false;
  for (int i = 0; i < 4; i++) {
    uint32_t w = RMTMEM[0x400 / 4 + i];
    uint32_t d0 = w & 0x7FFFu, l0 = (w >> 15) & 1u;
    uint32_t d1 = (w >> 16) & 0x7FFFu, l1 = (w >> 31) & 1u;
    Serial.printf("RX item%d = %lu/%lu %lu/%lu\n", i,
                  (unsigned long)d0, (unsigned long)l0,
                  (unsigned long)d1, (unsigned long)l1);
    if (l0 && d0 >= 136 && d0 <= 264) found = true;
    if (l1 && d1 >= 136 && d1 <= 264) found = true;
  }
  Serial.println(found ? "RMT RX PASS" : "RMT RX FAIL");
}

void loop() {}
