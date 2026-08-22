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
}

void loop() {}
