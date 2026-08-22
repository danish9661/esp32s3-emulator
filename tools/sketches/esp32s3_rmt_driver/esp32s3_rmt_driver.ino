// RMT driver (new esp-idf driver, GDMA-backed) validation for the ESP32-S3
// emulator. Uses the Arduino `rmtWrite` API, which on S3 routes the item
// buffer through GDMA into RMTMEM and then drives the pin via the RMT TX FSM.
// This exercises the full GDMA -> RMT path (unmodeled before the GDMA
// peripheral was added, which made the driver hang).
//
// The live pin waveform is timing-sensitive in the emulator (the RMT FSM
// advances every step, so a transmission completes inside the driver's
// rmt_transmit() call). Instead we assert the deterministic side effect that
// proves the GDMA path worked: after rmtWrite returns true, the item buffer
// must have been copied into RMTMEM (channel 0 item RAM at 0x60016800). The
// live waveform / GPIO_IN loopback path is covered by the Rust machine test
// `rmt_signal_drives_gpio_in_loopback`.

#include "Arduino.h"

#define TX_PIN 2
#define N_ITEMS 64

// RMT channel 0 item RAM (ESP-IDF RMT_CHANNEL_MEM: RMTMEM_BASE + ch*0x100).
#define RMTMEM_CH0 ((volatile uint32_t*)0x60016800)

rmt_data_t items[N_ITEMS];

void setup() {
  Serial.begin(115200);

  // 1 MHz RMT tick. Each item: 0x7FFF ticks HIGH, 0x7FFF ticks LOW.
  for (int i = 0; i < N_ITEMS; i++) {
    items[i].duration0 = 0x7FFF; items[i].level0 = 1;
    items[i].duration1 = 0x7FFF; items[i].level1 = 0;
  }

  bool ok = rmtInit(TX_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 1000000);
  if (!ok) {
    Serial.println("RMT INIT FAIL");
    return;
  }

  // Blocking transmit must return true: requires GDMA to copy the items into
  // RMTMEM and the RMT FSM to raise tx_end (the path that used to hang).
  ok = rmtWrite(TX_PIN, items, N_ITEMS, RMT_WAIT_FOR_EVER);
  if (!ok) {
    Serial.println("RMT DRIVER TX FAIL");
    return;
  }
  Serial.println("RMT DRIVER TX done");

  // Deterministically prove the GDMA copy happened: the driver's item buffer
  // must now live in RMTMEM channel 0.
  uint32_t want = items[0].val;
  uint32_t got = RMTMEM_CH0[0];
  if (got == want) {
    Serial.printf("RMT DRIVER GDMA copied item=%08x\n", got);
    Serial.println("RMT DRIVER PASS");
  } else {
    Serial.printf("RMT DRIVER FAIL rmtmem0=%08x want=%08x\n", got, want);
  }
}

void loop() {}
