// TWAI (CAN 2.0B) self-test loopback validation for the ESP32-S3 emulator.
// Pokes the TWAI registers directly (no esp-idf driver) to avoid unmodeled
// paths, mirrors tools/sketches/esp32s3_{rmt,pcnt,gdma}.
//
// Flow: enter reset mode, leave reset in self-test mode (stm), load a 13-byte
// frame into the TX buffer, issue a transmission request, then poll the RX
// buffer status and compare the looped-back frame to what was sent.

#define TWAI 0x6002B000u

static volatile uint32_t* R(uint32_t off) {
  return (volatile uint32_t*)(TWAI + off);
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 19);
  // Frame info: DLC=8, standard (11-bit) data frame -> byte0 = 0x08.
  uint8_t tx[13];
  tx[0] = 0x08;
  // 11-bit ID 0x123, big-endian left-aligned (see twai_ll_format_frame_buffer).
  tx[1] = 0x24;
  tx[2] = 0x60;
  for (int i = 3; i < 13; i++) tx[i] = (uint8_t)(0x10 + i);

  // Enter reset mode (rm = bit0) so the acceptance filter is writable.
  *R(0x00) = 1;
  // Acceptance filter: code 0, mask 0xFFFFFFFF (all don't-care) -> accept all.
  *R(0x40) = 0; *R(0x44) = 0; *R(0x48) = 0; *R(0x4C) = 0;
  *R(0x50) = 0xFFFFFFFFu; *R(0x54) = 0xFFFFFFFFu; *R(0x58) = 0xFFFFFFFFu; *R(0x5C) = 0xFFFFFFFFu;
  // Leave reset, enter self-test mode (stm = bit2) so TX loops back to RX.
  *R(0x00) = (1u << 2);

  // Load the frame into the TX buffer (operational-mode registers 0x40..0x70).
  for (int i = 0; i < 13; i++) *R(0x40 + i * 4) = tx[i];

  // Transmission request (command.tr = bit0).
  *R(0x04) = 1;

  // Poll the RX-buffer status bit (SR.rbs = bit0) until the frame arrives.
  uint32_t st = 0;
  bool got = false;
  for (int t = 0; t < 100000; t++) {
    st = *R(0x08);
    if (st & 1) { got = true; break; }
  }

  uint8_t rx[13];
  for (int i = 0; i < 13; i++) rx[i] = (uint8_t)*R(0x40 + i * 4);

  bool match = got;
  for (int i = 0; i < 13; i++) {
    if (rx[i] != tx[i]) match = false;
  }

  if (match) {
    Serial.println("TWAI LOOPBACK PASS");
  } else {
    Serial.print("TWAI LOOPBACK FAIL rbs=");
    Serial.println(got ? 1 : 0);
    Serial.print("rx0=");
    Serial.println(rx[0], HEX);
  }

  // Release the RX buffer (command.rrb = bit2) and confirm rbs clears.
  *R(0x04) = (1u << 2);
  if ((*R(0x08) & 1) == 0) {
    Serial.println("TWAI RRB OK");
  } else {
    Serial.println("TWAI RRB FAIL");
  }
}

void loop() {}
