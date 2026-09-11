// BOD (brown-out detector) reset validation sketch (direct pokes).
// Programs RTC_CNTL BROWN_OUT (ena + rst_ena + short waits) and spins;
// the HOST (run_flash with BOD_INJECT=1) holds the low-voltage condition,
// so the detector requests a chip reset and the emulator reboots into this
// sketch again (like the WDT reset sketch).
#include <Arduino.h>

#define RTC_CNTL_BASE 0x60008000u
#define R(x) ((volatile uint32_t*)(RTC_CNTL_BASE + (x)))

void setup() {
  Serial.begin(115200);
  Serial.println("BOD RESET TEST");
  *R(0xE8) = (1u << 30) | (1u << 26) | (50u << 16) | (50u << 4);
  for (;;) {
  }
}

void loop() {}
