// BOD (brown-out detector) interrupt validation sketch (direct pokes).
// Programs RTC_CNTL BROWN_OUT (ena + short int_wait, no reset) and polls
// INT_RAW brown_out (bit 9). The HOST (run_flash with BOD_INJECT=1) holds
// the low-voltage condition; without it nothing trips. The RTC interrupt
// enable for brown_out stays as the bootloader left it off here (polling
// RAW needs no matrix routing; an unhandled RTC interrupt would crash,
// so a driver using the interrupt must install an ISR).
#include <Arduino.h>

#define RTC_CNTL_BASE 0x60008000u
#define R(x) ((volatile uint32_t*)(RTC_CNTL_BASE + (x)))

void setup() {
  Serial.begin(115200);
  // Poll RAW without the matrix ISR: IDF installs its own brownout
  // handler (prints "Brownout detector was triggered" and restarts),
  // which would race these prints. Clearing the enable keeps RAW
  // latching while the ISR stays quiet — standard polling practice.
  *R(0x40) &= ~(1u << 9);             // INT_ENA without brown_out
  *R(0xE8) = (1u << 30) | (100u << 4);  // ena + int_wait=100
  uint32_t ok = 0;
  for (uint32_t t = 0; t < 2000000; t++) {
    if (*R(0x44) & (1u << 9)) {
      ok = 1;
      break;
    }
  }
  Serial.println(ok ? "BOD INT OK" : "BOD INT MISSING");
  *R(0x4C) = (1u << 9);  // INT_CLR
  Serial.println((*R(0x44) & (1u << 9)) == 0 ? "BOD PASS" : "BOD FAIL");
}

void loop() {}
