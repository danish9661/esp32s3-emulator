// Light-sleep UART0 wakeup validation (real esp-idf sleep driver): a byte
// pre-staged in the UART0 RX FIFO (harness UART_INJECT at the ARMED marker)
// wakes esp_light_sleep_start() immediately with the UART cause (TRIG bit 6).
// The byte is read back through the UART0 FIFO register (Serial is USB-CDC).
#include <Arduino.h>
#include "esp_sleep.h"

#define UART0_BASE 0x60000000u
#define U0(x) ((volatile uint32_t*)(UART0_BASE + (x)))

void setup() {
  Serial.begin(115200);
  esp_sleep_enable_uart_wakeup(0);
  Serial.println("SYNC");
  Serial.println("UART-SLEEP-ARMED");
  esp_err_t e = esp_light_sleep_start();
  auto cause = esp_sleep_get_wakeup_cause();
  // The sleep-entry FIFO reset drains the staged byte (silicon wakes on
  // the RX edge, not the level): rx must read EMPTY here, proving the wake
  // came from the sticky edge rather than a lingering level.
  uint32_t rx = *U0(0x00);  // pop UART0 RX FIFO (expect empty)
  Serial.printf("UART WOKE rc=%d cause=%d rx=%02lx\n", (int)e, (int)cause, (unsigned long)rx);
  bool ok = cause == ESP_SLEEP_WAKEUP_UART && rx == 0;
  Serial.println(ok ? "LIGHTSLEEP UART PASS" : "LIGHTSLEEP UART FAIL");
}

void loop() {}
