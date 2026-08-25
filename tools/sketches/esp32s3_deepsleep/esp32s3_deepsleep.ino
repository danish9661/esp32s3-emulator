// Deep-sleep validation for the ESP32-S3 emulator.
//
// On first boot (wakeup cause == UNDEFINED) it prints a marker and arms a
// 2-second timer wakeup, then enters deep sleep.  The emulator fast-forwards
// the sleep and reboots with the timer wakeup-cause bit set, so setup() runs
// again and (cause == TIMER) prints a PASS marker and idles.
//
// Expected run_flash output (grep): "DEEPSLEEP START" then (after reboot)
// "DEEPSLEEP WOKE" and "DEEPSLEEP PASS".

#include "esp_sleep.h"

#define uS_TO_S_FACTOR 1000000ULL

void setup() {
  Serial.begin(115200);
  delay(50);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("DEEPSLEEP WOKE");
    Serial.println("DEEPSLEEP PASS");
    while (1) {
      delay(10);
    }
  }

  Serial.println("DEEPSLEEP START");
  esp_sleep_enable_timer_wakeup(2 * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
  // Not reached.
  Serial.println("DEEPSLEEP SHOULD NOT REACH");
}

void loop() {}
