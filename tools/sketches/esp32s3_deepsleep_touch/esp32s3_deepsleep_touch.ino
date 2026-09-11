// Deep-sleep touch-pad wakeup validation for the ESP32-S3 emulator (real
// esp-idf sleep driver: `esp_sleep_enable_touchpad_wakeup` +
// `esp_deep_sleep_start`). The touch threshold is poked directly (THRES3 =
// SENS_BASE + 0x64 + 2*4); the host injects the pad counter via
// TOUCH_INJECT=3:1877, so pad 3 reads touched (1877 < 2000). Sleep entry
// evaluates WAKEUP_ENA bit 8 + the live touch state and wakes immediately
// with the touch cause; after reboot `esp_sleep_get_wakeup_cause()` must
// report ESP_SLEEP_WAKEUP_TOUCHPAD.

#include <Arduino.h>
#include "esp_sleep.h"

#define SENS_BASE 0x60008800u
#define THRES3 (SENS_BASE + 0x64 + 2 * 4)

void setup() {
  Serial.begin(115200);
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TOUCHPAD) {
    Serial.println("DEEPSLEEP TOUCH WOKE");
    // SLP_STATUS must report the triggering pad's counter (1877).
    uint32_t slp = *(volatile uint32_t *)(SENS_BASE + 0xDC) & 0x3FFFFFu;
    Serial.printf("DEEPSLEEP TOUCH SLP=%u\n", (unsigned)slp);
    Serial.println(slp == 1877 ? "DEEPSLEEP TOUCH PASS" : "DEEPSLEEP TOUCH FAIL");
    return;
  }
  Serial.println("DEEPSLEEP TOUCH START");
  *(volatile uint32_t *)THRES3 = 2000;
  esp_sleep_enable_touchpad_wakeup();
  esp_deep_sleep_start();
}

void loop() { delay(1000); }
