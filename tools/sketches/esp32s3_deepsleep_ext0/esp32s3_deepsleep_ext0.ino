// Deep-sleep EXT0 wakeup validation (ESP32-S3 emulator).
//
// First boot (cause UNDEFINED): drive GPIO4 HIGH (already-high level),
// arm EXT0 on GPIO4/HIGH via the real esp-idf driver, and enter deep
// sleep. The emulator must wake immediately with the EXT0 cause bit, so
// setup() runs again and prints WOKE/PASS, then idles.
//
// Expected run_flash output: "DEEPSLEEP EXT0 START" then (after reboot)
// "DEEPSLEEP EXT0 WOKE" and "DEEPSLEEP EXT0 PASS".

#include "esp_sleep.h"

void setup() {
  Serial.begin(115200);
  delay(50);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("DEEPSLEEP EXT0 WOKE");
    Serial.println("DEEPSLEEP EXT0 PASS");
    while (1) {
      delay(10);
    }
  }

  Serial.println("DEEPSLEEP EXT0 START");
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)4, 1);
  esp_deep_sleep_start();
  // Not reached.
  Serial.println("DEEPSLEEP EXT0 SHOULD NOT REACH");
}

void loop() {}
