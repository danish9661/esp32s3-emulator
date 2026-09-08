// Deep-sleep EXT1 wakeup validation (ESP32-S3 emulator).
//
// First boot (cause UNDEFINED): drive GPIO4 HIGH (GPIO5 stays LOW),
// arm EXT1 on pins {4,5} ANY_HIGH via the real esp-idf driver, and enter
// deep sleep. The emulator must wake immediately (GPIO4 already HIGH)
// with the EXT1 cause bit, so setup() runs again, prints WOKE/PASS plus
// the EXT1 wakeup status (bit 4 set), then idles.
//
// Expected run_flash output: "DEEPSLEEP EXT1 START" then (after reboot)
// "DEEPSLEEP EXT1 WOKE", "DEEPSLEEP EXT1 STATUS=10", "DEEPSLEEP EXT1 PASS".

#include "esp_sleep.h"

void setup() {
  Serial.begin(115200);
  delay(50);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("DEEPSLEEP EXT1 WOKE");
    uint64_t status = esp_sleep_get_ext1_wakeup_status();
    Serial.printf("DEEPSLEEP EXT1 STATUS=%llX\n", status);
    if ((status & (1ULL << 4)) && !(status & (1ULL << 5))) {
      Serial.println("DEEPSLEEP EXT1 PASS");
    } else {
      Serial.println("DEEPSLEEP EXT1 STATUS MISMATCH");
    }
    while (1) {
      delay(10);
    }
  }

  Serial.println("DEEPSLEEP EXT1 START");
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  esp_sleep_enable_ext1_wakeup((1ULL << 4) | (1ULL << 5), ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
  // Not reached.
  Serial.println("DEEPSLEEP EXT1 SHOULD NOT REACH");
}

void loop() {}
