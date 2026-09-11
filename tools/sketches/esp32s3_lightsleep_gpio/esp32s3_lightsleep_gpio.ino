// Light-sleep GPIO wakeup validation (real esp-idf sleep driver): GPIO4 is
// driven high, its RTCIO wakeup is armed for high level, then
// esp_light_sleep_start() must resume immediately with the GPIO cause
// (RTC_CNTL WAKEUP_STATE TRIG bit 2, esp_rom rtc.h GPIO_TRIG = BIT2).
#include <Arduino.h>
#include "esp_sleep.h"

void setup() {
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  // RTCIO PIN4_REG @ 0x60008438: WAKEUP_ENABLE (bit 10) + high level (5).
  *(volatile uint32_t*)0x60008438u |= (1u << 10) | (5u << 7);
  esp_sleep_enable_gpio_wakeup();
  Serial.println("SYNC");
  Serial.println("GPIO-SLEEP-ARMED");
  esp_err_t e = esp_light_sleep_start();
  auto cause = esp_sleep_get_wakeup_cause();
  Serial.printf("GPIO WOKE rc=%d cause=%d\n", (int)e, (int)cause);
  Serial.println(cause == ESP_SLEEP_WAKEUP_GPIO ? "LIGHTSLEEP GPIO PASS" : "LIGHTSLEEP GPIO FAIL");
}

void loop() {}
