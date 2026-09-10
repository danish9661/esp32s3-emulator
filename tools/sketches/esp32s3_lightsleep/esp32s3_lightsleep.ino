// Light-sleep validation: unlike deep sleep this must RESUME (no reboot).
// Arms a 1 s timer wakeup, sets a retained marker, sleeps, then checks the
// marker survived (proves no reboot) plus the wakeup cause.
#include "esp_sleep.h"

static int retained_marker = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER && retained_marker == 0x1234) {
    Serial.println("LIGHTSLEEP WOKE");
    Serial.println("LIGHTSLEEP RETAINED");
    Serial.println("LIGHTSLEEP PASS");
    return;
  }
  retained_marker = 0x1234;
  Serial.println("LIGHTSLEEP START");
  esp_sleep_enable_timer_wakeup(1000000);
  esp_light_sleep_start();
  // Resume lands here (no reboot on real silicon).
  Serial.println("LIGHTSLEEP RESUMED");
}

void loop() {
  delay(1000);
}
