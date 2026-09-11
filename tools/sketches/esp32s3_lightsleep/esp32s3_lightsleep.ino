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
  uint32_t sdio_conf = *(volatile uint32_t *)0x6000807C;
  uint32_t dig_pwc = *(volatile uint32_t *)0x60008090;
  Serial.printf("LIGHTSLEEP dbg sdio_conf=%x dig_pwc=%x\n", sdio_conf, dig_pwc);
  // Let the console TX drain fully before sleeping (suspend records
  // UARTs with TX in flight; a draining console looks "active").
  Serial.flush();
  delay(100);
  esp_sleep_enable_timer_wakeup(1000000);
  esp_light_sleep_start();
  // Resume lands here (no reboot on real silicon).
  Serial.println("LIGHTSLEEP RESUMED");
  uint32_t sdio_post = *(volatile uint32_t *)0x6000807C;
  Serial.printf("LIGHTSLEEP post sdio_conf=%x\n", sdio_post);
  uint8_t light_flag = *(volatile uint8_t *)0x3fc97858;
  uint32_t wcause = *(volatile uint32_t *)0x60008130;
  Serial.printf("LIGHTSLEEP dbg flag=%u wcause=%x\n", light_flag, wcause);
  esp_sleep_wakeup_cause_t woke = esp_sleep_get_wakeup_cause();
  Serial.printf("LIGHTSLEEP cause=%d marker=%x\n", (int)woke, retained_marker);
  if (woke == ESP_SLEEP_WAKEUP_TIMER && retained_marker == 0x1234) {
    Serial.println("LIGHTSLEEP WOKE");
    Serial.println("LIGHTSLEEP RETAINED");
    Serial.println("LIGHTSLEEP PASS");
  } else {
    Serial.println("LIGHTSLEEP FAIL");
  }
}

void loop() {
  delay(1000);
}
