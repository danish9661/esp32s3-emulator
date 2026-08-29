// Timer Group alarm interrupt validation: arms TIMG0 timer 0 with a
// 64-bit alarm value and an interrupt action.  The ISR fires and sets a
// flag; the main loop prints the result.  Validates the timer interrupt
// path end-to-end.

#include "driver/timer.h"

#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_IDX   TIMER_0
#define TIMER_DIV   80          // 80 MHz / 80 = 1 MHz (1 us/tick)
#define TIMER_ALARM 100000      // 100 ms alarm

volatile bool alarm_fired = false;

void IRAM_ATTR timer_isr(void* arg) {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_IDX);
  alarm_fired = true;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  timer_config_t cfg = {};
  cfg.divider = TIMER_DIV;
  cfg.counter_dir = TIMER_COUNT_UP;
  cfg.alarm_en = TIMER_ALARM_EN;
  cfg.auto_reload = TIMER_AUTORELOAD_DIS;
  cfg.counter_en = TIMER_PAUSE;
  timer_init(TIMER_GROUP, TIMER_IDX, &cfg);
  timer_set_counter_value(TIMER_GROUP, TIMER_IDX, 0);
  timer_set_alarm_value(TIMER_GROUP, TIMER_IDX, TIMER_ALARM);
  timer_enable_intr(TIMER_GROUP, TIMER_IDX);
  timer_isr_register(TIMER_GROUP, TIMER_IDX, timer_isr, NULL, 0, NULL);
  timer_start(TIMER_GROUP, TIMER_IDX);

  // Wait for the alarm
  uint32_t t0 = millis();
  while (!alarm_fired && (millis() - t0) < 2000) { /* spin */ }

  uint32_t elapsed = millis() - t0;
  Serial.printf("TIMER_ALARM elapsed=%lu fired=%d\n", elapsed, (int)alarm_fired);
  Serial.println(alarm_fired ? "TIMER_ALARM PASS" : "TIMER_ALARM FAIL");
}

void loop() {
  delay(1000);
}
