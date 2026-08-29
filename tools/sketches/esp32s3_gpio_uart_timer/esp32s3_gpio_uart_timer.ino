// Multi-peripheral interaction: timer-driven UART TX + GPIO toggle.
// TIMG0 alarm fires every 50 ms; the ISR toggles GPIO2 and prints a
// timestamp on UART0.  Validates that timer interrupts, GPIO output,
// and UART TX all work together under interrupt pressure.

#include "driver/timer.h"

#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_IDX   TIMER_0
#define TIMER_DIV   80
#define ALARM_US    50000   // 50 ms

volatile uint32_t tick_count = 0;
volatile bool toggle_state = false;

void IRAM_ATTR timer_isr(void* arg) {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_IDX);
  tick_count++;
  toggle_state = !toggle_state;
  digitalWrite(2, toggle_state ? HIGH : LOW);
  Serial.printf("[ISR] tick %lu gpio2=%d\n", tick_count, (int)toggle_state);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  timer_config_t cfg = {};
  cfg.divider = TIMER_DIV;
  cfg.counter_dir = TIMER_COUNT_UP;
  cfg.alarm_en = TIMER_ALARM_EN;
  cfg.auto_reload = TIMER_AUTORELOAD_EN;
  cfg.counter_en = TIMER_PAUSE;
  timer_init(TIMER_GROUP, TIMER_IDX, &cfg);
  timer_set_counter_value(TIMER_GROUP, TIMER_IDX, 0);
  timer_set_alarm_value(TIMER_GROUP, TIMER_IDX, ALARM_US);
  timer_enable_intr(TIMER_GROUP, TIMER_IDX);
  timer_isr_register(TIMER_GROUP, TIMER_IDX, timer_isr, NULL, 0, NULL);
  timer_start(TIMER_GROUP, TIMER_IDX);

  Serial.println("MULTI_PERIPH START");
  delay(300);  // let a few ISR ticks fire

  uint32_t count = tick_count;
  int gpio = digitalRead(2);
  Serial.printf("MULTI_PERIPH ticks=%lu gpio2=%d\n", count, gpio);
  Serial.println(count > 0 ? "MULTI_PERIPH PASS" : "MULTI_PERIPH FAIL");
}

void loop() {
  delay(1000);
}
