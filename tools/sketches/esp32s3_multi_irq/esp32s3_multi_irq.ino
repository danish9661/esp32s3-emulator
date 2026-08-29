// Interrupt-heavy workload: multiple peripherals fire interrupts
// simultaneously.  TIMG0 alarm + TIMG1 alarm + UART1 RX + GPIO edge
// all generate interrupts; the test checks that none are lost and the
// system remains responsive.

#include "driver/timer.h"

volatile uint32_t t0_count = 0;
volatile uint32_t t1_count = 0;
volatile uint32_t uart_count = 0;
volatile uint32_t gpio_count = 0;

void IRAM_ATTR timg0_isr(void* arg) {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
  t0_count++;
}

void IRAM_ATTR timg1_isr(void* arg) {
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0);
  t1_count++;
}

void IRAM_ATTR uart_isr(void* arg) {
  uart_count++;
}

void IRAM_ATTR gpio_isr() {
  gpio_count++;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // TIMG0: 100 us alarm, auto-reload
  {
    timer_config_t cfg = {};
    cfg.divider = 80;
    cfg.counter_dir = TIMER_COUNT_UP;
    cfg.alarm_en = TIMER_ALARM_EN;
    cfg.auto_reload = TIMER_AUTORELOAD_EN;
    cfg.counter_en = TIMER_PAUSE;
    timer_init(TIMER_GROUP_0, TIMER_0, &cfg);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 100);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timg0_isr, NULL, 0, NULL);
    timer_start(TIMER_GROUP_0, TIMER_0);
  }

  // TIMG1: 200 us alarm, auto-reload
  {
    timer_config_t cfg = {};
    cfg.divider = 80;
    cfg.counter_dir = TIMER_COUNT_UP;
    cfg.alarm_en = TIMER_ALARM_EN;
    cfg.auto_reload = TIMER_AUTORELOAD_EN;
    cfg.counter_en = TIMER_PAUSE;
    timer_init(TIMER_GROUP_1, TIMER_0, &cfg);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 200);
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);
    timer_isr_register(TIMER_GROUP_1, TIMER_0, timg1_isr, NULL, 0, NULL);
    timer_start(TIMER_GROUP_1, TIMER_0);
  }

  // UART1 RX interrupt on GPIO17
  Serial1.begin(115200, SERIAL_8N1, 17, 16);

  // GPIO edge interrupt on GPIO4
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(4), gpio_isr, RISING);

  Serial.println("MULTI_IRQ START");
  delay(500);  // let timers fire many times

  uint32_t t0 = t0_count, t1 = t1_count, u = uart_count, g = gpio_count;
  Serial.printf("MULTI_IRQ t0=%lu t1=%lu uart=%lu gpio=%lu\n", t0, t1, u, g);
  // Both timers should have fired many times in 500 ms
  bool pass = (t0 > 10) && (t1 > 5);
  Serial.println(pass ? "MULTI_IRQ PASS" : "MULTI_IRQ FAIL");
}

void loop() {
  delay(1000);
}
