// MCPWM dead-time DRIVER validation (new esp-idf MCPWM driver): timer +
// operator + generator pair with posedge/negedge dead times. Dumps the
// programmed DT0_CFG/FED/RED registers (revealing INSEL/DEB programming)
// and samples both outputs for the dead band.
#include <Arduino.h>
#include "driver/mcpwm_prelude.h"

#define MCPWM0_BASE 0x6001E000u
#define DT0_CFG  (*(volatile uint32_t*)(MCPWM0_BASE + 0x58))
#define DT0_FED  (*(volatile uint32_t*)(MCPWM0_BASE + 0x5C))
#define DT0_RED  (*(volatile uint32_t*)(MCPWM0_BASE + 0x60))

static int measure(int pin, uint32_t n) {
  uint32_t h = 0;
  for (uint32_t i = 0; i < n; i++) if (digitalRead(pin)) h++;
  return (int)(h * 100u / n);
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 17);
  delay(50);

  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_cfg = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = 1000,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

  mcpwm_cmpr_handle_t cmpr = NULL;
  mcpwm_comparator_config_t cmpr_cfg = {};
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_cfg, &cmpr));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, 500));

  mcpwm_gen_handle_t genA = NULL, genB = NULL;
  mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = 2 };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &genA));
  gen_cfg.gen_gpio_num = 3;
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &genB));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(genA,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(genA,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW)));
  // Complementary actions on B (high at compare, low at empty) so the
  // dead time shows as a both-low band between the edges.
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(genB,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(genB,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_HIGH)));

  mcpwm_dead_time_config_t dt = {
    .posedge_delay_ticks = 100,
    .negedge_delay_ticks = 200,
  };
  esp_err_t derr = mcpwm_generator_set_dead_time(genA, genB, &dt);
  Serial.printf("MCPWM DT DRIVER rc=%d\n", (int)derr);
  Serial.printf("MCPWM DT CFG=%08lX FED=%08lX RED=%08lX\n",
    (unsigned long)DT0_CFG, (unsigned long)DT0_FED, (unsigned long)DT0_RED);

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  int a = measure(2, 20000);
  int b = measure(3, 20000);
  Serial.printf("MCPWM DT DRIVER dutyA=%d%% dutyB=%d%%\n", a, b);
  // Complementary pair at 50% with dead time: both near 50, never high
  // together (sampled overlap must be ~0).
  uint32_t both = 0;
  for (uint32_t i = 0; i < 20000; i++) {
    if (digitalRead(2) && digitalRead(3)) both++;
  }
  Serial.printf("MCPWM DT DRIVER overlap=%lu\n", (unsigned long)both);
  // A few coincident samples at the sharp edges are sampling alias
  // (each digitalRead pair spans several emulator steps), not overlap:
  // a real overlap would read in the thousands (cf. 6822 pre-DEB-model).
  bool ok = derr == ESP_OK && a >= 40 && a <= 60 && b >= 40 && b <= 60 && both <= 100;
  Serial.println(ok ? "MCPWM DT DRIVER PASS" : "MCPWM DT DRIVER FAIL");

}

void loop() {}
