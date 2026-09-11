// Dedicated-GPIO validation for the ESP32-S3 emulator (real esp-idf
// `driver/dedic_gpio` path, no pokes).
//
// A single bundle on GPIO2 (out+in) loopbacks through the pad: bundle_write
// drives the pin (CPU TIE latch -> CORE1_GPIO_OUT matrix -> pad), bundle
// read_in/read_out observe it (pad -> CORE1_GPIO_IN matrix -> ee.get_gpio_in,
// latch via rur.gpio_out). digitalRead cross-checks the pad level. A bundle
// interrupt callback counts edges while the output toggles.

#include <Arduino.h>
#include "driver/dedic_gpio.h"

static const int PIN = 2;
static dedic_gpio_bundle_handle_t bundle = NULL;

void setup() {
  Serial.begin(115200);
  const int pins[1] = {PIN};
  dedic_gpio_bundle_config_t cfg = {};
  cfg.gpio_array = pins;
  cfg.array_size = 1;
  cfg.flags.out_en = 1;
  cfg.flags.in_en = 1;
  if (dedic_gpio_new_bundle(&cfg, &bundle) != ESP_OK || bundle == NULL) {
    Serial.println("DEDIC NEW FAIL");
    return;
  }
  uint32_t mask = 0, offset = 0;
  dedic_gpio_get_out_mask(bundle, &mask);
  dedic_gpio_get_out_offset(bundle, &offset);
  Serial.printf("DEDIC mask=%x offset=%u\n", mask, offset);

  dedic_gpio_bundle_write(bundle, mask, mask);
  delay(5);
  int pad_hi = digitalRead(PIN);
  uint32_t in_hi = dedic_gpio_bundle_read_in(bundle);
  uint32_t out_hi = dedic_gpio_bundle_read_out(bundle);
  Serial.printf("DEDIC hi pad=%d in=%x out=%x\n", pad_hi, in_hi, out_hi);

  dedic_gpio_bundle_write(bundle, mask, 0);
  delay(5);
  int pad_lo = digitalRead(PIN);
  uint32_t in_lo = dedic_gpio_bundle_read_in(bundle);
  Serial.printf("DEDIC lo pad=%d in=%x\n", pad_lo, in_lo);

  // Poll the input channel while toggling the output (S3 dedicated GPIO
  // has no bundle interrupts; polling read_in is the silicon path).
  int edges = 0, last = -1;
  for (int i = 0; i < 6; i++) {
    dedic_gpio_bundle_write(bundle, mask, (i & 1) ? 0 : mask);
    delay(5);
    int v = dedic_gpio_bundle_read_in(bundle) != 0;
    if (last >= 0 && v != last) edges++;
    last = v;
  }
  Serial.printf("DEDIC edges=%d\n", edges);

  bool ok = pad_hi == 1 && in_hi != 0 && out_hi != 0 && pad_lo == 0 &&
            in_lo == 0 && edges >= 4;
  Serial.println(ok ? "DEDIC GPIO PASS" : "DEDIC GPIO FAIL");
}

void loop() { delay(1000); }
