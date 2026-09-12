// Core-dump WRITE path validation: on the first boot there is no dump
// (read size 0/empty); then abort() crashes with a register/backtrace
// dump into the coredump partition; after the reboot the dump is found
// and its size is nonzero. Uses RTC slow memory to count boots (retained
// across the crash reboot like silicon).
#include <Arduino.h>
#include "esp_core_dump.h"
#include "esp_system.h"

#define RTC_SLOW_MEM ((volatile uint32_t*)0x50000000u)

void setup() {
  Serial.begin(115200);
  delay(200);
  uint32_t boots = RTC_SLOW_MEM[0];
  RTC_SLOW_MEM[0] = boots + 1;
  Serial.printf("COREDUMP boot %lu\n", (unsigned long)boots);

  size_t addr = 0, size = 0;
  esp_err_t e = esp_core_dump_image_get(&addr, &size);
  Serial.printf("COREDUMP get rc=%d addr=%08lx size=%lu\n",
    (int)e, (unsigned long)addr, (unsigned long)size);
  if (boots == 0) {
    Serial.println("COREDUMP CRASHING");
    abort();  // must not return: reboots into boot 1
  }
  bool ok = (e == ESP_OK) && (size > 0);
  Serial.println(ok ? "COREDUMP PASS" : "COREDUMP FAIL");
}

void loop() {}
