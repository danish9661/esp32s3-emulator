// Deep-sleep ULP wakeup validation (ESP32-S3 emulator).
//
// First boot (cause UNDEFINED): hand-load a tiny rv32im ULP program into
// RTC_SLOW_MEM that spins ~1M instructions (longer than the sleep-prep
// gap), stores a magic to ULP reg slot 0, then halts; release the ULP,
// arm ULP wakeup via the real esp-idf driver, and enter deep sleep (no
// timer armed). The emulator must keep ticking the ULP during the sleep,
// wake when it halts with the ULP cause bit, and print WOKE/PASS.
//
// Expected run_flash output: "DEEPSLEEP ULP START" then (after reboot)
// "DEEPSLEEP ULP WOKE" and "DEEPSLEEP ULP PASS".

#include "esp_sleep.h"

#define RTC_SLOW_MEM 0x50000000
#define ULP_BASE     0x60008100
#define ULP_REG0     (ULP_BASE + 0x0C)

volatile uint32_t *prog = (volatile uint32_t *)RTC_SLOW_MEM;
volatile uint32_t *ulp_core = (volatile uint32_t *)ULP_BASE;

// rv32im program (LE words, assembler-verified with riscv32-esp-elf-as
// -march=rv32im — do NOT hand-edit encodings; a past hand-encoded bnez
// missed its target and the ULP never halted):
//   nested 200x2000 delay (~800k ULP instrs, outlasts sleep prep),
//   store 0x1ECA01E9 to reg slot 0, ebreak.
static const uint32_t prog_words[12] = {
  0x0C800193, 0x7D000213, 0xFFF20213, 0xFE021EE3, 0xFFF18193,
  0xFE0198E3, 0x600080B7, 0x10C08093, 0x1ECA0137, 0x1E910113,
  0x0020A023, 0x00100073
};

void setup() {
  Serial.begin(115200);
  delay(50);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_ULP) {
    Serial.println("DEEPSLEEP ULP WOKE");
    uint32_t v = *(volatile uint32_t *)ULP_REG0;
    Serial.printf("DEEPSLEEP ULP REG0=%08X\n", v);
    if (v == 0x1ECA01E9) {
      Serial.println("DEEPSLEEP ULP PASS");
    } else {
      Serial.println("DEEPSLEEP ULP REG MISMATCH");
    }
    while (1) {
      delay(10);
    }
  }

  Serial.println("DEEPSLEEP ULP START");
  esp_err_t werr = esp_sleep_enable_ulp_wakeup();
  Serial.printf("DEEPSLEEP ULP ENABLE=%d\n", (int)werr);
  for (int i = 0; i < 12; i++) {
    prog[i] = prog_words[i];
  }
  *ulp_core = 1; // release the ULP last: it must still run at sleep entry
  esp_deep_sleep_start();
  // Not reached.
  Serial.println("DEEPSLEEP ULP SHOULD NOT REACH");
}

void loop() {}
