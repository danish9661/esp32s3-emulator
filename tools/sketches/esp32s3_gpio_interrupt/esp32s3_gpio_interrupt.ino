// GPIO edge-interrupt validation, fully internal (no jumper wire):
// RMT channel 0 transmits a pulse train onto GPIO2 (matrix signal 81)
// while a RISING interrupt is attached on GPIO2 through the real
// Arduino/IDF GPIO driver (`attachInterrupt` -> int_type/int_ena,
// shared ISR, STATUS clear). The pad input path always samples the driven
// level — on silicon and in the emulator — so each rising edge must fire
// the ISR. Long pulses (>>32 duration units) so every edge is sampled.

#include <Arduino.h>

#define RMT_BASE 0x60016000UL
#define RMTMEM_BASE 0x60016800UL
#define GPIO_BASE 0x60004000UL

volatile uint32_t* const RMT = (volatile uint32_t* const)RMT_BASE;
volatile uint32_t* const RMTMEM = (volatile uint32_t* const)RMTMEM_BASE;
volatile uint32_t* const GPIO = (volatile uint32_t* const)GPIO_BASE;

volatile uint32_t irq_count = 0;

void IRAM_ATTR gpio_isr_handler() {
  irq_count++;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(2, OUTPUT);
  GPIO[0x55C / 4] = 81;   // FUNC_OUT_SEL_CFG[2] = RMT TX channel 0
  GPIO[0x24 / 4] = 1u << 2; // GPIO_ENABLE_W1TS bit 2

  attachInterrupt(digitalPinToInterrupt(2), gpio_isr_handler, RISING);
  delay(10);

  // One pulse per transmission, two transmissions separated by a long
  // delay: the shared Arduino ISR is slow, so back-to-back edges would
  // merge into a single latched status (correctly, as on silicon).
  RMTMEM[0] = (500u) | (1u << 15) | (500u << 16);
  RMTMEM[1] = 0;
  for (int k = 0; k < 2; k++) {
    RMT[0x20 / 4] = (0 << 0) | (1 << 6); // clear tx_start first ...
    RMT[0x20 / 4] = (1 << 0) | (1 << 6); // ... then chnconf0: tx_start | idle_out_en
    delay(300);
  }

  uint32_t count = irq_count;
  Serial.printf("GPIO IRQ count=%lu\n", (unsigned long)count);
  Serial.println(count >= 2 ? "GPIO_IRQ PASS" : "GPIO_IRQ FAIL");
}

void loop() {
  delay(1000);
}
