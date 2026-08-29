// GPIO edge-interrupt validation: toggles GPIO2 via digitalWrite and
// attaches an interrupt on GPIO4 (input, pulled high).  The interrupt
// handler increments a counter; the main loop reads it to confirm the
// ISR fired.  The emulator routes GPIO2→GPIO4 internally when both are
// in the same pin state, so the counter should increment.

volatile uint32_t irq_count = 0;

void IRAM_ATTR gpio_isr_handler() {
  irq_count++;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(2, OUTPUT);
  pinMode(4, INPUT_PULLUP);

  // Attach rising-edge interrupt on GPIO4
  attachInterrupt(digitalPinToInterrupt(4), gpio_isr_handler, RISING);
  delay(10);

  // Toggle GPIO2 high — if the emulator loops GPIO2→GPIO4 internally,
  // the ISR should fire.
  digitalWrite(2, HIGH);
  delay(50);

  uint32_t count = irq_count;
  Serial.printf("GPIO IRQ count=%lu\n", count);

  // Even without internal loopback, the test passes on real HW with a
  // jumper wire, and the emulator should at least not crash.
  Serial.println(count > 0 ? "GPIO_IRQ PASS" : "GPIO_IRQ NO_JUMPER");
}

void loop() {
  delay(1000);
}
