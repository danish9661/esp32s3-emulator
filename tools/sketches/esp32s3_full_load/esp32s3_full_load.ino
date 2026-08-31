// Multi-peripheral stress test: GPIO + UART + ADC + SYSTIMER + dual-core.
volatile uint32_t core1_ticks = 0;

void core1_worker(void*) {
  uint32_t n = 0;
  for (;;) {
    n++;
    core1_ticks++;
    digitalWrite(2, digitalRead(4));
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(4, INPUT);
  delay(200);
  Serial.println("FULL_LOAD START");
  Serial.printf("FULL_LOAD chip=%s cores=%d\n", ESP.getChipModel(), ESP.getChipCores());
  xTaskCreatePinnedToCore(core1_worker, "c1w", 4096, NULL, 1, NULL, 1);
  Serial.println("FULL_LOAD boot OK");
}

void loop() {
  static uint32_t n = 0;
  n++;
  digitalWrite(2, n & 1);
  int adc = analogRead(4);
  Serial.printf("FULL_LOAD loop %lu adc4=%d gpio4=%d c1=%lu ms=%lu\n",
                n, adc, digitalRead(4), core1_ticks, millis());
  if (n >= 6) {
    Serial.printf("FULL_LOAD DONE c1=%lu\n", core1_ticks);
    Serial.println("FULL_LOAD PASS");
    for (;;) { delay(1000); }
  }
  delay(500);
}
