// Peripherals exercise sketch: runs on the emulator AND real silicon.
//   - serial console prints (UART0 via the USB-Serial-JTAG console)
//   - GPIO blink (GPIO 2) + LEDC-free square wave — the emulator can
//     verify pin state via gpio_output()
//   - analogRead(ADC1 channel 3 / GPIO 4) — the emulator injects a
//     voltage via adc_inject_voltage(0, 3, ...)
//   - millis()/micros() (SYSTIMER) and a second FreeRTOS core-1 task
//     (cross-core IPC + the SMP scheduler)

static bool blink_state = false;

void core1_worker(void*) {
  uint32_t n = 0;
  for (;;) {
    n++;
    Serial.printf("[core1] worker tick %lu (core=%u, millis=%lu)\n", n, xPortGetCoreID(), millis());
    vTaskDelay(pdMS_TO_TICKS(700));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(4, INPUT);            // ADC1_CH3 (injected voltage on the emulator)
  delay(200);
  Serial.println("Hello from ESP32-S3!");
  Serial.printf("chip model=%s rev=%d cores=%d freq=%lu\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                ESP.getCpuFreqMHz());
  xTaskCreatePinnedToCore(core1_worker, "core1_worker", 4096, NULL, 1, NULL, 1);
  Serial.println("boot OK");
}

void loop() {
  static uint32_t i = 0;
  i++;
  blink_state = !blink_state;
  digitalWrite(2, blink_state);
  int adc = analogRead(4);
  Serial.printf("[main] loop %lu gpio2=%d adc4=%d millis=%lu\n",
                i, digitalRead(2), adc, millis());
  vTaskDelay(pdMS_TO_TICKS(500));
}