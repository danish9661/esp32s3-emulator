// UART1 RX echo exercise: the emulator injects bytes into UART1 RX once
// the console prints the RXREADY marker (run_flash UART_INJECT env var);
// the sketch echoes every received byte back on UART1 TX and prints a
// per-byte line on the console (USB-Serial-JTAG).
//
// NOTE (verified in the emulator): this toolchain's Arduino core 3.3.10
// `digitalPinToGPIONumber()` mapping collapses every pin onto UART0's
// GPIOs — odd Arduino pins -> 44, even -> 43 (observed: Serial1.begin with
// 5/6, 15/16 or 18/17 ALL land UART1 on 44/43). So UART1 collides with
// UART0's 43/44 and the core's periman logic terminates Serial0
// (p_uart_obj[0] freed by hardware_serial_end). The emulator faithfully
// reproduces this real-hardware behavior; it is NOT an emulator bug.
// To demo UART1 echo end-to-end you must build firmware whose UART1 pins
// differ from 43/44 (e.g. a board variant with a sane digitalPinToGPIONumber).

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 18, 17);
  delay(200);
  Serial.println("Hello from ESP32-S3!");
  Serial.println("UART1 RX echo ready RXREADY");
}

void loop() {
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();
    Serial1.write(b);
    Serial.printf("[uart1] rx '%c' (0x%02X)\n", b >= 32 && b < 127 ? b : '.', b);
  }
  vTaskDelay(pdMS_TO_TICKS(20));
}