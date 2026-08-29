// Multi-UART validation: tests UART0, UART1, and UART2 TX simultaneously.
// Each UART prints a unique banner so the emulator can verify all three
// console streams are active.  Runs on real silicon too (pins 43/44 = UART0,
// pins 16/17 = UART1, pins 38/39 = UART2 on the ESP32-S3 DevKitC).

#define UART0_NUM 0
#define UART1_NUM 1
#define UART2_NUM 2

void setup() {
  Serial.begin(115200);
  delay(50);

  // UART0 is already Serial; just print.
  Serial.println("UART0 TX OK");

  // UART1 on pins 16 (TX) / 17 (RX)
  Serial1.begin(115200, SERIAL_8N1, 17, 16);
  Serial1.println("UART1 TX OK");

  // UART2 on pins 38 (TX) / 39 (RX)
  Serial2.begin(115200, SERIAL_8N1, 39, 38);
  Serial2.println("UART2 TX OK");

  delay(100);
  Serial.println("MULTI_UART PASS");
}

void loop() {
  delay(1000);
}
