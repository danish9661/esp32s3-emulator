// Direct register-poke validation for the emulator's LP_UART model.
// The LP_UART block lives at 0x6002_5400 (in the GPSPI3 page, offset 0x400).
// We write a few registers and read them back; the emulator models it as a
// register store, so the values must round-trip (P5 "poke" pattern).

#define LP_UART_BASE 0x60025400
#define LP_UART_FIFO_REG  (LP_UART_BASE + 0x00)
#define LP_UART_CLKDIV_REG (LP_UART_BASE + 0x14)
#define LP_UART_CONF0_REG (LP_UART_BASE + 0x20)

void setup() {
  Serial.begin(115200);
  delay(50);

  REG_WRITE(LP_UART_FIFO_REG, 0x000000AB);
  REG_WRITE(LP_UART_CLKDIV_REG, 0x00AA00BB);
  REG_WRITE(LP_UART_CONF0_REG, 0x12345678);

  uint32_t fifo = REG_READ(LP_UART_FIFO_REG);
  uint32_t clkdiv = REG_READ(LP_UART_CLKDIV_REG);
  uint32_t conf0 = REG_READ(LP_UART_CONF0_REG);

  if (fifo == 0xAB && clkdiv == 0x00AA00BB && conf0 == 0x12345678) {
    Serial.println("LP UART POKE PASS");
  } else {
    Serial.print("LP UART POKE FAIL fifo=");
    Serial.print(fifo, HEX);
    Serial.print(" clkdiv=");
    Serial.print(clkdiv, HEX);
    Serial.print(" conf0=");
    Serial.println(conf0, HEX);
  }
  Serial.println("DONE");
}

void loop() {}
