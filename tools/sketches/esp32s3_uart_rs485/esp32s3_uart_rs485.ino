// UART RS485-mode validation sketch (direct register pokes on UART1, so
// the USB-Serial-JTAG console stays clean).
//
// With RS485_CONF rs485_en + rs485tx_rx_en (TRM uart_struct.h bits 0 and
// 3) a TX byte echoes into the RX FIFO (half-duplex loopback); with only
// rs485_en the receiver is muted during TX and nothing echoes.
#include <Arduino.h>

#define UART1_BASE 0x60010000u
#define U1(x) ((volatile uint32_t*)(UART1_BASE + (x)))

void setup() {
  Serial.begin(115200);

  *U1(0x4C) = (1u << 0) | (1u << 3);  // rs485_en + rs485tx_rx_en
  *U1(0x00) = 0xA5;                   // TX FIFO
  uint32_t echo = *U1(0x00);          // RX FIFO
  Serial.printf("UART RS485 echo=%02lx\n", (unsigned long)echo);
  bool echo_ok = echo == 0xA5;

  *U1(0x4C) = (1u << 0);  // rs485_en only: muted during TX
  *U1(0x00) = 0x5A;
  uint32_t mute = *U1(0x00);
  Serial.printf("UART RS485 mute=%02lx\n", (unsigned long)mute);
  bool mute_ok = mute == 0;

  Serial.println(echo_ok && mute_ok ? "UART RS485 PASS" : "UART RS485 FAIL");
}

void loop() {}
