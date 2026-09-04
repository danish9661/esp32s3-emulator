// ESP32-S3 UART1 RX-timeout validation sketch (direct register pokes).
//
// Programs UART1 (0x60010000) with a high RXFIFO-full threshold (so FULL
// never fires for a 1-byte burst) plus rx_tout_en, then prints RXREADY — the
// HOST (run_flash with UART_INJECT=<byte>) pushes one byte into UART1 RX.
// The byte sits below the FULL threshold, so only the RXFIFO_TOUT interrupt
// (idle line + data pending) can report it.
#include <Arduino.h>

#define UART1_BASE 0x60010000u
#define U1_CONF1 (*(volatile uint32_t*)(UART1_BASE + 0x24))
#define U1_MEM_CONF (*(volatile uint32_t*)(UART1_BASE + 0x60))
#define U1_FIFO (*(volatile uint32_t*)(UART1_BASE + 0x00))
#define U1_INT_RAW (*(volatile uint32_t*)(UART1_BASE + 0x04))
#define U1_INT_CLR (*(volatile uint32_t*)(UART1_BASE + 0x10))

void setup() {
  Serial.begin(115200);
  // Full threshold 120 (a 1-byte burst never reaches it), timeout enable.
  U1_CONF1 = (U1_CONF1 & ~0x3FFu) | 120u | (1u << 23);
  // Timeout = 10 bit-times (explicit; also the reset default).
  U1_MEM_CONF = (U1_MEM_CONF & ~(0x3FFu << 17)) | (10u << 17);
  Serial.println("RXREADY");
  uint32_t fired = 0;
  for (uint32_t t = 0; t < 20000000; t++) {
    if (U1_INT_RAW & (1u << 8)) {
      fired = 1;
      break;
    }
  }
  if (!fired) {
    Serial.println("UART TOUT TIMEOUT");
    return;
  }
  uint32_t b = U1_FIFO & 0xFFu;
  Serial.printf("UART TOUT BYTE=%02x\n", b);
  Serial.println("UART TOUT OK");
}

void loop() {}
