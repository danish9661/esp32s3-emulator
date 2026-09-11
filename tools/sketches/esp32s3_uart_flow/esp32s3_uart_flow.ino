// UART hardware flow-control validation sketch (direct register pokes on
// UART1, so the USB-Serial-JTAG console stays clean).
//
// TX path: CONF0 TX_FLOW_EN (bit 15) gates the transmitter on the U1CTS
// matrix input (signal 16, uart_reg.h). GPIO2 drives CTS: high holds TX
// bytes in the FIFO (STATUS TXFIFO_CNT live, TXFIFO_EMPTY low), low
// flushes them. RX path: CONF0 RX_FLOW_EN (bit 22) drives U1RTS (signal
// 16) from the RX level vs MEM_CONF RX_FLOW_THRHD[16:7]; GPIO3 observes
// RTS (ready-low while RX empty, stop-high at threshold).
#include <Arduino.h>

#define UART1_BASE 0x60010000u
#define U1(x) ((volatile uint32_t*)(UART1_BASE + (x)))
#define GPIO_BASE 0x60004000u
#define G(x) ((volatile uint32_t*)(GPIO_BASE + (x)))

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 5); // UART1
  delay(50);

  // GPIO2 output (CTS driver), GPIO3 output-enabled for RTS readback
  // (peripheral-driven pins read back the driven signal via GPIO_IN).
  *G(0x24) = (1u << 2) | (1u << 3);  // ENABLE_W1TS pins 2,3
  *G(0x08) = (1u << 2);              // OUT_W1TS pin 2 -> CTS high (stop)
  *G(0x554 + 3 * 4) = 16;            // FUNC_OUT_SEL[3] = U1RTS
  *G(0x154 + 16 * 4) = 2;            // FUNC_IN_SEL[16] = GPIO2 (U1CTS)

  *U1(0x20) = (1u << 15);  // CONF0 TX_FLOW_EN
  *U1(0x24) = 96 | (1u << 10);  // CONF1: EMPTY threshold 1 (level check)
  *U1(0x10) = 0xFFFF;      // INT_CLR: drop the reset EMPTY/DONE latches
  *U1(0x00) = 0xA5;        // TX FIFO (held: CTS high)
  delay(50);
  uint32_t st = *U1(0x1C);
  uint32_t cnt = (st >> 16) & 0x3FF;
  uint32_t empty = *U1(0x04) & (1u << 1);
  Serial.printf("UART FLOW held cnt=%lu empty=%lu\n", (unsigned long)cnt, (unsigned long)(empty != 0));
  bool hold_ok = cnt == 1 && empty == 0;

  *G(0x0C) = (1u << 2);  // OUT_W1TC pin 2 -> CTS low (go)
  delay(50);
  uint32_t raw = *U1(0x04);
  bool flush_ok = (raw & (1u << 14)) != 0;  // TX_DONE latched on flush
  Serial.printf("UART FLOW flushed done=%d\n", (int)flush_ok);

  // RX flow: threshold 1, RTS ready-low while RX empty...
  *U1(0x20) = (1u << 15) | (1u << 22);  // TX+RX flow
  *U1(0x60) |= (1u << 7);               // MEM_CONF RX_FLOW_THRHD=1
  delay(20);
  uint32_t rts_ready = digitalRead(3);
  Serial.printf("UART FLOW rts_ready=%d\n", (int)rts_ready);
  // ...stop-high once RX reaches threshold: fill RX via the RS485 echo
  // path (CTS is low = go, so the echo byte also transmits).
  *U1(0x4C) = (1u << 0) | (1u << 3);  // rs485_en + tx_rx_en
  *U1(0x00) = 0x5A;                   // echoes into RX (level 1)
  delay(20);
  uint32_t rts_stop = digitalRead(3);
  Serial.printf("UART FLOW rts_stop=%d\n", (int)rts_stop);
  bool rts_ok = rts_ready == 0 && rts_stop == 1;

  Serial.println(hold_ok && flush_ok && rts_ok ? "UART FLOW PASS" : "UART FLOW FAIL");
}

void loop() {}
