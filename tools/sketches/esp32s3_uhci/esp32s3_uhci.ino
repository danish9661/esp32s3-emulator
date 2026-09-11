// ESP32-S3 UHCI UART-DMA validation for the emulator.
// UHCI0 (0x60014000) bridges UART1 to GDMA (peri_sel 2) as a framing-off
// passthrough pipe. We program GDMA descriptor chains by hand (volatile
// structs in DRAM, like the GDMA direct-poke sketch): an OUT link moves a
// DRAM message into UART1's TX FIFO (captured by the harness as console
// text), and an IN link moves harness-injected UART1 RX bytes back into a
// DRAM buffer which we print. UHCI INT_ST must show TX_START/RX_START.
// Markers: "UHCI TX ..." (message text), "UHCI RX ...", "UHCI INT OK",
// "UHCI PASS". The RX leg needs UART_INJECT=<text> (the harness pushes it
// into UART1 RX once RXREADY prints, like the echo sketch).

#define UHCI_BASE 0x60014000
#define GDMA_BASE 0x6003F000

volatile uint32_t* uhci_conf0 = (volatile uint32_t*)(UHCI_BASE + 0x00);
volatile uint32_t* uhci_int_raw = (volatile uint32_t*)(UHCI_BASE + 0x04);
volatile uint32_t* uhci_int_st = (volatile uint32_t*)(UHCI_BASE + 0x08);
volatile uint32_t* uhci_int_ena = (volatile uint32_t*)(UHCI_BASE + 0x0C);
volatile uint32_t* uhci_int_clr = (volatile uint32_t*)(UHCI_BASE + 0x10);

struct DmaDesc {
  volatile uint32_t dw0;
  volatile uint32_t buf;
  volatile uint32_t next;
  volatile uint32_t rsvd;
};

static DmaDesc tx_desc __attribute__((aligned(4)));
static DmaDesc rx_desc __attribute__((aligned(4)));
static char tx_msg[16] = "UHCI-DMA-TX-42";
static char rx_buf[16];

static void gdma_out(uint32_t ch, uint32_t peri, DmaDesc *d, void *buf, uint32_t len) {
  d->buf = (uint32_t)buf;
  d->next = 0;
  d->dw0 = (1u << 31) | (1u << 30) | ((len & 0xFFFu) << 12);  // owner+eof+len
  uint32_t base = GDMA_BASE + ch * 0xC0;
  *(volatile uint32_t*)(base + 0xA8) = peri;  // out peri_sel
  *(volatile uint32_t*)(base + 0x80) = ((uint32_t)d & 0xFFFFFu) | (1u << 21);  // start
}

static void gdma_in(uint32_t ch, uint32_t peri, DmaDesc *d, void *buf, uint32_t len) {
  d->buf = (uint32_t)buf;
  d->next = 0;
  d->dw0 = (1u << 31) | (1u << 30) | ((len & 0xFFFu) << 12);
  uint32_t base = GDMA_BASE + ch * 0xC0;
  *(volatile uint32_t*)(base + 0x48) = peri;  // in peri_sel
  *(volatile uint32_t*)(base + 0x20) = ((uint32_t)d & 0xFFFFFu) | (1u << 22);  // start
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 6);
  *(volatile uint32_t*)(0x600C0018) |= (1u << 8);
  delay(200);

  // UHCI: clock on, UART1 selected; enable both start interrupts.
  *uhci_conf0 = (1u << 11) | (1u << 3);
  *uhci_int_ena = (1u << 1) | (1u << 0);

  // TX leg: DRAM message -> UART1 TX (harness captures it as text).
  gdma_out(0, 2, &tx_desc, tx_msg, 14);
  Serial.println("UHCI TX SENT");

  // RX leg: ask the harness for bytes, then drain them to DRAM.
  Serial.println("UHCI RXREADY");
  delay(300);
  for (int i = 0; i < 16; i++) rx_buf[i] = 0;
  gdma_in(1, 2, &rx_desc, rx_buf, 16);
  Serial.print("UHCI RX [");
  Serial.print(rx_buf);
  Serial.println("]");
  bool ok = true;
  const char *want = "UHCI-DMA-RX";
  for (int i = 0; i < 11; i++) {
    if (rx_buf[i] != want[i]) ok = false;
  }

  uint32_t st = *uhci_int_st;
  Serial.print("UHCI INT ST=");
  Serial.println(st, HEX);
  if ((st & 3u) == 3u) {
    Serial.println("UHCI INT OK");
  } else {
    Serial.println("UHCI INT MISSING");
    ok = false;
  }
  *uhci_int_clr = 3u;

  if (ok) {
    Serial.println("UHCI PASS");
  } else {
    Serial.println("UHCI FAIL");
  }
  Serial.println("UHCI DONE");
}

void loop() {
  delay(1000);
}
