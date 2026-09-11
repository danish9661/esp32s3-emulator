// GDMA memory-to-memory validation sketch (direct register pokes).
//
// Programs GDMA ch0 with IN_CONF0 mem_trans_en (TRM gdma_struct.h bit 4):
// the OUT-link descriptor sources 8 DRAM pattern bytes the IN-link
// descriptor sinks into a DRAM buffer on the same channel pair. Both links
// are started (OUT first: a lone start only arms); completion raises OUT +
// IN done. The firmware verifies the copy byte-exact.
#include <Arduino.h>

#define GDMA_BASE 0x6003F000u
#define D(x) ((volatile uint32_t*)(GDMA_BASE + (x)))

struct Desc {
  uint32_t dw0, buf, next, rsvd;
};

static uint8_t g_src[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
static uint8_t g_dst[8];
static Desc g_odesc, g_idesc;

static int poll(uint32_t off, uint32_t bit) {
  for (uint32_t t = 0; t < 20000000; t++) {
    if (*D(off) & bit) return 1;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: frozen otherwise).
  *(volatile uint32_t*)(0x600C001C) |= (1u << 6);
  for (int i = 0; i < 8; i++) g_dst[i] = 0;

  g_odesc.dw0 = (8) | (8 << 12) | (1u << 30) | (1u << 31);
  g_odesc.buf = (uint32_t)g_src;
  g_odesc.next = 0;
  g_odesc.rsvd = 0;
  g_idesc.dw0 = (8) | (8 << 12) | (1u << 30) | (1u << 31);
  g_idesc.buf = (uint32_t)g_dst;
  g_idesc.next = 0;
  g_idesc.rsvd = 0;

  *D(0x00) = (1u << 4);  // in_conf0[0]: mem_trans_en
  uint32_t olsb = ((uint32_t)&g_odesc) & 0x000FFFFFu;
  *D(0x60 + 0x20) = olsb | (1u << 21);  // OUT start (arms: IN not started)
  uint32_t ilsb = ((uint32_t)&g_idesc) & 0x000FFFFFu;
  *D(0x20) = ilsb | (1u << 22);  // IN start: both started -> copy runs

  if (!poll(0x60 + 0x08, 1u)) {
    Serial.println("GDMA M2M NO OUT DONE");
    return;
  }
  if (!poll(0x08, 1u)) {
    Serial.println("GDMA M2M NO IN DONE");
    return;
  }
  bool ok = true;
  for (int i = 0; i < 8; i++) {
    if (g_dst[i] != g_src[i]) ok = false;
  }
  Serial.printf("GDMA M2M dst=%02x%02x%02x%02x%02x%02x%02x%02x\n",
                g_dst[0], g_dst[1], g_dst[2], g_dst[3],
                g_dst[4], g_dst[5], g_dst[6], g_dst[7]);
  Serial.println(ok ? "GDMA M2M PASS" : "GDMA M2M FAIL");
}

void loop() {}
