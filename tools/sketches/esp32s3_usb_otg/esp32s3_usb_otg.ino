// USB-OTG DWC core validation sketch (direct register pokes).
//
// Exercises the host-independent device-init path: core soft reset
// handshake (GRSTCTL.CSFTRST self-clears, AHBIDLE sets), force-device
// mode (GUSBCFG.FDMOD), device config (DCFG/DCTL soft-disconnect),
// EP0 control programming (DIEPCTL0/DOEPCTL0 activate + MPS), and TXFIFO0
// staging (DFIFO0 pushes, DTXFSTS0 space decreases). No USB traffic can
// exist without a host counterparty, so GINTSTS must read quiet —
// enumeration itself is out of scope (see usb_otg.rs).
#include <Arduino.h>

#define OTG_BASE 0x60080000u
#define O(x) ((volatile uint32_t*)(OTG_BASE + (x)))

static int poll_clear(uint32_t off, uint32_t bit) {
  for (uint32_t t = 0; t < 20000000; t++) {
    if ((*O(off) & bit) == 0) return 1;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  // Enable peripheral clocks (SYSCON gating: OTG engine frozen otherwise).
  *(volatile uint32_t*)(0x600C0018) |= (1u << 23); // USB-OTG

  *O(0x010) = 1u;  // GRSTCTL.CSFTRST
  bool rst_ok = poll_clear(0x010, 1u) && (*O(0x010) & (1u << 31)) != 0;
  Serial.println(rst_ok ? "OTG RESET OK" : "OTG RESET MISMATCH");
  if (!rst_ok) return;

  *O(0x00C) = (1u << 30);  // GUSBCFG.FDMOD (force device)
  *O(0x800) = 3u;          // DCFG full-speed
  *O(0x804) = (1u << 1);   // DCTL.SDIS (soft disconnect)
  bool cfg_ok = (*O(0x00C) >> 30) == 1 && (*O(0x800) & 3u) == 3 &&
                (*O(0x804) & (1u << 1)) != 0;
  *O(0x804) = 0;  // reconnect
  Serial.println(cfg_ok ? "OTG CFG OK" : "OTG CFG MISMATCH");
  if (!cfg_ok) return;

  *O(0x900) = (1u << 15);  // DIEPCTL0.USBACTEP, MPS 64B
  *O(0xB00) = (1u << 15);  // DOEPCTL0.USBACTEP, MPS 64B
  bool ep_ok = (*O(0x900) & (1u << 15)) != 0 && (*O(0xB00) & (1u << 15)) != 0;
  Serial.println(ep_ok ? "OTG EP0 OK" : "OTG EP0 MISMATCH");
  if (!ep_ok) return;

  uint32_t full = *O(0x914);  // DTXFSTS0 space
  // NOTE: DFIFO0 writes in device mode stage the EP0 loopback SETUP packet
  // (8 bytes = one SETUP), not the TXFIFO — TXFIFO staging needs host mode
  // (see the usb_host sketch). So the FIFO-space leg runs in force-host
  // mode here, then drops back to force-device for the loopback leg.
  *O(0x00C) = (1u << 29);  // GUSBCFG.FHMOD (force host, temporary)
  *O(0x1000) = 0x11111111u;
  *O(0x1000) = 0x22222222u;
  *O(0x1000) = 0x33333333u;
  *O(0x1000) = 0x44444444u;
  uint32_t used = *O(0x914);
  Serial.printf("OTG FIFO full=%lu used=%lu\n", (unsigned long)full, (unsigned long)used);
  bool fifo_ok = full == 256 && used == 252;
  Serial.println(fifo_ok ? "OTG FIFO OK" : "OTG FIFO MISMATCH");
  if (!fifo_ok) return;
  *O(0x010) = 1u;  // GRSTCTL.CSFTRST (flush host staging, back to clean)
  poll_clear(0x010, 1u);
  *O(0x00C) = (1u << 30);  // GUSBCFG.FDMOD (force device, for loopback)

  // Device-mode EP0 loopback (no external host): stage a GET_DESCRIPTOR
  // (DEVICE, 18 B) SETUP packet via DFIFO0, expect the EP0 OUT XFRC, pop
  // the 18 descriptor bytes, complete status, expect the EP0 IN XFRC.
  *O(0x1000) = 0x01000680u;  // [80 06 00 01]
  *O(0x1000) = 0x00120000u;  // [00 00 12 00]
  bool setup_ok = (*O(0xB08) & 1u) != 0;  // DOEPINT0.XFRC
  Serial.println(setup_ok ? "OTG SETUP OK" : "OTG SETUP MISMATCH");
  if (!setup_ok) return;
  *O(0xB08) = 1u;  // W1C
  uint8_t desc[18];
  for (int i = 0; i < 5; i++) {
    uint32_t w = *O(0x1000);
    for (int k = 0; k < 4 && 4 * i + k < 18; k++) {
      desc[4 * i + k] = (uint8_t)(w >> (8 * k));
    }
  }
  bool desc_ok = desc[0] == 18 && desc[1] == 1 && desc[8] == 0x3A && desc[9] == 0x30;
  Serial.println(desc_ok ? "OTG DESC OK" : "OTG DESC MISMATCH");
  if (!desc_ok) return;
  *O(0x910) = 0;  // DIEPTSIZ0 (status stage)
  bool status_ok = (*O(0x908) & 1u) != 0;  // DIEPINT0.XFRC
  Serial.println(status_ok ? "OTG STATUS OK" : "OTG STATUS MISMATCH");
  if (!status_ok) return;

  bool quiet = *O(0x014) == 0;  // GINTSTS: no host, no events
  Serial.println(quiet ? "OTG PASS" : "OTG FAIL");
}

void loop() {}
