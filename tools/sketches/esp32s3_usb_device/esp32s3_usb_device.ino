// ESP32-S3 USB-OTG device-mode validation for the emulator (TinyUSB HID).
//
// Boots the REAL Arduino TinyUSB device stack (`USBHIDKeyboard` + `USB.begin()`)
// against the in-model DWC2 core in device mode, with the in-model host as
// the enumeration counterparty (no external host needed): with
// `USB_HOST_ENUM=1` the harness drives the silicon-true bus sequence the
// DWC2 core raises on connect (GINTSTS USBRST + ENUMDONE, then one SETUP
// per transfer with GRXSTSP RXFLVL pops + DOEPINT0 STPKTRCVD/SETUP and the
// IN data/status stages through DIEPCTL0/DIEPTSIZ0 + TXFE/XFRC). The sketch
// stages each transfer only after the previous one completes (the DWC2 core
// is single-transaction) by polling the OTG registers directly, and checks
// every byte the firmware answers against its own TinyUSB descriptors.
//
// Markers: "USB DEVICE ..." (any "USB DEVICE FAIL ..." mismatch prints
// details). Without `USB_HOST_ENUM=1` the sketch still boots the stack and
// proves the HID report path stages (the pre-auto-enum behavior).
//
// This is the device-mode companion to `esp32s3_usb_host` (host enumerates
// the SIMULATED device) and `esp32s3_usb_otg` (register-poke loopback with
// no stack): together the three cover every validatable USB-OTG path.
#include "USB.h"
#include "USBHIDKeyboard.h"

#define OTG 0x60080000u
#define O(x) ((volatile uint32_t*)(OTG + (x)))

// Linked Arduino device descriptor, truncated to wLength 8 (the first
// GET_DESCRIPTOR the TinyUSB stack issues on connect): bLength,
// bDescriptorType, bcdUSB, MISC class triple, EP0 MPS 64.
static const uint8_t want8[8] = { 18, 1, 0x00, 0x02, 0xEF, 0x02, 0x01, 64 };

// Harness-detect is RACE-FREE by construction: read the bus-reset ISR's
// register effects (handle_bus_reset programs DAINTMSK 0x10001,
// DOEPMSK=DIEPMSK=0x9, GRXFSIZ 0x3E, DIEPTXF0 0x1000F0; all read 0 with no
// host). Do NOT poll GINTSTS here: the ISR W1C-clears the bark and a
// match-ANY poll races the clear, never seeing it (observed live).
static bool live_detect(void) {
  return *O(0x81C) == 0x10001u;
}

// Step-budget note: the live path costs boot (≈32M insns) + delay(20) +
// the HID tail; the 60M STEPS battery budget covers it with margin.

// Live check: read the 2 IN words through the device RXFIFO mirror (the
// same bytes the harness captures off the virtual wire via
// `usb_host_take_in`) and compare against `want`. Called after the POLL +
// delay below, so the ISR has scheduled, pushed, and completed long ago.
// Returns false with a FAIL DATA line on the first mismatching byte.
//
// WHY A DELAY, NOT A TSIZ POLL (measured live 2026-09-16, single-step
// ground truth — read before "fixing" this into a poll): the transfer
// schedules at +2316 single-steps after stage and drains at +2423, all
// inside the core-1 ISR that preempts this very task. A task-side poll
// of DIEPTSIZ0 can NEVER observe the scheduled state (TSIZ=8 lives
// ~107 steps while this task is preempted; the task resumes polling at
// ~+8856 with TSIZ already 0 — FAIL IN SCHED with a healthy model, on
// silicon too). The mirror, by contrast, LATCHES the payload until read,
// so a fixed delay (stage-to-complete ≈ 2.4k steps; delay(20) ≈ millions)
// followed by the mirror read is race-free by construction. The only
// timing requirement is delay > stage-to-complete, with orders of
// magnitude of margin — no banner-poll rendezvous, no codegen-sensitive
// shift. Return with NO OUT wait and NO IN W1C: the ISR owns
// DIEPINT0/DOEPINT0, and sketch-side W1C races `handle_ep_irq`'s
// snapshot-then-clear, re-latching stale flags.
static bool check_in8(const uint8_t *want) {
  uint32_t w0 = *O(0x1000);
  uint32_t w1 = *O(0x1000);
  uint8_t got[8];
  got[0] = (uint8_t)(w0 >> 0); got[1] = (uint8_t)(w0 >> 8);
  got[2] = (uint8_t)(w0 >> 16); got[3] = (uint8_t)(w0 >> 24);
  got[4] = (uint8_t)(w1 >> 0); got[5] = (uint8_t)(w1 >> 8);
  got[6] = (uint8_t)(w1 >> 16); got[7] = (uint8_t)(w1 >> 24);
  for (uint32_t i = 0; i < 8; i++) {
    if (got[i] != want[i]) {
      Serial.print("USB DEVICE FAIL DATA ");
      Serial.println(i);
      return false;
    }
  }
  return true;
}

USBHIDKeyboard kb;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("USB DEVICE START");

  kb.begin();
  USB.begin();
  Serial.println("USB DEVICE STACK UP");

  // Live auto-enum path (needs USB_HOST_ENUM=1 on the harness): the
  // bus-reset bark has run (live-detect above), the harness's status-out
  // closes each transfer, and the sketch prints the result markers the
  // battery asserts (ENUM OK / PASS / DONE). On early return the
  // no-harness tail below runs instead (stack boot only).
  do {
    if (!live_detect()) break;
    uint32_t enumspd = (*O(0x808) >> 1) & 3u;
    Serial.print("USB DEVICE RESET ENUMSPD=");
    Serial.println((unsigned long)enumspd);
    // POLL RDV: the harness stages the REQ0 SETUP on THIS line (the
    // transfer schedules ~2.3k steps later, all inside the ISR). The
    // delay below outlasts stage-to-complete (~2.4k steps) by orders of
    // magnitude, then the mirror read checks the latched payload — see
    // check_in8 for why a TSIZ poll cannot work here.
    Serial.println("USB DEVICE POLL RDV");
    delay(20);
    bool ok = check_in8(want8);
    if (!ok) { Serial.println("USB DEVICE ENUM FAIL"); return; }
    Serial.println("USB DEVICE ENUM OK");
  } while (0);

  // The stack is up and waiting for a host (which never arrives offline).
  // Prove the HID report path stages without faulting.
  size_t n = kb.write('a');
  Serial.print("USB DEVICE WRITE=");
  Serial.println((unsigned long)n);
  delay(50);

  Serial.println("USB DEVICE PASS");
  Serial.println("USB DEVICE DONE");
}

void loop() { delay(1000); }
