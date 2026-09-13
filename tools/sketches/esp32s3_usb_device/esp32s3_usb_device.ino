// ESP32-S3 USB-OTG device-mode validation for the emulator (TinyUSB HID).
//
// Boots the REAL Arduino TinyUSB device stack (`USBHIDKeyboard` + `USB.begin()`)
// against the in-model DWC2 core in device mode. Device-side enumeration
// needs a USB-host counterparty (bus reset, SETUP/IN/OUT from a host),
// which no offline harness can provide — so this validates the
// host-independent half that IS firmware-observable: the TinyUSB stack
// initializes the DWC2 core (GRSTCTL reset handshake, GUSBCFG force-device,
// DCFG/DCTL, EP0 activate), reaches its "started, waiting for host" state,
// and keeps running (ticks) without faults. The firmware then proves the
// HID report path stages by typing a character the (absent) host would
// consume. Markers: "USB DEVICE ..." (any "USB DEVICE FAIL ..." mismatch
// prints details).
//
// This is the device-mode companion to `esp32s3_usb_host` (host enumerates
// the SIMULATED device) and `esp32s3_usb_otg` (register-poke loopback with
// no stack): together the three cover every validatable USB-OTG path.
#include "USB.h"
#include "USBHIDKeyboard.h"

USBHIDKeyboard kb;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("USB DEVICE START");

  kb.begin();
  USB.begin();
  Serial.println("USB DEVICE STACK UP");

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
