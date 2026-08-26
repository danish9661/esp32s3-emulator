// Direct register-poke validation for the emulator's USB-Serial-JTAG model.
// The USB-Serial-JTAG block lives at 0x6003_8000. We drive the CDC TX data
// path directly: write bytes to EP1 (rdwr_byte, 0x00), then assert wr_done
// (EP1_CONF bit 0) which the model latches as serial_in_empty_int. The bytes
// are captured by the emulator and merged into the console stream, so
// "USBCDC:OK" appears in the run_flash output. The RX path is validated by the
// unit tests (host-injected bytes are popped from EP1).

#define USB_BASE 0x60038000
#define USB_EP1_REG      (USB_BASE + 0x00)
#define USB_EP1_CONF_REG (USB_BASE + 0x04)

static void usb_write(const char *s) {
  volatile uint32_t *ep1 = (volatile uint32_t *)USB_EP1_REG;
  volatile uint32_t *conf = (volatile uint32_t *)USB_EP1_CONF_REG;
  for (int i = 0; s[i] != 0; i++) {
    *ep1 = (uint32_t)(unsigned char)s[i];
  }
  *conf = 1;  // wr_done
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("USB TEST START");
  usb_write("USBCDC:OK");
  Serial.println("USB TEST END");
}

void loop() {}
