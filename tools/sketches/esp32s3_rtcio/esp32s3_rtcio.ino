// ESP32-S3 RTC_IO register-store validation for the emulator.
// Pokes RTC_IO registers directly (base 0x60008400) and asserts the
// write/store + w1ts/w1tc semantics match the model.

#define RTCIO_BASE 0x60008400
volatile uint32_t* out_r      = (volatile uint32_t*)(RTCIO_BASE + 0x00);
volatile uint32_t* out_w1ts   = (volatile uint32_t*)(RTCIO_BASE + 0x04);
volatile uint32_t* out_w1tc   = (volatile uint32_t*)(RTCIO_BASE + 0x08);
volatile uint32_t* enable_r   = (volatile uint32_t*)(RTCIO_BASE + 0x0C);
volatile uint32_t* enable_w1t = (volatile uint32_t*)(RTCIO_BASE + 0x10);
volatile uint32_t* enable_w1c = (volatile uint32_t*)(RTCIO_BASE + 0x14);
volatile uint32_t* pad        = (volatile uint32_t*)(RTCIO_BASE + 0x5BC);

void setup() {
  Serial.begin(115200);
  delay(200);

  *out_r = 0x55;
  if (*out_r != 0x55) { Serial.println("RTCIO FAIL out"); return; }

  *out_w1ts = 0xAA;
  if (*out_r != 0xFF) { Serial.println("RTCIO FAIL w1ts"); return; }

  *out_w1tc = 0x0F;
  if (*out_r != 0xF0) { Serial.println("RTCIO FAIL w1tc"); return; }

  *enable_r = 0x1;
  *enable_w1t = 0x4;
  if (*enable_r != 0x5) { Serial.println("RTCIO FAIL en_w1ts"); return; }

  *enable_w1c = 0x5;
  if (*enable_r != 0x0) { Serial.println("RTCIO FAIL en_w1tc"); return; }

  *pad = 0x12345678;
  if (*pad != 0x12345678) { Serial.println("RTCIO FAIL pad"); return; }

  Serial.println("RTCIO PASS");
}

void loop() {
  Serial.println("DONE");
  delay(1000);
}
