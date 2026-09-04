// ESP32-S3 SPI flash driver-READ validation (arduino-cli, P5).
//
// Exercises the real esp-idf flash-read path (ESP.flashRead ->
// spi_flash_read -> MEMSPI USR 0x0B/0x03 transactions) the way
// MicroPython's storage layer reads filesystem sectors. Asserts the
// partition-table magic (bytes AA 50 at flash 0x8000, second entry too),
// the app-image magic (0xE9 at 0x10000) and a 4 KB sector readback,
// mirroring MicroPython's 4096-byte SEC_SIZE block reads.
#include <Arduino.h>
#include <Esp.h>

static uint32_t s_words[1024];
static uint8_t *s_buf = (uint8_t *)s_words;

static void print16(const uint8_t *b) {
  for (int i = 0; i < 16; i++) {
    if (b[i] < 16) Serial.print('0');
    Serial.print(b[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("FLASHREAD START");

  bool ok = true;

  // 1. Partition table magic + second entry magic (4 KB read like MP).
  ESP.flashRead(0x8000, s_words, 4096);
  Serial.print("TABLE: ");
  print16(s_buf);
  if (s_buf[0] != 0xAA || s_buf[1] != 0x50) {
    Serial.println("FLASHREAD TABLE MAGIC FAIL");
    ok = false;
  } else {
    Serial.println("FLASHREAD TABLE MAGIC OK");
  }
  if (s_buf[32] != 0xAA || s_buf[33] != 0x50) {
    Serial.println("FLASHREAD TABLE ENTRY1 FAIL");
    ok = false;
  } else {
    Serial.println("FLASHREAD TABLE ENTRY1 OK");
  }

  // 2. App image magic at 0x10000 (first byte 0xE9).
  ESP.flashRead(0x10000, s_words, 16);
  Serial.print("APP: ");
  print16(s_buf);
  if (s_buf[0] != 0xE9) {
    Serial.println("FLASHREAD APP MAGIC FAIL");
    ok = false;
  } else {
    Serial.println("FLASHREAD APP MAGIC OK");
  }

  // 3. NVS region (0x9000) reads back erased 0xFF.
  ESP.flashRead(0x9000, s_words, 16);
  Serial.print("NVS: ");
  print16(s_buf);
  for (int i = 0; i < 16; i++) {
    if (s_buf[i] != 0xFF) {
      Serial.println("FLASHREAD NVS ERASED FAIL");
      ok = false;
      break;
    }
  }
  if (ok) Serial.println("FLASHREAD NVS ERASED OK");

  Serial.println(ok ? "FLASHREAD PASS" : "FLASHREAD FAIL");
  Serial.println("DONE");
}

void loop() {
  delay(1000);
}
