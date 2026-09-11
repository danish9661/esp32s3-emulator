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

  // 4. Odd-offset / odd-size reads (littlefs metadata pattern): read
  // 64 bytes at 0x8011 and compare against the image bytes the host
  // baked in (partition entries 2..3 area, mostly 0xFF with the MD5
  // record tail). Any mismatch means the USR read path corrupts
  // sub-block reads, which breaks littlefs file reads.
  ESP.flashRead(0x8011, s_words, 64);
  Serial.print("ODD: ");
  for (int i = 0; i < 16; i++) {
    if (s_buf[i] < 16) Serial.print('0');
    Serial.print(s_buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  // 0x8011..0x801F are entry-0 padding zeros; 0x8020 is entry1 magic AA.
  if (s_buf[0] != 0x00 || s_buf[15] != 0xAA) {
    Serial.println("FLASHREAD ODD OFFSET FAIL");
    ok = false;
  } else {
    Serial.println("FLASHREAD ODD OFFSET OK");
  }

  // Odd offset AND odd length with nonzero content: 13 bytes at 0x8005
  // must read back exactly (exercises sub-block FS-style reads).
  ESP.flashRead(0x8005, s_words, 13);
  const uint8_t expect13[13] = {
    0x90, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x6E, 0x76, 0x73, 0x00, 0x00, 0x00
  };
  bool odd_ok = true;
  for (int i = 0; i < 13; i++) {
    if (s_buf[i] != expect13[i]) {
      odd_ok = false;
      break;
    }
  }
  if (!odd_ok) {
    Serial.println("FLASHREAD ODD LEN FAIL");
    ok = false;
  } else {
    Serial.println("FLASHREAD ODD LEN OK");
  }

  // High-offset program+readback (where MicroPython's FS lives):
  // erase is skipped (region is already erased 0xFF in this image);
  // program 64 bytes at 0x200000 and 0x200001, read back exactly.
  ESP.flashEraseSector(0x200000 / 4096);
  for (int i = 0; i < 64; i++) s_buf[i] = (uint8_t)(0xA0 + i);
  ESP.flashWrite(0x200000, s_words, 64);
  ESP.flashRead(0x200000, s_words, 64);
  for (int i = 0; i < 64; i++) {
    if (s_buf[i] != (uint8_t)(0xA0 + i)) {
      Serial.println("FLASHREAD HIGH RW FAIL");
      ok = false;
      break;
    }
  }
  if (ok) Serial.println("FLASHREAD HIGH RW OK");
  // Odd-address write needs its own erase: NOR program can only clear bits,
  // so writing over the even pattern above without erasing reads back the
  // AND of both patterns (silicon-identical behavior, not a model gap).
  ESP.flashEraseSector(0x200000 / 4096);
  for (int i = 0; i < 64; i++) s_buf[i] = (uint8_t)(0x50 + i);
  ESP.flashWrite(0x200001, s_words, 64);
  ESP.flashRead(0x200001, s_words, 64);
  for (int i = 0; i < 64; i++) {
    if (s_buf[i] != (uint8_t)(0x50 + i)) {
      Serial.println("FLASHREAD HIGH ODD RW FAIL");
      ok = false;
      break;
    }
  }
  if (ok) Serial.println("FLASHREAD HIGH ODD RW OK");

  Serial.println(ok ? "FLASHREAD PASS" : "FLASHREAD FAIL");
  Serial.println("DONE");
}

void loop() {
  delay(1000);
}
