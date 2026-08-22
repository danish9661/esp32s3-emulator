// EFUSE validation sketch (ESP32-S3) — reads the factory MAC, chip revision
// and efuse chip id via the Arduino/esp-idf esp_efuse driver and prints them.
// Exercises the emulator's EFUSE read path (read_cmd handshake + RD_* mirrors).
// The modeled MAC is 0x112233445566 so the expected output is deterministic.

void setup() {
  Serial.begin(115200);
  delay(50);

  uint64_t mac = ESP.getEfuseMac();
  Serial.printf("MAC=%08X%08X\n", (uint32_t)(mac >> 32), (uint32_t)mac);

  uint32_t rev = ESP.getChipRevision();
  Serial.printf("CHIPREV=%u\n", rev);

  Serial.println("EFUSE DONE");
}

void loop() {}
