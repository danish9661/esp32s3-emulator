// eFuse custom-MAC validation: ESP_EFUSE_CUSTOM_MAC is a 6-byte field at
// USR_DATA offset 24 (bytes 24..29). Burning it exercises a partial-block
// PGM (staged words mostly zero) through the real batch driver, including
// the driver's read-back verify.
#include "esp_efuse.h"
#include "esp_efuse_table.h"
void setup() {
  Serial.begin(115200);
  delay(200);
  uint8_t cur[8] = {0};
  esp_efuse_read_field_blob(ESP_EFUSE_CUSTOM_MAC, cur, sizeof(cur) * 8);
  Serial.print("CUSTOM_MAC before=");
  for (int i = 0; i < 8; i++) { Serial.print(cur[i], HEX); Serial.print(' '); }
  Serial.println();
  uint8_t pat[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  esp_err_t e = esp_efuse_write_field_blob(ESP_EFUSE_CUSTOM_MAC, pat, sizeof(pat) * 8);
  Serial.print("CUSTOM_MAC burn rc=");
  Serial.println((int)e);
  uint8_t after[8] = {0};
  esp_efuse_read_field_blob(ESP_EFUSE_CUSTOM_MAC, after, sizeof(after) * 8);
  Serial.print("CUSTOM_MAC after=");
  for (int i = 0; i < 8; i++) { Serial.print(after[i], HEX); Serial.print(' '); }
  Serial.println();
  // Mirror check: the 6 bytes land at USR_DATA offset 24 (0x94/0x98) in
  // MAC byte order (blob byte j -> LSB-first word packing).
  uint32_t w5 = *(volatile uint32_t*)0x60007094u;
  uint32_t w6 = *(volatile uint32_t*)0x60007098u;
  bool ok = ((int)e == 0) && after[0] == 0x11 && after[5] == 0x66
    && after[6] == 0 && after[7] == 0
    && w5 == 0x33221100u && w6 == 0x00665544u;
  Serial.println(ok ? "EFUSE CUSTOM PASS" : "EFUSE CUSTOM FAIL");
  Serial.println("EFUSE CUSTOM DONE");
}
void loop() {}
