// eFuse burn validation: program USER_DATA fuses (one-way OR) and read
// them back. Uses esp_efuse_write_field_blob (batch) on ESP_EFUSE_USER_DATA.
#include "esp_efuse.h"
#include "esp_efuse_table.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  uint8_t before[32] = {0};
  esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA, before, sizeof(before) * 8);
  Serial.print("EFUSE BURN before0=");
  Serial.println(before[0], HEX);
  uint8_t pat[32];
  for (int i = 0; i < 32; i++) pat[i] = (uint8_t)(0xA5 + i);
  esp_err_t e1 = esp_efuse_write_field_blob(ESP_EFUSE_USER_DATA, pat, sizeof(pat) * 8);
  Serial.print("EFUSE BURN write rc=");
  Serial.println((int)e1);
  uint8_t after[32] = {0};
  esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA, after, sizeof(after) * 8);
  bool ok = (e1 == ESP_OK);
  for (int i = 0; i < 32; i++) {
    if (after[i] != (uint8_t)(0xA5 + i)) ok = false;
  }
  Serial.print("EFUSE BURN after0=");
  Serial.println(after[0], HEX);
  if (ok) {
    Serial.println("EFUSE BURN PASS");
  } else {
    Serial.println("EFUSE BURN FAIL");
  }
  Serial.println("EFUSE BURN DONE");
}

void loop() {
  delay(1000);
}
