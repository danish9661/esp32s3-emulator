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
  // Write-protect the USR_DATA block, then prove a second burn is
  // refused: the driver reads back WR_DIS and rejects protected fields.
  esp_err_t ew = esp_efuse_write_field_cnt(ESP_EFUSE_WR_DIS_USER_DATA, 1);
  Serial.print("EFUSE WRDIS rc=");
  Serial.println((int)ew);
  size_t cnt = 0;
  esp_efuse_read_field_cnt(ESP_EFUSE_WR_DIS_USER_DATA, &cnt);
  Serial.print("EFUSE WRDIS cnt=");
  Serial.println((unsigned long)cnt);
  uint8_t pat2[32];
  for (int i = 0; i < 32; i++) pat2[i] = (uint8_t)(0x5A + i);
  esp_err_t e2 = esp_efuse_write_field_blob(ESP_EFUSE_USER_DATA, pat2, sizeof(pat2) * 8);
  Serial.print("EFUSE REBURN rc=");
  Serial.println((int)e2);
  // The reburn must fail AND the original pattern must be intact (OR
  // semantics: a partial program could only set bits, never match pat2).
  uint8_t after2[32] = {0};
  esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA, after2, sizeof(after2) * 8);
  bool intact = true;
  for (int i = 0; i < 32; i++) {
    if (after2[i] != (uint8_t)(0xA5 + i)) intact = false;
  }
  if (ew == ESP_OK && cnt == 1 && e2 != ESP_OK && intact) {
    Serial.println("EFUSE WRDIS PASS");
  } else {
    Serial.println("EFUSE WRDIS FAIL");
  }
  Serial.println("EFUSE BURN DONE");
}

void loop() {
  delay(1000);
}
