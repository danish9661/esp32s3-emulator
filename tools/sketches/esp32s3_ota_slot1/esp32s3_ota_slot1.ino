// OTA slot-1 image for the end-to-end OTA test (see esp32s3_ota_update).
// This sketch is built once; its raw app binary is embedded into the
// updater sketch, which writes it to the ota_1 partition via the real
// esp_ota_* driver path. After esp_restart the emulator boots this image
// from ota_1 — proving otadata programming + OTA slot selection.
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("OTA SLOT1 ALIVE");
  Serial.println("OTA SLOT1 DONE");
}

void loop() {
  delay(1000);
}
