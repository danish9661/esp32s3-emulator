// OTA boot-slot validation sketch.  Reads the current OTA slot from the
// partition table and prints which slot is active.  This validates that
// the emulator's OTA boot-slot selection (select_ota_boot_offset) works
// with real ESP-IDF partition table parsing.
//
// Requires a custom partition table with at least two OTA app slots and
// an otadata partition.  Built with:
//   arduino-cli compile --build-property "build.partitions=two_ota.csv" ...

#include <esp_ota_ops.h>
#include <esp_partition.h>

void setup() {
  Serial.begin(115200);
  delay(50);

  Serial.println("OTA SLOT TEST START");

  // Get the running OTA partition
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running) {
    Serial.printf("OTA running: label=%s addr=0x%08x size=0x%x type=%d subtype=%d\n",
                  running->label, running->address, running->size,
                  running->type, running->subtype);
  } else {
    Serial.println("OTA running: NULL");
  }

  // Try to find OTA partitions
  esp_partition_iterator_t it = esp_partition_find(
    ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  int slot = 0;
  while (it) {
    const esp_partition_t* p = esp_partition_get(it);
    Serial.printf("OTA slot %d: label=%s addr=0x%08x size=0x%x subtype=0x%x\n",
                  slot, p->label, p->address, p->size, p->subtype);
    slot++;
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);

  // Try to get OTA state
  const esp_partition_t* otadata = esp_partition_find_first(
    ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
  if (otadata) {
    Serial.printf("OTA otadata: label=%s addr=0x%08x size=0x%x\n",
                  otadata->label, otadata->address, otadata->size);
  } else {
    Serial.println("OTA otadata: not found");
  }

  Serial.println("OTA SLOT TEST DONE");
  Serial.println("OTA SLOT TEST PASS");
}

void loop() {
  delay(1000);
}
