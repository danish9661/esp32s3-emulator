#include <WiFi.h>
#include <esp_wifi.h>
// SoftAP validation sketch (battery entry `wifi_ap`, fixture
// `WIFI_AP_FIXTURE=1`). The emulator has no RF: the host stages the AP
// fixture data (config + 192.168.4.1/24 LAN) at boot, the firmware posts
// WIFI_EVENT_AP_START itself, and the full Arduino chain (`_onApEvent` →
// STARTED bits → `softAP()` returns true) runs unmodified. Asserts
// `softAP 1` + IP + `stations 0` + IDF `clients 0` → `WIFI AP DONE`.
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("WIFI AP START");
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP("EmuAP", "password", 6);
  Serial.print("WIFI AP softAP ");
  Serial.println(ok ? 1 : 0);
  if (ok) {
    Serial.print("WIFI AP IP ");
    Serial.println(WiFi.softAPIP());
    Serial.print("WIFI AP stations ");
    Serial.println(WiFi.softAPgetStationNum());
    wifi_sta_list_t clients;
    esp_err_t serr = esp_wifi_ap_get_sta_list(&clients);
    Serial.print("WIFI AP clients ");
    Serial.println(serr == ESP_OK ? (int)clients.num : -1);
  } else {
    Serial.println("WIFI AP NO EVENT");
  }
  Serial.println("WIFI AP DONE");
}
void loop() { delay(1000); }
