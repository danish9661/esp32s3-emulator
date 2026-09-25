#include <WiFi.h>
#include <esp_wifi.h>
// SoftAP validation sketch (NOT yet validated — no battery entry and no
// emulator fixture support committed). The emulator has no RF, so this
// sketch will need a host-side fixture once SoftAP support lands: the
// firmware posts WIFI_EVENT_AP_START itself and the full Arduino chain
// (`_onApEvent` → STARTED bits → `softAP()` returns true) should then run
// unmodified. NOTE: `softAPSSID()` reads back the closed driver's default
// AP config ("ESP_..." fallback), so the sketch asserts boot + IP +
// station count, not the SSID.
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
