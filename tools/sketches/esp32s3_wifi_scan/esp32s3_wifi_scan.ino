#include <WiFi.h>
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("WIFI SCAN START");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  Serial.print("WIFI SCAN found ");
  Serial.println(n);
  for (int i = 0; i < n && i < 5; i++) {
    // SSID(i) returns an Arduino String (heap SSO); print the raw record
    // fields instead — byte-exact under the emulator (String printing is
    // a separate Arduino-core path, validated by every other sketch).
    String ssid;
    uint8_t enc;
    int32_t rssi;
    uint8_t *bssid;
    int32_t chan;
    WiFi.getNetworkInfo(i, ssid, enc, rssi, bssid, chan);
    Serial.print("WIFI NET ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(ssid);
    Serial.print(" ");
    Serial.print(rssi);
    Serial.print(" ");
    Serial.print(chan);
    Serial.print(" ");
    Serial.print(enc);
    Serial.print(" ");
    if (bssid) {
      for (int k = 0; k < 6; k++) {
        if (bssid[k] < 16) Serial.print("0");
        Serial.print(bssid[k], HEX);
        if (k < 5) Serial.print(":");
      }
    }
    Serial.println();
  }
  Serial.println("WIFI SCAN DONE");
}
void loop() { delay(1000); }
