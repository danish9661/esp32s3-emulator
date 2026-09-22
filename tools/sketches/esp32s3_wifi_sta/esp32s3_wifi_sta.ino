#include <WiFi.h>
// STA connect/disconnect validation sketch. The emulator has no RF, so the
// host completes the association at the firmware boundary (WIFI_STA_CONN=1):
// posts WIFI_EVENT_STA_CONNECTED + IP_EVENT_STA_GOT_IP through the real
// event chain, and the sketch prints the Arduino-visible results.
// Empty air (no fixture): begin() still succeeds, connect starts, but no
// events arrive and status() never reaches WL_CONNECTED (like silicon with
// no AP in range) — the sketch reports STA NO AP.
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("WIFI STA START");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin("EmuNet", "password");
  // waitForConnectResult polls status() (WL_CONNECTED == 3) with delay(100)
  uint8_t st = WiFi.waitForConnectResult(8000);
  Serial.print("WIFI STA status ");
  Serial.println(st);
  if (st == WL_CONNECTED) {
    Serial.print("WIFI STA IP ");
    Serial.println(WiFi.localIP());
    Serial.print("WIFI STA SSID ");
    Serial.println(WiFi.SSID());
    Serial.print("WIFI STA RSSI ");
    Serial.println(WiFi.RSSI());
    WiFi.disconnect();
    delay(500);
    Serial.print("WIFI STA after-disconnect ");
    Serial.println(WiFi.status());
  } else {
    Serial.println("WIFI STA NO AP");
  }
  Serial.println("WIFI STA DONE");
}
void loop() { delay(1000); }
