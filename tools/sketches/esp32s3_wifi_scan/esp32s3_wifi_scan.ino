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
    Serial.print("WIFI NET ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(WiFi.SSID(i));
  }
  Serial.println("WIFI SCAN DONE");
}
void loop() { delay(1000); }
