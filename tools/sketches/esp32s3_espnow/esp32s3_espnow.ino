#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
// ESP-NOW validation sketch (Arduino ESP_NOW peer API; battery entry
// `espnow`, fixture `WIFI_ESPNOW_LOOPBACK=1`). The emulator has no RF,
// so the host acts as a virtual second node: firmware-to-host sends
// complete via an in-firmware TX-callback invocation (peer `onSent`
// → `sent_ok`), and a 2-byte frame from the peer MAC is delivered via
// the peer's `onReceive` directly (`got_rx`, `rx_byte0 = 0xA5`).
#define ESPNOW_WIFI_CHANNEL 6
static volatile bool sent_ok = false;
static volatile bool got_rx = false;
static uint8_t rx_byte0 = 0;
class EspNowHandler : public ESP_NOW_Peer {
public:
  EspNowHandler(const uint8_t *mac, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk)
    : ESP_NOW_Peer(mac, channel, iface, lmk) {}
  bool peerBegin() {
    if (!ESP_NOW.begin() || !add()) { return false; }
    return true;
  }
  bool peerSend(const uint8_t *data, size_t len) { return send(data, len); }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) override {
    (void)broadcast;
    if (len >= 1) { rx_byte0 = data[0]; got_rx = true; }
  }
  void onSent(bool success) override { sent_ok = success; }
};
static uint8_t peer_mac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
static EspNowHandler handler(peer_mac, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("WIFI ESPNOW START");
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) { delay(100); }
  bool ok = handler.peerBegin();
  Serial.print("WIFI ESPNOW init ");
  Serial.println(ok ? 1 : 0);
  Serial.print("WIFI ESPNOW addpeer ");
  Serial.println(ok ? 1 : 0);
  uint8_t tx[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  bool s = handler.peerSend(tx, sizeof(tx));
  Serial.print("WIFI ESPNOW sent ");
  Serial.println(s ? 1 : 0);
  unsigned long t0 = millis();
  while ((!sent_ok || !got_rx) && millis() - t0 < 15000) { delay(50); }
  Serial.print("WIFI ESPNOW sendcb ");
  Serial.println(sent_ok ? 1 : 0);
  Serial.print("WIFI ESPNOW rxcb ");
  Serial.println(got_rx ? 1 : 0);
  if (got_rx) {
    Serial.print("WIFI ESPNOW rx0 ");
    Serial.println(rx_byte0, HEX);
  }
  Serial.println("WIFI ESPNOW DONE");
}
void loop() { delay(1000); }
