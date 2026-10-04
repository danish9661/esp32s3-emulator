#include <WiFi.h>
#include <esp_wifi.h>
// Full 802.11 LL MAC slice 1: promiscuous sniffer + raw TX tap.
//
// The emulator has no RF, so the host acts as a virtual AP: once the
// sketch enables promiscuous mode with an MGMT filter, the host stages
// beacon frames (SSID "EmuAP", BSSID 02:11:22:33:44:55, channel 6) by
// invoking the registered promiscuous callback IN FIRMWARE with a staged
// `wifi_promiscuous_pkt_t` (rx_ctrl.sig_len + 802.11 beacon payload),
// and it captures `esp_wifi_80211_tx` frames at the callee entry (the
// closed driver never touches the air).
//
// Markers: LLMAC TX <rc> (esp_wifi_80211_tx return), LLMAC RX <n>
// (beacons seen), LLMAC SSID <1/0>, LLMAC DONE.
static volatile int rx_beacons = 0;
static volatile int rx_ssid_ok = 0;

static void sniffer(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) { return; }
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  uint16_t sig_len = pkt->rx_ctrl.sig_len;
  if (sig_len < 38) { return; }
  uint8_t *f = pkt->payload;
  // Beacon: frame control 0x80 (type MGMT, subtype beacon).
  if (f[0] != 0x80) { return; }
  rx_beacons++;
  // Fixed params (timestamp 8 + interval 2 + caps 2) then IEs; SSID IE
  // is first (id 0 at offset 36). SSID "EmuAP" = 45 6D 75 41 50.
  if (f[36] == 0 && f[37] == 5 && f[38] == 'E' && f[39] == 'm' &&
      f[40] == 'u' && f[41] == 'A' && f[42] == 'P') {
    rx_ssid_ok = 1;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("LLMAC START");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_wifi_set_promiscuous_filter(nullptr);
  esp_wifi_set_promiscuous_rx_cb(sniffer);
  esp_wifi_set_promiscuous(true);
  Serial.println("LLMAC sniff 1");
  // Raw probe request for "EmuAP" (FC 0x40, broadcast DA, our STA MAC as
  // SA/TA from the driver, SSID IE + rates). The host captures it at the
  // `esp_wifi_80211_tx` entry and returns ESP_OK without touching RF.
  uint8_t sta[6];
  WiFi.macAddress(sta);
  uint8_t probe[41];
  memset(probe, 0, sizeof(probe));
  probe[0] = 0x40; probe[1] = 0x00;           // FC: probe request
  memset(probe + 4, 0xFF, 6);                  // DA broadcast
  memcpy(probe + 10, sta, 6);                  // SA our MAC
  memcpy(probe + 16, sta, 6);                  // TA our MAC
  probe[24] = 0; probe[25] = 5;                // SSID IE
  probe[26] = 'E'; probe[27] = 'm'; probe[28] = 'u';
  probe[29] = 'A'; probe[30] = 'P';
  probe[31] = 1; probe[32] = 8;                // rates IE
  probe[33] = 0x82; probe[34] = 0x84; probe[35] = 0x8B; probe[36] = 0x96;
  probe[37] = 0x0C; probe[38] = 0x12; probe[39] = 0x18; probe[40] = 0x24;
  esp_err_t rc = esp_wifi_80211_tx(WIFI_IF_STA, probe, sizeof(probe), true);
  Serial.print("LLMAC TX ");
  Serial.println((int)rc);
  unsigned long t0 = millis();
  while (rx_beacons < 3 && millis() - t0 < 15000) { delay(50); }
  Serial.print("LLMAC RX ");
  Serial.println(rx_beacons);
  Serial.print("LLMAC SSID ");
  Serial.println(rx_ssid_ok);
  Serial.println("LLMAC DONE");
}
void loop() { delay(1000); }
