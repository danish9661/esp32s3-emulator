#include <WiFi.h>
#include <NimBLEDevice.h>
// WiFi + BLE coexistence validation sketch (battery entry `coex`). Runs
// BOTH radios concurrently in one firmware image, proving the emulator's
// virtual RF medium needs no coexistence arbitration offline:
//
// * WiFi STA association through the host fixture (WIFI_STA_CONN=1 posts
//   CONNECTED + GOT_IP; same boundary as `wifi_sta`, no gateway needed).
// * BLE GATT-server init + advertise through the ROM controller loopback
//   (same path as `ble` without the bridge: init + service + advertise,
//   markers local, no Bumble needed).
//
// On silicon the two stacks share one antenna via the coexistence arbiter
// (PTA); in the emulator both media are virtual (fixture posts + loopback
// replies), so neither starves the other — the verdict is that BOTH reach
// DONE in the same boot with no crash, no WDT, no RF conflict. If the
// stacks interfered (shared IRQ, heap, or controller state clobbered),
// one side would miss its markers (status != 3 or BLE init != 1).
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("COEX START");
  // WiFi STA leg first (needs the WiFi task + event loop up before BLE
  // takes the controller; on silicon coex grants RF by priority — here
  // both are virtual, order only matters for marker readability).
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin("EmuNet", "password");
  uint8_t st = WiFi.waitForConnectResult(8000);
  Serial.print("COEX wifi status ");
  Serial.println(st);
  bool wifi_ok = (st == WL_CONNECTED);
  if (wifi_ok) {
    Serial.print("COEX wifi ip ");
    Serial.println(WiFi.localIP());
  }
  // BLE leg: same init sequence as the `ble` sketch (controller init +
  // enable + NimBLE host init + one service + advertise). No bridge: the
  // ROM loopback answers the init commands locally (proven by `ble`
  // passing with no bridge).
  bool ble_ok = false;
  NimBLEDevice::init("ESP32-S3-COEX");
  NimBLEServer *srv = NimBLEDevice::createServer();
  if (srv != nullptr) {
    NimBLEService *svc = srv->createService("180F");
    if (svc != nullptr) {
      svc->start();
      NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
      if (adv != nullptr) {
        adv->addServiceUUID("180F");
        ble_ok = adv->start();
      }
    }
  }
  Serial.print("COEX ble adv ");
  Serial.println(ble_ok ? 1 : 0);
  // Joint verdict: both sides up concurrently. Then clean shutdown (WiFi
  // disconnect like `wifi_sta`; BLE deinit is a no-op for the harness —
  // the run ends here, no reboot loop).
  if (wifi_ok) {
    WiFi.disconnect();
    delay(500);
  }
  Serial.print("COEX wifi ");
  Serial.print(wifi_ok ? 1 : 0);
  Serial.print(" ble ");
  Serial.println(ble_ok ? 1 : 0);
  if (wifi_ok && ble_ok) {
    Serial.println("COEX PASS");
  } else {
    Serial.println("COEX FAIL");
  }
  Serial.println("COEX DONE");
}

void loop() { delay(1000); }
