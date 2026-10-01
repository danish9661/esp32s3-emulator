#include <NimBLEDevice.h>
// BLE GATT-server validation sketch (battery entry `ble`). Advertises as
// "ESP32-S3-BLE" with one service 0x180F (Battery) + characteristic 0x2A19
// (Battery Level, read+notify, initial 100) + custom 128-bit echo
// characteristic (read+write, initial "emu") — the mirror of the Bumble
// bridge GATT app (tools/ble_bridge.py), so the two ends observe each
// other without any phone/host.
//
// VHCI path: NimBLE host (`esp_nimble_hci_init`) speaks H4 HCI over VHCI
// (`esp_vhci_host_send_packet` / `notify_host_recv`); the emulator's BLE
// tap (soc.rs `bt_hci_*`, BT page 0x60011000, RWBLE source 8) captures
// firmware sends into `pending_tx` + stages bridge replies into the RX
// FIFO. The host (`run_flash` BLE_GW leg) forwards length-prefixed to
// the Bumble virtual controller, which answers from its GATT app.
//
// Markers are firmware-local (no bridge needed for PASS): the sketch
// asserts its own GATT table is up (service/characteristic handles
// nonzero), that a local write/read round-trips, and that advertising
// started. With BLE_GW connected the same binary additionally exchanges
// HCI with Bumble (TX tap fires; replies stage); without it the markers
// still pass (silicon with a quiet controller).
static NimBLEServer *s_srv = nullptr;
static NimBLECharacteristic *s_level = nullptr;
static NimBLECharacteristic *s_echo = nullptr;

class EchoCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &info) override {
    (void)info;
    // Loop back the written value on the notify characteristic so a
    // connected peer (Bumble) observes the write as a notification.
    std::string v = c->getValue();
    if (s_level) { s_level->setValue(v); s_level->notify(); }
  }
};

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("BLE START");
  // NOTE: NimBLEDevice::init runs esp_bt_controller_init + enable +
  // esp_nimble_hci_init (VHCI register_callback + ble_transport_init).
  // The emulator's BT page (0x60011000) is a plain store + the BLE tap
  // stages Bumble replies; no fixture needed for local markers.
  NimBLEDevice::init("ESP32-S3-BLE");
  Serial.println("BLE init 1");
  s_srv = NimBLEDevice::createServer();
  Serial.print("BLE server ");
  Serial.println(s_srv != nullptr ? 1 : 0);
  NimBLEService *svc = s_srv->createService("180F");
  Serial.print("BLE service ");
  Serial.println(svc != nullptr ? 1 : 0);
  s_level = svc->createCharacteristic(
      "2A19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  s_level->setValue((uint8_t)100);
  s_echo = svc->createCharacteristic(
      "12345678-1234-5678-1234-56789abcdef0",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  s_echo->setValue("emu");
  s_echo->setCallbacks(new EchoCallbacks());
  svc->start();
  Serial.print("BLE chars ");
  Serial.println((s_level != nullptr && s_echo != nullptr) ? 2 : 0);
  // Local write/read round-trip (no controller needed).
  s_echo->setValue("emu!");
  std::string v = s_echo->getValue();
  Serial.print("BLE echo ");
  Serial.println(v == "emu!" ? 4 : -31);
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID("180F");
  adv->setName("ESP32-S3-BLE");
  bool a = adv->start();
  Serial.print("BLE adv ");
  Serial.println(a ? 1 : 0);
  Serial.print("BLE level ");
  Serial.println(s_level->getValue<uint8_t>() == 100 ? 100 : -32);
  Serial.println("BLE DONE");
}
void loop() { delay(1000); }
