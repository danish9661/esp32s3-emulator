// NVS (non-volatile storage) validation: Preferences put/get across a
// close/reopen cycle exercises the real IDF NVS stack (page bitmaps,
// TLV entries, CRC32) over our MEMSPI program/erase model.
#include <Arduino.h>
#include <Preferences.h>

void setup() {
  Serial.begin(115200);
  delay(200);
  Preferences prefs;
  bool ok = prefs.begin("emu", false);
  Serial.printf("NVS BEGIN %d\n", (int)ok);
  ok = ok && prefs.putInt("counter", 42);
  ok = ok && prefs.putString("name", "hello");
  Serial.printf("NVS WRITE %d\n", (int)ok);
  int32_t counter = prefs.getInt("counter", -1);
  String name = prefs.getString("name", "?");
  Serial.printf("NVS READ counter=%d name=%s\n", (int)counter, name.c_str());
  ok = ok && counter == 42 && name == "hello";
  // Close and reopen read-only: persistence within the run.
  prefs.end();
  ok = ok && prefs.begin("emu", true);
  int32_t counter2 = prefs.getInt("counter", -1);
  String name2 = prefs.getString("name", "?");
  Serial.printf("NVS REOPEN counter=%d name=%s\n", (int)counter2, name2.c_str());
  ok = ok && counter2 == 42 && name2 == "hello";
  prefs.end();
  Serial.println(ok ? "NVS PASS" : "NVS FAIL");
}

void loop() {}
