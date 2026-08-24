#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  for (int i=0;i<12;i++){ Serial.println("P"); }
  Serial.println("DONE");
}
void loop(){ delay(1000); }
