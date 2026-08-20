void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Hello from ESP32-S3!");
  Serial.println("boot OK");
}

void loop() {
  delay(1000);
}