// TWAI (CAN 2.0B) driver-path validation for the ESP32-S3 emulator.
// Exercises the REAL esp-idf TWAI driver (driver/twai.h) in self-test
// (TWAI_MODE_NO_ACK) loopback mode -- this drives interrupts + the TX/RX
// buffer handshake through the driver's ISR/semaphore path, which the
// direct-poke sketch (esp32s3_twai) does not.
//
// Flow: install driver, start, transmit a frame, receive the looped-back
// frame, compare, uninstall.

#include "driver/twai.h"
#include "freertos/FreeRTOS.h"

#define TX_PIN GPIO_NUM_2
#define RX_PIN GPIO_NUM_3

void setup() {
  Serial.begin(115200);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NO_ACK);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_25KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    Serial.println("TWAI DRIVER INSTALL FAIL");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI DRIVER START FAIL");
    return;
  }

  twai_message_t msg;
  msg.identifier = 0x123;
  msg.data_length_code = 8;
  msg.extd = 0;
  msg.rtr = 0;
  for (int i = 0; i < 8; i++) msg.data[i] = (uint8_t)(0x10 + i);

  if (twai_transmit(&msg, pdMS_TO_TICKS(1000)) != ESP_OK) {
    Serial.println("TWAI DRIVER TX FAIL");
    return;
  }

  twai_message_t rx;
  if (twai_receive(&rx, pdMS_TO_TICKS(1000)) != ESP_OK) {
    Serial.println("TWAI DRIVER RX FAIL");
    return;
  }

  bool match = (rx.identifier == 0x123) && (rx.data_length_code == 8);
  for (int i = 0; i < 8; i++) {
    if (rx.data[i] != msg.data[i]) match = false;
  }

  Serial.println(match ? "TWAI DRIVER LOOPBACK PASS" : "TWAI DRIVER LOOPBACK FAIL");

  twai_stop();
  twai_driver_uninstall();
}

void loop() {}
