#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"
#include "esp32_gpio.h"

#define TYPE_ESP32S3_GPIO "esp32s3.gpio"
#define ESP32S3_GPIO(obj)           OBJECT_CHECK(ESP32S3GPIOState, (obj), TYPE_ESP32S3_GPIO)
#define ESP32S3_GPIO_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3GPIOClass, obj, TYPE_ESP32S3_GPIO)
#define ESP32S3_GPIO_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3GPIOClass, klass, TYPE_ESP32S3_GPIO)

/* Bootstrap options for ESP32-S3 (4-bit) */
#define ESP32S3_STRAP_MODE_FLASH_BOOT 0x4   /* SPI Boot */

typedef struct ESP32S3State {
    Esp32GpioState parent;
} ESP32S3GPIOState;

typedef struct ESP32S3GPIOClass {
    Esp32GpioClass parent;
} ESP32S3GPIOClass;
