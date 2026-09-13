// ESP32-S3 SPI-mode SD (SDSPI) FAT validation for the emulator.
//
// Mounts the in-model SDSPI card through the real Arduino `SD` library
// (sd_diskio.cpp SPI byte path: CMD0/CMD8/CMD55+ACMD41/CMD58/CMD16/CMD17/
// CMD24 through GPSPI2), lists the root directory, reads the preformatted
// HELLO.TXT, and writes + reads back a second file. The card shares the
// SDMMC FAT16 image (host-provisioned), so this mounts the same volume
// the `sdfat` sketch uses — over SPI instead of the SDMMC host.
//
// Wiring (Arduino defaults on the generic S3 board): SCK=12, MISO=13,
// MOSI=11, SS=10.

#include "SD.h"
#include "SPI.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  SPI.begin(12 /*sck*/, 13 /*miso*/, 11 /*mosi*/, 10 /*ss*/);
  if (!SD.begin(10 /*ss*/)) {
    Serial.println("SDSPI BEGIN BAD");
    return;
  }
  Serial.println("SDSPI BEGIN OK");
  Serial.print("SDSPI SIZE MB=");
  Serial.println((uint32_t)(SD.cardSize() / (1024 * 1024)));
  Serial.print("SDSPI TYPE=");
  Serial.println((int)SD.cardType());

  File root = SD.open("/");
  if (!root) {
    Serial.println("SDSPI ROOT BAD");
    return;
  }
  Serial.print("SDSPI ROOT OK dir=");
  Serial.println(root.isDirectory() ? 1 : 0);
  File e = root.openNextFile();
  while (e) {
    Serial.print("SDSPI ENTRY ");
    Serial.println(e.name());
    e = root.openNextFile();
  }

  File f = SD.open("/hello.txt");
  if (!f) {
    Serial.println("SDSPI OPEN BAD");
    return;
  }
  String s = f.readString();
  f.close();
  Serial.print("SDSPI READ [");
  Serial.print(s);
  Serial.println("]");
  if (s == "Hello from SDMMC!\n") {
    Serial.println("SDSPI FAT READ PASS");
  } else {
    Serial.println("SDSPI READ MISMATCH");
  }

  File w = SD.open("/swrite.txt", FILE_WRITE);
  if (!w) {
    Serial.println("SDSPI WRITE OPEN BAD");
    return;
  }
  w.print("Hello from SDSPI!\n");
  w.close();
  File r = SD.open("/swrite.txt");
  String s2 = r ? r.readString() : "";
  if (r) r.close();
  Serial.print("SDSPI REREAD [");
  Serial.print(s2);
  Serial.println("]");
  if (s2 == "Hello from SDSPI!\n") {
    Serial.println("SDSPI FAT WRITE PASS");
  } else {
    Serial.println("SDSPI WRITE MISMATCH");
  }
  Serial.println("SDSPI DONE");
}

void loop() { delay(1000); }
