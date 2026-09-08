// ESP32-S3 SDMMC FAT filesystem validation for the emulator.
//
// Mounts the simulated SD card through the real Arduino SD_MMC stack
// (sdmmc_host + sdmmc_card_init + FATFS), lists the root directory, reads
// a preformatted text file, and writes + reads back a second file. This
// exercises the full driver path: CMD0/CMD5(RTO)/CMD8/ACMD41/CMD2/CMD3/
// CMD9/CMD7/ACMD51/ACMD13/CMD6(HS switch)/ACMD6(4-bit)/CMD16/CMD17/CMD24
// through the DesignWare host (IDMAC descriptors, interrupts, clocks).

#include "SD_MMC.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  // The generic S3 board has no default SDMMC pins; route anywhere valid
  // (no real card — the matrix routing is accepted and ignored).
  SD_MMC.setPins(12 /*clk*/, 11 /*cmd*/, 13 /*d0*/, 10 /*d1*/, 14 /*d2*/, 9 /*d3*/);
  if (!SD_MMC.begin("/sdcard", false)) {
    Serial.println("SD BEGIN BAD");
    return;
  }
  Serial.println("SD BEGIN OK");
  Serial.print("SD SIZE MB=");
  Serial.println((uint32_t)(SD_MMC.cardSize() / (1024 * 1024)));
  Serial.print("SD TYPE=");
  Serial.println((int)SD_MMC.cardType());

  File root = SD_MMC.open("/");
  if (!root) {
    Serial.println("SD ROOT BAD");
    return;
  }
  Serial.print("SD ROOT OK dir=");
  Serial.println(root.isDirectory() ? 1 : 0);
  File e = root.openNextFile();
  while (e) {
    Serial.print("SD ENTRY ");
    Serial.println(e.name());
    e = root.openNextFile();
  }

  File f = SD_MMC.open("/hello.txt");
  if (!f) {
    Serial.println("SD OPEN BAD");
    return;
  }
  String s = f.readString();
  f.close();
  Serial.print("SD READ [");
  Serial.print(s);
  Serial.println("]");
  if (s == "Hello from SDMMC!\n") {
    Serial.println("SD FAT READ PASS");
  } else {
    Serial.println("SD READ MISMATCH");
  }

  File w = SD_MMC.open("/write.txt", FILE_WRITE);
  if (!w) {
    Serial.println("SD WOPEN BAD");
    return;
  }
  w.print("0123456789ABCDEF");
  w.close();
  File r = SD_MMC.open("/write.txt");
  String s2 = r.readString();
  r.close();
  if (s2 == "0123456789ABCDEF") {
    Serial.println("SD FAT WRITE PASS");
  } else {
    Serial.println("SD WRITE MISMATCH");
  }
  Serial.println("SD DONE");
}

void loop() {
  delay(1000);
}
