// ESP32-S3 eMMC IDF-driver mount validation for the emulator.
//
// Mounts the in-model MMC card through the REAL Arduino SD_MMC stack
// (`esp_vfs_fat_sdmmc_mount` -> `sdmmc_card_init` MMC probe -> FATFS):
// the card's MMC personality (first CMD1 SEND_OP_COND switches SD->MMC)
// answers the JEDEC init CMD0/CMD1/CMD2/CMD3/CMD7/CMD9/CMD8/CMD6 sequence,
// then FAT lists root, reads the preformatted HELLO.TXT and writes +
// reads back a second file. Same volume the `sdfat`/`sdspi` sketches use.
//
// NOTE: Arduino's `cardType()` reports CARD_SDHC for any CCS card (it keys
// off the OCR CCS bit, not `is_mmc`), so TYPE=3 here is the CORRECT
// silicon behavior for our HCS MMC card — the MMC-ness is proven by the
// CMD1/SWITCH path the probe took, not the type code.
#include "SD_MMC.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  SD_MMC.setPins(12 /*clk*/, 11 /*cmd*/, 13 /*d0*/, 10 /*d1*/, 14 /*d2*/, 9 /*d3*/);
  if (!SD_MMC.begin("/sdcard", false)) {
    Serial.println("EMMC DRIVER BEGIN BAD");
    return;
  }
  Serial.println("EMMC DRIVER BEGIN OK");
  Serial.print("EMMC DRIVER SIZE MB=");
  Serial.println((uint32_t)(SD_MMC.cardSize() / (1024 * 1024)));
  Serial.print("EMMC DRIVER TYPE=");
  Serial.println((int)SD_MMC.cardType());

  File root = SD_MMC.open("/");
  if (!root) {
    Serial.println("EMMC DRIVER ROOT BAD");
    return;
  }
  Serial.print("EMMC DRIVER ROOT OK dir=");
  Serial.println(root.isDirectory() ? 1 : 0);

  File f = SD_MMC.open("/hello.txt");
  if (!f) {
    Serial.println("EMMC DRIVER OPEN BAD");
    return;
  }
  String s = f.readString();
  f.close();
  Serial.print("EMMC DRIVER READ [");
  Serial.print(s);
  Serial.println("]");
  if (s == "Hello from SDMMC!\n") {
    Serial.println("EMMC DRIVER FAT READ PASS");
  } else {
    Serial.println("EMMC DRIVER READ MISMATCH");
  }

  File w = SD_MMC.open("/ewrite.txt", FILE_WRITE);
  if (!w) {
    Serial.println("EMMC DRIVER WRITE OPEN BAD");
    return;
  }
  w.print("Hello from eMMC driver!\n");
  w.close();
  File r = SD_MMC.open("/ewrite.txt");
  String s2 = r ? r.readString() : "";
  if (r) r.close();
  Serial.print("EMMC DRIVER REREAD [");
  Serial.print(s2);
  Serial.println("]");
  if (s2 == "Hello from eMMC driver!\n") {
    Serial.println("EMMC DRIVER FAT WRITE PASS");
  } else {
    Serial.println("EMMC DRIVER WRITE MISMATCH");
  }
  Serial.println("EMMC DRIVER DONE");
}

void loop() { delay(1000); }
