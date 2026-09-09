// ESP32-S3 I2S esp-idf driver-path validation for the emulator.
//
// Brings up I2S0 in STD duplex mode through the real Arduino ESP_I2S stack
// (i2s_new_channel + i2s_channel_init_std_mode + GDMA descriptors +
// i2s_channel_write/read), with the model's word loopback poked on
// (I2S0 TX_CONF bit 27, I2S_SIG_LOOPBACK in i2s_reg.h) so transmitted words
// return to the receiver with no wires. Transfers are one DMA buffer each
// (240 stereo-16 frames = 960 bytes — the driver's buffer size).
//
// Full-duplex is REQUIRED: the S3 RX engine only captures while RX is
// running (the 16-word RX FIFO overruns otherwise), so a FreeRTOS reader
// task blocks in readBytes() BEFORE the writer sends — the same two-task
// shape as ESP-IDF's own std-loopback example. A sequential write-then-read
// loses the TX words on silicon too (RX idle during TX).
//
// Streaming notes (all silicon behavior, verified against IDF release/v5.3
// source): the TX descriptors form a ring (6 x 960 B, calloc-zeroed) with
// EOF set on every one; EOF is an event, not a stop, so GDMA streams them
// continuously once started. The single write lands in the first recycled
// buffer (descriptor 0, after its initial zeros pass), so the transmitted
// stream is [zeros, pattern, zeros...] with the pattern in the second cycle;
// Arduino auto-clear zeroes each buffer after its pass, so the pattern
// passes exactly once. The reader's read-entry resets wipe the word in
// flight (observed: stable 1 word), so the pattern read is one word short
// of a full buffer and completes with a zeros fill word. The reader
// therefore takes seven buffers and the verdict sync-searches the unique
// second sync word, requiring the following 238 pattern words byte-exact —
// the full driver path (channel setup, GDMA OUT + IN descriptor walks,
// TX/RX DONE interrupts, event queues) with byte-exact content.
//
// Expected run_flash output: "I2S DRIVER BEGIN OK" then
// "I2S DRIVER LOOPBACK PASS" and "I2S DRIVER DONE".

#include "ESP_I2S.h"

I2SClass i2s(I2S_NUM_0);

#define I2S0_TX_CONF (*(volatile uint32_t *)0x6000F024UL)
#define I2S_SIG_LOOPBACK (1u << 27)

// One DMA buffer = 240 frames x 4 bytes (stereo 16-bit); seven reads cover
// the zeros head, the pattern pass, and the zeros tail (see above).
#define XFER_WORDS 240
#define XFER_BYTES (XFER_WORDS * 4)
#define NREADS 7
static uint8_t tx_buf[XFER_BYTES];
static uint8_t rx_buf[NREADS][XFER_BYTES];
static volatile size_t g_wrote = 0;
static volatile size_t g_read = 0;

static void reader_task(void *arg) {
  (void)arg;
  size_t total = 0;
  for (int i = 0; i < NREADS; i++) {
    size_t n = i2s.readBytes((char *)rx_buf[i], sizeof(rx_buf[i]));
    if (n != sizeof(rx_buf[i])) break;
    total += n;
  }
  g_read = total;
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  i2s.setPins(4, 5, 6, 7); // bclk, ws, dout, din (any valid pins)
  if (!i2s.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("I2S DRIVER BEGIN BAD");
    return;
  }
  Serial.println("I2S DRIVER BEGIN OK");
  I2S0_TX_CONF |= I2S_SIG_LOOPBACK;

  // Unique sync head (the 0xA0 pattern repeats every 16 words): word0 =
  // 0x11111111, word1 = 0x22222222, then the repeating pattern.
  tx_buf[0] = 0x11; tx_buf[1] = 0x11; tx_buf[2] = 0x11; tx_buf[3] = 0x11;
  tx_buf[4] = 0x22; tx_buf[5] = 0x22; tx_buf[6] = 0x22; tx_buf[7] = 0x22;
  for (int i = 8; i < XFER_BYTES; i++) {
    tx_buf[i] = (uint8_t)(0xA0 + (i & 0x3F));
  }
  for (int i = 0; i < NREADS; i++) {
    memset(rx_buf[i], 0, sizeof(rx_buf[i]));
  }
  xTaskCreate(reader_task, "i2s_reader", 4096, NULL, 1, NULL);
  delay(100); // let the reader block in the first read first
  g_wrote = i2s.write(tx_buf, sizeof(tx_buf));
  Serial.printf("I2S DRIVER wrote %u\n", (unsigned)g_wrote);

  for (int i = 0; i < 400 && g_read == 0; i++) {
    delay(50); // wait for the reader task to finish (up to ~20 s)
  }
  Serial.printf("I2S DRIVER read %u\n", (unsigned)g_read);

  // Find the buffer holding the unique sync word 0x22222222 (pattern word
  // 1), then require the following words byte-exact through that buffer's
  // end (238+ words) — the pattern pass minus its wiped head word.
  bool ok = (g_wrote == sizeof(tx_buf)) && (g_read == NREADS * sizeof(rx_buf[0]));
  int sync_b = -1, sync_w = -1;
  if (ok) {
    for (int b = 0; b < NREADS && sync_b < 0; b++) {
      for (int w = 0; w < XFER_WORDS && sync_b < 0; w++) {
        uint32_t vw;
        memcpy(&vw, rx_buf[b] + 4 * w, 4);
        if (vw == 0x22222222ul) {
          sync_b = b;
          sync_w = w;
        }
      }
    }
  }
  int nwords = -1;
  if (ok && sync_b >= 0 && sync_w <= 2) {
    // Words after sync must equal tx words after word 1; bound by both the
    // buffer remainder and the tx remainder (239 words after word 1).
    int avail_rx = XFER_WORDS - sync_w;
    int avail_tx = XFER_WORDS - 1;
    nwords = avail_rx < avail_tx ? avail_rx : avail_tx;
    for (int i = 0; i < nwords * 4 && ok; i++) {
      if (rx_buf[sync_b][4 * sync_w + i] != tx_buf[4 * 1 + i]) ok = false;
    }
    if (nwords < 238) ok = false;
    Serial.printf("I2S DRIVER sync=%d:%d nwords=%d\n", sync_b, sync_w, ok ? nwords : -1);
  } else {
    ok = false;
  }
  if (ok) {
    Serial.println("I2S DRIVER LOOPBACK PASS");
  } else {
    Serial.print("I2S DRIVER MISMATCH w=");
    Serial.print(g_wrote);
    Serial.print(" r=");
    Serial.println(g_read);
  }
  Serial.println("I2S DRIVER DONE");
}

void loop() {
  delay(1000);
}
