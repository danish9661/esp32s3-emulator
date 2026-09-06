// IDF-level SPI transmit validation (mirrors MicroPython machine.SPI).
// Uses spi_bus_initialize + spi_bus_add_device + spi_device_transmit with
// TXDATA/RXDATA (the exact path MP takes for <=4 byte transfers): the
// transfer runs through the modeled GPSPI2 + trans_done interrupt (bit 12
// of the DMA_INT block) + ISR completion. With no device on the bus MISO
// reads back 0; the goal is completion (ret=0, no hang) and exact bytes.
#include <Arduino.h>
#include <driver/spi_master.h>

#define PIN_SCK 10
#define PIN_MOSI 12
#define PIN_MISO 11

#define SPI2_REG(o) (*(volatile uint32_t *)(0x60024000 + (o)))

void setup() {
  Serial.begin(115200);
  Serial.println("SPIDEV START");

  spi_bus_config_t buscfg = {};
  buscfg.miso_io_num = PIN_MISO;
  buscfg.mosi_io_num = PIN_MOSI;
  buscfg.sclk_io_num = PIN_SCK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, 0);
  Serial.printf("SPIDEV bus_init=%d\n", (int)ret);

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 1000000;
  devcfg.mode = 0;
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 2;
  spi_device_handle_t spi;
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  Serial.printf("SPIDEV add=%d\n", (int)ret);

  bool ok = true;
  spi_transaction_t t = {};
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = 32;
  t.tx_data[0] = 0xA5;
  t.tx_data[1] = 0x5A;
  t.tx_data[2] = 0x3C;
  t.tx_data[3] = 0xC3;
  ret = spi_device_transmit(spi, &t);
  Serial.printf("SPIDEV xmit ret=%d rx=%02X%02X%02X%02X\n", (int)ret,
                t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);
  if (ret != 0) ok = false;
  for (int i = 0; i < 4; i++) {
    if (t.rx_data[i] != 0x00) ok = false;  // no device: MISO zeros
  }
  // trans_done latched at bit 12 (S3 DMA_INT block, not classic bit 0).
  if ((SPI2_REG(0x3C) & (1 << 12)) == 0) {
    Serial.println("SPIDEV NODONE FAIL");
    ok = false;
  }
  if (ok) {
    Serial.println("SPIDEV PASS");
  }
  Serial.println("SPIDEV DONE");
}

void loop() { delay(1000); }
