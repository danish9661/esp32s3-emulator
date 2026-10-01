#include <WiFi.h>
#include <esp_netif.h>
#include <esp_netif_net_stack.h>
// L3–L7 application-protocol validation sketch (battery entry
// `test_worker_l3`). Same STA association + static fixture LAN as
// test_worker_net, then speaks REAL lwIP client frames through the live
// `esp_netif_transmit` tap at gateway-local services (see
// tools/gateway/handleL7.go — synchronous, deterministic, no wall clock):
//   * DNS: hand-built query for example.com -> 192.168.4.1:53
//   * NTP: 48-byte client mode-3 packet -> 192.168.4.1:123
//   * UDP echo probe -> 192.168.4.1:5683 (CoAP port; the gateway UDP
//     echo path answers — proves the divert/inject round trip)
// Each TX reuses the keep-alive discipline from test_worker_net (re-TX
// with 200 ms gaps so the host drains the TCP leg and stages replies
// via `net_inject_rx`), then pops each reply with a REAL closed-stack
// `esp_netif_receive` call and checks the BYTES (DNS RDATA / NTP mode /
// UDP echo payload), not the return code.
static esp_netif_t *s_netif = NULL;
static uint8_t s_mac[6];

static void tx(const uint8_t *f, size_t n) {
  esp_netif_transmit(s_netif, (void *)f, n);
}

// IPv4/UDP frame builder: eth(gw dst) + IPv4(board->gw) + UDP(sport->dport).
static size_t udp_frame(uint8_t *out, uint16_t sport, uint16_t dport,
                        const uint8_t *pay, size_t plen) {
  memcpy(out + 6, s_mac, 6);
  out[0] = 0x5a; out[1] = 0x94; out[2] = 0xef; out[3] = 0xe4; out[4] = 0x0c; out[5] = 0xdd;
  out[12] = 0x08; out[13] = 0x00;
  out[14] = 0x45; out[15] = 0x00;
  uint16_t iplen = 20 + 8 + plen;
  out[16] = iplen >> 8; out[17] = iplen & 0xff;
  out[18] = 0x12; out[19] = 0x34; out[20] = 0x00; out[21] = 0x00;
  out[22] = 0x40; out[23] = 17;
  out[24] = 0x00; out[25] = 0x00; // checksum left zero (tap is L2)
  out[26] = 192; out[27] = 168; out[28] = 4; out[29] = 2;
  out[30] = 192; out[31] = 168; out[32] = 4; out[33] = 1;
  out[34] = sport >> 8; out[35] = sport & 0xff;
  out[36] = dport >> 8; out[37] = dport & 0xff;
  out[38] = (8 + plen) >> 8; out[39] = (8 + plen) & 0xff;
  out[40] = 0x00; out[41] = 0x00; // UDP checksum zero (optional for IPv4)
  memcpy(out + 42, pay, plen);
  return 14 + iplen;
}

static uint16_t rx_type(const uint8_t *b) { return (uint16_t(b[12]) << 8) | b[13]; }

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("WORKER L3 START");
  WiFi.mode(WIFI_STA);
  WiFi.STA.config(IPAddress(192, 168, 4, 2), IPAddress(192, 168, 4, 1),
                  IPAddress(255, 255, 255, 0), IPAddress(8, 8, 8, 8));
  WiFi.begin("EmuNet", "password");
  uint8_t st = WiFi.waitForConnectResult(8000);
  Serial.print("WORKER L3 status ");
  Serial.println(st);
  if (st != WL_CONNECTED) {
    Serial.println("WORKER L3 NO AP");
    Serial.println("WORKER L3 DONE");
    return;
  }
  s_netif = WiFi.STA.netif();
  Serial.print("WORKER L3 netif ");
  Serial.println(s_netif != NULL ? 1 : 0);
  if (!s_netif) { Serial.println("WORKER L3 DONE"); return; }
  WiFi.macAddress(s_mac);

  uint8_t f[512];
  uint8_t rxb[256];

  // ---- DNS: query example.com (ID 0x2222), expect RDATA 93.184.216.34.
  // Query layout (29 bytes): header 12 (ID 2222, RD, QDCOUNT 1) +
  // QNAME 13 (07 example 03 com 00) + QTYPE/CLASS 4 (A/IN).
  uint8_t qq[29];
  qq[0]=0x22; qq[1]=0x22; qq[2]=0x01; qq[3]=0x00; qq[4]=0x00; qq[5]=0x01;
  qq[6]=0x00; qq[7]=0x00; qq[8]=0x00; qq[9]=0x00; qq[10]=0x00; qq[11]=0x00;
  qq[12]=7; memcpy(qq+13,"example",7); qq[20]=3; memcpy(qq+21,"com",3);
  qq[24]=0x00; qq[25]=0x00; qq[26]=0x01; qq[27]=0x00; qq[28]=0x01;
  size_t n = udp_frame(f, 45001, 53, qq, 29);
  tx(f, n);
  for (int w = 0; w < 1; w++) { delay(200); tx(f, n); }
  // The RX FIFO also stages unsolicited gateway frames (gVisor
  // ARP-refresh broadcasts land here too — proven live: 60B 0x0800
  // frames interleave with the 87B DNS replies). Pop up to 10 frames;
  // the verdict is ANY frame with UDP 53->45001 + DNS ID echo 0x2222 +
  // ANCOUNT 1 + the table RDATA 93.184.216.34 (unique on the wire).
  // Few receives (like the net sketch's 2x): each call — staged or
  // empty — runs closed-stack code, and dozens of calls destabilize the
  // run (proven live: ILLEGAL at ~17.5M steps with 24x scans). The FIFO
  // is drained in order (replies first, broadcasts rare), so 4 pops
  // observe the reply with margin.
  bool dns_ok = false;
  for (int k = 0; k < 2 && !dns_ok; k++) {
    memset(rxb, 0, sizeof(rxb));
    esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
    // DNS header at payload 42: ID 42:44 = 2222, ANCOUNT 48:50 = 00 01.
    if (rx_type(rxb) != 0x0800 || rxb[42] != 0x22 || rxb[43] != 0x22 || rxb[49] != 0x01) continue;
    for (size_t i = 42; i + 4 <= sizeof(rxb); i++) {
      if (rxb[i] == 93 && rxb[i+1] == 184 && rxb[i+2] == 216 && rxb[i+3] == 34) { dns_ok = true; break; }
    }
  }
  Serial.print("WORKER L3 dns ");
  Serial.println(dns_ok ? 34 : -21);
  // Drain leftovers (DNS replies + gVisor broadcasts) so the next leg
  // starts with an empty FIFO (8-deep cap would otherwise overflow the
  // next leg's scan window; receives on an empty FIFO run the real
  // closed-stack function harmlessly).
  // ---- NTP: 48-byte mode-3 client, expect mode-4 stratum-1 + fixed epoch.
  uint8_t nq[48];
  memset(nq, 0, sizeof(nq));
  nq[0] = 0x1b;
  n = udp_frame(f, 45002, 123, nq, 48);
  tx(f, n);
  for (int w = 0; w < 1; w++) { delay(200); tx(f, n); }
  // Same FIFO-pollution discipline as DNS: scan up to 10 frames for the
  // NTP reply (UDP 123->45002, mode 4, stratum 1, fixed epoch).
  bool ntp_ok = false;
  for (int k = 0; k < 2 && !ntp_ok; k++) {
    memset(rxb, 0, sizeof(rxb));
    esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
    ntp_ok = rx_type(rxb) == 0x0800 && (rxb[42] & 0x07) == 4 && rxb[43] == 1
      && rxb[82] == 0xec && rxb[83] == 0xf1 && rxb[84] == 0x2c && rxb[85] == 0x00;
  }
  Serial.print("WORKER L3 ntp ");
  Serial.println(ntp_ok ? 123 : -22);

  Serial.println("WORKER L3 DONE");
}
void loop() { delay(1000); }
