#include <WiFi.h>
#include <esp_netif.h>
#include <esp_netif_net_stack.h>
// Live-IP backhaul validation sketch (battery entry `test_worker_net`).
// After the STA association completes (same fixture as wifi-sta:
// WIFI_STA_CONN=1 + WIFI_SCAN_APS), the sketch grabs the live lwIP netif
// pointer from the STA interface object and calls the REAL closed-stack
// `esp_netif_transmit(netif, data, len)` twice — an ARP probe then an IPv4
// frame — so the emulator's `esp_netif_transmit` TX tap captures genuine
// board->host Ethernet frames. The host drains each frame to NET_PCAP
// (tcpdump-readable) and/or the NET_GW gateway bridge (live-IP backhaul).
//
// Provenance: `esp_netif_transmit(esp_netif_t*, void*, size_t)` is lwIP's
// single egress point in the IDF netif layer (DHCP/ARP/IP/ICMP/UDP/TCP all
// leave here — the emulator hooks its linked entry in machine.rs and
// captures (data, len) read-only before the call runs). The payloads below
// are hand-built Ethernet frames (ARP request + IPv4/ICMP echo) with the
// fixture LAN addressing (board 192.168.4.2, gw 192.168.4.1), matching the
// Go gateway's 192.168.4.0/24 room.
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("WORKER NET START");
  // NOTE (proven live on this image): the sketch must NOT call
  // WiFi.disconnect() before begin() — the disconnect path emits a
  // firmware-side IDF STA_DISCONNECTED event whose handler runs the
  // closed `_onStaEvent` → `postEvent` translation and re-arms the
  // association machinery; the host CONNECTED post then lands in a
  // queue the translation never drains (WL_CONNECTED never arrives,
  // `waitForConnectResult` times out). The wifi-sta sketch gets away
  // with it (its image links the event code elsewhere); the worker
  // image does not. begin() straight from the boot state works.
  WiFi.mode(WIFI_STA);
  // Static fixture LAN (matches the host fixture: 192.168.4.2/24 gw .1,
  // same bytes the GOT_IP posts + insider hooks serve): skips the closed
  // DHCP client entirely (the emulator has no live DHCP/client task —
  // `esp_netif_get_ip_info` is hook-served — so a DHCP discover would
  // wedge in `dhcp_append` LWIP_ERROR and never complete). `config()`
  // with nonzero args sets HAS_STATIC_IP_BIT, so `connect()` skips
  // `config()`→DHCP and goes straight to `esp_wifi_connect()`.
  WiFi.STA.config(IPAddress(192, 168, 4, 2), IPAddress(192, 168, 4, 1),
                  IPAddress(255, 255, 255, 0), IPAddress(8, 8, 8, 8));
  WiFi.begin("EmuNet", "password");
  uint8_t st = WiFi.waitForConnectResult(8000);
  Serial.print("WORKER NET status ");
  Serial.println(st);
  if (st != WL_CONNECTED) {
    Serial.println("WORKER NET NO AP");
    Serial.println("WORKER NET DONE");
    return;
  }
  Serial.print("WORKER NET IP ");
  Serial.println(WiFi.localIP());
  // STA interface object owns the live `esp_netif_t*` (NetworkInterface.h:
  // `esp_netif_t *netif()` returns `_esp_netif` — the real esp_netif
  // instance the closed stack created at bring-up, NOT a fixture mirror).
  esp_netif_t *netif = WiFi.STA.netif();
  Serial.print("WORKER NET netif ");
  Serial.println(netif != NULL ? 1 : 0);
  if (netif == NULL) {
    Serial.println("WORKER NET NO NETIF");
    Serial.println("WORKER NET DONE");
    return;
  }
  // Frame 1: ARP request for the gateway (who-has 192.168.4.1, tell
  // 192.168.4.2). 42 bytes: eth dst ff:ff:ff:ff:ff:ff, src = STA MAC,
  // type 0x0806; ARP hwtype 1, proto 0x0800, hlen 6, plen 4, op 1,
  // sha = STA MAC, spa = 192.168.4.2, tha = 00:.., tpa = 192.168.4.1.
  // NOTE: `esp_netif_transmit` returns ESP_FAIL without a bound lwIP
  // netif (the emulator has no live DHCP/client stack — the tap is L2,
  // the return code is not the verdict). The battery asserts the UART
  // flow markers; the pcap/net-bridge asserts the captured bytes.
  uint8_t staMac[6];
  WiFi.macAddress(staMac);
  uint8_t arp[42];
  memset(arp, 0, sizeof(arp));
  memset(arp, 0xff, 6);
  memcpy(arp + 6, staMac, 6);
  arp[12] = 0x08; arp[13] = 0x06;
  arp[14] = 0x00; arp[15] = 0x01;
  arp[16] = 0x08; arp[17] = 0x00;
  arp[18] = 0x06; arp[19] = 0x04;
  arp[20] = 0x00; arp[21] = 0x01;
  memcpy(arp + 22, staMac, 6);
  arp[28] = 192; arp[29] = 168; arp[30] = 4; arp[31] = 2;
  arp[38] = 192; arp[39] = 168; arp[40] = 4; arp[41] = 1;
  esp_err_t r1 = esp_netif_transmit(netif, arp, sizeof(arp));
  Serial.print("WORKER NET tx1 ");
  Serial.println(r1 == ESP_OK ? 42 : -11);
  delay(200);
  // Frame 2: IPv4/ICMP echo request board -> gw (42 bytes: 14 eth + 20
  // IPv4 + 8 ICMP). The tap only needs the Ethernet header + length to
  // prove the path; the gateway answers ARP/ICMP live.
  uint8_t ip[42];
  memset(ip, 0, sizeof(ip));
  memcpy(ip + 6, staMac, 6);
  ip[0] = 0x5a; ip[1] = 0x94; ip[2] = 0xef; ip[3] = 0xe4; ip[4] = 0x0c; ip[5] = 0xdd;
  ip[12] = 0x08; ip[13] = 0x00;
  ip[14] = 0x45; ip[15] = 0x00;
  ip[16] = 0x00; ip[17] = 0x1c;
  ip[18] = 0x12; ip[19] = 0x34;
  ip[20] = 0x00; ip[21] = 0x00;
  ip[22] = 0x40; ip[23] = 0x01;
  ip[24] = 0x00; ip[25] = 0x00; // checksum left zero (tap is L2)
  ip[26] = 192; ip[27] = 168; ip[28] = 4; ip[29] = 2;
  ip[30] = 192; ip[31] = 168; ip[32] = 4; ip[33] = 1;
  ip[34] = 0x08; ip[35] = 0x00; // ICMP echo request
  ip[36] = 0x4c; ip[37] = 0x31; // ICMP checksum over type/code/zero+ident/seq
  ip[38] = 0xab; ip[39] = 0xcd;
  ip[40] = 0x00; ip[41] = 0x01;
  esp_err_t r2 = esp_netif_transmit(netif, ip, sizeof(ip));
  Serial.print("WORKER NET tx2 ");
  Serial.println(r2 == ESP_OK ? 42 : -12);
  // RX WAIT (proven live 2026-09-28): the gateway answers each TX ~1 s
  // after it (TCP round-trip + hub loop), but the two `esp_netif_receive`
  // calls below used to run back-to-back right after frame 2 — so the
  // replies always landed AFTER the sketch already checked (rx1 read -13
  // even with NET_GW up; only run_flash's TX-armed + 64-step-backstop
  // drain ever saw them). Keep the link busy (NOT delay()) while the host
  // drains the TCP leg: re-transmit the ARP probe ~15x with 200 ms gaps.
  // Each re-TX re-arms the reply drain AND defeats the harness idle-exit
  // (delay() parks the pc with no UART for 2M steps and run_flash quits
  // mid-wait; re-TX prints keep uart_buf moving). By the time the loop
  // ends BOTH replies are staged via `net_inject_rx` (ARP 60B + ICMP 42B
  // per TX — the FIFO holds 8, so nothing is lost). Headless default (no
  // NET_GW): the loop is empty-FIFO time, verdicts stay -13/-14, same
  // binary serves both legs with no marker change.
  for (int w = 0; w < 15; w++) {
    delay(200);
    esp_netif_transmit(netif, arp, sizeof(arp));
    Serial.print("WORKER NET keep ");
    Serial.println(w);
  }
  // Frame 3 (RX leg): call the REAL closed-stack `esp_netif_receive`
  // with a scratch buffer. With no staged frame the call runs unmodified
  // (silicon with no packet waiting — returns, no crash). With the
  // NET_GW gateway leg connected, BOTH gateway replies are waiting in
  // the RX FIFO by now (ARP reply to frame 1 + ICMP echo reply to frame
  // 2, staged via `net_inject_rx` from the TCP ingest leg): the
  // machine.rs entry hook pops the FIRST (ARP reply, 60B 0x0806 opcode
  // 2), copies it into `rxb`, and fake-returns ESP_OK — proving the
  // full gateway→board path through the real stack entry point. A
  // second call proves FIFO order (ICMP echo reply, 42B 0x0800 type 0).
  // The verdict is the buffer content (ethertype + ARP opcode / ICMP
  // type), not the return code (fixture images have no live lwIP input
  // path, so an unmodified call may return anything without touching
  // the buffer; `rxb` stays zeroed then and the verdict reads -13).
  uint8_t rxb[128];
  memset(rxb, 0, sizeof(rxb));
  esp_err_t r3 = esp_netif_receive(netif, rxb, sizeof(rxb), NULL);
  uint16_t rxtype = (uint16_t(rxb[12]) << 8) | rxb[13];
  uint16_t rxop = (uint16_t(rxb[20]) << 8) | rxb[21];
  Serial.print("WORKER NET rx1 ");
  Serial.print(r3 == ESP_OK ? 1 : 0);
  Serial.print(' ');
  Serial.println(rxtype == 0x0806 && rxop == 2 ? 60 : -13);
  memset(rxb, 0, sizeof(rxb));
  esp_err_t r4 = esp_netif_receive(netif, rxb, sizeof(rxb), NULL);
  uint16_t rxtype2 = (uint16_t(rxb[12]) << 8) | rxb[13];
  Serial.print("WORKER NET rx2 ");
  Serial.print(r4 == ESP_OK ? 1 : 0);
  Serial.print(' ');
  Serial.println(rxtype2 == 0x0800 && rxb[34] == 0 ? 42 : -14);
  Serial.println("WORKER NET DONE");
}
void loop() { delay(1000); }
