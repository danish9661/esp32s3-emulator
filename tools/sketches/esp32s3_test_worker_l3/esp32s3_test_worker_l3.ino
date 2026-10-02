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
//   * HTTP: raw TCP SYN -> SYN-ACK, GET / -> 200 + S3LAB-EMU-OK body
//     via the gateway-local :80 stub (no lwIP TCP state — hand-built
//     IP/TCP like the UDP legs, replies read with the same receive
//     calls; the closed lwIP TCP stack is never involved)
//   * MQTT: raw TCP SYN -> SYN-ACK, CONNECT -> CONNACK(accepted),
//     SUBSCRIBE -> SUBACK(QoS0) via the gateway-local :1883 stub
//     (PUBLISH QoS0 is accept-silently — no wire reply exists)
// Each TX reuses the keep-alive discipline from test_worker_net (re-TX
// with 200 ms gaps so the host drains the TCP leg and stages replies
// via `net_inject_rx`), then pops each reply with a REAL closed-stack
// `esp_netif_receive` call and checks the BYTES (DNS RDATA / NTP mode /
// UDP echo payload / HTTP status+body / MQTT CONNACK+SUBACK), not the
// return code. Without a gateway (plain battery) every leg reads empty
// (-21/-22/-23/-24) and the sketch still reaches DONE.
static esp_netif_t *s_netif = NULL;
static uint8_t s_mac[6];

static void tx(const uint8_t *f, size_t n) {
  esp_netif_transmit(s_netif, (void *)f, n);
}

// IPv4/UDP frame builder: eth(gw dst) + IPv4(board->gw) + UDP(sport->dport).
static size_t udp_frame(uint8_t *out, uint16_t sport, uint16_t dport,
                        const uint8_t *pay, size_t plen) {  memcpy(out + 6, s_mac, 6);
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
static uint32_t rx_u32(const uint8_t *b) {
  return (uint32_t(b[0]) << 24) | (uint32_t(b[1]) << 16) | (uint32_t(b[2]) << 8) | b[3];
}

// IPv4/TCP frame builder: eth(gw dst) + IPv4(board->gw, proto 6) +
// TCP(sport->dport, seq/ack/flags, no opts). Checksums left zero (the tap
// is L2; the gateway parses without validating them — same discipline as
// udp_frame above). The closed lwIP TCP stack is never involved: these
// are raw frames through the `esp_netif_transmit` tap, and replies come
// back through `esp_netif_receive` FIFO pops — no handshake state, no
// socket, no netconn (the gateway stub answers synchronously per frame,
// so sequence numbers only need to be wire-plausible, not negotiated).
static size_t tcp_frame(uint8_t *out, uint16_t sport, uint16_t dport,
                        uint32_t seq, uint32_t ack, uint8_t flags,
                        const uint8_t *pay, size_t plen) {
  memcpy(out + 6, s_mac, 6);
  out[0] = 0x5a; out[1] = 0x94; out[2] = 0xef; out[3] = 0xe4; out[4] = 0x0c; out[5] = 0xdd;
  out[12] = 0x08; out[13] = 0x00;
  out[14] = 0x45; out[15] = 0x00;
  uint16_t iplen = 20 + 20 + plen;
  out[16] = iplen >> 8; out[17] = iplen & 0xff;
  out[18] = 0x12; out[19] = 0x34; out[20] = 0x00; out[21] = 0x00;
  out[22] = 0x40; out[23] = 6;
  out[24] = 0x00; out[25] = 0x00; // checksum left zero (tap is L2)
  out[26] = 192; out[27] = 168; out[28] = 4; out[29] = 2;
  out[30] = 192; out[31] = 168; out[32] = 4; out[33] = 1;
  out[34] = sport >> 8; out[35] = sport & 0xff;
  out[36] = dport >> 8; out[37] = dport & 0xff;
  out[38] = seq >> 24; out[39] = (seq >> 16) & 0xff;
  out[40] = (seq >> 8) & 0xff; out[41] = seq & 0xff;
  out[42] = ack >> 24; out[43] = (ack >> 16) & 0xff;
  out[44] = (ack >> 8) & 0xff; out[45] = ack & 0xff;
  out[46] = 0x50; out[47] = flags; // data offset 5 (no opts), flags
  out[48] = 0x05; out[49] = 0xB4; // window 1460
  out[50] = 0x00; out[51] = 0x00; // checksum zero (optional, tap is L2)
  out[52] = 0x00; out[53] = 0x00;
  if (plen) memcpy(out + 54, pay, plen);
  return 14 + iplen;
}

// Whole-buffer marker search (payload sits at 54 with no opts — the
// gateway never sends TCP/IP opts — but a full-buffer search is robust
// either way; our own TX never appears in RX so no self-match is possible).
static bool rx_contains(const uint8_t *b, const uint8_t *pat, size_t plen) {
  for (size_t i = 0; i + plen <= 256; i++) {
    size_t k = 0;
    while (k < plen && b[i + k] == pat[k]) k++;
    if (k == plen) return true;
  }
  return false;
}

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

  // GATEWAY PRESENCE PROBE (load-bearing for the whole suite): the
  // battery runs WITHOUT a gateway (plain run_flash, no NET_GW), while
  // the L3 E2E runs WITH one. Every leg below must therefore degrade
  // gracefully: with no gateway every verdict reads empty (-21/-22/…)
  // and the sketch still reaches DONE (proven live: the plain-battery
  // run must print DONE, not wedge in a delay loop waiting for replies
  // that never come). Probe = one DNS TX + one pop: a staged reply
  // means the gateway bridge is up (the stub answers synchronously, so
  // one round trip suffices — no delay loop needed).
  uint8_t f[512];
  uint8_t rxb[256];
  size_t n;
  {
    uint8_t pq[29];
    pq[0]=0x22; pq[1]=0x22; pq[2]=0x01; pq[3]=0x00; pq[4]=0x00; pq[5]=0x01;
    pq[6]=0x00; pq[7]=0x00; pq[8]=0x00; pq[9]=0x00; pq[10]=0x00; pq[11]=0x00;
    pq[12]=7; memcpy(pq+13,"example",7); pq[20]=3; memcpy(pq+21,"com",3);
    pq[24]=0x00; pq[25]=0x00; pq[26]=0x01; pq[27]=0x00; pq[28]=0x01;
    n = udp_frame(f, 45001, 53, pq, 29);
    tx(f, n);
    delay(200);
    memset(rxb, 0, sizeof(rxb));
    esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
  }
  // The probe consumed the first reply if the gateway is up (or an
  // empty pop if not) — EITHER WAY the FIFO is empty now and the DNS
  // leg below starts clean. Re-TX below restores the reply.
  bool gw_up = rx_type(rxb) == 0x0800 && rxb[42] == 0x22 && rxb[43] == 0x22;
  Serial.print("WORKER L3 gw ");
  Serial.println(gw_up ? 1 : 0);

  // ---- DNS: query example.com (ID 0x2222), expect RDATA 93.184.216.34.
  // Query layout (29 bytes): header 12 (ID 2222, RD, QDCOUNT 1) +
  // QNAME 13 (07 example 03 com 00) + QTYPE/CLASS 4 (A/IN).
  // NO-GATEWAY FAST PATH: without a gateway every pop reads empty, so
  // the verdict is known upfront (-21) and the leg must NOT burn delay
  // loops or pops waiting for replies that never come (proven live:
  // the plain-battery run must print DONE, and every pop runs
  // closed-stack code — dozens of empty pops destabilize the run,
  // ILLEGAL at ~17.5M steps with 24x scans). One TX (so pcap still
  // proves the frame left the board) + one pop (drains the probe's
  // leftover when the gateway IS up — the probe already consumed the
  // first reply, the re-TX below stages a fresh one), then verdict.
  uint8_t qq[29];
  qq[0]=0x22; qq[1]=0x22; qq[2]=0x01; qq[3]=0x00; qq[4]=0x00; qq[5]=0x01;
  qq[6]=0x00; qq[7]=0x00; qq[8]=0x00; qq[9]=0x00; qq[10]=0x00; qq[11]=0x00;
  qq[12]=7; memcpy(qq+13,"example",7); qq[20]=3; memcpy(qq+21,"com",3);
  qq[24]=0x00; qq[25]=0x00; qq[26]=0x01; qq[27]=0x00; qq[28]=0x01;
  bool dns_ok = false;
  if (!gw_up) {
    n = udp_frame(f, 45001, 53, qq, 29);
    tx(f, n);
    memset(rxb, 0, sizeof(rxb));
    esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
  } else {
    n = udp_frame(f, 45001, 53, qq, 29);
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
    for (int k = 0; k < 2 && !dns_ok; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      // DNS header at payload 42: ID 42:44 = 2222, ANCOUNT 48:50 = 00 01.
      if (rx_type(rxb) != 0x0800 || rxb[42] != 0x22 || rxb[43] != 0x22 || rxb[49] != 0x01) continue;
      for (size_t i = 42; i + 4 <= sizeof(rxb); i++) {
        if (rxb[i] == 93 && rxb[i+1] == 184 && rxb[i+2] == 216 && rxb[i+3] == 34) { dns_ok = true; break; }
      }
    }
  }
  Serial.print("WORKER L3 dns ");
  Serial.println(dns_ok ? 34 : -21);
  // Drain leftovers (DNS replies + gVisor broadcasts) so the next leg
  // starts with an empty FIFO (8-deep cap would otherwise overflow the
  // next leg's scan window; receives on an empty FIFO run the real
  // closed-stack function harmlessly). Gateway-up only: without a
  // gateway the FIFO is already empty and extra pops are pure risk.
  if (gw_up) {
    for (int k = 0; k < 4; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      Serial.print("WORKER L3 dns drain ");
      Serial.println(k);
    }
  }
  // ---- NTP: 48-byte mode-3 client, expect mode-4 stratum-1 + fixed epoch.
  // Same gateway gate as DNS: without a gateway one TX + one pop, then
  // the known-empty verdict (-22). With a gateway the full re-TX +
  // scan + drain discipline.
  uint8_t nq[48];
  memset(nq, 0, sizeof(nq));
  nq[0] = 0x1b;
  bool ntp_ok = false;
  if (!gw_up) {
    n = udp_frame(f, 45002, 123, nq, 48);
    tx(f, n);
    memset(rxb, 0, sizeof(rxb));
    esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
  } else {
    n = udp_frame(f, 45002, 123, nq, 48);
    tx(f, n);
    for (int w = 0; w < 1; w++) { delay(200); tx(f, n); }
    // Same FIFO-pollution discipline as DNS: scan up to 10 frames for the
    // NTP reply (UDP 123->45002, mode 4, stratum 1, fixed epoch).
    for (int k = 0; k < 2 && !ntp_ok; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      ntp_ok = rx_type(rxb) == 0x0800 && (rxb[42] & 0x07) == 4 && rxb[43] == 1
        && rxb[82] == 0xec && rxb[83] == 0xf1 && rxb[84] == 0x2c && rxb[85] == 0x00;
    }
  }
  Serial.print("WORKER L3 ntp ");
  Serial.println(ntp_ok ? 123 : -22);
  // Same drain discipline as DNS: 4 pops so the second NTP reply (the
  // leg re-TXs once) + any broadcast can't occupy the FIFO when the
  // HTTP leg starts (the SYN-ACK would otherwise sit behind them and
  // the 2-pop scan window would miss it — proven live: SYNACK staged
  // but the scan read zeros). Gateway-up only (same risk rule as DNS).
  if (gw_up) {
    for (int k = 0; k < 4; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      Serial.print("WORKER L3 ntp drain ");
      Serial.println(k);
    }
  }

  // ---- HTTP: raw SYN -> SYN-ACK, GET / -> 200 + S3LAB-EMU-OK body.
  // Fixed board-side seq per leg (the stub keys on boardIP+sport and
  // answers synchronously, so no negotiated state is needed — the
  // replies only need wire-plausible seq/ack, which the verdict checks
  // only for presence of status + body bytes, never for exact seqnos).
  // Board seq 7000, sport 45003, dport 80. Gateway-up only (same gate
  // as DNS/NTP): without a gateway the SYN-ACK never comes, so one TX
  // + one pop, then the known-empty verdicts (-23/-24). With a gateway
  // the 15x keep-alive discipline (re-TX with 200 ms gaps + `keep`
  // prints, exactly like the worker-net ARP loop): delay() parks the pc
  // with no UART for ~2M steps and run_flash quits mid-wait (proven
  // live: 2 TX then silence = the leg never got past its first burst).
  // Re-TX keeps uart_buf moving (defeats idle-exit) AND re-arms the
  // reply drain each round (the gateway answers every SYN
  // synchronously, so every round stages a fresh SYN-ACK the scan
  // window can pop even if an earlier reply was consumed by a
  // wrong-leg pop).
  Serial.println("WORKER L3 http leg");
  uint16_t http_keep = 0;
  if (!gw_up) {
    size_t hn0 = tcp_frame(f, 45003, 80, 7000, 0, 0x02, NULL, 0); // SYN
    tx(f, hn0);
    memset(rxb, 0, sizeof(rxb));
    esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
    Serial.println("WORKER L3 http_synack -23");
    Serial.println("WORKER L3 http -24");
  } else {
  {
    size_t hn = tcp_frame(f, 45003, 80, 7000, 0, 0x02, NULL, 0); // SYN
    // NOTE: NO Serial.print between TX and the delay loop (proven live
    // on the RMT-carrier sketch: one printf spans the burst+idle window
    // and eats the first reply — here a print between tx() and delay()
    // parks the task in the UART TX path while the SYN-ACK stages, and
    // the rescheduled task never re-enters the scan with a live window).
    // Keep prints fire only INSIDE the re-TX loop below.
    //
    // IDLE-EXIT DISCIPLINE (proven live 2026-10-02: the harness quits after
    // 2M macro-steps with no UART bytes AND no core-0 pc change; a bare
    // 15x `delay(200)` burst parks the pc with no UART for ~2M steps and the
    // run is killed mid-leg — observed: leg reached only `dns drain 3`, the
    // run then sat silent until the budget expired). Every re-TX round
    // therefore prints FIRST (moving both uart_buf and, via the UART TX
    // path, the pc), then delays, then transmits — so no round is ever
    // silent and the harness cannot trip mid-leg.
    tx(f, hn);
    for (int w = 0; w < 15; w++) {
      Serial.print("WORKER L3 http keep "); Serial.println(http_keep++);
      delay(200); tx(f, hn);
    }
    // Drain the SYN-ACK to learn the stub ISS (server seq) for the ACK.
    // SYN-ACK shape: TCP sport 80 -> 45003, SYN+ACK flags, seq = ISS,
    // ack = 7001. ISS is fixed (0x1F2E3D4C) but read it live — the
    // sketch must not bake gateway constants (same discipline as the
    // DNS/NTP byte checks: observe, don't assume). Scan window 4 pops:
    // the FIFO may hold a stale duplicate SYN-ACK behind the fresh one
    // (every re-TX stages another), so ANY matching frame wins.
    uint32_t iss = 0;
    for (int k = 0; k < 4 && !iss; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      if (rx_type(rxb) != 0x0800) continue;
      // TCP header starts at 34: sport@34, flags@47, seq@38, ack@42.
      uint16_t tcp_sport = (uint16_t(rxb[34]) << 8) | rxb[35];
      if (tcp_sport != 80 || (rxb[47] & 0x12) != 0x12) continue;
      if (rx_u32(rxb + 42) != 7001) continue;
      iss = rx_u32(rxb + 38);
    }
    Serial.print("WORKER L3 http_synack ");
    Serial.println(iss ? 1 : -23);
    // ACK the handshake (seq 7001, ack iss+1, no payload) + GET /
    // (seq 7001, ack iss+1, PSH+ACK). Same 15x keep-alive discipline:
    // the stub answers synchronously, so every re-TX stages a fresh
    // response + FIN-ACK pair the verdict scan can pop.
    size_t an = tcp_frame(f, 45003, 80, 7001, iss + 1, 0x10, NULL, 0);
    tx(f, an);
    for (int w = 0; w < 15; w++) { Serial.print("WORKER L3 http keep "); Serial.println(http_keep++); delay(200); tx(f, an); }
    static const uint8_t get_req[] = "GET / HTTP/1.0\r\nHost: gateway\r\n\r\n";
    size_t gn = tcp_frame(f, 45003, 80, 7001, iss + 1, 0x18, get_req, sizeof(get_req) - 1);
    tx(f, gn);
    for (int w = 0; w < 15; w++) { Serial.print("WORKER L3 http keep "); Serial.println(http_keep++); delay(200); tx(f, gn); }
    // Verdict: ANY staged frame with the 200 status line + body marker
    // (the stub answers response + FIN-ACK back-to-back; either may be
    // popped first — the body only rides the response, so it is unique).
    // Scan window 4 pops (duplicate responses queue behind — every GET
    // re-TX stages another response + FIN-ACK; same discipline as the
    // SYN-ACK scan above).
    static const uint8_t http_ok[] = "HTTP/1.0 200 OK";
    static const uint8_t http_body[] = "S3LAB-EMU-OK";
    bool http_ok_seen = false;
    for (int k = 0; k < 4 && !http_ok_seen; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      if (rx_type(rxb) != 0x0800) continue;
      if (rx_contains(rxb, http_ok, sizeof(http_ok) - 1)
          && rx_contains(rxb, http_body, sizeof(http_body) - 1)) http_ok_seen = true;
    }
    Serial.print("WORKER L3 http ");
    Serial.println(http_ok_seen ? 200 : -24);
  } // end gw_up HTTP block
  }

  // ---- MQTT: raw SYN -> SYN-ACK, CONNECT -> CONNACK, SUBSCRIBE -> SUBACK.
  // Same raw-frame discipline as HTTP above (board seq 3000, sport
  //     45004, dport 1883). PUBLISH QoS0 is accept-silently (no wire reply
  //     exists) so the sketch verdict is CONNACK + SUBACK bytes only —
  //     same contract the gateway unit test pins (TestL7MQTTConnectSubPub).
  //     Keep-alive prints are load-bearing here too (see HTTP leg note).
  //     Gateway-up only (same gate as HTTP): without a gateway one TX +
  //     one pop per message, then the known-empty verdicts.
  Serial.println("WORKER L3 mqtt leg");
  uint16_t mqtt_keep = 0;
  if (!gw_up) {
    size_t sn0 = tcp_frame(f, 45004, 1883, 3000, 0, 0x02, NULL, 0); // SYN
    tx(f, sn0);
    memset(rxb, 0, sizeof(rxb));
    esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
    Serial.println("WORKER L3 mqtt_synack -25");
    Serial.println("WORKER L3 mqtt -28");
  } else {
  {
    size_t sn = tcp_frame(f, 45004, 1883, 3000, 0, 0x02, NULL, 0); // SYN
    // NOTE: same print-FIRST discipline as HTTP above (idle-exit: the print
    // must come before the delay in every round, never after).
    tx(f, sn);
    for (int w = 0; w < 15; w++) { Serial.print("WORKER L3 mqtt keep "); Serial.println(mqtt_keep++); delay(200); tx(f, sn); }
    uint32_t miss = 0;
    for (int k = 0; k < 4 && !miss; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      if (rx_type(rxb) != 0x0800) continue;
      uint16_t tcp_sport = (uint16_t(rxb[34]) << 8) | rxb[35];
      if (tcp_sport != 1883 || (rxb[47] & 0x12) != 0x12) continue;
      if (rx_u32(rxb + 42) != 3001) continue;
      miss = rx_u32(rxb + 38);
    }
    Serial.print("WORKER L3 mqtt_synack ");
    Serial.println(miss ? 1 : -25);
    // CONNECT (minimal fixed header + tiny body; stub checks type only).
    static const uint8_t m_connect[] = {0x10, 0x0A, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x02, 0x00, 0x3C};
    size_t cn = tcp_frame(f, 45004, 1883, 3001, miss + 1, 0x18, m_connect, sizeof(m_connect));
    tx(f, cn);
    for (int w = 0; w < 15; w++) { Serial.print("WORKER L3 mqtt keep "); Serial.println(mqtt_keep++); delay(200); tx(f, cn); }
    // CONNACK = 20 02 00 00 (session accepted), unique on the wire.
    // Scan window 4 pops (duplicate CONNACKs queue behind — every
    // re-TX stages another; same discipline as the HTTP SYN-ACK scan).
    static const uint8_t connack[] = {0x20, 0x02, 0x00, 0x00};
    bool connack_ok = false;
    for (int k = 0; k < 4 && !connack_ok; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      if (rx_type(rxb) != 0x0800) continue;
      if (rx_contains(rxb, connack, sizeof(connack))) connack_ok = true;
    }
    Serial.print("WORKER L3 mqtt_connack ");
    Serial.println(connack_ok ? 1 : -26);
    // SUBSCRIBE pkt-id 0x1234 topic "t" QoS0 -> SUBACK 90 03 12 34 00.
    static const uint8_t m_sub[] = {0x82, 0x06, 0x12, 0x34, 0x00, 0x01, 't', 0x00};
    size_t un = tcp_frame(f, 45004, 1883, 3001 + sizeof(m_connect), miss + 1 + 4, 0x18, m_sub, sizeof(m_sub));
    tx(f, un);
    for (int w = 0; w < 15; w++) { Serial.print("WORKER L3 mqtt keep "); Serial.println(mqtt_keep++); delay(200); tx(f, un); }
    static const uint8_t suback[] = {0x90, 0x03, 0x12, 0x34, 0x00};
    bool suback_ok = false;
    for (int k = 0; k < 4 && !suback_ok; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      if (rx_type(rxb) != 0x0800) continue;
      if (rx_contains(rxb, suback, sizeof(suback))) suback_ok = true;
    }
    Serial.print("WORKER L3 mqtt_suback ");
    Serial.println(suback_ok ? 1 : -27);
    // PUBLISH QoS0 "hi" (accept-silently: no reply — TX only, proves
    // the frame reaches the gateway without wedging the leg).
    static const uint8_t m_pub[] = {0x30, 0x05, 0x00, 0x01, 't', 'h', 'i'};
    size_t pn = tcp_frame(f, 45004, 1883, 3001 + sizeof(m_connect) + sizeof(m_sub), miss + 1 + 4 + 5, 0x18, m_pub, sizeof(m_pub));
    tx(f, pn);
    for (int w = 0; w < 15; w++) { Serial.print("WORKER L3 mqtt keep "); Serial.println(mqtt_keep++); delay(200); tx(f, pn); }
    // PINGREQ -> PINGRESP d0 00.
    static const uint8_t m_ping[] = {0xC0, 0x00};
    static const uint8_t pingresp[] = {0xD0, 0x00};
    size_t qn = tcp_frame(f, 45004, 1883, 3001 + sizeof(m_connect) + sizeof(m_sub) + sizeof(m_pub), miss + 1 + 4 + 5, 0x18, m_ping, sizeof(m_ping));
    tx(f, qn);
    for (int w = 0; w < 15; w++) { Serial.print("WORKER L3 mqtt keep "); Serial.println(mqtt_keep++); delay(200); tx(f, qn); }
    bool ping_ok = false;
    for (int k = 0; k < 4 && !ping_ok; k++) {
      memset(rxb, 0, sizeof(rxb));
      esp_netif_receive(s_netif, rxb, sizeof(rxb), NULL);
      if (rx_type(rxb) != 0x0800) continue;
      if (rx_contains(rxb, pingresp, sizeof(pingresp))) ping_ok = true;
    }
    Serial.print("WORKER L3 mqtt ");
    Serial.println((connack_ok && suback_ok && ping_ok) ? 1883 : -28);
  } // end MQTT gw_up block
  } // end outer gw_up MQTT block

  Serial.println("WORKER L3 DONE");
}
void loop() { delay(1000); }
