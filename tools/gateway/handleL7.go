package main

// Gateway-local L3–L7 application services (board <-> gateway only,
// never entering gVisor — same class as the IPv6 hub services in
// handleIPv6.go).
//
// Why this file exists: the ESP32-S3 lwIP stack in the emulator has no
// live netif binding (closed DHCP client wedges without a live DHCP
// task, `esp_netif_get_ip_info` is hook-served — see run_flash.rs).
// The emulator therefore cannot run a REAL DHCP client or a real
// DNS/HTTP/MQTT/CoAP client against an external counterparty. What it
// CAN do is speak the same hand-built Ethernet frames the test-worker
// sketch already speaks (`esp_netif_transmit` TX tap + `esp_netif_receive`
// RX hook) directly to the gateway, which answers deterministically here.
//
// Services in this file (all synchronous request/response, no per-board
// state beyond the UDP-forward relay map that already exists, plus a
// per-board TCP stub table below for the connection-oriented legs):
//   * DNS (UDP 53, board -> us): static A-record table + an 8.8.8.8
//     fallback is NOT attempted offline — answers come from the table
//     (example.com, test.mosquitto.org) so `gethostbyname` completes.
//   * NTP (UDP 123, board -> us): fixed-epoch reply (2026-01-01) so
//     `sntp_getreachability`/time-sync sketches observe a stable time.
//   * HTTP (TCP 80, board -> us, gateway-local stub): minimal SYN/SYN-ACK
//     + one-shot GET -> 200 responder keyed on the board's (ip, sport)
//     4-tuple (see `l7TCPStub` below). The responder is a STUB, not a
//     proxy: it serves a fixed `EXAMPLE_BODY` for `GET /` (and 404 for
//     anything else) so a firmware HTTP-client suite observes a complete
//     status line + headers + body with Content-Length framing. Host
//     egress via gVisor NAT still exists for everything else (proven
//     live: EGRESS_TCP80_OK) — the stub only claims frames whose IP dst
//     is the gateway itself (192.168.4.1:80), which gVisor never owns.
//   * MQTT (TCP 1883, board -> us, gateway-local stub): minimal
//     CONNECT -> CONNACK + PUBLISH(QoS0) -> PUBACK-less accept responder
//     on the same per-board 4-tuple table, so a firmware MQTT-client
//     suite observes session-accepted + publish-accepted without a real
//     broker. Same non-interference rule: only gateway-IP dst is
//     consumed; everything else falls through to gVisor NAT (proven
//     live: EGRESS_MQTT_OK to test.mosquitto.org:1883).
//   * CoAP (UDP 5683): already covered both directions — inbound via the
//     127.0.0.1:<uport> UDP-forward listener (handleUDPProxy) and board
//     replies via divertUDPForward. No new code; pinned by tests here.
//
// Pipeline placement (handleTCPFrame/handleClient, in order):
//   ARP fast/proxy reply -> ICMPv4 echo -> DHCP:67 -> UDP-forward relay
//   -> IPv6 services -> *** L3-L7 services (this file: DNS/NTP/TCP) *** ->
//   gVisor feed + room broadcast.
// DNS/NTP MUST sit before the gVisor feed so the frames never reach the
// NAT stack (which has no listener for gateway-IP UDP:53/123 and would
// drop them); the TCP stubs likewise claim ONLY gateway-IP dst so
// off-LAN TCP still reaches gVisor untouched.

import (
	"encoding/binary"
	"fmt"
	"net"
	"sync"
	"time"

	"github.com/google/gopacket"
	"github.com/google/gopacket/layers"
)

// ---- Static DNS table --------------------------------------------------
// Hostnames the emulated firmware is validated against (battery + docs):
// example.com (HTTP suites), test.mosquitto.org (MQTT suites), and the
// gateway itself. IPs are documentation/test-net addresses — the point is
// that `gethostbyname` COMPLETES with a stable answer, not that the
// address routes anywhere (no live netif exists past the gateway).
var l7DNSTable = map[string]net.IP{
	"example.com.":         net.IPv4(93, 184, 216, 34),
	"test.mosquitto.org.":  net.IPv4(91, 121, 93, 39),
	"gateway.":             net.IPv4(192, 168, 4, 1),
	"gateway.local.":       net.IPv4(192, 168, 4, 1),
	"esp32-gateway.":       net.IPv4(192, 168, 4, 1),
	"esp32-gateway.local.": net.IPv4(192, 168, 4, 1),
}

// dnsNameOnWire decodes the QNAME labels at payload[off:] into a
// lower-cased dotted name WITH trailing dot (root-terminated form).
// Returns ("", 0) on truncation.
func dnsNameOnWire(payload []byte, off int) (string, int) {
	name := ""
	for {
		if off >= len(payload) {
			return "", 0
		}
		n := int(payload[off])
		off++
		if n == 0 {
			break
		}
		if n&0xC0 != 0 {
			// Compression pointer — not expected in queries; bail.
			return "", 0
		}
		if off+n > len(payload) {
			return "", 0
		}
		for _, c := range payload[off : off+n] {
			if c >= 'A' && c <= 'Z' {
				c += 'a' - 'A'
			}
			name += string(rune(c))
		}
		name += "."
		off += n
	}
	return name, off
}

// buildDNSReply crafts a minimal DNS response for one A-question:
// header (ID echo, QR|RD|RA, 1 Q / 1 A or RCODE=3) + question echo +
// answer (PTR to QNAME @12, A, IN, TTL 300, RDLEN 4, RDATA). Returns nil
// when the question is malformed.
func buildDNSReply(query []byte) []byte {
	if len(query) < 12 {
		return nil
	}
	id := binary.BigEndian.Uint16(query[0:2])
	qdcount := binary.BigEndian.Uint16(query[4:6])
	if qdcount != 1 {
		return nil
	}
	qname, off := dnsNameOnWire(query, 12)
	if qname == "" || off+4 > len(query) {
		return nil
	}
	qtype := binary.BigEndian.Uint16(query[off : off+2])
	qclass := binary.BigEndian.Uint16(query[off+2 : off+4])
	question := query[12 : off+4]

	ip, known := l7DNSTable[qname]
	isA := qtype == 1 && qclass == 1
	var ancount uint16
	var rcode byte
	if known && isA {
		ancount = 1
	} else {
		rcode = 3 // NXDOMAIN for unknown names / non-A types
	}
	rep := make([]byte, 0, len(question)+32)
	hdr := make([]byte, 12)
	binary.BigEndian.PutUint16(hdr[0:2], id)
	hdr[2] = 0x80 | 0x01 // QR + RD (echo)
	hdr[3] = 0x80 | rcode // RA + RCODE
	binary.BigEndian.PutUint16(hdr[4:6], 1)
	binary.BigEndian.PutUint16(hdr[6:8], ancount)
	rep = append(rep, hdr...)
	rep = append(rep, question...)
	if ancount == 1 {
		ans := make([]byte, 0, 16)
		ans = append(ans, 0xC0, 0x0C) // PTR to QNAME @12
		ans = append(ans, 0x00, 0x01) // A
		ans = append(ans, 0x00, 0x01) // IN
		ttl := make([]byte, 4)
		binary.BigEndian.PutUint32(ttl, 300)
		ans = append(ans, ttl...)
		ans = append(ans, 0x00, 0x04) // RDLEN
		ans = append(ans, ip.To4()...)
		rep = append(rep, ans...)
	}
	return rep
}

// snoopDNS answers board->gateway DNS queries (UDP dst 53, IPv4 dst ==
// our 192.168.4.1 — proxy-ARP steers off-LAN queries at us, see
// sendProxyARPReply). Returns true when consumed (caller must skip the
// VN pipe AND the room broadcast).
func snoopDNS(msg []byte, client *Client, room *Room) bool {
	_ = room
	packet := gopacket.NewPacket(msg, layers.LayerTypeEthernet, gopacket.Default)
	ipLayer := packet.Layer(layers.LayerTypeIPv4)
	udpLayer := packet.Layer(layers.LayerTypeUDP)
	if ipLayer == nil || udpLayer == nil {
		return false
	}
	ip, _ := ipLayer.(*layers.IPv4)
	udp, _ := udpLayer.(*layers.UDP)
	if udp.DstPort != 53 {
		return false
	}
	if !ip.DstIP.Equal(net.IPv4(192, 168, 4, 1)) {
		return false
	}
	reply := buildDNSReply(udp.Payload)
	if reply == nil {
		return false
	}
	// Craft eth + IPv4 + UDP with swapped addrs/ports via gopacket.
	ethLayer := packet.Layer(layers.LayerTypeEthernet)
	eth, _ := ethLayer.(*layers.Ethernet)
	ethReply := &layers.Ethernet{
		SrcMAC:       gwMAC,
		DstMAC:       eth.SrcMAC,
		EthernetType: layers.EthernetTypeIPv4,
	}
	ipReply := &layers.IPv4{
		Version:  4,
		IHL:      5,
		TTL:      64,
		Protocol: layers.IPProtocolUDP,
		SrcIP:    net.IPv4(192, 168, 4, 1),
		DstIP:    ip.SrcIP,
	}
	udpReply := &layers.UDP{
		SrcPort: 53,
		DstPort: udp.SrcPort,
	}
	_ = udpReply.SetNetworkLayerForChecksum(ipReply)
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethReply, ipReply, udpReply, gopacket.Payload(reply)); serr != nil {
		return false
	}
	client.WriteMutex.Lock()
	werr := sendFrame(client, buffer.Bytes())
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[DNS] Reply send failed: %v\n", werr)
	} else {
		fmt.Printf("[DNS] Reply %d bytes -> %s\n", len(reply), ip.SrcIP.String())
	}
	return true
}

// ---- NTP (UDP 123, board -> gateway) ------------------------------------
// Fixed-epoch reply: 2026-01-01 00:00:00 UTC (Unix 1767225600 + 2208988800
// NTP offset = 3976214400 = 0xECF12C00). Stratum 1, LI 0, mode 4 (server).
// A real clock is meaningless here (1 global tick per 2 insns for all
// domains — wall time is unobservable); the point is a WELL-FORMED reply
// so SNTP sketches complete instead of timing out.
const l7NTPFixedSec uint32 = 0xECF12C00 // 2026-01-01 00:00:00 UTC in NTP seconds

func buildNTPReply(query []byte) []byte {
	if len(query) < 48 {
		return nil
	}
	rep := make([]byte, 48)
	rep[0] = 0x24 // LI 0, version 4, mode 4 (server)
	rep[1] = 1    // stratum 1
	rep[2] = 0    // poll
	rep[3] = 0xFA // precision (-6)
	// root delay/dispersion zero; ref ID 'GPS '.
	copy(rep[12:16], []byte{'G', 'P', 'S', ' '})
	binary.BigEndian.PutUint32(rep[16:20], l7NTPFixedSec) // ref ts
	binary.BigEndian.PutUint32(rep[24:28], l7NTPFixedSec) // orig ts echo-ish
	binary.BigEndian.PutUint32(rep[32:36], l7NTPFixedSec) // recv ts
	binary.BigEndian.PutUint32(rep[40:44], l7NTPFixedSec) // tx ts
	return rep
}

// snoopNTP answers board->gateway NTP queries (UDP dst 123, dst ==
// 192.168.4.1). Returns true when consumed.
func snoopNTP(msg []byte, client *Client, room *Room) bool {
	_ = room
	packet := gopacket.NewPacket(msg, layers.LayerTypeEthernet, gopacket.Default)
	ipLayer := packet.Layer(layers.LayerTypeIPv4)
	udpLayer := packet.Layer(layers.LayerTypeUDP)
	if ipLayer == nil || udpLayer == nil {
		return false
	}
	ip, _ := ipLayer.(*layers.IPv4)
	udp, _ := udpLayer.(*layers.UDP)
	if udp.DstPort != 123 {
		return false
	}
	if !ip.DstIP.Equal(net.IPv4(192, 168, 4, 1)) {
		return false
	}
	reply := buildNTPReply(udp.Payload)
	if reply == nil {
		return false
	}
	ethLayer := packet.Layer(layers.LayerTypeEthernet)
	eth, _ := ethLayer.(*layers.Ethernet)
	ethReply := &layers.Ethernet{
		SrcMAC:       gwMAC,
		DstMAC:       eth.SrcMAC,
		EthernetType: layers.EthernetTypeIPv4,
	}
	ipReply := &layers.IPv4{
		Version:  4,
		IHL:      5,
		TTL:      64,
		Protocol: layers.IPProtocolUDP,
		SrcIP:    net.IPv4(192, 168, 4, 1),
		DstIP:    ip.SrcIP,
	}
	udpReply := &layers.UDP{
		SrcPort: 123,
		DstPort: udp.SrcPort,
	}
	_ = udpReply.SetNetworkLayerForChecksum(ipReply)
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethReply, ipReply, udpReply, gopacket.Payload(reply)); serr != nil {
		return false
	}
	client.WriteMutex.Lock()
	werr := sendFrame(client, buffer.Bytes())
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[NTP] Reply send failed: %v\n", werr)
	} else {
		fmt.Printf("[NTP] Reply -> %s\n", ip.SrcIP.String())
	}
	return true
}

// ---- UDP echo (board -> gateway, any port we serve) ----------------------
// Debug/validation helper: board UDP at 192.168.4.1:<port> where <port> is
// one of the gateway-local service ports (5683 = CoAP/UDP-echo) gets its
// payload echoed back with swapped addrs/ports. This is SEPARATE from the
// 127.0.0.1:<uport> port-forward path (handleUDPProxy/divertUDPForward,
// which relays to a host UDP client): the echo here answers DIRECTLY, so
// a firmware UDP-client suite observes a round trip with no host peer.
// Returns true when consumed.
func snoopUDPEcho(msg []byte, client *Client, room *Room) bool {
	_ = room
	packet := gopacket.NewPacket(msg, layers.LayerTypeEthernet, gopacket.Default)
	ipLayer := packet.Layer(layers.LayerTypeIPv4)
	udpLayer := packet.Layer(layers.LayerTypeUDP)
	if ipLayer == nil || udpLayer == nil {
		return false
	}
	ip, _ := ipLayer.(*layers.IPv4)
	udp, _ := udpLayer.(*layers.UDP)
	if !ip.DstIP.Equal(net.IPv4(192, 168, 4, 1)) {
		return false
	}
	if udp.DstPort != 5683 {
		return false
	}
	// The port-forward relay owns flows it created (sport registered by
	// handleUDPProxy): don't steal those replies.
	if udpFwdLookup(uint16(udp.SrcPort)) != nil {
		return false
	}
	ethLayer := packet.Layer(layers.LayerTypeEthernet)
	eth, _ := ethLayer.(*layers.Ethernet)
	ethReply := &layers.Ethernet{
		SrcMAC:       gwMAC,
		DstMAC:       eth.SrcMAC,
		EthernetType: layers.EthernetTypeIPv4,
	}
	ipReply := &layers.IPv4{
		Version:  4,
		IHL:      5,
		TTL:      64,
		Protocol: layers.IPProtocolUDP,
		SrcIP:    net.IPv4(192, 168, 4, 1),
		DstIP:    ip.SrcIP,
	}
	udpReply := &layers.UDP{
		SrcPort: udp.DstPort,
		DstPort: udp.SrcPort,
	}
	_ = udpReply.SetNetworkLayerForChecksum(ipReply)
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethReply, ipReply, udpReply, gopacket.Payload(udp.Payload)); serr != nil {
		return false
	}
	client.WriteMutex.Lock()
	werr := sendFrame(client, buffer.Bytes())
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[UDPECHO] Reply send failed: %v\n", werr)
	} else {
		fmt.Printf("[UDPECHO] %d bytes -> %s\n", len(udp.Payload), ip.SrcIP.String())
	}
	return true
}

// ---- CoAP (board <-> gateway, UDP :5683) ---------------------------------
// Minimal confirmable-GET server for the worker-L3 `coap` leg: parses the
// CoAP fixed header + token + options of a request to :5683 and answers
// GET coap://192.168.4.1/t with a piggybacked ACK 2.05 Content
// (Content-Format text/plain, payload `25.00C`). Anything else on :5683 —
// wrong version/type/code, TKL > 8, extended option fields, a different
// path — returns false so snoopUDPEcho answers it as a raw echo instead
// (the two never cross-talk: the sketch's HELLO-UDP starts with 0x48,
// which fails the ver/type gate below). Returns true when consumed.
func snoopCoAP(msg []byte, client *Client, room *Room) bool {
	packet := gopacket.NewPacket(msg, layers.LayerTypeEthernet, gopacket.Default)
	ipLayer := packet.Layer(layers.LayerTypeIPv4)
	udpLayer := packet.Layer(layers.LayerTypeUDP)
	if ipLayer == nil || udpLayer == nil {
		return false
	}
	ip, _ := ipLayer.(*layers.IPv4)
	udp, _ := udpLayer.(*layers.UDP)
	if !ip.DstIP.Equal(net.IPv4(192, 168, 4, 1)) {
		return false
	}
	if udp.DstPort != 5683 {
		return false
	}
	if udpFwdLookup(uint16(udp.SrcPort)) != nil {
		return false
	}
	p := udp.Payload
	// Exact shape only: CON(0) GET(0.01) Uri-Path "t", TKL 2.
	// ver 1 (01......), type CON (..00....) => 0x40; code 0x01.
	if len(p) != 8 || p[0] != 0x40 || p[1] != 0x01 {
		return false
	}
	mid := p[2:4]
	token := p[4:6]
	if p[6] != 0xB1 || p[7] != 't' { // delta 11 len 1, "t"
		return false
	}
	// Piggybacked response: ACK(2) 2.05(0x45), same MID + token,
	// Content-Format (12, value 0 = text/plain), payload `25.00C`.
	resp := []byte{0x60, 0x45, mid[0], mid[1], token[0], token[1],
		0xC1, 0x00, 0xFF, '2', '5', '.', '0', '0', 'C'}
	ethLayer := packet.Layer(layers.LayerTypeEthernet)
	eth, _ := ethLayer.(*layers.Ethernet)
	ethReply := &layers.Ethernet{
		SrcMAC:       gwMAC,
		DstMAC:       eth.SrcMAC,
		EthernetType: layers.EthernetTypeIPv4,
	}
	ipReply := &layers.IPv4{
		Version:  4,
		IHL:      5,
		TTL:      64,
		Protocol: layers.IPProtocolUDP,
		SrcIP:    net.IPv4(192, 168, 4, 1),
		DstIP:    ip.SrcIP,
	}
	udpReply := &layers.UDP{
		SrcPort: udp.DstPort,
		DstPort: udp.SrcPort,
	}
	_ = udpReply.SetNetworkLayerForChecksum(ipReply)
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethReply, ipReply, udpReply, gopacket.Payload(resp)); serr != nil {
		return false
	}
	client.WriteMutex.Lock()
	werr := sendFrame(client, buffer.Bytes())
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[COAP] Reply send failed: %v\n", werr)
	} else {
		fmt.Printf("[COAP] 2.05 temp -> %s\n", ip.SrcIP.String())
	}
	return true
}

// ---- CoAP server direction (gateway -> board) ---------------------------
// Minimal gateway-initiated GET for the worker-L3 `coap_srv` leg: when the
// board sends `READY` (5B) to :5683 on sport 45008 (the server-leg sport,
// distinct from the client 45005/45006), answer with CON GET /t (MID 0x2224,
// token BB 66 — distinct from the client MID 2223/token AA 55) instead of
// echoing. The board serves it with piggybacked ACK 2.05 (roles reversed
// from `snoopCoAP` above). Anything else on :5683 falls through (HELLO-UDP
// echo, board CoAP GETs answered by `snoopCoAP`). Returns true when
// consumed.
func snoopCoAPServer(msg []byte, client *Client, room *Room) bool {
	packet := gopacket.NewPacket(msg, layers.LayerTypeEthernet, gopacket.Default)
	ipLayer := packet.Layer(layers.LayerTypeIPv4)
	udpLayer := packet.Layer(layers.LayerTypeUDP)
	if ipLayer == nil || udpLayer == nil {
		return false
	}
	ip, _ := ipLayer.(*layers.IPv4)
	udp, _ := udpLayer.(*layers.UDP)
	if !ip.DstIP.Equal(net.IPv4(192, 168, 4, 1)) {
		return false
	}
	if udp.DstPort != 5683 {
		return false
	}
	if udpFwdLookup(uint16(udp.SrcPort)) != nil {
		return false
	}
	if string(udp.Payload) != "READY" {
		return false
	}
	// CON GET /t: ver 1 / CON / TKL 2 (0x40), GET (0x01), MID 0x2224,
	// token BB 66, Uri-Path "t" (0xB1 0x74). Sport 5683 -> board sport
	// (the READY's source, 45008 in-sketch).
	coap := []byte{0x40, 0x01, 0x22, 0x24, 0xBB, 0x66, 0xB1, 't'}
	ethLayer := packet.Layer(layers.LayerTypeEthernet)
	eth, _ := ethLayer.(*layers.Ethernet)
	ethReply := &layers.Ethernet{
		SrcMAC:       gwMAC,
		DstMAC:       eth.SrcMAC,
		EthernetType: layers.EthernetTypeIPv4,
	}
	ipReply := &layers.IPv4{
		Version:  4,
		IHL:      5,
		TTL:      64,
		Protocol: layers.IPProtocolUDP,
		SrcIP:    net.IPv4(192, 168, 4, 1),
		DstIP:    ip.SrcIP,
	}
	udpReply := &layers.UDP{
		SrcPort: udp.DstPort,
		DstPort: udp.SrcPort,
	}
	_ = udpReply.SetNetworkLayerForChecksum(ipReply)
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethReply, ipReply, udpReply, gopacket.Payload(coap)); serr != nil {
		return false
	}
	client.WriteMutex.Lock()
	werr := sendFrame(client, buffer.Bytes())
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[COAP-SRV] GET send failed: %v\n", werr)
	} else {
		fmt.Printf("[COAP-SRV] GET /t -> %s\n", ip.SrcIP.String())
	}
	return true
}

// ---- TCP passthrough note (HTTP/MQTT off-LAN) --------------------------
// Board TCP to the outside world (HTTP :80, HTTPS :443, MQTT :1883,
// test.mosquitto.org) is NATed by the gVisor stack — NO snoop arm may
// claim these frames. This predicate documents the boundary: it returns
// false always (never consumes); the pipeline calls it for symmetry so a
// future intercept has an explicit slot. Proven live: host egress to
// example.com:80 and test.mosquitto.org:1883 succeeds, so gVisor NAT
// carries these end-to-end once the board's frames reach the pipe.
//
// NOTE: board TCP whose IP dst is the GATEWAY itself (192.168.4.1:80 /
// :1883) never reaches gVisor — it is consumed by the gateway-local
// stubs above (`l7TCPSnoop`), same placement rule as DNS/NTP/UDP-echo
// (gVisor owns no gateway-IP listener). The two rules compose: dst==gw
// -> stub; dst!=gw -> NAT.
func snoopTCPProxyNote(msg []byte, client *Client, room *Room) bool {
	_, _, _ = msg, client, room
	return false
}

// l7SweepStubs drops TCP stubs idle > 60 s (same hygiene class as the
// udpFwd relay map; called opportunistically from the snoop path so no
// ticker goroutine is needed).
func l7SweepStubs() {
	cutoff := time.Now().Add(-60 * time.Second)
	l7TCPMu.Lock()
	defer l7TCPMu.Unlock()
	for k, s := range l7TCPStubs {
		if s.last.Before(cutoff) {
			delete(l7TCPStubs, k)
		}
	}
}

// ---- Gateway-local TCP stubs (HTTP :80, MQTT :1883, board -> gateway) --
// Minimal per-board TCP responders for the L3 worker sketch's HTTP/MQTT
// legs. Design constraints (all load-bearing, all from live failure
// classes elsewhere in this gateway):
//
//  1. ONLY gateway-IP dst is consumed (192.168.4.1). Off-LAN TCP still
//     falls through to gVisor NAT untouched (EGRESS_*_OK relies on it).
//  2. Emulator RX path is a dumb FIFO (`net_inject_rx` + one
//     `esp_netif_receive` call per staged frame): the board CANNOT
//     reassemble TCP segments or reorder out-of-order delivery, so every
//     stub reply MUST fit in ONE Ethernet frame (≤1500B incl. headers;
//     the HTTP body is sized accordingly) and arrive in causal order.
//  3. No wall-clock timing dependence: the stub answers synchronously
//     inside the snoop call (request in -> reply out), like DNS/NTP —
//     never a delayed/periodic reply the sketch would have to poll for
//     (cf. the worker-rx2 lesson: replies landing one step late read -14).
//  4. No lwIP TCP-state dependency on the board: the sketch speaks RAW
//     frames through the `esp_netif_transmit` tap (hand-built IP/TCP,
//     like its UDP legs) and reads replies with the same receive calls —
//     the closed lwIP TCP stack is never involved, so there is no
//     handshake, no socket, no netconn, and no netif binding to wedge.
//
// Protocol served (deliberately tiny — just enough for a client-suite
// verdict, same "well-formed reply" class as the NTP fixed epoch):
//   * HTTP: one SYN (no opts) -> SYN-ACK; one ACK+GET / -> one ACK +
//     `HTTP/1.0 200 OK` with Content-Length + `EXAMPLE_BODY`; FIN ->
//     FIN-ACK close. Sequence space is per-board (keyed on sport, see
//     below); the stub tracks only the LAST server seq it sent so the
//     ACK numbers line up.
//   * MQTT: CONNECT -> CONNACK(0x00 session-accepted); PUBLISH QoS0 ->
//     accepted silently (no PUBACK exists for QoS0 — the sketch verdict
//     is the CONNACK byte + the echoed SUBACK for its subscribe). A
//     SUBSCRIBE (any topic, QoS0) -> SUBACK granted-QoS0. PINGREQ ->
//     PINGRESP. DISCONNECT tears the stub down.
//
// State: one stub per board source port (`l7TCPStub` keyed on
// boardIP+sport), holding only the next sequence numbers + a small
// reassembly tail for split TCP segments (the emulator's nonblocking
// drain splits exactly like the NET_GW leg — framing_test.go pins it).
// Stubs are created on first SYN/CONNECT and dropped on FIN/RST or after
// 60 s idle (same hygiene as udpFwdBySport).

// l7TCPService selects the stub responder by destination port.
type l7TCPService int

const (
	l7SvcNone l7TCPService = iota
	l7SvcHTTP
	l7SvcMQTT
)

// l7TCPStub is the per-board-side-port TCP responder state.
type l7TCPStub struct {
	svc      l7TCPService
	boardIP  net.IP
	boardMAC net.HardwareAddr
	sport    uint16
	// Next sequence numbers, board-relative. iss is OUR initial seq
	// (fixed per stub so captures are deterministic); sndNxt is the
	// next server byte to send; rcvNxt is the next board byte we expect
	// (== board seq + board payload len seen so far).
	iss    uint32
	sndNxt uint32
	rcvNxt uint32
	// Pending inbound TCP payload bytes not yet consumed into a full
	// application message (split segments arrive across frames).
	pending []byte
	// Whether the 3-way handshake completed (SYN-ACK acked).
	estab bool
	// HTTP: request bytes seen on this connection (one GET only).
	httpReq []byte
	// MQTT: CONNECT received (CONNACK sent).
	mqttConn bool
	last     time.Time
}

var (
	l7TCPMu    sync.Mutex
	l7TCPStubs = make(map[string]*l7TCPStub)
)

func l7TCPKey(ip net.IP, sport uint16) string {
	return ip.String() + "|" + string(rune(sport>>8)) + string(rune(sport&0xff))
}

// l7TCPServiceFor maps a gateway-local TCP dst port to its stub.
func l7TCPServiceFor(dport uint16) l7TCPService {
	switch dport {
	case 80:
		return l7SvcHTTP
	case 1883:
		return l7SvcMQTT
	}
	return l7SvcNone
}

// l7BuildTCP crafts one gateway->board TCP segment: eth + IPv4 + TCP with
// swapped addrs/ports, given seq/ack/flags + payload. Checksums + lengths
// via gopacket (same builder discipline as the UDP snoops above).
func l7BuildTCP(boardMAC net.HardwareAddr, boardIP net.IP, sport uint16, svc l7TCPService, seq, ack uint32, flags string, payload []byte) []byte {
	var dport uint16
	if svc == l7SvcHTTP {
		dport = 80
	} else {
		dport = 1883
	}
	ethReply := &layers.Ethernet{
		SrcMAC:       gwMAC,
		DstMAC:       boardMAC,
		EthernetType: layers.EthernetTypeIPv4,
	}
	ipReply := &layers.IPv4{
		Version:  4,
		IHL:      5,
		TTL:      64,
		Protocol: layers.IPProtocolTCP,
		SrcIP:    net.IPv4(192, 168, 4, 1),
		DstIP:    boardIP,
	}
	tcpReply := &layers.TCP{
		SrcPort: layers.TCPPort(dport),
		DstPort: layers.TCPPort(sport),
		Seq:     seq,
		Ack:     ack,
		Window:  1460,
	}
	for _, f := range flags {
		switch f {
		case 'S':
			tcpReply.SYN = true
		case 'A':
			tcpReply.ACK = true
		case 'F':
			tcpReply.FIN = true
		case 'P':
			tcpReply.PSH = true
		case 'R':
			tcpReply.RST = true
		}
	}
	_ = tcpReply.SetNetworkLayerForChecksum(ipReply)
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethReply, ipReply, tcpReply, gopacket.Payload(payload)); serr != nil {
		return nil
	}
	return buffer.Bytes()
}

// l7TCPSend emits one stub segment on the requesting client's transport.
func l7TCPSend(client *Client, boardMAC net.HardwareAddr, boardIP net.IP, sport uint16, svc l7TCPService, seq, ack uint32, flags string, payload []byte, tag string) {
	frame := l7BuildTCP(boardMAC, boardIP, sport, svc, seq, ack, flags, payload)
	if frame == nil {
		return
	}
	client.WriteMutex.Lock()
	werr := sendFrame(client, frame)
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[L7TCP] %s send failed: %v\n", tag, werr)
	} else {
		fmt.Printf("[L7TCP] %s %dB seq=%d ack=%d flags=%s -> %s\n", tag, len(frame), seq, ack, flags, boardIP.String())
	}
}

// --- HTTP stub ----------------------------------------------------------
// Serves `GET /` (any HTTP/1.x request line with path `/`) with a fixed
// 200 + Content-Length body; any other path -> 404 with a short body.
// One request per connection (Connection: close semantics — the FIN leg
// below closes after the response; a second GET needs a new stub).

var l7HTTPBody = []byte("S3LAB-EMU-OK")

func l7HTTPResponse(req []byte) []byte {
	// Minimal request-line parse: `METHOD SP PATH SP`.
	path := "/"
	if i := indexOf(req, []byte("\r\n")); i >= 0 {
		line := req[:i]
		if j := indexOf(line, []byte(" ")); j >= 0 {
			rest := line[j+1:]
			if k := indexOf(rest, []byte(" ")); k >= 0 {
				path = string(rest[:k])
			} else if len(rest) > 0 {
				path = string(rest)
			}
		}
	}
	body := l7HTTPBody
	status := "200 OK"
	if path != "/" {
		body = []byte("not found")
		status = "404 Not Found"
	}
	hdr := "HTTP/1.0 " + status + "\r\nContent-Length: " + itoa(len(body)) + "\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n"
	out := make([]byte, 0, len(hdr)+len(body))
	out = append(out, hdr...)
	out = append(out, body...)
	return out
}

func indexOf(hay, needle []byte) int {
	for i := 0; i+len(needle) <= len(hay); i++ {
		match := true
		for j := 0; j < len(needle); j++ {
			if hay[i+j] != needle[j] {
				match = false
				break
			}
		}
		if match {
			return i
		}
	}
	return -1
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	var b [8]byte
	i := len(b)
	for n > 0 {
		i--
		b[i] = byte('0' + n%10)
		n /= 10
	}
	return string(b[i:])
}

// --- MQTT stub ----------------------------------------------------------
// Minimal MQTT 3.1.1 server side (spec §2-3, fixed-header only):
// CONNECT (0x10) -> CONNACK 0x20 0x02 0x00 0x00 (session accepted);
// SUBSCRIBE (0x82) -> SUBACK 0x90 + echoed packet-id + granted QoS0;
// PUBLISH QoS0 (0x30) -> accepted silently (sketch verdict reads the
// payload back from its own TX mirror + the CONNACK/SUBACK bytes);
// PINGREQ (0xC0) -> PINGRESP (0xD0); DISCONNECT (0xE0) -> drop stub.
// Remaining-length uses the 1-byte fast path (all sketch messages are
// < 127 bytes; longer encodings are rejected, not parsed).

func l7MQTTRespond(stub *l7TCPStub, msg []byte) (replies [][]byte, drop bool) {
	if len(msg) < 2 {
		return nil, false
	}
	typ := msg[0] & 0xF0
	rl := int(msg[1])
	if rl&0x80 != 0 {
		// Multi-byte remaining length: out of stub scope.
		return nil, false
	}
	if len(msg) < 2+rl {
		// Split MQTT message across TCP segments: ask the caller to
		// buffer more (caller holds `pending`; returning drop=false
		// with no replies keeps the stub alive).
		return nil, false
	}
	body := msg[2 : 2+rl]
	rest := msg[2+rl:]
	// Packet type is the HIGH nibble (low nibble is flags — notably
	// SUBSCRIBE arrives as 0x82, type 8 with flags 0010).
	switch typ {
	case 0x10: // CONNECT
		stub.mqttConn = true
		replies = append(replies, []byte{0x20, 0x02, 0x00, 0x00}) // CONNACK accepted
	case 0x80: // SUBSCRIBE (any flags): SUBACK + echoed pkt-id + granted QoS0
		if len(body) >= 2 {
			replies = append(replies, []byte{0x90, 0x03, body[0], body[1], 0x00})
		}
	case 0x30: // PUBLISH QoS0: accept silently (no wire reply exists)
		// NOTE: the low nibble holds DUP/QoS/RETAIN; QoS0 from the
		// sketch is 0x30 exactly, QoS1/2 (0x32/0x34) fall through to
		// the default below (PUBACK lives there if ever needed).
	case 0xC0: // PINGREQ
		replies = append(replies, []byte{0xD0, 0x00}) // PINGRESP
	case 0xE0: // DISCONNECT: tear down
		return nil, true
	default:
		// Unknown/duplicate-flagged type with a well-formed length:
		// consume (never falls to gVisor) but answer nothing.
	}
	if len(rest) > 0 {
		// Pipelined messages in one segment (CONNECT+SUBSCRIBE back to
		// back): recurse on the tail so one board segment can complete
		// a whole leg without another round trip.
		more, drop2 := l7MQTTRespond(stub, rest)
		replies = append(replies, more...)
		drop = drop2
	}
	return replies, drop
}

// l7TCPSnoop is the pipeline entry for gateway-destined TCP (IPv4 dst ==
// 192.168.4.1, dport 80/1883). It owns the segment (caller must skip the
// VN pipe AND the room broadcast on true) and answers synchronously:
// SYN -> SYN-ACK (+ remembers ISS); ACK (+optional first payload) ->
// service bytes; FIN -> FIN-ACK + drop; RST -> drop. Retransmitted SYNs
// re-answer SYN-ACK idempotently (same ISS — the board's 200 ms re-TX
// discipline from the UDP legs applies here too).
func l7TCPSnoop(msg []byte, client *Client, room *Room) bool {
	_ = room
	packet := gopacket.NewPacket(msg, layers.LayerTypeEthernet, gopacket.Default)
	ipLayer := packet.Layer(layers.LayerTypeIPv4)
	tcpLayer := packet.Layer(layers.LayerTypeTCP)
	if ipLayer == nil || tcpLayer == nil {
		return false
	}
	ip, _ := ipLayer.(*layers.IPv4)
	tcp, _ := tcpLayer.(*layers.TCP)
	if !ip.DstIP.Equal(net.IPv4(192, 168, 4, 1)) {
		return false
	}
	svc := l7TCPServiceFor(uint16(tcp.DstPort))
	if svc == l7SvcNone {
		return false
	}
	if tcp.RST {
		l7TCPMu.Lock()
		delete(l7TCPStubs, l7TCPKey(ip.SrcIP, uint16(tcp.SrcPort)))
		l7TCPMu.Unlock()
		return true // consume: a dead stub must not reach gVisor either
	}
	ethLayer := packet.Layer(layers.LayerTypeEthernet)
	eth, _ := ethLayer.(*layers.Ethernet)
	boardMAC := eth.SrcMAC
	boardIP := ip.SrcIP
	sport := uint16(tcp.SrcPort)
	key := l7TCPKey(boardIP, sport)
	l7SweepStubs()

	l7TCPMu.Lock()
	stub, ok := l7TCPStubs[key]
	if !ok {
		stub = &l7TCPStub{svc: svc, boardIP: append(net.IP(nil), boardIP...), boardMAC: append(net.HardwareAddr(nil), boardMAC...), sport: sport, iss: 0x1F2E3D4C, sndNxt: 0x1F2E3D4C + 1, last: time.Now()}
		l7TCPStubs[key] = stub
	} else if stub.svc != svc {
		// Same sport recycled across services (never observed — sports
		// differ per leg — but cheap to be safe): reset the stub.
		stub.svc = svc
		stub.pending = nil
		stub.httpReq = nil
		stub.mqttConn = false
		stub.estab = false
		stub.iss = 0x1F2E3D4C
		stub.sndNxt = 0x1F2E3D4C + 1
		stub.rcvNxt = 0 // re-learned from the next SYN/segment
	}
	stub.last = time.Now()
	l7TCPMu.Unlock()

	payload := tcp.Payload
	tcpHdrLen := int(tcp.DataOffset) * 4
	_ = tcpHdrLen

	if tcp.SYN {
		// SYN (or SYN retransmit): SYN-ACK with fixed ISS. rcvNxt =
		// board seq+1 (SYN consumes one) and sndNxt = iss+1 (our SYN
		// consumes one) — idempotent on retransmit. Data segments
		// therefore start at iss+1, which is what the board ACKs.
		stub.rcvNxt = tcp.Seq + 1
		stub.sndNxt = stub.iss + 1
		l7TCPSend(client, boardMAC, boardIP, sport, svc, stub.iss, stub.rcvNxt, "SA", nil, "SYNACK")
		return true
	}
	// Data/ACK path: account board bytes first (payload len; FIN also
	// consumes one sequence number).
	if len(payload) > 0 {
		stub.pending = append(stub.pending, payload...)
		stub.rcvNxt = tcp.Seq + uint32(len(payload))
	} else if stub.rcvNxt == 0 {
		stub.rcvNxt = tcp.Seq
	}
	if tcp.FIN {
		stub.rcvNxt++
		l7TCPSend(client, boardMAC, boardIP, sport, svc, stub.sndNxt, stub.rcvNxt, "FA", nil, "FINACK")
		stub.sndNxt++ // our FIN consumes one
		l7TCPMu.Lock()
		delete(l7TCPStubs, key)
		l7TCPMu.Unlock()
		return true
	}
	// Pure ACK (handshake completion or post-data ack): ack it only if
	// we have something outstanding... simplest correct: always ACK
	// back with current sndNxt (idempotent; the board's stack only
	// checks ack coverage, and duplicate ACKs are harmless).
	if len(stub.pending) == 0 {
		// Nothing to serve: bare ACK (or keepalive). Stay silent unless
		// we owe the handshake's final ACK... we already sent SYN-ACK;
		// the board's ACK completes it. Mark established.
		stub.estab = true
		return true // consumed (never falls to gVisor)
	}
	if svc == l7SvcHTTP {
		req := stub.pending
		stub.pending = nil
		stub.httpReq = append([]byte(nil), req...)
		resp := l7HTTPResponse(req)
		// One segment: response is ~60B headers + 12B body — far under
		// the 1500B single-frame rule. PSH+ACK, then FIN-ACK in the
		// same snoop (the sketch reads twice: data, then close).
		l7TCPSend(client, boardMAC, boardIP, sport, svc, stub.sndNxt, stub.rcvNxt, "PA", resp, "HTTPRESP")
		stub.sndNxt += uint32(len(resp))
		l7TCPSend(client, boardMAC, boardIP, sport, svc, stub.sndNxt, stub.rcvNxt, "FA", nil, "HTTPFIN")
		stub.sndNxt++
		stub.estab = true
		return true
	}
	// MQTT: feed the pending bytes through the message responder;
	// short (split-segment) returns mean "need more bytes" — keep
	// buffering (the sketch re-TXs on its 200 ms cadence, same as UDP).
	replies, drop := l7MQTTRespond(stub, stub.pending)
	if drop {
		l7TCPMu.Lock()
		delete(l7TCPStubs, key)
		l7TCPMu.Unlock()
		return true
	}
	// l7MQTTRespond returns (nil,false) both when the bytes are an
	// incomplete message AND when the message needs no reply (PUBLISH).
	// Distinguish: incomplete iff the first message's declared length
	// exceeds what we hold.
	needMore := len(stub.pending) >= 2 && len(stub.pending) < 2+int(stub.pending[1]&0x7F)
	if needMore {
		return true // consumed; wait for the rest
	}
	stub.pending = nil
	for _, r := range replies {
		l7TCPSend(client, boardMAC, boardIP, sport, svc, stub.sndNxt, stub.rcvNxt, "PA", r, "MQTTRESP")
		stub.sndNxt += uint32(len(r))
	}
	stub.estab = true
	return true
}
