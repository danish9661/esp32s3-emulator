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
// state beyond the UDP-forward relay map that already exists):
//   * DNS (UDP 53, board -> us): static A-record table + an 8.8.8.8
//     fallback is NOT attempted offline — answers come from the table
//     (example.com, test.mosquitto.org) so `gethostbyname` completes.
//   * NTP (UDP 123, board -> us): fixed-epoch reply (2026-01-01) so
//     `sntp_getreachability`/time-sync sketches observe a stable time.
//   * HTTP (TCP 80, board -> us via gVisor NAT): NOT intercepted —
//     gVisor already NATs board TCP to the outside world (proven live:
//     EGRESS_TCP80_OK). Documented here so the pipeline order is clear.
//   * MQTT (TCP 1883, board -> us via gVisor NAT): same — NAT handles it
//     (proven live: EGRESS_MQTT_OK to test.mosquitto.org:1883). The
//     gateway only needs to NOT consume these frames (no snoop arm).
//   * CoAP (UDP 5683): already covered both directions — inbound via the
//     127.0.0.1:<uport> UDP-forward listener (handleUDPProxy) and board
//     replies via divertUDPForward. No new code; pinned by tests here.
//
// Pipeline placement (handleTCPFrame/handleClient, in order):
//   ARP fast/proxy reply -> ICMPv4 echo -> DHCP:67 -> UDP-forward relay
//   -> IPv6 services -> *** L3-L7 services (this file: DNS/NTP) *** ->
//   gVisor feed + room broadcast.
// DNS/NTP MUST sit before the gVisor feed so the frames never reach the
// NAT stack (which has no listener for gateway-IP UDP:53/123 and would
// drop them); everything else falls through to gVisor untouched.

import (
	"encoding/binary"
	"fmt"
	"net"

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

// ---- TCP passthrough note (HTTP/MQTT) ------------------------------------
// Board TCP to the outside world (HTTP :80, HTTPS :443, MQTT :1883,
// test.mosquitto.org) is NATed by the gVisor stack — NO snoop arm may
// claim these frames. This predicate documents the boundary: it returns
// false always (never consumes); the pipeline calls it for symmetry so a
// future intercept (e.g. a local MQTT broker) has an explicit slot.
// Proven live: host egress to example.com:80 and test.mosquitto.org:1883
// succeeds, so gVisor NAT carries these end-to-end once the board's
// frames reach the pipe.
func snoopTCPProxyNote(msg []byte, client *Client, room *Room) bool {
	_, _, _ = msg, client, room
	return false
}
