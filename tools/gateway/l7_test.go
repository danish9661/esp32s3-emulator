package main

// Unit tests for the gateway-local L3–L7 services (handleL7.go): DNS
// static-table replies and NTP fixed-epoch replies. These pin the
// synchronous request/response contract the emulated firmware observes
// (board -> 192.168.4.1:53/123, gateway answers without touching gVisor),
// so a refactor can't silently break the byte layout lwIP parses.

import (
	"encoding/binary"
	"net"
	"testing"
	"time"

	"github.com/google/gopacket"
	"github.com/google/gopacket/layers"
)

// dnsQuery builds a minimal one-question A query for name (dotted, no
// trailing dot) with the given ID.
func dnsQuery(id uint16, name string) []byte {
	q := make([]byte, 12)
	binary.BigEndian.PutUint16(q[0:2], id)
	binary.BigEndian.PutUint16(q[2:4], 0x0100) // RD
	binary.BigEndian.PutUint16(q[4:6], 1)
	for _, label := range splitDots(name) {
		q = append(q, byte(len(label)))
		q = append(q, label...)
	}
	q = append(q, 0x00)
	q = append(q, 0x00, 0x01, 0x00, 0x01) // A IN
	return q
}

func splitDots(s string) []string {
	var out []string
	cur := ""
	for _, c := range s {
		if c == '.' {
			out = append(out, cur)
			cur = ""
		} else {
			cur += string(c)
		}
	}
	return append(out, cur)
}

func TestDNSReplyKnownHost(t *testing.T) {
	q := dnsQuery(0x1234, "example.com")
	rep := buildDNSReply(q)
	if rep == nil {
		t.Fatal("nil reply for known host")
	}
	if binary.BigEndian.Uint16(rep[0:2]) != 0x1234 {
		t.Fatal("ID not echoed")
	}
	if rep[2]&0x80 == 0 || rep[3]&0x0F != 0 {
		t.Fatalf("flags wrong: %02x %02x (want QR set, RCODE 0)", rep[2], rep[3])
	}
	if binary.BigEndian.Uint16(rep[6:8]) != 1 {
		t.Fatal("want exactly 1 answer")
	}
	// Answer RDATA is the last 4 bytes; must equal the table IP.
	want := net.IPv4(93, 184, 216, 34).To4()
	got := rep[len(rep)-4:]
	if string(got) != string(want) {
		t.Fatalf("RDATA %v, want %v", net.IP(got), net.IP(want))
	}
}

func TestDNSReplyUnknownHostNXDOMAIN(t *testing.T) {
	q := dnsQuery(0x5678, "nope.invalid")
	rep := buildDNSReply(q)
	if rep == nil {
		t.Fatal("nil reply for unknown host (want NXDOMAIN, not nil)")
	}
	if rep[3]&0x0F != 3 {
		t.Fatalf("RCODE=%d, want 3 (NXDOMAIN)", rep[3]&0x0F)
	}
	if binary.BigEndian.Uint16(rep[6:8]) != 0 {
		t.Fatal("NXDOMAIN must carry 0 answers")
	}
}

func TestDNSMQTTOthermosquittoTable(t *testing.T) {
	// The MQTT suite resolves this name; the table must carry it.
	q := dnsQuery(0x9, "test.mosquitto.org")
	rep := buildDNSReply(q)
	if rep == nil {
		t.Fatal("nil reply for test.mosquitto.org")
	}
	if binary.BigEndian.Uint16(rep[6:8]) != 1 {
		t.Fatal("want 1 answer for the MQTT broker name")
	}
}

func TestDNSNameCaseInsensitive(t *testing.T) {
	q := dnsQuery(0x1, "EXAMPLE.COM")
	rep := buildDNSReply(q)
	if rep == nil {
		t.Fatal("nil reply for upper-case query")
	}
	if binary.BigEndian.Uint16(rep[6:8]) != 1 {
		t.Fatal("decoder must lowercase QNAME before the table lookup")
	}
}

func TestNTPReplyShape(t *testing.T) {
	q := make([]byte, 48)
	q[0] = 0x1b // LI 0, version 3, mode 3 (client)
	rep := buildNTPReply(q)
	if rep == nil {
		t.Fatal("nil reply for 48-byte query")
	}
	if len(rep) != 48 {
		t.Fatalf("len %d, want 48", len(rep))
	}
	if rep[0]&0x7 != 4 {
		t.Fatalf("mode %d, want 4 (server)", rep[0]&0x7)
	}
	if rep[1] != 1 {
		t.Fatalf("stratum %d, want 1", rep[1])
	}
	if binary.BigEndian.Uint32(rep[40:44]) != l7NTPFixedSec {
		t.Fatal("TX timestamp must be the fixed 2026-01-01 epoch")
	}
	if buildNTPReply(make([]byte, 10)) != nil {
		t.Fatal("short query must return nil (snoop skips, no consume)")
	}
}

func TestUDPEchoRoundTrip(t *testing.T) {
	// Board UDP at 192.168.4.1:5683 with an unregistered sport gets its
	// payload echoed with swapped addrs/ports. Build the frame by hand
	// (same shape the worker sketch's udp_frame emits).
	mac := net.HardwareAddr{0x66, 0x55, 0x44, 0x33, 0x22, 0xC0}
	pay := []byte("L3ECHO")
	eth := &layers.Ethernet{SrcMAC: mac, DstMAC: gwMAC, EthernetType: layers.EthernetTypeIPv4}
	ip := &layers.IPv4{Version: 4, IHL: 5, TTL: 64, Protocol: layers.IPProtocolUDP,
		SrcIP: net.IPv4(192, 168, 4, 2), DstIP: net.IPv4(192, 168, 4, 1)}
	udp := &layers.UDP{SrcPort: 45003, DstPort: 5683}
	_ = udp.SetNetworkLayerForChecksum(ip)
	buf := gopacket.NewSerializeBuffer()
	if serr := gopacket.SerializeLayers(buf, gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true},
		eth, ip, udp, gopacket.Payload(pay)); serr != nil {
		t.Fatal(serr)
	}
	frame := buf.Bytes()
	// Capture the reply via a pipe-backed TCP-leg client.
	c1, c2 := net.Pipe()
	defer c1.Close()
	defer c2.Close()
	client := &Client{TCP: c2}
	done := make(chan []byte, 1)
	go func() {
		var hdr [4]byte
		if _, err := c1.Read(hdr[:]); err != nil {
			return
		}
		n := int(binary.BigEndian.Uint32(hdr[:]))
		body := make([]byte, n)
		off := 0
		for off < n {
			m, err := c1.Read(body[off:])
			if err != nil {
				return
			}
			off += m
		}
		done <- body
	}()
	room := &Room{}
	if !snoopUDPEcho(frame, client, room) {
		t.Fatal("UDP:5683 frame not consumed by echo snoop")
	}
	select {
	case rep := <-done:
		pkt := gopacket.NewPacket(rep, layers.LayerTypeEthernet, gopacket.Default)
		udpl := pkt.Layer(layers.LayerTypeUDP)
		if udpl == nil {
			t.Fatal("reply has no UDP layer")
		}
		u, _ := udpl.(*layers.UDP)
		if u.SrcPort != 5683 || u.DstPort != 45003 {
			t.Fatalf("ports not swapped: %d -> %d", u.SrcPort, u.DstPort)
		}
		if string(u.Payload) != "L3ECHO" {
			t.Fatalf("payload not echoed: %q", u.Payload)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("no echo reply arrived")
	}
}

func TestSnoopDNSPortGate(t *testing.T) {
	// A UDP frame to port 80 must NOT be consumed by the DNS snoop
	// (nil client is safe exactly when unconsumed — same discipline as
	// the ICMP shape test in framing_test.go).
	frame := make([]byte, 14+20+8+12)
	binary.BigEndian.PutUint16(frame[12:14], 0x0800)
	frame[14] = 0x45
	frame[23] = 17
	copy(frame[26:30], net.IPv4(192, 168, 4, 2).To4())
	copy(frame[30:34], net.IPv4(192, 168, 4, 1).To4())
	binary.BigEndian.PutUint16(frame[34:36], 1234) // sport
	binary.BigEndian.PutUint16(frame[36:38], 80)   // dport != 53
	if snoopDNS(frame, &Client{}, nil) {
		t.Fatal("non-DNS port consumed by DNS snoop")
	}
	if snoopNTP(frame, &Client{}, nil) {
		t.Fatal("non-NTP port consumed by NTP snoop")
	}
}
