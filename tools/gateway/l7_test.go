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

func TestCoAPTempGet(t *testing.T) {
	// CON GET coap://192.168.4.1/t (MID 0x2223, token AA 55) gets a
	// piggybacked ACK 2.05 with the same MID/token + `25.00C` payload.
	// Same pipe-backed client harness as TestUDPEchoRoundTrip.
	mac := net.HardwareAddr{0x66, 0x55, 0x44, 0x33, 0x22, 0xC0}
	coap := []byte{0x40, 0x01, 0x22, 0x23, 0xAA, 0x55, 0xB1, 't'}
	eth := &layers.Ethernet{SrcMAC: mac, DstMAC: gwMAC, EthernetType: layers.EthernetTypeIPv4}
	ip := &layers.IPv4{Version: 4, IHL: 5, TTL: 64, Protocol: layers.IPProtocolUDP,
		SrcIP: net.IPv4(192, 168, 4, 2), DstIP: net.IPv4(192, 168, 4, 1)}
	udp := &layers.UDP{SrcPort: 45006, DstPort: 5683}
	_ = udp.SetNetworkLayerForChecksum(ip)
	buf := gopacket.NewSerializeBuffer()
	if serr := gopacket.SerializeLayers(buf, gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true},
		eth, ip, udp, gopacket.Payload(coap)); serr != nil {
		t.Fatal(serr)
	}
	frame := buf.Bytes()
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
	if !snoopCoAP(frame, client, room) {
		t.Fatal("CoAP GET /t not consumed")
	}
	select {
	case rep := <-done:
		pkt := gopacket.NewPacket(rep, layers.LayerTypeEthernet, gopacket.Default)
		udpl := pkt.Layer(layers.LayerTypeUDP)
		if udpl == nil {
			t.Fatal("reply has no UDP layer")
		}
		u, _ := udpl.(*layers.UDP)
		if u.SrcPort != 5683 || u.DstPort != 45006 {
			t.Fatalf("ports not swapped: %d -> %d", u.SrcPort, u.DstPort)
		}
		p := u.Payload
		if len(p) < 8 || p[0] != 0x60 || p[1] != 0x45 {
			t.Fatalf("not ACK 2.05: %x", p)
		}
		if p[2] != 0x22 || p[3] != 0x23 || p[4] != 0xAA || p[5] != 0x55 {
			t.Fatalf("MID/token not echoed: %x", p)
		}
		found := false
		for i := 0; i+6 <= len(p); i++ {
			if string(p[i:i+6]) == "25.00C" {
				found = true
			}
		}
		if !found {
			t.Fatalf("payload missing 25.00C: %x", p)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("no CoAP reply arrived")
	}
}

func TestCoAPFallsThroughToEcho(t *testing.T) {
	// Non-CoAP bytes on :5683 (the sketch's HELLO-UDP) must NOT be
	// consumed by snoopCoAP — snoopUDPEcho still answers them as raw echo.
	mac := net.HardwareAddr{0x66, 0x55, 0x44, 0x33, 0x22, 0xC0}
	pay := []byte("HELLO-UDP")
	eth := &layers.Ethernet{SrcMAC: mac, DstMAC: gwMAC, EthernetType: layers.EthernetTypeIPv4}
	ip := &layers.IPv4{Version: 4, IHL: 5, TTL: 64, Protocol: layers.IPProtocolUDP,
		SrcIP: net.IPv4(192, 168, 4, 2), DstIP: net.IPv4(192, 168, 4, 1)}
	udp := &layers.UDP{SrcPort: 45005, DstPort: 5683}
	_ = udp.SetNetworkLayerForChecksum(ip)
	buf := gopacket.NewSerializeBuffer()
	if serr := gopacket.SerializeLayers(buf, gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true},
		eth, ip, udp, gopacket.Payload(pay)); serr != nil {
		t.Fatal(serr)
	}
	frame := buf.Bytes()
	c1, c2 := net.Pipe()
	defer c1.Close()
	defer c2.Close()
	client := &Client{TCP: c2}
	room := &Room{}
	if snoopCoAP(frame, client, room) {
		t.Fatal("raw HELLO-UDP consumed as CoAP")
	}
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
	if !snoopUDPEcho(frame, client, room) {
		t.Fatal("HELLO-UDP not consumed by echo snoop")
	}
	select {
	case rep := <-done:
		pkt := gopacket.NewPacket(rep, layers.LayerTypeEthernet, gopacket.Default)
		udpl := pkt.Layer(layers.LayerTypeUDP)
		if udpl == nil {
			t.Fatal("reply has no UDP layer")
		}
		u, _ := udpl.(*layers.UDP)
		if string(u.Payload) != "HELLO-UDP" {
			t.Fatalf("payload not echoed: %q", u.Payload)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("no echo reply arrived")
	}
}

func TestCoAPServerReadyTriggersGet(t *testing.T) {
	// Board `READY` (5B) on :5683 gets a CON GET /t (MID 0x2224, token
	// BB 66) — not a raw echo. Same pipe-backed client harness as the
	// CoAP tests above.
	mac := net.HardwareAddr{0x66, 0x55, 0x44, 0x33, 0x22, 0xC0}
	pay := []byte("READY")
	eth := &layers.Ethernet{SrcMAC: mac, DstMAC: gwMAC, EthernetType: layers.EthernetTypeIPv4}
	ip := &layers.IPv4{Version: 4, IHL: 5, TTL: 64, Protocol: layers.IPProtocolUDP,
		SrcIP: net.IPv4(192, 168, 4, 2), DstIP: net.IPv4(192, 168, 4, 1)}
	udp := &layers.UDP{SrcPort: 45008, DstPort: 5683}
	_ = udp.SetNetworkLayerForChecksum(ip)
	buf := gopacket.NewSerializeBuffer()
	if serr := gopacket.SerializeLayers(buf, gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true},
		eth, ip, udp, gopacket.Payload(pay)); serr != nil {
		t.Fatal(serr)
	}
	frame := buf.Bytes()
	c1, c2 := net.Pipe()
	defer c1.Close()
	defer c2.Close()
	client := &Client{TCP: c2}
	room := &Room{}
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
	if !snoopCoAPServer(frame, client, room) {
		t.Fatal("READY not consumed by CoAP-server snoop")
	}
	// NOTE: `snoopUDPEcho` would also answer READY as a raw echo, so the
	// production order (server before echo in main.go, both legs) is what
	// keeps the two apart — reviewed, not unit-probed here (probing echo
	// after the server consumed would block on the pipe with no reader).
	select {
	case rep := <-done:
		pkt := gopacket.NewPacket(rep, layers.LayerTypeEthernet, gopacket.Default)
		udpl := pkt.Layer(layers.LayerTypeUDP)
		if udpl == nil {
			t.Fatal("reply has no UDP layer")
		}
		u, _ := udpl.(*layers.UDP)
		if u.SrcPort != 5683 || u.DstPort != 45008 {
			t.Fatalf("ports not swapped to board server leg: %d -> %d", u.SrcPort, u.DstPort)
		}
		p := u.Payload
		if len(p) != 8 || p[0] != 0x40 || p[1] != 0x01 {
			t.Fatalf("not CON GET: %x", p)
		}
		if p[2] != 0x22 || p[3] != 0x24 || p[4] != 0xBB || p[5] != 0x66 {
			t.Fatalf("MID/token not server-leg values: %x", p)
		}
		if p[6] != 0xB1 || p[7] != 't' {
			t.Fatalf("path not /t: %x", p)
		}
	case <-time.After(5 * time.Second):
		t.Fatal("no CoAP GET arrived")
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

// ---- Gateway-local TCP stubs (HTTP :80, MQTT :1883) ----------------------
// These pin the synchronous stub contract the worker_l3 firmware legs
// observe (board -> 192.168.4.1:port, gateway answers without touching
// gVisor), so a refactor can't silently break the byte layout lwIP
// parses. Replies are captured via a pipe-backed TCP-leg client — the
// same trick as TestUDPEchoRoundTrip — so the tests exercise the real
// send path, not a copy of it.

// tcpClientPipe returns a TCP-leg client whose replies can be read as
// length-prefixed frames off the returned conn.
func tcpClientPipe(t *testing.T) (*Client, net.Conn) {
	t.Helper()
	c1, c2 := net.Pipe()
	t.Cleanup(func() { c1.Close(); c2.Close() })
	return &Client{TCP: c2}, c1
}

// readOneFrame reads one length-prefixed frame (the emulator's RX
// reassembly twin: header + body, short piped writes coalesced).
// NOTE: net.Pipe is synchronous (unbuffered): the reader MUST already be
// waiting when the snoop writes, otherwise the snoop blocks forever.
// Use `pumpPipe` (background reader) for all stub tests — never call the
// snoop synchronously and *then* read.
func readOneFrame(t *testing.T, c net.Conn) []byte {
	t.Helper()
	c.SetReadDeadline(time.Now().Add(5 * time.Second))
	var hdr [4]byte
	if _, err := c.Read(hdr[:]); err != nil {
		t.Fatalf("frame header read: %v", err)
	}
	n := int(binary.BigEndian.Uint32(hdr[:]))
	if n == 0 || n > 1600 {
		t.Fatalf("bad frame length %d", n)
	}
	body := make([]byte, n)
	off := 0
	for off < n {
		m, err := c.Read(body[off:])
		if err != nil {
			t.Fatalf("frame body read: %v", err)
		}
		off += m
	}
	return body
}

// pumpPipe starts a background reader on the pipe end and returns a
// channel of decoded frames. Start it BEFORE the first snoop so every
// stub write has a live reader (see readOneFrame note).
func pumpPipe(t *testing.T, c net.Conn) <-chan []byte {
	t.Helper()
	ch := make(chan []byte, 16)
	go func() {
		for {
			c.SetReadDeadline(time.Now().Add(10 * time.Second))
			var hdr [4]byte
			if _, err := c.Read(hdr[:]); err != nil {
				return
			}
			n := int(binary.BigEndian.Uint32(hdr[:]))
			if n == 0 || n > 1600 {
				return
			}
			body := make([]byte, n)
			off := 0
			for off < n {
				m, err := c.Read(body[off:])
				if err != nil {
					return
				}
				off += m
			}
			ch <- body
		}
	}()
	return ch
}

// nextFrame pulls one frame off the pump with a test-scoped timeout.
func nextFrame(t *testing.T, ch <-chan []byte) []byte {
	t.Helper()
	select {
	case f := <-ch:
		return f
	case <-time.After(5 * time.Second):
		t.Fatal("timed out waiting for stub reply")
		return nil
	}
}

// tcpSeg builds a board->gateway TCP segment: eth + IPv4 + TCP with the
// given seq/ack/flags + payload (same shape the worker sketch's raw
// frame builder emits; checksums computed).
func tcpSeg(boardMAC net.HardwareAddr, sport, dport uint16, seq, ack uint32, flags string, payload []byte) []byte {
	eth := &layers.Ethernet{SrcMAC: boardMAC, DstMAC: gwMAC, EthernetType: layers.EthernetTypeIPv4}
	ip := &layers.IPv4{Version: 4, IHL: 5, TTL: 64, Protocol: layers.IPProtocolTCP,
		SrcIP: net.IPv4(192, 168, 4, 2), DstIP: net.IPv4(192, 168, 4, 1)}
	tcp := &layers.TCP{SrcPort: layers.TCPPort(sport), DstPort: layers.TCPPort(dport), Seq: seq, Ack: ack, Window: 1460}
	for _, f := range flags {
		switch f {
		case 'S':
			tcp.SYN = true
		case 'A':
			tcp.ACK = true
		case 'F':
			tcp.FIN = true
		case 'P':
			tcp.PSH = true
		case 'R':
			tcp.RST = true
		}
	}
	_ = tcp.SetNetworkLayerForChecksum(ip)
	buf := gopacket.NewSerializeBuffer()
	if serr := gopacket.SerializeLayers(buf, gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true},
		eth, ip, tcp, gopacket.Payload(payload)); serr != nil {
		panic(serr)
	}
	return buf.Bytes()
}

// tcpReplyLayers decodes one stub reply frame into its TCP layer +
// payload for assertions.
func tcpReplyLayers(t *testing.T, frame []byte) (*layers.TCP, []byte) {
	t.Helper()
	pkt := gopacket.NewPacket(frame, layers.LayerTypeEthernet, gopacket.Default)
	tl := pkt.Layer(layers.LayerTypeTCP)
	if tl == nil {
		t.Fatal("reply has no TCP layer")
	}
	tcp, _ := tl.(*layers.TCP)
	return tcp, tcp.Payload
}

func TestL7TCPServiceGate(t *testing.T) {
	// Only gateway-IP TCP :80/:1883 is stubbed; everything else falls
	// through (nil client is safe exactly when unconsumed).
	mac := net.HardwareAddr{0x66, 0x55, 0x44, 0x33, 0x22, 0xC0}
	// Off-LAN dst (proxy-ARP would steer it, but the stub must not
	// claim it — gVisor NAT owns it).
	offLan := tcpSeg(mac, 45001, 80, 1000, 0, "S", nil)
	offLan[30], offLan[31], offLan[32], offLan[33] = 93, 184, 216, 34 // dst 93.184.216.34
	if l7TCPSnoop(offLan, &Client{}, nil) {
		t.Fatal("off-LAN TCP:80 consumed by gateway stub")
	}
	// Wrong dport on gateway IP: not stubbed either.
	other := tcpSeg(mac, 45001, 8883, 1000, 0, "S", nil)
	if l7TCPSnoop(other, &Client{}, nil) {
		t.Fatal("gateway TCP:8883 consumed by stub")
	}
	// UDP to :80 is not TCP at all.
	udp80 := make([]byte, 14+20+8+4)
	binary.BigEndian.PutUint16(udp80[12:14], 0x0800)
	udp80[14] = 0x45
	udp80[23] = 17
	copy(udp80[26:30], net.IPv4(192, 168, 4, 2).To4())
	copy(udp80[30:34], net.IPv4(192, 168, 4, 1).To4())
	binary.BigEndian.PutUint16(udp80[34:36], 45001)
	binary.BigEndian.PutUint16(udp80[36:38], 80)
	if l7TCPSnoop(udp80, &Client{}, nil) {
		t.Fatal("UDP:80 consumed by TCP stub")
	}
}

func TestL7HTTPHandshakeAndGet(t *testing.T) {
	// Full HTTP leg against the stub: SYN -> SYN-ACK; ACK; GET / ->
	// 200 with Content-Length body; FIN -> FIN-ACK. Mirrors the
	// worker_l3 HTTP leg (same ports, same assertions on status +
	// body bytes).
	mac := net.HardwareAddr{0x66, 0x55, 0x44, 0x33, 0x22, 0xC1}
	const sport = 45011
	client, pipe := tcpClientPipe(t)
	frames := pumpPipe(t, pipe) // live reader BEFORE the first snoop
	room := &Room{}
	syn := tcpSeg(mac, sport, 80, 7000, 0, "S", nil)
	if !l7TCPSnoop(syn, client, room) {
		t.Fatal("SYN not consumed")
	}
	rep := nextFrame(t, frames)
	tcp, _ := tcpReplyLayers(t, rep)
	if !tcp.SYN || !tcp.ACK {
		t.Fatalf("want SYN-ACK, got SYN=%v ACK=%v", tcp.SYN, tcp.ACK)
	}
	if tcp.Ack != 7001 {
		t.Fatalf("SYN-ACK ack=%d, want 7001", tcp.Ack)
	}
	iss := tcp.Seq
	// ACK the handshake (no payload).
	ack := tcpSeg(mac, sport, 80, 7001, iss+1, "A", nil)
	if !l7TCPSnoop(ack, client, room) {
		t.Fatal("handshake ACK not consumed")
	}
	// GET / (PSH+ACK).
	get := tcpSeg(mac, sport, 80, 7001, iss+1, "PA", []byte("GET / HTTP/1.0\r\nHost: gateway\r\n\r\n"))
	if !l7TCPSnoop(get, client, room) {
		t.Fatal("GET not consumed")
	}
	resp := nextFrame(t, frames)
	rtcp, body := tcpReplyLayers(t, resp)
	if !rtcp.PSH || !rtcp.ACK {
		t.Fatalf("want PSH+ACK, got PSH=%v ACK=%v", rtcp.PSH, rtcp.ACK)
	}
	if rtcp.Seq != iss+1 {
		t.Fatalf("response seq=%d, want iss+1=%d", rtcp.Seq, iss+1)
	}
	if len(body) < 15 || string(body[:15]) != "HTTP/1.0 200 OK" {
		t.Fatalf("bad status line: %q", body[:minInt(15, len(body))])
	}
	if !containsBytes(body, l7HTTPBody) {
		t.Fatalf("response body missing marker: %q", body)
	}
	// FIN leg arrives in the same snoop (stub sends FIN-ACK right
	// after the response): drain it, then ACK it.
	finack := nextFrame(t, frames)
	ftcp, _ := tcpReplyLayers(t, finack)
	if !ftcp.FIN || !ftcp.ACK {
		t.Fatalf("want FIN-ACK, got FIN=%v ACK=%v", ftcp.FIN, ftcp.ACK)
	}
}

func TestL7HTTPNotFound(t *testing.T) {
	// Unknown path -> 404 with a short body (same single-segment rule).
	mac := net.HardwareAddr{0x66, 0x55, 0x44, 0x33, 0x22, 0xC2}
	const sport = 45012
	client, pipe := tcpClientPipe(t)
	frames := pumpPipe(t, pipe) // live reader BEFORE the first snoop
	room := &Room{}
	if !l7TCPSnoop(tcpSeg(mac, sport, 80, 9000, 0, "S", nil), client, room) {
		t.Fatal("SYN not consumed")
	}
	synack := nextFrame(t, frames)
	stcp, _ := tcpReplyLayers(t, synack)
	iss := stcp.Seq
	get := tcpSeg(mac, sport, 80, 9001, iss+1, "PA", []byte("GET /nope HTTP/1.0\r\n\r\n"))
	if !l7TCPSnoop(get, client, room) {
		t.Fatal("GET /nope not consumed")
	}
	resp := nextFrame(t, frames)
	_, body := tcpReplyLayers(t, resp)
	if len(body) < 17 || string(body[:17]) != "HTTP/1.0 404 Not " {
		t.Fatalf("want 404 status, got %q", body[:minInt(17, len(body))])
	}
}

func TestL7MQTTConnectSubPub(t *testing.T) {
	// CONNECT -> CONNACK(accepted); SUBSCRIBE -> SUBACK QoS0; PUBLISH
	// QoS0 accepted silently. Mirrors the worker_l3 MQTT leg byte for
	// byte (fixed packet-ids so the sketch can assert exact echoes).
	mac := net.HardwareAddr{0x66, 0x55, 0x44, 0x33, 0x22, 0xC3}
	const sport = 45013
	client, pipe := tcpClientPipe(t)
	frames := pumpPipe(t, pipe) // live reader BEFORE the first snoop
	room := &Room{}
	if !l7TCPSnoop(tcpSeg(mac, sport, 1883, 3000, 0, "S", nil), client, room) {
		t.Fatal("MQTT SYN not consumed")
	}
	synack := nextFrame(t, frames)
	stcp, _ := tcpReplyLayers(t, synack)
	iss := stcp.Seq
	// CONNECT (minimal: fixed header + tiny body; stub only checks type).
	connect := []byte{0x10, 0x0A, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x02, 0x00, 0x3C}
	if !l7TCPSnoop(tcpSeg(mac, sport, 1883, 3001, iss+1, "PA", connect), client, room) {
		t.Fatal("CONNECT not consumed")
	}
	ca := nextFrame(t, frames)
	_, cabody := tcpReplyLayers(t, ca)
	if len(cabody) != 4 || cabody[0] != 0x20 || cabody[1] != 0x02 || cabody[2] != 0x00 || cabody[3] != 0x00 {
		t.Fatalf("bad CONNACK: %x", cabody)
	}
	// SUBSCRIBE pkt-id 0x1234, topic "t", QoS0 (rl=6: 2 pkt-id +
	// 2 topic-len + 1 topic + 1 qos).
	sub := []byte{0x82, 0x06, 0x12, 0x34, 0x00, 0x01, 't', 0x00}
	if !l7TCPSnoop(tcpSeg(mac, sport, 1883, 3001+uint32(len(connect)), iss+1+4, "PA", sub), client, room) {
		t.Fatal("SUBSCRIBE not consumed")
	}
	sa := nextFrame(t, frames)
	_, sabody := tcpReplyLayers(t, sa)
	if len(sabody) != 5 || sabody[0] != 0x90 || sabody[1] != 0x03 || sabody[2] != 0x12 || sabody[3] != 0x34 || sabody[4] != 0x00 {
		t.Fatalf("bad SUBACK: %x", sabody)
	}
	// PUBLISH QoS0 "hi" (no reply expected — accepted silently;
	// rl=5: 2 topic-len + 1 topic + 2 payload).
	pub := []byte{0x30, 0x05, 0x00, 0x01, 't', 'h', 'i'}
	before := len(l7TCPStubs)
	if !l7TCPSnoop(tcpSeg(mac, sport, 1883, 3001+uint32(len(connect))+uint32(len(sub)), iss+1+4+5, "PA", pub), client, room) {
		t.Fatal("PUBLISH not consumed")
	}
	if len(l7TCPStubs) != before {
		t.Fatal("PUBLISH must not change stub count")
	}
	// PINGREQ -> PINGRESP.
	if !l7TCPSnoop(tcpSeg(mac, sport, 1883, 3001+uint32(len(connect))+uint32(len(sub))+uint32(len(pub)), iss+1+4+5, "PA", []byte{0xC0, 0x00}), client, room) {
		t.Fatal("PINGREQ not consumed")
	}
	pr := nextFrame(t, frames)
	_, prbody := tcpReplyLayers(t, pr)
	if len(prbody) != 2 || prbody[0] != 0xD0 || prbody[1] != 0x00 {
		t.Fatalf("bad PINGRESP: %x", prbody)
	}
}

func minInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func containsBytes(hay, needle []byte) bool {
	for i := 0; i+len(needle) <= len(hay); i++ {
		match := true
		for j := 0; j < len(needle); j++ {
			if hay[i+j] != needle[j] {
				match = false
				break
			}
		}
		if match {
			return true
		}
	}
	return false
}
