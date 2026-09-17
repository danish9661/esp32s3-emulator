package main

import (
	"encoding/binary"
	"fmt"
	"net"
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

// ---- IPv6 gateway support (board <-> gateway only) ----------------------
// The gVisor stack is v4-NAT only, so IPv6 lives entirely at the hub
// layer here: Router Advertisements (SLAAC), Neighbor Discovery answers,
// ICMPv6 echo and a UDP echo service, all crafted by hand. Everything is
// synchronous request/response — no per-board state, no rooms involved.

var (
	gwMAC       = net.HardwareAddr{0x5a, 0x94, 0xef, 0xe4, 0x0c, 0xdd}
	gwLinkLocal = net.ParseIP("fe80::5894:efff:fee4:cdd")
	gwULA       = net.ParseIP("fd00::1")
)

// Boards seen doing IPv6 (by MAC): unicast RA target list. LWIP only
// processes UNICAST RAs here — multicast-destined RAs never surface past
// the driver/LWIP multicast filter, so re-advertising must also be
// unicast. Refreshed on every RS; the ticker below re-advertises so
// lifetimes never expire and missed RS/RA exchanges self-heal.
type raTarget struct {
	room   *Room
	mac    net.HardwareAddr
	ll     net.IP
	client *Client
	last   time.Time
}

var (
	raMu      sync.Mutex
	raTargets = make(map[string]*raTarget)
)

func noteRATarget(mac net.HardwareAddr, ll net.IP, room *Room, client *Client) {
	raMu.Lock()
	defer raMu.Unlock()
	raTargets[mac.String()] = &raTarget{room: room, mac: append(net.HardwareAddr(nil), mac...), ll: append(net.IP(nil), ll...), client: client, last: time.Now()}
}

func startPeriodicRA() {
	ticker := time.NewTicker(30 * time.Second)
	go func() {
		for range ticker.C {
			raMu.Lock()
			for id, t := range raTargets {
				if time.Since(t.last) > 120*time.Second {
					delete(raTargets, id)
					continue
				}
				roomsMutex.Lock()
				_, ok := rooms[t.room.SessionId]
				roomsMutex.Unlock()
				if !ok {
					delete(raTargets, id)
					continue
				}
				sendRA(t.client, t.mac, t.ll)
			}
			raMu.Unlock()
		}
	}()
}

func isGatewayIP(ip net.IP) bool {
	return ip.Equal(gwLinkLocal) || ip.Equal(gwULA)
}

// v6Checksum covers the IPv6 pseudo-header + message. msg must have its
// checksum field zeroed.
func v6Checksum(src, dst net.IP, nextHeader byte, msg []byte) uint16 {
	sum := uint32(0)
	add := func(b []byte) {
		for i := 0; i+1 < len(b); i += 2 {
			sum += uint32(b[i])<<8 | uint32(b[i+1])
		}
		if len(b)%2 != 0 {
			sum += uint32(b[len(b)-1]) << 8
		}
	}
	add(src.To16())
	add(dst.To16())
	l := uint32(len(msg))
	sum += (l >> 16) + (l & 0xffff)
	sum += uint32(nextHeader)
	add(msg)
	for sum>>16 != 0 {
		sum = (sum >> 16) + (sum & 0xffff)
	}
	return uint16(^sum)
}

// sendIPv6 delivers one IPv6 packet to a board client. nextHeader selects
// the checksum protocol (58 ICMPv6, 17 UDP); payload must have its checksum
// field zeroed. IPv6 header layout (RFC 8200 §3): ver/TC/flow,
// payload-len [18:20], next [20], hop [21], src [22:38], dst [38:54].
// (An earlier revision wrote every field 2 bytes early — matching the
// payload-length position instead of the header start — producing frames
// LWIP silently drops. Verified field-by-field since.)
func sendIPv6(client *Client, dstMAC net.HardwareAddr, srcIP, dstIP net.IP, nextHeader byte, payload []byte) {
	frame := make([]byte, 14+40+len(payload))
	copy(frame[0:6], []byte(dstMAC))
	copy(frame[6:12], gwMAC)
	binary.BigEndian.PutUint16(frame[12:14], 0x86DD)
	frame[14] = 0x60
	frame[15] = 0x00
	binary.BigEndian.PutUint16(frame[16:18], 0)
	binary.BigEndian.PutUint16(frame[18:20], uint16(len(payload)))
	frame[20] = nextHeader
	frame[21] = 255
	copy(frame[22:38], srcIP.To16())
	copy(frame[38:54], dstIP.To16())
	copy(frame[54:], payload)
	// Fill in the checksum now that addresses are known.
	ck := v6Checksum(srcIP, dstIP, nextHeader, frame[54:])
	if nextHeader == 58 {
		binary.BigEndian.PutUint16(frame[56:58], ck)
	} else if nextHeader == 17 {
		binary.BigEndian.PutUint16(frame[60:62], ck)
	}
	client.WriteMutex.Lock()
	werr := client.Conn.WriteMessage(websocket.BinaryMessage, frame)
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[IPv6] send to %s failed: %v\n", dstIP.String(), werr)
	}
}

func v6Addr(b []byte) net.IP {
	a := make(net.IP, 16)
	copy(a, b)
	return a
}

// sendRA answers a Router Solicitation: fd00::/64 autonomous prefix so the
// board SLAACs a global ULA + default route via our link-local.
func sendRA(client *Client, dstMAC net.HardwareAddr, dstIP net.IP) {
	sendIPv6(client, dstMAC, gwLinkLocal, dstIP, 58, buildRA())
	fmt.Printf("[IPv6] RA -> %s\n", dstIP.String())
}

// buildRA constructs the 64-byte Router Advertisement body (checksum
// field zeroed; sendIPv6 fills it in).
func buildRA() []byte {
	msg := make([]byte, 16+8+8+32)
	msg[0] = 134
	msg[1] = 0
	msg[4] = 64
	msg[5] = 0
	binary.BigEndian.PutUint16(msg[6:8], 1800)
	msg[16] = 1
	msg[17] = 1
	copy(msg[18:24], gwMAC)
	msg[24] = 5
	msg[25] = 1
	binary.BigEndian.PutUint32(msg[28:32], 1500)
	msg[32] = 3
	msg[33] = 4
	msg[34] = 64
	msg[35] = 0xC0
	// Finite lifetimes (NOT infinite): LWIP may mishandle 0xFFFFFFFF
	// (expiry arithmetic overflow -> instantly expired). Expiry in sim
	// time is harmless because the periodic re-advertisements below
	// refresh lifetimes every 30 wall-seconds, like a real router.
	binary.BigEndian.PutUint32(msg[36:40], 86400)
	binary.BigEndian.PutUint32(msg[40:44], 14400)
	copy(msg[48:64], net.ParseIP("fd00::").To16())
	return msg
}

// sendNA answers a Neighbor Solicitation for one of our addresses.
func sendNA(client *Client, dstMAC net.HardwareAddr, dstIP, target net.IP) {
	msg := make([]byte, 24+8)
	msg[0] = 136
	msg[1] = 0
	// checksum filled by sendIPv6
	msg[4] = 0x60 // solicited + override
	copy(msg[8:24], target.To16())
	msg[24] = 2 // target link-layer option
	msg[25] = 1
	copy(msg[26:32], gwMAC)
	sendIPv6(client, dstMAC, target, dstIP, 58, msg)
}

// snoopIPv6 handles board -> hub IPv6 frames destined for the gateway
// itself (RS / NS-for-us / echo / UDP-echo). Returns true when consumed
// (caller must skip the VN pipe AND the room broadcast for it).
func snoopIPv6(msg []byte, client *Client, room *Room) bool {
	if len(msg) < 14+40+8 {
		return false
	}
	if binary.BigEndian.Uint16(msg[12:14]) != 0x86DD || msg[14]>>4 != 6 {
		return false
	}
	nextHdr := msg[20]
	srcMAC := net.HardwareAddr(append([]byte(nil), msg[6:12]...))
	_ = srcMAC
	srcIP := v6Addr(msg[22:38])
	dstIP := v6Addr(msg[38:54])
	payload := msg[54:]
	switch nextHdr {
	case 58: // ICMPv6
		if len(payload) < 8 {
			return false
		}
		switch payload[0] {
		case 133: // Router Solicitation -> unicast RA back. (Unicast is
			// required here: multicast-destined RAs never surface past
			// the driver/LWIP multicast filter in this stack.)
			sendRA(client, srcMAC, srcIP)
			noteRATarget(srcMAC, srcIP, room, client)
			return true
		case 135: // Neighbor Solicitation -> NA if asking for us
			if len(payload) < 24 {
				return false
			}
			if isGatewayIP(v6Addr(payload[8:24])) {
				sendNA(client, srcMAC, srcIP, v6Addr(payload[8:24]))
				return true
			}
		case 128: // Echo request -> echo reply if for us
			if !isGatewayIP(dstIP) || len(payload) < 8 {
				return false
			}
			rep := make([]byte, len(payload))
			copy(rep, payload)
			rep[0] = 129
			rep[1] = 0
			rep[2], rep[3] = 0, 0
			sendIPv6(client, srcMAC, dstIP, srcIP, 58, rep)
			return true
		}
	case 17: // UDP -> echo service on our addresses, port 5683
		if len(payload) < 8 || !isGatewayIP(dstIP) {
			return false
		}
		if binary.BigEndian.Uint16(payload[2:4]) != 5683 {
			return false
		}
		rep := make([]byte, len(payload))
		copy(rep, payload)
		// swap ports
		rep[0], rep[1], rep[2], rep[3] = payload[2], payload[3], payload[0], payload[1]
		rep[6], rep[7] = 0, 0
		sendIPv6(client, srcMAC, dstIP, srcIP, 17, rep)
		return true
	}
	return false
}
