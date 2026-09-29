package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"net"
	"os"
	"sync"
	"time"

	"github.com/google/gopacket"
	"github.com/google/gopacket/layers"
	"github.com/gorilla/websocket"
	"github.com/insomniacslk/dhcp/dhcpv4"
)

var (
	globalNextIP byte = 2
	globalMacToIP = make(map[string]net.IP)
	globalIPToPort = make(map[string]int)
	dhcpMutex sync.Mutex
)

// ---- UDP forward (host -> board) --------------------------------------
// The gVisor virtualnetwork.Dial API only supports TCP, so UDP server
// roles on the board (e.g. CoAP on 5683) are forwarded manually:
//   * inbound: 127.0.0.1:<uport>/udp -> injected into the VN as
//     192.168.4.1:<alloc> -> board:5683 (raw eth frame on the room pipe)
//   * replies: board -> 192.168.4.1:<alloc> are snooped in handleClient
//     (before they reach the VN pipe) and relayed to the host client.
// This keeps everything inside addresses the board can route (no hairpin
// through the host stack, which gVisor NAT cannot return).
type udpFwdEntry struct {
	uconn      *net.UDPConn
	clientAddr *net.UDPAddr
	last       time.Time
}

var (
	udpFwdMu    sync.Mutex
	udpFwdBySport = make(map[uint16]*udpFwdEntry)
	udpFwdNextSport uint16 = 40000
	vnPipeMu    sync.Mutex // serializes length-prefixed writes to room pipes
)

// One UDP listener per board IP (created on first DHCP). The room binding
// is refreshed on EVERY DHCP: rooms are per-connection, so a listener that
// keeps injecting into a dead room's pipe would blackhole everything.
type udpFwdListener struct {
	uconn *net.UDPConn
	room  *Room
	mac   string
}

var udpFwdListeners = make(map[string]*udpFwdListener)

func refreshUDPFwdRoom(ipStr string, room *Room, mac string) {
	udpFwdMu.Lock()
	defer udpFwdMu.Unlock()
	if l, ok := udpFwdListeners[ipStr]; ok {
		l.room = room
		l.mac = mac
	}
}

func udpFwdLookup(sport uint16) *udpFwdEntry {
	udpFwdMu.Lock()
	defer udpFwdMu.Unlock()
	return udpFwdBySport[sport]
}

// handleDHCP processes DHCP Discover/Request packets, assigns an IP, and sends an Offer/ACK directly back to the client.
func handleDHCP(msg []byte, packet gopacket.Packet, client *Client, room *Room) {
	fmt.Println("\n[DHCP] --- Intercepted UDP Port 67 Packet! ---")
	ethLayer := packet.Layer(layers.LayerTypeEthernet)
	if ethLayer == nil {
		fmt.Println("[DHCP] Error: No Ethernet Layer found.")
		return
	}
	eth, _ := ethLayer.(*layers.Ethernet)
	
	udpLayer := packet.Layer(layers.LayerTypeUDP)
	if udpLayer == nil {
		fmt.Println("[DHCP] Error: No UDP Layer found.")
		return
	}
	udp, _ := udpLayer.(*layers.UDP)

	dhcpPacket, err := dhcpv4.FromBytes(udp.Payload)
	if err != nil {
		fmt.Printf("[DHCP] Error parsing DHCP: %v\n", err)
		return
	}

	macString := eth.SrcMAC.String()
	
	gatewayMode := os.Getenv("GATEWAY_MODE")
	var assignedIP net.IP

	if gatewayMode == "public" {
		room.Lock()
		ip, exists := room.MacToIP[macString]
		if !exists {
			ip = net.IPv4(192, 168, 4, room.NextIP)
			room.NextIP++
			if room.NextIP > 250 {
				room.NextIP = 2
			}
			room.MacToIP[macString] = ip
		}
		assignedIP = ip
		room.Unlock()

		// BOARD_IP is a frontend text message (WebSocket-only). TCP-ingest
		// (headless emulator) clients have no WS Conn — skip it (the
		// length-prefixed DHCP reply below is the real verdict; the
		// emulator never reads BOARD_IP). Unconditional WriteMessage here
		// nil-panics the whole gateway (proven live 2026-09-28: a DHCP
		// DISCOVER over the TCP leg crashed handleTCPFrame).
		if client.Conn != nil {
			client.WriteMutex.Lock()
			client.Conn.WriteMessage(websocket.TextMessage, []byte("BOARD_IP:"+assignedIP.String()))
			client.WriteMutex.Unlock()
		}
	} else {
		dhcpMutex.Lock()
		ip, exists := globalMacToIP[macString]
		if !exists {
			ip = net.IPv4(192, 168, 4, globalNextIP)
			globalNextIP++
			if globalNextIP > 250 {
				globalNextIP = 2
			}
			globalMacToIP[macString] = ip
		}
		assignedIP = ip

		ipStr := assignedIP.String()
		_, hasProxy := globalIPToPort[ipStr]
		if !hasProxy {
			port := 8080
			for {
				ln, err := net.Listen("tcp", fmt.Sprintf("127.0.0.1:%d", port))
				if err == nil {
					globalIPToPort[ipStr] = port
					go func(listener net.Listener, targetIP string, assignedPort int) {
						defer listener.Close()
						for {
							conn, err := listener.Accept()
							if err != nil {
								return
							}
							go handleProxy(conn, targetIP)
						}
					}(ln, ipStr, port)

								fmt.Printf("[Port Forward] Mapped 127.0.0.1:%d -> %s:80\n", port, ipStr)

					// UDP forward for server roles on the board (e.g. CoAP
					// on its well-known port): 127.0.0.1:<uport> -> board:5683.
					// Implemented below via raw-frame injection (the VN Dial
					// API is TCP-only) — see handleUDPProxy.
					uport := 5683
					for {
						uaddr, uerr := net.ResolveUDPAddr("udp", fmt.Sprintf("127.0.0.1:%d", uport))
						if uerr != nil {
							break
						}
						uconn, uerr := net.ListenUDP("udp", uaddr)
						if uerr == nil {
							udpFwdMu.Lock()
							udpFwdListeners[ipStr] = &udpFwdListener{uconn: uconn, room: room, mac: macString}
							udpFwdMu.Unlock()
							go handleUDPProxy(uconn, ipStr)
							fmt.Printf("[Port Forward] Mapped 127.0.0.1:%d/udp -> %s:5683\n", uport, ipStr)
							break
						}
						uport++
					}

					// Send structured messages to the frontend (WebSocket-only;
					// TCP-ingest clients have no WS Conn — same nil-guard as
					// above).
					if client.Conn != nil {
						client.WriteMutex.Lock()
						client.Conn.WriteMessage(websocket.TextMessage, []byte("BOARD_IP:"+ipStr))
						client.Conn.WriteMessage(websocket.TextMessage, []byte(fmt.Sprintf("PORT_FORWARD:http://127.0.0.1:%d", port)))
						client.WriteMutex.Unlock()
					}
					break
				}
				port++
			}
		}
		dhcpMutex.Unlock()
	}

	// Refresh the UDP-forward room binding on EVERY DHCP: rooms are
	// per-connection, so injecting into a dead room's pipe would blackhole
	// all forwarded datagrams for the rest of the gateway's life.
	refreshUDPFwdRoom(assignedIP.String(), room, macString)

	var replyDHCP *dhcpv4.DHCPv4
	serverIP := net.IPv4(192, 168, 4, 1)
	routerIP := net.IPv4(192, 168, 4, 1)
	dnsIP := net.IPv4(8, 8, 8, 8)
	netmask := net.IPv4Mask(255, 255, 255, 0)

	msgType := dhcpPacket.MessageType()
	if msgType == dhcpv4.MessageTypeDiscover {
		fmt.Printf("[DHCP] Intercepted DISCOVER from %s. Offering %s\n", macString, assignedIP)
		replyDHCP, _ = dhcpv4.NewReplyFromRequest(dhcpPacket,
			dhcpv4.WithMessageType(dhcpv4.MessageTypeOffer),
			dhcpv4.WithYourIP(assignedIP),
			dhcpv4.WithServerIP(serverIP),
			dhcpv4.WithOption(dhcpv4.OptServerIdentifier(serverIP)),
			dhcpv4.WithRouter(routerIP),
			dhcpv4.WithDNS(dnsIP),
			dhcpv4.WithNetmask(netmask),
			dhcpv4.WithLeaseTime(86400),
		)
	} else if msgType == dhcpv4.MessageTypeRequest {
		fmt.Printf("[DHCP] Intercepted REQUEST from %s. Acknowledging %s\n", macString, assignedIP)
		replyDHCP, _ = dhcpv4.NewReplyFromRequest(dhcpPacket,
			dhcpv4.WithMessageType(dhcpv4.MessageTypeAck),
			dhcpv4.WithYourIP(assignedIP),
			dhcpv4.WithServerIP(serverIP),
			dhcpv4.WithOption(dhcpv4.OptServerIdentifier(serverIP)),
			dhcpv4.WithRouter(routerIP),
			dhcpv4.WithDNS(dnsIP),
			dhcpv4.WithNetmask(netmask),
			dhcpv4.WithLeaseTime(86400),
		)
	} else {
		return
	}

	ethReply := &layers.Ethernet{
		SrcMAC:       net.HardwareAddr{0x5a, 0x94, 0xef, 0xe4, 0x0c, 0xdd},
		DstMAC:       net.HardwareAddr{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, // Broadcast MAC for DHCP Reply
		EthernetType: layers.EthernetTypeIPv4,
	}
	ipv4Reply := &layers.IPv4{
		Version:  4,
		IHL:      5,
		TTL:      64,
		Protocol: layers.IPProtocolUDP,
		SrcIP:    serverIP,
		DstIP:    net.IPv4(255, 255, 255, 255), // Broadcast IP
	}
	udpReply := &layers.UDP{
		SrcPort: 67,
		DstPort: 68,
	}
	udpReply.SetNetworkLayerForChecksum(ipv4Reply)

	buffer := gopacket.NewSerializeBuffer()
	options := gopacket.SerializeOptions{
		ComputeChecksums: true,
		FixLengths:       true,
	}
	gopacket.SerializeLayers(buffer, options,
		ethReply,
		ipv4Reply,
		udpReply,
		gopacket.Payload(replyDHCP.ToBytes()),
	)

	replyMsg := buffer.Bytes()

	fmt.Printf("[DHCP] Sending %d byte reply back to client MAC: %s\n", len(replyMsg), eth.SrcMAC.String())

	client.WriteMutex.Lock()
	werr := sendFrame(client, replyMsg)
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[DHCP] Reply send failed: %v\n", werr)
	}
}

// handleTCPDHCP is the TCP-ingest twin of handleDHCP: same DISCOVER/OFFER
// + REQUEST/ACK exchange, but the reply goes back over the TCP leg
// (handleDHCP's reply path writes to the WS connection, which a headless
// TCP client does not have). The UDP-forward listener setup is shared.
// NOTE: TCP-ingest boards use static fixture addressing, not DHCP — the
// worker sketch never sends DHCP — so this path is currently exercised
// only if a future worker image runs a real DHCP client. Kept in sync
// with handleDHCP by construction (same offer/ack builders below).
func handleTCPDHCP(msg []byte, packet gopacket.Packet, client *Client, room *Room) {
	handleDHCP(msg, packet, client, room)
}

// UDP port forward 127.0.0.1:<listen port> -> board:5683 (CoAP and other
// UDP server roles on the board). The VN Dial API is TCP-only, so frames
// are injected raw: 192.168.4.1:<alloc> -> board:5683 straight onto the
// room pipe. Board replies to 192.168.4.1:<alloc> are snooped in
// handleClient (main.go) and relayed to the host client — never entering
// gVisor, which has no listener for them.
func handleUDPProxy(uconn *net.UDPConn, targetIP string) {
	defer uconn.Close()
	buf := make([]byte, 2048)
	for {
		uconn.SetReadDeadline(time.Now().Add(60 * time.Second))
		n, addr, err := uconn.ReadFromUDP(buf)
		if err != nil {
			continue
		}
		udpFwdMu.Lock()
		l, ok := udpFwdListeners[targetIP]
		if !ok || l.room == nil {
			udpFwdMu.Unlock()
			continue
		}
		room, mac := l.room, l.mac
		udpFwdNextSport++
		if udpFwdNextSport < 40000 {
			udpFwdNextSport = 40000
		}
		sport := udpFwdNextSport
		udpFwdBySport[sport] = &udpFwdEntry{uconn: uconn, clientAddr: addr, last: time.Now()}
		udpFwdMu.Unlock()
		if derr := injectToVN(room, mac, targetIP, sport, buf[:n]); derr != nil {
			udpFwdMu.Lock()
			delete(udpFwdBySport, sport)
			udpFwdMu.Unlock()
		}
	}
}

// sendARPReply answers an ARP request for the gateway IP (192.168.4.1)
// directly on the requesting client (see main.go hub loop). gVisor answers
// ARPs itself, but its cold stack can take ~1s for the first one — long
// enough for the board to give up on its first reply of a session.
func sendARPReply(client *Client, req *layers.ARP) {
	ethReply := &layers.Ethernet{
		SrcMAC:       net.HardwareAddr{0x5a, 0x94, 0xef, 0xe4, 0x0c, 0xdd},
		DstMAC:       req.SourceHwAddress,
		EthernetType: layers.EthernetTypeARP,
	}
	arpReply := &layers.ARP{
		AddrType:          req.AddrType,
		Protocol:          req.Protocol,
		HwAddressSize:     req.HwAddressSize,
		ProtAddressSize:   req.ProtAddressSize,
		Operation:         layers.ARPReply,
		SourceHwAddress:   []byte{0x5a, 0x94, 0xef, 0xe4, 0x0c, 0xdd},
		SourceProtAddress: req.DstProtAddress,
		DstHwAddress:      req.SourceHwAddress,
		DstProtAddress:    req.SourceProtAddress,
	}
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethReply, arpReply); serr != nil {
		return
	}
	client.WriteMutex.Lock()
	werr := sendFrame(client, buffer.Bytes())
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[ARP] Reply send failed: %v\n", werr)
	}
}

// sendProxyARPReply answers an ARP request for ANY 192.168.4.0/24 address
// (except the gateway itself, which sendARPReply already handled) with the
// gateway MAC, so the board ARPs once and then sends the IP packet to us.
// This is what makes off-LAN DNS/UDP work through the gVisor stack: gVisor
// only answers ARP for addresses it owns (its gateway IP), so without this
// the board's ARP for 8.8.8.8 (or any internet IP) goes unanswered, the
// board never transmits the IP packet, and DNS/UDP/TCP to the outside
// world silently never happens (proven live 2026-09-29: DNS to 8.8.8.8
// got zero replies — only gVisor ARP-refresh broadcasts came back — until
// this proxy was added). Transport-agnostic via sendFrame (WS + TCP legs).
func sendProxyARPReply(client *Client, req *layers.ARP) {
	ethReply := &layers.Ethernet{
		SrcMAC:       net.HardwareAddr{0x5a, 0x94, 0xef, 0xe4, 0x0c, 0xdd},
		DstMAC:       req.SourceHwAddress,
		EthernetType: layers.EthernetTypeARP,
	}
	arpReply := &layers.ARP{
		AddrType:          req.AddrType,
		Protocol:          req.Protocol,
		HwAddressSize:     req.HwAddressSize,
		ProtAddressSize:   req.ProtAddressSize,
		Operation:         layers.ARPReply,
		SourceHwAddress:   []byte{0x5a, 0x94, 0xef, 0xe4, 0x0c, 0xdd},
		SourceProtAddress: req.DstProtAddress,
		DstHwAddress:      req.SourceHwAddress,
		DstProtAddress:    req.SourceProtAddress,
	}
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethReply, arpReply); serr != nil {
		return
	}
	client.WriteMutex.Lock()
	werr := sendFrame(client, buffer.Bytes())
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[ARP] Proxy reply send failed: %v\n", werr)
	} else {
		fmt.Printf("[ARP] Proxy reply for %v -> gw MAC\n", net.IP(req.DstProtAddress))
	}
}

// sendTCPARPReply is the TCP-ingest twin of sendARPReply (same packet,
// TCP-leg framing). Currently unused — handleTCPFrame calls sendARPReply
// directly, which now routes via sendFrame — kept as documentation that
// the ARP fast-reply path is transport-agnostic.
func sendTCPARPReply(client *Client, req *layers.ARP) {
	sendARPReply(client, req)
}

func divertUDPForward(packet gopacket.Packet, room *Room) bool {
	_ = room
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
	entry := udpFwdLookup(uint16(udp.DstPort))
	if entry == nil {
		return false
	}
	entry.last = time.Now()
	entry.uconn.WriteToUDP(udp.Payload, entry.clientAddr)
	fmt.Printf("[Port Forward] UDP relay %d bytes -> host client for sport %d\n", len(udp.Payload), uint16(udp.DstPort))
	return true
}
func injectToVN(room *Room, boardMAC string, targetIP string, sport uint16, payload []byte) error {
	// Crafts 192.168.4.1:<sport> -> board:5683 and writes it
	// length-prefixed onto the room pipe (same framing as the hub writer,
	// serialized by vnPipeMu).
	dstMAC, err := net.ParseMAC(boardMAC)
	if err != nil {
		return err
	}
	ethLayer := &layers.Ethernet{
		SrcMAC:       net.HardwareAddr{0x5a, 0x94, 0xef, 0xe4, 0x0c, 0xdd},
		DstMAC:       dstMAC,
		EthernetType: layers.EthernetTypeIPv4,
	}
	ipLayer := &layers.IPv4{
		Version:  4,
		IHL:      5,
		TTL:      64,
		Protocol: layers.IPProtocolUDP,
		SrcIP:    net.IPv4(192, 168, 4, 1),
		DstIP:    net.ParseIP(targetIP),
	}
	udpLayer := &layers.UDP{
		SrcPort: layers.UDPPort(sport),
		DstPort: 5683,
	}
	udpLayer.SetNetworkLayerForChecksum(ipLayer)
	buffer := gopacket.NewSerializeBuffer()
	opts := gopacket.SerializeOptions{ComputeChecksums: true, FixLengths: true}
	if serr := gopacket.SerializeLayers(buffer, opts, ethLayer, ipLayer, udpLayer, gopacket.Payload(payload)); serr != nil {
		return serr
	}
	frame := buffer.Bytes()
	room.Lock()
	pipe := room.PipeToVN
	room.Unlock()
	if pipe == nil {
		return fmt.Errorf("room pipe gone")
	}
	fmt.Printf("[Port Forward] UDP inject %d bytes -> %s sport %d\n", len(payload), targetIP, sport)
	vnPipeMu.Lock()
	defer vnPipeMu.Unlock()
	if werr := binary.Write(pipe, binary.BigEndian, uint32(len(frame))); werr != nil {
		return werr
	}
	_, werr := pipe.Write(frame)
	return werr
}

func handleProxy(clientConn net.Conn, targetIP string) {
	defer clientConn.Close()
	
	if globalVN == nil {
		return
	}
	
	destConn, err := globalVN.Dial("tcp", fmt.Sprintf("%s:80", targetIP))
	if err != nil {
		fmt.Printf("[Port Forward] Failed to connect to %s:80 - %v\n", targetIP, err)
		return
	}
	defer destConn.Close()

	var wg sync.WaitGroup
	wg.Add(2)

	go func() {
		defer wg.Done()
		io.Copy(destConn, clientConn)
		if cw, ok := destConn.(interface{ CloseWrite() error }); ok {
			cw.CloseWrite()
		}
	}()

	go func() {
		defer wg.Done()
		io.Copy(clientConn, destConn)
		if cw, ok := clientConn.(interface{ CloseWrite() error }); ok {
			cw.CloseWrite()
		}
	}()

	wg.Wait()
}

// ---- ICMPv4 echo (board -> gateway) -------------------------------------
// The worker sketch sends an ICMP echo request at 192.168.4.1; answer it
// here (gateway-sourced echo reply, checksums recomputed) instead of
// relying on gVisor NAT timing. Same transport-agnostic reply path as
// ARP (sendFrame picks WS vs TCP). Returns true when consumed (caller
// must skip the VN pipe AND the room broadcast for it).
var gwIPv4 = net.IPv4(192, 168, 4, 1)

func ipv4Checksum(hdr []byte) uint16 {
	sum := uint32(0)
	for i := 0; i+1 < len(hdr); i += 2 {
		sum += uint32(hdr[i])<<8 | uint32(hdr[i+1])
	}
	if len(hdr)%2 != 0 {
		sum += uint32(hdr[len(hdr)-1]) << 8
	}
	for sum>>16 != 0 {
		sum = (sum >> 16) + (sum & 0xffff)
	}
	return uint16(^sum)
}

func snoopICMPv4(msg []byte, client *Client, room *Room) bool {
	_ = room
	// Eth(14) + IPv4(20 min) + ICMP(8 min); ethertype 0x0800, proto 1,
	// ICMP type 8 (echo request), dst == gateway.
	if len(msg) < 14+20+8 {
		return false
	}
	if binary.BigEndian.Uint16(msg[12:14]) != 0x0800 || (msg[14]>>4) != 4 {
		return false
	}
	ihl := int(msg[14]&0x0F) * 4
	if ihl < 20 || len(msg) < 14+ihl+8 {
		return false
	}
	if msg[14+9] != 1 {
		return false
	}
	if msg[14+16] != 192 || msg[14+17] != 168 || msg[14+18] != 4 || msg[14+19] != 1 {
		return false
	}
	icmp := 14 + ihl
	if msg[icmp] != 8 {
		return false
	}
	// Build the echo reply: swap MACs + IPs, type 0, recompute both
	// checksums (IP header + ICMP). Payload (ident/seq/data) echoed.
	// NOTE (proven live 2026-09-28): the worker sketch's frame-2 leaves
	// the IPv4 header checksum ZERO (tap is L2) — but the ICMP checksum
	// field must ALSO be zero for `ipv4Checksum` to compute the correct
	// reply checksum here (sketch sends 0x0000, so this holds; stated
	// explicitly because a nonzero garbage field would silently produce
	// a wrong reply checksum the board would drop).
	rep := make([]byte, len(msg))
	copy(rep, msg)
	copy(rep[0:6], msg[6:12])
	copy(rep[6:12], gwMAC)
	copy(rep[14+12:14+16], msg[14+16:14+20]) // src = old dst (gw)
	copy(rep[14+16:14+20], msg[14+12:14+16]) // dst = old src (board)
	rep[14+10], rep[14+11] = 0, 0
	ck := ipv4Checksum(rep[14 : 14+ihl])
	rep[14+10] = byte(ck >> 8)
	rep[14+11] = byte(ck)
	rep[icmp] = 0 // echo reply
	rep[icmp+2], rep[icmp+3] = 0, 0
	ick := ipv4Checksum(rep[icmp:])
	rep[icmp+2] = byte(ick >> 8)
	rep[icmp+3] = byte(ick)
	client.WriteMutex.Lock()
	werr := sendFrame(client, rep)
	client.WriteMutex.Unlock()
	if werr != nil {
		fmt.Printf("[ICMPv4] Reply send failed: %v\n", werr)
	} else {
		fmt.Printf("[ICMPv4] Echo reply -> %x (%dB)\n", msg[6:12], len(rep))
	}
	return true
}
