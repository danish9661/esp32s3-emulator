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

		client.WriteMutex.Lock()
		client.Conn.WriteMessage(websocket.TextMessage, []byte("BOARD_IP:"+assignedIP.String()))
		client.WriteMutex.Unlock()
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

					// Send structured messages to the frontend
					client.WriteMutex.Lock()
					client.Conn.WriteMessage(websocket.TextMessage, []byte("BOARD_IP:"+ipStr))
					client.Conn.WriteMessage(websocket.TextMessage, []byte(fmt.Sprintf("PORT_FORWARD:http://127.0.0.1:%d", port)))
					client.WriteMutex.Unlock()
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
	client.Conn.WriteMessage(websocket.BinaryMessage, replyMsg)
	client.WriteMutex.Unlock()
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

// divertUDPForward relays board -> 192.168.4.1:<alloc> replies to the
// mapped host client. Returns true when the frame was consumed (caller
// must skip the VN pipe AND the room broadcast for it).
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
	client.Conn.WriteMessage(websocket.BinaryMessage, buffer.Bytes())
	client.WriteMutex.Unlock()
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
