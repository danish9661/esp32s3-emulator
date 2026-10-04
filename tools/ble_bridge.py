#!/usr/bin/env python3
"""Bumble BLE virtual-controller bridge for the ESP32-S3 emulator.

Chain:
  emulated firmware (NimBLE host, VHCI tap in soc.rs: `bt_hci_*`)
    <-> run_flash BLE_GW=<host:port> (length-prefixed HCI framing)
    <-> this bridge (TCP server for the emulator + Bumble stack)
    <-> Bumble virtual controller (in-process, no radio, no root)

Why Bumble (not BlueZ): BlueZ needs a real radio + kernel VHCI + root.
Bumble is a pure-Python BLE stack whose controller is virtual — it runs
as a normal user process, which is exactly what an offline emulator
needs. HCI packet flow uses Bumble's own `PacketParser` (H4 type +
length rules per packet kind), so framing can never desync silently.

Protocol (emulator <-> bridge, both directions):
  4-byte big-endian length + one HCI packet (H4 type byte + payload).
  Same framing discipline as the NET_GW Ethernet leg (length-prefixed
  binary over TCP).

Bumble side (proven shape — see bumble/apps/controllers.py + scan.py):
  * LocalLink: shared simulated air between controllers.
  * Controller("emu-ctrl", host_source, host_sink, link): virtual LE
    controller on a cross-connected UDP-loopback transport pair
    (host->ctrl and ctrl->host, each transport's remote = the other's
    local — both locals pre-allocated via ephemeral binds).
  * Device.from_config_with_hci(DeviceConfiguration(name=...),
    hci_source, hci_sink): NimBLE-equivalent host + GATT server.
  * GATT app: service 0x180F (Battery) + 0x2A19 (Battery Level,
    read+notify, 100) + custom 128-bit echo (read+write, b"emu"), so a
    firmware GATT client observes discovery + read + write + notify
    without any phone/host.

Legs served:
  * --emu-port (default 9545): TCP server with LENGTH-PREFIXED HCI for
    the emulator's BLE_GW leg (run_flash dials it; run_flash also serves
    this framing). Bytes from the emulator feed the Bumble host side;
    Bumble answers flow back the same way.
  * --port (default 9544): TCP server with RAW HCI for the Go gateway's
    /api/ble-gateway proxy (main.go handleBLEGateway is a byte pump, so
    no length prefix here — Bumble's H4 PacketParser reframes the stream).

Usage:
  python3 tools/ble_bridge.py [--port 9544] [--emu-port 9545]
  python3 tools/ble_bridge.py --self-test   # boot device, print GATT, exit 0

Battery: `ble_bridge` (python harness asserts the GATT table + both TCP
legs accept). The firmware sketch is `tools/sketches/esp32s3_ble/`
(NimBLE GATT server: advertise -> write -> read -> notify, all through
`bt_hci_*` + BLE_GW).
"""

import argparse
import asyncio
import logging
import struct
import sys

logging.basicConfig(level=logging.INFO, format="[BLE] %(message)s")
logger = logging.getLogger(__name__)

# GATT demo app (stable UUIDs so firmware assertions are exact):
# service 0x180F (Battery Service) + characteristic 0x2A19 (Battery Level,
# read+notify, initial 100) + custom 128-bit echo (read+write, b"emu").
ECHO_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef0"


async def read_frame(reader: asyncio.StreamReader) -> bytes | None:
    """Length-prefixed HCI frame (4-byte BE length + packet)."""
    try:
        hdr = await reader.readexactly(4)
    except (asyncio.IncompleteReadError, ConnectionError):
        return None
    (n,) = struct.unpack(">I", hdr)
    if n == 0 or n > 4096:
        logger.warning("bad HCI frame length %d", n)
        return None
    try:
        return await reader.readexactly(n)
    except (asyncio.IncompleteReadError, ConnectionError):
        return None


def write_frame(writer: asyncio.StreamWriter, pkt: bytes) -> None:
    writer.write(struct.pack(">I", len(pkt)) + pkt)


# Opcodes short-circuited with a synthesized Command Complete (status 0)
# instead of reaching the Bumble controller (see serve_emu). Vendor OGF
# (0x3F) is matched by mask, not enumeration.
_SHORT_CIRCUIT_OPS = {
    0x204E,  # LE_SET_PRIVACY_MODE (NimBLE init sends it with a null
    # identity entry; Bumble 0.0.231 reports it Unsupported)
    0x041D,  # READ_REMOTE_VERSION_INFORMATION (Link Control OGF — Bumble
    # 0.0.231 reports it Unsupported, but the NimBLE conn handler sends
    # it right after connect (ble_gap_rd_rem_ver_tx) and RESETS THE HOST
    # (`ble_hs_sched_reset`) when its 0x7d0-tick waiter times out
    # ("HCI wait for ack returned 19", proven live 2026-10-03 — no
    # onConnect, no ATT response, permanent stall). The CC alone wakes
    # the waiter; the Read-Remote-Version Complete event (0x0C) follows
    # separately (see _VERSION_COMPLETE_EVENT below) in case the handler
    # pends on it too.
}

# Read-Remote-Version Complete event (0x0C) sent after the 0x041D CC:
# status 0, handle 1, version 0x09 (BT 5.0, ESP32-S3's headline), company
# Espressif 0x02E5, subversion 0. The sketch never reads the version —
# these bytes only need to satisfy the handler's parse, not match silicon
# exactly.
_VERSION_COMPLETE_EVENT = bytes((
    0x04, 0x0C, 0x08, 0x00,
    0x01, 0x00, 0x09, 0xE5, 0x02, 0x00, 0x00,
))


def short_circuit_reply(pkt: bytes, force: bool = False) -> bytes | None:
    """Synthesize a Command Complete (status 0) for commands Bumble cannot
    answer, else None (forward to the controller).

    Rules: H4 CMD (0x01) with vendor OGF (0x3F) or in _SHORT_CIRCUIT_OPS
    always short-circuits; anything else returns None unless `force`
    (the controller raised on it — answer rather than stall). Reply is
    the minimal 7-byte CC: [04, 0E, 04, 01, op_lo, op_hi, 00]. Commands
    with mandatory return params (e.g. Read BD ADDR) must NOT be
    short-circuited blind — they are never in the set and `force` only
    fires after a controller raise, which is logged either way.
    """
    if len(pkt) < 4 or pkt[0] != 0x01:
        return None
    opcode = pkt[1] | (pkt[2] << 8)
    vendor = (opcode & 0xFC00) == 0xFC00
    if not (vendor or opcode in _SHORT_CIRCUIT_OPS or force):
        return None
    logger.info("short-circuit op=%#06x (%s): CC status=0",
                opcode, "vendor" if vendor else ("forced" if force else "listed"))
    return bytes((0x04, 0x0E, 0x04, 0x01, opcode & 0xFF, (opcode >> 8) & 0xFF, 0x00))


async def run_bumble_device() -> object:
    """Start the virtual controller + GATT app. Returns the Device."""
    from bumble.device import Device, DeviceConfiguration
    from bumble.controller import Controller
    from bumble import gatt

    # Cross-connected UDP-loopback transport pair (each transport's
    # remote = the OTHER transport's local; both locals pre-allocated
    # via ephemeral binds to avoid the post-bind getsockname race).
    import socket as _socket

    from bumble.link import LocalLink
    from bumble.transport import open_transport

    def _free_port() -> int:
        s = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        p = s.getsockname()[1]
        s.close()
        return p

    link = LocalLink()
    # TEMP (2026-10-03): count link-layer ADV PDUs per sender name.
    _adv_count: dict = {}
    _orig_send_adv = link.send_advertising_pdu

    def _counting_send_adv(sender, *args, **kwargs):
        _adv_count[getattr(sender, "name", "?")] = _adv_count.get(getattr(sender, "name", "?"), 0) + 1
        return _orig_send_adv(sender, *args, **kwargs)

    link.send_advertising_pdu = _counting_send_adv  # type: ignore[method-assign]
    # TEMP (2026-10-03): log every link-layer ACL transfer (central ATT
    # path observability — the GATT timeout means the ATT request never
    # became an emu-wire HCI ACL; this shows which side drops it).
    _orig_send_acl = link.send_acl_data

    def _logging_send_acl(sender, receiver, transport, pdu, *args, **kwargs):
        try:
            logger.info("LINK-ACL %s -> %s len=%d %s",
                        getattr(sender, "name", "?"),
                        getattr(receiver, "name", "?") if hasattr(receiver, "name") else receiver,
                        len(pdu), bytes(pdu).hex()[:96])
        except Exception:
            pass
        return _orig_send_acl(sender, receiver, transport, pdu, *args, **kwargs)

    link.send_acl_data = _logging_send_acl  # type: ignore[method-assign]
    host_port, ctrl_port = _free_port(), _free_port()
    host_transport = await open_transport(
        f"udp:127.0.0.1:{host_port},127.0.0.1:{ctrl_port}"
    )
    ctrl_transport = await open_transport(
        f"udp:127.0.0.1:{ctrl_port},127.0.0.1:{host_port}"
    )
    # Keep handles alive for the process lifetime (attribute on the
    # device below); closing either transport drops the host<->ctrl link.
    controller = Controller(
        "emu-ctrl",
        host_source=ctrl_transport.source,
        host_sink=ctrl_transport.sink,
        link=link,
    )
    # Second controller on the SAME link: the emulator's wire peer. Its
    # host side is never constructed — the emulator leg pumps HCI
    # directly into/out of it (see serve_emu). Because it shares the
    # LocalLink with emu-ctrl, Bumble's link layer delivers LE
    # advertisements/connections between the two controllers, while the
    # device's own host<->ctrl pair stays private (no response-stealing:
    # the device host only ever sees completions for commands IT sent).
    # NOTE: Bumble Controller defaults public_address to 00:00:00:00:00:00
    # when unset — the emu-ctrl (bridge device) and emu-wire (firmware)
    # controllers would SHARE it, and the central scan drops emu-wire's
    # ADV_IND as its own (proven live 2026-10-03: 6/6 CENTRAL_SCAN misses
    # with valid ADV_ENABLE on the link; isolated replay advertised
    # fine). Distinct addresses here (firmware set a RANDOM address via
    # 0x2005 but advertises own=PUBLIC, so this public address is what
    # the central actually sees).
    emu_controller = Controller(
        "emu-wire",
        host_source=None,
        host_sink=None,
        link=link,
        public_address="13:37:13:37:13:37",
    )
    device = Device.from_config_with_hci(
        DeviceConfiguration(name="ESP32-S3-Emu"),
        host_transport.source,
        host_transport.sink,
    )
    device._emu_transports = (host_transport, ctrl_transport)  # noqa: SLF001
    device._emu_controller = emu_controller  # noqa: SLF001 (serve_emu pump target)
    device._adv_count = _adv_count  # noqa: SLF001 (TEMP link counter)
    device._emu_host_controller = controller  # noqa: SLF001 (TEMP scan-side)
    # TEMP (2026-10-03): count link->scanner PDU deliveries.
    _rx_count = {"n": 0}
    _orig_ll = controller.on_ll_advertising_pdu
    _ll_name = type(controller).on_ll_advertising_pdu

    def _counting_ll(*args, **kwargs):
        _rx_count["n"] += 1
        return _orig_ll(*args, **kwargs)

    controller.on_ll_advertising_pdu = _counting_ll  # type: ignore[method-assign]
    device._rx_count = _rx_count  # noqa: SLF001 (TEMP)
    # LINK-ADDRESS FALLBACK (proven live 2026-10-03): Bumble's
    # `LocalLink.send_acl_data` always stamps LE data with the sender's
    # RANDOM address, but the connection table is keyed by the address the
    # central actually used in CONNECT_IND (public) — so the peripheral's
    # `on_link_acl_data` lookup misses and the ATT request is dropped with
    # "!!! no connection" (observed: LINK-ACL emu-ctrl -> emu-wire logged,
    # no EMU-OUT ACL ever emitted, central GATT timeout). With exactly one
    # firmware peer on this link, falling back to the sole connection is
    # sound (no ambiguity possible).
    _orig_wire_acl = emu_controller.on_link_acl_data

    def _wire_acl_with_fallback(sender_address, transport, data, *args, **kwargs):
        try:
            logger.info("WIRE-ACL got %s len=%d conns=%d",
                        sender_address, len(data), len(emu_controller.le_connections))
        except Exception:
            pass
        try:
            if transport is not None:
                import bumble.hci as _hci
                from bumble.core import PhysicalTransport as _PT
                if transport == _PT.LE and sender_address not in emu_controller.le_connections:
                    conns = list(emu_controller.le_connections.values())
                    if len(conns) == 1:
                        logger.info("WIRE-ACL fallback to sole connection (was %s)", sender_address)
                        sender_address = next(iter(emu_controller.le_connections.keys()))
        except Exception as e:
            logger.info("WIRE-ACL fallback error %s", e)
        return _orig_wire_acl(sender_address, transport, data, *args, **kwargs)

    emu_controller.on_link_acl_data = _wire_acl_with_fallback  # type: ignore[method-assign]
    # TEMP (2026-10-03): log every emu-wire controller→host emission to see
    # whether the ATT ACL ever reaches EmuTap (send_hci_packet is the single
    # funnel for events AND acl packets).
    _orig_wire_send = emu_controller.send_hci_packet

    def _wire_send_logging(pkt, *args, **kwargs):
        try:
            _b = bytes(pkt)
            logger.info("WIRE-SEND h4=%#04x len=%d %s",
                        _b[0] if len(_b) else -1, len(_b), _b.hex()[:96])
        except Exception as e:
            logger.info("WIRE-SEND error %s", e)
        return _orig_wire_send(pkt, *args, **kwargs)

    emu_controller.send_hci_packet = _wire_send_logging  # type: ignore[method-assign]
    # TEMP (2026-10-03): count HCI packets emu-ctrl emits toward its host,
    # split by LE Advertising Report events vs everything else.
    _ev_count: dict = {"n": 0, "adv": 0}
    _orig_send = controller.send_hci_packet

    def _counting_send(pkt, *args, **kwargs):
        _ev_count["n"] += 1
        try:
            import bumble.hci as _hci
            if isinstance(pkt, _hci.HCI_LE_Advertising_Report_Event):
                _ev_count["adv"] += 1
        except Exception:
            pass
        return _orig_send(pkt, *args, **kwargs)

    controller.send_hci_packet = _counting_send  # type: ignore[method-assign]
    device._ev_count = _ev_count  # noqa: SLF001 (TEMP)
    # NOTE: device._emu_host_controller is (re-)stashed below (kept alive).
    await device.power_on()

    # GATT app: Battery Service + echo characteristic.
    svc_uuid = gatt.UUID("180F")
    level_char = gatt.Characteristic(
        "2A19",
        gatt.Characteristic.Properties.READ | gatt.Characteristic.Properties.NOTIFY,
        gatt.Characteristic.Permissions.READABLE,
        bytes([100]),
    )
    echo_char = gatt.Characteristic(
        ECHO_CHAR_UUID,
        gatt.Characteristic.Properties.READ | gatt.Characteristic.Properties.WRITE,
        gatt.Characteristic.Permissions.READABLE
        | gatt.Characteristic.Permissions.WRITEABLE,
        b"emu",
    )
    svc = gatt.Service(svc_uuid, [level_char, echo_char])
    device.add_service(svc)

    await device.start_advertising(auto_restart=True)
    logger.info("Bumble device up: advertising as ESP32-S3-Emu (Battery + echo)")
    return device


async def serve_emu(device: object, port: int) -> None:
    """TCP server for the emulator BLE_GW leg (length-prefixed HCI).

    run_flash dials this as a client; each frame feeds the Bumble host
    side (device.host receives controller-bound HCI), and Bumble answers
    flow back length-prefixed. The pump taps the host's controller_sink:
    wrap it so every outbound packet is ALSO forwarded to the emulator.
    """

    # The wire controller is stashed on the device by run_bumble_device.
    # It has NO host attached (host_source/host_sink None), so every
    # command the emulator sends gets its Command Complete delivered to
    # `controller.host` — which serve_emu wraps with the EmuTap below.
    # Nothing else consumes these completions (the device's own private
    # host<->ctrl pair is untouched), so no response is ever stolen.
    controller = device._emu_controller  # type: ignore[attr-defined]

    async def on_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        peer = writer.get_extra_info("peername")
        logger.info("emulator connected: %s", peer)
        # Tap outbound controller->host packets back to the emulator by
        # wrapping the CONTROLLER's host sink (Host.set_packet_sink
        # stores it as hci_sink; the attribute may differ by version,
        # so resolve defensively).
        host = device.host  # type: ignore[attr-defined]
        orig_sink = getattr(controller, "host", None)
        loop = asyncio.get_running_loop()

        class EmuTap:
            def on_packet(self, packet: bytes) -> None:
                if orig_sink is not None:
                    orig_sink.on_packet(packet)
                # TEMP (2026-10-03): log every controller→emulator packet so
                # the RX synthesis path is observable (H4 + len + hex).
                try:
                    logger.info("EMU-OUT h4=%#04x len=%d %s",
                                packet[0] if len(packet) else -1,
                                len(packet), bytes(packet).hex())
                except Exception:
                    pass
                # Controller->host replies are SYNC here (send_hci_packet
                # uses call_soon on the same loop): write the
                # length-prefixed frame IMMEDIATELY — deferring via
                # call_soon races the test client's read window and the
                # reply lands after the harness already timed out
                # (proven live: LE_SET_EVENT_MASK TIMEOUT with deferred
                # write, PASS with inline write).
                try:
                    write_frame(writer, packet)
                except Exception as e:
                    logger.warning("emu write failed: %s", e)

            def close(self) -> None:
                pass

        controller.host = EmuTap()  # type: ignore[attr-defined]
        # Inbound parse errors must not kill the leg: a malformed HCI
        # packet from firmware (or a Bumble-version quirk like the
        # Read-BD-ADDR address-length parse) raises inside
        # controller.on_packet — catch per-packet, keep pumping.
        #
        # SHORT-CIRCUIT (Bumble hci_bridge.py pattern): commands Bumble
        # cannot answer get a synthesized Command Complete (status 0)
        # instead of a stall — proven live 2026-10-03: NimBLE init sends
        # ESP32 vendor commands (OGF 0x3F, e.g. 0xFC01) plus
        # LE_SET_PRIVACY_MODE, Bumble raised ("Unsupported command",
        # "index out of range") and the leg stalled with no reply.
        # Short-circuited opcodes: ALL vendor OGF (0x3F) commands plus an
        # explicit list, plus anything else that raises (fallback). Only
        # commands (H4 type 0x01) are short-circuited — ACL data always
        # goes to the controller. Every short-circuit is logged (a drop
        # would be silent; a fake SUCCESS is visible in firmware behavior).
        try:
            while True:
                pkt = await read_frame(reader)
                if pkt is None:
                    break
                # Inbound: emulator (acting as a second host) -> Bumble
                # controller. The controller IS the TransportSink for
                # host->controller HCI (see controllers.py wiring).
                if len(pkt) >= 3 and pkt[0] == 0x01:
                    _op = pkt[1] | (pkt[2] << 8)
                    if _op in (0x2005, 0x2006, 0x2008, 0x2009, 0x200A):
                        logger.info("ADV-CMD op=%#06x params=%s", _op, pkt[4:].hex())
                reply = short_circuit_reply(pkt)
                if reply is not None:
                    try:
                        write_frame(writer, reply)
                    except Exception as e:
                        logger.warning("emu write failed: %s", e)
                    # Read-Remote-Version needs its Complete event too
                    # (0x0C): the handler may pend on it after the CC.
                    # Send it slightly later (silicon delivers it when the
                    # remote answers, i.e. after the command completes —
                    # back-to-back preserves the order without racing the
                    # CC through the firmware's dispatcher).
                    if len(pkt) >= 3 and (pkt[1] | (pkt[2] << 8)) == 0x041D:
                        async def _send_ver(w=writer):
                            try:
                                await asyncio.sleep(0.2)
                                write_frame(w, _VERSION_COMPLETE_EVENT)
                                logger.info("short-circuit 0x041d version event sent")
                            except Exception as e:
                                logger.warning("version event write failed: %s", e)

                        asyncio.get_running_loop().create_task(_send_ver())
                    continue
                try:
                    controller.on_packet(pkt)
                except Exception as e:
                    logger.warning("controller dropped inbound packet [%s]: %s",
                                   pkt.hex(), e)
                    fb = short_circuit_reply(pkt, force=True)
                    if fb is not None:
                        try:
                            write_frame(writer, fb)
                        except Exception as e2:
                            logger.warning("emu write failed: %s", e2)
        except ConnectionError:
            pass
        finally:
            controller.host = orig_sink  # type: ignore[attr-defined]
        logger.info("emulator disconnected: %s", peer)

    server = await asyncio.start_server(on_client, "127.0.0.1", port)
    logger.info("BLE emulator leg serving on 127.0.0.1:%d (length-prefixed HCI)", port)
    async with server:
        await server.serve_forever()


async def serve_gateway(device: object, port: int) -> None:
    """TCP server for the Go gateway /api/ble-gateway proxy (port 9544).

    Framing on THIS leg is raw HCI (no length prefix — the Go proxy is a
    byte pump, see main.go handleBLEGateway). Each connection gets a pump
    both directions; packets are complete HCI frames as Bumble emits them
    (H4 type + header + body), so a streaming pump is framing-safe.
    """
    _ = device

    async def on_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        peer = writer.get_extra_info("peername")
        logger.info("gateway proxy connected: %s", peer)
        logger.info("GATT ready: svc=180F chars=[2A19, %s]", ECHO_CHAR_UUID)
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                # Loopback until the SoC VHCI tap lands: proves the proxy
                # path end-to-end (bytes in == bytes out, framing intact).
                writer.write(data)
                await writer.drain()
        except ConnectionError:
            pass
        logger.info("gateway proxy disconnected: %s", peer)

    server = await asyncio.start_server(on_client, "127.0.0.1", port)
    logger.info("BLE gateway proxy serving on 127.0.0.1:%d", port)
    async with server:
        await server.serve_forever()


async def amain() -> None:
    ap = argparse.ArgumentParser(description="Bumble BLE bridge for the ESP32-S3 emulator")
    ap.add_argument("--port", type=int, default=9544)
    ap.add_argument("--emu-port", type=int, default=9545)
    ap.add_argument("--self-test", action="store_true",
                    help="boot the Bumble device, print the GATT layout, exit 0")
    args = ap.parse_args()

    device = await run_bumble_device()
    if not hasattr(device, "_emu_controller"):
        # run_bumble_device always stashes it; defensive for API drift.
        raise RuntimeError("run_bumble_device did not stash _emu_controller")
    if args.self_test:
        # Deterministic proof without any peer: walk the GATT table.
        print("BLE SELF-TEST GATT services:", len(device.gatt_server.services))  # type: ignore[attr-defined]
        for svc in device.gatt_server.services:  # type: ignore[attr-defined]
            print("BLE SELF-TEST svc:", svc.uuid)
            for ch in svc.characteristics:
                print("BLE SELF-TEST char:", ch.uuid, "props=", ch.properties)
        print("BLE SELF-TEST PASS")
        return
    await asyncio.gather(
        serve_emu(device, args.emu_port),
        serve_gateway(device, args.port),
        run_central(device),
    )


async def run_central(device: object) -> None:
    """Bumble CENTRAL against the emulated firmware's advertiser.

    The firmware (NimBLE GATT server "ESP32-S3-BLE", service 180F) talks
    HCI to the emu-wire controller, which shares the LocalLink with this
    device — so this central can scan, connect, and run GATT against REAL
    firmware with no phone. Every ATT request travels link -> emu-wire ->
    emulator leg -> firmware host (RX synthesis under test); every
    response travels back (TX tap). Verdict lines:
    CENTRAL_SCAN (advertiser found), CENTRAL_CONN (connected),
    CENTRAL_READ_<n> (Battery Level bytes), CENTRAL_WRITE (echo write ok),
    CENTRAL_ECHO_<hex> (echo read-back), CENTRAL_PASS / CENTRAL_FAIL_<why>.
    Runs until CENTRAL_PASS (rounds retry forever): each round scans up to
    10 attempts; a failed GATT round (firmware run ended mid-exchange,
    ATT timeout, no service) starts a new round instead of exiting — the
    emulator runs come and go (fixed STEPS budgets), the central must
    outlive them. The emulator may reboot under it without harm.
    """
    import asyncio as _asyncio

    from bumble.device import AdvertisingData as _ADDATA

    found: dict = {}

    def _on_adv(adv) -> None:
        # NOTE (2026-10-03 bug): this handler runs INSIDE the pyee emit —
        # ANY exception here (e.g. bytes() on an already-str name) kills
        # the dispatch and the sighting is lost silently apart from an
        # asyncio "Exception in on_packet" traceback. Keep it total.
        try:
            raw = adv.data.get(_ADDATA.COMPLETE_LOCAL_NAME, b"")
            name = bytes(raw) if isinstance(raw, (bytes, bytearray)) else str(raw).encode()
        except Exception:
            name = b""
        logger.info("CENTRAL seen adv addr=%s name=%r connectable=%s",
                    adv.address, name, adv.is_connectable)
        if name == b"ESP32-S3-BLE" and "adv" not in found:
            found["adv"] = adv
        elif (name != b"ESP32-S3-Emu" and adv.is_connectable
                and "fb" not in found):
            # Fallback: any foreign connectable advertiser (the emu-wire
            # controller may advertise without the firmware's name if
            # ADV_DATA went to the ROM loopback instead).
            found["fb"] = adv

    device.on("advertisement", _on_adv)  # type: ignore[attr-defined]
    await _asyncio.sleep(12)  # firmware boot + advertise (emulated time)
    # Scan ONCE and leave it running: stop→start cycling breaks Bumble
    # 0.0.231 scanning (single-start isolated scans see everything; the
    # attempt loop's stop/start never saw a thing — attempt 0 was
    # legitimately empty, masking the breakage).
    try:
        await device.start_scanning(filter_duplicates=True)  # type: ignore[attr-defined]
    except Exception as e:
        logger.warning("CENTRAL scan error: %s", e)
        return
    round_no = 0
    while True:
        round_no += 1
        logger.info("CENTRAL round %d", round_no)
        found.clear()
        for attempt in range(10):
            # TEMP (2026-10-03): dump emu-wire advertising state.
            try:
                _w = device._emu_controller  # type: ignore[attr-defined]
                _adv = getattr(_w, "le_legacy_advertiser", None)
                logger.info("WIRE-STATE adv_enabled=%s adv_sets=%s rand=%s",
                            getattr(_adv, "enabled", "?"),
                            {k: getattr(v, "enabled", "?") for k, v in
                             getattr(_w, "advertising_sets", {}).items()},
                            getattr(_w, "_random_address", "?"))
                try:
                    logger.info("LINK-ADV %s", dict(device._adv_count))  # type: ignore[attr-defined]
                except Exception as e:
                    logger.info("LINK-ADV error %s", e)
                try:
                    _hc = device._emu_host_controller  # type: ignore[attr-defined]
                    logger.info("SCAN-STATE enable=%s rx_pdus=%s ev=%s",
                                getattr(_hc, "le_scan_enable", "?"),
                                device._rx_count,  # type: ignore[attr-defined]
                                device._ev_count)  # type: ignore[attr-defined]
                except Exception as e:
                    logger.info("SCAN-STATE error %s", e)
            except Exception as e:
                logger.info("WIRE-STATE error %s", e)
            await _asyncio.sleep(8)
            adv = found.get("adv") or found.get("fb")
            if adv is None:
                logger.info("CENTRAL_SCAN miss (attempt %d)", attempt)
                await _asyncio.sleep(5)
                continue
            logger.info("CENTRAL_SCAN addr=%s%s", adv.address,
                        "" if found.get("adv") else " (fallback unnamed)")
            try:
                connection = await device.connect(adv.address)  # type: ignore[attr-defined]
            except Exception as e:
                logger.warning("CENTRAL_FAIL_CONNECT %s", e)
                await _asyncio.sleep(5)
                continue
            logger.info("CENTRAL_CONN ok")
            # ADDRESS ALIAS (2026-10-04, proven live via `!!! no connection
            # for ...` drops): the firmware programs a RANDOM controller
            # address via 0x2005 while advertising own=PUBLIC, so central
            # connects to 13:37.../P but link data arrives sourced from the
            # random address (e.g. CE:D0:...) and emu-ctrl's
            # le_connections.get() misses — every firmware→central ATT
            # response is dropped before the host. Alias the live connection
            # under emu-wire's current random address (re-aliased per
            # connect; same-run stable). Harness-only (model untouched).
            try:
                _emu_wire = device._emu_controller  # type: ignore[attr-defined]
                _emu_host_ctrl = device._emu_host_controller  # type: ignore[attr-defined]
                _rand = _emu_wire.random_address
                _emu_host_ctrl.le_connections[_rand] = connection
                logger.info("CENTRAL_ALIAS %s ok", str(_rand))
            except Exception as e:
                logger.warning("CENTRAL_ALIAS failed: %s", e)
            # SAME-CONNECTION ATT RETRY (2026-10-04, proven live): the first
            # ATT (service discovery) goes out immediately after connect,
            # before the firmware finishes version/features/data-length —
            # the emulator drops it pre-conn (head-of-line fix) and the
            # read times out (FAIL_GATT). The link itself is FINE (conn
            # stable, no disc) — reconnecting abandons a good link and
            # stalls in re-handshake. Retry discovery + read on the SAME
            # connection (bounded) before giving up to a fresh round.
            # A failed retry disconnects cleanly so the next round starts
            # fresh (no half-open stall).
            gatt_ok = False
            for gatt_try in range(5):
                try:
                    from bumble.device import Peer as _Peer

                    async with _Peer(connection) as peer:
                        await peer.discover_services()
                        lvl = None
                        echo = None
                        for svc in peer.services:
                            if str(svc.uuid).upper() == "180F":
                                await svc.discover_characteristics()
                                for ch in svc.characteristics:
                                    u = str(ch.uuid).upper()
                                    if u == "2A19":
                                        lvl = ch
                                    if "12345678-1234-5678-1234-56789ABCDEF0" in u:
                                        echo = ch
                        if lvl is None:
                            logger.warning("CENTRAL_FAIL_NOSVC (round %d, try %d)", round_no, gatt_try)
                            break
                        val = await lvl.read_value()
                        logger.info("CENTRAL_READ_%s", val.hex())
                        if echo is not None:
                            await echo.write_value(b"hi!")
                            logger.info("CENTRAL_WRITE ok")
                            back = await echo.read_value()
                            logger.info("CENTRAL_ECHO_%s", back.hex())
                        logger.info("CENTRAL_PASS")
                        gatt_ok = True
                        break
                except Exception as e:
                    logger.warning("CENTRAL_FAIL_GATT %s (round %d, try %d, retrying same conn)", e, round_no, gatt_try)
                    await _asyncio.sleep(5)
                    continue
            if gatt_ok:
                return
            # Same-conn retries exhausted — disconnect cleanly so the next
            # round starts fresh (no half-open stall).
            try:
                await connection.disconnect()
            except Exception:
                pass
            found.clear()
            continue
        logger.warning("CENTRAL_FAIL_NOSCAN (round %d, retrying)", round_no)


if __name__ == "__main__":
    sys.exit(asyncio.run(amain()))
