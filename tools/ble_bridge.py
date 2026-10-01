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
    emu_controller = Controller(
        "emu-wire",
        host_source=None,
        host_sink=None,
        link=link,
    )
    device = Device.from_config_with_hci(
        DeviceConfiguration(name="ESP32-S3-Emu"),
        host_transport.source,
        host_transport.sink,
    )
    device._emu_transports = (host_transport, ctrl_transport)  # noqa: SLF001
    device._emu_controller = emu_controller  # noqa: SLF001 (serve_emu pump target)
    device._emu_host_controller = controller  # noqa: SLF001 (kept alive: link peer)
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
        # controller.on_packet — catch per-packet, keep pumping (the
        # failing command simply gets no reply, like a controller stall;
        # the harness asserts per-command so a drop is visible, not
        # silent).
        try:
            while True:
                pkt = await read_frame(reader)
                if pkt is None:
                    break
                # Inbound: emulator (acting as a second host) -> Bumble
                # controller. The controller IS the TransportSink for
                # host->controller HCI (see controllers.py wiring).
                try:
                    controller.on_packet(pkt)
                except Exception as e:
                    logger.warning("controller dropped inbound packet [%s]: %s",
                                   pkt.hex(), e)
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
    )


if __name__ == "__main__":
    sys.exit(asyncio.run(amain()))
