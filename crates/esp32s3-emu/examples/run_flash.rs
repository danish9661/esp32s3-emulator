//! Host runner for real firmware flash images (Arduino CLI / ESP-IDF merged
//! images).  Usage: `cargo run -p esp32s3-emu --example run_flash -- <merged.bin>`.
//!
//! Boots the image via `Esp32S3::boot_from_flash`, steps it, and prints UART0
//! console output plus a stall/exception report so real-firmware boot can be
//! validated without QEMU.

use std::env;
use std::fs;
use std::time::Instant;

use esp32s3_emu::Esp32S3;
use xtensa_core::cpu::SR_EPC1;
use xtensa_core::{Bus, StepResult};

/// Describe the faulting instruction behind an Unimplemented trap (these
/// are ee.* DSP/TIE extensions: recognized by the decoder but without an
/// execution model).
fn unimp_detail(m: &mut Esp32S3, core: usize) -> String {
    use xtensa_core::generated::{decode_inst, decode_inst16a, decode_inst16b, insn_len};
    let pc = m.cpu[core].pc;
    let raw = m.soc.read32(pc);
    let b0 = (raw & 0xFF) as u8;
    let opc = match insn_len(b0) {
        2 => {
            let w = raw & 0xFFFF;
            if b0 & 0xf <= 11 {
                decode_inst16a(w)
            } else {
                decode_inst16b(w)
            }
        }
        _ => decode_inst(raw),
    };
    match opc {
        Some(o) if xtensa_core::ee::is_ee_opcode(o) => {
            // TIE/DSP trap: name the hardware unit, not just the mnemonic.
            format!(
                "{} [ee:{}] (raw {raw:#010x})",
                o.name(),
                xtensa_core::ee::ee_family(o)
            )
        }
        Some(o) => format!("{} (raw {raw:#010x})", o.name()),
        None => format!("undecodable (raw {raw:#010x})"),
    }
}

// TEMP (2026-10-04, forensics — DELETE after): pc-blacklist for BLE
// synthetic delivery (see gate docs). True when `pc` lies inside a
// pool/queue/scheduler critical function (nm on the BLE ELF, 2026-10-04).
fn ble_in_crit(pc: u32) -> bool {
    const RANGES: [(u32, u32); 14] = [
        (0x4038_03d4, 0x4038_04d8), // xQueueGenericSend
        (0x4038_057c, 0x4038_0628), // xQueueGenericSendFromISR
        (0x4038_0628, 0x4038_06bc), // xQueueGiveFromISR
        (0x4038_06bc, 0x4038_0784), // xQueueReceive
        (0x4038_0aa0, 0x4038_0be8), // xPortEnterCriticalTimeout
        (0x4038_0be8, 0x4038_0c7c), // vPortExitCritical
        (0x4038_1aec, 0x4038_1d74), // vTaskSwitchContext
        (0x4201_44b4, 0x4201_44d4), // ble_transport_alloc_evt
        (0x4201_44d4, 0x4201_44ec), // ble_transport_alloc_acl_from_ll
        (0x4201_44ec, 0x4201_4540), // ble_transport_free
        (0x4201_4e70, 0x4201_4e8c), // os_mbuf_prepend_pullup
        (0x4201_504c, 0x4201_50b8), // os_memblock_get + put_from_cb
        (0x4201_50b8, 0x4201_50f4), // os_memblock_put
        (0x4201_529c, 0x4201_52f8), // npl_freertos_eventq_put
    ];
    RANGES.iter().any(|&(s, e)| pc >= s && pc < e)
}

fn main() {
    let path = env::args().nth(1).expect("usage: run_flash <flash image>");
    let flash = fs::read(&path).expect("read flash image");
    // Image layout selection for the WiFi fixtures (scan vs STA sketch
    // link the pool/RAM differently; used by the layout tables below).
    let bin_name_contains_wifi_sta = path.contains("wifi_sta")
        || path.contains("test_worker_net")
        || path.contains("test_worker_l3")
        || path.contains("coex");

    let mut m = Esp32S3::new();
    // Secure-boot fixture: SECURE_BOOT_EN=1 burns eFuse SECURE_BOOT_EN
    // (BLK0 word 5 = REPEAT_DATA4 bit 20) through the real PGM path before
    // boot, so `boot_from_flash` verifies the app region's signature sector
    // (fail-closed: unsigned/bad-signature boots park the CPUs with no
    // output). Pair with a genuinely `espsecure.py sign-data`-signed image
    // (see tools/sketches/esp32s3_secure_boot/) for the allow path.
    if env::var("SECURE_BOOT_EN").is_ok() {
        const EFUSE_BASE: u32 = 0x6000_7000;
        for (i, w) in [0u32, 0, 0, 0, 0, 1 << 20, 0, 0].iter().enumerate() {
            m.soc.write32(EFUSE_BASE + (i as u32) * 4, *w);
        }
        m.soc.write32(EFUSE_BASE + 0x1D4, 0x2); // PGM bit, BLK_NUM 0
        println!("[host] SECURE_BOOT_EN burned (fail-closed gate armed)");
    }
    // Flash-encryption fixture: FLASHENC_KEY=<64 hex> provisions the eFuse
    // XTS key + crypt count, then uniformly encrypts the whole image
    // (every 16-byte block at its absolute offset, like esptool) before
    // boot. The firmware boots and runs on decrypted plaintext.
    let flashenc: Option<Vec<u8>> = env::var("FLASHENC_KEY").ok().map(|hex| {
        assert_eq!(hex.len(), 64, "FLASHENC_KEY must be 64 hex chars");
        (0..32)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex key"))
            .collect()
    });
    if let Some(key) = &flashenc {
        let mut key32 = [0u8; 32];
        key32.copy_from_slice(key);
        m.soc.flashenc_provision(&key32);
        println!("[host] flash encryption provisioned");
    }
    let boot_image: Vec<u8>;
    let boot_ref: &[u8] = if flashenc.is_some() {
        assert_eq!(flash.len() % 16, 0, "image length must be 16-byte aligned");
        // Encrypt a copy through the backing (provisioned above), then boot
        // the ciphertext. load+encrypt here mirrors the factory flow.
        m.soc.load_flash_image(0, &flash);
        m.soc.flashenc_encrypt_region(0, flash.len() as u32);
        boot_image = m.soc.flash_image().to_vec();
        println!("[host] flash image encrypted ({} bytes)", boot_image.len());
        &boot_image
    } else {
        &flash
    };
    m.boot_from_flash(boot_ref);

    // WiFi fixture layout: program the image-specific addresses into the
    // SoC once, before any fixture call (scan vs STA vs AP vs ESP-NOW
    // sketch link the pool and RAM differently; the SoC holds no image
    // addresses itself). The test-worker-net sketch links the STA-side
    // cells identically to wifi-sta except the event vars / scan_start /
    // connect / transmit pcs (nm on its own ELF — see WORKER_LAYOUT).
    // `wifi_image` selects the per-image hook tables in BOTH engines
    // (run_flash layout tables here + the SoC fixture engine + the
    // machine.rs TX-tap/delete-site gates); the default Scan image is
    // wrong for non-scan runs (its event vars point at the scan image's
    // event loop instances — posts would land in dead queues). Every
    // image therefore selects its engine table here, then programs the
    // run_flash layout cells (same five cells the engine's own
    // `wifi_fixture_layout_program` writes — see soc.rs).
    {
        let layout = if path.contains("wifi_ap") {
            m.soc.wifi_fixture_image_ap();
            &AP_LAYOUT
        } else if path.contains("test_worker_l3") {
            m.soc.wifi_fixture_image_worker_l3();
            &WORKER_L3_LAYOUT
        } else if path.contains("coex") {
            m.soc.wifi_fixture_image_coex();
            &COEX_LAYOUT
        } else if path.contains("test_worker_net") {
            m.soc.wifi_fixture_image_worker();
            &WORKER_LAYOUT
        } else if bin_name_contains_wifi_sta {
            m.soc.wifi_fixture_image(true);
            &STA_LAYOUT
        } else {
            m.soc.wifi_fixture_image(false);
            &SCAN_LAYOUT
        };
        m.soc.wifi_layout_set(
            Some(layout.reg_heaps),
            Some(layout.pxcur),
            Some(layout.sta_network_if),
            Some(layout.wifi_event_var),
            Some(layout.ip_event_var),
        );
    }

    // RNG reseed: RNG_SEED=<u32> varies the esp_random() stream across
    // runs (deterministic within a run; default seed keeps tests stable).
    if let Ok(seed) = env::var("RNG_SEED")
        && let Ok(seed) = seed.parse::<u32>()
    {
        m.soc.rng_reseed(seed);
        println!("[host] reseeded RNG ({seed:#x})");
    }

    // ADC injection for sketches doing analogRead: ADC_INJECT_MV=<mv>
    // applies the voltage to ADC1 channel 3 (GPIO4).
    if let Ok(mv) = env::var("ADC_INJECT_MV")
        && let Ok(mv) = mv.parse::<u32>()
    {
        m.soc.adc_inject_voltage(0, 3, mv);
        println!("[host] injected {mv} mV on ADC1_CH3 (GPIO4)");
    }

    // TSENS injection for temperatureRead: TEMP_RAW=<0..255> sets the
    // SENS_TSENS_OUT DAC code the firmware reads.
    if let Ok(raw) = env::var("TEMP_RAW")
        && let Ok(raw) = raw.parse::<u8>()
    {
        m.soc.tsens_inject(raw);
        println!("[host] injected TSENS raw={raw}");
    }
    // Celsius shorthand: TEMP_C=<c> inverts the driver-observed middle
    // line T = 0.4375*raw - 21 (blank-eFuse cal; ends are nonlinear).
    if let Ok(c) = env::var("TEMP_C")
        && let Ok(c) = c.parse::<f32>()
    {
        let raw = (((c + 21.0) * 16.0 / 7.0).round() as i32).clamp(0, 255) as u8;
        m.soc.tsens_inject(raw);
        println!("[host] injected TSENS {c}C as raw={raw}");
    }

    // Touch injection for sketches doing touchRead: TOUCH_INJECT=<pad>:<val>
    // sets the counter touch pad 1..=14 reports (falls when touched).
    if let Ok(spec) = env::var("TOUCH_INJECT")
        && let Some((pad, val)) = spec.split_once(':')
        && let (Ok(pad), Ok(val)) = (pad.parse::<usize>(), val.parse::<u32>())
    {
        m.soc.touch_inject(pad, val);
        println!("[host] injected touch pad {pad} = {val}");
    }

    // Fake temperature devices (tempdev-sketch support): I2C_TEMP_RX=<hex>
    // pre-injects the I2C thermometer read bytes (e.g. 1900 = TMP102
    // 25.00 C), SPI_TEMP_RX=<hex> the next SPI thermometer frame bytes
    // (e.g. 0320 = MAX6675 25.00 C). Same hooks the virtual-demo browser
    // harness uses (i2c_inject_rx / spi_inject_miso).
    fn parse_hex_bytes(s: &str) -> Vec<u8> {
        let h: Vec<char> = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();
        (0..h.len() / 2)
            .map(|i| {
                let pair: String = [h[2 * i], h[2 * i + 1]].iter().collect();
                u8::from_str_radix(&pair, 16).unwrap_or(0)
            })
            .collect()
    }
    if let Ok(hex) = env::var("I2C_TEMP_RX") {
        let b = parse_hex_bytes(&hex);
        m.soc.i2c_inject_rx(0, &b);
        println!("[host] injected I2C temp bytes {b:02x?}");
    }
    if let Ok(hex) = env::var("SPI_TEMP_RX") {
        let b = parse_hex_bytes(&hex);
        m.soc.spi_inject_miso(0, &b);
        println!("[host] injected SPI temp bytes {b:02x?}");
    }

    // Brown-out injection for BOD sketches: BOD_INJECT=1 holds the
    // low-voltage condition so an enabled detector trips (interrupt
    // after int_wait, chip reset after rst_wait with rst_ena).
    if env::var("BOD_INJECT").is_ok() {
        m.soc.bod_inject(true);
        println!("[host] injected brownout condition");
    }

    // PSRAM density override for 16 MB validation: PSRAM_MR2=<n> sets the
    // MR2 reset default (3 = 64 Mb / 8 MB, 5 = 128 Mb / 16 MB) before the
    // OPI sizing path runs at boot.
    if let Ok(mr2) = env::var("PSRAM_MR2")
        && let Ok(mr2) = mr2.parse::<u8>()
    {
        m.soc.psram_set_mr2(mr2);
        println!("[host] PSRAM MR2 override ={mr2}");
    }

    // UART1 RX injection (echo-sketch support): UART_INJECT=<text> is
    // pushed into UART1 RX as soon as the console shows the RXREADY marker.
    let uart1_inject: Option<Vec<u8>> = env::var("UART_INJECT").ok().map(|s| s.into_bytes());
    let mut uart1_injected = false;
    // UART0 RX injection (REPL experiment): UART0_INJECT=<text> is pushed
    // into UART0 RX once the console shows UART0_MARKER (MicroPython's
    // REPL may listen on UART0 rather than USB-CDC depending on the
    // board's console configuration).
    let uart0_inject: Option<Vec<u8>> = env::var("UART0_INJECT")
        .ok()
        .map(|s| s.replace("\\n", "\n").replace("\\r", "\r").into_bytes());
    let uart0_marker: Vec<u8> = env::var("UART0_MARKER")
        .ok()
        .map(|s| s.into_bytes())
        .unwrap_or_else(|| b">>> ".to_vec());
    let mut uart0_injected = false;
    // USB-Serial-JTAG RX injection (REPL experiment): USB_INJECT=<text> is
    // pushed into the USB CDC RX FIFO once the console shows USB_MARKER
    // (default: the MicroPython post-PSRAM boot line).
    let usb_inject: Option<Vec<u8>> = env::var("USB_INJECT")
        .ok()
        .map(|s| s.replace("\\n", "\n").replace("\\r", "\r").into_bytes());
    let usb_marker: Vec<u8> = env::var("USB_MARKER")
        .ok()
        .map(|s| s.into_bytes())
        .unwrap_or_else(|| b"continuing without it".to_vec());
    let mut usb_injected = false;

    // SPI-slave host exchange (slave-sketch support): SPI_SLAVE_XCHG=1 drives
    // both halves when the app prints its markers — a master-write-to-slave
    // ([0x11, 0x22, 0x33]) on "SPI SLAVE READY", then a 2-byte
    // master-read-from-slave (expecting the preloaded [0xA5, 0xC3]) on
    // "SPI SLAVE TX-REQ".
    let spi_slave_xchg = env::var("SPI_SLAVE_XCHG").is_ok();
    let mut spi_slave_wrote = false;
    let mut spi_slave_read = false;
    // Slave-DMA halves (separate markers, same env flag).
    let mut spi_slave_dma_wrote = false;
    let mut spi_slave_dma_read = false;

    // I2C-slave host exchange (slave-sketch support): I2C_SLAVE_XCHG=1 drives
    // both halves when the app prints its markers — a master-write-to-slave
    // (addr 0x42, [0x11, 0x22]) on "I2C SLAVE READY", then a 1-byte
    // master-read-from-slave (expecting the preloaded [0xA5]) on
    // "I2C SLAVE TX-REQ".
    let i2c_slave_xchg = env::var("I2C_SLAVE_XCHG").is_ok();
    let mut i2c_slave_wrote = false;
    let mut i2c_slave_read = false;

    // Fake quad-SPI device store (quaddev-sketch support): SPI_QUADDEV=1
    // provisions a 256-byte incrementing pattern (byte i = i) on GPSPI2
    // before boot, matching the sketch's expected address windows.
    if env::var("SPI_QUADDEV").is_ok() {
        let pat: Vec<u8> = (0..=255u16).map(|i| i as u8).collect();
        m.soc.spi_quad_fake_provision(0, &pat);
        println!("[host] provisioned fake quad-SPI device on GPSPI2");
    }

    // SPI-mode SD card (sdspi-sketch support): SPI_SDSPI=1 attaches the
    // in-model card on GPSPI2 sharing the SDMMC FAT16 image, so the
    // Arduino `SD` library mounts the same volume the SDMMC path uses.
    if env::var("SPI_SDSPI").is_ok() {
        m.soc.spi_sdspi_attach_sdmmc_image(0);
        println!("[host] attached SDSPI card on GPSPI2 (SDMMC image)");
    }

    // Live-IP backhaul tap (test-worker-net sketches): NET_PCAP=<path>
    // writes every captured board→host Ethernet frame to a tcpdump-readable
    // pcap (global header on first frame, then per-packet headers with a
    // monotonic microsecond clock), and NET_GW=<host:port> forwards each
    // frame as a length-prefixed binary message over TCP to the Go
    // SLIRP/NAT gateway bridge (`tools/gateway`, Ethernet-bridge mode —
    // same framing the WebSocket bridge client uses). Both are fed from
    // the `esp_netif_transmit` capture hook (see machine.rs) via
    // `Soc::net_take_tx`; with neither set the tap is compiled out of the
    // loop (zero-cost when idle). Monotonic clock: wall time is
    // meaningless in emulation (1 global tick per 2 insns for all
    // domains), so pcap timestamps use an incrementing counter.
    // Full-duplex: the same TCP leg carries gateway→board replies back
    // (length-prefixed, same framing): each reply is staged via
    // `Soc::net_inject_rx` and delivered at the next `esp_netif_receive`
    // entry (see machine.rs RX hook). The socket is NONBLOCKING and the
    // reply drain below runs ONLY on steps where a TX frame was just
    // forwarded (gateway replies are always causally after a board TX —
    // ARP/DHCP answers, gVisor returns) plus one poll every 1024 steps
    // as a backstop for unsolicited frames (IPv6 RAs, UDP-forward
    // injects). This keeps framing sync without per-step blocking:
    // nonblocking `read_exact` on a half-arrived header consumes the
    // partial bytes then fails, so the drain uses single `read` calls
    // with a small reassembly buffer (never `read_exact` on a
    // nonblocking socket — proven live: the ARP reply arrived split
    // across two TCP segments → "reply body short" → leg dropped).
    // A NET_RX_LOG=1 env additionally logs every injected reply
    // (ethertype + length) for the NET_RX battery assertion.
    let net_pcap_path: Option<String> = env::var("NET_PCAP").ok();
    let mut net_pcap_file: Option<std::fs::File> = None;
    let mut net_pcap_n: u64 = 0;
    let net_gw_addr: Option<String> = env::var("NET_GW").ok();
    // TX-sink presence for the machine's `esp_netif_transmit` tap (see
    // soc.rs `net_sink_present`): fake success only when pcap/gateway
    // drains exist; otherwise the driver runs (gateway-less fail verdicts).
    m.soc.net_sink_present = net_pcap_path.is_some() || net_gw_addr.is_some();
    let mut net_gw: Option<std::net::TcpStream> = None;
    let net_rx_log = env::var("NET_RX_LOG").is_ok();
    // BLE HCI bridge (ble-sketch support): BLE_GW=<host:port> dials the
    // Bumble bridge's emulator leg (`tools/ble_bridge.py --emu-port`,
    // default 127.0.0.1:9545; length-prefixed HCI both directions — the
    // same framing discipline as the NET_GW Ethernet leg). Host drains
    // each captured firmware→controller frame via `bt_hci_take_tx` and
    // forwards it; bridge replies are staged via `bt_hci_inject_rx` for
    // the firmware's `host_rcv_pkt` VHCI-recv path, then delivered
    // IN-FIRMWARE by running the registered callback (see the VHCI RX
    // hook below — same windowed-ABI discipline as `run_espnow_callback`).
    // With BLE_GW unset the leg compiles out (zero-cost when idle) and
    // the firmware observes a quiet controller (silicon with no peer).
    let ble_gw_addr: Option<String> = env::var("BLE_GW").ok();
    let mut ble_gw: Option<std::net::TcpStream> = None;
    let mut ble_rx_buf: Vec<u8> = Vec::new();
    // vhci_host_cb addresses (BLE image only — nm on the esp32s3_ble ELF;
    // sibling images never link libbt/NimBLE so the addrs never match):
    // `vhci_host_cb` rodata (notify_host_recv fn pointer slot), the
    // `host_rcv_pkt` entry itself, and `vhci_send_sem` (the counting
    // semaphore `controller_rcv_pkt_ready` gives). The RX hook stages
    // one queued bridge reply per `host_rcv_pkt` call.
    const BLE_HOST_CB: u32 = 0x3c06_b82c;
    let mut ble_cb_entry: Option<u32> = None;
    // TEMP (2026-10-03): deterministic BLE NULL-call repro (DELETE after
    // forensics). `BLE_RX_CANNED=1` stages one canned connection-complete
    // with no bridge; `BLE_RX_ACL_DROP=1` drops ATT ACLs instead of
    // delivering them.
    let ble_canned = std::env::var("BLE_RX_CANNED").is_ok();
    let mut ble_canned_done = false;
    // TEMP (2026-10-03, forensics — DELETE after): IRAM probe latch.
    let mut ble_iram_probed = false;
    // TEMP (2026-10-03): enable the event-dispatch tracer in canned mode
    // (DELETE after).
    if ble_canned {
        m.soc.set_ble_trace_evt(true);
    }
    // TEMP (2026-10-03): last capture seq served by the canned controller
    // (DELETE after).
    let mut ble_canned_last_op = 0u64;
    // TEMP (2026-10-03): opcode of the last served canned CC + version
    // follow-up flag (DELETE after).
    let mut ble_canned_last_cc_op = 0u32;
    let mut ble_canned_ver_done = false;
    // TEMP (2026-10-03): remote-features complete follow-up (DELETE after).
    // 0x2016 (LE_RD_REM_FEAT) is async like 0x041d: the CC acks the command
    // but the handler stalls awaiting the LE Meta Read-Remote-Features
    // Complete event (subevent 0x04) — without it no onConnect, proven live
    // (post-0x2016 silence with no waiter timeout).
    let mut ble_canned_feat_done = false;
    // TEMP (2026-10-03): data-length-change follow-up (DELETE after).
    // 0x2022 (LE_SET_DATA_LEN) is async: CC acks, then the handler awaits
    // the LE Meta Data-Length-Change event (subevent 0x07) — without it
    // the link disconnects right after the CC (proven live: conn then
    // disc with no further TX). Values mirror typical negotiated
    // maxima (251B/2120us both directions, handle 1 from the 22B).
    let mut ble_canned_dl_done = false;
    // TEMP (2026-10-04, forensics — DELETE after): canned ATT Read-By-Group
    // (service discovery) replay. Bytes captured from a live Bumble run
    // (pre-conn 16B, dropped there as premature). Normally staged one-shot
    // after the data-length event; SIZE TEST below reorders it after a small
    // Read. If firmware answers with an ATT Response ACL (H4=0x02 TX), the
    // GATT server path works; if it panics in att_tx (0x4200724d:91), the
    // PANIC-POOL dump says which pool.
    let mut ble_canned_att_done = false;
    // TEMP (2026-10-04, forensics — DELETE after): canned ATT script
    // (Find-Info → Read → Write → Read-back) for full GATT proof offline.
    // Phase 1 (this run): Find-Info 0x000e-0xFFFF after the discovery
    // response (first ACL TX) to learn characteristic/value handles
    // (decoded from the TXACL response hex). Phase 2 hardcodes them for
    // Read (Battery Level → `BLE read 1` + 100), Write (echo `hi!` →
    // `BLE write`), Read-back (echo value → ECHO/PASS).
    // `ble_canned_acl_tx_n` counts H4=0x02 TX drained (each ATT response).
    let mut ble_canned_findinfo_done = false;
    // TEMP (2026-10-04, forensics — DELETE after): Phase 2 (handles from
    // the Find-Info response: Battery Level value 0x0010 (decl 0x000f type
    // 0x2803 + value 0x0010 type 0x2A19), echo value 0x0013 (decl 0x0012
    // type 0x2803, value next sequential — verified by the read-back
    // value, loud fail otherwise). Read → `BLE read 1` + 100; Write `hi!`
    // → `BLE write 3` + notify; Read-back → echo value `hi!` (loopback).
    // Pace one-shot each on the response counter + empty FIFO (same
    // discipline: request N staged after response N-1 drained).
    let mut ble_canned_read_done = false;
    let mut ble_canned_write_done = false;
    let mut ble_canned_readback_done = false;
    // ATT OUTSTANDING PACING (2026-10-04, DELETE after): live Bumble ATTs
    // arrive unpaced (try 0,1,2 back-to-back — retries while the handler
    // still holds the previous request/response mbufs) and exhaust the
    // shared msys pools (prepend NULL → att_tx asserts 0x4200724d:91;
    // canned paces strictly one-at-a-time via acl_tx_n + FIFO-empty and
    // answers clean — proven live). Deliver an ATT only when none is
    // outstanding (increment on successful delivery, decrement on each
    // ATT *response* TX drained — not notify/indicate 0x1B/0x1D, which
    // are server-initiated, not answers). Level-triggered: gated ATTs stay
    // queued until the response drains. Shared by canned + bridge (canned
    // script is already paced and stays green; bridge becomes paced too).
    let mut ble_att_outstanding = 0u32;
    let mut ble_canned_acl_tx_n = 0u32;
    // TEMP (2026-10-03): capture seq at link-up (DELETE after — part of
    // the V2 gating: only commands sent after link-up get completed).
    let mut ble_link_up_seq = 0u64;
    // TEMP (2026-10-03): host-task-seen latch (DELETE after).
    let mut ble_host_seen = false;
    // HOST-ENABLED deferral latch (2026-10-04, DELETE after): transition
    // logging for the enabled_state==0 deferral (per-step spam floods).
    let mut ble_host_disabled = false;
    // PRE-DL HOLD (2026-10-04, DELETE after): pop-and-hold the premature
    // ATT instead of DROPPING it. DROP was correct for a retrying central
    // (30s GATT timeout → try 0,1,2… eventually hits post-dl), but fatal
    // for a patient central (300s timeout for slow emulator answers → the
    // single pre-dl discovery is dropped and never resent within the run).
    // HOLD pops the ATT into this slot (FIFO goes empty → the data-length
    // stager unblocks → dl_done sets), and the re-inject below restores it
    // AFTER the data-length event delivers (handshake fully established,
    // so the ATT TX no longer asserts). One slot: a second premature ATT
    // while held drops (log) — central sends one at a time (semaphore).
    let mut ble_held_att: Option<Vec<u8>> = None;
    // TEMP (2026-10-04, forensics — DELETE after): l2cap_tx return latch.
    // Logs each NEW (core, ret) observed at 0x4200723c (see machine.rs).
    let mut ble_l2cap_last: Option<(usize, u32)> = None;
    // TEMP (2026-10-04, forensics — DELETE after): management-TX hook latch.
    let mut pptx_last = 0u32;
    // TEMP (2026-10-04, RF forensics — DELETE after): park-tracer one-shot.
    let mut park_traced = false;
    // TEMP (2026-10-03): post-delivery pc-window countdown (DELETE after).
    let mut ble_watch_n = 0u32;
    // TEMP (2026-10-03): pc rings for the canned NULL-call forensics
    // (DELETE after). Last 64 block-start pcs per core — dumped when the
    // run breaks on an exception, so the wild call site is identified
    // even though the fault vectors away from it.
    let mut ring0 = [0u32; 64];
    let mut ring1 = [0u32; 64];
    let mut ring_i: usize = 0;
    if let Some(ref addr) = ble_gw_addr {
        match std::net::TcpStream::connect(addr.as_str()) {
            Ok(s) => {
                if let Err(e) = s.set_nonblocking(true) {
                    println!("[host] BLE bridge nonblocking failed: {e} (HCI leg disabled)");
                    ble_gw = None;
                } else {
                    println!("[host] BLE bridge connected to {addr}");
                    ble_gw = Some(s);
                }
            }
            Err(e) => {
                println!("[host] BLE bridge connect to {addr} failed: {e} (quiet-controller mode)")
            }
        }
    }
    // Reassembly buffer for the nonblocking reply drain (length header
    // + body may arrive split across steps; never reset except on
    // fatal framing errors).
    let mut net_rx_buf: Vec<u8> = Vec::new();
    // Set when the current step forwarded a TX frame (reply drain arm).
    let mut net_tx_this_step: bool;
    // Step counter for the periodic backstop poll (unsolicited frames).
    let mut net_step_n: u64 = 0;
    if let Some(ref addr) = net_gw_addr {
        match std::net::TcpStream::connect(addr.as_str()) {
            Ok(s) => {
                if let Err(e) = s.set_nonblocking(true) {
                    println!("[host] net bridge nonblocking failed: {e} (reply drain disabled)");
                    net_gw = None;
                } else {
                    println!("[host] net bridge connected to {addr}");
                    net_gw = Some(s);
                }
            }
            Err(e) => println!(
                "[host] net bridge connect to {addr} failed: {e} (frames still go to pcap)"
            ),
        }
    }

    // USB-OTG device auto-enumeration (usb-device-sketch support):
    // USB_HOST_ENUM=1 makes the in-model host the enumeration counterparty
    // for the firmware's own TinyUSB device stack — no external host needed.
    // Staged strictly in order per control transfer (one host action per
    // marker window): bus-reset on "USB DEVICE STACK UP", then per
    // "USB DEV REQ<n>" line (printed by the sketch before staging each
    // transfer) the matching SETUP (+ optional status OUT), with the
    // previous transfer's IN capture drained and asserted by the sketch
    // itself through `usb_host_take_in` byte checks.
    let usb_host_enum = env::var("USB_HOST_ENUM").is_ok();
    let mut usb_enum_step: usize = 0;

    // WiFi fixture support (scan + STA-connect sketches): WIFI_SCAN_FIXTURE=1
    // / WIFI_STA_CONN=1 arm host-driven completions at the firmware
    // boundary. When the firmware enters `esp_wifi_scan_start` /
    // `esp_wifi_connect` (closed lib, addresses below) the dwell starts;
    // when it elapses the host posts the REAL esp_event (SCAN_DONE /
    // STA_CONNECTED + GOT_IP) through `sys_evt`, so the full Arduino
    // `_scanDone` / STA-status chain runs unmodified. Empty air (no
    // WIFI_SCAN_APS) reports `found 0`, which is what silicon reports with
    // no APs in range. With fixtures, the host writes the `wifi_ap_record_t`
    // records DIRECTLY into the calloc'd `_scanResult` buffer at the
    // records-return check (see below) — the closed BSS queue never carries
    // nodes without RF stimulus, so the copy loop always derives 0 there.
    //
    // IMAGE LAYOUTS: every address below is a LINKED address, stable for
    // the pinned esp32 core but DIFFERENT per sketch (the STA sketch links
    // the pool/RAM elsewhere). The harness picks the layout by binary name
    // (`wifi_sta` vs default scan) and programs it into the SoC via
    // `wifi_layout_set` — the SoC itself holds no image addresses (fixture
    // calls fail softly while undiscovered, never a wrong-address write).
    // WLAN event IDs from `esp_wifi_types_generic.h` / `esp_netif_types.h`:
    // WIFI_EVENT: READY=0, SCAN_DONE=1, STA_START=2, STA_CONNECTED=4,
    // STA_DISCONNECTED=5; IP_EVENT: STA_GOT_IP=0.
    #[allow(dead_code)]
    struct WifiLayout {
        scan_start: u32,
        connect: u32,
        wifi_event_var: u32,
        ip_event_var: u32,
        count_cell: u32,
        scan_count: u32,
        scan_result: u32,
        records_check: u32,
        ready_lists: u32,
        top_prio: u32,
        reg_heaps: u32,
        pxcur: u32,
        sta_network_if: u32,
    }
    // wifi-scan image layout (nm on the wifi-scan ELF).
    const SCAN_LAYOUT: WifiLayout = WifiLayout {
        scan_start: 0x4206_3b90,
        connect: 0x4203_c84c,
        wifi_event_var: 0x3c0b_4264,
        ip_event_var: 0x3c0b_3b60,
        count_cell: 0x3fc9_f93e,
        scan_count: 0x3fc9_aef4,
        scan_result: 0x3fc9_aef0,
        records_check: 0x4200_3ee0,
        ready_lists: 0x3fc9_b8f4,
        top_prio: 0x3fc9_b864,
        reg_heaps: 0x3fc9_b7ac,
        pxcur: 0x3fc9_b7e0,
        sta_network_if: 0x3fc9_ae7c,
    };
    // wifi-sta image layout (nm on the wifi-sta ELF; pxcur =
    // pxCurrentTCBs, sta_network_if = `_ZL15_sta_network_if` bss static).
    const STA_LAYOUT: WifiLayout = WifiLayout {
        scan_start: 0x4206_3b78,
        connect: 0x4203_c7c4,
        wifi_event_var: 0x3c0b_42bc,
        ip_event_var: 0x3c0b_3bb8,
        count_cell: 0x3fc9_f926,
        scan_count: 0x3fc9_aee4,
        scan_result: 0x3fc9_aee0,
        records_check: 0x4200_3f9c,
        ready_lists: 0x3fc9_b8dc,
        top_prio: 0x3fc9_b84c,
        reg_heaps: 0x3fc9_b794,
        pxcur: 0x3fc9_bad0,
        sta_network_if: 0x3fc9_ae6c,
    };
    // test-worker-net image layout (nm on the test-worker-net ELF —
    // re-verified after the keep-alive relink: WIFI_EVENT 0x3c0b4304,
    // IP_EVENT 0x3c0b3c00, scan_start 0x42063c3c, connect 0x4203c8f8,
    // transmit 0x4202e60c / receive 0x4202e670 (machine.rs wifi_image
    // gate); the remaining cells identical to wifi-sta; the worker sketch
    // never scans, so records_check is unused (set to the STA value as a
    // harmless placeholder — the scan leg never arms on this image).
    // NOTE: the sketch source pins the layout — any .ino edit relinks
    // the closed libs elsewhere, so EVERY pc/var above must be re-nm'd
    // after every sketch change (proven live twice: removing one
    // disconnect() call moved transmit/connect/delete by 0x4c; the
    // keep-alive loop moved them again + both event vars by 0x10).
    const WORKER_LAYOUT: WifiLayout = WifiLayout {
        scan_start: 0x4206_3c3c,
        connect: 0x4203_c8f8,
        wifi_event_var: 0x3c0b_4304,
        ip_event_var: 0x3c0b_3c00,
        count_cell: 0x3fc9_f926,
        scan_count: 0x3fc9_aee4,
        scan_result: 0x3fc9_aee0,
        records_check: 0x4200_3f9c,
        ready_lists: 0x3fc9_b8dc,
        top_prio: 0x3fc9_b84c,
        reg_heaps: 0x3fc9_b794,
        pxcur: 0x3fc9_bad0,
        sta_network_if: 0x3fc9_ae6c,
    };
    // test-worker-l3 image layout (nm on the test-worker-l3 ELF —
    // re-nm'd after the L3-legs .ino edit (udp/coap/ipv6 builders):
    // scan_start 0x42065b90 (= esp_wifi_scan_start entry),
    // connect 0x4203e95c (= esp_wifi_connect entry), event vars
    // WIFI_EVENT 0x3c0b45a8 / IP_EVENT 0x3c0b3ea4; TX/RX tap + hook pcs
    // re-nm'd in machine.rs/soc.rs below. BSS cells shifted +0x18
    // uniformly vs the previous link (ready_lists/top_prio/pxcur/netif/
    // scan_count/scan_result verified by symbol name).
    // records_check is the `call8 esp_wifi_scan_get_ap_records` INSIDE
    // `_scanDoneEv` (0x42006129 here — objdump-verified on the new ELF).
    const WORKER_L3_LAYOUT: WifiLayout = WifiLayout {
        scan_start: 0x4206_5f18,
        connect: 0x4203_e95c,
        wifi_event_var: 0x3c0b_45ec,
        ip_event_var: 0x3c0b_3ee8,
        count_cell: 0x3fc9_f936,
        scan_count: 0x3fc9_af08,
        scan_result: 0x3fc9_af04,
        records_check: 0x4200_6129,
        ready_lists: 0x3fc9_b904,
        top_prio: 0x3fc9_b874,
        reg_heaps: 0x3fc9_b7bc,
        pxcur: 0x3fc9_baf8,
        sta_network_if: 0x3fc9_ae90,
    };
    // coex image layout (nm on the esp32s3_coex ELF; STA-side assoc + BLE
    // init concurrently; never scans so records_check is 0/unused; RF skips
    // unneeded for this short run — see soc.rs Coex arm).
    const COEX_LAYOUT: WifiLayout = WifiLayout {
        scan_start: 0x4208_6f54,
        connect: 0x4205_0eb8,
        wifi_event_var: 0x3c0d_6fcc,
        ip_event_var: 0x3c0d_68c8,
        count_cell: 0x3fc9_f97c,
        scan_count: 0x3fc9_f97c,
        scan_result: 0x3fc9_f978,
        records_check: 0x0000_0000,
        ready_lists: 0x3fca_179c,
        top_prio: 0x3fca_170c,
        reg_heaps: 0x3fca_1654,
        pxcur: 0x3fca_1990,
        sta_network_if: 0x3fc9_f904,
    };
    // wifi-ap image layout (nm on the wifi-ap ELF; sta_network_if =
    // `_ZL14_ap_network_if` bss static; esp_wifi_start = 0x42063870
    // (re-verified 2026-09-28; the old 0x42063840/0xc4 never fires,
    // so the AP leg never staged and `stations` read an unstaged count).
    const AP_LAYOUT: WifiLayout = WifiLayout {
        scan_start: 0x4206_3b78,
        connect: 0x4203_c7c4,
        wifi_event_var: 0x3c0b_42b0,
        ip_event_var: 0x3c0b_3bac,
        count_cell: 0x3fc9_f926,
        scan_count: 0x3fc9_aee4,
        scan_result: 0x3fc9_aee0,
        records_check: 0x4200_3f9c,
        ready_lists: 0x3fc9_b8dc,
        top_prio: 0x3fc9_b84c,
        reg_heaps: 0x3fc9_b794,
        pxcur: 0x3fc9_bad0,
        sta_network_if: 0x3fc9_ae64,
    };
    let wifi_scan_fixture = env::var("WIFI_SCAN_FIXTURE").is_ok();
    let wifi_sta_conn = env::var("WIFI_STA_CONN").is_ok();
    let wifi_ap_fixture = env::var("WIFI_AP_FIXTURE").is_ok();
    let mut wifi_ap_armed = false;
    let mut wifi_ap_done = false;
    let wifi_espnow_loopback = env::var("WIFI_ESPNOW_LOOPBACK").is_ok();
    let mut wifi_espnow_tx_done = false;
    let mut wifi_espnow_rx_done = false;
    // Full-802.11-LL-MAC slice-1 leg (`esp32s3_llmac` sketch): TX tap +
    // virtual-AP beacon injection (see the LLMAC block in the step loop).
    let wifi_llmac = env::var("WIFI_LLMAC").is_ok();

    let wifi_scan_aps: Vec<esp32s3_soc::wifi::ScanFixtureAp> = env::var("WIFI_SCAN_APS")
        .ok()
        .map(|s| esp32s3_soc::wifi::parse_scan_fixtures(&s))
        .unwrap_or_default();
    // Closed-RF dispatch-table hook gate (see
    // `Soc::maybe_complete_phyfuns_slot`): arm it on every Wi-Fi run so
    // the recalibration `callx8` lands on the benign no-op instead of heap
    // garbage. Fixture-less = hello = must not fire (proven by the hello
    // ILLEGAL at EPC1=0x4037a0a8). Deliberately NOT arming the SoC fixture
    // engine here: the run_flash host blocks below are the sole leg driver
    // on this path (single-driver separation, the proven architecture —
    // the engine serves the browser/bridge path, which has no host
    // blocks).
    if wifi_ap_fixture || wifi_espnow_loopback || wifi_scan_fixture || wifi_sta_conn || wifi_llmac {
        m.soc.wifi_phyfuns_gate_enable();
    }
    if wifi_llmac {
        m.soc.llmac_arm();
        m.soc.llmac_arm_beacons(3);
    }
    let mut wifi_scan_armed = false;
    let mut wifi_scan_done = false;
    let mut wifi_scan_records_done = false;
    let mut wifi_sta_armed = false;
    let mut wifi_sta_done = false;
    let mut wifi_sta_conn_stage = 0u8;
    let mut wifi_sta_ip_done = false;
    let mut wifi_sta_disc_armed = false;
    let mut wifi_sta_disc_done = false;

    // Step budget in INSTRUCTIONS (`step_fast` executes whole blocks and
    // reports how many instructions ran): one old loop iteration stepped a
    // single instruction per core (2/cross-core pair), so the old 48M-step
    // default ~= 96M instructions.  The loop also exits early when the
    // firmware has been idle (no UART output AND no PC change) for 2M
    // consecutive steps — indicates setup() is done and the firmware is in
    // its main loop or halted.
    let max_insns: usize = env::var("STEPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(96_000_000);
    let t0 = Instant::now();
    let mut uart_buf: Vec<u8> = Vec::new();
    let mut last_pc = 0u32;
    let mut last_uart_len = 0usize;
    let mut stuck = 0u32;
    let mut idle_steps: usize = 0;
    let mut executed: u64 = 0;
    let mut i: usize = 0; // macro-step counter (diagnostics only)

    loop {
        if executed >= max_insns as u64 {
            break;
        }
        // WiFi STA insider hooks live in `run_fast_core` (machine.rs —
        // per-op sampling inside the block loop; pre/post-step sampling
        // here would miss mid-block entry pcs).
        let (r, r1, n) = m.step_fast();
        executed += n as u64;
        i += 1;
        // TEMP (2026-10-04, RF forensics — DELETE after): park-loop tracer
        // for the WorkerL3 management-TX stall (core0 parks at 0x42074b4f
        // `ppTxFragmentProc+0x13f` with entry/caller/queue-processor hooks
        // all missing — reached via wdev-table tail-jump). Fires once when
        // parked here: single-steps 12× logging pc + raw word + regs so the
        // polling load (same address every iteration) names the flag the
        // host must satisfy (same discipline as the BLE sem pre-seed: direct
        // write, no window surgery). Gated to the exact stall pc on the
        // worker-L3 path (other images never park here; normal DONE-idles
        // park elsewhere).
        // (One-shot via `park_traced` latch below.)
        if !park_traced
            && path.contains("test_worker_l3")
            && (m.cpu[0].pc == 0x4207_4b4f || m.cpu[1].pc == 0x4207_4b4f)
        {
            park_traced = true;
            let c = if m.cpu[0].pc == 0x4207_4b4f { 0 } else { 1 };
            // TEMP (2026-10-04, RF forensics — DELETE after): ground-truth
            // decode of the park word via the emulator's own decoder (like
            // the UNIMPLEMENTED trap detail — no objdump/walkvec byte-order
            // traps). Settles ee_unimplemented vs S32I_N definitively.
            println!("[host] PARKDECODE {}", unimp_detail(&mut m, c));
            for _ in 0..12 {
                let p = m.cpu[c].pc;
                let w = m.soc.read32(p);
                let mut regs = [0u32; 16];
                for (k, r) in regs.iter_mut().enumerate() {
                    *r = m.cpu[c].reg(k as u32);
                }
                println!("[host] PARKTRACE{c} pc={p:#010x} word={w:#010x} regs={regs:08x?}");
                let _ = m.cpu[c].step_one(&mut m.soc);
            }
        }
        // TEMP (2026-10-04, forensics — DELETE after): log each new
        // l2cap_tx return code observed at 0x4200723c (machine.rs per-op
        // probe). Proves whether live-first-ATT fails via prepend (6) or
        // another path, without exact-pc post-step polling.
        if m.ble_l2cap_ret != ble_l2cap_last {
            ble_l2cap_last = m.ble_l2cap_ret;
            if let Some((core, ret)) = m.ble_l2cap_ret {
                println!("[host] BLE L2CAP_RET core{core} ret={ret}");
            }
        }
        // TEMP (2026-10-04, forensics — DELETE after): log management-TX
        // hook fires (counter increments). Proves the ppTxFragmentProc
        // fake-return arm fires vs misses (0 = still parks inside).
        if m.pptx_hook_fires != pptx_last {
            pptx_last = m.pptx_hook_fires;
            println!("[host] PPTX hook fires={pptx_last}");
        }
        let pc = m.cpu[0].pc;
        if ble_canned {
            ring0[ring_i % 64] = pc;
            ring1[ring_i % 64] = m.cpu[1].pc;
            ring_i += 1;
        }
        // --- Exception handling ---
        if let StepResult::Exception { cause } = r {
            if (32..=37).contains(&cause) {
                continue; // Window overflow/underflow — normal.
            }
            if cause == 5 {
                // ALLOCA (movsp stack-guard spill request) — normal,
                // firmware-handled VM event like window spills: the
                // handler spills windows and resumes past the movsp.
                // Aborting here killed healthy MicroPython REPL runs.
                continue;
            }
            if cause == 1 && env::var("SYSCALL_CONTINUE").is_ok() {
                // Let the firmware's own exception vector handle syscalls
                // (raise_cause already vectored; e.g. MicroPython issues
                // syscalls its kernel handles).
                continue;
            }
            if cause == 0 {
                // Illegal instruction — likely the ESP-IDF panic `ill`.
                // PANIC_CONTINUE lets the firmware's own handler run it
                // (panic_abort ends in a deliberate `ill`; the coredump
                // sketch needs the reboot it triggers).
                if env::var("PANIC_CONTINUE").is_ok() {
                    continue;
                }
                let epc = m.cpu[0].sreg(SR_EPC1);
                println!(
                    "\n>> ILLEGAL at step {i}, pc {:#010x}, EPC1 {:#010x}, wb {}, b0={:#04x}",
                    pc,
                    epc,
                    m.cpu[0].windowbase(),
                    m.soc.read8(epc)
                );
                // TEMP (2026-10-04, RF forensics — DELETE after): dump the
                // g_phyFuns dispatch table around the heap-overlap window
                // (tbl+0x200..0x27f) on worker-L3 ILLEGALs. Long gateway
                // runs overlap heap network buffers into the table (proven:
                // DNS "example" bytes at tbl+0x22f); each clobbered slot
                // needs a done-bit-abstracted skip in machine.rs (4 known:
                // 0x24c/0x228/0x210/0x224). A 19-leg crash here means a 5th
                // slot joined — the ASCII side names the buffer (DNS vs
                // CoAP-server READY/GET/2.05 vs MQTT), the offset names the
                // slot. WorkerL3-gated (other images have their own cells).
                if path.contains("test_worker_l3") {
                    let cell = m.soc.wifi_phyfuns_cell();
                    let tbl = m.soc.read32(cell);
                    let mut words = [0u32; 32];
                    for (k, v) in words.iter_mut().enumerate() {
                        *v = m.soc.read32(tbl.wrapping_add(0x200 + (k as u32) * 4));
                    }
                    let mut asc = [0u8; 128];
                    for (k, b) in asc.iter_mut().enumerate() {
                        *b = m.soc.read8(tbl.wrapping_add(0x200 + k as u32)) as u8;
                    }
                    println!("[host] RF-TBL tbl={tbl:#010x} {words:08x?}");
                    println!("[host] RF-ASC {:?}", String::from_utf8_lossy(&asc));
                }
                // TEMP (2026-10-03, forensics — DELETE after): dump panic
                // message when the `ill` is panic_abort's deliberate trap
                // (EPC1 in its range). a2 often holds the message pointer.
                if (0x4037_fdb4..0x4037_fde0).contains(&epc) {
                    let a2 = m.cpu[0].reg(2);
                    let mut msg = Vec::new();
                    for k in 0..128u32 {
                        let b = m.soc.read8(a2.wrapping_add(k)) as u8;
                        if b == 0 {
                            break;
                        }
                        msg.push(b);
                    }
                    println!(
                        "[host] PANIC-MSG a2={a2:#010x} {:?}",
                        String::from_utf8_lossy(&msg)
                    );
                    let sp = m.cpu[0].reg(1);
                    println!(
                        "[host] PANIC-STACK sp={sp:#010x} {:#010x} {:#010x} {:#010x} {:#010x}",
                        m.soc.read32(sp),
                        m.soc.read32(sp.wrapping_add(4)),
                        m.soc.read32(sp.wrapping_add(8)),
                        m.soc.read32(sp.wrapping_add(12)),
                    );
                    // TEMP (2026-10-04, forensics — DELETE after): VHCI
                    // semaphore + HCI credits post-mortem. `ble_hs_hci_acl_
                    // tx_now` can only fail the att_tx assert via the VHCI
                    // send itself (all pools healthy at every panic, queue
                    // paths return 1 not assert): vhci_send_sem 0x3fc9d6d0
                    // (1 initial, taken per send, given per TX-done) and
                    // avail_pkts 0x3fc9e110 / buf_sz 0x3fc9e2b0 (ROM-
                    // reported controller buffers). sem 0 => exhaustion
                    // (fix = host give per captured ACL TX); avail 0 =>
                    // queue path (rc 1, rules out credits).
                    println!(
                        "[host] PANIC-VHCI sem={} avail={} bufsz={}",
                        m.soc.read32(0x3fc9_d6d0),
                        m.soc.read16(0x3fc9_e110),
                        m.soc.read16(0x3fc9_e2b0),
                    );
                    // TEMP (2026-10-04, forensics — DELETE after): pointed-to
                    // queue dump (8 words at *handle) to locate
                    // uxMessagesWaiting (binary count 0/1). The handle word
                    // itself is a pointer (not the count — proven: reads
                    // 0x3fcb13c4). Compare with the pre-delivery dump below
                    // (healthy=1 vs timed-out=0) to find the count offset;
                    // the pre-seed writes queue+0x38 (I2C Queue_t precedent).
                    {
                        let h = m.soc.read32(0x3fc9_d6d0);
                        let mut w = [0u32; 8];
                        for (k, v) in w.iter_mut().enumerate() {
                            *v = m.soc.read32(h.wrapping_add((k as u32) * 4));
                        }
                        println!("[host] PANIC-SEM [{h:#010x}] {w:08x?}");
                    }
                    // TEMP (2026-10-04, forensics — DELETE after): mbuf pool
                    // free counts. Pools at these addrs are `os_mbuf_pool`
                    // (omp_databuf_len u16 @+0, omp_pool *os_mempool @+4);
                    // the counts live in the pointed-to `os_mempool`
                    // (num_blocks u16 @+4, num_free u16 @+6, min_free u16
                    // @+8). (An earlier revision read +4/+6/+8 directly off
                    // the mbuf_pool and printed garbage — proven live.)
                    // Walk the authoritative msys list (g_msys_pool_list
                    // 0x3fc98e84 → os_mbuf_pool → omp_next @+8) so NO pool
                    // is missed (a fifth dry pool explains healthy msys1/2
                    // + failing prepend). Plus the non-msys frag/acl pools.
                    // Pools (nm on ble ELF): msys1 0x3fc9eab8, msys2
                    // 0x3fc9ea8c, hci_frag 0x3fc9e130, acl 0x3fc9e9c8
                    // (mpool_acl — the ATT/ACL data pool prepend likely
                    // uses; msys can show all-free while acl is dry).
                    let mut mp = m.soc.read32(0x3fc9_8e84);
                    let mut guard = 0u32;
                    while mp != 0 && guard < 8 {
                        let dl = m.soc.read16(mp);
                        let ipool = m.soc.read32(mp.wrapping_add(4));
                        let (blocks, free, min_free) = if ipool != 0 {
                            (
                                m.soc.read16(ipool.wrapping_add(4)),
                                m.soc.read16(ipool.wrapping_add(6)),
                                m.soc.read16(ipool.wrapping_add(8)),
                            )
                        } else {
                            (0xFFFF, 0xFFFF, 0xFFFF)
                        };
                        println!(
                            "[host] PANIC-POOL msys mp={mp:#010x} datalen={dl} blocks={blocks} free={free} min_free={min_free}",
                        );
                        mp = m.soc.read32(mp.wrapping_add(8));
                        guard += 1;
                    }
                    for (name, base) in [
                        ("msys1", 0x3fc9_eab8u32),
                        ("msys2", 0x3fc9_ea8cu32),
                        ("frag", 0x3fc9_e130u32),
                        ("acl", 0x3fc9_e9c8u32),
                    ] {
                        let mp = m.soc.read32(base.wrapping_add(4));
                        let (blocks, free, min_free) = if mp != 0 {
                            (
                                m.soc.read16(mp.wrapping_add(4)),
                                m.soc.read16(mp.wrapping_add(6)),
                                m.soc.read16(mp.wrapping_add(8)),
                            )
                        } else {
                            (0xFFFF, 0xFFFF, 0xFFFF)
                        };
                        println!(
                            "[host] PANIC-POOL {name} mp={mp:#010x} blocks={blocks} free={free} min_free={min_free}",
                        );
                    }
                }
                if ble_canned {
                    println!("[host] BLE ring0 (oldest first):");
                    for k in 0..64 {
                        print!(" {:#010x}", ring0[(ring_i + k) % 64]);
                        if k % 4 == 3 {
                            println!();
                        }
                    }
                    println!("[host] BLE ring1 (oldest first):");
                    for k in 0..64 {
                        print!(" {:#010x}", ring1[(ring_i + k) % 64]);
                        if k % 4 == 3 {
                            println!();
                        }
                    }
                }
                break;
            }
            let epc = m.cpu[0].sreg(SR_EPC1);
            println!(
                "\n== step {i}: exception(cause={cause}) at pc {:#010x}; EPC1 {:#010x}, a0={:#x} a1={:#x} a2={:#x}",
                pc,
                epc,
                m.cpu[0].reg(0),
                m.cpu[0].reg(1),
                m.cpu[0].reg(2),
            );
            // TEMP (2026-10-03, forensics — DELETE after): dump panic message
            // when dying inside panic_abort (EPC1 in its range). a2 at wb
            // often holds the abort-message pointer; dump 128B as C string.
            if (0x4037_fdb4..0x4037_fde0).contains(&epc) {
                let a2 = m.cpu[0].reg(2);
                let mut msg = Vec::new();
                for k in 0..128u32 {
                    let b = m.soc.read8(a2.wrapping_add(k)) as u8;
                    if b == 0 {
                        break;
                    }
                    msg.push(b);
                }
                println!(
                    "[host] PANIC-MSG a2={a2:#010x} {:?}",
                    String::from_utf8_lossy(&msg)
                );
                // Also dump 8 words at sp for backtrace.
                let sp = m.cpu[0].reg(1);
                println!(
                    "[host] PANIC-STACK sp={sp:#010x} {:#010x} {:#010x} {:#010x} {:#010x}",
                    m.soc.read32(sp),
                    m.soc.read32(sp.wrapping_add(4)),
                    m.soc.read32(sp.wrapping_add(8)),
                    m.soc.read32(sp.wrapping_add(12)),
                );
            }
            break;
        }
        if let StepResult::Exception { cause } = r1 {
            if cause != 0 && (32..=37).contains(&cause) {
                continue;
            }
            if cause == 5 {
                continue; // ALLOCA spill request (see core0 arm).
            }
            if cause == 1 && env::var("SYSCALL_CONTINUE").is_ok() {
                continue; // Same as above (core1 syscalls).
            }
            if cause == 0 && env::var("PANIC_CONTINUE").is_ok() {
                continue; // Deliberate panic `ill` (see core0 arm).
            }
            println!(
                "\n== step {i}: core1 exception(cause={cause}) at pc {:#010x}; EPC1 {:#010x}",
                m.cpu[1].pc,
                m.cpu[1].sreg(SR_EPC1),
            );
            break;
        }
        // Unimplemented instructions (ee.* DSP/TIE unmapped patterns: no
        // execution model) trap LOUDLY instead of hanging: the pc is
        // frozen on the faulting op, so ignoring the result would spin
        // forever. A dynamic audit (2026-09-03: 24 sketches x 96M insns,
        // both cores) shows zero executions, so this never fires today.
        // EXCEPTION: the closed Wi-Fi/BT blob idles core 0 on a 2-byte
        // word the decoder's TIE catch-all labels `ee_unimplemented`
        // (raw 0x00000100 at e.g. 0x40377367 — objdump shows `retw.n`
        // fall-through into the next routine's `extui/bnone/salt/call0`
        // bytes; single-stepping OVER it advances cleanly, proven
        // live). The ESP-NOW poll loop parks there while core 1 prints
        // the verdict lines one byte per ~1k insns, so breaking here
        // would swallow the DONE bytes still in flight. Drain first and
        // keep stepping until DONE arrives or the budget runs out (a
        // real trap prints no further output, so the loop still ends;
        // the DONE break below exits normally).
        if matches!(r, StepResult::Unimplemented(_)) {
            let pc = m.cpu[0].pc;
            // Drain before deciding: DONE bytes may already be queued.
            let tx = m.take_uart_tx(0);
            let tx1 = m.take_uart_tx(1);
            if !tx1.is_empty() {
                uart_buf.extend_from_slice(&tx1);
            }
            if !tx.is_empty() {
                uart_buf.extend_from_slice(&tx);
            }
            if uart_buf
                .windows(b"WIFI ESPNOW DONE".len())
                .any(|w| w == b"WIFI ESPNOW DONE")
            {
                break;
            }
            // Idle-NOP park (known Wi-Fi/BT blob address): single-step
            // OVER the word (executes cleanly, proven live) and keep
            // running instead of breaking. Anything else is a real
            // trap — report LOUD and break.
            //
            // WiFi-TX DSP skip (WorkerL3 management-TX stall): 0x42074b4f
            // (`ppTxFragmentProc+0x13f`, raw 0x49040ca4) decodes via the
            // ground-truth runtime decoder as `ee_unimplemented` (PARKDECODE
            // proven live) — an unmapped TIE/DSP pattern on the WiFi TX
            // fragment path that `step_one` refuses to advance past (pc
            // frozen 12/12 single-steps, PARKTRACE proven). Manually skip
            // past it (pc += len, no exec effect) and keep running: the DSP
            // op has zero validatable effect offline (management fragments
            // never transmit — no RF, no pcap for management, association
            // is fixture-driven; same discipline as the RF call-site skips
            // which abstract unrealized RF effects via done-bits). Length
            // from `insn_len` (ground truth, like `unimp_detail` — never
            // objdump/walkvec byte-order guessing).
            if pc == 0x4037_7367 || (path.contains("test_worker_l3") && pc == 0x4207_4b4f) {
                if pc == 0x4207_4b4f {
                    use xtensa_core::generated::insn_len;
                    let b0 = (m.soc.read32(pc) & 0xFF) as u8;
                    let len = insn_len(b0) as u32;
                    m.cpu[0].pc = pc.wrapping_add(len);
                    println!("[host] DSP-SKIP pc={pc:#010x} len={len}");
                } else {
                    m.cpu[0].step_one(&mut m.soc);
                }
                let tx = m.take_uart_tx(0);
                let tx1 = m.take_uart_tx(1);
                if !tx1.is_empty() {
                    uart_buf.extend_from_slice(&tx1);
                }
                if !tx.is_empty() {
                    uart_buf.extend_from_slice(&tx);
                }
            } else {
                let detail = unimp_detail(&mut m, 0);
                println!("\n>> UNIMPLEMENTED core0 at pc {pc:#010x}: {detail}",);
                break;
            }
        }
        if matches!(r1, StepResult::Unimplemented(_)) {
            let pc = m.cpu[1].pc;
            let detail = unimp_detail(&mut m, 1);
            println!("\n>> UNIMPLEMENTED core1 at pc {pc:#010x}: {detail}",);
            break;
        }

        // --- UART output (every step): the drain must stay per-step.
        // `take_uart_tx(0)` merges the UART0 and USB-Serial-JTAG FIFOs and
        // its ROM-doubling dedup pairs each UART byte with its USB twin in
        // the same drain call. Both cores print concurrently during boot,
        // so any batching wider than a step interleaves the two streams
        // across drain windows (`out != usb`) and the single-byte
        // cross-call state lets doubled bytes through — batching this was
        // tried and corrupted the early-boot log line. The per-step drain
        // itself is cheap (empty Vec handoffs + a DRAM fast-path read).
        let tx = m.take_uart_tx(0);
        let tx1 = m.take_uart_tx(1);
        if !tx1.is_empty() {
            uart_buf.extend_from_slice(&tx1);
        }
        if !tx.is_empty() {
            uart_buf.extend_from_slice(&tx);
        }

        // Live-IP backhaul tap drain: every captured board→host Ethernet
        // frame goes to pcap and/or the gateway bridge. Drained per step
        // (frames are rare — the check is a single empty-Vec handoff when
        // idle, same discipline as the UART fast path).
        net_tx_this_step = false;
        if net_pcap_path.is_some() || net_gw.is_some() {
            let frame = m.soc.net_take_tx();
            if !frame.is_empty() {
                // pcap record: ts_sec/ts_usec (monotonic counter as usec),
                // incl_len/orig_len, then the raw Ethernet frame.
                if let Some(ref path) = net_pcap_path {
                    use std::io::Write as _Write;
                    let need_hdr = net_pcap_file.is_none();
                    let f = net_pcap_file.get_or_insert_with(|| {
                        let mut f =
                            std::fs::File::create(path).expect("NET_PCAP path must be writable");
                        // Global header: magic d4 c3 b2 a1, ver 2.4,
                        // zone 0, sigfigs 0, snaplen 1600, LINKTYPE_ETHERNET.
                        let hdr: [u8; 24] = [
                            0xd4, 0xc3, 0xb2, 0xa1, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x40, 0x06, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                        ];
                        f.write_all(&hdr).expect("pcap header write");
                        f
                    });
                    let _ = need_hdr;
                    net_pcap_n += 1;
                    let us = (net_pcap_n * 1000) as u32;
                    let len = frame.len() as u32;
                    let mut rec = [0u8; 16];
                    rec[0..4].copy_from_slice(&0u32.to_le_bytes());
                    rec[4..8].copy_from_slice(&us.to_le_bytes());
                    rec[8..12].copy_from_slice(&len.to_le_bytes());
                    rec[12..16].copy_from_slice(&len.to_le_bytes());
                    f.write_all(&rec).expect("pcap record write");
                    f.write_all(&frame).expect("pcap frame write");
                    if frame.len() >= 14 {
                        println!(
                            "[host] net TX {}B ethertype {:#06x}",
                            frame.len(),
                            u16::from_be_bytes([frame[12], frame[13]])
                        );
                    }
                }
                // Gateway bridge: 4-byte big-endian length prefix + raw
                // frame (same framing the WebSocket bridge client uses;
                // the TCP listener on the gateway side splits on it).
                if let Some(ref mut gw) = net_gw {
                    use std::io::Write as _WriteGw;
                    let len = (frame.len() as u32).to_be_bytes();
                    // NOTE: the socket is NONBLOCKING (set at connect for
                    // the reply drain), so `write_all` can return
                    // WouldBlock mid-frame — the old code dropped the whole
                    // bridge leg on ANY write error, which killed the leg
                    // exactly when the gateway answered faster than the
                    // host drained (proven live: SYNACK staged, leg
                    // dropped, run then wedged). Retry the single frame a
                    // few times on WouldBlock (frames are ≤1600B; the
                    // kernel buffer drains in ms); only a hard error drops
                    // the leg.
                    let mut werr: Option<std::io::Error> = None;
                    for _ in 0..50 {
                        match gw.write_all(&len).and(gw.write_all(&frame)) {
                            Ok(()) => {
                                werr = None;
                                break;
                            }
                            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                                werr = Some(e);
                                std::thread::sleep(std::time::Duration::from_millis(2));
                                continue;
                            }
                            Err(e) => {
                                werr = Some(e);
                                break;
                            }
                        }
                    }
                    if let Some(e) = werr {
                        if e.kind() == std::io::ErrorKind::WouldBlock {
                            println!(
                                "[host] net bridge write still blocked after retry; dropping bridge leg"
                            );
                        } else {
                            println!("[host] net bridge write failed ({e}); dropping bridge leg");
                        }
                        net_gw = None;
                    } else {
                        // A forwarded TX almost always has a causally
                        // linked reply (ARP/DHCP answers, gVisor returns)
                        // — arm the reply drain below for this step.
                        net_tx_this_step = true;
                    }
                }
            }
        }
        // Gateway→board replies (full-duplex leg): nonblocking drain of
        // length-prefixed frames off the same NET_GW TCP connection,
        // staged via `Soc::net_inject_rx` for the `esp_netif_receive`
        // entry hook. Runs ONLY on steps where a TX frame was just
        // forwarded (replies are causally linked to board TX) plus one
        // poll every 64 steps as a backstop for in-flight replies whose
        // TCP segments arrive a few steps after the TX (proven live:
        // the ICMP echo reply arrives ~1 step after its TX, so a
        // TX-only drain delivers the ARP reply but misses the ICMP one;
        // the 64-step backstop catches it — every other step still costs
        // nothing: no syscall at all) and every 1024 steps for truly
        // unsolicited frames (IPv6 RAs, UDP-forward injects). Single
        // `read` calls with a persistent reassembly buffer (never
        // `read_exact` on a nonblocking socket — it consumes partial
        // headers then fails).
        net_step_n += 1;
        let net_poll_backstop = net_step_n.is_multiple_of(64) || net_step_n.is_multiple_of(1024);
        if net_gw.is_some() && (net_tx_this_step || net_poll_backstop) {
            use std::io::Read as _ReadGw;
            // Up to 4 reply frames per drain so a chatty gateway
            // can't starve the emulation loop (each frame ≤1600B;
            // the RX FIFO itself is also bounded at 8). A `drop_leg`
            // flag carries the drop decision out of the `gw` borrow
            // (reassigning `net_gw` while borrowed is E0506).
            let mut drop_leg = false;
            if let Some(ref mut gw) = net_gw {
                // Fill the reassembly buffer: one nonblocking read
                // per iteration (bytes may arrive split across steps
                // — the buffer persists, framing never resyncs).
                for _ in 0..4 {
                    let mut tmp = [0u8; 1600];
                    match gw.read(&mut tmp) {
                        Ok(0) => break, // orderly shutdown; next read errors
                        Ok(n) => net_rx_buf.extend_from_slice(&tmp[..n]),
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                        Err(_) => {
                            println!("[host] net bridge read failed; dropping bridge leg");
                            drop_leg = true;
                            net_rx_buf.clear();
                            break;
                        }
                    }
                    // Extract complete frames from the buffer.
                    loop {
                        if net_rx_buf.len() < 4 {
                            break;
                        }
                        let rlen = u32::from_be_bytes([
                            net_rx_buf[0],
                            net_rx_buf[1],
                            net_rx_buf[2],
                            net_rx_buf[3],
                        ]) as usize;
                        if rlen == 0 || rlen > 1600 {
                            println!(
                                "[host] net bridge bad reply length {rlen}; dropping bridge leg"
                            );
                            drop_leg = true;
                            net_rx_buf.clear();
                            break;
                        }
                        if net_rx_buf.len() < 4 + rlen {
                            break; // body still arriving; keep buffering
                        }
                        let rbuf: Vec<u8> = net_rx_buf[4..4 + rlen].to_vec();
                        net_rx_buf.drain(..4 + rlen);
                        if net_rx_log && rbuf.len() >= 14 {
                            println!(
                                "[host] net RX {}B ethertype {:#06x}",
                                rbuf.len(),
                                u16::from_be_bytes([rbuf[12], rbuf[13]])
                            );
                        }
                        m.soc.net_inject_rx(&rbuf);
                    }
                    // Only loop for more frames if the buffer already
                    // holds another complete one (no extra syscalls).
                    if net_rx_buf.len() < 4 {
                        break;
                    }
                }
            }
            if drop_leg {
                net_gw = None;
            }
        }

        // BLE HCI bridge legs (Bumble virtual controller): forward each
        // captured firmware→controller frame length-prefixed, and drain
        // bridge replies into the RX FIFO. Same nonblocking +
        // reassembly-buffer discipline as the NET_GW leg above (never
        // `read_exact` on a nonblocking socket). Replies are consumed by
        // the VHCI RX hook below (one queued packet per `host_rcv_pkt`
        // call), not staged blindly here.
        if ble_gw.is_some() {
            let frame = m.soc.bt_hci_take_tx();
            if !frame.is_empty() {
                use std::io::Write as _BleW;
                let ok = if let Some(ref mut gw) = ble_gw {
                    let len = (frame.len() as u32).to_be_bytes();
                    gw.write_all(&len).and(gw.write_all(&frame)).is_ok()
                } else {
                    false
                };
                if !ok {
                    println!("[host] BLE bridge write failed; dropping HCI leg");
                    ble_gw = None;
                } else if frame.len() >= 4 {
                    println!(
                        "[host] BLE TX {}B h4={:#04x} op={:#06x}",
                        frame.len(),
                        frame[0],
                        u16::from_le_bytes([frame[1], frame[2]])
                    );
                    // TEMP (2026-10-04, forensics — DELETE after): count
                    // ATT responses for script pacing (shared counter with
                    // the canned drain; scripted twins pace on responses).
                    // ATT pacing: responses (not notify 0x1B/indicate 0x1D)
                    // clear outstanding. Opcode at [9]; guard short.
                    if frame[0] == 0x02 {
                        ble_canned_acl_tx_n += 1;
                        let op = if frame.len() > 9 { frame[9] } else { 0 };
                        if op != 0x1B && op != 0x1D {
                            ble_att_outstanding = ble_att_outstanding.saturating_sub(1);
                            println!("[host] BLE ATT outstanding={ble_att_outstanding}");
                        }
                    }
                }
            }
            let mut drop_ble = false;
            if let Some(ref mut gw) = ble_gw {
                use std::io::Read as _BleR;
                for _ in 0..4 {
                    let mut tmp = [0u8; 4096];
                    match gw.read(&mut tmp) {
                        Ok(0) => break,
                        Ok(n) => ble_rx_buf.extend_from_slice(&tmp[..n]),
                        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                        Err(_) => {
                            println!("[host] BLE bridge read failed; dropping HCI leg");
                            drop_ble = true;
                            ble_rx_buf.clear();
                            break;
                        }
                    }
                    loop {
                        if ble_rx_buf.len() < 4 {
                            break;
                        }
                        let rlen = u32::from_be_bytes([
                            ble_rx_buf[0],
                            ble_rx_buf[1],
                            ble_rx_buf[2],
                            ble_rx_buf[3],
                        ]) as usize;
                        if rlen == 0 || rlen > 4096 {
                            println!("[host] BLE bridge bad reply length {rlen}; dropping HCI leg");
                            drop_ble = true;
                            ble_rx_buf.clear();
                            break;
                        }
                        if ble_rx_buf.len() < 4 + rlen {
                            break;
                        }
                        let rbuf: Vec<u8> = ble_rx_buf[4..4 + rlen].to_vec();
                        ble_rx_buf.drain(..4 + rlen);
                        // WHITELIST (2026-10-04, proven live — DELETE after
                        // with the rest of the BLE TEMP): Bumble sends
                        // post-conn extras (PHY/conn-update/encryption?/keys)
                        // whose unexpected mbufs leak the shared msys pools
                        // (first ATT response then fails at l2cap prepend
                        // NULL → att_tx asserts 0x4200724d:91, while canned
                        // with the same ATT bytes answers 29B clean — proven
                        // live). Allow ONLY what the validated path needs:
                        // 22B conn-complete (LE Meta sub 0x01), ATT ACL
                        // (H4 0x02 — live Bumble ATT post-conn; pre-conn
                        // still dropped by the conn gate below),
                        // Number-of-Completed-Packets (ev 0x13, mbuf credits
                        // — dropping it starves later ATT), Disconnect (ev
                        // 0x05, link lifecycle). Everything else is dropped
                        // here (host-side FIFO pop, no pool): handshake CCs/
                        // version/features/data-length (ROM + canned stager
                        // own them; Bumble's Status-vs-Complete shape
                        // mismatches → `ack 12` + disc, proven live),
                        // PHY/conn-update/encryption (link works at defaults;
                        // central GATT doesn't depend on them). Drops are
                        // logged with ev/op/len — if firmware ever stalls
                        // awaiting a dropped packet, the log names it.
                        let allow = (rbuf.len() == 22
                            && rbuf[0] == 0x04
                            && rbuf[1] == 0x3E
                            && rbuf[3] == 0x01)
                            || (rbuf.first().copied().unwrap_or(0) == 0x02)
                            || (rbuf.len() >= 2 && rbuf[0] == 0x04 && rbuf[1] == 0x13)
                            || (rbuf.len() >= 2 && rbuf[0] == 0x04 && rbuf[1] == 0x05);
                        if !allow {
                            if rbuf.len() >= 7 && rbuf[0] == 0x04 && rbuf[1] == 0x0E {
                                println!(
                                    "[host] BLE RX bridge drop CC op={:#06x} len={}",
                                    u16::from_le_bytes([rbuf[4], rbuf[5]]),
                                    rbuf.len()
                                );
                            } else if rbuf.len() >= 7 && rbuf[0] == 0x04 && rbuf[1] == 0x0F {
                                println!(
                                    "[host] BLE RX bridge drop CS op={:#06x} len={}",
                                    u16::from_le_bytes([rbuf[5], rbuf[6]]),
                                    rbuf.len()
                                );
                            } else if rbuf.len() >= 4 && rbuf[0] == 0x04 && rbuf[1] == 0x3E {
                                println!(
                                    "[host] BLE RX bridge drop LEmeta sub={:#04x} len={}",
                                    rbuf.get(3).copied().unwrap_or(0),
                                    rbuf.len()
                                );
                            } else if rbuf.len() >= 2 && rbuf[0] == 0x04 {
                                println!(
                                    "[host] BLE RX bridge drop ev={:#04x} len={}",
                                    rbuf[1],
                                    rbuf.len()
                                );
                            } else {
                                println!(
                                    "[host] BLE RX bridge drop h4={:#04x} len={}",
                                    rbuf[0],
                                    rbuf.len()
                                );
                            }
                            continue;
                        }
                        if rbuf.len() == 22 && rbuf[0] == 0x04 && rbuf[1] == 0x3E {
                            println!(
                                "[host] BLE RX 22B h4=0x04 ev=0x3e sub=0x01 handle={:#06x} raw={:02x?}",
                                u16::from_le_bytes([rbuf[5], rbuf[6]]),
                                &rbuf[..]
                            );
                        } else if rbuf.len() >= 6
                            && rbuf[0] == 0x04
                            && (rbuf[1] == 0x0E || rbuf[1] == 0x0F)
                        {
                            let rop = u16::from_le_bytes([rbuf[4], rbuf[5]]);
                            println!(
                                "[host] BLE RX {}B h4={:#04x} ev={:#04x} op={:#06x}",
                                rbuf.len(),
                                rbuf[0],
                                rbuf[1],
                                rop
                            );
                        } else if rbuf.len() >= 4 && rbuf[0] == 0x04 {
                            println!(
                                "[host] BLE RX {}B h4={:#04x} ev={:#04x} sub={:#04x}",
                                rbuf.len(),
                                rbuf[0],
                                rbuf[1],
                                rbuf.get(3).copied().unwrap_or(0)
                            );
                        } else {
                            println!("[host] BLE RX {}B h4={:#04x}", rbuf.len(), rbuf[0]);
                        }
                        m.soc.bt_hci_inject_rx(&rbuf);
                    }
                    if ble_rx_buf.len() < 4 {
                        break;
                    }
                }
            }
            if drop_ble {
                ble_gw = None;
            }
        }
        // BLE reply deliver (async event/data path — see
        // `Esp32S3::run_ble_host_recv`): the ROM loopback owns sync command
        // acks firmware-side (the sketch boots to DONE with no bridge), so
        // bridge Command Complete/Status frames (0x04 0x0E / 0x04 0x0F) are
        // DUPLICATES — popped and dropped with a log, never delivered
        // (proven live 2026-10-03: delivering the first Reset CC wedged the
        // boot in panic_abort right after "BLE init 1"). Only async frames
        // (LE Meta, Disconnect, ACL) reach `run_ble_host_recv`, one packet
        // per step max (each delivery runs up to 10k firmware insns).
        // EVT size gate: the firmware resets the host on EVT frames over
        // 71B total (`host_rcv_pkt` length check → `ble_hs_sched_reset` —
        // silicon-true but run-ending), so oversized EVTs are popped and
        // dropped with a log instead of delivered. ACL frames rely on the
        // firmware's own silent bounds checks. A failed delivery (callback
        // undiscovered, or the bounded call aborts) leaves the packet
        // consumed — observable via the central's GATT timeouts, which is
        // the verdict that matters.
        // Boot-phase gate: deliver only after the sketch's post-init
        // marker, i.e. with both cores in valid task contexts. The
        // synthetic `host_rcv_pkt` call runs ON core 1 (like the ESP-NOW
        // closures); before setup() core 1 has no task/SP yet and the
        // call's `entry` spills through a wild stack, smashing DRAM and
        // killing the scheduler (proven live 2026-10-03: delivery on the
        // first staged Reset CC wedged core1 at the exception vector with
        // sp=0x1800). Counts (not a latch) so a mid-run reset re-closes
        // the gate until the rebooted firmware re-inits. The pool gate
        // below stays as defense-in-depth for the transport state.
        let ble_starts = uart_buf
            .windows(b"BLE START".len())
            .filter(|w| *w == b"BLE START")
            .count();
        let ble_inits = uart_buf
            .windows(b"BLE init 1".len())
            .filter(|w| *w == b"BLE init 1")
            .count();
        if (ble_gw.is_some() || ble_canned)
            && ble_starts >= 1
            && ble_inits >= ble_starts
            && m.soc.ble_evt_pool_ready()
            && m.soc.bt_hci_rx_pending() > 0
            // Scheduler-lock gate (proven live 2026-10-03: delivering
            // while EITHER core sits in a scheduler critical section
            // corrupts scheduler lists — the synthetic call reenters
            // queue/scheduler primitives the frozen core holds. The
            // smoking gun was a delivery with core0 parked inside
            // `vTaskSwitchContext` (0x40381bf6) and the lock held,
            // followed by a wild `retw` through heap paint 0xa5a5a5a5 →
            // 0x65a5a5a5. xKernelLock (nm on the BLE ELF) is a mux whose
            // unlocked word is the 0xB33FFFFF magic (observed free at
            // boot/idle; `vPortExitCritical` writes the same magic on
            // release —objdump 0x40380c29; independent S32C1I forensics
            // agrees) — anything else means held. Level-triggered retry:
            // held now just defers to a later step (critical sections
            // are brief; the packet stays queued).
            && m.soc.read32(0x3fc9_9110) == 0xB33F_FFFF
            // PC-BLACKLIST gate (2026-10-04, DELETE after): SMP race fix.
            // The xKernelLock word can read FREE while a core sits
            // mid-pool/queue-update inside these functions (lock released
            // but freelist inconsistent, or a *different* mutex held —
            // pool/list corruption → wild pcs, H4==0 faults, mbuf leaks).
            // Skip delivery while EITHER core's pc lies inside any of them
            // (all brief; packets stay queued). Ranges from nm on the BLE
            // ELF (start..next-symbol, verified 2026-10-04; BLE-image-only
            // leg, so linked addrs are stable).
            && !ble_in_crit(m.cpu[0].pc)
            && !ble_in_crit(m.cpu[1].pc)
        {
            let peek = m.soc.bt_hci_peek_rx_evt();
            let is_cc = matches!(
                peek,
                Some((0x04, Some(0x0E), _)) | Some((0x04, Some(0x0F), _))
            );
            // CC policy by link state (proven live 2026-10-03): pre-link_up
            // every CC is a ROM-loopback duplicate (the ROM answers the
            // whole init sequence firmware-side — drop). Post-link_up the
            // ROM never sees these commands (proven: adv-disable stalls
            // without its CC), so CCs route through the firmware's OWN
            // event path (`host_rcv_pkt` → opcode dispatch → ack match →
            // sem give — correct by construction, self-protecting against
            // duplicates: a CC with no pending command is freed+dropped).
            // The direct-ack shortcut (`ble_ack_deliver_at` into the TX
            // mbuf) is WRONG here and stays parked: `ble_hci_trans_hs_cmd_
            // tx` FREES the command mbuf right after send (objdump
            // 0x420052e5), so by arrival time the tap address names a
            // free-list block, not the checked-out mbuf (proven live:
            // `inrange=false want=0x0000` — the tap at 0x3fcb12b0 names a
            // reused static buffer, opcode bytes long gone).
            if is_cc && !m.soc.ble_link_up() {
                let dropped = m.soc.bt_hci_take_rx();
                println!(
                    "[host] BLE RX sync CC/CS dropped ({}B, ROM owns sync)",
                    dropped.map(|f| f.len()).unwrap_or(0)
                );
            } else {
                let oversize = matches!(peek, Some((0x04, _, len)) if len > 71);
                // Connection-gate (proven live 2026-10-03): an ATT ACL for a
                // connection the host hasn't established yet walks a NULL
                // conn struct (wild `retw` through heap paint 0xa5a5a5a5 →
                // 0x65a5a5a5, double-fault `break`). The 22B connection
                // event and the 16B ATT arrive back-to-back from the
                // bridge, but the firmware task needs many steps to turn
                // the event into a conn object — delivering the ATT first
                // races it. Gate ACLs on the sketch's `BLE conn 1` marker
                // (same marker-gated discipline as ESP-NOW's `sent 1`):
                // the packet stays queued (level-triggered retry) until
                // the connection exists. Events are never gated (they ARE
                // what establishes it).
                let is_acl = matches!(peek, Some((0x02, _, _)));
                let conn_up = uart_buf
                    .windows(b"BLE conn 1".len())
                    .any(|w| w == b"BLE conn 1");
                // TEMP (2026-10-03): `BLE_RX_ACL_DROP=1` pops and drops
                // ATT ACLs instead of delivering them (split forensics).
                let acl_drop = is_acl && std::env::var("BLE_RX_ACL_DROP").is_ok();
                // FULL-HANDSHAKE gate (2026-10-04, DELETE after): the `BLE
                // conn 1` marker fires from gap onConnect EARLY (right after
                // the 22B, before version/features/data-length complete), so
                // marker-gated ATTs land on a HALF-ESTABLISHED conn (version
                // unknown? features pending? data-length default?) and die
                // in att_tx (l2cap assert 0x4200724d:91, deterministic 5/5
                // live runs; canned stages ATT after dl_done and answers 29B
                // clean — proven live). Require the data-length event staged
                // too (ble_canned_dl_done is set for bridge as well via the
                // extended stager — handshake fully driven by then). Pre-dl
                // ATTs DROP (central retries post-handshake; same discipline
                // as pre-conn).
                // NOTE: ble_canned_dl_done is declared below (canned stager
                // section) — Rust block scoping needs it visible here. It is
                // a `let mut` in the same fn scope ABOVE this point? No —
                // declarations sit near the top (before the loop), so it is
                // in scope here (assigned later in-loop). Borrowck: read-only
                // use here, mutable assign later — fine (sequential).
                // ATT pacing (2026-10-04, DELETE after): deliver only when
                // none outstanding (see counter docs). Gated ATTs stay queued
                // (level-triggered) until the response drains.
                if is_acl && (!conn_up || !ble_canned_dl_done || ble_att_outstanding > 0) {
                    // Head-of-line block fix (2026-10-04, proven live): a
                    // premature ATT ACL (central's service discovery sent
                    // immediately after the 22B, before the firmware
                    // finishes version/features/data-length) sits at the
                    // FIFO head and STARVES the version/features events
                    // behind it (firmware times out `HCI wait for ack 19`,
                    // no conn, central GATT timeout). The old leave-queued
                    // policy deadlocks (ACL needs conn, conn needs events
                    // behind the ACL).
                    //
                    // HOLD vs DROP (2026-10-04): pre-dl/pre-conn ATTs are
                    // HELD (pop into `ble_held_att`, FIFO goes empty so the
                    // data-length stager unblocks; re-injected after the
                    // data-length event delivers — see below). DROP only
                    // applies when already held (second premature ATT) or
                    // when pacing-blocked post-dl (outstanding>0, central
                    // pipelines — retry covers it). DROP for a patient
                    // (300s-timeout) central is fatal (single discovery
                    // never resent); HOLD preserves it.
                    if (!conn_up || !ble_canned_dl_done) && ble_held_att.is_none() {
                        ble_held_att = m.soc.bt_hci_take_rx();
                        if let Some(ref f) = ble_held_att {
                            println!(
                                "[host] BLE RX pre-dl ATT held ({}B, re-inject after data-length) bytes={:02x?}",
                                f.len(),
                                &f[..f.len().min(16)],
                            );
                        } else {
                            println!(
                                "[host] BLE RX pre-dl ATT hold missed (0B, central retries post-handshake)"
                            );
                        }
                    } else {
                        let dropped = m.soc.bt_hci_take_rx();
                        // TEMP (2026-10-04, forensics — DELETE after): full hex
                        // of the premature ATT (replay it post-conn in canned
                        // to prove ATT→response offline).
                        if let Some(ref f) = dropped {
                            println!(
                                "[host] BLE RX pre-conn ACL dropped ({}B, central retries post-conn) bytes={:02x?}",
                                f.len(),
                                f,
                            );
                        } else {
                            println!(
                                "[host] BLE RX pre-conn ACL dropped (0B, central retries post-conn)"
                            );
                        }
                    }
                } else if acl_drop {
                    let dropped = m.soc.bt_hci_take_rx();
                    println!(
                        "[host] BLE RX ACL dropped by BLE_RX_ACL_DROP ({}B)",
                        dropped.map(|f| f.len()).unwrap_or(0)
                    );
                } else if oversize {
                    let dropped = m.soc.bt_hci_take_rx();
                    println!(
                        "[host] BLE RX oversize EVT dropped ({}B)",
                        dropped.map(|f| f.len()).unwrap_or(0)
                    );
                } else if let Some(cb) = ble_cb_entry
                    && {
                        // HOST-ENABLED gate (2026-10-04, DELETE after):
                        // host_rcv_pkt opens with `if (!ble_hs_enabled_state)
                        // return 0` (BSS 0x3fc9dd70, nm on the BLE ELF —
                        // BLE-image-only leg, like ble_cb_entry itself). A
                        // disabled host means the packet would be CONSUMED
                        // (stage pops first) and dropped on the floor; worse,
                        // pre-SYNTH_RETPC builds faulted the early retw and
                        // corrupted state into a restage loop. Skip WITHOUT
                        // consuming while disabled (level-triggered retry,
                        // same discipline as no-callback-yet). Log
                        // transitions only (per-step spam would flood).
                        // NOTE: a host that never re-enables stalls here by
                        // design (firmware-side waiter timeout → reset is
                        // observable + diagnosable; silent corruption is not).
                        if m.soc.read32(0x3fc9_dd70) == 0 {
                            if !ble_host_disabled {
                                println!(
                                    "[host] BLE host disabled (enabled_state==0), deferring delivery"
                                );
                                ble_host_disabled = true;
                            }
                            false
                        } else {
                            ble_host_disabled = false;
                            // TEMP (2026-10-03): pool drain watch (DELETE after).
                            // Did the synthetic call consume an ev-pool block?
                            // Plus the SMP interleaving: both cores' pcs + the
                            // scheduler lock (xKernelLock 0x3fc99110 — nm on the
                            // ble ELF). If the other core sits mid-critical-
                            // section when we synthesize scheduler-touching
                            // firmware, lists corrupt -> garbage TCB -> wild
                            // retw through paint.
                            let f0 = m.soc.ble_evt_pool_free();
                            // TEMP (2026-10-03): capture the pre-call pool
                            // free-list head (DELETE after) — the block our
                            // call will check out. NPL event = {queued@0,
                            // fn@4, arg@8} per npl_freertos.h.
                            let pre_head = m.soc.read32(0x3fc9_df6c + 20);
                            // TEMP (2026-10-03): queue depth pre/post (DELETE
                            // after). Corrected layout (from the sem mw/len
                            // offsets the ack path proves: counts at +56/+60):
                            // depth=[q+56]. 0→1 = our post landed; 1→0 = task
                            // took it.
                            let evq0 = m.soc.read32(0x3fc9_dd50);
                            let q0 = m.soc.read32(evq0);
                            let depth0 = m.soc.read32(q0.wrapping_add(56));
                            println!(
                                "[host] BLE pre core0={:#010x} core1={:#010x} lock={:#010x}",
                                m.cpu[0].pc,
                                m.cpu[1].pc,
                                m.soc.read32(0x3fc9_9110)
                            );
                            // TEMP (2026-10-03): evq waiter check (DELETE after).
                            // Is any task parked on ble_hs_evq when we post?
                            // ble_hs_evq (0x3fc9dd50) word0 = 0x3fc9ea68 =
                            // g_eventq_dflt STRUCT whose word0 (0x3fcb1430)
                            // is the real FreeRTOS Queue_t (heap). Check the
                            // waiter on the QUEUE, not the wrapper, and dump
                            // both layouts. PLUS the receive list itself
                            // (+36: count, pxIndex+40, end.next+44): whose TCB
                            // is parked? (host_task_h for comparison.)
                            // PLUS pre-call IRAM probe (DELETE after): is
                            // 0x40380a7c still intact right BEFORE the
                            // synthetic call? Splits synthetic-call clobber
                            // vs idle-time drift.
                            // PLUS window-liveness snapshot (DELETE after):
                            // stale WINDOWSTART bits (never cleared) make
                            // overflow rotate into ALIASED live windows (a1
                            // = another task's SP, [SP-12] = its saved
                            // retaddr) → spill base wild → IRAM clobber.
                            println!(
                                "[host] BLE pre IRAM @0x40380a7c = {:#010x}",
                                m.soc.read32(0x4038_0a7c)
                            );
                            println!(
                                "[host] BLE pre wb={} wstart={:#010x} excsave1={:#010x}",
                                m.cpu[0].windowbase(),
                                m.cpu[0].sreg(xtensa_core::cpu::SR_WINDOW_START),
                                m.cpu[0].sreg(xtensa_core::cpu::SR_EXCSAVE1),
                            );
                            // TEMP (2026-10-03, forensics — DELETE after): OF12
                            // handler inputs — a13 (scratch base) + [SP-12]
                            // (spill base) in the CURRENT (wb+2-rotated? no:
                            // pre-call, firmware) view. If either names IRAM
                            // code, the first overflow spill clobbers it.
                            {
                                let sp = m.cpu[0].reg(1);
                                println!(
                                    "[host] BLE pre a13={:#010x} sp={:#010x} sp-12-mem={:#010x}",
                                    m.cpu[0].reg(13),
                                    sp,
                                    m.soc.read32(sp.wrapping_sub(12)),
                                );
                            }
                            // PLUS waiter pre/post (DELETE after): does our
                            // post unblock it? Sampled around the synthetic
                            // call — unblock is synchronous inside Send.
                            let evq = m.soc.read32(0x3fc9_dd50);
                            let queue = m.soc.read32(evq);
                            let evq_wait = if queue != 0 {
                                m.soc.queue_recv_waiting(queue)
                            } else {
                                false
                            };
                            let ht0 = m.soc.read32(0x3fc9_eac8);
                            println!(
                                "[host] BLE pre evq={evq:#010x} queue={queue:#010x} q_wait={evq_wait} host={ht0:#010x}"
                            );
                            println!(
                                "[host] BLE pre rlist-detail count={} first={:#010x}",
                                m.soc.read32(queue.wrapping_add(32)),
                                m.soc.read32(queue.wrapping_add(40)),
                            );
                            // TEMP (2026-10-03): full Queue_t dump (DELETE
                            // after). Is the queue FULL when we post (post
                            // fails silently -> block leaks -> waiter stays
                            // parked -> silence)? Words +0..+64.
                            println!(
                                "[host] BLE pre qstruct={:#010x} {:#010x} {:#010x} {:#010x} {:#010x} {:#010x} {:#010x} {:#010x}",
                                m.soc.read32(queue),
                                m.soc.read32(queue.wrapping_add(4)),
                                m.soc.read32(queue.wrapping_add(8)),
                                m.soc.read32(queue.wrapping_add(12)),
                                m.soc.read32(queue.wrapping_add(16)),
                                m.soc.read32(queue.wrapping_add(20)),
                                m.soc.read32(queue.wrapping_add(24)),
                                m.soc.read32(queue.wrapping_add(28)),
                            );
                            // TEMP (2026-10-03): queue length/occupancy words
                            // (DELETE after). Standard Queue_t has
                            // uxMessagesWaiting@48, uxLength@52, uxItemSize@56
                            // (after two 16B lists at +16/+32). A length-1
                            // queue + an occupying timer event at post time =
                            // our post fails silently (0 timeout) and leaks.
                            // Dump +32..+72 to find the real count fields
                            // (small ints among pointers; sample twice).
                            println!(
                                "[host] BLE pre qlen={:#010x} {:#010x} {:#010x} {:#010x}",
                                m.soc.read32(queue.wrapping_add(48)),
                                m.soc.read32(queue.wrapping_add(52)),
                                m.soc.read32(queue.wrapping_add(56)),
                                m.soc.read32(queue.wrapping_add(60)),
                            );
                            println!(
                                "[host] BLE pre qx32={:#010x} {:#010x} {:#010x} {:#010x} {:#010x} {:#010x}",
                                m.soc.read32(queue.wrapping_add(32)),
                                m.soc.read32(queue.wrapping_add(36)),
                                m.soc.read32(queue.wrapping_add(40)),
                                m.soc.read32(queue.wrapping_add(44)),
                                m.soc.read32(queue.wrapping_add(64)),
                                m.soc.read32(queue.wrapping_add(68)),
                            );
                            // TEMP (2026-10-04, forensics — DELETE after):
                            // pre-delivery pointed-queue dump (8 words at
                            // *handle) to compare with PANIC-SEM (find count
                            // offset; pre-seed writes queue+0x38).
                            {
                                let h = m.soc.read32(0x3fc9_d6d0);
                                let mut w = [0u32; 8];
                                for (k, v) in w.iter_mut().enumerate() {
                                    *v = m.soc.read32(h.wrapping_add((k as u32) * 4));
                                }
                                println!("[host] BLE pre-SEM [{h:#010x}] {w:08x?}");
                            }
                            let ok = m.run_ble_host_recv(0, cb);
                            println!(
                                "[host] BLE RX host_rcv_pkt ok={ok} evfree {f0}->{})",
                                m.soc.ble_evt_pool_free()
                            );
                            // TEMP (2026-10-04, forensics — DELETE after): staged
                            // bytes (H4==0 dead-path diagnosis).
                            if let Some((sb, sl, sx)) = m.last_recv_stage {
                                println!(
                                    "[host] BLE RX staged buf={sb:#010x} len={sl} bytes={sx:02x?}"
                                );
                            }
                            // PRE-DL HOLD re-inject (2026-10-04, DELETE after):
                            // when the data-length event itself delivers (14B
                            // LE-meta subevent 0x07 — the handshake is now
                            // fully established firmware-side), restore the
                            // held ATT to the FIFO back. Next steps deliver
                            // it with conn_up + dl_done true, so the ATT TX
                            // no longer asserts. Fires once per hold (slot
                            // clears); a failed delivery (ok=false) still
                            // re-injects (the data-length event reached the
                            // handler queue either way — abort records show
                            // delivery faults, not staging faults).
                            if m.last_recv_stage
                                .is_some_and(|(_, sl, sx)| sl == 14 && sx[3] == 0x07)
                                && let Some(held) = ble_held_att.take()
                            {
                                println!(
                                    "[host] BLE held ATT re-injected ({}B) after data-length",
                                    held.len(),
                                );
                                m.soc.bt_hci_inject_rx(&held);
                            }
                            // TEMP (2026-10-03, forensics — DELETE after): print
                            // the synthetic-call abort record (fault pc + step
                            // result) when delivery fails.
                            if !ok {
                                if let Some((apc, ar, trip)) = m.last_recv_abort {
                                    println!(
                                        "[host] BLE RX abort at {apc:#010x}: {ar:?} woe_trip={trip:#010x?}"
                                    );
                                    // TEMP (2026-10-03, forensics — DELETE
                                    // after): IRAM clobber site, if observed.
                                    if let Some(cpc) = m.last_recv_clobber {
                                        println!(
                                            "[host] BLE RX clobber first seen after step at {cpc:#010x}"
                                        );
                                    }
                                    // TEMP (2026-10-03, forensics — DELETE
                                    // after): PS at synthetic exit (EXCM set
                                    // ⇒ inside a vector handler).
                                    if let Some((ps, epc1, wb)) = m.last_recv_ps {
                                        println!(
                                            "[host] BLE RX exit ps={ps:#010x} epc1={epc1:#010x} wb={wb}"
                                        );
                                    }
                                    // TEMP (2026-10-03, forensics — DELETE
                                    // after): dump what the CPU actually
                                    // fetched at the fault pc (IRAM backing
                                    // may differ from the ELF file).
                                    println!(
                                        "[host] BLE RX fault bytes @ {apc:#010x} = {:#010x}",
                                        m.soc.read32(apc)
                                    );
                                } else {
                                    println!(
                                        "[host] BLE RX abort: clean exit without retw (10k cap?)"
                                    );
                                }
                            }
                            // TEMP (2026-10-03): WOE snapshot (DELETE after).
                            // ENTRY faults cause 0 iff PS.WOE==0 — snapshot PS
                            // at every delivery attempt: was WOE already clear
                            // in the idle task state, or cleared mid-call?
                            println!(
                                "[host] BLE pre ps={:#010x} woe={}",
                                m.cpu[0].sreg(xtensa_core::cpu::SR_PS),
                                (m.cpu[0].sreg(xtensa_core::cpu::SR_PS) & xtensa_core::cpu::PS_WOE)
                                    != 0,
                            );
                            // TEMP (2026-10-03): waiter post/post (DELETE
                            // after). Unblock is synchronous inside Send: if
                            // still parked after our post, the post didn't
                            // unblock (wrong queue/full/silent error).
                            println!(
                                "[host] BLE pre waiter_post={} depth_post={}",
                                if queue != 0 {
                                    m.soc.queue_recv_waiting(queue)
                                } else {
                                    false
                                },
                                m.soc.read32(queue.wrapping_add(56)),
                            );
                            // TEMP (2026-10-03): depth after (DELETE after).
                            let evq1 = m.soc.read32(0x3fc9_dd50);
                            let q1 = m.soc.read32(evq1);
                            println!(
                                "[host] BLE pre depth {depth0}->{}",
                                m.soc.read32(q1.wrapping_add(56))
                            );
                            // TEMP (2026-10-03): handler set on our block?
                            // (DELETE after). Expect fn == ble_hs_event_rx_hci_ev.
                            // PLUS the data mbuf head (ev_arg points at it):
                            // [0]==0x3E means H4-stripped (correct), 0x04 means
                            // the H4 byte leaked into the mbuf (dispatcher then
                            // misindexes the LE table and drops).
                            println!(
                                "[host] BLE pre ev_fn={:#010x} ev_arg={:#010x}",
                                m.soc.read32(pre_head.wrapping_add(4)),
                                m.soc.read32(pre_head.wrapping_add(8)),
                            );
                            let dm = m.soc.read32(pre_head.wrapping_add(8));
                            println!(
                                "[host] BLE pre dmbuf={:02x?}",
                                [
                                    m.soc.read8(dm) as u8,
                                    m.soc.read8(dm.wrapping_add(1)) as u8,
                                    m.soc.read8(dm.wrapping_add(2)) as u8,
                                    m.soc.read8(dm.wrapping_add(3)) as u8,
                                    m.soc.read8(dm.wrapping_add(4)) as u8,
                                    m.soc.read8(dm.wrapping_add(5)) as u8,
                                ]
                            );
                            ok
                        } // end else (host enabled): ok from the call above
                    }
                {
                    println!("[host] BLE RX delivered via host_rcv_pkt");
                    // First async delivery proves the link is up (see the
                    // `ble_link_up` field docs): from here CCs belong to
                    // post-connection commands (arrival ack leg below),
                    // not to the ROM loopback.
                    m.soc.ble_mark_link_up();
                    ble_link_up_seq = m.soc.ble_tx_seq();
                    // ATT pacing (2026-10-04, DELETE after): count this
                    // delivery if ATT (response TX will clear it).
                    if is_acl {
                        ble_att_outstanding += 1;
                        println!("[host] BLE ATT outstanding={ble_att_outstanding}");
                    }
                    // TEMP (2026-10-03): post-delivery pc window (DELETE
                    // after). Record both cores' pcs for the next 150
                    // macro-steps: what does the woken task do?
                    ble_watch_n = 150;
                }
            }
        }
        // TEMP (2026-10-03): post-delivery pc window (DELETE after).
        if ble_watch_n > 0 {
            ble_watch_n -= 1;
            println!(
                "[host] BLEWATCH step={i} c0={:#010x} c1={:#010x}",
                m.cpu[0].pc, m.cpu[1].pc
            );
        }
        // Command-ack arrival notes (no active leg — see the CC policy
        // above): post-link_up CCs flow through the async event path
        // (`host_rcv_pkt`), never through direct mbuf writes. The
        // `ble_ack_deliver_at` helper stays parked and unit-tested for a
        // hypothetical image whose TX mbuf stays checked out across the
        // ack (this image frees at send — objdump 0x420052e5).
        //
        // Direct-ack V2 completing parked post-connection waiters (see
        // `ble_ack_write_cc`): fires when a waiter is parked on the ack
        // sem for a command sent AFTER link-up (mutex-serialized, so the
        // latest capture is the waited one — never a stale init opcode),
        // with the scheduler lock free and the ev pool ready. The ack-cell
        // gate inside makes it once-per-episode (no double-bump, no leak).
        // TEMP-NOTE (2026-10-03): V2 DISABLED (proven harmful live 2026-10-04):
        // with the WINDOWSTART fix the event path delivers (host_rcv_pkt
        // ok=true for CCs), so V2 double-completes: V2 writes the ack cell
        // for 0x041d while the ROM loopback ALSO answers it → firmware
        // panics in ble_transport_free (`assert failed: 0x4201453a:290`,
        // a2=0x3fcb78a4, proven live with bridge). V2 was load-bearing
        // BEFORE the fix (event path ok=false, V2 the only completion);
        // now the event path works and V2 must stay off. Kept (not deleted)
        // for the forensics record; re-enable only with a bridge-off test.
        // (Original note preserved: V2 ALSO covered bridge, first-come-wins
        // was claimed safe — disproven by the 0x4201453a panic.)
        if false
            && (ble_gw.is_some() || ble_canned)
            && m.soc.ble_link_up()
            && m.soc.ble_evt_pool_ready()
            && m.soc.read32(0x3fc9_9110) == 0xB33F_FFFF
            && m.soc.ble_tx_seq() > ble_link_up_seq
            && m.soc.ble_last_op() != 0
        {
            let sem = m.soc.ble_ack_sem();
            if sem != 0
                && m.soc.queue_recv_waiting(sem)
                && let Some(tcb) = m.soc.ble_ack_write_cc(m.soc.ble_last_op())
            {
                m.soc.ble_ready_task(tcb);
                println!("[host] BLE ack V2 delivered");
            }
        }
        // BLE VHCI RX hook: the callback address is discovered once from
        // the `vhci_host_cb` rodata (`BLE_HOST_CB`, BLE image only):
        // slot 0 = `notify_host_send_available`, slot 1 =
        // `notify_host_recv` (struct order in esp_nimble_hci.c, proven by
        // the 0x42005134 / 0x42005158 words at 0x3c06b82c). Delivery itself
        // happens in the block above via `run_ble_host_recv`.
        if (ble_gw.is_some() || ble_canned) && ble_cb_entry.is_none() {
            let cb0 = m.soc.read32(BLE_HOST_CB);
            let cb1 = m.soc.read32(BLE_HOST_CB + 4);
            if (0x4000_0000..0x4240_0000).contains(&cb1) && cb0 != 0 {
                ble_cb_entry = Some(cb1);
                println!("[host] BLE vhci_host_cb: notify_host_recv={cb1:#010x}");
            }
        }

        // TEMP (2026-10-03): host-task schedule watch (DELETE after).
        // Does the NimBLE host task run AFTER our post? Latch on the
        // first step either core currently runs it, gated on link_up
        // (set at delivery success) so pre-delivery runs don't count.
        // PLUS queue trend: depth (uxMessagesWaiting @ +48, same layout
        // `queue_recv_waiting` uses) + waiter present, throttled — does
        // the queued event ever drain?
        if ble_canned && m.soc.ble_link_up() {
            if !ble_host_seen {
                let ht = m.soc.read32(0x3fc9_eac8);
                if ht != 0 {
                    let c0 = m.soc.read32(0x3fc9_f618);
                    let c1 = m.soc.read32(0x3fc9_f61c);
                    if c0 == ht || c1 == ht {
                        println!(
                            "[host] BLEDBG-HOST step={i} host_runs c0={c0:#010x} c1={c1:#010x} ht={ht:#010x}"
                        );
                        ble_host_seen = true;
                    }
                }
            }
            if i.is_multiple_of(1_000_000) {
                let evq = m.soc.read32(0x3fc9_dd50);
                println!(
                    "[host] BLEDBG-Q step={i} depth={} waiter={} evfree={}",
                    m.soc.read32(evq.wrapping_add(48)),
                    if evq != 0 {
                        m.soc.queue_recv_waiting(evq)
                    } else {
                        false
                    },
                    m.soc.ble_evt_pool_free(),
                );
            }
        }

        if ble_gw.is_some() || ble_canned {
            // TEMP (2026-10-03): in canned mode (no bridge) drain+log TX
            // captures so post-22B firmware commands are visible (does the
            // conn handler send Read-Remote-Version and stall awaiting its
            // CC?). DELETE after forensics.
            if ble_canned {
                let frame = m.soc.bt_hci_take_tx();
                if !frame.is_empty() && frame.len() >= 3 {
                    // TEMP (2026-10-03): tap-address audit (DELETE after).
                    // Which pool does each command mbuf come from?
                    let (ld, _ll) = m.soc.ble_last_tx().unwrap_or((0, 0));
                    println!(
                        "[host] BLE TX canned {}B h4={:#04x} op={:#06x} tap={:#010x}",
                        frame.len(),
                        frame[0],
                        u16::from_le_bytes([frame[1], frame[2]]),
                        ld,
                    );
                    // TEMP (2026-10-04, forensics — DELETE after): full hex
                    // for ACL TX (decode ATT discovery response handles for
                    // canned READ/WRITE replay). Count ACL TX (each ATT
                    // response) for script pacing (see findinfo stager).
                    // ATT pacing (2026-10-04, DELETE after): responses clear
                    // outstanding (not notify 0x1B / indicate 0x1D — those
                    // are server-initiated, not answers). Opcode at [9]
                    // (H4+handle2+acl_len2+l2cap_len2+cid2); guard short.
                    if frame[0] == 0x02 {
                        println!("[host] BLE TXACL {:02x?}", &frame[..]);
                        ble_canned_acl_tx_n += 1;
                        let op = if frame.len() > 9 { frame[9] } else { 0 };
                        if op != 0x1B && op != 0x1D {
                            ble_att_outstanding = ble_att_outstanding.saturating_sub(1);
                            println!("[host] BLE ATT outstanding={ble_att_outstanding}");
                        }
                    }
                }
            }
        }
        // Full-802.11-LL-MAC slice-1 leg (`esp32s3_llmac` sketch,
        // `WIFI_LLMAC=1`): the machine tap arms capture the TX frame +
        // the promiscuous callback pointer; here the host drains captures
        // (one log line per frame, with the battery-asserted CAP marker)
        // and injects virtual-AP beacons by running the registered
        // callback IN FIRMWARE, one per step max (each delivery runs up
        // to 10k firmware insns). Boot-phase gate: deliver only after the
        // sketch's `LLMAC sniff 1` marker (setup done — the synthetic
        // call needs a valid task stack, same wild-stack class as the
        // BLE boot-phase gate).
        // TEMP (2026-10-03): deterministic BLE NULL-call repro — with
        // (no bridge needed) the first delivery opportunity after init
        // stages a canned LE Connection Complete (bytes captured from a
        // live Bumble run) instead of needing the central rendezvous.
        // Isolates 22B content processing from link timing. DELETE after
        // forensics.
        // DONE-gate (load-bearing): staging the 22B mid-init flips
        // `link_up` while init sends are still in flight — the CC policy
        // switches from drop-as-ROM-dup to deliver, and V2 + the stager
        // below start completing init commands the ROM ALSO answers
        // (double completion → `ack returned 12/17`, `BLE adv 0`, proven
        // live). Post-DONE the firmware sends nothing until the conn
        // handler runs, so no capture can straddle the mark.
        if ble_canned && !ble_canned_done && m.soc.bt_hci_rx_pending() == 0 {
            let ble_starts = uart_buf
                .windows(b"BLE START".len())
                .filter(|w| *w == b"BLE START")
                .count();
            let ble_done = uart_buf
                .windows(b"BLE DONE".len())
                .any(|w| w == b"BLE DONE");
            // TEMP (2026-10-03, forensics — DELETE after): one-shot IRAM
            // probe — is 0x40380a7c (xPortInIsrContext entry) intact
            // post-boot, pre-delivery? Distinguishes loader hole (bad from
            // load) from runtime clobber.
            if ble_done && !ble_canned_done && !ble_iram_probed {
                println!(
                    "[host] BLE IRAM probe @0x40380a7c = {:#010x} (expect 0xa0004136)",
                    m.soc.read32(0x4038_0a7c)
                );
                ble_iram_probed = true;
            }
            if ble_starts >= 1 && ble_done && m.soc.ble_evt_pool_ready() {
                // Supervision timeout widened 0x000A→0x0C80 (100ms→32s):
                // the live-captured 100ms timeout disconnects the canned
                // link before ATT runs (no central to sustain it); 32s
                // keeps it up through GATT. Interval/latency/accuracy kept.
                m.soc.bt_hci_inject_rx(&[
                    0x04, 0x3E, 0x13, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x25, 0xE3, 0xD9, 0x2E,
                    0x18, 0xFF, 0x0A, 0x00, 0x00, 0x00, 0x80, 0x0C, 0x07,
                ]);
                ble_canned_done = true;
                println!("[host] BLE canned 22B staged");
            }
        }
        // TEMP (2026-10-03): canned controller (DELETE after). Once the
        // link is up, every outstanding command needs its Command Complete
        // (canned has no bridge): the conn handler sends adv-disable, then
        // read-remote-version, etc., and stalls on each waiter without its
        // CC (no app `onConnect`, no ATT response — proven live: post-22B
        // TX then silence). Stage a status-0 CC for the outstanding TX
        // opcode whenever the FIFO is empty (post-conn CCs are status-only;
        // init-time commands never reach here — the ROM loopback answers
        // those firmware-side and `ble_link_up` is false until the first
        // async delivery).
        // TEMP (2026-10-03): generic canned-CC stager RE-ENABLED
        // (DELETE after). The trace proves the handler chain runs
        // (le_meta → table[1] → gap_conn_complete) but stalls awaiting
        // the adv-disable CC (no app `onConnect` without it). One CC per
        // captured command (seq-guarded against the static-buffer stale
        // reads). The `> ble_link_up_seq` gate is load-bearing: without
        // it the first post-link-up evaluation serves the LAST INIT
        // capture (watermark starts at 0), staging a duplicate CC the
        // waiter consumes as a mismatch (`ack returned 12`, proven live).
        if (ble_canned || ble_gw.is_some())
            && m.soc.ble_link_up()
            && m.soc.bt_hci_rx_pending() == 0
            && m.soc.ble_tx_seq() > ble_link_up_seq
            && m.soc.ble_tx_seq() != ble_canned_last_op
        {
            let op = m.soc.ble_last_op();
            // Serve exactly one CC per captured command (see `ble_tx_seq`).
            // Bridge override (2026-10-04, pragmatic unblock — DELETE
            // after): with a bridge only 0x041d/0x2016/0x2022 are staged
            // here (bridge's own replies for those are dropped at ingest;
            // Bumble's Status-vs-Complete shape mismatches NimBLE → `ack
            // 12` + disc, proven live). Canned serves all (no Bumble).
            if op != 0 && (ble_canned || op == 0x041D || op == 0x2016 || op == 0x2022) {
                // Opcode-specific CC shape (BT Core + hci_common.h): most
                // post-conn commands ack status-only, but 0x2022
                // (LE_SET_DATA_LEN) returns status + conn_handle
                // (`ble_hci_le_set_data_len_rp`: u16 handle). A short CC
                // makes the handler read handle 0, log `Received status 0`
                // (E1069) and disconnect — proven live.
                if op == 0x2022 {
                    m.soc.bt_hci_inject_rx(&[
                        0x04,
                        0x0E,
                        0x06,
                        0x01,
                        (op & 0xFF) as u8,
                        ((op >> 8) & 0xFF) as u8,
                        0x00,
                        0x01,
                        0x00,
                    ]);
                } else {
                    m.soc.bt_hci_inject_rx(&[
                        0x04,
                        0x0E,
                        0x04,
                        0x01,
                        (op & 0xFF) as u8,
                        ((op >> 8) & 0xFF) as u8,
                        0x00,
                    ]);
                }
                ble_canned_last_op = m.soc.ble_tx_seq();
                ble_canned_last_cc_op = op;
                println!("[host] BLE canned CC staged for op={op:#06x}");
            }
        }
        // TEMP (2026-10-03): canned version-complete event (DELETE after).
        // Mirrors the bridge's 0x041D follow-up: after the rd-rem-ver CC
        // is served, stage the 0x0C event (status 0, handle 1, BT 5.0,
        // Espressif) so the handler doesn't stall awaiting it. Bridge
        // override (2026-10-04): also serves with a bridge (its own
        // version is dropped at ingest); Bumble owns the rest.
        if (ble_canned || ble_gw.is_some())
            && ble_canned_last_cc_op == 0x041D
            && !ble_canned_ver_done
            && m.soc.bt_hci_rx_pending() == 0
        {
            m.soc.bt_hci_inject_rx(&[
                0x04, 0x0C, 0x08, 0x00, 0x01, 0x00, 0x09, 0xE5, 0x02, 0x00, 0x00,
            ]);
            ble_canned_ver_done = true;
            println!("[host] BLE canned version event staged");
        }
        // TEMP (2026-10-03): canned remote-features-complete event (DELETE
        // after). After the 0x2016 CC, stage the LE Meta subevent-0x04
        // (status 0, handle 1 from the 22B, all-FF features = full LE
        // support) so the handler completes and fires onConnect. Zeros
        // proved the path (conn fires) but NimBLE then logs `Controller
        // doesn't support LE` and disconnects; FF keeps the link up.
        // Bridge override (2026-10-04): also serves with a bridge
        // (its features event is dropped at ingest).
        if (ble_canned || ble_gw.is_some())
            && ble_canned_last_cc_op == 0x2016
            && !ble_canned_feat_done
            && m.soc.bt_hci_rx_pending() == 0
        {
            m.soc.bt_hci_inject_rx(&[
                0x04, 0x3E, 0x0C, 0x04, 0x00, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                0xFF,
            ]);
            ble_canned_feat_done = true;
            println!("[host] BLE canned features event staged");
        }
        // TEMP (2026-10-03): canned data-length-change event (DELETE after).
        // After the 0x2022 CC, stage the LE Meta subevent-0x07 (handle 1,
        // 251B/2120us both directions) so the handler completes instead of
        // disconnecting. Bridge override (2026-10-04): also serves with a
        // bridge (its data-length event is dropped at ingest).
        if (ble_canned || ble_gw.is_some())
            && ble_canned_last_cc_op == 0x2022
            && !ble_canned_dl_done
            && m.soc.bt_hci_rx_pending() == 0
        {
            // LE Data Length Change: subevent 0x07, NO status byte (BT Core
            // 7.7.65.13): handle + max_tx_oct/time + max_rx_oct/time.
            m.soc.bt_hci_inject_rx(&[
                0x04, 0x3E, 0x0B, 0x07, 0x01, 0x00, 0xFB, 0x00, 0x48, 0x08, 0xFB, 0x00, 0x48, 0x08,
            ]);
            ble_canned_dl_done = true;
            println!("[host] BLE canned data-length event staged");
        }
        // TEMP (2026-10-04, forensics — DELETE after): canned ATT replay
        // (see flag docs). SIZE TEST ORDER: after the READFIRST response
        // (acl_tx_n>=1), not directly after dl_done.
        if ble_canned
            && ble_canned_dl_done
            && !ble_canned_att_done
            && m.soc.bt_hci_rx_pending() == 0
        {
            m.soc.bt_hci_inject_rx(&[
                0x02, 0x01, 0x20, 0x0B, 0x00, 0x07, 0x00, 0x04, 0x00, 0x10, 0x01, 0x00, 0xFF, 0xFF,
                0x00, 0x28,
            ]);
            ble_canned_att_done = true;
            println!("[host] BLE canned ATT staged");
        }
        // TEMP (2026-10-04, forensics — DELETE after): canned Find-Info
        // (Phase 1). After the discovery response (first ACL TX) with the
        // FIFO empty, request attributes 0x000e-0xFFFF (Find Info 0x04) to
        // learn characteristic/value handles (decoded from the TXACL
        // response hex). Full ACL: H4 + handle/flags 0x2001 + acl_len 9 +
        // l2cap_len 5 + CID ATT + 04 0e00 ffff (5B ATT).
        if ble_canned
            && ble_canned_att_done
            && !ble_canned_findinfo_done
            && ble_canned_acl_tx_n >= 1
            && m.soc.bt_hci_rx_pending() == 0
        {
            m.soc.bt_hci_inject_rx(&[
                0x02, 0x01, 0x20, 0x09, 0x00, 0x05, 0x00, 0x04, 0x00, 0x04, 0x0E, 0x00, 0xFF, 0xFF,
            ]);
            ble_canned_findinfo_done = true;
            println!("[host] BLE canned FINDINFO staged");
        }
        // TEMP (2026-10-04, forensics — DELETE after): Phase 2 (handles in
        // flag docs). Read Battery Level (0x0A + 0x0010) after the Find-Info
        // response (2nd ACL TX). Full ACL: acl_len 7 + l2cap_len 3 + 0A 1000.
        if ble_canned
            && ble_canned_findinfo_done
            && !ble_canned_read_done
            && ble_canned_acl_tx_n >= 2
            && m.soc.bt_hci_rx_pending() == 0
        {
            m.soc.bt_hci_inject_rx(&[
                0x02, 0x01, 0x20, 0x07, 0x00, 0x03, 0x00, 0x04, 0x00, 0x0A, 0x10, 0x00,
            ]);
            ble_canned_read_done = true;
            println!("[host] BLE canned READ staged");
        }
        // Write echo `hi!` (0x12 + 0x0013 + 68 69 21) after the Read
        // response (3rd ACL TX). ATT len 6, l2cap 6, acl 10 (15B total).
        if ble_canned
            && ble_canned_read_done
            && !ble_canned_write_done
            && ble_canned_acl_tx_n >= 3
            && m.soc.bt_hci_rx_pending() == 0
        {
            m.soc.bt_hci_inject_rx(&[
                0x02, 0x01, 0x20, 0x0A, 0x00, 0x06, 0x00, 0x04, 0x00, 0x12, 0x13, 0x00, 0x68, 0x69,
                0x21,
            ]);
            ble_canned_write_done = true;
            println!("[host] BLE canned WRITE staged");
        }
        // Read-back echo value (0x0A + 0x0013) after the Write response
        // (4th ACL TX). Expects 0x0B + `hi!` (loopback via onWrite).
        if ble_canned
            && ble_canned_write_done
            && !ble_canned_readback_done
            && ble_canned_acl_tx_n >= 4
            && m.soc.bt_hci_rx_pending() == 0
        {
            m.soc.bt_hci_inject_rx(&[
                0x02, 0x01, 0x20, 0x07, 0x00, 0x03, 0x00, 0x04, 0x00, 0x0A, 0x13, 0x00,
            ]);
            ble_canned_readback_done = true;
            println!("[host] BLE canned READBACK staged");
        }
        if wifi_llmac {
            while let Some(frame) = m.soc.llmac_take_tx() {
                let fc = frame.first().copied().unwrap_or(0);
                let ssid = frame.windows(5).any(|w| w == b"EmuAP");
                println!(
                    "[host] LLMAC CAP len={} fc={:#04x} ssid={}",
                    frame.len(),
                    fc,
                    ssid as u8
                );
            }
            if m.soc.llmac_beacons_left() > 0
                && uart_buf
                    .windows(b"LLMAC sniff 1".len())
                    .any(|w| w == b"LLMAC sniff 1")
                // Scheduler-lock gate (same wild-`retw` class the BLE leg
                // hit: the sniffer runs thousands of insns over Arduino
                // heap/UART state — never synthesize it while a core sits
                // in a scheduler critical section. xKernelLock nm on the
                // llmac ELF; unlocked word is the 0xB33FFFFF mux magic,
                // same protocol. Level-triggered retry).
                && m.soc.read32(0x3fc9_5a80) == 0xB33F_FFFF
                && m.run_wifi_promisc_cb(0)
            {
                println!("[host] LLMAC RX beacon delivered");
            }
        }

        // UART1 RX injection: once the app prints the ready marker, push
        // the host payload into UART1 RX (the echo sketch reads it back).
        // Gated on new bytes: the buffer only changes when a drain above
        // appended, so scanning every step is wasted O(buffer) work.
        if !uart1_injected
            && (!tx.is_empty() || !tx1.is_empty())
            && let Some(bytes) = &uart1_inject
            && uart_buf.windows(b"RXREADY".len()).any(|w| w == b"RXREADY")
        {
            for &b in bytes {
                m.soc.uart_inject_rx(1, b);
            }
            println!(
                "[host] injected {:?} into UART1 RX",
                String::from_utf8_lossy(bytes)
            );
            uart1_injected = true;
        }

        // UART0 RX injection: same pattern with a configurable marker.
        if !uart0_injected
            && (!tx.is_empty() || !tx1.is_empty())
            && let Some(bytes) = &uart0_inject
            && uart_buf
                .windows(uart0_marker.len())
                .any(|w| w == uart0_marker.as_slice())
        {
            for &b in bytes {
                m.soc.uart_inject_rx(0, b);
            }
            println!(
                "[host] injected {:?} into UART0 RX",
                String::from_utf8_lossy(bytes)
            );
            uart0_injected = true;
        }

        // USB-CDC RX injection: same pattern with a configurable marker.
        if !usb_injected
            && (!tx.is_empty() || !tx1.is_empty())
            && let Some(bytes) = &usb_inject
            && uart_buf
                .windows(usb_marker.len())
                .any(|w| w == usb_marker.as_slice())
        {
            for &b in bytes {
                m.soc.usb_inject_rx(b);
            }
            println!(
                "[host] injected {:?} into USB CDC RX",
                String::from_utf8_lossy(bytes)
            );
            usb_injected = true;
        }

        // SPI-slave host exchange: marker-driven, one shot per half.
        if spi_slave_xchg && (!tx.is_empty() || !tx1.is_empty()) {
            if !spi_slave_wrote
                && uart_buf
                    .windows(b"SPI SLAVE READY".len())
                    .any(|w| w == b"SPI SLAVE READY")
            {
                m.soc.spi_slave_inject_write(0, &[0x11, 0x22, 0x33]);
                println!("[host] SPI slave master-write [11 22 33]");
                spi_slave_wrote = true;
            }
            if spi_slave_wrote
                && !spi_slave_read
                && uart_buf
                    .windows(b"SPI SLAVE TX-REQ".len())
                    .any(|w| w == b"SPI SLAVE TX-REQ")
            {
                let got = m.soc.spi_slave_take_read(0, 2);
                println!("[host] SPI slave master-read -> {got:02x?}");
                assert_eq!(got, vec![0xA5, 0xC3], "slave TX preload mismatch");
                spi_slave_read = true;
            }
            // Slave-DMA halves: the model routes through the GDMA links
            // when the sketch enables DMA_CONF rx/tx (same env flag).
            if !spi_slave_dma_wrote
                && uart_buf
                    .windows(b"SPI SLAVE DMA READY".len())
                    .any(|w| w == b"SPI SLAVE DMA READY")
            {
                m.soc.spi_slave_inject_write(0, &[0xDE, 0xAD, 0xBE, 0xEF]);
                println!("[host] SPI slave DMA master-write [de ad be ef]");
                spi_slave_dma_wrote = true;
            }
            if spi_slave_dma_wrote
                && !spi_slave_dma_read
                && uart_buf
                    .windows(b"SPI SLAVE DMA TX-REQ".len())
                    .any(|w| w == b"SPI SLAVE DMA TX-REQ")
            {
                let got = m.soc.spi_slave_take_read(0, 4);
                println!("[host] SPI slave DMA master-read -> {got:02x?}");
                assert_eq!(got, vec![0x12, 0x34, 0x56, 0x78], "slave DMA TX mismatch");
                spi_slave_dma_read = true;
            }
        }

        // I2C-slave host exchange: marker-driven, one shot per half.
        if i2c_slave_xchg && (!tx.is_empty() || !tx1.is_empty()) {
            if !i2c_slave_wrote
                && uart_buf
                    .windows(b"I2C SLAVE READY".len())
                    .any(|w| w == b"I2C SLAVE READY")
            {
                m.soc.i2c_slave_inject_write(0, 0x42, &[0x11, 0x22]);
                println!("[host] I2C slave master-write [11 22]");
                i2c_slave_wrote = true;
            }
            if i2c_slave_wrote
                && !i2c_slave_read
                && uart_buf
                    .windows(b"I2C SLAVE TX-REQ".len())
                    .any(|w| w == b"I2C SLAVE TX-REQ")
            {
                let got = m.soc.i2c_slave_take_read(0, 0x42, 1);
                println!("[host] I2C slave master-read -> {got:02x?}");
                assert_eq!(got, vec![0xA5], "slave TX preload mismatch");
                i2c_slave_read = true;
            }
        }

        // USB-OTG device auto-enumeration: marker-driven, strictly one
        // host action per step (the DWC2 core is single-transaction: each
        // SETUP must be consumed — GRXSTSP pops + DOEPINT0 W1C — before
        // the next lands, or packets coalesce). Gated on new UART bytes
        // like the other injections (the buffer only changes on drain).
        //
        // LIVE path (what the sketch actually drives): bus-reset on
        // "USB DEVICE STACK UP", then one GET_DESCRIPTOR(device, 8)
        // SETUP on "USB DEV REQ0". The REAL TinyUSB ISR consumes the
        // transfer (RXFLVL/GRXSTSP + STPKTRCVD, then the control handler
        // pushes the IN descriptor payload through the slave TXFIFO
        // path); the sketch only waits for the completed IN stage and
        // checks the bytes, while the harness asserts the same capture
        // off the virtual wire via `usb_host_take_in`. (A full
        // 7-transfer enumeration is staged below as documentation of the
        // intended end state — the sketch does not print REQ1..REQ6 yet,
        // so the harness stops after REQ0 by construction.)
        //
        // RENDEZVOUS (read before restructuring! measured live 2026-09-16,
        // single-step ground truth): the sketch prints POLL RDV, delays
        // delay(20) (millions of steps, outlasting stage-to-complete ≈ 2.4k
        // steps by orders of magnitude), then reads the latched IN mirror.
        // The harness fires the SETUP on sight of the POLL bytes (~1 drain
        // later); the ISR schedules TSIZ=8 at ~+2.3k single-steps and the
        // transfer drains at ~+2.4k, all inside the preempting ISR — a
        // task-side TSIZ poll can NEVER observe it (TSIZ=8 lives ~107
        // steps while the sketch task is preempted; it resumes polling at
        // ~+8.9k with TSIZ already 0). So the sketch does NOT poll TSIZ at
        // all (see check_in8): delay-then-mirror-read is race-free by
        // construction. Staging the SETUP any earlier is equally fine for
        // the mirror (it latches until read) — but keep it strictly on the
        // POLL marker anyway: staging on STACK UP would complete the
        // transfer before the sketch's delay even starts, which is harmless
        // today yet needlessly couples harness order to sketch timing.
        //
        // MATRIX ROUTING the harness owns: the TinyUSB device ISR is
        // allocated on core 1 (`dwc2_int_set` → `esp_intr_alloc(38)` binds
        // the core-1 CPU that calls `tud_task`), but the emulator's
        // interrupt matrix resets every source to line 6 (unmapped) and
        // the ROM/firmware `intr_matrix_set` only routes what the IDF
        // allocator programs — the DWC2 path programs the core-0 entry
        // (probed live: map38 = 6/3 at STACK UP, i.e. core-0 unmapped,
        // core-1 on line 3) yet the bus-reset bark never reaches the
        // core-1 line (GINTSTS stays 0x3000, ISR never runs). Routing
        // source 38 to the same line on core 0 here (like every other
        // peripheral sketch's matrix setup) delivers the bark the ISR
        // polls — silicon-equivalent, since the ISR body is core-agnostic
        // (it only reads GINTSTS/GRXSTSP/DFIFO and queues events).
        if usb_host_enum && (!tx.is_empty() || !tx1.is_empty()) {
            // Full enumeration against the firmware's own descriptors:
            // reset, GET_DESCRIPTOR device (8 + 18), SET_ADDRESS(7),
            // GET_DESCRIPTOR device@7 (18), GET_DESCRIPTOR config (9 +
            // 32), SET_CONFIGURATION(1). wLength-8/9 first reads are what
            // the TinyUSB stack actually issues; the sketch prints
            // "USB DEV REQ<n>" before staging each transfer. LIVE: only
            // REQ0 is wired (the sketch's only live transfer); REQ1..REQ6
            // below are dead entries documenting the intended full
            // sequence — unreachable until the sketch prints them.
            const ENUM: &[(&[u8], &[u8])] = &[
                (b"USB DEVICE STACK UP", &[]),
                (
                    b"USB DEVICE POLL RDV",
                    &[0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x08, 0x00],
                ),
                (
                    b"USB DEVICE REQ1",
                    &[0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00],
                ),
                (
                    b"USB DEVICE REQ2",
                    &[0x00, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00],
                ),
                (
                    b"USB DEVICE REQ3",
                    &[0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00],
                ),
                (
                    b"USB DEVICE REQ4",
                    &[0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x09, 0x00],
                ),
                (
                    b"USB DEVICE REQ5",
                    &[0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x20, 0x00],
                ),
                (
                    b"USB DEVICE REQ6",
                    &[0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
                ),
            ];
            if usb_enum_step < ENUM.len()
                && uart_buf
                    .windows(ENUM[usb_enum_step].0.len())
                    .any(|w| w == ENUM[usb_enum_step].0)
            {
                let (_, setup) = ENUM[usb_enum_step];
                if setup.is_empty() {
                    // Route the DWC2 bark to a CPU line the firmware
                    // polls (see the MATRIX ROUTING note above), then
                    // raise bus-reset + enumdone like silicon on connect.
                    m.soc.write32(0x600C_2000 + 4 * 38, 3);
                    m.soc.usb_host_bus_reset();
                    println!("[host] USB auto-enum: bus reset + enumdone");
                } else {
                    // Complete the previous transfer's status stage first
                    // (except REQ0, which follows the reset with no data
                    // stage pending): silicon completes each control
                    // transfer before the next SETUP lands.
                    if usb_enum_step > 1 {
                        m.soc.usb_host_status_out();
                    }
                    let mut pkt = [0u8; 8];
                    pkt.copy_from_slice(setup);
                    m.soc.usb_host_setup(pkt);
                    println!("[host] USB auto-enum: SETUP {pkt:02x?}");
                }
                usb_enum_step += 1;
            }
            // Per-transfer status close: the sketch prints "<TAG> OK" after
            // it verified that transfer's IN bytes sketch-side; the harness
            // then closes the control transfer the way silicon would
            // (status OUT → both XFRC flags, session cleared) so the NEXT
            // staged SETUP starts from a clean single-transaction state.
            // Each close is guarded by its own "[status-out-N]" tag appended
            // below so it fires exactly once (the marker stays in uart_buf).
            // NOTE: the tag bytes are appended to uart_buf ONLY (never
            // printed) so they cannot collide with real firmware output —
            // no sketch prints "[status-out".
            const CLOSES: &[(&[u8], &[u8])] = &[
                (b"USB DEVICE ENUM OK", b"[status-out-0]"),
                (b"USB DEVICE REQ1 OK", b"[status-out-1]"),
                (b"USB DEVICE REQ2 OK", b"[status-out-2]"),
                (b"USB DEVICE REQ3 OK", b"[status-out-3]"),
                (b"USB DEVICE REQ4 OK", b"[status-out-4]"),
                (b"USB DEVICE REQ5 OK", b"[status-out-5]"),
                (b"USB DEVICE REQ6 OK", b"[status-out-6]"),
            ];
            for (marker, tag) in CLOSES {
                if !uart_buf.windows(tag.len()).any(|w| w == *tag)
                    && uart_buf.windows(marker.len()).any(|w| w == *marker)
                {
                    m.soc.usb_host_status_out();
                    println!(
                        "[host] USB auto-enum: status OUT closed ({})",
                        String::from_utf8_lossy(marker)
                    );
                    // Only once (marker stays in uart_buf forever).
                    uart_buf.extend_from_slice(tag);
                }
            }
        }

        // --- Stall / idle detection ---
        // (Skipped while fast-forwarding deep sleep: the CPUs are halted by
        // design for the whole sleep, however long the ULP program runs.)
        if m.is_asleep() {
            idle_steps = 0;
            stuck = 0;
        }
        // WiFi scan completion: arm the dwell when the firmware enters
        // `esp_wifi_scan_start` (either core); when the dwell elapses, stage
        // the fixture records (if any) and post the REAL SCAN_DONE esp_event
        // so the full `_scanDone` record path runs unmodified. Level, not
        // edge: the arm fires once per scan; the completion retries until
        // the queue/group handles are valid, then latches done.
        if wifi_scan_fixture && !wifi_scan_done {
            let layout = if path.contains("coex") {
                &COEX_LAYOUT
            } else if path.contains("test_worker_l3") {
                &WORKER_L3_LAYOUT
            } else if path.contains("test_worker_net") {
                &WORKER_LAYOUT
            } else if bin_name_contains_wifi_sta {
                &STA_LAYOUT
            } else {
                &SCAN_LAYOUT
            };
            if !wifi_scan_armed
                && (m.cpu[0].pc == layout.scan_start || m.cpu[1].pc == layout.scan_start)
            {
                m.soc.wifi_scan_begin();
                wifi_scan_armed = true;
                println!("[host] WiFi scan dwell armed");
            }
            if wifi_scan_armed && m.soc.wifi_scan_tick_complete() {
                // Count cell first, so the record path the event triggers
                // already sees the store. Empty air (no WIFI_SCAN_APS)
                // stages nothing: count cell stays 0. (The BSS queue is
                // NOT staged: with no RF stimulus the closed scan machine
                // never enqueues nodes; the records themselves are written
                // at the records check below.)
                if !wifi_scan_aps.is_empty() {
                    m.soc
                        .write16(layout.count_cell, wifi_scan_aps.len().min(8) as u32);
                    println!(
                        "[host] WiFi scan staged count {}",
                        wifi_scan_aps.len().min(8)
                    );
                }
                match m.soc.wifi_scan_post_event(layout.wifi_event_var) {
                    Some(tcb) => {
                        m.soc
                            .ready_task_on_list(tcb, layout.ready_lists, layout.top_prio);
                        println!("[host] WiFi SCAN_DONE posted (woke sys_evt {tcb:#x})");
                    }
                    None => println!("[host] WiFi SCAN_DONE post failed (no sys_evt yet)"),
                }
                wifi_scan_done = true;
            }
        }
        // WiFi scan records: at the records-return check the calloc'd
        // buffer is final — write the fixture records + force the verdict
        // the closed copy loop would leave with live BSS nodes (ESP_OK +
        // count). Either core may run `_scanDone`; exactly once per scan.
        if wifi_scan_fixture
            && wifi_scan_done
            && !wifi_scan_records_done
            && !wifi_scan_aps.is_empty()
        {
            let layout = if path.contains("coex") {
                &COEX_LAYOUT
            } else if path.contains("test_worker_l3") {
                &WORKER_L3_LAYOUT
            } else if path.contains("test_worker_net") {
                &WORKER_LAYOUT
            } else if bin_name_contains_wifi_sta {
                &STA_LAYOUT
            } else {
                &SCAN_LAYOUT
            };
            for c in 0..2 {
                if m.cpu[c].pc == layout.records_check {
                    let buf = m.soc.read32(layout.scan_result);
                    if buf != 0 {
                        let n = wifi_scan_aps.len().min(8);
                        for (k, ap) in wifi_scan_aps.iter().take(n).enumerate() {
                            m.soc.wifi_scan_record_ap(buf, k, ap);
                        }
                        m.cpu[c].set_reg(10, 0); // a10 = ESP_OK
                        m.soc.write16(layout.count_cell, n as u32);
                        m.soc.write16(layout.scan_count, n as u32);
                        println!("[host] WiFi scan recorded {n} AP(s) on core{c}");
                        wifi_scan_records_done = true;
                    }
                }
            }
        }
        // WiFi STA connect (wifi-sta sketch, WIFI_STA_CONN=1): arm the dwell
        // when the firmware enters `esp_wifi_connect` (either core); when it
        // elapses, post the two halves of the association:
        // Stage 1: WIFI_EVENT_STA_CONNECTED (id 4) with the
        // `wifi_event_sta_connected_t` payload (ssid/bssid/chan/authmode
        // from the first WIFI_SCAN_APS fixture, or EmuNet defaults) on the
        // IDF bus; the firmware's own `_onStaEvent` → `postEvent` translates
        // it into ARDUINO_EVENT_WIFI_STA_CONNECTED (112) on the arduino bus
        // (a host arduino post RACES that translation and wedges the queue —
        // proven live: pending storm + use-after-free `_ZdlPvj` of the host
        // event — so stage 1 posts IDF-only).
        // Stage 2 (once stage 1 is consumed): IP_EVENT_STA_GOT_IP (id 0)
        // with the `ip_event_got_ip_t` payload (netif pointer read live
        // from the STA `_esp_netif` cell, fixed 192.168.4.2/24/gw .1) on the
        // IDF bus (handler-veracity; the emulator-side `_ip_event_cb` path is
        // dormant — no IDF tcpip registration exists) PLUS a host post of
        // ARDUINO_EVENT_WIFI_STA_GOT_IP (115) with the full 20-byte
        // `ip_event_got_ip_t` at the info-union head on the arduino bus (the
        // real `_onStaArduinoEvent` → `_setStatus(WL_CONNECTED)` chain then
        // runs unmodified and `waitForConnectResult` returns WL_CONNECTED).
        // Level, not edge: each post retries until the queue accepts it,
        // then latches done.
        // Arduino event IDs (`NetworkEvents.h`): WIFI_STA_CONNECTED=112,
        // WIFI_STA_GOT_IP=115. (An earlier revision used 114/117 — the enum
        // slots for STA_STOP/AUTHMODE_CHANGE — and the cb silently dropped
        // the events; the enum header is ground truth.) The 44-byte info
        // unions: connected carries
        // ssid[32]@0 + ssid_len@32 + bssid[6]@33 + channel@39 (matching the
        // IDF struct head, which is all `_onStaArduinoEvent` inspects
        // before setting status); got_ip carries the 20-byte
        // `ip_event_got_ip_t` (netif@0/ip@4/mask@8/gw@12/changed@16 —
        // a flat ip/mask/gw@0 layout corrupts the sized-delete and panics,
        // proven live at 0x4037bf00).
        if wifi_sta_conn && !wifi_sta_done {
            let layout = if path.contains("coex") {
                &COEX_LAYOUT
            } else if path.contains("test_worker_l3") {
                &WORKER_L3_LAYOUT
            } else if path.contains("test_worker_net") {
                &WORKER_LAYOUT
            } else if bin_name_contains_wifi_sta {
                &STA_LAYOUT
            } else {
                &SCAN_LAYOUT
            };
            if !wifi_sta_armed && (m.cpu[0].pc == layout.connect || m.cpu[1].pc == layout.connect) {
                m.soc.wifi_scan_begin();
                wifi_sta_armed = true;
                println!("[host] WiFi STA connect dwell armed");
            }
            // Level, not edge: the dwell fires once, but each post below
            // retries every step until the queue/sys_evt accepts it.
            // Single-shot per stage (NOT every step): the IDF post mutates
            // the queue (mw+unlink+ready), so retrying it after a partial
            // success double-posts and wedges the queue at len (the
            // pending-storm class). Each stage below therefore attempts its
            // posts, and latches done as soon as the IDF half lands; the
            // arduino half is driven by the IDF chain itself (see below).
            if wifi_sta_armed
                && (m.soc.wifi_scan_tick_complete() || wifi_sta_armed)
                && wifi_sta_conn_stage == 0
            {
                // Stage 1: STA_CONNECTED with the fixture payload.
                let ap = wifi_scan_aps.first().copied().unwrap_or(
                    esp32s3_soc::wifi::parse_scan_fixture("EmuNet,-50,6,02:11:22:33:44:55")
                        .expect("default fixture parses"),
                );
                // Full `wifi_event_sta_connected_t` (48B, ground truth =
                // arduino-lib 3.3.10 `esp_wifi_types.h` + `_onStaEvent`'s
                // memcpy into the arduino event): ssid[32]@0, ssid_len@32,
                // bssid[6]@33, channel@39, authmode u32@40 (=3 WPA2_PSK),
                // aid u16@44 (=1). An earlier 42-byte-short revision left
                // ssid_len/bssid/channel/auth all zero, so the firmware's
                // own translated postEvent carried zeros and the sketch
                // could never match the fixture (proven live by the
                // all-zeros arduino event + PRE-DELETE dump).
                let mut payload = [0u8; 48];
                let n = (ap.ssid_len as usize).min(32);
                payload[..n].copy_from_slice(&ap.ssid[..n]);
                payload[32] = ap.ssid_len;
                payload[33..39].copy_from_slice(&ap.bssid);
                payload[39] = ap.chan;
                payload[40..44].copy_from_slice(&3u32.to_le_bytes());
                payload[44..46].copy_from_slice(&1u16.to_le_bytes());
                let mut info = [0u8; 44];
                info[..n].copy_from_slice(&ap.ssid[..n]);
                info[32] = ap.ssid_len;
                info[33..39].copy_from_slice(&ap.bssid);
                info[39] = ap.chan;
                let idf_ok = match m.soc.wifi_post_event_with_data(
                    layout.wifi_event_var,
                    4, // WIFI_EVENT_STA_CONNECTED
                    &payload,
                ) {
                    Some(tcb) => {
                        m.soc
                            .ready_task_on_list(tcb, layout.ready_lists, layout.top_prio);
                        true
                    }
                    None => false,
                };
                // Arduino half: NO host post. The IDF chain (`_arduino_event_cb`
                // → `_onStaEvent` → `postEvent`) delivers the translated
                // event into the arduino queue itself (proven live: ard mw
                // 0→1 right after the IDF cb runs). A host ard post would
                // race the firmware's own `postEvent` for the same waiter
                // and wedge the queue (proven live: pending storm +
                // use-after-free `delete` of the host event).
                let _ = info;
                if idf_ok {
                    println!(
                        "[host] WiFi STA_CONNECTED posted (IDF bus; arduino via firmware postEvent)"
                    );
                    wifi_sta_conn_stage = 1;
                } else {
                    println!("[host] WiFi STA_CONNECTED post pending (idf={idf_ok})");
                }
                if wifi_sta_conn_stage == 1 {
                    wifi_sta_done = true;
                }
            }
        }
        // WiFi STA stage 2: GOT_IP on BOTH buses, posted BACK-TO-BACK with
        // stage 1 (same step, no consume-gate between them). The arduino
        // half is a HOST post (not the firmware's own `_onIpEvent` →
        // `postEvent`): that chain routes IDP `IP_EVENT` through the
        // IDF-registered `_ip_event_cb` → `getNetifByEspNetif(netif)` lookup,
        // which needs the IDF tcpip registration the emulator never runs
        // (closed DHCP/client stack — no host counterparty). Posting the IDF
        // half alone leaves `status()` at WL_IDLE_STATUS forever (proven live:
        // GOT_IP idf=true consumed, zero `_setStatus(WL_CONNECTED)`,
        // `waitForConnectResult` times out). So the host ALSO posts
        // ARDUINO_EVENT_WIFI_STA_GOT_IP (115) with the 20-byte got_ip info
        // directly into the arduino queue via `wifi_ard_post`; the real
        // `_onStaArduinoEvent` → `_setStatus(WL_CONNECTED)` chain then runs
        // unmodified and the sketch's `localIP()` reads the same fixture IP
        // from the info union it prints (`WIFI STA IP 192.168.4.2`).
        // NO consume-gate between stage 1 and stage 2 (both halves post
        // while sys_evt/arduino waiters are parked at dwell time): gating
        // stage 2 on the CONNECTED consume lets sys_evt drain and UNPARK
        // (its loop exits when the queue empties), after which the waiter
        // never re-parks and every GOT_IP post fails forever (proven live:
        // sys_q=0x0 at all 331 GOT_IP attempts, WL_CONNECTED never
        // arrives). The netif pointer is read live from the STA `_esp_netif`
        // cell (discovered per-image; kept in the payload for
        // handler-veracity even though the emulator-side `_ip_event_cb`
        // path is dormant). The arduino half still gates on
        // `wifi_ard_idle` (mw back to 0) — the two arduino events must not
        // race in the same queue — but the IDF half posts immediately.
        if wifi_sta_conn && wifi_sta_conn_stage == 1 && !wifi_sta_ip_done {
            let layout = if path.contains("coex") {
                &COEX_LAYOUT
            } else if path.contains("test_worker_l3") {
                &WORKER_L3_LAYOUT
            } else if path.contains("test_worker_net") {
                &WORKER_LAYOUT
            } else if bin_name_contains_wifi_sta {
                &STA_LAYOUT
            } else {
                &SCAN_LAYOUT
            };
            // Fixed fixture LAN: 192.168.4.2/24, gw 192.168.4.1 (matches
            // the SoftAP default subnet family; documented in the sketch).
            let ip = [192u8, 168, 4, 2];
            let mask = [255u8, 255, 255, 0];
            let gw = [192u8, 168, 4, 1];
            let netif = m.soc.wifi_sta_netif();
            // Never post GOT_IP with a null netif: `_ip_event_cb` would
            // `getNetifByEspNetif(NULL)` → NULL → drop the event while the
            // host latches done and WL_CONNECTED never arrives (retry next
            // step instead — level, not edge).
            if netif == 0 {
                if i.is_multiple_of(100_000) {
                    println!("[host] WiFi STA GOT_IP post pending (netif undiscovered)");
                }
                continue;
            }
            let mut payload = [0u8; 20];
            payload[0..4].copy_from_slice(&netif.to_le_bytes());
            payload[4..8].copy_from_slice(&u32::from_le_bytes(ip).to_le_bytes());
            payload[8..12].copy_from_slice(&u32::from_le_bytes(mask).to_le_bytes());
            payload[12..16].copy_from_slice(&u32::from_le_bytes(gw).to_le_bytes());
            payload[16] = 1; // ip_changed = true (first assignment)
            // Arduino half: HOST post of ARDUINO_EVENT_WIFI_STA_GOT_IP
            // (115) with the full `ip_event_got_ip_t` (20B) at the head
            // of the 44-byte info union (`arduino_event_t.event_info.got_ip`,
            // `NetworkEvents.h`): esp_netif@0 (kept for handler-veracity),
            // ip@4/mask@8/gw@12, ip_changed@16. A flat ip/mask/gw@0 layout
            // is WRONG — the 20-byte struct (not 12) is what `_onIpEvent`
            // memcpys into the arduino event (proven live: flat layout's
            // info read `c0 a8 04 02` as the netif pointer → sized-delete
            // of a low bogus address → IN-PANIC at 0x4037bf00).
            // `wifi_ard_post` fails softly while the waiter is unparked
            // (returns false — retry next step, level not edge).
            let mut info = [0u8; 44];
            info[0..20].copy_from_slice(&payload);
            let idf_ok = match m.soc.wifi_post_event_with_data(
                layout.ip_event_var,
                0, // IP_EVENT_STA_GOT_IP
                &payload,
            ) {
                Some(tcb) => {
                    m.soc
                        .ready_task_on_list(tcb, layout.ready_lists, layout.top_prio);
                    true
                }
                None => false,
            };
            let ard_ok = m
                .soc
                .wifi_ard_post(115, &info, layout.ready_lists, layout.top_prio);
            // Decouple: WL_CONNECTED gates on the ARDUINO half alone (the
            // IDF half is handler-veracity only — the emulator-side
            // `_ip_event_cb` path is dormant, and sys_evt may never re-park
            // after the CONNECTED consume). Latch done + stage the insider
            // reads as soon as the ard post lands; a failed IDF half is
            // retried best-effort (level, not edge) without blocking that.
            if ard_ok && !wifi_sta_ip_done {
                // Stage the insider reads the sketch performs AFTER
                // WL_CONNECTED: `localIP()` ← `esp_netif_get_ip_info`
                // (ip/mask/gw) and `SSID()`/`RSSI()` ←
                // `esp_wifi_sta_get_ap_info` (92-byte record) — both served
                // by the pc-intercept hooks below from this same fixture
                // AP + LAN (single source of truth).
                let ap = wifi_scan_aps.first().copied().unwrap_or(
                    esp32s3_soc::wifi::parse_scan_fixture("EmuNet,-50,6,02:11:22:33:44:55")
                        .expect("default fixture parses"),
                );
                m.soc.wifi_stage_sta_data(&ap, ip);
                println!(
                    "[host] WiFi STA GOT_IP posted (netif {netif:#x}, idf={idf_ok} ard={ard_ok})"
                );
                wifi_sta_ip_done = true;
            } else if !wifi_sta_ip_done {
                println!("[host] WiFi STA GOT_IP post pending (idf={idf_ok} ard={ard_ok})");
            }
        }
        // WiFi STA disconnect leg: after the sketch calls `WiFi.disconnect()`
        // (the `esp_wifi_disconnect` hook above reports ESP_OK), post the
        // real status-change events so `status()` leaves WL_CONNECTED:
        // WIFI_EVENT_STA_DISCONNECTED (id 5) with the
        // `wifi_event_sta_disconnected_t` payload (ssid/bssid/reason) on the
        // IDF bus + ARDUINO_EVENT_WIFI_STA_DISCONNECTED (113) into the
        // arduino queue. The real chains (`_onStaEvent` → clear CONNECTED
        // bits → `postEvent` → `_onStaArduinoEvent` → `_setStatus`) then run
        // unmodified and the sketch prints `after-disconnect` + DONE.
        // Armed by the disconnect hook (one-shot); each half retries until
        // its queue accepts it, then latches done (level, not edge).
        // Arm the disconnect leg from the machine hook latch (fires when
        // the sketch calls `WiFi.disconnect()` → `esp_wifi_disconnect`).
        // Gate on GOT_IP done AND the RSSI leg consumed (the staged reads
        // prove the sketch reached the post-WL_CONNECTED prints): the
        // wifi-sta sketch ALSO calls `WiFi.disconnect()` in setup() before
        // `begin()` (unstaged hook lets it run, but the latch still fires)
        // — only the post-RSSI disconnect arms the status-change leg.
        // The worker image must NOT arm this leg at all (proven live
        // 2026-09-28: its setup() never disconnects, but the closed
        // `esp_wifi_connect` path calls `esp_wifi_disconnect` internally
        // on retry — arming posts DISCONNECTED, the firmware tears the
        // association down, and the run parks at `esp_wifi_connect`
        // forever; the worker sketch never calls `WiFi.disconnect()`
        // itself, so nothing is lost by skipping).
        let needs_disc_leg = !path.contains("test_worker_net") && !path.contains("test_worker_l3");
        if wifi_sta_conn
            && needs_disc_leg
            && wifi_sta_ip_done
            && m.soc.wifi_hook_rssi_done()
            && m.soc.wifi_take_disconnect()
        {
            wifi_sta_disc_armed = true;
            println!("[host] WiFi STA disconnect hook fired");
        } else {
            // Drain a pre-leg latch fire so it can't arm the leg late.
            let _ = m.soc.wifi_take_disconnect();
        }
        if wifi_sta_conn
            && wifi_sta_ip_done
            && wifi_sta_disc_armed
            && !wifi_sta_disc_done
            && m.soc.wifi_ard_idle()
        {
            let layout = if path.contains("coex") {
                &COEX_LAYOUT
            } else if path.contains("test_worker_l3") {
                &WORKER_L3_LAYOUT
            } else if path.contains("test_worker_net") {
                &WORKER_LAYOUT
            } else if bin_name_contains_wifi_sta {
                &STA_LAYOUT
            } else {
                &SCAN_LAYOUT
            };
            let ap = wifi_scan_aps.first().copied().unwrap_or(
                esp32s3_soc::wifi::parse_scan_fixture("EmuNet,-50,6,02:11:22:33:44:55")
                    .expect("default fixture parses"),
            );
            // `wifi_event_sta_disconnected_t` (28B, `esp_wifi_types.h`):
            // ssid[32]@0... actually ssid[32]+bssid[6]+reason u8 — pack
            // ssid/bssid/reason; the handler only reads reason for the
            // status verdict (+ ASSOC_LEAVE = no reconnect).
            let mut dpayload = [0u8; 28];
            let n = (ap.ssid_len as usize).min(32);
            // Full struct is ssid[32]@0, ssid_len@32, bssid[6]@33,
            // reason@39 — same head as the connected event.
            let mut dpay_full = [0u8; 48];
            dpay_full[..n].copy_from_slice(&ap.ssid[..n]);
            dpay_full[32] = ap.ssid_len;
            dpay_full[33..39].copy_from_slice(&ap.bssid);
            dpay_full[39] = 8; // WIFI_REASON_ASSOC_LEAVE (voluntary — no reconnect)
            dpayload.copy_from_slice(&dpay_full[..28]);
            let didf_ok = match m.soc.wifi_post_event_with_data(
                layout.wifi_event_var,
                5, // WIFI_EVENT_STA_DISCONNECTED
                &dpay_full,
            ) {
                Some(tcb) => {
                    m.soc
                        .ready_task_on_list(tcb, layout.ready_lists, layout.top_prio);
                    true
                }
                None => false,
            };
            // Arduino 113 info = `wifi_sta_disconnected` (ssid/bssid/reason
            // at the union head).
            let mut dinfo = [0u8; 44];
            dinfo[..n].copy_from_slice(&ap.ssid[..n]);
            dinfo[32] = ap.ssid_len;
            dinfo[33..39].copy_from_slice(&ap.bssid);
            dinfo[39] = 8;
            let dard_ok = m
                .soc
                .wifi_ard_post(113, &dinfo, layout.ready_lists, layout.top_prio);
            if didf_ok && dard_ok {
                println!("[host] WiFi STA DISCONNECTED posted (IDF + arduino bus)");
                wifi_sta_disc_done = true;
            } else if !wifi_sta_disc_done {
                println!("[host] WiFi STA DISCONNECT post pending (idf={didf_ok} ard={dard_ok})");
            }
        }
        // WiFi SoftAP fixture (wifi-ap sketch, WIFI_AP_FIXTURE=1): the
        // firmware posts the IDF AP_START itself (`_ap_event_cb` fires at
        // the `esp_wifi_start` return path — proven live: id=12/data=0 at
        // step ~23M with NO host post); the host only (a) stages the AP
        // fixture data (config + 192.168.4.1/24 LAN) the hooks serve back,
        // and (b) posts the arduino ARDUINO_EVENT_WIFI_AP_START (130)
        // — NO wait, see below. The real chains (`_onApEvent` →
        // setStatusBits(STARTED) → `postEvent` → arduino_events →
        // `_onApArduinoEvent`) then run unmodified and `softAP()`'s
        // `waitStatusBits(STARTED)` returns true.
        //
        // NO HOST IDF POST: the firmware's own id=12 post already drives
        // the full chain (proven: set(bits=12) via `_onApEvent` at step
        // ~22.98M, begin-resume ret=1). A duplicate host id=12 post would
        // double-start the netif (second `netif_add` on the same netif
        // aborts in "netif already added" — proven live). NO HOST ARD
        // POST either: the IDF chain's own `Network.postEvent` allocates
        // a heap-valid event; a host ard post points into `ard_pool` and
        // aborts in `heap_caps_free` when consumed (proven live).
        // WLAN event IDs from `esp_wifi_types_generic.h`: AP_START=12.
        if wifi_ap_fixture && !wifi_ap_done {
            // NOTE: the stage pc is checked on CORE 0 ONLY. Core 1 runs
            // `esp_wifi_start` itself at boot (step ~5.5M, proven live:
            // pc1==esp_wifi_start while core1 still owns the wifi task);
            // staging there would arm mid-bring-up and the very next
            // step_fast block on core1 vectors through the kernel
            // `_UserExceptionVector` into the ROM hole (UNIMPLEMENTED
            // trap, proven live). Core 0 reaches the same pc later, once
            // the wifi task migrated and the bring-up is settled.
            if !wifi_ap_armed && m.cpu[0].pc == 0x4206_3870 {
                // SSID/passphrase/channel from the fixture (defaults match
                // the sketch): the staged config is what `softAPSSID()`
                // reads back via `esp_wifi_get_config`.
                let (ssid, pass, chan) = ("EmuAP", "password", 6);
                m.soc
                    .wifi_stage_ap_data(ssid.as_bytes(), pass.as_bytes(), chan);
                wifi_ap_armed = true;
                println!("[host] WiFi AP fixture staged (firmware posts AP_START itself)");
            }
            // Completion: the sketch prints DONE when its own chain ran
            // (softAP 1 + IP + SSID + stations + clients). The host only
            // staged the data; the DONE marker arriving latches the leg
            // (level, not edge — the marker stays in uart_buf).
            if wifi_ap_armed
                && uart_buf
                    .windows(b"WIFI AP DONE".len())
                    .any(|w| w == b"WIFI AP DONE")
            {
                println!("[host] WiFi AP DONE observed");
                wifi_ap_done = true;
            }
        }
        // WiFi ESP-NOW loopback (espnow sketch, WIFI_ESPNOW_LOOPBACK=1):
        // the host is the virtual second node. The closed `libespnow.a`
        // send path delivers TX-complete/RX frames by DIRECT C calls
        // (no FreeRTOS queue to post to — proven live: zero DRAM
        // transitions across `esp_now_send`, firmware parks in
        // `delay()`), so the host invokes the registered wrappers IN
        // FIRMWARE via `run_espnow_callback` (windowed-ABI call frame
        // synthesis — see machine.rs):
        // - TX leg: once the sketch's `handler.send()` returned ESP_OK
        //   (the closed send runs fully in-emulator and returns 0 —
        //   proven live at the caller resume), invoke the registered
        //   TX wrapper (closed .bss cell) with the peer mac → dispatches
        //   to the peer's `onSent(true)` → `sent_ok = true`.
        // - RX leg: after TX, invoke the registered RX wrapper with a
        //   2-byte frame from the peer mac → dispatches to the peer's
        //   `onReceive` → `got_rx = true`, `rx_byte0 = 0xA5`.
        // Level, not edge: each leg runs once (latched). The sketch's
        // own `delay(50)` poll loop observes the flags and prints.
        if wifi_espnow_loopback && (!wifi_espnow_tx_done || !wifi_espnow_rx_done) {
            // Closed cb cells live in closed .bss (espnow image, nm-proof
            // is impossible — discovered live: RX @0x3fc9dd28, TX
            // @0x3fc9dd2c hold the Arduino wrapper entries once
            // `ESP_NOW.begin()` registers them; the peer object sits at
            // the Arduino `_esp_now_peers[0]` cell).
            //
            // TIMING RULE (proven live — stack-smash otherwise): the TX
            // leg fires ONLY after the sketch's `handler.send()` RETURNED
            // (the `WIFI ESPNOW sent 1` marker). At the cells-live moment
            // the sketch is still INSIDE `ESP_NOW_Peer::add()` (core1 pc
            // in add(), the stack-canary frame live); invoking the TX
            // wrapper then runs a nested call on the same core while
            // add()'s canary slot is live, and add()'s epilogue check
            // (`__stack_chk_fail` at +0x55) fires ~2k steps later. The
            // `sent 1` marker proves `esp_now_send` returned ESP_OK *and*
            // the sketch left add() (markers print between the calls).
            const ESPNOW_RX_CELL: u32 = 0x3fc9_dd28;
            const ESPNOW_TX_CELL: u32 = 0x3fc9_dd2c;
            const ESPNOW_PEERS: u32 = 0x3fc9_aeac;
            const ESPNOW_PEER_OBJ: u32 = 0x3fc9_ae60;
            let espnow_send_done = uart_buf
                .windows(b"WIFI ESPNOW sent 1".len())
                .any(|w| w == b"WIFI ESPNOW sent 1");
            // NOTE: no `continue` while waiting for the `sent 1`
            // marker — the UART drain + idle accounting below must run
            // every macro-step (a `continue` skips the drain; proven
            // live: bytes sit in the SoC FIFOs while `uart_buf` looks
            // unchanged → false IDLE/STUCK trips + swallowed markers).
            // Guard the TX leg on the marker instead (the RX leg is
            // already ordered behind the TX latch).
            if !wifi_espnow_tx_done && !espnow_send_done {
            } else if !wifi_espnow_tx_done {
                let txcb = m.soc.read32(ESPNOW_TX_CELL);
                let peer = m.soc.read32(ESPNOW_PEERS);
                if txcb != 0 && peer != 0 {
                    // Peer mac from the Arduino peer object (+4/+8, the
                    // mac bytes — proven live: 02:11:22:33:44:55).
                    let m0 = m.soc.read32(ESPNOW_PEER_OBJ + 4);
                    let m1 = m.soc.read32(ESPNOW_PEER_OBJ + 8);
                    let mut mac = [0u8; 6];
                    mac[..4].copy_from_slice(&m0.to_le_bytes());
                    mac[4..].copy_from_slice(&m1.to_le_bytes()[..2]);
                    m.soc.wifi_espnow_invoke_tx_cb(txcb, &mac);
                    // Run on core 1 (parks in IDLE — always safe).
                    if m.run_espnow_callback(1) {
                        println!("[host] WiFi ESP-NOW TX-cb invoked (sent_ok)");
                        wifi_espnow_tx_done = true;
                    }
                }
            } else if !wifi_espnow_rx_done {
                let rxcb = m.soc.read32(ESPNOW_RX_CELL);
                // Direct vtable dispatch (slot 2 = onReceive): the closed
                // wrapper's memcmp peer gate would need a known-peer mac;
                // the vtable call carries (this, data, len, bcast) with no
                // gate (proven live: wrapper path leaves got_rx=0, direct
                // path sets got_rx=1/rx0=0xA5).
                let recv = m.soc.wifi_espnow_peer_slot(ESPNOW_PEER_OBJ, 2);
                if rxcb != 0 && recv != 0 {
                    let peer = [0x02u8, 0x11, 0x22, 0x33, 0x44, 0x55];
                    // Full A→B→A exchange: the sketch sends ASCII
                    // "hello" (68656c6c6f); the virtual second node
                    // echoes the same 5 bytes back (B→A), so the
                    // sketch's rx0/rxlen/rxsum prove the whole frame
                    // both directions (not just byte 0).
                    m.soc.wifi_espnow_invoke_rx_cb(
                        recv,
                        &peer,
                        &[0x68, 0x65, 0x6C, 0x6C, 0x6F],
                        true,
                        ESPNOW_PEER_OBJ,
                    );
                    if m.run_espnow_callback(1) {
                        println!("[host] WiFi ESP-NOW RX-cb invoked (got_rx)");
                        wifi_espnow_rx_done = true;
                    }
                }
            }
        }
        // WiFi STA insider hooks live in `run_fast_core` (machine.rs —
        // per-op sampling inside the block loop; pre/post-step sampling
        // here would miss mid-block entry pcs).
        let uart_len = uart_buf.len();
        if pc == last_pc && uart_len == last_uart_len {
            idle_steps += 1;
            stuck += 1;
            // 1M macro-steps: the ROM stub loader copies DRAM segments
            // byte-wise (~265k iterations for a big .rodata image like
            // MicroPython's), and block-at-a-time execution samples the
            // same copy-loop pc every macro-step — 200k tripped mid-copy.
            if stuck == 1_000_000 {
                println!(
                    "\n== STUCK: core0 pc {:#010x} unchanged for 1M steps at step {i} (a0={:#x} a1={:#x} a2={:#x}) ==",
                    pc,
                    m.cpu[0].reg(0),
                    m.cpu[0].reg(1),
                    m.cpu[0].reg(2),
                );
                break;
            }
            // Early-exit: if both cores are idle (no PC change + no new UART
            // output) for 2M steps, the firmware's setup() has finished and
            // the main loop is spinning — we've seen all the meaningful output.
            // IDLE_STEPS overrides (slow boots like MicroPython need more).
            let idle_limit: usize = env::var("IDLE_STEPS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(2_000_000);
            if idle_steps >= idle_limit {
                println!("\n== IDLE: no output or PC change for 2M steps at step {i} ==");
                // TEMP (2026-10-04, RF forensics — DELETE after): park-loop
                // tracer for the WorkerL3 management-TX stall (core0 parks
                // at 0x42074b4f `ppTxFragmentProc+0x13f` with the entry hook
                // never firing). Single-step 12× logging pc + raw word +
                // regs so the polling load (same address every iteration)
                // names the flag the host must satisfy (like the BLE sem
                // pre-seed). Gated to the stall pc range to avoid spamming
                // normal DONE-idles on every other image.
                if path.contains("test_worker_l3") && (0x4207_4a10..0x4207_519c).contains(&pc) {
                    for _ in 0..12 {
                        let p = m.cpu[0].pc;
                        let w = m.soc.read32(p);
                        let mut regs = [0u32; 16];
                        for (k, r) in regs.iter_mut().enumerate() {
                            *r = m.cpu[0].reg(k as u32);
                        }
                        println!("[host] PARKTRACE pc={p:#010x} word={w:#010x} regs={regs:08x?}");
                        let _ = m.cpu[0].step_one(&mut m.soc);
                    }
                }
                break;
            }
        } else {
            idle_steps = 0;
            stuck = 0;
        }
        last_pc = pc;
        last_uart_len = uart_buf.len();
    }

    // --- Final drain: grab any remaining UART/USB-Serial TX bytes ---
    {
        let tx1 = m.take_uart_tx(1);
        uart_buf.extend_from_slice(&tx1);
        let tx = m.take_uart_tx(0);
        uart_buf.extend_from_slice(&tx);
    }

    // TEMP (2026-10-03): dump the event-dispatch trace ring in canned mode
    // (DELETE after). Shows the exact handler path our delivered CCs took.
    if ble_canned {
        for (pc, a10, a11, a12) in m.soc.ble_trace_dump() {
            if pc != 0 {
                println!(
                    "[host] BLETRACE pc={pc:#010x} a10={a10:#010x} a11={a11:#010x} a12={a12:#010x}"
                );
            }
        }
        // TEMP (2026-10-03): queue storage post-mortem (DELETE after). Is
        // our event still sitting in the evq storage (never taken)? The
        // storage observed at post time was 12B at 0x3fcb148c.
        println!(
            "[host] BLEQ post-mortem stor={:#010x} {:#010x} {:#010x}",
            m.soc.read32(0x3fcb_148c),
            m.soc.read32(0x3fcb_1490),
            m.soc.read32(0x3fcb_1494),
        );
    }

    // --- USB-OTG auto-enum IN-capture report (device answering the host) ---
    // The sketch checks the IN bytes itself through DFIFO0; the harness
    // asserts the identical transfer off the virtual wire here (the two
    // observe the same transfer from opposite ends — the model captures
    // the TXFIFO payload at completion, so a drain here is non-destructive
    // to the sketch path but proves the bytes moved end to end).
    if usb_host_enum {
        let got = m.soc.usb_host_take_in();
        println!("== usb IN capture ({}): {got:02x?}", got.len());
    }

    // --- Final report (MIPS = executed instructions/sec) ---
    let elapsed = t0.elapsed();
    let mips = executed as f64 / elapsed.as_secs_f64() / 1_000_000.0;
    println!(
        "\n== end: core0 pc {:#010x}, core1 pc {:#010x}, {} insns ({} macro-steps) in {:.2}s ({:.1} MIPS) ==",
        m.cpu[0].pc,
        m.cpu[1].pc,
        executed,
        i,
        elapsed.as_secs_f64(),
        mips
    );
    for c in 0..2 {
        let cpu = &m.cpu[c];
        println!(
            "== core{c}: a0={:#010x} a2={:#010x} sp={:#010x} wb={} ps={:#x} a10={:#010x} a11={:#010x}",
            cpu.reg(0),
            cpu.reg(2),
            cpu.reg(1),
            cpu.windowbase(),
            cpu.ps(),
            // TEMP (2026-10-04, forensics — DELETE after): a10/a11 carry
            // callee return codes at panic (e.g. l2cap_tx rc in a10 when
            // ble_att_tx_with_conn asserts) — needed to distinguish
            // ENOMEM vs EINVAL without another instrumented run.
            cpu.reg(10),
            cpu.reg(11),
        );
    }
    println!(
        "== uart bytes ({}): {:?}",
        uart_buf.len(),
        String::from_utf8_lossy(&uart_buf)
    );
    println!(
        "== irq taken core0={} core1={}",
        m.cpu[0].dbg_irq_taken, m.cpu[1].dbg_irq_taken,
    );
    println!(
        "== int_pending(0)={:#x} int_pending(1)={:#x}",
        m.soc.int_pending(0),
        m.soc.int_pending(1),
    );
}
