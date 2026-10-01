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

fn main() {
    let path = env::args().nth(1).expect("usage: run_flash <flash image>");
    let flash = fs::read(&path).expect("read flash image");
    // Image layout selection for the WiFi fixtures (scan vs STA sketch
    // link the pool/RAM differently; used by the layout tables below).
    let bin_name_contains_wifi_sta = path.contains("wifi_sta")
        || path.contains("test_worker_net")
        || path.contains("test_worker_l3");

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
    const BLE_HOST_CB: u32 = 0x3c06_b7c8;
    let mut ble_cb_entry: Option<u32> = None;
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
    // test-worker-l3 image layout (nm on the test-worker-l3 ELF
    // 2026-09-29 — same field order as WORKER_LAYOUT; the sketch source
    // pins the layout like every other image, re-nm after any .ino edit).
    const WORKER_L3_LAYOUT: WifiLayout = WifiLayout {
        scan_start: 0x4206_3d18,
        connect: 0x4203_c9d4,
        wifi_event_var: 0x3c0b_42b0,
        ip_event_var: 0x3c0b_3bac,
        count_cell: 0x3fc9_f936,
        scan_count: 0x3fc9_aef0,
        scan_result: 0x3fc9_aeec,
        records_check: 0x4200_3fb0,
        ready_lists: 0x3fc9_b8ec,
        top_prio: 0x3fc9_b85c,
        reg_heaps: 0x3fc9_b7a4,
        pxcur: 0x3fc9_bae0,
        sta_network_if: 0x3fc9_ae78,
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

    let wifi_scan_aps: Vec<esp32s3_soc::wifi::ScanFixtureAp> = env::var("WIFI_SCAN_APS")
        .ok()
        .map(|s| esp32s3_soc::wifi::parse_scan_fixtures(&s))
        .unwrap_or_default();
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
        let pc = m.cpu[0].pc;
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
            if pc == 0x4037_7367 {
                m.cpu[0].step_one(&mut m.soc);
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
                    if gw.write_all(&len).and(gw.write_all(&frame)).is_err() {
                        println!("[host] net bridge write failed; dropping bridge leg");
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
                        println!("[host] BLE RX {}B h4={:#04x}", rbuf.len(), rbuf[0]);
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
        // BLE reply deliver (level-triggered ACK path — see
        // `Soc::ble_ack_deliver_at`): whenever a bridge reply is queued
        // AND the firmware's ack waiter is parked on `ble_hs_hci_sem`
        // (`npl_freertos_sem_pend` → `xQueueSemaphoreTake`), route the
        // oldest reply through the firmware's OWN ack path (store EVT
        // bytes on the in-flight TX block + sem release + ready the
        // woken waiter). The waiter-parked gate is load-bearing: with a
        // one-block pool the arena block is the checked-out TX mbuf ONLY
        // while its owner is blocked waiting — delivering into it while
        // the firmware runs free would race the next alloc (proven live
        // class: foreign-buffer `assert failed: 0x42014482`). With no
        // waiter the reply stays queued (the tap-time deliver covers
        // the send-side race; this covers the RX-arrival side). One
        // deliver per step max (replies are rare — the checks are two
        // word reads + a FIFO-length check when idle).
        if ble_gw.is_some() && m.soc.bt_hci_rx_pending() > 0 {
            let sem = m.soc.ble_ack_sem();
            if sem != 0 && m.soc.queue_recv_waiting(sem) {
                use esp32s3_soc::Soc as _BleSoc;
                if let Some(tcb) = m.soc.ble_ack_deliver_at(_BleSoc::BLE_POOL_CMD_ARENA, 0) {
                    m.soc.ble_ready_task(tcb);
                    println!("[host] BLE ack delivered (rx-arrival)");
                }
            }
        }
        // BLE VHCI RX hook: deliver ONE queued controller→host packet per
        // step by running the registered `notify_host_recv` callback
        // IN FIRMWARE (same windowed-ABI discipline as
        // `run_espnow_callback`: 2-arg call, args in caller a10/a11 =
        // callee a2/a3, fake-RETW on return). Drained every step the leg
        // is up (packets are rare — the take is a single empty-FIFO check
        // when idle, same discipline as the UART fast path).
        // The callback address is discovered once from the `vhci_host_cb`
        // rodata (`BLE_HOST_CB`, BLE image only): slot 0 =
        // `notify_host_send_available`, slot 1 = `notify_host_recv`
        // (struct order in esp_nimble_hci.c, proven by the 0x42005004 /
        // 0x4200a050 words at 0x3c06b7c8). The packet bytes ride the
        // host scratch window (`Soc::bt_hci_stage_rx` at
        // `WIFI_SCRATCH + 0x2500`, past the net-RX slots — never heap,
        // never freed: `host_rcv_pkt` only READS the bytes into an mbuf
        // it allocates itself, same class as the `WIFI_SCRATCH` fixture
        // window).
        //
        // SCOPE (proven live via pb5: the plain `host_rcv_pkt` firmware
        // path asserts in `ble_transport_free` even with NO hook and NO
        // bridge traffic): the bare-metal firmware→controller leg is
        // validated (TX tap captures the HCI Reset, the bridge answers
        // Command Complete, the RX FIFO stages it — all observable in
        // the `[host] BLE TX/RX` lines). Driving the reply INTO the
        // firmware via a synthetic `host_rcv_pkt` call is NOT wired:
        // the reply must enter through the firmware's OWN VHCI poll
        // (`host_rcv_pkt` at 0x420050a0, called by the controller glue
        // with its own buffer at 0x3fcacfd6) — a host-staged buffer at a
        // foreign address walks the mbuf pool free path with a block the
        // pool does not recognize (`assert failed: 0x42014482` in
        // `os_memblock_from(pool_cmd)`, panic_abort EPC1=0x4037fdc4).
        // Until the controller-glue poll is modeled (the RWBLE ISR path
        // that hands the firmware its own buffer), the hook stays parked
        // here and the firmware runs its quiet-controller path.
        if ble_gw.is_some() && ble_cb_entry.is_none() {
            let cb0 = m.soc.read32(BLE_HOST_CB);
            let cb1 = m.soc.read32(BLE_HOST_CB + 4);
            if (0x4000_0000..0x4240_0000).contains(&cb1) && cb0 != 0 {
                ble_cb_entry = Some(cb1);
                println!("[host] BLE vhci_host_cb: notify_host_recv={cb1:#010x}");
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
            let layout = if path.contains("test_worker_l3") {
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
            let layout = if path.contains("test_worker_l3") {
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
            let layout = if path.contains("test_worker_l3") {
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
            let layout = if path.contains("test_worker_l3") {
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
            let layout = if path.contains("test_worker_l3") {
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
            "== core{c}: a0={:#010x} a2={:#010x} sp={:#010x} wb={} ps={:#x}",
            cpu.reg(0),
            cpu.reg(2),
            cpu.reg(1),
            cpu.windowbase(),
            cpu.ps()
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
