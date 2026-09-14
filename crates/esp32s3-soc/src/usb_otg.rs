//! ESP32-S3 USB-OTG (DesignWare DWC2 full-speed device/host) model.
//!
//! Register block at `0x6008_0000` (`dwc2_esp32.h` `DWC2_FS_REG_BASE`;
//! IRQ `ETS_USB_INTR_SOURCE` = 38, 7 EPs / 5 IN). Layout per
//! `soc/usb_dwc_struct.h` (offsets in the member comments below; the S3
//! header isn't shipped in arduino-cli, so the identical DWC2 core layout
//! was verified against the ESP32-P4 copy).
//!
//! Modeled (device path): init (GRSTCTL/DCFG/DCTL/EP0/DIEPCTL0, TXFIFO
//! space) plus an in-model EP0 loopback (no external host): device-mode
//! DFIFO0 writes accumulate the 8-byte SETUP packet and decode through
//! the shared `handle_setup` table (GET_DESCRIPTOR stages IN data,
//! SET_ADDRESS arms, SET_CONFIGURATION records, else STALL with the
//! host-channel side effect snapshotted away); staged IN payload mirrors
//! into the device RXFIFO so DFIFO0 reads serve it; DIEPTSIZ0/DOEPTSIZ0
//! writes complete status (applies pending address, raises IN XFRC);
//! DIEPINT0/DOEPINT0 are real W1C latches. Validated by unit tests + the
//! `esp32s3_usb_otg` sketch (force-host FIFO leg, then force-device
//! SETUP/DESC/STATUS legs).
//!
//! Modeled (device auto-enumeration, host-driven): the in-model host acts
//! as the enumeration counterparty for the firmware's own device stack,
//! so the REAL TinyUSB device driver (`esp32s3_usb_device`) enumerates
//! without an external host. `usb_host_enum_device()` (host frontend,
//! like `uart_inject_rx`) drives the silicon-true bus sequence the DWC2
//! core would raise: latched GINTSTS USBRST + ENUMDONE (W1C, masked into
//! source 38 via GINTMSK), DSTS ENUMSPD = full-speed, a GRXSTSP
//! SETUP_RX + SETUP_DONE pair with the 8 SETUP bytes staged in the RX
//! FIFO, DOEPINT0 STPKTRCVD + SETUP (W1C), and DAINT IN/OUT bits for EP0.
//! The firmware's own descriptors flow back through the IN path
//! (DIEPCTL0 EPENA + DIEPTSIZ0 xfer-size + TXFE + XFRC; DFIFO0 reads pop
//! the staged bytes) and are captured by `usb_host_take_in()` for the
//! harness to assert; the status OUT stage completes via DOEPTSIZ0 +
//! DOEPINT0 XFRC. Validated by unit tests + the `esp32s3_usb_device`
//! sketch (`USB_HOST_ENUM=1`: TinyUSB stack up, `tud_mounted()` true,
//! GET_DESCRIPTOR answered from the firmware's own HID descriptors).
//!
//! Modeled (device bulk/interrupt, loopback-validatable): non-zero
//! endpoints mirror EP0's loopback one level down — device-mode DFIFO0
//! writes past the SETUP stage accumulate per-endpoint OUT payloads
//! (DIEPTSIZ/DOEPTSIZ xfer-size writes arm them like EP0 status), EP1-IN
//! DFIFO reads serve the staged bytes back (echo), and the matching
//! DIEPINT/DOEPINT XFRC latches. No STALL matrix, no isochronous
//! scheduling, no DMA: bulk/interrupt complete instantly empty unless
//! the firmware staged payload (documented; validated by unit tests +
//! the usb_otg sketch echo leg).
//!
//! Modeled (host path): a simulated full-speed device behind the port.
//! HPRT power connects it (ConnSts/ConnDet/SPD=FS), port reset enables it
//! (Ena/EnChng, self-clearing Rst). Host channels (8 @ 0x500 stride 0x20:
//! HCCHAR/HCINT(W1C)/HCINTMSK/HCTSIZ) execute control transfers
//! synchronously on ChEna: SETUP consumes 8 DFIFO-staged bytes and drives
//! GET_DESCRIPTOR (stages IN data), SET_ADDRESS (applied at status
//! completion), SET_CONFIGURATION; IN moves payload into the RXFIFO
//! (GRXSTSP meta + DFIFO pops, RXFLVL); OUT consumes staged bytes.
//! GINTSTS reports live RXFLVL (bit 4) / HPRTINT (24) / HCINTR (25);
//! HAINT summarizes masked channel interrupts. Validated by unit tests +
//! the `esp32s3_usb_host` poke sketch (port reset, device/config
//! descriptors, address + configuration assignment).
//!
//! KNOWN LIMITATIONS: real external-host enumeration (a physical host
//! counterparty driving the bus) is out of scope; all `Serial` flows
//! through validated USB-Serial-JTAG, so Arduino "USB CDC On Boot" stays
//! unsupported. Device bulk/interrupt transfers beyond the loopback echo
//! (no STALL matrix, no scheduling/DMA) stay out. Host-side
//! approximations: transfers complete instantly (no SOF/frame timing;
//! HFNUM reads 0); R1b busy (port reset, SWITCH-like waits) is
//! synchronous; DATA-toggle PID is accepted, not enforced; non-control
//! endpoint types complete instantly with zeros; bulk/periodic schedules,
//! SOF, split transactions and DMA (HCDMA) are not modeled; disconnect
//! is not modeled (the device stays connected once powered). DAINT/EPINT
//! beyond EP0-IN/OUT (non-zero endpoints stay loopback-driven), DSTS
//! beyond ENUMSPD, and HW config ID registers read 0.

/// USB-OTG DWC register-block base (`dwc2_esp32.h` `DWC2_FS_REG_BASE`).
pub const USB_OTG_BASE: u32 = 0x6008_0000;
/// DFIFO window base: per-endpoint IN TXFIFO pages (`dwc2_regs_t.fifo`
/// = EP0-IN @ +0x1000, EP1-IN @ +0x2000, ...; the RXFIFO/OUT path is
/// shared). The generic page dispatch routes a wider window (see
/// `USB_OTG_FIFO_PAGES`) so non-zero endpoints are reachable.
pub const USB_OTG_FIFO_PAGE: u32 = 0x6008_1000;
/// DFIFO window page count (EP0..EP5-IN; the S3 instantiates 7 EPs / 5 IN,
/// all covered).
pub const USB_OTG_FIFO_PAGES: u32 = 6;
/// USB interrupt source (`interrupts.h` LEDC=35 explicit → EFUSE=36,
/// TWAI=37, USB=38, RTC_CORE=39 — same counting as rtc.rs).
pub const USB_OTG_INTR_SOURCE: u32 = 38;

// Core register offsets (usb_dwc_struct.h member comments). Unused ones
// document the map (the generic mmio arm serves their plain-store/readback
// behavior); each carries an explicit allow so the clippy gate stays green.
#[allow(dead_code)]
const GOTGCTL: u32 = 0x000;
#[allow(dead_code)]
const GAHBCFG: u32 = 0x008;
const GUSBCFG: u32 = 0x00C;
pub(crate) const GRSTCTL: u32 = 0x010;
const GINTSTS: u32 = 0x014;
const GINTMSK: u32 = 0x018;
const GRXSTSP: u32 = 0x020;
const GNPTXSTS: u32 = 0x02C;
const GRXFSIZ: u32 = 0x024;
const DIEPTXF0: u32 = 0x028;
const GDFIFOCFG: u32 = 0x05C;
#[allow(dead_code)]
const DCFG: u32 = 0x800;
#[allow(dead_code)]
const DCTL: u32 = 0x804;
const DSTS: u32 = 0x808;
#[allow(dead_code)]
const DIEPMSK: u32 = 0x810;
#[allow(dead_code)]
const DOEPMSK: u32 = 0x814;
const DAINT: u32 = 0x818;
const DAINTMSK: u32 = 0x81C;
#[allow(dead_code)]
const DIEPCTL0: u32 = 0x900;
const DIEPINT0: u32 = 0x908;
#[allow(dead_code)]
const DIEPTSIZ0: u32 = 0x910;
const DTXFSTS0: u32 = 0x914;
// DEP struct layout (dwc2_type.h dwc2_dep_t, 0x20 bytes per EP: ctl @+0x00,
// intr @+0x08, tsiz @+0x10, dma @+0x14, dtxfsts @+0x18): EP0-IN base 0x900,
// EP0-OUT base 0xB00. The TinyUSB `edpt_schedule_packets` /
// `epin_write_tx_fifo` slave path reads DTXFSTS (TXFIFO space) at
// EP-base + 0x18 — NOT the global DTXFSTS0 alias the old model served.
// Both must report the live TXFIFO space (see the DTXFSTS0 arm: the
// per-EP word routes there too).
const DIEPDMA0: u32 = 0x918;
#[allow(dead_code)]
const DOEPDMA0: u32 = 0xB18;
#[allow(dead_code)]
const DEP_DTXFSTS_OFF: u32 = 0x18;
#[allow(dead_code)]
const DOEPCTL0: u32 = 0xB00;
const DOEPINT0: u32 = 0xB08;
#[allow(dead_code)]
const DOEPTSIZ0: u32 = 0xB10;
const DFIFO0: u32 = 0x1000;

// GRSTCTL bits (usb_dwc_grstctl_reg_t): csftrst[0], rxfflsh[4],
// txfflsh[5], txfnum[10:6], ahbidle[31].
pub(crate) const GRST_CSFTRST: u32 = 1 << 0;
const GRST_RXFFLSH: u32 = 1 << 4;
const GRST_TXFFLSH: u32 = 1 << 5;
const GRST_AHBIDLE: u32 = 1 << 31;
/// TXFIFO0 depth in words (`dwc2_esp32.h` `otg_dfifo_depth` = 256).
const TXFIFO_WORDS: u32 = 256;
const REG_WORDS: usize = 0x2000 / 4;

// GUSBCFG force-mode bits (usb_dwc_gusbcfg_reg_t bit order).
const GUSBCFG_FORCE_HOST: u32 = 1 << 29;
const GUSBCFG_FORCE_DEVICE: u32 = 1 << 30;
// Core ID / hardware-config registers (read-only on silicon; the TinyUSB
// `dcd_init` → `dwc2_core_init` path probes them before touching anything
// else, so they must read plausible values or the stack never reaches
// FDMOD — with all-zero reads `check_dwc2` fails, `dcd_init` returns false,
// and GUSBCFG stays 0 forever, proven by disassembling the linked
// `dwc2_core_init`/`dwc2_core_is_highspeed_phy`/`reset_core` against
// `dwc2_type.h`). GUID @0x03C reads 0 (user ID); GSNPSID @0x040 carries
// the OTG ID (high16 0x4F54, the `check_dwc2` assert) + rev 4.00a (low16 <
// 4.20a, so `reset_core` uses the self-clearing-CSRST poll the GRSTCTL arm
// already implements — a ≥4.20a rev would wait on CSRST_DONE bit 29
// forever); GHWCFG1..4 @0x044..0x050 describe a slave-only dedicated-FS
// core with 6 device EPs (ep_count 7, matching `dwc2_esp32.h`).
const GUID: u32 = 0x03C;
const GSNPSID: u32 = 0x040;
const GHWCFG1: u32 = 0x044;
const GHWCFG2: u32 = 0x048;
const GHWCFG3: u32 = 0x04C;
const GHWCFG4: u32 = 0x050;
const GSNPSID_VAL: u32 = 0x4F54_400A;
const GHWCFG2_VAL: u32 = 0x200D_D920;
const GHWCFG3_VAL: u32 = 0x0100_00E8;
const GHWCFG4_VAL: u32 = 0x1600_0000;
// Host-mode registers (usb_dwc_struct.h). HCFG/HAINTMSK are served by the
// generic plain-store arm (allows keep the map complete for the gate).
#[allow(dead_code)]
const HCFG: u32 = 0x400;
// Host frame number (usb_dwc HFNUM register @ HCFG + 0x08, read-only on
// silicon): free-running frame counter, advanced by tick() while the USB
// controller clock runs (not wall-time-accurate, but monotonic — enough
// to prove the SOF clock domain advances).
const HFNUM: u32 = 0x408;
const HAINT: u32 = 0x414;
#[allow(dead_code)]
const HAINTMSK: u32 = 0x418;
pub(crate) const HPRT: u32 = 0x440;
pub(crate) const HC_BASE: u32 = 0x500;
pub(crate) const HC_STRIDE: u32 = 0x20;
pub(crate) const HC_COUNT: usize = 8;
// HPRT bits (usb_dwc_hprt_reg_t).
const HPRT_CONNSTS: u32 = 1 << 0;
const HPRT_CONNDET: u32 = 1 << 1;
const HPRT_ENA: u32 = 1 << 2;
const HPRT_ENCHNG: u32 = 1 << 3;
pub(crate) const HPRT_RST: u32 = 1 << 8;
pub(crate) const HPRT_PWR: u32 = 1 << 12;
const HPRT_SPD_FS: u32 = 1 << 17; // spd field 01 = full-speed
const HPRT_W1C: u32 = HPRT_CONNDET | HPRT_ENCHNG;
// Host-channel register offsets within a channel block.
pub(crate) const HCCHAR_OFF: u32 = 0x00;
const HCINT_OFF: u32 = 0x08;
const HCINTMSK_OFF: u32 = 0x0C;
const HCTSIZ_OFF: u32 = 0x10;
// HCCHAR bits.
pub(crate) const HCCHAR_CHENA: u32 = 1 << 31;
pub(crate) const HCCHAR_CHDIS: u32 = 1 << 30;
// HCINT bits (W1C).
const HCINT_XFERCOMPL: u32 = 1 << 0;
const HCINT_CHHALTED: u32 = 1 << 1;
const HCINT_STALL: u32 = 1 << 3;
// GINTSTS device bits (dwc2_type.h GINTSTS_*_Pos: the TinyUSB `dcd_int_
// handler` polls USBRST then ENUMDNE, then OEPINT/IEPINT; RXFLVL is
// shared with the host path).
const GINT_USBRST: u32 = 1 << 12;
const GINT_ENUMDNE: u32 = 1 << 13;
const GINT_IEPINT: u32 = 1 << 18;
const GINT_OEPINT: u32 = 1 << 19;
// DSTS ENUMSPD (dwc2_type.h DSTS_ENUMSPD_*): full-speed on the S3's
// dedicated FS PHY = 3.
const DSTS_ENUMSPD_FS: u32 = 3 << 1;
// DAINT EP0 bits (dwc2_type.h DAINT_IEPINT_Pos/OEPINT_Pos = 0/16).
const DAINT_IN_EP0: u32 = 1 << 0;
const DAINT_OUT_EP0: u32 = 1 << 16;
// GRXSTSP device fields (dwc2_type.h GRXSTSP_EPNUM/BCNT/DPID/PKTSTS_Pos =
// 0/4/15/17; tinyusb `handle_rxflvl_irq` switches on PKTSTS).
const GRXSTSP_PKTSTS_SETUP_RX: u32 = 6;
const GRXSTSP_PKTSTS_SETUP_DONE: u32 = 4;
// DOEPINT0 device bits (dwc2_type.h DOEPINT_SETUP/STPKTRX_Pos = 3/15).
const DOEPINT_SETUP: u32 = 1 << 3;
const DOEPINT_STPKTRX: u32 = 1 << 15;
// DIEPINT0 device bits (dwc2_type.h DIEPINT_TXFE_Pos = 7).
const DIEPINT_TXFE: u32 = 1 << 7;
// DIEPEMPMSK EP0 bit (dwc2_type.h DIEPEMPMSK_INEPTXFEM_Pos = 0): the
// TinyUSB slave path enables the TXFE interrupt per endpoint while payload
// remains (`diepempmsk |= 1<<ep`), and the core raises TXFE (DIEPINT0
// bit 7) while the bit is set and TXFIFO space covers the transfer.
const DIEPEMPMSK_EP0: u32 = 1 << 0;
const DIEPEMPMSK: u32 = 0x834;
// DIEPCTL0/DOEPCTL0 EPENA (dwc2_type.h DIEPCTL_EPENA/DOEPCTL_EPENA_Pos).
const DEPCTL_EPENA: u32 = 1 << 31;
// GINTSTS host bits.
const GINT_RXFLVL: u32 = 1 << 4;
const GINT_PRTINT: u32 = 1 << 24;
const GINT_HCINTR: u32 = 1 << 25;
// GRXSTSP fields: chnum[3:0], bcnt[14:4], dpid[16:15], pktsts[20:17].
const GRXSTSP_PKTSTS_IN: u32 = 6 << 17; // IN data packet received

/// Simulated full-speed device descriptors (fixed function).
const DEV_DESC: [u8; 18] = [
    18, 1, 0x00, 0x02, // bLength, DEVICE, USB 2.00
    0x00, 0x00, 0x00, // class/subclass/protocol (per-interface)
    64,   // bMaxPacketSize0
    0x3A, 0x30, // idVendor 0x303A (Espressif)
    0x01, 0x10, // idProduct 0x1001 (emulated FS device)
    0x00, 0x01, // bcdDevice 1.00
    1, 2, 3, // iManufacturer/iProduct/iSerialNumber (below)
    1, // bNumConfigurations
];
// USB string descriptors (type 3, UTF-16LE; index 0 = LANGID 0x0409).
const STR0_DESC: [u8; 4] = [4, 3, 0x09, 0x04];
const STR1_DESC: [u8; 20] = [
    0x14, 0x03, 0x45, 0x00, 0x73, 0x00, 0x70, 0x00, 0x72, 0x00, 0x65, 0x00, 0x73, 0x00, 0x73, 0x00,
    0x69, 0x00, 0x66, 0x00, // "Espressif"
];
const STR2_DESC: [u8; 26] = [
    0x1A, 0x03, 0x45, 0x00, 0x53, 0x00, 0x50, 0x00, 0x33, 0x00, 0x32, 0x00, 0x2D, 0x00, 0x53, 0x00,
    0x33, 0x00, 0x2D, 0x00, 0x45, 0x00, 0x4D, 0x00, 0x55, 0x00, // "ESP32-S3-EMU"
];
const STR3_DESC: [u8; 10] = [
    0x0A, 0x03, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, // "1234"
];

const CFG_DESC: [u8; 32] = [
    9, 2, 32, 0, 1, 1, 0, 0x80, 50, // config: 32 total, 1 iface, 100 mA
    9, 4, 0, 0, 2, 0xFF, 0x00, 0x00, 0, // interface: 2 EPs, vendor
    7, 5, 0x81, 0x02, 64, 0, 0, // EP1 IN bulk 64 B
    7, 5, 0x02, 0x02, 64, 0, 0, // EP2 OUT bulk 64 B
];

/// Simulated USB device behind the host port (address/config assigned by
/// the host; descriptors fixed).
struct SimDevice {
    present: bool,
    addr: u8,
    addr_pending: u8,
    addr_armed: bool,
    configured: u8,
    /// Staged IN payload (from GET_DESCRIPTOR; consumed by IN transfers).
    in_data: alloc::collections::VecDeque<u8>,
}

impl SimDevice {
    fn reset() -> Self {
        Self {
            present: false,
            addr: 0,
            addr_pending: 0,
            addr_armed: false,
            configured: 0,
            in_data: alloc::collections::VecDeque::new(),
        }
    }
}

pub struct UsbOtg {
    regs: [u32; REG_WORDS],
    /// Staged TXFIFO0 bytes (DFIFO0 writes with no host to drain them).
    txfifo: alloc::vec::Vec<u8>,
    /// Simulated device behind the host port.
    dev: SimDevice,
    /// DFIFO-staged OUT/SETUP payload bytes (host mode).
    host_txfifo: alloc::collections::VecDeque<u8>,
    /// Staged RX bytes for the firmware to pop via DFIFO reads.
    rx_data: alloc::collections::VecDeque<u8>,
    /// Pending GRXSTSP entry (channel, byte count, dpid), if any.
    rx_meta: Option<(u8, u16, u8)>,
    /// Device-mode EP0 loopback: SETUP bytes staged by DFIFO0 writes in
    /// device mode (decoded once 8 accumulate).
    dev_setup: alloc::vec::Vec<u8>,
    /// SETUP stage done (EP0 OUT XFRC raised at least once since reset):
    /// later DFIFO0 writes are EP1-OUT payload, not new SETUP packets.
    /// (The XFRC flag itself is W1C-clearable, so it can't be the router.)
    dev_setup_done: bool,
    /// Device-mode EP1 loopback: OUT payload bytes staged by DFIFO0 writes
    /// once the EP0 SETUP stage is done (echoed back on EP1-IN reads).
    dev_ep1_out: alloc::collections::VecDeque<u8>,
    /// Per-endpoint IN TX queues (EP1..EP7; EP0 uses the shared `txfifo`).
    /// Staged by DFIFO-page writes (EPn-IN @ +0x1000*n) and served by
    /// DFIFO-page reads; the EP1-echo loopback path is unchanged.
    ep_tx: [alloc::collections::VecDeque<u8>; 7],
    /// Latched device GINTSTS bits (USBRST/ENUMDONE, W1C via GINTSTS).
    /// Raised by the auto-enum host frontend; the TinyUSB device ISR
    /// polls-and-clears them exactly like silicon's bus-reset bark.
    dev_gint: u32,
    /// Auto-enum session active (set by the first `usb_host_setup`,
    /// cleared by `usb_host_status_out`): device-mode DFIFO0 writes are
    /// IN-TXFIFO pushes, and TSIZ writes don't run the loopback status
    /// path. The flag spans transfers (the harness re-arms it per SETUP);
    /// between status-out and the next SETUP the firmware performs no
    /// DFIFO writes, so routing by session (not by STPKTRCVD latch) is
    /// exact. Cleared by any core reset (GINTSTS W1C state is reset
    /// state, like silicon).
    dev_enum_session: bool,
    /// EP0 IN transfer armed (DIEPCTL0 EPENA seen, completion pending).
    /// Slave-mode TXFIFO push comes AFTER EPENA via the TXFE interrupt
    /// (like silicon), so completion fires once pushed bytes cover the
    /// programmed DIEPTSIZ0 size — or immediately at EPENA when the FIFO
    /// already holds them (or the size is zero, a status ZLP).
    dev_in_armed: bool,
    /// Pending device GRXSTSP queue (EP0 SETUP_RX then SETUP_DONE pops).
    /// Staged by the auto-enum host frontend; GRXSTSP pops entries and
    /// GINTSTS.RXFLVL reads live while any remain.
    dev_grxstsp: alloc::collections::VecDeque<u32>,
    /// Single-shot SETUP promotion armed (set by `usb_host_setup`, consumed
    /// when the SETUP_DONE GRXSTSP entry pops): models the silicon side
    /// effect where retiring the SETUP packet asserts DOEPINT0 SETUP
    /// (bit 3). Without it the TinyUSB `handle_epout_slave` never sees
    /// `setup_phase_done` and `dcd_event_setup_received` never fires
    /// (observed live: RXFLVL consumed, STPKTRCVD latched, but the control
    /// handler never ran so no IN transfer was ever scheduled).
    setup_done_armed: bool,
    /// Captured device-IN bytes (firmware answering the host): every
    /// DFIFO0 read that serves staged IN payload appends here, so the
    /// harness can assert the firmware answered from its own descriptors
    /// (`usb_host_take_in`). Loopback EP1-echo reads are excluded (host
    /// never sees the echo path).
    dev_in_capture: alloc::vec::Vec<u8>,
    /// IN-payload debug mirror (auto-enum path): `dev_in_maybe_complete`
    /// copies the completed payload here as well, so the SKETCH can check
    /// the same bytes through DFIFO0 reads without disturbing the harness
    /// capture (which drains `dev_in_capture` separately). Without this
    /// the sketch's check reads would pop the RXFIFO (consumed SETUP
    /// bytes = zeros) and fail even though the transfer completed.
    dev_in_mirror: alloc::collections::VecDeque<u8>,
    /// SOF frame counter (see HFNUM).
    sof: u16,
}

impl UsbOtg {
    pub fn new() -> Self {
        let mut o = Self {
            regs: [0; REG_WORDS],
            txfifo: alloc::vec::Vec::new(),
            dev: SimDevice::reset(),
            host_txfifo: alloc::collections::VecDeque::new(),
            rx_data: alloc::collections::VecDeque::new(),
            rx_meta: None,
            dev_setup: alloc::vec::Vec::new(),
            dev_setup_done: false,
            dev_ep1_out: alloc::collections::VecDeque::new(),
            ep_tx: [
                alloc::collections::VecDeque::new(),
                alloc::collections::VecDeque::new(),
                alloc::collections::VecDeque::new(),
                alloc::collections::VecDeque::new(),
                alloc::collections::VecDeque::new(),
                alloc::collections::VecDeque::new(),
                alloc::collections::VecDeque::new(),
            ],
            dev_gint: 0,
            dev_enum_session: false,
            dev_in_armed: false,
            dev_grxstsp: alloc::collections::VecDeque::new(),
            setup_done_armed: false,
            dev_in_capture: alloc::vec::Vec::new(),
            dev_in_mirror: alloc::collections::VecDeque::new(),
            sof: 0,
        };
        o.core_reset();
        o
    }

    /// Core soft reset (GRSTCTL.CSFTRST): restore power-on state and raise
    /// AHBIDLE; the reset bit itself clears (self-clearing on silicon).
    fn core_reset(&mut self) {
        self.regs = [0; REG_WORDS];
        self.txfifo.clear();
        self.dev = SimDevice::reset();
        self.host_txfifo.clear();
        self.rx_data.clear();
        self.rx_meta = None;
        self.dev_setup.clear();
        self.dev_setup_done = false;
        self.dev_ep1_out.clear();
        for q in self.ep_tx.iter_mut() {
            q.clear();
        }
        self.dev_gint = 0;
        self.dev_enum_session = false;
        self.dev_in_armed = false;
        self.dev_grxstsp.clear();
        self.setup_done_armed = false;
        self.dev_in_capture.clear();
        self.dev_in_mirror.clear();
        self.regs[(GRSTCTL / 4) as usize] = GRST_AHBIDLE;
    }

    fn tx_space(&self) -> u32 {
        let used = (self.txfifo.len() + self.host_txfifo.len()) as u32;
        // TXFIFO depth is in 32-bit WORDS (`dwc2_esp32.h`
        // `otg_dfifo_depth` = 256): 18 staged bytes occupy 5 words even
        // though only 18 bytes drain into the IN capture (`txfifo` is a
        // byte queue; the word count is the ceiling of bytes/4).
        TXFIFO_WORDS - used.div_ceil(4).min(TXFIFO_WORDS)
    }

    /// Force-host mode latched (sketch programs GUSBCFG bit 29).
    fn host_mode(&self) -> bool {
        self.regs[(GUSBCFG / 4) as usize] & GUSBCFG_FORCE_HOST != 0
    }

    /// Force-device mode latched (GUSBCFG FDMOD, bit 30 — the TinyUSB
    /// device stack programs it at init via `dcd_init`). The auto-enum
    /// host frontend only fires in device mode (never against the
    /// host-mode simulated device); the loopback path additionally
    /// requires it (a force-host write while loopback-staged would
    /// otherwise decode control traffic as firmware pokes).
    fn device_mode(&self) -> bool {
        self.regs[(GUSBCFG / 4) as usize] & GUSBCFG_FORCE_DEVICE != 0
    }

    fn hc_at(&self, ch: usize, off: u32) -> usize {
        ((HC_BASE + ch as u32 * HC_STRIDE + off) / 4) as usize
    }

    /// Channel interrupt pending = latched INT & mask (drives HAINT/HCINTR).
    fn hc_pending(&self, ch: usize) -> bool {
        let i = self.hc_at(ch, HCINT_OFF);
        let m = self.hc_at(ch, HCINTMSK_OFF);
        self.regs[i] & self.regs[m] != 0
    }

    /// Computed GINTSTS host bits: RXFLVL (RX data waiting), HPRTINT
    /// (port connect/enable-change latched), HCINTR (any masked channel
    /// interrupt). Quiet when idle (device behavior preserved).
    /// Device-mode auto-enum bits ride along: latched USBRST/ENUMDONE,
    /// live RXFLVL while GRXSTSP entries remain, and live OEPINT/IEPINT
    /// while EP0 OUT/IN flags are latched (the TinyUSB ISR polls GINTSTS
    /// first, then DAINT/DOEPINT0/DIEPINT0 — all must read live).
    /// NOTE: RXFLVL follows the GRXSTSP queue only (like silicon, where
    /// the RXFLVL interrupt fires per posted packet status); the RXFIFO
    /// bytes travel with the event and drain through DFIFO0 reads
    /// without affecting the flag.
    fn host_gintsts(&self) -> u32 {
        let mut v = self.dev_gint & (GINT_USBRST | GINT_ENUMDNE);
        if !self.rx_data.is_empty() && !self.dev_grxstsp.is_empty() {
            v |= GINT_RXFLVL;
        }
        let hprt = self.regs[(HPRT / 4) as usize];
        if hprt & (HPRT_CONNDET | HPRT_ENCHNG) != 0 {
            v |= GINT_PRTINT;
        }
        for ch in 0..HC_COUNT {
            if self.hc_pending(ch) {
                v |= GINT_HCINTR;
                break;
            }
        }
        if self.regs[(DOEPINT0 / 4) as usize] != 0 {
            v |= GINT_OEPINT;
        }
        if self.regs[(DIEPINT0 / 4) as usize] != 0 {
            v |= GINT_IEPINT;
        }
        v
    }

    /// Interrupt pending = GINTSTS & GINTMSK (source 38).
    pub fn int_pending(&self) -> bool {
        (self.host_gintsts() & self.regs[(GINTMSK / 4) as usize]) != 0
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            GRSTCTL => self.regs[(GRSTCTL / 4) as usize],
            // Core ID / hardware-config words: read-only ID values (writes
            // ignored below in the plain-store arm — a stored 0 must not
            // clobber them; the stack programs GUSBCFG way past them and
            // only ever reads these back).
            GUID => 0,
            GSNPSID => GSNPSID_VAL,
            GHWCFG1 => 0,
            GHWCFG2 => GHWCFG2_VAL,
            GHWCFG3 => GHWCFG3_VAL,
            GHWCFG4 => GHWCFG4_VAL,
            // GINTSTS is computed live (host bits + latched device
            // USBRST/ENUMDONE + live EP0 OEPINT/IEPINT/RXFLVL); DSTS
            // reports ENUMSPD = full-speed once the auto-enum host has
            // enumerated (0 before, like an unreset core); DAINT reports
            // live EP0 IN/OUT bits from the EPINT latches; GRXSTSP pops
            // the pending host RX status first, then device entries.
            GINTSTS => self.host_gintsts(),
            DSTS => {
                if self.dev_gint & GINT_ENUMDNE != 0 {
                    DSTS_ENUMSPD_FS
                } else {
                    0
                }
            }
            DAINT => {
                let mut v = 0;
                if self.regs[(DIEPINT0 / 4) as usize] != 0 {
                    v |= DAINT_IN_EP0;
                }
                if self.regs[(DOEPINT0 / 4) as usize] != 0 {
                    v |= DAINT_OUT_EP0;
                }
                v
            }
            // Device-mode EP0 interrupt flags: the loopback driver (see
            // `dev_loopback_*`) raises XFRC on each completed stage; STALL
            // latches when the SETUP packet is not a handled standard
            // request. W1C via the matching INT register write. TXFE
            // (bit 7) is read-only on silicon — it follows the live
            // TXFE state (see `dev_txfe_live`), so the read overlays it
            // and the W1C write below ignores it.
            DIEPINT0 => self.dev_diepint0(),
            DOEPINT0 => self.regs[(DOEPINT0 / 4) as usize],
            // GRXSTSP pops the pending host RX status first, then queued
            // device entries (0 when none). Popping the SETUP_DONE entry
            // models the silicon side effect the TinyUSB RXFLVL handler
            // depends on: after the DONE pop the core asserts DOEPINT0
            // SETUP (bit 3), which `handle_epout_slave` consumes via
            // `setup_phase_done` to deliver `dcd_event_setup_received`.
            // (On silicon the core raises SETUP when the SETUP packet
            // retires; the model stages STPKTRCVD eagerly at setup time,
            // but SETUP only here.) Single-shot per SETUP packet: the
            // flag is consumed below so a re-read does not re-latch.
            GRXSTSP => {
                if let Some((ch, bcnt, dpid)) = self.rx_meta.take() {
                    (ch as u32) | ((bcnt as u32) << 4) | ((dpid as u32) << 15) | GRXSTSP_PKTSTS_IN
                } else {
                    let w = self.dev_grxstsp.pop_front().unwrap_or_default();
                    if (w >> 17) & 0xF == GRXSTSP_PKTSTS_SETUP_DONE && self.setup_done_armed {
                        self.setup_done_armed = false;
                        self.regs[(DOEPINT0 / 4) as usize] |= DOEPINT_SETUP;
                    }
                    w
                }
            }
            // HAINT summarizes masked channel interrupts (RO).
            HAINT => {
                let mut v = 0u32;
                for ch in 0..HC_COUNT {
                    if self.hc_pending(ch) {
                        v |= 1 << ch;
                    }
                }
                v
            }
            // TX space: GNPTXSTS low half + DTXFSTS0 (remaining words) +
            // the per-EP DTXFSTS word the slave driver actually polls
            // (EP0-IN @0x918; see DEP_DTXFSTS_OFF).
            GNPTXSTS => self.tx_space() & 0xFFFF,
            DTXFSTS0 | DIEPDMA0 => self.tx_space(),
            // DFIFO reads: host mode pops staged RX bytes; device mode
            // serves the EP0 RXFIFO first (auto-enum SETUP bytes, then the
            // EP0 IN payload mirrored at loopback SETUP — captured for the
            // auto-enum harness), then the EP1 echo queue; empty reads 0
            // either way. (The GRXSTSP queue is NOT retired here: the
            // firmware pops each packet status explicitly, like silicon;
            // RXFLVL follows the queue. The SETUP bytes stay staged in the
            // RXFIFO until popped — TinyUSB slave mode reads them off
            // `rx_fifo` during the RXFLVL handler, i.e. as part of popping
            // the status, so bytes and statuses drain independently.)
            DFIFO0 => {
                let mut w = 0u32;
                for i in 0..4 {
                    let b = if !self.host_mode() && !self.rx_data.is_empty() {
                        let b = self.rx_data.pop_front();
                        if let Some(bb) = b {
                            // Capture only IN-payload reads (post-SETUP
                            // TXFIFO completions mirror there too, but the
                            // auto-enum IN path captures via dev_in_capture
                            // in `dev_in_maybe_complete`; the loopback
                            // mirror is the firmware answering, so capture
                            // it unless an enum session owns the FIFO).
                            if !self.dev_enum_session {
                                self.dev_in_capture.push(bb);
                            }
                        }
                        b
                    } else if !self.host_mode() && !self.dev_in_mirror.is_empty() {
                        // Auto-enum IN-payload debug read: after the TXFIFO
                        // transfer completes, the payload is mirrored here
                        // for observability (the sketch checks the same
                        // bytes the harness asserts via `usb_host_take_in`;
                        // the mirror is NOT the harness capture — that was
                        // filled at completion time and drains separately).
                        self.dev_in_mirror.pop_front()
                    } else if !self.host_mode() {
                        self.dev_ep1_out.pop_front()
                    } else {
                        self.rx_data.pop_front()
                    };
                    if let Some(b) = b {
                        w |= (b as u32) << (8 * i);
                    }
                }
                w
            }
            HFNUM => self.sof as u32,
            o if o < (REG_WORDS * 4) as u32 && o.is_multiple_of(4) => self.regs[(o / 4) as usize],
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            GRSTCTL => {
                // RX/TX FIFO flushes self-clear (both TX stagings drop);
                // a core soft reset restores the bank.
                if value & GRST_TXFFLSH != 0 {
                    self.txfifo.clear();
                    self.host_txfifo.clear();
                }
                if value & GRST_CSFTRST != 0 {
                    self.core_reset();
                } else {
                    self.regs[(GRSTCTL / 4) as usize] = value & !(GRST_RXFFLSH | GRST_TXFFLSH);
                }
            }
            // GINTSTS device bits are W1C (USBRST/ENUMDONE latched by the
            // auto-enum host; host-path bits are computed live, writes
            // ignored). DSTS/DAINT are read-only; EP0 interrupt flags are
            // W1C; GRXSTSP/HAINT are read-only pops/summaries; GUID/GSNPSID/
            // GHWCFG1..4 are read-only ID words.
            GINTSTS => {
                self.dev_gint &= !(value & (GINT_USBRST | GINT_ENUMDNE));
            }
            DSTS | DAINT | GRXSTSP | HAINT | GUID | GSNPSID | GHWCFG1 | GHWCFG2 | GHWCFG3
            | GHWCFG4 => {}
            // Device FIFO sizing (GRXFSIZ/DIEPTXF0/GDFIFOCFG): the TinyUSB
            // `handle_bus_reset` path (`dfifo_device_init` +
            // `dfifo_alloc(0x80,...)`) programs them during enumeration;
            // plain stores (silicon-observable for later sizing math).
            GRXFSIZ | DIEPTXF0 | GDFIFOCFG => {
                self.regs[(offset / 4) as usize] = value;
            }
            // Device endpoint interrupt masks (DAINTMSK/DOEPMSK/DIEPMSK):
            // programmed by `handle_bus_reset` (EP0 IN/OUT); plain stores.
            DAINTMSK | DOEPMSK | DIEPMSK => {
                self.regs[(offset / 4) as usize] = value;
            }
            // Device FIFO empty mask (DIEPEMPMSK @0x834): enables the TXFE
            // interrupt per IN endpoint (the TinyUSB slave path sets EP0's
            // bit while payload remains, clears it when done); plain store.
            DIEPEMPMSK => {
                self.regs[(offset / 4) as usize] = value;
            }
            DIEPINT0 | DOEPINT0 => {
                let w = (offset / 4) as usize;
                // TXFE (DIEPINT0 bit 7) is read-only live state (see
                // `dev_diepint0`): a W1C write must not latch it.
                let mask = if offset == DIEPINT0 {
                    value & !DIEPINT_TXFE
                } else {
                    value
                };
                self.regs[w] &= !mask;
            }
            HPRT => self.hprt_write(value),
            // Device-mode EP0 loopback (no external host): the firmware
            // itself stages a SETUP packet by writing its 8 bytes to DFIFO0
            // in device mode; the controller decodes it immediately (same
            // `handle_setup` table as the host path: GET_DESCRIPTOR stages
            // IN data, SET_ADDRESS arms, SET_CONFIGURATION records, else
            // STALL) and raises the matching EP0 interrupt flag
            // (DOEPINT0 XFRC for the SETUP stage, STALL on reject). The IN
            // data stage is served by reading DFIFO0 (device RXFIFO pop,
            // like silicon's EP0 IN FIFO); the status stage completes on
            // the DIEPTSIZ0/DOEPTSIZ0 xfer-size write like the host path's
            // ChEna. See the `dev_loopback_*` machine tests.
            DOEPTSIZ0 | DIEPTSIZ0 => {
                self.regs[(offset / 4) as usize] = value;
                // Loopback-only: during an auto-enum session the IN path
                // completes via `dev_in_maybe_complete` (TXFIFO push +
                // EPENA) and status via `usb_host_status_out` — running
                // the loopback status here would latch IN XFRC at
                // TSIZ-program time, before the firmware pushes any data
                // (the TinyUSB `edpt_schedule_packets` order is TSIZ,
                // EPENA, then FIFO push).
                if !self.dev_enum_session {
                    self.dev_loopback_status();
                }
            }
            DFIFO0 => {
                if self.host_mode() {
                    // Host-mode OUT/SETUP payload staging (1024-byte cap
                    // like the TXFIFO; overfill dropped).
                    if self.host_txfifo.len() + 4 <= TXFIFO_WORDS as usize * 4 {
                        self.host_txfifo.extend(value.to_le_bytes());
                    }
                } else if self.dev_enum_session {
                    // Auto-enum IN data stage: the firmware pushes its
                    // descriptor payload into TXFIFO0 (slave-mode TXFIFO
                    // push, like silicon's EP0 IN FIFO); completion fires
                    // once pushed words cover the programmed DIEPTSIZ0
                    // size (or immediately for a status ZLP) — see
                    // `dev_in_maybe_complete`. TXFIFO words pushed while
                    // NO transfer is armed (stale session, e.g. a second
                    // DFIFO write after completion re-arms nothing) are
                    // dropped: on silicon they would sit in the FIFO, but
                    // keeping them lets a LATER transfer complete against
                    // STALE bytes (observed live: the sketch's DFIFO debug
                    // reads after completion re-staged the same 8 bytes,
                    // and a re-armed transfer would have completed against
                    // the echo instead of fresh ISR payload).
                    if self.dev_in_armed {
                        if self.txfifo.len() + 4 <= TXFIFO_WORDS as usize * 4 {
                            self.txfifo.extend(value.to_le_bytes());
                        }
                        self.dev_in_maybe_complete();
                    }
                } else if !self.dev_setup_done {
                    // Device-mode EP0 loopback: accumulate the SETUP packet;
                    // once 8 bytes stage, decode via the shared table and
                    // raise the EP0 OUT transfer-complete (or STALL).
                    // After the SETUP stage completes, further DFIFO0
                    // writes are EP1-OUT payload (echoed on EP1-IN reads).
                    self.dev_setup.extend_from_slice(&value.to_le_bytes());
                    if self.dev_setup.len() >= 8 {
                        let mut pkt = [0u8; 8];
                        pkt.copy_from_slice(&self.dev_setup[..8]);
                        self.dev_setup.drain(..8);
                        self.dev_loopback_setup(&pkt);
                    }
                } else if self.dev_ep1_out.len() + 4 <= 256 {
                    self.dev_ep1_out.extend(value.to_le_bytes());
                }
            }
            o if o < (REG_WORDS * 4) as u32 && o.is_multiple_of(4) => {
                // Host-channel interrupt clear (W1C) and channel enable
                // (runs the transfer synchronously); device EP0 IN enable
                // (DIEPCTL0 EPENA) stages the IN payload for the auto-enum
                // host to capture; all other registers are plain stores.
                let w = (o / 4) as usize;
                if o >= HC_BASE && o < HC_BASE + HC_COUNT as u32 * HC_STRIDE {
                    let ch = ((o - HC_BASE) / HC_STRIDE) as usize;
                    let reg = o - (HC_BASE + ch as u32 * HC_STRIDE);
                    if reg == HCINT_OFF {
                        self.regs[w] &= !value;
                        return;
                    }
                    self.regs[w] = value;
                    if reg == HCCHAR_OFF {
                        if value & HCCHAR_CHDIS != 0 && value & HCCHAR_CHENA != 0 {
                            // Halt sequence: complete instantly halted.
                            self.regs[w] &= !(HCCHAR_CHENA | HCCHAR_CHDIS);
                            self.raise_hc(ch, HCINT_CHHALTED);
                        } else if value & HCCHAR_CHENA != 0 {
                            self.run_host_channel(ch);
                        }
                    }
                    return;
                }
                if o == DIEPCTL0 {
                    // EP0 IN enable: arm the transfer (slave-mode TXFIFO
                    // push follows via TXFE, like silicon). Completion
                    // fires once pushed bytes cover the programmed
                    // DIEPTSIZ0 size — or immediately when the FIFO
                    // already holds them (or the size is zero, a status
                    // ZLP). EPENA self-clears like silicon. TXFE is NOT
                    // latched here: it is live state (see `dev_txfe_live`)
                    // — the driver spins on it after EPENA, then pushes
                    // the payload the completion check consumes.
                    self.regs[w] = value & !DEPCTL_EPENA;
                    if value & DEPCTL_EPENA != 0 {
                        self.dev_in_armed = true;
                        self.dev_in_maybe_complete();
                    }
                    return;
                }
                if o == DOEPCTL0 {
                    // EP0 OUT enable: arm the status-OUT reception (the
                    // TinyUSB slave path enables EP0-OUT to receive the
                    // zero-length status packet). Like silicon, arrival
                    // fires XFRC; without a real packet the model completes
                    // it on the harness status-out call — but the EPENA
                    // self-clears now, like silicon clearing enable once
                    // the transfer is accepted.
                    self.regs[w] = value & !DEPCTL_EPENA;
                    return;
                }
                self.regs[w] = value;
            }
            _ => {}
        }
    }

    /// Device EP0 TXFE live state (DIEPINT0 bit 7): silicon raises it while
    /// the TXFIFO has space for the armed transfer — i.e. an IN transfer is
    /// armed (DIEPCTL0 EPENA seen) and the DIEPEMPMSK EP0 bit is set (the
    /// TinyUSB slave path enables it while payload remains). The TinyUSB
    /// `edpt_schedule_packets` → `epin_write_tx_fifo` path spins on TXFE
    /// before every word push, so without this the first word never lands
    /// and the IN transfer never completes (observed live: REQ0 staged,
    /// RXFLVL consumed, but DIEPINT0.XFRC never latched).
    fn dev_txfe_live(&self) -> bool {
        self.dev_in_armed && self.regs[(DIEPEMPMSK / 4) as usize] & DIEPEMPMSK_EP0 != 0
    }

    /// DIEPINT0 read: latched flags (XFRC/STALL/...) OR the live TXFE bit.
    fn dev_diepint0(&self) -> u32 {
        let mut v = self.regs[(DIEPINT0 / 4) as usize];
        if self.dev_txfe_live() {
            v |= DIEPINT_TXFE;
        }
        v
    }

    /// Device EP0 IN transfer completion check: when armed (DIEPCTL0 EPENA
    /// seen) and the TXFIFO holds at least the programmed DIEPTSIZ0 size,
    /// move the payload into the IN-capture queue (what the host reads off
    /// the wire) and latch XFRC. A zero programmed size completes at once
    /// (control status ZLP carries no payload).
    ///
    /// XFRSIZ DRAIN TIMING (measured live 2026-09-16, single-step ground
    /// truth — read before "optimizing" this): the drain below must happen
    /// SYNCHRONOUSLY in the DFIFO-push call that covers XFRSIZ (i.e. the
    /// transfer completes ~100 single-steps after the ISR programs TSIZ
    /// at +2316 and pushes the last word at +2423). An earlier revision
    /// deferred the drain and broke the rendezvous the other way (TSIZ
    /// stayed 8 forever, sketch passed sched but hung in done). The live
    /// sketch reaches its TSIZ poll ~8.9k steps after the POLL line the
    /// harness stages on (FreeRTOS println path), so a transfer that
    /// drains at +2423 is MISSED by the poll — that is a HARNESS
    /// rendezvous problem (stage later, from inside the poll), NOT a
    /// reason to delay this drain: holding XFRSIZ nonzero past completion
    /// would fake a scheduled state the ISR already retired (the ISR's
    /// `handle_ep_irq` consumed XFRC and rearmed the endpoint). Keep the
    /// drain synchronous; fix rendezvous in the sketch/harness.
    ///
    /// On completion the DIEPTSIZ0 XFRSIZ field is cleared to 0 (like
    /// silicon, which decrements the transfer-size counter as bytes go
    /// out; PKTCNT likewise drains). This is the race-free completion
    /// signal the sketch polls: DIEPINT0.XFRC is W1C-consumed by the
    /// TinyUSB ISR (`handle_ep_irq` snapshots then clears), so a
    /// sketch-side XFRC poll can miss the bit entirely when the ISR wins
    /// the race (observed live: model latched XFRC, harness captured the
    /// 8 IN bytes, but the sketch spun on DIEPINT0==0 forever because the
    /// ISR cleared it first). DIEPTSIZ0 is never touched by the ISR, so
    /// scheduled (XFRSIZ!=0) → done (XFRSIZ==0) is observable without a
    /// race.
    fn dev_in_maybe_complete(&mut self) {
        if !self.dev_in_armed {
            return;
        }
        // DIEPTSIZ0 packs XFRSIZ[18:0] + PKTCNT[28:19] (the TinyUSB
        // `edpt_schedule_packets` programs both: an 18-byte EP0 IN lands as
        // 18 | (1<<19)); only the byte count gates completion.
        let tsiz = (self.regs[(DIEPTSIZ0 / 4) as usize] & 0x7FFFF) as usize;
        if tsiz != 0 && self.txfifo.len() < tsiz {
            return;
        }
        // The transfer consumes whole TXFIFO words: an 18-byte payload is
        // pushed as 5 words (the last word's upper bytes are padding, like
        // silicon's word-wise TXFIFO push — only XFRSIZ bytes go on the
        // wire). Capture the payload, discard the padding. The payload is
        // ALSO mirrored for sketch-side DFIFO0 debug reads (see
        // `dev_in_mirror`): the sketch checks the same bytes the harness
        // asserts, from opposite ends of the identical transfer.
        let words = tsiz.div_ceil(4) * 4;
        let n = tsiz.min(self.txfifo.len());
        self.dev_in_mirror.clear();
        self.dev_in_mirror
            .extend(self.txfifo.iter().take(n).cloned());
        self.dev_in_capture.extend(self.txfifo.drain(..n));
        let pad = words.saturating_sub(n).min(self.txfifo.len());
        self.txfifo.drain(..pad);
        self.dev_in_armed = false;
        // XFRSIZ drains to 0 (silicon transfer-size counter); the full
        // PKTCNT field [28:19] likewise drains (single-packet EP0 transfer
        // done). XFRC still latches for the ISR path (unit tests +
        // `handle_ep_irq`).
        self.regs[(DIEPTSIZ0 / 4) as usize] &= !0x1FFF_FFFFu32;
        self.regs[(DIEPINT0 / 4) as usize] |= UsbOtg::EP_XFRC;
    }
    /// HPRT write: W1C status bits (connect/enable-change), stored control
    /// bits (power/reset/suspend), HW-owned live bits (connected/speed/
    /// enable, driven by port events below, never by firmware writes).
    fn hprt_write(&mut self, value: u32) {
        let i = (HPRT / 4) as usize;
        self.regs[i] &= !(value & HPRT_W1C);
        let prev = self.regs[i];
        // Store control bits; live bits are preserved as driven.
        let keep = HPRT_CONNSTS | HPRT_CONNDET | HPRT_ENA | HPRT_ENCHNG | (3 << 17);
        self.regs[i] = (prev & keep) | (value & !keep & !(1 << 8));
        // Power-up connects the simulated full-speed device.
        if value & HPRT_PWR != 0 && prev & HPRT_PWR == 0 && !self.dev.present {
            self.dev.present = true;
            self.dev.addr = 0;
            self.dev.configured = 0;
            self.regs[i] |= HPRT_CONNSTS | HPRT_CONNDET | HPRT_SPD_FS;
        }
        // Port reset (self-clearing): resets the device, then enables it.
        if value & HPRT_RST != 0 {
            self.dev.addr = 0;
            self.dev.addr_armed = false;
            self.dev.configured = 0;
            self.dev.in_data.clear();
            self.regs[i] |= HPRT_ENA | HPRT_ENCHNG;
            self.regs[i] &= !HPRT_RST;
        }
    }

    /// Raise host-channel interrupt bits (caller already decided).
    fn raise_hc(&mut self, ch: usize, bits: u32) {
        let i = self.hc_at(ch, HCINT_OFF);
        self.regs[i] |= bits;
    }

    /// Advance the SOF frame counter (called per SoC step while the USB
    /// controller clock runs).
    pub fn tick(&mut self) {
        self.sof = self.sof.wrapping_add(1);
    }

    /// Simulate a device disconnect observed by the host port (test/host
    /// frontend): drop ConnSts, disable the port, latch enable-change —
    /// the HPRT interrupt the stack polls for removal events.
    pub fn host_disconnect(&mut self) {
        let i = (HPRT / 4) as usize;
        self.dev.present = false;
        self.regs[i] &= !HPRT_CONNSTS;
        self.regs[i] &= !HPRT_ENA;
        self.regs[i] |= HPRT_ENCHNG;
    }

    /// Execute one host-channel transfer synchronously on ChEna (control
    /// transfers fully; other endpoint types complete instantly empty).
    /// ChEna clears and XferCompl (+ChHalted) latch like completed HW.
    fn run_host_channel(&mut self, ch: usize) {
        let ci = self.hc_at(ch, HCCHAR_OFF);
        let si = self.hc_at(ch, HCTSIZ_OFF);
        let hcchar = self.regs[ci];
        let hctsiz = self.regs[si];
        let out = hcchar & (1 << 15) == 0;
        let eptype = (hcchar >> 18) & 3;
        let devaddr = ((hcchar >> 22) & 0x7F) as u8;
        let xfer = (hctsiz & 0x7FFFF) as usize;
        let pid = (hctsiz >> 29) & 3;
        self.regs[ci] &= !HCCHAR_CHENA;
        // Address check (SETUP/IN/OUT all carry it): mismatches STALL.
        if !self.dev.present || devaddr != self.dev.addr {
            self.raise_hc(ch, HCINT_STALL | HCINT_CHHALTED);
            return;
        }
        if eptype != 0 {
            // Bulk/interrupt/iso: instant empty completion (documented).
            if !out {
                self.stage_rx(ch, pid as u8, &[]);
            }
            self.raise_hc(ch, HCINT_XFERCOMPL | HCINT_CHHALTED);
            return;
        }
        if pid == 3 {
            // SETUP stage: 8 packet bytes from the staged OUT FIFO.
            let mut pkt = [0u8; 8];
            for b in pkt.iter_mut() {
                *b = self.host_txfifo.pop_front().unwrap_or(0);
            }
            if self.handle_setup(ch, &pkt) {
                return; // STALL raised; no completion event.
            }
            self.raise_hc(ch, HCINT_XFERCOMPL | HCINT_CHHALTED);
            return;
        }
        if out {
            // OUT data/status: consume the payload; a completed status
            // stage applies a pending SET_ADDRESS.
            for _ in 0..xfer {
                self.host_txfifo.pop_front();
            }
            if self.dev.addr_armed {
                self.dev.addr = self.dev.addr_pending;
                self.dev.addr_armed = false;
            }
            self.raise_hc(ch, HCINT_XFERCOMPL | HCINT_CHHALTED);
            return;
        }
        // IN data/status: move the staged payload (possibly empty) into
        // the RXFIFO; a zero-length status also applies SET_ADDRESS.
        if xfer == 0 && self.dev.addr_armed {
            self.dev.addr = self.dev.addr_pending;
            self.dev.addr_armed = false;
        }
        let mut data = alloc::vec::Vec::new();
        while data.len() < xfer {
            if let Some(b) = self.dev.in_data.pop_front() {
                data.push(b);
            } else {
                break;
            }
        }
        self.stage_rx(ch, pid as u8, &data);
        self.raise_hc(ch, HCINT_XFERCOMPL | HCINT_CHHALTED);
    }

    /// Queue received bytes + packet status for DFIFO/GRXSTSP pops.
    fn stage_rx(&mut self, ch: usize, dpid: u8, data: &[u8]) {
        self.rx_data.extend(data.iter().cloned());
        self.rx_meta = Some((ch as u8, data.len().min(0x7FF) as u16, dpid));
    }

    /// Dispatch one 8-byte SETUP packet to the simulated device.
    /// Returns true when the endpoint STALLs (completion is skipped).
    fn handle_setup(&mut self, ch: usize, pkt: &[u8; 8]) -> bool {
        let rt = pkt[0];
        let req = pkt[1];
        let val = u16::from_le_bytes([pkt[2], pkt[3]]);
        let len = u16::from_le_bytes([pkt[6], pkt[7]]) as usize;
        match (rt, req) {
            // GET_DESCRIPTOR (IN): stage the descriptor bytes, truncated
            // to wLength like silicon (short reads are the norm: 8/9/18
            // first, then full).
            (0x80, 6) => {
                let src: &[u8] = match (val >> 8) as u8 {
                    1 => &DEV_DESC,
                    2 => &CFG_DESC,
                    3 => match (val & 0xFF) as u8 {
                        0 => &STR0_DESC,
                        1 => &STR1_DESC,
                        2 => &STR2_DESC,
                        3 => &STR3_DESC,
                        _ => &[],
                    },
                    _ => &[],
                };
                self.dev.in_data.clear();
                self.dev.in_data.extend(src.iter().take(len).cloned());
            }
            // SET_ADDRESS: applied when the status stage completes.
            (0x00, 5) => {
                self.dev.addr_pending = (val & 0x7F) as u8;
                self.dev.addr_armed = true;
            }
            // SET_CONFIGURATION: recorded.
            (0x00, 9) => {
                self.dev.configured = (val & 0xFF) as u8;
            }
            // Anything else: STALL the endpoint (no completion event).
            _ => {
                self.dev.in_data.clear();
                self.raise_hc(ch, HCINT_STALL | HCINT_CHHALTED);
                return true;
            }
        }
        false
    }

    // EP0 interrupt flag bits (usb_dwc DIEPINT0/DOEPINT0 layout):
    // XFRC[0] = transfer completed, STALL[3] = STALL response.
    const EP_XFRC: u32 = 1 << 0;
    const EP_STALL: u32 = 1 << 3;

    /// Device-mode EP0 loopback SETUP stage: decode the staged 8-byte
    /// packet through the shared `handle_setup` table (channel id unused
    /// there — endpoint 0), then mirror the staged IN payload into the
    /// device RXFIFO so DFIFO0 reads serve it, and raise the EP0 OUT
    /// transfer-complete (or STALL on reject). The firmware then drives
    /// the IN/status stages with DIEPTSIZ0/DOEPTSIZ0 writes (see
    /// `dev_loopback_status`).
    fn dev_loopback_setup(&mut self, pkt: &[u8; 8]) {
        // Reuse the host-path table with a dummy channel: HCINT_STALL on
        // reject would raise a host interrupt, so snapshot + clear any
        // host-channel side effect first (host channels are idle in
        // device mode; the raise only touches ch0's bits).
        let hc0 = self.regs[self.hc_at(0, HCINT_OFF)];
        let stalled = self.handle_setup(0, pkt);
        self.regs[self.hc_at(0, HCINT_OFF)] = hc0;
        // Mirror staged IN data into the device RXFIFO (DFIFO0 pops serve
        // it, like silicon's EP0 IN FIFO on the bus).
        self.rx_data.clear();
        self.rx_data.extend(self.dev.in_data.iter().cloned());
        if stalled {
            self.regs[(DOEPINT0 / 4) as usize] |= Self::EP_STALL;
        } else {
            self.regs[(DOEPINT0 / 4) as usize] |= UsbOtg::EP_XFRC;
            self.dev_setup_done = true;
        }
    }

    /// Device-mode EP0 status stage: a DIEPTSIZ0/DOEPTSIZ0 transfer-size
    /// write completes the handshake — applies a pending SET_ADDRESS
    /// (like the host path's status stage) and raises the EP0 IN
    /// transfer-complete. Zero-length either way (control status carries
    /// no payload on the loopback).
    fn dev_loopback_status(&mut self) {
        if self.host_mode() {
            return;
        }
        if self.dev.addr_armed {
            self.dev.addr = self.dev.addr_pending;
            self.dev.addr_armed = false;
        }
        self.regs[(DIEPINT0 / 4) as usize] |= UsbOtg::EP_XFRC;
    }

    /// Auto-enum session active (see `dev_enum_session`): device-mode
    /// DFIFO0 writes are IN-TXFIFO pushes while set.
    #[allow(dead_code)]
    fn dev_enum_active(&self) -> bool {
        self.dev_enum_session
    }

    /// Auto-enum host frontend: deliver one bus reset + enumeration-done
    /// (the silicon-true pair the DWC2 core raises on connect: the TinyUSB
    /// `dcd_int_handler` runs `handle_bus_reset` on USBRST — NAKs OUT EPs,
    /// masks EP0 IN/OUT, sizes EP0 at 64 B — then `handle_enum_done` on
    /// ENUMDNE, which reads the speed from DSTS and queues the bus-reset
    /// event the device task consumes). Both latch W1C; ENUMSPD reads
    /// full-speed while ENUMDONE is latched. Also clears any stale EP0
    /// IN/OUT latches first (a fresh reset retires in-flight stages, like
    /// silicon's FIFO flush in `handle_bus_reset`).
    pub fn usb_host_bus_reset(&mut self) {
        if !self.device_mode() {
            return;
        }
        self.regs[(DOEPINT0 / 4) as usize] = 0;
        self.regs[(DIEPINT0 / 4) as usize] = 0;
        self.txfifo.clear();
        self.rx_data.clear();
        self.dev_grxstsp.clear();
        self.dev_in_capture.clear();
        self.dev_in_mirror.clear();
        self.dev_in_armed = false;
        self.dev_setup_done = false;
        self.dev_enum_session = false;
        self.setup_done_armed = false;
        self.dev_gint |= GINT_USBRST | GINT_ENUMDNE;
    }

    /// Auto-enum host frontend: deliver one 8-byte SETUP packet from the
    /// host.
    ///
    /// Stages the 8 SETUP bytes in the RXFIFO (popped LE-first via DFIFO0,
    /// like silicon's EP0 OUT FIFO) plus the GRXSTSP SETUP_RX then
    /// SETUP_DONE pops. STPKTRCVD latches eagerly (the core posts it with
    /// the packet); SETUP (bit 3) latches when the SETUP_DONE GRXSTSP entry
    /// pops (see the GRXSTSP arm — the silicon retire side effect the
    /// TinyUSB `handle_epout_slave` consumes via `setup_phase_done`).
    /// GINTSTS RXFLVL + OEPINT read live while the queue/flags are pending.
    /// The RXFIFO drains through DFIFO0 reads; the GRXSTSP pops only report
    /// the packet (like silicon, the bytes travel with the RXFLVL event,
    /// not the status word).
    ///
    /// DWC2 SINGLE-TRANSACTION NOTE (measured live 2026-09-16): the staged
    /// SETUP is consumed by the TinyUSB ISR ~2k single-steps after stage
    /// (`dcd_int_handler` RXFLVL → `handle_setup` → `edpt_schedule_packets`
    /// programs DIEPTSIZ0=8 at +1957, the IN payload drains it at +2064),
    /// so the packet must linger ~2k steps. A sketch that polls DIEPTSIZ0
    /// reaches its wait loop only ~8.9k steps after printing its banner
    /// (FreeRTOS `println` + queue round-trip), so staging on the banner
    /// completes the whole transfer ~6.8k steps before the poll starts —
    /// the scheduled wait misses it every time (FAIL IN SCHED with a
    /// healthy model). Stage strictly on the sketch's POLL-RDV line,
    /// printed from inside the wait (see the usb_device sketch), never on
    /// an earlier banner.
    pub fn usb_host_setup(&mut self, pkt: [u8; 8]) {
        if !self.device_mode() {
            return;
        }
        self.dev_enum_session = true;
        self.rx_data.extend(pkt.iter().cloned());
        // GRXSTSP pops use the silicon field layout (dwc2_type.h
        // GRXSTSP_EPNUM/BCNT/DPID/PKTSTS_Pos = 0/4/15/17): BCNT = 8 SETUP
        // bytes, DPID = DATA0 (1) for a SETUP packet (the TinyUSB RXFLVL
        // handler doesn't check it, but the sketch-side status decode
        // does — a 0 there reads as a different PID).
        let rx_word = (8u32 << 4) | (1u32 << 15) | (GRXSTSP_PKTSTS_SETUP_RX << 17);
        let done_word = (1u32 << 15) | (GRXSTSP_PKTSTS_SETUP_DONE << 17);
        self.dev_grxstsp.push_back(rx_word);
        self.dev_grxstsp.push_back(done_word);
        self.regs[(DOEPINT0 / 4) as usize] |= DOEPINT_STPKTRX;
        self.regs[(DOEPINT0 / 4) as usize] &= !DOEPINT_SETUP;
        self.setup_done_armed = true;
    }

    /// Auto-enum host frontend: complete the OUT status stage (zero-length
    /// status OUT after an IN data stage, or the SET_ADDRESS status). Pops
    /// like silicon's RX_COMPLETE + Transfer Completed: latches DOEPINT0
    /// XFRC (W1C) with DAINT OUT EP0 live from the latch, and applies a
    /// pending SET_ADDRESS (status-stage completion semantics, same as the
    /// host/loopback paths).
    pub fn usb_host_status_out(&mut self) {
        if self.dev.addr_armed {
            self.dev.addr = self.dev.addr_pending;
            self.dev.addr_armed = false;
        }
        self.dev_enum_session = false;
        self.regs[(DIEPINT0 / 4) as usize] |= UsbOtg::EP_XFRC;
        self.regs[(DOEPINT0 / 4) as usize] |= UsbOtg::EP_XFRC;
    }

    /// DFIFO-page access for IN endpoint `ep` (soc.rs routes EPn-IN @
    /// +0x1000*n): EP0 behaves exactly like DFIFO0 in the core window
    /// (loopback accumulator / auto-enum TXFIFO push); non-zero EPs stage
    /// per-endpoint TX bytes for the EP1-echo-style IN path (each EP has
    /// its own TX queue; reads serve the matching queue).
    pub fn write_dfifo(&mut self, ep: usize, off: u32, value: u32) {
        if ep == 0 {
            self.write32(DFIFO0, value);
            return;
        }
        if off < 0x1000 && self.ep_txfifo(ep).len() + 4 <= TXFIFO_WORDS as usize * 4 {
            let q = self.ep_txfifo(ep);
            q.extend(value.to_le_bytes());
        }
    }

    /// DFIFO-page read for IN endpoint `ep` (soc.rs routes EPn-IN @
    /// +0x1000*n): EP0 behaves like DFIFO0; non-zero EPs pop their own TX
    /// queue (empty reads 0).
    pub fn read_dfifo(&mut self, ep: usize, _off: u32) -> u32 {
        if ep == 0 {
            return self.read32(DFIFO0);
        }
        let mut w = 0u32;
        // Borrow dance: pop from the endpoint queue without holding the
        // borrow across the loop.
        for i in 0..4 {
            let b = self.ep_txfifo(ep).pop_front();
            if let Some(b) = b {
                w |= (b as u32) << (8 * i);
            }
        }
        w
    }

    /// Per-endpoint IN TX queue (EP1+; EP0 uses the shared `txfifo`).
    fn ep_txfifo(&mut self, ep: usize) -> &mut alloc::collections::VecDeque<u8> {
        let idx = (ep - 1).min(6);
        &mut self.ep_tx[idx]
    }

    /// Drain bytes the firmware pushed for IN stages (device answering the
    /// host): DFIFO0 reads that served staged IN payload (loopback mirror
    /// or auto-enum TXFIFO completions). The harness asserts the firmware
    /// answered from its own descriptors.
    pub fn usb_host_take_in(&mut self) -> alloc::vec::Vec<u8> {
        core::mem::take(&mut self.dev_in_capture)
    }
}

impl Default for UsbOtg {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn core_soft_reset_handshake() {
        let mut u = UsbOtg::new();
        // AHBIDL reads set out of reset.
        assert_ne!(u.read32(GRSTCTL) & GRST_AHBIDLE, 0);
        // Dirty some state, then reset: bank restores, bit self-clears.
        u.write32(DCFG, 0x1234_5678);
        u.write32(DFIFO0, 0xDEAD_BEEF);
        u.write32(GRSTCTL, GRST_CSFTRST);
        assert_eq!(u.read32(GRSTCTL) & GRST_CSFTRST, 0, "self-clears");
        assert_ne!(u.read32(GRSTCTL) & GRST_AHBIDLE, 0, "idle after reset");
        assert_eq!(u.read32(DCFG), 0, "bank restored");
        assert_eq!(u.read32(DTXFSTS0), TXFIFO_WORDS, "FIFO drained");
        assert!(!u.int_pending(), "quiet without a host");
    }

    #[test]
    fn device_config_and_ep0_round_trip() {
        let mut u = UsbOtg::new();
        u.write32(GUSBCFG, 1 << 30); // forcedevmode
        assert_eq!(u.read32(GUSBCFG) >> 30, 1);
        u.write32(DCFG, 3); // full-speed device
        assert_eq!(u.read32(DCFG) & 3, 3);
        u.write32(DCTL, 1 << 1); // soft disconnect
        assert_ne!(u.read32(DCTL) & (1 << 1), 0);
        u.write32(DCTL, 0);
        assert_eq!(u.read32(DCTL) & (1 << 1), 0, "reconnect");
        // EP0: activate + 64-byte MPS both directions.
        u.write32(DIEPCTL0, 1 << 15);
        u.write32(DOEPCTL0, 1 << 15);
        assert_ne!(u.read32(DIEPCTL0) & (1 << 15), 0);
        assert_ne!(u.read32(DOEPCTL0) & (1 << 15), 0);
        // No bus events: device/endpoint status reads 0.
        assert_eq!(u.read32(GINTSTS), 0);
        assert_eq!(u.read32(DSTS), 0);
        assert_eq!(u.read32(DAINT), 0);
        assert_eq!(u.read32(GRXSTSP), 0);
    }

    #[test]
    fn txfifo_staging_reports_space() {
        let mut u = UsbOtg::new();
        // Device mode (reset default): DFIFO0 writes stage the EP0 SETUP
        // packet, not the TXFIFO — space stays full.
        assert_eq!(u.read32(DTXFSTS0), TXFIFO_WORDS);
        u.write32(GUSBCFG, GUSBCFG_FORCE_HOST); // host mode stages TXFIFO
        for i in 0..4u32 {
            u.write32(DFIFO0, 0x1111_1111 * (i + 1));
        }
        assert_eq!(u.read32(DTXFSTS0), TXFIFO_WORDS - 4);
        assert_eq!(u.read32(GNPTXSTS) & 0xFFFF, TXFIFO_WORDS - 4);
        // TX flush drops staged bytes.
        u.write32(GRSTCTL, GRST_TXFFLSH | (1 << 6));
        assert_eq!(u.read32(DTXFSTS0), TXFIFO_WORDS);
    }

    const HCCH: u32 = HC_BASE;
    const HCTS: u32 = HC_BASE + HCTSIZ_OFF;
    const HCIN: u32 = HC_BASE + HCINT_OFF;

    /// Force host mode + power the port (connects the simulated device).
    fn host_up(u: &mut UsbOtg) {
        u.write32(GUSBCFG, GUSBCFG_FORCE_HOST);
        u.write32(HPRT, HPRT_PWR);
        assert_ne!(u.read32(HPRT) & HPRT_CONNSTS, 0, "connected");
        assert_ne!(u.read32(HPRT) & HPRT_CONNDET, 0, "connect detected");
        assert_eq!((u.read32(HPRT) >> 17) & 3, 1, "full-speed");
        u.write32(HPRT, HPRT_RST | HPRT_PWR);
        assert_eq!(u.read32(HPRT) & HPRT_RST, 0, "reset self-clears");
        assert_ne!(u.read32(HPRT) & HPRT_ENA, 0, "enabled");
        assert_ne!(u.read32(HPRT) & HPRT_ENCHNG, 0, "enable changed");
        assert_ne!(u.read32(GINTSTS) & GINT_PRTINT, 0, "port interrupt");
        u.write32(HPRT, HPRT_ENCHNG | HPRT_CONNDET | HPRT_PWR); // W1C
        assert_eq!(u.read32(GINTSTS) & GINT_PRTINT, 0, "port int clears");
    }

    /// Program channel 0 (control, EP0, MPS 64) and run one transfer.
    fn hc_xfer(u: &mut UsbOtg, addr: u8, out: bool, pid: u32, xfer: u32) {
        u.write32(HCTS, xfer | (1 << 19) | (pid << 29));
        let dir = if out { 0 } else { 1 << 15 };
        u.write32(HCCH, 64 | dir | ((addr as u32) << 22) | HCCHAR_CHENA);
    }

    fn hc_flags(u: &mut UsbOtg) -> u32 {
        u.read32(HCIN)
    }

    fn hc_clear(u: &mut UsbOtg) {
        u.write32(HCIN, HCINT_XFERCOMPL | HCINT_CHHALTED | HCINT_STALL);
    }

    /// Full control read: SETUP(8B staged) + IN(xfer B) + zero-length OUT.
    fn ctrl_read(u: &mut UsbOtg, addr: u8, setup: [u32; 2], xfer: u32) -> alloc::vec::Vec<u8> {
        u.write32(DFIFO0, setup[0]);
        u.write32(DFIFO0, setup[1]);
        hc_xfer(u, addr, true, 3, 8); // SETUP
        assert_ne!(hc_flags(u) & HCINT_XFERCOMPL, 0, "setup completes");
        hc_clear(u);
        hc_xfer(u, addr, false, 2, xfer); // IN DATA1
        assert_ne!(hc_flags(u) & HCINT_XFERCOMPL, 0, "in completes");
        hc_clear(u);
        let meta = u.read32(GRXSTSP);
        assert_eq!((meta >> 4) & 0x7FF, xfer.min(512), "rx byte count");
        let mut out = alloc::vec::Vec::new();
        for _ in 0..xfer.div_ceil(4) {
            out.extend_from_slice(&u.read32(DFIFO0).to_le_bytes());
        }
        out.truncate(xfer as usize);
        hc_xfer(u, addr, true, 2, 0); // OUT status
        assert_ne!(hc_flags(u) & HCINT_XFERCOMPL, 0, "status completes");
        hc_clear(u);
        out
    }

    #[test]
    fn host_port_connect_and_reset() {
        let mut u = UsbOtg::new();
        assert_eq!(u.read32(GINTSTS), 0, "quiet before attach");
        host_up(&mut u);
        assert!(u.dev.present);
        assert_eq!(u.dev.addr, 0);
    }

    #[test]
    fn host_get_descriptor_device() {
        let mut u = UsbOtg::new();
        host_up(&mut u);
        // GET_DESCRIPTOR device: rt=0x80 req=6 val=0x0100 len=18.
        let setup = [0x01000680u32, 0x00120000u32]; // LE words of the 8B packet
        let desc = ctrl_read(&mut u, 0, setup, 18);
        assert_eq!(desc.len(), 18);
        assert_eq!(desc[0], 18, "bLength");
        assert_eq!(desc[1], 1, "DEVICE");
        assert_eq!(
            u16::from_le_bytes([desc[8], desc[9]]),
            0x303A,
            "Espressif VID"
        );
        assert_eq!(u16::from_le_bytes([desc[10], desc[11]]), 0x1001, "PID");
        assert_eq!(u.read32(GINTSTS) & GINT_RXFLVL, 0, "rx drained");
    }

    #[test]
    fn host_set_address_then_config() {
        let mut u = UsbOtg::new();
        host_up(&mut u);
        // SET_ADDRESS(7): SETUP + zero-length IN status applies it.
        u.write32(DFIFO0, 0x00070500);
        u.write32(DFIFO0, 0x00000000);
        hc_xfer(&mut u, 0, true, 3, 8);
        hc_clear(&mut u);
        hc_xfer(&mut u, 0, false, 2, 0);
        assert_ne!(hc_flags(&mut u) & HCINT_XFERCOMPL, 0);
        hc_clear(&mut u);
        assert_eq!(u.dev.addr, 7, "address applied at status");
        // Old address now STALLs.
        hc_xfer(&mut u, 0, false, 2, 8);
        assert_ne!(hc_flags(&mut u) & HCINT_STALL, 0, "stale addr stalls");
        assert_eq!(hc_flags(&mut u) & HCINT_XFERCOMPL, 0, "no completion");
        hc_clear(&mut u);
        // GET_DESCRIPTOR config at the new address (full 32 B).
        let setup = [0x02000680u32, 0x00200000u32];
        let cfg = ctrl_read(&mut u, 7, setup, 32);
        assert_eq!(cfg[0], 9, "config length");
        assert_eq!(cfg[1], 2, "CONFIGURATION");
        assert_eq!(cfg[2], 32, "total length");
        assert_eq!(cfg[13], 2, "two endpoints");
        // SET_CONFIGURATION(1): SETUP + IN status.
        u.write32(DFIFO0, 0x00010900);
        u.write32(DFIFO0, 0x00000000);
        hc_xfer(&mut u, 7, true, 3, 8);
        hc_clear(&mut u);
        hc_xfer(&mut u, 7, false, 2, 0);
        hc_clear(&mut u);
        assert_eq!(u.dev.configured, 1);
    }
}

#[cfg(test)]
mod dev_loopback_tests {
    use super::*;

    // Device-mode EP0 loopback: stage a GET_DESCRIPTOR(DEVICE) SETUP via
    // DFIFO0, expect DOEPINT0 XFRC; DFIFO0 reads serve the 18-byte device
    // descriptor; the transfer-size write completes status (DIEPINT0 XFRC).
    #[test]
    fn dev_loopback_get_descriptor_device() {
        let mut u = UsbOtg::new();
        // Write the 8 SETUP bytes as two LE words: [80 06 00 01 | 00 00 12 00]
        // = GET_DESCRIPTOR device, wLength 18.
        u.write32(DFIFO0, 0x0100_0680);
        u.write32(DFIFO0, 0x0012_0000);
        assert_ne!(u.read32(DOEPINT0) & 1, 0, "SETUP XFRC");
        // IN data stage: 18 descriptor bytes pop via DFIFO0.
        let mut got = [0u8; 18];
        for chunk in got.chunks_mut(4) {
            let w = u.read32(DFIFO0);
            for (i, b) in chunk.iter_mut().enumerate() {
                *b = ((w >> (8 * i)) & 0xFF) as u8;
            }
        }
        assert_eq!(&got, &DEV_DESC, "device descriptor round-trip");
        // Status stage: transfer-size write raises IN XFRC.
        u.write32(DIEPTSIZ0, 0);
        assert_ne!(u.read32(DIEPINT0) & 1, 0, "status XFRC");
        u.write32(DIEPINT0, 1);
        u.write32(DOEPINT0, 1);
        assert_eq!(u.read32(DIEPINT0) & 1, 0, "W1C clears");
        assert_eq!(u.read32(DOEPINT0) & 1, 0, "W1C clears");
    }

    // Unknown SETUP request STALLs (DOEPINT0 STALL, no XFRC); SET_ADDRESS
    // applies at the status-stage write.
    #[test]
    fn dev_loopback_stall_and_set_address() {
        let mut u = UsbOtg::new();
        // Vendor request 0xFF: not handled -> STALL.
        u.write32(DFIFO0, 0x0000_FFC0);
        u.write32(DFIFO0, 0x0000_0000);
        assert_ne!(u.read32(DOEPINT0) & (1 << 3), 0, "STALL latched");
        assert_eq!(u.read32(DOEPINT0) & 1, 0, "no XFRC on STALL");
        u.write32(DOEPINT0, 1 << 3);
        // SET_ADDRESS 7: addr applies at the status write.
        u.write32(DFIFO0, 0x0007_0000 | ((0x05u32) << 8));
        u.write32(DFIFO0, 0x0000_0000);
        assert_ne!(u.read32(DOEPINT0) & 1, 0, "SETUP XFRC");
        u.write32(DOEPTSIZ0, 0);
        assert_eq!(u.dev.addr, 7, "address applied at status");
    }
}

#[cfg(test)]
mod dev_ep1_tests {
    use super::*;

    // Device EP1 echo: after the EP0 SETUP stage, DFIFO0 writes stage OUT
    // payload; DFIFO0 reads past the EP0 IN payload serve it back (echo),
    // proving bulk/interrupt-style data movement without a host.
    #[test]
    fn dev_ep1_out_payload_echoes_on_in_reads() {
        let mut u = UsbOtg::new();
        // SETUP: GET_DESCRIPTOR device wLength 0 (no IN payload to drain).
        u.write32(DFIFO0, 0x0100_0680);
        u.write32(DFIFO0, 0x0000_0000);
        assert_ne!(u.read32(DOEPINT0) & 1, 0, "SETUP XFRC");
        u.write32(DOEPINT0, 1);
        assert_eq!(u.read32(DOEPINT0) & 1, 0, "XFRC clears");
        // OUT payload: two words staged to EP1.
        u.write32(DFIFO0, 0xDDCC_BBAA);
        u.write32(DFIFO0, 0x4433_2211);
        // IN reads echo the staged payload back LE-first.
        assert_eq!(u.read32(DFIFO0), 0xDDCC_BBAA);
        assert_eq!(u.read32(DFIFO0), 0x4433_2211);
        assert_eq!(u.read32(DFIFO0), 0, "queue drains to empty");
    }
}

#[cfg(test)]
mod dev_auto_enum_tests {
    use super::*;

    // Auto-enum bus reset: the host frontend latches USBRST + ENUMDONE
    // (W1C); DSTS reports full-speed ENUMSPD while ENUMDONE is latched;
    // GINTSTS shows both; source 38 pends once GINTMSK is programmed.
    fn dev_mode(u: &mut UsbOtg) {
        u.write32(GUSBCFG, GUSBCFG_FORCE_DEVICE);
        assert_ne!(u.read32(GUSBCFG) & GUSBCFG_FORCE_DEVICE, 0, "FDMOD latched");
    }

    #[test]
    fn auto_enum_bus_reset_latches_usbrst_enumdone() {
        let mut u = UsbOtg::new();
        dev_mode(&mut u);
        assert_eq!(u.read32(GINTSTS), 0, "quiet before reset");
        assert_eq!(u.read32(DSTS), 0, "no speed before reset");
        u.usb_host_bus_reset();
        assert_ne!(u.read32(GINTSTS) & GINT_USBRST, 0, "USBRST latched");
        assert_ne!(u.read32(GINTSTS) & GINT_ENUMDNE, 0, "ENUMDNE latched");
        assert_eq!(u.read32(DSTS) & 0x6, DSTS_ENUMSPD_FS, "full-speed");
        // Masked into source 38 once the firmware programs GINTMSK
        // (TinyUSB enables USBRST/ENUMDNE masks at init).
        u.write32(GINTMSK, GINT_USBRST | GINT_ENUMDNE);
        assert!(u.int_pending(), "source 38 pends");
        // W1C clear like the TinyUSB ISR (`dwc2->gintsts = GINTSTS_USBRST`).
        u.write32(GINTSTS, GINT_USBRST | GINT_ENUMDNE);
        assert_eq!(
            u.read32(GINTSTS) & (GINT_USBRST | GINT_ENUMDNE),
            0,
            "W1C clears"
        );
        assert!(!u.int_pending(), "source 38 quiet after clear");
        assert_eq!(u.read32(DSTS), 0, "speed clears with ENUMDONE");
    }

    // Auto-enum SETUP delivery: GRXSTSP pops SETUP_RX then SETUP_DONE,
    // the 8 SETUP bytes pop via DFIFO, STPKTRCVD latches at stage time
    // while SETUP latches on the SETUP_DONE pop (the silicon retire side
    // effect), DAINT shows OUT EP0, and GINTSTS shows RXFLVL + OEPINT.
    #[test]
    fn auto_enum_setup_posts_grxstsp_and_ep0_flags() {
        let mut u = UsbOtg::new();
        dev_mode(&mut u);
        // GET_DESCRIPTOR device wLength 18: [80 06 00 01 00 00 12 00].
        u.usb_host_setup([0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]);
        assert_ne!(u.read32(GINTSTS) & GINT_RXFLVL, 0, "RXFLVL live");
        // STPKTRCVD latches at stage time; SETUP only after the DONE pop.
        assert_ne!(u.read32(DOEPINT0) & DOEPINT_STPKTRX, 0, "STPKTRCVD staged");
        assert_eq!(
            u.read32(DOEPINT0) & DOEPINT_SETUP,
            0,
            "no SETUP before DONE pop"
        );
        let rx = u.read32(GRXSTSP);
        assert_eq!((rx >> 17) & 0xF, GRXSTSP_PKTSTS_SETUP_RX, "SETUP_RX first");
        assert_eq!((rx >> 4) & 0x7FF, 8, "8 SETUP bytes");
        let done = u.read32(GRXSTSP);
        assert_eq!(
            (done >> 17) & 0xF,
            GRXSTSP_PKTSTS_SETUP_DONE,
            "SETUP_DONE second"
        );
        assert_eq!(u.read32(GRXSTSP), 0, "queue drains");
        assert_eq!(
            u.read32(GINTSTS) & GINT_RXFLVL,
            0,
            "RXFLVL clears with queue"
        );
        // SETUP bytes pop LE-first via DFIFO0.
        assert_eq!(u.read32(DFIFO0), 0x0100_0680);
        assert_eq!(u.read32(DFIFO0), 0x0012_0000);
        // EP0 OUT flags + DAINT + OEPINT all live from the latch.
        assert_ne!(u.read32(DOEPINT0) & DOEPINT_STPKTRX, 0, "STPKTRCVD");
        assert_ne!(
            u.read32(DOEPINT0) & DOEPINT_SETUP,
            0,
            "SETUP after DONE pop"
        );
        assert_ne!(u.read32(DAINT) & DAINT_OUT_EP0, 0, "DAINT OUT EP0");
        assert_ne!(u.read32(GINTSTS) & GINT_OEPINT, 0, "OEPINT live");
        // W1C clears (TinyUSB clears DOEPINT0 after handling).
        u.write32(DOEPINT0, DOEPINT_STPKTRX | DOEPINT_SETUP);
        assert_eq!(
            u.read32(DOEPINT0) & (DOEPINT_STPKTRX | DOEPINT_SETUP),
            0,
            "W1C clears"
        );
        assert_eq!(
            u.read32(DAINT) & DAINT_OUT_EP0,
            0,
            "DAINT clears with latch"
        );
    }

    // Auto-enum IN transfer: firmware TXFIFO push + DIEPTSIZ0 size +
    // DIEPCTL0 EPENA captures the payload, latches XFRC, drives DAINT IN
    // EP0 + GINTSTS IEPINT, and self-clears EPENA. TXFE is live state
    // (not a latch): visible while the transfer is armed with DIEPEMPMSK
    // EP0 set, clearing when the transfer completes (disarmed).
    #[test]
    fn auto_enum_in_transfer_captures_txfifo_and_raises_xfrc() {
        let mut u = UsbOtg::new();
        dev_mode(&mut u);
        u.usb_host_setup([0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]);
        // TinyUSB `edpt_schedule_packets` order (TSIZ first, then EPENA,
        // then the TXFE-driven FIFO push): an 18-byte EP0 IN programs
        // XFRSIZ=18|PKTCNT=1<<19, arms, then pushes the payload. The
        // driver enables DIEPEMPMSK EP0 while payload remains (the TXFE
        // spin in `epin_write_tx_fifo`).
        u.write32(DIEPTSIZ0, 18 | (1 << 19));
        u.write32(DIEPCTL0, DEPCTL_EPENA);
        assert_eq!(u.read32(DIEPCTL0) & DEPCTL_EPENA, 0, "EPENA self-clears");
        assert_eq!(
            u.read32(DIEPINT0) & UsbOtg::EP_XFRC,
            0,
            "no XFRC before push"
        );
        u.write32(DIEPEMPMSK, DIEPEMPMSK_EP0);
        assert_ne!(
            u.read32(DIEPINT0) & DIEPINT_TXFE,
            0,
            "TXFE live while armed"
        );
        // Five TXFIFO pushes (18 bytes): completion fires once pushed
        // bytes cover XFRSIZ, independent of the PKTCNT field (the
        // transfer consumes whole TXFIFO words — the 18-byte payload
        // occupies 5 words, the last word's padding discarded with the
        // payload, like silicon's word-wise TXFIFO eject; DTXFSTS reads
        // full once the transfer completes).
        u.write32(DFIFO0, 0x0403_0201);
        u.write32(DFIFO0, 0x0807_0605);
        u.write32(DFIFO0, 0x0C0B_0A09);
        u.write32(DFIFO0, 0x100F_0E0D);
        assert_eq!(
            u.read32(DTXFSTS0),
            TXFIFO_WORDS - 4,
            "16 B staged over 4 words"
        );
        assert_ne!(
            u.read32(DIEPINT0) & DIEPINT_TXFE,
            0,
            "TXFE still live mid-push"
        );
        u.write32(DFIFO0, 0x0000_1211);
        assert_eq!(
            u.read32(DTXFSTS0),
            TXFIFO_WORDS,
            "transfer completes, TXFIFO drains"
        );
        assert_ne!(u.read32(DIEPINT0) & UsbOtg::EP_XFRC, 0, "XFRC latched");
        assert_eq!(
            u.read32(DIEPINT0) & DIEPINT_TXFE,
            0,
            "TXFE clears at completion"
        );
        // XFRSIZ drains to 0 on completion (silicon transfer-size counter;
        // the race-free completion signal the sketch polls — DIEPINT0.XFRC
        // is W1C-consumed by the ISR and can be missed by a racing poller).
        assert_eq!(
            u.read32(DIEPTSIZ0) & 0x7FFFF,
            0,
            "XFRSIZ drains at completion"
        );
        assert_ne!(u.read32(DAINT) & DAINT_IN_EP0, 0, "DAINT IN EP0");
        assert_ne!(u.read32(GINTSTS) & GINT_IEPINT, 0, "IEPINT live");
        // Captured payload is what the host reads off the wire.
        assert_eq!(
            u.usb_host_take_in(),
            alloc::vec![
                1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18
            ]
        );
        assert!(u.usb_host_take_in().is_empty(), "capture drains");
        u.write32(DIEPINT0, UsbOtg::EP_XFRC);
        assert_eq!(u.read32(DAINT) & DAINT_IN_EP0, 0, "DAINT clears with latch");
    }

    // Auto-enum status OUT: latches both XFRC flags (like silicon's
    // RX_COMPLETE + Transfer Completed pair) and applies a pending
    // SET_ADDRESS armed by the loopback path.
    #[test]
    fn auto_enum_status_out_raises_xfrc_pair() {
        let mut u = UsbOtg::new();
        dev_mode(&mut u);
        u.dev.addr_pending = 7;
        u.dev.addr_armed = true;
        u.usb_host_status_out();
        assert_ne!(u.read32(DOEPINT0) & UsbOtg::EP_XFRC, 0, "OUT XFRC");
        assert_ne!(u.read32(DIEPINT0) & UsbOtg::EP_XFRC, 0, "IN XFRC");
        assert_eq!(u.dev.addr, 7, "address applied at status");
        u.write32(DOEPINT0, UsbOtg::EP_XFRC);
        u.write32(DIEPINT0, UsbOtg::EP_XFRC);
        assert_eq!(u.read32(DOEPINT0) & UsbOtg::EP_XFRC, 0, "W1C clears");
    }

    // Loopback regression: the firmware-driven loopback still works with
    // the auto-enum bits present (no STPKTRCVD latched, so DFIFO0 writes
    // accumulate SETUP as before and the capture only sees IN reads).
    #[test]
    fn loopback_unaffected_without_enum_session() {
        let mut u = UsbOtg::new();
        u.write32(DFIFO0, 0x0100_0680);
        u.write32(DFIFO0, 0x0012_0000);
        assert_ne!(u.read32(DOEPINT0) & 1, 0, "SETUP XFRC");
        let mut got = [0u8; 18];
        for chunk in got.chunks_mut(4) {
            let w = u.read32(DFIFO0);
            for (i, b) in chunk.iter_mut().enumerate() {
                *b = ((w >> (8 * i)) & 0xFF) as u8;
            }
        }
        assert_eq!(&got, &DEV_DESC, "descriptor round-trip");
        assert_eq!(
            u.usb_host_take_in(),
            alloc::vec::Vec::from(DEV_DESC),
            "IN reads captured"
        );
    }
}
