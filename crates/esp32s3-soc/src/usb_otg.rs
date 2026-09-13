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
//! KNOWN LIMITATIONS: real host enumeration (bus reset, external SETUP/
//! IN/OUT from a host counterparty) is out of scope; all `Serial` flows
//! through validated USB-Serial-JTAG, so Arduino "USB CDC On Boot" stays
//! unsupported. Device bulk/interrupt transfers beyond the loopback echo
//! (no STALL matrix, no scheduling/DMA) stay out. Host-side
//! approximations: transfers complete instantly (no SOF/frame timing;
//! HFNUM reads 0); R1b busy (port reset, SWITCH-like waits) is
//! synchronous; DATA-toggle PID is accepted, not enforced; non-control
//! endpoint types complete instantly with zeros; bulk/periodic schedules,
//! SOF, split transactions and DMA (HCDMA) are not modeled; disconnect
//! is not modeled (the device stays connected once powered). DSTS/DAINT/
//! EPINT device interrupts still read 0; HW config ID registers read 0.

/// USB-OTG DWC register-block base (`dwc2_esp32.h` `DWC2_FS_REG_BASE`).
pub const USB_OTG_BASE: u32 = 0x6008_0000;
/// Second page (DFIFO0 @ +0x1000 lives past the first 4 KB page).
pub const USB_OTG_FIFO_PAGE: u32 = 0x6008_1000;
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
#[allow(dead_code)]
const DAINTMSK: u32 = 0x81C;
#[allow(dead_code)]
const DIEPCTL0: u32 = 0x900;
const DIEPINT0: u32 = 0x908;
#[allow(dead_code)]
const DIEPTSIZ0: u32 = 0x910;
const DTXFSTS0: u32 = 0x914;
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
        self.regs[(GRSTCTL / 4) as usize] = GRST_AHBIDLE;
    }

    fn tx_space(&self) -> u32 {
        let used = (self.txfifo.len() + self.host_txfifo.len()) as u32;
        TXFIFO_WORDS - used.div_ceil(4).min(TXFIFO_WORDS)
    }

    /// Force-host mode latched (sketch programs GUSBCFG bit 29).
    fn host_mode(&self) -> bool {
        self.regs[(GUSBCFG / 4) as usize] & GUSBCFG_FORCE_HOST != 0
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
    fn host_gintsts(&self) -> u32 {
        let mut v = 0;
        if !self.rx_data.is_empty() {
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
        v
    }

    /// Interrupt pending = GINTSTS & GINTMSK (source 38).
    pub fn int_pending(&self) -> bool {
        (self.host_gintsts() & self.regs[(GINTMSK / 4) as usize]) != 0
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            GRSTCTL => self.regs[(GRSTCTL / 4) as usize],
            // No bus events without a host: device/endpoint interrupt and
            // packet-status registers read 0; RXFIFO pops read 0 (empty).
            GINTSTS => self.host_gintsts(),
            DSTS | DAINT => 0,
            // Device-mode EP0 interrupt flags: the loopback driver (see
            // `dev_loopback_*`) raises XFRC on each completed stage; STALL
            // latches when the SETUP packet is not a handled standard
            // request. W1C via the matching INT register write.
            DIEPINT0 => self.regs[(DIEPINT0 / 4) as usize],
            DOEPINT0 => self.regs[(DOEPINT0 / 4) as usize],
            // GRXSTSP pops the pending RX packet status (0 when none).
            GRXSTSP => {
                if let Some((ch, bcnt, dpid)) = self.rx_meta.take() {
                    (ch as u32) | ((bcnt as u32) << 4) | ((dpid as u32) << 15) | GRXSTSP_PKTSTS_IN
                } else {
                    0
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
            // TX space: GNPTXSTS low half + DTXFSTS0 (remaining words).
            GNPTXSTS => self.tx_space() & 0xFFFF,
            DTXFSTS0 => self.tx_space(),
            // DFIFO reads: host mode pops staged RX bytes; device mode
            // serves the EP0 IN payload first (mirrored at SETUP), then the
            // EP1 echo queue; empty reads 0 either way.
            DFIFO0 => {
                let mut w = 0u32;
                for i in 0..4 {
                    let b = if !self.host_mode() && !self.rx_data.is_empty() {
                        self.rx_data.pop_front()
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
            // GINTSTS is computed live (host bits) — writes ignored.
            // DSTS/DAINT are read-only; EP0 interrupt flags are W1C.
            GINTSTS | DSTS | DAINT | GRXSTSP | HAINT => {}
            DIEPINT0 | DOEPINT0 => {
                let w = (offset / 4) as usize;
                self.regs[w] &= !value;
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
                self.dev_loopback_status();
            }
            DFIFO0 => {
                if self.host_mode() {
                    // Host-mode OUT/SETUP payload staging (1024-byte cap
                    // like the TXFIFO; overfill dropped).
                    if self.host_txfifo.len() + 4 <= TXFIFO_WORDS as usize * 4 {
                        self.host_txfifo.extend(value.to_le_bytes());
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
                // (runs the transfer synchronously); all other registers
                // are plain stores.
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
                self.regs[w] = value;
            }
            _ => {}
        }
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
            self.regs[(DOEPINT0 / 4) as usize] |= Self::EP_XFRC;
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
        self.regs[(DIEPINT0 / 4) as usize] |= Self::EP_XFRC;
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
