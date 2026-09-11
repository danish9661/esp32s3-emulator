//! ESP32-S3 USB-OTG (DesignWare DWC2 full-speed device/host) model.
//!
//! Register block at `0x6008_0000` (`dwc2_esp32.h` `DWC2_FS_REG_BASE`;
//! IRQ `ETS_USB_INTR_SOURCE` = 38, 7 EPs / 5 IN). Layout per
//! `soc/usb_dwc_struct.h` (offsets in the member comments below; the S3
//! header isn't shipped in arduino-cli, so the identical DWC2 core layout
//! was verified against the ESP32-P4 copy).
//!
//! Modeled (device-init path, all host-independent):
//! - GRSTCTL core soft reset (`CSFTRST` bit 0 self-clears and restores the
//!   bank; `AHBIDLE` bit 31 reads 1 when idle); RX/TX FIFO flush bits are
//!   accepted and self-clear (TX flush drops staged bytes).
//! - GUSBCFG force-device/host latches, GAHBCFG, device config (DCFG/DCTL:
//!   speed/address/soft-disconnect), EP0 control (DIEPCTL0/DOEPCTL0 MPS +
//!   activate) and transfer-size registers: plain stores with readback.
//! - TXFIFO0 (DFIFO0 @ 0x1000): pushed words stage bytes; DTXFSTS0/GNPTXSTS
//!   report the remaining space (256-word capacity).
//! - GINTSTS/GINTMSK: no USB events exist without a host, so status reads
//!   0 and `int_pending` (RAW & MSK, source 38) stays quiet.
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
//! KNOWN LIMITATIONS: device-mode enumeration still needs an external USB
//! host counterparty (bus reset, SETUP/IN/OUT, descriptors — out of scope;
//! zero in-tree firmware needs it; all `Serial` flows through validated
//! USB-Serial-JTAG, so Arduino "USB CDC On Boot" stays unsupported).
//! Host-side approximations: transfers complete instantly (no SOF/frame
//! timing; HFNUM reads 0); R1b busy (port reset, SWITCH-like waits) is
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
            DSTS | DAINT | DIEPINT0 | DOEPINT0 => 0,
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
            // DFIFO0 reads pop staged RX bytes (host mode); empty reads 0
            // (device RXFIFO is always empty without a host).
            DFIFO0 => {
                let mut w = 0u32;
                for i in 0..4 {
                    if let Some(b) = self.rx_data.pop_front() {
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
                // RX/TX FIFO flushes self-clear (TX flush drops staged
                // bytes); a core soft reset restores the bank.
                if value & GRST_TXFFLSH != 0 {
                    self.txfifo.clear();
                }
                if value & GRST_CSFTRST != 0 {
                    self.core_reset();
                } else {
                    self.regs[(GRSTCTL / 4) as usize] = value & !(GRST_RXFFLSH | GRST_TXFFLSH);
                }
            }
            // GINTSTS is computed live (host bits) — writes ignored.
            // DSTS/DAINT/DIEPINT0/DOEPINT0 are read-only.
            GINTSTS | DSTS | DAINT | DIEPINT0 | DOEPINT0 | GRXSTSP | HAINT => {}
            HPRT => self.hprt_write(value),
            DFIFO0 => {
                if self.host_mode() {
                    // Host-mode OUT/SETUP payload staging (1024-byte cap
                    // like the TXFIFO; overfill dropped).
                    if self.host_txfifo.len() + 4 <= TXFIFO_WORDS as usize * 4 {
                        self.host_txfifo.extend(value.to_le_bytes());
                    }
                } else if self.txfifo.len() + 4 <= TXFIFO_WORDS as usize * 4 {
                    // TXFIFO0 push (little-endian word); overfill is dropped.
                    self.txfifo.extend_from_slice(&value.to_le_bytes());
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
        assert_eq!(u.read32(DTXFSTS0), TXFIFO_WORDS);
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
