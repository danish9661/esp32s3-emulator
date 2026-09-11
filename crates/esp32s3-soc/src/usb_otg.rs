//! ESP32-S3 USB-OTG (DesignWare DWC2 full-speed device/host) model.
//!
//! Register block at `0x6008_0000` (`dwc2_esp32.h` `DWC2_FS_REG_BASE`;
//! IRQ `ETS_USB_INTR_SOURCE` = 38, 7 EPs / 5 IN). Layout per
//! `soc/usb_dwc_struct.h` (offsets in the member comments below).
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
//! KNOWN LIMITATION: device enumeration needs a USB host counterparty
//! (bus reset, SETUP/IN/OUT packets, SOF timing, descriptors, address +
//! config assignment, class drivers) — weeks of work with an external
//! party, and zero in-tree firmware needs it (every sketch's `Serial`
//! flows through the validated USB-Serial-JTAG block). Consequently the
//! Arduino "USB CDC On Boot" config (which routes `Serial` through OTG)
//! is unsupported; the default Serial/JTAG console path is fully
//! validated. DSTS/DAINT/EPINT/GRXSTSP read 0 (no bus events); HW config
//! ID registers (GSNPSID/GHWCFG) read 0 (the driver uses static config).

/// USB-OTG DWC register-block base (`dwc2_esp32.h` `DWC2_FS_REG_BASE`).
pub const USB_OTG_BASE: u32 = 0x6008_0000;
/// Second page (DFIFO0 @ +0x1000 lives past the first 4 KB page).
pub const USB_OTG_FIFO_PAGE: u32 = 0x6008_1000;
/// USB interrupt source (`interrupts.h` LEDC=35 explicit → EFUSE=36,
/// TWAI=37, USB=38, RTC_CORE=39 — same counting as rtc.rs).
pub const USB_OTG_INTR_SOURCE: u32 = 38;

// Core register offsets (usb_dwc_struct.h member comments).
const GOTGCTL: u32 = 0x000;
const GAHBCFG: u32 = 0x008;
const GUSBCFG: u32 = 0x00C;
const GRSTCTL: u32 = 0x010;
const GINTSTS: u32 = 0x014;
const GINTMSK: u32 = 0x018;
const GRXSTSP: u32 = 0x020;
const GNPTXSTS: u32 = 0x02C;
const DCFG: u32 = 0x800;
const DCTL: u32 = 0x804;
const DSTS: u32 = 0x808;
const DIEPMSK: u32 = 0x810;
const DOEPMSK: u32 = 0x814;
const DAINT: u32 = 0x818;
const DAINTMSK: u32 = 0x81C;
const DIEPCTL0: u32 = 0x900;
const DIEPINT0: u32 = 0x908;
const DIEPTSIZ0: u32 = 0x910;
const DTXFSTS0: u32 = 0x914;
const DOEPCTL0: u32 = 0xB00;
const DOEPINT0: u32 = 0xB08;
const DOEPTSIZ0: u32 = 0xB10;
const DFIFO0: u32 = 0x1000;

// GRSTCTL bits (usb_dwc_grstctl_reg_t): csftrst[0], rxfflsh[4],
// txfflsh[5], txfnum[10:6], ahbidle[31].
const GRST_CSFTRST: u32 = 1 << 0;
const GRST_RXFFLSH: u32 = 1 << 4;
const GRST_TXFFLSH: u32 = 1 << 5;
const GRST_AHBIDLE: u32 = 1 << 31;
/// TXFIFO0 depth in words (`dwc2_esp32.h` `otg_dfifo_depth` = 256).
const TXFIFO_WORDS: u32 = 256;
const REG_WORDS: usize = 0x2000 / 4;

pub struct UsbOtg {
    regs: [u32; REG_WORDS],
    /// Staged TXFIFO0 bytes (DFIFO0 writes with no host to drain them).
    txfifo: alloc::vec::Vec<u8>,
}

impl UsbOtg {
    pub fn new() -> Self {
        let mut o = Self {
            regs: [0; REG_WORDS],
            txfifo: alloc::vec::Vec::new(),
        };
        o.core_reset();
        o
    }

    /// Core soft reset (GRSTCTL.CSFTRST): restore power-on state and raise
    /// AHBIDLE; the reset bit itself clears (self-clearing on silicon).
    fn core_reset(&mut self) {
        self.regs = [0; REG_WORDS];
        self.txfifo.clear();
        self.regs[(GRSTCTL / 4) as usize] = GRST_AHBIDLE;
    }

    fn tx_space(&self) -> u32 {
        TXFIFO_WORDS - (self.txfifo.len() as u32).div_ceil(4).min(TXFIFO_WORDS)
    }

    /// Interrupt pending = GINTSTS & GINTMSK (quiet without a host).
    pub fn int_pending(&self) -> bool {
        (self.regs[(GINTSTS / 4) as usize] & self.regs[(GINTMSK / 4) as usize]) != 0
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            GRSTCTL => self.regs[(GRSTCTL / 4) as usize],
            // No bus events without a host: device/endpoint interrupt and
            // packet-status registers read 0; RXFIFO pops read 0 (empty).
            GINTSTS | DSTS | DAINT | DIEPINT0 | DOEPINT0 | GRXSTSP => 0,
            // TX space: GNPTXSTS low half + DTXFSTS0 (remaining words).
            GNPTXSTS => self.tx_space() & 0xFFFF,
            DTXFSTS0 => self.tx_space(),
            DFIFO0 => 0,
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
            // GINTSTS is event-latched (nothing latches without a host);
            // endpoint/packet IDs are read-only.
            GINTSTS | DSTS | DAINT | DIEPINT0 | DOEPINT0 | GRXSTSP => {}
            DFIFO0 => {
                // TXFIFO0 push (little-endian word); overfill is dropped.
                if self.txfifo.len() + 4 <= TXFIFO_WORDS as usize * 4 {
                    self.txfifo.extend_from_slice(&value.to_le_bytes());
                }
            }
            o if o < (REG_WORDS * 4) as u32 && o.is_multiple_of(4) => {
                self.regs[(o / 4) as usize] = value;
            }
            _ => {}
        }
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
        u.write32(DIEPCTL0, (1 << 15) | 0);
        u.write32(DOEPCTL0, (1 << 15) | 0);
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
}
