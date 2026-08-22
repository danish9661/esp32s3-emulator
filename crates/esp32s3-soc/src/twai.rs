//! ESP32-S3 TWAI (Two-Wire Automotive Interface, CAN 2.0B) controller model.
//!
//! Register block at `0x6000_C000` (TRM TWAI chapter; `soc/twai_struct.h`). The
//! registers are 8-bit but mapped to the LSB of every 32-bit word, so each
//! register occupies one 32-bit slot and only its low byte is significant.
//!
//! The model implements the PeliCAN-style register set: mode, command, status,
//! interrupt (IR), interrupt-enable (IER), bus timing, error counters/capture,
//! a 4-byte acceptance filter (ACR/AMR), and a 13-byte shared TX/RX buffer.
//! Registers at `0x40..0x70` are dual-purpose: in **reset mode** (`mode.rm`)
//! they hold the acceptance filter (ACR[4] @ 0x40, AMR[4] @ 0x50); in
//! **operational mode** they are the TX (write) / RX (read) buffer
//! (`twai_ll_frame_buffer_t`, 13 bytes).
//!
//! TX is completed synchronously when software writes `command.tr` (or `srr`).
//! In self-test mode (`mode.stm`) or on a self-reception request the transmitted
//! frame is looped back into the RX buffer (subject to the acceptance filter),
//! which is exactly how the esp-idf `TWAI_MODE_NO_ACK` self-test and the
//! `self_reception` flag exercise the receive path without a real bus. This is
//! sufficient to validate the controller's register behavior and loopback end
//! to end. Real bus arbitration/ACK/error-frame timing is not modeled.

/// TWAI register-block base (APB).
pub const TWAI_BASE: u32 = 0x6000_C000;

/// TWAI interrupt source for the interrupt matrix (esp32s3 interrupts.h
/// `ETS_TWAI_INTR_SOURCE = 37`).
pub const TWAI_INTR_SOURCE: u32 = 37;

/// Number of bytes in the shared TX/RX frame buffer.
const FRAME_LEN: usize = 13;

#[derive(Default)]
pub struct Twai {
    /// MOD register (0x00): rm/lom/stm/afm in low bits.
    mode: u32,
    /// SR register (0x08), recomputed on read.
    status: u32,
    /// IR register (0x0C): raw pending interrupt bits.
    ir: u32,
    /// IER register (0x10): interrupt enable mask.
    ier: u32,
    /// BTR0 (0x18) / BTR1 (0x1C): stored, not used for timing.
    bt0: u32,
    bt1: u32,
    /// ALC (0x2C) / ECC (0x30): captured on error (unmodeled -> 0).
    alc: u32,
    ecc: u32,
    /// EWL (0x34) / RXERR (0x38) / TXERR (0x3C): stored.
    ewl: u32,
    rxerr: u32,
    txerr: u32,
    /// Acceptance code (ACR[0..3]) / mask (AMR[0..3]) — reset-mode only.
    acr: [u8; 4],
    amr: [u8; 4],
    /// Shared TX/RX frame buffer (operational mode).
    buf: [u8; FRAME_LEN],
    /// CLKDIV (0x7C): stored.
    clk_div: u32,
    /// True when a frame is waiting in the (single-slot) RX buffer.
    rx_full: bool,
}

impl Twai {
    pub fn new() -> Twai {
        // On reset the controller powers up in reset mode (rm = 1) with the
        // transmit buffer free and the last transmission "complete".
        Twai {
            mode: 1,
            status: (1 << 2) | (1 << 3), // tbs (free) | tcs (complete)
            ..Default::default()
        }
    }

    /// True if any interrupt is pending (raw & enabled) for the matrix.
    pub fn int_pending(&self) -> bool {
        self.ir & self.ier != 0
    }

    /// Masked interrupt status (raw & enabled).
    pub fn int_st(&self) -> u32 {
        self.ir & self.ier
    }

    fn in_reset(&self) -> bool {
        self.mode & 1 != 0
    }

    /// Acceptance filter: a frame is accepted when, for each of the first
    /// four frame bytes, `(byte & amr[i]) == (acr[i] & amr[i])` — i.e. the
    /// masked identifier bytes match the acceptance code (PeliCAN 4-byte
    /// filter; `amr[i]==0` accepts any value, giving accept-all).
    fn accepts(&self, frame: &[u8; FRAME_LEN]) -> bool {
        frame
            .iter()
            .zip(self.acr.iter())
            .zip(self.amr.iter())
            .take(4)
            .all(|((f, acr), amr)| (f & amr) == (acr & amr))
    }

    /// Begin a transmission: copy the TX buffer into the RX buffer (loopback)
    /// when self-test/self-reception applies, mark TX complete, and assert the
    /// relevant interrupt bits.
    fn transmit(&mut self, self_rx: bool) {
        let frame = self.buf;
        // TX complete: buffer free + transmission complete; no longer sending.
        self.status |= (1 << 2) | (1 << 3);
        self.status &= !(1 << 5); // ts = 0 (not transmitting)
        self.ir |= 1 << 1; // ti (transmit interrupt)
        if self_rx && self.accepts(&frame) {
            if self.rx_full {
                // New frame with the RX buffer still occupied -> data overrun.
                self.status |= 1 << 1; // dos
                self.ir |= 1 << 3; // doi
            }
            self.buf = frame;
            self.rx_full = true;
            self.status |= 1 << 0; // rbs
            self.status &= !(1 << 4); // rs = 0
            self.ir |= 1 << 0; // ri (receive interrupt)
        }
    }

    pub fn write32(&mut self, off: u32, value: u32) {
        let v = (value & 0xFF) as u8;
        match off {
            0x00 => self.mode = value & 0xF, // rm/lom/stm/afm
            0x04 => self.do_command(value),
            0x08 => {}                 // status is read-only
            0x0C => self.ir &= !value, // IR is write-1-to-clear (RI cleared by RRB)
            0x10 => self.ier = value & 0xFF,
            0x14 => {}
            0x18 => self.bt0 = value,
            0x1C => self.bt1 = value,
            0x20 | 0x24 | 0x28 => {}
            0x2C => {} // ALC read-only
            0x30 => {} // ECC read-only
            0x34 => self.ewl = value & 0xFF,
            0x38 => self.rxerr = value & 0xFF,
            0x3C => self.txerr = value & 0xFF,
            0x40 | 0x44 | 0x48 | 0x4C => {
                let i = ((off - 0x40) / 4) as usize;
                if self.in_reset() {
                    self.acr[i] = v;
                } else {
                    self.buf[i] = v;
                }
            }
            0x50 | 0x54 | 0x58 | 0x5C => {
                let i = ((off - 0x50) / 4) as usize;
                if self.in_reset() {
                    self.amr[i] = v;
                } else {
                    let bi = ((off - 0x40) / 4) as usize;
                    if bi < FRAME_LEN {
                        self.buf[bi] = v;
                    }
                }
            }
            0x60 | 0x64 | 0x68 | 0x6C | 0x70 => {
                if !self.in_reset() {
                    let i = ((off - 0x40) / 4) as usize;
                    if i < FRAME_LEN {
                        self.buf[i] = v;
                    }
                }
            }
            0x74 => {} // RMC read-only
            0x78 => {}
            0x7C => self.clk_div = value,
            _ => {}
        }
    }

    fn do_command(&mut self, value: u32) {
        if self.in_reset() {
            return; // no bus activity while in reset mode
        }
        if value & (1 << 2) != 0 {
            // Release RX buffer: free the single RX slot and clear RI.
            self.rx_full = false;
            self.status &= !(1 << 0); // rbs = 0
            self.ir &= !(1 << 0); // ri cleared by buffer release
        }
        if value & (1 << 3) != 0 {
            // Clear data overrun status.
            self.status &= !(1 << 1);
        }
        let self_rx = (self.mode & (1 << 2) != 0) || (value & (1 << 4) != 0);
        if value & (1 << 0) != 0 || value & (1 << 4) != 0 {
            // Transmission request (tr) or self-reception request (srr).
            self.transmit(self_rx);
        }
    }

    pub fn read32(&mut self, off: u32) -> u32 {
        match off {
            0x00 => self.mode,
            0x04 => 0, // command is write-only (reads 0)
            0x08 => self.compute_status(),
            0x0C => {
                let v = self.ir;
                // Reading IR clears all interrupts except the receive interrupt
                // (RI is cleared by releasing the RX buffer).
                self.ir &= 0x1;
                v
            }
            0x10 => self.ier,
            0x14 => 0,
            0x18 => self.bt0,
            0x1C => self.bt1,
            0x20 | 0x24 | 0x28 => 0,
            0x2C => self.alc,
            0x30 => self.ecc,
            0x34 => self.ewl,
            0x38 => self.rxerr,
            0x3C => self.txerr,
            0x40 | 0x44 | 0x48 | 0x4C => {
                let i = ((off - 0x40) / 4) as usize;
                if self.in_reset() {
                    self.acr[i] as u32
                } else {
                    self.buf[i] as u32
                }
            }
            0x50 | 0x54 | 0x58 | 0x5C => {
                let i = ((off - 0x50) / 4) as usize;
                if self.in_reset() {
                    self.amr[i] as u32
                } else {
                    let bi = ((off - 0x40) / 4) as usize;
                    if bi < FRAME_LEN {
                        self.buf[bi] as u32
                    } else {
                        0
                    }
                }
            }
            0x60 | 0x64 | 0x68 | 0x6C | 0x70 => {
                if !self.in_reset() {
                    let i = ((off - 0x40) / 4) as usize;
                    if i < FRAME_LEN {
                        return self.buf[i] as u32;
                    }
                }
                0
            }
            0x74 => {
                if self.rx_full {
                    1
                } else {
                    0
                }
            }
            0x78 => 0,
            0x7C => self.clk_div,
            _ => 0,
        }
    }

    fn compute_status(&self) -> u32 {
        // Recompute the live bits; tbs/tcs/dos/rbs are stored in `status`.
        let mut s = self.status & ((1 << 2) | (1 << 3) | (1 << 1) | (1 << 0));
        // Bus on, no error, not currently sending/receiving.
        s &= !((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8));
        s
    }
}
