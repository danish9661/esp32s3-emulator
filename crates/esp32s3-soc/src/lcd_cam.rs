//! ESP32-S3 LCD_CAM controller (= parallel I/O / "PARLIO").
//!
//! Base `DR_REG_LCD_CAM_BASE = 0x6004_1000`. Register layout per esp-idf
//! `components/soc/esp32s3/register/soc/lcd_cam_reg.h`.
//!
//! Functional model: a TX/RX FIFO pair plus a transfer-start / transfer-done
//! interrupt path. Writing words to `LCD_DATA` (0x40) pushes the TX FIFO;
//! `LCD_FIFO_STATUS` (0x44) reports the count. Setting `LCD_START` (bit 27 of
//! `LCD_USER` 0x14) begins a parallel transfer: each TX FIFO word is presented
//! on the LCD_CAM GPIO-matrix data signals (`LCD_DATA_OUT0..15` = 133..148) for
//! one `LCD_PCLK` (sig 154) cycle, toggling PCLK and asserting `LCD_CS` (sig
//! 132, active low) and `LCD_DC` (sig 153, from `LCD_USER` bit 26) for the
//! duration. When the FIFO empties `LCD_TRANS_DONE` (bit 1 of the
//! `LC_DMA_INT_*` block at 0x64/0x68/0x6C/0x70) is raised. The camera (RX)
//! path mirrors this with `CAM_DATA` (0x48) / `CAM_FIFO_STATUS` (0x4C) and
//! `CAM_START` (bit 29 of `CAM_CTRL1` 0x08) but has no data source, so its
//! FIFO stays empty.
//!
//! The presented signals are observable on GPIO pins whose `FUNC_OUT_SEL` is
//! routed to the corresponding LCD_CAM signal index (see `signal_level`).

pub const LCD_CAM_BASE: u32 = 0x6004_1000;

// LCD_CAM GPIO-matrix signal indices (gpio_sig_map.h).
const SIG_LCD_CS: u32 = 132;
const SIG_DATA0: u32 = 133; // .. SIG_DATA15 = 148
const SIG_H_ENABLE: u32 = 150;
const SIG_H_SYNC: u32 = 151;
const SIG_V_SYNC: u32 = 152;
const SIG_LCD_DC: u32 = 153;
const SIG_LCD_PCLK: u32 = 154;
const SIG_CAM_PCLK: u32 = 149; // CAM_PCLK/CAM_CLK shared PCLK

const REG_COUNT: usize = 0x1000 / 4;

const LCD_USER: u32 = 0x14;
const LCD_CMD_BIT: u32 = 1 << 26;
const CAM_CTRL1: u32 = 0x08;
const LCD_DATA: u32 = 0x40;
const LCD_FIFO_STATUS: u32 = 0x44;
const CAM_DATA: u32 = 0x48;
const CAM_FIFO_STATUS: u32 = 0x4C;
const LC_DMA_INT_ENA: u32 = 0x64;
const LC_DMA_INT_RAW: u32 = 0x68;
const LC_DMA_INT_ST: u32 = 0x6C;
const LC_DMA_INT_CLR: u32 = 0x70;

const LCD_START_BIT: u32 = 1 << 27;
const LCD_RESET_BIT: u32 = 1 << 28;
const CAM_RESET_BIT: u32 = 1 << 30;

const LCD_TRANS_DONE: u32 = 1 << 1;
const CAM_VSYNC_INT: u32 = 1 << 2;
const CAM_HS_INT: u32 = 1 << 3;
const LCD_VSYNC_INT: u32 = 1 << 0;
const INT_MASK: u32 = LCD_VSYNC_INT | LCD_TRANS_DONE | CAM_VSYNC_INT | CAM_HS_INT;

const FIFO_DEPTH: usize = 16;

pub struct LcdCam {
    regs: [u32; REG_COUNT],
    tx_fifo: [u32; FIFO_DEPTH],
    tx_count: usize,
    rx_fifo: [u32; FIFO_DEPTH],
    rx_count: usize,
    int_raw: u32,
    int_ena: u32,
    // Parallel-transfer output state.
    busy: bool,
    cur_word: u32,
    pclk: u32,
    dc: u32,
}

impl LcdCam {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            regs: [0; REG_COUNT],
            tx_fifo: [0; FIFO_DEPTH],
            tx_count: 0,
            rx_fifo: [0; FIFO_DEPTH],
            rx_count: 0,
            int_raw: 0,
            int_ena: 0,
            busy: false,
            cur_word: 0,
            pclk: 0,
            dc: 0,
        }
    }

    fn idx(off: u32) -> usize {
        ((off & 0xFFF) / 4) as usize
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let o = offset & 0xFFF;
        match o {
            LCD_DATA => 0, // write-only push register
            LCD_FIFO_STATUS => {
                let c = (self.tx_count as u32) & 0x7FF;
                c | (c << 16)
            }
            CAM_DATA => {
                if self.rx_count > 0 {
                    let v = self.rx_fifo[0];
                    for i in 1..self.rx_count {
                        self.rx_fifo[i - 1] = self.rx_fifo[i];
                    }
                    self.rx_count -= 1;
                    v
                } else {
                    0
                }
            }
            CAM_FIFO_STATUS => {
                let c = (self.rx_count as u32) & 0x7FF;
                c | (c << 16)
            }
            LC_DMA_INT_RAW => self.int_raw,
            LC_DMA_INT_ST => self.int_raw & self.int_ena,
            LC_DMA_INT_ENA => self.int_ena,
            _ => self.regs[Self::idx(o)],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let o = offset & 0xFFF;
        match o {
            LCD_DATA => {
                if self.tx_count < FIFO_DEPTH {
                    self.tx_fifo[self.tx_count] = value;
                    self.tx_count += 1;
                }
            }
            LCD_FIFO_STATUS | CAM_FIFO_STATUS | CAM_DATA => { /* read-only */ }
            LC_DMA_INT_ENA => self.int_ena = value & INT_MASK,
            LC_DMA_INT_CLR => self.int_raw &= !value,
            LC_DMA_INT_RAW => { /* read-only */ }
            LCD_USER => {
                self.regs[Self::idx(o)] = value;
                if value & LCD_RESET_BIT != 0 {
                    self.tx_count = 0;
                    self.int_raw = 0;
                    self.busy = false;
                    self.pclk = 0;
                }
                if value & LCD_START_BIT != 0 {
                    // Begin a parallel transfer.
                    self.busy = true;
                    self.pclk = 0;
                    self.dc = (value & LCD_CMD_BIT) >> 26;
                    self.cur_word = 0;
                    // Present the first word on the next PCLK rising edge.
                }
            }
            CAM_CTRL1 => {
                self.regs[Self::idx(o)] = value;
                if value & CAM_RESET_BIT != 0 {
                    self.rx_count = 0;
                }
                // CAM_START has no data source modeled; RX FIFO stays empty.
            }
            _ => {
                self.regs[Self::idx(o)] = value;
            }
        }
    }

    /// Advance the parallel transfer by one emulator step (one PCLK half-cycle).
    pub fn tick(&mut self) {
        if !self.busy {
            return;
        }
        self.pclk ^= 1;
        if self.pclk == 1 {
            // Rising PCLK edge: present the next TX FIFO word.
            if self.tx_count > 0 {
                self.cur_word = self.tx_fifo[0];
                for i in 1..self.tx_count {
                    self.tx_fifo[i - 1] = self.tx_fifo[i];
                }
                self.tx_count -= 1;
            } else {
                // FIFO drained: finish the transfer.
                self.busy = false;
                self.int_raw |= LCD_TRANS_DONE;
            }
        }
    }

    /// Current level (0/1) of an LCD_CAM GPIO-matrix signal index.
    pub fn signal_level(&self, sig: u32) -> u32 {
        if sig == SIG_LCD_CS {
            if self.busy { 0 } else { 1 }
        } else if (SIG_DATA0..=SIG_DATA0 + 15).contains(&sig) {
            (self.cur_word >> (sig - SIG_DATA0)) & 1
        } else if sig == SIG_LCD_PCLK || sig == SIG_CAM_PCLK {
            self.pclk
        } else if sig == SIG_LCD_DC {
            self.dc
        } else if sig == SIG_H_ENABLE || sig == SIG_H_SYNC || sig == SIG_V_SYNC {
            if self.busy { 1 } else { 0 }
        } else {
            0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_fifo_counts_and_drains_on_start() {
        let mut d = LcdCam::new();
        d.write32(LCD_DATA, 0x1111_1111);
        d.write32(LCD_DATA, 0x2222_2222);
        d.write32(LCD_DATA, 0x3333_3333);
        assert_eq!(d.read32(LCD_FIFO_STATUS) & 0x7FF, 3, "TX FIFO count");
        d.write32(LC_DMA_INT_ENA, LCD_TRANS_DONE);
        d.write32(LC_DMA_INT_CLR, 0xF);
        d.write32(LCD_USER, LCD_START_BIT);
        // Each TX word is presented over one PCLK cycle (~2 ticks); after the
        // FIFO drains the next rising edge finishes and raises TRANS_DONE.
        for _ in 0..32 {
            d.tick();
        }
        assert_eq!(d.read32(LCD_FIFO_STATUS) & 0x7FF, 0, "TX drained");
        assert_eq!(d.read32(LC_DMA_INT_RAW) & LCD_TRANS_DONE, LCD_TRANS_DONE);
        assert_eq!(d.read32(LC_DMA_INT_ST) & LCD_TRANS_DONE, LCD_TRANS_DONE);
        // Clear and confirm.
        d.write32(LC_DMA_INT_CLR, LCD_TRANS_DONE);
        assert_eq!(d.read32(LC_DMA_INT_RAW) & LCD_TRANS_DONE, 0, "RAW cleared");
    }

    #[test]
    fn parallel_signals_driven_during_transfer() {
        let mut d = LcdCam::new();
        d.write32(LCD_DATA, 0x0000_FFF0); // data lines 4..15 high
        d.write32(LCD_USER, LCD_START_BIT | LCD_CMD_BIT);
        assert_eq!(d.signal_level(SIG_LCD_CS), 0, "CS active (low) at start");
        assert_eq!(d.signal_level(SIG_LCD_DC), 1, "DC = cmd bit");
        d.tick();
        d.tick(); // present first word
        for i in 4..16 {
            assert_eq!(d.signal_level(SIG_DATA0 + i as u32), 1, "data line {}", i);
        }
        assert_eq!(d.signal_level(SIG_DATA0), 0);
    }

    #[test]
    fn config_registers_round_trip() {
        let mut d = LcdCam::new();
        d.write32(0x10, 0xABCD_0000);
        d.write32(0x24, 0x1234_5678);
        assert_eq!(d.read32(0x10), 0xABCD_0000);
        assert_eq!(d.read32(0x24), 0x1234_5678);
    }
}
