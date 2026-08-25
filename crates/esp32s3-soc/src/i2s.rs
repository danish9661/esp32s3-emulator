//! ESP32-S3 I2S controller (audio) — `I2S0_BASE = 0x6000_F000`,
//! `I2S1_BASE = 0x6002_D000`.
//!
//! Register layout per esp-idf `i2s_struct.h` / `i2s_reg.h`. Functional model:
//! a TX/RX FIFO plus a bit-clocked serial shift-out path that drives the
//! I2S GPIO-matrix output signals (`I2SxO_BCK`, `I2SxO_WS`, `I2SxO_SD`) so the
//! data is observable on routed GPIO pins. `TX_START` (bit 2 of `TX_CONF`
//! 0x24) begins a transmission; each FIFO word is shifted out MSB/LSB-first
//! (per `tx_bit_order`) over `tx_ticks` emulator steps, one serial bit per
//! step, toggling BCK; the word-select (WS) line toggles once per word. When
//! the TX FIFO empties the `tx_done` interrupt (bit 1 of the `INT_*`
//! block at 0x0C/0x10/0x14/0x18) is raised.
//!
//! KNOWN LIMITATIONS: the RX path has no data source (RX FIFO stays empty, no
//! `rx_done`); master/slave clock generation, TDM, PDM and the esp-idf DMA
//! path are not modeled. The serial shift rate is fixed at one bit per
//! emulator step (a faithful cycle-accurate BCK is out of scope).

pub const I2S0_BASE: u32 = 0x6000_F000;
pub const I2S1_BASE: u32 = 0x6002_D000;

const REG_COUNT: usize = 0x1000 / 4;

// Register offsets.
const INT_RAW: u32 = 0x0C;
const INT_ST: u32 = 0x10;
const INT_ENA: u32 = 0x14;
const INT_CLR: u32 = 0x18;
const RX_CONF: u32 = 0x20;
const TX_CONF: u32 = 0x24;
const TX_CONF1: u32 = 0x2C;
const FIFO: u32 = 0x80;
const DATE: u32 = 0xFC;

const TX_START_BIT: u32 = 1 << 2;
const RX_START_BIT: u32 = 1 << 2;

const RX_DONE: u32 = 1 << 0;
const TX_DONE: u32 = 1 << 1;
const RX_HUNG: u32 = 1 << 2;
const TX_HUNG: u32 = 1 << 3;
const INT_MASK: u32 = RX_DONE | TX_DONE | RX_HUNG | TX_HUNG;

const FIFO_DEPTH: usize = 16;

#[derive(Clone, Copy)]
pub struct I2s {
    idx: u32,
    regs: [u32; REG_COUNT],
    tx_fifo: [u32; FIFO_DEPTH],
    tx_count: usize,
    rx_fifo: [u32; FIFO_DEPTH],
    rx_count: usize,
    int_raw: u32,
    int_ena: u32,
    // TX serial-shift state.
    tx_busy: bool,
    tx_shift: u32,
    tx_bit: u32,
    tx_bck: u32,
    tx_ws: u32,
    tx_sd: u32,
}

impl I2s {
    pub fn new(idx: u32) -> Self {
        Self {
            idx,
            regs: [0; REG_COUNT],
            tx_fifo: [0; FIFO_DEPTH],
            tx_count: 0,
            rx_fifo: [0; FIFO_DEPTH],
            rx_count: 0,
            int_raw: 0,
            int_ena: 0,
            tx_busy: false,
            tx_shift: 0,
            tx_bit: 0,
            tx_bck: 0,
            tx_ws: 0,
            tx_sd: 0,
        }
    }

    fn idx_of(off: u32) -> usize {
        ((off & 0xFFF) / 4) as usize
    }

    fn bits_per_word(&self) -> u32 {
        let f = (self.regs[Self::idx_of(TX_CONF1)] >> 13) & 0x1F;
        if f == 0 { 16 } else { f + 1 }
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let o = offset & 0xFFF;
        match o {
            INT_RAW => self.int_raw,
            INT_ST => self.int_raw & self.int_ena,
            INT_ENA => self.int_ena,
            FIFO => {
                // RX pop (no data source modeled -> 0).
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
            DATE => 0x2020_0100,
            _ => self.regs[Self::idx_of(o)],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let o = offset & 0xFFF;
        match o {
            FIFO => {
                if self.tx_count < FIFO_DEPTH {
                    self.tx_fifo[self.tx_count] = value;
                    self.tx_count += 1;
                }
            }
            INT_ENA => self.int_ena = value & INT_MASK,
            INT_CLR => self.int_raw &= !value,
            INT_RAW => { /* read-only */ }
            RX_CONF => {
                self.regs[Self::idx_of(o)] = value;
                if value & RX_START_BIT != 0 {
                    // No data source modeled; RX stays empty.
                }
            }
            TX_CONF => {
                self.regs[Self::idx_of(o)] = value;
                if value & TX_START_BIT != 0 && !self.tx_busy {
                    self.tx_busy = true;
                    self.tx_bit = 0;
                    self.tx_bck = 0;
                    self.tx_ws = 0;
                    self.tx_sd = 0;
                    if self.tx_count > 0 {
                        self.tx_shift = self.tx_fifo[0];
                        for i in 1..self.tx_count {
                            self.tx_fifo[i - 1] = self.tx_fifo[i];
                        }
                        self.tx_count -= 1;
                    } else {
                        self.tx_shift = 0;
                    }
                }
            }
            _ => {
                self.regs[Self::idx_of(o)] = value;
            }
        }
    }

    /// Advance the serial shift-out by one emulator step.
    pub fn tick(&mut self) {
        if !self.tx_busy {
            return;
        }
        self.tx_bck ^= 1;
        if self.tx_bck == 1 {
            // Rising BCK edge: present the next serial bit.
            let bits = self.bits_per_word();
            let order = (self.regs[Self::idx_of(TX_CONF1)] >> 17) & 1;
            let bit = if order == 1 {
                (self.tx_shift >> self.tx_bit) & 1
            } else {
                (self.tx_shift >> (bits - 1 - self.tx_bit)) & 1
            };
            self.tx_sd = bit;
            self.tx_bit += 1;
            if self.tx_bit >= bits {
                self.tx_bit = 0;
                self.tx_ws ^= 1; // toggle word-select each word
                if self.tx_count > 0 {
                    self.tx_shift = self.tx_fifo[0];
                    for i in 1..self.tx_count {
                        self.tx_fifo[i - 1] = self.tx_fifo[i];
                    }
                    self.tx_count -= 1;
                } else {
                    self.tx_busy = false;
                    self.int_raw |= TX_DONE;
                }
            }
        }
    }

    /// Current level (0/1) of an I2S output signal index (see gpio_sig_map.h).
    pub fn signal_level(&self, sig: u32) -> u32 {
        let (bck, ws, sd) = if self.idx == 0 {
            (22u32, 24u32, 25u32)
        } else {
            (28u32, 29u32, 30u32)
        };
        if sig == bck {
            self.tx_bck
        } else if sig == ws {
            self.tx_ws
        } else if sig == sd {
            self.tx_sd
        } else {
            0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_fifo_shifts_out_and_asserts_tx_done() {
        let mut d = I2s::new(0);
        d.write32(INT_ENA, TX_DONE);
        d.write32(FIFO, 0xABCD);
        d.write32(TX_CONF, TX_START_BIT);
        // 16 bits -> 16 rising BCK edges -> ~32 ticks (bck toggles each tick).
        for _ in 0..64 {
            d.tick();
        }
        assert!(!d.tx_busy, "TX should have finished");
        assert_eq!(d.int_raw & TX_DONE, TX_DONE, "tx_done set");
        assert_eq!(d.int_raw & d.int_ena, TX_DONE, "masked tx_done");
    }

    #[test]
    fn msb_first_matches_word() {
        let mut d = I2s::new(1);
        // tx_bit_order = 0 (MSB first); tx_bits_mod default -> 16 bits.
        d.write32(FIFO, 0x8000);
        d.write32(TX_CONF, TX_START_BIT);
        d.tick(); // bck -> 1 (rising), shifts MSB (bit15) = 1
        assert_eq!(d.signal_level(30), 1, "I2S1 SD (sig 30) = MSB first");
    }

    #[test]
    fn config_registers_round_trip() {
        let mut d = I2s::new(0);
        d.write32(TX_CONF1, 0x1234_0000);
        assert_eq!(d.read32(TX_CONF1), 0x1234_0000);
        d.write32(0x50, 0xDEAD_BEEF);
        assert_eq!(d.read32(0x50), 0xDEAD_BEEF);
    }
}
