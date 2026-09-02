//! ESP32-S3 I2S controller (audio) — `I2S0_BASE = 0x6000_F000`,
//! `I2S1_BASE = 0x6002_D000`.
//!
//! Register layout per esp-idf `i2s_struct.h`. Functional model:
//! - **TX**: a TX FIFO (FIFO reg 0x80, depth 16) plus a bit-clocked serial
//!   shift-out that drives the I2S GPIO-matrix output signals (`I2SxO_BCK`,
//!   `I2SxO_WS`, `I2SxO_SD`) so the data is observable on routed GPIO pins.
//!   `TX_START` (bit 2 of `TX_CONF` 0x24) begins a transmission; each FIFO
//!   word is shifted out MSB/LSB-first (per `tx_bit_order`, `TX_CONF` bit 18)
//!   over `tx_clkm_div_num` emulator steps per BCK half-cycle (the clock
//!   generator; `TX_CLKM_DIV_CONF` 0x3C), toggling BCK; the word-select (WS)
//!   line toggles once per channel/slot. When the TX FIFO empties the
//!   `tx_done` interrupt (bit 1 of the INT block at 0x0C/0x10/0x14/0x18) is
//!   raised.
//! - **RX**: a data source is provided two ways. (1) `inject_rx` pushes words
//!   into an injected-RX FIFO that the receiver shifts in, filling the readable
//!   RX FIFO and raising `rx_done` (bit 0) when drained. (2) `sig_loopback`
//!   (`TX_CONF` bit 27) feeds the transmitted serial stream back into the
//!   receiver, so a self-test transmits via TX and reads the same words from
//!   the RX FIFO — this is the firmware-visible RX path (no external codec
//!   needed). The esp-idf GDMA path is also modeled: a GDMA `out` channel with
//!   `peri_sel == 3/4` (I2S0/I2S1) copies descriptor words into the TX FIFO
//!   register, and an `in` channel copies words out of the RX FIFO register
//!   (see `gdma.rs` / `soc.rs`).
//!
//! Sub-features modeled:
//! - **Clock generation**: `tx_clkm_div_num` / `rx_clkm_div_num` set the BCK
//!   divider (master mode drives BCK/WS from an internal clock at that rate).
//!   `tx_slave_mod` / `rx_slave_mod` (`RX/TX_CONF` bit 3) select slave mode,
//!   where the peripheral does NOT generate BCK/WS (it expects an external
//!   clock, which is not modeled — see limitations).
//! - **TDM**: `tx_tdm_en` (`TX_CONF` bit 19) + `tx_tdm_tot_chan_num`
//!   (`TX_TDM_CTRL` 0x54, bits 16-19) produce a multi-slot frame; each slot
//!   shifts one FIFO word and toggles WS per slot. When TDM is disabled,
//!   `tx_chan_mod` (`TX_CONF` bits 24-26) selects 1/2/4 slots (mono/stereo/
//!   4-channel).
//! - **PDM**: `tx_pdm_en` (`TX_CONF` bit 20) re-encodes each transmitted word
//!   as a first-order sigma-delta PDM bitstream on the SD line.
//!
//! KNOWN LIMITATIONS: `tx_clkm_conf.clk_en` (bit 29) gating is not modeled
//! (the clock always runs once TX is started); slave mode requires an external
//! BCK/WS that is not simulated; TDM ignores per-channel enable masks and the
//! WS-width field; PDM uses a simple first-order sigma-delta (no sinc
//! decimation / PDM2PCM path); the esp-idf GDMA driver path is validated via
//! the GDMA walk but the full ping-pong descriptor chaining is not timed.

pub const I2S0_BASE: u32 = 0x6000_F000;
pub const I2S1_BASE: u32 = 0x6002_D000;

const REG_COUNT: usize = 0x1000 / 4;

// Register offsets (per i2s_struct.h; INT/FIFO verified against the S3 TRM).
const INT_RAW: u32 = 0x0C;
const INT_ST: u32 = 0x10;
const INT_ENA: u32 = 0x14;
const INT_CLR: u32 = 0x18;
const RX_CONF: u32 = 0x20;
const TX_CONF: u32 = 0x24;
const RX_CONF1: u32 = 0x28;
const TX_CONF1: u32 = 0x2C;
const RX_CLKM_DIV_CONF: u32 = 0x38;
const TX_CLKM_DIV_CONF: u32 = 0x3C;
const TX_TDM_CTRL: u32 = 0x54;
pub const FIFO: u32 = 0x80;
const DATE: u32 = 0xFC;

const TX_START_BIT: u32 = 1 << 2;
const RX_START_BIT: u32 = 1 << 2;
const TX_BIT_ORDER: u32 = 1 << 18;
const RX_BIT_ORDER: u32 = 1 << 18;
const TX_SLAVE_MOD: u32 = 1 << 3;
const RX_SLAVE_MOD: u32 = 1 << 3;
const TX_TDM_EN: u32 = 1 << 19;
const TX_PDM_EN: u32 = 1 << 20;
const SIG_LOOPBACK: u32 = 1 << 27;

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
    rx_in: [u32; FIFO_DEPTH],
    rx_in_count: usize,
    int_raw: u32,
    int_ena: u32,
    // TX serial-shift state.
    tx_busy: bool,
    tx_shift: u32,
    tx_bit: u32,
    tx_slot: u32,
    tx_bck: u32,
    tx_ws: u32,
    tx_sd: u32,
    // TX clock divider: counts down to the next BCK half-cycle.
    tx_bck_div: u32,
    // TX PDM sigma-delta accumulator (first-order).
    tx_pdm_acc: i32,
    // RX serial-shift state.
    rx_busy: bool,
    rx_shift: u32,
    rx_recv: u32,
    rx_bit: u32,
    rx_bck: u32,
    rx_ws: u32,
    // RX clock divider.
    rx_bck_div: u32,
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
            rx_in: [0; FIFO_DEPTH],
            rx_in_count: 0,
            int_raw: 0,
            int_ena: 0,
            tx_busy: false,
            tx_shift: 0,
            tx_bit: 0,
            tx_slot: 0,
            tx_bck: 0,
            tx_ws: 0,
            tx_sd: 0,
            tx_bck_div: 1,
            tx_pdm_acc: 0,
            rx_busy: false,
            rx_shift: 0,
            rx_recv: 0,
            rx_bit: 0,
            rx_bck: 0,
            rx_ws: 0,
            rx_bck_div: 1,
        }
    }

    fn idx_of(off: u32) -> usize {
        ((off & 0xFFF) / 4) as usize
    }

    fn tx_bits(&self) -> u32 {
        let f = (self.regs[Self::idx_of(TX_CONF1)] >> 13) & 0x1F;
        if f == 0 { 16 } else { f + 1 }
    }

    fn rx_bits(&self) -> u32 {
        let f = (self.regs[Self::idx_of(RX_CONF1)] >> 13) & 0x1F;
        if f == 0 { 16 } else { f + 1 }
    }

    fn loopback(&self) -> bool {
        self.regs[Self::idx_of(TX_CONF)] & SIG_LOOPBACK != 0
    }

    fn tx_slave(&self) -> bool {
        self.regs[Self::idx_of(TX_CONF)] & TX_SLAVE_MOD != 0
    }

    fn rx_slave(&self) -> bool {
        self.regs[Self::idx_of(RX_CONF)] & RX_SLAVE_MOD != 0
    }

    fn tx_tdm(&self) -> bool {
        self.regs[Self::idx_of(TX_CONF)] & TX_TDM_EN != 0
    }

    fn tx_pdm(&self) -> bool {
        self.regs[Self::idx_of(TX_CONF)] & TX_PDM_EN != 0
    }

    /// Number of slots (channels) per I2S frame for TX.
    fn num_tx_slots(&self) -> u32 {
        if self.tx_tdm() {
            // tx_tdm_tot_chan_num is 0-based; slots = tot + 1.
            ((self.regs[Self::idx_of(TX_TDM_CTRL)] >> 16) & 0xF) + 1
        } else {
            match (self.regs[Self::idx_of(TX_CONF)] >> 24) & 0x7 {
                0 => 1,     // mono (single channel)
                1 | 2 => 2, // stereo / 2-channel
                _ => 4,     // 4-channel
            }
        }
    }

    /// BCK half-cycle period in emulator steps (1 = one bit per step).
    fn tx_bck_divisor(&self) -> u32 {
        let d = self.regs[Self::idx_of(TX_CLKM_DIV_CONF)] & 0xFF;
        if d == 0 { 1 } else { d }
    }

    fn rx_bck_divisor(&self) -> u32 {
        let d = self.regs[Self::idx_of(RX_CLKM_DIV_CONF)] & 0xFF;
        if d == 0 { 1 } else { d }
    }

    /// First-order sigma-delta PDM bit for the current TX sample. The sample
    /// is treated as an unsigned offset-binary value in `[0, 2^bits)`, so 0
    /// maps to all-zero density and `2^bits - 1` to all-one density.
    fn tx_pdm_bit(&mut self) -> u32 {
        let bits = self.tx_bits();
        let mask = if bits >= 32 {
            0xFFFF_FFFF
        } else {
            (1u32 << bits) - 1
        };
        let sample = (self.tx_shift & mask) as i32;
        let half = 1i32 << (bits - 1);
        self.tx_pdm_acc = self.tx_pdm_acc.wrapping_add(sample - half);
        let b = if self.tx_pdm_acc >= 0 { 1i32 } else { 0i32 };
        self.tx_pdm_acc = self
            .tx_pdm_acc
            .wrapping_sub(if b == 1 { half } else { -half });
        b as u32
    }

    /// Push a word into the injected-RX source FIFO (used as the RX data
    /// source when there is no external codec, e.g. in tests).
    pub fn inject_rx(&mut self, word: u32) {
        if self.rx_in_count < FIFO_DEPTH {
            self.rx_in[self.rx_in_count] = word;
            self.rx_in_count += 1;
        }
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let o = offset & 0xFFF;
        match o {
            INT_RAW => self.int_raw,
            INT_ST => self.int_raw & self.int_ena,
            INT_ENA => self.int_ena,
            FIFO => {
                // RX pop.
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
                if value & RX_START_BIT != 0 && !self.rx_busy && !self.loopback() {
                    self.start_rx();
                }
            }
            TX_CONF => {
                self.regs[Self::idx_of(o)] = value;
                if value & TX_START_BIT != 0 && !self.tx_busy {
                    self.tx_busy = true;
                    self.tx_bit = 0;
                    self.tx_slot = 0;
                    self.tx_bck = 0;
                    self.tx_ws = 0;
                    self.tx_sd = 0;
                    self.tx_bck_div = self.tx_bck_divisor();
                    if self.tx_count > 0 {
                        self.tx_shift = self.tx_fifo[0];
                        for i in 1..self.tx_count {
                            self.tx_fifo[i - 1] = self.tx_fifo[i];
                        }
                        self.tx_count -= 1;
                    } else {
                        self.tx_shift = 0;
                    }
                    if self.loopback() {
                        self.rx_busy = true;
                        self.rx_bit = 0;
                        self.rx_ws = 0;
                        self.rx_recv = 0;
                    }
                }
            }
            _ => {
                self.regs[Self::idx_of(o)] = value;
            }
        }
    }

    fn start_rx(&mut self) {
        self.rx_busy = true;
        self.rx_bit = 0;
        self.rx_bck = 0;
        self.rx_ws = 0;
        self.rx_recv = 0;
        self.rx_bck_div = self.rx_bck_divisor();
        if self.rx_in_count > 0 {
            self.rx_shift = self.rx_in[0];
            for i in 1..self.rx_in_count {
                self.rx_in[i - 1] = self.rx_in[i];
            }
            self.rx_in_count -= 1;
        } else {
            self.rx_shift = 0;
        }
    }

    /// Advance the serial shift paths by one emulator step.
    /// True while transmitting or receiving. The SoC skips `tick()`
    /// otherwise — `tick_tx`/`tick_rx` both return immediately when their
    /// busy flag is clear, so gating is behavior-preserving. (Conservative:
    /// a loopback-idle RX also keeps the gate open; its tick no-ops.)
    pub fn is_active(&self) -> bool {
        self.tx_busy || self.rx_busy
    }

    pub fn tick(&mut self) {
        self.tick_tx();
        self.tick_rx();
    }

    fn tick_tx(&mut self) {
        if !self.tx_busy {
            return;
        }
        // Slave mode relies on an external BCK/WS clock (not modeled).
        if self.tx_slave() {
            return;
        }
        // Clock generator: count down to the next BCK half-cycle.
        self.tx_bck_div = self.tx_bck_div.wrapping_sub(1);
        if self.tx_bck_div != 0 {
            return;
        }
        self.tx_bck_div = self.tx_bck_divisor();
        self.tx_bck ^= 1;
        if self.tx_bck == 1 {
            let bits = self.tx_bits();
            let order = (self.regs[Self::idx_of(TX_CONF)] & TX_BIT_ORDER) != 0;
            let pcm_bit = if order {
                (self.tx_shift >> self.tx_bit) & 1
            } else {
                (self.tx_shift >> (bits - 1 - self.tx_bit)) & 1
            };
            let bit = if self.tx_pdm() {
                self.tx_pdm_bit()
            } else {
                pcm_bit
            };
            self.tx_sd = bit;
            self.tx_bit += 1;
            if self.tx_bit >= bits {
                self.tx_bit = 0;
                let slots = self.num_tx_slots();
                self.tx_slot = (self.tx_slot + 1) % slots;
                self.tx_ws = self.tx_slot & 1;
                // Loopback: the word just transmitted is received.
                if self.loopback() && self.rx_count < FIFO_DEPTH {
                    self.rx_fifo[self.rx_count] = self.tx_shift;
                    self.rx_count += 1;
                }
                if self.tx_count > 0 {
                    self.tx_shift = self.tx_fifo[0];
                    for i in 1..self.tx_count {
                        self.tx_fifo[i - 1] = self.tx_fifo[i];
                    }
                    self.tx_count -= 1;
                } else {
                    self.tx_busy = false;
                    self.int_raw |= TX_DONE;
                    if self.loopback() {
                        self.rx_busy = false;
                        self.int_raw |= RX_DONE;
                    }
                }
            }
        }
    }

    fn tick_rx(&mut self) {
        if !self.rx_busy || self.loopback() {
            return; // loopback RX is filled from TX in tick_tx
        }
        if self.rx_slave() {
            return; // external clock not modeled
        }
        self.rx_bck_div = self.rx_bck_div.wrapping_sub(1);
        if self.rx_bck_div != 0 {
            return;
        }
        self.rx_bck_div = self.rx_bck_divisor();
        self.rx_bck ^= 1;
        if self.rx_bck == 1 {
            let bits = self.rx_bits();
            let order = (self.regs[Self::idx_of(RX_CONF)] & RX_BIT_ORDER) != 0;
            let bit = if order {
                (self.rx_shift >> self.rx_bit) & 1
            } else {
                (self.rx_shift >> (bits - 1 - self.rx_bit)) & 1
            };
            if order {
                self.rx_recv |= bit << self.rx_bit;
            } else {
                self.rx_recv |= bit << (bits - 1 - self.rx_bit);
            }
            self.rx_bit += 1;
            if self.rx_bit >= bits {
                self.rx_bit = 0;
                self.rx_ws ^= 1;
                if self.rx_count < FIFO_DEPTH {
                    self.rx_fifo[self.rx_count] = self.rx_recv;
                    self.rx_count += 1;
                }
                self.rx_recv = 0;
                if self.rx_in_count > 0 {
                    self.rx_shift = self.rx_in[0];
                    for i in 1..self.rx_in_count {
                        self.rx_in[i - 1] = self.rx_in[i];
                    }
                    self.rx_in_count -= 1;
                } else {
                    self.rx_busy = false;
                    self.int_raw |= RX_DONE;
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
        let (rx_bck, rx_ws) = if self.idx == 0 {
            (26u32, 27u32)
        } else {
            (31u32, 32u32)
        };
        if sig == bck {
            self.tx_bck
        } else if sig == ws {
            self.tx_ws
        } else if sig == sd {
            self.tx_sd
        } else if sig == rx_bck {
            self.rx_bck
        } else if sig == rx_ws {
            self.rx_ws
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
        // tx_bit_order defaults to 0 (MSB first); tx_bits_mod default -> 16 bits.
        d.write32(FIFO, 0x8000);
        d.write32(TX_CONF, TX_START_BIT);
        d.tick(); // bck -> 1 (rising), shifts MSB (bit15) = 1
        assert_eq!(d.signal_level(30), 1, "I2S1 SD (sig 30) = MSB first");
    }

    #[test]
    fn lsb_first_matches_word() {
        let mut d = I2s::new(0);
        d.write32(TX_CONF, TX_BIT_ORDER); // LSB first
        d.write32(FIFO, 0x0001);
        d.write32(TX_CONF, TX_BIT_ORDER | TX_START_BIT);
        d.tick();
        assert_eq!(d.signal_level(25), 1, "I2S0 SD = LSB first (bit0=1)");
    }

    #[test]
    fn clock_divisor_slows_bck() {
        let mut d = I2s::new(0);
        // div_num = 2 -> BCK toggles every 2 emulator steps.
        d.write32(TX_CLKM_DIV_CONF, 2);
        d.write32(FIFO, 0xFFFF);
        d.write32(TX_CONF, TX_START_BIT);
        d.tick(); // step1: div 2->1, no edge
        assert_eq!(d.signal_level(22), 0, "BCK still low after 1 step (div=2)");
        d.tick(); // step2: div 1->0, edge -> BCK=1
        assert_eq!(d.signal_level(22), 1, "BCK high after 2 steps (div=2)");
    }

    #[test]
    fn master_slave_mode_gates_clock() {
        let mut d = I2s::new(0);
        // Enable slave TX: no internal clock, so TX never advances.
        d.write32(TX_CONF, TX_SLAVE_MOD | TX_START_BIT);
        d.write32(FIFO, 0xABCD);
        for _ in 0..64 {
            d.tick();
        }
        assert!(d.tx_busy, "slave TX stays busy (no clock)");
        assert_eq!(d.signal_level(22), 0, "slave BCK stays 0");
    }

    #[test]
    fn tdm_multi_slot_shifts_multiple_words() {
        let mut d = I2s::new(0);
        // TDM with tot_chan_num=1 -> 2 slots; push 2 words then start once.
        d.write32(TX_TDM_CTRL, 1 << 16); // tot_chan_num = 1
        d.write32(INT_ENA, TX_DONE | RX_DONE);
        d.write32(FIFO, 0x1111);
        d.write32(FIFO, 0x2222);
        d.write32(TX_CONF, TX_TDM_EN | SIG_LOOPBACK | TX_START_BIT);
        for _ in 0..128 {
            d.tick();
        }
        assert_eq!(d.int_raw & TX_DONE, TX_DONE, "tx_done");
        assert_eq!(d.int_raw & RX_DONE, RX_DONE, "rx_done (loopback)");
        // Two slots per frame; both FIFO words looped back in order.
        assert_eq!(d.read32(FIFO), 0x1111, "slot0 word");
        assert_eq!(d.read32(FIFO), 0x2222, "slot1 word");
    }

    #[test]
    fn pdm_encodes_extremes() {
        let mut d = I2s::new(0);
        // PDM of max sample -> all 1s; min sample -> all 0s.
        d.write32(FIFO, 0xFFFF);
        d.write32(TX_CONF, TX_PDM_EN | TX_START_BIT);
        d.tick();
        assert_eq!(d.signal_level(25), 1, "PDM max -> SD=1");
        let mut d2 = I2s::new(0);
        d2.write32(FIFO, 0x0000);
        d2.write32(TX_CONF, TX_PDM_EN | TX_START_BIT);
        d2.tick();
        assert_eq!(d2.signal_level(25), 0, "PDM min -> SD=0");
    }

    #[test]
    fn injected_rx_fills_fifo_and_asserts_rx_done() {
        let mut d = I2s::new(0);
        d.inject_rx(0x1234);
        d.inject_rx(0x5678);
        d.write32(INT_ENA, RX_DONE);
        d.write32(RX_CONF, RX_START_BIT);
        for _ in 0..64 {
            d.tick();
        }
        assert!(!d.rx_busy, "RX should have finished");
        assert_eq!(d.int_raw & RX_DONE, RX_DONE, "rx_done set");
        // Read back the two received words.
        assert_eq!(d.read32(FIFO), 0x1234, "RX word 0");
        assert_eq!(d.read32(FIFO), 0x5678, "RX word 1");
    }

    #[test]
    fn loopback_tx_feeds_rx() {
        let mut d = I2s::new(1);
        d.write32(INT_ENA, TX_DONE | RX_DONE);
        d.write32(FIFO, 0xCAFE);
        d.write32(TX_CONF, SIG_LOOPBACK | TX_START_BIT);
        for _ in 0..64 {
            d.tick();
        }
        assert_eq!(d.int_raw & TX_DONE, TX_DONE, "tx_done");
        assert_eq!(d.int_raw & RX_DONE, RX_DONE, "rx_done (loopback)");
        assert_eq!(d.read32(FIFO), 0xCAFE, "loopback RX == TX word");
    }

    #[test]
    fn loopback_after_prior_tx() {
        // Mirror the firmware poke sketch: a plain TX, then a loopback TX.
        let mut d = I2s::new(0);
        d.write32(INT_ENA, TX_DONE | RX_DONE);
        d.write32(INT_CLR, 0xF);
        d.write32(FIFO, 0xABCD);
        d.write32(TX_CONF, TX_START_BIT);
        for _ in 0..64 {
            d.tick();
        }
        assert_eq!(d.int_raw & TX_DONE, TX_DONE);
        d.write32(INT_CLR, 0xF);
        d.write32(FIFO, 0xCAFE);
        d.write32(TX_CONF, SIG_LOOPBACK | TX_START_BIT);
        for _ in 0..64 {
            d.tick();
        }
        assert_eq!(d.int_raw & RX_DONE, RX_DONE, "rx_done after prior tx");
        assert_eq!(d.read32(FIFO), 0xCAFE, "loopback rx word");
    }

    #[test]
    fn config_registers_round_trip() {
        let mut d = I2s::new(0);
        d.write32(TX_CONF1, 0x1234_0000);
        assert_eq!(d.read32(TX_CONF1), 0x1234_0000);
        d.write32(TX_CLKM_DIV_CONF, 0x0000_00AB);
        assert_eq!(d.read32(TX_CLKM_DIV_CONF), 0x0000_00AB);
        d.write32(TX_TDM_CTRL, 0xDEAD_BEEF);
        assert_eq!(d.read32(TX_TDM_CTRL), 0xDEAD_BEEF);
        d.write32(0x40, 0x1357_9246); // TX_PCM2PDM_CONF
        assert_eq!(d.read32(0x40), 0x1357_9246);
    }
}
