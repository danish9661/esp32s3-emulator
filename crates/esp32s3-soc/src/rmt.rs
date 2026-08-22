//! ESP32-S3 RMT (Remote Control Transceiver) model — TX path.
//!
//! Register block at `0x6001_6000` (TRM RMT chapter), 0xD0 bytes; item RAM
//! (`RMTMEM`) at `0x6001_6800` = RMT_BASE + 0x800 (ESP-IDF `RMT_CHANNEL_MEM`
//! docs: channel n block starts at `RMT base + 0x800 + 64*4*n`).  The block
//! holds 64 32-bit items per channel; each item (`rmt_item32_t`) encodes two
//! pulses: `duration0[14:0], level0[15], duration1[30:16], level1[31]`.
//!
//! Only the TX direction (channels 0..3) is modeled.  A transmission is
//! started by writing `chnconf0[n].tx_start` (bit 0); the FSM reads items
//! from channel n's RAM block and drives the output level, raising the
//! per-channel `tx_end` interrupt (raw bit n) when all items are consumed.
//! RX channels (4..7) and carrier modulation are not modeled.
//!
//! The TX output is exposed to the GPIO matrix via signals RMT_SIG_OUT0..3 =
//! 81..84 (gpio_sig_map.h), so `gpio_output()` reflects the live level.

/// RMT register-block base (APB).
pub const RMT_BASE: u32 = 0x6001_6000;
/// RMT item-RAM base (`RMTMEM`): RMT_BASE + 0x800 (ESP-IDF RMT_CHANNEL_MEM).
pub const RMTMEM_BASE: u32 = 0x6001_6800;

const REG_COUNT: usize = 0xD0 / 4; // 52 registers
const ITEMS_PER_CH: usize = 64;
const NUM_TX_CH: usize = 4;
const MEM_ITEMS: usize = 8 * ITEMS_PER_CH; // 512 shared items (8 channels)

// chnconf0[n] register indices (n = 0..3, the TX channels).
const CHNCONF0: usize = 0x20 / 4; // 8
// chnstatus[n] indices.
const CHNSTATUS: usize = 0x50 / 4; // 20
const INT_RAW: usize = 0x70 / 4; // 28
const INT_ENA: usize = 0x78 / 4; // 30
const CHN_TX_LIM: usize = 0xA0 / 4; // 40

// chnconf0 field bits (TRM RMT_CHnCONF0).
const TX_START: u32 = 1 << 0;
const IDLE_OUT_LV: u32 = 1 << 5;
const IDLE_OUT_EN: u32 = 1 << 6;
// tx_conti_mode (bit 15): continuous transmission — reload item 0 on end
// instead of raising tx_end.  The IDF `rmt_transmit` loop_count path uses
// tx_loop_cnt_en (bit 14) + tx_loop_cnt (bits [23:16]) == 0 for infinite
// looping; both mechanisms keep the FSM re-reading RMTMEM.
const TX_CONTI_MODE: u32 = 1 << 15;
const TX_LOOP_CNT_EN: u32 = 1 << 14;

// GPIO-matrix output signal indices for TX channels 0..3 (gpio_sig_map.h
// RMT_SIG_OUT0..3 = 81..84).
pub const RMT_TX_SIGNAL_BASE: u32 = 81;

// RMT interrupt source number for the interrupt matrix (esp32s3 interrupts.h
// ETS_RMT_INTR_SOURCE = 40).
pub const RMT_INTR_SOURCE: u32 = 40;

/// RMT ticks advanced per emulator `tick()` call.  Paces a transmission so it
/// completes within a bounded number of steps without being instantaneous
/// (relative pulse widths are preserved; absolute timing is not modeled).
const TICKS_PER_STEP: u32 = 32;

#[derive(Default)]
struct TxCh {
    active: bool,
    conti: bool, // continuous (loop) mode
    item_idx: usize,
    pulse: u32, // 0 = duration0/level0, 1 = duration1/level1
    ticks_left: u32,
    level: u32, // current output level (0/1)
    num_items: usize,
}

/// The RMT module.
pub struct Rmt {
    regs: [u32; REG_COUNT],
    /// Item RAM: 512 items (8 channels x 64).  Item (ch, i) at `ch*64 + i`.
    mem: [u32; MEM_ITEMS],
    tx: [TxCh; NUM_TX_CH],
}

impl Default for Rmt {
    fn default() -> Self {
        Self::new()
    }
}

impl Rmt {
    pub fn new() -> Self {
        Self {
            regs: [0; REG_COUNT],
            mem: [0; MEM_ITEMS],
            tx: Default::default(),
        }
    }

    fn item(&self, ch: usize, i: usize) -> u32 {
        self.mem[ch * ITEMS_PER_CH + i]
    }

    fn idle_level(&self, ch: usize) -> u32 {
        let conf = self.regs[CHNCONF0 + ch];
        if conf & IDLE_OUT_EN != 0 {
            (conf & IDLE_OUT_LV) >> 5
        } else {
            0
        }
    }

    fn begin_tx(&mut self, ch: usize) {
        let conf = self.regs[CHNCONF0 + ch];
        let mem_size = ((conf >> 16) & 0xF) as usize;
        let mem_size = if mem_size == 0 { 1 } else { mem_size };
        let tx_lim = (self.regs[CHN_TX_LIM + ch] & 0x1FF) as usize;
        let num_items = if tx_lim > 0 {
            tx_lim
        } else {
            mem_size * ITEMS_PER_CH
        }
        .min(MEM_ITEMS);
        let num_items = num_items.max(1);

        let loop_en = (conf & TX_LOOP_CNT_EN) != 0 && ((conf >> 16) & 0xFF) == 0;
        let mut t = TxCh {
            active: true,
            conti: (conf & TX_CONTI_MODE) != 0 || loop_en,
            item_idx: 0,
            pulse: 0,
            ticks_left: 0,
            level: self.idle_level(ch),
            num_items,
        };
        // Load the first pulse.
        let item = self.item(ch, 0);
        t.level = (item >> 15) & 1;
        t.ticks_left = item & 0x7FFF;
        self.tx[ch] = t;
    }

    fn advance(&mut self, ch: usize, mut ticks: u32) {
        while ticks > 0 && self.tx[ch].active {
            if self.tx[ch].ticks_left == 0 {
                // Transition to the next pulse / item (no tick consumed).
                if self.tx[ch].pulse == 0 {
                    self.tx[ch].pulse = 1;
                    let item = self.item(ch, self.tx[ch].item_idx);
                    self.tx[ch].level = (item >> 31) & 1;
                    self.tx[ch].ticks_left = (item >> 16) & 0x7FFF;
                } else {
                    self.tx[ch].item_idx += 1;
                    if self.tx[ch].item_idx >= self.tx[ch].num_items {
                        if self.tx[ch].conti {
                            // Continuous mode: reload item 0 and keep running.
                            let item = self.item(ch, 0);
                            self.tx[ch].item_idx = 0;
                            self.tx[ch].pulse = 0;
                            self.tx[ch].level = (item >> 15) & 1;
                            self.tx[ch].ticks_left = item & 0x7FFF;
                            continue;
                        }
                        self.tx[ch].active = false;
                        self.regs[INT_RAW] |= 1 << ch;
                        self.tx[ch].level = self.idle_level(ch);
                        break;
                    }
                    let item = self.item(ch, self.tx[ch].item_idx);
                    self.tx[ch].pulse = 0;
                    self.tx[ch].level = (item >> 15) & 1;
                    self.tx[ch].ticks_left = item & 0x7FFF;
                }
                continue;
            }
            self.tx[ch].ticks_left -= 1;
            ticks -= 1;
        }
    }

    /// Advance all active TX channels by `TICKS_PER_STEP`.
    pub fn tick(&mut self) {
        for ch in 0..NUM_TX_CH {
            if self.tx[ch].active {
                self.advance(ch, TICKS_PER_STEP);
            }
        }
    }

    /// Raw TX-end interrupts (bit n = channel n done).
    pub fn int_st(&self) -> u32 {
        self.regs[INT_RAW] & self.regs[INT_ENA]
    }

    /// Level of a GPIO-matrix output signal (RMT_SIG_OUT0..3 = 81..84).
    pub fn signal_level(&self, sig: u32) -> u32 {
        if (RMT_TX_SIGNAL_BASE..RMT_TX_SIGNAL_BASE + NUM_TX_CH as u32).contains(&sig) {
            let ch = (sig - RMT_TX_SIGNAL_BASE) as usize;
            if ch < NUM_TX_CH {
                return self.tx[ch].level;
            }
        }
        0
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let off = offset & 0xFFF;
        if in_mem_range(off) {
            return self.mem[((off - 0x800) / 4) as usize];
        }
        match off {
            o if (0x20..=0x2C).contains(&o) && o % 4 == 0 => {
                self.regs[CHNCONF0 + (o - 0x20) as usize / 4]
            }
            o if (0x50..=0x5C).contains(&o) && o % 4 == 0 => {
                let ch = (o - 0x50) as usize / 4;
                let mut v = self.regs[CHNSTATUS + ch];
                // state field [24:22]: 0 = idle, 1 = running (modeled).
                let state = if self.tx[ch].active { 1 } else { 0 };
                v = (v & !(0x7 << 22)) | (state << 22);
                v
            }
            0x70 => self.regs[INT_RAW],
            0x74 => self.int_st(),
            0x78 => self.regs[INT_ENA],
            _ => {
                if off.is_multiple_of(4) && (off as usize / 4) < REG_COUNT {
                    self.regs[(off / 4) as usize]
                } else {
                    0
                }
            }
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let off = offset & 0xFFF;
        if in_mem_range(off) {
            self.mem[((off - 0x800) / 4) as usize] = value;
            return;
        }
        match off {
            o if (0x20..=0x2C).contains(&o) && o % 4 == 0 => {
                let ch = (o - 0x20) as usize / 4;
                let old = self.regs[CHNCONF0 + ch];
                self.regs[CHNCONF0 + ch] = value;
                if value & TX_START != 0 && old & TX_START == 0 {
                    self.begin_tx(ch);
                }
            }
            0x70 => { /* int_raw is set by hardware only */ }
            0x74 => { /* int_st is read-only */ }
            0x78 => self.regs[INT_ENA] = value,
            0x7C => self.regs[INT_RAW] &= !value,
            _ => {
                if off.is_multiple_of(4) && (off as usize / 4) < REG_COUNT {
                    self.regs[(off / 4) as usize] = value;
                }
            }
        }
    }
}

#[inline]
fn in_mem_range(off: u32) -> bool {
    (0x800..0x800 + MEM_ITEMS as u32 * 4).contains(&off)
}
