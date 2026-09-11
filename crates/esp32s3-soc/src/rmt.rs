//! ESP32-S3 RMT (Remote Control Transceiver) model — TX path (channels 0..3)
//! plus RX capture (channels 4..7).
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
//! RX channels (4..7): enabled by `chmconf1[m].rx_en`, they sample matrix
//! input 81+m once per tick (widths in the same channel-tick quantum the TX
//! side emits, so TX->RX pad loopback round-trips), pack edges into RMTMEM
//! items, and raise `rx_end` (raw bit 16+m) on idle timeout / rx_lim / a
//! full block. Glitch filter, wrap mode and mem_owner hand-off are modeled;
//! carrier modulation/demodulation is not.
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
// chmconf0[m] / chmconf1[m] register indices (m = 0..3, HW RX ch 4..7).
const CHMCONF0: usize = 0x30 / 4; // 12
const CHMCONF1: usize = 0x34 / 4; // 13
// chnstatus[n] indices.
const CHNSTATUS: usize = 0x50 / 4; // 20
// chm_rx_lim[m] indices.
const CHM_RX_LIM: usize = 0xB0 / 4; // 44
const INT_RAW: usize = 0x70 / 4; // 28
const INT_ENA: usize = 0x78 / 4; // 30
const CHN_TX_LIM: usize = 0xA0 / 4; // 40

// chnconf0 field bits (TRM RMT_CHnCONF0).
const TX_START: u32 = 1 << 0;
const IDLE_OUT_LV: u32 = 1 << 5;
const IDLE_OUT_EN: u32 = 1 << 6;
// Carrier modulation (TRM RMT_CHnCONF0 + RMT_CHnCARRIER_DUTY @ 0x80+4n):
// CARRIER_EFF_EN (bit 20): 1 = modulate only while sending data (reset),
// 0 = modulate at all states (always-on, even when idle). CARRIER_EN
// (bit 21) gates; CARRIER_OUT_LV (bit 22) selects the modulated level
// (1 = high phases carry the wave, 0 = low phases). Duty = high [31:16] /
// low [15:0] carrier clocks; a zero total disables (covers the reset state
// alongside EN=0, so unprogrammed channels are bit-identical to before).
const CARRIER_EFF_EN: u32 = 1 << 20;
const CARRIER_EN: u32 = 1 << 21;
const CARRIER_OUT_LV: u32 = 1 << 22;
const CARRIER_DUTY_BASE: usize = 0x80 / 4; // word idx 32, stride 1/chn
// tx_conti_mode (bit 15): continuous transmission — reload item 0 on end
// instead of raising tx_end.  The IDF `rmt_transmit` loop_count path uses
// tx_loop_cnt_en (bit 14) + tx_loop_cnt (bits [23:16]) == 0 for infinite
// looping; both mechanisms keep the FSM re-reading RMTMEM.
const TX_CONTI_MODE: u32 = 1 << 15;
const TX_LOOP_CNT_EN: u32 = 1 << 14;

// GPIO-matrix output signal indices for TX channels 0..3 (gpio_sig_map.h
// RMT_SIG_OUT0..3 = 81..84).
pub const RMT_TX_SIGNAL_BASE: u32 = 81;
// GPIO-matrix input signal base for RX HW channels 4..7: channel (4+m)
// samples input 81+m (mirrors the TX numbering; 81..84 are the only RMT
// inputs the signal map defines).
pub const RMT_RX_SIGNAL_BASE: u32 = 81;
// RX-end raw interrupt bit for HW channel (4+m): bit (16+m).
const RX_END_BIT: u32 = 16;
// chmconf1 field bits (rmt_struct.h chmconf1).
const RX_EN: u32 = 1 << 0;
const MEM_WR_RST: u32 = 1 << 1;
const APB_MEM_RST: u32 = 1 << 2;
const MEM_OWNER: u32 = 1 << 3;
const RX_FILTER_EN: u32 = 1 << 4;
const RX_FILTER_THRES_SHIFT: u32 = 5; // [12:5]
const MEM_RX_WRAP: u32 = 1 << 13;
// RX carrier demodulation (TRM RMT_CHmCONF0): CARRIER_EN (bit 28) latches
// the envelope, CARRIER_OUT_LV (bit 29) selects which level is the burst
// (1 = high bursts demodulate to steady high). Unprogrammed channels keep
// bit 28 clear (our reset), so existing captures are untouched.
const DEMOD_EN: u32 = 1 << 28;
const DEMOD_OUT_LV: u32 = 1 << 29;
// Demodulated opposite-level persistence: a lone opposite sample (carrier
// aliasing through the 32-tick sampling quantum) holds the burst level;
// two in a row flip it (real transition, ~64-tick edge uncertainty).
/// Demodulator lock/release runs, in sampling quanta (one quantum =
/// TICKS_PER_STEP channel ticks per step). Lock is fast (a real edge
/// asserts within ~2 quanta); release is slow on purpose: a 10/10-tick
/// carrier sampled at a 32-tick quantum aliases into runs of up to a few
/// consecutive gap samples, which must NOT split the envelope, while a
/// genuine end-of-burst (hundreds of steady quanta) still releases a few
/// hundred ticks late (inside envelope tolerances). Silicon's exact
/// demodulator thresholds are unknown; this is a tuned approximation.
const DEMOD_LOCK: u32 = 2;
const DEMOD_RELEASE: u32 = 8;

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
    level: u32, // current data level (0/1), pre-carrier
    num_items: usize,
    /// Carrier phase in channel ticks (advanced per consumed tick).
    car_phase: u32,
}

const NUM_RX_CH: usize = 4;
const RX_HW_BASE: usize = 4;

/// RX capture state for logical channel m (HW channel 4+m).
#[derive(Default)]
struct RxCh {
    /// Capture running (between rx_en start and an end condition).
    active: bool,
    /// rx_en rising edge seen (one-shot per write, mirroring TX_START).
    armed: bool,
    /// Filtered level of the open pulse.
    flevel: u32,
    /// Consecutive sub-ticks disagreeing with `flevel` (glitch filter).
    fcount: u32,
    /// Open pulse width in channel ticks.
    pwidth: u32,
    /// Ticks since the last accepted edge (idle timeout).
    idle: u32,
    /// Next item slot in this channel's RAM block.
    item_idx: usize,
    /// Half (0/1) of the open item being filled.
    half: u32,
    /// Demodulated opposite-level run: consecutive quanta the raw input
    /// read opposite the currently reported level (singletons are carrier
    /// aliasing and hold the level).
    dm_count: u32,
    /// Currently reported demodulated level (optimistic `burst` at arming).
    dm_lvl: u32,
}

/// The RMT module.
pub struct Rmt {
    regs: [u32; REG_COUNT],
    /// Item RAM: 512 items (8 channels x 64).  Item (ch, i) at `ch*64 + i`.
    mem: [u32; MEM_ITEMS],
    tx: [TxCh; NUM_TX_CH],
    rx: [RxCh; NUM_RX_CH],
}

impl Default for Rmt {
    fn default() -> Self {
        Self::new()
    }
}

impl Rmt {
    pub fn new() -> Self {
        let mut regs = [0; REG_COUNT];
        // Silicon reset defaults (rmt_struct.h) for the RX channels:
        // div_cnt = 2, idle_thres = 32767, mem_size = 1 block, rx_lim = 128.
        // Without the idle default a capture would never time out.
        for m in 0..NUM_RX_CH {
            regs[CHMCONF0 + m * 2] = 2 | (32767 << 8) | (1 << 24);
            regs[CHM_RX_LIM + m] = 128;
        }
        Self {
            regs,
            mem: [0; MEM_ITEMS],
            tx: Default::default(),
            rx: Default::default(),
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
            // Deterministic carrier start (each transmission begins at the
            // rising half of the wave).
            car_phase: 0,
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
                        // One-shot transmit consumes the start pulse: clear
                        // TX_START so a later write retriggers (a second
                        // write of an already-set bit would otherwise be a
                        // no-op edge-wise). Single-shot users never observe
                        // this; repeated transmissions require it.
                        self.regs[CHNCONF0 + ch] &= !TX_START;
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
            self.tx[ch].car_phase = self.tx[ch].car_phase.wrapping_add(1);
            ticks -= 1;
        }
    }

    /// Carrier duty (high, low) in channel ticks for TX channel `ch`.
    /// Approximated 1:1 from the programmed module-clock ticks (exact at
    /// divider 1; proportionally faster at higher dividers — the validated
    /// property is visible modulation, not the absolute carrier rate,
    /// which the emulator's untimed step model cannot resolve anyway).
    fn carrier_duty(&self, ch: usize) -> (u32, u32) {
        let duty = self.regs[CARRIER_DUTY_BASE + ch];
        (duty & 0xFFFF, (duty >> 16) & 0xFFFF)
    }

    /// Carrier active for TX channel `ch`: enabled with a nonzero period.
    /// (Reset state is duty 0/EN APP-side 0 here, so this is false until
    /// firmware programs it; existing waveforms are untouched.)
    fn carrier_on(&self, ch: usize) -> bool {
        let (high, low) = self.carrier_duty(ch);
        self.regs[CHNCONF0 + ch] & CARRIER_EN != 0 && high + low > 0
    }

    /// Always-on carrier (EFF_EN clear): modulates even while idle. Gated
    /// like `carrier_on` (duty programmed).
    fn carrier_always(&self, ch: usize) -> bool {
        self.carrier_on(ch) && self.regs[CHNCONF0 + ch] & CARRIER_EFF_EN == 0
    }

    /// Modulated output level of TX channel `ch` (data level with the
    /// carrier wave applied during the configured level).
    fn tx_out(&self, ch: usize) -> u32 {
        let level = self.tx[ch].level;
        if !self.carrier_on(ch) {
            return level;
        }
        if !self.tx[ch].active && self.regs[CHNCONF0 + ch] & CARRIER_EFF_EN != 0 {
            // Data-state-only modulation while idle: steady idle level.
            return level;
        }
        let lv = (self.regs[CHNCONF0 + ch] & CARRIER_OUT_LV) >> 22;
        if level != lv {
            return level;
        }
        let (high, low) = self.carrier_duty(ch);
        if self.tx[ch].car_phase % (high + low) < high {
            1
        } else {
            0
        }
    }

    /// True when any TX channel is mid-transfer (or driving an always-on
    /// carrier while idle). The SoC skips `tick()` otherwise.
    pub fn is_active(&self) -> bool {
        self.tx.iter().any(|t| t.active)
            || (0..NUM_TX_CH).any(|ch| self.carrier_always(ch))
            || self.rx_pending()
    }

    /// Advance all active TX channels by `TICKS_PER_STEP` (plus idle
    /// always-on carrier phases, which modulate with no data running).
    pub fn tick(&mut self) {
        for ch in 0..NUM_TX_CH {
            if self.tx[ch].active {
                self.advance(ch, TICKS_PER_STEP);
            } else if self.carrier_always(ch) {
                let t = &mut self.tx[ch];
                t.car_phase = t.car_phase.wrapping_add(TICKS_PER_STEP);
            }
        }
    }

    /// Any RX channel armed or capturing (tick gate).
    pub fn rx_pending(&self) -> bool {
        self.rx.iter().any(|r| r.active || r.armed)
    }

    /// Close the open pulse of RX channel `m` into its RAM block, advancing
    /// halves/items. Returns true when an end condition fires (rx_lim or a
    /// full block without wrap).
    fn rx_close_pulse(&mut self, m: usize, level: u32, width: u32) -> bool {
        let conf0 = self.regs[CHMCONF0 + m * 2];
        let conf1 = self.regs[CHMCONF1 + m * 2];
        let mem_blocks = (((conf0 >> 24) & 0xF) as usize).max(1);
        let capacity = mem_blocks * ITEMS_PER_CH;
        let rx_lim = (self.regs[CHM_RX_LIM + m] & 0x1FF) as usize;
        let wrap = conf1 & MEM_RX_WRAP != 0;
        let slot = (RX_HW_BASE + m) * ITEMS_PER_CH + self.rx[m].item_idx;
        let dur = width.min(0x7FFF);
        if self.rx[m].half == 0 {
            self.mem[slot] = dur | (level << 15);
            self.rx[m].half = 1;
            return false;
        }
        self.mem[slot] |= dur << 16 | (level << 31);
        self.rx[m].half = 0;
        self.rx[m].item_idx += 1;
        if rx_lim > 0 && self.rx[m].item_idx >= rx_lim {
            return true;
        }
        if self.rx[m].item_idx >= capacity {
            if wrap {
                self.rx[m].item_idx = 0;
                return false;
            }
            return true;
        }
        false
    }

    /// Finish an RX capture: flush the open pulse, raise rx_end, hand the
    /// RAM block back to software. One-shot per rx_en write (mirrors the
    /// TX_START auto-clear); the driver re-enables explicitly.
    fn rx_finish(&mut self, m: usize) {
        // The open pulse (possibly zero-width right after an edge) lands in
        // the current half, like the hardware writer offset advancing past
        // partial items.
        if self.rx[m].pwidth > 0 {
            let (lv, w) = (self.rx[m].flevel, self.rx[m].pwidth);
            let slot = (RX_HW_BASE + m) * ITEMS_PER_CH + self.rx[m].item_idx;
            let dur = w.min(0x7FFF);
            if self.rx[m].half == 0 {
                self.mem[slot] = dur | (lv << 15);
                self.rx[m].half = 1;
            } else {
                self.mem[slot] |= dur << 16 | (lv << 31);
                self.rx[m].item_idx += 1;
                self.rx[m].half = 0;
            }
            self.rx[m].pwidth = 0;
        }
        self.rx[m].active = false;
        self.rx[m].armed = false;
        self.regs[INT_RAW] |= 1 << (RX_END_BIT + m as u32);
        self.regs[CHMCONF1 + m * 2] &= !MEM_OWNER;
    }

    /// Carrier-demodulated sample for RX channel `m`: while demodulation
    /// is enabled, lone opposite-level samples are carrier aliasing through
    /// the 32-tick sampling quantum and hold the burst level; a run of
    /// DEMOD_LOCK opposite samples toward the burst level locks it, while
    /// leaving the burst level needs a run of DEMOD_RELEASE (a real
    /// end-of-burst; carrier gap runs are shorter). Reporting starts
    /// optimistic (burst level) so a burst opening right at arming is not
    /// missed. Disabled = transparent passthrough (existing captures
    /// bit-identical).
    fn demod_sample(&mut self, m: usize, raw: u32) -> u32 {
        let conf0 = self.regs[CHMCONF0 + m * 2];
        if conf0 & DEMOD_EN == 0 {
            return raw;
        }
        let burst = (conf0 & DEMOD_OUT_LV) >> 29;
        let cur = self.rx[m].dm_lvl;
        if raw == cur {
            self.rx[m].dm_count = 0;
            cur
        } else {
            self.rx[m].dm_count += 1;
            let need = if cur == burst {
                DEMOD_RELEASE
            } else {
                DEMOD_LOCK
            };
            if self.rx[m].dm_count >= need {
                self.rx[m].dm_lvl = raw;
                self.rx[m].dm_count = 0;
                raw
            } else {
                cur
            }
        }
    }

    /// Advance RX captures by one step. `input(sig)` resolves an RMT input
    /// signal index to its pad level (undriven inputs read pull-up high).
    /// Widths are measured in the same channel-tick quantum the TX side
    /// emits, so TX->RX pad loopback round-trips.
    pub fn tick_rx<F: Fn(u32) -> u32>(&mut self, input: &F) {
        for m in 0..NUM_RX_CH {
            if self.rx[m].armed && !self.rx[m].active {
                // Capture starts on the rx_en edge: the first sample both
                // baselines the filtered level and counts its first quantum.
                self.rx[m].active = true;
                let first = input(RMT_RX_SIGNAL_BASE + m as u32) & 1;
                let base = self.demod_sample(m, first);
                self.rx[m].flevel = base;
                self.rx[m].fcount = 0;
                self.rx[m].pwidth = TICKS_PER_STEP;
                self.rx[m].item_idx = 0;
                self.rx[m].half = 0;
                self.rx[m].idle = 0;
                self.regs[CHMCONF1 + m * 2] |= MEM_OWNER;
                continue;
            }
            if !self.rx[m].active {
                continue;
            }
            let conf0 = self.regs[CHMCONF0 + m * 2];
            let conf1 = self.regs[CHMCONF1 + m * 2];
            let idle_thres = (conf0 >> 8) & 0x7FFF;
            let filter_on = conf1 & RX_FILTER_EN != 0;
            let filter_thres = (conf1 >> RX_FILTER_THRES_SHIFT) & 0xFF;
            let raw = input(RMT_RX_SIGNAL_BASE + m as u32) & 1;
            let sampled = self.demod_sample(m, raw);
            // Glitch filter: only transitions held for the threshold reach
            // the edge detector (disabled = every sample passes through).
            let mut edge = false;
            if sampled != self.rx[m].flevel {
                self.rx[m].fcount += TICKS_PER_STEP;
                if !filter_on || self.rx[m].fcount >= filter_thres {
                    self.rx[m].flevel = sampled;
                    self.rx[m].fcount = 0;
                    edge = true;
                }
            } else {
                self.rx[m].fcount = 0;
            }
            if edge {
                self.rx[m].idle = 0;
                // The closed pulse is the pre-flip level with the accrued width.
                let done = {
                    let pl = self.rx[m].flevel ^ 1;
                    let pw = self.rx[m].pwidth;
                    self.rx_close_pulse(m, pl, pw)
                };
                self.rx[m].pwidth = 0;
                if done {
                    self.rx_finish(m);
                }
            } else {
                self.rx[m].pwidth += TICKS_PER_STEP;
                self.rx[m].idle += TICKS_PER_STEP;
                if idle_thres != 0 && self.rx[m].idle >= idle_thres {
                    self.rx_finish(m);
                }
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
                return self.tx_out(ch);
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
            // chmconf0[m] @ 0x30+8m: while the channel is not capturing,
            // re-seed the optimistic demodulator level from DEMOD_OUT_LV
            // (firmware may arm via rx_en before programming demod bits).
            o if (0x30..=0x4C).contains(&o) && o % 8 == 0 => {
                let m = (o - 0x30) as usize / 8;
                self.regs[CHMCONF0 + m * 2] = value;
                if !self.rx[m].active {
                    self.rx[m].dm_lvl = (value & DEMOD_OUT_LV) >> 29;
                    self.rx[m].dm_count = 0;
                }
            }
            // chmconf1[m] @ 0x34+8m: rx_en rising edge arms a capture (the
            // writer pointer resets); mem reset bits reset it too.
            o if (0x34..=0x4C).contains(&o) && o % 8 == 4 => {
                let m = (o - 0x34) as usize / 8;
                let old = self.regs[CHMCONF1 + m * 2];
                self.regs[CHMCONF1 + m * 2] = value;
                if value & (MEM_WR_RST | APB_MEM_RST) != 0 {
                    self.rx[m].item_idx = 0;
                    self.rx[m].half = 0;
                }
                if value & RX_EN != 0 && old & RX_EN == 0 {
                    self.rx[m].armed = true;
                    self.rx[m].active = false;
                    self.rx[m].item_idx = 0;
                    self.rx[m].half = 0;
                    // A re-arm after a finished capture must not inherit the
                    // old capture's counters, or the stale idle count fires
                    // rx_end on the first tick of the new capture.
                    self.rx[m].idle = 0;
                    self.rx[m].pwidth = 0;
                    self.rx[m].flevel = 0;
                    // Optimistic demodulator start: assume the burst level
                    // until a DEMOD_RELEASE run disproves it, so a burst
                    // opening right at arming is not missed.
                    let c0 = self.regs[CHMCONF0 + m * 2];
                    self.rx[m].dm_lvl = (c0 & DEMOD_OUT_LV) >> 29;
                    self.rx[m].dm_count = 0;
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
