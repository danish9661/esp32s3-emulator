//! ESP32-S3 touch sensor controller (capacitive touch pads 1..14).
//!
//! Register block: the SENS page (`DR_REG_SENS_BASE = 0x6000_8800`,
//! `soc/sens_struct.h` `sar_touch_*`). Offsets below are relative to
//! SENS_BASE; the SoC routes SENS offsets `0x5C..0x400` here, the ADC
//! oneshot owns `0x00..0x5C` (no overlap: its highest register is
//! `SENS_SAR_SLAVE_ADDR1` @ `0x40`).
//! ```text
//! 0x5C CONF      outen[14:0] pad enable, status_clr[15], data_sel[17:16]
//! 0x60 DENOISE   denoise_data[21:0]
//! 0x64..0x98 THRES1..14  thresh[21:0] per pad (finger threshold)
//! 0x9C CHN_ST    pad_active[14:0] (live), channel_clr[29:15], meas_done[31]
//! 0xA0 STATUS0   denoise data / scan channel (plain store here)
//! 0xA4..0xD8 STATUS1..14  pad_data[21:0] per pad (the touch counter)
//! 0xDC SLP_STATUS / 0xE0 APPR_STATUS (plain stores)
//! ```
//! Model: synchronous (like SHA BUSY): every pad always reads "measured"
//! (`meas_done` = 1, the flag `touch_pad_read_raw_data` polls), and each
//! pad counter reads its host-injected value (`touch_inject`, default 0).
//! `pad_active` is live: pad N is active when its threshold is nonzero and
//! the counter is below it (S3 counters fall when touched). `data_sel`
//! (smooth/benchmark/raw views) is not modeled — one counter per pad.
//! Proximity mode: CONF.approach_padN (4-bit pad numbers) arm three approach
//! channels; each channel's APPR_STATUS counter saturates at 255 while its
//! pad is active and clears when released. SLP_STATUS latches the sleep-
//! entry triggering pad's counter (first active pad) for post-wake
//! identification. STATUS0 denoise_data reads the pad-0 injection
//! (`touch_inject(0, v)`); scan_curr is always 0 (no scan engine).
//! KNOWN LIMITATION: no interrupt is wired. On S3 the touch interrupt goes
//! through the ULP-coprocessor block (`SENS_COCPU_*`), which has no
//! interrupt-matrix source (`interrupts.h`); threshold *status* reads work,
//! the ISR path does not.

#![allow(clippy::identity_op)]

use alloc::vec::Vec;

/// Number of touch pads on ESP32-S3 (pads 1..14, GPIO1..GPIO14 1:1).
pub const TOUCH_PADS: usize = 14;
/// Register window size in bytes (SENS offsets `0x5C..0x15C`).
const REG_BYTES: usize = 0x100;
/// SENS_BASE-relative offsets (`soc/sens_reg.h`).
pub const TOUCH_CONF_OFF: u32 = 0x5C;
const TOUCH_STATUS0_OFF: u32 = 0xA0;
const TOUCH_SLP_STATUS_OFF: u32 = 0xDC;
const TOUCH_APPR_STATUS_OFF: u32 = 0xE0;
const TOUCH_THRES_BASE: u32 = 0x64;
pub const TOUCH_CHN_ST_OFF: u32 = 0x9C;
const TOUCH_STATUS_BASE: u32 = 0xA4;
/// CHN_ST bit: measurement done (always set: synchronous model).
/// `SENS_TOUCH_MEAS_DONE` = bit 31 (the flag the driver's oneshot wait
/// polls with `bltz`, verified by disassembly).
const MEAS_DONE: u32 = 1 << 31;
/// CONF.data_sel field (ignored: single counter per pad).
const _DATA_SEL_MASK: u32 = 0x3 << 16;
/// STATUS pad_data field width (22 bits).
const DATA_MASK: u32 = 0x3F_FFFF;
/// First SENS offset owned by this device (carve-out from the ADC window).
pub const TOUCH_OFF_START: u32 = 0x5C;

#[derive(Clone)]
pub struct Touch {
    regs: Vec<u32>,
    injected: [u32; TOUCH_PADS + 1],
    /// Proximity approach counters (APPR_STATUS padN_cnt, saturate 255).
    appr: [u8; 3],
    /// Sleep-entry captured pad counter (SLP_STATUS slp_data).
    slp_data: u32,
}

impl Touch {
    pub fn new() -> Self {
        Self {
            regs: alloc::vec![0; REG_BYTES / 4],
            injected: [0; TOUCH_PADS + 1],
            appr: [0; 3],
            slp_data: 0,
        }
    }

    fn idx(off: u32) -> usize {
        ((off - TOUCH_OFF_START) / 4) as usize
    }

    /// Host injection: the counter pad 0..=14 reports (22-bit counter that
    /// falls when touched; the sketch asserts the exact injected value).
    /// Pad 0 is the denoise channel (STATUS0 denoise_data); pads 1..=14
    /// are the touch pads (STATUS1..14).
    pub fn inject(&mut self, pad: usize, value: u32) {
        if (0..=TOUCH_PADS).contains(&pad) {
            self.injected[pad] = value & DATA_MASK;
        }
    }

    /// Configured approach pads (CONF.approach_padN, 4-bit; 0 = disabled).
    fn approach_pads(&self) -> [usize; 3] {
        let conf = self.regs[Self::idx(TOUCH_CONF_OFF)];
        [
            ((conf >> 28) & 0xF) as usize,
            ((conf >> 24) & 0xF) as usize,
            ((conf >> 20) & 0xF) as usize,
        ]
    }

    /// Advance the proximity counters by `cycles` ticks: an armed channel
    /// (nonzero pad) saturates at 255 while its pad reads active, and
    /// clears the moment it releases.
    pub fn tick(&mut self, cycles: u64) {
        let pads = self.approach_pads();
        if pads == [0, 0, 0] {
            return;
        }
        let active = self.active();
        for (i, pad) in pads.iter().enumerate() {
            if *pad == 0 || *pad > TOUCH_PADS {
                self.appr[i] = 0;
            } else if active & (1 << (pad - 1)) != 0 {
                self.appr[i] = self.appr[i].saturating_add(cycles.min(255) as u8);
            } else {
                self.appr[i] = 0;
            }
        }
    }

    /// Latch the sleep-entry triggering pad's counter (first active pad,
    /// like the wakeup evaluation) into SLP_STATUS for post-wake reads.
    pub fn set_sleep_data(&mut self) {
        self.slp_data = 0;
        for pad in 1..=TOUCH_PADS {
            if self.thresh(pad) != 0 && self.injected[pad] < self.thresh(pad) {
                self.slp_data = self.injected[pad];
                break;
            }
        }
    }

    fn thresh(&self, pad: usize) -> u32 {
        self.regs[((TOUCH_THRES_BASE - TOUCH_OFF_START) / 4) as usize + pad - 1] & DATA_MASK
    }

    /// Live pad-active bitmap (CHN_ST[14:0]): pad active when its threshold
    /// is programmed and the counter reads below it.
    fn active(&self) -> u32 {
        let mut bits = 0u32;
        for pad in 1..=TOUCH_PADS {
            let th = self.thresh(pad);
            if th != 0 && self.injected[pad] < th {
                bits |= 1 << (pad - 1);
            }
        }
        bits
    }

    /// True while any touch pad is touched (threshold programmed and counter
    /// below it). Used by deep-sleep entry to evaluate a touch wakeup.
    pub fn any_touched(&self) -> bool {
        self.active() != 0
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            TOUCH_CHN_ST_OFF => {
                // Live active bitmap + always-done (synchronous model).
                // channel_clr reads back the stored (cleared-by-write) bits.
                (self.regs[Self::idx(offset)] & 0x3FFF_8000) | self.active() | MEAS_DONE
            }
            o if (TOUCH_STATUS_BASE..TOUCH_STATUS_BASE + 4 * TOUCH_PADS as u32).contains(&o)
                && o.is_multiple_of(4) =>
            {
                // STATUSn: the pad counter (host-injected).
                let pad = ((o - TOUCH_STATUS_BASE) / 4) as usize + 1;
                (self.regs[Self::idx(o)] & !DATA_MASK) | self.injected[pad]
            }
            TOUCH_STATUS0_OFF => {
                // denoise_data = pad-0 injection; scan_curr = 0 (no scan
                // engine); upper reserved bits read stored.
                (self.regs[Self::idx(TOUCH_STATUS0_OFF)] & !DATA_MASK) | self.injected[0]
            }
            TOUCH_SLP_STATUS_OFF => {
                // Sleep-captured pad counter (latched at sleep entry when
                // touch wakeup fires); debounce reads 0.
                self.slp_data & DATA_MASK
            }
            TOUCH_APPR_STATUS_OFF => {
                // Live approach counters (pad2 | pad1 | pad0 | slp=0).
                ((self.appr[2] as u32) << 24)
                    | ((self.appr[1] as u32) << 16)
                    | ((self.appr[0] as u32) << 8)
            }
            _ if offset >= TOUCH_OFF_START
                && offset < TOUCH_OFF_START + REG_BYTES as u32
                && offset.is_multiple_of(4) =>
            {
                self.regs[Self::idx(offset)]
            }
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            TOUCH_CHN_ST_OFF => {
                // channel_clr[29:15] is write-1-to-clear on the stored bits
                // (the live active bitmap itself is computed, not latched;
                // meas_done[30] is read-only status, always set).
                let idx = Self::idx(offset);
                self.regs[idx] &= !(value & 0x3FFF_8000);
            }
            _ if offset >= TOUCH_OFF_START
                && offset < TOUCH_OFF_START + REG_BYTES as u32
                && offset.is_multiple_of(4) =>
            {
                self.regs[Self::idx(offset)] = value;
            }
            _ => {}
        }
    }
}

impl Default for Touch {
    fn default() -> Self {
        Self::new()
    }
}
