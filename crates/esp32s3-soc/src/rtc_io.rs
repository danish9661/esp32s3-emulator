//! ESP32-S3 RTC_IO (RTC GPIO) register block.
//!
//! Base `DR_REG_RTCIO_BASE = 0x6000_8400` (the `0x6000_8000` page + 0x400).
//! Per `soc/rtc_io_struct.h` the block covers `0x400..0x800` of that page:
//! `out`/`out_w1ts`/`out_w1tc` (RTC GPIO 0..21 output data), `enable`/
//! `enable_w1ts`/`enable_w1tc`, `status`/`status_w1ts`/`status_w1tc`, `in_val`
//! (RTC GPIO input levels), per-pad config registers (`rtc_padN`, `xtal_*_pad`,
//! `ext_wakeup0`, `sar_i2c_io`, `touch_ctrl`, `date`, ...).
//!
//! Modeled as a register store (software-defined defaults of 0) so firmware
//! writes round-trip. `out`/`enable`/`status` additionally honor the
//! write-1-to-set/clear registers (`*_w1ts`/`*_w1tc`) like real silicon. The
//! block is not boot-critical — returning 0 for unwritten registers is exactly
//! the legacy behavior (the page dispatch used to return 0 for the whole
//! 0x400..0x800 window), so this is boot-neutral.

pub const RTC_IO_BASE: u32 = 0x6000_8400;

// The block occupies off 0x400..0x800 in the 0x6000_8000 page = 0x400 bytes.
const REG_COUNT: usize = 0x400 / 4;

// Register offsets (relative to RTC_IO_BASE) for the w1ts/w1tc pairs.
const OUT_OFF: u32 = 0x00;
const OUT_W1TS_OFF: u32 = 0x04;
const OUT_W1TC_OFF: u32 = 0x08;
const ENABLE_OFF: u32 = 0x0C;
const ENABLE_W1TS_OFF: u32 = 0x10;
const ENABLE_W1TC_OFF: u32 = 0x14;
const STATUS_OFF: u32 = 0x18;
const STATUS_W1TS_OFF: u32 = 0x1C;
const STATUS_W1TC_OFF: u32 = 0x20;

// Word index of each data register within the regs array.
const OUT_IDX: usize = (OUT_OFF / 4) as usize;
const ENABLE_IDX: usize = (ENABLE_OFF / 4) as usize;
const STATUS_IDX: usize = (STATUS_OFF / 4) as usize;

pub struct RtcIo {
    regs: [u32; REG_COUNT],
}

impl Default for RtcIo {
    fn default() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
        }
    }
}

impl RtcIo {
    pub fn new() -> Self {
        Self::default()
    }

    fn idx(&self, offset: u32) -> usize {
        // Page dispatch passes the full 0x6000_8000-page offset (0x400..0x800);
        // index by (off & 0x3FF) / 4.
        ((offset & 0x3FF) / 4) as usize
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let o = offset & 0x3FF;
        // Reading the w1ts/w1tc registers is not meaningful on real silicon;
        // the data lives in the plain out/enable/status registers.
        match o {
            OUT_W1TS_OFF | OUT_W1TC_OFF | ENABLE_W1TS_OFF | ENABLE_W1TC_OFF | STATUS_W1TS_OFF
            | STATUS_W1TC_OFF => 0,
            _ => self.regs[self.idx(offset)],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let o = offset & 0x3FF;
        match o {
            OUT_W1TS_OFF => self.regs[OUT_IDX] |= value,
            OUT_W1TC_OFF => self.regs[OUT_IDX] &= !value,
            ENABLE_W1TS_OFF => self.regs[ENABLE_IDX] |= value,
            ENABLE_W1TC_OFF => self.regs[ENABLE_IDX] &= !value,
            STATUS_W1TS_OFF => self.regs[STATUS_IDX] |= value,
            STATUS_W1TC_OFF => self.regs[STATUS_IDX] &= !value,
            _ => {
                let i = self.idx(offset);
                if i < REG_COUNT {
                    self.regs[i] = value;
                }
            }
        }
    }
}
