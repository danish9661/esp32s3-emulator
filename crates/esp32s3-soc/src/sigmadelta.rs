//! ESP32-S3 Sigma-Delta modulator (`GPIO_SD`, `DR_REG_GPIO_SD_BASE =
//! 0x60004F00`).
//!
//! Register layout per `gpio_sd_struct.h`: `channel[8]` (`duty[7:0]`,
//! `prescale[15:8]`) at 0x00..0x1F, `cg` (`clk_en` bit 31) at 0x20, `misc`
//! (`function_clk_en` bit 30, `spi_swap` bit 31) at 0x24, `version` (`date`)
//! at 0x28. Output is routed through the GPIO-matrix signals
//! `GPIO_SD0..7_OUT_IDX` (93..100, `gpio_sig_map.h`).
//!
//! Model: the modulator produces a pulse-density output whose high fraction
//! over one 8-bit cycle equals `duty/256`. The phase counter advances by one
//! every `prescale+1` CPU ticks; the output bit is high while `phase < duty`.
//! The `cg`/`misc` clock-gate bits are not modeled (the clock is treated as
//! always running) — they are a power-gating detail irrelevant to functional
//! output in the emulator.

/// GPIO_SD register block base (`DR_REG_GPIO_SD_BASE`, TRM GPIO_SD memory map).
pub const GPIO_SD_BASE: u32 = 0x6000_4F00;

/// First GPIO-matrix output signal for the 8 Sigma-Delta channels
/// (`GPIO_SD0_OUT_IDX`, `gpio_sig_map.h`).
pub const SDM_SIGNAL_BASE: u32 = 93;

const CH_OFF: u32 = 0x00;
const CG_OFF: u32 = 0x20;
const MISC_OFF: u32 = 0x24;
const VERSION_OFF: u32 = 0x28;
const REG_COUNT: usize = 0x2C / 4; // channel[8]=0x20 + cg + misc + version

#[derive(Default)]
pub struct Sdm {
    regs: [u32; REG_COUNT],
    /// 8-bit phase accumulator per channel (wraps at 256); output is high
    /// while `phase < duty`.
    phase: [u8; 8],
    /// Prescale countdown per channel: advance `phase` every `prescale+1`
    /// ticks.
    sub: [u16; 8],
}

impl Sdm {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn write32(&mut self, off: u32, val: u32) {
        match off {
            CH_OFF..=0x1C => {
                // channel[N]: duty[7:0] | prescale[15:8]; reserved[31:16]=0.
                self.regs[(off / 4) as usize] = val & 0xFFFF;
            }
            CG_OFF | MISC_OFF | VERSION_OFF => {
                self.regs[(off / 4) as usize] = val;
            }
            _ => {}
        }
    }

    pub fn read32(&mut self, off: u32) -> u32 {
        match off {
            CH_OFF..=0x1C | CG_OFF | MISC_OFF | VERSION_OFF => self.regs[(off / 4) as usize],
            _ => 0,
        }
    }

    fn duty(&self, ch: usize) -> u8 {
        (self.regs[ch] & 0xFF) as u8
    }

    fn prescale(&self, ch: usize) -> u16 {
        ((self.regs[ch] >> 8) & 0xFF) as u16
    }

    /// High threshold (0..255) for channel `ch`. The 8-bit duty register holds
    /// a SIGNED value (esp-idf `sdm_channel_set_duty` writes the signed duty
    /// directly into the `uint8_t` field): output is high while the 8-bit
    /// phase counter is below `signed_duty + 128`, i.e. high for
    /// `(signed_duty + 128)/256` of the time — duty=0 (reg 0x00) -> 50%,
    /// duty=-128 (reg 0x80) -> 0%, duty=127 (reg 0x7F) -> ~100%.
    fn threshold(&self, ch: usize) -> u32 {
        let signed = (self.duty(ch) as i8) as i32;
        (signed + 128) as u32
    }

    /// Logical output level (0/1) of GPIO-matrix signal `sig`, if it is one of
    /// the 8 Sigma-Delta channels (93..100).
    pub fn signal_level(&self, sig: u32) -> u32 {
        if (SDM_SIGNAL_BASE..SDM_SIGNAL_BASE + 8).contains(&sig) {
            let ch = (sig - SDM_SIGNAL_BASE) as usize;
            ((self.phase[ch] as u32) < self.threshold(ch)) as u32
        } else {
            0
        }
    }

    /// Advance the modulators by one CPU tick (called from `tick_timers` per
    /// cycle).
    pub fn tick(&mut self) {
        for ch in 0..8 {
            let p = self.prescale(ch);
            if self.sub[ch] >= p {
                self.sub[ch] = 0;
                self.phase[ch] = self.phase[ch].wrapping_add(1);
            } else {
                self.sub[ch] += 1;
            }
        }
    }
}
