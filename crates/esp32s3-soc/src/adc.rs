//! ESP32-S3 SAR ADC: RTC oneshot (SENS) + digital (APB_SARADC) controllers.
//!
//! Two register blocks model the same ADC hardware:
//! - SENS (0x6000_8800) — the RTC oneshot controller driven by ESP-IDF's
//!   `adc_oneshot` (adc_oneshot_ll: measN_ctrl2 start/done/data, channel
//!   bitmap sarN_en_pad, sar_attenN, controller select sar1_dig_force, and
//!   the shared sar_slave_addr1.meas_status the driver polls before
//!   starting). Layout per sens_struct.h.
//! - APB_SARADC (0x6004_0000) — the digital controller (continuous/DMA
//!   path; adc_ll_digi_*): ctrl work_mode/sar_sel/sar_clk_div/pattern
//!   lengths, sarN_patt_tab items (one byte = atten[1:0] | channel[6:2]),
//!   ctrl2 timer trigger, results in apb_saradcN_data_status, done flags in
//!   int_raw (adc1_done = bit 31, adc2_done = bit 30). The DMA transfer
//!   itself is NOT modeled — continuous-mode firmware reads data_status
//!   directly.
//!
//! The host (frontend) injects an analog voltage per (unit, channel) in mV
//! via [`Adc::inject_voltage`]; conversion scales it by the attenuation
//! full-scale (0dB = 1.1 V, 2.5dB = 1.5 V, 6dB = 2.2 V, 11dB = 3.9 V — TRM
//! SAR ADC chapter) into the 12-bit raw value the firmware reads.
//!
//! Conversions take a fixed latency in APB cycles (the real SAR FSM runs on
//! RTC fast clocks and regi2c sample times that we don't model); firmware
//! busy-polls meas_status / done_sar, so only the ordering matters.

// SENS offsets (sens_struct.h member order at SENS = 0x6000_8800).
pub const SENS_SAR_READER1_CTRL: u32 = 0x00;
pub const SENS_SAR_READER1_STATUS: u32 = 0x04;
pub const SENS_SAR_MEAS1_CTRL1: u32 = 0x08;
pub const SENS_SAR_MEAS1_CTRL2: u32 = 0x0C;
pub const SENS_SAR_MEAS1_MUX: u32 = 0x10;
pub const SENS_SAR_ATTEN1: u32 = 0x14;
pub const SENS_SAR_READER2_CTRL: u32 = 0x24;
pub const SENS_SAR_READER2_STATUS: u32 = 0x28;
pub const SENS_SAR_MEAS2_CTRL1: u32 = 0x2C;
pub const SENS_SAR_MEAS2_CTRL2: u32 = 0x30;
pub const SENS_SAR_MEAS2_MUX: u32 = 0x34;
pub const SENS_SAR_ATTEN2: u32 = 0x38;
pub const SENS_SAR_POWER_XPD_SAR: u32 = 0x3C;
pub const SENS_SAR_SLAVE_ADDR1: u32 = 0x40;
// SENS block size (touch / tsens / cocpu after the SAR registers).
const SENS_REGS: usize = 0x200 / 4;

// APB_SARADC offsets (apb_saradc_struct.h at 0x6004_0000).
pub const APB_CTRL: u32 = 0x00;
pub const APB_CTRL2: u32 = 0x04;
pub const APB_FSM_WAIT: u32 = 0x0C;
pub const APB_SAR1_PATT_TAB: u32 = 0x18;
pub const APB_SAR2_PATT_TAB: u32 = 0x28;
pub const APB_FILTER_CTRL0: u32 = 0x3C;
pub const APB_SARADC1_DATA_STATUS: u32 = 0x40;
pub const APB_THRES0_CTRL: u32 = 0x44;
pub const APB_THRES1_CTRL: u32 = 0x48;
pub const APB_INT_ENA: u32 = 0x5C;
pub const APB_INT_RAW: u32 = 0x60;
pub const APB_INT_ST: u32 = 0x64;
pub const APB_INT_CLR: u32 = 0x68;
/// APB_SARADC done-interrupt matrix source (`interrupts.h` recount: the
/// ADC done flags live in the APB block's own INT_ST, gated by INT_ENA).
pub const APB_ADC_INTR_SOURCE: u32 = 65;
pub const APB_DMA_CONF: u32 = 0x6C;
pub const APB_CLKM_CONF: u32 = 0x70;
pub const APB_SARADC2_DATA_STATUS: u32 = 0x78;
pub const APB_CTRL_DATE: u32 = 0x3FC;
const APB_REGS: usize = 0x400 / 4;

// SENS sar_readerN_ctrl bits (sens_struct.h).
const SAR1_DATA_INV: u32 = 1 << 28;
const SAR2_DATA_INV: u32 = 1 << 29;
// SENS sar_measN_ctrl2 fields (sens_struct.h) — meas1 and meas2 share the
// same bit positions.
const MEAS_DONE: u32 = 1 << 16;
const MEAS_START: u32 = 1 << 17;
const MEAS_START_FORCE: u32 = 1 << 18;
const EN_PAD_SHIFT: u32 = 19;
const EN_PAD_MASK: u32 = 0xFFF;
// SENS sar_meas1_mux (sens_struct.h): 1 = SAR ADC1 controlled by DIG.
const SAR1_DIG_FORCE: u32 = 1 << 31;
// SENS sar_slave_addr1 (sens_struct.h): meas_status [29:22].
const MEAS_STATUS_SHIFT: u32 = 22;
const MEAS_STATUS_MASK: u32 = 0xFF << 22;

// APB_SARADC ctrl bits (apb_saradc_struct.h).
const APB_START_FORCE: u32 = 1 << 0;
const APB_START: u32 = 1 << 1;
const APB_SAR_CLK_GATED: u32 = 1 << 6;
const APB_WORK_MODE_SHIFT: u32 = 3;
const APB_WORK_MODE_MASK: u32 = 0x3;
const APB_PATT_LEN_SHIFT: [u32; 2] = [15, 19];
// ctrl2 bits: sar1_inv [9], sar2_inv [10], timer_sel [11], timer_target
// [23:12], timer_en [24].
const APB_TIMER_EN: u32 = 1 << 24;
const APB_TIMER_SEL: u32 = 1 << 11;
const APB_TIMER_TARGET_SHIFT: u32 = 12;
const APB_SAR1_INV: u32 = 1 << 9;
const APB_SAR2_INV: u32 = 1 << 10;
// int_* done bits (apb_saradc_struct.h): adc1_done = bit 31, adc2_done =
// bit 30 (the ESP32-S3 packs the interrupt flags at the TOP of the word).
pub const APB_ADC1_DONE: u32 = 1 << 31;
pub const APB_ADC2_DONE: u32 = 1 << 30;

// Full-scale input per attenuation code (mV), TRM SAR ADC chapter (the
// same table is used by the IDF calibration helpers).
const ATTN_FULL_SCALE_MV: [u32; 4] = [1100, 1500, 2200, 3900];

// Fixed conversion latency in APB cycles (see module doc).
const ONESHOT_CYCLES: u64 = 8;

const NUM_UNITS: usize = 2;
const NUM_CHANNELS: usize = 12;

// Per-unit register offsets (meas1 = ADC1, meas2 = ADC2).
const SENS_MEAS_CTRL2: [u32; NUM_UNITS] = [SENS_SAR_MEAS1_CTRL2, SENS_SAR_MEAS2_CTRL2];
const SENS_ATTEN: [u32; NUM_UNITS] = [SENS_SAR_ATTEN1, SENS_SAR_ATTEN2];
const SENS_READER_CTRL: [u32; NUM_UNITS] = [SENS_SAR_READER1_CTRL, SENS_SAR_READER2_CTRL];
const DATA_STATUS: [u32; NUM_UNITS] = [APB_SARADC1_DATA_STATUS, APB_SARADC2_DATA_STATUS];

/// The SAR ADC model: one SENS (RTC oneshot) + one APB_SARADC (digital)
/// register file sharing the injected channel voltages.
pub struct Adc {
    /// SENS (RTC oneshot) register file.
    sens: [u32; SENS_REGS],
    /// APB_SARADC (digital) register file.
    apb: [u32; APB_REGS],
    /// Injected analog voltage per (unit, channel) in mV (host frontend).
    voltages: [[u32; NUM_CHANNELS]; NUM_UNITS],
    /// APB cycles remaining for each unit's RTC oneshot conversion.
    oneshot_pending: [u64; NUM_UNITS],
    /// SENS sar_slave_addr1.meas_status busy flag (shared SAR FSM).
    meas_busy: bool,
    /// APB timer-trigger accumulator (APB cycles).
    timer_cycle: u64,
    /// APB alternate-mode phase (work_mode = 2).
    alt_phase: bool,
    /// Digital-conversion results queued for the GDMA `in` walk
    /// (`peri_sel` ADC): each timer pass pushes its data_status word here
    /// (cap 256, oldest dropped). The IN walk drains them to DRAM.
    dma_queue: alloc::collections::VecDeque<u32>,
    /// TSENS 8-bit DAC output code (host-injected; SENS_TSENS_OUT).
    tsens_raw: u8,
}

impl Adc {
    pub fn new() -> Self {
        let mut s = Self {
            sens: [0; SENS_REGS],
            apb: [0; APB_REGS],
            voltages: [[0; NUM_CHANNELS]; NUM_UNITS],
            oneshot_pending: [0; NUM_UNITS],
            meas_busy: false,
            timer_cycle: 0,
            alt_phase: false,
            dma_queue: alloc::collections::VecDeque::new(),
            tsens_raw: 128,
        };
        // Reset values from sens_reg.h / apb_saradc_reg.h (idle FSM fields,
        // so firmware that reads config back sees sensible defaults).
        s.sens[(SENS_SAR_READER1_CTRL / 4) as usize] = (1 << 29) | (1 << 18) | 2;
        s.sens[(SENS_SAR_READER2_CTRL / 4) as usize] = (1 << 30) | (1 << 18) | 2;
        s.sens[(SENS_SAR_ATTEN1 / 4) as usize] = 0xFFFF_FFFF;
        s.sens[(SENS_SAR_ATTEN2 / 4) as usize] = 0xFFFF_FFFF;
        s.sens[(SENS_SAR_MEAS2_CTRL1 / 4) as usize] = 0x0702_0200;
        s.apb[(APB_CTRL / 4) as usize] = 0x43C7_8240;
        s.apb[(APB_CTRL2 / 4) as usize] = 0x0000_A1FE;
        s.apb[(APB_FSM_WAIT / 4) as usize] = 0x00FF_0808;
        s.apb[(APB_FILTER_CTRL0 / 4) as usize] = 0x006B_4000;
        s.apb[(APB_THRES0_CTRL / 4) as usize] = 0x7FFF_FFFD;
        s.apb[(APB_THRES1_CTRL / 4) as usize] = 0x7FFF_FFFD;
        s.apb[(APB_DMA_CONF / 4) as usize] = 0x0000_00FF;
        s.apb[(APB_CLKM_CONF / 4) as usize] = 0x0000_0004;
        s.apb[(APB_CTRL_DATE / 4) as usize] = 0x0210_1180;
        s
    }

    /// Inject an analog voltage (mV) on `unit` (0 = ADC1, 1 = ADC2)
    /// `channel` (0..=9 on real silicon).  Host frontend API.
    /// Pop one staged digital-conversion result for the GDMA `in` walk
    /// (`None` when the queue is empty — reads back zeros on the bus).
    pub fn dma_pop(&mut self) -> Option<u32> {
        self.dma_queue.pop_front()
    }

    pub fn inject_voltage(&mut self, unit: usize, channel: usize, milli_volts: u32) {
        if unit < NUM_UNITS && channel < NUM_CHANNELS {
            self.voltages[unit][channel] = milli_volts;
        }
    }

    /// Inject the TSENS 8-bit DAC output code (host frontend — drives
    /// what the firmware reads from SENS_TSENS_OUT).
    pub fn tsens_inject(&mut self, raw: u8) {
        self.tsens_raw = raw;
    }

    /// 12-bit raw for (unit, channel) scaled by its SENS attenuation, with
    /// the reader's data-invert applied, as the RTC oneshot controller
    /// reports it.
    fn oneshot_raw(&self, unit: usize, channel: usize) -> u32 {
        let atten = (self.sens[(SENS_ATTEN[unit] / 4) as usize] >> (channel * 2)) & 3;
        let raw = Self::raw_scaled(self.voltages[unit][channel], atten);
        let inv = if unit == 0 {
            SAR1_DATA_INV
        } else {
            SAR2_DATA_INV
        };
        if self.sens[(SENS_READER_CTRL[unit] / 4) as usize] & inv != 0 {
            raw ^ 0xFFF
        } else {
            raw
        }
    }

    /// Scale a mV voltage to the 12-bit raw for an attenuation code.
    fn raw_scaled(milli_volts: u32, atten: u32) -> u32 {
        let fs = ATTN_FULL_SCALE_MV[(atten & 3) as usize] as u64;
        let mv = milli_volts as u64;
        if mv >= fs {
            4095
        } else {
            (mv * 4095 / fs) as u32
        }
    }

    /// Oneshot conversion for `unit` finished: latch the selected channel's
    /// raw into measN_data_sar and raise measN_done_sar.
    fn complete_oneshot(&mut self, unit: usize) {
        let idx = (SENS_MEAS_CTRL2[unit] / 4) as usize;
        let pad = (self.sens[idx] >> EN_PAD_SHIFT) & EN_PAD_MASK;
        let channel = if pad == 0 { 0 } else { pad.trailing_zeros() } as usize;
        let raw = self.oneshot_raw(unit, channel);
        self.sens[idx] = (self.sens[idx] & !0xFFFF) | raw;
        self.sens[idx] |= MEAS_DONE;
    }

    /// Advance `cycles` APB cycles: finish pending oneshot conversions and
    /// service the digital timer trigger.
    pub fn tick(&mut self, cycles: u64) {
        // Fast path: no pending oneshot and no digital timer → nothing to do.
        let ctrl2 = self.apb[(APB_CTRL2 / 4) as usize];
        let digital_active = ctrl2 & APB_TIMER_EN != 0 && ctrl2 & APB_TIMER_SEL != 0;
        if self.oneshot_pending[0] == 0 && self.oneshot_pending[1] == 0 && !digital_active {
            return;
        }
        for _ in 0..cycles {
            for unit in 0..NUM_UNITS {
                if self.oneshot_pending[unit] > 0 {
                    self.oneshot_pending[unit] -= 1;
                    if self.oneshot_pending[unit] == 0 {
                        self.complete_oneshot(unit);
                    }
                }
            }
            self.meas_busy = self.oneshot_pending[0] > 0 || self.oneshot_pending[1] > 0;
            self.tick_digital();
        }
    }

    /// APB timer-triggered digital conversion (ctrl2.timer_en +
    /// timer_sel): one pattern-table pass every (timer_target + 1) cycles.
    fn tick_digital(&mut self) {
        let ctrl2 = self.apb[(APB_CTRL2 / 4) as usize];
        if ctrl2 & APB_TIMER_EN == 0 || ctrl2 & APB_TIMER_SEL == 0 {
            return;
        }
        let target = (ctrl2 >> APB_TIMER_TARGET_SHIFT) & 0xFFF;
        self.timer_cycle += 1;
        if self.timer_cycle > target as u64 {
            self.timer_cycle = 0;
            self.run_digital();
        }
    }

    /// One APB_SARADC conversion pass: run the selected unit(s)' pattern
    /// tables, write the last result to data_status and raise the done raw
    /// flags (adc_ll adc1_done/adc2_done).
    fn run_digital(&mut self) {
        let ctrl = self.apb[(APB_CTRL / 4) as usize];
        if ctrl & APB_SAR_CLK_GATED == 0 {
            return; // SAR clock gated off: no conversion (like SPI clk_en).
        }
        let mode = (ctrl >> APB_WORK_MODE_SHIFT) & APB_WORK_MODE_MASK;
        let sar_sel = (ctrl >> 5) & 1;
        let mut units = [false; NUM_UNITS];
        match mode {
            0 => units[sar_sel as usize] = true, // single
            1 => {
                units[0] = true;
                units[1] = true; // double
            }
            _ => {
                units[if self.alt_phase { 1 } else { 0 }] = true; // alternate
                self.alt_phase = !self.alt_phase;
            }
        }
        for unit in 0..NUM_UNITS {
            if !units[unit] {
                continue;
            }
            let len = ((ctrl >> APB_PATT_LEN_SHIFT[unit]) & 0xF) as usize;
            let tab_base = if unit == 0 {
                APB_SAR1_PATT_TAB
            } else {
                APB_SAR2_PATT_TAB
            };
            let mut last = 0;
            for i in 0..=len {
                // Four 1-byte items per 32-bit word.
                let word = self.apb[((tab_base + (i as u32 / 4) * 4) / 4) as usize];
                let byte = (word >> (8 * (i % 4))) & 0xFF;
                let atten = byte & 3;
                let channel = (((byte >> 2) & 0x3F) as usize).min(NUM_CHANNELS - 1);
                last = Self::raw_scaled(self.voltages[unit][channel], atten);
            }
            let inv = if unit == 0 {
                APB_SAR1_INV
            } else {
                APB_SAR2_INV
            };
            if self.apb[(APB_CTRL2 / 4) as usize] & inv != 0 {
                last ^= 0xFFF;
            }
            self.apb[(DATA_STATUS[unit] / 4) as usize] = last & 0x1FFFF;
            // Stage for GDMA: the DMA engine moves each conversion result
            // to DRAM (the IN walk drains this queue).
            if self.dma_queue.len() >= 256 {
                self.dma_queue.pop_front();
            }
            self.dma_queue.push_back(last & 0x1FFFF);
            self.apb[(APB_INT_RAW / 4) as usize] |= if unit == 0 {
                APB_ADC1_DONE
            } else {
                APB_ADC2_DONE
            };
        }
    }

    /// Read a SENS register (`offset` relative to SENS_BASE).
    pub fn sens_read32(&mut self, offset: u32) -> u32 {
        match offset {
            SENS_SAR_SLAVE_ADDR1 => {
                // Live meas_status [29:22]: the shared SAR FSM busy flag
                // (adc_oneshot_ll_start polls this before starting).
                let mut v = self.sens[(SENS_SAR_SLAVE_ADDR1 / 4) as usize];
                v &= !MEAS_STATUS_MASK;
                if self.meas_busy {
                    v |= 1 << MEAS_STATUS_SHIFT;
                }
                v
            }
            // TSENS_CTRL @ 0x50 (sens_reg.h): READY (bit 8) latches once
            // powered; OUT[7:0] is the host-injected DAC code. The tsens
            // driver polls READY then reads OUT (no ROM involved).
            0x50 => {
                let mut v = self.sens[0x50 / 4] & !0x1FF;
                v |= 1 << 8;
                v |= self.tsens_raw as u32;
                v
            }
            // TEMP WIFI BRING-UP (phy RF-cal): the closed PHY ROM polls
            // SENS2 0x6000E04C (NOT SENS 0x6000884C — objdump says
            // `SENS+0x584C`, i.e. the 0x6000E000 page; the disassembler's
            // `(SENS+0x584C)` annotation is relative to SENS_BASE because
            // the literal is 0x6000E04C = SENS_BASE + 0x584C) for bit 24
            // (`txdc_cal_v70`: l32i.n a12,[a10=0x6000E04C];
            // bnone a12,a11(=0x1000000),spin). See the SENS2 arm in
            // soc.rs, which reports the bit set. This SENS-page 0x4C
            // (SAR_SLAVE_ADDR4) keeps plain store semantics.
            0x4C => self.sens[0x4C / 4],
            _ if offset.is_multiple_of(4) && offset < (SENS_REGS * 4) as u32 => {
                self.sens[(offset / 4) as usize]
            }
            _ => 0,
        }
    }

    /// Write a SENS register (`offset` relative to SENS_BASE).
    pub fn sens_write32(&mut self, offset: u32, value: u32) {
        if offset == SENS_SAR_MEAS1_CTRL2 {
            self.meas_ctrl2_write(0, value);
        } else if offset == SENS_SAR_MEAS2_CTRL2 {
            self.meas_ctrl2_write(1, value);
        } else if offset.is_multiple_of(4) && offset < (SENS_REGS * 4) as u32 {
            self.sens[(offset / 4) as usize] = value;
        }
    }

    /// measN_ctrl2 write: any start write clears done_sar (latched until
    /// the next start); a SW start (start_force) with the RTC controller
    /// owning the unit begins a conversion (adc_oneshot_ll_start writes
    /// start 0 then 1; ADC1 requires sar1_dig_force = 0).
    fn meas_ctrl2_write(&mut self, unit: usize, value: u32) {
        let idx = (SENS_MEAS_CTRL2[unit] / 4) as usize;
        self.sens[idx] = value & !MEAS_DONE;
        let rtc_owned =
            unit == 1 || self.sens[(SENS_SAR_MEAS1_MUX / 4) as usize] & SAR1_DIG_FORCE == 0;
        if value & (MEAS_START | MEAS_START_FORCE) == (MEAS_START | MEAS_START_FORCE)
            && self.oneshot_pending[unit] == 0
            && rtc_owned
        {
            self.oneshot_pending[unit] = ONESHOT_CYCLES;
            self.meas_busy = true;
        }
    }

    /// ADC done-interrupt status (`INT_RAW & INT_ENA`) for matrix source 65.
    pub fn int_st(&self) -> u32 {
        self.apb[(APB_INT_RAW / 4) as usize] & self.apb[(APB_INT_ENA / 4) as usize]
    }

    /// Read an APB_SARADC register (`offset` relative to APB_SARADC_BASE).
    pub fn apb_read32(&mut self, offset: u32) -> u32 {
        match offset {
            APB_INT_ST => {
                self.apb[(APB_INT_RAW / 4) as usize] & self.apb[(APB_INT_ENA / 4) as usize]
            }
            _ if offset.is_multiple_of(4) && offset < (APB_REGS * 4) as u32 => {
                self.apb[(offset / 4) as usize]
            }
            _ => 0,
        }
    }

    /// Write an APB_SARADC register (`offset` relative to APB_SARADC_BASE).
    pub fn apb_write32(&mut self, offset: u32, value: u32) {
        match offset {
            APB_CTRL => {
                // start is a self-clearing pulse bit (like SPI CMD.usr).
                self.apb[(APB_CTRL / 4) as usize] = value & !APB_START;
                if value & (APB_START | APB_START_FORCE) == (APB_START | APB_START_FORCE) {
                    self.run_digital();
                }
            }
            APB_INT_CLR => {
                self.apb[(APB_INT_RAW / 4) as usize] &= !value;
            }
            // Read-only status / raw registers ignore writes.
            APB_INT_RAW | APB_SARADC1_DATA_STATUS | APB_SARADC2_DATA_STATUS => {}
            _ if offset.is_multiple_of(4) && offset < (APB_REGS * 4) as u32 => {
                self.apb[(offset / 4) as usize] = value;
            }
            _ => {}
        }
    }
}

impl Default for Adc {
    fn default() -> Self {
        Self::new()
    }
}
