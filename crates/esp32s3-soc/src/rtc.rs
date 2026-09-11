//! ESP32-S3 RTC_CNTL slow-clock timer + deep-sleep control.
//!
//! Register layout per the TRM (RTC_CNTL chapter): TIME_UPDATE_REG at
//! 0x6000800C (TIME_UPDATE bit 31 — write 1 to latch the running count into
//! the TIME_VALUE registers), TIME0_REG at 0x60008010 and TIME1_REG at
//! 0x60008014 (the latched low/high 32 bits; IDF `rtc_time_get()` reads
//! them, `rtc_time_us_to_slowclk()` converts, and `rtc_clk_cal_internal`
//! spin on the count).
//!
//! Time model: the counter advances at the RC_SLOW oscillator rate
//! (nominally 32.5 kHz, TRM §27); the frontend drives CPU time via
//! `tick(cycles)` (240 MHz / 32.5 kHz = 7385 CPU cycles per slow tick).
//! Without this model `rtc_time_get()` returns 0 forever and every
//! slow-clock timeout loop spins indefinitely (observed: rtc_init stuck at
//! 0x42020a74 in the Arduino core boot).
//!
//! Deep-sleep: the legacy (S3, esp-idf ≤ v5.3) power-down path sets
//! `RTC_CNTL_SLEEP_EN` (bit 31 of `RTC_CNTL_STATE0_REG` @ +0x18); the sleep
//! period is programmed in `RTC_CNTL_SLP_TIMER0_REG` (@ +0x4, low 32) and
//! `RTC_CNTL_SLP_TIMER1_REG` (@ +0x8, high 16 + MAIN_TIMER_ALARM_EN bit 16).
//! On wake the ROM/firmware reads `RTC_CNTL_SLP_WAKEUP_CAUSE_REG` (@ +0x130,
//! field `RTC_CNTL_WAKEUP_CAUSE` [16:0]); `RTC_TIMER_TRIG_EN` (bit 3) flags a
//! timer wakeup. The emulator fast-forwards the sleep as a fixed step budget
//! and reboots with the timer wakeup-cause bit set.

/// RTC_CNTL register block base (TRM RTC_CNTL memory map).
pub const RTC_CNTL_BASE: u32 = 0x6000_8000;

/// The ULP-RISC-V control/status registers live inside the RTC_CNTL page at
/// offset `0x100..0x200` (TRM RTC_CNTL memory map).  `RTC_CNTL_SLP_WAKEUP_CAUSE`
/// (0x130) and a few other RTC_CNTL registers also fall in this window, so
/// `Rtc` special-cases those before delegating the ULP sub-range.
const ULP_OFF_START: u32 = 0x100;
const ULP_OFF_END: u32 = 0x200;

// TIME_UPDATE_REG bit 31 = TIME_UPDATE: write 1 to latch the count.
pub const TIME_UPDATE_OFF: u32 = 0x0C;
pub const TIME_UPDATE_BIT: u32 = 1 << 31;
pub const TIME_VALUE_LO_OFF: u32 = 0x10;
pub const TIME_VALUE_HI_OFF: u32 = 0x14;

// Deep-sleep control registers (TRM RTC_CNTL + esp-idf rtc_cntl_reg.h).
pub const SLP_TIMER0_OFF: u32 = 0x04;
pub const SLP_TIMER1_OFF: u32 = 0x08;
pub const STATE0_OFF: u32 = 0x18;
pub const SLEEP_EN_BIT: u32 = 1 << 31;
pub const SLP_WAKEUP_CAUSE_OFF: u32 = 0x130;
// RTC_CNTL interrupt block (rtc_cntl_reg.h): ENA @ +0x40, RAW @ +0x44,
// ST @ +0x48, CLR @ +0x4C. Touch DONE = bit 6, SCAN_DONE = bit 4
// (matches touch_sensor_ll.h TOUCH_LL_INTR_MASK_*).
const INT_ENA_OFF: u32 = 0x40;
const INT_RAW_OFF: u32 = 0x44;
const INT_ST_OFF: u32 = 0x48;
const INT_CLR_OFF: u32 = 0x4C;
const TOUCH_DONE_BIT: u32 = 1 << 6;
const TOUCH_SCAN_DONE_BIT: u32 = 1 << 4;
// Sleep-event interrupt bits (rtc_cntl_reg.h INT_RAW): SLP_REJECT = bit 0,
// SLP_WAKEUP = bit 1 (`rtc_sleep_start` spins on bits [1:0]).
const SLP_WAKEUP_BIT: u32 = 1 << 1;
// Touch FSM trigger registers (rtc_cntl_reg.h): CTRL2 @ +0x10C
// (touch_start_force / timer_force_done), SCAN_CTRL @ +0x110 (pad map).
const TOUCH_CTRL2_OFF: u32 = 0x10C;
const TOUCH_SCAN_CTRL_OFF: u32 = 0x110;
// RTC-core interrupt source (interrupts.h: WIFI_MAC 0..UHCI1 15, GPIO
// 16..19, SPI1 20, SPI2 21, SPI3 22, (23), LCD_CAM 24, I2S0/1 25/26,
// UART0/1/2 27/28/29, SDIO 30, PWM0/1 31/32, (33, 34), LEDC 35, EFUSE 36,
// TWAI 37, USB 38, RTC_CORE 39). Carries the touch DONE/SCAN_DONE (and
// WDT) interrupts; TWAI=37 cross-checks against twai.rs (driver-validated).
pub const RTC_CORE_INTR_SOURCE: u32 = 39;
// Reset-cause register (rtc_cntl_reg.h RTC_CNTL_RESET_STATE_REG @ +0x38):
// PROCPU cause [5:0], APPCPU cause [11:6]. The live ROM's
// `esp_rom_get_reset_reason` (0x4000057C) returns these fields directly
// (extui 0,6 / extui 6,6), and `esp_sleep_get_wakeup_cause` only reads the
// wakeup-cause register when the PRO reason is DEEPSLEEP (5).
pub const RESET_STATE_OFF: u32 = 0x38;
/// Power-on reset-cause code (what silicon reports after power-up).
pub const RESET_CAUSE_POWERON: u32 = 1;
/// Deep-sleep-wake reset-cause code (`esp_sleep_get_wakeup_cause` gate).
pub const RESET_CAUSE_DEEPSLEEP: u32 = 5;

// Wakeup-source registers (rtc_cntl_reg.h; trigger enable bits from
// soc/rtc.h: TRIG_EN bit N == wakeup-cause bit N).
pub const WAKEUP_STATE_OFF: u32 = 0x3C; // WAKEUP_ENA bitmap [31:15]
pub const EXT_CONF_OFF: u32 = 0x64; // EXT_WAKEUP_CONF: EXT1_LV[31], EXT0_LV[30]
pub const EXT1_SEL_OFF: u32 = 0xE0; // EXT_WAKEUP1: STATUS_CLR[22], SEL[21:0]
pub const EXT1_STATUS_OFF: u32 = 0xE4; // EXT_WAKEUP1_STATUS (triggered pads)
/// Wakeup-enable bit for cause bit N (WAKEUP_STATE field starts at bit 15).
pub const fn wakeup_ena(n: u32) -> u32 {
    1 << (15 + n)
}
// Wakeup-cause bits (match TRIG_EN + `esp_sleep_get_wakeup_cause` decode,
// verified against the linked driver's disassembly: bit 9 (FSM ULP) and
// bit 11 (COCPU/RISCV ULP) both return ULP).
pub const CAUSE_EXT0: u32 = 1 << 0;
pub const CAUSE_EXT1: u32 = 1 << 1;
pub const CAUSE_TIMER: u32 = 1 << 3;
// Touch-pad wakeup (`TOUCH_TRIG_EN = BIT8`, esp_rom rtc.h): the touch
// controller runs during sleep and wakes on a threshold crossing.
pub const CAUSE_TOUCH: u32 = 1 << 8;
pub const CAUSE_ULP: u32 = 1 << 9;
pub const CAUSE_COCPU: u32 = 1 << 11;

// RC_SLOW nominal frequency (TRM §27): 32.5 kHz; CPU clock 240 MHz.
const SLOW_CLK_DIV: u64 = 240_000_000 / 32_500;

pub struct Rtc {
    /// Free-running slow-clock counter (RTC time, TRM RTC_CNTL_TIME0/1).
    count: u64,
    /// Count latched by the last TIME_UPDATE write.
    latched: u64,
    /// CPU-cycle accumulator for the slow-clock divider.
    acc: u64,
    /// Deep-sleep period (SLP_TIMER0 | SLP_TIMER1[15:0] << 32).
    slp_timer0: u32,
    slp_timer1: u32,
    /// Set when firmware writes RTC_CNTL_SLEEP_EN (power-down request).
    sleep_req: bool,
    /// Captured sleep duration (slow ticks) for the pending request.
    sleep_target: u64,
    /// Deep (reboot on wake) vs light (resume) sleep, captured at SLEEP_EN
    /// from DIG_PWC DG_WRAP_PD_EN (bit 31): the deep-sleep driver powers
    /// down the digital core, light sleep keeps it (rtc_cntl_reg.h
    /// RTC_CNTL_DIG_PWC_REG @ +0x90; verified: deep driver programs
    /// 0xC0020010, light sleep 0x00020010, direct pokes 0x00020000).
    sleep_deep: bool,
    /// RTC_CNTL_SLP_WAKEUP_CAUSE_REG mirror (set by the emulator on wake).
    wakeup_cause: u32,
    /// EXT_WAKEUP1_STATUS mirror (triggering RTC pads, set on EXT1 wake).
    ext1_status: u32,
    /// RTC_CNTL_RESET_STATE_REG mirror (reset causes for PRO/APPCPU).
    /// Seeded POWERON/POWERON like silicon; the machine sets DEEPSLEEP on
    /// wake. Real hardware is read-only; firmware writes fall into the
    /// generic `regs` store and do not disturb this mirror.
    reset_state: u32,
    /// SLP_TIMER0/1 have been programmed since reset (timer-armed
    /// heuristic: the direct-poke sleep flow programs them without touching
    /// WAKEUP_ENA, while no-timer flows never write them).
    slp_timer_written: bool,
    /// Latched touch FSM completion (DONE + SCAN_DONE, rtc_cntl_reg.h
    /// INT_RAW bits 6/4): raised synchronously when firmware triggers a
    /// scan via TOUCH_CTRL2/SCAN_CTRL (the event the driver's oneshot wait
    /// blocks on), cleared by INT_CLR. Other INT_RAW bits live in `regs`.
    touch_raw: u32,
    /// Latched sleep-event bits (SLP_REJECT bit 0 / SLP_WAKEUP bit 1,
    /// rtc_cntl_reg.h INT_RAW): `rtc_sleep_start` spins on bits [1:0]
    /// after triggering sleep, so a light-sleep wake latches SLP_WAKEUP
    /// (cleared by INT_CLR like the touch latch). SLP_REJECT is never
    /// raised (no rejected-sleep flow is modeled).
    sleep_raw: u32,
    /// Generic backing store for the full RTC_CNTL page (0x000..0x400).  Most
    /// registers are simple stores; the special-cased ones below override this.
    regs: [u32; 0x400 / 4],
    /// ULP-RISC-V control/status block (offset 0x100..0x200 of this page).
    ulp: crate::ulp::Ulp,
    /// RTC watchdog (RWDT, WDTCONFIG0..4 @ 0x98..0xA8, FEED @ 0xAC,
    /// WPROTECT @ 0xB0): counter, per-stage cumulative holds, fired latches,
    /// reset request, and write-protect key latch (mirrors the MWDT model).
    wdt_count: u64,
    wdt_fired: [bool; 4],
    wdt_reset: bool,
}

impl Default for Rtc {
    fn default() -> Self {
        Self {
            count: 0,
            latched: 0,
            acc: 0,
            slp_timer0: 0,
            slp_timer1: 0,
            sleep_req: false,
            sleep_target: 0,
            sleep_deep: true,
            wakeup_cause: 0,
            ext1_status: 0,
            // Silicon-accurate POWERON causes (verified live-ROM decode:
            // `esp_rom_get_reset_reason` returns RESET_STATE PROCPU/APPCPU).
            // POWERON boots take ~3x the instructions (~34M vs ~13M for
            // hello): core 0 waits in the flash-stall handshake while core 1
            // finishes its own POWERON init (RF-cal/BBPLL), then boot
            // completes normally — an early 20-30M STEP budget misread this
            // wait as a hang. Size validation budgets accordingly.
            reset_state: RESET_CAUSE_POWERON | (RESET_CAUSE_POWERON << 6),
            slp_timer_written: false,
            touch_raw: 0,
            sleep_raw: 0,
            regs: [0u32; 0x400 / 4],
            ulp: crate::ulp::Ulp::default(),
            wdt_count: 0,
            wdt_fired: [false; 4],
            wdt_reset: false,
        }
    }
}

impl Rtc {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn tick(&mut self, cycles: u64) {
        self.acc += cycles;
        while self.acc >= SLOW_CLK_DIV {
            self.acc -= SLOW_CLK_DIV;
            self.count += 1;
        }
        self.tick_rwdt(cycles);
    }

    /// RTC watchdog: counts up while WDT_EN (CONFIG0[31]); each stage fires
    /// once at its cumulative hold (CONFIG1..4, full 32-bit each); stage
    /// actions mirror MWDT (0 = none, 1 = interrupt (not wired; RTC_CORE is
    /// shared with touch), 2/3 = system reset via `wdt_reset`). FEED (any
    /// write) restarts. CONFIG writes require the WPROTECT key latched.
    fn tick_rwdt(&mut self, cycles: u64) {
        let cfg0 = self.regs[0x98 / 4];
        if cfg0 & (1 << 31) == 0 {
            return;
        }
        self.wdt_count += cycles;
        let stg = [
            (cfg0 >> 28) & 7,
            (cfg0 >> 25) & 7,
            (cfg0 >> 22) & 7,
            (cfg0 >> 19) & 7,
        ];
        let mut cum: u64 = 0;
        for i in 0..4 {
            cum += self.regs[(0x9C + 4 * i as u32) as usize / 4] as u64;
            if !self.wdt_fired[i] && self.wdt_count >= cum {
                self.wdt_fired[i] = true;
                let a = stg[i];
                if a == 2 || a == 3 {
                    self.wdt_reset = true;
                }
            }
        }
    }

    /// True when an RWDT reset stage elapsed (machine reboots); cleared.
    pub fn consume_reset(&mut self) -> bool {
        core::mem::replace(&mut self.wdt_reset, false)
    }

    /// True if firmware requested a sleep since the last call; returns the
    /// captured sleep duration (slow-clock ticks) plus whether it is a
    /// deep sleep (reboot on wake) or light sleep (resume), and clears
    /// the flag.
    pub fn sleep_req(&self) -> bool {
        self.sleep_req
    }

    pub fn consume_sleep_request(&mut self) -> Option<(u64, bool)> {
        if self.sleep_req {
            self.sleep_req = false;
            Some((self.sleep_target, self.sleep_deep))
        } else {
            None
        }
    }

    /// Record the wakeup-cause bits read back by `esp_sleep_get_wakeup_cause`
    /// after a deep-sleep reboot (called by the machine on wake).
    pub fn set_wakeup_cause(&mut self, bits: u32) {
        self.wakeup_cause = bits;
    }

    /// Latch the SLP_WAKEUP interrupt (called by the machine on a
    /// light-sleep wake so `rtc_sleep_start`'s INT_RAW spin exits).
    pub fn set_sleep_wakeup(&mut self) {
        self.sleep_raw |= SLP_WAKEUP_BIT;
    }

    /// Record the EXT1 triggering pads for `esp_sleep_get_ext1_wakeup_status`.
    pub fn set_ext1_status(&mut self, pads: u32) {
        self.ext1_status = pads & 0x3FFFFF;
    }

    /// Whether the sleep timer was programmed since reset (timer-armed
    /// heuristic for flows that never touch WAKEUP_ENA).
    pub fn slp_timer_written(&self) -> bool {
        self.slp_timer_written
    }

    /// Raw WAKEUP_STATE (enable bitmap) register value.
    pub fn wakeup_state(&self) -> u32 {
        self.regs[(WAKEUP_STATE_OFF / 4) as usize]
    }

    /// Raw EXT_WAKEUP_CONF register value (EXT1_LV[31], EXT0_LV[30]).
    pub fn ext_conf(&self) -> u32 {
        self.regs[(EXT_CONF_OFF / 4) as usize]
    }

    /// Raw EXT_WAKEUP1 SEL mask (RTC pads [21:0]).
    pub fn ext1_sel(&self) -> u32 {
        self.regs[(EXT1_SEL_OFF / 4) as usize] & 0x3FFFFF
    }

    /// Record the reset causes read back by the live ROM's
    /// `esp_rom_get_reset_reason` (called by the machine on wake).
    pub fn set_reset_cause(&mut self, pro: u32, app: u32) {
        self.reset_state = (pro & 0x3F) | ((app & 0x3F) << 6);
    }

    /// Masked RTC-core interrupt status (INT_ST = RAW & ENA), covering
    /// the latched touch DONE/SCAN_DONE bits.
    pub fn int_st(&self) -> u32 {
        (self.regs[INT_RAW_OFF as usize / 4] | self.touch_raw) & self.regs[INT_ENA_OFF as usize / 4]
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            TIME_VALUE_LO_OFF => self.latched as u32,
            TIME_VALUE_HI_OFF => (self.latched >> 32) as u32,
            SLP_TIMER0_OFF => self.slp_timer0,
            SLP_TIMER1_OFF => self.slp_timer1,
            SLP_WAKEUP_CAUSE_OFF => self.wakeup_cause,
            RESET_STATE_OFF => self.reset_state,
            EXT1_STATUS_OFF => self.ext1_status,
            // Interrupt status: live touch completion latched in
            // `touch_raw` ORed over the stored RAW; ST masks by ENA.
            INT_RAW_OFF => self.regs[INT_RAW_OFF as usize / 4] | self.touch_raw | self.sleep_raw,
            INT_ST_OFF => {
                (self.regs[INT_RAW_OFF as usize / 4] | self.touch_raw | self.sleep_raw)
                    & self.regs[INT_ENA_OFF as usize / 4]
            }
            // ULP-RISC-V block lives at offset 0x100..0x200 of this page.
            o if (ULP_OFF_START..ULP_OFF_END).contains(&o) => self.ulp.read32(RTC_CNTL_BASE + o),
            o => self.regs[o as usize / 4],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            TIME_UPDATE_OFF if value & TIME_UPDATE_BIT != 0 => {
                self.latched = self.count;
            }
            SLP_TIMER0_OFF => {
                self.slp_timer0 = value;
                self.slp_timer_written = true;
            }
            SLP_TIMER1_OFF => {
                self.slp_timer1 = value;
                self.slp_timer_written = true;
            }
            SLP_WAKEUP_CAUSE_OFF => self.wakeup_cause = value,
            EXT1_SEL_OFF => {
                // SEL[21:0] persist; STATUS_CLR[22] (WO) clears the status.
                self.regs[EXT1_SEL_OFF as usize / 4] = value & 0x3F_FFFF;
                if value & (1 << 22) != 0 {
                    self.ext1_status = 0;
                }
            }
            STATE0_OFF if value & SLEEP_EN_BIT != 0 => {
                // Sleep trigger (shared by deep and light sleep). Capture
                // the period the firmware already programmed into
                // SLP_TIMER0/1 plus the deep/light kind: DG_WRAP_PD_EN
                // (DIG_PWC bit 31) means the digital core powers down, so
                // the CPUs cannot resume and the machine must reboot.
                self.sleep_req = true;
                self.sleep_deep = self.regs[0x90 / 4] & (1 << 31) != 0;
                self.sleep_target =
                    (self.slp_timer0 as u64) | ((self.slp_timer1 as u64 & 0xFFFF) << 32);
            }
            // Touch FSM trigger: any scan-control write latches DONE +
            // SCAN_DONE (synchronous completion, like SHA BUSY). The
            // driver's oneshot wait blocks on the resulting interrupt/event.
            // RTC watchdog (WDTCONFIG0..4 @ 0x98..0xA8, FEED @ 0xAC,
            // WPROTECT @ 0xB0): CONFIG gated on the protect key, FEED
            // restarts the counter and clears fired latches.
            0xB0 => {
                self.regs[0xB0 / 4] = value;
                // Latch key state implicitly via stored value (checked below).
            }
            0x98 | 0x9C | 0xA0 | 0xA4 | 0xA8 => {
                if self.regs[0xB0 / 4] == 0x50D8_3AA1 {
                    self.regs[offset as usize / 4] = value;
                }
            }
            0xAC => {
                self.wdt_count = 0;
                self.wdt_fired = [false; 4];
            }
            TOUCH_CTRL2_OFF | TOUCH_SCAN_CTRL_OFF => {
                self.regs[offset as usize / 4] = value;
                self.touch_raw |= TOUCH_DONE_BIT | TOUCH_SCAN_DONE_BIT;
            }
            INT_ENA_OFF => {
                self.regs[offset as usize / 4] = value;
            }
            INT_CLR_OFF => {
                // Write-1-to-clear over stored RAW and latched touch bits.
                self.regs[INT_RAW_OFF as usize / 4] &= !value;
                self.touch_raw &= !value;
                self.sleep_raw &= !value;
                self.regs[offset as usize / 4] = value;
            }
            o if (ULP_OFF_START..ULP_OFF_END).contains(&o) => {
                self.ulp.write32(RTC_CNTL_BASE + o, value);
            }
            o => {
                self.regs[o as usize / 4] = value;
            }
        }
    }
}
