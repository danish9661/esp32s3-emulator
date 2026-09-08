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
    /// RTC_CNTL_SLP_WAKEUP_CAUSE_REG mirror (set by the emulator on wake).
    wakeup_cause: u32,
    /// RTC_CNTL_RESET_STATE_REG mirror (reset causes for PRO/APPCPU).
    /// Seeded UNKNOWN (0) — see Default; the machine sets DEEPSLEEP on
    /// wake. Real hardware is read-only; firmware writes fall into the
    /// generic `regs` store and do not disturb this mirror.
    reset_state: u32,
    /// Generic backing store for the full RTC_CNTL page (0x000..0x400).  Most
    /// registers are simple stores; the special-cased ones below override this.
    regs: [u32; 0x400 / 4],
    /// ULP-RISC-V control/status block (offset 0x100..0x200 of this page).
    ulp: crate::ulp::Ulp,
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
            wakeup_cause: 0,
            // Reset causes report UNKNOWN (0) on normal boot — NOT the
            // silicon-accurate POWERON (1): seeding 0x41 hangs every boot
            // in `spi_flash_disable_interrupts_caches_and_other_cpu`
            // (0x403757C5: polls the core1-stall ack byte after
            // `esp_ipc_call_nonblocking`). That POWERON-only path (RF-cal
            // flash save during startup) needs early-boot IPC preemption
            // of core 1, which the model doesn't deliver before core 1
            // installs its handler. DEEPSLEEP (5) is set on wake — the
            // value firmware actually gates on (validated WOKE/PASS).
            reset_state: 0,
            regs: [0u32; 0x400 / 4],
            ulp: crate::ulp::Ulp::default(),
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
    }

    /// True if firmware requested a deep-sleep since the last call; returns
    /// the captured sleep duration (slow-clock ticks) and clears the flag.
    pub fn sleep_req(&self) -> bool {
        self.sleep_req
    }

    pub fn consume_sleep_request(&mut self) -> Option<u64> {
        if self.sleep_req {
            self.sleep_req = false;
            Some(self.sleep_target)
        } else {
            None
        }
    }

    /// Record the wakeup-cause bits read back by `esp_sleep_get_wakeup_cause`
    /// after a deep-sleep reboot (called by the machine on wake).
    pub fn set_wakeup_cause(&mut self, bits: u32) {
        self.wakeup_cause = bits;
    }

    /// Record the reset causes read back by the live ROM's
    /// `esp_rom_get_reset_reason` (called by the machine on wake).
    pub fn set_reset_cause(&mut self, pro: u32, app: u32) {
        self.reset_state = (pro & 0x3F) | ((app & 0x3F) << 6);
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            TIME_VALUE_LO_OFF => self.latched as u32,
            TIME_VALUE_HI_OFF => (self.latched >> 32) as u32,
            SLP_TIMER0_OFF => self.slp_timer0,
            SLP_TIMER1_OFF => self.slp_timer1,
            SLP_WAKEUP_CAUSE_OFF => self.wakeup_cause,
            RESET_STATE_OFF => self.reset_state,
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
            SLP_TIMER0_OFF => self.slp_timer0 = value,
            SLP_TIMER1_OFF => self.slp_timer1 = value,
            SLP_WAKEUP_CAUSE_OFF => self.wakeup_cause = value,
            STATE0_OFF if value & SLEEP_EN_BIT != 0 => {
                // Legacy S3 deep-sleep trigger.  Capture the period the
                // firmware already programmed into SLP_TIMER0/1.
                self.sleep_req = true;
                self.sleep_target =
                    (self.slp_timer0 as u64) | ((self.slp_timer1 as u64 & 0xFFFF) << 32);
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
