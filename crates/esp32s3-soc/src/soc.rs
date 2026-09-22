//! ESP32-S3 SoC: full address-space `Bus` implementation.
//!
//! Owns the internal SRAM (512 KB, aliased at the DRAM and IRAM windows —
//! same physical RAM on real silicon), the boot ROM stub storage (IROM), the
//! RTC memories, the SPI flash + PSRAM backing (reachable through the shared
//! cache MMU windows), and all modeled peripherals (UART0/1/2, GPIO, TIMG0/1,
//! interrupt matrix, cache/MMU). Addresses and windows follow QEMU `esp32s3.c`
//! + `esp32s3_reg.h` (mirrors the TRM memory map chapter).
//!
//! Unimplemented APB addresses behave like QEMU's unimplemented devices but
//! return 0 / ignore writes instead of trapping (firmware probes peripheral
//! space during boot).

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;
use core::cell::Cell;
use xtensa_core::Bus;
use xtensa_core::generated::{Opcode, decode_inst, decode_inst16a, decode_inst16b, insn_len};

use crate::adc::Adc;
use crate::aes::Aes;
use crate::cache::{Cache, CacheTarget};
use crate::ds::Ds;
use crate::ecdsa::Ecdsa;
use crate::efuse::Efuse;
use crate::gdma::{GDMA_BASE, Gdma};
use crate::gpio::Gpio;
use crate::hmac::Hmac;
use crate::i2c::I2c;
use crate::i2s::I2s;
use crate::intc::Intc;
use crate::lcd_cam::LcdCam;
use crate::ledc::Lcdc;
use crate::lp_uart::LpUart;
use crate::mcpwm::{MCPWM_BASE, MCPWM1_BASE, Mcpwm};
use crate::memmap::*;
use crate::memspi::Memspi;
use crate::pcnt::{PCNT_BASE, Pcnt};
use crate::regstore::RegStore;
use crate::rmt::{RMT_BASE, Rmt};
use crate::rng::Rng;
use crate::rsa::Rsa;
use crate::rtc::Rtc;
use crate::rtc_i2c::RtcI2c;
use crate::rtc_io::RtcIo;
use crate::sdmmc::Sdmmc;
use crate::sha::Sha;
use crate::sigmadelta::Sdm;
use crate::spi::Spi;
use crate::systimer::Systimer;
use crate::timg::{INT_T0, INT_T1, INT_WDT, Timg};
use crate::touch::Touch;
use crate::twai::{TWAI_BASE, Twai};
use crate::uart::Uart;
use crate::uhci::{UHCI0_BASE, Uhci};
use crate::ulp::{ULP_OFF_END, ULP_OFF_START, Ulp};
use crate::usb_otg::{USB_OTG_BASE, USB_OTG_FIFO_PAGE, USB_OTG_FIFO_PAGES, UsbOtg};
use crate::usb_serial_jtag::{USB_SERIAL_JTAG_INTR_SOURCE, UsbSerialJtag};
use crate::wifi::Wifi;

/// A host-observable emulator event, drained once per animation frame and
/// dispatched to virtual-peripheral JS objects (Wokwi-style). This is a plain
/// struct so the wasm-bridge can map it onto a `#[wasm_bindgen]` type without
/// coupling `esp32s3-soc` to `wasm-bindgen`.
///
/// `kind` discriminates the event; `a`/`b` carry scalar payload:
/// * `EVT_GPIO` (0): `a` = pin number, `b` = level (0/1).
/// * `EVT_SPI_XFER` (1): `a` = channel (0=GPSPI2, 1=GPSPI3), `b` = MOSI byte
///   count; the bytes themselves are retrieved via [`Soc::spi_take_tx`].
/// * `EVT_I2C_START` (2): `a` = channel.
/// * `EVT_I2C_WRITE` (3): `a` = channel, `b` = data byte.
/// * `EVT_I2C_READ` (4): `a` = channel, `b` = data byte (value the MCU read).
/// * `EVT_I2C_STOP` (5): `a` = channel.
#[derive(Clone, Copy, Debug)]
pub struct EmuEvent {
    pub kind: u8,
    pub a: u32,
    pub b: u32,
}

pub const EVT_GPIO: u8 = 0;
pub const EVT_SPI_XFER: u8 = 1;
pub const EVT_I2C_START: u8 = 2;
pub const EVT_I2C_WRITE: u8 = 3;
pub const EVT_I2C_READ: u8 = 4;
pub const EVT_I2C_STOP: u8 = 5;

/// RTC-retained state (slow/fast memory + ULP core) snapshotted before a
/// deep-sleep reboot and restored after (silicon retention behavior).
#[derive(Clone)]
pub struct RtcRetain {
    slow: alloc::boxed::Box<[u8; RTC_SLOW_SIZE as usize]>,
    fast: alloc::boxed::Box<[u8; RTC_FAST_SIZE as usize]>,
    ulp: crate::ulp::Ulp,
    // Touch controller state (thresholds, approach counters, SLP latch):
    // SENS is RTC-domain and retains over deep sleep like the ULP.
    touch: crate::touch::Touch,
}

/// I2S GDMA streaming cursor: the esp-idf I2S driver streams multi-hundred-
/// byte frames through a 16-word TX/RX FIFO, so descriptors are consumed
/// gradually as FIFO space/data allow (not dumped synchronously). One pass
/// per link-start; stops at unowned/null/eof descriptors like the sync walk.
/// LCD_CAM GDMA-RX streaming cursor (camera capture into IN
/// descriptors): like `I2sDma` but single-direction (capture only).
#[derive(Clone, Copy, Default)]
struct CamDma {
    active: bool,
    ch: usize, // GDMA channel pair carrying the IN link
    desc: u32, // current descriptor address (0 = none)
    off: u32,  // bytes already moved in the current descriptor
}

#[derive(Clone, Copy, Default)]
struct I2sDma {
    active: bool,
    ch: usize, // GDMA channel pair carrying this port's link
    desc: u32, // current descriptor address (0 = none)
    off: u32,  // bytes already moved in the current descriptor
}

macro_rules! in_range {
    ($addr:expr, $base:expr, $size:expr) => {
        ($addr).wrapping_sub($base) < ($size)
    };
}

// SRAM window size used in range checks (kept in sync with DRAM_SIZE).
const SRAM_BASE_RANGE: u32 = DRAM_SIZE;

// Block-boundary cache geometry (see `Soc::fast_tag`): 2048 entries per
// core, indexed by `(pc >> 1) & mask` (10 KB per core — L1-resident; no
// measurable difference vs 4096 on hello-boot, so the smaller table wins).
// Only flow lengths are cached — the decoded ops themselves stay in the Cpu
// decode cache, which revalidates `raw` on every op.
const FAST_CACHE_SIZE: usize = 2048;
const FAST_TAG_INVALID: u32 = 0xFFFF_FFFF;
const FAST_MAX_OPS: u8 = 16;

// Xtensa "I/O block" aliases (xtensa/config/system.h: XSHAL_IOBLOCK_CACHED/
// BYPASS). On ESP32-S3 these are cached/uncached VADDR windows over the SAME
// physical DRAM as DRAM_BASE. The esp-idf runtime stores struct pointers in
// this alias space (e.g. 0x70004), so the emulator must mirror them to DRAM.
const CACHED_IOBLOCK_BASE: u32 = 0x7000_0000;
const BYPASS_IOBLOCK_BASE: u32 = 0x9000_0000;
#[inline]
fn ioblock_remap(addr: u32) -> u32 {
    if in_range!(addr, CACHED_IOBLOCK_BASE, SRAM_BASE_RANGE) {
        DRAM_BASE + (addr - CACHED_IOBLOCK_BASE)
    } else if in_range!(addr, BYPASS_IOBLOCK_BASE, SRAM_BASE_RANGE) {
        DRAM_BASE + (addr - BYPASS_IOBLOCK_BASE)
    } else {
        addr
    }
}

// esp_image_header_t magic byte (esp_image_format.h: ESP_IMAGE_HEADER_MAGIC).
const ESP_IMAGE_MAGIC: u8 = 0xE9;

/// Per-image linked addresses for the Wi-Fi fixture engine (see
/// `Soc::wifi_fixture_layout`). Every address is a LINKED address, stable
/// for the pinned esp32 core but DIFFERENT per sketch (the STA sketch links
/// the pool/RAM elsewhere) — ground truth = `nm` on each sketch ELF.
#[derive(Clone, Copy)]
struct WifiImageLayout {
    scan_start: u32,
    connect: u32,
    wifi_event_var: u32,
    ip_event_var: u32,
    count_cell: u32,
    scan_count: u32,
    scan_result: u32,
    records_check: u32,
    ready_lists: u32,
    top_prio: u32,
    reg_heaps: u32,
    pxcur: u32,
    sta_network_if: u32,
}

/// Fixture-engine state machine (one per armed run; see `WifiFixture`
/// below for field docs).
#[derive(Clone, Copy, Default)]
struct WifiFixtureState {
    scan_armed: bool,
    scan_done: bool,
    records_done: bool,
    sta_armed: bool,
    sta_done: bool,
    sta_stage: u8,
    sta_ip_done: bool,
    disc_armed: bool,
    disc_done: bool,
}

/// An armed Wi-Fi fixture run: parsed AP list + fixed LAN + engine state.
/// Staging order mirrors run_flash exactly (count cell → SCAN_DONE post →
/// records write; CONNECTED post → GOT_IP posts → disconnect posts), so
/// firmware observes identical bytes.
#[derive(Clone)]
struct WifiFixture {
    aps: alloc::vec::Vec<crate::wifi::ScanFixtureAp>,
    ip: [u8; 4],
    st: WifiFixtureState,
}

/// SENS2 PLL-lock status model (TRM SENS2 SAR_PLL_FORCE_CTRL @ 0x6000E040).
///
/// rtc_clk (rtc_clk.c) powers the CPU PLL by writing the force/power bits of
/// this register, then polls bit 24 (PLL_LOCK) until the ~200 us lock time
/// elapses.  On silicon PLL_LOCK is a read-only status bit; the lock time is
/// independent of the CPU clock (the PLL is still settling), so we count
/// cycles (one per instruction) rather than RTC ticks.
#[derive(Default)]
pub struct PllLock {
    /// SAR_PLL_FORCE_CTRL value (bits [23:0] and [31:25] written by rtc_clk;
    /// bit 24 is the live lock status).
    force_ctrl: u32,
    /// Cycle count when a lock was armed (0 = idle).
    lock_armed_at: u64,
    /// Total cycles ticked since boot.
    cycles: u64,
}

/// PLL lock delay in CPU cycles: ~200 us at the boot-stage CPU clock.  QEMU
/// models no SENS2 at all (its esp32s3.c omits 0x6000E000), so the app's
/// lock poll would spin forever there too — the S3 machines that boot IDF
/// (esp-idf build) rely on the bootloader skipping the PLL path.
const PLL_LOCK_CYCLES: u64 = 240_000_000 / 5_000; // 200 us at 240 MHz

impl PllLock {
    /// Write to SAR_PLL_FORCE_CTRL: arm the lock timer when the PLL is
    /// powered up (bit 3 set — the app's enable sequence writes it after
    /// clearing the power-down bits [2:0]), clear the lock on power-down.
    fn write32(&mut self, value: u32) {
        self.force_ctrl = value & !(1 << 24);
        if value & (1 << 3) != 0 && self.force_ctrl & (1 << 24) == 0 {
            self.lock_armed_at = self.cycles + PLL_LOCK_CYCLES;
        } else if value & (1 << 3) == 0 {
            self.lock_armed_at = 0;
        }
    }

    fn read32(&mut self) -> u32 {
        if self.lock_armed_at != 0 && self.cycles >= self.lock_armed_at {
            self.force_ctrl |= 1 << 24;
            self.lock_armed_at = 0;
        }
        self.force_ctrl
    }

    fn tick(&mut self, cycles: u64) {
        self.cycles += cycles;
    }
}

pub struct Soc {
    /// Internal SRAM backing store (aliased at DRAM_BASE and IRAM_BASE).
    sram: Box<[u8; SRAM_BYTES]>,

    /// SRAM0 backing (32 KB, instruction-only; see memmap.rs SRAM0_SIZE).
    /// Separate cells from the D-side: the data window 0x3FC80000-0x3FC87FFF
    /// is ROM-data RAM, NOT an alias of these (esp32s3-mm.pdf).
    iram0: Box<[u8; SRAM0_SIZE as usize]>,
    /// Boot ROM storage (IROM, read-only — P3 loads the ROM stub here).
    irom: Box<[u8; IROM_SIZE as usize]>,
    /// SPI flash backing store (read-only via the XIP cache windows).
    flash: Box<[u8; FLASH_SIZE as usize]>,
    /// Last decrypted 16-byte flash block (decrypt-on-read cache for
    /// `flash_byte`: XTS works on 16-byte units; instruction fetch is
    /// sequential/loopy so one entry hits ~always; invalidated by
    /// invalid base = u32::MAX and after every MEMSPI write).
    flashenc_last: Cell<(u32, [u8; 16])>,
    /// PSRAM backing store (read-write through MMU-mapped cache pages).
    psram: Box<[u8; PSRAM_SIZE as usize]>,
    rtc_slow: Box<[u8; RTC_SLOW_SIZE as usize]>,
    rtc_fast: Box<[u8; RTC_FAST_SIZE as usize]>,
    rom_data: Box<[u8; ROM_DATA_SIZE as usize]>,
    /// Host event-block pool for arduino-queue posts (see `wifi_ard_post`):
    /// four 192-byte slots (768 B) the host wraps as fake heap blocks. Kept
    /// as a SEPARATE backing (not inside `sram`) so the addresses are always
    /// outside every heap's `[start, end]` bounds — `heap_caps_free` then
    /// skips the host pointer in its registered-heap walk (no heap claims
    /// it) and the event leaks by design (4 slots/run max) instead of
    /// aborting in `assert_valid_block`. Reads/writes route through the Bus
    /// impl below (`WIFI_ARD_POOL` range check before the DRAM check).
    ard_pool: Box<[u8; 768]>,
    /// USB-Serial-JTAG (CDC-ACM console) controller. The boot ROM's console
    /// (uart_tx_one_char @ 0x40048C30) writes chars to the USB_SERIAL_JTAG FIFO
    /// (0x60038000), NOT UART0 — the S3's ROM messages come out of the USB-CDC
    /// port on real hardware. `Esp32S3::take_usb_serial_tx` drains this too.
    usb: UsbSerialJtag,
    /// ULP-RISC-V coprocessor (its own rv32im core; runs from RTC_SLOW_MEM).
    ulp: Ulp,
    uarts: [Uart; 3],
    /// UHCI0 DMA bridge (UART0/1/2 <-> GDMA peri_sel 2).
    uhci: Uhci,
    gpio: Gpio,
    ledc: Lcdc,
    mcpwm: Mcpwm,
    /// MCPWM group 1: independent copy of the group-0 block.
    mcpwm1: Mcpwm,
    /// Dedicated-GPIO output latches mirrored per CPU core from the CPUs'
    /// `tie_gpio` TIE registers (the machine syncs them every step; the
    /// SoC cannot see CPU state itself). Bit c = OUT channel c.
    dedic_out: [u32; 2],
    lp_uart: LpUart,
    spi: [Spi; 2],
    /// SPI1 (0x60002000) + SPIMEM0 (0x60003000) flash controllers, sharing
    /// the `flash` backing (see `crate::memspi`).
    pub memspi: [Memspi; 2],
    i2c: [I2c; 2],
    rmt: Rmt,
    twai: Twai,
    adc: Adc,
    pcnt: Pcnt,
    gdma: Gdma,
    efuse: Efuse,
    sha: Sha,
    aes: Aes,
    rsa: Rsa,
    ecdsa: Ecdsa,
    hmac: Hmac,
    ds: Ds,
    cache: Cache,
    timg: [Timg; 2],
    systimer: Systimer,
    touch: Touch,
    rtc: Rtc,
    rtc_i2c: RtcI2c,
    rtc_io: RtcIo,
    /// Deep-sleep wake cause evaluated at sleep entry (applied on wake).
    sleep_cause: u32,
    /// EXT1 triggering pads evaluated at sleep entry.
    sleep_ext1: u32,
    /// Watched ULP cause bits (ULP and/or COCPU): set when armed and the
    /// ULP is running at sleep entry; latched into the cause on its halt.
    sleep_ulp_watch: u32,
    /// I2S GDMA streaming cursors (OUT = TX, IN = RX) per I2S port.
    i2s_dma_out: [I2sDma; 2],
    i2s_dma_in: [I2sDma; 2],
    cam_dma_in: CamDma,
    /// IN-link freshness for the retroactive CAM arm: set when an IN link
    /// for the camera peri starts, consumed when a capture uses it (retro
    /// arm) or the pump parks the cursor — so a later CAM_START for a
    /// polling-mode capture can't re-arm off the stale link.
    cam_link_fresh: bool,
    rng: Rng,
    sdmmc: Sdmmc,
    sdm: Sdm,
    intc: Intc,
    /// SENS2 PLL-lock status (TRM SENS2 SAR_PLL_FORCE_CTRL @ 0x6000E040):
    /// rtc_clk powers the CPU PLL, then polls bit 24 (PLL_LOCK) until the
    /// ~200 us lock time elapses; the register is read-only on silicon for
    /// that bit, so writes to it only matter for arming the lock timer.
    pll: PllLock,
    /// SENS2 TX-DC cal raw store @ 0x6000E04C (WIFI BRING-UP, see the
    /// SENS2 mmio arm + `Wifi::{txdc_write,txdc_read}`): plain store of
    /// the last-written value; the read arm overlays the proven bit-24
    /// done bit via the one-shot latch in `Wifi`.
    pll_cal4c: u32,

    // ── P5 register-store peripherals (configure-and-forget, no observable
    //    side-effects modeled — see regstore.rs). Bases from esp-idf
    //    components/soc/esp32s3/register/soc/reg_base.h.
    /// SENSITIVE (= Mem-Protection/PMS, DR_REG_SENSITIVE_BASE 0x600C1000).
    sensitive: RegStore,
    /// WCL (= World Controller / TEE, DR_REG_WCL_BASE 0x600D0000).
    wcl: RegStore,
    /// PERI_BACKUP (retention registers, DR_REG_PERI_BACKUP_BASE 0x6002A000).
    peri_backup: RegStore,
    /// SYSCON (sysclk/tick/clock-out config, 0x60026000; NOT peripheral
    /// clocks — those are SYSTEM_PERIP_CLK_EN0/1, see sys_clk_en* below).
    syscon: RegStore,
    /// ASSIST_DEBUG (watchpoint/breakpoint unit, DR_REG_ASSIST_DEBUG_BASE
    /// 0x600CE000).
    assist_debug: RegStore,
    /// USB-OTG DWC core (device/host, 0x60080000 + DFIFO page 0x60081000).
    /// Functional init path: core soft reset, device config, EP0 control,
    /// TXFIFO staging; enumeration needs a host (see usb_otg.rs).
    usb_otg: UsbOtg,
    /// USB_WRAP (OTG PHY wrapper, DR_REG_USB_WRAP_BASE 0x60039000):
    /// plain register store (PHY test/pullup pokes never panic).
    usb_wrap: RegStore,
    /// WiFi radio blocks (FE/FE2 @ 0x60006000/0x60005000, BB @ 0x6001D000,
    /// NRX @ 0x6001CC00). TEMP WIFI BRING-UP scaffold: plain stores +
    /// proven RF-cal done-bits (see wifi.rs).
    wifi: Wifi,
    /// Which firmware image the Wi-Fi fixture engine serves (scan vs STA
    /// sketch link the pool/RAM differently; the layout table lives in
    /// `wifi_fixture_layout`). Set once via `wifi_fixture_image` before
    /// arming; defaults to scan.
    wifi_image_sta: bool,
    /// Image-specific addresses discovered live by the host (WiFi fixture
    /// support): the `registered_heaps` SLIST head, the `pxCurrentTCBs`
    /// (current-TCB-per-core) array, the `_ZL15_sta_network_if` STA-instance
    /// static (nm per image — a bss pointer to the STAClass instance, NOT
    /// the `B WiFi` object), and the `D WIFI_EVENT` / `D IP_EVENT` pointer
    /// variables. All default to `None` = undiscovered (fixture calls fail
    /// softly, like an allocation failure — never a wrong-address write);
    /// the host sets per-image via `wifi_layout_*` before arming the dwell
    /// (defaults for the pinned wifi-scan image live in the harness, not
    /// here).
    ///
    /// Cached by the host after first discovery (the arduino_events queue
    /// and waiter are stable once `Network.initEvents` runs).
    wifi_reg_heaps: Option<u32>,
    wifi_pxcur: Option<u32>,
    wifi_network: Option<u32>,
    wifi_event_var: Option<u32>,
    wifi_ip_event_var: Option<u32>,
    wifi_ard_queue: Option<u32>,
    #[allow(dead_code)]
    wifi_ard_waiter: Option<u32>,
    /// Bump cursor into `WIFI_ARD_POOL` (see `wifi_ard_post`): slot index
    /// 0..4, advanced per post, never wraps within a run (one host event
    /// queued at a time in every fixture flow — same single-flight
    /// discipline as the IDF-side scratch).
    wifi_ard_slot: u32,
    /// Staged STA fixture data (programmed by the host when it posts
    /// GOT_IP; served back by the pc-intercept hooks below): the connected
    /// AP record (92-byte `wifi_ap_record_t`) + the interface IP info
    /// (12-byte ip/mask/gw). `None` until staged.
    wifi_ap_record: Option<[u8; 92]>,
    wifi_ip_info: Option<[u8; 12]>,
    /// Served ap-record reads (SSID + RSSI = 2): the disconnect-leg arm
    /// gate (see `wifi_hook_rssi_done`).
    wifi_ap_reads: u32,
    /// Latch set by the machine when the `esp_wifi_disconnect` hook fires;
    /// consumed by the run_flash disconnect leg (one-shot arm).
    wifi_disc_fired: bool,
    /// Self-contained Wi-Fi fixture engine (browser/bridge path — mirrors
    /// the run_flash host blocks; see `wifi_fixture_*` below). `None` =
    /// no fixture armed (firmware runs unmodified, like silicon with no
    /// AP in range).
    wifi_fixture: Option<WifiFixture>,
    /// I2S audio controllers (I2S0 @ 0x6000F000, I2S1 @ 0x6002D000).
    /// Functional model: TX/RX FIFO + serial shift-out onto GPIO-matrix
    /// signals (BCK/WS/SD).
    i2s: [I2s; 2],

    /// LCD_CAM (= parallel I/O / PARLIO, DR_REG_LCD_CAM_BASE 0x60041000).
    /// Functional model: FIFO data path + transfer-done interrupt, with the
    /// parallel data/clock signals driven onto the GPIO matrix.
    lcd_cam: LcdCam,

    /// SYSTEM.APPCPU_CTRL_A (0x600C0004): the APP-CPU release register.
    /// `ets_set_appcpu_boot_addr` (ROM 0x40043664) stores the core-1 entry
    /// address here; the core-1 reset path (real ROM fastboot / our
    /// CORE1_WAIT stub) jumps to it.  The rest of the SYSTEM page is not
    /// modeled (dropped writes, reads 0).
    appcpu_ctrl_a: u32,
    /// SYSTEM.PERIP_CLK_EN0/1 (0x600C0018/1C): peripheral clock gates.
    /// Seeded with the system_reg.h reset defaults (every clock that
    /// matters is on out of reset); drivers RMW individual bits.
    /// `clk_on` below gates frozen peripherals on these (see there).
    sys_clk_en0: u32,
    sys_clk_en1: u32,

    /// SYSTEM.CPU_INT_FROM_CPU_0..3 (0x600C0030/4/8/C): cross-core
    /// interrupt registers.  +0x30 asserts FROM_CPU_INTR0 (source 79, used
    /// by FreeRTOS SMP for yields) on core 0's matrix; +0x34 asserts
    /// FROM_CPU_INTR1 (source 80) on core 1's.  +0x38/+0x3C assert
    /// FROM_CPU_INTR2/3 (sources 81/82, used by `esp_ipc_isr` for the
    /// stall/mute handshake, e.g. deep-sleep entry) on core 0/1's matrix.
    /// The ISR clears its bit by writing 0 back.  Level-style sticky bit
    /// per core (bit 0 = asserted).
    cpu_int_from_cpu: [u32; 4],

    /// ROM-boot phase: while set, cache-window reads bypass the MMU and map
    /// 1:1 to raw flash. The real ROM bootloader reads flash via SPI with the
    /// MMU uninvolved; our ROM stub reads through the data window as a
    /// stand-in, but the machine pre-maps the app's text/rodata pages before
    /// the stub runs — without this bypass the stub's image-header/segment
    /// reads would be redirected to those mapped flash pages (garbage).
    /// Cleared by `Esp32S3::step` once core 0 leaves the ROM (stub done).
    rom_boot_mode: bool,

    /// Byte length of the app image staged at the loader scratch offset
    /// (set by `map_app_flash_segments`; defaults to the legacy 384 KB cap).
    /// Images bigger than 384 KB (e.g. MicroPython, ~1.8 MB) address the
    /// whole image through the scratch view — reads past a fixed cap alias
    /// to raw flash at the window offset (zeros/wrong bytes), silently
    /// dropping later segment headers.
    loader_scratch_len: u32,

    /// Cached interrupt source bitmap, valid only while `src_valid` is true.
    /// Each bit corresponds to an ETS_*_INTR_SOURCE number.  Set by
    /// `int_pending()` on the first call after `tick_timers()` invalidates it.
    /// The second core within the same step reuses this cache, halving the
    /// 22-peripheral scan from 2 calls/step to 1 call/step.
    cached_src: u128,
    /// Whether `cached_src` is current.  Cleared by `tick_timers()` (which
    /// may change peripheral interrupt state) and by `write32()` / `inject_rx`
    /// paths that modify interrupt state between ticks.
    src_valid: bool,
    /// Cached GPIO_IN pad-readback word, valid only while `rb_valid` is
    /// true. `gpio_in_readback()` walks all 46 pins (enable + out_sel +
    /// signal_level per pin); tick_timers calls it up to 9x per tick
    /// (UART CTS x3, RMT RX, MCPWM cap/fault/sync x2 groups, GPIO IRQ,
    /// PCNT-less dedic path...), and the inputs only change when firmware
    /// writes GPIO/SPI/RMT/MCPWM/LCD_CAM registers or a peripheral tick
    /// advances a waveform. Cleared at the START of every tick (before any
    /// caller runs) and re-armed by the first caller; WITHIN a tick every
    /// caller sees the same word. Sound because all `gpio_in_readback`
    /// callers run inside `tick_timers` (the machine steps CPUs and ticks
    /// strictly alternately — no CPU write can land mid-tick), and each
    /// new tick clears the flag before re-reading. Callers OUTSIDE ticks
    /// (MMIO reads, sleep/wake evaluation, tests) also share the flag,
    /// which is safe for the same reason: any MMIO write that could move
    /// a pad level clears it first (see `write32`).
    cached_rb: u32,
    /// Whether `cached_rb` is current (same invalidation as `src_valid`).
    rb_valid: bool,

    /// Host-observable event queue (GPIO/SPI/I2C), drained once per frame by
    /// the wasm bridge and dispatched to virtual-peripheral JS objects.
    events: Vec<EmuEvent>,
    /// Last GPIO output mask reported via `drain_events` (for edge detection).
    last_gpio_out: u32,
    /// Most recent SPI MOSI byte stream per channel, retrieved by the host
    /// when it sees an `EVT_SPI_XFER` event.
    pending_spi_tx: [Vec<u8>; 2],

    /// Block-boundary cache for block-at-a-time execution (machine
    /// `step_fast`): per core, `fast_tag[c][i]` is the block-start pc
    /// (`FAST_TAG_INVALID` = empty) and `fast_len[c][i]` the instruction
    /// count (1..=16) of the straight-line run starting there, branch op
    /// inclusive. Only control flow is cached — decode still goes through
    /// the Cpu decode cache on every op (which revalidates `raw`, so
    /// patched immediates/data execute correctly), and the runner aborts on
    /// any pc deviation. Deliberately NOT flushed on RAM writes: the hot
    /// write path (stack spills) stays untouched, and no in-tree firmware
    /// modifies code after executing it (the loader writes precede the jump;
    /// WDT reset builds a fresh SoC). Known limitation: inserting a branch
    /// into the middle of an already-cached block is not observed until the
    /// entry is evicted by index collision.
    fast_tag: [Box<[u32; FAST_CACHE_SIZE]>; 2],
    fast_len: [Box<[u8; FAST_CACHE_SIZE]>; 2],
}

impl Soc {
    pub fn new() -> Self {
        Self {
            sram: Box::new([0; SRAM_BYTES]),
            iram0: Box::new([0; SRAM0_SIZE as usize]),
            irom: Box::new([0; IROM_SIZE as usize]),
            flash: Box::new([0; FLASH_SIZE as usize]),
            flashenc_last: Cell::new((u32::MAX, [0; 16])),
            psram: Box::new([0; PSRAM_SIZE as usize]),
            rtc_slow: Box::new([0; RTC_SLOW_SIZE as usize]),
            rtc_fast: Box::new([0; RTC_FAST_SIZE as usize]),
            rom_data: Box::new([0; ROM_DATA_SIZE as usize]),
            ard_pool: Box::new([0; 768]),
            uarts: [Uart::new(), Uart::new(), Uart::new()],
            uhci: Uhci::new(),
            usb: UsbSerialJtag::new(),
            ulp: Ulp::new(),
            gpio: Gpio::new(),
            ledc: Lcdc::new(),
            mcpwm: Mcpwm::new(),
            dedic_out: [0; 2],
            mcpwm1: Mcpwm::new(),
            lp_uart: LpUart::new(),
            spi: [Spi::new(0), Spi::new(1)],
            memspi: [Memspi::new(), Memspi::new()],
            i2c: [I2c::new(0), I2c::new(1)],
            rmt: Rmt::new(),
            twai: Twai::new(),
            adc: Adc::new(),
            pcnt: Pcnt::new(),
            gdma: Gdma::default(),
            efuse: Efuse::new(),
            sha: Sha::new(),
            aes: Aes::new(),
            rsa: Rsa::default(),
            ecdsa: Ecdsa::new(),
            hmac: Hmac::new(),
            ds: Ds::new(),
            cache: Cache::new(),
            timg: [Timg::new(), Timg::new()],
            systimer: Systimer::new(),
            touch: Touch::new(),
            rtc: Rtc::new(),
            rtc_i2c: RtcI2c::new(),
            rtc_io: RtcIo::new(),
            sleep_cause: 0,
            sleep_ext1: 0,
            sleep_ulp_watch: 0,
            i2s_dma_out: [I2sDma::default(); 2],
            i2s_dma_in: [I2sDma::default(); 2],
            cam_dma_in: CamDma::default(),
            cam_link_fresh: false,
            rng: Rng::new(),
            sdmmc: Sdmmc::new(),
            sdm: Sdm::new(),
            intc: Intc::new(),
            pll: PllLock::default(),
            pll_cal4c: 0,
            sensitive: RegStore::new(0x1000),
            wcl: RegStore::new(0x1000),
            peri_backup: RegStore::new(0x1000),
            syscon: RegStore::new(0x1000),
            i2s: [I2s::new(0), I2s::new(1)],
            assist_debug: RegStore::new(0x1000),
            usb_otg: UsbOtg::new(),
            usb_wrap: RegStore::new(0x1000),
            wifi: Wifi::new(),
            wifi_image_sta: false,
            wifi_reg_heaps: None,
            wifi_pxcur: None,
            wifi_network: None,
            wifi_event_var: None,
            wifi_ip_event_var: None,
            wifi_ard_queue: None,
            wifi_ard_waiter: None,
            wifi_ard_slot: 0,
            wifi_ap_record: None,
            wifi_ip_info: None,
            wifi_ap_reads: 0,
            wifi_disc_fired: false,
            wifi_fixture: None,
            lcd_cam: LcdCam::new(),
            appcpu_ctrl_a: 0,
            sys_clk_en0: 0xF9C1_E06F,
            sys_clk_en1: 0x0000_0600,
            cpu_int_from_cpu: [0, 0, 0, 0],
            rom_boot_mode: false,
            loader_scratch_len: 0x6_0000,
            cached_src: 0,
            src_valid: false,
            cached_rb: 0,
            rb_valid: false,
            events: Vec::new(),
            last_gpio_out: 0,
            pending_spi_tx: [Vec::new(), Vec::new()],
            fast_tag: [
                Box::new([FAST_TAG_INVALID; FAST_CACHE_SIZE]),
                Box::new([FAST_TAG_INVALID; FAST_CACHE_SIZE]),
            ],
            fast_len: [
                Box::new([0; FAST_CACHE_SIZE]),
                Box::new([0; FAST_CACHE_SIZE]),
            ],
        }
    }

    /// ROM storage, for the boot ROM stub (P3).
    pub fn irom_mut(&mut self) -> &mut [u8] {
        &mut self.irom[..]
    }

    /// Load flash image bytes at flash offset `offset` (e.g. a whole merged
    /// esptool image at offset 0). Read-only for the CPU via the XIP windows.
    pub fn load_flash_image(&mut self, offset: u32, bytes: &[u8]) {
        let start = offset as usize;
        let end = (start + bytes.len()).min(FLASH_SIZE as usize);
        self.flash[start..end].copy_from_slice(&bytes[..end - start]);
    }

    /// Peripheral clock gate (SYSTEM PERIP_CLK_EN0 @ +0x18 / EN1 @ +0x1C,
    /// reset defaults per system_reg.h — everything validated runs
    /// clocked). `en1` selects EN1, else EN0. A gated peripheral is frozen:
    /// its tick stops and triggered transfers/engines don't start (no
    /// completion events — firmware hangs like silicon). Plain register
    /// reads/writes still round-trip (documented leniency, so init
    /// sequences that program before enabling keep working).
    fn clk_on(&self, en1: bool, bit: u32) -> bool {
        if en1 {
            self.sys_clk_en1 & (1 << bit) != 0
        } else {
            self.sys_clk_en0 & (1 << bit) != 0
        }
    }

    /// Current SPI flash contents, including MEMSPI program/erase writes.
    /// Silicon flash is non-volatile: the machine snapshots this across
    /// resets (OTA updates, NVS) instead of rebooting from the original
    /// image.
    pub fn flash_image(&self) -> &[u8] {
        &self.flash[..]
    }

    /// Owned plaintext view of the whole flash image (block-decrypted when
    /// encryption is enabled, else a copy). Boot parsing (partition table,
    /// app headers) reads from this — the ROM bootloader sees decrypted
    /// bytes through the hardware transparently.
    pub fn flash_image_decrypted(&self) -> Vec<u8> {
        if !self.flash_enc_enabled() {
            return self.flash.to_vec();
        }
        let key = self.flash_xts_key();
        let mut out = vec![0u8; self.flash.len()];
        for (b, dst) in out.chunks_exact_mut(16).enumerate() {
            let off = (b * 16) as u32;
            let mut blk = [0u8; 16];
            blk.copy_from_slice(&self.flash[b * 16..b * 16 + 16]);
            dst.copy_from_slice(&crate::aes::flash_xts_decrypt(&key, off, &blk));
        }
        out
    }

    /// Secure-boot gate: eFuse RD_REPEAT_DATA4 SECURE_BOOT_EN
    /// (efuse_reg.h bit 20). The mask-ROM verifies the bootloader signature
    /// when set; the emulator's ROM stub performs no signature verification,
    /// so a secure-enabled boot is fail-closed (see `boot_from_flash`).
    /// Resets 0 (disabled) like silicon.
    pub fn secure_boot_enabled(&self) -> bool {
        self.efuse.secure_boot_enabled()
    }

    /// Flash-encryption gate: SPI_BOOT_CRYPT_CNT (eFuse RD_REPEAT_DATA1
    /// bits [20:18]) has odd parity. Matches
    /// `efuse_hal_flash_encryption_enabled` exactly (disassembled ground
    /// truth); resets 0 (disabled) like silicon.
    pub fn flash_enc_enabled(&self) -> bool {
        self.efuse.crypt_cnt().count_ones() & 1 == 1
    }

    /// XTS key bytes from eFuse BLOCK_KEY0 (the flash-encryption key
    /// block; same big-endian-per-word layout `hmac_key` reads).
    fn flash_xts_key(&self) -> [u8; 32] {
        self.efuse.hmac_key(0)
    }

    /// Host-provision a factory-encrypted device fixture: install the
    /// 256-bit XTS key into BLOCK_KEY0's mirror and set SPI_BOOT_CRYPT_CNT
    /// (one-way, like a burn), then mirror into both MEMSPI controllers.
    /// Real burn flows are validated separately (efuse_burn sketch).
    pub fn flashenc_provision(&mut self, key: &[u8; 32]) {
        for i in 0..8 {
            let w = ((key[4 * i] as u32) << 24)
                | ((key[4 * i + 1] as u32) << 16)
                | ((key[4 * i + 2] as u32) << 8)
                | (key[4 * i + 3] as u32);
            self.efuse.write32(0x9C + 4 * i as u32, w);
        }
        self.efuse.write32(0x34, 1 << 18);
        self.refresh_flashenc_mirror();
    }

    /// Mirror the eFuse flash-encryption state into both MEMSPI controllers
    /// (key when enabled, else plaintext) and drop the XIP decrypt cache.
    /// Called after provisioning and after every eFuse MMIO write, so the
    /// mirror can never go stale (firmware provisions HMAC keys at
    /// runtime through the same registers).
    fn refresh_flashenc_mirror(&mut self) {
        let key = self.flash_enc_enabled().then(|| self.flash_xts_key());
        for m in self.memspi.iter_mut() {
            m.set_flashenc(key);
        }
        self.flashenc_last.set((u32::MAX, [0; 16]));
    }

    /// XTS-encrypt `len` bytes of backing at `off` in place (fixture
    /// encryption; both must be 16-byte multiples — the whole image is,
    /// and so is every flash transaction). Tweak per block =
    /// LE128(absolute flash offset), so blocks stay independently
    /// addressable. No-op unless provisioned.
    pub fn flashenc_encrypt_region(&mut self, off: u32, len: u32) {
        if !self.flash_enc_enabled() {
            return;
        }
        let key = self.flash_xts_key();
        let end = ((off + len) as usize).min(self.flash.len());
        let mut b = (off as usize) & !15;
        while b + 16 <= end {
            let mut blk = [0u8; 16];
            blk.copy_from_slice(&self.flash[b..b + 16]);
            let enc = crate::aes::flash_xts_encrypt(&key, b as u32, &blk);
            self.flash[b..b + 16].copy_from_slice(&enc);
            b += 16;
        }
        self.refresh_flashenc_mirror();
    }

    /// Snapshot eFuse OTP state across a chip reset (eFuse is non-volatile
    /// silicon; without this an encrypted device loses its key on reboot).
    pub fn efuse_snapshot(&self) -> Efuse {
        self.efuse.clone()
    }

    /// Restore eFuse OTP state after a chip reset (see `efuse_snapshot`).
    pub fn restore_efuse(&mut self, e: Efuse) {
        self.efuse = e;
        self.refresh_flashenc_mirror();
    }

    /// Parse an ESP-IDF app image at flash offset `app_flash_off` and program
    /// the cache MMU so the firmware can run:
    ///
    /// 1. The whole app image is mapped at the loader scratch offset
    ///    (`LOADER_SCRATCH_OFF`), where the ROM stub reads it.  The I/D
    ///    windows share ONE 512-entry MMU table indexed by window offset, so
    ///    the app image at flash offset 0x10000 (page 1) cannot stay readable
    ///    at its 1:1 offset once step 2 maps the instruction-window pages the
    ///    app's `.flash.text` lives on — the real bootloader reads the app
    ///    via SPI instead, which is why it never collides.
    /// 2. The app's flash-mapped segments (`.flash.text` in the 0x4200_0000
    ///    instruction window, `.flash.rodata` in the 0x3C00_0000 data
    ///    window) — the real 2nd-stage bootloader maps these instead of
    ///    copying them to RAM (esp_image_format.h; ESP32-S3 TRM cache MMU).
    ///
    /// The app's own cache init later re-programs the same segment mappings.
    ///
    /// `img` is the flash image bytes the headers/segments are parsed from:
    /// normally the raw backing, or the decrypted view on encrypted devices
    /// (the MMU still maps the raw backing pages — reads decrypt on the
    /// fly — and RTC preloads land as plaintext either way).
    pub fn map_app_flash_segments(&mut self, img: &[u8], app_flash_off: u32) {
        let base = app_flash_off as usize;
        if base + 24 > img.len() || img[base] != ESP_IMAGE_MAGIC {
            return;
        }
        let nseg = img[base + 1] as usize;
        // Walk once to find the image end (segments are back-to-back).
        let mut end = base + 24; // esp_image_header_t is 24 bytes
        for _ in 0..nseg {
            if end + 8 > img.len() {
                break;
            }
            let len = u32::from_le_bytes(img[end + 4..end + 8].try_into().unwrap()) as usize;
            end += 8 + len;
        }
        // 1. Loader scratch: map every flash page the app image spans.
        self.loader_scratch_len = (end - base) as u32;
        let scratch_page0 = LOADER_SCRATCH_OFF / CACHE_PAGE_SIZE;
        let fpage0 = (base / CACHE_PAGE_SIZE as usize) as u32;
        let app_pages = (end - base).div_ceil(CACHE_PAGE_SIZE as usize) as u32;
        for i in 0..app_pages {
            self.cache
                .mmu_write32(((scratch_page0 + i) & 0x1FF) * 4, (fpage0 + i) & 0x3FFF);
        }
        // 2. Flash-mapped segments.
        let mut pos = base + 24;
        for _ in 0..nseg {
            if pos + 8 > img.len() {
                break;
            }
            let load = u32::from_le_bytes(img[pos..pos + 4].try_into().unwrap());
            let len = u32::from_le_bytes(img[pos + 4..pos + 8].try_into().unwrap()) as usize;
            let data = pos + 8;
            if in_range!(load, FLASH_DATA_BASE, FLASH_WINDOW_SIZE)
                || in_range!(load, FLASH_INST_BASE, FLASH_WINDOW_SIZE)
            {
                // Map every 64 KB page the segment spans: vpage from the
                // window offset, flash page from the segment's flash offset.
                let vpage0 = (load & WINDOW_MASK) / CACHE_PAGE_SIZE;
                let fpage0 = (data / CACHE_PAGE_SIZE as usize) as u32;
                let pages = len.div_ceil(CACHE_PAGE_SIZE as usize) as u32;
                for i in 0..pages {
                    self.cache
                        .mmu_write32(((vpage0 + i) & 0x1FF) * 4, (fpage0 + i) & 0x3FFF);
                }
            } else if in_range!(load, RTC_SLOW_BASE, RTC_SLOW_SIZE) {
                // RTC slow-memory segment (e.g. ULP init data): the ROM stub
                // loader skips every load >= 0x42000000, so preload host-side.
                let src_end = (data + len).min(img.len());
                let room = (RTC_SLOW_SIZE - (load - RTC_SLOW_BASE)) as usize;
                let n = (src_end - data).min(room);
                for i in 0..n {
                    let b = img[data + i];
                    self.rtc_slow[(load - RTC_SLOW_BASE) as usize + i] = b;
                }
            } else if in_range!(load, RTC_FAST_BASE, RTC_FAST_SIZE) {
                // RTC fast-memory segment (linker rtc_iram_seg @ 0x600FE000:
                // IPC/sleep helpers core 1 executes). Skipped by the stub
                // loader like the slow segment above — preload host-side.
                let src_end = (data + len).min(img.len());
                let room = (RTC_FAST_SIZE - (load - RTC_FAST_BASE)) as usize;
                let n = (src_end - data).min(room);
                for i in 0..n {
                    let b = img[data + i];
                    self.rtc_fast[(load - RTC_FAST_BASE) as usize + i] = b;
                }
            }
            pos = data + len;
        }
    }

    /// Bytes emitted by UART `n` since the last call (host console output).
    /// Number of bytes queued in the USB-Serial-JTAG TX capture (debug probe).
    pub fn usb_tx_len(&self) -> usize {
        self.usb.tx_len()
    }

    /// Number of bytes queued in UART `n`'s TX capture (debug probe).
    pub fn uart_tx_len(&self, n: usize) -> usize {
        self.uarts[n].tx_len()
    }

    pub fn take_uart_tx(&mut self, n: usize) -> Vec<u8> {
        self.uarts[n].take_tx()
    }

    /// True when UART `n` has undrained console bytes. Cheap field read for
    /// the host console fast path (avoids Vec handoffs when idle).
    #[inline]
    pub fn uart_tx_pending(&self, n: usize) -> bool {
        self.uarts[n].tx_len() != 0
    }

    /// True when the USB-Serial-JTAG controller has undrained TX bytes.
    #[inline]
    pub fn usb_tx_pending(&self) -> bool {
        self.usb.tx_len() != 0
    }

    /// Block-cache index for a pc (both cores share the function, never the
    /// arrays).
    #[inline]
    fn fast_idx(pc: u32) -> usize {
        ((pc >> 1) as usize) & (FAST_CACHE_SIZE - 1)
    }

    /// Look up (building on first touch) the straight-line instruction count
    /// starting at `pc` for `core`. Returns None when the first op doesn't
    /// decode — the caller falls back to single-step, which raises ILLEGAL
    /// through the normal path.
    pub fn fast_len_for(&mut self, core: usize, pc: u32) -> Option<u8> {
        let idx = Self::fast_idx(pc);
        if self.fast_tag[core][idx] == pc {
            let l = self.fast_len[core][idx];
            if l > 0 {
                return Some(l);
            }
        }
        let built = self.build_fast_len(pc)?;
        self.fast_tag[core][idx] = pc;
        self.fast_len[core][idx] = built;
        Some(built)
    }

    /// Decode the straight-line run at `pc`: up to `FAST_MAX_OPS` ops,
    /// branch op inclusive (the runner stops on any pc deviation, so windowed
    /// calls/returns are safe to include — the per-op window checks still run
    /// inside `step_one`). Matches the machine `is_branch` list.
    fn build_fast_len(&mut self, pc: u32) -> Option<u8> {
        let mut cur = pc;
        let mut n = 0u8;
        for _ in 0..FAST_MAX_OPS {
            let b0 = Bus::read8(self, cur) as u8;
            let len = insn_len(b0);
            if len == 0 || len > 4 {
                break;
            }
            let raw = if len == 2 {
                Bus::read16(self, cur)
            } else {
                Bus::read32(self, cur)
            };
            let opc = if len == 2 {
                if b0 & 0xf <= 11 {
                    decode_inst16a(raw)
                } else {
                    decode_inst16b(raw)
                }
            } else {
                decode_inst(raw)
            };
            let opc = match opc {
                Some(o) => o,
                None => break,
            };
            n += 1;
            if Self::fast_is_branch(opc) {
                break;
            }
            cur = cur.wrapping_add(len);
        }
        if n == 0 { None } else { Some(n) }
    }

    /// Branch terminators for fast blocks (same set as machine `is_branch`:
    /// any op that can leave the linear flow ends the run, inclusive).
    fn fast_is_branch(opc: Opcode) -> bool {
        matches!(
            opc,
            Opcode::OPCODE_J
                | Opcode::OPCODE_CALL0
                | Opcode::OPCODE_CALL4
                | Opcode::OPCODE_CALL8
                | Opcode::OPCODE_CALL12
                | Opcode::OPCODE_CALLX0
                | Opcode::OPCODE_CALLX4
                | Opcode::OPCODE_CALLX8
                | Opcode::OPCODE_CALLX12
                | Opcode::OPCODE_RET
                | Opcode::OPCODE_RETW
                | Opcode::OPCODE_RET_N
                | Opcode::OPCODE_RFI
                | Opcode::OPCODE_RFE
                | Opcode::OPCODE_LOOP
                | Opcode::OPCODE_LOOPNEZ
                | Opcode::OPCODE_LOOPGTZ
                | Opcode::OPCODE_ENTRY
                | Opcode::OPCODE_BNE
                | Opcode::OPCODE_BEQ
                | Opcode::OPCODE_BLT
                | Opcode::OPCODE_BLTU
                | Opcode::OPCODE_BGE
                | Opcode::OPCODE_BGEU
                | Opcode::OPCODE_BNEZ
                | Opcode::OPCODE_BEQZ
                | Opcode::OPCODE_BNEZ_N
                | Opcode::OPCODE_BEQZ_N
                | Opcode::OPCODE_JX
        )
    }

    /// Push one received byte into UART `n`'s RX FIFO (host console input).
    pub fn uart_inject_rx(&mut self, n: usize, byte: u8) {
        self.uarts[n].inject_rx(byte);
    }

    /// Simulate a USB host-observed device disconnect on the OTG port
    /// (test/host frontend for removal-event paths).
    pub fn usb_otg_disconnect(&mut self) {
        self.usb_otg.host_disconnect();
    }

    /// Auto-enum host frontend: deliver one bus reset + enumeration-done
    /// to the OTG device core (the silicon-true pair the DWC2 raises on
    /// connect; the TinyUSB device ISR runs its bus-reset handler).
    pub fn usb_host_bus_reset(&mut self) {
        self.usb_otg.usb_host_bus_reset();
    }

    /// Auto-enum host frontend: deliver one 8-byte SETUP packet from the
    /// host to the OTG device core (GRXSTSP SETUP_RX + SETUP_DONE pops,
    /// RXFIFO bytes, DOEPINT0 STPKTRCVD + SETUP, DAINT OUT EP0).
    pub fn usb_host_setup(&mut self, pkt: [u8; 8]) {
        self.usb_otg.usb_host_setup(pkt);
    }

    /// Auto-enum host frontend: complete the OUT status stage on the OTG
    /// device core (latches both XFRC flags, applies pending SET_ADDRESS).
    pub fn usb_host_status_out(&mut self) {
        self.usb_otg.usb_host_status_out();
    }

    /// Drain bytes the OTG device firmware pushed for IN stages (what the
    /// host reads off the wire): assert the firmware answered from its own
    /// descriptors.
    pub fn usb_host_take_in(&mut self) -> alloc::vec::Vec<u8> {
        self.usb_otg.usb_host_take_in()
    }

    /// Deliver one peer CAN frame into the TWAI RX buffer (virtual second
    /// node; acceptance-filtered, RRB-released like a bus reception).
    pub fn twai_inject_rx(&mut self, frame: [u8; crate::twai::FRAME_LEN]) {
        self.twai.inject_rx(frame);
    }

    /// Push one received byte into the USB-Serial-JTAG RX FIFO (host console input).
    pub fn usb_inject_rx(&mut self, byte: u8) {
        self.usb.inject_rx(byte);
    }

    /// Bytes written to the USB-Serial-JTAG TX FIFO (ROM console) since the
    /// last call.
    pub fn take_usb_serial_tx(&mut self) -> Vec<u8> {
        self.usb.take_tx()
    }

    /// Append host-generated console bytes to UART `n`'s TX stream (the
    /// ROM `ets_printf` mailbox path).
    pub fn uart_push_tx(&mut self, n: usize, bytes: &[u8]) {
        self.uarts[n].push_tx(bytes);
    }

    /// Inject an analog voltage (mV) on an ADC unit/channel (host
    /// frontend — drives what the firmware reads from the SAR ADC).
    pub fn adc_inject_voltage(&mut self, unit: usize, channel: usize, milli_volts: u32) {
        self.adc.inject_voltage(unit, channel, milli_volts);
    }

    /// Reseed the hardware-RNG LCG (host frontend).
    pub fn rng_reseed(&mut self, seed: u32) {
        self.rng.reseed(seed);
    }

    /// Find a task TCB by its first 8 name bytes (host frontend for WiFi
    /// scan completion: locates `sys_evt`, the esp_event loop task, whose
    /// event-wait queue IS the default loop's queue).
    pub fn find_task_by_name(&mut self, name8: &[u8; 8]) -> Option<u32> {
        use xtensa_core::Bus as _Bus;
        let mut addr = 0x3FC9_0000u32;
        while addr < 0x3FCF_0000 {
            let mut nb = [0u8; 8];
            for (k, b) in nb.iter_mut().enumerate() {
                *b = self.read8(addr + 52 + k as u32) as u8;
            }
            if &nb == name8 {
                // Sanity: prio sane + state/event containers in DRAM.
                let prio = self.read32(addr + 44);
                let ev_c = self.read32(addr + 40);
                let st_c = self.read32(addr + 20);
                let in_dram = |x: u32| (0x3FC8_0000..0x3FD0_0000).contains(&x) || x == 0;
                if prio <= 25 && in_dram(ev_c) && in_dram(st_c) {
                    return Some(addr);
                }
                // else: name matched but failed sanity — keep looking
            }
            addr += 4;
        }
        None
    }

    /// Post a raw queue item to a FreeRTOS queue with REAL
    /// `xQueueGenericSend` head semantics (host frontend for the WiFi
    /// SCAN_DONE esp_event post): memcpy `item` (itemsize bytes) to
    /// pcWriteTo with wraparound at pcTail, advance pcWriteTo, bump
    /// uxMessagesWaiting. The waiter wakeup is the HOST's follow-up (the
    /// `sys_evt` task is unblocked via `queue_unblock_receiver`, then
    /// readied — the tick ISR switches to it within a tick).
    ///
    /// Queue_t layout for this build (§8b): +0 pcHead, +4 pcWriteTo,
    /// +8 pcTail, +12 pcReadFrom, +16 send-list (20B), +36 recv-list (20B),
    /// +56 mw, +60 len, +64 itemsize.
    pub fn queue_post_raw(&mut self, queue: u32, item: &[u8]) -> bool {
        use xtensa_core::Bus as _Bus;
        let len = self.read32(queue + 60);
        let isz = self.read32(queue + 64);
        let mw = self.read32(queue + 56);
        if mw >= len || isz == 0 || item.len() != isz as usize {
            return false;
        }
        let head = self.read32(queue);
        let tail = self.read32(queue + 8);
        if head == 0 || tail == 0 {
            return false;
        }
        let mut wr = self.read32(queue + 4);
        for b in item.iter() {
            self.write8(wr, *b as u32);
            wr += 1;
            if wr >= tail {
                wr = head;
            }
        }
        self.write32(queue + 4, wr);
        self.write32(queue + 56, mw + 1);
        true
    }

    /// Unblock a task parked in `xQueueReceive` on `queue` with REAL
    /// `xTaskRemoveFromEventList` semantics (host frontend for the WiFi
    /// SCAN_DONE esp_event delivery): removes the head waiter (highest
    /// priority — the list is priority-ordered) from the receive event
    /// list, stamps nothing (plain queue, no event-item value), unlinks it
    /// from its state list, and returns the woken TCB. The host readies it
    /// via `ready_task_on_list` next (same step, same borrow).
    ///
    /// Returns `None` when the receive list is empty.
    ///
    /// NOTE (proven live 2026-09-20): waking the SCAN_DONE *event-group
    /// waiter* (loopTask) directly is NOT enough — the Arduino `_scanDone` record path (get_ap_num/records + calloc into
    /// `_scanResult`) only runs from the esp_event callback
    /// (`_eventCallback` ← arduino_events task ← `sys_evt` ← default-loop
    /// queue). The host must post the SCAN_DONE esp_event
    /// (`wifi_scan_post_event`) and unblock `sys_evt`; the real chain then
    /// sets the DONE bit itself via `setStatusBits`.
    /// True when `queue` has a parked receive waiter (non-empty receive
    /// event list). Side-effect-free probe so host retries never mutate
    /// state (posting into a waiter-less queue would bump `mw` with nobody
    /// to consume it, wedging the queue at `len` — proven live by the STA
    /// pending storm, where every-step retries filled `sys_evt` to 32/32).
    pub fn queue_recv_waiting(&mut self, queue: u32) -> bool {
        use xtensa_core::Bus as _Bus;
        let list = queue + 36;
        let end = list + 8;
        let item = self.read32(end + 4);
        item != end && item != 0
    }

    /// sys_evt's event-wait queue handle (0 while undiscovered): the
    /// default esp_event loop queue `sys_evt` blocks on (`evC - 36`).
    /// Side-effect-free probe for TEMP-DIAG logging (never mutates).
    pub fn sys_evt_queue(&mut self) -> u32 {
        use xtensa_core::Bus as _Bus;
        let Some(sys) = self.find_task_by_name(b"sys_evt\0") else {
            return 0;
        };
        let ev_c = self.read32(sys + 40);
        if ev_c == 0 {
            return 0;
        }
        ev_c.wrapping_sub(36)
    }

    pub fn queue_unblock_receiver(&mut self, queue: u32) -> Option<u32> {
        use xtensa_core::Bus as _Bus;
        let list = queue + 36;
        let end = list + 8;
        let item = self.read32(end + 4);
        if item == end || item == 0 {
            return None;
        }
        let next = self.read32(item + 4);
        let owner = self.read32(item + 12);
        // Unlink from the event list.
        let iprev = self.read32(item + 8);
        if iprev != 0 {
            self.write32(iprev + 4, next);
        }
        self.write32(next + 8, iprev);
        let n = self.read32(list).wrapping_sub(1);
        self.write32(list, n);
        self.write32(item + 16, 0);
        // Unlink from the delayed/suspended state list.
        let st_item = owner + 4;
        let sc = self.read32(st_item + 16);
        if sc != 0 {
            let sprev = self.read32(st_item + 8);
            let snext = self.read32(st_item + 4);
            if sprev != 0 {
                self.write32(sprev + 4, snext);
            }
            self.write32(snext + 8, sprev);
            let sn = self.read32(sc).wrapping_sub(1);
            self.write32(sc, sn);
            self.write32(st_item + 16, 0);
        }
        Some(owner)
    }

    /// Host-side TLSF carve: allocate `user_bytes` (already 4-aligned,
    /// INCLUDING the IDF 4-byte block-owner prefix) from the main DRAM
    /// heap and return the user pointer (what IDF hands out, i.e. owner
    /// prefix + 4). Caller writes the record image at `ptr`, with the
    /// owner word before it.
    ///
    /// Ground truth: `heap_caps_base.c` (owner prefix + `heap->caps`
    /// match + `aligned_or_unaligned_alloc`), `multi_heap.c`
    /// (`tlsf_create_with_pool(start+sizeof(heap_t))`, `block_to_ptr =
    /// block+8`, `block_next = ptr+size-4`), `tlsf_block_functions.h`
    /// (free bit 0, `block_header_overhead = 4`, `block_size_min = 12`),
    /// `tlsf_control_functions.h` (`block_can_split = size >= 16+size`,
    /// `mapping_insert` small/large, `search_suitable_block`,
    /// `remove_free_block`, `block_split`, `block_trim_free`,
    /// `block_mark_as_used`). Live image facts (wifi-scan ELF, sdkconfig
    /// `HEAP_TASK_TRACKING=n`): `heap_t` = caps[3]+start+end+mux(8B)+
    /// handle+next (36B); DRAM heap `handle=0x3fca0398`, pool
    /// `handle+20`; owner prefix = current task handle (`pxCurrentTCBs`,
    /// else 0).
    ///
    /// The carve mirrors `tlsf_malloc` exactly (index search, unlink,
    /// split-if-fits with the remainder re-inserted, mark-used) so the
    /// firmware's later `free()` walks a consistent pool. Best-fit would
    /// also work; first-fit is what the model does and is equally valid
    /// TLSF behavior. `free_bytes` accounting is NOT updated (the
    /// firmware never reads it on this path; `multi_heap_get_info` is
    /// only diagnostics).
    ///
    /// Returns 0 when no free block fits (caller falls back to empty-air
    /// completion — same as silicon's allocation failure, which reports
    /// `found 0` via the calloc guard in `_scanDone`).
    pub fn wifi_heap_carve(&mut self, user_bytes: u32) -> u32 {
        // Local bus shims: read/write through the Soc Bus impl without
        // holding `&mut self` across the call (the Bus methods take
        // `&mut self`, so `self.read32(x)` inside `self.write32(a, ...)`
        // args double-borrows; these free functions take the reborrow).
        fn r32(s: &mut Soc, a: u32) -> u32 {
            use xtensa_core::Bus as _Bus;
            s.read32(a)
        }
        fn w32(s: &mut Soc, a: u32, v: u32) {
            use xtensa_core::Bus as _Bus;
            s.write32(a, v);
        }
        // 1. Find the DRAM heap: first registered heap whose caps carry
        // DEFAULT (1<<12 = 0x1000, `heap_caps_match` for MALLOC_CAP_DEFAULT
        // — what `malloc`/`calloc`/`new` use) AND 8BIT (1<<2 — byte
        // accesses; excludes the RTC-exec pool at caps0=0x8000). The scan
        // image's main pool reads caps0=0x10580f (DEFAULT|INTERNAL|
        // 32BIT|8BIT|DMA|...). (An earlier attempt matched INTERNAL
        // (0x800) at the wrong struct offset — `heap_t` starts with
        // caps[3] (NO_PRIOS=3, `heap_private.h`), NOT start/end; the dump
        // showed the `start` word where caps[0] was expected. Match caps
        // the way the IDF allocator does.)
        let Some(reg_heaps) = self.wifi_reg_heaps else {
            return 0;
        };
        let mut heap_t = r32(self, reg_heaps);
        let mut handle = 0u32;
        let mut guard = 0u32;
        while heap_t != 0 && guard < 8 {
            guard += 1;
            let caps0 = r32(self, heap_t);
            if caps0 & 0x1004 == 0x1004 {
                handle = r32(self, heap_t + 28);
                break;
            }
            heap_t = r32(self, heap_t + 32);
        }
        if handle == 0 {
            return 0;
        }
        // 2. TLSF control at handle+20 (after multi_heap's heap_t header).
        let tlsf = handle + 20;
        // Recompute control->size like control_construct: sizeof(control_t)
        // = block_null(16) + bitfield word(4) + size(4) + fl_bitmap(4) +
        // sl_bitmap ptr(4) + blocks ptr(4) = 36; then sl_bitmap (4*fl_count)
        // and blocks (4*fl_count*sl_count) follow, 4-aligned.
        // control fields needed for mapping: re-read packed word.
        // Bitfield layout (`tlsf_control_functions.h struct control_t`,
        // LSB-first): fl_index_count:5 [4:0], fl_index_shift:3 [7:5],
        // fl_index_max:6 [13:8], sl_index_count:6 [19:14],
        // sl_index_count_log2:3 [22:20], small_block_size:8 [30:23].
        // (Two earlier revisions mis-decoded this — first sl2=0/small=32
        // from bits [22:20]/[7:5]-as-value, then sl2=3/small=32 from the
        // right fields but the wrong small: small_block_size is its OWN
        // 8-bit field [30:23], NOT 1<<fls. The probe `packed=0x10320eaa`
        // decodes to fls=5/sl2=3/small=32 ONLY by the 1<<fls coincidence
        // (real small=0x20=32 here — same value, but the field is
        // authoritative). Ground truth is the header above, verified by
        // decoding the live word.)
        let packed = r32(self, tlsf + 16);
        let _flc = packed & 0x1F;
        let fls = (packed >> 5) & 0x7;
        let slc = (packed >> 14) & 0x3F;
        let mut sl2 = 0u32;
        while (1u32 << sl2) < slc {
            sl2 += 1;
        }
        let small_block = (packed >> 23) & 0xFF;
        // fl_bitmap @ tlsf+24, sl_bitmap ptr @ tlsf+28, blocks ptr @ tlsf+32.
        let fl_bitmap = r32(self, tlsf + 24);
        let sl_base = r32(self, tlsf + 28);
        let bl_base = r32(self, tlsf + 32);
        // 3. Adjust request like adjust_request_size (align 4, min 12).
        let mut need = (user_bytes + 3) & !3;
        if need < 12 {
            need = 12;
        }
        // mapping_search: round up for large sizes.
        let mut size = need;
        if size >= small_block {
            // fls_of(size): position of highest set bit (0-based).
            let fl = 31 - size.leading_zeros();
            let round = 1u32 << (fl - sl2);
            size = (size + round - 1) & !(round - 1);
        }
        // mapping_insert(size) -> (fl, sl). Ground truth
        // (tlsf_control_functions.h mapping_insert): large sizes use
        // fl = tlsf_fls(size) = 31-clz, sl = (size >> (fl-sl_log2)) ^
        // (1<<sl_log2), then fl -= (fl_index_shift - 1) (the -1 is real:
        // fls=7 pools place a 0x210 block at fl=4, proven by the live
        // free-list audit — without the -1 the request lands at fl=5 and
        // misses it).
        let (fl, sl) = if size < small_block {
            (0, size / (small_block / slc))
        } else {
            let f = 31 - size.leading_zeros();
            let s = (size >> (f - sl2)) ^ (1u32 << sl2);
            (f - fls + 1, s)
        };
        // search_suitable_block: sl_map masked from sl upward, then higher fl.
        // NOTE: fl here counts from fl_index_shift; bitmap bit k = fl k.
        // control->fl_index_count limits the search. The found block's
        // (fl,sl) come from the SEARCH (sfl,ssl), NOT the request (fl,sl) —
        // remove_free_block must unlink from the list the block was found
        // on (else the request list's head is corrupted and the next
        // malloc trips block_is_free on a used block).
        // search_suitable_block (tlsf_control_functions.h): masked sl in
        // the request fl; else fl_map masked above fl, ffs for the next fl,
        // then ffs over the FULL sl_map (no sl mask — the size check below
        // guards fit). Indices are raw bitmap positions (no flc masking —
        // the bitmaps only ever carry valid lists).
        let mut sfl = fl;
        // First: masked sl in the request fl.
        let mut sl_map = r32(self, sl_base + sfl * 4) & (!0u32 << sl);
        if sl_map == 0 {
            // Next-largest fl with any bit (fl_map masked above fl).
            let fmap = fl_bitmap & (!0u32 << (fl + 1));
            if fmap == 0 {
                return 0;
            }
            sfl = fmap.trailing_zeros();
            sl_map = r32(self, sl_base + sfl * 4);
        }
        if sl_map == 0 {
            return 0;
        }
        let bsl = sl_map.trailing_zeros();
        let block = r32(self, bl_base + (sfl * slc + bsl) * 4);
        let bfl = sfl;
        if block == 0 {
            return 0;
        }
        // Sanity: block must be free and big enough.
        let bsize_word = r32(self, block + 4);
        let bsize = bsize_word & !3;
        if bsize_word & 1 == 0 || bsize < size {
            return 0;
        }
        // remove_free_block(control, block, bfl, bsl).
        let prev = r32(self, block + 8);
        let next = r32(self, block + 12);
        w32(self, next + 8, prev);
        w32(self, prev + 12, next);
        if r32(self, bl_base + (bfl * slc + bsl) * 4) == block {
            w32(self, bl_base + (bfl * slc + bsl) * 4, next);
        }
        // block_trim_free: split remainder back into the pool.
        // block_can_split = bsize >= sizeof(block_header_t)+size = 20+size.
        if bsize >= 20 + size {
            // remaining = offset_to_block(block_to_ptr(block), size-4)
            //          = (block+8) + (size-4) = block+4+size.
            let rem = block + 4 + size;
            let remain_size = bsize - (size + 4);
            // block_set_size(block, size): keep flags.
            w32(self, block + 4, size | (bsize_word & 3));
            // block_split's tail = block_mark_as_free(remaining):
            // link_next(remaining) [rem.next.prev_phys = rem] +
            // rem.next.prev_free=1 + rem.free=1. (block_set_size does NOT
            // touch links/flags — the split helper does. All three are
            // needed or the pool chain + free lists corrupt.)
            let rem_next = rem + 4 + remain_size; // block_next(rem)
            w32(self, rem_next, rem);
            let rnw = r32(self, rem_next + 4);
            w32(self, rem_next + 4, rnw | 2);
            // block_set_size(remaining, remain_size) + free bit.
            w32(self, rem + 4, remain_size | 1);
            // block_link_next(block): next->prev_phys = block.
            let nxt = block + 4 + bsize; // block_next(block) with OLD size = block+4+bsize
            w32(self, nxt, block);
            // block_set_prev_free(remaining) [prev block is used -> no-op
            // value-wise: keep the free bit from above, ensure prev_free
            // CLEAR since our used block precedes it].
            w32(self, rem + 4, remain_size | 1);
            // block_insert(remaining): same mapping_insert with the
            // fl_index_shift - 1 correction (see above).
            let (rfl, rsl) = if remain_size < small_block {
                (0, remain_size / (small_block / slc))
            } else {
                let f = 31 - remain_size.leading_zeros();
                let s = (remain_size >> (f - sl2)) ^ (1u32 << sl2);
                (f - fls + 1, s)
            };
            let cur = r32(self, bl_base + (rfl * slc + rsl) * 4);
            w32(self, rem + 12, cur);
            w32(self, rem + 8, tlsf);
            w32(self, cur + 8, rem);
            w32(self, bl_base + (rfl * slc + rsl) * 4, rem);
            let slv = r32(self, sl_base + rfl * 4);
            w32(self, sl_base + rfl * 4, slv | (1u32 << rsl));
            w32(self, tlsf + 24, fl_bitmap | (1u32 << rfl));
        }
        // block_mark_as_used(block): next->prev_used; block used bit.
        // NOTE: block_next MUST use the block's CURRENT size (post-trim),
        // not the stale pre-split size — with the stale size the mark lands
        // mid-pool and the recorded next block's prev_free bit is never
        // cleared, so the firmware's later free() merges into our used
        // block and trips block_is_free (proven live: postEvent's `new`
        // aborted in block_trim_free).
        let bsz_now = r32(self, block + 4) & !3;
        let used_next = block + 4 + bsz_now; // block_next(block) = block+4+size
        let unw = r32(self, used_next + 4);
        w32(self, used_next + 4, unw & !2);
        let bw = r32(self, block + 4);
        w32(self, block + 4, (bw & !3) & !1);
        // block_to_ptr(block) = block+8 = user pointer (owner prefix first).
        let ptr = block + 8;
        // Owner prefix = current task handle (pxCurrentTCBs[0] —
        // discovered live via the `sys_evt` TCB scan: TCBs live in DRAM and
        // carry their stack pointer at +48, so any TCB-shaped hit yields the
        // core-0 current TCB cell), else 0.
        let owner = match self.wifi_pxcur {
            Some(pxcur) => r32(self, pxcur),
            None => 0,
        };
        w32(
            self,
            ptr,
            if (0x3FC8_0000..0x3FD0_0000).contains(&owner) {
                owner
            } else {
                0
            },
        );
        ptr + 4
    }

    /// Snapshot the PSRAM backing (CPU/system resets retain it).
    pub fn psram_snapshot(&self) -> alloc::boxed::Box<[u8]> {
        self.psram.clone()
    }

    /// Restore PSRAM across a non-deep reset (see `psram_wipe`).
    pub fn restore_psram(&mut self, snap: alloc::boxed::Box<[u8]>) {
        if snap.len() == self.psram.len() {
            self.psram.copy_from_slice(&snap);
        }
    }

    /// Wipe PSRAM to erased zeros (deep-sleep power-down loss).
    pub fn psram_wipe(&mut self) {
        self.psram.fill(0);
    }

    /// Inject the TSENS DAC code (host frontend for temperatureRead).
    pub fn tsens_inject(&mut self, raw: u8) {
        self.adc.tsens_inject(raw);
    }

    /// Inject a touch counter value on pad 1..=14 (host frontend — drives
    /// what the firmware reads from the touch STATUS registers).
    pub fn touch_inject(&mut self, pad: usize, value: u32) {
        self.touch.inject(pad, value);
    }

    /// Inject the BOD low-voltage condition (host frontend — with the
    /// detector enabled the interrupt fires after int_wait and, with
    /// rst_ena, a chip reset after rst_wait more ticks).
    pub fn bod_inject(&mut self, low: bool) {
        self.rtc.bod_inject(low);
    }

    /// Arm the WiFi scan dwell (host frontend — the harness calls this when
    /// the firmware enters `esp_wifi_scan_start`; see `wifi.rs`).
    pub fn wifi_scan_begin(&mut self) {
        self.wifi.wifi_scan_begin();
    }

    /// Read the STA `_esp_netif` pointer live (host frontend for the GOT_IP
    /// fixture): read the STAClass instance from the `_ZL15_sta_network_if`
    /// static (address programmed per-image via `wifi_layout_set`; nm:
    /// 0x3fc9ae6c wifi-sta, 0x3fc9ae7c wifi-scan) and return its
    /// `NetworkInterface::_esp_netif` field (vtable @0, `_esp_netif` @4 —
    /// `NetworkInterface.h`, arduino-lib 3.3.10). The sketch's own
    /// `localIP()` reads the ip_info through this same pointer, so host and
    /// firmware agree by construction. Returns 0 while undiscovered/null
    /// (host retries — level, not edge). (An earlier DRAM-scan revision
    /// matched vtable-in-flash + `_interface_id == 0` at +24 and returned a
    /// garbage pointer into the arduino-event queue area — proven live by
    /// the GOT_IP storm + dropped waiter.)
    pub fn wifi_sta_netif(&mut self) -> u32 {
        use xtensa_core::Bus as _Bus;
        let Some(sta_if) = self.wifi_network else {
            return 0;
        };
        let inst = self.read32(sta_if);
        if inst == 0 || !(0x3FC8_0000..0x3FD0_0000).contains(&inst) {
            return 0;
        }
        let netif = self.read32(inst + 4);
        if !(0x3FC8_0000..0x3FD0_0000).contains(&netif) {
            return 0;
        }
        netif
    }

    /// True when the arduino event queue is drained (mw==0), discovering
    /// it first if needed (side-effect-free probe for gating stage-2
    /// posts: GOT_IP must not be posted while the CONNECTED arduino event
    /// is still queued, or the consumer wedges — proven live by the
    /// GOT_IP `idf=false` storm while ard mw=1). Discovers lazily because
    /// the run_flash stage-1 path never calls `wifi_ard_queue_discover`
    /// itself (the firmware's own postEvent drives the arduino half).
    pub fn wifi_ard_idle(&mut self) -> bool {
        use xtensa_core::Bus as _Bus;
        let q = match self.wifi_ard_queue {
            Some(q) => q,
            None => {
                let q = self.wifi_ard_queue_discover();
                if q == 0 {
                    return false;
                }
                q
            }
        };
        self.read32(q + 56) == 0
    }

    /// Discover the arduino `_arduino_event_queue` + its parked waiter
    /// (host frontend — call once the dwell fires; the queue is stable once
    /// `Network.initEvents` runs): scans DRAM for the `len==32,
    /// itemsize==4` queue whose receive waiter is the `arduino_events`
    /// task, and caches both. Returns the queue handle (0 = not yet
    /// created — host retries, level not edge).
    ///
    /// Ground truth (`NetworkEvents.cpp`, arduino-lib 3.3.10):
    /// `_arduino_event_queue = xQueueCreate(32, sizeof(arduino_event_t *))`
    /// and the `arduino_events` task blocks in `xQueueReceive` on it with
    /// `portMAX_DELAY` (proven live: queue @0x3fcecd0c, waiter
    /// `arduino_events`, on the STA image).
    pub fn wifi_ard_queue_discover(&mut self) -> u32 {
        use xtensa_core::Bus as _Bus;
        if let Some(q) = self.wifi_ard_queue {
            return q;
        }
        let mut addr = 0x3FC9_0000u32;
        while addr < 0x3FCF_0000 {
            // Queue storage overlay: `pcHead`(+0)/`pcTail`(+8) hold
            // `memset`-painted garbage (0x0/0xa5a5a5a5) until the first wrap
            // copies real storage pointers over them — so `head`/`tail` are
            // UNRELIABLE as a discovery filter (they wrongly reject the real
            // queue). Match on `len==32` (+60) + the parked `arduino_`
            // waiter instead; the storage pointers are re-read live at post
            // time (see `wifi_ard_post`).
            if self.read32(addr + 60) == 32 && self.read32(addr + 64) == 4 {
                let head = self.read32(addr);
                let tail = self.read32(addr + 8);
                if head != 0 && tail != 0 && (0x3FC8_0000..0x3FD0_0000).contains(&head) {
                    let list = addr + 36;
                    let end = list + 8;
                    let item = self.read32(end + 4);
                    if item != end && item != 0 {
                        let owner = self.read32(item + 12);
                        let mut nb = [0u8; 8];
                        for (k, b) in nb.iter_mut().enumerate() {
                            *b = self.read8(owner + 52 + k as u32) as u8;
                        }
                        if &nb == b"arduino_" {
                            self.wifi_ard_queue = Some(addr);
                            self.wifi_ard_waiter = Some(owner);
                            return addr;
                        }
                    }
                }
            }
            addr += 4;
        }
        0
    }

    /// Post one `arduino_event_t` (event_id + 44-byte info union) into the
    /// arduino queue and wake + ready its waiter (host frontend — the
    /// arduino-side half of every WiFi/IP fixture post). Full 188-byte
    /// `arduino_event_t` (event_id u32 @0 + info union @4..188 —
    /// `postEvent` memsets/copies 188, ground truth above), posted as a
    /// POINTER into the queue ring (itemsize 4 — the queue holds
    /// `arduino_event_t *`). The real chain (`_checkForEvent` →
    /// `_cbEventList` dispatch → `_onStaArduinoEvent` / `_eventCallback` →
    /// status bits) runs unmodified. Returns false while the queue is
    /// undiscovered/full (host retries — level, not edge).
    ///
    /// POOL NOTE: the pointer targets a host slot in the dedicated
    /// `ard_pool` backing (NOT DRAM, NOT heap — outside every heap's bounds
    /// by construction, so `heap_caps_free` skips it; the machine
    /// intercepts the free as a no-op leak-by-design, see `wifi_ard_free`).
    /// A live `wifi_heap_carve` is unusable (pool exhausted at dwell time:
    /// 47/47 blocks used, 12 free bytes, proven by the physical pool walk);
    /// raw DRAM scratch aborts the consumer's unsized `delete` on garbage
    /// header words (proven live IN-PANIC at 0x4037bf00). 4 slots/run max.
    pub fn wifi_ard_post(
        &mut self,
        event_id: u32,
        info: &[u8; 44],
        list_base: u32,
        top_prio: u32,
    ) -> bool {
        use xtensa_core::Bus as _Bus;
        let queue = self.wifi_ard_queue_discover();
        if queue == 0 {
            return false;
        }
        // Side-effect-free gating FIRST (a failed post must not consume
        // anything, or every-step retries wedge the queue; the same
        // pending-storm class as the sys_evt queue fill).
        if !self.queue_recv_waiting(queue) {
            return false;
        }
        // Full 188-byte `arduino_event_t` (event_id u32 @0 + info union
        // @4..188 — `postEvent` memsets/copies 188, ground truth above).
        // Posted as a POINTER into the queue ring (itemsize 4 — the queue
        // holds `arduino_event_t *`, `xQueueCreate(32,
        // sizeof(arduino_event_t *))`, ground truth above).
        let len = self.read32(queue + 60);
        let isz = self.read32(queue + 64);
        let mw = self.read32(queue + 56);
        if mw >= len || isz != 4 {
            return false;
        }
        // Slot = host bump cursor (never reused within a run — each post
        // consumes a fresh slot, so a still-queued event's block can never
        // be stomped by a later post; 4 slots cover every fixture flow).
        // The 188-byte event is written PLAIN (no heap header — the block
        // is never freed: the machine intercepts `heap_caps_free` on host
        // slots, see `wifi_ard_free`, as a no-op leak-by-design).
        if self.wifi_ard_slot >= Self::WIFI_ARD_SLOTS {
            return false;
        }
        let slot = self.wifi_ard_slot;
        self.wifi_ard_slot += 1;
        let ev = Self::WIFI_ARD_POOL + slot * Self::WIFI_ARD_SLOT;
        self.write32(ev, event_id);
        for (k, b) in info.iter().enumerate() {
            self.write8(ev + 4 + k as u32, *b as u32);
        }
        // Queue holds POINTERS: post &ev with xQueueGenericSend semantics.
        let head = self.read32(queue);
        let tail = self.read32(queue + 8);
        if head == 0 || tail == 0 {
            return false;
        }
        let mut wr = self.read32(queue + 4);
        for k in 0..4 {
            self.write8(wr + k, (ev >> (8 * k)) & 0xFF);
        }
        wr += 4;
        if wr >= tail {
            wr = head;
        }
        self.write32(queue + 4, wr);
        self.write32(queue + 56, mw + 1);
        // Wake + ready the waiter (same unlink semantics as
        // `queue_unblock_receiver`, then ready-list insert).
        let woken = match self.queue_unblock_receiver(queue) {
            Some(t) => t,
            None => return false,
        };
        self.ready_task_on_list(woken, list_base, top_prio);
        true
    }

    /// Set the image-specific WiFi fixture addresses (host frontend — the
    /// harness discovers these per-image, e.g. via ELF symbols at build
    /// time, and sets them before arming the dwell; see the field docs).
    /// Any `None` leaves a previous value alone (so the harness can set a
    /// subset); the scan path fails softly while undiscovered.
    pub fn wifi_layout_set(
        &mut self,
        reg_heaps: Option<u32>,
        pxcur: Option<u32>,
        network: Option<u32>,
        event_var: Option<u32>,
        ip_event_var: Option<u32>,
    ) {
        if reg_heaps.is_some() {
            self.wifi_reg_heaps = reg_heaps;
        }
        if pxcur.is_some() {
            self.wifi_pxcur = pxcur;
        }
        if network.is_some() {
            self.wifi_network = network;
        }
        if event_var.is_some() {
            self.wifi_event_var = event_var;
        }
        if ip_event_var.is_some() {
            self.wifi_ip_event_var = ip_event_var;
        }
    }

    /// Stage the connected-AP record + interface IP info the pc-intercept
    /// hooks serve back (`wifi_hook_get_ap_info` / `wifi_hook_get_ip_info`
    /// below). Called by the host when it posts GOT_IP, from the same
    /// fixture AP + LAN the event payloads carry (single source of truth —
    /// the sketch's `SSID()`/`RSSI()`/`localIP()` then agree with the
    /// posted events by construction).
    pub fn wifi_stage_sta_data(&mut self, ap: &crate::wifi::ScanFixtureAp, ip: [u8; 4]) {
        // 92-byte `wifi_ap_record_t` (same layout as `wifi_scan_record_ap`
        // below): bssid@0, ssid@6 (+NUL), primary@39, rssi@44,
        // authmode@48 = WPA2_PSK, pairwise/group@52/56 = CCMP.
        let mut rec = [0u8; 92];
        rec[0..6].copy_from_slice(&ap.bssid);
        let n = (ap.ssid_len as usize).min(32);
        rec[6..6 + n].copy_from_slice(&ap.ssid[..n]);
        rec[6 + n] = 0;
        rec[39] = ap.chan;
        rec[44] = ap.rssi as u8;
        rec[48..52].copy_from_slice(&3u32.to_le_bytes());
        rec[52..56].copy_from_slice(&4u32.to_le_bytes());
        rec[56..60].copy_from_slice(&4u32.to_le_bytes());
        self.wifi_ap_record = Some(rec);
        let mut info = [0u8; 12];
        info[0..4].copy_from_slice(&ip);
        info[4..8].copy_from_slice(&[255, 255, 255, 0]);
        info[8..12].copy_from_slice(&[192, 168, 4, 1]);
        self.wifi_ip_info = Some(info);
    }

    /// pc-intercept hook for `esp_netif_get_ip_info` (0x4202e77c): the
    /// closed lwIP stack has no live netif state (no DHCP/client stack
    /// runs), so serve the staged fixture IP info directly. `out` is the
    /// caller's `esp_netif_ip_info_t*` (ip@0/mask@4/gw@8); writes 12 bytes
    /// and reports whether it fired (caller then skips the call).
    /// Returns false while unstaged (caller lets the call run — it fails
    /// soft like silicon with no lease, `localIP()` → 0.0.0.0).
    pub fn wifi_hook_get_ip_info(&mut self, out: u32) -> bool {
        use xtensa_core::Bus as _Bus;
        let Some(info) = self.wifi_ip_info else {
            return false;
        };
        for (k, b) in info.iter().enumerate() {
            self.write8(out + k as u32, *b as u32);
        }
        true
    }

    /// pc-intercept hook for `esp_wifi_sta_get_ap_info` (0x42064118):
    /// serve the staged 92-byte `wifi_ap_record_t` into the caller's
    /// buffer. Returns false while unstaged (caller lets the call run —
    /// fails soft like silicon with no association).
    pub fn wifi_hook_get_ap_info(&mut self, out: u32) -> bool {
        use xtensa_core::Bus as _Bus;
        let Some(rec) = self.wifi_ap_record else {
            return false;
        };
        for (k, b) in rec.iter().enumerate() {
            self.write8(out + k as u32, *b as u32);
        }
        // Count served reads: SSID() + RSSI() = 2 (the BSSID leg, if the
        // sketch called it, would be a 3rd). The run_flash disconnect leg
        // arms once both post-WL_CONNECTED reads consumed (proves the
        // sketch reached the post-RSSI `WiFi.disconnect()`).
        self.wifi_ap_reads += 1;
        true
    }

    /// True once the sketch consumed both post-WL_CONNECTED ap reads
    /// (SSID + RSSI) — the disconnect-leg arm gate (see run_flash).
    pub fn wifi_hook_rssi_done(&self) -> bool {
        self.wifi_ap_reads >= 2
    }

    /// pc-intercept hook for `esp_wifi_disconnect` (0x4203c7d0): the closed
    /// scan/connect machine has no live association to tear down, so report
    /// success immediately (caller then skips the call AND posts the
    /// disconnect events — see the run_flash disconnect leg).
    pub fn wifi_hook_disconnect(&mut self) -> bool {
        self.wifi_ap_record.is_some()
    }

    /// Take the disconnect-hook latch (true once per hook fire).
    pub fn wifi_take_disconnect(&mut self) -> bool {
        core::mem::replace(&mut self.wifi_disc_fired, false)
    }

    /// Record a disconnect-hook fire (machine calls this alongside the
    /// hook skip).
    pub fn wifi_notify_disconnect(&mut self) {
        self.wifi_disc_fired = true;
    }

    // ── Self-contained Wi-Fi fixture engine (browser/bridge path) ──
    // Mirrors the run_flash host blocks line-for-line (same Soc calls, same
    // order, same payloads), but driven from machine state instead of env
    // vars + uart_buf markers, so the wasm bridge (no env, no stdout) can
    // serve the identical fixtures. run_flash keeps its own copy (it needs
    // per-step println diagnostics); behavior parity is pinned by the
    // shared Soc primitives both paths call.
    //
    /// Layout table shared by the fixture engine (selected by
    /// `wifi_image_sta`, programmed once via `wifi_fixture_image`).
    fn wifi_fixture_layout(&self) -> WifiImageLayout {
        if self.wifi_image_sta {
            // wifi-sta image layout (nm on the wifi-sta ELF).
            WifiImageLayout {
                scan_start: 0x4206_3b78,
                connect: 0x4203_c7c4,
                wifi_event_var: 0x3c0b_42bc,
                ip_event_var: 0x3c0b_3bb8,
                count_cell: 0x3fc9_f926,
                scan_count: 0x3fc9_aee4,
                scan_result: 0x3fc9_aee0,
                records_check: 0x4200_3f9c,
                ready_lists: 0x3fc9_b8dc,
                top_prio: 0x3fc9_b84c,
                reg_heaps: 0x3fc9_b794,
                pxcur: 0x3fc9_bad0,
                sta_network_if: 0x3fc9_ae6c,
            }
        } else {
            // wifi-scan image layout (nm on the wifi-scan ELF).
            WifiImageLayout {
                scan_start: 0x4206_3b90,
                connect: 0x4203_c84c,
                wifi_event_var: 0x3c0b_4264,
                ip_event_var: 0x3c0b_3b60,
                count_cell: 0x3fc9_f93e,
                scan_count: 0x3fc9_aef4,
                scan_result: 0x3fc9_aef0,
                records_check: 0x4200_3ee0,
                ready_lists: 0x3fc9_b8f4,
                top_prio: 0x3fc9_b864,
                reg_heaps: 0x3fc9_b7ac,
                pxcur: 0x3fc9_b7e0,
                sta_network_if: 0x3fc9_ae7c,
            }
        }
    }
    /// Re-apply the active layout addresses after a boot/reset (which
    /// rebuilds heap-adjacent state). Called by the fixture arm wrappers
    /// so main.js load-then-arm order always wins over boot-time wipes.
    pub fn wifi_fixture_layout_reapply(&mut self) {
        let l = self.wifi_fixture_layout();
        self.wifi_reg_heaps = Some(l.reg_heaps);
        self.wifi_pxcur = Some(l.pxcur);
        self.wifi_network = Some(l.sta_network_if);
        self.wifi_event_var = Some(l.wifi_event_var);
        self.wifi_ip_event_var = Some(l.ip_event_var);
    }

    /// Select the STA vs scan image for the fixture engine (must precede
    /// arming; programs the layout addresses like run_flash does at boot).
    pub fn wifi_fixture_image(&mut self, is_sta: bool) {
        self.wifi_image_sta = is_sta;
        let l = self.wifi_fixture_layout();
        // Program the layout addresses (same five cells run_flash sets).
        self.wifi_reg_heaps = Some(l.reg_heaps);
        self.wifi_pxcur = Some(l.pxcur);
        self.wifi_network = Some(l.sta_network_if);
        self.wifi_event_var = Some(l.wifi_event_var);
        self.wifi_ip_event_var = Some(l.ip_event_var);
    }

    /// Arm the scan fixture: `aps_spec` is `WIFI_SCAN_APS`
    /// (`ssid,rssi,chan,bssid[;...]`, empty = empty air). Idempotent
    /// pre-boot setup (no stepping yet — the engine fires on firmware pcs
    /// like the run_flash blocks do).
    pub fn wifi_fixture_scan(&mut self, aps_spec: &str) {
        let aps = crate::wifi::parse_scan_fixtures(aps_spec);
        self.wifi_fixture = Some(WifiFixture {
            aps,
            ip: [192, 168, 4, 2],
            st: WifiFixtureState::default(),
        });
    }

    /// Arm the STA-connect fixture (same AP list drives the association;
    /// fixed LAN 192.168.4.2/24 gw .1, like run_flash).
    pub fn wifi_fixture_sta(&mut self, aps_spec: &str) {
        let mut aps = crate::wifi::parse_scan_fixtures(aps_spec);
        if aps.is_empty() {
            aps = crate::wifi::parse_scan_fixtures("EmuNet,-50,6,02:11:22:33:44:55");
        }
        self.wifi_fixture = Some(WifiFixture {
            aps,
            ip: [192, 168, 4, 2],
            st: WifiFixtureState::default(),
        });
    }

    /// Records-check pc of the active layout (0 while no fixture armed):
    /// lets the machine force the trapping core's a10 = ESP_OK when the
    /// records leg stages (the Soc cannot see CPU regs itself).
    pub fn wifi_fixture_records_pc(&self) -> u32 {
        if self.wifi_fixture.is_none() {
            return 0;
        }
        self.wifi_fixture_layout().records_check
    }

    /// True once the records leg staged (machine edge-detects the
    /// transition to force a10 on the trapping core).
    pub fn wifi_fixture_records_staged(&self) -> bool {
        self.wifi_fixture
            .as_ref()
            .is_some_and(|f| f.st.records_done)
    }

    /// Drive one engine step (call once per macro-step from the machine,
    /// with both cores' pcs sampled post-step like run_flash does after
    /// `step_fast`). Runs the armed scan and/or STA completion legs.
    /// No-op while no fixture is armed.
    pub fn wifi_fixture_poll(&mut self, pc0: u32, pc1: u32) {
        use xtensa_core::Bus as _Bus;
        // Snapshot the layout + AP data first (borrow-split: the legs below
        // need `&mut self` for queue/heap calls, so nothing here may hold
        // the fixture borrow across them).
        let Some(fx) = self.wifi_fixture.clone() else {
            return;
        };
        let l = self.wifi_fixture_layout();
        let mut st = fx.st;
        let aps = fx.aps.clone();
        let ip = fx.ip;
        let mut dirty = false;

        // — Scan leg: arm the dwell at `esp_wifi_scan_start`; on elapse,
        // stage the count cell + post the REAL SCAN_DONE esp_event. —
        if !st.scan_done {
            if !st.scan_armed && (pc0 == l.scan_start || pc1 == l.scan_start) {
                self.wifi_scan_begin();
                st.scan_armed = true;
                dirty = true;
            }
            if st.scan_armed && self.wifi_scan_tick_complete() {
                if !aps.is_empty() {
                    self.write16(l.count_cell, aps.len().min(8) as u32);
                }
                if let Some(tcb) = self.wifi_scan_post_event(l.wifi_event_var) {
                    self.ready_task_on_list(tcb, l.ready_lists, l.top_prio);
                }
                st.scan_done = true;
                dirty = true;
            }
        }
        // — Scan records leg: at the records-return check the calloc'd
        // buffer is final — write fixture records + force ESP_OK + count. —
        if st.scan_done && !st.records_done && !aps.is_empty() {
            for c in [pc0, pc1] {
                if c == l.records_check {
                    let buf = self.read32(l.scan_result);
                    if buf != 0 {
                        let n = aps.len().min(8);
                        for (k, ap) in aps.iter().take(n).enumerate() {
                            self.wifi_scan_record_ap(buf, k, ap);
                        }
                        self.write16(l.count_cell, n as u32);
                        self.write16(l.scan_count, n as u32);
                        st.records_done = true;
                        dirty = true;
                        break;
                    }
                }
            }
            // NOTE: run_flash also forces a10 = ESP_OK on the trapping core
            // via `cpu.set_reg`; the machine wrapper below does that (the
            // Soc cannot see CPU regs itself).
        }
        // — STA stage 1: arm the dwell at `esp_wifi_connect`; on elapse,
        // post STA_CONNECTED (IDF bus; the firmware translates it to the
        // arduino bus itself — a host arduino post races it, proven live).
        // NOTE: run_flash posts stage 1 on the SAME poll that arms (its
        // dwell check is `tick_complete() || wifi_sta_armed`, i.e. the
        // just-armed dwell counts as elapsed) — do the same here via the
        // `just_armed` flag, since a fresh `wifi_scan_begin` can never read
        // back complete in the same poll.
        if !st.sta_done {
            let mut just_armed = false;
            if !st.sta_armed && (pc0 == l.connect || pc1 == l.connect) {
                self.wifi_scan_begin();
                st.sta_armed = true;
                just_armed = true;
                dirty = true;
            }
            if st.sta_armed && st.sta_stage == 0 && (just_armed || self.wifi_scan_tick_complete()) {
                let ap = aps.first().copied().unwrap_or(
                    crate::wifi::parse_scan_fixture("EmuNet,-50,6,02:11:22:33:44:55")
                        .expect("default fixture parses"),
                );
                let mut payload = [0u8; 48];
                let n = (ap.ssid_len as usize).min(32);
                payload[..n].copy_from_slice(&ap.ssid[..n]);
                payload[32] = ap.ssid_len;
                payload[33..39].copy_from_slice(&ap.bssid);
                payload[39] = ap.chan;
                payload[40..44].copy_from_slice(&3u32.to_le_bytes());
                payload[44..46].copy_from_slice(&1u16.to_le_bytes());
                if let Some(tcb) = self.wifi_post_event_with_data(l.wifi_event_var, 4, &payload) {
                    self.ready_task_on_list(tcb, l.ready_lists, l.top_prio);
                    st.sta_stage = 1;
                    st.sta_done = true;
                    dirty = true;
                }
            }
        }
        // — STA stage 2 (GOT_IP, both buses back-to-back, no consume-gate;
        // see run_flash notes): IDF half is handler-veracity; the arduino
        // 115 half (full 20-byte `ip_event_got_ip_t` at the info head) is
        // what drives WL_CONNECTED. Decoupled: done latches on the arduino
        // half; a failed IDF half retries best-effort.
        if st.sta_stage == 1 && !st.sta_ip_done {
            let netif = self.wifi_sta_netif();
            if netif != 0 {
                let mask = [255u8, 255, 255, 0];
                let gw = [192u8, 168, 4, 1];
                let mut payload = [0u8; 20];
                payload[0..4].copy_from_slice(&netif.to_le_bytes());
                payload[4..8].copy_from_slice(&u32::from_le_bytes(ip).to_le_bytes());
                payload[8..12].copy_from_slice(&u32::from_le_bytes(mask).to_le_bytes());
                payload[12..16].copy_from_slice(&u32::from_le_bytes(gw).to_le_bytes());
                payload[16] = 1;
                let mut info = [0u8; 44];
                info[0..20].copy_from_slice(&payload);
                let idf_ok = match self.wifi_post_event_with_data(l.ip_event_var, 0, &payload) {
                    Some(tcb) => {
                        self.ready_task_on_list(tcb, l.ready_lists, l.top_prio);
                        true
                    }
                    None => false,
                };
                let ard_ok = self.wifi_ard_post(115, &info, l.ready_lists, l.top_prio);
                if ard_ok {
                    let ap = aps.first().copied().unwrap_or(
                        crate::wifi::parse_scan_fixture("EmuNet,-50,6,02:11:22:33:44:55")
                            .expect("default fixture parses"),
                    );
                    self.wifi_stage_sta_data(&ap, ip);
                    st.sta_ip_done = true;
                    dirty = true;
                }
                let _ = idf_ok;
            }
        }
        // — Disconnect leg arming is latch-driven (machine notifies on the
        // `esp_wifi_disconnect` hook skip); the engine only posts. Gate on
        // GOT_IP done + both ap reads consumed (post-RSSI position) +
        // arduino idle (the two arduino events must not race).
        if st.sta_ip_done && !st.disc_done {
            if self.wifi_hook_rssi_done() && self.wifi_take_disconnect() {
                st.disc_armed = true;
                dirty = true;
            } else {
                let _ = self.wifi_take_disconnect();
            }
            // NOTE: pre-leg latch fires (setup-time disconnect) are drained
            // above so they can't arm the leg late — same as run_flash.
            if st.disc_armed && self.wifi_ard_idle() {
                let ap = aps.first().copied().unwrap_or(
                    crate::wifi::parse_scan_fixture("EmuNet,-50,6,02:11:22:33:44:55")
                        .expect("default fixture parses"),
                );
                let n = (ap.ssid_len as usize).min(32);
                let mut full = [0u8; 48];
                full[..n].copy_from_slice(&ap.ssid[..n]);
                full[32] = ap.ssid_len;
                full[33..39].copy_from_slice(&ap.bssid);
                full[39] = 8; // WIFI_REASON_ASSOC_LEAVE (voluntary)
                let didf_ok = match self.wifi_post_event_with_data(l.wifi_event_var, 5, &full) {
                    Some(tcb) => {
                        self.ready_task_on_list(tcb, l.ready_lists, l.top_prio);
                        true
                    }
                    None => false,
                };
                let mut dinfo = [0u8; 44];
                dinfo[..n].copy_from_slice(&ap.ssid[..n]);
                dinfo[32] = ap.ssid_len;
                dinfo[33..39].copy_from_slice(&ap.bssid);
                dinfo[39] = 8;
                let dard_ok = self.wifi_ard_post(113, &dinfo, l.ready_lists, l.top_prio);
                if didf_ok && dard_ok {
                    st.disc_done = true;
                    dirty = true;
                }
            }
        }
        if dirty && let Some(slot) = self.wifi_fixture.as_mut() {
            slot.st = st;
        }
    }

    /// Poll the scan-dwell completion (host frontend — `true` exactly once
    /// per armed scan, when the dwell budget elapses; see `wifi.rs`).
    pub fn wifi_scan_tick_complete(&mut self) -> bool {
        self.wifi.scan_tick_complete()
    }

    /// Complete a WiFi scan host-side, exactly as the closed scan state
    /// machine would on beacon/probe-response reception — but at the
    /// firmware boundary, with every post-boundary instruction unmodified.
    ///
    /// What this does (all verifiable in IDF/Arduino source, no invented
    /// ABI — field offsets from `queue.c`/`event_groups.c`/`list.h`, §8b
    /// TCB layout, Arduino `NetworkEvents` source):
    /// 1. Discovers the Arduino `NetworkEvents` event group live: the
    ///    `Network` object at `NETWORK_OBJ` holds the group handle at +4
    ///    (objdump-verified: `setStatusBits`/`waitStatusBits` load
    ///    `[Network+4]` then call `xEventGroupSetBits/WaitBits`).
    /// 2. Applies REAL `xEventGroupSetBits(group, WIFI_SCAN_DONE_BIT)`
    ///    semantics by hand (the host cannot call into firmware):
    ///    `uxEventBits |= DONE`, then walks the unordered waiter list and
    ///    unblocks every waiter whose ANY-bit matches (the waiter asked
    ///    any-bit, see `waitStatusBits`), via REAL
    ///    `vTaskRemoveFromUnorderedEventList` semantics: stamp the task's
    ///    event item `uxEventBits | UNBLOCKED_DUE_TO_BIT_SET`, unlink both
    ///    list items (readying is the host's follow-up via
    ///    `ready_task_on_list`).  A woken waiter needs no immediate yield —
    ///    the tick ISR performs the switch within a tick.
    /// 3. Leaves the IDF ap store untouched (count cell stays 0 = empty
    ///    air; fixture counts are staged separately by the host before the
    ///    post — see `run_flash`).
    ///
    /// Returns the woken task (`Some(tcb)`) when a waiter was unblocked, or
    /// `None` when the group handle is not yet valid (host retries next
    /// step — the completion is level, not edge) or no waiter matched.  On
    /// `Some` the host must call `ready_task_on_list(tcb, ...)` next (same
    /// step, same borrow).
    ///
    /// SCOPE NOTE (proven live 2026-09-20): this wakes the SCAN_DONE
    /// *event-group waiter* (loopTask) directly, which is enough for the
    /// empty-air `found 0` completion — but it BYPASSES the Arduino
    /// `_scanDone` record path (get_ap_num/records + calloc), which only
    /// runs from the esp_event callback. For fixture records (N > 0) the
    /// host must use `wifi_scan_post_event` instead (real SCAN_DONE post
    /// through `sys_evt`); the real chain then sets the DONE bit itself.
    /// (Kept for probe/empty-air use; `run_flash` uses the post path.)
    ///
    /// Event-group (EventGroup_t) layout for this build (TRACE=y,
    /// STATIC+DYNAMIC=y, 32-bit ticks): +0 `uxEventBits`, +4
    /// `xTasksWaitingForBits` List (20B: n, idx, end-val, end-next,
    /// end-prev), +24 `uxEventGroupNumber`, +28 `ucStaticallyAllocated`.
    /// ListItem (20B, no integrity bytes): +0 value, +4 next, +8 prev,
    /// +12 owner, +16 container.
    pub fn wifi_scan_complete_empty(&mut self, network_obj: u32) -> Option<u32> {
        use xtensa_core::Bus as _Bus;
        const DONE_BIT: u32 = 1 << 1; // Arduino WIFI_SCAN_DONE_BIT = BIT1
        const UNBLOCKED_DUE_TO_BIT_SET: u32 = 0x0200_0000; // event_groups.h (32-bit)
        const EVENT_IN_USE: u32 = 0x8000_0000; // taskEVENT_LIST_ITEM_VALUE_IN_USE (32-bit)
        let group = self.read32(network_obj + 4);
        if !(0x3FC8_0000..0x3FD0_0000).contains(&group) {
            return None;
        }
        // 1. Set the bits.
        let bits = self.read32(group) | DONE_BIT;
        self.write32(group, bits);
        // 2. Walk the unordered waiter list (head = end.next at +12).
        let list = group + 4;
        let end = list + 8;
        let mut item = self.read32(end + 4);
        let mut guard = 0u32;
        let mut woken: Option<u32> = None;
        while item != end && item != 0 && guard < 32 {
            guard += 1;
            let next = self.read32(item + 4);
            let waited = self.read32(item);
            // Waiters from xEventGroupWaitBits ask ANY-bit (see
            // NetworkEvents::waitStatusBits: xClearOnExit=false,
            // xWaitForAllBits=false).
            if waited & bits != 0 {
                let owner = self.read32(item + 12);
                // Stamp the event item (value + IN_USE, like the kernel).
                self.write32(item, bits | UNBLOCKED_DUE_TO_BIT_SET | EVENT_IN_USE);
                // Unlink from the event list.
                let iprev = self.read32(item + 8);
                if iprev != 0 {
                    self.write32(iprev + 4, next);
                }
                self.write32(next + 8, iprev);
                let n = self.read32(list).wrapping_sub(1);
                self.write32(list, n);
                self.write32(item + 16, 0);
                // Unlink from the delayed/suspended state list + ready.
                let st_item = owner + 4;
                let sc = self.read32(st_item + 16);
                if sc != 0 {
                    let sprev = self.read32(st_item + 8);
                    let snext = self.read32(st_item + 4);
                    if sprev != 0 {
                        self.write32(sprev + 4, snext);
                    }
                    self.write32(snext + 8, sprev);
                    let sn = self.read32(sc).wrapping_sub(1);
                    self.write32(sc, sn);
                    self.write32(st_item + 16, 0);
                }
                // NOTE: readying (list insert) is the HOST's job — it owns
                // the ready-list base + top-priority cell (see
                // `ready_task_on_list`); this fn only unblocks.
                woken = Some(owner);
            }
            item = next;
        }
        woken
    }

    /// Post a 16-byte event item into the default esp_event loop queue
    /// (host frontend — shared by all WiFi/IP fixture completions): finds
    /// `sys_evt`, appends `{allocated=0, set=0, base, id, val=0}` with REAL
    /// `xQueueGenericSend` head semantics, and unblocks the waiter. The
    /// real chain then runs unmodified (`esp_event_loop_run` →
    /// `handler_execute` → arduino `_eventCallback` → status bits).
    ///
    /// `base_var` is the address of the event-base POINTER VARIABLE (its
    /// CONTENT is the compared base — e.g. `D WIFI_EVENT` / `D IP_EVENT`;
    /// proven live: `[0x3C0B4264]=0x3C0AAF7D`). `allocated=0` means the
    /// task frees nothing (a host scratch pointer would corrupt the heap).
    ///
    /// Returns the woken `sys_evt` TCB on success; `None` when `sys_evt`
    /// is not found/parked, the queue is full, or the item does not fit
    /// (host retries next step — the completion is level, not edge).
    pub fn wifi_post_event(&mut self, base_var: u32, id: u32) -> Option<u32> {
        use xtensa_core::Bus as _Bus;
        let sys = self.find_task_by_name(b"sys_evt\0")?;
        let ev_c = self.read32(sys + 40);
        if ev_c == 0 {
            return None;
        }
        let queue = ev_c.wrapping_sub(36);
        // Sanity: itemsize must be 16 (esp_event_post_instance_t with
        // POST_FROM_ISR flags) and the queue must have room.
        if self.read32(queue + 64) != 16 {
            return None;
        }
        let base = self.read32(base_var);
        let mut item = [0u8; 16];
        item[4..8].copy_from_slice(&base.to_le_bytes());
        item[8..12].copy_from_slice(&id.to_le_bytes());
        // Waiter gating IS required: posting while sys_evt is momentarily
        // unparked (between loop iterations) bumps mw with nobody consuming
        // — every-step retries then wedge the queue at len 32/32 and the
        // TCB is never found again (proven live: GOT_IP storm filled
        // sys_evt to 32/32, sys_evt unfindable, WL_CONNECTED never
        // arrives). Gate on the parked waiter; retry next step otherwise
        // (level, not edge). The waiter re-parks one step later and the
        // post lands then.
        if !self.queue_recv_waiting(queue) {
            return None;
        }
        if !self.queue_post_raw(queue, &item) {
            return None;
        }
        self.queue_unblock_receiver(queue)
    }

    /// Post the WiFi SCAN_DONE esp_event into the default event-loop queue,
    /// exactly as the closed `wifi_event_post(WIFI_EVENT, SCAN_DONE, ...)`
    /// would on scan completion — but driven host-side at the firmware
    /// boundary (see `wifi.rs` for the ground truth + why the post is
    /// host-driven, not firmware-run).
    ///
    /// What this does, in order (all offsets proven live on the wifi-scan
    /// image; every address is discovered live by the host — never
    /// hardcoded image guesswork beyond the call):
    /// 1. Locates `sys_evt` (the esp_event loop task) via
    ///    `find_task_by_name`; its event-wait queue (`evC - 36`,
    ///    `xTasksWaitingToReceive` offset) IS the default loop's queue
    ///    (proven: len 32 = `CONFIG_ESP_EVENT_LOOP_QUEUE_SIZE`, itemsize 16
    ///    = `sizeof(esp_event_post_instance_t)`).
    /// 2. Appends one 16-byte post item `{allocated=0, set=0,
    ///    base=<loaded WIFI_EVENT>, id=SCAN_DONE, val=0}` with REAL
    ///    `xQueueGenericSend` head semantics (`queue_post_raw`). `base` is
    ///    the LOADED event-base pointer (`read32(wifi_event_var)` — the
    ///    dispatcher compares base POINTERS, and `WIFI_EVENT` the symbol is
    ///    the pointer VARIABLE, proven live: `[0x3C0B4264]=0x3C0AAF7D`).
    ///    The empty payload is silicon-true ENOUGH: the Arduino `_scanDone`
    ///    record path re-reads the count/records from the ap store and
    ///    ignores the event data (only the verbose log reads it);
    ///    `allocated=0` means the task frees nothing (a host scratch
    ///    pointer would corrupt the heap on free).
    /// 3. Unblocks `sys_evt` (`queue_unblock_receiver`); the host readies
    ///    it via `ready_task_on_list` next (same step, same borrow). The
    ///    real chain then runs unmodified: `esp_event_loop_run` →
    ///    `handler_execute` → `_arduino_event_cb` → `postEvent` (arduino
    ///    queue) → arduino_events task → `_eventCallback` → `_scanDone` →
    ///    get_ap_num/records → `setStatusBits(DONE)` → `waitStatusBits`
    ///    returns → prints.
    ///
    /// Returns the woken `sys_evt` TCB on success. Returns `None` when
    /// `sys_evt` is not found, not event-parked, the queue is full, or the
    /// post item does not fit (host retries next step).
    ///
    /// `wifi_event_var` is the address of the `D WIFI_EVENT` pointer
    /// variable (its CONTENT is the compared base); `SCAN_DONE` is
    /// `WIFI_EVENT_SCAN_DONE = 1` (`esp_wifi_types_generic.h`:
    /// `WIFI_READY = 0`, `SCAN_DONE` next).
    pub fn wifi_scan_post_event(&mut self, wifi_event_var: u32) -> Option<u32> {
        const SCAN_DONE_ID: u32 = 1;
        self.wifi_post_event(wifi_event_var, SCAN_DONE_ID)
    }

    /// Post a scratch-payload event item into the default esp_event loop
    /// queue (host frontend — shared by all WiFi/IP fixture completions
    /// that carry event data): copies `payload` into the fixed host
    /// scratch region past the end of the main DRAM heap (see
    /// `WIFI_SCRATCH`), then appends `{allocated=0, set=1, base, id,
    /// val=ptr}` with REAL `xQueueGenericSend` head semantics and unblocks
    /// the waiter. The real chain then runs unmodified (`esp_event_loop_run`
    /// → `handler_execute` hands `&post.data.val` (the scratch pointer) to
    /// every registered handler → arduino `_eventCallback` → status bits).
    /// `allocated=0` means the loop frees nothing afterwards — the scratch
    /// is a deliberate, documented fixture window (same class as the
    /// scan-record calloc the firmware itself never frees until
    /// `scanDelete`).
    ///
    /// Why scratch and not `wifi_heap_carve`: the carve walks the live
    /// TLSF free lists (correct but fragile across images — the STA image
    /// needs a mapping-index fix the scan image does not); the scratch
    /// region is image-independent and read-only-safe (handlers only
    /// `memcpy` OUT of it). `wifi_heap_carve` is kept for probe use.
    ///
    /// `base_var` is the event-base POINTER VARIABLE (content = compared
    /// base); `netif_ptr` is prepended by the caller into the payload where
    /// the IDF struct expects it (e.g. `ip_event_got_ip_t.esp_netif`).
    ///
    /// Returns the woken `sys_evt` TCB on success (like `wifi_post_event`;
    /// the scratch payload address is internal — the loop owns it from
    /// here); `None` when `sys_evt` is not found/parked, the queue is full,
    /// or the item does not fit (host retries next step — the completion
    /// is level, not edge). On `Some` the host must call
    /// `ready_task_on_list(tcb, ...)` next (same step, same borrow).
    /// Fixed host scratch base for event payloads (see above): past the end
    /// of the main DRAM heap (`heap end` from the registered-heap list), so
    /// it can never be a heap block header, pool free block, or `calloc`
    /// target. Verified on the STA image: heap end `0x3fced710`, run
    /// `0x3fced710 len 0x858` (2136 B — plenty for a handful of ≤48 B
    /// posts). Successive posts advance a bump cursor (no reuse within a
    /// run). NOTE: the old base (`0x3fcb29d0`) sat INSIDE the pool (a TLSF
    /// free block) — the first `calloc` after staging overwrote the event
    /// and poisoned the pool (`CORRUPT HEAP: Bad head`, proven live).
    pub const WIFI_SCRATCH: u32 = 0x3fce_d710;
    /// Host event-block pool for arduino-queue posts (see `wifi_ard_post`):
    /// four 192-byte slots at `WIFI_SCRATCH + 0x10000` (past the pool end,
    /// never heap — the same fixture-window class as `WIFI_SCRATCH`
    /// itself). Backed by the dedicated `ard_pool` array (NOT DRAM — see
    /// its docs), so `heap_caps_free` never claims these blocks; the
    /// machine intercepts their free as a no-op leak-by-design (see
    /// `wifi_ard_free` + the `run_fast_core` hook).
    pub const WIFI_ARD_POOL: u32 = 0x3fce_d710 + 0x10000;
    /// Ard pool slot stride (188-byte event + 4-byte alignment pad).
    const WIFI_ARD_SLOT: u32 = 192;
    /// Number of host event slots (one queued host event at a time in every
    /// fixture flow; 4 is headroom).
    const WIFI_ARD_SLOTS: u32 = 4;

    /// Host-pool free interception (called by the machine when firmware
    /// enters `heap_caps_free`): true when `ptr` is a host event slot, in
    /// which case the caller must SKIP the call (the block is host-owned
    /// pool memory, not heap — freeing it would corrupt the heap walk;
    /// leaking 188 B/run is the documented fixture cost, same class as the
    /// IDF-side scratch payloads the loop never frees).
    pub fn wifi_ard_free(&mut self, ptr: u32) -> bool {
        (0..Self::WIFI_ARD_SLOTS).any(|s| Self::WIFI_ARD_POOL + s * Self::WIFI_ARD_SLOT == ptr)
    }

    pub fn wifi_post_event_with_data(
        &mut self,
        base_var: u32,
        id: u32,
        payload: &[u8],
    ) -> Option<u32> {
        use xtensa_core::Bus as _Bus;
        // Bump-allocate from the fixed scratch (4-aligned, never reused).
        const SCRATCH_SIZE: u32 = 0x858;
        let sys_probe = self.find_task_by_name(b"sys_evt\0")?;
        let ev_c_probe = self.read32(sys_probe + 40);
        if ev_c_probe == 0 {
            return None;
        }
        let queue_probe = ev_c_probe.wrapping_sub(36);
        if self.read32(queue_probe + 64) != 16 {
            return None;
        }
        // Waiter gating (same reason as `wifi_post_event` above — never
        // post into an unparked queue or retries wedge it at 32/32).
        if !self.queue_recv_waiting(queue_probe) {
            return None;
        }
        // Cursor from mw is stale-on-retry: derive the slot from the
        // scratch occupancy instead — scan DRAM words at each 64B slot for
        // a nonzero first word (all our payloads start with a nonzero
        // ssid byte / netif pointer; posts never zero their slot).
        let mut slot = 0u32;
        while slot + 64 <= SCRATCH_SIZE {
            if self.read32(Self::WIFI_SCRATCH + slot) == 0 {
                break;
            }
            slot += 64;
        }
        if slot + payload.len() as u32 > SCRATCH_SIZE {
            return None;
        }
        let buf = Self::WIFI_SCRATCH + slot;
        for (k, b) in payload.iter().enumerate() {
            self.write8(buf + k as u32, *b as u32);
        }
        let sys = self.find_task_by_name(b"sys_evt\0")?;
        let ev_c = self.read32(sys + 40);
        if ev_c == 0 {
            return None;
        }
        let queue = ev_c.wrapping_sub(36);
        let base = self.read32(base_var);
        let mut item = [0u8; 16];
        // ESP_EVENT_POST_FROM_ISR=y: bool allocated=0 (inline value, NOT
        // heap — the loop passes `&post.data.val` as data_ptr and frees
        // nothing), bool set=1, then base, id, data-as-u32-val (= the
        // carved heap pointer, read inline by every handler).
        item[0] = 0;
        item[1] = 1;
        item[4..8].copy_from_slice(&base.to_le_bytes());
        item[8..12].copy_from_slice(&id.to_le_bytes());
        item[12..16].copy_from_slice(&buf.to_le_bytes());
        if !self.queue_post_raw(queue, &item) {
            return None;
        }
        self.queue_unblock_receiver(queue)
    }

    /// Write ONE fixture AP directly into the Arduino `_scanResult` buffer
    /// as a `wifi_ap_record_t` (92 bytes), exactly as the closed
    /// `wifi_copy_ap_record` would have copied it — but without the BSS
    /// queue (see `wifi.rs`: with no RF stimulus the closed scan machine
    /// never enqueues nodes, and the copy loop skips everything when its
    /// count is 0).
    ///
    /// Layout (arduino-lib 3.3.10 `esp_wifi_types_generic.h`, PROVEN by a
    /// host g++ offsetof probe + the sketch's own codegen: `_scanDone`
    /// calloc's 92 bytes/slot and `_getScanInfoByIndex` strides 92 via
    /// addx2/subx8/addx4; `getNetworkInfo` reads ssid@6, bssid@0,
    /// channel@39, rssi@44, authmode@48): bssid[6]@0, ssid[33]@6
    /// (NUL-terminated: only `ssid_len` bytes written, then an explicit 0),
    /// primary@39, second u32@40 (=0 NONE), rssi i8@44, authmode u32@48
    /// (=3 WPA2_PSK), pairwise u32@52 (=4 CCMP), group u32@56 (=4 CCMP),
    /// ant u32@60 (=0), flags u32@64 (=0), country[7]@68 (zeros),
    /// he_ap[2]@80 (zeros), bandwidth u32@84 (=0 HT20), vht@88..89
    /// (zeros). Total 92 bytes. (C enums are 4 bytes — the earlier 62-byte
    /// packed guess read rssi/authmode 3-6 bytes early and printed
    /// garbage/empty.)
    /// `authmode`/`pairwise`/`group` must be nonzero-plausible: the sketch
    /// prints `encType` via `encryptionType(i)` which reads authmode, and
    /// a zeroed record would print OPEN for a WPA2 fixture (cosmetic, but
    /// wrong — the record must describe the fixture).
    ///
    /// `out` is the `_scanResult` buffer address (read live from the
    /// Arduino BSS by the host); `index` selects the 92-byte slot.
    pub fn wifi_scan_record_ap(&mut self, out: u32, index: usize, ap: &crate::wifi::ScanFixtureAp) {
        use xtensa_core::Bus as _Bus;
        const REC_SIZE: u32 = 92;
        let base = out + index as u32 * REC_SIZE;
        for (k, b) in ap.bssid.iter().enumerate() {
            self.write8(base + k as u32, *b as u32);
        }
        for k in 0..ap.ssid_len as usize {
            self.write8(base + 6 + k as u32, ap.ssid[k] as u32);
        }
        // ssid NUL terminator (host buffer is zero-filled by calloc; on
        // re-runs the slot may hold a stale longer SSID, so explicitly
        // terminate at ssid_len).
        self.write8(base + 6 + ap.ssid_len as u32, 0);
        self.write8(base + 39, ap.chan as u32);
        self.write32(base + 40, 0); // second = NONE
        self.write8(base + 44, ap.rssi as u8 as u32);
        self.write32(base + 48, 3); // authmode = WPA2_PSK
        self.write32(base + 52, 4); // pairwise = CCMP
        self.write32(base + 56, 4); // group = CCMP
        self.write32(base + 60, 0); // ant = ANT0
        self.write32(base + 64, 0); // phy/flags
        // country@68, he@80, bw@84, vht@88 stay as the buffer holds
        // (zeros from calloc).
    }

    /// Ready a task (host-side `prvAddTaskToReadyList` minimal form for the
    /// scan completion): append the state item to the tail of the ready
    /// list for the task's priority, bump the count, track top priority —
    /// then cross-core-yield any core currently running a LOWER-priority
    /// task, so the switch happens on the next instruction boundary
    /// instead of whenever the tick ISR next decides (proven live: without
    /// the yield, a readied prio-20 sys_evt sat on the ready list for 3M+
    /// steps while core1 idled — the tick ISR never re-evaluated because
    /// nothing pended a yield; FreeRTOS posts from task context call
    /// `taskYIELD_IF_USING_PREEMPTION` for exactly this reason).
    /// SRAM addresses for the ready lists live in ROM-data/high-DRAM; the
    /// list base + top-priority cell are passed in (discovered live by the
    /// host).
    pub fn ready_task_on_list(&mut self, owner: u32, list_base: u32, top_prio: u32) {
        use xtensa_core::Bus as _Bus;
        let prio = self.read32(owner + 44);
        let list = list_base + prio * 20;
        // Insert at tail: before end marker (end.prev chain).
        let end = list + 8;
        let prev = self.read32(end + 8);
        let item = owner + 4;
        self.write32(item + 4, end);
        self.write32(item + 8, prev);
        self.write32(prev + 4, item);
        self.write32(end + 8, item);
        self.write32(item, 0);
        self.write32(item + 16, list);
        let n = self.read32(list) + 1;
        self.write32(list, n);
        if prio > self.read32(top_prio) {
            self.write32(top_prio, prio);
        }
        // Cross-core yield: the readied task must PREEMPT whatever runs
        // now, or it sits until the tick ISR happens to re-evaluate (which
        // it won't — nothing pends a yield). The machine cannot see the
        // firmware's current-TCB cells, so the SoC asserts the yield here:
        // for each core whose `pxCurrentTCBs[core]` ( image layout cell)
        // holds a LOWER-priority task, raise FROM_CPU_INTR0/1 (sources
        // 79/80 = FreeRTOS yields; the ISR writes 0 to deassert, TRM
        // SYSTEM_CPU_INT_FROM_CPU_*).
        if let Some(pxcur) = self.wifi_pxcur {
            for core in 0..2 {
                let cur = self.read32(pxcur + core as u32 * 4);
                if cur != 0 && cur != owner {
                    let cur_prio = self.read32(cur + 44);
                    if prio > cur_prio {
                        self.cpu_int_from_cpu[core] |= 1;
                    }
                }
            }
        }
    }

    /// Override the PSRAM MR2 density nibble on the SPI1 PSRAM device
    /// (host frontend for 16 MB validation; see `Memspi::set_mr2`).
    pub fn psram_set_mr2(&mut self, v: u8) {
        self.memspi[0].set_mr2(v);
    }

    /// Snapshot of driven output-pin state (host LED visualization).  A
    /// pin whose FUNC_OUT_SEL selects a peripheral signal (LEDC 96..103)
    /// follows that signal instead of GPIO_OUT (TRM GPIO matrix).
    pub fn gpio_output(&self) -> u32 {
        let mut out = 0u32;
        for i in 0..self.gpio.pin_count() {
            if !self.gpio.enabled(i) {
                continue;
            }
            let sel = self.gpio.out_sel(i);
            // A pin drives its GPIO_OUT bit when FUNC_OUT_SEL_CFG selects
            // the GPIO function: SIG_GPIO_OUT_IDX = 256 = 0x100 (what the
            // S3 ROM leaves behind and `pinMatrixOutDetach` writes).
            // `pinMode`+`gpio_config` never touches FUNC_OUT_SEL, so 0x100
            // is also what a plain digital output reads. VALUE 0 also
            // selects GPIO_OUT on silicon (the reset Reception: FUNC_OUT_SEL
            // powers up 0 before the ROM writes 0x100 — and hand-assembled
            // machine tests write small signal numbers that must not route
            // to SPICLK_OUT_IDX). Any other value is a peripheral matrix
            // signal, whose level comes from signal_level. (The old
            // `sel == 0x80 || sel == 0` test was the CLASSIC-ESP32 sentinel;
            // on S3, 0x80 = SIG_IN_FUNC80, a peripheral. Probed live: FSEL10
            // read 0x100 after detach and the SS pin never drove HIGH.)
            let bit = if sel == 0x100 || sel == 0 {
                self.gpio.out_bit(i)
            } else {
                self.signal_level(sel)
            };
            out |= bit << i;
        }
        out
    }

    /// Mirror a CPU's dedicated-GPIO output latch (`tie_gpio`, written by
    /// the `ee.*gpio_out`/`wur.gpio_out` TIE instructions) into the SoC so
    /// `signal_level` can drive the CORE1_GPIO_OUT matrix signals. Called
    /// by the machine every step for both cores.
    pub fn set_dedic_out(&mut self, core: usize, bits: u32) {
        if core < 2 {
            self.dedic_out[core] = bits;
        }
    }

    /// Drain all host-observable events accumulated since the last call
    /// (GPIO edges + SPI/I2C transactions). The wasm bridge calls this once
    /// per animation frame and dispatches each event to the matching
    /// virtual-peripheral JS object (Wokwi-style). GPIO edge detection runs
    /// against the last reported output mask, so only net transitions survive
    /// a step batch (cheap, and matches per-frame rendering anyway).
    pub fn drain_events(&mut self) -> Vec<EmuEvent> {
        let cur = self.gpio_output();
        let prev = self.last_gpio_out;
        if cur != prev {
            // gpio_output() is a u32 mask, so only pins 0..32 are observable.
            for i in 0..self.gpio.pin_count().min(32) {
                let bit = 1u32 << i;
                if (cur & bit) != (prev & bit) {
                    self.events.push(EmuEvent {
                        kind: EVT_GPIO,
                        a: i as u32,
                        b: (cur >> i) & 1,
                    });
                }
            }
            self.last_gpio_out = cur;
        }
        core::mem::take(&mut self.events)
    }

    /// Retrieve the MOSI byte stream of the most recent SPI transfer on
    /// `chan` (0=GPSPI2, 1=GPSPI3) and clear it. Call this when an
    /// `EVT_SPI_XFER` event arrives; feed the response back with
    /// [`Soc::spi_inject_miso`].
    pub fn spi_take_tx(&mut self, chan: usize) -> Vec<u8> {
        core::mem::take(&mut self.pending_spi_tx[chan])
    }

    /// Inject MISO bytes for the next SPI transfer on `chan`
    /// (0=GPSPI2, 1=GPSPI3). The bytes are shifted into the RX buffer at
    /// transfer completion, so a virtual SPI device can answer the MCU.
    pub fn spi_inject_miso(&mut self, chan: usize, bytes: &[u8]) {
        self.spi[chan].inject_miso(bytes);
    }

    /// Provision the host fake quad-SPI device store on `chan` (a fixed
    /// pattern the firmware reads/writes through wide-mode USR transfers;
    /// see `Spi::quad_fake_provision`). The wasm bridge exposes this for
    /// virtual-SPI-flash frontends.
    pub fn spi_quad_fake_provision(&mut self, chan: usize, pattern: &[u8]) {
        self.spi[chan].quad_fake_provision(pattern);
    }

    /// Attach the SPI-mode SD card (SDSPI) on `chan` with `blocks`
    /// 512-byte blocks (host frontend for the Arduino `SD` library path).
    pub fn spi_sdspi_attach(&mut self, chan: usize, blocks: usize) {
        self.spi[chan].sdspi_attach(blocks);
    }

    /// Attach the SPI-mode SD card with a preformatted image (shares the
    /// SDMMC FAT16 bytes so `SD.begin` mounts a real filesystem).
    pub fn spi_sdspi_attach_image(&mut self, chan: usize, image: &[u8]) {
        self.spi[chan].sdspi_attach_image(image);
    }

    /// Attach the SDSPI card sharing the SDMMC card image (same FAT16
    /// volume, so SPI `SD.begin` mounts what SDMMC formatted).
    pub fn spi_sdspi_attach_sdmmc_image(&mut self, chan: usize) {
        let img = self.sdmmc.storage_image();
        self.spi[chan].sdspi_attach_image(&img);
    }

    /// Quad/dual wire-mode level of `chan` (0 single, 1 dual, 2 quad —
    /// see `Spi::quad_mode`).
    pub fn spi_quad_mode(&self, chan: usize) -> u32 {
        self.spi[chan].quad_mode()
    }

    /// Host-driven SPI slave master-write: capture `bytes` into the slave's
    /// data buffer on `chan` (0=GPSPI2, 1=GPSPI3), recording the bitlen and
    /// raising trans_done. Only acts when the controller is in slave mode.
    /// With DMA receive enabled (slave_mode + DMA_CONF.dma_rx_ena) the bytes
    /// stream into the GDMA IN-link DRAM buffers of the channel wired to
    /// this SPI peripheral instead (descriptors handed back, IN done raised)
    /// and completion latches SLV_WR_DMA_DONE.
    pub fn spi_slave_inject_write(&mut self, chan: usize, bytes: &[u8]) {
        if self.spi[chan].slave_dma_rx_enabled() {
            let peri = if chan == 0 {
                crate::gdma::GDMA_SPI2_PERIPH
            } else {
                crate::gdma::GDMA_SPI3_PERIPH
            };
            let mut done_ch = None;
            for rx in 0..crate::gdma::NCH {
                if self.gdma.in_peri_sel(rx) != peri || !self.gdma.in_link_started(rx) {
                    continue;
                }
                let mut idesc = self.gdma.in_link_addr(rx);
                let mut off = 0usize;
                loop {
                    let dw0 = self.read32(idesc);
                    let ibuf = self.read32(idesc + 4);
                    let inext = self.read32(idesc + 8);
                    let ilen = ((dw0 >> 12) & 0xFFF) as usize;
                    let ieof = (dw0 >> 30) & 1;
                    let iowner = (dw0 >> 31) & 1;
                    if iowner == 0 || off >= bytes.len() {
                        break;
                    }
                    let n = (bytes.len() - off).min(ilen);
                    for k in 0..n {
                        self.write8(ibuf + k as u32, bytes[off + k] as u32);
                    }
                    off += n;
                    if n >= ilen {
                        self.write32(idesc, dw0 & !(1u32 << 31));
                    }
                    if off >= bytes.len() || inext == 0 || ieof == 1 {
                        break;
                    }
                    idesc = inext;
                }
                done_ch = Some(rx);
                break;
            }
            self.spi[chan].slave_dma_done((bytes.len() * 8) as u32, true);
            if let Some(rx) = done_ch {
                self.gdma.raise_in_done(rx);
            }
        } else {
            self.spi[chan].slave_inject_write(bytes);
        }
    }

    /// Host-driven SPI slave master-read: return the first `nbytes` of the
    /// slave's preloaded data buffer on `chan`, recording the bitlen and
    /// raising trans_done. Only acts when the controller is in slave mode.
    /// With DMA transmit enabled the bytes source the GDMA OUT-link DRAM
    /// buffers instead (fully consumed descriptors handed back, OUT done
    /// raised) and completion latches SLV_RD_DMA_DONE.
    pub fn spi_slave_take_read(&mut self, chan: usize, nbytes: usize) -> Vec<u8> {
        if self.spi[chan].slave_dma_tx_enabled() {
            let peri = if chan == 0 {
                crate::gdma::GDMA_SPI2_PERIPH
            } else {
                crate::gdma::GDMA_SPI3_PERIPH
            };
            let mut out = alloc::vec::Vec::with_capacity(nbytes);
            for tx in 0..crate::gdma::NCH {
                if self.gdma.out_peri_sel(tx) != peri || !self.gdma.out_link_started(tx) {
                    continue;
                }
                let mut odesc = self.gdma.out_link_addr(tx);
                loop {
                    let dw0 = self.read32(odesc);
                    let obuf = self.read32(odesc + 4);
                    let onext = self.read32(odesc + 8);
                    let olen = ((dw0 >> 12) & 0xFFF) as usize;
                    let oeof = (dw0 >> 30) & 1;
                    let oowner = (dw0 >> 31) & 1;
                    if oowner == 0 || out.len() >= nbytes {
                        break;
                    }
                    let n = (nbytes - out.len()).min(olen);
                    for k in 0..n {
                        out.push(self.read8(obuf + k as u32) as u8);
                    }
                    if n >= olen {
                        self.write32(odesc, dw0 & !(1u32 << 31));
                    }
                    if out.len() >= nbytes || onext == 0 || oeof == 1 {
                        break;
                    }
                    odesc = onext;
                }
                self.gdma.raise_out_done(tx);
                break;
            }
            self.spi[chan].slave_dma_done((out.len() * 8) as u32, false);
            out
        } else {
            self.spi[chan].slave_take_read(nbytes)
        }
    }

    /// Inject RX bytes for the next I2C master-read on `chan`
    /// (0=I2CEXT0, 1=I2CEXT1). Each byte is returned to the MCU on a READ
    /// command; an empty supply reads back 0xFF (no device).
    pub fn i2c_inject_rx(&mut self, chan: usize, bytes: &[u8]) {
        self.i2c[chan].inject_rx(bytes);
    }

    /// Stage one camera frame for LCD_CAM capture: `bytes` are packed
    /// little-endian into words (one frame per call; frames queue and each
    /// `CAM_START` capture consumes the next). A virtual camera device
    /// supplies the sensor stream the RX FIFO would otherwise lack.
    pub fn cam_inject_frame(&mut self, bytes: &[u8]) {
        let mut words = alloc::vec::Vec::with_capacity(bytes.len().div_ceil(4));
        for chunk in bytes.chunks(4) {
            let mut w = [0u8; 4];
            w[..chunk.len()].copy_from_slice(chunk);
            words.push(u32::from_le_bytes(w));
        }
        self.lcd_cam.cam_inject_frame(&words);
    }

    /// Host-driven I2C slave master-write: if `addr7` matches, capture
    /// `bytes` into the slave's RX FIFO on `chan` (0=I2CEXT0, 1=I2CEXT1),
    /// latching the slave status and completion interrupts. Only acts in
    /// slave mode.
    pub fn i2c_slave_inject_write(&mut self, chan: usize, addr7: u32, bytes: &[u8]) {
        self.i2c[chan].slave_inject_write(addr7, bytes);
    }

    /// Host-driven I2C slave master-read: if `addr7` matches, pop up to `n`
    /// bytes from the slave's TX FIFO on `chan`. Only acts in slave mode.
    pub fn i2c_slave_take_read(&mut self, chan: usize, addr7: u32, n: usize) -> Vec<u8> {
        self.i2c[chan].slave_take_read(addr7, n)
    }

    /// ROM-boot phase flag (see the `rom_boot_mode` field docs).
    pub fn rom_boot_mode(&self) -> bool {
        self.rom_boot_mode
    }

    /// Set/clear the ROM-boot phase flag (machine boot/stub handoff).
    pub fn set_rom_boot_mode(&mut self, on: bool) {
        self.rom_boot_mode = on;
    }

    /// Advance timer groups by `cycles` (frontend time source).
    pub fn matrix_source(&self, cpu: usize, line: usize) -> u32 {
        self.intc.source_for_line(cpu, line)
    }

    pub fn tick_timers(&mut self, cycles: u64) {
        // Invalidate the per-step interrupt source bitmap cache.  Peripheral
        // tick() calls may change int_raw (alarm matches, timer overflows),
        // so any cached bitmap from a previous step is stale. The pad
        // readback cache goes with it (waveform ticks move driven levels).
        self.src_valid = false;
        self.rb_valid = false;
        for _ in 0..cycles {
            // ULP-RISC-V coprocessor: run one instruction per machine step when
            // released (the `core` sw_start bit). It has full access to the SoC
            // bus (RTC_SLOW_MEM, peripherals, DRAM). Swapped out via
            // mem::replace to satisfy the borrow checker: `ulp.step` needs
            // `&mut self` as its bus while `ulp` must be a separate local.
            if self.ulp.is_running() {
                let mut ulp = core::mem::take(&mut self.ulp);
                ulp.step(&mut *self);
                self.ulp = ulp;
            }
            // Peripheral clock gates (SYSCON): a gated peripheral freezes
            // (see `clk_on`). TIMERGROUP0/1 bits 13/15, SYSTIMER bit 29,
            // LEDC bit 11.
            if self.clk_on(false, 13) {
                self.timg[0].tick(1);
            }
            if self.clk_on(false, 15) {
                self.timg[1].tick(1);
            }
            if self.clk_on(false, 29) {
                self.systimer.tick(1);
            }
            if self.clk_on(false, 11) {
                self.ledc.tick();
            }
            // UART RX-timeout counters (no-op unless RX data is pending with
            // rx_tout_en set). Clocks: UART0 bit 2, UART1 bit 5 (EN0),
            // UART2 bit 9 (EN1).
            // UART CTS inputs (UnCTS_IN = 13/16/19): resolved through
            // the matrix input routing against the pad readback (loopback
            // sketches drive CTS from a GPIO). Reset routing reads GPIO0
            // (strap-high = stop), like silicon's pulled CTS pin: firmware
            // must route CTS to use TX flow control.
            let rb = self.gpio_in_readback();
            for (n, sig) in [(0u32, 13u32), (1, 16), (2, 19)] {
                let cts = match self.gpio.in_sel(sig) {
                    Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                    Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                    None => 1,
                };
                self.uarts[n as usize].set_cts(cts);
            }
            if self.clk_on(false, 2) {
                self.uarts[0].tick(1);
            }
            if self.clk_on(false, 5) {
                self.uarts[1].tick(1);
            }
            if self.clk_on(true, 9) {
                self.uarts[2].tick(1);
            }
            // SPI bit clocks: SPI2 bit 6, SPI3 bit 16 (EN0).
            if self.clk_on(false, 6) {
                self.spi[0].tick(1);
            }
            if let Some(tx) = self.spi[0].take_last_tx() {
                self.pending_spi_tx[0] = tx;
                self.events.push(EmuEvent {
                    kind: EVT_SPI_XFER,
                    a: 0,
                    b: self.pending_spi_tx[0].len() as u32,
                });
            }
            if self.clk_on(false, 16) {
                self.spi[1].tick(1);
            }
            if let Some(tx) = self.spi[1].take_last_tx() {
                self.pending_spi_tx[1] = tx;
                self.events.push(EmuEvent {
                    kind: EVT_SPI_XFER,
                    a: 1,
                    b: self.pending_spi_tx[1].len() as u32,
                });
            }
            // I2C bus: skip entirely when idle (common case during boot);
            // when active, batch the remaining cycles to avoid per-step
            // function-call overhead on the hot path. Clocks: I2C_EXT0
            // bit 7, I2C_EXT1 bit 18 (EN0).
            if !self.i2c[0].is_idle() && self.clk_on(false, 7) {
                let n = self.i2c[0].remaining_cycles().max(1);
                self.i2c[0].tick(n);
                if self.i2c[0].has_events() {
                    self.events.extend(self.i2c[0].drain_events());
                }
            }
            if !self.i2c[1].is_idle() && self.clk_on(false, 18) {
                let n = self.i2c[1].remaining_cycles().max(1);
                self.i2c[1].tick(n);
                if self.i2c[1].has_events() {
                    self.events.extend(self.i2c[1].drain_events());
                }
            }
            // APB_SARADC clock bit 28 (EN0); the SENS oneshot path is RTC
            // domain and needs no gate.
            if self.clk_on(false, 28) {
                self.adc.tick(1);
            }
            // Touch proximity counters (no-op unless approach pads armed).
            self.touch.tick(1);
            // Waveform peripherals: skip the tick while idle. Each gate
            // mirrors its tick's own early-out (inactive RMT channels /
            // stopped MCPWM timers / non-busy LCD / non-busy I2S), so a
            // skipped tick would have no-opped identically. Clock gates:
            // RMT bit 9, MCPWM0/1 bits 17/20 (EN0).
            if self.rmt.is_active() && self.clk_on(false, 9) {
                self.rmt.tick();
            }
            // RMT RX sampling: skipped unless a capture is armed/running.
            // Input levels resolve through the GPIO-matrix input routing
            // against the pad readback (so TX-driven pads loop back);
            // unrouted inputs read pull-up high. Pins 32+ are out of the
            // u32 readback word and also read high (same limit as the GPIO
            // edge sampler).
            if self.rmt.rx_pending() && self.clk_on(false, 9) {
                let rb = self.gpio_in_readback();
                let rmt_rx = |sig: u32| -> u32 {
                    match self.gpio.in_sel(sig) {
                        // Pins 32+ are out of the u32 readback word and read
                        // high (same limit as the GPIO edge sampler).
                        Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                        _ => 1,
                    }
                };
                self.rmt.tick_rx(&rmt_rx);
            }
            if self.mcpwm.is_active() && self.clk_on(false, 17) {
                self.mcpwm.tick();
            }
            if self.mcpwm1.is_active() && self.clk_on(false, 20) {
                self.mcpwm1.tick();
            }
            // MCPWM capture samples its channel inputs via the GPIO-matrix
            // input routing (PCNT-style); group 0 listens on 166..168,
            // group 1 on 175..177 (gpio_sig_map.h PWMx_CAPn_IN_IDX), sync on
            // 160..162 / 169..171 (the same SYNC0..2 as the timers). Levels
            // come from the pad readback (not pin_level: an output-enabled
            // peripheral-driven pin reads GPIO_OUT there, not the driven
            // signal).
            if self.mcpwm.cap_timer_enabled() && self.clk_on(false, 17) {
                let rb = self.gpio_in_readback();
                let cap_in = |sig: u32| -> u32 {
                    match self.gpio.in_sel(sig) {
                        Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                        Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                        None => 0,
                    }
                };
                self.mcpwm.tick_capture(166, 160, &cap_in);
            }
            if self.mcpwm1.cap_timer_enabled() && self.clk_on(false, 20) {
                let rb = self.gpio_in_readback();
                let cap_in = |sig: u32| -> u32 {
                    match self.gpio.in_sel(sig) {
                        Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                        Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                        None => 0,
                    }
                };
                self.mcpwm1.tick_capture(175, 169, &cap_in);
            }
            // MCPWM fault/trip: level-driven FAULT0..2 inputs (group 0 =
            // 163..165, group 1 = 172..174, gpio_sig_map.h PWMx_Fn_IN_IDX),
            // sampled even with every timer stopped. Unrouted inputs read
            // low so an enabled-but-unconnected detector never trips.
            if self.mcpwm.fault_active() && self.clk_on(false, 17) {
                let rb = self.gpio_in_readback();
                let fault_in = |sig: u32| -> u32 {
                    match self.gpio.in_sel(sig) {
                        Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                        Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                        None => 0,
                    }
                };
                self.mcpwm.tick_fault(163, &fault_in);
            }
            if self.mcpwm1.fault_active() && self.clk_on(false, 20) {
                let rb = self.gpio_in_readback();
                let fault_in = |sig: u32| -> u32 {
                    match self.gpio.in_sel(sig) {
                        Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                        Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                        None => 0,
                    }
                };
                self.mcpwm1.tick_fault(172, &fault_in);
            }
            // MCPWM timer sync: SYNC0..2 external inputs (group 0 =
            // 160..162, group 1 = 169..171) reload PHASE on rising edges
            // when SYNCI_EN is set. Sampled even with timers stopped.
            if self.mcpwm.sync_armed() && self.clk_on(false, 17) {
                let rb = self.gpio_in_readback();
                let sync_in = |sig: u32| -> u32 {
                    match self.gpio.in_sel(sig) {
                        Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                        Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                        None => 0,
                    }
                };
                self.mcpwm.tick_sync(160, &sync_in);
            }
            if self.mcpwm1.sync_armed() && self.clk_on(false, 20) {
                let rb = self.gpio_in_readback();
                let sync_in = |sig: u32| -> u32 {
                    match self.gpio.in_sel(sig) {
                        Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                        Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                        None => 0,
                    }
                };
                self.mcpwm1.tick_sync(169, &sync_in);
            }
            if self.sdm.is_active() {
                self.sdm.tick();
            }
            if self.lcd_cam.is_active() && self.clk_on(true, 8) {
                self.lcd_cam.tick();
            }
            if self.i2s[0].is_active() && self.clk_on(false, 4) {
                self.i2s[0].tick();
            }
            if self.i2s[1].is_active() && self.clk_on(false, 21) {
                self.i2s[1].tick();
            }
            // I2S GDMA streaming pump: trickle words between owned DMA
            // descriptors and the TX/RX FIFOs as space/data allow (the
            // esp-idf driver streams multi-hundred-byte frames through a
            // 16-word FIFO; a synchronous dump would overflow it). Gated on
            // any armed link (cold otherwise) plus the GDMA DMA clock
            // (EN1 bit 6): with the DMA clock off the walks below never
            // run, so the pump must idle too.
            if (self.i2s_dma_out.iter().any(|d| d.active)
                || self.i2s_dma_in.iter().any(|d| d.active))
                && self.clk_on(true, 6)
            {
                self.poll_i2s_dma();
            }
            // Camera-capture GDMA pump: trickle staged words into owned IN
            // descriptors (the LCD clock feeds the tick streamer, the DMA
            // clock feeds the walks below — both on, like the I2S pump).
            if self.cam_dma_in.active && self.clk_on(true, 8) && self.clk_on(true, 6) {
                self.poll_cam_dma();
            }
            // GPIO edge/level sampling: skipped unless a pin interrupt is
            // armed (the common case). Levels come from the pad readback so
            // peripheral-driven pins (RMT/MCPWM/LEDC...) also fire edges.
            if self.gpio.irq_armed() {
                let lv = self.gpio_in_readback();
                self.gpio.poll_interrupts(lv);
            }
            // PCNT samples its unit/channel signal inputs via the GPIO-matrix
            // input routing (FUNC_IN_SEL_CFG); resolve each signal index to the
            // GPIO pin's current level. Skipped while no unit is counting
            // (and after the one-time prev-level sampling) — the common case.
            if (!self.pcnt.is_init() || self.pcnt.is_counting()) && self.clk_on(false, 10) {
                let pcnt_input = |sig: u32| -> u32 {
                    match self.gpio.in_sel(sig) {
                        Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                        None => 0,
                    }
                };
                self.pcnt.tick(&pcnt_input);
            }
            // USB-Serial-JTAG: re-assert serial_in_empty_int when the
            // ISR cleared it but the FIFO is still empty (level-triggered
            // host-poll behavior). USB_DEVICE clock bit 10 (EN1).
            if self.clk_on(true, 10) {
                self.usb.tick();
            }
            // USB-OTG SOF frame counter (USB controller clock bit 23,
            // EN0): free-running while clocked (see HFNUM).
            if self.clk_on(false, 23) {
                self.usb_otg.tick();
            }
            // UHCI receive-hung watchdog (CONF CLK_EN + SYSTEM EN0.8):
            // counts ticks with RX_START latched and a dry UART RX.
            if self.uhci.clk_on() && self.clk_on(false, 8) {
                let empty = match self.uhci.uart_sel() {
                    Some(n) => !self.uarts[n].rx_pending(),
                    None => true,
                };
                self.uhci.tick_rx_idle(empty);
            }
        }
        self.rtc.tick(cycles);
        self.pll.tick(cycles);
        self.wifi.tick(cycles);
    }

    /// Internal SRAM access at `addr` (must be inside DRAM or IRAM window).
    /// D-bus: 0x3FC80000-0x3FD00000 is one 512 KB backing (ROM-data 32 KB +
    /// D/IRAM 416 KB + SRAM2 64 KB).  I-bus: SRAM0 (0x40370000-0x40377FFF,
    /// 32 KB, instruction-only, separate cells) and the D/IRAM instruction
    /// window (0x40378000-0x403DFFFF alias of data 0x3FC88000-0x3FCEFFFF,
    /// offset 0x6F0000 — memory.ld.in I_D_SRAM_OFFSET).
    fn ram8(&self, addr: u32) -> u8 {
        // Host event-block pool (NOT DRAM — outside every heap's bounds by
        // design; the range check must come first so pool addresses never
        // hit the sram indexing below, which would panic out-of-bounds).
        if in_range!(addr, Self::WIFI_ARD_POOL, 768) {
            return self.ard_pool[(addr - Self::WIFI_ARD_POOL) as usize];
        }
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE) {
            self.sram[(addr - DRAM_BASE) as usize]
        } else if in_range!(addr, IRAM_BASE, SRAM0_SIZE) {
            self.iram0[(addr - IRAM_BASE) as usize]
        } else {
            // D/IRAM instruction window (top 64 KB of the old window is
            // unmapped — the caller's range check already excluded it).
            self.sram[(DIRAM_DATA_BASE - DRAM_BASE + (addr - DIRAM_INST_BASE)) as usize]
        }
    }

    /// Flash byte at a physical flash offset (past the 4 MB end -> 0 like an
    /// unmapped flash region). With flash encryption enabled, decrypts the
    /// 16-byte block through a one-entry cache (XTS units are 16 bytes;
    /// instruction fetch is sequential/loopy so this hits ~always).
    fn flash_byte(&self, off: u32) -> u8 {
        if off >= FLASH_SIZE {
            0
        } else if self.flash_enc_enabled() {
            let base = off & !15;
            let (cb, blk) = self.flashenc_last.get();
            let blk = if cb == base {
                blk
            } else {
                let mut raw = [0xFFu8; 16];
                for (i, slot) in raw.iter_mut().enumerate() {
                    if let Some(v) = self.flash.get(base as usize + i) {
                        *slot = *v;
                    }
                }
                let dec = crate::aes::flash_xts_decrypt(&self.flash_xts_key(), base, &raw);
                self.flashenc_last.set((base, dec));
                dec
            };
            blk[(off & 15) as usize]
        } else {
            self.flash[off as usize]
        }
    }

    /// Byte read through a cache window (data or instruction), translated by
    /// the cache MMU to flash (read-only) or PSRAM (read-write) backing.
    fn cache_read8(&self, addr: u32) -> u8 {
        if self.rom_boot_mode {
            // ROM-boot phase: raw flash at the window offset (see field docs).
            // Window offset = low 25 bits, shared by both windows (Cache
            // `translate` uses the same `vaddr & WINDOW_MASK`).
            let off = addr & (FLASH_WINDOW_SIZE - 1);
            // The loader scratch (host-mapped app image for the ROM stub) is
            // MMU-routed even in ROM-boot mode — the stub reads the image
            // through that mapping (map_app_flash_segments).
            let scratch_end = LOADER_SCRATCH_OFF + self.loader_scratch_len;
            if !(off >= LOADER_SCRATCH_OFF && off < scratch_end) {
                return self.flash_byte(off);
            }
        }
        match self.cache.translate(addr) {
            Some(CacheTarget::Flash(off)) => self.flash_byte(off),
            Some(CacheTarget::Psram(off)) => {
                if off >= PSRAM_SIZE {
                    0
                } else {
                    self.psram[off as usize]
                }
            }
            None => 0,
        }
    }

    /// Byte write through a cache window: MMU-mapped PSRAM pages are
    /// writable; flash pages (including the invalid-entry 1:1 alias) are
    /// read-only and the write is dropped.
    fn cache_write8(&mut self, addr: u32, val: u8) {
        match self.cache.translate(addr) {
            Some(CacheTarget::Psram(off)) if off < PSRAM_SIZE => self.psram[off as usize] = val,
            _ => {}
        }
    }

    fn ram_write8(&mut self, addr: u32, val: u8) {
        // Host event-block pool (see `ram8`).
        if in_range!(addr, Self::WIFI_ARD_POOL, 768) {
            self.ard_pool[(addr - Self::WIFI_ARD_POOL) as usize] = val;
            return;
        }
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE) {
            if addr == 0x3FCEF750 || addr == 0x3FCEF748 {
                return;
            }
            self.sram[(addr - DRAM_BASE) as usize] = val;
        } else if in_range!(addr, IRAM_BASE, SRAM0_SIZE) {
            self.iram0[(addr - IRAM_BASE) as usize] = val;
        } else {
            self.sram[(DIRAM_DATA_BASE - DRAM_BASE + (addr - DIRAM_INST_BASE)) as usize] = val;
        }
    }

    /// GPIO_IN readback with output loopback resolved to the actual driven
    /// level. For a pin whose FUNC_OUT_SEL selects a peripheral matrix signal
    /// (e.g. MCPWM 160..165) the loopback level is that peripheral's output,
    /// not GPIO_OUT — matching real silicon where a peripheral-driven pad is
    /// readable via digitalRead (TRM GPIO matrix). Pins driving GPIO_OUT
    /// (FUNC_OUT_SEL_CFG 0x100 = SIG_GPIO_OUT_IDX, or 0 — see `gpio_output`)
    /// loop back the GPIO_OUT bit.
    ///
    /// PERF: up to 9 callers per tick share one 46-pin walk via the
    /// `cached_rb`/`rb_valid` pair. The cache is valid for the CURRENT tick
    /// only: `tick_timers` clears it at tick start, and `mmio32` clears it
    /// on every MMIO write. Within one tick all callers share the word —
    /// sound PROVIDED no caller runs after a state change in the same tick.
    /// That holds for the tick-internal callers (UART CTS x3, RMT RX,
    /// MCPWM cap/fault/sync, GPIO IRQ, PCNT-less dedic path: all sampled
    /// from the same frozen peripheral state). The CAM overlay is the one
    /// exception: `lcd_cam.tick()` advances capture state MID-tick (VSYNC
    /// goes live on the first tick after CAM_START), so the overlay reads
    /// the LIVE camera levels, never the cached word (see below).
    pub fn gpio_in_readback(&mut self) -> u32 {
        if self.rb_valid {
            return self.cached_rb;
        }
        let v = self.gpio_in_readback_uncached();
        self.cached_rb = v;
        self.rb_valid = true;
        v
    }

    /// Pad-readback WITH the live camera overlay (never cached). For the
    /// CAM-sensor loopback path: the overlay reads capture state that
    /// `lcd_cam.tick()` advances mid-tick, so sharing the frozen word
    /// would pin VSYNC at its pre-tick value (proven by the CAM machine
    /// test). All other callers use the cached `gpio_in_readback`.
    pub fn gpio_in_readback_with_cam(&self) -> u32 {
        self.gpio_in_readback_inner(true)
    }

    /// Uncached 46-pin pad-readback walk (see `gpio_in_readback`).
    /// `skip_cam` skips the camera-sensor overlay: the overlay reads LIVE
    /// capture state (`cam_driving`/`cam_input_level`), which `lcd_cam.tick`
    /// advances mid-tick — caching it would freeze VSYNC at its pre-tick
    /// value for every later caller in the same tick (proven by the CAM
    /// machine test: VSYNC latched but invisible on the routed pad).
    /// All other inputs are frozen within a tick, so they cache safely.
    fn gpio_in_readback_uncached(&self) -> u32 {
        self.gpio_in_readback_inner(false)
    }

    fn gpio_in_readback_inner(&self, with_cam: bool) -> u32 {
        let mut v = self.gpio.raw_in();
        for i in 0..self.gpio.pin_count() {
            if !self.gpio.enabled(i) {
                continue;
            }
            let sel = self.gpio.out_sel(i);
            let driven = if sel == 0x100 || sel == 0 {
                self.gpio.out_bit(i)
            } else {
                self.signal_level(sel)
            };
            if driven != 0 {
                v |= 1 << i;
            } else {
                v &= !(1 << i);
            }
        }
        // Camera sensor loopback: while a capture runs, pads whose input
        // routing selects a CAM signal read the sensor-driven level (like a
        // real sensor driving the pad). Gated on capturing (cold otherwise).
        // LIVE overlay — skipped by the cached path (see `gpio_in_readback`).
        if with_cam && self.lcd_cam.cam_driving() {
            for sig in [149, 150, 151, 152] {
                // Pins 32+ are out of the u32 readback word (same limit as
                // the RMT sampler above).
                if let Some((pin, inv)) = self.gpio.in_sel(sig)
                    && pin < 32
                {
                    if self.lcd_cam.cam_input_level(sig) ^ (inv as u32) != 0 {
                        v |= 1 << pin;
                    } else {
                        v &= !(1 << pin);
                    }
                }
            }
        }
        v
    }

    /// GPIO matrix output signal level for a peripheral signal index.
    /// LEDC occupies 73..80, I2CEXT0 SCL/SDA = 89/90, I2CEXT1 = 91/92,
    /// GPSPI2 (FSPI) 101..105 + CS 110/111, GPSPI3 66..72 (S3
    /// gpio_sig_map.h); everything else reads 0.
    ///
    /// Dedicated-GPIO OUT channels (CORE1_GPIO_OUT0..2 = 129..131,
    /// OUT3..6 = 252..255, OUT7 = 54) drive the OR of both cores'
    /// mirrored `tie_gpio` latches (either core writing lights the pin;
    /// single-core firmware is exact).
    fn signal_level(&self, sig: u32) -> u32 {
        if (73..=80).contains(&sig) {
            self.ledc.signal_level(sig)
        } else if (81..=84).contains(&sig) {
            self.rmt.signal_level(sig)
        } else if (160..=165).contains(&sig) {
            // MCPWM0 operator 0..2 output A/B (PWM0_OUT0A..OUT2B_IDX).
            self.mcpwm.signal_level(sig)
        } else if (166..=171).contains(&sig) {
            // MCPWM1 operator 0..2 output A/B (PWM1_OUT0A..OUT2B_IDX).
            // (Indices overlap MCPWM0's CAPx/SYNCx *input* signals, which
            // live in the separate input-routing table.)
            self.mcpwm1.signal_level(sig - 6)
        } else if (93..=100).contains(&sig) {
            // Sigma-Delta channels 0..7 (GPIO_SD0..7_OUT_IDX).
            self.sdm.signal_level(sig)
        } else if (132..=154).contains(&sig) {
            // LCD_CAM parallel data / clock / control signals.
            self.lcd_cam.signal_level(sig)
        } else if (22..=27).contains(&sig) {
            // I2S0 output signals (BCK/MCLK/WS/SD + RX BCK/WS).
            self.i2s[0].signal_level(sig)
        } else if (28..=32).contains(&sig) {
            // I2S1 output signals (BCK/WS/SD + RX BCK/WS).
            self.i2s[1].signal_level(sig)
        } else if matches!(sig, 13 | 16 | 19) {
            // UART RTS outputs (UnRTS_OUT, active-low ready).
            let n = match sig {
                13 => 0,
                16 => 1,
                _ => 2,
            };
            self.uarts[n].rts_level()
        } else if matches!(sig, 54 | 129..=131 | 252..=255) {
            // Dedicated-GPIO OUT channels 0..2/3..6/7 (CORE1_GPIO_OUTx).
            let ch = match sig {
                129..=131 => sig - 129,
                252..=255 => sig - 252 + 3,
                _ => 7,
            };
            ((self.dedic_out[0] | self.dedic_out[1]) >> ch) & 1
        } else {
            self.spi[0].signal_level(sig)
                | self.spi[1].signal_level(sig)
                | self.i2c[0].signal_level(sig)
                | self.i2c[1].signal_level(sig)
        }
    }

    /// Expose the interrupt-matrix table for diagnostics (run_flash probes).
    pub fn intc(&self) -> &Intc {
        &self.intc
    }

    fn mmio32(&mut self, addr: u32, is_write: bool, value: u32) -> u32 {
        let base = addr & !3;
        let off = base & 0xFFF;
        let dev = base & !0xFFF;
        // Any MMIO write can move a pad level (GPIO OUT/ENABLE/FUNC_SEL,
        // an RMT/MCPWM/LEDC/SPI waveform register, ...), so the cached
        // pad-readback word dies here. Reads never move levels.
        if is_write {
            self.rb_valid = false;
        }
        match dev {
            UART0_BASE | UART1_BASE | UART2_BASE => {
                let n = if dev == UART0_BASE {
                    0
                } else if dev == UART1_BASE {
                    1
                } else {
                    2
                };
                if is_write {
                    self.uarts[n].write32(off, value);
                    0
                } else {
                    self.uarts[n].read32(off)
                }
            }
            UHCI0_BASE => {
                // UHCI0 DMA bridge (CONF0 + INT block; framing codec not
                // modeled, see uhci.rs).
                if is_write {
                    self.uhci.write32(off, value);
                    0
                } else {
                    self.uhci.read32(off)
                }
            }
            GPIO_BASE => {
                // Page 0x6000_4000 also holds the Sigma-Delta block at
                // 0x6000_4F00 (DR_REG_GPIO_SD_BASE). Route its window to the
                // SDM device; everything else is plain GPIO. The reserved gap
                // above the SDM window reads as 0 (no registers mapped).
                if off >= 0xF00 {
                    if (0xF00..=0xF28).contains(&off) {
                        if is_write {
                            self.sdm.write32(off - 0xF00, value);
                            0
                        } else {
                            self.sdm.read32(off - 0xF00)
                        }
                    } else {
                        0
                    }
                } else if is_write {
                    self.gpio.write32(off, value);
                    0
                } else if off == crate::gpio::GPIO_IN {
                    // Resolve the output loopback to the real driven level
                    // (peripheral signal for matrix-routed pins).
                    self.gpio_in_readback()
                } else {
                    self.gpio.read32(off)
                }
            }
            USB_SERIAL_JTAG_BASE => {
                // USB-Serial-JTAG CDC console (functional model).
                if is_write {
                    self.usb.write32(off, value);
                    0
                } else {
                    self.usb.read32(off)
                }
            }
            LEDC_BASE => {
                if is_write {
                    self.ledc.write32(off, value);
                    0
                } else {
                    self.ledc.read32(off)
                }
            }
            SPI2_BASE | SPI3_BASE => {
                // LP_UART (0x6002_5400) shares the GPSPI3 4KB page but sits at
                // offset 0x400, past SPI3's register block (< 0x400).
                if dev == SPI3_BASE && off >= 0x400 {
                    let lp_off = off - 0x400;
                    if is_write {
                        self.lp_uart.write32(lp_off, value);
                        0
                    } else {
                        self.lp_uart.read32(lp_off)
                    }
                } else {
                    let n = if dev == SPI2_BASE { 0 } else { 1 };
                    if is_write {
                        // SPI peripheral clock (EN0 bits 6/16): a USR
                        // transfer doesn't start gated (its CMD bit is
                        // masked out; every other register processes).
                        let bit = if n == 0 { 6 } else { 16 };
                        let v = if !self.clk_on(false, bit) && off == 0x00 {
                            value & !(1u32 << 24)
                        } else {
                            value
                        };
                        self.spi[n].write32(off, v);
                        // Latch the SS-pin level for the SDSPI card's
                        // CS gate: the Arduino `SD` driver bit-bangs SS as
                        // plain GPIO (pinMode OUTPUT + digitalWrite HIGH/
                        // LOW around each `sdSelectCard`). The pin stays on
                        // the GPIO function the whole time (the driver never
                        // routes a peripheral there — FSEL10 reads 0x100
                        // from boot through init, probed live), so plain
                        // `gpio_out_bit` IS the driven SS level. Sample on
                        // every SPI write (cheap: two array reads); the
                        // card consumes it at `complete()`. SS pin = GPIO10
                        // in the `sdspi` sketch wiring (SCK=12, MISO=13,
                        // MOSI=11, SS=10).
                        if n == 0 {
                            let low = self.gpio.enabled(10) && self.gpio.out_bit(10) == 0;
                            self.spi[0].set_sdspi_cs(low);
                        }
                        0
                    } else {
                        self.spi[n].read32(off)
                    }
                }
            }
            // SPI1 (SPIMEM1) + SPI0 (SPIMEM0): flash controller registers;
            // transactions execute synchronously against the flash backing.
            SPI1_BASE | SPIMEM0_BASE => {
                let n = if dev == SPI1_BASE { 0 } else { 1 };
                if is_write {
                    self.memspi[n].write32(&mut self.flash[..], off, value);
                    // A programmed transaction may have rewritten backing
                    // under the XIP decrypt cache (encrypted devices).
                    self.flashenc_last.set((u32::MAX, [0; 16]));
                    0
                } else {
                    self.memspi[n].read32(off)
                }
            }
            I2C0_BASE | I2C1_BASE => {
                let n = if dev == I2C0_BASE { 0 } else { 1 };
                if is_write {
                    // I2C peripheral clock (EN0 bits 7/18): TRANS_START
                    // doesn't fire gated (bit masked out, rest processes).
                    let bit = if n == 0 { 7 } else { 18 };
                    let v = if !self.clk_on(false, bit) && off == 0x04 {
                        value & !(1u32 << 5)
                    } else {
                        value
                    };
                    self.i2c[n].write32(off, v);
                    0
                } else {
                    self.i2c[n].read32(off)
                }
            }
            RMT_BASE => {
                if is_write {
                    self.rmt.write32(off, value);
                    0
                } else {
                    self.rmt.read32(off)
                }
            }
            PCNT_BASE => {
                if is_write {
                    self.pcnt.write32(off, value);
                    0
                } else {
                    self.pcnt.read32(off)
                }
            }
            GDMA_BASE => {
                if is_write {
                    // A write that starts an OUT (TX) or IN (RX) channel transfer
                    // triggers the descriptor-walk copy — gated on the GDMA
                    // DMA clock (EN1 bit 6): with the clock off the link
                    // stays armed but nothing moves and no done raises
                    // (firmware hangs like silicon until it enables it).
                    if let Some((ch, is_out)) = self.gdma.write32(off, value) {
                        if self.clk_on(true, 6) {
                            let link_addr = if is_out {
                                self.gdma.out_link_addr(ch)
                            } else {
                                self.gdma.in_link_addr(ch)
                            };
                            let peri = if is_out {
                                self.gdma.out_peri_sel(ch)
                            } else {
                                self.gdma.in_peri_sel(ch)
                            };
                            // Walk the descriptor chain. Descriptor addresses are
                            // in DRAM; the link register holds the 20 LSBs
                            // (GDMA_DESC_BASE), buffer/next are full 32-bit
                            // addresses. Copy `length` bytes between each frame and
                            // the connected peripheral.
                            // SPI DMA transfers trigger once per OUT-link start
                            // (all descriptors staged first); see below.
                            let mut spi_dma_pending: Option<usize> = None;
                            let mut desc = link_addr;
                            // SPI slave DMA links arm only: the host-driven
                            // exchange (spi_slave_inject_write/take_read) walks
                            // the descriptors and raises completion when the
                            // external master actually clocks. A master-style
                            // walk here would stage phantom TX bytes (and even
                            // fire a master dma_trigger) or consume the RX
                            // descriptor setup with zeros before the exchange.
                            let slave_arm_only = if peri == crate::gdma::GDMA_SPI2_PERIPH
                                || peri == crate::gdma::GDMA_SPI3_PERIPH
                            {
                                let idx = if peri == crate::gdma::GDMA_SPI2_PERIPH {
                                    0
                                } else {
                                    1
                                };
                                self.spi[idx].is_slave()
                            } else {
                                false
                            };
                            // I2S GDMA streams through the per-tick pump (its
                            // frames dwarf the 16-word FIFO); arm the cursor and
                            // skip the synchronous walk. All other peripherals
                            // keep the immediate walk below.
                            //
                            // Memory-to-memory (IN_CONF0 mem_trans_en, TRM
                            // gdma_struct.h bit 4): the OUT-link descriptors
                            // source DRAM bytes the IN-link descriptors sink on
                            // the same channel pair. The copy runs once BOTH
                            // links are started (order-independent: a lone start
                            // only arms — without this gate the first start
                            // would fall into the normal peri walk and consume
                            // the source descriptors) and hands both descriptor
                            // chains back with OUT + IN done raised. Descriptor
                            // pairs advance lockstep 1:1 (a longer side's excess
                            // bytes are dropped) — validation uses exact pairs.
                            let m2m_armed = self.gdma.in_mem_trans_en(ch);
                            let m2m_ready = m2m_armed
                                && self.gdma.out_link_started(ch)
                                && self.gdma.in_link_started(ch);
                            if m2m_ready {
                                let mut odesc = self.gdma.out_link_addr(ch);
                                let mut idesc = self.gdma.in_link_addr(ch);
                                loop {
                                    let odw0 = self.read32(odesc);
                                    let obuf = self.read32(odesc + 4);
                                    let onext = self.read32(odesc + 8);
                                    let olen = ((odw0 >> 12) & 0xFFF) as usize;
                                    let oeof = (odw0 >> 30) & 1;
                                    if (odw0 >> 31) & 1 == 0 {
                                        break;
                                    }
                                    let idw0 = self.read32(idesc);
                                    let ibuf = self.read32(idesc + 4);
                                    let inext = self.read32(idesc + 8);
                                    let ilen = ((idw0 >> 12) & 0xFFF) as usize;
                                    let ieof = (idw0 >> 30) & 1;
                                    if (idw0 >> 31) & 1 == 0 {
                                        break;
                                    }
                                    let n = olen.min(ilen);
                                    for k in 0..n {
                                        let b = self.read8(obuf + k as u32);
                                        self.write8(ibuf + k as u32, b);
                                    }
                                    self.write32(odesc, odw0 & !(1u32 << 31));
                                    self.write32(idesc, idw0 & !(1u32 << 31));
                                    self.gdma.set_out_eof_des_addr(ch, odesc);
                                    if onext == 0 || oeof == 1 || inext == 0 || ieof == 1 {
                                        break;
                                    }
                                    odesc = onext;
                                    idesc = inext;
                                }
                                self.gdma.raise_out_done(ch);
                                self.gdma.raise_in_done(ch);
                            } else if m2m_armed {
                                // Lone M2M start: arm only (see above).
                            } else if slave_arm_only {
                            } else if is_out
                                && (peri == crate::gdma::GDMA_I2S0_PERIPH
                                    || peri == crate::gdma::GDMA_I2S1_PERIPH)
                            {
                                let p = (peri - crate::gdma::GDMA_I2S0_PERIPH) as usize;
                                self.i2s_dma_out[p] = I2sDma {
                                    active: true,
                                    ch,
                                    desc: link_addr,
                                    off: 0,
                                };
                                self.i2s[p].set_tx_dma_pending(true);
                            } else if !is_out
                                && (peri == crate::gdma::GDMA_I2S0_PERIPH
                                    || peri == crate::gdma::GDMA_I2S1_PERIPH)
                            {
                                let p = (peri - crate::gdma::GDMA_I2S0_PERIPH) as usize;
                                self.i2s_dma_in[p] = I2sDma {
                                    active: true,
                                    ch,
                                    desc: link_addr,
                                    off: 0,
                                };
                            } else if !is_out && peri == crate::gdma::GDMA_LCD_PERIPH {
                                // Camera capture into IN descriptors: arm the
                                // streaming cursor (the tick streamer feeds
                                // it; done raises per filled descriptor).
                                // LCD-TX shares peri 5 but is OUT direction.
                                self.cam_dma_in = CamDma {
                                    active: true,
                                    ch,
                                    desc: link_addr,
                                    off: 0,
                                };
                                self.lcd_cam.set_dma_active(true);
                                self.cam_link_fresh = true;
                            } else {
                                loop {
                                    let dw0 = self.read32(desc);
                                    let buf = self.read32(desc + 4);
                                    let next = self.read32(desc + 8);
                                    let len = (dw0 >> 12) & 0xFFF;
                                    let eof = (dw0 >> 30) & 1;
                                    let owner = (dw0 >> 31) & 1;
                                    if owner == 0 {
                                        break;
                                    }
                                    if is_out {
                                        if peri == crate::gdma::GDMA_RMT_PERIPH {
                                            let dst = crate::rmt::RMTMEM_BASE + (ch as u32) * 0x100;
                                            // Copy in 32-bit words. RMT items are
                                            // word-aligned and the item buffer length is a
                                            // multiple of 4; a byte-wise copy would clobber
                                            // whole words because RMTMEM writes via
                                            // write32 store the full word (see Rmt::write32).
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                let w = self.read32(buf + k);
                                                self.write32(dst + k, w);
                                                k += 4;
                                            }
                                            self.gdma.set_out_eof_des_addr(ch, desc);
                                        } else if peri == crate::gdma::GDMA_SHA_PERIPH {
                                            // SHA: copy the descriptor's message bytes into
                                            // the SHA engine's message buffer (reconstructed
                                            // LSB-first per 32-bit word, the byte order the
                                            // SHA engine consumes). The driver passes an
                                            // already-padded, complete 64-byte block.
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                let w = self.read32(buf + k);
                                                self.sha.feed_byte((w & 0xFF) as u8);
                                                self.sha.feed_byte(((w >> 8) & 0xFF) as u8);
                                                self.sha.feed_byte(((w >> 16) & 0xFF) as u8);
                                                self.sha.feed_byte(((w >> 24) & 0xFF) as u8);
                                                k += 4;
                                            }
                                            self.gdma.set_out_eof_des_addr(ch, desc);
                                        } else if peri == crate::gdma::GDMA_AES_PERIPH {
                                            // AES: copy the descriptor's plaintext into the
                                            // AES TEXT_IN buffer (LSB-first per word), then
                                            // run the transform. In DMA mode the engine
                                            // auto-pushes the ciphertext to the paired GDMA
                                            // `in` channel, so walk that channel's
                                            // descriptors and copy TEXT_OUT -> DRAM here.
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                let w = self.read32(buf + k);
                                                self.aes.feed_text_in_byte((w & 0xFF) as u8);
                                                self.aes.feed_text_in_byte(((w >> 8) & 0xFF) as u8);
                                                self.aes
                                                    .feed_text_in_byte(((w >> 16) & 0xFF) as u8);
                                                self.aes
                                                    .feed_text_in_byte(((w >> 24) & 0xFF) as u8);
                                                k += 4;
                                            }
                                            self.aes.transform();
                                            self.gdma.set_out_eof_des_addr(ch, desc);
                                            // Auto-push ciphertext through the AES RX
                                            // channel. The esp-idf AES driver allocates a
                                            // SEPARATE GDMA channel for the ciphertext `in`
                                            // link (crypto_shared_gdma_new_channel is called
                                            // twice: TX then RX), so the RX channel index is
                                            // not the same as this TX channel. Find the
                                            // channel whose `in` block is wired to AES and
                                            // copy TEXT_OUT -> its descriptor buffers.
                                            for rx in 0..crate::gdma::NCH {
                                                if self.gdma.in_peri_sel(rx)
                                                    == crate::gdma::GDMA_AES_PERIPH
                                                {
                                                    let mut idesc = self.gdma.in_link_addr(rx);
                                                    loop {
                                                        let dw0 = self.read32(idesc);
                                                        let ibuf = self.read32(idesc + 4);
                                                        let inext = self.read32(idesc + 8);
                                                        let ilen = (dw0 >> 12) & 0xFFF;
                                                        let ieof = (dw0 >> 30) & 1;
                                                        let iowner = (dw0 >> 31) & 1;
                                                        if iowner == 0 {
                                                            break;
                                                        }
                                                        let mut k = 0u32;
                                                        while k + 4 <= ilen {
                                                            let w = self.aes.out_word_at(k);
                                                            self.write32(ibuf + k, w);
                                                            k += 4;
                                                        }
                                                        // DMA hands the descriptor back: clear owner
                                                        // (the driver polls owner / reads length/suc_eof).
                                                        self.write32(idesc, dw0 & !(1u32 << 31));
                                                        if inext == 0 || ieof == 1 {
                                                            break;
                                                        }
                                                        idesc = inext;
                                                    }
                                                    self.gdma.raise_in_done(rx);
                                                    break;
                                                }
                                            }
                                        } else if peri == crate::gdma::GDMA_SPI2_PERIPH
                                            || peri == crate::gdma::GDMA_SPI3_PERIPH
                                        {
                                            // SPI master DMA: stage the descriptor's
                                            // bytes (address order) for one DMA-backed
                                            // transfer, triggered after the walk (see
                                            // below) so multi-descriptor chains send
                                            // once. The transfer runs like a USR op
                                            // (waveform + trans_done); start the IN
                                            // link after trans_done latches.
                                            let idx = if peri == crate::gdma::GDMA_SPI2_PERIPH {
                                                0
                                            } else {
                                                1
                                            };
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                let w = self.read32(buf + k);
                                                self.spi[idx].spi_dma_feed(&w.to_le_bytes());
                                                k += 4;
                                            }
                                            if k < len {
                                                let w = self.read32(buf + k);
                                                let tail = &w.to_le_bytes()[..(len - k) as usize];
                                                self.spi[idx].spi_dma_feed(tail);
                                            }
                                            self.gdma.set_out_eof_des_addr(ch, desc);
                                            spi_dma_pending = Some(idx);
                                        } else if peri == crate::gdma::GDMA_LCD_PERIPH {
                                            // LCD_CAM DMA: stream the descriptor's
                                            // words through the TX FIFO as one
                                            // synchronous 8080 transfer (word-aligned
                                            // lengths; a partial tail word is dropped).
                                            let mut words =
                                                alloc::vec::Vec::with_capacity((len / 4) as usize);
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                words.push(self.read32(buf + k));
                                                k += 4;
                                            }
                                            self.lcd_cam.dma_transfer(&words);
                                            self.gdma.set_out_eof_des_addr(ch, desc);
                                        } else if peri == crate::gdma::GDMA_UHCI0_PERIPH {
                                            // UHCI TX: move descriptor bytes into
                                            // the UHCI-selected UART's TX FIFO
                                            // (framing-off passthrough; dropped
                                            // when the UHCI clock is off).
                                            let mut k = 0u32;
                                            let mut bytes =
                                                alloc::vec::Vec::with_capacity(len as usize);
                                            while k + 4 <= len {
                                                let w = self.read32(buf + k);
                                                bytes.extend_from_slice(&w.to_le_bytes());
                                                k += 4;
                                            }
                                            if k < len {
                                                let w = self.read32(buf + k);
                                                bytes.extend_from_slice(
                                                    &w.to_le_bytes()[..(len - k) as usize],
                                                );
                                            }
                                            if self.uhci.clk_on()
                                                && self.clk_on(false, 8)
                                                && let Some(n) = self.uhci.uart_sel()
                                            {
                                                // SLIP-framed when SEPER_EN is set.
                                                let wire = self.uhci.slip_encode(&bytes);
                                                self.uarts[n].push_tx(&wire);
                                            }
                                            self.uhci.latch_tx_start();
                                            self.gdma.set_out_eof_des_addr(ch, desc);
                                        }
                                    } else {
                                        // IN (RX) channel: copy from the peripheral's data
                                        // registers into the descriptor's DRAM buffer.
                                        if peri == crate::gdma::GDMA_AES_PERIPH {
                                            // AES ciphertext out -> DRAM.
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                let w = self.aes.out_word_at(k);
                                                self.write32(buf + k, w);
                                                k += 4;
                                            }
                                            // The AES engine has already transformed the
                                            // block (the TX `out` write fed plaintext and
                                            // ran the cipher); raise the RX done so the
                                            // driver's completion ISR fires regardless of
                                            // whether the RX link was started before or
                                            // after the TX link.
                                            self.gdma.enable_in_int_all(ch);
                                            self.gdma.raise_in_done(ch);
                                        } else if peri == crate::gdma::GDMA_SPI2_PERIPH
                                            || peri == crate::gdma::GDMA_SPI3_PERIPH
                                        {
                                            // SPI RX: copy the last DMA transfer's
                                            // captured bytes into the descriptor's DRAM
                                            // buffer (zeros beyond the capture; start
                                            // this link after trans_done latches).
                                            let idx = if peri == crate::gdma::GDMA_SPI2_PERIPH {
                                                0
                                            } else {
                                                1
                                            };
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                let w = self.spi[idx].dma_rx_word(k);
                                                self.write32(buf + k, w);
                                                k += 4;
                                            }
                                            self.gdma.raise_in_done(ch);
                                        } else if peri == crate::gdma::GDMA_ADC_PERIPH {
                                            // ADC digital DMA: drain staged conversion
                                            // results to DRAM (zeros once the queue
                                            // runs dry).
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                let w = self.adc.dma_pop().unwrap_or(0);
                                                self.write32(buf + k, w);
                                                k += 4;
                                            }
                                            self.gdma.raise_in_done(ch);
                                        } else if peri == crate::gdma::GDMA_UHCI0_PERIPH {
                                            // UHCI RX: drain the selected UART's RX
                                            // FIFO to DRAM (zeros once it runs dry;
                                            // empty when the UHCI clock is off).
                                            // HEAD capture (HEAD_EN + SAVE_HEAD): the
                                            // transfer's first 2 payload bytes land
                                            // in RX_HEAD instead of DRAM.
                                            let capture = self.uhci.head_capture();
                                            let mut head = [0u8; 2];
                                            let mut head_n = 0usize;
                                            let mut k = 0u32;
                                            while k + 4 <= len {
                                                let mut w = [0u8; 4];
                                                if self.uhci.clk_on()
                                                    && self.clk_on(false, 8)
                                                    && let Some(n) = self.uhci.uart_sel()
                                                {
                                                    // Deframed when SEPER_EN is set (loop
                                                    // while separators compress the yield).
                                                    let mut data = alloc::vec::Vec::new();
                                                    while data.len() < 4 {
                                                        let raw = self.uarts[n].take_rx(4);
                                                        if raw.is_empty() {
                                                            break;
                                                        }
                                                        data.extend(self.uhci.slip_decode(&raw));
                                                    }
                                                    data.truncate(4);
                                                    let mut di = 0;
                                                    while capture && head_n < 2 && di < data.len() {
                                                        head[head_n] = data[di];
                                                        head_n += 1;
                                                        di += 1;
                                                    }
                                                    let rest = &data[di..];
                                                    w[..rest.len()].copy_from_slice(rest);
                                                }
                                                self.write32(buf + k, u32::from_le_bytes(w));
                                                k += 4;
                                            }
                                            if capture && head_n > 0 {
                                                let hi = if head_n > 1 { head[1] } else { 0 };
                                                self.uhci.set_rx_head(head[0], hi);
                                            }
                                            self.uhci.latch_rx_start();
                                            self.gdma.raise_in_done(ch);
                                        }
                                    }
                                    // DMA hands the descriptor back: clear owner.
                                    self.write32(desc, dw0 & !(1u32 << 31));
                                    if next == 0 || eof == 1 {
                                        break;
                                    }
                                    desc = next;
                                }
                                if let Some(idx) = spi_dma_pending {
                                    // One DMA-backed SPI transfer per OUT-link start.
                                    self.spi[idx].dma_trigger();
                                }
                                if is_out {
                                    self.gdma.raise_out_done(ch);
                                } else {
                                    self.gdma.raise_in_done(ch);
                                }
                            } // end non-I2S synchronous walk
                        } // end DMA-clock gate
                    } else if off < 5 * 0xC0 {
                        // Retroactive I2S pump arm: the esp-idf I2S driver
                        // programs the link (with START) BEFORE peri_sel, so
                        // the start write above falls into the synchronous
                        // walk with a non-I2S peri and arms nothing; the
                        // transfer would then never stream (observed: the IN
                        // link holds START pre-burst yet the pump stays idle
                        // until a later rewrite). On silicon order is
                        // irrelevant, so arm here when peri_sel lands on an
                        // I2S peri while its link is already started (and no
                        // cursor is active). Spurious re-arms are harmless:
                        // completed descriptors read owner=0 and the cursor
                        // deactivates on the next pump pass.
                        let ch = (off / 0xC0) as usize;
                        let rem = off % 0xC0;
                        let i2s_peri = |v: u32| {
                            v == crate::gdma::GDMA_I2S0_PERIPH || v == crate::gdma::GDMA_I2S1_PERIPH
                        };
                        if rem == 0xA8 && i2s_peri(value) && self.gdma.out_link_started(ch) {
                            let p = (value - crate::gdma::GDMA_I2S0_PERIPH) as usize;
                            if !self.i2s_dma_out[p].active {
                                self.i2s_dma_out[p] = I2sDma {
                                    active: true,
                                    ch,
                                    desc: self.gdma.out_link_addr(ch),
                                    off: 0,
                                };
                                self.i2s[p].set_tx_dma_pending(true);
                            }
                        } else if rem == 0x48 && i2s_peri(value) && self.gdma.in_link_started(ch) {
                            let p = (value - crate::gdma::GDMA_I2S0_PERIPH) as usize;
                            if !self.i2s_dma_in[p].active {
                                self.i2s_dma_in[p] = I2sDma {
                                    active: true,
                                    ch,
                                    desc: self.gdma.in_link_addr(ch),
                                    off: 0,
                                };
                            }
                        }
                    }
                    0
                } else {
                    self.gdma.read32(off)
                }
            }
            TWAI_BASE => {
                if is_write {
                    // TWAI peripheral clock (EN0 bit 19): TR/SRR requests
                    // don't fire gated (bits masked out, RRB and the rest
                    // still process).
                    let v = if !self.clk_on(false, 19) && off == 0x04 {
                        value & !((1u32 << 0) | (1u32 << 4))
                    } else {
                        value
                    };
                    self.twai.write32(off, v);
                    0
                } else {
                    self.twai.read32(off)
                }
            }
            MCPWM_BASE => {
                if is_write {
                    self.mcpwm.write32(off, value);
                    0
                } else {
                    self.mcpwm.read32(off)
                }
            }
            MCPWM1_BASE => {
                if is_write {
                    self.mcpwm1.write32(off, value);
                    0
                } else {
                    self.mcpwm1.read32(off)
                }
            }
            // Page 0x6000_8000 holds RTC_CNTL (0x000), RTC_IO (0x400),
            // SENS (0x800, the SAR ADC RTC oneshot controller) and
            // RTC_I2C (0xC00); RTC_CNTL's slow-clock timer is modeled
            // (rtc_time_get drives boot timeout loops); RTC_IO and RTC_I2C
            // are modeled as register stores (see rtc_io.rs / rtc_i2c.rs);
            // RTC_MEM is not.
            0x6000_8000 => {
                if in_range!(off, 0x800, 0x400) {
                    // SENS page: the ADC oneshot owns offsets 0x00..0x5C;
                    // the touch block (CONF 0x5C through APPR_STATUS 0xE0)
                    // owns 0x5C..0x100 (see touch.rs; no overlap with any ADC
                    // register, whose highest is SLAVE_ADDR1 @ 0x40).
                    let soff = off - 0x800;
                    if (0x85C..0x900).contains(&off) {
                        if is_write {
                            self.touch.write32(soff, value);
                            0
                        } else {
                            self.touch.read32(soff)
                        }
                    } else if is_write {
                        self.adc.sens_write32(off - 0x800, value);
                        0
                    } else {
                        self.adc.sens_read32(off - 0x800)
                    }
                } else if in_range!(off, 0xC00, 0x100) {
                    // RTC_I2C (LP/I2C) block at offset 0xC00 of this page.
                    if is_write {
                        self.rtc_i2c.write32(off, value);
                        0
                    } else {
                        self.rtc_i2c.read32(off)
                    }
                } else if off < 0x400 {
                    // RTC_CNTL page (0x000..0x400).  The ULP-RISC-V control
                    // block (0x100..0x200) is carved out to the `Ulp` core.
                    if in_range!(off, ULP_OFF_START, ULP_OFF_END - ULP_OFF_START) && off != 0x130 {
                        // RTC_CNTL_SLP_WAKEUP_CAUSE (0x130) lives in RTC_CNTL,
                        // not the ULP block; route it to `Rtc` (deep-sleep).
                        let full = 0x6000_8000 + off;
                        if is_write {
                            self.ulp.write32(full, value);
                            0
                        } else {
                            self.ulp.read32(full)
                        }
                    } else if is_write {
                        self.rtc.write32(off, value);
                        0
                    } else {
                        self.rtc.read32(off)
                    }
                } else {
                    if is_write {
                        self.rtc_io.write32(off, value);
                        0
                    } else {
                        self.rtc_io.read32(off)
                    }
                }
            }
            APB_SARADC_BASE => {
                if is_write {
                    self.adc.apb_write32(off, value);
                    0
                } else {
                    self.adc.apb_read32(off)
                }
            }
            // SENS2 (TRM memory map): SAR_PLL_FORCE_CTRL @ +0x40 is touched
            // by the firmware (rtc_clk PLL power-up + lock poll); +0x4C is
            // the PHY RF-cal done poll (closed PHY ROM `txdc_cal_v70`:
            // l32i.n a12,[a10=0x6000E04C]; bnone a12,a11(=0x1000000),spin
            // — objdump-verified against the wifi-scan ELF; the register
            // itself is undocumented, the address is 0x6000E04C and the
            // polled bit is 24). +0x50 was an earlier misread of the same
            // loop (a2 = 0x6000E050 belonged to a different ROM wait that
            // already passes). The rest of the page reads 0.
            0x6000_E000 => {
                if off == 0x40 {
                    if is_write {
                        self.pll.write32(value);
                        0
                    } else {
                        self.pll.read32()
                    }
                } else if off == 0x4C {
                    // WIFI BRING-UP: TX-DC cal done (bit 24, objdump-verified
                    // against the wifi-scan ELF: closed PHY ROM `txdc_cal_v70`
                    // does `l32i.n a12,[a10=0x6000E04C]` then `bnone
                    // a12,a11(=0x1000000),spin`). The driver RMWs this
                    // register BEFORE arming (0x420819ca: l32i_n; and with
                    // 0xff000000; or 0x00113cf1; s32i_n — the RMW at
                    // 0x420819c1..0x420819dc writes 0x00113cf1|kept-bits),
                    // then polls bit 24 at 0x420819fb/fd. The write arm
                    // latches the one-shot in `Wifi` (first write per
                    // invocation; value-agnostic — the poll loop RMWs every
                    // pass so any value gate re-arms forever, proven live);
                    // the read arm reports bit 24 once and consumes the
                    // latch (immediate — any timed arm stalls under
                    // `step_fast`, proven live; see `Wifi::txdc_read`).
                    // NOTE: `txdc_read` CONSUMES the latch, so a
                    // host/harness read between the arming write and the
                    // firmware poll would swallow the done bit — only the
                    // firmware poll path reads here in production; do NOT
                    // add harness reads of this register.
                    if is_write {
                        self.pll_cal4c = value;
                        self.wifi.txdc_write(value);
                        0
                    } else {
                        self.wifi.txdc_read(self.pll_cal4c)
                    }
                } else if off == 0x50 {
                    0x0700_0000
                } else {
                    0
                }
            }
            TIMG0_BASE | TIMG1_BASE => {
                let n = if dev == TIMG0_BASE { 0 } else { 1 };
                if is_write {
                    self.timg[n].write32(off, value);
                    0
                } else {
                    self.timg[n].read32(off)
                }
            }
            SYSTIMER_BASE => {
                if is_write {
                    self.systimer.write32(off, value);
                    0
                } else {
                    self.systimer.read32(off)
                }
            }
            INT_MATRIX_BASE => {
                if is_write {
                    self.intc.write32(off, value);
                    0
                } else {
                    self.intc.read32(off)
                }
            }
            EXTMEM_BASE => {
                if is_write {
                    self.cache.write32(off, value);
                    // Mirror XIP activity into the SPI flash controllers so
                    // a post-XIP RDID is answered by the PSRAM device (the
                    // silenced flash cannot reply); see `Memspi::xip`.
                    let xip = self.cache.xip_active();
                    self.memspi[0].xip = xip;
                    self.memspi[1].xip = xip;
                    0
                } else {
                    self.cache.read32(off)
                }
            }
            // Shared cache MMU table (512 x u32) sits one page past the
            // EXTMEM control registers (esp32s3_cache.h MMU_TABLE_OFFSET).
            MMU_TABLE_BASE => {
                if is_write {
                    self.cache.mmu_write32(off, value);
                    0
                } else {
                    self.cache.mmu_read32(off)
                }
            }
            EFUSE_BASE => {
                if is_write {
                    self.efuse.write32(off, value);
                    self.refresh_flashenc_mirror();
                    0
                } else {
                    self.efuse.read32(off)
                }
            }
            SHA_BASE => {
                if is_write {
                    // SHA engine clock (EN1 bit 2): trigger writes start a
                    // transform; dropped while gated (no completion event).
                    if !self.clk_on(true, 2)
                        && (off == 0x10 || off == 0x14 || off == 0x1C || off == 0x20)
                    {
                        0
                    } else {
                        self.sha.write32(off, value);
                        0
                    }
                } else {
                    self.sha.read32(off)
                }
            }
            crate::aes::AES_BASE => {
                if is_write {
                    // AES engine clock (EN1 bit 1): TRIGGER starts a transform.
                    if !self.clk_on(true, 1) && off == 0x48 {
                        0
                    } else {
                        self.aes.write32(off, value);
                        0
                    }
                } else {
                    self.aes.read32(off)
                }
            }
            crate::rsa::RSA_BASE => {
                if is_write {
                    // RSA engine clock (EN1 bit 3): MODEXP_START runs it.
                    if !self.clk_on(true, 3) && off == 0x80C && value & 1 != 0 {
                        0
                    } else {
                        self.rsa.write32(off, value);
                        0
                    }
                } else {
                    self.rsa.read32(off)
                }
            }
            crate::ecdsa::ECDSA_BASE => {
                if is_write {
                    self.ecdsa.write32(off, value);
                    0
                } else {
                    self.ecdsa.read32(off)
                }
            }
            crate::hmac::HMAC_BASE => {
                if is_write {
                    // HMAC engine clock (EN1 bit 5): SET_START runs it.
                    if !self.clk_on(true, 5) && off == 0x40 {
                        0
                    } else {
                        self.hmac.write32(off, value);
                        // On SET_PARA_FINISH the engine latches the eFuse key for the
                        // selected key_id into its working key.
                        if off == (crate::hmac::SET_PARA_FINISH_OFF * 4) as u32 {
                            self.hmac.fetch_key(&self.efuse);
                        }
                        0
                    }
                } else {
                    self.hmac.read32(off)
                }
            }
            crate::ds::DS_BASE => {
                if is_write {
                    // DS engine clock (EN1 bit 4): SET_START (word 896,
                    // byte 0xE00) runs the signature.
                    if !self.clk_on(true, 4) && off == 0xE00 && value != 0 {
                        0
                    } else {
                        self.ds.write32(off, value, &self.efuse);
                        0
                    }
                } else {
                    self.ds.read32(off)
                }
            }
            crate::sdmmc::SDMMC_BASE => {
                if is_write {
                    // SD/MMC host clock (EN1 bit 7): a CMD with START set
                    // doesn't issue gated (dropped; other writes process).
                    if !self.clk_on(true, 7) && off == 0x2C && value & (1u32 << 31) != 0 {
                        0
                    } else {
                        self.sdmmc.write32(off, value);
                        // If a CMD started a data transfer with IDMAC enabled, walk
                        // the descriptor ring now (needs DRAM access via the bus).
                        if let Some(xfer) = self.sdmmc.take_idmac() {
                            let mut transfers: Vec<(u32, usize)> = Vec::new();
                            let mut desc = xfer.dbaddr;
                            let mut remaining = xfer.bytcnt as usize;
                            loop {
                                let des0 = self.read32(desc);
                                let des1 = self.read32(desc.wrapping_add(4));
                                let buf = self.read32(desc.wrapping_add(8));
                                let mut size = (des1 & 0x1FFF) as usize;
                                if size == 0 {
                                    size = 4096;
                                }
                                let size = size.min(remaining);
                                transfers.push((buf, size));
                                // Host now owns the descriptor: clear OWN (bit 31).
                                self.write32(desc, des0 & !(1u32 << 31));
                                remaining = remaining.saturating_sub(size);
                                if des0 & (1u32 << 2) != 0 {
                                    break; // LD (last descriptor)
                                }
                                let next = self.read32(desc.wrapping_add(12));
                                if next == 0 {
                                    break;
                                }
                                desc = next;
                            }
                            if xfer.write {
                                // host -> card: gather descriptor buffers from DRAM.
                                let mut data: Vec<u8> = Vec::new();
                                for &(buf, size) in &transfers {
                                    for i in 0..size {
                                        data.push(self.read8(buf.wrapping_add(i as u32)) as u8);
                                    }
                                }
                                self.sdmmc.idmac_store(xfer.lba, &data);
                            } else {
                                // card -> host: scatter storage into descriptor buffers.
                                let data = self.sdmmc.idmac_load(xfer.lba, xfer.bytcnt as usize);
                                let mut off = 0usize;
                                for &(buf, size) in &transfers {
                                    for i in 0..size {
                                        if off < data.len() {
                                            self.write8(
                                                buf.wrapping_add(i as u32),
                                                data[off] as u32,
                                            );
                                            off += 1;
                                        }
                                    }
                                }
                            }
                            self.sdmmc.finish_idmac();
                        }
                        0
                    }
                } else {
                    self.sdmmc.read32(off)
                }
            }
            crate::rng::RNG_BASE => {
                // Shared RNG/WDEV page (both bases are 0x6003_5000): the
                // RNG data register (+0x7C = WDEV_RND_REG) belongs to Rng,
                // everything else to the Wifi WDEV TSF/timer block.
                if off == 0x7C {
                    if is_write {
                        self.rng.write32(off, value);
                    } else {
                        return self.rng.read32(off);
                    }
                    return 0;
                }
                if is_write {
                    self.wifi.write32(crate::memmap::WDEV_BASE, off, value);
                    0
                } else {
                    self.wifi.read32(crate::memmap::WDEV_BASE, off)
                }
            }
            // SYSTEM peripheral (0x600C0000): only APPCPU_CTRL_A @ +0x04 is
            // modeled — the APP-CPU release register (the ROM's
            // ets_set_appcpu_boot_addr stores the core-1 entry here and the
            // core-1 reset path jumps to it; TRM SYSTEM_APPCPU_CTRL_A, bit
            // 31 = boot-addr-valid).  The app's system_early_init RMWs
            // CPU_PER_CONF @ +0x00 but never reads it back for a branch, so
            // the rest of the page stays unmodeled.
            0x600C_0000 => {
                if off == 0x004 {
                    if is_write {
                        self.appcpu_ctrl_a = value;
                        0
                    } else {
                        self.appcpu_ctrl_a
                    }
                } else if off == 0x018 || off == 0x01C {
                    // Peripheral clock gates (SYSTEM_PERIP_CLK_EN0/1): stored
                    // RMW-able words driving `clk_on` gating below. EN1
                    // writes also shadow the USB_DEVICE clock (bit 10) into
                    // the CDC controller for its TX hold.
                    if is_write {
                        if off == 0x018 {
                            self.sys_clk_en0 = value;
                        } else {
                            self.sys_clk_en1 = value;
                            self.usb.set_sys_clk(value & (1 << 10) != 0);
                        }
                        0
                    } else if off == 0x018 {
                        self.sys_clk_en0
                    } else {
                        self.sys_clk_en1
                    }
                } else if off == 0x030 || off == 0x034 || off == 0x038 || off == 0x03C {
                    // Cross-core interrupt: write 1 asserts the FROM_CPU
                    // source for the target core (+0x30/+0x38 -> core 0 as
                    // sources 79/81; +0x34/+0x3C -> core 1 as sources
                    // 80/82); the ISR writes 0 to deassert (esp_crosscore_isr
                    // clears its own core's reg, esp_ipc_isr_handler clears
                    // FROM_CPU_2/3).  TRM SYSTEM_CPU_INT_FROM_CPU_*.
                    let idx = ((off - 0x030) / 4) as usize;
                    if is_write {
                        self.cpu_int_from_cpu[idx] = value & 1;
                        0
                    } else {
                        self.cpu_int_from_cpu[idx]
                    }
                } else {
                    0
                }
            }
            // ── P5 register-store peripherals (see regstore.rs). Each is a
            //    dedicated 4 KB APB page; pokes are retained and read back.
            0x6000_F000 => {
                if is_write {
                    self.i2s[0].write32(off, value);
                    0
                } else {
                    self.i2s[0].read32(off)
                }
            }
            0x6002_D000 => {
                if is_write {
                    self.i2s[1].write32(off, value);
                    0
                } else {
                    self.i2s[1].read32(off)
                }
            }
            0x6002_6000 => store_dispatch(is_write, off, value, &mut self.syscon),
            0x6002_A000 => store_dispatch(is_write, off, value, &mut self.peri_backup),
            0x6004_1000 => {
                if is_write {
                    // LCD_CAM clock (EN1 bit 8): LCD_START / CAM_START
                    // don't fire gated (bits masked out, config still lands).
                    let v = if !self.clk_on(true, 8) {
                        match off {
                            0x14 => value & !(1u32 << 27),
                            0x08 => value & !(1u32 << 29),
                            _ => value,
                        }
                    } else {
                        value
                    };
                    self.lcd_cam.write32(off, v);
                    // Retroactive CAM DMA arm (mirrors the I2S pump arm):
                    // firmware may start the IN link before CAM_START, so
                    // arm here when CAM_START lands while its link already
                    // runs (spurious re-arms are harmless: an active
                    // cursor ignores them).
                    if off == 0x08
                        && v & (1u32 << 29) != 0
                        && !self.cam_dma_in.active
                        && self.cam_link_fresh
                    {
                        for ch in 0..5 {
                            if self.gdma.in_link_started(ch)
                                && self.gdma.in_peri_sel(ch) == crate::gdma::GDMA_LCD_PERIPH
                            {
                                self.cam_dma_in = CamDma {
                                    active: true,
                                    ch,
                                    desc: self.gdma.in_link_addr(ch),
                                    off: 0,
                                };
                                self.lcd_cam.set_dma_active(true);
                                self.cam_link_fresh = false;
                                break;
                            }
                        }
                    }
                    0
                } else {
                    self.lcd_cam.read32(off)
                }
            }
            0x600C_1000 => store_dispatch(is_write, off, value, &mut self.sensitive),
            // WiFi radio blocks (TEMP WIFI BRING-UP scaffold, see wifi.rs):
            // FE/FE2 @ 0x60006000/0x60005000, BB @ 0x6001D000,
            // NRX @ 0x6001CC00, MAC @ 0x6001C000, MAC-CTRL @ 0x60033000.
            // (WDEV @ 0x60035000 shares the RNG page and is routed by the
            // RNG arm above, not here.) Plain stores + proven RF-cal
            // done-bits.
            FE_BASE | FE2_BASE | BB_BASE | NRX_BASE | WIFI_MAC_BASE | WIFI_MAC_CTRL_BASE => {
                if is_write {
                    self.wifi.write32(dev, off, value);
                    0
                } else {
                    self.wifi.read32(dev, off)
                }
            }
            0x600C_E000 => store_dispatch(is_write, off, value, &mut self.assist_debug),
            USB_OTG_BASE => {
                // DWC core (0x60080000).
                let base = 0;
                if is_write {
                    // USB-OTG controller clock (EN0 bit 23): engine actions
                    // don't fire gated (bits masked out, config still lands)
                    // — GRSTCTL core-soft-reset, HPRT power/reset (device
                    // connect/port enable), and host-channel ChEna/ChDis
                    // (transfer run/halt). DFIFO staging + plain registers
                    // are memory-like and always land.
                    use crate::usb_otg::{
                        GRST_CSFTRST, GRSTCTL, HC_BASE, HC_COUNT, HC_STRIDE, HCCHAR_CHDIS,
                        HCCHAR_CHENA, HCCHAR_OFF, HPRT, HPRT_PWR, HPRT_RST,
                    };
                    let v = if !self.clk_on(false, 23) && base == 0 {
                        if off == GRSTCTL {
                            value & !GRST_CSFTRST
                        } else if off == HPRT {
                            value & !(HPRT_PWR | HPRT_RST)
                        } else if off >= HC_BASE
                            && off < HC_BASE + HC_COUNT as u32 * HC_STRIDE
                            && (off - HC_BASE) % HC_STRIDE == HCCHAR_OFF
                        {
                            value & !(HCCHAR_CHENA | HCCHAR_CHDIS)
                        } else {
                            value
                        }
                    } else {
                        value
                    };
                    self.usb_otg.write32(base + off, v);
                    0
                } else {
                    self.usb_otg.read32(base + off)
                }
            }
            dev if (USB_OTG_FIFO_PAGE..USB_OTG_FIFO_PAGE + USB_OTG_FIFO_PAGES * 0x1000)
                .contains(&dev) =>
            {
                // DWC per-endpoint IN TXFIFO pages (EPn-IN @ +0x1000*n).
                // The generic dispatch only routes one page per arm, so a
                // ranged arm covers EP0..EP5-IN (see USB_OTG_FIFO_PAGES).
                let ep = ((dev - USB_OTG_FIFO_PAGE) / 0x1000) as usize;
                if is_write {
                    self.usb_otg.write_dfifo(ep, off, value);
                    0
                } else {
                    self.usb_otg.read_dfifo(ep, off)
                }
            }
            // USB_WRAP (OTG PHY wrapper, 0x60039000): plain store.
            0x6003_9000 => store_dispatch(is_write, off, value, &mut self.usb_wrap),
            0x600D_0000 => store_dispatch(is_write, off, value, &mut self.wcl),
            // Everything else in the APB space: no model yet (reads 0 /
            // writes dropped, like QEMU's unimplemented devices).
            _ => 0,
        }
    }
}

impl Default for Soc {
    fn default() -> Self {
        Self::new()
    }
}

/// Route a single 32-bit MMIO access to a `RegStore` (register-store stub).
/// Returns the read value (or 0 for writes), per the bus contract.
fn store_dispatch(is_write: bool, off: u32, value: u32, store: &mut RegStore) -> u32 {
    if is_write {
        store.write32(off, value);
        0
    } else {
        store.read32(off)
    }
}

impl Soc {
    /// Host WiFi fixture frontends (scan completion, STA connect, event
    /// injection) live in the main `impl Soc` block above/below this line,
    /// next to `find_task_by_name`, `queue_post_raw`, and
    /// `queue_unblock_receiver`, which they build on.
    /// True if any device requested a hard reset (e.g. a WDT stage action of
    /// reset-CPU/system). The machine consumes this each step to reboot.
    pub fn consume_reset(&mut self) -> bool {
        self.timg[0].consume_reset() || self.timg[1].consume_reset() || self.rtc.consume_reset()
    }

    /// True if firmware requested a sleep (wrote `RTC_CNTL_SLEEP_EN`).
    /// Returns the captured sleep duration (slow-clock ticks) plus the
    /// deep/light kind, and clears the flag. The machine consumes this each
    /// step to fast-forward the sleep.
    ///
    /// Wake-source evaluation happens here (all state is visible: RTC
    /// registers, GPIO levels, ULP run state): EXT0/EXT1 level triggers met
    /// at entry set their cause bits immediately; the timer runs its
    /// programmed period; a running ULP with wakeup armed is watched for
    /// its halt edge during the fast-forward (`sleep_ulp_fired`). The
    /// computed cause + EXT1 status are stashed for `wake()` (which reboots
    /// before applying them). With no timer and no immediate trigger the
    /// budget is unbounded — the sleep ends on a watched event.
    pub fn consume_sleep_request(&mut self) -> Option<(u64, bool)> {
        let (target, deep) = self.rtc.consume_sleep_request()?;
        let ena = self.rtc.wakeup_state();
        let timer_armed = ena & crate::rtc::wakeup_ena(3) != 0 || self.rtc.slp_timer_written();
        let mut cause = 0;
        let mut ext1 = 0;
        let pads = self.gpio_in_readback();
        let pad_level = |rtc_pad: u32| -> u32 {
            // S3 RTC pads 0..21 map 1:1 to GPIO0..21 (verified: EXT0 SEL reads
            // back the programmed GPIO number; sketches use low pins).
            if rtc_pad < 22 {
                (pads >> rtc_pad) & 1
            } else {
                0
            }
        };
        // EXT0: single RTC pad at the programmed level.
        if ena & crate::rtc::wakeup_ena(0) != 0 {
            let sel = (self.rtc_io.read32(0x4DC) >> 27) & 0x1F;
            let lv = (self.rtc.ext_conf() >> 30) & 1;
            if pad_level(sel) == lv {
                cause |= crate::rtc::CAUSE_EXT0;
            }
        }
        // EXT1: RTC-pad mask at the programmed level (ANY_HIGH when LV=1,
        // ALL_LOW when LV=0 — unified trigger mode, rtc_cntl_ll.h).
        if ena & crate::rtc::wakeup_ena(1) != 0 {
            let mask = self.rtc.ext1_sel();
            let lv = (self.rtc.ext_conf() >> 31) & 1;
            let mut trig = 0u32;
            for pad in 0..22 {
                if mask & (1 << pad) != 0 && pad_level(pad) == lv {
                    trig |= 1 << pad;
                }
            }
            let fired = if lv == 1 {
                trig != 0
            } else {
                mask != 0 && trig == (mask & 0x3F_FFFF)
            };
            if fired {
                cause |= crate::rtc::CAUSE_EXT1;
                ext1 = trig;
            }
        }
        // GPIO wakeup (WAKEUP_ENA bit 2, light-sleep RTC-GPIO wakeup
        // via esp_sleep_enable_gpio_wakeup + rtc_gpio_wakeup_enable):
        // any RTC pin 0..21 with PINn WAKEUP_ENABLE (RTCIO PINn_REG bit
        // 10, rtc_io_reg.h) whose INT_TYPE[9:7] matches the pad level wakes
        // immediately. Level types are exact; edge types approximate (the
        // fast-forward freezes inputs, so the entry level stands in for
        // the edge: rising = high, falling = low, any = fire).
        // GPIO/UART wakeups are light-sleep-only (esp_sleep_enable_gpio/
        // uart_wakeup have no effect in deep sleep — the digital core that
        // would sense them is powered down).
        if !deep && ena & crate::rtc::wakeup_ena(2) != 0 {
            for pin in 0..22 {
                let cfg = self.rtc_io.read32(0x428 + pin * 4);
                if cfg & (1 << 10) == 0 {
                    continue;
                }
                let lv = pad_level(pin);
                let fire = match (cfg >> 7) & 7 {
                    1 | 5 => lv == 1, // rising edge (past) / high level
                    2 | 4 => lv == 0, // falling edge (past) / low level
                    3 => true,        // any edge (see above)
                    _ => false,       // disabled
                };
                if fire {
                    cause |= crate::rtc::CAUSE_GPIO;
                    break;
                }
            }
        }
        // UART0/UART1 wakeup (WAKEUP_ENA bits 6/7): pending RX bytes at
        // entry wake immediately (the line already signaled).
        // Level (FIFO holds bytes) or the sticky pre-entry edge both
        // wake; the edge is consumed whether or not it fires (entry reset
        // semantics: a stale edge does not survive past one sleep entry).
        let uart_edge0 = self.uarts[0].take_rx_edge();
        let uart_edge1 = self.uarts[1].take_rx_edge();
        if ena & crate::rtc::wakeup_ena(6) != 0 && (self.uarts[0].rx_pending() || uart_edge0) {
            cause |= crate::rtc::CAUSE_UART0;
        }
        if ena & crate::rtc::wakeup_ena(7) != 0 && (self.uarts[1].rx_pending() || uart_edge1) {
            cause |= crate::rtc::CAUSE_UART1;
        }
        if timer_armed {
            cause |= crate::rtc::CAUSE_TIMER;
        }
        // Touch-pad wakeup (WAKEUP_ENA bit 8): any touched pad (threshold
        // programmed and counter below it — the same live condition the
        // touch STATUS path reports) wakes immediately with the touch
        // cause. The sleep-mode SET1/SET2 trigger-source selection is not
        // modeled (any pad wakes, matching the default BOTH configuration).
        if ena & crate::rtc::wakeup_ena(8) != 0 && self.touch.any_touched() {
            cause |= crate::rtc::CAUSE_TOUCH;
            // Capture the triggering pad's counter for SLP_STATUS.
            self.touch.set_sleep_data();
        }
        // ULP halt edge is watched during the fast-forward (the ULP keeps
        // ticking while the CPUs halt); nothing to set yet. Either ULP
        // trigger (FSM bit 9, COCPU bit 11) arms the watch when the ULP is
        // running; the matching cause bit latches on its halt.
        self.sleep_ulp_watch = 0;
        if self.ulp.is_running() {
            if ena & crate::rtc::wakeup_ena(9) != 0 {
                self.sleep_ulp_watch |= crate::rtc::CAUSE_ULP;
            }
            if ena & crate::rtc::wakeup_ena(11) != 0 {
                self.sleep_ulp_watch |= crate::rtc::CAUSE_COCPU;
            }
            if ena & crate::rtc::wakeup_ena(13) != 0 {
                self.sleep_ulp_watch |= crate::rtc::CAUSE_TRAP;
            }
        }
        self.sleep_cause = cause;
        self.sleep_ext1 = ext1;
        if timer_armed {
            Some((target, deep))
        } else if cause != 0 {
            Some((1, deep)) // immediate trigger: wake on the next step
        } else {
            Some((u64::MAX, deep)) // ULP edge or nothing: fast-forward until an event
        }
    }

    /// Wake cause stashed at sleep entry (applied by the machine on wake).
    pub fn take_sleep_cause(&mut self) -> u32 {
        core::mem::take(&mut self.sleep_cause)
    }

    /// EXT1 triggering pads stashed at sleep entry.
    pub fn take_sleep_ext1(&mut self) -> u32 {
        core::mem::take(&mut self.sleep_ext1)
    }

    /// True once when a watched ULP halts mid-sleep (latches the ULP cause).
    /// Call each fast-forward step; edge-triggered (no repeat wakeups).
    pub fn sleep_ulp_fired(&mut self) -> bool {
        if self.sleep_ulp_watch != 0 && !self.ulp.is_running() {
            self.sleep_cause |= core::mem::take(&mut self.sleep_ulp_watch);
            true
        } else {
            false
        }
    }

    /// Debug accessor: has the firmware requested a deep-sleep (pending consume)?
    pub fn rtc_sleep_req(&self) -> bool {
        self.rtc.sleep_req()
    }

    /// I2S GDMA streaming pump: move words between owned descriptors and
    /// the TX/RX FIFOs as space/data allow, raising the per-descriptor EOF
    /// event (address + done interrupt) as their lengths fill. Called once
    /// per `tick_timers` while any I2S link is armed.
    /// LCD_CAM GDMA-RX streaming pump: move staged capture words into
    /// the active IN descriptor chain as they arrive, raising the
    /// per-descriptor EOF event. A filled descriptor is handed back
    /// (owner cleared, single-shot like the sync walk); owner=0 or a null
    /// next parks the cursor.
    fn poll_cam_dma(&mut self) {
        // Capture over with nothing staged and no fresh link awaiting
        // capture: park (later-streamed words fall back to the RX FIFO,
        // so DMA-then-poll mixing on one capture works). The freshness guard is
        // load-bearing: a cursor armed by an IN-link start must survive
        // until CAM_START begins streaming (parking it early routes the
        // frame to the RX FIFO and the descriptor never fills).
        // Short-frame tail: the capture ended with a partial fill
        // outstanding — complete it with the actual length (owner
        // cleared, done raised) instead of hanging the descriptor.
        if self.lcd_cam.take_dma_eof() && self.cam_dma_in.active {
            let desc = self.cam_dma_in.desc;
            if desc != 0 {
                let dw0 = self.read32(desc);
                if dw0 & (1u32 << 31) != 0 {
                    let ch = self.cam_dma_in.ch;
                    self.gdma.set_in_eof_des_addr(ch, desc);
                    self.gdma.raise_in_done(ch);
                    self.write32(desc, dw0 & !(1u32 << 31));
                }
            }
            self.cam_dma_in.active = false;
            self.cam_link_fresh = false;
            self.lcd_cam.set_dma_active(false);
            return;
        }
        if !self.cam_link_fresh && !self.lcd_cam.is_capturing() && self.lcd_cam.dma_pending() == 0 {
            self.cam_dma_in.active = false;
            self.cam_link_fresh = false;
            self.lcd_cam.set_dma_active(false);
            return;
        }
        for _ in 0..8 {
            let (desc, off) = {
                let c = &self.cam_dma_in;
                (c.desc, c.off)
            };
            if desc == 0 {
                self.cam_dma_in.active = false;
                self.cam_link_fresh = false;
                self.lcd_cam.set_dma_active(false);
                break;
            }
            let dw0 = self.read32(desc);
            let len = (dw0 >> 12) & 0xFFF;
            let owner = (dw0 >> 31) & 1;
            if owner == 0 {
                self.cam_dma_in.active = false;
                self.cam_link_fresh = false;
                self.lcd_cam.set_dma_active(false);
                break;
            }
            let buf = self.read32(desc + 4);
            let next = self.read32(desc + 8);
            let mut off = off;
            while self.lcd_cam.dma_pending() > 0 && off + 4 <= len {
                let w = self.lcd_cam.dma_pop().unwrap_or(0);
                self.write32(buf + off, w);
                off += 4;
            }
            self.cam_dma_in.off = off;
            if off + 4 > len {
                let ch = self.cam_dma_in.ch;
                self.gdma.set_in_eof_des_addr(ch, desc);
                self.gdma.raise_in_done(ch);
                self.write32(desc, dw0 & !(1u32 << 31));
                if next == 0 {
                    self.cam_dma_in.active = false;
                    self.cam_link_fresh = false;
                    self.lcd_cam.set_dma_active(false);
                    break;
                }
                self.cam_dma_in.desc = next;
                self.cam_dma_in.off = 0;
            } else {
                break; // staging dry; resume next tick
            }
        }
    }

    fn poll_i2s_dma(&mut self) {
        for port in 0..2 {
            // OUT (TX): DRAM -> TX FIFO.
            if self.i2s_dma_out[port].active {
                for _ in 0..8 {
                    let (desc, off) = {
                        let s = &self.i2s_dma_out[port];
                        (s.desc, s.off)
                    };
                    if desc == 0 {
                        self.i2s_dma_out[port].active = false;
                        break;
                    }
                    let dw0 = self.read32(desc);
                    let len = (dw0 >> 12) & 0xFFF;
                    let owner = (dw0 >> 31) & 1;
                    if owner == 0 {
                        self.i2s_dma_out[port].active = false;
                        break;
                    }
                    let buf = self.read32(desc + 4);
                    let next = self.read32(desc + 8);
                    let mut off = off;
                    while self.i2s[port].tx_space() > 0 && off + 4 <= len {
                        let w = self.read32(buf + off);
                        self.i2s[port].tx_push(w);
                        off += 4;
                    }
                    self.i2s_dma_out[port].off = off;
                    if off + 4 > len {
                        let ch = self.i2s_dma_out[port].ch;
                        // Per-descriptor EOF event (the IDF TX callback
                        // recycles the buffer into the free queue on each
                        // one). EOF does NOT stop the stream: GDMA keeps
                        // walking while descriptors are owned (the S3 TX
                        // channel streams continuously once enabled, repeating
                        // auto-cleared silence when idle); only owner=0 or a
                        // null next parks the cursor. (An earlier revision
                        // stopped at eof and cleared owner, which truncated
                        // every multi-descriptor IDF transfer after its first
                        // buffer.)
                        self.gdma.set_out_eof_des_addr(ch, desc);
                        self.gdma.raise_out_done(ch);
                        if next == 0 || owner == 0 {
                            self.i2s_dma_out[port].active = false;
                            break;
                        }
                        self.i2s_dma_out[port].desc = next;
                        self.i2s_dma_out[port].off = 0;
                    } else {
                        break; // FIFO full; resume next tick
                    }
                }
            }
            // IN (RX): RX FIFO -> DRAM.
            if self.i2s_dma_in[port].active {
                for _ in 0..8 {
                    let (desc, off) = {
                        let s = &self.i2s_dma_in[port];
                        (s.desc, s.off)
                    };
                    if desc == 0 {
                        self.i2s_dma_in[port].active = false;
                        break;
                    }
                    let dw0 = self.read32(desc);
                    let len = (dw0 >> 12) & 0xFFF;
                    let owner = (dw0 >> 31) & 1;
                    if owner == 0 {
                        self.i2s_dma_in[port].active = false;
                        break;
                    }
                    let buf = self.read32(desc + 4);
                    let next = self.read32(desc + 8);
                    let mut off = off;
                    while self.i2s[port].rx_ready() > 0 && off + 4 <= len {
                        let w = self.i2s[port].rx_pop();
                        self.write32(buf + off, w);
                        off += 4;
                    }
                    self.i2s_dma_in[port].off = off;
                    if off + 4 > len {
                        let ch = self.i2s_dma_in[port].ch;
                        // Per-descriptor EOF event (see OUT above): EOF does
                        // not stop the stream; only owner=0 or null next
                        // parks the cursor (an overrun then just keeps
                        // overwriting, like silicon).
                        self.gdma.set_in_eof_des_addr(ch, desc);
                        self.gdma.raise_in_done(ch);
                        if next == 0 || owner == 0 {
                            self.i2s_dma_in[port].active = false;
                            break;
                        }
                        self.i2s_dma_in[port].desc = next;
                        self.i2s_dma_in[port].off = 0;
                    } else {
                        break; // FIFO drained; resume next tick
                    }
                }
            }
            self.i2s[port].set_tx_dma_pending(self.i2s_dma_out[port].active);
        }
    }

    /// Record the wakeup-cause bits read by `esp_sleep_get_wakeup_cause`
    /// after a deep-sleep reboot (machine writes this on wake).
    pub fn set_sleep_wakeup_cause(&mut self, bits: u32) {
        self.rtc.set_wakeup_cause(bits);
    }

    /// Latch the RTC SLP_WAKEUP interrupt (machine writes this on a
    /// light-sleep wake so `rtc_sleep_start`'s INT_RAW spin exits).
    pub fn set_sleep_wakeup_int(&mut self) {
        self.rtc.set_sleep_wakeup();
    }

    /// Record the reset causes read by the live ROM's
    /// `esp_rom_get_reset_reason` (machine writes this on wake).
    pub fn set_reset_cause(&mut self, pro: u32, app: u32) {
        self.rtc.set_reset_cause(pro, app);
    }

    /// Record the EXT1 triggering pads for `esp_sleep_get_ext1_wakeup_status`.
    pub fn set_ext1_status(&mut self, pads: u32) {
        self.rtc.set_ext1_status(pads);
    }

    /// Snapshot of RTC-retained state across a deep-sleep reboot (silicon
    /// keeps RTC slow/fast memory + ULP state over sleep; only the causes
    /// are refreshed on wake).
    pub fn snapshot_rtc(&self) -> RtcRetain {
        RtcRetain {
            slow: self.rtc_slow.clone(),
            fast: self.rtc_fast.clone(),
            ulp: self.ulp.clone(),
            touch: self.touch.clone(),
        }
    }

    /// Snapshot digital-GPIO state for pad-hold retention (deep sleep):
    /// OUT + ENABLE words, all FUNC_OUT_SEL values, and the live
    /// DIG_PAD_HOLD mask (only masked pins 0..31 are restored).
    pub fn snapshot_gpio_hold(&self) -> ([u32; 2], [u32; 54], u32) {
        let (out, en, func) = self.gpio.hold_snapshot();
        ([out, en], func, self.rtc.dig_pad_hold())
    }

    /// Restore held pads after a deep-sleep reboot (see DIG_PAD_HOLD).
    pub fn restore_gpio_hold(&mut self, snap: ([u32; 2], [u32; 54], u32)) {
        let ([out, en], func, mask) = snap;
        let mut rout = 0u32;
        let mut ren = 0u32;
        let mut rfunc = [0x80u32; 54];
        // Fresh-Soc defaults: OUT/ENABLE 0, FUNC_OUT_SEL 0x80.
        for i in 0..32 {
            if mask & (1 << i) != 0 {
                rout |= out & (1 << i);
                ren |= en & (1 << i);
            }
        }
        for (i, f) in func.iter().enumerate() {
            if i < 32 && mask & (1 << i) != 0 {
                rfunc[i] = *f;
            }
        }
        self.gpio.hold_restore(rout, ren, &rfunc);
    }

    /// Restore RTC-retained state after a deep-sleep reboot.
    pub fn restore_rtc(&mut self, s: RtcRetain) {
        self.rtc_slow = s.slow;
        self.rtc_fast = s.fast;
        self.ulp = s.ulp;
        self.touch = s.touch;
    }

    /// Debug accessor for the AES interrupt raw&enabled state (validation harness).
    pub fn aes_debug_int(&self) -> (u32, u32) {
        self.aes.debug_int()
    }

    /// Debug accessor for the GDMA interrupt-pending state (validation harness).
    pub fn gdma_int_pending(&self) -> bool {
        self.gdma.int_pending()
    }

    /// TEMP I2S-driver probe (remove): completed-walk counters.
    pub fn gdma_dbg_walks(&self) -> (u64, u64) {
        (self.gdma.dbg_walks_out, self.gdma.dbg_walks_in)
    }

    /// TEMP I2S-driver probe (remove): recent I2S CONF writes.
    pub fn i2s_conf_writes(&self, idx: usize) -> alloc::vec::Vec<(u32, u32)> {
        self.i2s[idx].dbg_writes.clone()
    }

    /// TEMP I2S-driver probe (remove): recent raw GDMA writes.
    pub fn gdma_debug_log(&self) -> alloc::vec::Vec<(u32, u32)> {
        self.gdma.debug_log()
    }

    /// Debug accessor for the I2C interrupt status (INT_RAW & INT_ENA).
    pub fn i2c_int_st(&self, n: usize) -> u32 {
        self.i2c[n].int_st()
    }

    /// Debug accessor for the I2C raw interrupt bits (INT_RAW).
    pub fn i2c_int_raw(&self, n: usize) -> u32 {
        self.i2c[n].int_raw()
    }

    /// Debug: read 4 bytes LE from DRAM (validation harness).
    pub fn read_dram(&self, addr: u32) -> u32 {
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE) {
            let o = (addr - DRAM_BASE) as usize;
            u32::from_le_bytes([
                self.sram[o],
                self.sram[o + 1],
                self.sram[o + 2],
                self.sram[o + 3],
            ])
        } else {
            0
        }
    }

    /// Debug: per-channel GDMA interrupt/peri state (validation harness).
    pub fn gdma_debug(&self) -> Vec<(u32, u32, u32, u32, u32, u32)> {
        self.gdma.debug_state()
    }

    /// Debug: recent raw GDMA writes (validation harness).
    pub fn gdma_log(&self) -> Vec<(u32, u32)> {
        self.gdma.debug_log()
    }

    /// Scan all peripheral interrupt status registers and build the source
    /// bitmap (excluding per-cpu cross-core bits).  Called from `int_pending()`
    /// when the per-step cache is invalid.
    fn scan_peripheral_sources(&mut self) -> u128 {
        // Peripheral sources asserted per the TRM interrupt-source table
        // (esp32s3 interrupts.h ETS_*_INTR_SOURCE numbers): UART0/1/2 =
        // 27/28/29, TIMG0 T0/T1/WDT = 50/51/52, TIMG1 T0/T1/WDT =
        // 53/54/55, SYSTIMER target0/1/2 = 57/58/59 (56 = CACHE_IA — the
        // cache-invalid-access source, NOT a systimer source!).  Each
        // peripheral gates its line on INT_ST = RAW & ENA (QEMU
        // esp32_timg.c / esp32_uart.c update_irq); the matrix then
        // resolves the asserted sources to the requesting CPU's lines
        // (per-CPU core_0/core_1 maps, TRM interrupt matrix).  u128 bitmap:
        // sources 94/95 sit beyond u64.
        let mut src = 0u128;
        for (i, u) in self.uarts.iter().enumerate() {
            if u.int_st() != 0 {
                src |= 1 << (27 + i);
            }
        }
        // GPIO edge/level interrupt (ETS_GPIO_INTR_SOURCE = 16).
        if self.gpio.int_pending() {
            src |= 1 << crate::gpio::GPIO_INTR_SOURCE;
        }
        for (g, t) in self.timg.iter().enumerate() {
            let st = t.int_st();
            let base = 50 + g * 3;
            if st & INT_T0 != 0 {
                src |= 1 << base;
            }
            if st & INT_T1 != 0 {
                src |= 1 << (base + 1);
            }
            if st & INT_WDT != 0 {
                src |= 1 << (base + 2);
            }
        }
        let sst = self.systimer.int_st();
        for n in 0..3 {
            if sst & (1 << n) != 0 {
                src |= 1 << (57 + n);
            }
        }
        for (i, ic) in self.i2c.iter().enumerate() {
            if ic.int_st() != 0 {
                src |= 1 << (42 + i);
            }
        }
        for (i, s) in self.spi.iter().enumerate() {
            if s.int_st() != 0 {
                src |= 1 << (21 + i);
            }
        }
        if self.rmt.int_st() != 0 {
            src |= 1 << crate::rmt::RMT_INTR_SOURCE;
        }
        if self.pcnt.int_st() != 0 {
            src |= 1 << crate::pcnt::PCNT_INTR_SOURCE;
        }
        // RTC core (ETS_RTC_CORE_INTR_SOURCE = 39): touch DONE/SCAN_DONE
        // (oneshot completion event) and friends, INT_ST = RAW & ENA.
        if self.rtc.int_st() != 0 {
            src |= 1 << crate::rtc::RTC_CORE_INTR_SOURCE;
        }
        // ADC digital done (`ETS_APB_ADC_INTR_SOURCE` = 65): the done flags
        // live in the APB block's own INT_ST (polled drivers never enable).
        if self.adc.int_st() != 0 {
            src |= 1 << crate::adc::APB_ADC_INTR_SOURCE;
        }
        // GDMA channels have per-channel sources (ETS_DMA_IN_CH0..4 =
        // 66..70, ETS_DMA_OUT_CH0..4 = 71..75) on the single shared
        // controller (all peripherals incl. AES/SHA allocate channels here).
        for ch in 0..crate::gdma::NCH {
            if self.gdma.in_int_st(ch) != 0 {
                src |= 1 << (crate::gdma::GDMA_IN_INTR_BASE + ch as u32);
            }
            if self.gdma.out_int_st(ch) != 0 {
                src |= 1 << (crate::gdma::GDMA_OUT_INTR_BASE + ch as u32);
            }
        }
        if self.aes.int_pending() {
            src |= 1 << 77;
        }
        // SHA done (`ETS_SHA_INTR_SOURCE` = 78): latched per transform,
        // level-gated by INT_ENA (polling drivers never enable it).
        if self.sha.int_st() != 0 {
            src |= 1 << crate::sha::SHA_INTR_SOURCE;
        }
        if self.twai.int_pending() {
            src |= 1 << crate::twai::TWAI_INTR_SOURCE;
        }
        if self.mcpwm.int_pending() {
            src |= 1 << crate::mcpwm::MCPWM_INTR_SOURCE;
        }
        if self.mcpwm1.int_pending() {
            src |= 1 << crate::mcpwm::MCPWM1_INTR_SOURCE;
        }
        if self.rsa.int_pending() {
            src |= 1 << crate::rsa::RSA_INTR_SOURCE;
        }
        // NOTE: ECDSA has no matrix source on the S3 (polled via RESULT,
        // like HMAC/DS) — deliberately unwired.
        if self.lcd_cam.int_st() != 0 {
            src |= 1 << crate::lcd_cam::LCD_CAM_INTR_SOURCE;
        }
        if self.i2s[0].int_st() != 0 {
            src |= 1 << crate::i2s::I2S0_INTR_SOURCE;
        }
        if self.i2s[1].int_st() != 0 {
            src |= 1 << crate::i2s::I2S1_INTR_SOURCE;
        }
        if self.sdmmc.int_st() != 0 {
            src |= 1 << crate::sdmmc::SDMMC_INTR_SOURCE;
        }
        // USB-OTG DWC core (quiet without a host counterparty).
        if self.usb_otg.int_pending() {
            src |= 1 << crate::usb_otg::USB_OTG_INTR_SOURCE;
        }
        // UHCI0 DMA bridge event interrupts (TX/RX_START via INT_ST).
        if self.uhci.int_st() {
            src |= 1 << crate::uhci::UHCI0_INTR_SOURCE;
        }
        if self.ledc.int_st() != 0 {
            src |= 1 << crate::ledc::LEDC_INTR_SOURCE;
        }
        if self.usb.int_pending() {
            src |= 1 << USB_SERIAL_JTAG_INTR_SOURCE;
        }
        self.cached_src = src;
        self.src_valid = true;
        src
    }
}

impl Bus for Soc {
    /// Atomic compare-and-swap WITHOUT an interleaving peripheral tick.
    /// `s32c1i` (portMUX spinlocks) must read-compare-write as one bus
    /// transaction: the default trait impl (read32, maybe write32) runs
    /// `tick_timers` between the two only if the caller ticks — but the
    /// Xtensa `step` path calls `int_pending` (which scans peripherals
    /// but does not tick)... the REAL hazard is `run_fast_core`'s
    /// `fast_maybe_tick`, which ticks INSIDE multi-op blocks. The exec
    /// calls this hook directly, so no tick can slip between read and
    /// write here regardless of driver. DRAM/IRAM fast paths only; MMIO
    /// CAS falls back to read+write (device regs are not spinlock
    /// words — no firmware CASes them).
    #[inline(always)]
    fn cas32(&mut self, addr: u32, compare: u32, val: u32) -> u32 {
        let addr = ioblock_remap(addr);
        // Fast path: aligned DRAM word (spinlock words live here).
        if addr & 3 == 0 && in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE) {
            let o = (addr - DRAM_BASE) as usize;
            // SAFETY: o < SRAM_BASE_RANGE <= SRAM_BYTES, checked by in_range
            let old = u32::from_le_bytes(unsafe {
                [
                    *self.sram.get_unchecked(o),
                    *self.sram.get_unchecked(o + 1),
                    *self.sram.get_unchecked(o + 2),
                    *self.sram.get_unchecked(o + 3),
                ]
            });
            if old == compare {
                self.sram[o..o + 4].copy_from_slice(&val.to_le_bytes());
            }
            return old;
        }
        // IRAM-windowed DRAM (same backing, instruction-side alias).
        if addr & 3 == 0 && in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE) {
            let o = (addr - IRAM_BASE) as usize;
            if o < SRAM0_SIZE as usize {
                let old = u32::from_le_bytes(unsafe {
                    [
                        *self.iram0.get_unchecked(o),
                        *self.iram0.get_unchecked(o + 1),
                        *self.iram0.get_unchecked(o + 2),
                        *self.iram0.get_unchecked(o + 3),
                    ]
                });
                if old == compare {
                    self.iram0[o..o + 4].copy_from_slice(&val.to_le_bytes());
                }
                return old;
            }
            let o = (DIRAM_DATA_BASE - DRAM_BASE + (addr - DIRAM_INST_BASE)) as usize;
            let old = u32::from_le_bytes([
                self.sram[o],
                self.sram[o + 1],
                self.sram[o + 2],
                self.sram[o + 3],
            ]);
            if old == compare {
                self.sram[o..o + 4].copy_from_slice(&val.to_le_bytes());
            }
            return old;
        }
        // MMIO / flash / RTC: no firmware spinlocks here; plain RMW.
        let old = self.read32(addr);
        if old == compare {
            self.write32(addr, val);
        }
        old
    }

    fn int_pending(&mut self, cpu: usize) -> u32 {
        // Per-step cache: `tick_timers()` invalidates src_valid at the start
        // of each step.  The first int_pending call (core 0) recomputes the
        // full 22-peripheral source bitmap; the second call (core 1) reuses
        // it.  Between the two calls, only cpu[1].step() instructions execute
        // (no tick_timers), so peripheral state is effectively identical.
        // Worst case: a cpu[1] INT_CLR write is delayed 1 step — negligible.
        //
        // The cross-core FROM_CPU bit is per-cpu, so it is NOT cached — it is
        // added separately below after the cached_or_fresh source bitmap.
        let src = if self.src_valid {
            self.cached_src
        } else {
            self.scan_peripheral_sources()
        };

        // Cross-core: SYSTEM.CPU_INT_FROM_CPU_0/1 assert FROM_CPU_INTR0/1
        // = sources 79/80 (FreeRTOS yields); _2/3 assert FROM_CPU_INTR2/3
        // = sources 81/82 (esp_ipc_isr stall/mute handshake).  Regs are
        // per-target-core (+0x30/+0x38 -> core 0, +0x34/+0x3C -> core 1).
        // This is per-cpu so it must not be cached.
        let mut final_src = src;
        if self.cpu_int_from_cpu[cpu] & 1 != 0 {
            final_src |= 1 << (79 + cpu);
        }
        if self.cpu_int_from_cpu[2 + cpu] & 1 != 0 {
            final_src |= 1 << (81 + cpu);
        }
        self.intc.pending_lines(cpu, final_src)
    }

    /// Dedicated-GPIO input channels for `ee.get_gpio_in`: the 8
    /// CORE1_GPIO_IN signals (129..131, 252..255, 54, gpio_sig_map.h)
    /// resolved through the GPIO-matrix input routing against the pad
    /// readback (pins < 32) or pad level, exactly like MCPWM
    /// capture/fault sampling. Unrouted channels read 0.
    fn dedic_gpio_in(&mut self) -> u32 {
        const IN_SIGS: [u32; 8] = [129, 130, 131, 252, 253, 254, 255, 54];
        let rb = self.gpio_in_readback();
        let mut v = 0u32;
        for (c, sig) in IN_SIGS.iter().enumerate() {
            let lvl = match self.gpio.in_sel(*sig) {
                Some((pin, inv)) if pin < 32 => ((rb >> pin) & 1) ^ (inv as u32),
                Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                None => 0,
            };
            v |= (lvl & 1) << c;
        }
        v
    }

    #[inline(always)]
    fn read8(&mut self, addr: u32) -> u32 {
        let addr = ioblock_remap(addr);
        // Host event-block pool (NOT DRAM — outside every heap's bounds by
        // design, so `heap_caps_free` skips these blocks; see `ard_pool`).
        if in_range!(addr, Self::WIFI_ARD_POOL, 768) {
            return self.ard_pool[(addr - Self::WIFI_ARD_POOL) as usize] as u32;
        }
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE)
        {
            self.ram8(addr) as u32
        } else if in_range!(addr, IROM_BASE, IROM_SIZE) {
            self.irom[(addr - IROM_BASE) as usize] as u32
        } else if in_range!(addr, FLASH_DATA_BASE, FLASH_WINDOW_SIZE)
            || in_range!(addr, FLASH_INST_BASE, FLASH_WINDOW_SIZE)
        {
            self.cache_read8(addr) as u32
        } else if in_range!(addr, RTC_SLOW_BASE, RTC_SLOW_SIZE) {
            self.rtc_slow[(addr - RTC_SLOW_BASE) as usize] as u32
        } else if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE) {
            self.rtc_fast[(addr - RTC_FAST_BASE) as usize] as u32
        } else if in_range!(addr, ROM_DATA_BASE, ROM_DATA_SIZE) {
            // ROM constant tables (NOT RTC fast — separate memory; aliasing
            // them let load_rom_data stomp the app's RTC segment).
            self.rom_data[(addr - ROM_DATA_BASE) as usize] as u32
        } else if in_range!(addr, APB_START, APB_END) {
            (self.mmio32(addr, false, 0) >> ((addr & 3) * 8)) & 0xFF
        } else {
            0
        }
    }

    fn read16(&mut self, addr: u32) -> u32 {
        let lo = self.read8(addr);
        let hi = self.read8(addr + 1);
        lo | (hi << 8)
    }

    #[inline(always)]
    fn read32(&mut self, addr: u32) -> u32 {
        let addr = ioblock_remap(addr);
        // Fast path: aligned SRAM reads (the overwhelmingly common case).
        // DRAM alias: 0x3FC80000-0x3FD00000; IRAM alias: 0x40378000-0x403E0000
        // (offset 0x6F0000 into the same 512 KB backing).
        if addr & 3 == 0 && in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE) {
            let o = (addr - DRAM_BASE) as usize;
            // SAFETY: o < SRAM_BASE_RANGE <= SRAM_BYTES, checked by in_range
            return u32::from_le_bytes(unsafe {
                [
                    *self.sram.get_unchecked(o),
                    *self.sram.get_unchecked(o + 1),
                    *self.sram.get_unchecked(o + 2),
                    *self.sram.get_unchecked(o + 3),
                ]
            });
        }
        if addr & 3 == 0 && in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE) {
            let o = (addr - IRAM_BASE) as usize;
            if o < SRAM0_SIZE as usize {
                // SAFETY: o < SRAM0_SIZE, checked above
                return u32::from_le_bytes(unsafe {
                    [
                        *self.iram0.get_unchecked(o),
                        *self.iram0.get_unchecked(o + 1),
                        *self.iram0.get_unchecked(o + 2),
                        *self.iram0.get_unchecked(o + 3),
                    ]
                });
            }
            let o = DIRAM_DATA_BASE - DRAM_BASE + (addr - DIRAM_INST_BASE);
            let o = o as usize;
            // SAFETY: o < SRAM_BYTES, DIRAM window is within sram backing
            return u32::from_le_bytes(unsafe {
                [
                    *self.sram.get_unchecked(o),
                    *self.sram.get_unchecked(o + 1),
                    *self.sram.get_unchecked(o + 2),
                    *self.sram.get_unchecked(o + 3),
                ]
            });
        }
        // Slow path: unaligned or non-SRAM — fall back to byte-by-byte
        // (ram8 already routes the host pool, so no separate check needed).
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE)
        {
            return u32::from_le_bytes([
                self.ram8(addr),
                self.ram8(addr + 1),
                self.ram8(addr + 2),
                self.ram8(addr + 3),
            ]);
        }
        if in_range!(addr, IROM_BASE, IROM_SIZE) {
            let o = (addr - IROM_BASE) as usize;
            return u32::from_le_bytes([
                self.irom[o],
                self.irom[o + 1],
                self.irom[o + 2],
                self.irom[o + 3],
            ]);
        }
        if in_range!(addr, FLASH_DATA_BASE, FLASH_WINDOW_SIZE)
            || in_range!(addr, FLASH_INST_BASE, FLASH_WINDOW_SIZE)
        {
            return u32::from_le_bytes([
                self.cache_read8(addr),
                self.cache_read8(addr + 1),
                self.cache_read8(addr + 2),
                self.cache_read8(addr + 3),
            ]);
        }
        if in_range!(addr, RTC_SLOW_BASE, RTC_SLOW_SIZE) {
            let o = (addr - RTC_SLOW_BASE) as usize;
            return u32::from_le_bytes([
                self.rtc_slow[o],
                self.rtc_slow[o + 1],
                self.rtc_slow[o + 2],
                self.rtc_slow[o + 3],
            ]);
        }
        if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE) {
            let o = (addr - RTC_FAST_BASE) as usize;
            return u32::from_le_bytes([
                self.rtc_fast[o],
                self.rtc_fast[o + 1],
                self.rtc_fast[o + 2],
                self.rtc_fast[o + 3],
            ]);
        }
        if in_range!(addr, ROM_DATA_BASE, ROM_DATA_SIZE) {
            let o = (addr - ROM_DATA_BASE) as usize;
            return u32::from_le_bytes([
                self.rom_data[o],
                self.rom_data[o + 1],
                self.rom_data[o + 2],
                self.rom_data[o + 3],
            ]);
        }
        if in_range!(addr, APB_START, APB_END) {
            self.mmio32(addr, false, 0)
        } else {
            0
        }
    }

    fn write8(&mut self, addr: u32, val: u32) {
        let addr = ioblock_remap(addr);
        // Host event-block pool (see `read8`).
        if in_range!(addr, Self::WIFI_ARD_POOL, 768) {
            self.ard_pool[(addr - Self::WIFI_ARD_POOL) as usize] = val as u8;
            return;
        }
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE)
        {
            self.ram_write8(addr, val as u8);
        } else if in_range!(addr, IROM_BASE, IROM_SIZE) {
            // ROM: read-only.
        } else if in_range!(addr, FLASH_DATA_BASE, FLASH_WINDOW_SIZE)
            || in_range!(addr, FLASH_INST_BASE, FLASH_WINDOW_SIZE)
        {
            // Cache windows: only MMU-mapped PSRAM pages are writable.
            self.cache_write8(addr, val as u8);
        } else if in_range!(addr, RTC_SLOW_BASE, RTC_SLOW_SIZE) {
            self.rtc_slow[(addr - RTC_SLOW_BASE) as usize] = val as u8;
        } else if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE) {
            self.rtc_fast[(addr - RTC_FAST_BASE) as usize] = val as u8;
        } else if in_range!(addr, ROM_DATA_BASE, ROM_DATA_SIZE) {
            self.rom_data[(addr - ROM_DATA_BASE) as usize] = val as u8;
        } else if in_range!(addr, APB_START, APB_END) {
            self.mmio32(addr, true, val);
        }
    }

    fn write16(&mut self, addr: u32, val: u32) {
        // RAM regions: byte-lane merge. MMIO: APB registers are 32-bit;
        // a sub-word write is approximated as a full-word store.
        // Cache windows: byte-lane merge like RAM — a widened write32 would
        // zero-clobber the adjacent halfword on MMU-mapped PSRAM pages
        // (real S16I semantics; broke MicroPython's u16 qstr table which is
        // written in hash order).
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE)
        {
            self.ram_write8(addr, val as u8);
            self.ram_write8(addr + 1, (val >> 8) as u8);
        } else if in_range!(addr, FLASH_DATA_BASE, FLASH_WINDOW_SIZE)
            || in_range!(addr, FLASH_INST_BASE, FLASH_WINDOW_SIZE)
        {
            self.cache_write8(addr, val as u8);
            self.cache_write8(addr + 1, (val >> 8) as u8);
        } else if in_range!(addr, RTC_SLOW_BASE, RTC_SLOW_SIZE)
            || in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE)
            || in_range!(addr, ROM_DATA_BASE, ROM_DATA_SIZE)
        {
            // RTC memories: byte-lane merge like RAM — falling through to
            // write32 would zero-clobber the adjacent halfword (same bug
            // class as the MicroPython PSRAM write16 widening).
            self.write8(addr, val);
            self.write8(addr + 1, val >> 8);
        } else {
            self.write32(addr, val);
        }
    }

    #[inline(always)]
    fn write32(&mut self, addr: u32, val: u32) {
        let addr = ioblock_remap(addr);
        let bytes = val.to_le_bytes();
        // Fast path: aligned SRAM writes (the overwhelmingly common case).
        if addr & 3 == 0 && in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE) {
            // Block writes to _putc2 addresses: the firmware's inlined
            // esp_rom_install_uart_printf() writes _putc2 to the same function
            // as _putc1; the ROM's ets_printf calls both per character,
            // doubling every console message.  Keeping _putc2 = 0 ensures
            // ets_printf only calls _putc1.
            if addr == 0x3FCEF750 || addr == 0x3FCEF748 {
                return;
            }
            let o = (addr - DRAM_BASE) as usize;
            self.sram[o..o + 4].copy_from_slice(&bytes);
            return;
        }
        if addr & 3 == 0 && in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE) {
            let o = (addr - IRAM_BASE) as usize;
            if o < SRAM0_SIZE as usize {
                self.iram0[o..o + 4].copy_from_slice(&bytes);
                return;
            }
            let o = (DIRAM_DATA_BASE - DRAM_BASE + (addr - DIRAM_INST_BASE)) as usize;
            self.sram[o..o + 4].copy_from_slice(&bytes);
            return;
        }
        // Slow path: unaligned or non-SRAM — fall back to byte-by-byte.
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE)
        {
            for (i, b) in bytes.into_iter().enumerate() {
                self.ram_write8(addr + i as u32, b);
            }
        } else if in_range!(addr, IROM_BASE, IROM_SIZE) {
            // ROM: read-only.
        } else if in_range!(addr, FLASH_DATA_BASE, FLASH_WINDOW_SIZE)
            || in_range!(addr, FLASH_INST_BASE, FLASH_WINDOW_SIZE)
        {
            for (i, b) in bytes.into_iter().enumerate() {
                self.cache_write8(addr + i as u32, b);
            }
        } else if in_range!(addr, RTC_SLOW_BASE, RTC_SLOW_SIZE) {
            // Byte-by-byte: a widened 4-byte copy would clobber neighbors on
            // sub-word stores and panic at the array tail (o+4 > len).
            for (i, b) in bytes.into_iter().enumerate() {
                let o = (addr - RTC_SLOW_BASE) as usize + i;
                if o < self.rtc_slow.len() {
                    self.rtc_slow[o] = b;
                }
            }
        } else if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE) {
            // Byte-by-byte: a widened 4-byte copy would clobber neighbors on
            // sub-word stores and panic at the array tail (o+4 > len).
            for (i, b) in bytes.into_iter().enumerate() {
                let o = (addr - RTC_FAST_BASE) as usize + i;
                if o < self.rtc_fast.len() {
                    self.rtc_fast[o] = b;
                }
            }
        } else if in_range!(addr, ROM_DATA_BASE, ROM_DATA_SIZE) {
            for (i, b) in bytes.into_iter().enumerate() {
                let o = (addr - ROM_DATA_BASE) as usize + i;
                if o < self.rom_data.len() {
                    self.rom_data[o] = b;
                }
            }
        } else if in_range!(addr, APB_START, APB_END) {
            self.mmio32(addr, true, val);
        }
    }
}
