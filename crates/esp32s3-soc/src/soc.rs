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
use alloc::vec::Vec;
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
use crate::twai::{TWAI_BASE, Twai};
use crate::uart::Uart;
use crate::ulp::{ULP_OFF_END, ULP_OFF_START, Ulp};
use crate::usb_serial_jtag::{USB_SERIAL_JTAG_INTR_SOURCE, UsbSerialJtag};

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
    /// PSRAM backing store (read-write through MMU-mapped cache pages).
    psram: Box<[u8; PSRAM_SIZE as usize]>,
    rtc_slow: Box<[u8; RTC_SLOW_SIZE as usize]>,
    rtc_fast: Box<[u8; RTC_FAST_SIZE as usize]>,
    /// USB-Serial-JTAG (CDC-ACM console) controller. The boot ROM's console
    /// (uart_tx_one_char @ 0x40048C30) writes chars to the USB_SERIAL_JTAG FIFO
    /// (0x60038000), NOT UART0 — the S3's ROM messages come out of the USB-CDC
    /// port on real hardware. `Esp32S3::take_usb_serial_tx` drains this too.
    usb: UsbSerialJtag,
    /// ULP-RISC-V coprocessor (its own rv32im core; runs from RTC_SLOW_MEM).
    ulp: Ulp,
    uarts: [Uart; 3],
    gpio: Gpio,
    ledc: Lcdc,
    mcpwm: Mcpwm,
    /// MCPWM group 1: independent copy of the group-0 block.
    mcpwm1: Mcpwm,
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
    /// Crypto/shared GDMA (`DR_REG_GDMA_BASE = 0x6003F000`): the dedicated DMA
    /// controller the esp-idf crypto drivers (AES, SHA) feed through
    /// `esp_crypto_shared_gdma`. AES routes its plaintext `out` link here and
    /// the ciphertext `in` link back to DRAM (the GENERAL GDMA at 0x60042000
    /// is a different controller used by SPI/RMT/etc).
    crypto_dma: Gdma,
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
    rtc: Rtc,
    rtc_i2c: RtcI2c,
    rtc_io: RtcIo,
    rng: Rng,
    sdmmc: Sdmmc,
    sdm: Sdm,
    intc: Intc,
    /// SENS2 PLL-lock status (TRM SENS2 SAR_PLL_FORCE_CTRL @ 0x6000E040):
    /// rtc_clk powers the CPU PLL, then polls bit 24 (PLL_LOCK) until the
    /// ~200 us lock time elapses; the register is read-only on silicon for
    /// that bit, so writes to it only matter for arming the lock timer.
    pll: PllLock,

    // ── P5 register-store peripherals (configure-and-forget, no observable
    //    side-effects modeled — see regstore.rs). Bases from esp-idf
    //    components/soc/esp32s3/register/soc/reg_base.h.
    /// SENSITIVE (= Mem-Protection/PMS, DR_REG_SENSITIVE_BASE 0x600C1000).
    sensitive: RegStore,
    /// WCL (= World Controller / TEE, DR_REG_WCL_BASE 0x600D0000).
    wcl: RegStore,
    /// PERI_BACKUP (retention registers, DR_REG_PERI_BACKUP_BASE 0x6002A000).
    peri_backup: RegStore,
    /// SYSCON (= peripheral clock/reset control, "PCR" on S3, 0x60026000).
    syscon: RegStore,
    /// ASSIST_DEBUG (watchpoint/breakpoint unit, DR_REG_ASSIST_DEBUG_BASE
    /// 0x600CE000).
    assist_debug: RegStore,
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

    /// SYSTEM.CPU_INT_FROM_CPU_0/1 (0x600C0030/0x600C0034): the cross-core
    /// interrupt registers.  Writing bit 0 to +0x30 asserts the CROSS_CORE0
    /// interrupt source (48) on core 0's matrix; +0x34 asserts CROSS_CORE1
    /// (49) on core 1's.  This is how the FreeRTOS SMP scheduler forces a
    /// yield on the peer/self core (`esp_crosscore_int_send` at 0x40375EF8
    /// in the app, ROM's ets_ipc_*); the ISR clears it by writing 0 back.
    /// Level-style sticky bit per core (bit 0 = asserted).
    cpu_int_from_cpu: [u32; 2],

    /// ROM-boot phase: while set, cache-window reads bypass the MMU and map
    /// 1:1 to raw flash. The real ROM bootloader reads flash via SPI with the
    /// MMU uninvolved; our ROM stub reads through the data window as a
    /// stand-in, but the machine pre-maps the app's text/rodata pages before
    /// the stub runs — without this bypass the stub's image-header/segment
    /// reads would be redirected to those mapped flash pages (garbage).
    /// Cleared by `Esp32S3::step` once core 0 leaves the ROM (stub done).
    rom_boot_mode: bool,

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
            psram: Box::new([0; PSRAM_SIZE as usize]),
            rtc_slow: Box::new([0; RTC_SLOW_SIZE as usize]),
            rtc_fast: Box::new([0; RTC_FAST_SIZE as usize]),
            uarts: [Uart::new(), Uart::new(), Uart::new()],
            usb: UsbSerialJtag::new(),
            ulp: Ulp::new(),
            gpio: Gpio::new(),
            ledc: Lcdc::new(),
            mcpwm: Mcpwm::new(),
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
            crypto_dma: {
                let mut g = Gdma::default();
                g.ignore_ena = true;
                g
            },
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
            rtc: Rtc::new(),
            rtc_i2c: RtcI2c::new(),
            rtc_io: RtcIo::new(),
            rng: Rng::new(),
            sdmmc: Sdmmc::new(),
            sdm: Sdm::new(),
            intc: Intc::new(),
            pll: PllLock::default(),
            sensitive: RegStore::new(0x1000),
            wcl: RegStore::new(0x1000),
            peri_backup: RegStore::new(0x1000),
            syscon: RegStore::new(0x1000),
            i2s: [I2s::new(0), I2s::new(1)],
            assist_debug: RegStore::new(0x1000),
            lcd_cam: LcdCam::new(),
            appcpu_ctrl_a: 0,
            cpu_int_from_cpu: [0, 0],
            rom_boot_mode: false,
            cached_src: 0,
            src_valid: false,
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
    pub fn map_app_flash_segments(&mut self, app_flash_off: u32) {
        let base = app_flash_off as usize;
        if base + 24 > FLASH_SIZE as usize || self.flash[base] != ESP_IMAGE_MAGIC {
            return;
        }
        let nseg = self.flash[base + 1] as usize;
        // Walk once to find the image end (segments are back-to-back).
        let mut end = base + 24; // esp_image_header_t is 24 bytes
        for _ in 0..nseg {
            if end + 8 > FLASH_SIZE as usize {
                break;
            }
            let len = u32::from_le_bytes(self.flash[end + 4..end + 8].try_into().unwrap()) as usize;
            end += 8 + len;
        }
        // 1. Loader scratch: map every flash page the app image spans.
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
            if pos + 8 > FLASH_SIZE as usize {
                break;
            }
            let load = u32::from_le_bytes(self.flash[pos..pos + 4].try_into().unwrap());
            let len = u32::from_le_bytes(self.flash[pos + 4..pos + 8].try_into().unwrap()) as usize;
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
            // A pin drives its GPIO_OUT bit when FUNC_OUT_SEL selects the
            // GPIO function. The matrix encodes this two ways: the 0x80
            // sentinel (our documented default) and 0 (what pinMode leaves /
            // writes for a plain digital output). Any other value is a
            // peripheral matrix signal, whose level comes from signal_level.
            let bit = if sel == 0x80 || sel == 0 {
                self.gpio.out_bit(i)
            } else {
                self.signal_level(sel)
            };
            out |= bit << i;
        }
        out
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

    /// Host-driven SPI slave master-write: capture `bytes` into the slave's
    /// data buffer on `chan` (0=GPSPI2, 1=GPSPI3), recording the bitlen and
    /// raising trans_done. Only acts when the controller is in slave mode.
    pub fn spi_slave_inject_write(&mut self, chan: usize, bytes: &[u8]) {
        self.spi[chan].slave_inject_write(bytes);
    }

    /// Host-driven SPI slave master-read: return the first `nbytes` of the
    /// slave's preloaded data buffer on `chan`, recording the bitlen and
    /// raising trans_done. Only acts when the controller is in slave mode.
    pub fn spi_slave_take_read(&mut self, chan: usize, nbytes: usize) -> Vec<u8> {
        self.spi[chan].slave_take_read(nbytes)
    }

    /// Inject RX bytes for the next I2C master-read on `chan`
    /// (0=I2CEXT0, 1=I2CEXT1). Each byte is returned to the MCU on a READ
    /// command; an empty supply reads back 0xFF (no device).
    pub fn i2c_inject_rx(&mut self, chan: usize, bytes: &[u8]) {
        self.i2c[chan].inject_rx(bytes);
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
        // so any cached bitmap from a previous step is stale.
        self.src_valid = false;
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
            self.timg[0].tick(1);
            self.timg[1].tick(1);
            self.systimer.tick(1);
            self.ledc.tick();
            self.spi[0].tick(1);
            if let Some(tx) = self.spi[0].take_last_tx() {
                self.pending_spi_tx[0] = tx;
                self.events.push(EmuEvent {
                    kind: EVT_SPI_XFER,
                    a: 0,
                    b: self.pending_spi_tx[0].len() as u32,
                });
            }
            self.spi[1].tick(1);
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
            // function-call overhead on the hot path.
            if !self.i2c[0].is_idle() {
                let n = self.i2c[0].remaining_cycles().max(1);
                self.i2c[0].tick(n);
                if self.i2c[0].has_events() {
                    self.events.extend(self.i2c[0].drain_events());
                }
            }
            if !self.i2c[1].is_idle() {
                let n = self.i2c[1].remaining_cycles().max(1);
                self.i2c[1].tick(n);
                if self.i2c[1].has_events() {
                    self.events.extend(self.i2c[1].drain_events());
                }
            }
            self.adc.tick(1);
            // Waveform peripherals: skip the tick while idle. Each gate
            // mirrors its tick's own early-out (inactive RMT channels /
            // stopped MCPWM timers / non-busy LCD / non-busy I2S), so a
            // skipped tick would have no-opped identically.
            if self.rmt.is_active() {
                self.rmt.tick();
            }
            // RMT RX sampling: skipped unless a capture is armed/running.
            // Input levels resolve through the GPIO-matrix input routing
            // against the pad readback (so TX-driven pads loop back);
            // unrouted inputs read pull-up high. Pins 32+ are out of the
            // u32 readback word and also read high (same limit as the GPIO
            // edge sampler).
            if self.rmt.rx_pending() {
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
            if self.mcpwm.is_active() {
                self.mcpwm.tick();
            }
            if self.mcpwm1.is_active() {
                self.mcpwm1.tick();
            }
            if self.sdm.is_active() {
                self.sdm.tick();
            }
            if self.lcd_cam.is_active() {
                self.lcd_cam.tick();
            }
            if self.i2s[0].is_active() {
                self.i2s[0].tick();
            }
            if self.i2s[1].is_active() {
                self.i2s[1].tick();
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
            if !self.pcnt.is_init() || self.pcnt.is_counting() {
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
            // host-poll behavior).
            self.usb.tick();
        }
        self.rtc.tick(cycles);
        self.pll.tick(cycles);
    }

    /// Internal SRAM access at `addr` (must be inside DRAM or IRAM window).
    /// D-bus: 0x3FC80000-0x3FD00000 is one 512 KB backing (ROM-data 32 KB +
    /// D/IRAM 416 KB + SRAM2 64 KB).  I-bus: SRAM0 (0x40370000-0x40377FFF,
    /// 32 KB, instruction-only, separate cells) and the D/IRAM instruction
    /// window (0x40378000-0x403DFFFF alias of data 0x3FC88000-0x3FCEFFFF,
    /// offset 0x6F0000 — memory.ld.in I_D_SRAM_OFFSET).
    fn ram8(&self, addr: u32) -> u8 {
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
    /// unmapped flash region).
    fn flash_byte(&self, off: u32) -> u8 {
        if off >= FLASH_SIZE {
            0
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
            let scratch_end = LOADER_SCRATCH_OFF + 0x6_0000;
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
    /// (FUNC_OUT_SEL 0x80 or 0) loop back the GPIO_OUT bit.
    pub fn gpio_in_readback(&self) -> u32 {
        let mut v = self.gpio.raw_in();
        for i in 0..self.gpio.pin_count() {
            if !self.gpio.enabled(i) {
                continue;
            }
            let sel = self.gpio.out_sel(i);
            let driven = if sel == 0x80 || sel == 0 {
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
        v
    }

    /// GPIO matrix output signal level for a peripheral signal index.
    /// LEDC occupies 73..80, I2CEXT0 SCL/SDA = 89/90, I2CEXT1 = 91/92,
    /// GPSPI2 (FSPI) 101..105 + CS 110/111, GPSPI3 66..72 (S3
    /// gpio_sig_map.h); everything else reads 0.
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
                        self.spi[n].write32(off, value);
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
                    0
                } else {
                    self.memspi[n].read32(off)
                }
            }
            I2C0_BASE | I2C1_BASE => {
                let n = if dev == I2C0_BASE { 0 } else { 1 };
                if is_write {
                    self.i2c[n].write32(off, value);
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
                    // triggers the descriptor-walk copy.
                    if let Some((ch, is_out)) = self.gdma.write32(off, value) {
                        let link_addr = self.gdma.out_link_addr(ch);
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
                        let mut desc = link_addr;
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
                                        self.aes.feed_text_in_byte(((w >> 16) & 0xFF) as u8);
                                        self.aes.feed_text_in_byte(((w >> 24) & 0xFF) as u8);
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
                                        if self.gdma.in_peri_sel(rx) == crate::gdma::GDMA_AES_PERIPH
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
                                } else if peri == crate::gdma::GDMA_I2S0_PERIPH {
                                    // I2S0 TX: copy the descriptor's words into the
                                    // I2S0 TX FIFO register (each 32-bit write is
                                    // one FIFO push).
                                    let mut k = 0u32;
                                    while k + 4 <= len {
                                        let w = self.read32(buf + k);
                                        self.write32(
                                            crate::memmap::I2S0_BASE + crate::i2s::FIFO,
                                            w,
                                        );
                                        k += 4;
                                    }
                                    self.gdma.set_out_eof_des_addr(ch, desc);
                                } else if peri == crate::gdma::GDMA_I2S1_PERIPH {
                                    let mut k = 0u32;
                                    while k + 4 <= len {
                                        let w = self.read32(buf + k);
                                        self.write32(
                                            crate::memmap::I2S1_BASE + crate::i2s::FIFO,
                                            w,
                                        );
                                        k += 4;
                                    }
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
                                    self.gdma.raise_in_done(ch);
                                } else if peri == crate::gdma::GDMA_I2S0_PERIPH
                                    || peri == crate::gdma::GDMA_I2S1_PERIPH
                                {
                                    // I2S RX: copy words out of the I2S RX FIFO
                                    // register into the descriptor's DRAM buffer
                                    // (each read pops one FIFO word).
                                    let fifo = if peri == crate::gdma::GDMA_I2S0_PERIPH {
                                        crate::memmap::I2S0_BASE + crate::i2s::FIFO
                                    } else {
                                        crate::memmap::I2S1_BASE + crate::i2s::FIFO
                                    };
                                    let mut k = 0u32;
                                    while k + 4 <= len {
                                        let w = self.read32(fifo);
                                        self.write32(buf + k, w);
                                        k += 4;
                                    }
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
                        if is_out {
                            self.gdma.raise_out_done(ch);
                        } else {
                            self.gdma.raise_in_done(ch);
                        }
                    }
                    0
                } else {
                    self.gdma.read32(off)
                }
            }
            TWAI_BASE => {
                if is_write {
                    self.twai.write32(off, value);
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
                    if is_write {
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
            // SENS2 (TRM memory map): only SAR_PLL_FORCE_CTRL @ +0x40 is
            // touched by the firmware (rtc_clk PLL power-up + lock poll);
            // the rest of the page reads 0.
            0x6000_E000 => {
                if off == 0x40 {
                    if is_write {
                        self.pll.write32(value);
                        0
                    } else {
                        self.pll.read32()
                    }
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
                    0
                } else {
                    self.efuse.read32(off)
                }
            }
            SHA_BASE => {
                if is_write {
                    self.sha.write32(off, value);
                    0
                } else {
                    self.sha.read32(off)
                }
            }
            crate::aes::AES_BASE => {
                if is_write {
                    self.aes.write32(off, value);
                    0
                } else {
                    self.aes.read32(off)
                }
            }
            crate::rsa::RSA_BASE => {
                if is_write {
                    self.rsa.write32(off, value);
                    0
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
                    self.hmac.write32(off, value);
                    // On SET_PARA_FINISH the engine latches the eFuse key for the
                    // selected key_id into its working key.
                    if off == (crate::hmac::SET_PARA_FINISH_OFF * 4) as u32 {
                        self.hmac.fetch_key(&self.efuse);
                    }
                    0
                } else {
                    self.hmac.read32(off)
                }
            }
            crate::ds::DS_BASE => {
                if is_write {
                    self.ds.write32(off, value, &self.efuse);
                    0
                } else {
                    self.ds.read32(off)
                }
            }
            crate::sdmmc::SDMMC_BASE => {
                if is_write {
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
                                        self.write8(buf.wrapping_add(i as u32), data[off] as u32);
                                        off += 1;
                                    }
                                }
                            }
                        }
                        self.sdmmc.finish_idmac();
                    }
                    0
                } else {
                    self.sdmmc.read32(off)
                }
            }
            crate::rng::RNG_BASE => {
                if is_write {
                    self.rng.write32(off, value);
                    0
                } else {
                    self.rng.read32(off)
                }
            }
            // Crypto/shared GDMA (`DR_REG_GDMA_BASE = 0x6003F000`): dedicated DMA
            // for the crypto engines (AES, SHA). Mirrors the general GDMA arm but
            // routes to AES/SHA. AES is the default (the crypto DMA is
            // crypto-dedicated, so the exact peri_sel value is irrelevant).
            0x6003_F000 => {
                if is_write {
                    if let Some((ch, is_out)) = self.crypto_dma.write32(off, value) {
                        let link_addr = if is_out {
                            self.crypto_dma.out_link_addr(ch)
                        } else {
                            self.crypto_dma.in_link_addr(ch)
                        };
                        let peri = if is_out {
                            self.crypto_dma.out_peri_sel(ch)
                        } else {
                            self.crypto_dma.in_peri_sel(ch)
                        };
                        let mut desc = link_addr;
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
                                if peri == crate::gdma::GDMA_SHA_PERIPH {
                                    let mut k = 0u32;
                                    while k + 4 <= len {
                                        let w = self.read32(buf + k);
                                        self.sha.feed_byte((w & 0xFF) as u8);
                                        self.sha.feed_byte(((w >> 8) & 0xFF) as u8);
                                        self.sha.feed_byte(((w >> 16) & 0xFF) as u8);
                                        self.sha.feed_byte(((w >> 24) & 0xFF) as u8);
                                        k += 4;
                                    }
                                    self.crypto_dma.set_out_eof_des_addr(ch, desc);
                                } else {
                                    // AES (default): feed plaintext, transform,
                                    // then auto-push ciphertext through the RX link.
                                    let mut k = 0u32;
                                    while k + 4 <= len {
                                        let w = self.read32(buf + k);
                                        self.aes.feed_text_in_byte((w & 0xFF) as u8);
                                        self.aes.feed_text_in_byte(((w >> 8) & 0xFF) as u8);
                                        self.aes.feed_text_in_byte(((w >> 16) & 0xFF) as u8);
                                        self.aes.feed_text_in_byte(((w >> 24) & 0xFF) as u8);
                                        k += 4;
                                    }
                                    self.aes.transform();
                                    self.crypto_dma.set_out_eof_des_addr(ch, desc);
                                    for rx in 0..crate::gdma::NCH {
                                        if self.crypto_dma.in_peri_sel(rx) == peri {
                                            let mut idesc = self.crypto_dma.in_link_addr(rx);
                                            loop {
                                                let idw0 = self.read32(idesc);
                                                let ibuf = self.read32(idesc + 4);
                                                let inext = self.read32(idesc + 8);
                                                let ilen = (idw0 >> 12) & 0xFFF;
                                                let ieof = (idw0 >> 30) & 1;
                                                let iowner = (idw0 >> 31) & 1;
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
                                                self.write32(idesc, idw0 & !(1u32 << 31));
                                                if inext == 0 || ieof == 1 {
                                                    break;
                                                }
                                                idesc = inext;
                                            }
                                            self.crypto_dma.raise_in_done(rx);
                                            break;
                                        }
                                    }
                                }
                            } else {
                                // IN (RX) channel: copy ciphertext to DRAM.
                                if peri != crate::gdma::GDMA_SHA_PERIPH {
                                    let mut k = 0u32;
                                    while k + 4 <= len {
                                        let w = self.aes.out_word_at(k);
                                        self.write32(buf + k, w);
                                        k += 4;
                                    }
                                    self.crypto_dma.enable_in_int_all(ch);
                                    self.crypto_dma.raise_in_done(ch);
                                }
                            }
                            // DMA hands the descriptor back: clear owner.
                            self.write32(desc, dw0 & !(1u32 << 31));
                            if next == 0 || eof == 1 {
                                break;
                            }
                            desc = next;
                        }
                        if is_out {
                            self.crypto_dma.raise_out_done(ch);
                        } else {
                            self.crypto_dma.raise_in_done(ch);
                        }
                    }
                    0
                } else {
                    self.crypto_dma.read32(off)
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
                } else if off == 0x030 || off == 0x034 {
                    // Cross-core interrupt: write 1 asserts the CROSS_CORE0
                    // (core 0) / CROSS_CORE1 (core 1) source; the ISR writes
                    // 0 to deassert (esp_crosscore_isr clears its own core's
                    // reg).  TRM SYSTEM_CPU_INT_FROM_CPU_0/1.
                    let idx = if off == 0x030 { 0 } else { 1 };
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
                    self.lcd_cam.write32(off, value);
                    0
                } else {
                    self.lcd_cam.read32(off)
                }
            }
            0x600C_1000 => store_dispatch(is_write, off, value, &mut self.sensitive),
            0x600C_E000 => store_dispatch(is_write, off, value, &mut self.assist_debug),
            0x600D_0000 => store_dispatch(is_write, off, value, &mut self.wcl),
            // Everything else in the APB space: no model yet.
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
    /// True if any device requested a hard reset (e.g. a WDT stage action of
    /// reset-CPU/system). The machine consumes this each step to reboot.
    pub fn consume_reset(&mut self) -> bool {
        self.timg[0].consume_reset() || self.timg[1].consume_reset()
    }

    /// True if firmware requested a deep-sleep (wrote `RTC_CNTL_SLEEP_EN`).
    /// Returns the captured sleep duration (slow-clock ticks) and clears the
    /// flag. The machine consumes this each step to fast-forward the sleep.
    pub fn consume_sleep_request(&mut self) -> Option<u64> {
        self.rtc.consume_sleep_request()
    }

    /// Debug accessor: has the firmware requested a deep-sleep (pending consume)?
    pub fn rtc_sleep_req(&self) -> bool {
        self.rtc.sleep_req()
    }

    /// Record the wakeup-cause bits read by `esp_sleep_get_wakeup_cause`
    /// after a deep-sleep reboot (machine writes this on wake).
    pub fn set_sleep_wakeup_cause(&mut self, bits: u32) {
        self.rtc.set_wakeup_cause(bits);
    }

    /// Debug accessor for the AES interrupt raw&enabled state (validation harness).
    pub fn aes_debug_int(&self) -> (u32, u32) {
        self.aes.debug_int()
    }

    /// Debug accessor for the GDMA interrupt-pending state (validation harness).
    pub fn gdma_int_pending(&self) -> bool {
        self.gdma.int_pending()
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

    /// Debug: raw crypto-DMA register writes (validation harness).
    pub fn crypto_dma_debug_log(&self) -> Vec<(u32, u32)> {
        self.crypto_dma.debug_log()
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
        // GDMA channels have per-channel sources (ETS_DMA_IN_CH0..4 =
        // 66..70, ETS_DMA_OUT_CH0..4 = 71..75). The crypto/shared DMA
        // instance is unwired: its consumers (AES/SHA drivers) poll, and no
        // distinct source exists for it.
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
        // = sources 79/80.  This is per-cpu so it must not be cached.
        let mut final_src = src;
        if self.cpu_int_from_cpu[cpu] & 1 != 0 {
            final_src |= 1 << (79 + cpu);
        }
        self.intc.pending_lines(cpu, final_src)
    }

    #[inline(always)]
    fn read8(&mut self, addr: u32) -> u32 {
        let addr = ioblock_remap(addr);
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
        } else if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE)
            || in_range!(addr, RTC_FAST_DATA_BASE, RTC_FAST_SIZE)
        {
            // RTC_FAST_DATA_BASE (0x3FF18000) sits BELOW RTC_FAST_BASE (0x600F8000);
            // select by comparing against the higher base so the 0x600F8000 window
            // does not alias into the 0x3FF18000 window (TRM RTC_FAST_MEM).
            let base = if addr < RTC_FAST_BASE {
                RTC_FAST_DATA_BASE
            } else {
                RTC_FAST_BASE
            };
            self.rtc_fast[(addr - base) as usize] as u32
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
        // Slow path: unaligned or non-SRAM — fall back to byte-by-byte.
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
        if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE)
            || in_range!(addr, RTC_FAST_DATA_BASE, RTC_FAST_SIZE)
        {
            let base = if addr < RTC_FAST_BASE {
                RTC_FAST_DATA_BASE
            } else {
                RTC_FAST_BASE
            };
            let o = (addr - base) as usize;
            return u32::from_le_bytes([
                self.rtc_fast[o],
                self.rtc_fast[o + 1],
                self.rtc_fast[o + 2],
                self.rtc_fast[o + 3],
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
        } else if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE)
            || in_range!(addr, RTC_FAST_DATA_BASE, RTC_FAST_SIZE)
        {
            // RTC_FAST_DATA_BASE (0x3FF18000) sits BELOW RTC_FAST_BASE (0x600F8000);
            // select by comparing against the higher base so the 0x600F8000 window
            // does not alias into the 0x3FF18000 window (TRM RTC_FAST_MEM).
            let base = if addr < RTC_FAST_BASE {
                RTC_FAST_DATA_BASE
            } else {
                RTC_FAST_BASE
            };
            self.rtc_fast[(addr - base) as usize] = val as u8;
        } else if in_range!(addr, APB_START, APB_END) {
            self.mmio32(addr, true, val);
        }
    }

    fn write16(&mut self, addr: u32, val: u32) {
        // RAM regions: byte-lane merge. MMIO: APB registers are 32-bit;
        // a sub-word write is approximated as a full-word store.
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE)
        {
            self.ram_write8(addr, val as u8);
            self.ram_write8(addr + 1, (val >> 8) as u8);
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
            let o = (addr - RTC_SLOW_BASE) as usize;
            self.rtc_slow[o..o + 4].copy_from_slice(&bytes);
        } else if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE)
            || in_range!(addr, RTC_FAST_DATA_BASE, RTC_FAST_SIZE)
        {
            let base = if addr < RTC_FAST_BASE {
                RTC_FAST_DATA_BASE
            } else {
                RTC_FAST_BASE
            };
            let o = (addr - base) as usize;
            self.rtc_fast[o..o + 4].copy_from_slice(&bytes);
        } else if in_range!(addr, APB_START, APB_END) {
            self.mmio32(addr, true, val);
        }
    }
}
