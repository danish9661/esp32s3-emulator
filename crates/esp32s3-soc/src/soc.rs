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
use crate::mcpwm::{MCPWM_BASE, Mcpwm};
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

macro_rules! in_range {
    ($addr:expr, $base:expr, $size:expr) => {
        ($base..$base + $size).contains(&$addr)
    };
}

// SRAM window size used in range checks (kept in sync with DRAM_SIZE).
const SRAM_BASE_RANGE: u32 = DRAM_SIZE;

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
    /// USB-Serial-JTAG TX capture: the boot ROM's console (uart_tx_one_char
    /// @ 0x40048C30) writes chars to the USB_SERIAL_JTAG FIFO (0x60038000),
    /// NOT UART0 — the S3's ROM messages come out of the USB-CDC port on
    /// real hardware.  `Esp32S3::take_uart_tx(0)` drains this too.
    usb_serial_tx: Vec<u8>,
    uarts: [Uart; 3],
    gpio: Gpio,
    ledc: Lcdc,
    mcpwm: Mcpwm,
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
            usb_serial_tx: Vec::new(),
            gpio: Gpio::new(),
            ledc: Lcdc::new(),
            mcpwm: Mcpwm::new(),
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
        self.usb_serial_tx.len()
    }

    /// Number of bytes queued in UART `n`'s TX capture (debug probe).
    pub fn uart_tx_len(&self, n: usize) -> usize {
        self.uarts[n].tx_len()
    }

    pub fn take_uart_tx(&mut self, n: usize) -> Vec<u8> {
        self.uarts[n].take_tx()
    }

    /// Push one received byte into UART `n`'s RX FIFO (host console input).
    pub fn uart_inject_rx(&mut self, n: usize, byte: u8) {
        self.uarts[n].inject_rx(byte);
    }

    /// Bytes written to the USB-Serial-JTAG TX FIFO (ROM console) since the
    /// last call.
    pub fn take_usb_serial_tx(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.usb_serial_tx)
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
        for _ in 0..cycles {
            self.timg[0].tick(1);
            self.timg[1].tick(1);
            self.systimer.tick(1);
            self.ledc.tick();
            self.spi[0].tick(1);
            self.spi[1].tick(1);
            self.i2c[0].tick(1);
            self.i2c[1].tick(1);
            self.adc.tick(1);
            self.rmt.tick();
            self.mcpwm.tick();
            self.sdm.tick();
            self.lcd_cam.tick();
            self.i2s[0].tick();
            self.i2s[1].tick();
            // PCNT samples its unit/channel signal inputs via the GPIO-matrix
            // input routing (FUNC_IN_SEL_CFG); resolve each signal index to the
            // GPIO pin's current level.
            let pcnt_input = |sig: u32| -> u32 {
                match self.gpio.in_sel(sig) {
                    Some((pin, inv)) => self.gpio.pin_level(pin) ^ (inv as u32),
                    None => 0,
                }
            };
            self.pcnt.tick(&pcnt_input);
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
                // TX FIFO @ +0: byte write = console char (ROM uart_tx_one_char);
                // the ROM's status poll reads +4 bit 1 = "writable" (always set —
                // no FIFO backpressure modeled); reads of +0 return 0 (RX empty).
                if is_write {
                    if off == 0 {
                        self.usb_serial_tx.push((value & 0xFF) as u8);
                    }
                    0
                } else if off == 4 {
                    2
                } else {
                    0
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
                                                    let w = self
                                                        .read32(crate::aes::AES_TEXT_OUT_BASE + k);
                                                    self.write32(ibuf + k, w);
                                                    k += 4;
                                                }
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
                                        let w = self.read32(crate::aes::AES_TEXT_OUT_BASE + k);
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
                    // block (0x100..0x200) is handled inside `Rtc`.
                    if is_write {
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
                                                    let w = self
                                                        .read32(crate::aes::AES_TEXT_OUT_BASE + k);
                                                    self.write32(ibuf + k, w);
                                                    k += 4;
                                                }
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
                                        let w = self.read32(crate::aes::AES_TEXT_OUT_BASE + k);
                                        self.write32(buf + k, w);
                                        k += 4;
                                    }
                                    self.crypto_dma.enable_in_int_all(ch);
                                    self.crypto_dma.raise_in_done(ch);
                                }
                            }
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
}

impl Bus for Soc {
    fn int_pending(&mut self, cpu: usize) -> u32 {
        // Peripheral sources asserted per the TRM interrupt-source table
        // (esp32s3 interrupts.h ETS_*_INTR_SOURCE numbers): UART0/1/2 =
        // 27/28/29, TIMG0 T0/T1/WDT = 50/51/52, TIMG1 T0/T1/WDT =
        // 53/54/55, SYSTIMER target0/1/2 = 57/58/59 (56 = CACHE_IA — the
        // cache-invalid-access source, NOT a systimer source!).  Each
        // peripheral gates its line on INT_ST = RAW & ENA (QEMU
        // esp32_timg.c / esp32_uart.c update_irq); the matrix then
        // resolves the asserted sources to the requesting CPU's lines
        // (per-CPU core_0/core_1 maps, TRM interrupt matrix).  u128 bitmap:
        // sources 79/80 (cross-core) and 94/95 sit beyond u64.
        let mut src = 0u128;
        for (i, u) in self.uarts.iter().enumerate() {
            if u.int_st() != 0 {
                src |= 1 << (27 + i);
            }
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
        // SYSTIMER target0/1/2 = sources 57/58/59 (esp32s3 interrupts.h).
        let sst = self.systimer.int_st();
        for n in 0..3 {
            if sst & (1 << n) != 0 {
                src |= 1 << (57 + n);
            }
        }
        // I2C master (I2CEXT0/1) = sources 42/43, GPSPI2/GPSPI3 = 21/22
        // (esp32s3 interrupts.h ETS_I2C_EXT*_INTR_SOURCE /
        // ETS_SPI2/3_INTR_SOURCE). The driver ISR waits on a semaphore for
        // trans_complete / trans_done, so the done bit must reach the CPU or
        // the Arduino Wire/SPI library blocks forever.
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
        // RMT TX (channels 0..3) = source 40 (ETS_RMT_INTR_SOURCE).  The
        // Arduino/esp-idf RMT driver waits on the tx_end interrupt (or a
        // semaphore given from its ISR), so the done bit must reach the CPU
        // or rmt_write_items blocks forever.
        if self.rmt.int_st() != 0 {
            src |= 1 << crate::rmt::RMT_INTR_SOURCE;
        }
        // PCNT threshold events (units 0..3) = source 41 (ETS_PCNT_INTR_SOURCE).
        if self.pcnt.int_st() != 0 {
            src |= 1 << crate::pcnt::PCNT_INTR_SOURCE;
        }
        // GDMA (shared interrupt, sources = OUT/IN channel done/eof). The
        // firmware GDMA ISR (esp-idf gdma_hal) reads each channel's int_st and
        // dispatches to the registered tx/rx-event callback.
        if self.gdma.int_pending() {
            src |= 1 << crate::gdma::GDMA_INTR_SOURCE;
        }
        // Crypto/shared GDMA (0x6003F000) shares the same GDMA interrupt source;
        // its RX/TX done must also deliver the ISR so the esp-idf GDMA driver's
        // completion callback (which clears the AES driver's "dma done" flag that
        // `esp_aes_dma_done` polls) runs.
        if self.crypto_dma.int_pending() {
            src |= 1 << crate::gdma::GDMA_INTR_SOURCE;
        }
        // AES DMA-done interrupt (`ETS_AES_INTR_SOURCE`, 77). The esp-idf AES
        // driver registers its completion ISR on this source (NOT the GDMA
        // interrupt) and waits on `op_complete_sem`.
        if self.aes.int_pending() {
            src |= 1 << 77;
        }
        // TWAI (CAN) controller = source 37 (ETS_TWAI_INTR_SOURCE).
        if self.twai.int_pending() {
            src |= 1 << crate::twai::TWAI_INTR_SOURCE;
        }
        // MCPWM0 group = source 31 (ETS_PWM0_INTR_SOURCE).
        if self.mcpwm.int_pending() {
            src |= 1 << crate::mcpwm::MCPWM_INTR_SOURCE;
        }
        // RSA accelerator = source 95 (ETS_RSA_INTR_SOURCE). The esp-idf RSA
        // driver polls the peripheral `QUERY_INTERRUPT` register for completion,
        // but wiring the matrix source too is harmless (covered if the driver
        // path ever registers an ISR on this source).
        if self.rsa.int_pending() {
            src |= 1 << crate::rsa::RSA_INTR_SOURCE;
        }
        // ECDSA accelerator = source 97 (ETS_ECDSA_INTR_SOURCE). The esp-idf
        // ECDSA driver polls `QUERY_INTERRUPT`/`INT_RAW` for completion, but
        // wiring the matrix source is harmless (covered if it ever registers an
        // ISR on this source).
        if self.ecdsa.int_pending() {
            src |= 1 << crate::ecdsa::ECDSA_INTR_SOURCE;
        }
        // Cross-core interrupts: SYSTEM.CPU_INT_FROM_CPU_0/1 (0x600C0030/34)
        // assert the FROM_CPU_INTR0/1 sources = 79/80 (esp32s3 interrupts.h
        // ETS_FROM_CPU_INTR0/1; the esp-idf crosscore_int.c allocates
        // ETS_FROM_CPU_INTR0 on core 0 and ETS_FROM_CPU_INTR1 on core 1).
        // FreeRTOS SMP depends on this to force scheduler yields on the
        // target core (ipc_task's esp_crosscore_int_send_yield); without it
        // a blocking task never gets switched out and the boot stalls.
        // NOTE: the bitmap must be u128 — source 79/80 exceed u64's range.
        if self.cpu_int_from_cpu[cpu] & 1 != 0 {
            src |= 1 << (79 + cpu);
        }
        self.intc.pending_lines(cpu, src)
    }

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

    fn read32(&mut self, addr: u32) -> u32 {
        let addr = ioblock_remap(addr);
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE)
        {
            u32::from_le_bytes([
                self.ram8(addr),
                self.ram8(addr + 1),
                self.ram8(addr + 2),
                self.ram8(addr + 3),
            ])
        } else if in_range!(addr, IROM_BASE, IROM_SIZE) {
            let o = (addr - IROM_BASE) as usize;
            u32::from_le_bytes([
                self.irom[o],
                self.irom[o + 1],
                self.irom[o + 2],
                self.irom[o + 3],
            ])
        } else if in_range!(addr, FLASH_DATA_BASE, FLASH_WINDOW_SIZE)
            || in_range!(addr, FLASH_INST_BASE, FLASH_WINDOW_SIZE)
        {
            u32::from_le_bytes([
                self.cache_read8(addr),
                self.cache_read8(addr + 1),
                self.cache_read8(addr + 2),
                self.cache_read8(addr + 3),
            ])
        } else if in_range!(addr, RTC_SLOW_BASE, RTC_SLOW_SIZE) {
            let o = (addr - RTC_SLOW_BASE) as usize;
            u32::from_le_bytes([
                self.rtc_slow[o],
                self.rtc_slow[o + 1],
                self.rtc_slow[o + 2],
                self.rtc_slow[o + 3],
            ])
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
            let o = (addr - base) as usize;
            u32::from_le_bytes([
                self.rtc_fast[o],
                self.rtc_fast[o + 1],
                self.rtc_fast[o + 2],
                self.rtc_fast[o + 3],
            ])
        } else if in_range!(addr, APB_START, APB_END) {
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

    fn write32(&mut self, addr: u32, val: u32) {
        let addr = ioblock_remap(addr);
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, IRAM_WINDOW_SIZE)
        {
            for (i, b) in val.to_le_bytes().into_iter().enumerate() {
                self.ram_write8(addr + i as u32, b);
            }
        } else if in_range!(addr, IROM_BASE, IROM_SIZE) {
            // ROM: read-only.
        } else if in_range!(addr, FLASH_DATA_BASE, FLASH_WINDOW_SIZE)
            || in_range!(addr, FLASH_INST_BASE, FLASH_WINDOW_SIZE)
        {
            // Cache windows: only MMU-mapped PSRAM pages are writable.
            for (i, b) in val.to_le_bytes().into_iter().enumerate() {
                self.cache_write8(addr + i as u32, b);
            }
        } else if in_range!(addr, RTC_SLOW_BASE, RTC_SLOW_SIZE) {
            let o = (addr - RTC_SLOW_BASE) as usize;
            self.rtc_slow[o..o + 4].copy_from_slice(&val.to_le_bytes());
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
            let o = (addr - base) as usize;
            self.rtc_fast[o..o + 4].copy_from_slice(&val.to_le_bytes());
        } else if in_range!(addr, APB_START, APB_END) {
            self.mmio32(addr, true, val);
        }
    }
}
