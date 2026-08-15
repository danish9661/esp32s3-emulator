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
use crate::cache::{Cache, CacheTarget};
use crate::gpio::Gpio;
use crate::i2c::I2c;
use crate::intc::Intc;
use crate::ledc::Lcdc;
use crate::memmap::*;
use crate::spi::Spi;
use crate::timg::{INT_T0, INT_T1, INT_WDT, Timg};
use crate::uart::Uart;

macro_rules! in_range {
    ($addr:expr, $base:expr, $size:expr) => {
        ($base..$base + $size).contains(&$addr)
    };
}

// SRAM window size used in range checks (kept in sync with DRAM_SIZE).
const SRAM_BASE_RANGE: u32 = DRAM_SIZE;

pub struct Soc {
    /// Internal SRAM backing store (aliased at DRAM_BASE and IRAM_BASE).
    sram: Box<[u8; SRAM_BYTES]>,
    /// Boot ROM storage (IROM, read-only — P3 loads the ROM stub here).
    irom: Box<[u8; IROM_SIZE as usize]>,
    /// SPI flash backing store (read-only via the XIP cache windows).
    flash: Box<[u8; FLASH_SIZE as usize]>,
    /// PSRAM backing store (read-write through MMU-mapped cache pages).
    psram: Box<[u8; PSRAM_SIZE as usize]>,
    rtc_slow: Box<[u8; RTC_SLOW_SIZE as usize]>,
    rtc_fast: Box<[u8; RTC_FAST_SIZE as usize]>,
    uarts: [Uart; 3],
    gpio: Gpio,
    ledc: Lcdc,
    spi: [Spi; 2],
    i2c: [I2c; 2],
    adc: Adc,
    cache: Cache,
    timg: [Timg; 2],
    intc: Intc,
}

impl Soc {
    pub fn new() -> Self {
        Self {
            sram: Box::new([0; SRAM_BYTES]),
            irom: Box::new([0; IROM_SIZE as usize]),
            flash: Box::new([0; FLASH_SIZE as usize]),
            psram: Box::new([0; PSRAM_SIZE as usize]),
            rtc_slow: Box::new([0; RTC_SLOW_SIZE as usize]),
            rtc_fast: Box::new([0; RTC_FAST_SIZE as usize]),
            uarts: [Uart::new(), Uart::new(), Uart::new()],
            gpio: Gpio::new(),
            ledc: Lcdc::new(),
            spi: [Spi::new(0), Spi::new(1)],
            i2c: [I2c::new(0), I2c::new(1)],
            adc: Adc::new(),
            cache: Cache::new(),
            timg: [Timg::new(), Timg::new()],
            intc: Intc::new(),
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

    /// Bytes emitted by UART `n` since the last call (host console output).
    pub fn take_uart_tx(&mut self, n: usize) -> Vec<u8> {
        self.uarts[n].take_tx()
    }

    /// Push one received byte into UART `n`'s RX FIFO (host console input).
    pub fn uart_inject_rx(&mut self, n: usize, byte: u8) {
        self.uarts[n].inject_rx(byte);
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
            out |= if sel == 0x80 {
                self.gpio.out_bit(i)
            } else {
                self.signal_level(sel)
            } << i;
        }
        out
    }

    /// Advance timer groups by `cycles` (frontend time source).
    pub fn tick_timers(&mut self, cycles: u64) {
        for _ in 0..cycles {
            self.timg[0].tick(1);
            self.timg[1].tick(1);
            self.ledc.tick(1);
            self.spi[0].tick(1);
            self.spi[1].tick(1);
            self.i2c[0].tick(1);
            self.i2c[1].tick(1);
            self.adc.tick(1);
        }
    }

    /// Internal SRAM access at `addr` (must be inside DRAM or IRAM window).
    /// Both windows alias the same 512 KB physical SRAM.
    fn ram_index(addr: u32) -> usize {
        let idx = if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE) {
            addr - DRAM_BASE
        } else {
            addr - IRAM_BASE
        };
        idx as usize
    }

    fn ram8(&self, addr: u32) -> u8 {
        self.sram[Self::ram_index(addr)]
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
        let idx = Self::ram_index(addr);
        self.sram[idx] = val;
    }

    /// GPIO matrix output signal level for a peripheral signal index.
    /// LEDC occupies 73..80, I2CEXT0 SCL/SDA = 89/90, I2CEXT1 = 91/92,
    /// GPSPI2 (FSPI) 101..105 + CS 110/111, GPSPI3 66..72 (S3
    /// gpio_sig_map.h); everything else reads 0.
    fn signal_level(&self, sig: u32) -> u32 {
        if (73..=80).contains(&sig) {
            self.ledc.signal_level(sig)
        } else {
            self.spi[0].signal_level(sig)
                | self.spi[1].signal_level(sig)
                | self.i2c[0].signal_level(sig)
                | self.i2c[1].signal_level(sig)
        }
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
                if is_write {
                    self.gpio.write32(off, value);
                    0
                } else {
                    self.gpio.read32(off)
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
                let n = if dev == SPI2_BASE { 0 } else { 1 };
                if is_write {
                    self.spi[n].write32(off, value);
                    0
                } else {
                    self.spi[n].read32(off)
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
            // Page 0x6000_8000 holds RTC_CNTL (0x000), RTC_IO (0x400),
            // SENS (0x800, the SAR ADC RTC oneshot controller) and
            // RTC_MEM (0xC00); only SENS is modeled.
            0x6000_8000 => {
                if in_range!(off, 0x800, 0x400) {
                    if is_write {
                        self.adc.sens_write32(off - 0x800, value);
                        0
                    } else {
                        self.adc.sens_read32(off - 0x800)
                    }
                } else {
                    0
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
            TIMG0_BASE | TIMG1_BASE => {
                let n = if dev == TIMG0_BASE { 0 } else { 1 };
                if is_write {
                    self.timg[n].write32(off, value);
                    0
                } else {
                    self.timg[n].read32(off)
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

impl Bus for Soc {
    fn int_pending(&mut self, cpu: usize) -> u32 {
        // Peripheral sources asserted per the TRM interrupt-source table
        // (QEMU esp32s3_intc.h ETS_*_INTR_SOURCE numbers): UART0/1/2 =
        // 27/28/29, TIMG0 T0/T1/WDT = 50/51/52, TIMG1 T0/T1/WDT =
        // 53/54/55.  Each peripheral gates its line on INT_ST = RAW & ENA
        // (QEMU esp32_timg.c / esp32_uart.c update_irq); the matrix then
        // resolves the asserted sources to the requesting CPU's lines
        // (per-CPU core_0/core_1 maps, TRM interrupt matrix).
        let mut src = 0u64;
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
        self.intc.pending_lines(cpu, src)
    }

    fn read8(&mut self, addr: u32) -> u32 {
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, SRAM_BASE_RANGE)
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
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, SRAM_BASE_RANGE)
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
        } else if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE) {
            let o = (addr - RTC_FAST_BASE) as usize;
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
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, SRAM_BASE_RANGE)
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
        } else if in_range!(addr, APB_START, APB_END) {
            self.mmio32(addr, true, val);
        }
    }

    fn write16(&mut self, addr: u32, val: u32) {
        // RAM regions: byte-lane merge. MMIO: APB registers are 32-bit;
        // a sub-word write is approximated as a full-word store.
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, SRAM_BASE_RANGE)
        {
            self.ram_write8(addr, val as u8);
            self.ram_write8(addr + 1, (val >> 8) as u8);
        } else {
            self.write32(addr, val);
        }
    }

    fn write32(&mut self, addr: u32, val: u32) {
        if in_range!(addr, DRAM_BASE, SRAM_BASE_RANGE)
            || in_range!(addr, IRAM_BASE, SRAM_BASE_RANGE)
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
        } else if in_range!(addr, RTC_FAST_BASE, RTC_FAST_SIZE) {
            let o = (addr - RTC_FAST_BASE) as usize;
            self.rtc_fast[o..o + 4].copy_from_slice(&val.to_le_bytes());
        } else if in_range!(addr, APB_START, APB_END) {
            self.mmio32(addr, true, val);
        }
    }
}
