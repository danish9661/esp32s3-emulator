//! ESP32-S3 SoC: full address-space `Bus` implementation.
//!
//! Owns the internal SRAM (512 KB, aliased at the DRAM and IRAM windows —
//! same physical RAM on real silicon), the boot ROM stub storage (IROM), the
//! RTC memories, and all modeled peripherals (UART0/1/2, GPIO, TIMG0/1,
//! interrupt matrix). Addresses and windows follow QEMU `esp32s3.c` +
//! `esp32s3_reg.h` (mirrors the TRM memory map chapter).
//!
//! Unimplemented APB addresses behave like QEMU's unimplemented devices but
//! return 0 / ignore writes instead of trapping (firmware probes peripheral
//! space during boot).

use alloc::boxed::Box;
use alloc::vec::Vec;
use xtensa_core::Bus;

use crate::gpio::Gpio;
use crate::intc::Intc;
use crate::memmap::*;
use crate::timg::Timg;
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
    rtc_slow: Box<[u8; RTC_SLOW_SIZE as usize]>,
    rtc_fast: Box<[u8; RTC_FAST_SIZE as usize]>,
    uarts: [Uart; 3],
    gpio: Gpio,
    timg: [Timg; 2],
    intc: Intc,
}

impl Soc {
    pub fn new() -> Self {
        Self {
            sram: Box::new([0; SRAM_BYTES]),
            irom: Box::new([0; IROM_SIZE as usize]),
            flash: Box::new([0; FLASH_SIZE as usize]),
            rtc_slow: Box::new([0; RTC_SLOW_SIZE as usize]),
            rtc_fast: Box::new([0; RTC_FAST_SIZE as usize]),
            uarts: [Uart::new(), Uart::new(), Uart::new()],
            gpio: Gpio::new(),
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

    /// Snapshot of driven output-pin state (host LED visualization).
    pub fn gpio_output(&self) -> u32 {
        self.gpio.output()
    }

    /// Advance timer groups by `cycles` (frontend time source).
    pub fn tick_timers(&mut self, cycles: u64) {
        for _ in 0..cycles {
            self.timg[0].tick(1);
            self.timg[1].tick(1);
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

    /// Flash byte at `addr` (inside a cache window, beyond physical flash
    /// returns 0 like an unmapped cache line).
    fn flash8(&self, addr: u32) -> u8 {
        let idx = if addr >= FLASH_INST_BASE {
            (addr - FLASH_INST_BASE) as usize
        } else {
            (addr - FLASH_DATA_BASE) as usize
        };
        if idx >= FLASH_SIZE as usize {
            return 0;
        }
        self.flash[idx]
    }

    fn ram_write8(&mut self, addr: u32, val: u8) {
        let idx = Self::ram_index(addr);
        self.sram[idx] = val;
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
            self.flash8(addr) as u32
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
                self.flash8(addr),
                self.flash8(addr + 1),
                self.flash8(addr + 2),
                self.flash8(addr + 3),
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
