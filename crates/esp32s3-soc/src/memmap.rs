//! ESP32-S3 memory map constants.
//!
//! Addresses are authoritative from QEMU's `include/hw/misc/esp32s3_reg.h`
//! (peripheral bases) and `hw/xtensa/esp32s3.c` (memory region layout), which
//! mirror the ESP32-S3 Technical Reference Manual (memory map chapter).

// ── CPU-visible memory regions (QEMU esp32s3_memmap) ────────────────────────

/// Read-only ROM region; holds the 1st-stage boot ROM (P3 uses this).
pub const IROM_BASE: u32 = 0x4000_0000;
pub const IROM_SIZE: u32 = 0x0006_0000;

/// Instruction-side alias of the internal SRAM (512 KB, same physical RAM
/// as DRAM — data/instruction views of the same SRAM on real silicon).
pub const IRAM_BASE: u32 = 0x4037_0000;
pub const IRAM_SIZE: u32 = 0x0008_0000;

/// Data-side internal SRAM (512 KB).
pub const DRAM_BASE: u32 = 0x3FC8_0000;
pub const DRAM_SIZE: u32 = 0x0008_0000;

/// RTC slow memory (8 KB, data).
pub const RTC_SLOW_BASE: u32 = 0x5000_0000;
pub const RTC_SLOW_SIZE: u32 = 0x0000_2000;

/// GPSPI2 (general-purpose SPI), TRM GPSPI chapter.
pub const SPI2_BASE: u32 = 0x6002_4000;
/// GPSPI3.
pub const SPI3_BASE: u32 = 0x6002_8000;

/// LEDC (LED PWM controller), TRM LEDC chapter.
pub const LEDC_BASE: u32 = 0x6001_9000;

/// RTC fast memory (8 KB, data alias in APB space).
pub const RTC_FAST_BASE: u32 = 0x600F_E000;
pub const RTC_FAST_SIZE: u32 = 0x0000_2000;

/// SPI flash, memory-mapped read-only through the data-cache window (TRM
/// memory map: 0x3C000000..0x3DFFFFFF). The same flash appears at the
/// instruction-cache window (0x42000000..0x427FFFFF) once the MMU/cache is
/// initialized; both windows alias one physical flash here.
pub const FLASH_DATA_BASE: u32 = 0x3C00_0000;
pub const FLASH_INST_BASE: u32 = 0x4200_0000;
/// Size of each cache window (16 MB virtual each).
pub const FLASH_WINDOW_SIZE: u32 = 0x0100_0000;
/// Physical flash size modeled (4 MB, ESP32-S3 flash is 2-16 MB).
pub const FLASH_SIZE: u32 = 0x0040_0000;

// ── Peripheral bases (QEMU esp32s3_reg.h) ───────────────────────────────────

pub const UART0_BASE: u32 = 0x6000_0000;
pub const UART1_BASE: u32 = 0x6001_0000;
pub const UART2_BASE: u32 = 0x6002_E000;

pub const GPIO_BASE: u32 = 0x6000_4000;

pub const TIMG0_BASE: u32 = 0x6001_F000;
pub const TIMG1_BASE: u32 = 0x6002_0000;

/// Interrupt matrix: maps peripheral interrupt sources to CPU interrupt lines.
/// Register space = 512 sources × 2 CPUs × 4 bytes.
pub const INT_MATRIX_BASE: u32 = 0x600C_2000;
pub const INT_MATRIX_INPUTS: usize = 0x800 / 4;
pub const INT_MATRIX_CPUS: usize = 2;
pub const INT_MATRIX_SIZE: u32 = (INT_MATRIX_INPUTS * INT_MATRIX_CPUS * 4) as u32;

/// First/last APB peripheral address (everything inside is either handled by
/// a device or returns 0 / ignores writes like QEMU's unimplemented regions).
pub const APB_START: u32 = 0x6000_0000;
pub const APB_END: u32 = 0x6010_0000;

/// Total internal SRAM in bytes.
pub const SRAM_BYTES: usize = DRAM_SIZE as usize;
