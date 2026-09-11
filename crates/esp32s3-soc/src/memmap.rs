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

/// SRAM0 (32 KB): instruction-only internal SRAM (I-bus 0x40370000-
/// 0x40377FFF).  NO data-bus window — the first 16/32 KB is the Icache
/// storage for external memory when the instruction cache is on (Espressif
/// "ESP32-S3 Memory map" pdf; ESP-IDF memory.ld.in `SRAM_IRAM_ORG =
/// SRAM_IRAM_START + CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE`; heap
/// memory_layout.c lists SRAM0 as IRAM-only with no DRAM alias).  The ROM
/// loader still writes it through the I-bus addresses (D-port reaches the
/// I-window), but the data-side addresses 0x3FC80000-0x3FC87FFF are a
/// separate 32 KB D-only region (ROM data / boot stack area, QEMU DRAM
/// region start).
pub const SRAM0_SIZE: u32 = 0x0000_8000;

/// D/IRAM: the 416 KB SRAM1 block, reachable from BOTH buses — data
/// window 0x3FC88000-0x3FCEFFFF, instruction window 0x40378000-0x403DFFFF
/// (Espressif "ESP32-S3 Memory map" pdf; ESP-IDF memory.ld.in
/// `I_D_SRAM_OFFSET (SRAM_DIRAM_I_START - SRAM_DRAM_START) = 0x6F0000`).
/// Alias: data = instruction - 0x6F0000.  The linker relies on it
/// (esp32s3.ld dram0_0_seg is placed right after the text's data-side
/// alias).  The final 64 KB of the old window (0x403E0000+) is unmapped.
pub const DIRAM_DATA_BASE: u32 = 0x3FC8_8000;
pub const DIRAM_INST_BASE: u32 = 0x4037_8000;
pub const DIRAM_SIZE: u32 = 0x0006_8000;

/// Instruction-window size that is actually backed (SRAM0 + D/IRAM).
pub const IRAM_WINDOW_SIZE: u32 = SRAM0_SIZE + DIRAM_SIZE;

/// Data-side internal SRAM (512 KB).
pub const DRAM_BASE: u32 = 0x3FC8_0000;
pub const DRAM_SIZE: u32 = 0x0008_0000;

/// RTC slow memory (8 KB, data).
pub const RTC_SLOW_BASE: u32 = 0x5000_0000;
pub const RTC_SLOW_SIZE: u32 = 0x0000_2000;

/// SPI1 (SPIMEM1): the flash controller the CPU-side esp_flash driver talks
/// to (TRM SPI chapter, spi_mem_struct.h; `spimem_flash_ll_get_hw` returns
/// &SPIMEM1 for SPI1_HOST).
pub const SPI1_BASE: u32 = 0x6000_2000;
/// SPI0 (SPIMEM0): the cache-side flash controller, same register layout
/// (QEMU esp32s3_soc.c creates both with the esp32s3_spi model).
pub const SPIMEM0_BASE: u32 = 0x6000_3000;

/// GPSPI2 (general-purpose SPI), TRM GPSPI chapter.
pub const SPI2_BASE: u32 = 0x6002_4000;
/// GPSPI3 (general-purpose SPI). TRM GPSPI chapter. NOTE: on ESP32-S3 GPSPI3
/// is at 0x6002_5000; 0x6002_8000 is the SD/MMC host controller (SDMMC_BASE),
/// a distinct peripheral.
pub const SPI3_BASE: u32 = 0x6002_5000;

/// LEDC (LED PWM controller), TRM LEDC chapter.
pub const LEDC_BASE: u32 = 0x6001_9000;

/// SENS RTC controller — the SAR ADC oneshot path (SAR ADC chapter of the
/// TRM; sens_struct.h).  NOT page-aligned: it sits in the 0x6000_8000 page
/// alongside RTC_CNTL (0x000) / RTC_IO (0x400) / RTC_MEM (0xC00).
pub const SENS_BASE: u32 = 0x6000_8800;

/// APB_SARADC digital controller (SAR ADC continuous/DMA path,
/// apb_saradc_struct.h).
pub const APB_SARADC_BASE: u32 = 0x6004_0000;

/// I2C0 (I2C EXT0), TRM I2C chapter.
pub const I2C0_BASE: u32 = 0x6001_3000;
/// I2C1 (I2C EXT1), 0x14000 apart from I2C0.
pub const I2C1_BASE: u32 = 0x6002_7000;

/// RTC fast memory: 8 KB executable, `rtc_iram_seg @ 0x600FE000`
/// (esp32s3 memory.ld; `SOC_RTC_IRAM_LOW/HIGH` in soc.h). Both the
/// instruction and data views are that same address on S3 (unlike the
/// classic ESP32's separate 0x3FF80000 data alias).
pub const RTC_FAST_BASE: u32 = 0x600F_E000;
pub const RTC_FAST_SIZE: u32 = 0x0000_2000;
/// ROM constant tables (`esp32s3.rom.ld`: `.rodata @ 0x3FF18C00`,
/// `ets_rom_layout_p = 0x3FF1FFFC`). A SEPARATE memory from RTC fast —
/// aliasing them into one array let `load_rom_data` stomp the app's RTC
/// segment (e.g. IPC/sleep helpers at 0x600FE000) with ROM bytes.
/// `ets_rom_layout_p` itself is written host-side at boot (the word the
/// app's heap init dereferences).
pub const ROM_DATA_BASE: u32 = 0x3FF1_8000;
pub const ROM_DATA_SIZE: u32 = 0x0000_8000;

/// USB-Serial-JTAG controller (0x60038000) — the boot ROM's console output
/// goes through its TX FIFO (uart_tx_one_char @ 0x40048C30 writes
/// 0x60038000); the ROM's status poll reads 0x60038004 bit 1 (writable).
pub const USB_SERIAL_JTAG_BASE: u32 = 0x6003_8000;
pub const USB_SERIAL_JTAG_SIZE: u32 = 0x80;

/// SPI flash / PSRAM cache windows. The data-cache window (0x3C000000) and
/// instruction-cache window (0x42000000) are both 32 MB aliases over ONE
/// shared cache MMU (QEMU esp32s3_cache.h: `ESP32S3_EXTMEM_REGION_SIZE
/// 0x2000000`, both `dcache`/`icache` alias the same MMU IOMMU region). Each
/// 64 KB virtual page maps through an MMU entry to a physical flash page
/// (read-only) or PSRAM page (read-write); see `crate::cache`.
pub const FLASH_DATA_BASE: u32 = 0x3C00_0000;
pub const FLASH_INST_BASE: u32 = 0x4200_0000;
/// Size of each cache window (32 MB virtual each, QEMU ESP32S3_EXTMEM_REGION_SIZE).
pub const FLASH_WINDOW_SIZE: u32 = 0x0200_0000;
/// Physical flash size modeled (4 MB, ESP32-S3 flash is 2-16 MB).
pub const FLASH_SIZE: u32 = 0x0040_0000;
/// Physical PSRAM size modeled (16 MB — covers both the 8 MB DevKit
/// default (MR2=3) and 16 MB parts (MR2=5); smaller densities simply never
/// map the upper pages).
pub const PSRAM_SIZE: u32 = 0x0100_0000;
/// Cache MMU virtual page size (64 KB, QEMU ESP32S3_PAGE_SIZE).
pub const CACHE_PAGE_SIZE: u32 = 0x0001_0000;
/// Shared cache MMU entry count (32 MB window / 64 KB page).
pub const MMU_ENTRIES: usize = 512;
/// Cache window offset mask: both bases share the low 25 bits, so the window
/// offset is `vaddr & WINDOW_MASK` for either the data or instruction window.
pub const WINDOW_MASK: u32 = FLASH_WINDOW_SIZE - 1;
/// Window offset of the loader's scratch mapping: the app image is
/// host-mapped here (64 KB pages above the app's own mappings) so the ROM
/// stub can read it through the data window even after the app's
/// flash-mapped pages are programmed (the I/D windows share one MMU table,
/// so the app image at flash offset 0x10000 cannot stay readable at its 1:1
/// offset once the instruction window maps those pages).
pub const LOADER_SCRATCH_OFF: u32 = 0x001F_0000;

// ── Peripheral bases (QEMU esp32s3_reg.h) ───────────────────────────────────

pub const UART0_BASE: u32 = 0x6000_0000;
pub const UART1_BASE: u32 = 0x6001_0000;
pub const UART2_BASE: u32 = 0x6002_E000;

/// UHCI0 DMA bridge base (`DR_REG_UHCI0_BASE`, soc/reg_base.h).
pub const UHCI0_BASE: u32 = 0x6001_4000;

pub const GPIO_BASE: u32 = 0x6000_4000;

pub const TIMG0_BASE: u32 = 0x6001_F000;
pub const TIMG1_BASE: u32 = 0x6002_0000;

/// System timer (esp32s3 reg_base.h DR_REG_SYSTIMER_BASE): 2× 52-bit
/// counters, 3 alarm targets; esp_timer's clock source on the S3.
pub const SYSTIMER_BASE: u32 = 0x6002_3000;

/// eFuse controller (esp32s3 reg_base.h DR_REG_EFUSE_BASE): holds the read-data
/// registers that mirror the eFuse array blocks (MAC, chip version, keys...).
pub const EFUSE_BASE: u32 = 0x6000_7000;

/// SHA acceleration peripheral (`DR_REG_SHA_BASE`, soc/reg_base.h): the
/// message blocks are fed through the GDMA (`SOC_GDMA_TRIG_PERIPH_SHA0`).
pub const SHA_BASE: u32 = 0x6003_B000;

/// Interrupt matrix: maps peripheral interrupt sources to CPU interrupt lines.
/// Register space = 512 sources × 2 CPUs × 4 bytes.
pub const INT_MATRIX_BASE: u32 = 0x600C_2000;
pub const INT_MATRIX_INPUTS: usize = 0x800 / 4;
pub const INT_MATRIX_CPUS: usize = 2;
pub const INT_MATRIX_SIZE: u32 = (INT_MATRIX_INPUTS * INT_MATRIX_CPUS * 4) as u32;

/// SYSTEM peripheral base (TRM memory map). Only APPCPU_CTRL_A
/// (base + 0x04, the APP-CPU release register) is modeled by the SoC.
pub const SYSTEM_BASE: u32 = 0x600C_0000;

/// P5 register-store peripheral bases (esp-idf components/soc/esp32s3/
/// register/soc/reg_base.h). These are configure-and-forget blocks; the
/// emulator retains pokes in a `RegStore` (see regstore.rs) so firmware
/// touching them during boot/init never panics.
pub const I2S0_BASE: u32 = 0x6000_F000;
pub const I2S1_BASE: u32 = 0x6002_D000;
pub const SYSCON_BASE: u32 = 0x6002_6000; // sysclk/tick/out config (NOT peripheral clocks: those are SYSTEM_PERIP_CLK_EN0/1)
pub const PERI_BACKUP_BASE: u32 = 0x6002_A000;
pub const LCD_CAM_BASE: u32 = 0x6004_1000;
pub const SENSITIVE_BASE: u32 = 0x600C_1000;
pub const ASSIST_DEBUG_BASE: u32 = 0x600C_E000;
pub const WCL_BASE: u32 = 0x600D_0000;

/// Cache / MMU controller registers (EXTMEM, esp32s3_reg.h): dcache/icache
/// enable, sync/preload/autoload/freeze handshakes, cache state.
pub const EXTMEM_BASE: u32 = 0x600C_4000;
/// Shared cache MMU table (512 x u32, 64 KB pages). `ESP32S3_MMU_TABLE_OFFSET`
/// = DR_REG_MMU_TABLE - DR_REG_EXTMEM_BASE = 0x1000 (QEMU esp32s3_cache.h).
pub const MMU_TABLE_BASE: u32 = 0x600C_5000;

/// First/last APB peripheral address (everything inside is either handled by
/// a device or returns 0 / ignores writes like QEMU's unimplemented regions).
pub const APB_START: u32 = 0x6000_0000;
pub const APB_END: u32 = 0x6010_0000;

/// Total internal SRAM in bytes.
pub const SRAM_BYTES: usize = DRAM_SIZE as usize;
