//! ESP32-S3 SPI1/SPIMEM0 flash controller (memspi) + minimal SPI NOR chip.
//!
//! Register layout and transaction semantics are ported from QEMU's
//! `esp32s3_spi.c` (espressif/qemu, GPLv2) and `esp32s3_spi.h`; the flash
//! chip model mirrors QEMU's `m25p80.c` for a Winbond w25q32 (JEDEC id
//! 0xEF4016, 4 MB, 64 KB blocks / 4 KB sectors).
//!
//! SPI1 also carries the external RAM on CS1 (selected via MISC.cs bits).
//! The PSRAM device answers the ID probe (density/KGD 0x5D) and init/test
//! pattern traffic; runtime heap access bypasses SPI1 (cache MMU).
//!
//! Modeled:
//! - Writing CMD.USR (bit 18) executes a **USR transaction** synchronously:
//!   command bytes from USER2.USR_COMMAND_VALUE/BITLEN, then (if
//!   USER.USR_ADDR) address bytes from ADDR + USER1.USR_ADDR_BITLEN (the
//!   address is byte-swapped and right-aligned exactly like QEMU), then (if
//!   USER.USR_DUMMY — only meaningful with USR_ADDR, QEMU nests it there)
//!   `(USR_DUMMY_CYCLELEN+7)/8` dummy bytes, then data: MOSI bytes from the
//!   W0..W15 buffer (MOSI_DLEN bits), MISO bytes into it (MISO_DLEN bits).
//!   The command byte is ALWAYS sent even when USER.USR_COMMAND is clear —
//!   QEMU: "In theory we should test mem_user's command bit. In practice,
//!   if we do, esptool cannot write flash successfully".
//! - Writing CMD without USR dispatches the special commands in bits
//!   [31:19] (FLASH_READ/WREN/WRDI/RDID/RDSR/WRSR/PP/SE/BE/CE/DP/RES/HPM)
//!   with QEMU's exact shapes, including the legacy PP length byte in
//!   ADDR[31:24] and RDID/RES/HPM 3-byte reads into W0.
//! - CMD always reads 0 (IDF `spimem_flash_ll_cmd_is_done` polls it);
//!   FSM reads 0 (host idle).  Transactions are instantaneous (no timing).
//!
//! The flash backing is passed in as `&mut [u8]` so the device borrows
//! nothing from the SoC (the SoC owns the storage; cache-window reads are
//! handled separately by `crate::cache` and do NOT go through this
//! controller — the S3's esp_flash driver reads flash through the MMU-mapped
//! data window and uses SPI1 only for id/status/erase/program).
//!
//! The chip model is a byte-stream SPI NOR state machine (m25p80): CS-low
//! transactions decode a command byte, collect address/dummy bytes, then
//! stream data.  JEDEC read replies with the id bytes cyclically (id[0..3]
//! then zeros, then idle); RDSR repeats the status byte (bit 1 = WEL);
//! WREN/WRDI latch write enable; PP ANDs bytes into storage; erase fills
//! 0xFF; FAST_READ consumes 3 addr + 8 dummy bytes (QEMU models Winbond
//! dummy cycles as bytes).  CS-high after each transaction resets the
//! stream state but keeps write-enable, exactly like m25p80_cs().

use alloc::boxed::Box;

// Register offsets (esp32s3_spi.h REG32 list). Public: firmware-style
// flash update flows (esp_ota_write -> PP/SE) are driven through this bus
// by machine tests exactly like the IDF spi_flash driver drives silicon.
pub const REG_CMD: u32 = 0x000;
pub const REG_ADDR: u32 = 0x004;
const REG_CTRL: u32 = 0x008;
const REG_CTRL1: u32 = 0x00C;
const REG_CTRL2: u32 = 0x010;
const REG_CLOCK: u32 = 0x014;
pub const REG_USER: u32 = 0x018;
pub const REG_USER1: u32 = 0x01C;
pub const REG_USER2: u32 = 0x020;
pub const REG_MOSI_DLEN: u32 = 0x024;
const REG_MISO_DLEN: u32 = 0x028;
const REG_RD_STATUS: u32 = 0x02C;
const REG_MISC: u32 = 0x034;
const REG_CACHE_FCTRL: u32 = 0x03C;
const REG_FSM: u32 = 0x054;
pub const REG_W0: u32 = 0x058;
const REG_W15: u32 = 0x094;
const REG_SUS_STATUS: u32 = 0x0A4;
const REG_DDR_CTRL: u32 = 0x0E0;
const REG_CLOCK_GATE: u32 = 0x0E8;

// USER bits (esp32s3_spi.h SPI_MEM_USER).
pub const USER_USR_COMMAND: u32 = 1 << 31;
pub const USER_USR_ADDR: u32 = 1 << 30;
const USER_USR_DUMMY: u32 = 1 << 29;
const USER_USR_MISO: u32 = 1 << 28;
pub const USER_USR_MOSI: u32 = 1 << 27;

// MISC chip-select bits (spi_mem_struct.h `misc`: cs0_dis bit 0 deselects
// the SPI flash, cs1_dis bit 1 deselects the external RAM; both share SPI1).
const MISC_CS0_DIS: u32 = 1 << 0;
const MISC_CS1_DIS: u32 = 1 << 1;

// PSRAM (Ext_RAM) ID reply bytes, MSB-first on the wire (APM 64 Mb part:
// mfr 0x0D, density/KGD 0x5D). A 3-byte MISO read lands the KGD byte at ID
// word bits [15:8], where the esp-idf quad PSRAM probe checks it
// (`PSRAM_KGD(id) == 0x5D`, esp_psram_impl_quad.c).
const PSRAM_ID: [u8; 3] = [0x0D, 0x5D, 0x00];

// PSRAM backing for the CS1 device (pattern/test traffic only — runtime
// heap access goes through the cache MMU's own `psram` array, never SPI1).
// Sized to the physical part (APM 64 Mb = 8 MB).
const PSRAM_DEV_SIZE: usize = 0x0080_0000;

// CMD bits (esp32s3_spi.h SPI_MEM_CMD).  Special-command dispatch mask
// keeps bits [31:19] (QEMU `command >> 19 << 19`).
pub const CMD_USR: u32 = 1 << 18;
const CMD_SPECIAL_MASK: u32 = 0xFFFF_F800;
const CMD_FLASH_READ: u32 = 1 << 31;
pub const CMD_FLASH_WREN: u32 = 1 << 30;
const CMD_FLASH_WRDI: u32 = 1 << 29;
const CMD_FLASH_RDID: u32 = 1 << 28;
const CMD_FLASH_RDSR: u32 = 1 << 27;
const CMD_FLASH_WRSR: u32 = 1 << 26;
const CMD_FLASH_PP: u32 = 1 << 25;
const CMD_FLASH_SE: u32 = 1 << 24;
const CMD_FLASH_BE: u32 = 1 << 23;
const CMD_FLASH_CE: u32 = 1 << 22;
const CMD_FLASH_DP: u32 = 1 << 21;
const CMD_FLASH_RES: u32 = 1 << 20;
const CMD_FLASH_HPM: u32 = 1 << 19;

// Flash commands (m25p80.c FlashCMD enum + esp32s3_spi.c CMD_*).
const CMD_WRSR: u8 = 0x01;
const CMD_PP: u8 = 0x02;
const CMD_READ: u8 = 0x03;
const CMD_WRDI: u8 = 0x04;
const CMD_RDSR: u8 = 0x05;
const CMD_WREN: u8 = 0x06;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_SE: u8 = 0x20;
// Winbond 0x35 = RDCR_EQIO: read status register 2 (bit 1 = QE).
const CMD_RDSR2: u8 = 0x35;
const CMD_CE: u8 = 0x60;
const CMD_BE: u8 = 0xD8;
const CMD_JEDEC: u8 = 0x9F;
const CMD_RES: u8 = 0xAB;
const CMD_HPM: u8 = 0xA3;
const CMD_DP: u8 = 0xB9;
const CMD_CE_ALT: u8 = 0xC7;

/// JEDEC id of the modeled chip (Winbond w25q32, QEMU m25p80 `w25q32` row):
/// mfg 0xEF, type 0x40, capacity 0x16 (2^22 = 4 MB).  Sent MSB-first, so a
/// 3-byte MISO read lands in W0 as 0x1640EF.
const JEDEC_ID: [u8; 3] = [0xEF, 0x40, 0x16];

const REG_COUNT: usize = 0x400 / 4;

/// m25p80 stream state (subset used by the modeled commands).
#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum ChipState {
    /// Waiting for a command byte.
    #[default]
    Idle,
    /// Consuming `need` address/dummy bytes (got so far); the first byte
    /// is also latched into `data[0]` for WRSR.
    Collect { need: u8, got: u8 },
    /// Streaming the internal data buffer (JEDEC id / RDSR status);
    /// repeats when `loop_` is set.
    ReadData,
    /// Streaming storage bytes (READ/FAST_READ).
    Read,
    /// Streaming storage writes (PP).
    Program,
}

/// SPI NOR chip state (m25p80 `Flash` struct subset).
#[derive(Default)]
struct Chip {
    state: ChipState,
    cmd: u8,
    addr: u32,
    write_enable: bool,
    /// Status register 1 (SRWD/BP bits; RDSR reports bit 1 = WEL live).
    status: u8,
    /// Winbond QE (quad enable): status register 2 bit 1 (QEMU m25p80
    /// `quad_enable`, WRSR byte 1 / RDCR_EQIO).
    quad_enable: bool,
    data: [u8; 6],
    dlen: u8,
    dpos: u8,
    loop_: bool,
}

impl Chip {
    /// One byte of a CS-low transaction (m25p80_transfer8 equivalent).
    fn transfer(&mut self, flash: &mut [u8], tx: u8) -> u8 {
        let mask = (flash.len() - 1) as u32;
        match self.state {
            ChipState::Idle => {
                self.decode(flash, tx);
                0
            }
            ChipState::Collect { need, got } => {
                self.data[got as usize] = tx;
                self.addr = (self.addr << 8) | tx as u32;
                let got = got + 1;
                if got == need {
                    self.finish_collect(flash);
                } else {
                    self.state = ChipState::Collect { need, got };
                }
                0
            }
            ChipState::ReadData => {
                let r = self.data[self.dpos as usize];
                self.dpos += 1;
                if self.dpos == self.dlen {
                    self.dpos = 0;
                    if !self.loop_ {
                        self.state = ChipState::Idle;
                    }
                }
                r
            }
            ChipState::Read => {
                let r = flash[self.addr as usize];
                self.addr = (self.addr + 1) & mask;
                r
            }
            ChipState::Program => {
                // NOR program clears bits only (m25p80 flash_write8 ANDs).
                if self.write_enable {
                    flash[self.addr as usize] &= tx;
                }
                self.addr = (self.addr + 1) & mask;
                0
            }
        }
    }

    /// Command decode (m25p80 decode_new_cmd subset).
    fn decode(&mut self, flash: &mut [u8], cmd: u8) {
        self.cmd = cmd;
        match cmd {
            CMD_JEDEC => {
                self.data = [JEDEC_ID[0], JEDEC_ID[1], JEDEC_ID[2], 0, 0, 0];
                self.dlen = 6;
                self.dpos = 0;
                self.loop_ = false;
                self.state = ChipState::ReadData;
            }
            CMD_RDSR => {
                self.data = [
                    self.status | ((self.write_enable as u8) << 1),
                    0,
                    0,
                    0,
                    0,
                    0,
                ];
                self.dlen = 1;
                self.dpos = 0;
                self.loop_ = true;
                self.state = ChipState::ReadData;
            }
            // Winbond 0x35 = RDCR_EQIO: status register 2, bit 1 = QE
            // (QEMU m25p80 `RDCR_EQIO`; s->data[0] = quad_enable << 1).
            CMD_RDSR2 => {
                self.data = [(self.quad_enable as u8) << 1, 0, 0, 0, 0, 0];
                self.dlen = 1;
                self.dpos = 0;
                self.loop_ = true;
                self.state = ChipState::ReadData;
            }
            CMD_WREN => self.write_enable = true,
            CMD_WRDI => self.write_enable = false,
            // Winbond WRSR collects TWO bytes: status reg 1 then status reg 2
            // (QEMU m25p80 WRSR needed_bytes=2 for MAN_WINBOND; the QE bit
            // lives in data[1]).
            CMD_WRSR if self.write_enable => self.state = ChipState::Collect { need: 2, got: 0 },
            CMD_READ => self.state = ChipState::Collect { need: 3, got: 0 },
            // Winbond fast-read: 3 addr bytes + 8 dummy bytes — QEMU models
            // dummy cycles as BYTES (m25p80 decode_fast_read_cmd).
            CMD_FAST_READ => self.state = ChipState::Collect { need: 11, got: 0 },
            CMD_PP | CMD_SE | CMD_BE => self.state = ChipState::Collect { need: 3, got: 0 },
            CMD_CE | CMD_CE_ALT => {
                if self.write_enable {
                    flash.fill(0xFF);
                }
            }
            // Unknown commands (DP, RES-as-0xAB, HPM, ...): m25p80 replies
            // with repeated 0x00 bytes.
            _ => {
                self.data = [0; 6];
                self.dlen = 1;
                self.dpos = 0;
                self.loop_ = true;
                self.state = ChipState::ReadData;
            }
        }
    }

    /// Address/dummy collection complete (m25p80 complete_collecting_data).
    fn finish_collect(&mut self, flash: &mut [u8]) {
        let mask = (flash.len() - 1) as u32;
        self.addr &= mask;
        match self.cmd {
            CMD_WRSR => {
                self.status = self.data[0];
                self.quad_enable = self.data[1] & 0x02 != 0;
                self.write_enable = false;
                self.state = ChipState::Idle;
            }
            CMD_READ | CMD_FAST_READ => self.state = ChipState::Read,
            CMD_PP => self.state = ChipState::Program,
            CMD_SE => {
                if self.write_enable {
                    let a = self.addr as usize;
                    let end = (a + 0x1000).min(flash.len());
                    flash[a..end].fill(0xFF);
                }
                self.state = ChipState::Idle;
            }
            CMD_BE => {
                if self.write_enable {
                    let a = self.addr as usize;
                    let end = (a + 0x1_0000).min(flash.len());
                    flash[a..end].fill(0xFF);
                }
                self.state = ChipState::Idle;
            }
            _ => self.state = ChipState::Idle,
        }
    }

    /// CS-high (m25p80_cs select=1): reset the stream state but keep
    /// write-enable and the status register.
    fn cs_high(&mut self) {
        self.state = ChipState::Idle;
        self.dlen = 0;
        self.dpos = 0;
        self.loop_ = false;
    }
}

/// Where a transaction's MISO bytes land.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Rxn {
    /// W0..W15 data buffer.
    Data,
    /// RD_STATUS register (special RDSR only).
    RdStatus,
}

/// A prepared CS-low transaction: command + address + dummy + data phases.
struct Txn {
    cmd: u32,
    cmd_bytes: u32,
    addr: u32,
    addr_bytes: u32,
    dummy_bytes: u32,
    tx_bytes: u32,
    rx_bytes: u32,
    rx: Rxn,
}

/// Host-readable record of one triggered transaction (debugging aid).
#[derive(Clone, Copy)]
pub struct TxRec {
    pub cmd_reg: u32,
    pub addr: u32,
    pub user: u32,
    pub user1: u32,
    pub user2: u32,
    pub mosi_dlen: u32,
    pub miso_dlen: u32,
    pub w0: u32,
    /// MISC at transaction time (cs0_dis bit 0 / cs1_dis bit 1 select the
    /// flash vs PSRAM chip on the shared SPI1 bus).
    pub misc: u32,
}

/// ESP32-S3 SPI flash controller (SPI1 @ 0x60002000, SPIMEM0 @ 0x60003000).
pub struct Memspi {
    regs: [u32; REG_COUNT],
    chip: Chip,
    /// Set while the attached SPI flash is in continuous-read (XIP) mode.
    /// Mirrored from the cache enable flags by the SoC (see there): once the
    /// ROM bootloader enables XIP for cache reads, single-line commands
    /// like RDID no longer reach the flash — the (also-selected) PSRAM chip
    /// answers instead. Fresh controllers start non-XIP (pre-bootloader).
    pub xip: bool,
    /// PSRAM (Ext_RAM on CS1) backing for init/test-pattern traffic
    /// (extram_test writes magic via 0x02 and reads it back via 0x03).
    psram: Box<[u8; PSRAM_DEV_SIZE]>,
    /// Ring of the most recent transactions (host debug tracing).
    pub tx_trace: [TxRec; 16],
    pub tx_head: usize,
    pub tx_count: u64,
}

impl Memspi {
    pub fn new() -> Self {
        let mut regs = [0u32; REG_COUNT];
        // QEMU esp32s3_spi_reset_hold defaults.
        regs[(REG_CTRL1 >> 2) as usize] = 0x3FF << 2; // CS_HOLD_DLY_RES
        regs[(REG_CLOCK >> 2) as usize] = (3 << 16) | (1 << 8) | 3;
        regs[(REG_USER >> 2) as usize] = USER_USR_COMMAND;
        regs[(REG_USER1 >> 2) as usize] = (23 << 26) | 7;
        regs[(REG_USER2 >> 2) as usize] = 7 << 28;
        Self {
            regs,
            chip: Chip::default(),
            xip: false,
            psram: Box::new([0; PSRAM_DEV_SIZE]),
            tx_trace: [TxRec {
                cmd_reg: 0,
                addr: 0,
                user: 0,
                user1: 0,
                user2: 0,
                mosi_dlen: 0,
                miso_dlen: 0,
                w0: 0,
                misc: 0,
            }; 16],
            tx_head: 0,
            tx_count: 0,
        }
    }

    fn trace(&mut self, cmd_reg: u32) {
        self.tx_trace[self.tx_head] = TxRec {
            cmd_reg,
            addr: self.regs[(REG_ADDR >> 2) as usize],
            user: self.regs[(REG_USER >> 2) as usize],
            user1: self.regs[(REG_USER1 >> 2) as usize],
            user2: self.regs[(REG_USER2 >> 2) as usize],
            mosi_dlen: self.regs[(REG_MOSI_DLEN >> 2) as usize],
            miso_dlen: self.regs[(REG_MISO_DLEN >> 2) as usize],
            w0: self.regs[(REG_W0 >> 2) as usize],
            misc: self.regs[(REG_MISC >> 2) as usize],
        };
        self.tx_head = (self.tx_head + 1) & 15;
        self.tx_count += 1;
    }

    pub fn read32(&self, off: u32) -> u32 {
        match off {
            // CMD always reads 0 (cmd_is_done); FSM reads 0 (host idle).
            REG_CMD | REG_FSM => 0,
            REG_ADDR
            | REG_CTRL
            | REG_CTRL1
            | REG_CTRL2
            | REG_CLOCK
            | REG_USER
            | REG_USER1
            | REG_USER2
            | REG_MOSI_DLEN
            | REG_MISO_DLEN
            | REG_RD_STATUS
            | REG_MISC
            | REG_CACHE_FCTRL
            | REG_SUS_STATUS
            | REG_DDR_CTRL
            | REG_CLOCK_GATE
            | REG_W0..=REG_W15 => self.regs[(off >> 2) as usize],
            _ => 0,
        }
    }

    pub fn write32(&mut self, flash: &mut [u8], off: u32, value: u32) {
        match off {
            REG_CMD => {
                if value & CMD_USR != 0 {
                    self.begin_usr(flash);
                } else {
                    self.special_command(flash, value);
                }
                self.trace(value);
            }
            REG_ADDR
            | REG_CTRL
            | REG_CTRL1
            | REG_CTRL2
            | REG_CLOCK
            | REG_USER
            | REG_USER1
            | REG_USER2
            | REG_MOSI_DLEN
            | REG_MISO_DLEN
            | REG_RD_STATUS
            | REG_MISC
            | REG_CACHE_FCTRL
            | REG_SUS_STATUS
            | REG_DDR_CTRL
            | REG_CLOCK_GATE
            | REG_W0..=REG_W15 => self.regs[(off >> 2) as usize] = value,
            _ => {}
        }
    }

    /// USR transaction (QEMU esp32s3_spi_begin_transaction).
    fn begin_usr(&mut self, flash: &mut [u8]) {
        let user = self.regs[(REG_USER >> 2) as usize];
        let user1 = self.regs[(REG_USER1 >> 2) as usize];
        let user2 = self.regs[(REG_USER2 >> 2) as usize];

        let cmd = user2 & 0xFFFF;
        let cmd_bytes = ((user2 >> 28) + 1) / 8;

        let mut addr = 0u32;
        let mut addr_bytes = 0u32;
        let mut dummy_bytes = 0u32;
        if user & USER_USR_ADDR != 0 {
            addr = self.regs[(REG_ADDR >> 2) as usize].swap_bytes();
            addr_bytes = ((user1 >> 26) + 1) / 8;
            if (1..=4).contains(&addr_bytes) {
                addr >>= 32 - addr_bytes * 8;
            }
            // Dummy cycles only count when the address phase is enabled
            // (QEMU nests the dummy calculation inside the addr branch).
            if user & USER_USR_DUMMY != 0 {
                dummy_bytes = (user1 & 0x3F).div_ceil(8);
            }
        }

        let mut tx_bytes = 0u32;
        if user & USER_USR_MOSI != 0 {
            tx_bytes = ((self.regs[(REG_MOSI_DLEN >> 2) as usize] & 0x3FF) + 1) / 8;
        }
        let mut rx_bytes = 0u32;
        if user & USER_USR_MISO != 0 {
            rx_bytes = ((self.regs[(REG_MISO_DLEN >> 2) as usize] & 0x3FF) + 1) / 8;
        }

        // Chip routing (MISC.cs bits): CS1-only (flash deselected) talks to
        // the PSRAM device; anything else talks to the NOR flash.
        let misc = self.regs[(REG_MISC >> 2) as usize];
        if (misc & (MISC_CS0_DIS | MISC_CS1_DIS)) == MISC_CS0_DIS {
            self.run_psram_usr(cmd, addr, tx_bytes, rx_bytes);
            return;
        }
        // XIP-silenced flash: once the ROM bootloader enables continuous
        // reads for cache XIP, a single-line RDID no longer reaches the
        // flash — the (also-selected) PSRAM chip answers the ID probe
        // instead. Gated on the exact probe shape so normal flash traffic
        // (which suspends XIP around programmed I/O) is unaffected.
        if self.xip && cmd == CMD_JEDEC as u32 && addr_bytes == 0 && tx_bytes == 0 && rx_bytes > 0 {
            for (i, &b) in PSRAM_ID.iter().enumerate().take(rx_bytes as usize) {
                self.set_data_byte(i, b);
            }
            return;
        }

        self.run_transaction(
            flash,
            Txn {
                cmd,
                cmd_bytes,
                addr,
                addr_bytes,
                dummy_bytes,
                tx_bytes,
                rx_bytes,
                rx: Rxn::Data,
            },
        );
    }

    /// PSRAM (Ext_RAM, CS1) USR transaction: ID probe, array reads/writes,
    /// and mode-command absorbs. Addresses are plain (no NOR swap quirk);
    /// writes overwrite (RAM, not flash-AND). MISO for unrecognized reads
    /// is zeros (undriven bus).
    fn run_psram_usr(&mut self, cmd: u32, addr: u32, tx_bytes: u32, rx_bytes: u32) {
        let ntx = tx_bytes as usize;
        let nrx = rx_bytes as usize;
        match cmd as u8 {
            CMD_JEDEC => {
                for (i, &b) in PSRAM_ID.iter().enumerate().take(nrx) {
                    self.set_data_byte(i, b);
                }
            }
            CMD_READ | CMD_FAST_READ | 0xEB => {
                for i in 0..nrx {
                    let a = (addr as usize).wrapping_add(i);
                    self.set_data_byte(i, *self.psram.get(a).unwrap_or(&0));
                }
            }
            CMD_PP | 0x38 => {
                for i in 0..ntx {
                    let b = self.data_byte(i);
                    let a = (addr as usize).wrapping_add(i);
                    if let Some(slot) = self.psram.get_mut(a) {
                        *slot = b;
                    }
                }
                for i in 0..nrx {
                    let a = (addr as usize).wrapping_add(i);
                    self.set_data_byte(i, *self.psram.get(a).unwrap_or(&0));
                }
            }
            _ => {
                for i in 0..nrx {
                    self.set_data_byte(i, 0);
                }
            }
        }
    }

    /// Special-command dispatch (QEMU esp32s3_spi_special_command), matched
    /// on CMD bits [31:19].
    fn special_command(&mut self, flash: &mut [u8], cmd_reg: u32) {
        let addr_word = self.regs[(REG_ADDR >> 2) as usize];
        let addr_bitlen = ((self.regs[(REG_USER1 >> 2) as usize] >> 26) + 1) / 8;
        let addr = addr_word.swap_bytes();
        // 3-byte address, right-aligned (bswap32 >> 8 — for addr_bitlen=23
        // this drops the legacy length byte in ADDR[31:24]).
        let addr3 = addr >> 8;
        let norm = |a: u32| {
            if (1..=4).contains(&addr_bitlen) {
                a >> (32 - addr_bitlen * 8)
            } else {
                a
            }
        };
        match cmd_reg & CMD_SPECIAL_MASK {
            CMD_FLASH_READ => {
                let rx = ((self.regs[(REG_MISO_DLEN >> 2) as usize] & 0x3FF) + 1) / 8;
                self.run_transaction(
                    flash,
                    Txn {
                        cmd: CMD_READ as u32,
                        cmd_bytes: 1,
                        addr: norm(addr),
                        addr_bytes: addr_bitlen,
                        dummy_bytes: 0,
                        tx_bytes: 0,
                        rx_bytes: rx,
                        rx: Rxn::Data,
                    },
                );
            }
            CMD_FLASH_WREN => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_WREN as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 0,
                    rx: Rxn::Data,
                },
            ),
            CMD_FLASH_WRDI => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_WRDI as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 0,
                    rx: Rxn::Data,
                },
            ),
            CMD_FLASH_RDID => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_JEDEC as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 3,
                    rx: Rxn::Data,
                },
            ),
            CMD_FLASH_RDSR => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_RDSR as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 1,
                    rx: Rxn::RdStatus,
                },
            ),
            CMD_FLASH_WRSR => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_WRSR as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 1,
                    rx_bytes: 0,
                    rx: Rxn::RdStatus,
                },
            ),
            // Legacy page-program: byte count lives in ADDR[31:24], address
            // bytes are bswap32(ADDR) >> 8 (QEMU's fixed-shift adjustment).
            CMD_FLASH_PP => {
                let len = (addr_word >> 24) & 0xFF;
                self.run_transaction(
                    flash,
                    Txn {
                        cmd: CMD_PP as u32,
                        cmd_bytes: 1,
                        addr: addr3,
                        addr_bytes: 3,
                        dummy_bytes: 0,
                        tx_bytes: len,
                        rx_bytes: 0,
                        rx: Rxn::Data,
                    },
                );
            }
            CMD_FLASH_SE => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_SE as u32,
                    cmd_bytes: 1,
                    addr: norm(addr),
                    addr_bytes: addr_bitlen,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 0,
                    rx: Rxn::Data,
                },
            ),
            CMD_FLASH_BE => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_BE as u32,
                    cmd_bytes: 1,
                    addr: norm(addr),
                    addr_bytes: addr_bitlen,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 0,
                    rx: Rxn::Data,
                },
            ),
            CMD_FLASH_CE => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_CE as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 0,
                    rx: Rxn::Data,
                },
            ),
            CMD_FLASH_DP => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_DP as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 0,
                    rx: Rxn::Data,
                },
            ),
            CMD_FLASH_RES => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_RES as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 3,
                    rx: Rxn::Data,
                },
            ),
            CMD_FLASH_HPM => self.run_transaction(
                flash,
                Txn {
                    cmd: CMD_HPM as u32,
                    cmd_bytes: 1,
                    addr: 0,
                    addr_bytes: 0,
                    dummy_bytes: 0,
                    tx_bytes: 0,
                    rx_bytes: 3,
                    rx: Rxn::Data,
                },
            ),
            _ => {}
        }
    }

    /// Execute a full CS-low transaction: command, address, dummy, then
    /// data (MOSI from the W buffer, MISO into `rx`), then CS-high.
    fn run_transaction(&mut self, flash: &mut [u8], t: Txn) {
        let Txn {
            cmd,
            cmd_bytes,
            addr,
            addr_bytes,
            dummy_bytes,
            tx_bytes,
            rx_bytes,
            rx,
        } = t;
        let data_start = cmd_bytes + addr_bytes + dummy_bytes;
        let total = data_start + tx_bytes.max(rx_bytes);
        for i in 0..total {
            let mut txb = 0u8;
            let mut di = usize::MAX;
            if i < cmd_bytes {
                txb = ((cmd >> (8 * i)) & 0xFF) as u8;
            } else if i < cmd_bytes + addr_bytes {
                txb = ((addr >> (8 * (i - cmd_bytes))) & 0xFF) as u8;
            } else if i < data_start {
                txb = 0;
            } else {
                di = (i - data_start) as usize;
                if (di as u32) < tx_bytes {
                    txb = self.data_byte(di);
                }
            }
            let r = self.chip.transfer(flash, txb);
            if di != usize::MAX && (di as u32) < rx_bytes {
                match rx {
                    Rxn::Data => self.set_data_byte(di, r),
                    Rxn::RdStatus => self.regs[(REG_RD_STATUS >> 2) as usize] = r as u32,
                }
            }
        }
        self.chip.cs_high();
    }

    /// Byte `i` of the W0..W15 data buffer (little-endian word order).
    fn data_byte(&self, i: usize) -> u8 {
        let w = self.regs[((REG_W0 + 4 * (i as u32 / 4)) >> 2) as usize];
        (w >> (8 * (i % 4))) as u8
    }

    fn set_data_byte(&mut self, i: usize, v: u8) {
        let r = ((REG_W0 + 4 * (i as u32 / 4)) >> 2) as usize;
        let sh = 8 * (i % 4);
        self.regs[r] = (self.regs[r] & !(0xFF << sh)) | ((v as u32) << sh);
    }
}

impl Default for Memspi {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::boxed::Box;
    use alloc::vec;

    fn flash4m() -> Box<[u8]> {
        vec![0u8; 0x400000].into_boxed_slice()
    }

    /// The firmware's read_id USR transaction: command 0x9F from USER2,
    /// no address/dummy/MOSI, 24 MISO bits -> W0 = 0x1640EF (id bytes
    /// MSB-first), which the driver byte-swaps to 0xEF4016.
    #[test]
    fn read_id_usr_transaction() {
        let mut f = flash4m();
        let mut m = Memspi::new();
        m.write32(&mut f, REG_USER, USER_USR_COMMAND | USER_USR_MISO);
        m.write32(&mut f, REG_USER2, (CMD_JEDEC as u32) | (7 << 28));
        m.write32(&mut f, REG_MISO_DLEN, 23);
        m.write32(&mut f, REG_CMD, CMD_USR);
        assert_eq!(m.read32(REG_W0), 0x1640EF);
        // CMD always reads 0 (the driver's poll_cmd_done loop exits).
        assert_eq!(m.read32(REG_CMD), 0);
    }

    /// CS1-selected ID probe answers the PSRAM (not flash) ID, with the
    /// KGD density byte where the esp-idf quad probe checks it.
    #[test]
    fn psram_id_on_cs1() {
        let mut f = flash4m();
        let mut m = Memspi::new();
        m.write32(&mut f, REG_MISC, 0x1); // cs0_dis=1 (flash off), cs1_dis=0 (PSRAM on)
        m.write32(&mut f, REG_USER, USER_USR_COMMAND | USER_USR_MISO);
        m.write32(&mut f, REG_USER2, (CMD_JEDEC as u32) | (7 << 28));
        m.write32(&mut f, REG_MISO_DLEN, 23);
        m.write32(&mut f, REG_CMD, CMD_USR);
        assert_eq!((m.read32(REG_W0) >> 8) & 0xFF, 0x5D);
    }

    /// With XIP active, an ambiguous (both-selected) ID probe is answered
    /// by the PSRAM (silenced flash); pre-XIP it returns the flash ID.
    #[test]
    fn psram_id_when_xip_silences_flash() {
        let mut f = flash4m();
        let mut m = Memspi::new();
        m.write32(&mut f, REG_USER, USER_USR_COMMAND | USER_USR_MISO);
        m.write32(&mut f, REG_USER2, (CMD_JEDEC as u32) | (7 << 28));
        m.write32(&mut f, REG_MISO_DLEN, 23);
        m.write32(&mut f, REG_CMD, CMD_USR);
        assert_eq!(m.read32(REG_W0), 0x1640EF, "pre-XIP: flash ID");
        m.xip = true;
        m.write32(&mut f, REG_CMD, CMD_USR);
        assert_eq!((m.read32(REG_W0) >> 8) & 0xFF, 0x5D, "XIP: PSRAM ID");
    }

    /// CS1 pattern traffic round-trips through the PSRAM backing (the
    /// extram_test write-magic/read-back shape).
    #[test]
    fn psram_pattern_round_trip() {
        let mut f = flash4m();
        let mut m = Memspi::new();
        m.write32(&mut f, REG_MISC, 0x1); // cs0_dis=1 (flash off), cs1_dis=0 (PSRAM on)
        m.write32(
            &mut f,
            REG_USER,
            USER_USR_COMMAND | USER_USR_ADDR | USER_USR_MOSI,
        );
        m.write32(&mut f, REG_USER2, (CMD_PP as u32) | (7 << 28));
        m.write32(&mut f, REG_ADDR, 0x100);
        m.write32(&mut f, REG_USER1, 23 << 26); // 24-bit address
        m.write32(&mut f, REG_MOSI_DLEN, 31);
        m.write32(&mut f, REG_W0, 0x5A6B7C8D);
        m.write32(&mut f, REG_CMD, CMD_USR);
        m.write32(
            &mut f,
            REG_USER,
            USER_USR_COMMAND | USER_USR_ADDR | USER_USR_MISO,
        );
        m.write32(&mut f, REG_USER2, (CMD_READ as u32) | (7 << 28));
        m.write32(&mut f, REG_MISO_DLEN, 31);
        m.write32(&mut f, REG_CMD, CMD_USR);
        assert_eq!(m.read32(REG_W0), 0x5A6B7C8D);
        // Flash backing untouched by the CS1 write.
        assert_eq!(f[0x100], 0);
    }

    /// Special-command RDID (CMD bit 28) reads 3 id bytes into W0.
    #[test]
    fn special_rdid() {
        let mut f = flash4m();
        let mut m = Memspi::new();
        m.write32(&mut f, REG_CMD, CMD_FLASH_RDID);
        assert_eq!(m.read32(REG_W0), 0x1640EF);
    }

    /// WREN sets WEL; RDSR reports it in bit 1 (special-command paths).
    #[test]
    fn wren_then_rdsr() {
        let mut f = flash4m();
        let mut m = Memspi::new();
        m.write32(&mut f, REG_CMD, CMD_FLASH_WREN);
        m.write32(&mut f, REG_CMD, CMD_FLASH_RDSR);
        assert_eq!(m.read32(REG_RD_STATUS), 0x02);
        m.write32(&mut f, REG_CMD, CMD_FLASH_WRDI);
        m.write32(&mut f, REG_CMD, CMD_FLASH_RDSR);
        assert_eq!(m.read32(REG_RD_STATUS), 0x00);
    }

    /// USR READ (0x03): 3-byte address from ADDR/USER1, data streamed into
    /// the W buffer.
    #[test]
    fn usr_read_streams_flash() {
        let mut f = flash4m();
        f[0x1000] = 0x5A;
        f[0x1001] = 0xA5;
        f[0x1002] = 0xFF;
        f[0x1003] = 0x00;
        let mut m = Memspi::new();
        m.write32(
            &mut f,
            REG_USER,
            USER_USR_COMMAND | USER_USR_ADDR | USER_USR_MISO,
        );
        m.write32(&mut f, REG_USER2, (CMD_READ as u32) | (7 << 28));
        m.write32(&mut f, REG_ADDR, 0x1000);
        m.write32(&mut f, REG_MISO_DLEN, 31);
        m.write32(&mut f, REG_CMD, CMD_USR);
        assert_eq!(m.read32(REG_W0), 0x00FF_A55A);
    }

    /// USR page-program (0x02) ANDs MOSI bytes into flash at the address.
    #[test]
    fn usr_program_writes_flash() {
        let mut f = flash4m();
        f[0x2000] = 0xFF;
        f[0x2001] = 0xFF;
        let mut m = Memspi::new();
        m.write32(&mut f, REG_CMD, CMD_FLASH_WREN);
        m.write32(
            &mut f,
            REG_USER,
            USER_USR_COMMAND | USER_USR_ADDR | USER_USR_MOSI,
        );
        m.write32(&mut f, REG_USER2, (CMD_PP as u32) | (7 << 28));
        m.write32(&mut f, REG_ADDR, 0x2000);
        m.write32(&mut f, REG_MOSI_DLEN, 15);
        m.write32(&mut f, REG_W0, 0xAA55);
        m.write32(&mut f, REG_CMD, CMD_USR);
        // W buffer bytes stream out little-endian (QEMU memcpy), so the
        // first programmed byte is the low byte of W0.
        assert_eq!(f[0x2000], 0x55);
        assert_eq!(f[0x2001], 0xAA);
    }

    /// Winbond 2-byte WRSR: byte 0 = status reg 1, byte 1 = status reg 2.
    /// The QE (quad enable) bit is status-2 bit 1; RDSR2 (0x35) reports it
    /// as `QE << 1` (QEMU m25p80 WRSR/RDCR_EQIO).
    #[test]
    fn wrsr_2byte_sets_quad_enable() {
        let mut f = flash4m();
        let mut m = Memspi::new();
        m.write32(&mut f, REG_CMD, CMD_FLASH_WREN);
        // W0 = 0x0200: LE bytes [0x00, 0x02] = status-1, status-2.
        m.write32(&mut f, REG_USER, USER_USR_COMMAND | USER_USR_MOSI);
        m.write32(&mut f, REG_USER2, (CMD_WRSR as u32) | (7 << 28));
        m.write32(&mut f, REG_MOSI_DLEN, 15);
        m.write32(&mut f, REG_W0, 0x0200);
        m.write32(&mut f, REG_CMD, CMD_USR);
        assert_eq!(m.read32(REG_RD_STATUS), 0); // status-1 unchanged
        // RDSR2 via USR: MISO byte 0x02 = QE << 1.
        m.write32(&mut f, REG_USER, USER_USR_COMMAND | USER_USR_MISO);
        m.write32(&mut f, REG_USER2, (CMD_RDSR2 as u32) | (7 << 28));
        m.write32(&mut f, REG_MISO_DLEN, 7);
        m.write32(&mut f, REG_CMD, CMD_USR);
        assert_eq!(m.read32(REG_W0) & 0xFF, 0x02);
        // RDSR still reports status-1 = 0.
        m.write32(&mut f, REG_CMD, CMD_FLASH_RDSR);
        assert_eq!(m.read32(REG_RD_STATUS), 0);
        // WEL cleared by the WRSR.
        m.write32(&mut f, REG_CMD, CMD_FLASH_WREN);
        m.write32(&mut f, REG_CMD, CMD_FLASH_RDSR);
        assert_eq!(m.read32(REG_RD_STATUS), 0x02);
    }

    /// Erase without WEL is a no-op (m25p80 write-protect semantics).
    #[test]
    fn erase_requires_wren() {
        let mut f = flash4m();
        f[0x3000] = 0x12;
        let mut m = Memspi::new();
        m.write32(&mut f, REG_ADDR, 0x3000);
        m.write32(&mut f, REG_CMD, CMD_FLASH_SE);
        assert_eq!(f[0x3000], 0x12);
        m.write32(&mut f, REG_CMD, CMD_FLASH_WREN);
        m.write32(&mut f, REG_CMD, CMD_FLASH_SE);
        // Sector 0x3000..0x4000 erased; neighbours untouched.
        assert_eq!(f[0x2FFF], 0x00);
        assert_eq!(f[0x3000], 0xFF);
        assert_eq!(f[0x3FFF], 0xFF);
        assert_eq!(f[0x4000], 0x00);
    }

    /// Block erase (0xD8) clears 64 KB.
    #[test]
    fn special_be_erases_64k() {
        let mut f = flash4m();
        let mut m = Memspi::new();
        m.write32(&mut f, REG_CMD, CMD_FLASH_WREN);
        m.write32(&mut f, REG_ADDR, 0x1_0000);
        m.write32(&mut f, REG_CMD, CMD_FLASH_BE);
        assert_eq!(f[0x1_0000], 0xFF);
        assert_eq!(f[0x1_FFFF], 0xFF);
        assert_eq!(f[0x2_0000], 0x00);
        assert_eq!(f[0x0_FFFF], 0x00);
    }

    /// Chip erase via special CE clears everything.
    #[test]
    fn special_ce_erases_all() {
        let mut f = flash4m();
        f[0x1234] = 0xAB;
        let mut m = Memspi::new();
        m.write32(&mut f, REG_CMD, CMD_FLASH_CE);
        assert_eq!(f[0x1234], 0xAB); // needs WEL
        m.write32(&mut f, REG_CMD, CMD_FLASH_WREN);
        m.write32(&mut f, REG_CMD, CMD_FLASH_CE);
        assert_eq!(f[0x1234], 0xFF);
        assert_eq!(f[0x3FFFFF], 0xFF);
    }

    /// QEMU reset defaults are readable back.
    #[test]
    fn reset_defaults() {
        let mut f = flash4m();
        let m = Memspi::new();
        assert_eq!(m.read32(REG_CTRL1), 0x3FF << 2);
        assert_eq!(m.read32(REG_CLOCK), 0x0003_0103);
        assert_eq!(m.read32(REG_USER), USER_USR_COMMAND);
        assert_eq!(m.read32(REG_USER1), (23 << 26) | 7);
        assert_eq!(m.read32(REG_USER2), 7 << 28);
        assert_eq!(m.read32(REG_FSM), 0);
        // Unmodeled offsets read 0.
        assert_eq!(m.read32(0x1E0), 0);
    }
}
