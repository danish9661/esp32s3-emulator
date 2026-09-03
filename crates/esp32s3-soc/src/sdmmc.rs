//! ESP32-S3 SD/MMC host controller (DesignWare MMC, `DR_REG_SDMMC_BASE`).
//!
//! Base `DR_REG_SDMMC_BASE = 0x6002_8000`. The controller is a Synopsys
//! DesignWare MMC host. We model both the register interface AND a simulated
//! SD card behind it: writing `CMD` (0x2C) with `start_command` (bit 31) set
//! issues a command to the (modeled) card, which fills `RESP0..3` (0x30..0x3C)
//! and latches the command-done / data-over interrupts (`RINTSTS` 0x44, bits
//! 2 and 3).
//!
//! Data transfers use either the PIO FIFO (0x100) or the **IDMAC** (internal
//! DMA). The IDMAC descriptor walk itself runs in `soc.rs` (it needs the bus to
//! reach DRAM descriptor buffers); this module exposes the card model and the
//! descriptor metadata. Block data transfers are served from a multi-block
//! `storage` array indexed by the command's LBA (SDHC byte-address / 512).
//!
//! Register layout / `CMD` bitfield from `sdmmc_struct.h`:
//! - `cmd_index`   bits [5:0]
//! - `response_expect`   bit 6
//! - `response_long`     bit 7   (R2 = 136-bit CID/CSD)
//! - `check_response_crc` bit 8
//! - `data_expected`     bit 9
//! - `rw`         bit 10  (0 = read from card, 1 = write to card)
//! - `send_auto_stop`    bit 12
//! - `stop_abort_cmd`    bit 14
//! - `send_init`  bit 15  (80 init clocks — CMD0)
//! - `update_clk_reg`    bit 21  (clock-only update, no command)
//! - `start_command`     bit 31
//!
//! IDMAC registers (`sdmmc_struct.h`):
//! - `idmac_ctrl`   0x80  (bit 0 = enable, bit 1 = reset)
//! - `idmac_bsize`  0x84  (descriptor ring length, in descriptors)
//! - `idmac_dbaddr` 0x88  (descriptor list base, DRAM pointer)
//! - `idmac_rintsts` 0x8C (DMA transfer-complete / error status, w1c)
//! - `idmac_status` 0x90
//!
//! Validated by the `esp32s3_sdmmc` poke sketch (direct register pokes of the
//! full init sequence + PIO block read/write) and the IDMAC machine test
//! (`sdmmc_idmac_walks_descriptors`).

const BLOCK_LEN: usize = 512;
const STORAGE_BLOCKS: usize = 1024; // 512 KB modeled card

use alloc::collections::VecDeque;
use alloc::vec;
use alloc::vec::Vec;

pub const SDMMC_BASE: u32 = 0x6002_8000;

/// SDIO-host interrupt source for the matrix (ETS_SDIO_HOST_INTR_SOURCE).
pub const SDMMC_INTR_SOURCE: u32 = 30;

// Register offsets (DesignWare MMC).
pub const CTRL: u32 = 0x00;
pub const PWREN: u32 = 0x04;
pub const CLKDIV: u32 = 0x08;
pub const CLKENA: u32 = 0x10;
pub const CTYPE: u32 = 0x18;
pub const BLKSIZ: u32 = 0x1C;
pub const BYTCNT: u32 = 0x20;
pub const CMDARG: u32 = 0x28;
pub const CMD: u32 = 0x2C;
pub const RESP0: u32 = 0x30;
pub const RESP1: u32 = 0x34;
pub const RESP2: u32 = 0x38;
pub const RESP3: u32 = 0x3C;
pub const MINTSTS: u32 = 0x40;
pub const RINTSTS: u32 = 0x44;
pub const STATUS: u32 = 0x48;
pub const CDETECT: u32 = 0x50;
pub const WRTPRT: u32 = 0x54;
pub const FIFO: u32 = 0x100;
pub const IDMAC_CTRL: u32 = 0x80;
pub const IDMAC_BSIZE: u32 = 0x84;
pub const IDMAC_DBADDR: u32 = 0x88;
pub const IDMAC_RINTSTS: u32 = 0x8C;
pub const IDMAC_STATUS: u32 = 0x90;

// RINTSTS / MINTSTS interrupt bits.
const INT_CMD_DONE: u32 = 1 << 2;
const INT_DATA_OVER: u32 = 1 << 3;

// IDMAC_RINTSTS bits.
const IDMAC_TI: u32 = 1 << 0; // transfer complete

// CMD bit positions.
const CMD_INDEX: u32 = 0x3F; // bits [5:0]
const CMD_RESPONSE_EXPECT: u32 = 1 << 6;
const CMD_RESPONSE_LONG: u32 = 1 << 7;
const CMD_DATA_EXPECTED: u32 = 1 << 9;
const CMD_RW: u32 = 1 << 10; // 0 = read from card, 1 = write to card
const CMD_UPDATE_CLK: u32 = 1 << 21;
const CMD_START: u32 = 1 << 31;

#[derive(Clone, Copy, PartialEq, Debug)]
enum CardState {
    Idle,
    Ready,
    Ident,
    Stby,
    Tran,
}

/// Pending IDMAC transfer, consumed by `soc.rs` which owns the bus.
pub struct IdmacXfer {
    pub write: bool, // true = host->card, false = card->host
    pub bytcnt: u32,
    pub dbaddr: u32,
    pub lba: u32,
}

/// Simulated SD card + host controller.
pub struct Sdmmc {
    regs: [u32; 0x400 / 4],
    card_state: CardState,
    app_cmd: bool,
    acmd41_count: u32,
    rca: u32,
    /// Modeled card storage: STORAGE_BLOCKS x 512-byte blocks (round-trips
    /// writes), indexed by LBA. Initialized with a recognizable byte pattern.
    storage: Vec<u8>,
    /// Current transfer LBA (set by CMD17/18/24/25).
    lba: u32,
    /// Pending data transfer FIFO (bytes) for the PIO path.
    data: VecDeque<u8>,
    data_remaining: usize,
    data_dir: u8, // 0 = read (card->host), 1 = write (host->card)
    data_active: bool,
    /// Set when the next data command serves the SCR register instead of storage.
    serve_scr: bool,
    /// Pending IDMAC transfer (None for PIO).
    idmac: Option<IdmacXfer>,
}

impl Default for Sdmmc {
    fn default() -> Self {
        Self::new()
    }
}

impl Sdmmc {
    pub fn new() -> Self {
        let mut storage = vec![0u8; STORAGE_BLOCKS * BLOCK_LEN];
        for (i, b) in storage.iter_mut().enumerate() {
            *b = i as u8; // recognizable pattern
        }
        Self {
            regs: [0u32; 0x400 / 4],
            card_state: CardState::Idle,
            app_cmd: false,
            acmd41_count: 0,
            rca: 0x1234,
            storage,
            lba: 0,
            data: VecDeque::new(),
            data_remaining: 0,
            data_dir: 0,
            data_active: false,
            serve_scr: false,
            idmac: None,
        }
    }

    fn idx(&self, offset: u32) -> usize {
        ((offset & 0xFFF) / 4) as usize
    }

    /// Issue a command to the (modeled) SD card.
    fn do_command(&mut self, cmd: u32, arg: u32) {
        let index = cmd & CMD_INDEX;
        let resp_expect = (cmd & CMD_RESPONSE_EXPECT) != 0;
        let resp_long = (cmd & CMD_RESPONSE_LONG) != 0;
        let data_exp = (cmd & CMD_DATA_EXPECTED) != 0;
        let rw = (cmd & CMD_RW) != 0;

        // Clear the start bit in the stored CMD register (HW clears it).
        self.regs[self.idx(CMD)] &= !CMD_START;

        // ACMD detection: CMD55 sets the app_cmd flag for the next command.
        if index == 55 {
            self.app_cmd = true;
            self.set_resp_short(0);
            self.regs[self.idx(RINTSTS)] |= INT_CMD_DONE;
            return;
        }
        // The next command is an ACMD only if CMD55 preceded it.
        let is_acmd = self.app_cmd;
        self.app_cmd = false;

        // Record the transfer LBA for data commands (SDHC: arg = block addr).
        match index {
            17 | 18 | 24 | 25 => self.lba = arg,
            _ => {}
        }
        self.serve_scr = is_acmd && index == 51; // ACMD51 = SEND_SCR (data)

        let mut resp = [0u32; 4];
        match index {
            0 => {
                // GO_IDLE_STATE: card -> idle.
                self.card_state = CardState::Idle;
                resp[0] = 0;
            }
            6 => {
                // SWITCH_FUNC (R1): benign.
                resp[0] = 0;
            }
            8 => {
                // SEND_IF_COND: echo the argument (R7).
                resp[0] = arg & 0x0000_FFFF;
            }
            41 => {
                // SD_SEND_OP_COND (ACMD41).
                self.acmd41_count += 1;
                if self.acmd41_count == 1 {
                    // First call: not yet ready.
                    self.card_state = CardState::Ready;
                    resp[0] = arg & 0x00FF_FFFF;
                } else {
                    // Subsequent: ready + high-capacity (CCS) card.
                    resp[0] = (1 << 31) | (1 << 30) | (arg & 0x00FF_FFFF);
                    self.card_state = CardState::Ready;
                }
            }
            2 => {
                // ALL_SEND_CID (R2): return a fixed CID.
                resp[0] = 0x1234_5678;
                resp[1] = 0x9ABC_DEF0;
                resp[2] = 0x1357_2468;
                resp[3] = 0x0000_00AA;
                self.card_state = CardState::Ident;
            }
            3 => {
                // SEND_RCA (R6): assign/return our RCA.
                resp[0] = self.rca << 16;
                self.card_state = CardState::Stby;
            }
            7 => {
                // SELECT/DESELECT: arg = RCA<<16.
                let arg_rca = (arg >> 16) & 0xFFFF;
                if arg_rca == self.rca {
                    self.card_state = CardState::Tran;
                    resp[0] = 0;
                } else if arg_rca == 0 {
                    self.card_state = CardState::Stby;
                    resp[0] = 0;
                } else {
                    resp[0] = 1 << 3; // status error
                }
            }
            9 => {
                // SEND_CSD (R2): fixed CSD (SDHC, size ~ 256 MB).
                resp[0] = 0x4000_00B5;
                resp[1] = 0x5B59_0000;
                resp[2] = 0x9ABC_DEF0;
                resp[3] = 0x1357_2468;
            }
            10 => {
                // SEND_CID (R2): fixed CID.
                resp[0] = 0x1234_5678;
                resp[1] = 0x9ABC_DEF0;
                resp[2] = 0x1357_2468;
                resp[3] = 0x0000_00AA;
            }
            13 => {
                // SEND_STATUS (R1).
                resp[0] = 0;
            }
            16 => {
                // SET_BLOCKLEN: arg = block length.
                resp[0] = 0;
            }
            51 => {
                // SEND_SCR (ACMD51, R1 + 8-byte data): benign R1.
                resp[0] = 0;
            }
            _ => {
                // Unknown command: return a benign R1=0.
                resp[0] = 0;
            }
        }
        if resp_expect {
            if resp_long {
                self.regs[self.idx(RESP0)] = resp[0];
                self.regs[self.idx(RESP1)] = resp[1];
                self.regs[self.idx(RESP2)] = resp[2];
                self.regs[self.idx(RESP3)] = resp[3];
            } else {
                self.regs[self.idx(RESP0)] = resp[0];
                self.regs[self.idx(RESP1)] = 0;
                self.regs[self.idx(RESP2)] = 0;
                self.regs[self.idx(RESP3)] = 0;
            }
        }
        // Command done.
        self.regs[self.idx(RINTSTS)] |= INT_CMD_DONE;
        // If data was expected, set up the transfer. IDMAC if enabled, else PIO.
        if data_exp {
            let idmac_en = (self.regs[self.idx(IDMAC_CTRL)] & 1) != 0;
            if idmac_en {
                let bytcnt = self.regs[self.idx(BYTCNT)];
                let bytcnt = if bytcnt == 0 {
                    BLOCK_LEN as u32
                } else {
                    bytcnt
                };
                self.idmac = Some(IdmacXfer {
                    write: rw,
                    bytcnt,
                    dbaddr: self.regs[self.idx(IDMAC_DBADDR)],
                    lba: self.lba,
                });
            } else {
                self.begin_data(rw);
            }
        }
    }

    fn set_resp_short(&mut self, v: u32) {
        self.regs[self.idx(RESP0)] = v;
        self.regs[self.idx(RESP1)] = 0;
        self.regs[self.idx(RESP2)] = 0;
        self.regs[self.idx(RESP3)] = 0;
    }

    /// Begin a PIO data transfer of `BYTCNT` bytes.
    fn begin_data(&mut self, write: bool) {
        let bytcnt = self.regs[self.idx(BYTCNT)] as usize;
        let bytcnt = if bytcnt == 0 { BLOCK_LEN } else { bytcnt };
        self.data_remaining = bytcnt;
        self.data_dir = if write { 1 } else { 0 };
        self.data_active = true;
        self.data.clear();
        if !write {
            // Load the data FIFO from storage (or SCR) byte stream, LE words.
            let src = self.data_source(bytcnt);
            let n = bytcnt.min(src.len());
            self.data.extend(src[..n].iter().cloned());
            if bytcnt > self.data.len() {
                self.data.resize(bytcnt, 0);
            }
        }
    }

    /// Source bytes for a read transfer (SCR for ACMD51, else storage).
    fn data_source(&self, bytcnt: usize) -> Vec<u8> {
        if self.serve_scr {
            // SCR: SD_SPEC=2, bus widths 1-bit+4-bit supported.
            vec![0x02, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00]
                .into_iter()
                .cycle()
                .take(bytcnt)
                .collect()
        } else {
            let base = (self.lba as usize) * BLOCK_LEN;
            let end = (base + bytcnt).min(self.storage.len());
            self.storage[base..end].to_vec()
        }
    }

    /// Finish a PIO write transfer: copy received bytes into storage.
    fn finish_write(&mut self) {
        let base = (self.lba as usize) * BLOCK_LEN;
        let n = self.data.len();
        if base + n > self.storage.len() {
            self.storage.resize(base + n, 0);
        }
        for (i, b) in self.data.iter().take(n).enumerate() {
            self.storage[base + i] = *b;
        }
        self.data_active = false;
        self.data_remaining = 0;
        self.regs[self.idx(RINTSTS)] |= INT_DATA_OVER;
    }

    /// Take a pending IDMAC transfer (consumed by `soc.rs`).
    pub fn take_idmac(&mut self) -> Option<IdmacXfer> {
        self.idmac.take()
    }

    /// Copy `data` (host->card) into storage at the transfer LBA.
    pub fn idmac_store(&mut self, lba: u32, data: &[u8]) {
        let base = (lba as usize) * BLOCK_LEN;
        let end = base + data.len();
        if end > self.storage.len() {
            self.storage.resize(end, 0);
        }
        self.storage[base..end].copy_from_slice(data);
        self.regs[self.idx(IDMAC_RINTSTS)] |= IDMAC_TI;
        self.regs[self.idx(RINTSTS)] |= INT_DATA_OVER;
    }

    /// Return `len` bytes from storage at the transfer LBA (card->host).
    pub fn idmac_load(&self, lba: u32, len: usize) -> Vec<u8> {
        let base = (lba as usize) * BLOCK_LEN;
        let end = (base + len).min(self.storage.len());
        if base >= self.storage.len() {
            return vec![0u8; len];
        }
        self.storage[base..end].to_vec()
    }

    /// Mark an IDMAC transfer complete (called by `soc.rs` after the walk).
    pub fn finish_idmac(&mut self) {
        self.regs[self.idx(IDMAC_RINTSTS)] |= IDMAC_TI;
        self.regs[self.idx(RINTSTS)] |= INT_DATA_OVER;
    }

    /// Interrupt status for the matrix: the masked view (MINTSTS reads the
    /// same RINTSTS word; no separate enable mask is modeled, and status
    /// is write-1-to-clear like silicon).
    pub fn int_st(&self) -> u32 {
        self.regs[self.idx(RINTSTS)]
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            CDETECT => 0, // bit0=0 means card present
            WRTPRT => 0,  // bit0=0 means not write-protected
            RINTSTS => self.regs[self.idx(RINTSTS)],
            MINTSTS => self.regs[self.idx(RINTSTS)],
            IDMAC_RINTSTS => self.regs[self.idx(IDMAC_RINTSTS)],
            FIFO => {
                if self.data_active && self.data_dir == 0 {
                    // Read 4 bytes (LE) from the data FIFO.
                    let mut w = 0u32;
                    for shift in 0..4 {
                        if let Some(b) = self.data.pop_front() {
                            w |= (b as u32) << (shift * 8);
                        }
                    }
                    if self.data.is_empty() {
                        self.data_active = false;
                        self.data_remaining = 0;
                        self.regs[self.idx(RINTSTS)] |= INT_DATA_OVER;
                    }
                    w
                } else {
                    0
                }
            }
            _ => {
                let i = self.idx(offset);
                if i < self.regs.len() { self.regs[i] } else { 0 }
            }
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            RINTSTS | MINTSTS => {
                // Write-1-to-clear.
                self.regs[self.idx(RINTSTS)] &= !value;
            }
            IDMAC_RINTSTS => {
                self.regs[self.idx(IDMAC_RINTSTS)] &= !value;
            }
            CMD => {
                if value & CMD_START != 0 {
                    if value & CMD_UPDATE_CLK != 0 {
                        // Clock-only update: no command issued.
                        self.regs[self.idx(CMD)] = value & !CMD_START;
                        return;
                    }
                    let arg = self.regs[self.idx(CMDARG)];
                    self.do_command(value, arg);
                } else {
                    self.regs[self.idx(CMD)] = value;
                }
            }
            FIFO => {
                if self.data_active && self.data_dir == 1 {
                    for shift in 0..4 {
                        let b = ((value >> (shift * 8)) & 0xFF) as u8;
                        self.data.push_back(b);
                    }
                    self.data_remaining = self.data_remaining.saturating_sub(4);
                    if self.data_remaining == 0 {
                        self.finish_write();
                    }
                }
            }
            _ => {
                let i = self.idx(offset);
                if i < self.regs.len() {
                    self.regs[i] = value;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Issue a command with the given index/arg and response/data flags.
    fn issue(d: &mut Sdmmc, index: u8, arg: u32, resp: bool, data: bool, rw: bool) {
        let mut cmd = (index as u32) | CMD_START;
        if resp {
            cmd |= CMD_RESPONSE_EXPECT;
        }
        if data {
            cmd |= CMD_DATA_EXPECTED;
        }
        if rw {
            cmd |= CMD_RW;
        }
        d.write32(CMDARG, arg);
        d.write32(CMD, cmd);
    }

    #[test]
    fn init_sequence_produces_ready_card() {
        let mut d = Sdmmc::new();
        issue(&mut d, 0, 0, false, false, false); // GO_IDLE
        assert_eq!(d.card_state, CardState::Idle);
        issue(&mut d, 8, 0x0000_01AA, true, false, false); // SEND_IF_COND
        assert_eq!(d.read32(RESP0), 0x0000_01AA);
        // ACMD41: CMD55 then CMD41 (twice).
        issue(&mut d, 55, 0, true, false, false);
        issue(&mut d, 41, 0x40FF_8000, true, false, false);
        assert_eq!(d.read32(RESP0) & (1 << 31), 0); // not ready first time
        issue(&mut d, 55, 0, true, false, false);
        issue(&mut d, 41, 0x40FF_8000, true, false, false);
        assert_eq!(d.read32(RESP0) & (1 << 31), 1 << 31); // ready
        assert_eq!(d.read32(RESP0) & (1 << 30), 1 << 30); // CCS (SDHC)
        issue(&mut d, 2, 0, true, false, false); // ALL_SEND_CID (R2)
        assert_eq!(d.read32(RESP0), 0x1234_5678);
        issue(&mut d, 3, 0, true, false, false); // SEND_RCA
        assert_eq!(d.read32(RESP0), 0x1234_0000); // rca<<16
        issue(&mut d, 7, 0x1234_0000, true, false, false); // SELECT
        assert_eq!(d.card_state, CardState::Tran);
        assert_eq!(d.read32(RESP0), 0);
    }

    #[test]
    fn read_block_returns_initial_pattern() {
        let mut d = Sdmmc::new();
        issue(&mut d, 0, 0, false, false, false);
        d.write32(BYTCNT, 512);
        issue(&mut d, 17, 0, true, true, false); // READ_SINGLE_BLOCK
        assert!(d.read32(RINTSTS) & INT_CMD_DONE != 0);
        // Read 512 bytes = 128 FIFO words.
        let mut mismatch = 0;
        for i in 0..128u32 {
            let w = d.read32(FIFO);
            let expected = (((4 * i) & 0xFF) as u32)
                | ((((4 * i + 1) & 0xFF) as u32) << 8)
                | ((((4 * i + 2) & 0xFF) as u32) << 16)
                | ((((4 * i + 3) & 0xFF) as u32) << 24);
            if w != expected {
                mismatch += 1;
            }
        }
        assert_eq!(mismatch, 0);
        assert!(d.read32(RINTSTS) & INT_DATA_OVER != 0);
    }

    #[test]
    fn write_then_read_round_trips() {
        let mut d = Sdmmc::new();
        issue(&mut d, 0, 0, false, false, false);
        d.write32(BYTCNT, 512);
        issue(&mut d, 24, 0, true, true, true); // WRITE_BLOCK
        // Write a recognizable pattern.
        for i in 0..128u32 {
            d.write32(FIFO, i as u32 * 0x0101_0101);
        }
        assert!(d.read32(RINTSTS) & INT_DATA_OVER != 0);
        // Now read it back.
        d.write32(BYTCNT, 512);
        issue(&mut d, 17, 0, true, true, false);
        let mut mismatch = 0;
        for i in 0..128u32 {
            let w = d.read32(FIFO);
            if w != i as u32 * 0x0101_0101 {
                mismatch += 1;
            }
        }
        assert_eq!(mismatch, 0);
    }

    #[test]
    fn acmd51_serves_scr() {
        let mut d = Sdmmc::new();
        issue(&mut d, 0, 0, false, false, false);
        d.write32(BYTCNT, 8);
        issue(&mut d, 55, 0, true, false, false); // CMD55
        issue(&mut d, 51, 0, true, true, false); // ACMD51 SEND_SCR
        assert!(d.read32(RINTSTS) & INT_CMD_DONE != 0);
        let w0 = d.read32(FIFO);
        // SCR first byte = 0x02 (SD_SPEC=2), second = 0x00, third = 0x00, fourth = 0x03.
        assert_eq!(w0, 0x0300_0002);
    }
}
