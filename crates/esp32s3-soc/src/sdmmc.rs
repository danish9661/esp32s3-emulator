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
//! full init sequence + PIO block read/write), the IDMAC machine test
//! (`sdmmc_idmac_walks_descriptors`), and the `esp32s3_sdfat` sketch (full
//! esp-idf SDMMC driver init + FATFS mount/read/write through Arduino
//! `SD_MMC`: CMD5 RTO, CMD6 HS-switch data, ACMD13 SSR data, 4-bit bus,
//! R1 READY/STATE status, CTRL reset self-clear, BMOD.DE-gated IDMAC).

const BLOCK_LEN: usize = 512;
const STORAGE_BLOCKS: usize = 8192; // 4 MB modeled card (FAT16 preformatted)

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
const INT_RTO: u32 = 1 << 8; // response timeout (e.g. CMD5 on a mem-only card)

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

// SD mode R1 status bits (sd_protocol_defs.h): the esp-idf driver gates on
// APP_CMD (CMD55/ACMD handshake), READY_FOR_DATA + TRAN state (every data
// wait), so every R1 carries live status — a constant 0 hangs init.
const R1_APP_CMD: u32 = 1 << 5;
const R1_READY_FOR_DATA: u32 = 1 << 8;
const R1_STATE_POS: u32 = 9;

// BMOD (IDMAC 0x80): sw_reset = bit 0, enable (DE) = bit 7. The old model
// checked bit 0, which is the reset bit — the driver sets DE, so gate on it.
const BMOD_ENABLE: u32 = 1 << 7;
// CTRL (0x00): controller_reset/fifo_reset/dma_reset = bits 0..2.
// `sdmmc_host_reset` polls them until hardware clears; clear on write.
const CTRL_RESET_BITS: u32 = 0x7;

// SCR (ACMD51, 8 bytes MSB-first): SD_SPEC=2 (v2.00), BUS_WIDTHS=1+4 bit.
// The driver requires the 4-bit flag for `sdmmc_init_sd_bus_width`.
const SCR_BYTES: [u8; 8] = [0x02, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
// SWITCH_FUNC status (CMD6, 64 bytes): laid out so that after the driver's
// byte-flip the version reads 1 and function-group-1 reports SDR25 (func 1)
// supported and not busy (sd_protocol_defs.h SD_SFUNC_*).
const SWITCH_RAW: [u32; 16] = [
    0,
    0,
    0,
    0x0000_0200,
    0x0000_0100,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
];
// SD Status (ACMD13, 64 bytes): DAT_BUS_WIDTH=4-bit, AU_SIZE=1MB,
// ERASE_SIZE/TIMEOUT/OFFSET nonzero, DISCARD supported. `sdmmc_decode_ssr`
// never fails, so these are informational + erase-timeout inputs only.
const SSR_RAW: [u32; 16] = [
    0x0000_0080,
    0,
    0x0000_0700,
    0x0000_2908,
    0,
    0,
    0x0000_0002,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
];
// CSD v2.0 (R2 words low -> high = RESP0..RESP3): STRUCTURE=1 (v2.0),
// TAAC=0x0E, TRAN_SPEED=0x5A (50 MHz — the post-HS-switch re-read requires
// it; nothing checks the pre-switch value), CCC=BASIC|BR|BW|ERASE|SWITCH,
// READ_BL_LEN=9, C_SIZE=7 (capacity = 8*1024 = 8192 sectors = 4 MB),
// ERASE_BLK_EN=1, SECTOR_SIZE=0x7F, R2W_FACTOR=2, WRITE_BL_LEN=9.
const CSD_WORDS: [u32; 4] = [0x0A40_0001, 0x0007_7F80, 0x4359_0000, 0x400E_005A];

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
    /// writes), indexed by LBA. Preformatted with a FAT16 volume (MBR +
    /// `HELLO.TXT`) so the esp-idf FATFS driver mounts it.
    storage: Vec<u8>,
    /// Current transfer LBA (set by CMD17/18/24/25).
    lba: u32,
    /// Erase-group bounds (SDHC LBAs) latched by CMD32/CMD33; CMD38 wipes
    /// the inclusive range with 0xFF like a real erase.
    erase_start: u32,
    erase_end: u32,
    /// Pending data transfer FIFO (bytes) for the PIO path.
    data: VecDeque<u8>,
    data_remaining: usize,
    data_dir: u8, // 0 = read (card->host), 1 = write (host->card)
    data_active: bool,
    /// Set when the next data command serves the SCR register instead of storage.
    serve_scr: bool,
    /// Set when the next data command serves CMD6 SWITCH_FUNC status.
    serve_switch: bool,
    /// Set when the next data command serves ACMD13 SD Status.
    serve_ssr: bool,
    /// Pending IDMAC transfer (None for PIO).
    idmac: Option<IdmacXfer>,
}

impl Default for Sdmmc {
    fn default() -> Self {
        Self::new()
    }
}

/// Expand 16 LE words into 64 bytes (LE byte order per word): the byte
/// stream the driver DMAs for CMD6 SWITCH_FUNC / ACMD13 SSR responses.
fn raw_words_le(words: &[u32; 16]) -> Vec<u8> {
    let mut v = Vec::with_capacity(64);
    for w in words {
        v.extend_from_slice(&w.to_le_bytes());
    }
    v
}

fn le16(dst: &mut [u8], off: usize, v: u16) {
    dst[off..off + 2].copy_from_slice(&v.to_le_bytes());
}

fn le32(dst: &mut [u8], off: usize, v: u32) {
    dst[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

/// Preformat storage with a FAT16 volume so the esp-idf FATFS driver mounts:
/// MBR (partition 0 = FAT16 at LBA 64) + BPB + 2x32-sector FATs + 512-entry
/// root dir + one data cluster holding `HELLO.TXT` ("Hello from SDMMC!\n").
/// Geometry: 8192 sectors, 512 B clusters (8024 clusters: FAT16, not FAT12).
fn format_fat16(storage: &mut [u8]) {
    const PART_START: u32 = 64;
    let total: u32 = STORAGE_BLOCKS as u32;
    let part_count = total - PART_START; // 8128
    // MBR partition entry 0 @ 0x1BE: boot 0, type 0x06 (FAT16),
    // LBA start @ 0x1C6, sector count @ 0x1CA.
    storage[0x1C2] = 0x06;
    le32(storage, 0x1C6, PART_START);
    le32(storage, 0x1CA, part_count);
    storage[510] = 0x55;
    storage[511] = 0xAA;
    // BPB @ partition start.
    let v = PART_START as usize * BLOCK_LEN;
    let bpb = &mut storage[v..v + BLOCK_LEN];
    bpb[0..3].copy_from_slice(&[0xEB, 0x3C, 0x90]);
    bpb[3..11].copy_from_slice(b"SDMMCEMU");
    le16(bpb, 11, 512); // bytes/sector
    bpb[13] = 1; // sectors/cluster
    le16(bpb, 14, 8); // reserved sectors
    bpb[16] = 2; // FAT count
    le16(bpb, 17, 512); // root entries
    le16(bpb, 19, part_count as u16); // total16 (8128 fits)
    bpb[21] = 0xF8; // media
    le16(bpb, 22, 32); // FAT size (sectors)
    le16(bpb, 24, 63); // sectors/track
    le16(bpb, 26, 255); // heads
    bpb[36] = 0x80; // drive number
    bpb[38] = 0x29; // extended signature
    le32(bpb, 39, 0x1234_5678); // volume id
    bpb[43..54].copy_from_slice(b"NO NAME    ");
    bpb[54..62].copy_from_slice(b"FAT16   ");
    bpb[510] = 0x55;
    bpb[511] = 0xAA;
    // Both FATs: media + EOC + cluster 2 EOC (single-cluster file).
    for f in 0..2 {
        let fbase = v + (8 + f * 32) * BLOCK_LEN;
        le16(storage, fbase, 0xFFF8);
        le16(storage, fbase + 2, 0xFFFF);
        le16(storage, fbase + 4, 0xFFFF);
    }
    // Root dir entry 0: HELLO.TXT, cluster 2, 18 bytes.
    let rbase = v + (8 + 64) * BLOCK_LEN;
    let e = &mut storage[rbase..rbase + 32];
    e[0..8].copy_from_slice(b"HELLO   ");
    e[8..11].copy_from_slice(b"TXT");
    e[11] = 0x20; // archive
    le16(e, 26, 2); // first cluster
    le32(e, 28, 18); // file size
    // Data cluster 2.
    let dbase = v + (8 + 64 + 32) * BLOCK_LEN;
    storage[dbase..dbase + 18].copy_from_slice(b"Hello from SDMMC!\n");
}

impl Sdmmc {
    pub fn new() -> Self {
        let mut storage = vec![0u8; STORAGE_BLOCKS * BLOCK_LEN];
        format_fat16(&mut storage);
        Self {
            regs: [0u32; 0x400 / 4],
            card_state: CardState::Idle,
            app_cmd: false,
            acmd41_count: 0,
            rca: 0x1234,
            storage,
            lba: 0,
            erase_start: 0,
            erase_end: 0,
            data: VecDeque::new(),
            data_remaining: 0,
            data_dir: 0,
            data_active: false,
            serve_scr: false,
            serve_switch: false,
            serve_ssr: false,
            idmac: None,
        }
    }

    fn idx(&self, offset: u32) -> usize {
        ((offset & 0xFFF) / 4) as usize
    }

    /// Short (R1/R6/R7) status word: APP_CMD when this command follows
    /// CMD55, READY_FOR_DATA in Tran state, live CURRENT_STATE. The esp-idf
    /// driver gates ACMDs, data waits and bus-width setup on these bits.
    fn r1(&self, acmd: bool) -> u32 {
        let state = self.card_state as u32; // Idle=0 Ready=1 Ident=2 Stby=3 Tran=4
        (if acmd { R1_APP_CMD } else { 0 })
            | (if self.card_state == CardState::Tran {
                R1_READY_FOR_DATA
            } else {
                0
            })
            | (state << R1_STATE_POS)
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

        // CMD5 (IO_SEND_OP_COND): a memory-only card never responds. Raise
        // response-timeout so `sdmmc_init_io` classifies it as non-SDIO mem.
        // Real hardware reports CMD_DONE alongside RTO (the driver relies on
        // it to leave SENDING_CMD), so set both.
        if index == 5 {
            self.app_cmd = false;
            self.regs[self.idx(RINTSTS)] |= INT_RTO | INT_CMD_DONE;
            return;
        }

        // ACMD detection: CMD55 sets the app_cmd flag for the next command.
        if index == 55 {
            self.app_cmd = true;
            self.set_resp_short(R1_APP_CMD);
            self.regs[self.idx(RINTSTS)] |= INT_CMD_DONE;
            return;
        }
        // The next command is an ACMD only if CMD55 preceded it.
        let is_acmd = self.app_cmd;
        self.app_cmd = false;

        // Record the transfer LBA for data commands (SDHC: arg = block addr).
        // CMD32/33 latch the erase-group bounds; CMD38 wipes the range.
        match index {
            17 | 18 | 24 | 25 => self.lba = arg,
            32 => self.erase_start = arg,
            33 => self.erase_end = arg,
            38 => self.do_erase(),
            _ => {}
        }
        self.serve_scr = is_acmd && index == 51; // ACMD51 = SEND_SCR (data)
        self.serve_switch = !is_acmd && index == 6 && data_exp; // CMD6 SWITCH_FUNC
        self.serve_ssr = is_acmd && index == 13 && data_exp; // ACMD13 SD_STATUS

        let mut resp = [0u32; 4];
        match index {
            0 => {
                // GO_IDLE_STATE: card -> idle.
                self.card_state = CardState::Idle;
                resp[0] = 0;
            }
            6 => {
                // SWITCH_FUNC (R1 + 512-bit status) or ACMD6 SET_BUS_WIDTH.
                resp[0] = self.r1(is_acmd);
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
                // SEND_RCA (R6): RCA in the high half, card status below.
                self.card_state = CardState::Stby;
                resp[0] = (self.rca << 16) | (self.r1(false) & 0xFFFF);
            }
            7 => {
                // SELECT/DESELECT: arg = RCA<<16.
                let arg_rca = (arg >> 16) & 0xFFFF;
                if arg_rca == self.rca {
                    self.card_state = CardState::Tran;
                    resp[0] = self.r1(false);
                } else if arg_rca == 0 {
                    self.card_state = CardState::Stby;
                    resp[0] = self.r1(false);
                } else {
                    resp[0] = 1 << 3; // status error
                }
            }
            9 => {
                // SEND_CSD (R2): CSD v2.0, 4 MB / 8192 sectors (see CSD_WORDS).
                resp = CSD_WORDS;
            }
            10 => {
                // SEND_CID (R2): fixed CID.
                resp[0] = 0x1234_5678;
                resp[1] = 0x9ABC_DEF0;
                resp[2] = 0x1357_2468;
                resp[3] = 0x0000_00AA;
            }
            12 | 13 | 16 | 17 | 18 | 23 | 24 | 25 | 32 | 33 | 38 | 52 => {
                // Status-bearing commands (STOP, STATUS, BLOCKLEN,
                // single/multi read/write, ERASE_GROUP_*, ERASE, IO_RW).
                resp[0] = self.r1(is_acmd);
            }
            51 => {
                // SEND_SCR (ACMD51, R1 + 8-byte data): live status + SCR data.
                resp[0] = self.r1(is_acmd);
            }
            _ => {
                // Unknown command: return live status (no error bits).
                resp[0] = self.r1(is_acmd);
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
        // If data was expected, set up the transfer. IDMAC if enabled
        // (BMOD.DE, bit 7), else PIO.
        if data_exp {
            let idmac_en = (self.regs[self.idx(IDMAC_CTRL)] & BMOD_ENABLE) != 0;
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

    /// CMD38: wipe the CMD32/33-latched inclusive LBA range with 0xFF
    /// (erased state), clamped to the modeled storage.
    fn do_erase(&mut self) {
        let (lo, hi) = (
            self.erase_start.min(self.erase_end),
            self.erase_start.max(self.erase_end),
        );
        let blocks = STORAGE_BLOCKS as u32;
        for lba in lo..=hi.min(blocks.saturating_sub(1)) {
            let base = lba as usize * BLOCK_LEN;
            if let Some(slice) = self.storage.get_mut(base..base + BLOCK_LEN) {
                slice.fill(0xFF);
            }
        }
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

    /// Source bytes for a read transfer: SWITCH_FUNC status for CMD6, SD
    /// Status for ACMD13, SCR for ACMD51, else storage at the transfer LBA.
    fn data_source(&self, bytcnt: usize) -> Vec<u8> {
        if self.serve_switch {
            return raw_words_le(&SWITCH_RAW).into_iter().take(bytcnt).collect();
        }
        if self.serve_ssr {
            return raw_words_le(&SSR_RAW).into_iter().take(bytcnt).collect();
        }
        if self.serve_scr {
            // SCR: SD_SPEC=2, bus widths 1-bit+4-bit supported.
            return SCR_BYTES.into_iter().cycle().take(bytcnt).collect();
        }
        let base = (self.lba as usize) * BLOCK_LEN;
        let end = (base + bytcnt).min(self.storage.len());
        let mut v = self.storage[base..end].to_vec();
        v.resize(bytcnt, 0);
        v
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
        let _ = lba; // LBA is captured in self.lba; special sources win.
        let src = self.data_source(len);
        if src.len() >= len {
            src[..len].to_vec()
        } else {
            let mut v = src;
            v.resize(len, 0);
            v
        }
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
            CTRL => {
                // Reset bits (controller/fifo/dma) self-clear like silicon;
                // `sdmmc_host_reset` polls them.
                self.regs[self.idx(CTRL)] = value & !CTRL_RESET_BITS;
            }
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
        issue_long(d, index, arg, resp, false, data, rw)
    }

    /// `issue` plus an R2 (136-bit long response) flag.
    fn issue_long(
        d: &mut Sdmmc,
        index: u8,
        arg: u32,
        resp: bool,
        long: bool,
        data: bool,
        rw: bool,
    ) {
        let mut cmd = (index as u32) | CMD_START;
        if resp {
            cmd |= CMD_RESPONSE_EXPECT;
        }
        if long {
            cmd |= CMD_RESPONSE_LONG;
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
        issue_long(&mut d, 2, 0, true, true, false, false); // ALL_SEND_CID (R2)
        assert_eq!(d.read32(RESP0), 0x1234_5678);
        issue(&mut d, 3, 0, true, false, false); // SEND_RCA
        assert_eq!(d.read32(RESP0), 0x1234_0600); // rca<<16 | stby status
        issue(&mut d, 7, 0x1234_0000, true, false, false); // SELECT
        assert_eq!(d.card_state, CardState::Tran);
        assert_eq!(d.read32(RESP0), 0x900); // READY + tran
    }

    #[test]
    fn cmd5_times_out_with_rto() {
        // A memory-only card never answers IO_SEND_OP_COND: RTO (+CMD_DONE,
        // which real hardware reports alongside — the driver needs it to
        // leave SENDING_CMD).
        let mut d = Sdmmc::new();
        issue(&mut d, 5, 0, true, false, false);
        assert_eq!(d.read32(RINTSTS) & INT_RTO, INT_RTO);
        assert_eq!(d.read32(RINTSTS) & INT_CMD_DONE, INT_CMD_DONE);
    }

    #[test]
    fn cmd55_sets_app_cmd_status() {
        // The driver requires APP_CMD in the CMD55 response for every ACMD.
        let mut d = Sdmmc::new();
        issue(&mut d, 55, 0, true, false, false);
        assert_eq!(d.read32(RESP0) & R1_APP_CMD, R1_APP_CMD);
        // ACMD41 returns R3 (OCR, no status bits): first poll not ready...
        issue(&mut d, 41, 0x40FF_8000, true, false, false);
        assert_eq!(d.read32(RESP0) & (1 << 31), 0);
        // ...second poll ready + CCS (SDHC).
        issue(&mut d, 55, 0, true, false, false);
        issue(&mut d, 41, 0x40FF_8000, true, false, false);
        assert_eq!(d.read32(RESP0) & (1 << 31), 1 << 31);
        assert_eq!(d.read32(RESP0) & (1 << 30), 1 << 30);
    }

    #[test]
    fn csd_v2_reports_4mb_capacity() {
        let mut d = Sdmmc::new();
        issue_long(&mut d, 9, 0, true, true, false, false);
        assert_eq!(d.read32(RESP0), CSD_WORDS[0]);
        assert_eq!(d.read32(RESP3), CSD_WORDS[3]);
        // C_SIZE (bits 69:48) = 7 -> capacity = 8*1024 = 8192 sectors.
        let csize = ((d.read32(RESP2) & 0x3F) << 16) | (d.read32(RESP1) >> 16);
        assert_eq!(csize, 7);
        assert_eq!((csize + 1) * 1024, STORAGE_BLOCKS as u32);
        // TRAN_SPEED (bits 103:96) = 0x5A (50 MHz, for the post-HS check).
        assert_eq!(d.read32(RESP3) & 0xFF, 0x5A);
    }

    #[test]
    fn cmd6_serves_switch_status() {
        let mut d = Sdmmc::new();
        d.write32(BYTCNT, 64);
        issue(&mut d, 6, 0x00FF_FFF1, true, true, false); // SWITCH_FUNC query
        assert!(d.read32(RINTSTS) & INT_CMD_DONE != 0);
        let mut words = [0u32; 16];
        for w in words.iter_mut() {
            *w = d.read32(FIFO);
        }
        // After the driver's byte-flip: version 1, SDR25 supported, idle.
        assert_eq!(words[3], 0x0000_0200);
        assert_eq!(words[4], 0x0000_0100);
    }

    #[test]
    fn acmd13_serves_ssr() {
        let mut d = Sdmmc::new();
        d.write32(BYTCNT, 64);
        issue(&mut d, 55, 0, true, false, false);
        issue(&mut d, 13, 0, true, true, false); // ACMD13 SD_STATUS
        assert!(d.read32(RINTSTS) & INT_CMD_DONE != 0);
        let mut words = [0u32; 16];
        for w in words.iter_mut() {
            *w = d.read32(FIFO);
        }
        assert_eq!(words, SSR_RAW);
    }

    #[test]
    fn ctrl_reset_bits_self_clear() {
        // `sdmmc_host_reset` polls these until hardware clears them.
        let mut d = Sdmmc::new();
        d.write32(CTRL, 0x7);
        assert_eq!(d.read32(CTRL) & 0x7, 0);
    }

    #[test]
    fn erase_group_wipes_range_with_ff() {
        let mut d = Sdmmc::new();
        // Scribble block 10, then erase groups 8..12 and verify.
        d.write32(BYTCNT, 512);
        issue(&mut d, 24, 10, true, true, true); // WRITE_BLOCK LBA 10
        for _ in 0..128u32 {
            d.write32(FIFO, 0x1234_5678);
        }
        assert!(d.read32(RINTSTS) & INT_DATA_OVER != 0);
        issue(&mut d, 32, 8, true, false, false); // ERASE_WR_BLK_START
        issue(&mut d, 33, 12, true, false, false); // ERASE_WR_BLK_END
        issue(&mut d, 38, 0, true, false, false); // ERASE
        let base = 10 * BLOCK_LEN;
        assert!(d.storage[base..base + BLOCK_LEN].iter().all(|&b| b == 0xFF));
        // Outside the range is untouched (MBR signature still there).
        assert_eq!(d.storage[510], 0x55);
        assert_eq!(d.storage[511], 0xAA);
    }

    #[test]
    fn fat16_volume_is_formatted() {
        let d = Sdmmc::new();
        // MBR signature + FAT16 partition (LBA start + count fields).
        assert_eq!(d.storage[510], 0x55);
        assert_eq!(d.storage[511], 0xAA);
        assert_eq!(d.storage[0x1C2], 0x06);
        assert_eq!(
            u32::from_le_bytes(d.storage[0x1C6..0x1CA].try_into().unwrap()),
            64
        );
        assert_eq!(
            u32::from_le_bytes(d.storage[0x1CA..0x1CE].try_into().unwrap()),
            STORAGE_BLOCKS as u32 - 64
        );
        // BPB signature + "FAT16" type string.
        let v = 64 * BLOCK_LEN;
        assert_eq!(d.storage[v + 510], 0x55);
        assert_eq!(d.storage[v + 511], 0xAA);
        assert_eq!(&d.storage[v + 54..v + 62], b"FAT16   ");
        // FAT0 media descriptor + HELLO.TXT first-cluster EOC.
        assert_eq!(d.storage[v + 8 * BLOCK_LEN], 0xF8);
        // Root dir entry + file content.
        let r = v + 72 * BLOCK_LEN;
        assert_eq!(&d.storage[r..r + 8], b"HELLO   ");
        assert_eq!(&d.storage[r + 8..r + 11], b"TXT");
        let f = v + 104 * BLOCK_LEN;
        assert_eq!(&d.storage[f..f + 18], b"Hello from SDMMC!\n");
    }

    #[test]
    fn read_mbr_block_via_pio() {
        let mut d = Sdmmc::new();
        d.write32(BYTCNT, 512);
        issue(&mut d, 17, 0, true, true, false); // READ_SINGLE_BLOCK LBA 0
        assert!(d.read32(RINTSTS) & INT_CMD_DONE != 0);
        let mut last = 0u32;
        for _ in 0..128u32 {
            last = d.read32(FIFO);
        }
        // Last word holds the 0x55AA signature (bytes 510..511).
        assert_eq!(last, 0xAA55_0000);
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
            d.write32(FIFO, i * 0x0101_0101);
        }
        assert!(d.read32(RINTSTS) & INT_DATA_OVER != 0);
        // Now read it back.
        d.write32(BYTCNT, 512);
        issue(&mut d, 17, 0, true, true, false);
        let mut mismatch = 0;
        for i in 0..128u32 {
            let w = d.read32(FIFO);
            if w != i * 0x0101_0101 {
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
        // SCR first bytes = 0x02 (SD_SPEC=2), 0x05 (bus widths 1+4 bit).
        assert_eq!(w0, 0x0000_0502);
    }
}
