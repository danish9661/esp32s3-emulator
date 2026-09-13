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
//!
//! eMMC (JEDEC JESD84) is a second personality of the same simulated card:
//! the first MMC-only CMD1 (SEND_OP_COND, which no SD flow ever sends)
//! switches the card into MMC mode (sticky until CMD0). MMC init is
//! CMD0 -> CMD1 (poll OCR busy) -> CMD2 (CID) -> CMD3 (host-assigned RCA)
//! -> CMD7 (select) -> CMD9 (MMC CSD) -> CMD8 (SEND_EXT_CSD, 512 B data)
//! -> CMD6 (SWITCH, applies index/value into the EXT_CSD shadow) ->
//! CMD16/17/24... Block I/O, erase and IDMAC are shared with the SD path
//! (sector addressing; EXT_CSD ERASE_GROUP_DEF=1 justifies LBA erase).
//! Validated by unit tests + the `esp32s3_emmc` poke sketch. The Arduino
//! `SD_MMC` stack always takes the SD path (ACMD41 succeeds), so a full
//! IDF-driver MMC mount is unreachable — poke-level, like TWAI/HMAC/DS.
//! Approximations: CMD6 SWITCH completes instantly (R1b-as-instant, no
//! DAT0 busy); unknown commands return live R1 (lenient, shared).

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
// MMC CSD v1.2 (same RESP order; representative values, poke-asserted):
// STRUCTURE=2 (v1.2, capacity authoritative in EXT_CSD SEC_COUNT like SDHC),
// SPEC_VERS=4, TAAC=0x0E, TRAN_SPEED=0x32 (25 MHz legacy; HS via SWITCH),
// CCC=0xFFF, READ_BL_LEN=9, C_SIZE=maxed, end bit set.
const MMC_CSD_WORDS: [u32; 4] = [0x0000_0001, 0xC000_0000, 0xFFF9_0FFF, 0x840E_0032];
// EXT_CSD (CMD8, 512 bytes) field offsets (JEDEC JESD84).
const EXT_CSD_REV: usize = 192; // EXT_CSD revision (8 = 1.8)
const EXT_CSD_CARD_TYPE: usize = 196; // bit0=26MHz, bit1=52MHz SDR, bit2=DDR52
const EXT_CSD_BUS_WIDTH: usize = 183; // 0=1-bit, 1=4-bit, 2=8-bit (SWITCH)
const EXT_CSD_HS_TIMING: usize = 185; // 0=legacy, 1=high-speed (SWITCH)
const EXT_CSD_ERASE_GROUP_DEF: usize = 175; // 1 = sector-addressed erase
const EXT_CSD_SEC_COUNT: usize = 212; // u32 LE sector count
const EXT_CSD_PARTITION_CONFIG: usize = 179; // access[2:0]: 3 = RPMB
const EXT_CSD_RPMB_SIZE_MULT: usize = 168; // 1 = 128 KB provisioned
// RPMB frame layout (JEDEC JESD84, 512 B): stuff[0..196), MAC[196..228),
// data[228..484), nonce[484..500), write_counter[500..504) BE,
// address[504..506) BE, block_count[506..508) BE, result[508..510) BE,
// type[510..512) BE. The HMAC-SHA256 covers data+nonce+counter+address+
// count+result+type in frame order with the provisioned key.
const RPMB_MAC: usize = 196;
const RPMB_DATA: usize = 228;
const RPMB_NONCE: usize = 484;
const RPMB_COUNTER: usize = 500;
const RPMB_ADDR: usize = 504;
const RPMB_COUNT: usize = 506;
const RPMB_RESULT: usize = 508;
const RPMB_TYPE: usize = 510;
const RPMB_FRAME: usize = 512;
// RPMB request types; responses are request << 8.
const RPMB_REQ_KEY: u16 = 0x0001;
const RPMB_REQ_COUNTER: u16 = 0x0002;
const RPMB_REQ_WRITE: u16 = 0x0003;
const RPMB_REQ_READ: u16 = 0x0004;
// RPMB result codes.
const RPMB_OK: u16 = 0x0000;
const RPMB_GENERAL_FAIL: u16 = 0x0001;
const RPMB_AUTH_FAIL: u16 = 0x0002;
const RPMB_COUNTER_FAIL: u16 = 0x0003;
const RPMB_ADDR_FAIL: u16 = 0x0004;
const RPMB_NO_KEY: u16 = 0x0007;
// Simulated RPMB data frames (256 B each).
const RPMB_FRAMES: usize = 32;

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
    /// MMC personality: set by the first CMD1 (MMC-only; no SD flow sends
    /// it), cleared by CMD0. Selects the MMC responses for CMD3/6/8/9.
    mmc: bool,
    cmd1_count: u32,
    /// EXT_CSD shadow (CMD8 data; CMD6 SWITCH writes bytes into it).
    ext_csd: [u8; 512],
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
    /// Set when the next data command serves CMD8 SEND_EXT_CSD (MMC).
    serve_extcsd: bool,
    /// SDIO function-0 CCCR (256 B, function-0 CIS at 0x1000) for CMD52
    /// byte I/O. Only function 0 exists (no WiFi/BT function behind the
    /// bus — out-of-scope class); CMD52 reads/writes round-trip here and
    /// CMD53 block mode is rejected as unsupported (R1 error), which is
    /// exactly what a function-less card reports.
    cccr: [u8; 256],
    /// RPMB partition selected (EXT_CSD PARTITION_CONFIG access == 3).
    rpmb_selected: bool,
    /// Provisioned RPMB authentication key (None until a 0x0001 programs
    /// it — trusted-environment provisioning, like manufacturing).
    rpmb_key: Option<[u8; 32]>,
    /// RPMB write counter (increments per authenticated write).
    rpmb_counter: u32,
    /// RPMB data store (frame payloads, 256 B each).
    rpmb_data: Vec<u8>,
    /// Staged RPMB response bytes (served to CMD18 reads, then drained).
    rpmb_resp: VecDeque<u8>,
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
        let mut ext_csd = [0u8; 512];
        ext_csd[EXT_CSD_REV] = 8; // EXT_CSD rev 1.8
        ext_csd[EXT_CSD_CARD_TYPE] = 0x07; // 26 MHz + 52 MHz SDR + DDR52
        ext_csd[EXT_CSD_BUS_WIDTH] = 0; // 1-bit default (SWITCH-writable)
        ext_csd[EXT_CSD_HS_TIMING] = 0; // legacy timing (SWITCH-writable)
        ext_csd[EXT_CSD_ERASE_GROUP_DEF] = 1; // sector-addressed erase
        let sectors = (STORAGE_BLOCKS as u32).to_le_bytes();
        ext_csd[EXT_CSD_SEC_COUNT..EXT_CSD_SEC_COUNT + 4].copy_from_slice(&sectors);
        ext_csd[EXT_CSD_RPMB_SIZE_MULT] = 1; // 128 KB RPMB provisioned
        Self {
            regs: [0u32; 0x400 / 4],
            card_state: CardState::Idle,
            app_cmd: false,
            acmd41_count: 0,
            mmc: false,
            cmd1_count: 0,
            ext_csd,
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
            serve_extcsd: false,
            cccr: [0u8; 256],
            rpmb_selected: false,
            rpmb_key: None,
            rpmb_counter: 0,
            rpmb_data: vec![0u8; RPMB_FRAMES * 256],
            rpmb_resp: VecDeque::new(),
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
        self.serve_switch = !is_acmd && !self.mmc && index == 6 && data_exp; // CMD6 SWITCH_FUNC
        self.serve_ssr = is_acmd && index == 13 && data_exp; // ACMD13 SD_STATUS
        self.serve_extcsd = self.mmc && index == 8 && data_exp; // CMD8 SEND_EXT_CSD

        let mut resp = [0u32; 4];
        match index {
            0 => {
                // GO_IDLE_STATE: card -> idle (leaves MMC mode).
                self.card_state = CardState::Idle;
                self.mmc = false;
                self.cmd1_count = 0;
                resp[0] = 0;
            }
            1 => {
                // SEND_OP_COND (MMC-only; no SD flow sends CMD1): enter MMC
                // mode. First poll busy, then ready + HCS (sector mode).
                self.mmc = true;
                self.cmd1_count += 1;
                self.card_state = CardState::Ready;
                if self.cmd1_count == 1 {
                    resp[0] = arg & 0x00FF_FFFF;
                } else {
                    resp[0] = (1 << 31) | (1 << 30) | (arg & 0x00FF_FFFF);
                }
            }
            6 => {
                // SD: SWITCH_FUNC (R1 + 512-bit status) or ACMD6
                // SET_BUS_WIDTH. MMC: SWITCH (R1b, instant here) — write
                // access (arg[31:26] == 3) stores value at index.
                if self.mmc && !is_acmd && (arg >> 26) == 3 {
                    let idx = ((arg >> 16) & 0xFF) as usize;
                    let val = ((arg >> 8) & 0xFF) as u8;
                    self.ext_csd[idx] = val;
                    // PARTITION_CONFIG access field selects the RPMB
                    // partition (3) for subsequent data commands; any
                    // other value returns to the user area.
                    if idx == EXT_CSD_PARTITION_CONFIG {
                        self.rpmb_selected = val & 7 == 3;
                    }
                }
                resp[0] = self.r1(is_acmd);
            }
            8 => {
                if self.mmc {
                    // SEND_EXT_CSD (R1 + 512-byte EXT_CSD data).
                    resp[0] = self.r1(is_acmd);
                } else {
                    // SEND_IF_COND: echo the argument (R7).
                    resp[0] = arg & 0x0000_FFFF;
                }
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
                // SEND_RCA (R6): SD returns the card RCA; MMC takes the
                // host-assigned RCA from the argument.
                if self.mmc {
                    self.rca = (arg >> 16) & 0xFFFF;
                }
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
                // SEND_CSD (R2): SD CSD v2.0, MMC CSD v1.2 (capacity
                // authoritative in EXT_CSD SEC_COUNT, like SDHC).
                resp = if self.mmc { MMC_CSD_WORDS } else { CSD_WORDS };
            }
            10 => {
                // SEND_CID (R2): fixed CID.
                resp[0] = 0x1234_5678;
                resp[1] = 0x9ABC_DEF0;
                resp[2] = 0x1357_2468;
                resp[3] = 0x0000_00AA;
            }
            12 | 13 | 16 | 17 | 18 | 23 | 24 | 25 | 32 | 33 | 38 => {
                // Status-bearing commands (STOP, STATUS, BLOCKLEN,
                // single/multi read/write, ERASE_GROUP_*, ERASE).
                resp[0] = self.r1(is_acmd);
            }
            52 => {
                // IO_RW_DIRECT (SDIO function-0 CCCR byte I/O, no data
                // phase on the DWMMC data path — the byte rides the R5
                // response). Arg: rw[31], func[30:28], raw[27] (read-after-
                // write), addr[25:9], data[7:0]. Only function 0 exists;
                // other functions report the R5 ERROR/COM_CRC bits like a
                // function-less card. R5 = R1 live status with the read
                // byte in [15:8] (sd_protocol_defs.h R5 layout).
                let rw_flag = (arg >> 31) & 1;
                let func = (arg >> 28) & 7;
                let reg = ((arg >> 9) & 0x1_FFFF) as usize;
                let dat = (arg & 0xFF) as u8;
                if func == 0 && reg < 256 {
                    if rw_flag != 0 {
                        self.cccr[reg] = dat;
                    }
                    resp[0] = self.r1(is_acmd) | ((self.cccr[reg] as u32) << 8);
                } else {
                    // No function behind the bus: R5 flags the error
                    // (ERROR bit 11 + FUNCTION_NUMBER bits, no data).
                    resp[0] = self.r1(is_acmd) | (1 << 11) | (func << 4);
                }
            }
            53 => {
                // IO_RW_EXTENDED (SDIO block mode): unsupported — no
                // function behind the bus. Report live status with the R5
                // ERROR bit rather than hanging the driver in a data wait.
                resp[0] = self.r1(is_acmd) | (1 << 11);
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

    /// Source bytes for a read transfer: EXT_CSD for MMC CMD8,
    /// SWITCH_FUNC status for SD CMD6, SD Status for ACMD13, SCR for ACMD51,
    /// else storage at the transfer LBA.
    fn data_source(&mut self, bytcnt: usize) -> Vec<u8> {
        // RPMB partition: serve staged response frames (drained), zeros
        // past them — like a card whose response FIFO runs dry.
        if self.mmc && self.rpmb_selected {
            let mut v = Vec::new();
            while v.len() < bytcnt {
                if let Some(b) = self.rpmb_resp.pop_front() {
                    v.push(b);
                } else {
                    break;
                }
            }
            v.resize(bytcnt, 0);
            return v;
        }
        if self.serve_extcsd {
            let mut v = self.ext_csd[..bytcnt.min(512)].to_vec();
            v.resize(bytcnt, 0);
            return v;
        }
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

    /// HMAC input for an RPMB frame: data + nonce + counter + address +
    /// count + result + type in frame order (JEDEC JESD84), 284 bytes.
    fn rpmb_mac_input(f: &[u8]) -> Vec<u8> {
        let mut v = Vec::with_capacity(284);
        v.extend_from_slice(&f[RPMB_DATA..RPMB_DATA + 256]);
        v.extend_from_slice(&f[RPMB_NONCE..RPMB_NONCE + 16]);
        v.extend_from_slice(&f[RPMB_COUNTER..RPMB_COUNTER + 4]);
        v.extend_from_slice(&f[RPMB_ADDR..RPMB_ADDR + 2]);
        v.extend_from_slice(&f[RPMB_COUNT..RPMB_COUNT + 2]);
        v.extend_from_slice(&f[RPMB_RESULT..RPMB_RESULT + 2]);
        v.extend_from_slice(&f[RPMB_TYPE..RPMB_TYPE + 2]);
        v
    }

    fn rpmb_u16(f: &[u8], off: usize) -> u16 {
        u16::from_be_bytes([f[off], f[off + 1]])
    }

    fn rpmb_u32(f: &[u8], off: usize) -> u32 {
        u32::from_be_bytes([f[off], f[off + 1], f[off + 2], f[off + 3]])
    }

    /// Stage one response frame (type/result/addr/count/data/nonce/
    /// counter + MAC under the provisioned key; zero MAC without a key).
    fn rpmb_respond(
        &mut self,
        ty: u16,
        result: u16,
        addr: u16,
        count: u16,
        data: &[u8],
        nonce: &[u8; 16],
    ) {
        let mut f = [0u8; RPMB_FRAME];
        let take = data.len().min(count as usize * 256);
        f[RPMB_DATA..RPMB_DATA + take].copy_from_slice(&data[..take]);
        f[RPMB_NONCE..RPMB_NONCE + 16].copy_from_slice(nonce);
        f[RPMB_COUNTER..RPMB_COUNTER + 4].copy_from_slice(&self.rpmb_counter.to_be_bytes());
        f[RPMB_ADDR..RPMB_ADDR + 2].copy_from_slice(&addr.to_be_bytes());
        f[RPMB_COUNT..RPMB_COUNT + 2].copy_from_slice(&count.to_be_bytes());
        f[RPMB_RESULT..RPMB_RESULT + 2].copy_from_slice(&result.to_be_bytes());
        f[RPMB_TYPE..RPMB_TYPE + 2].copy_from_slice(&ty.to_be_bytes());
        if let Some(key) = &self.rpmb_key {
            let mac = crate::hmac::hmac_sha256(key, &Self::rpmb_mac_input(&f));
            f[RPMB_MAC..RPMB_MAC + 32].copy_from_slice(&mac);
        }
        self.rpmb_resp.extend(f.iter().cloned());
    }

    /// Process a multi-block authenticated-write transfer (first frame
    /// headers + count-1 raw data chunks): same validation as a single
    /// write, all data blocks stored, one response.
    fn rpmb_write_multi(&mut self, bytes: &[u8], count: u16) {
        let f = &bytes[..RPMB_FRAME];
        let addr = Self::rpmb_u16(f, RPMB_ADDR);
        let mut nonce = [0u8; 16];
        nonce.copy_from_slice(&f[RPMB_NONCE..RPMB_NONCE + 16]);
        let Some(key) = self.rpmb_key else {
            self.rpmb_respond(
                RPMB_REQ_WRITE.wrapping_shl(8),
                RPMB_NO_KEY,
                addr,
                count,
                &[],
                &nonce,
            );
            return;
        };
        let mac = crate::hmac::hmac_sha256(&key, &Self::rpmb_mac_input(f));
        if mac[..] != f[RPMB_MAC..RPMB_MAC + 32]
            || Self::rpmb_u16(f, RPMB_RESULT) != 0
            || Self::rpmb_u32(f, RPMB_COUNTER) != self.rpmb_counter
            || addr as usize + count as usize > RPMB_FRAMES
        {
            let code =
                if mac[..] != f[RPMB_MAC..RPMB_MAC + 32] || Self::rpmb_u16(f, RPMB_RESULT) != 0 {
                    RPMB_AUTH_FAIL
                } else if Self::rpmb_u32(f, RPMB_COUNTER) != self.rpmb_counter {
                    RPMB_COUNTER_FAIL
                } else {
                    RPMB_ADDR_FAIL
                };
            self.rpmb_respond(
                RPMB_REQ_WRITE.wrapping_shl(8),
                code,
                addr,
                count,
                &[],
                &nonce,
            );
            return;
        }
        for i in 0..count as usize {
            let src = RPMB_DATA + i * RPMB_FRAME;
            let base = (addr as usize + i) * 256;
            self.rpmb_data[base..base + 256].copy_from_slice(&bytes[src..src + 256]);
        }
        self.rpmb_counter = self.rpmb_counter.wrapping_add(1);
        self.rpmb_respond(
            RPMB_REQ_WRITE.wrapping_shl(8),
            RPMB_OK,
            addr,
            count,
            &[],
            &nonce,
        );
    }

    /// Process one 512-byte RPMB request frame; stage the response.
    /// Response type = request << 8 (0x0001->0x0100 ... 0x0004->0x0400);
    /// anything else completes with GENERAL_FAILURE.
    fn rpmb_request(&mut self, f: &[u8]) {
        let ty = Self::rpmb_u16(f, RPMB_TYPE);
        let addr = Self::rpmb_u16(f, RPMB_ADDR);
        let count = Self::rpmb_u16(f, RPMB_COUNT).max(1);
        let mut nonce = [0u8; 16];
        nonce.copy_from_slice(&f[RPMB_NONCE..RPMB_NONCE + 16]);
        let resp_ty = ty.wrapping_shl(8);
        // Key programming (trusted-environment provision): the first
        // request stores the MAC field as the key; later ones must carry
        // a valid MAC (key rotation) like an authenticated write.
        if ty == RPMB_REQ_KEY {
            if self.rpmb_key.is_none() {
                let mut key = [0u8; 32];
                key.copy_from_slice(&f[RPMB_MAC..RPMB_MAC + 32]);
                self.rpmb_key = Some(key);
                self.rpmb_counter = 0;
                self.rpmb_respond(resp_ty, RPMB_OK, 0, 1, &[], &nonce);
            } else {
                let key = self.rpmb_key.unwrap_or([0u8; 32]);
                let mac = crate::hmac::hmac_sha256(&key, &Self::rpmb_mac_input(f));
                if mac[..] != f[RPMB_MAC..RPMB_MAC + 32] {
                    self.rpmb_respond(resp_ty, RPMB_AUTH_FAIL, 0, 1, &[], &nonce);
                } else {
                    let mut key = [0u8; 32];
                    key.copy_from_slice(&f[RPMB_DATA..RPMB_DATA + 32]);
                    self.rpmb_key = Some(key);
                    self.rpmb_counter = 0;
                    self.rpmb_respond(resp_ty, RPMB_OK, 0, 1, &[], &nonce);
                }
            }
            return;
        }
        // Everything else needs a provisioned key.
        let Some(key) = self.rpmb_key else {
            self.rpmb_respond(resp_ty, RPMB_NO_KEY, addr, count, &[], &nonce);
            return;
        };
        // Authenticated requests (counter/read/write) must carry a valid
        // MAC computed over the transmitted fields (request result = 0).
        if matches!(ty, RPMB_REQ_COUNTER | RPMB_REQ_WRITE | RPMB_REQ_READ) {
            let mac = crate::hmac::hmac_sha256(&key, &Self::rpmb_mac_input(f));
            if mac[..] != f[RPMB_MAC..RPMB_MAC + 32] {
                self.rpmb_respond(resp_ty, RPMB_AUTH_FAIL, addr, count, &[], &nonce);
                return;
            }
        }
        match ty {
            RPMB_REQ_COUNTER => {
                self.rpmb_respond(resp_ty, RPMB_OK, 0, 1, &[], &nonce);
            }
            RPMB_REQ_WRITE => {
                if Self::rpmb_u16(f, RPMB_RESULT) != 0 {
                    self.rpmb_respond(resp_ty, RPMB_AUTH_FAIL, addr, count, &[], &nonce);
                } else if Self::rpmb_u32(f, RPMB_COUNTER) != self.rpmb_counter {
                    self.rpmb_respond(resp_ty, RPMB_COUNTER_FAIL, addr, count, &[], &nonce);
                } else if addr as usize + count as usize > RPMB_FRAMES as u16 as usize {
                    self.rpmb_respond(resp_ty, RPMB_ADDR_FAIL, addr, count, &[], &nonce);
                } else {
                    // Multi-block writes pack frames back-to-back after the
                    // first (the caller sends count frames; only the first
                    // carries nonce/counter/address — but our PIO/IDMAC
                    // path delivers whole transfers, so re-slice here is
                    // limited to this frame's single data block).
                    let base = addr as usize * 256;
                    self.rpmb_data[base..base + 256]
                        .copy_from_slice(&f[RPMB_DATA..RPMB_DATA + 256]);
                    self.rpmb_counter = self.rpmb_counter.wrapping_add(1);
                    self.rpmb_respond(resp_ty, RPMB_OK, addr, count, &[], &nonce);
                }
            }
            RPMB_REQ_READ => {
                if addr as usize + count as usize > RPMB_FRAMES as u16 as usize {
                    self.rpmb_respond(resp_ty, RPMB_ADDR_FAIL, addr, count, &[], &nonce);
                } else {
                    let base = addr as usize * 256;
                    let end = base + (count as usize) * 256;
                    let data = self.rpmb_data[base..end].to_vec();
                    // One response frame per data block (each with MAC).
                    for (i, blk) in data.chunks(256).enumerate() {
                        self.rpmb_respond(resp_ty, RPMB_OK, addr + i as u16, 1, blk, &nonce);
                    }
                }
            }
            _ => {
                self.rpmb_respond(resp_ty, RPMB_GENERAL_FAIL, addr, count, &[], &nonce);
            }
        }
    }

    /// Finish a PIO write transfer: RPMB request frames go to the
    /// authentication engine, anything else lands in storage.
    fn rpmb_receive(&mut self, bytes: &[u8]) {
        let mut off = 0;
        while off + RPMB_FRAME <= bytes.len() {
            // Multi-block authenticated write: the first frame carries
            // headers, the following count-1 chunks are raw data blocks.
            let ty = Self::rpmb_u16(&bytes[off..], RPMB_TYPE);
            let count = Self::rpmb_u16(&bytes[off..], RPMB_COUNT).max(1);
            if ty == RPMB_REQ_WRITE
                && count > 1
                && off + (count as usize) * RPMB_FRAME <= bytes.len()
            {
                let end = off + (count as usize) * RPMB_FRAME;
                self.rpmb_write_multi(&bytes[off..end], count);
                off = end;
            } else {
                self.rpmb_request(&bytes[off..off + RPMB_FRAME]);
                off += RPMB_FRAME;
            }
        }
        self.data_active = false;
        self.data_remaining = 0;
        self.regs[self.idx(RINTSTS)] |= INT_DATA_OVER;
    }

    /// Finish a PIO write transfer: copy received bytes into storage.
    fn finish_write(&mut self) {
        if self.mmc && self.rpmb_selected {
            let data: Vec<u8> = core::mem::take(&mut self.data).into_iter().collect();
            self.rpmb_receive(&data);
            return;
        }
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
        if self.mmc && self.rpmb_selected {
            self.rpmb_receive(data);
            self.regs[self.idx(IDMAC_RINTSTS)] |= IDMAC_TI;
            return;
        }
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
    pub fn idmac_load(&mut self, lba: u32, len: usize) -> Vec<u8> {
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

    /// Whole-card storage image (FAT16 preformatted): shared with the
    /// SDSPI card so `SD.begin` mounts the same filesystem over SPI.
    pub fn storage_image(&self) -> Vec<u8> {
        self.storage.clone()
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
    pub(super) fn issue(d: &mut Sdmmc, index: u8, arg: u32, resp: bool, data: bool, rw: bool) {
        issue_long(d, index, arg, resp, false, data, rw)
    }

    /// `issue` plus an R2 (136-bit long response) flag.
    pub(super) fn issue_long(
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

    /// CMD12 STOP_TRANSMISSION is accepted with a live R1 (no error):
    /// the single-shot card model never opens a multi-block transfer, so
    /// there is nothing to stop, but drivers may still emit CMD12 (e.g.
    /// after ACMD51/CMD6 data) and must not see a failure. SDSC
    /// byte-addressing is unreachable by construction (the card reports
    /// SDHC/CCS, so the driver always uses block addresses).
    #[test]
    fn stop_transmission_accepted_with_live_r1() {
        let mut d = Sdmmc::new();
        issue(&mut d, 12, 0, true, false, false); // STOP_TRANSMISSION
        assert_ne!(d.read32(RINTSTS) & INT_CMD_DONE, 0, "CMD_DONE latches");
        assert_eq!(d.read32(RINTSTS) & INT_RTO, 0, "no timeout");
        // Idle-state R1 is exactly 0 (no APP_CMD, not Tran, state Idle):
        // no error bits by construction (r1() only sets APP_CMD/READY/
        // CURRENT_STATE, never errors).
        assert_eq!(d.read32(RESP0), 0, "clean idle R1");
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

    /// Drive the card into MMC mode (shared by the MMC tests below).
    fn mmc_init(d: &mut Sdmmc) {
        issue(d, 0, 0, false, false, false); // GO_IDLE (clears MMC too)
        assert!(!d.mmc);
        issue(d, 1, 0x40FF_8000, true, false, false); // SEND_OP_COND
        assert!(d.mmc, "first CMD1 selects MMC mode");
        assert_eq!(d.read32(RESP0) & (1 << 31), 0); // busy
        issue(d, 1, 0x40FF_8000, true, false, false);
        assert_eq!(d.read32(RESP0) & (1 << 31), 1 << 31); // ready
        assert_eq!(d.read32(RESP0) & (1 << 30), 1 << 30); // HCS/sector mode
    }

    #[test]
    fn mmc_cmd1_busy_then_ready() {
        let mut d = Sdmmc::new();
        mmc_init(&mut d);
        assert_eq!(d.card_state, CardState::Ready);
    }

    #[test]
    fn mmc_init_assigns_host_rca_and_selects() {
        let mut d = Sdmmc::new();
        mmc_init(&mut d);
        issue_long(&mut d, 2, 0, true, true, false, false); // ALL_SEND_CID
        assert_eq!(d.card_state, CardState::Ident);
        issue(&mut d, 3, 1 << 16, true, false, false); // SET_RELATIVE_ADDR RCA=1
        assert_eq!(d.rca, 1, "MMC takes the host-assigned RCA");
        assert_eq!(d.card_state, CardState::Stby);
        issue(&mut d, 7, 1 << 16, true, false, false); // SELECT
        assert_eq!(d.card_state, CardState::Tran);
        assert_eq!(d.read32(RESP0), 0x900); // READY + tran
    }

    #[test]
    fn mmc_csd_reports_structure_and_blocklen() {
        let mut d = Sdmmc::new();
        mmc_init(&mut d);
        issue_long(&mut d, 9, 0, true, true, false, false); // SEND_CSD
        assert_eq!(d.read32(RESP0), MMC_CSD_WORDS[0]);
        assert_eq!(d.read32(RESP3), MMC_CSD_WORDS[3]);
        // CSD_STRUCTURE (bits 127:126) = 2 (v1.2, capacity in EXT_CSD).
        assert_eq!(d.read32(RESP3) >> 30, 2);
        // TRAN_SPEED = 0x32, READ_BL_LEN = 9 (512 B).
        assert_eq!(d.read32(RESP3) & 0xFF, 0x32);
        assert_eq!((d.read32(RESP2) >> 16) & 0xF, 9);
    }

    /// Read the 512-byte EXT_CSD into bytes (LE words from the PIO FIFO).
    fn read_ext_csd(d: &mut Sdmmc) -> [u8; 512] {
        d.write32(BYTCNT, 512);
        issue(d, 8, 0, true, true, false); // SEND_EXT_CSD
        assert!(d.read32(RINTSTS) & INT_CMD_DONE != 0);
        let mut out = [0u8; 512];
        for (i, chunk) in out.chunks_mut(4).enumerate() {
            let w = d.read32(FIFO);
            chunk.copy_from_slice(&w.to_le_bytes());
            let _ = i;
        }
        out
    }

    #[test]
    fn mmc_ext_csd_reports_capacity() {
        let mut d = Sdmmc::new();
        mmc_init(&mut d);
        let ext = read_ext_csd(&mut d);
        assert_eq!(ext[EXT_CSD_REV], 8);
        assert_eq!(ext[EXT_CSD_CARD_TYPE], 0x07);
        assert_eq!(ext[EXT_CSD_BUS_WIDTH], 0, "default 1-bit");
        assert_eq!(ext[EXT_CSD_HS_TIMING], 0, "default legacy timing");
        let sec = u32::from_le_bytes([
            ext[EXT_CSD_SEC_COUNT],
            ext[EXT_CSD_SEC_COUNT + 1],
            ext[EXT_CSD_SEC_COUNT + 2],
            ext[EXT_CSD_SEC_COUNT + 3],
        ]);
        assert_eq!(sec, STORAGE_BLOCKS as u32);
    }

    #[test]
    fn mmc_switch_updates_ext_csd_shadow() {
        let mut d = Sdmmc::new();
        mmc_init(&mut d);
        // SWITCH write-byte: access=3, index=BUS_WIDTH(183), value=1 (4-bit).
        let arg = (3 << 26) | ((EXT_CSD_BUS_WIDTH as u32) << 16) | (1 << 8);
        issue(&mut d, 6, arg, true, false, false);
        assert!(d.read32(RINTSTS) & INT_CMD_DONE != 0);
        // SWITCH HS_TIMING(185) = 1 (high-speed).
        let arg = (3 << 26) | ((EXT_CSD_HS_TIMING as u32) << 16) | (1 << 8);
        issue(&mut d, 6, arg, true, false, false);
        let ext = read_ext_csd(&mut d);
        assert_eq!(ext[EXT_CSD_BUS_WIDTH], 1);
        assert_eq!(ext[EXT_CSD_HS_TIMING], 1);
    }

    #[test]
    fn mmc_block_write_read_round_trips() {
        let mut d = Sdmmc::new();
        mmc_init(&mut d);
        issue(&mut d, 3, 1 << 16, true, false, false);
        issue(&mut d, 7, 1 << 16, true, false, false);
        assert_eq!(d.card_state, CardState::Tran);
        d.write32(BYTCNT, 512);
        issue(&mut d, 24, 100, true, true, true); // WRITE_BLOCK LBA 100
        for i in 0..128u32 {
            d.write32(FIFO, 0xDEAD_0000 | i);
        }
        assert!(d.read32(RINTSTS) & INT_DATA_OVER != 0);
        d.write32(BYTCNT, 512);
        issue(&mut d, 17, 100, true, true, false); // READ_SINGLE_BLOCK
        for i in 0..128u32 {
            assert_eq!(d.read32(FIFO), 0xDEAD_0000 | i);
        }
    }
}

#[cfg(test)]
mod rpmb_tests {
    use super::*;
    use crate::hmac::hmac_sha256;

    const KEY: [u8; 32] = [0x42; 32];

    fn frame(
        ty: u16,
        addr: u16,
        count: u16,
        counter: u32,
        data: &[u8],
        key: &[u8; 32],
    ) -> [u8; 512] {
        let mut f = [0u8; 512];
        let n = data.len().min(256);
        f[228..228 + n].copy_from_slice(&data[..n]);
        f[500..504].copy_from_slice(&counter.to_be_bytes());
        f[504..506].copy_from_slice(&addr.to_be_bytes());
        f[506..508].copy_from_slice(&count.to_be_bytes());
        f[510..512].copy_from_slice(&ty.to_be_bytes());
        let mut msg = Vec::new();
        msg.extend_from_slice(&f[228..484]);
        msg.extend_from_slice(&f[484..500]);
        msg.extend_from_slice(&f[500..504]);
        msg.extend_from_slice(&f[504..506]);
        msg.extend_from_slice(&f[506..508]);
        msg.extend_from_slice(&f[508..510]);
        msg.extend_from_slice(&f[510..512]);
        let mac = hmac_sha256(key, &msg);
        f[196..228].copy_from_slice(&mac);
        f
    }

    fn result_of(f: &[u8]) -> u16 {
        u16::from_be_bytes([f[508], f[509]])
    }

    fn type_of(f: &[u8]) -> u16 {
        u16::from_be_bytes([f[510], f[511]])
    }

    /// Enter MMC mode and select the RPMB partition (SWITCH index 179 = 3).
    fn rpmb_mode(d: &mut Sdmmc) {
        super::tests::issue(d, 0, 0, false, false, false);
        super::tests::issue(d, 1, 0x40FF_8000, true, false, false);
        super::tests::issue(d, 1, 0x40FF_8000, true, false, false);
        super::tests::issue(d, 6, (3 << 26) | (179 << 16) | (3 << 8), true, false, false);
        assert!(d.rpmb_selected, "RPMB partition selected");
    }

    fn write_frames(d: &mut Sdmmc, bytes: &[u8]) {
        d.write32(BYTCNT, bytes.len() as u32);
        super::tests::issue(d, 25, 0, true, true, true);
        for w in bytes.chunks(4) {
            let mut b = [0u8; 4];
            b[..w.len()].copy_from_slice(w);
            d.write32(FIFO, u32::from_le_bytes(b));
        }
    }

    fn read_frames(d: &mut Sdmmc, nbytes: usize) -> Vec<u8> {
        d.write32(BYTCNT, nbytes as u32);
        super::tests::issue(d, 18, 0, true, true, false);
        let mut out = Vec::new();
        for _ in 0..nbytes / 4 {
            out.extend_from_slice(&d.read32(FIFO).to_le_bytes());
        }
        out
    }

    #[test]
    fn rpmb_provision_counter_write_read_round_trip() {
        let mut d = Sdmmc::new();
        rpmb_mode(&mut d);
        // Provision (type 1 carries the key in the MAC field).
        let mut prov = [0u8; 512];
        prov[196..228].copy_from_slice(&KEY);
        prov[510..512].copy_from_slice(&1u16.to_be_bytes());
        write_frames(&mut d, &prov);
        let r = read_frames(&mut d, 512);
        assert_eq!(type_of(&r), 0x0100, "provision response type");
        assert_eq!(result_of(&r), 0, "provision OK");
        // Counter reads 0 with a valid MAC.
        let q = frame(2, 0, 1, 0, &[], &KEY);
        write_frames(&mut d, &q);
        let r = read_frames(&mut d, 512);
        assert_eq!(type_of(&r), 0x0200);
        assert_eq!(result_of(&r), 0);
        assert_eq!(&r[500..504], &[0, 0, 0, 0], "counter 0");
        // Authenticated write of one block, then read it back.
        let data = [0xA5u8; 256];
        let w = frame(3, 2, 1, 0, &data, &KEY);
        write_frames(&mut d, &w);
        let r = read_frames(&mut d, 512);
        assert_eq!(type_of(&r), 0x0300);
        assert_eq!(result_of(&r), 0, "write OK");
        let q = frame(4, 2, 1, 0, &[], &KEY);
        write_frames(&mut d, &q);
        let r = read_frames(&mut d, 512);
        assert_eq!(type_of(&r), 0x0400);
        assert_eq!(result_of(&r), 0, "read OK");
        assert_eq!(&r[228..484], &data, "data round-trips");
        // Counter advanced exactly once.
        let q = frame(2, 0, 1, 0, &[], &KEY);
        write_frames(&mut d, &q);
        let r = read_frames(&mut d, 512);
        assert_eq!(&r[500..504], &[0, 0, 0, 1], "counter 1");
    }

    #[test]
    fn rpmb_rejects_bad_mac_stale_counter_and_range() {
        let mut d = Sdmmc::new();
        rpmb_mode(&mut d);
        // Unprovisioned reads fail with NO_KEY.
        let q = frame(2, 0, 1, 0, &[], &KEY);
        write_frames(&mut d, &q);
        let r = read_frames(&mut d, 512);
        assert_eq!(result_of(&r), 0x0007, "NO_KEY before provision");
        // Provision, then tamper one MAC byte on a write.
        let mut prov = [0u8; 512];
        prov[196..228].copy_from_slice(&KEY);
        prov[510..512].copy_from_slice(&1u16.to_be_bytes());
        write_frames(&mut d, &prov);
        let _ = read_frames(&mut d, 512);
        let mut w = frame(3, 0, 1, 0, &[0x5Au8; 256], &KEY);
        w[200] ^= 0xFF;
        write_frames(&mut d, &w);
        let r = read_frames(&mut d, 512);
        assert_eq!(result_of(&r), 0x0002, "AUTH_FAIL on tampered MAC");
        // Stale counter rejected.
        let w = frame(3, 0, 1, 99, &[0x5Au8; 256], &KEY);
        write_frames(&mut d, &w);
        let r = read_frames(&mut d, 512);
        assert_eq!(result_of(&r), 0x0003, "COUNTER_FAIL on stale counter");
        // Out-of-range address rejected.
        let w = frame(4, 100, 1, 0, &[], &KEY);
        write_frames(&mut d, &w);
        let r = read_frames(&mut d, 512);
        assert_eq!(result_of(&r), 0x0004, "ADDR_FAIL out of range");
    }
}

#[cfg(test)]
mod sdio_tests {
    use super::tests::issue;
    use super::*;

    /// CMD52 SDIO function-0 CCCR byte I/O round-trips through the R5
    /// response (write then read-back); unknown functions flag R5 ERROR;
    /// CMD53 block mode reports ERROR instead of hanging a data wait.
    #[test]
    fn sdio_cmd52_cccr_round_trip_and_cmd53_rejected() {
        let mut d = Sdmmc::new();
        issue(
            &mut d,
            52,
            (1 << 31) | (0x02 << 9) | 0x42,
            true,
            false,
            false,
        );
        issue(&mut d, 52, 0x02 << 9, true, false, false);
        assert_eq!((d.read32(RESP0) >> 8) & 0xFF, 0x42, "CCCR read-back");
        issue(&mut d, 52, (1 << 28) | (0x02 << 9), true, false, false);
        assert_ne!(
            d.read32(RESP0) & (1 << 11),
            0,
            "R5 ERROR for missing function"
        );
        issue(&mut d, 53, 0, true, false, false);
        assert_ne!(d.read32(RESP0) & (1 << 11), 0, "CMD53 rejected");
    }
}
