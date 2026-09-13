//! ESP32-S3 GPSPI2/GPSPI3 (general-purpose SPI) master model.
//!

//! Register layout per the S3 TRM GPSPI chapter / spi_struct.h (GPSPI2 at
//! 0x6002_4000, GPSPI3 at 0x6002_5000; 0x6002_8000 is the SD/MMC host, a
//! separate peripheral — see sdmmc.rs).
//!
//! Modeled: CPU-controlled master USR transactions plus GDMA-backed master
//! DMA (peri_sel 0/1). Writing CMD.usr (bit 24) starts a transfer whose phases are enabled by USER:
//! command (USER2.usr_command_value/bitlen), address (ADDR +
//! USER1.usr_addr_bitlen), dummy (USER1.usr_dummy_cyclelen) and data
//! (MS_DLEN.ms_data_bitlen bits, half-duplex MOSI or MISO, or both with
//! USER.doutdin).  The SPI clock period is
//! (clkdiv_pre+1)*(clkcnt_n+1) APB cycles (1 if CLOCK.clk_equ_sysclk),
//! TRM: spi_clk = system/(clkdiv_pre+1)/(clkcnt_n+1).  CMD.usr stays set
//! for the whole transaction and self-clears when it ends; the data buffer
//! (data_buf[16] @ 0x98) is a LITTLE-endian word array: stream byte `i` is
//! word byte `i % 4` (verified against `spi_ll_write_buffer` /
//! `spi_ll_read_buffer`, which memcpy each 32-bit chunk — the Arduino byte
//! path writes `data_buf[0]` as a plain LOW byte and reads the LOW byte
//! back; the word paths pre/post byte-swap with MSB_16/32_SET for
//! MSBFIRST). Bit order (CTRL.wr_bit_order / rd_bit_order, MSB-first
//! default) is honored in the shift register, matching the Arduino
//! MSBFIRST default the sketches use.
//! The module clock gate (CLK_GATE.clk_en) must be set for the clock to
//! run, exactly like real hardware (IDF spi_ll_enable_clock).
//!
//! Interrupts: transaction completion latches INT_RAW.trans_done (bit 12,
//! TRM SPI_DMA_INT_RAW; the classic-ESP32 bit-0 layout does NOT apply);
//! delivery is wired in soc.rs int_pending. Master DMA (GDMA peri_sel 0/1,
//! validated by the spi_dma sketch) and slave DMA (host-driven exchanges
//! through the GDMA IN/OUT links, validated by the spi_slave_dma sketch)
//! are modeled. Not modeled: quad/octal, segments, the CMD.update latch
//! (values are used as written — functionally equivalent once firmware
//! follows the IDF update sequence). MISO input has no device attached, so
//! RX phases read back zeros (unless the host injects, or the SDSPI card
//! below answers).
//!
//! SPI-mode SD card (SDSPI): an in-model card answers CMD frames
//! synchronously inside `complete()` (the Arduino `SD` library polls
//! byte-wise, finishing before any host event round-trip — see
//! `SdspiCard`). Enabled by `sdspi_attach`; validated by the `sdspi`
//! sketch (full `SD.begin` + FAT mount through the real Arduino stack).
//!
//! Slave mode: when SPI_SLAVE.slave_mode (bit 26) is set, CMD.usr no longer
//! starts a master transaction. The external master does not exist in the
//! emulator, so the host drives slave exchanges synchronously at the buffer
//! level (the exact bytes-and-interrupt contract the firmware observes):
//! `slave_inject_write` emulates a master-write-to-slave (captures the bytes
//! into data_buf, records SLAVE1.data_bitlen, raises trans_done),
//! `slave_take_read` emulates a master-read-from-slave (returns the
//! firmware-preloaded data_buf bytes, records the bitlen, raises
//! trans_done). DMA variants of the same exchanges move the bytes through
//! the GDMA IN/OUT links instead. The live slave waveform on Q is not
//! modeled.

// Register offsets (TRM GPSPI chapter).
use alloc::vec;
use alloc::vec::Vec;

pub const SPI_CMD: u32 = 0x00;
pub const SPI_ADDR: u32 = 0x04;
pub const SPI_CTRL: u32 = 0x08;
pub const SPI_CLOCK: u32 = 0x0C;
pub const SPI_USER: u32 = 0x10;
pub const SPI_USER1: u32 = 0x14;
pub const SPI_USER2: u32 = 0x18;
pub const SPI_MS_DLEN: u32 = 0x1C;
pub const SPI_MISC: u32 = 0x20;
pub const SPI_DATA_BUF: u32 = 0x98;
pub const SPI_SLAVE: u32 = 0xE0;
pub const SPI_CLK_GATE: u32 = 0xE8;

// CMD bits (TRM SPI_CMD, per esp32s3 spi_struct.h: update=bit23, usr=bit24).
const CMD_UPDATE: u32 = 1 << 23;
const CMD_USR: u32 = 1 << 24;
// CLOCK bits (TRM SPI_CLOCK).
const CLOCK_EQU_SYSCLK: u32 = 1 << 31;
const CLOCK_CLKDIV_PRE_SHIFT: u32 = 18;
const CLOCK_CLKCNT_N_SHIFT: u32 = 12;
// USER bits (TRM SPI_USER; struct order is LSB-first: doutdin is bit 0,
// usr_mosi bit 27, usr_miso bit 28 — NOT bit-reversed).
// CRITICAL: doutdin is bit 0, so the Arduino idle value USER=0x18000001
// HAS doutdin SET (0x...001, not clear). An earlier revision of this file
// misread doutdin as bit 1 / clear and built a whole sequential-mode
// theory on it — wrong. doutdin SET = full-duplex shared window (the only
// mode the Arduino SD path uses).
const USER_DOUTDIN: u32 = 1 << 0;
const USER_USR_COMMAND: u32 = 1 << 31;
const USER_USR_ADDR: u32 = 1 << 30;
const USER_USR_DUMMY: u32 = 1 << 29;
const USER_USR_MISO: u32 = 1 << 28;
const USER_USR_MOSI: u32 = 1 << 27;
// USER1 fields (TRM SPI_USER1).
const USER1_ADDR_BITLEN_SHIFT: u32 = 27;
const USER1_DUMMY_CYCLELEN_SHIFT: u32 = 0;
// USER2 fields (TRM SPI_USER2).
const USER2_CMD_BITLEN_SHIFT: u32 = 28;
const USER2_CMD_VALUE_SHIFT: u32 = 0;
// MS_DLEN field (TRM SPI_MS_DLEN).
const MS_DLEN_DATA_BITLEN_MASK: u32 = (1 << 18) - 1;
// MISC bits (TRM SPI_MISC).
const MISC_CK_IDLE_EDGE: u32 = 1 << 29;
const MISC_CS0_DIS: u32 = 1 << 0;
const MISC_CS1_DIS: u32 = 1 << 1;
// SLAVE bits (TRM SPI_SLAVE_REG @ 0xE0, spi_struct.h `slave`: clk_mode[1:0],
// clk_mode_13[2], rsck_data_out[3], reserved[7:4], rddma/wrdma/rdbuf/wrbuf
// _bitlen_en[11:8], reserved[21:12], dma_seg_magic[25:22], slave_mode[26],
// soft_reset[27], usr_conf[28]).
const SLAVE_MODE: u32 = 1 << 26;
// SLAVE1 fields (TRM SPI_SLAVE1_REG @ 0xE4): data_bitlen[17:0] (transfer
// length in slave FD/HD mode), last_command[25:18], last_addr[31:26].
pub const SPI_SLAVE1: u32 = 0xE4;
const SLAVE1_DATA_BITLEN_MASK: u32 = (1 << 18) - 1;
// CTRL bits (TRM SPI_CTRL, spi_struct.h `ctrl` + spi_reg.h): idle MOSI
// polarity (d_pol), quad/dual read enables (fread_quad/fread_dual) and
// their command-phase mirrors (fcmd_quad/fcmd_dual), plus address-phase
// mirrors (faddr_quad/faddr_dual), plus per-direction bit order
// (wr_bit_order[30:29] MOSI-first, rd_bit_order[28:27] MISO-first —
// 2-bit fields, only bit 0 used: 1 = LSB-first). The GPSPI USR engine
// serializes every phase single-line (only the byte flow is
// firmware-observable — no external device samples the extra wires, and
// no IDF SPI-master driver flow sets the wide-mode bits), so they are
// accepted as R/W config with no timing effect; they DO route into
// `spi_quad_mode()` for the host fake-device hook. Bit order DOES take
// effect: the shift register honors it (MSB-first default, matching the
// Arduino MSBFIRST default the sketches use).
const CTRL_D_POL: u32 = 1 << 20;
const CTRL_WR_BIT_ORDER: u32 = 1 << 29;
const CTRL_RD_BIT_ORDER: u32 = 1 << 27;
const CTRL_FREAD_QUAD: u32 = 1 << 15;
const CTRL_FREAD_DUAL: u32 = 1 << 14;
const CTRL_FCMD_QUAD: u32 = 1 << 9;
const CTRL_FCMD_DUAL: u32 = 1 << 8;
const CTRL_FADDR_QUAD: u32 = 1 << 6;
const CTRL_FADDR_DUAL: u32 = 1 << 5;
// CLK_GATE bits (TRM SPI_CLK_GATE).
const CLK_GATE_CLK_EN: u32 = 1 << 0;
// Interrupt registers (TRM SPI_DMA_INT_*, NOT the classic-ESP32 SLV layout):
// trans_done = bit 12 (verified against esp32s3 spi_struct.h
// `dma_int_raw.trans_done`; the old bit-0 model hung the real IDF
// `spi_device_transmit`, which enables/waits on bit 12).
pub const SPI_INT_ENA: u32 = 0x34;
pub const SPI_INT_CLR: u32 = 0x38;
pub const SPI_INT_RAW: u32 = 0x3C;
pub const SPI_INT_ST: u32 = 0x40;
/// Software-set register: writing a bit ORs it into INT_RAW (TRM
/// SPI_DMA_INT_SET; `spi_hal_init` forces trans_done this way so the
/// first queued transfer can kick the ISR via `esp_intr_enable`).
pub const SPI_INT_SET: u32 = 0x44;
const INT_TRANS_DONE: u32 = 1 << 12;
// Slave DMA completion (TRM SPI_DMA_INT_RAW: SLV_RD_DMA_DONE bit 8,
// SLV_WR_DMA_DONE bit 9 — same INT_RAW/ENA/CLR block as trans_done).
const INT_RD_DMA_DONE: u32 = 1 << 8;
const INT_WR_DMA_DONE: u32 = 1 << 9;
// DMA_CONF (@ 0x30, TRM SPI_DMA_CONF_REG): dma_rx_ena[25] enables DMA
// receive, dma_tx_ena[26] DMA transmit (master or slave mode alike).
const SPI_DMA_CONF: u32 = 0x30;
const DMA_RX_ENA: u32 = 1 << 25;
const DMA_TX_ENA: u32 = 1 << 26;

const REG_COUNT: usize = 0xF4 / 4;
const DATA_WORDS: usize = 16;

/// CRC-16/XMODEM (poly 0x1021, init 0x0000): the SD data-block check
/// (`sd_diskio_crc.c` `CRC16`, same table). Used for CMD17/CMD24 block
/// framing when the driver enables CRC (CMD59); R1/R7 responses are
/// uncoded (CRC7 is checked by the driver's `crc error` path only on
/// command frames, which real cards gate — and the Arduino driver never
/// rejects our R1 bytes).
fn crc16_xmodem(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// SPI-mode SD card (SDSPI) behind a GPSPI controller: the Arduino `SD`
/// library (`sd_diskio.cpp`, Apache) talks to the card with byte-wise
/// `transfer()` calls (CMD frame + polled R1 + 512-byte block tokens).
/// Those transfers complete inside one `tick()` — before any host event
/// round-trip — so the card MUST answer synchronously in `complete()`.
/// Ground truth is `sd_diskio.cpp` in the installed esp32 core, not the
/// SD physical-layer spec: CMD0 -> R1 0x01 (idle), CMD8 (arg 0x1AA) ->
/// R1 0x01 + R7 echo, CMD55 -> R1 0x01, ACMD41 (HCS) -> R1 0x01 then
/// 0x00 (ready), CMD58 -> R1 0x00 + OCR (CCS), CMD16 -> R1 0x00,
/// CMD17/CMD24 -> R1 0x00 + 0xFE token framing the 512-byte block
/// (storage shares the FAT16 preformat via host provision). Bytes the
/// firmware clocks while the card has nothing staged read 0xFF (the
/// pull-up idle byte `sdWait`/`sdReadBytes` poll for). CMD17/CMD24 data
/// blocks carry a real CRC-16 (`crc16_xmodem`), mandatory once the driver
/// enables CRC with CMD59 (`sdReadBytes` compares `transfer16` against
/// `CRC16(buffer, 512)`; `sdWriteBytes` sends `CRC16(buffer, 512)`).
struct SdspiCard {
    attached: bool,
    idle: bool,
    storage: Vec<u8>,
    /// Staged MISO bytes the next transfers shift out (R1/R7/data).
    out: Vec<u8>,
    /// Bytes collected toward the current 6-byte CMD frame.
    cmd: Vec<u8>,
    /// Pending block-write payload (token + 512 data + 2 CRC).
    write: Vec<u8>,
    /// CMD24/CMD25 armed a write payload: the next token byte (0xFE/0xFC)
    /// must enter `write` even though it is empty (without this the token
    /// falls into the command collector and desyncs the stream — found by
    /// reading `sdWriteBytes`, which sends the token separately via
    /// `spi->write(token)`).
    write_pending: bool,
    /// LBA of the in-progress write (latched by CMD24/CMD25).
    write_lba: u32,
    /// Multi-block write in progress (CMD25: blocks framed by 0xFC, stop
    /// 0xFD — `sdWriteSectors`).
    write_multi: bool,
    /// Multi-block read in progress (CMD18: blocks streamed until CMD12 —
    /// `sdReadSectors`).
    read_multi: bool,
    /// LBA of the next multi-block read.
    read_lba: u32,
    /// DAT0-busy bytes remaining after a block write (card holds the bus
    /// low while programming; the driver polls for nonzero).
    busy: u32,
    /// ACMD41 poll count (first = busy, then ready).
    acmd41: u32,
    /// CMD55 prefix seen: the next CMD41/23/22 decodes as its ACMD.
    app: bool,
    /// Card-selected latch (CS-gated SPI framing): the card only samples
    /// MOSI/responds on MISO while its chip-select is asserted (LOW).
    /// Real SD SPI framing selects the card per-transfer (`sdSelectCard`
    /// drives SS LOW, clocks the frame + polls the R1, then `sdDeselectCard`
    /// releases HIGH). The host calls `select(bool)` on the SS pin's driven
    /// level, and `feed` ignores all MOSI (returning 0xFF pull-up idle) and
    /// drops all collector state while deselected — exactly like silicon,
    /// where a command byte clocked with CS HIGH is never sampled. Without
    /// this, the boot ROM's SPI-flash traffic on the SHARED MOSI line
    /// (same GPSPI2 MOSI/SD pads strapped to the flash in the harness)
    /// pre-fills the collector with garbage and every decode shifts by one
    /// (live SDSPI failure: CMD0 frame reached the card as a 2nd shifted
    /// frame, MISO stayed 0xFF through init, BEGIN BAD — while the
    /// host-driven unit tests, which never desynchronize CS, stayed green).
    selected: bool,
}

impl SdspiCard {
    fn new() -> Self {
        Self {
            attached: false,
            idle: true,
            storage: Vec::new(),
            out: Vec::new(),
            cmd: Vec::new(),
            write: Vec::new(),
            write_pending: false,
            write_lba: 0,
            write_multi: false,
            read_multi: false,
            read_lba: 0,
            busy: 0,
            acmd41: 0,
            app: false,
            selected: false,
        }
    }

    /// Attach the card with `blocks` 512-byte blocks (host frontend).
    fn attach(&mut self, blocks: usize) {
        self.attached = true;
        self.idle = true;
        self.storage = vec![0u8; blocks * 512];
        self.out.clear();
        self.cmd.clear();
        self.write.clear();
        self.write_pending = false;
        self.write_multi = false;
        self.read_multi = false;
        self.busy = 0;
        self.acmd41 = 0;
        self.app = false;
        self.selected = false;
    }

    /// Attach with a preformatted image (host frontend shares the SDMMC
    /// FAT16 bytes so `SD.begin` mounts a real filesystem).
    fn attach_image(&mut self, image: &[u8]) {
        self.attached = true;
        self.idle = true;
        self.storage = image.to_vec();
        self.out.clear();
        self.cmd.clear();
        self.write.clear();
        self.write_pending = false;
        self.write_multi = false;
        self.read_multi = false;
        self.busy = 0;
        self.acmd41 = 0;
        self.app = false;
        self.selected = false;
    }

    /// CS-gate the card: `sel` = true while SS is LOW (selected), false
    /// while HIGH (deselected). Deselection drops ALL in-flight stream
    /// state (partial CMD frame, partial write payload, app-prefix flag) —
    /// silicon samples nothing with CS HIGH, so a deselect/reselect
    /// boundary always starts a fresh frame. Staged `out` responses,
    /// `busy`, and `idle`/`acmd41` SURVIVE (the card keeps its init state
    /// across transactions — only the byte-stream framing resets).
    /// Real firmware selects via the SS pin level (see `Spi::complete`);
    /// unit tests select explicitly (their `xfer` helper calls it).
    fn select(&mut self, sel: bool) {
        if self.selected && !sel {
            self.cmd.clear();
            self.write.clear();
            self.write_pending = false;
            self.write_multi = false;
            self.read_multi = false;
            self.app = false;
        }
        self.selected = sel;
    }

    /// Stage one 512-byte block read (0xFE token + data + CRC-16) for the
    /// LBA in `lba` (`sdReadBytes` polls for 0xFE, streams 512, checks
    /// `transfer16` against `CRC16(buffer, 512)` when CRC is on).
    fn stage_block(&mut self, lba: u32) {
        let base = lba as usize * 512;
        self.out.push(0xFE);
        let start = self.out.len();
        for i in 0..512 {
            self.out.push(*self.storage.get(base + i).unwrap_or(&0));
        }
        let crc = crc16_xmodem(&self.out[start..start + 512]);
        self.out.extend_from_slice(&crc.to_be_bytes());
    }

    fn attached(&self) -> bool {
        self.attached
    }

    /// Feed one MOSI byte; returns the MISO byte for the same clocks.
    /// Command frames are 6 bytes (`cmd|0x40`, arg[4], crc): the response
    /// stages when the 6th byte is collected and is returned by the SAME
    /// call (route-then-pop: the frame's last transfer returns its own
    /// R1, exactly like silicon — the card drives MISO during the same
    /// clocks the master shifts the frame out; the init-sequence test
    /// proves it: CMD0's 6th byte returns 0x01).
    /// The write-path ordering subtlety is handled the other way: a
    /// token/payload byte that COMPLETES the 515-byte write stages 0x05 +
    /// busy — and must NOT pop them in the same call (the CRC byte would
    /// return 0x05 instead of 0xFF and desync every subsequent poll). So
    /// `feed_write` reports whether it JUST completed, and `feed`
    /// suppresses the pop then: the 0x05 surfaces on the next poll, after
    /// the busy bytes drain. Block payload bytes (token/data/CRC,
    /// collected into `write`) never touch the response queue anyway; only
    /// the COMPLETING byte needs the suppression.
    /// While DAT0-busy (post-write programming phase) the bus reads 0x00
    /// AND incoming bytes still route (a CMD frame may legally follow the
    /// busy release with no idle gap).
    /// Feed one MOSI byte; returns the MISO byte for the same clocks.
    /// Silicon order: POP-then-ROUTE. The card shifts the head of its
    /// response queue out on the CURRENT clocks, THEN samples MOSI into
    /// its command collector (the DO line is driven from flops clocked
    /// before the DI sample — there is no same-cycle command→response
    /// path; minimum Ncr is 1 byte, exactly what `sdCommand`'s
    /// `transfer(0xFF)` poll loop absorbs).
    ///
    /// Concretely: the response to a 6-byte frame stages when its 6th byte
    /// is collected, and surfaces on the NEXT byte's clocks (the frame's
    /// last transfer returns the STALE head, 0xFF when idle — this is why
    /// the frame buffer must be re-read AFTER completion, and why the
    /// driver always polls: `sdCommand`'s 9x `transfer(0xFF)` loop).
    ///
    /// The two historical bugs this ordering fixes (both found live,
    /// 2026-09-13):
    /// (1) route-then-pop returned the JUST-staged R1 on the frame's own
    /// clocks (init-sequence unit test green) but desyncs real firmware:
    /// the driver's W1-overwriting `writeBytes(cmdPacket,6)` + SEPARATE
    /// `transfer(0xFF)` polls mean route-first double-counts — the poll
    /// byte re-enters the collector and every decode shifts by one (live
    /// SDSPI trace: CMD0 frame sent, MISO stayed 0xFF, BEGIN BAD).
    /// (2) the write path's `just_completed` pop-suppression is unnecessary
    /// under pop-then-route: the completing CRC byte pops the pre-completion
    /// head (0xFF) and only then stages 0x05+busy — the CRC byte correctly
    /// returns 0xFF with no special case.
    ///
    /// While DAT0-busy (post-write programming phase) the bus reads 0x00
    /// AND incoming bytes still route (a CMD frame may legally follow the
    /// busy release with no idle gap — popping first keeps the release
    /// byte visible instead of swallowing it).
    fn feed(&mut self, mosi: u8) -> u8 {
        if !self.attached {
            return 0xFF;
        }
        // CS-gate FIRST: with SS HIGH the card samples nothing (silicon
        // holds DO tri-stated = pull-up 0xFF) and drops any partial stream
        // state on the falling edge (see `select`). Unit tests drive the
        // card selected the whole time (unit-test `xfer` selects explicitly).
        if !self.selected {
            return 0xFF;
        }
        // Pop FIRST: MISO for the current clocks is whatever was staged
        // before this byte arrived (0xFF pull-up when the queue is empty;
        // DAT0-busy 0x00 ONLY when the queue is empty — a staged 0x05
        // data-response pops ahead of the busy line, see `feed_write`).
        // NOTE: the pop-time `out.is_empty()` check and the busy decrement
        // MUST be atomic with the pop (same branch): checking `busy>0`
        // without the empty-gate, or decrementing in a separate branch,
        // buries a just-staged 0x05 behind 0x00s (found twice via the
        // round-trip trace).
        let r = if self.busy > 0 && self.out.is_empty() {
            // DAT0 busy after a block write: hold the bus low (the driver
            // polls for the release before reading the data response).
            self.busy -= 1;
            if self.busy == 0 && self.read_multi {
                // CMD18 stream tail: the next block stages once the stop
                // handshake drains (keeps `sdReadSectors` fed per block).
                let lba = self.read_lba;
                self.read_lba = self.read_lba.wrapping_add(1);
                self.stage_block(lba);
            }
            // Busy still routes (see above) unless a write payload is
            // open — payload bytes are absorbed by `feed_write`. The
            // returned byte is busy-0x00 regardless (DAT0 is driven, not
            // the response queue).
            if !self.write.is_empty() || self.write_pending || (self.write_multi && mosi != 0xFF) {
                self.feed_write(mosi);
            } else {
                self.collect_cmd(mosi);
            }
            return 0x00;
        } else {
            // A CMD18 multi-block stream stages the next block once the
            // current one is fully consumed (the stop handshake drains
            // first). Multi-write idle bytes (0xFF between 0x05 and the
            // next 0xFC, or ahead of 0xFD) must NOT be mistaken for command
            // frames.
            if self.read_multi
                && self.out.is_empty()
                && !self.write_pending
                && self.write.is_empty()
            {
                let lba = self.read_lba;
                self.read_lba = self.read_lba.wrapping_add(1);
                self.stage_block(lba);
            }
            if !self.out.is_empty() {
                self.out.remove(0)
            } else {
                0xFF
            }
        };
        // Route the incoming byte AFTER popping (see above).
        // An in-progress block write consumes its payload transparently
        // (data-response token 0x05 stages once the 515 bytes land); in a
        // multi stream the framing bytes route there too (feed_write parks
        // idle 0xFF and consumes the 0xFD stop). `write_pending` (armed by
        // CMD24/CMD25) routes ONLY the token byte itself: it is consumed
        // (set false) by the token's own `feed_write` call, so LATER bytes
        // (data/CRC) route via `!write.is_empty()`.
        // RESPONSE-POLL BYTES (0xFF, or a 0x00 R1-poll echo) between CMD24's
        // R1 and the token MUST NOT clear the arm: they are pure pops (the
        // collector drops them: 0xFF is idle-skipped, a non-start byte with
        // an empty collector is dropped). `write_pending` is therefore
        // cleared ONLY in `feed_write`'s token branch — never here and never
        // by a poll (an earlier revision cleared a `pending`-style flag in
        // `feed` itself and the token fell into `collect_cmd`, desyncing the
        // stream — per-feed log showed wlen=0 on all 515 payload bytes).
        // BUSY-NOTE: the DAT0-busy check lives ONLY in the pop half above.
        // `busy` is armed by `feed_write` DURING this route half (payload
        // completion stages 0x05 + sets busy=2); checking busy here again
        // would return 0x00 for the completing CRC byte itself instead of
        // the popped head 0xFF (found via the round-trip trace: crc2 read
        // 0x00, 0x05 never surfaced, write appeared to hang).
        // PENDING-NOTE: `write_pending` is checked here (route half) but
        // CLEARED only inside `feed_write`'s token branch — never by a
        // response-poll byte. A poll (0xFF) between CMD24's R1 and the
        // token must NOT clear the arm (an earlier revision cleared it in
        // `feed` itself and the token fell into `collect_cmd`, desyncing
        // the stream — found via the per-feed debug log: wlen=0 on all 515
        // payload bytes).
        if !self.write.is_empty() || self.write_pending || (self.write_multi && mosi != 0xFF) {
            self.feed_write(mosi);
        } else {
            self.collect_cmd(mosi);
        }
        r
    }

    /// Collect CMD-frame bytes; on the 6th byte decode and stage output.
    /// `sd_diskio.cpp` `sdCommand` sends 6-byte frames via MOSI-only
    /// `spi->writeBytes` (MISO-unstaged there) but polls the R1 via
    /// full-duplex `transfer(0xFF)` — either path may carry frame bytes, so
    /// both route here. A frame's 6 bytes may also arrive across several
    /// transfers (byte-wise `transfer()`), so bytes accumulate until 6.
    /// Only a byte with the `01` top bits (`cmd|0x40`) opens a frame: 0xFF
    /// is the pull-up idle level, and any other non-start byte (a data
    /// token like 0xFE/0xFC, or a stray 0x00/0x01 poll echo) is dropped
    /// rather than accreted — otherwise each command shifts the next
    /// decode (found live: a stale 0x00 ahead of CMD24's 0x58 decoded as
    /// cmd 0x18/arg 0x000002xx and no R1 ever staged).
    fn collect_cmd(&mut self, mosi: u8) {
        if self.cmd.is_empty() {
            // Idle (no frame): only a command-start byte opens a frame.
            // 0xFF is the pull-up idle level; any other byte without the
            // `01` top bits is a stale response/token — drop it.
            if mosi & 0xC0 != 0x40 {
                return;
            }
        }
        self.cmd.push(mosi);
        if self.cmd.len() < 6 {
            return;
        }
        let f = [
            self.cmd[0],
            self.cmd[1],
            self.cmd[2],
            self.cmd[3],
            self.cmd[4],
            self.cmd[5],
        ];
        self.cmd.clear();
        self.decode(f);
    }

    /// Collect a whole MOSI-only transfer's bytes as command-frame bytes
    /// (the `spi->writeBytes(cmdPacket, 6)` path stages no MISO, so the
    /// per-byte `feed` never runs — without this no command is ever
    /// decoded). Same stale-byte rule as `collect_cmd`: only a byte with
    /// the `01` top bits opens a frame (0xFF fill, e.g. the trailing
    /// `cmdPacket[6]`, and stale R1/token bytes are skipped).
    fn collect_cmd_bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            if self.cmd.is_empty() && b & 0xC0 != 0x40 {
                continue;
            }
            self.cmd.push(b);
            if self.cmd.len() >= 6 {
                let f = [
                    self.cmd[0],
                    self.cmd[1],
                    self.cmd[2],
                    self.cmd[3],
                    self.cmd[4],
                    self.cmd[5],
                ];
                self.cmd.clear();
                self.decode(f);
            }
        }
    }

    /// Decode one 6-byte CMD frame and stage its response bytes.
    /// `app` (CMD55 prefix) is consumed by EVERY decode: only the command
    /// immediately following CMD55 is its ACMD — any other command in
    /// between is a new non-ACMD command (a stale flag would mis-decode a
    /// later bare CMD41 the driver retries after init).
    /// Unexpected bytes (not the `01xxxxxx` command-start pattern) are
    /// IGNORED, not decoded: the collector only calls this with 6 bytes
    /// starting at a command-start byte (see `collect_cmd`), so reaching
    /// here with anything else is a harness bug — and decoding it would
    /// stage a phantom R1 that desyncs the stream (found live: a stale
    /// 0x00 decoded as cmd 0, re-entering idle mid-session and silently
    /// swallowing the real CMD24's R1).
    fn decode(&mut self, f: [u8; 6]) {
        if f[0] & 0xC0 != 0x40 {
            return;
        }
        let cmd = f[0] & 0x3F;
        let arg = u32::from_be_bytes([f[1], f[2], f[3], f[4]]);
        let app = core::mem::replace(&mut self.app, false);
        match cmd {
            0 => {
                self.idle = true;
                self.acmd41 = 0;
                self.out.push(0x01);
            }
            8 => {
                self.out.push(if self.idle { 0x01 } else { 0x00 });
                self.out.extend_from_slice(&arg.to_be_bytes());
            }
            55 => {
                // APP_CMD prefix: R1 only. Sets the app-flag so the NEXT
                // command decodes as its ACMD (41/23/22/42 — `sdCommand`
                // wraps those in a CMD55 prefix). Cleared when any other
                // command decodes (a bare CMD41/23/22 outside the prefix is
                // illegal, R1 0x04 like silicon — the old model answered
                // busy/ready to a bare CMD41, which can never happen on the
                // wire; and a stale flag would mis-decode a later bare
                // CMD41 the driver retries after init).
                self.app = true;
                self.out.push(if self.idle { 0x01 } else { 0x00 });
            }
            41 => {
                // ACMD41 (APP_OP_COND): only valid after a CMD55 prefix —
                // without it this is an illegal command (R1 0x04, like
                // silicon; the old model answered busy/ready to a bare
                // CMD41, which can never happen on the wire).
                if !app {
                    self.out.push(0x04);
                } else {
                    self.acmd41 += 1;
                    if self.acmd41 == 1 {
                        self.out.push(0x01);
                    } else {
                        self.idle = false;
                        self.out.push(0x00);
                    }
                }
            }
            58 => {
                // READ_OCR: R1 + 4-byte OCR. Bit 20 (3.3 V accepted) must
                // be set or `sdcard_init` fails ("READ_OCR failed"); bit 30
                // (CCS) selects SDHC vs SD (`sd_diskio.cpp` init).
                self.out.push(if self.idle { 0x01 } else { 0x00 });
                let ocr: u32 = if self.idle {
                    0x0010_0000 | 0x00FF_8000
                } else {
                    0xC010_0000 | 0x00FF_8000
                };
                self.out.extend_from_slice(&ocr.to_be_bytes());
            }
            59 => {
                // CRC_ON_OFF: R1 only (sd_diskio enables CRC; the card
                // accepts — real cards reply 0x01/0x00, never 0x05).
                self.out.push(if self.idle { 0x01 } else { 0x00 });
            }
            16 => {
                self.out.push(if self.idle { 0x01 } else { 0x00 });
            }
            9 => {
                // SEND_CSD (R1 + 0xFE token + 16-byte CSD + CRC-16): framed
                // exactly like a single-block read (`sdGetSectorsCount`
                // fetches it via `sdReadBytes`, which polls for 0xFE and
                // validates `transfer16` against `CRC16(buffer, 16)` while
                // CRC is on). CSD v2.0 words (see `sdmmc.rs` CSD_WORDS);
                // byte order MSW-first (proven: only that order yields
                // STRUCTURE v2.0 + C_SIZE 7 = 8192 sectors).
                self.out.push(if self.idle { 0x01 } else { 0x00 });
                if !self.idle {
                    self.out.push(0xFE);
                    let start = self.out.len();
                    for w in [0x400E_005Au32, 0x4359_0000, 0x0007_7F80, 0x0A40_0001] {
                        self.out.extend_from_slice(&w.to_be_bytes());
                    }
                    let crc = crc16_xmodem(&self.out[start..start + 16]);
                    self.out.extend_from_slice(&crc.to_be_bytes());
                }
            }
            17 => {
                self.out.push(if self.idle { 0x01 } else { 0x00 });
                if !self.idle {
                    self.stage_block(arg);
                }
            }
            18 => {
                // READ_MULTIPLE_BLOCK: R1 then one staged block per sector
                // (`sdReadSectors`); CMD12 (`_`) ends the stream. CMD18 with
                // count already handled per-block the same way.
                self.out.push(if self.idle { 0x01 } else { 0x00 });
                if !self.idle {
                    self.stage_block(arg);
                    self.read_multi = true;
                    self.read_lba = arg.wrapping_add(1);
                }
            }
            12 => {
                // STOP_TRANSMISSION ends a CMD18 stream. If a block is still
                // staged (single CMD12 polling), complete it first — the
                // driver checks the stop R1 before reading the tail. R1 0x00
                // in data-accept state.
                self.read_multi = false;
                self.out.push(if self.idle { 0x01 } else { 0x00 });
            }
            13 => {
                // SEND_STATUS: R1 (0x00 ready) + status byte (`sdCommand`
                // reads one more byte for SEND_STATUS; `sdWriteSector` gates
                // the write on `resp == 0`).
                self.out.push(if self.idle { 0x01 } else { 0x00 });
                self.out.push(0x00);
            }
            23 => {
                // SET_WR_BLK_ERASE_COUNT (ACMD23 pre-erase, `sdWriteSectors`
                // `sdTransaction`): R1 only (ACMD framing via `sdCommand`).
                // Requires the CMD55 prefix like ACMD41 (`app`); a bare
                // CMD23 is illegal (R1 0x04).
                if !app {
                    self.out.push(0x04);
                } else {
                    self.out.push(if self.idle { 0x01 } else { 0x00 });
                }
            }
            22 => {
                // SEND_NUM_WR_BLOCKS (ACMD22 post-`STOP_TRANSMISSION`,
                // `sdWriteSectors` recovery): R1 + 0xFE token + 4-byte
                // count + CRC-16 (`sdReadBytes` framing: 0xFE poll, 4 data
                // bytes, `transfer16` CRC check while CRC is on). Zero
                // blocks written so far is honest on the success path
                // (never read there). Requires the CMD55 prefix (`app`).
                if !app {
                    self.out.push(0x04);
                } else {
                    self.out.push(if self.idle { 0x01 } else { 0x00 });
                    if !self.idle {
                        self.out.push(0xFE);
                        let start = self.out.len();
                        self.out.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]);
                        let crc = crc16_xmodem(&self.out[start..start + 4]);
                        self.out.extend_from_slice(&crc.to_be_bytes());
                    }
                }
            }
            24 => {
                self.out.push(if self.idle { 0x01 } else { 0x00 });
                if !self.idle {
                    self.write_lba = arg;
                    self.write.clear();
                    self.write_pending = true;
                    self.write_multi = false;
                    // NOTE: no DAT0 busy here — `busy` belongs to the
                    // post-payload programming phase (armed in `feed_write`
                    // once the 515 bytes land). Arming it at CMD24 time
                    // holds the token/data bytes at 0x00 and the payload
                    // never stages (found via the write round-trip test).
                    // Same for CMD25 below. ALSO no `app` gate: CMD24/25
                    // are plain (non-ACMD) commands — `sdWriteSector` sends
                    // them via `sdCommand` directly, never wrapped in
                    // CMD55 (only 41/23/22/42 take the prefix).
                }
            }
            25 => {
                // WRITE_MULTIPLE_BLOCK: R1 then per-block 0xFC framing
                // (`sdWriteSectors`); 0xFD (`sdStop`) ends the stream.
                self.out.push(if self.idle { 0x01 } else { 0x00 });
                if !self.idle {
                    self.write_lba = arg;
                    self.write.clear();
                    self.write_pending = true;
                    self.write_multi = true;
                }
            }
            _ => {
                self.out.push(if self.idle { 0x01 } else { 0x00 });
            }
        }
    }

    /// Feed one byte of a CMD24/CMD25 block-write payload (token + 512
    /// data + 2 CRC, per `sdWriteBytes` which sends the token with
    /// `spi->write(token)` then the block). Single (CMD24, token 0xFE) and
    /// multi (CMD25, per-block token 0xFC, stop 0xFD via `sdStop`) share
    /// this path: the token byte selects framing, 0xFD outside a block
    /// ends the multi stream (consumed).
    ///
    /// No return value: under pop-then-route (`feed`) the completing byte
    /// pops the pre-completion head and only then stages 0x05+busy, so the
    /// CRC byte correctly returns 0xFF with no suppression needed (the
    /// 0x05 surfaces on a later poll, after the busy bytes drain).
    fn feed_write(&mut self, mosi: u8) {
        // `write_pending` (armed by CMD24/CMD25) means "the token byte is
        // still expected" and is CONSUMED (set false) by the token's own
        // call below — never re-armed here (an earlier revision set it on
        // the 0xFF-idle path and every payload byte fell into `collect_cmd`
        // instead; the write buffer stayed empty and no 0x05 ever staged).
        // Idle 0xFF between multi blocks: stay parked (the next 0xFC or
        // 0xFD starts) WITHOUT clearing the arm (a 0xFF poll between R1
        // and token must not disarm — the token arrives later). 0xFD ends
        // the multi stream (consumed).
        // First payload byte must be a data token (0xFE single, 0xFC
        // multi); anything else aborts the write (stale-stream protection,
        // then re-collect as command).
        if self.write.is_empty() {
            if mosi == 0xFD && self.write_multi {
                self.write_multi = false;
                self.write_pending = false;
                self.collect_cmd(mosi);
                return;
            }
            if mosi == 0xFF {
                // Pure idle poll: hold the arm (single) or park (multi).
                // NEVER route into the collector (0xFF is skipped there
                // anyway) and NEVER clear a single-block arm.
                return;
            }
            self.write_pending = false;
            if mosi != 0xFE && !(mosi == 0xFC && self.write_multi) {
                self.collect_cmd(mosi);
            } else {
                self.write.push(mosi);
            }
            return;
        }
        self.write.push(mosi);
        if self.write.len() >= 1 + 512 + 2 {
            let base = self.write_lba as usize * 512;
            for (i, &b) in self.write[1..1 + 512].iter().enumerate() {
                if base + i < self.storage.len() {
                    self.storage[base + i] = b;
                }
            }
            self.write.clear();
            if self.write_multi {
                self.write_lba = self.write_lba.wrapping_add(1);
            }
            // Post-payload programming phase: the card holds DAT0 low
            // (0x00 bytes) while it commits the block, then releases to
            // 0xFF idle. `sdWriteSector` polls `transfer(0xFF)` until
            // nonzero, so at least one busy byte must precede the release.
            // (Armed HERE, not at CMD24 time — the token/payload bytes
            // must clock 0xFF, not busy-0x00.)
            // QUEUE ORDER MATTERS: the data-response 0x05 MUST be popped
            // BEFORE any busy-0x00 (silicon: the card answers 0x05 on the
            // first clocks after the payload, THEN holds DAT0 low while
            // programming). So 0x05 is pushed FIRST and busy SECOND — but
            // `busy` is only consulted when the RESPONSE QUEUE IS EMPTY
            // (see `feed`'s pop half): while 0x05 (or any staged byte) is
            // queued it pops ahead of the busy line. Push-0x05-then-arm
            // alone is NOT enough (found via the round-trip trace: busy
            // checked first buried the 0x05 behind 0x00s the driver never
            // polls past).
            self.out.push(0x05);
            self.busy = 2;
        }
    }
}

/// Snapshot of a running USR transaction.
struct Txn {
    /// APB cycles per SPI clock bit.
    bit_cycles: u64,
    /// APB ticks the clock stays LOW per SPI clock cycle:
    /// (clkcnt_n - clkcnt_h) * (clkdiv_pre+1) (TRM SPI_CLOCK: the (clkcnt_n+1)
    /// counter runs on the pre-divided clock).
    low_ticks: u64,
    /// Total bit slots: command + address + dummy + data.
    total_bits: u64,
    cmd_bits: u64,
    addr_bits: u64,
    /// Data phase width in bits: `d` (MS_DLEN+1) ALWAYS (spi_struct.h has a
    /// single `ms_data_bitlen` for master CPU and DMA transfers; with
    /// USER.doutdin set MOSI and MISO share the window — the Arduino
    /// polling path `spiTransferByteNL` runs with doutdin SET
    /// (`spiStartBus` sets usr_mosi|usr_miso|doutdin once, so every
    /// transfer shows USER=0x18000001 — proven by the live SDSPI probe).
    data_bits: u64,
    have_mosi: bool,
    have_miso: bool,
    /// USER.doutdin at trigger: 1 = full-duplex overlap (MOSI and MISO
    /// share the `d`-bit window, the Arduino/SDSPI reality); 0 =
    /// sequential half-duplex (MOSI `d` bits first, then MISO `d` bits).
    doutdin: bool,
    /// DMA-backed (GDMA-fed) transfer: MOSI bits come from staged GDMA
    /// bytes, MISO lands in `dma_rx` instead of the data buffer.
    dma: bool,
    /// Idle clock level (CPOL).
    ck_pol: u32,
    /// Idle MOSI level (CTRL.d_pol).
    d_pol: u32,
    /// MOSI bit order (CTRL.wr_bit_order bit 0: 1 = LSB-first).
    wr_lsb: bool,
    /// MISO bit order (CTRL.rd_bit_order bit 0: 1 = LSB-first).
    rd_lsb: bool,
    cmd_value: u32,
    addr_value: u32,
    /// Data buffer snapshot: LITTLE-endian words (stream byte `i` is word
    /// byte `i % 4`, matching `spi_ll_write_buffer`'s memcpy layout).
    /// 16 words for USR transfers, sized to cover data_bits for DMA
    /// transfers.
    buf: Vec<u32>,
    /// APB cycles remaining.
    remain: u64,
}

/// Phase/timing plan shared by USR and DMA triggers.
struct TxnPlan {
    bit_cycles: u64,
    low_ticks: u64,
    cmd_bits: u64,
    addr_bits: u64,
    dummy_cycles: u64,
    data_bits: u64,
    have_mosi: bool,
    have_miso: bool,
    doutdin: bool,
    ck_pol: u32,
    d_pol: u32,
    wr_lsb: bool,
    rd_lsb: bool,
    cmd_value: u32,
    addr_value: u32,
}

/// One general-purpose SPI controller (GPSPI2 = idx 0, GPSPI3 = idx 1).
pub struct Spi {
    regs: [u32; REG_COUNT],
    idx: u32,
    txn: Option<Txn>,
    /// MOSI byte stream of the last completed transfer (host reads it on an
    /// `EVT_SPI_XFER` event). `None` until a transfer finishes.
    last_tx: Option<Vec<u8>>,
    /// MISO bytes injected by the host for the next transfer (a virtual SPI
    /// device's response). `None` => read back zeros (no device).
    pending_miso: Option<Vec<u8>>,
    /// GDMA-staged TX bytes for a DMA-backed master transfer (fed by the
    /// GDMA `out` walk, consumed by `dma_trigger`).
    dma_tx: Vec<u8>,
    /// Captured RX bytes of the last DMA transfer's captured RX bytes (served
    /// to the GDMA `in` walk via `dma_rx_word`; overwritten per transfer,
    /// never drained, so the IN link may start before/after completion).
    dma_rx: Vec<u8>,
    /// Host fake quad-SPI device store (see `quad_fake_provision`).
    quad_fake: Option<Vec<u8>>,
    /// SPI-mode SD card (SDSPI, Arduino `SD` lib): answers CMD frames
    /// synchronously inside `complete()` (byte-wise polling transfers
    /// finish before any host event round-trip).
    sdspi: SdspiCard,
    /// SS-pin level latched by the SoC before each transaction completes
    /// (true = SS LOW = card selected). The SoC owns the GPIO block, so it
    /// samples the pin and hands the level down via `set_sdspi_cs` (called
    /// from the SPI mmio arm on every USR transfer). Defaults to selected
    /// so unit tests (which never drive a pin) work unwired.
    sdspi_cs_low: bool,
}

impl Spi {
    pub fn new(idx: u32) -> Self {
        Self {
            regs: [0; REG_COUNT],
            idx,
            txn: None,
            last_tx: None,
            pending_miso: None,
            dma_tx: Vec::new(),
            dma_rx: Vec::new(),
            quad_fake: None,
            sdspi: SdspiCard::new(),
            sdspi_cs_low: true,
        }
    }

    /// Latch the SS-pin level for the next `complete()` (SoC-side; see the
    /// `sdspi_cs_low` field). True = SS LOW = selected. Defaults to selected
    /// (unit tests never drive a pin); real firmware selects per-transfer.
    pub fn set_sdspi_cs(&mut self, low: bool) {
        self.sdspi_cs_low = low;
    }

    /// Test hook: force the card selected regardless of the latched SS
    /// level (unit tests drive the card without GPIO traffic).
    #[cfg(test)]
    pub fn sdspi_test_select(&mut self) {
        self.sdspi_cs_low = true;
    }

    /// Attach the SPI-mode SD card with `blocks` 512-byte zeroed blocks.
    pub fn sdspi_attach(&mut self, blocks: usize) {
        self.sdspi.attach(blocks);
    }

    /// Attach the SPI-mode SD card with a preformatted image (shares the
    /// SDMMC FAT16 bytes so `SD.begin` mounts a real filesystem).
    pub fn sdspi_attach_image(&mut self, image: &[u8]) {
        self.sdspi.attach_image(image);
    }

    /// True once the SDSPI card is attached (for harness asserts).
    pub fn sdspi_attached(&self) -> bool {
        self.sdspi.attached()
    }

    /// CS-gate the SDSPI card (true = SS LOW/selected). Unit-test `xfer`
    /// helpers call this once (selected for the whole sequence); real
    /// firmware drives it per-transfer from the SS pin level (see
    /// `Spi::complete`, which samples the pin each transaction).
    pub fn sdspi_select(&mut self, sel: bool) {
        self.sdspi.select(sel);
    }

    /// Inject MISO bytes for the next transfer (host virtual device response).
    /// One-shot: consumed by the NEXT transfer with a MISO window, then
    /// cleared — even when an SDSPI card is attached (the card wins those;
    /// see the priority note in `complete`). A stale injection must never
    /// shadow live card traffic across transfers.
    pub fn inject_miso(&mut self, bytes: &[u8]) {
        self.pending_miso = Some(bytes.to_vec());
    }

    /// Quad/dual wire-mode level from SPI_CTRL (spi_reg.h FREAD/FCMD/
    /// FADDR_QUAD/DUAL): 0 = single, 1 = dual, 2 = quad (quad wins when
    /// both set). The USR engine still serializes single-line; this only
    /// feeds the host fake-device hook (`quad_fake_*`) so firmware that
    /// programs wide modes observes the matching device behavior.
    pub fn quad_mode(&self) -> u32 {
        let ctrl = self.regs[(SPI_CTRL / 4) as usize];
        if ctrl & (CTRL_FREAD_QUAD | CTRL_FCMD_QUAD | CTRL_FADDR_QUAD) != 0 {
            2
        } else if ctrl & (CTRL_FREAD_DUAL | CTRL_FCMD_DUAL | CTRL_FADDR_DUAL) != 0 {
            1
        } else {
            0
        }
    }

    /// Host fake quad-SPI device: a fixed 256-byte pattern store the host
    /// pre-loads (`quad_fake_provision`) and the firmware reads through
    /// USR MISO transfers. Reads wrap the store (offset modulo length);
    /// writes (MOSI) update it in place so a write/read-back round-trips.
    /// `None` until provisioned — MISO then stays zeros (no device).
    /// Separate from `pending_miso` (one-shot injection): this is the
    /// persistent wide-mode device behind the quad hook.
    pub fn quad_fake_provision(&mut self, pattern: &[u8]) {
        self.quad_fake = Some(pattern.to_vec());
    }

    /// Serve a MISO data phase from the fake quad device (called at
    /// transaction completion when `quad_mode() != 0` and no one-shot
    /// `pending_miso` is staged). `addr` selects the start offset so
    /// address-phase reads land at the right pattern window.
    fn quad_fake_read(&self, addr: u32, nbytes: usize) -> Option<Vec<u8>> {
        let store = self.quad_fake.as_ref()?;
        if store.is_empty() || nbytes == 0 {
            return None;
        }
        let mut out = Vec::with_capacity(nbytes);
        for i in 0..nbytes {
            out.push(store[((addr as usize) + i) % store.len()]);
        }
        Some(out)
    }

    /// Commit a MOSI data phase into the fake quad device (write path of
    /// the round-trip): bytes land at `addr` modulo the store length.
    fn quad_fake_write(&mut self, addr: u32, bytes: &[u8]) {
        if let Some(store) = self.quad_fake.as_mut() {
            if store.is_empty() {
                return;
            }
            for (i, &b) in bytes.iter().enumerate() {
                let k = ((addr as usize) + i) % store.len();
                store[k] = b;
            }
        }
    }

    /// True when the controller is in slave mode (SPI_SLAVE.slave_mode).
    pub fn is_slave(&self) -> bool {
        self.regs[(SPI_SLAVE / 4) as usize] & SLAVE_MODE != 0
    }

    /// Slave DMA receive enabled (slave_mode + DMA_CONF.dma_rx_ena): a
    /// host-driven master-write lands in the GDMA IN-link DRAM buffers
    /// (walked by the SoC) and completes with SLV_WR_DMA_DONE, not the
    /// CPU data buffer + trans_done.
    pub fn slave_dma_rx_enabled(&self) -> bool {
        self.is_slave() && self.regs[(SPI_DMA_CONF / 4) as usize] & DMA_RX_ENA != 0
    }

    /// Slave DMA transmit enabled (slave_mode + DMA_CONF.dma_tx_ena): a
    /// host-driven master-read sources the GDMA OUT-link DRAM buffers and
    /// completes with SLV_RD_DMA_DONE.
    pub fn slave_dma_tx_enabled(&self) -> bool {
        self.is_slave() && self.regs[(SPI_DMA_CONF / 4) as usize] & DMA_TX_ENA != 0
    }

    /// Record a DMA-backed slave exchange length + completion: SLAVE1
    /// data_bitlen in bits plus the WR (master-write) or RD (master-read)
    /// DMA-done latch. The SoC moves the bytes through the GDMA links.
    pub fn slave_dma_done(&mut self, bits: u32, is_write: bool) {
        let bitlen = bits.min(SLAVE1_DATA_BITLEN_MASK + 1);
        let s1 = &mut self.regs[(SPI_SLAVE1 / 4) as usize];
        *s1 = (*s1 & !SLAVE1_DATA_BITLEN_MASK) | (bitlen & SLAVE1_DATA_BITLEN_MASK);
        self.regs[(SPI_INT_RAW / 4) as usize] |= if is_write {
            INT_WR_DMA_DONE
        } else {
            INT_RD_DMA_DONE
        };
    }

    /// Host-driven master-write-to-slave: capture `bytes` into the data
    /// buffer in LE word order (stream byte `i` is word byte `i % 4`,
    /// matching `spi_ll_write_buffer`'s memcpy — what real slave firmware
    /// reads back), as if an external master clocked them in; record the
    /// transfer length in SLAVE1.data_bitlen, and latch trans_done. Only
    /// acts in slave mode.
    pub fn slave_inject_write(&mut self, bytes: &[u8]) {
        if !self.is_slave() {
            return;
        }
        let mut buf = [0u32; DATA_WORDS];
        for (i, &byte) in bytes.iter().enumerate() {
            let w = i / 4;
            if w < DATA_WORDS {
                buf[w] |= (byte as u32) << (8 * (i % 4));
            }
        }
        self.regs[SPI_DATA_BUF as usize / 4..SPI_DATA_BUF as usize / 4 + DATA_WORDS]
            .copy_from_slice(&buf);
        let bitlen = ((bytes.len() * 8) as u32).min(SLAVE1_DATA_BITLEN_MASK + 1);
        let s1 = &mut self.regs[(SPI_SLAVE1 / 4) as usize];
        *s1 = (*s1 & !SLAVE1_DATA_BITLEN_MASK) | (bitlen & SLAVE1_DATA_BITLEN_MASK);
        self.regs[(SPI_INT_RAW / 4) as usize] |= INT_TRANS_DONE;
    }

    /// Host-driven master-read-from-slave: return the first `nbytes` of the
    /// firmware-preloaded data buffer (LE word order, mirrors the pack),
    /// record the transfer length in SLAVE1.data_bitlen, and latch
    /// trans_done. Only acts in slave mode.
    pub fn slave_take_read(&mut self, nbytes: usize) -> Vec<u8> {
        if !self.is_slave() {
            return Vec::new();
        }
        let base = SPI_DATA_BUF as usize / 4;
        let mut out = Vec::with_capacity(nbytes);
        for b in 0..nbytes {
            let w = b / 4;
            let v = if w < DATA_WORDS {
                ((self.regs[base + w] >> (8 * (b % 4))) & 0xFF) as u8
            } else {
                0
            };
            out.push(v);
        }
        let bitlen = ((nbytes * 8) as u32).min(SLAVE1_DATA_BITLEN_MASK + 1);
        let s1 = &mut self.regs[(SPI_SLAVE1 / 4) as usize];
        *s1 = (*s1 & !SLAVE1_DATA_BITLEN_MASK) | (bitlen & SLAVE1_DATA_BITLEN_MASK);
        self.regs[(SPI_INT_RAW / 4) as usize] |= INT_TRANS_DONE;
        out
    }

    /// Take the MOSI byte stream of the last completed transfer (host side).
    pub fn take_last_tx(&mut self) -> Option<Vec<u8>> {
        self.last_tx.take()
    }

    /// Advance `cycles` APB cycles; finishes the transaction when the bit
    /// stream is complete and writes MISO results back into the buffer.
    pub fn tick(&mut self, cycles: u64) {
        let Some(t) = self.txn.as_mut() else {
            return;
        };
        let run = self.regs[(SPI_CLK_GATE / 4) as usize] & CLK_GATE_CLK_EN != 0;
        if !run {
            return;
        }
        for _ in 0..cycles {
            if t.remain == 0 {
                self.complete();
                return;
            }
            t.remain -= 1;
            if t.remain == 0 {
                // The last SPI clock cycle of the transaction just
                // finished: sample MISO and clear CMD.usr.
                self.complete();
                return;
            }
        }
    }

    /// Finish the transaction: sample MISO (no device -> zeros, one-shot
    /// host injection, the fake quad-device store when a wide mode is
    /// programmed, or the SDSPI card when attached) into the data buffer
    /// in LE word order (stream byte `i` is word byte `i % 4`, matching
    /// `spi_ll_read_buffer`'s memcpy — what the real HAL reads back),
    /// latch trans_done, and clear CMD.usr (self-clearing, TRM SPI_CMD.usr).
    /// Captures the MOSI bytes for the host event queue. DMA-backed
    /// transfers capture MISO into `dma_rx` (served to the GDMA `in` walk)
    /// instead of the data buffer.
    fn complete(&mut self) {
        // CS-gate the SDSPI card from the SS level latched by the SoC (see
        // `sdspi_cs_low`): with SS HIGH the card samples nothing and keeps
        // prior state. Defaults to selected (unit tests never drive a pin).
        let cs_low = self.sdspi_cs_low;
        self.sdspi.select(cs_low);
        let have_mosi = self.txn.as_ref().is_some_and(|t| t.have_mosi);
        let have_miso = self.txn.as_ref().is_some_and(|t| t.have_miso);
        let dma = self.txn.as_ref().is_some_and(|t| t.dma);
        let rd_lsb = self.txn.as_ref().is_some_and(|t| t.rd_lsb);
        let data_bits = self.txn.as_ref().map_or(0, |t| t.data_bits);
        let doutdin = self.txn.as_ref().is_some_and(|t| t.doutdin);
        // MISO window: with doutdin the phases overlap (one shared `d`-bit
        // window per spi_struct.h ms_dlen); sequential half-duplex
        // (mosi+miso WITHOUT doutdin) runs MOSI `d` bits first, then MISO
        // `d` bits.
        let miso_bits = data_bits;
        let addr_value = self.txn.as_ref().map_or(0, |t| t.addr_value);
        let quad = self.quad_mode();
        let mosi_bytes = if have_mosi {
            self.txn.as_ref().map(Self::collect_mosi)
        } else {
            None
        };
        // SDSPI card: full-duplex data phases (the Arduino SD stack polls
        // byte-wise with doutdin SET: the SAME clocks carry MOSI out and
        // MISO in) feed each MOSI byte through the card and take its answer
        // as this transfer's MISO (synchronous — finishes before any host
        // round-trip). Sequential half-duplex (doutdin clear) runs MOSI `d`
        // bits first, then MISO `d` bits instead. Route-then-pop: the
        // frame's last transfer returns its own R1, exactly like silicon
        // (see `SdspiCard::feed`).
        // MISO-only phases (no MOSI) clock 0xFF (pull-up idle) instead.
        // MOSI-ONLY SHORTCUT (the CMD0 case): the Arduino SD driver sends
        // 6-byte CMD frames via MOSI-only `writeBytes` (usr_mosi set,
        // usr_miso CLEAR — see `spiWriteNL`: only mosi_dlen programmed),
        // then polls the R1 with SEPARATE full-duplex `transfer(0xFF)`
        // calls. The frame bytes MUST reach the card's collector here —
        // the per-byte `feed` path above never runs (have_miso false).
        // Full-duplex transfers route their bytes through `feed`, so they
        // must NOT also land here (double-decode).
        let sdspi_on = self.sdspi.attached() && !dma;
        let mut sdspi_rx: Vec<u8> = Vec::new();
        if sdspi_on && have_miso {
            let nbytes = miso_bits.div_ceil(8) as usize;
            if doutdin {
                if let Some(ref mosi) = mosi_bytes {
                    for &b in mosi.iter().take(nbytes) {
                        sdspi_rx.push(self.sdspi.feed(b));
                    }
                } else {
                    for _ in 0..nbytes {
                        sdspi_rx.push(self.sdspi.feed(0xFF));
                    }
                }
            } else if have_mosi {
                // Sequential: MOSI window feeds the command collector; the
                // MISO window clocks idle (the response stages for the NEXT
                // transfer, like silicon).
                if let Some(ref mosi) = mosi_bytes {
                    self.sdspi.collect_cmd_bytes(mosi);
                }
                for _ in 0..nbytes {
                    sdspi_rx.push(self.sdspi.feed(0xFF));
                }
            } else {
                for _ in 0..nbytes {
                    sdspi_rx.push(self.sdspi.feed(0xFF));
                }
            }
        }
        // MOSI-only transfers (MISO not staged: `spi->writeBytes`) still
        // carry command frames — decoded ABOVE via `collect_cmd_bytes`
        // (the per-byte `feed` never runs on this path, and full-duplex
        // transfers must NOT also land there — double-decode).
        if sdspi_on
            && have_mosi
            && !have_miso
            && let Some(ref mosi) = mosi_bytes
        {
            self.sdspi.collect_cmd_bytes(mosi);
        }
        if dma {
            // MISO byte stream over the data phase (zeros unless injected).
            let nbytes = miso_bits.div_ceil(8) as usize;
            let mut rx = vec![0u8; nbytes];
            if let Some(miso) = self.pending_miso.take() {
                for (i, b) in miso.iter().enumerate().take(nbytes) {
                    rx[i] = *b;
                }
            } else if quad != 0
                && let Some(fake) = self.quad_fake_read(addr_value, nbytes)
            {
                rx = fake;
            }
            self.dma_rx = rx;
        } else if have_miso {
            let mut buf = [0u32; DATA_WORDS];
            // Priority: SDSPI card feed first when a card is attached,
            // then one-shot host injection (virtual-device path), then the
            // quad fake store.
            //
            // WHY card-first: the card is a LIVE synchronous responder —
            // every transfer's MISO clocks MUST come from `feed` or the
            // card's command collector desyncs (bytes accumulate, R1 never
            // stages — found live: all-0xFF with staged=Some because a stale
            // `pending_miso` shadowed the card and its bytes were never
            // fed). Injection is a STALE one-shot: once consumed it must not
            // shadow later transfers. When both are present the card wins;
            // harnesses that need injection use a bus with no card attached
            // (tempdev/virtual-demo), so no conflict arises in practice.
            let staged = if !sdspi_rx.is_empty() {
                // Card bytes present: consume any stale injection alongside
                // (it must not survive to shadow a later transfer).
                self.pending_miso.take();
                Some(core::mem::take(&mut sdspi_rx))
            } else if self.pending_miso.is_some() {
                self.pending_miso.take()
            } else {
                (quad != 0).then(|| {
                    let nbytes = miso_bits.div_ceil(8) as usize;
                    self.quad_fake_read(addr_value, nbytes).unwrap_or_default()
                })
            };
            if let Some(miso) = &staged {
                // Pack the MISO bytes little-endian within the word
                // (matching `spi_ll_read_buffer`'s memcpy: W0's LOW byte is
                // stream byte 0 — what the Arduino byte path reads back via
                // `data_buf[0] & 0xFF`), honoring CTRL.rd_bit_order
                // (LSB-first reverses each byte).
                for (i, &byte) in miso.iter().enumerate() {
                    let w = i / 4;
                    if w < DATA_WORDS {
                        let b = if rd_lsb { byte.reverse_bits() } else { byte };
                        buf[w] |= (b as u32) << (8 * (i % 4));
                    }
                }
            }
            self.regs[SPI_DATA_BUF as usize / 4..SPI_DATA_BUF as usize / 4 + DATA_WORDS]
                .copy_from_slice(&buf);
        }
        if have_mosi {
            if let Some(mosi) = mosi_bytes {
                // Fake quad-device write path: MOSI data phases commit into
                // the provisioned store so a write/read-back round-trips.
                if quad != 0 && self.quad_fake.is_some() {
                    self.quad_fake_write(addr_value, &mosi);
                }
                self.last_tx = Some(mosi);
            }
        } else {
            self.last_tx = Some(Vec::new());
        }
        self.regs[(SPI_INT_RAW / 4) as usize] |= INT_TRANS_DONE;
        self.regs[(SPI_CMD / 4) as usize] &= !CMD_USR;
        self.txn = None;
    }

    /// Extract the MOSI byte stream from a finished transaction's data
    /// buffer: LE word order (stream byte `b` is word byte `b % 4`,
    /// matching `spi_ll_write_buffer`'s memcpy — the Arduino HAL writes
    /// `data_buf[0]` as a plain LOW byte and word paths pre-byte-swap with
    /// MSB_16/32_SET). Bit order (CTRL.wr_bit_order) selects MSB-first
    /// (default, MSBFIRST) vs LSB-first within each byte.
    /// The host reads this on an `EVT_SPI_XFER` event.
    fn collect_mosi(t: &Txn) -> Vec<u8> {
        // LE word order: stream byte `b` is word byte `b % 4` (NOT
        // `3 - (b % 4)` — that BE indexing mirrored multi-byte streams and
        // dropped the Arduino byte path's LOW byte; verified against
        // `spi_ll_write_buffer`'s LE-word memcpy).
        let bits = if t.have_mosi { t.data_bits } else { 0 };
        let nbytes = bits.div_ceil(8);
        let nwords = t.buf.len();
        let mut out = Vec::with_capacity(nbytes as usize);
        for b in 0..nbytes {
            let w = (b / 4) as usize;
            let byte = if w < nwords {
                ((t.buf[w] >> (8 * (b % 4))) & 0xFF) as u8
            } else {
                0
            };
            out.push(if t.wr_lsb { byte.reverse_bits() } else { byte });
        }
        out
    }

    /// Trigger a transfer if CMD.usr was set; snapshots all phase config.
    /// In slave mode CMD.usr does not start a master transaction (slave
    /// transfers are host-driven via slave_inject_write/slave_take_read).
    fn maybe_trigger(&mut self) {
        let cmd = self.regs[(SPI_CMD / 4) as usize];
        if cmd & CMD_USR == 0 || self.txn.is_some() || self.is_slave() {
            return;
        }
        let plan = self.plan();
        let total_bits = plan.cmd_bits + plan.addr_bits + plan.dummy_cycles + plan.data_bits;
        if total_bits == 0 {
            // Nothing to shift out: the transfer ends immediately.
            self.regs[(SPI_CMD / 4) as usize] &= !CMD_USR;
            return;
        }
        let buf =
            self.regs[SPI_DATA_BUF as usize / 4..SPI_DATA_BUF as usize / 4 + DATA_WORDS].to_vec();
        // Sequential half-duplex (mosi+miso WITHOUT doutdin): the MISO
        // window follows the MOSI window (total 2*`d` data bits). doutdin
        // overlaps them (total `d`). DMA triggers always overlap.
        let total_data = if plan.have_mosi && plan.have_miso && !plan.doutdin {
            plan.data_bits * 2
        } else {
            plan.data_bits
        };
        let total_bits = plan.cmd_bits + plan.addr_bits + plan.dummy_cycles + total_data;
        self.txn = Some(Txn {
            bit_cycles: plan.bit_cycles,
            low_ticks: plan.low_ticks,
            total_bits,
            cmd_bits: plan.cmd_bits,
            addr_bits: plan.addr_bits,
            data_bits: plan.data_bits,
            have_mosi: plan.have_mosi,
            have_miso: plan.have_miso,
            doutdin: plan.doutdin,
            dma: false,
            ck_pol: plan.ck_pol,
            d_pol: plan.d_pol,
            wr_lsb: plan.wr_lsb,
            rd_lsb: plan.rd_lsb,
            cmd_value: plan.cmd_value,
            addr_value: plan.addr_value,
            buf,
            remain: total_bits * plan.bit_cycles,
        });
    }

    /// Snapshot the phase/timing config from USER/USER1/USER2/MS_DLEN/
    /// CLOCK/MISC/CTRL.
    fn plan(&self) -> TxnPlan {
        let user = self.regs[(SPI_USER / 4) as usize];
        let user1 = self.regs[(SPI_USER1 / 4) as usize];
        let user2 = self.regs[(SPI_USER2 / 4) as usize];
        let ms_dlen = self.regs[(SPI_MS_DLEN / 4) as usize];
        let clock = self.regs[(SPI_CLOCK / 4) as usize];
        let misc = self.regs[(SPI_MISC / 4) as usize];
        let ctrl = self.regs[(SPI_CTRL / 4) as usize];

        let cmd_bits = if user & USER_USR_COMMAND != 0 {
            (user2 >> USER2_CMD_BITLEN_SHIFT) as u64 + 1
        } else {
            0
        };
        let addr_bits = if user & USER_USR_ADDR != 0 {
            ((user1 >> USER1_ADDR_BITLEN_SHIFT) & 0x1F) as u64 + 1
        } else {
            0
        };
        let dummy_cycles = if user & USER_USR_DUMMY != 0 {
            ((user1 >> USER1_DUMMY_CYCLELEN_SHIFT) & 0xFF) as u64 + 1
        } else {
            0
        };
        let have_mosi = user & USER_USR_MOSI != 0;
        let have_miso = user & USER_USR_MISO != 0;
        let doutdin = user & USER_DOUTDIN != 0;
        let data_bits = if have_mosi || have_miso {
            (ms_dlen & MS_DLEN_DATA_BITLEN_MASK) as u64 + 1
        } else {
            0
        };
        // SPI clock period = (clkcnt_n+1) APB cycles, high for
        // (clkcnt_h+1) of them (TRM SPI_CLOCK).
        let n = ((clock >> CLOCK_CLKCNT_N_SHIFT) & 0x3F) as u64;
        let h = ((clock >> 6) & 0x3F) as u64;
        let low_cycles = (n + 1) - (h + 1).min(n + 1);
        let pre_plus1 = ((clock >> CLOCK_CLKDIV_PRE_SHIFT) & 0xF) as u64 + 1;
        let bit_cycles = if clock & CLOCK_EQU_SYSCLK != 0 {
            1
        } else {
            (pre_plus1 * (n + 1)).max(1)
        };
        let low_ticks = low_cycles * pre_plus1;
        TxnPlan {
            bit_cycles,
            low_ticks,
            cmd_bits,
            addr_bits,
            dummy_cycles,
            data_bits,
            have_mosi,
            have_miso,
            doutdin,
            ck_pol: (misc >> 29) & 1,
            d_pol: (ctrl >> 20) & 1,
            wr_lsb: ctrl & CTRL_WR_BIT_ORDER != 0,
            rd_lsb: ctrl & CTRL_RD_BIT_ORDER != 0,
            cmd_value: (user2 >> USER2_CMD_VALUE_SHIFT) & 0xFFFF,
            addr_value: self.regs[(SPI_ADDR / 4) as usize],
        }
    }

    /// Append GDMA-fed bytes to the DMA TX staging (called by the GDMA
    /// `out` walk before `dma_trigger`).
    pub fn spi_dma_feed(&mut self, bytes: &[u8]) {
        self.dma_tx.extend_from_slice(bytes);
    }

    /// Start a DMA-backed master transfer from the staged GDMA bytes. The
    /// MOSI bits come from `dma_tx` (not data_buf); MISO lands in `dma_rx`
    /// for the GDMA `in` walk. `data_bits` follows MS_DLEN as usual, or the
    /// staged length when that is larger (an unset MS_DLEN means 1 bit).
    pub fn dma_trigger(&mut self) {
        if self.txn.is_some() || self.is_slave() {
            return;
        }
        let staged = core::mem::take(&mut self.dma_tx);
        let plan = self.plan();
        let fed_bits = staged.len() as u64 * 8;
        let data_bits = plan.data_bits.max(fed_bits);
        let total_bits = plan.cmd_bits + plan.addr_bits + plan.dummy_cycles + data_bits;
        if total_bits == 0 {
            return;
        }
        // Pack staged bytes little-endian within the word (LE word order,
        // same layout as data_buf).
        let nwords = data_bits.div_ceil(32) as usize;
        let mut buf = vec![0u32; nwords];
        for (i, &byte) in staged.iter().enumerate() {
            let w = i / 4;
            if w < nwords {
                buf[w] |= (byte as u32) << (8 * (i % 4));
            }
        }
        self.txn = Some(Txn {
            bit_cycles: plan.bit_cycles,
            low_ticks: plan.low_ticks,
            total_bits,
            cmd_bits: plan.cmd_bits,
            addr_bits: plan.addr_bits,
            data_bits,
            have_mosi: plan.have_mosi,
            have_miso: plan.have_miso,
            doutdin: true,
            dma: true,
            ck_pol: plan.ck_pol,
            d_pol: plan.d_pol,
            wr_lsb: plan.wr_lsb,
            rd_lsb: plan.rd_lsb,
            cmd_value: plan.cmd_value,
            addr_value: plan.addr_value,
            buf,
            remain: total_bits * plan.bit_cycles,
        });
    }

    /// Little-endian word of the last DMA transfer's captured RX bytes at
    /// byte offset `off` (zeros beyond the capture; served to the GDMA `in`
    /// walk word by word).
    pub fn dma_rx_word(&self, off: u32) -> u32 {
        let o = off as usize;
        let b = |i: usize| {
            if i < self.dma_rx.len() {
                self.dma_rx[i]
            } else {
                0
            }
        };
        u32::from_le_bytes([b(o), b(o + 1), b(o + 2), b(o + 3)])
    }

    fn txn(&self) -> Option<&Txn> {
        self.txn.as_ref()
    }

    /// Level of stream `bit` currently shifting out on the MOSI line
    /// during the data phase. The data buffer holds LE words (stream byte
    /// `i` is word byte `i % 4`, matching `spi_ll_write_buffer`'s memcpy);
    /// within each byte the MSB-first default shifts bit 7 first (an
    /// external sampler sees the HAL's intended stream order).
    /// `CTRL.wr_bit_order` (Arduino LSBFIRST) reverses within
    /// each byte: wire bit = word bit `(wbit ^ 7)`.
    fn data_bit(&self, t: &Txn, bit: u64) -> u32 {
        if bit >= t.data_bits {
            return 0;
        }
        // LE words: byte `i` = word byte `i % 4`; within the byte,
        // MSB-first default shifts bit 7 first.
        let byte = (bit / 8) as usize;
        let w = byte / 4;
        if w >= t.buf.len() {
            return 0;
        }
        let word = t.buf[w];
        let bbit = 7 - (bit % 8);
        let wbit = ((byte % 4) as u64 * 8) + bbit;
        let wbit = if t.wr_lsb { wbit ^ 7 } else { wbit };
        (word >> wbit) & 1
    }

    /// MOSI line level at `elapsed` APB cycles into the transaction.
    /// Sequential half-duplex (mosi+miso WITHOUT doutdin) runs the MOSI
    /// window first, then the MISO window (which drives 0 — the controller
    /// tristates MISO while sampling); doutdin overlaps them.
    fn mosi_level(&self, t: &Txn, elapsed: u64) -> u32 {
        let slot = elapsed / t.bit_cycles;
        if slot >= t.total_bits {
            return t.d_pol;
        }
        if slot < t.cmd_bits {
            let b = (t.cmd_bits - 1 - slot) as u32;
            return (t.cmd_value >> b) & 1;
        }
        let rest = slot - t.cmd_bits;
        if rest < t.addr_bits {
            let b = (t.addr_bits - 1 - rest) as u32;
            return (t.addr_value >> b) & 1;
        }
        let rest = rest - t.addr_bits;
        if !t.have_mosi {
            return 0;
        }
        self.data_bit(t, rest)
    }

    /// SPI clock level at `elapsed` APB cycles into the transaction.
    fn clock_level(&self, t: &Txn, elapsed: u64) -> u32 {
        if elapsed >= t.total_bits * t.bit_cycles {
            return t.ck_pol;
        }
        // One SPI clock cycle is bit_cycles APB ticks; the (clkcnt_n+1)
        // counter runs on the pre-divided clock, so it stays LOW for
        // low_ticks = low_cycles * (clkdiv_pre+1) ticks of each cycle.
        // (With clkdiv_pre=0 this reduces to the old phase computation.)
        let phase = elapsed % t.bit_cycles;
        // Mode 0/2: idle low, clock low for low_ticks then high for the
        // rest of each cycle; mode 1/3 (ck_idle_edge=1) inverted.
        t.ck_pol ^ u32::from(phase >= t.low_ticks)
    }

    /// CS line level at `elapsed` APB cycles (active low during transfer).
    fn cs_level(&self, t: &Txn, elapsed: u64, dis: u32) -> u32 {
        if dis != 0 {
            return 1;
        }
        if elapsed < t.total_bits * t.bit_cycles {
            0
        } else {
            1
        }
    }

    /// Current output level of GPIO-matrix signal `sig`, 0 if not ours.
    /// Signal ranges (S3 gpio_sig_map.h): GPSPI2 = FSPI 101..105 + CS
    /// 110/111; GPSPI3 = 66..72. NOTE: this is the controller's OUTPUT
    /// signal bundle (what the controller drives onto pads). The FSPIQ/Q
    /// input path (what the controller samples on MISO) is NOT modeled
    /// here — MISO bytes arrive via `inject_miso` / the SDSPI card feed in
    /// `complete()`, exactly like the I2C model's `pending_rx` injection.
    pub fn signal_level(&self, sig: u32) -> u32 {
        let base = if self.idx == 0 { 101 } else { 66 };
        let clk_sig = base;
        let d_sig = base + 2;
        let cs0_sig = if self.idx == 0 { 110 } else { 71 };
        let cs1_sig = if self.idx == 0 { 111 } else { 72 };
        let Some(t) = self.txn() else {
            // Idle: clock at CPOL, MOSI at d_pol, CS high.
            let ctrl = self.regs[(SPI_CTRL / 4) as usize];
            let misc = self.regs[(SPI_MISC / 4) as usize];
            let ck_pol = (misc & MISC_CK_IDLE_EDGE) >> 29;
            let d_pol = (ctrl & CTRL_D_POL) >> 20;
            return match sig {
                s if s == clk_sig => ck_pol,
                s if s == d_sig => d_pol,
                s if s == cs0_sig => 1,
                s if s == cs1_sig => 1,
                _ => 0,
            };
        };
        let elapsed = t.total_bits * t.bit_cycles - t.remain;
        match sig {
            s if s == clk_sig => self.clock_level(t, elapsed),
            s if s == d_sig => self.mosi_level(t, elapsed),
            s if s == cs0_sig => {
                let dis = self.regs[(SPI_MISC / 4) as usize] & MISC_CS0_DIS;
                self.cs_level(t, elapsed, dis)
            }
            s if s == cs1_sig => {
                let dis = (self.regs[(SPI_MISC / 4) as usize] & MISC_CS1_DIS) >> 1;
                self.cs_level(t, elapsed, dis)
            }
            _ => 0,
        }
    }

    /// Interrupt status: INT_RAW & INT_ENA (TRM SPI_SLV_INT_STATUS). The
    /// driver ISR reads this to identify the cause before clearing INT_CLR.
    /// NOTE: `trans_done` is a pure latch (set on completion and by the
    /// INT_SET register at `spi_hal_init`; cleared by INT_CLR / at the next
    /// transaction start). It must NOT read back as set merely when idle:
    /// an idle-always-set overlay storms the ISR (proven: interrupt-WDT
    /// panic) because the ISR's own CLR is immediately re-asserted.
    pub fn int_st(&self) -> u32 {
        let raw = self.regs[(SPI_INT_RAW / 4) as usize];
        let ena = self.regs[(SPI_INT_ENA / 4) as usize];
        raw & ena
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        if offset >= (REG_COUNT * 4) as u32 {
            return 0;
        }
        match offset {
            SPI_INT_RAW => self.regs[(SPI_INT_RAW / 4) as usize],
            SPI_INT_ENA => self.regs[(SPI_INT_ENA / 4) as usize],
            SPI_INT_ST => self.int_st(),
            SPI_INT_CLR => 0,
            _ => self.regs[(offset / 4) as usize],
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        if offset.is_multiple_of(4) && offset < (REG_COUNT * 4) as u32 {
            match offset {
                SPI_INT_CLR => {
                    // Clearing the status clears the matching RAW bits.
                    self.regs[(SPI_INT_RAW / 4) as usize] &= !value;
                }
                SPI_INT_SET => {
                    // Software-set: ORs into RAW (TRM SPI_DMA_INT_SET).
                    self.regs[(SPI_INT_RAW / 4) as usize] |= value;
                    self.regs[(SPI_INT_SET / 4) as usize] = value;
                }
                _ => {
                    self.regs[(offset / 4) as usize] = value;
                    if offset == SPI_CMD {
                        // UPDATE (bit 23) is self-clearing: it latches the APB
                        // register image into the SPI module clock domain, then
                        // the hardware clears it. The driver busy-waits on it.
                        if value & CMD_UPDATE != 0 {
                            self.regs[(SPI_CMD / 4) as usize] &= !CMD_UPDATE;
                        }
                        self.maybe_trigger();
                    }
                }
            }
        }
    }
}

impl Default for Spi {
    fn default() -> Self {
        Self::new(0)
    }
}
