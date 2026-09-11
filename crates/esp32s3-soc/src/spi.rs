//! ESP32-S3 GPSPI2/GPSPI3 (general-purpose SPI) master model.
//!

//! Register layout per the S3 TRM GPSPI chapter / spi_struct.h (GPSPI2 at
//! 0x6002_4000, GPSPI3 at 0x6002_5000; 0x6002_8000 is the SD/MMC host, a
//! separate peripheral — see sdmmc.rs).
//!
//! Modeled: CPU-controlled master USR transactions (no DMA).  Writing
//! CMD.usr (bit 24) starts a transfer whose phases are enabled by USER:
//! command (USER2.usr_command_value/bitlen), address (ADDR +
//! USER1.usr_addr_bitlen), dummy (USER1.usr_dummy_cyclelen) and data
//! (MS_DLEN.ms_data_bitlen bits, half-duplex MOSI or MISO, or both with
//! USER.doutdin).  The SPI clock period is
//! (clkdiv_pre+1)*(clkcnt_n+1) APB cycles (1 if CLOCK.clk_equ_sysclk),
//! TRM: spi_clk = system/(clkdiv_pre+1)/(clkcnt_n+1).  CMD.usr stays set
//! for the whole transaction and self-clears when it ends; the data buffer
//! (data_buf[16] @ 0x98) is a left-aligned MSB-first shift register.
//! The module clock gate (CLK_GATE.clk_en) must be set for the clock to
//! run, exactly like real hardware (IDF spi_ll_enable_clock).
//!
//! Interrupts: transaction completion latches INT_RAW.trans_done (bit 12,
//! TRM SPI_DMA_INT_RAW; the classic-ESP32 bit-0 layout does NOT apply);
//! delivery is wired in soc.rs int_pending.  Not modeled: DMA slave mode,
//! quad/octal, segments, the CMD.update latch (values are used as written
//! — functionally equivalent once firmware follows the IDF update sequence).
//! MISO input has no device attached, so RX phases read back zeros.
//!
//! Slave mode: when SPI_SLAVE.slave_mode (bit 26) is set, CMD.usr no longer
//! starts a master transaction. The external master does not exist in the
//! emulator, so the host drives slave exchanges synchronously at the buffer
//! level (the exact bytes-and-interrupt contract the firmware observes):
//! `slave_inject_write` emulates a master-write-to-slave (captures the bytes
//! into data_buf, records SLAVE1.data_bitlen, raises trans_done),
//! `slave_take_read` emulates a master-read-from-slave (returns the
//! firmware-preloaded data_buf bytes, records the bitlen, raises
//! trans_done). CPU-controlled (Rd_BUF/Wr_BUF) semantics; slave DMA and the
//! live slave waveform on Q are not modeled.

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
// USER bits (TRM SPI_USER).
const USER_USR_COMMAND: u32 = 1 << 31;
const USER_USR_ADDR: u32 = 1 << 30;
const USER_USR_DUMMY: u32 = 1 << 29;
const USER_USR_MISO: u32 = 1 << 28;
const USER_USR_MOSI: u32 = 1 << 27;
const USER_DOUTDIN: u32 = 1 << 0;
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
// CTRL bits (TRM SPI_CTRL): idle MOSI polarity (d_pol).
const CTRL_D_POL: u32 = 1 << 20;
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
    /// Data phase width in bits (MOSI+MISO count when !doutdin).
    data_bits: u64,
    have_mosi: bool,
    have_miso: bool,
    /// DMA-backed (GDMA-fed) transfer: MOSI bits come from staged GDMA
    /// bytes, MISO lands in `dma_rx` instead of the data buffer.
    dma: bool,
    /// Idle clock level (CPOL).
    ck_pol: u32,
    /// Idle MOSI level (CTRL.d_pol).
    d_pol: u32,
    cmd_value: u32,
    addr_value: u32,
    /// Data buffer snapshot (left-aligned MSB-first); 16 words for USR
    /// transfers, sized to cover data_bits for DMA transfers.
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
    ck_pol: u32,
    d_pol: u32,
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
    /// Captured RX bytes of the last DMA-backed transfer (served to the
    /// GDMA `in` walk via `dma_rx_word`; overwritten per transfer, never
    /// drained, so the IN link may start before or after completion).
    dma_rx: Vec<u8>,
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
        }
    }

    /// Inject MISO bytes for the next transfer (host virtual device response).
    pub fn inject_miso(&mut self, bytes: &[u8]) {
        self.pending_miso = Some(bytes.to_vec());
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

    /// Host-driven master-write-to-slave: capture `bytes` (MSB-first) into
    /// the data buffer as if an external master clocked them in, record the
    /// transfer length in SLAVE1.data_bitlen, and latch trans_done. Only acts
    /// in slave mode.
    pub fn slave_inject_write(&mut self, bytes: &[u8]) {
        if !self.is_slave() {
            return;
        }
        let mut buf = [0u32; DATA_WORDS];
        let mut bitpos = 0u32;
        for &byte in bytes {
            for j in (0..8).rev() {
                let w = (bitpos / 32) as usize;
                if w < DATA_WORDS {
                    let wbit = 31 - (bitpos % 32);
                    buf[w] &= !(1u32 << wbit);
                    buf[w] |= (((byte >> j) & 1) as u32) << wbit;
                }
                bitpos += 1;
            }
        }
        self.regs[SPI_DATA_BUF as usize / 4..SPI_DATA_BUF as usize / 4 + DATA_WORDS]
            .copy_from_slice(&buf);
        let bitlen = bitpos.min(SLAVE1_DATA_BITLEN_MASK + 1);
        let s1 = &mut self.regs[(SPI_SLAVE1 / 4) as usize];
        *s1 = (*s1 & !SLAVE1_DATA_BITLEN_MASK) | (bitlen & SLAVE1_DATA_BITLEN_MASK);
        self.regs[(SPI_INT_RAW / 4) as usize] |= INT_TRANS_DONE;
    }

    /// Host-driven master-read-from-slave: return the first `nbytes` of the
    /// firmware-preloaded data buffer (MSB-first), record the transfer length
    /// in SLAVE1.data_bitlen, and latch trans_done. Only acts in slave mode.
    pub fn slave_take_read(&mut self, nbytes: usize) -> Vec<u8> {
        if !self.is_slave() {
            return Vec::new();
        }
        let base = SPI_DATA_BUF as usize / 4;
        let mut out = Vec::with_capacity(nbytes);
        for b in 0..nbytes {
            let mut byte = 0u8;
            for j in 0..8 {
                let bitidx = (b * 8 + j) as u32;
                let w = (bitidx / 32) as usize;
                let v = if w < DATA_WORDS {
                    (self.regs[base + w] >> (31 - (bitidx % 32))) & 1
                } else {
                    0
                };
                byte = (byte << 1) | v as u8;
            }
            out.push(byte);
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

    /// Finish the transaction: sample MISO (no device -> zeros, or the host
    /// injected bytes) into the data buffer, latch trans_done, and clear
    /// CMD.usr (self-clearing, TRM SPI_CMD.usr). Captures the MOSI bytes for
    /// the host event queue. DMA-backed transfers capture MISO into `dma_rx`
    /// (served to the GDMA `in` walk) instead of the data buffer.
    fn complete(&mut self) {
        let have_mosi = self.txn.as_ref().is_some_and(|t| t.have_mosi);
        let have_miso = self.txn.as_ref().is_some_and(|t| t.have_miso);
        let dma = self.txn.as_ref().is_some_and(|t| t.dma);
        let data_bits = self.txn.as_ref().map_or(0, |t| t.data_bits);
        if dma {
            // MISO byte stream over the data phase (zeros unless injected).
            let nbytes = data_bits.div_ceil(8) as usize;
            let mut rx = vec![0u8; nbytes];
            if let Some(miso) = self.pending_miso.take() {
                for (i, b) in miso.iter().enumerate().take(nbytes) {
                    rx[i] = *b;
                }
            }
            self.dma_rx = rx;
        } else if have_miso {
            let mut buf = [0u32; DATA_WORDS];
            if let Some(miso) = self.pending_miso.take() {
                // Shift the injected MISO bytes (MSB-first) into the
                // left-aligned data buffer.
                let mut bitpos = 0u32;
                for &byte in &miso {
                    for j in (0..8).rev() {
                        let w = (bitpos / 32) as usize;
                        let wbit = 31 - (bitpos % 32);
                        if w < DATA_WORDS {
                            buf[w] &= !(1u32 << wbit);
                            buf[w] |= (((byte >> j) & 1) as u32) << wbit;
                        }
                        bitpos += 1;
                    }
                }
            }
            self.regs[SPI_DATA_BUF as usize / 4..SPI_DATA_BUF as usize / 4 + DATA_WORDS]
                .copy_from_slice(&buf);
        }
        if have_mosi {
            if let Some(t) = self.txn.as_ref() {
                self.last_tx = Some(Self::collect_mosi(t));
            }
        } else {
            self.last_tx = Some(Vec::new());
        }
        self.regs[(SPI_INT_RAW / 4) as usize] |= INT_TRANS_DONE;
        self.regs[(SPI_CMD / 4) as usize] &= !CMD_USR;
        self.txn = None;
    }

    /// Extract the MOSI byte stream from a finished transaction's left-aligned
    /// data buffer (MSB-first). For full-duplex (doutdin) the MOSI portion is
    /// the first half of the data bits; for half-duplex MOSI it is the whole
    /// data phase. The host reads this on an `EVT_SPI_XFER` event.
    fn collect_mosi(t: &Txn) -> Vec<u8> {
        let bits = if t.have_mosi {
            if t.have_miso {
                t.data_bits / 2
            } else {
                t.data_bits
            }
        } else {
            0
        };
        let nbytes = bits.div_ceil(8);
        let nwords = t.buf.len();
        let mut out = Vec::with_capacity(nbytes as usize);
        for b in 0..nbytes {
            let mut byte = 0u8;
            for j in 0..8 {
                let bitidx = b * 8 + j;
                let w = (bitidx / 32) as usize;
                let wbit = 31 - (bitidx % 32);
                let v = if w < nwords {
                    (t.buf[w] >> wbit) & 1
                } else {
                    0
                };
                byte = (byte << 1) | v as u8;
            }
            out.push(byte);
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
        self.txn = Some(Txn {
            bit_cycles: plan.bit_cycles,
            low_ticks: plan.low_ticks,
            total_bits,
            cmd_bits: plan.cmd_bits,
            addr_bits: plan.addr_bits,
            data_bits: plan.data_bits,
            have_mosi: plan.have_mosi,
            have_miso: plan.have_miso,
            dma: false,
            ck_pol: plan.ck_pol,
            d_pol: plan.d_pol,
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
            let d = (ms_dlen & MS_DLEN_DATA_BITLEN_MASK) as u64 + 1;
            if doutdin {
                d
            } else if have_mosi && have_miso {
                2 * d
            } else {
                d
            }
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
            ck_pol: (misc >> 29) & 1,
            d_pol: (ctrl >> 20) & 1,
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
        // Pack staged bytes MSB-first, left-aligned (same layout as data_buf).
        let nwords = data_bits.div_ceil(32) as usize;
        let mut buf = vec![0u32; nwords];
        for (i, &byte) in staged.iter().enumerate() {
            let w = i / 4;
            if w < nwords {
                buf[w] |= (byte as u32) << (24 - 8 * (i % 4));
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
            dma: true,
            ck_pol: plan.ck_pol,
            d_pol: plan.d_pol,
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

    /// Level of the `bit` (bit index, MSB first) currently shifting out on
    /// the MOSI line during the data phase.
    fn data_bit(&self, t: &Txn, bit: u64) -> u32 {
        if bit >= t.data_bits {
            return 0;
        }
        let idx = (bit / 32) as usize;
        if idx >= t.buf.len() {
            return 0;
        }
        let w = t.buf[idx];
        // Left-aligned, MSB first: bit 0 of the stream is word bit 31.
        (w >> (31 - (bit % 32))) & 1
    }

    /// MOSI line level at `elapsed` APB cycles into the transaction.
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
    /// 110/111; GPSPI3 = 66..72.
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
