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
//! Interrupts: transaction completion latches INT_RAW.trans_done (bit 0,
//! TRM SPI_SLV_INT_RAW); INT_STATUS = INT_RAW & INT_ENA. The matrix + CPU
//! delivery is wired in soc.rs int_pending.  Not modeled: DMA slave mode,
//! quad/octal, segments, the CMD.update latch (values are used as written
//! — functionally equivalent once firmware follows the IDF update sequence).
//! MISO input has no device attached, so RX phases read back zeros.

// Register offsets (TRM GPSPI chapter).
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
// CTRL bits (TRM SPI_CTRL): idle MOSI polarity (d_pol).
const CTRL_D_POL: u32 = 1 << 20;
// CLK_GATE bits (TRM SPI_CLK_GATE).
const CLK_GATE_CLK_EN: u32 = 1 << 0;
// Interrupt registers (TRM SPI_SLV_INT_*): trans_done = bit 0.
pub const SPI_INT_ENA: u32 = 0x34;
pub const SPI_INT_CLR: u32 = 0x38;
pub const SPI_INT_RAW: u32 = 0x3C;
pub const SPI_INT_ST: u32 = 0x40;
const INT_TRANS_DONE: u32 = 1 << 0;

const REG_COUNT: usize = 0xF4 / 4;
const DATA_WORDS: usize = 16;

/// Snapshot of a running USR transaction.
struct Txn {
    /// APB cycles per SPI clock bit.
    bit_cycles: u64,
    /// SPI clock period (clkcnt_n+1) and low cycles per period
    /// (clkcnt_n - clkcnt_h), TRM SPI_CLOCK.
    period: u64,
    low_cycles: u64,
    /// Total bit slots: command + address + dummy + data.
    total_bits: u64,
    cmd_bits: u64,
    addr_bits: u64,
    /// Data phase width in bits (MOSI+MISO count when !doutdin).
    data_bits: u64,
    have_mosi: bool,
    have_miso: bool,
    /// Idle clock level (CPOL).
    ck_pol: u32,
    /// Idle MOSI level (CTRL.d_pol).
    d_pol: u32,
    cmd_value: u32,
    addr_value: u32,
    /// Data buffer snapshot (left-aligned MSB-first).
    buf: [u32; DATA_WORDS],
    /// APB cycles remaining.
    remain: u64,
}

/// One general-purpose SPI controller (GPSPI2 = idx 0, GPSPI3 = idx 1).
pub struct Spi {
    regs: [u32; REG_COUNT],
    idx: u32,
    txn: Option<Txn>,
}

impl Spi {
    pub fn new(idx: u32) -> Self {
        Self {
            regs: [0; REG_COUNT],
            idx,
            txn: None,
        }
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

    /// Finish the transaction: sample MISO (no device -> zeros) into the
    /// data buffer, latch trans_done, and clear CMD.usr (self-clearing,
    /// TRM SPI_CMD.usr).
    fn complete(&mut self) {
        if self.txn.as_ref().filter(|t| t.have_miso).is_some() {
            let zeros = [0u32; DATA_WORDS];
            self.regs[SPI_DATA_BUF as usize / 4..SPI_DATA_BUF as usize / 4 + DATA_WORDS]
                .copy_from_slice(&zeros);
        }
        self.regs[(SPI_INT_RAW / 4) as usize] |= INT_TRANS_DONE;
        self.regs[(SPI_CMD / 4) as usize] &= !CMD_USR;
        self.txn = None;
    }

    /// Trigger a transfer if CMD.usr was set; snapshots all phase config.
    fn maybe_trigger(&mut self) {
        let cmd = self.regs[(SPI_CMD / 4) as usize];
        if cmd & CMD_USR == 0 || self.txn.is_some() {
            return;
        }
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
        let bit_cycles = if clock & CLOCK_EQU_SYSCLK != 0 {
            1
        } else {
            let pre = ((clock >> CLOCK_CLKDIV_PRE_SHIFT) & 0xF) as u64 + 1;
            (pre * (n + 1)).max(1)
        };
        let total_bits = cmd_bits + addr_bits + dummy_cycles + data_bits;
        if total_bits == 0 {
            // Nothing to shift out: the transfer ends immediately.
            self.regs[(SPI_CMD / 4) as usize] &= !CMD_USR;
            return;
        }
        let mut buf = [0u32; DATA_WORDS];
        buf.copy_from_slice(
            &self.regs[SPI_DATA_BUF as usize / 4..SPI_DATA_BUF as usize / 4 + DATA_WORDS],
        );
        self.txn = Some(Txn {
            bit_cycles,
            period: n + 1,
            low_cycles,
            total_bits,
            cmd_bits,
            addr_bits,
            data_bits,
            have_mosi,
            have_miso,
            ck_pol: (misc >> 29) & 1,
            d_pol: (ctrl >> 20) & 1,
            cmd_value: (user2 >> USER2_CMD_VALUE_SHIFT) & 0xFFFF,
            addr_value: self.regs[(SPI_ADDR / 4) as usize],
            buf,
            remain: total_bits * bit_cycles,
        });
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
        if idx >= DATA_WORDS {
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
        let phase = (elapsed % t.bit_cycles) % t.period;
        // Mode 0/2: idle low, clock low for low_cycles then high for the
        // rest of each period; mode 1/3 (ck_idle_edge=1) inverted.
        t.ck_pol ^ u32::from(phase >= t.low_cycles)
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
