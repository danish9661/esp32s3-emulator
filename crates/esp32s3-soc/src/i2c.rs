//! ESP32-S3 I2C (I2CEXT0/I2CEXT1) master model.
//!
//! Register layout per the S3 TRM I2C chapter / i2c_struct.h (I2C0 at
//! 0x6001_3000, I2C1 at 0x6002_7000, 0x14000 apart).
//!
//! Modeled: the master command list.  The firmware writes up to 8 commands
//! into comd[0..8) (0x58 + 4i), then sets ctr.trans_start; the FSM runs
//! the list in index order, setting each slot's done bit (31) as it
//! finishes, halting at END (IDF i2c_ll_start_trans).  Command fields per
//! IDF i2c_ll_hw_cmd_t: byte_num [7:0], ack_en [8], ack_exp [9],
//! ack_val [10], op_code [13:11]; op codes: 1 = WRITE, 2 = STOP, 3 = READ,
//! 4 = END, 6 = RSTART (NOT the ESP32-classic 0..4 values — the S3
//! restructured the register; verified against i2c_ll.h used by the real
//! driver).
//!
//! SCL/SDA waveform per TRM timing registers, in I2C module clocks where
//! module = APB / (clk_conf.sclk_div_num + 1): SCL low lasts
//! (scl_low_period + 1) module clocks, high lasts scl_high_period +
//! scl_wait_high_period (IDF measures the high half without the +1 the
//! TRM shows — i2c_ll_get_scl_timing sums the two fields); START holds
//! SDA low for (scl_start_hold + 1) before dropping SCL; STOP drives SDA
//! low for (scl_stop_hold + 1) then raises it for (scl_stop_setup + 1),
//! the observed SDA rising edge while SCL is high.  During a WRITE ACK
//! cycle and READ bits the slave drives SDA; with no device the pull-up
//! reads back 1 (NACK, latched in SR.resp_rec) and READ shifts in 1s.
//! END raises INT_RAW.trans_complete.
//!
//! FIFO: the `data` register (0x1C) is the FIFO port (write = TX push,
//! read = RX pop — i2c_ll_write_txfifo/read_rxfifo).  The nonfifo RAM
//! windows txfifo_mem (0x100) / rxfifo_mem (0x180) approximate the same
//! pushes/pops.
//!
//! Interrupts: END latches INT_RAW.trans_complete (bit 7); INT_STATUS =
//! INT_RAW & INT_ENA (TRM I2C_INT_STATUS) so the driver ISR can read the
//! cause. The matrix + CPU delivery is wired in soc.rs int_pending.

use alloc::collections::VecDeque;
use alloc::vec::Vec;

use crate::soc::{EVT_I2C_READ, EVT_I2C_START, EVT_I2C_STOP, EVT_I2C_WRITE, EmuEvent};

// Register offsets (TRM I2C chapter / i2c_struct.h member order).
pub const I2C_SCL_LOW_PERIOD: u32 = 0x00;
pub const I2C_CTR: u32 = 0x04;
pub const I2C_SR: u32 = 0x08;
pub const I2C_FIFO_CONF: u32 = 0x18;
pub const I2C_DATA: u32 = 0x1C;
pub const I2C_INT_RAW: u32 = 0x20;
pub const I2C_INT_CLR: u32 = 0x24;
pub const I2C_INT_ENA: u32 = 0x28;
pub const I2C_INT_ST: u32 = 0x2C;
pub const I2C_SDA_HOLD: u32 = 0x30;
pub const I2C_SDA_SAMPLE: u32 = 0x34;
pub const I2C_SCL_HIGH_PERIOD: u32 = 0x38;
pub const I2C_SCL_START_HOLD: u32 = 0x40;
pub const I2C_SCL_RSTART_SETUP: u32 = 0x44;
pub const I2C_SCL_STOP_HOLD: u32 = 0x48;
pub const I2C_SCL_STOP_SETUP: u32 = 0x4C;
pub const I2C_CLK_CONF: u32 = 0x54;
pub const I2C_COMD: u32 = 0x58;
pub const I2C_TXFIFO_MEM: u32 = 0x100;
pub const I2C_RXFIFO_MEM: u32 = 0x180;

// CTR bits (TRM I2C_CTR).
const CTR_TRANS_START: u32 = 1 << 5;
// SR bits (TRM I2C_SR).
const SR_RESP_REC: u32 = 1 << 0;
const SR_BUS_BUSY: u32 = 1 << 4;
const SR_RXFIFO_CNT_SHIFT: u32 = 8;
const SR_TXFIFO_CNT_SHIFT: u32 = 18;
// FIFO_CONF bits (TRM I2C_FIFO_CONF).
const FIFO_CONF_RX_FIFO_RST: u32 = 1 << 12;
const FIFO_CONF_TX_FIFO_RST: u32 = 1 << 13;
// CLK_CONF bits (TRM I2C_CLK_CONF).
const CLK_SCLK_DIV_NUM_SHIFT: u32 = 0;
// COMD fields (IDF i2c_ll_hw_cmd_t).
const COMD_BYTE_NUM_MASK: u32 = 0xFF;
const COMD_ACK_VAL_SHIFT: u32 = 10;
const COMD_OP_CODE_SHIFT: u32 = 11;
const COMD_OP_CODE_MASK: u32 = 0x7;
const COMD_DONE: u32 = 1 << 31;
// INT_RAW bits (TRM I2C_INT_RAW): end_detect + trans_complete + nack.
const INT_END_DETECT: u32 = 1 << 3;
const INT_TRANS_COMPLETE: u32 = 1 << 7;
const INT_NACK: u32 = 1 << 10;

// Master op codes (IDF i2c_ll.h I2C_LL_CMD_*).
const OP_RSTART: u32 = 6;
const OP_WRITE: u32 = 1;
const OP_READ: u32 = 3;
const OP_STOP: u32 = 2;
const OP_END: u32 = 4;

const REG_COUNT: usize = 0x200 / 4;
const FIFO_DEPTH: usize = 32;

/// A queued command: which comd slot produced it plus its value.
#[derive(Clone, Copy)]
struct Pending {
    slot: usize,
    value: u32,
}

/// A running master bus operation.  `phase` for WRITE/READ: 0..7 = data
/// bits, 8 = ACK low half, 9 = ACK high half; `scl` records the current
/// half (0 = low phase in progress, 1 = high).
struct Op {
    kind: u32,
    slot: usize,
    /// Bytes still to transfer (WRITE/READ).
    bytes_left: u32,
    /// Byte being shifted out (WRITE) or accumulated (READ).
    byte: u32,
    phase: u32,
    /// APB cycles remaining in the current phase.
    remain: u64,
    /// ACK level the master sends after a READ byte (comd.ack_val).
    ack: u32,
    /// Injected byte to shift in on a READ (from the host virtual device),
    /// `None` => bus reads back 1 (no device / 0xFF).
    shift_in: Option<u8>,
}

/// One I2C controller (I2C0 = idx 0, I2C1 = idx 1).
pub struct I2c {
    regs: [u32; REG_COUNT],
    idx: u32,
    txfifo: [u8; FIFO_DEPTH],
    tx_head: u32,
    tx_cnt: u32,
    rxfifo: [u8; FIFO_DEPTH],
    rx_head: u32,
    rx_cnt: u32,
    pending: [Pending; 8],
    pending_len: u32,
    op: Option<Op>,
    /// Current driven bus levels (idle/released = (1, 1)).
    scl: u32,
    sda: u32,
    /// Transmission cursor: bytes already clocked out in the current
    /// transaction. The TX FIFO is NOT drained on transmit (real HW keeps the
    /// bytes; the driver's NACK-retry path re-runs the FSM to re-send the same
    /// data), so `tx_head`/`tx_cnt` persist across re-runs and the retry
    /// re-NACKs instead of succeeding on an empty FIFO.
    tx_pos: u32,
    /// Host-observable event queue (START/WRITE/READ/STOP), drained per frame.
    events: Vec<EmuEvent>,
    /// Injected RX bytes for the next master-read (host virtual device supply).
    pending_rx: VecDeque<u8>,
}

impl I2c {
    pub fn new(idx: u32) -> Self {
        Self {
            regs: [0; REG_COUNT],
            idx,
            txfifo: [0; FIFO_DEPTH],
            tx_head: 0,
            tx_cnt: 0,
            rxfifo: [0; FIFO_DEPTH],
            rx_head: 0,
            rx_cnt: 0,
            pending: [Pending { slot: 0, value: 0 }; 8],
            pending_len: 0,
            op: None,
            scl: 1,
            sda: 1,
            tx_pos: 0,
            events: Vec::new(),
            pending_rx: VecDeque::new(),
        }
    }

    /// Module clock divisor (APB cycles per I2C module clock).
    fn div(&self) -> u64 {
        u64::from((self.regs[(I2C_CLK_CONF / 4) as usize] >> CLK_SCLK_DIV_NUM_SHIFT) & 0xFF) + 1
    }

    /// Inject RX bytes for the next master-read (host virtual device supply).
    pub fn inject_rx(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.pending_rx.push_back(b);
        }
    }

    /// SCL low half width in APB cycles: (value + 1) module clocks.
    fn low_len(&self) -> u64 {
        (u64::from(self.regs[(I2C_SCL_LOW_PERIOD / 4) as usize] & 0x1FF) + 1) * self.div()
    }

    /// SCL high half width in APB cycles: value + scl_wait_high_period
    /// module clocks (IDF i2c_ll_get_scl_timing convention).
    fn high_len(&self) -> u64 {
        let hi = (self.regs[(I2C_SCL_HIGH_PERIOD / 4) as usize] & 0x1FF) as u64;
        let wait = ((self.regs[(I2C_SCL_HIGH_PERIOD / 4) as usize] >> 9) & 0x7F) as u64;
        (hi + wait) * self.div()
    }

    fn start_hold_len(&self) -> u64 {
        (u64::from(self.regs[(I2C_SCL_START_HOLD / 4) as usize] & 0x1FF) + 1) * self.div()
    }

    fn stop_hold_len(&self) -> u64 {
        (u64::from(self.regs[(I2C_SCL_STOP_HOLD / 4) as usize] & 0x1FF) + 1) * self.div()
    }

    fn stop_setup_len(&self) -> u64 {
        (u64::from(self.regs[(I2C_SCL_STOP_SETUP / 4) as usize] & 0x1FF) + 1) * self.div()
    }

    fn tx_byte(&self) -> u32 {
        if self.tx_cnt == 0 {
            return 0xFF;
        }
        u32::from(self.txfifo[((self.tx_head + self.tx_pos) % FIFO_DEPTH as u32) as usize])
    }

    fn pop_rx(&mut self) -> u32 {
        if self.rx_cnt == 0 {
            return 0;
        }
        let b = u32::from(self.rxfifo[(self.rx_head % FIFO_DEPTH as u32) as usize]);
        self.rx_head += 1;
        self.rx_cnt -= 1;
        b
    }

    /// Begin executing command `value` (issued to comd `slot`), or finish
    /// it immediately when it performs no bus activity.
    fn run_command(&mut self, slot: usize, value: u32) {
        let kind = (value >> COMD_OP_CODE_SHIFT) & COMD_OP_CODE_MASK;
        let bytes = value & COMD_BYTE_NUM_MASK;
        match kind {
            OP_RSTART => {
                // SDA falls while SCL is high, held for start_hold.
                self.scl = 1;
                self.sda = 0;
                self.events.push(EmuEvent {
                    kind: EVT_I2C_START,
                    a: self.idx,
                    b: 0,
                });
                self.op = Some(Op {
                    kind,
                    slot,
                    bytes_left: 0,
                    byte: 0,
                    phase: 0,
                    remain: self.start_hold_len(),
                    ack: 0,
                    shift_in: None,
                });
            }
            OP_STOP => {
                // Drive SDA low while SCL high, then raise it: the stop
                // condition (SDA rising while SCL high).
                self.scl = 1;
                self.sda = 0;
                self.events.push(EmuEvent {
                    kind: EVT_I2C_STOP,
                    a: self.idx,
                    b: 0,
                });
                self.op = Some(Op {
                    kind,
                    slot,
                    bytes_left: 0,
                    byte: 0,
                    phase: 0,
                    remain: self.stop_hold_len(),
                    ack: 0,
                    shift_in: None,
                });
            }
            OP_WRITE => {
                if self.tx_cnt == 0 {
                    // The TX FIFO is empty. On real HW the driver's NACK
                    // retry re-runs the FSM still holding the byte in the
                    // hardware FIFO, so the byte is re-sent and re-NACKed;
                    // our model's retry sees an empty FIFO. Either way, with
                    // no slave present the bus stays high and the master
                    // receives a NACK, so run the NACK phases with a dummy
                    // byte rather than completing silently (which would let
                    // the retry report success with no device).
                    self.scl = 0;
                    self.sda = 1;
                    self.op = Some(Op {
                        kind,
                        slot,
                        bytes_left: 0,
                        byte: 0,
                        phase: 0,
                        remain: self.start_hold_len(),
                        ack: 0,
                        shift_in: None,
                    });
                    return;
                }
                let byte = self.tx_byte();
                self.scl = 0;
                self.sda = (byte >> 7) & 1;
                self.op = Some(Op {
                    kind,
                    slot,
                    bytes_left: bytes.min(self.tx_cnt),
                    byte,
                    phase: 0,
                    remain: self.low_len(),
                    ack: 0,
                    shift_in: None,
                });
            }
            OP_READ => {
                // Slave drives SDA (injected byte, else none: reads back 1).
                let shift_in = self.pending_rx.pop_front();
                self.scl = 0;
                self.sda = shift_in.map_or(1u32, |b| ((b >> 7) & 1) as u32);
                self.op = Some(Op {
                    kind,
                    slot,
                    bytes_left: bytes,
                    byte: 0,
                    phase: 0,
                    remain: self.low_len(),
                    ack: (value >> COMD_ACK_VAL_SHIFT) & 1,
                    shift_in,
                });
            }
            _ => {
                // END: no bus activity; latches end_detect (on END command)
                // and trans_complete (master finished STOP).  The esp-idf
                // master ISR waits on END_DETECT (I2C_LL_INTR_END_DETECT)
                // to signal transaction completion, so both must be raised.
                self.regs[(I2C_COMD / 4) as usize + slot] |= COMD_DONE;
                self.regs[(I2C_INT_RAW / 4) as usize] |= INT_END_DETECT | INT_TRANS_COMPLETE;
                self.op = None;
            }
        }
    }

    fn finish_op(&mut self, slot: usize) {
        self.regs[(I2C_COMD / 4) as usize + slot] |= COMD_DONE;
        if self.pending_len > 0 {
            let p = self.pending[0];
            for i in 1..self.pending_len as usize {
                self.pending[i - 1] = self.pending[i];
            }
            self.pending_len -= 1;
            self.run_command(p.slot, p.value);
            // A command that completes with no bus activity (empty WRITE,
            // END) leaves op == None; chain to the next queued command the
            // same way trans_start() does, otherwise pending_len stays > 0
            // and the next trans_start() bails forever.
            if self.op.is_none() {
                self.finish_op(p.slot);
            }
        } else {
            self.op = None;
        }
    }

    /// Advance to the next phase of the running operation.
    fn advance(&mut self) {
        let low = self.low_len();
        let high = self.high_len();
        let stop_setup = self.stop_setup_len();
        let Some(op) = self.op.as_mut() else {
            return;
        };
        let mut finished = false;
        match op.kind {
            OP_RSTART => {
                // Hold expired: drop SCL; the next command's first data
                // phase continues from SCL low.
                self.scl = 0;
                self.sda = 0;
                finished = true;
            }
            OP_STOP => {
                if self.sda == 0 {
                    // SDA driven low: raise it for the stop condition.
                    self.sda = 1;
                    op.remain = stop_setup;
                } else {
                    finished = true;
                }
            }
            OP_WRITE => {
                if self.scl == 0 {
                    // Low half done: clock high, SDA stays driven.
                    self.scl = 1;
                    op.remain = high;
                } else if op.phase < 7 {
                    // Data-bit high done: next bit, SDA transitions while
                    // SCL is low.
                    op.phase += 1;
                    self.scl = 0;
                    self.sda = (op.byte >> (7 - op.phase)) & 1;
                    op.remain = low;
                } else if op.phase == 7 {
                    // Last data bit done: ACK cycle, release SDA (no
                    // device -> NACK latched).
                    op.phase = 8;
                    self.scl = 0;
                    self.sda = 1;
                    op.remain = low;
                    self.regs[(I2C_SR / 4) as usize] |= SR_RESP_REC;
                    self.tx_pos += 1;
                    op.bytes_left = op.bytes_left.saturating_sub(1);
                    self.events.push(EmuEvent {
                        kind: EVT_I2C_WRITE,
                        a: self.idx,
                        b: op.byte,
                    });
                } else if op.phase == 8 {
                    // ACK low done: clock high, SDA released. With no slave
                    // present the bus pull-up holds SDA high -> NACK, which
                    // the driver detects via nack_int_raw.
                    op.phase = 9;
                    self.scl = 1;
                    self.sda = 1;
                    self.regs[(I2C_INT_RAW / 4) as usize] |= INT_NACK;
                    op.remain = high;
                } else {
                    // ACK high done: next byte or command end.
                    if op.bytes_left > 0 {
                        op.phase = 0;
                        op.byte = if self.tx_cnt == 0 {
                            0xFF
                        } else {
                            u32::from(
                                self.txfifo
                                    [((self.tx_head + self.tx_pos) % FIFO_DEPTH as u32) as usize],
                            )
                        };
                        self.scl = 0;
                        self.sda = (op.byte >> 7) & 1;
                        op.remain = low;
                    } else {
                        finished = true;
                    }
                }
            }
            OP_READ => {
                if self.scl == 0 {
                    // Low half done: just clock high; SDA is driven by the
                    // slave (or held by the master's ACK) and sampled during
                    // the high half. Do NOT change SDA here.
                    self.scl = 1;
                    op.remain = high;
                } else if op.phase < 7 {
                    // Sample the line during the high half; then set up the
                    // next bit's low half with the slave's value (or 1 if no
                    // device injected a byte).
                    op.byte = (op.byte << 1) | self.sda;
                    op.phase += 1;
                    self.scl = 0;
                    self.sda = op
                        .shift_in
                        .map_or(1u32, |b| ((b >> (7 - op.phase)) & 1) as u32);
                    op.remain = low;
                } else if op.phase == 7 {
                    // Last data bit sampled: master ACK cycle driving its
                    // ack level.
                    op.byte = (op.byte << 1) | self.sda;
                    if (self.rx_cnt as usize) < FIFO_DEPTH {
                        self.rxfifo[((self.rx_head + self.rx_cnt) % FIFO_DEPTH as u32) as usize] =
                            op.byte as u8;
                        self.rx_cnt += 1;
                    }
                    op.phase = 8;
                    self.scl = 0;
                    self.sda = op.ack;
                    op.remain = low;
                    op.bytes_left = op.bytes_left.saturating_sub(1);
                    self.events.push(EmuEvent {
                        kind: EVT_I2C_READ,
                        a: self.idx,
                        b: op.byte,
                    });
                } else if op.phase == 8 {
                    op.phase = 9;
                    self.scl = 1;
                    self.sda = op.ack;
                    op.remain = high;
                } else {
                    if op.bytes_left > 0 {
                        op.phase = 0;
                        op.byte = 0;
                        op.shift_in = self.pending_rx.pop_front();
                        self.scl = 0;
                        self.sda = op.shift_in.map_or(1u32, |b| ((b >> 7) & 1) as u32);
                        op.remain = low;
                    } else {
                        finished = true;
                    }
                }
            }
            _ => finished = true,
        }
        if finished {
            let slot = self.op.as_ref().map_or(0, |o| o.slot);
            self.op = None;
            self.finish_op(slot);
        }
    }

    /// Build the pending command queue from the comd slots, stopping at the
    /// first END, and start execution.
    ///
    /// The `done` bit (COMD_DONE, bit 31) is read-only status latched by the
    /// controller as each command completes; on a new `trans_start` the
    /// hardware clears it for every slot (TRM I2C: software re-issues the
    /// command list from slot 0 and polls done to detect completion).  The
    /// Arduino Wire driver reuses the same comd slots across transactions and
    /// relies on this auto-clear — if we instead *skip* done-set slots the
    /// queue is truncated after the first transfer and the FSM hangs.
    fn trans_start(&mut self) {
        if self.op.is_some() || self.pending_len > 0 {
            return;
        }
        self.tx_pos = 0;
        // Controller clears every command's done bit on (re)start.
        for i in 0..8 {
            self.regs[(I2C_COMD / 4) as usize + i] &= !COMD_DONE;
        }
        for i in 0..8 {
            let value = self.regs[(I2C_COMD / 4) as usize + i];
            let opc = (value >> COMD_OP_CODE_SHIFT) & COMD_OP_CODE_MASK;
            self.pending[self.pending_len as usize] = Pending {
                slot: i,
                value: value & !COMD_DONE,
            };
            self.pending_len += 1;
            if opc == OP_END {
                break;
            }
        }
        if self.pending_len > 0 {
            let p = self.pending[0];
            for i in 1..self.pending_len as usize {
                self.pending[i - 1] = self.pending[i];
            }
            self.pending_len -= 1;
            self.run_command(p.slot, p.value);
            if self.op.is_none() {
                // Command completed immediately (END / empty tx).
                self.finish_op(p.slot);
            }
        }
    }

    /// Advance `cycles` APB cycles through the bus operation, returning any
    /// host-observable events (START/WRITE/READ/STOP) generated this call.
    pub fn tick(&mut self, cycles: u64) -> Vec<EmuEvent> {
        for _ in 0..cycles {
            let Some(op) = self.op.as_mut() else {
                return core::mem::take(&mut self.events);
            };
            if op.remain > 0 {
                op.remain -= 1;
            }
            if op.remain == 0 {
                self.advance();
            }
        }
        core::mem::take(&mut self.events)
    }

    /// Current driven output levels: (SCL, SDA), idle = (1, 1).
    pub fn levels(&self) -> (u32, u32) {
        (self.scl, self.sda)
    }

    /// Output level of GPIO-matrix signal `sig` (I2CEXT0 SCL/SDA = 89/90,
    /// I2CEXT1 = 91/92, S3 gpio_sig_map.h).
    pub fn signal_level(&self, sig: u32) -> u32 {
        let (scl_sig, sda_sig) = if self.idx == 0 { (89, 90) } else { (91, 92) };
        if sig == scl_sig {
            self.scl
        } else if sig == sda_sig {
            self.sda
        } else {
            0
        }
    }

    /// Interrupt status: INT_RAW & INT_ENA (TRM I2C_INT_STATUS). The driver
    /// ISR reads this to identify the cause before clearing INT_CLR.
    pub fn int_st(&self) -> u32 {
        let raw = self.regs[(I2C_INT_RAW / 4) as usize];
        let ena = self.regs[(I2C_INT_ENA / 4) as usize];
        raw & ena
    }

    /// Raw interrupt bits (INT_RAW) regardless of enable.
    pub fn int_raw(&self) -> u32 {
        self.regs[(I2C_INT_RAW / 4) as usize]
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            I2C_SR => {
                // Live status: FIFO counts + bus busy (TRM I2C_SR).
                let mut sr = self.regs[(I2C_SR / 4) as usize];
                sr &= !((FIFO_DEPTH as u32 - 1) << SR_RXFIFO_CNT_SHIFT);
                sr |= self.rx_cnt.min(FIFO_DEPTH as u32 - 1) << SR_RXFIFO_CNT_SHIFT;
                sr &= !((FIFO_DEPTH as u32 - 1) << SR_TXFIFO_CNT_SHIFT);
                sr |= self.tx_cnt.min(FIFO_DEPTH as u32 - 1) << SR_TXFIFO_CNT_SHIFT;
                if self.op.is_some() {
                    sr |= SR_BUS_BUSY;
                } else {
                    sr &= !SR_BUS_BUSY;
                }
                sr
            }
            I2C_DATA | I2C_RXFIFO_MEM => self.pop_rx(),
            I2C_INT_RAW => self.regs[(I2C_INT_RAW / 4) as usize],
            I2C_INT_ENA => self.regs[(I2C_INT_ENA / 4) as usize],
            I2C_INT_ST => self.int_st(),
            _ if offset.is_multiple_of(4) && offset < (REG_COUNT * 4) as u32 => {
                self.regs[(offset / 4) as usize]
            }
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        if !offset.is_multiple_of(4) || offset >= (REG_COUNT * 4) as u32 {
            return;
        }
        match offset {
            I2C_CTR => {
                self.regs[(I2C_CTR / 4) as usize] = value;
                if value & CTR_TRANS_START != 0 {
                    self.trans_start();
                }
            }
            I2C_FIFO_CONF => {
                self.regs[(I2C_FIFO_CONF / 4) as usize] = value;
                // FIFO resets (TRM I2C_FIFO_CONF).
                if value & FIFO_CONF_TX_FIFO_RST != 0 {
                    self.tx_head = 0;
                    self.tx_cnt = 0;
                }
                if value & FIFO_CONF_RX_FIFO_RST != 0 {
                    self.rx_head = 0;
                    self.rx_cnt = 0;
                }
            }
            I2C_DATA | I2C_TXFIFO_MEM => {
                // FIFO port: byte pushes into the TX FIFO
                // (i2c_ll_write_txfifo).
                if (self.tx_cnt as usize) < FIFO_DEPTH {
                    self.txfifo[((self.tx_head + self.tx_cnt) % FIFO_DEPTH as u32) as usize] =
                        (value & 0xFF) as u8;
                    self.tx_cnt += 1;
                }
            }
            I2C_INT_CLR => {
                self.regs[(I2C_INT_RAW / 4) as usize] &= !value;
            }
            _ if (I2C_COMD..I2C_COMD + 8 * 4).contains(&offset) => {
                let slot = ((offset - I2C_COMD) / 4) as usize;
                self.regs[(I2C_COMD / 4) as usize + slot] = value & !COMD_DONE;
            }
            _ => {
                self.regs[(offset / 4) as usize] = value;
            }
        }
    }
}

impl Default for I2c {
    fn default() -> Self {
        Self::new(0)
    }
}
