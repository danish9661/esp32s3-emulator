//! ESP32-S3 ULP-RISC-V coprocessor (functional core, P5).
//!
//! The ULP-RISC-V is a small `rv32im` core that runs firmware from
//! `RTC_SLOW_MEM` (`0x5000_0000`). The control block
//! (`DR_REG_ULP_RISCV_BASE = 0x6000_8100`) holds the `core` start/stall bits,
//! the `debug` halted flag, and 16 general-purpose `reg` slots
//! (`0x6000_810C..0x6000_814C`) used to exchange results with the main CPU.
//!
//! We model a real (if minimal) `rv32im` interpreter: the main CPU writes a
//! RISC-V program into `RTC_SLOW_MEM`, then writes the `core` `sw_start` bit
//! (bit 0 of `0x6000_8100`) to release the ULP core; it runs one instruction
//! per `Soc` tick, executes the program, and halts on `ebreak` (which sets the
//! `debug` halted flag). Stores to the ULP `reg` slots land in the shared `regs`
//! array so the main CPU can poll them.
//!
//! **Known limitation:** the compressed (C) extension and RV32F/D are not
//! modeled, and the ULP can only reach `RTC_SLOW_MEM` + the ULP `reg` slots
//! (other RTC peripheral accesses are ignored). Validation is via the
//! `esp32s3_ulp` direct-poke sketch (hand-assembled rv32im program).

use crate::memmap::RTC_SLOW_BASE;
use crate::memmap::RTC_SLOW_SIZE;

pub const ULP_BASE: u32 = 0x6000_8100;

// Region of the 0x6000_8000 page carved out for ULP (off 0x100..0x200).
pub const ULP_OFF_START: u32 = 0x100;
pub const ULP_OFF_END: u32 = 0x200;

// `core` register (off 0x00 in the block): sw_start (bit 0), sw_stall (bit 1).
const CORE_OFF: u32 = 0x00;
// `debug` register (off 0x08): halted flag (bit 0).
const DEBUG_OFF: u32 = 0x08;
// First shared `reg` slot (off 0x0C in the block).
const REG_SLOT_OFF: u32 = 0x0C;

const REG_COUNT: usize = (ULP_OFF_END - ULP_OFF_START) as usize / 4;

/// The ULP-RISC-V core: register file + control, plus the surrounding register
/// store (control/status/reg slots).
pub struct Ulp {
    regs: [u32; REG_COUNT],
    x: [u32; 32],
    pc: u32,
    running: bool,
    halted: bool,
}

impl Default for Ulp {
    fn default() -> Self {
        Self::new()
    }
}

impl Ulp {
    pub fn new() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
            x: [0u32; 32],
            pc: RTC_SLOW_BASE,
            running: false,
            halted: false,
        }
    }

    /// `offset` is the full peripheral address (ULP_BASE .. ULP_BASE+0x100).
    fn idx(&self, offset: u32) -> usize {
        ((offset - ULP_BASE) / 4) as usize
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        let i = self.idx(offset);
        if i < REG_COUNT { self.regs[i] } else { 0 }
    }

    pub fn write32(&mut self, offset: u32, value: u32) {
        let i = self.idx(offset);
        if i < REG_COUNT {
            self.regs[i] = value;
        }
        // `core` register: bit 0 = sw_start releases the ULP core.
        if offset == ULP_BASE + CORE_OFF {
            if value & 1 != 0 {
                self.running = true;
                self.halted = false;
                self.pc = RTC_SLOW_BASE;
                self.x = [0u32; 32];
            }
            if value & 2 != 0 {
                self.running = false;
            }
        }
    }

    /// True if the ULP core is currently executing (not halted, started).
    pub fn is_running(&self) -> bool {
        self.running && !self.halted
    }

    /// Run one ULP instruction. `rtc_slow` is the ULP code/data memory.
    pub fn step(&mut self, rtc_slow: &mut [u8]) {
        if !self.is_running() {
            return;
        }
        let pc = self.pc;
        if !(RTC_SLOW_BASE..RTC_SLOW_BASE + RTC_SLOW_SIZE).contains(&pc) {
            // PC out of range → halt.
            self.halted = true;
            self.regs[self.idx(ULP_BASE + DEBUG_OFF)] |= 1;
            return;
        }
        let insn = read32(rtc_slow, pc - RTC_SLOW_BASE);
        let next = self.exec(insn, rtc_slow);
        if self.halted {
            self.regs[self.idx(ULP_BASE + DEBUG_OFF)] |= 1;
            return;
        }
        self.pc = next;
    }

    fn exec(&mut self, insn: u32, mem: &mut [u8]) -> u32 {
        let opcode = insn & 0x7F;
        let rd = ((insn >> 7) & 0x1F) as usize;
        let funct3 = (insn >> 12) & 0x7;
        let rs1 = ((insn >> 15) & 0x1F) as usize;
        let rs2 = ((insn >> 20) & 0x1F) as usize;
        let funct7 = (insn >> 25) & 0x7F;
        let pc = self.pc;
        let mut next_pc = pc.wrapping_add(4);

        let imm_i = ((insn as i32) >> 20) as u32; // sign-extended
        let imm_s = ((insn >> 25) << 5) | ((insn >> 7) & 0x1F);
        let imm_s_se = (((imm_s as i32) << 20) >> 20) as u32;
        let imm_u = insn & 0xFFFF_F000;
        let imm_j = (((insn >> 31) & 1) << 20)
            | (((insn >> 21) & 0x3FF) << 1)
            | (((insn >> 20) & 1) << 11)
            | ((insn >> 12) & 0xFF) << 12;
        let imm_j_se = (((imm_j as i32) << 11) >> 11) as u32;
        let imm_b = (((insn >> 31) & 1) << 12)
            | (((insn >> 7) & 1) << 11)
            | (((insn >> 25) & 0x3F) << 5)
            | (((insn >> 8) & 0xF) << 1);
        let imm_b_se = (((imm_b as i32) << 19) >> 19) as u32;

        match opcode {
            0x37 => {
                // LUI
                self.x[rd] = imm_u;
            }
            0x17 => {
                // AUIPC
                self.x[rd] = pc.wrapping_add(imm_u);
            }
            0x6F => {
                // JAL
                self.x[rd] = pc.wrapping_add(4);
                next_pc = pc.wrapping_add(imm_j_se);
            }
            0x67 => {
                // JALR
                let t = pc.wrapping_add(4);
                next_pc = (self.x[rs1].wrapping_add(imm_i)) & !3;
                self.x[rd] = t;
            }
            0x63 => {
                // BRANCH
                let a = self.x[rs1];
                let b = self.x[rs2];
                let take = match funct3 {
                    0 => a == b,
                    1 => a != b,
                    4 => (a as i32) < (b as i32),
                    5 => (a as i32) >= (b as i32),
                    6 => a < b,
                    7 => a >= b,
                    _ => false,
                };
                if take {
                    next_pc = pc.wrapping_add(imm_b_se);
                }
            }
            0x03 => {
                // LOAD
                let addr = self.x[rs1].wrapping_add(imm_i);
                let v = self.load(mem, addr, funct3);
                self.x[rd] = v;
            }
            0x23 => {
                // STORE
                let addr = self.x[rs1].wrapping_add(imm_s_se);
                let v = self.x[rs2];
                self.store(mem, addr, v, funct3);
            }
            0x13 => {
                // OP-IMM
                let a = self.x[rs1];
                let v = match funct3 {
                    0 => a.wrapping_add(imm_i),
                    2 => if (a as i32) < (imm_i as i32) { 1 } else { 0 },
                    3 => if a < imm_i { 1 } else { 0 },
                    4 => a ^ imm_i,
                    6 => a | imm_i,
                    7 => a & imm_i,
                    1 => a << (imm_i & 0x1F),
                    5 => {
                        if funct7 & 0x20 != 0 {
                            ((a as i32) >> (imm_i & 0x1F)) as u32
                        } else {
                            a >> (imm_i & 0x1F)
                        }
                    }
                    _ => 0,
                };
                self.x[rd] = v;
            }
            0x33 => {
                // OP (register)
                let a = self.x[rs1];
                let b = self.x[rs2];
                let v = match (funct7, funct3) {
                    (0x00, 0) => a.wrapping_add(b),
                    (0x20, 0) => a.wrapping_sub(b),
                    (0x00, 1) => a << (b & 0x1F),
                    (0x00, 2) => if (a as i32) < (b as i32) { 1 } else { 0 },
                    (0x00, 3) => if a < b { 1 } else { 0 },
                    (0x00, 4) => a ^ b,
                    (0x00, 5) => a >> (b & 0x1F),
                    (0x20, 5) => ((a as i32) >> (b & 0x1F)) as u32,
                    (0x00, 6) => a | b,
                    (0x00, 7) => a & b,
                    (0x01, 0) => a.wrapping_mul(b),
                    (0x01, 1) => (((a as i64) * (b as i64)) >> 32) as u32,
                    (0x01, 2) => {
                        let sa = (a as i32) as i64;
                        let sb = b as i64;
                        ((sa * sb) >> 32) as u32
                    }
                    (0x01, 3) => (((a as u64) * (b as u64)) >> 32) as u32,
                    (0x01, 4) => {
                        if b == 0 {
                            a
                        } else if (a as i32) >= 0 {
                            a / b
                        } else {
                            a.wrapping_sub(b).wrapping_add(1) / b
                        }
                    }
                    (0x01, 5) => a.checked_div(b).unwrap_or(a),
                    (0x01, 6) => {
                        if b == 0 {
                            a
                        } else if (a as i32) >= 0 {
                            a % b
                        } else {
                            a.wrapping_sub((a / b).wrapping_mul(b))
                        }
                    }
                    (0x01, 7) => a.checked_rem(b).unwrap_or(a),
                    _ => 0,
                };
                self.x[rd] = v;
            }
            0x0F => {
                // FENCE (treat as nop)
            }
            0x73 => {
                // SYSTEM: ECALL (imm=0) nop, EBREAK (imm=1) halt.
                let imm = (insn >> 20) & 0xFFF;
                if imm == 1 {
                    self.halted = true;
                    next_pc = pc; // stay put
                }
            }
            _ => {
                // Illegal opcode → halt to avoid running garbage.
                self.halted = true;
                next_pc = pc;
            }
        }
        next_pc
    }

    /// Load from ULP-visible memory: the `reg` slots (ULP_BASE+0x0C..) or
    /// `RTC_SLOW_MEM`. Other addresses read as 0.
    fn load(&self, mem: &[u8], addr: u32, funct3: u32) -> u32 {
        if (ULP_BASE + REG_SLOT_OFF..ULP_BASE + ULP_OFF_END).contains(&addr) {
            let i = self.idx(addr);
            let v = if i < REG_COUNT { self.regs[i] } else { 0 };
            match funct3 {
                0 => (v as i8) as i32 as u32,
                1 => (v as i16) as i32 as u32,
                2 => v,
                4 => (v as u8) as u32,
                5 => (v as u16) as u32,
                _ => v,
            }
        } else if (RTC_SLOW_BASE..RTC_SLOW_BASE + RTC_SLOW_SIZE).contains(&addr) {
            read32(mem, addr - RTC_SLOW_BASE)
        } else {
            0
        }
    }

    /// Store to ULP-visible memory (reg slots or RTC_SLOW_MEM).
    fn store(&mut self, mem: &mut [u8], addr: u32, v: u32, funct3: u32) {
        if (ULP_BASE + REG_SLOT_OFF..ULP_BASE + ULP_OFF_END).contains(&addr) {
            let i = self.idx(addr);
            if i < REG_COUNT {
                let cur = self.regs[i];
                self.regs[i] = match funct3 {
                    0 => (v & 0xFF) | (cur & 0xFFFF_FF00),
                    1 => (v & 0xFFFF) | (cur & 0xFFFF_0000),
                    2 => v,
                    _ => v,
                };
            }
        } else if (RTC_SLOW_BASE..RTC_SLOW_BASE + RTC_SLOW_SIZE).contains(&addr) {
            let off = (addr - RTC_SLOW_BASE) as usize;
            mem[off..off + 4].copy_from_slice(&v.to_le_bytes());
        }
    }
}

fn read32(mem: &[u8], off: u32) -> u32 {
    let o = off as usize;
    u32::from_le_bytes([mem[o], mem[o + 1], mem[o + 2], mem[o + 3]])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand-assembled rv32im program: store 0x12345678 to ULP reg slot 0, then
    /// `ebreak`. Words (LE): lui/addi to build the reg-slot address, lui/addi
    /// to build the value, sw, ebreak.
    const PROG: [u32; 6] = [
        0x6000_80B7, // lui x1, 0x60008        -> 0x60008000
        0x10C0_8093, // addi x1, x1, 0x10C     -> 0x6000810C (reg slot 0)
        0x1234_5137, // lui x2, 0x12345        -> 0x12345000
        0x6781_0113, // addi x2, x2, 0x678     -> 0x12345678
        0x0020_A023, // sw x2, 0(x1)
        0x0010_0073, // ebreak
    ];

    fn reg_slot0_idx() -> usize {
        ((ULP_BASE + REG_SLOT_OFF) - ULP_BASE) as usize / 4
    }

    #[test]
    fn ulp_runs_program_and_writes_reg_slot() {
        let mut ulp = Ulp::new();
        let mut mem = [0u8; RTC_SLOW_SIZE as usize];
        for (i, w) in PROG.iter().enumerate() {
            mem[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        // Release the core.
        ulp.write32(ULP_BASE, 1);
        assert!(ulp.is_running());
        // Run until halted.
        for _ in 0..100 {
            ulp.step(&mut mem[..]);
            if !ulp.is_running() {
                break;
            }
        }
        assert!(!ulp.is_running());
        assert_eq!(ulp.regs[reg_slot0_idx()], 0x1234_5678);
    }

    #[test]
    fn ulp_halts_on_ebreak_sets_debug_flag() {
        let mut ulp = Ulp::new();
        let mut mem = [0u8; RTC_SLOW_SIZE as usize];
        mem[0..4].copy_from_slice(&0x0010_0073u32.to_le_bytes()); // ebreak
        ulp.write32(ULP_BASE, 1);
        ulp.step(&mut mem[..]);
        assert!(!ulp.is_running());
        // debug register (off 0x08 -> idx 2) halted bit set.
        assert_eq!(ulp.regs[2] & 1, 1);
    }

    #[test]
    fn ulp_register_store_round_trips() {
        let mut ulp = Ulp::new();
        ulp.write32(ULP_BASE + 0x50, 0xCAFE_BEEF);
        assert_eq!(ulp.read32(ULP_BASE + 0x50), 0xCAFE_BEEF);
    }
}

