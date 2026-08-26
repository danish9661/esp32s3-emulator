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
//! **Known limitation:** RV32F/D are not modeled, and the ULP can only reach
//! `RTC_SLOW_MEM` + the ULP `reg` slots (other RTC peripheral accesses are
//! ignored). The compressed (C) extension IS modeled. Validation is via the
//! `esp32s3_ulp` direct-poke sketch (hand-assembled rv32im/rv32imc program).

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
        let lo = read16(rtc_slow, pc - RTC_SLOW_BASE);
        let next = if (lo & 0x3) == 0x3 {
            // 32-bit instruction: read the high half too.
            let hi = read16(rtc_slow, pc - RTC_SLOW_BASE + 2);
            self.exec((lo as u32) | ((hi as u32) << 16), rtc_slow)
        } else {
            self.exec16(lo, rtc_slow)
        };
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

    /// Execute one 16-bit (compressed) instruction. Returns the next PC.
    fn exec16(&mut self, insn: u16, mem: &mut [u8]) -> u32 {
        let insn = insn as u32;
        let pc = self.pc;
        let mut next = pc.wrapping_add(2);
        let quad = insn & 0x3;
        let funct3 = (insn >> 13) & 0x7;
        let bit = |b: u32| (insn >> b) & 1;

        // 32-bit instructions never reach here (step reads 16 bits first).
        if quad == 0x3 {
            self.halted = true;
            return pc;
        }

        match quad {
            0 => match funct3 {
                0 => {
                    // C.ADDI4SPN: rd'=x8..x15 = x2 + (nzimm[9:2] << 2).
                    let nzimm = (bit(6)
                        | (bit(5) << 1)
                        | (bit(11) << 2)
                        | (bit(12) << 3)
                        | (bit(7) << 4)
                        | (bit(8) << 5)
                        | (bit(9) << 6)
                        | (bit(10) << 7))
                        & 0x3FF;
                    let rd = 8 + ((insn >> 10) & 0x7) as usize;
                    if nzimm != 0 {
                        self.x[rd] = self.x[2].wrapping_add(nzimm << 2);
                    }
                }
                2 => {
                    // C.LW: rd'=x8..x15 = mem[x1' + (uimm[6:2] << 2)].
                    let rd = 8 + ((insn >> 2) & 0x7) as usize;
                    let rs1 = 8 + ((insn >> 7) & 0x7) as usize;
                    let uimm = (bit(5) << 4)
                        | (bit(12) << 3)
                        | (bit(11) << 2)
                        | (bit(10) << 1)
                        | bit(6);
                    let addr = self.x[rs1].wrapping_add(uimm << 2);
                    self.x[rd] = self.load(mem, addr, 2);
                }
                6 => {
                    // C.SW: mem[x1' + (uimm[6:2] << 2)] = rs2'.
                    let rs2 = 8 + ((insn >> 2) & 0x7) as usize;
                    let rs1 = 8 + ((insn >> 7) & 0x7) as usize;
                    let uimm = (bit(5) << 4)
                        | (bit(12) << 3)
                        | (bit(11) << 2)
                        | (bit(10) << 1)
                        | bit(6);
                    let addr = self.x[rs1].wrapping_add(uimm << 2);
                    self.store(mem, addr, self.x[rs2], 2);
                }
                _ => {
                    self.halted = true;
                    return pc;
                }
            },
            1 => match funct3 {
                0 => {
                    // C.ADDI: rd = rd + sext6(imm).
                    let rd = ((insn >> 7) & 0x1F) as usize;
                    let imm = sext((bit(12) << 5) | ((insn >> 2) & 0x1F), 6);
                    self.x[rd] = self.x[rd].wrapping_add(imm);
                }
                1 => {
                    // C.JAL: ra = pc+2; jump.
                    self.x[1] = pc.wrapping_add(2);
                    next = pc.wrapping_add(c_j_imm(insn));
                }
                2 => {
                    // C.LI: rd = sext6(imm).
                    let rd = ((insn >> 7) & 0x1F) as usize;
                    let imm = sext((bit(12) << 5) | ((insn >> 2) & 0x1F), 6);
                    self.x[rd] = imm;
                }
                3 => {
                    // C.LUI (rd != 0,2) / C.ADDI16SP (rd == 2).
                    let rd = ((insn >> 7) & 0x1F) as usize;
                    if rd == 2 {
                        let nzimm = ((bit(12) << 5)
                            | (bit(4) << 4)
                            | (bit(3) << 3)
                            | (bit(5) << 2)
                            | (bit(2) << 1)
                            | bit(6))
                            & 0x3F;
                        let imm = sext(nzimm, 6) << 4;
                        self.x[2] = self.x[2].wrapping_add(imm);
                    } else if rd != 0 {
                        let imm = sext((bit(12) << 5) | ((insn >> 2) & 0x1F), 6);
                        self.x[rd] = imm << 12;
                    }
                }
                4 => {
                    // C.MISC-ALU (quad1, funct3=4). GAS (esp-rv32 toolchain) layout:
                    // bit12 is 0 for shifts/andi/alu; bit12=1 is reserved for
                    // C.ADD/C.MV/C.JR/C.JALR/C.EBREAK. funct2 = bits[11:10].
                    //   funct2=0 -> C.SRLI (bit12=0) / C.EBREAK (bit12=1, fields=0)
                    //   funct2=1 -> C.SRAI/C.MV/C.JR (bit12=0) or C.ADD/C.JALR (bit12=1)
                    //              C.MV/C.JR always have bits[6:5]==0 (rs2<8); C.SRAI
                    //              uses bits[6:2] as a 5-bit shamt, so bits[6:5]!=0.
                    //   funct2=2 -> C.ANDI
                    //   funct2=3 -> alu group SUB/XOR/OR/AND via bits[6:5]
                    let funct2 = (insn >> 10) & 0x3;
                    match funct2 {
                        0 => {
                            if bit(12) == 0 {
                                let rd = 8 + ((insn >> 7) & 0x7) as usize;
                                self.x[rd] >>= (insn >> 2) & 0x1F;
                            } else {
                                // C.EBREAK (rd/rs2 fields = 0)
                                self.halted = true;
                                return pc;
                            }
                        }
                        1 => {
                            let rs2f = (insn >> 2) & 0x7;
                            let rd = 8 + ((insn >> 7) & 0x7) as usize;
                            let b65 = (insn >> 5) & 0x3; // bits[6:5]
                            if bit(12) == 0 {
                                if rs2f == 0 {
                                    if b65 == 0 {
                                        // C.JR
                                        next = self.x[rd];
                                    } else {
                                        // C.SRAI: shamt in bits[6:2] (shamt >= 8).
                                        let sh = (insn >> 2) & 0x1F;
                                        self.x[rd] = ((self.x[rd] as i32) >> sh) as u32;
                                    }
                                } else if b65 == 0 {
                                    // C.SRAI: shamt in bits[6:2] (shamt < 8).
                                    let sh = (insn >> 2) & 0x1F;
                                    self.x[rd] = ((self.x[rd] as i32) >> sh) as u32;
                                } else {
                                    // C.MV: rd = rs2.
                                    self.x[rd] = self.x[8 + rs2f as usize];
                                }
                            } else if rs2f == 0 {
                                // C.JALR (or EBREAK if rd==0)
                                if rd == 0 {
                                    self.halted = true;
                                    return pc;
                                }
                                self.x[1] = pc.wrapping_add(2);
                                next = self.x[rd];
                            } else {
                                // C.ADD
                                self.x[rd] = self.x[rd].wrapping_add(self.x[8 + rs2f as usize]);
                            }
                        }
                        2 => {
                            // C.ANDI (bit12 is 0 in GAS; imm is 6-bit signed)
                            let rd = 8 + ((insn >> 7) & 0x7) as usize;
                            let imm = sext((insn >> 2) & 0x1F, 6);
                            self.x[rd] &= imm;
                        }
                        3 => {
                            // alu group: SUB/XOR/OR/AND via bits[6:5]
                            let op = (insn >> 5) & 0x3;
                            let rd = 8 + ((insn >> 7) & 0x7) as usize;
                            let rs2 = 8 + ((insn >> 2) & 0x7) as usize;
                            match op {
                                0 => self.x[rd] = self.x[rd].wrapping_sub(self.x[rs2]),
                                1 => self.x[rd] ^= self.x[rs2],
                                2 => self.x[rd] |= self.x[rs2],
                                3 => self.x[rd] &= self.x[rs2],
                                _ => unreachable!(),
                            }
                        }
                        _ => {
                            self.halted = true;
                            return pc;
                        }
                    }
                }
                5 => {
                    // C.J
                    next = pc.wrapping_add(c_j_imm(insn));
                }
                6 => {
                    // C.BEQZ
                    let rs1 = 8 + ((insn >> 7) & 0x7) as usize;
                    if self.x[rs1] == 0 {
                        next = pc.wrapping_add(c_b_imm(insn));
                    }
                }
                7 => {
                    // C.BNEZ
                    let rs1 = 8 + ((insn >> 7) & 0x7) as usize;
                    if self.x[rs1] != 0 {
                        next = pc.wrapping_add(c_b_imm(insn));
                    }
                }
                _ => {
                    self.halted = true;
                    return pc;
                }
            },
            2 => match funct3 {
                0 => {
                    // C.SLLI: rd <<= shamt (5-bit).
                    let rd = ((insn >> 7) & 0x1F) as usize;
                    let sh = (insn >> 2) & 0x1F;
                    self.x[rd] <<= sh;
                }
                2 => {
                    // C.LWSP: rd = mem[x2 + (uimm[7:2] << 2)].
                    let rd = ((insn >> 7) & 0x1F) as usize;
                    if rd != 0 {
                        let uimm = (bit(3) << 5)
                            | (bit(2) << 4)
                            | (bit(12) << 3)
                            | (bit(6) << 2)
                            | (bit(5) << 1)
                            | bit(4);
                        let addr = self.x[2].wrapping_add(uimm << 2);
                        self.x[rd] = self.load(mem, addr, 2);
                    }
                }
                4 => {
                    // C.JR / C.MV / C.JALR / C.ADD / C.EBREAK.
                    let rd = ((insn >> 7) & 0x1F) as usize;
                    let rs2 = ((insn >> 2) & 0x1F) as usize;
                    if bit(12) == 0 {
                        if rs2 == 0 {
                            // C.JR
                            next = self.x[rd] & !1;
                        } else {
                            // C.MV: rd = rs2
                            self.x[rd] = self.x[rs2];
                        }
                    } else if rs2 == 0 {
                        if rd == 0 {
                            // C.EBREAK
                            self.halted = true;
                            return pc;
                        } else {
                            // C.JALR: ra = pc+2; jump.
                            self.x[1] = pc.wrapping_add(2);
                            next = self.x[rd] & !1;
                        }
                    } else {
                        // C.ADD
                        self.x[rd] = self.x[rd].wrapping_add(self.x[rs2]);
                    }
                }
                6 => {
                    // C.SWSP: mem[x2 + (uimm[7:2] << 2)] = rs2.
                    let rs2 = ((insn >> 2) & 0x1F) as usize;
                    let uimm = (bit(8) << 5)
                        | (bit(7) << 4)
                        | (bit(12) << 3)
                        | (bit(11) << 2)
                        | (bit(10) << 1)
                        | bit(9);
                    let addr = self.x[2].wrapping_add(uimm << 2);
                    self.store(mem, addr, self.x[rs2], 2);
                }
                _ => {
                    self.halted = true;
                    return pc;
                }
            },
            _ => {
                self.halted = true;
                return pc;
            }
        }
        next
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

fn read16(mem: &[u8], off: u32) -> u16 {
    let o = off as usize;
    u16::from_le_bytes([mem[o], mem[o + 1]])
}

/// Sign-extend `v` (a value with `n` meaningful low bits) to a 32-bit word.
fn sext(v: u32, n: u32) -> u32 {
    let s = 32 - n;
    (((v as i32) << s) >> s) as u32
}

/// Decode the 12-bit (signed, ×2) offset of a C.J / C.JAL instruction.
fn c_j_imm(insn: u32) -> u32 {
    let b = |x: u32| (insn >> x) & 1;
    let imm = (b(3) << 1)
        | (b(4) << 2)
        | (b(5) << 3)
        | (b(11) << 4)
        | (b(2) << 5)
        | (b(7) << 6)
        | (b(6) << 7)
        | (b(9) << 8)
        | (b(10) << 9)
        | (b(8) << 10)
        | (b(12) << 11);
    sext(imm, 12)
}

/// Decode the 9-bit (signed, ×2) offset of a C.BEQZ / C.BNEZ instruction.
fn c_b_imm(insn: u32) -> u32 {
    let b = |x: u32| (insn >> x) & 1;
    let imm =
        (b(3) << 1) | (b(4) << 2) | (b(10) << 3) | (b(11) << 4) | (b(2) << 5) | (b(5) << 6) | (b(6) << 7) | (b(12) << 8);
    sext(imm, 9)
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

    /// Hand-assembled rv32imc program (built with riscv32-esp-elf-as, -march=rv32imc)
    /// that exercises the compressed (C) extension: C.ADDI4SPN, C.LWSP/C.SWSP,
    /// C.LI, C.ADDI, C.ADD, C.SUB, C.SLLI, C.SRAI, C.ANDI, C.XOR/C.OR/C.AND via
    /// C.MV+C.<op>, C.LW/C.SW, C.BEQZ, C.J, C.MV and C.EBREAK. It computes a set
    /// of results into `RTC_SLOW_MEM` (base 0x5000_0000 + 0x80 data region).
    const C_PROG: &[u8] = &[
        0x37, 0x01, 0x00, 0x50, 0x00, 0x01, 0xa2, 0xc8, 0xc6, 0x44, 0xa6, 0xdc, 0x29, 0x45, 0xd1, 0x45,
        0x2e, 0x95, 0x2a, 0x86, 0x0d, 0x8e, 0x0a, 0x05, 0x09, 0x85, 0x9d, 0x89, 0xb2, 0x86, 0xad, 0x8e,
        0x32, 0x87, 0x4d, 0x8f, 0xb2, 0x87, 0xed, 0x8f, 0x08, 0xc0, 0x54, 0xc0, 0x18, 0xc4, 0x5c, 0xc4,
        0x10, 0xcc, 0x91, 0x47, 0x81, 0x48, 0x85, 0x08, 0x99, 0xc3, 0xfd, 0x17, 0xed, 0xbf, 0x23, 0x28,
        0x14, 0x01, 0x04, 0x40, 0x44, 0xc8, 0x02, 0x90,
    ];

    fn rd(mem: &[u8], off: usize) -> u32 {
        u32::from_le_bytes([mem[off], mem[off + 1], mem[off + 2], mem[off + 3]])
    }

    #[test]
    fn ulp_runs_compressed_rv32imc_program() {
        let mut ulp = Ulp::new();
        let mut mem = [0u8; RTC_SLOW_SIZE as usize];
        mem[..C_PROG.len()].copy_from_slice(C_PROG);
        ulp.write32(ULP_BASE, 1);
        for _ in 0..200 {
            if !ulp.is_running() {
                break;
            }
            ulp.step(&mut mem[..]);
        }
        assert!(!ulp.is_running(), "ULP did not halt");
        // SP-relative (C.LWSP/C.SWSP): x8 = base+0x80 stored at both 0x50 and 0x78.
        assert_eq!(rd(&mem, 0x50), 0x5000_0080);
        assert_eq!(rd(&mem, 0x78), 0x5000_0080);
        // Arithmetic results in the 0x80 data region.
        assert_eq!(rd(&mem, 0x80), 30); // x10 = (10+20) <<2 >>2
        assert_eq!(rd(&mem, 0x84), 14); // x13 = (10 ^ 4)
        assert_eq!(rd(&mem, 0x88), 14); // x14 = (10 | 4)
        assert_eq!(rd(&mem, 0x8C), 0); // x15 = (10 & 4)
        assert_eq!(rd(&mem, 0x90), 5); // loop ran 5 times
        assert_eq!(rd(&mem, 0x94), 30); // C.LW round-trip of 0x80
        assert_eq!(rd(&mem, 0x98), 10); // x12 = (30 - 20)
    }
}

