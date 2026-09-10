#![no_std]
extern crate alloc;

#[cfg(test)]
extern crate std;

pub mod bus;
pub mod cpu;
pub mod ee;
mod exec;
pub mod generated;

pub use bus::Bus;
pub use cpu::{Cpu, StepResult};
#[cfg(test)]
mod tests {
    use crate::generated::{decode_inst, *};

    fn enc(insn: u32) -> Option<Opcode> {
        decode_inst(insn)
    }

    #[test]
    fn l32r_opnds() {
        // l32r a3, lit: op0=1, t=[7:4]=3, imm16=[23:8]=0x1234
        let insn = 0x0012_3431u32;
        assert_eq!(enc(insn), Some(Opcode::OPCODE_L32R));
        let o = opnds(Opcode::OPCODE_L32R, insn, 0x4000_0100);
        assert_eq!((o[0].value, o[0].is_reg), (3, true));
        // L32R pc-relative offset: (((0xffff)<<16)|imm16)<<2 — the top 16
        // ones force a negative offset for every imm16 (QEMU operand
        // uimm16x4), so L32R only references backward.  sext16(imm16)<<2
        // would be wrong for fields with bit 15 clear (forward target).
        assert_eq!(
            o[1].value,
            (0xffff_0000u32 | 0x1234)
                .wrapping_shl(2)
                .wrapping_add((0x4000_0100u32 + 3) & !3)
        );
    }

    #[test]
    fn call0_opnds() {
        // call0: op0=5, n=[5:4]=0, offset=[23:6] sext 18, <<2, pc-aligned
        let insn = 0x0000_0005u32; // offset = 0
        assert_eq!(enc(insn), Some(Opcode::OPCODE_CALL0));
        let o = opnds(Opcode::OPCODE_CALL0, insn, 0x4000_0008);
        // 4 + (sext(0)<<2) + (pc & ~3)
        assert_eq!(o[0].value, 0x4000_000c);
        assert!(!o[0].is_reg);
        assert_eq!((o[1].value, o[1].is_reg), (0, true)); // invisible n=0 (reg_hi)
        assert!(!o[1].visible);
    }

    #[test]
    fn entry_opnds() {
        // entry a1, 0x20: op0=6, n=3, m=0; s=[11:8]=1, imm12=[19:12]=0x20
        let insn = 0x0002_0136u32;
        assert_eq!(enc(insn), Some(Opcode::OPCODE_ENTRY));
        let o = opnds(Opcode::OPCODE_ENTRY, insn, 0);
        assert_eq!((o[0].value, o[1].value, o[2].value), (1, 1, 0x20 << 3));
    }

    #[test]
    fn loop_opnds() {
        // loop a2, end: op0=6, n=3, m=1, r=8 (loop), s=[11:8]=2, imm8=[23:16]=0x40
        let insn = 0x0040_8276u32;
        assert_eq!(enc(insn), Some(Opcode::OPCODE_LOOP));
        let o = opnds(Opcode::OPCODE_LOOP, insn, 0x4000_1000);
        assert_eq!((o[0].value, o[0].is_reg), (2, true));
        // LEND = pc + 4 + imm8 (unsigned)
        assert_eq!(o[1].value, 0x4000_1000 + 4 + 0x40);
    }

    #[test]
    fn slli_opnds() {
        // slli a3, a4, 5: op0=0, op1=1, op2=1; SAL split field = {insn[20], insn[7:4]}
        // assembler encodes 32-shift, so raw sal = 0x1b = 27 -> operand 32-27 = 5
        let insn = 0x0011_34b0u32; // sal=27: bit20=1, bits[7:4]=0xb; r=3, s=4
        assert_eq!(enc(insn), Some(Opcode::OPCODE_SLLI));
        let o = opnds(Opcode::OPCODE_SLLI, insn, 0);
        assert_eq!((o[0].value, o[1].value, o[2].value), (3, 4, 5));
    }

    #[test]
    fn beqi_opnds() {
        // beqi a1, 1, label: op0=6, n=2, m=0; s=1, r=1 (b4c->1), imm8=0xfc (-4)
        let insn = 0x00fc_1126u32;
        assert_eq!(enc(insn), Some(Opcode::OPCODE_BEQI));
        let o = opnds(Opcode::OPCODE_BEQI, insn, 0x4000_0000);
        assert_eq!(o[0].value, 1);
        assert_eq!(o[1].value, 1); // b4c_TBL[1] = 1
        assert_eq!(o[2].value, 0x4000_0000); // label8 = 4 + sext8(0xfc) + pc = pc
    }

    #[test]
    fn addi_opnds() {
        // addi a3, a4, -2: op0=2, r=0xc, s=4, t=3, imm8 = 0xfe
        let insn = 0x00fe_c432u32;
        assert_eq!(enc(insn), Some(Opcode::OPCODE_ADDI));
        let o = opnds(Opcode::OPCODE_ADDI, insn, 0);
        assert_eq!((o[0].value, o[1].value, o[2].value), (3, 4, 0xfffffffe));
    }

    #[test]
    fn ee_dsp_decodes_as_unimplemented() {
        // A known `ee.*` opcode (ee.stf.64.xp: fld_inst_19_16==7,
        // fld_inst_3_0==0) decodes to its named opcode and is unimplemented at
        // runtime (exec's `_ => Outcome::Unimplemented` arm). The decoder's
        // final catch-all routes every undecoded TIE/DSP instruction to
        // OPCODE_EE_UNIMPLEMENTED instead of returning None (which would raise
        // a spurious illegal-instruction exception on real S3 silicon, where
        // the DSP ISA is implemented).
        let o = enc(0x0007_0000);
        assert!(
            matches!(o, Some(Opcode::OPCODE_EE_STF_64_XP)),
            "got {:?}",
            o
        );
    }
}

#[cfg(test)]
mod cpu_tests {
    use crate::cpu::*;
    use crate::{Bus, Cpu, StepResult};
    use std::collections::HashMap;
    use std::vec::Vec;

    /// Simple RAM test bus: anything outside the loaded image reads 0.
    struct RamBus {
        mem: HashMap<u32, u8>,
    }

    impl RamBus {
        fn load(prog: &[(u32, u32)]) -> Self {
            let mut mem = HashMap::new();
            for (addr, insn) in prog {
                mem.insert(*addr, (insn & 0xff) as u8);
                mem.insert(*addr + 1, ((insn >> 8) & 0xff) as u8);
                mem.insert(*addr + 2, ((insn >> 16) & 0xff) as u8);
                mem.insert(*addr + 3, ((insn >> 24) & 0xff) as u8);
            }
            RamBus { mem }
        }
    }

    impl Bus for RamBus {
        fn read8(&mut self, addr: u32) -> u32 {
            self.mem.get(&addr).copied().unwrap_or(0) as u32
        }
        fn read16(&mut self, addr: u32) -> u32 {
            self.read8(addr) | (self.read8(addr + 1) << 8)
        }
        fn read32(&mut self, addr: u32) -> u32 {
            self.read8(addr)
                | (self.read8(addr + 1) << 8)
                | (self.read8(addr + 2) << 16)
                | (self.read8(addr + 3) << 24)
        }
        fn write8(&mut self, addr: u32, val: u32) {
            self.mem.insert(addr, val as u8);
        }
        fn write16(&mut self, addr: u32, val: u32) {
            self.write8(addr, val);
            self.write8(addr + 1, val >> 8);
        }
        fn write32(&mut self, addr: u32, val: u32) {
            self.write8(addr, val);
            self.write8(addr + 1, val >> 8);
            self.write8(addr + 2, val >> 16);
            self.write8(addr + 3, val >> 24);
        }
    }

    /// Step until pc reaches `end` (or an exception/unimplemented occurs).
    fn run<B: Bus>(cpu: &mut Cpu, bus: &mut B, end: u32) {
        for _ in 0..10_000 {
            if cpu.pc >= end {
                return;
            }
            match cpu.step(bus) {
                StepResult::Ok => {}
                StepResult::Exception { cause } => {
                    panic!("unexpected exception {cause} at pc {:#010x}", cpu.pc)
                }
                StepResult::Unimplemented(op) => {
                    panic!("unimplemented opcode {op} at pc {:#010x}", cpu.pc)
                }
            }
        }
        panic!(
            "run did not reach {end:#010x}, pc stuck at {:#010x}",
            cpu.pc
        );
    }

    /// RamBus with externally-driven CPU interrupt lines.
    struct IntBus {
        mem: HashMap<u32, u8>,
        lines: u32,
    }

    impl IntBus {
        fn load(prog: &[(u32, u32)], lines: u32) -> Self {
            let mut mem = HashMap::new();
            for (addr, insn) in prog {
                mem.insert(*addr, (insn & 0xff) as u8);
                mem.insert(*addr + 1, ((insn >> 8) & 0xff) as u8);
                mem.insert(*addr + 2, ((insn >> 16) & 0xff) as u8);
                mem.insert(*addr + 3, ((insn >> 24) & 0xff) as u8);
            }
            IntBus { mem, lines }
        }
    }

    impl Bus for IntBus {
        fn read8(&mut self, addr: u32) -> u32 {
            self.mem.get(&addr).copied().unwrap_or(0) as u32
        }
        fn read16(&mut self, addr: u32) -> u32 {
            self.read8(addr) | (self.read8(addr + 1) << 8)
        }
        fn read32(&mut self, addr: u32) -> u32 {
            self.read8(addr)
                | (self.read8(addr + 1) << 8)
                | (self.read8(addr + 2) << 16)
                | (self.read8(addr + 3) << 24)
        }
        fn write8(&mut self, addr: u32, val: u32) {
            self.mem.insert(addr, val as u8);
        }
        fn write16(&mut self, addr: u32, val: u32) {
            self.write8(addr, val);
            self.write8(addr + 1, val >> 8);
        }
        fn write32(&mut self, addr: u32, val: u32) {
            self.write8(addr, val);
            self.write8(addr + 1, val >> 8);
            self.write8(addr + 2, val >> 16);
            self.write8(addr + 3, val >> 24);
        }
        fn int_pending(&mut self, _cpu: usize) -> u32 {
            self.lines
        }
    }

    /// Assemble one instruction at `*addr`, advancing by its real length
    /// (op0 0..=7: 3 bytes, 8..=13: 2 bytes).
    fn put(prog: &mut Vec<(u32, u32)>, addr: &mut u32, insn: u32) {
        prog.push((*addr, insn));
        *addr += crate::generated::insn_len((insn & 0xff) as u8);
    }

    // Hand-assembled encodings verified against the generated decoder:
    //   movi at,imm:   (imm8<<16)|(0xa<<12)|(imm_hi<<8)|(at<<4)|2
    //   addi at,as,imm:(imm8<<16)|(0xc<<12)|(as<<8)|(at<<4)|2
    //   add at,as,bt:  (8<<20)|(at<<12)|(as<<8)|(bt<<4)
    //   slli at,as,n:  (1<<20)|(1<<16)|(at<<12)|(as<<8)|((32-n)<<4)   [sal split field]
    //   l32i/s32i:     (imm8<<16)|(r<<12)|(as<<8)|(at<<4)|2, r=2/6
    //   l8ui/s8i:      r=0/4, l16ui/s16i: r=1/5, s32c1i: r=14
    //   loop a,off:    (off<<16)|(8<<12)|(a<<8)|(3<<4)|(1<<6)|6, LEND=pc+4+off
    //   loopnez:       r=9; beqz/bnez: op0=6 n=1 m=0/1; beqi: n=2
    //   bne:           (imm8<<16)|(9<<12)|(s<<8)|(t<<4)|7
    //   j off:         (off<<6)|6, target=4+off+pc; call4: op0=5 n=1, target=4+(off<<2)+(pc&~3)
    //   ssr:           (as<<8); sll: op1=1 op2=10; srl: op1=1 op2=9 s=0; sra: op1=1 op2=11 s=0
    //   ill: 0; rfe: (3<<12); syscall: (5<<12); break: (4<<12)
    //   wsr.ps: (1<<20)|(3<<16)|(0xe6<<8)|(t<<4); rsr.ps: op2=0
    //   wsr.epc1: sr=177=0xb1; retw: (2<<6)|(1<<4); entry: op0=6 n=3 m=0
    //   16-bit: add.n r=s+t (op0=10), movi.n (op0=12 i=0), mov.n (op0=13 r=0),
    //           s32i.n (op0=9)

    #[test]
    fn alu_mem_branch_loop() {
        // ALU, shifts, zero-overhead loops (loop/loopnez), loads/stores,
        // s32c1i, branches and 16-bit density ops.
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0004_A022); // movi a2, 4
        put(&mut prog, &mut a, 0x0000_A032); // movi a3, 0
        let loop_pc = a;
        put(&mut prog, &mut a, 0x0005_8276); // loop a2, 5 (LEND = pc+4+5)
        put(&mut prog, &mut a, 0x0001_C332); // addi a3, a3, 1
        put(&mut prog, &mut a, 0x0001_C332); // addi a3, a3, 1
        assert_eq!(a, loop_pc + 4 + 5, "loop body ends at LEND");
        put(&mut prog, &mut a, 0x0034_A242); // movi a4, 0x1234
        put(&mut prog, &mut a, 0x0000_6432); // s32i a3, a4, 0   ; mem[0x234] = 8
        put(&mut prog, &mut a, 0x0000_2452); // l32i a5, a4, 0
        put(&mut prog, &mut a, 0x0000_2462); // s32c1i a6, a4, 0 ; cmp 0 vs 8: no store
        put(&mut prog, &mut a, 0x0000_A062); // movi a6, 0
        put(&mut prog, &mut a, 0x0000_2462); // s32c1i a6, a4, 0 ; old=8 != 0: a6 = 8
        put(&mut prog, &mut a, 0x0000_A022); // movi a2, 0
        let lnez_pc = a;
        put(&mut prog, &mut a, 0x0005_9276); // loopnez a2, 5    ; skip body
        put(&mut prog, &mut a, 0x0063_A0E2); // movi a14, 99     ; not executed
        put(&mut prog, &mut a, 0x0000_20F0); // nop              ; not executed
        assert_eq!(a, lnez_pc + 4 + 5, "loopnez body ends at LEND");
        put(&mut prog, &mut a, 0x002A_A042); // movi a4, 42
        put(&mut prog, &mut a, 0x0003_A022); // movi a2, 3
        put(&mut prog, &mut a, 0x0040_0200); // ssr a2           ; SAR = 3 (op2=4, r=t=0)
        put(&mut prog, &mut a, 0x00FF_AF32); // movi a3, -1
        put(&mut prog, &mut a, 0x00A1_7300); // sll a7, a3       ; -1 << 29
        put(&mut prog, &mut a, 0x0091_8030); // srl a8, a3       ; -1 >> 3 (logical)
        put(&mut prog, &mut a, 0x00B1_9030); // sra a9, a3       ; -1 >> 3 (arith)
        put(&mut prog, &mut a, 0x0001_A0A2); // movi a10, 1
        put(&mut prog, &mut a, 0x0000_5A16); // beqz a10, 5      ; not taken
        put(&mut prog, &mut a, 0x0000_5A56); // bnez a10, 5      ; taken -> a+12
        put(&mut prog, &mut a, 0x0000_20F0); // nop              ; skipped
        put(&mut prog, &mut a, 0x0000_20F0); // nop              ; skipped
        put(&mut prog, &mut a, 0x0005_9A27); // bne a10, a2, 5   ; taken -> a+21
        put(&mut prog, &mut a, 0x0000_20F0); // nop              ; skipped
        put(&mut prog, &mut a, 0x0000_20F0); // nop              ; skipped
        put(&mut prog, &mut a, 0x0005_3226); // beqi a2, 3, 5    ; taken -> a+30
        put(&mut prog, &mut a, 0x0000_20F0); // nop              ; skipped
        put(&mut prog, &mut a, 0x0000_20F0); // nop              ; skipped
        put(&mut prog, &mut a, 0x0011_A142); // movi a4, 0x1111
        put(&mut prog, &mut a, 0x0000_222A); // add.n a2, a2, a2 ; a2 = 6
        put(&mut prog, &mut a, 0x0000_530C); // movi.n a3, 5   ; dest=s[11:8], imm7={z,n,r}
        put(&mut prog, &mut a, 0x0000_03BD); // mov.n a11, a3
        put(&mut prog, &mut a, 0x0000_0469); // s32i.n a6, a4, 0 ; mem[0x1111] = 8
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000; // programs live at 0x40000000
        run(&mut cpu, &mut bus, end);

        assert_eq!(cpu.reg(2), 6, "add.n result");
        assert_eq!(cpu.reg(3), 5, "movi.n result");
        assert_eq!(cpu.reg(4), 0x111, "beqi taken path");
        assert_eq!(cpu.reg(5), 8, "loop sum via l32i");
        assert_eq!(cpu.reg(6), 8, "s32c1i result");
        assert_eq!(cpu.reg(7), 0xE000_0000, "sll");
        assert_eq!(cpu.reg(8), 0x1FFF_FFFF, "srl");
        assert_eq!(cpu.reg(9), 0xFFFF_FFFF, "sra");
        assert_eq!(cpu.reg(10), 1, "branch condition reg");
        assert_eq!(cpu.reg(11), 5, "mov.n result");
        assert_eq!(cpu.reg(14), 0, "loopnez body skipped");
        assert_eq!(bus.read32(0x234), 8, "stored loop sum");
        assert_eq!(bus.read32(0x111), 8, "s32i.n store");
        assert_eq!(cpu.sreg(SR_SAR), 3, "ssr wrote SAR");
    }

    /// Run one raw instruction word (assembled with
    /// xtensa-esp32s3-elf-as; every encoding below was verified against
    /// the generated decoder) with registers preset, then return.
    fn run1(prog_word: u32, setup: impl FnOnce(&mut Cpu, &mut RamBus)) -> (Cpu, RamBus) {
        let mut bus = RamBus::load(&[(0x4000_0000, prog_word)]);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000;
        setup(&mut cpu, &mut bus);
        run(&mut cpu, &mut bus, 0x4000_0003);
        (cpu, bus)
    }

    #[test]
    fn mac16_mul_overwrites_acc() {
        // MUL.AA.HH a2, a3 = 0x770234: ACC <- sext(a2.hi) * sext(a3.hi).
        let (cpu, _) = run1(0x770234, |cpu, _| {
            cpu.set_reg(2, 0x0002_0003);
            cpu.set_reg(3, 0x0004_0005);
        });
        assert_eq!(cpu.sreg(SR_ACCLO), 8, "2*4");
        assert_eq!(cpu.sreg(SR_ACCHI), 0, "positive sign extension");
        // Signed halves: (-1) * 3 = -3.
        let (cpu, _) = run1(0x770234, |cpu, _| {
            cpu.set_reg(2, 0xFFFF_0000);
            cpu.set_reg(3, 0x0003_0000);
        });
        assert_eq!(cpu.sreg(SR_ACCLO), 0xFFFF_FFFD, "-3 low");
        assert_eq!(cpu.sreg(SR_ACCHI), 0xFFFF_FFFF, "-3 sign extension");
        // UMUL.AA.HH a2, a3 = 0x730234: unsigned halves, ACCHI = 0.
        let (cpu, _) = run1(0x730234, |cpu, _| {
            cpu.set_reg(2, 0xFFFF_0000);
            cpu.set_reg(3, 0xFFFF_0000);
        });
        assert_eq!(cpu.sreg(SR_ACCLO), 0xFFFE_0001, "65535^2 low");
        assert_eq!(cpu.sreg(SR_ACCHI), 0, "umul clears high");
    }

    #[test]
    fn mac16_mula_muls_accumulate() {
        // MULA.DA.HH m0, a3 = 0x6b0034: ACC += sext(MR0.hi) * sext(a3.hi).
        let (cpu, _) = run1(0x6b0034, |cpu, _| {
            cpu.set_sreg(SR_ACCLO, 100);
            cpu.set_sreg(SR_ACCHI, 0);
            cpu.set_sreg(SR_M0, 0x0002_0003);
            cpu.set_reg(3, 0x0004_0005);
        });
        assert_eq!(cpu.sreg(SR_ACCLO), 108, "100 + 2*4");
        assert_eq!(cpu.sreg(SR_ACCHI), 0);
        // MULS.AD.HL a2, m3 = 0x3d0244: ACC -= sext(a2.hi) * sext(MR3.lo).
        let (cpu, _) = run1(0x3d0244, |cpu, _| {
            cpu.set_sreg(SR_ACCLO, 1000);
            cpu.set_sreg(SR_ACCHI, 0);
            cpu.set_reg(2, 0x000A_0000);
            cpu.set_sreg(SR_M0 + 3, 0x0000_0009);
        });
        assert_eq!(cpu.sreg(SR_ACCLO), 1000 - 10 * 9, "1000 - 90");
        // 40-bit wrap: (0x7F:FFFFFFFF) + 1 -> LO=0, HI=ext8s(0x80).
        // MULA.AA.HH a2, a3 = 0x7b0234.
        let (cpu, _) = run1(0x7b0234, |cpu, _| {
            cpu.set_sreg(SR_ACCLO, 0xFFFF_FFFF);
            cpu.set_sreg(SR_ACCHI, 0x7F);
            cpu.set_reg(2, 0x0001_0000);
            cpu.set_reg(3, 0x0001_0000);
        });
        assert_eq!(cpu.sreg(SR_ACCLO), 0, "wrapped low");
        assert_eq!(cpu.sreg(SR_ACCHI), 0xFFFF_FF80, "ext8s(0x80)");
    }

    #[test]
    fn mac16_mr_select_bits() {
        // MUL.AD.HH a2, m3 = 0x370244 must read MR3 (not MR2): with
        // MR2.hi = 0x1111 and MR3.hi = 5, result 2*5 = 10 proves the
        // t[2] select bit (a const-MR2 decoder would give 2*0x1111).
        let (cpu, _) = run1(0x370244, |cpu, _| {
            cpu.set_reg(2, 0x0002_0003);
            cpu.set_sreg(SR_M0 + 2, 0x1111_1111);
            cpu.set_sreg(SR_M0 + 3, 0x0005_0007);
        });
        assert_eq!(cpu.sreg(SR_ACCLO), 10, "AR.hi * MR3.hi");
        // MUL.DD.HH m1, m3 word 0x274044 (assembler-verified): MR1.hi=7,
        // MR3.hi=9 -> 63. (DD mx aliases mod 2 like DA: m2->m0, m3->m1.)
        let (cpu, _) = run1(0x274044, |cpu, _| {
            cpu.set_sreg(SR_M0 + 1, 0x0007_0000);
            cpu.set_sreg(SR_M0 + 3, 0x0009_0000);
        });
        assert_eq!(cpu.sreg(SR_ACCLO), 63, "MR1.hi * MR3.hi");
    }

    #[test]
    fn mac16_ldinc_lddec() {
        // MULA.DA.HH.LDINC m2, a2, m1, a3 = 0x4b6234: MR2 <- mem[AR2+4],
        // AR2 += 4, ACC += sext(MR1.hi) * sext(AR3.hi).
        let (cpu, mut bus) = run1(0x4b6234, |cpu, bus| {
            bus.write32(0x1004, 0xDEAD_BEEF);
            cpu.set_reg(2, 0x1000);
            cpu.set_sreg(SR_M0 + 1, 0x0002_0003);
            cpu.set_reg(3, 0x0004_0005);
            cpu.set_sreg(SR_ACCLO, 1000);
            cpu.set_sreg(SR_ACCHI, 0);
        });
        assert_eq!(cpu.sreg(SR_M0 + 2), 0xDEAD_BEEF, "loaded word to MR2");
        assert_eq!(cpu.reg(2), 0x1004, "postincrement by 4");
        assert_eq!(cpu.sreg(SR_ACCLO), 1008, "1000 + 2*4");
        assert_eq!(bus.read32(0x1004), 0xDEAD_BEEF, "memory unchanged");
        // MULA.DA.HH.LDDEC m1, a4, m0, a5 = 0x5b1454: MR1 <- mem[AR4-4],
        // AR4 -= 4, ACC += sext(MR0.hi) * sext(AR5.hi).
        let (cpu, _) = run1(0x5b1454, |cpu, bus| {
            bus.write32(0x1FFC, 0xCAFE_BABE);
            cpu.set_reg(4, 0x2000);
            cpu.set_sreg(SR_M0, 0x0003_0002);
            cpu.set_reg(5, 0x0005_0004);
        });
        assert_eq!(cpu.sreg(SR_M0 + 1), 0xCAFE_BABE, "loaded word to MR1");
        assert_eq!(cpu.reg(4), 0x1FFC, "postdecrement by 4");
        assert_eq!(cpu.sreg(SR_ACCLO), 15, "3*5");
    }

    #[test]
    fn mac16_dot_product_unrolled() {
        // Four MULA.AA.LL words assembled with xtensa-esp32s3-elf-as
        // (0x780234/0x780454/0x780674/0x780894): dot([1,2,3,4],[5,6,7,8])
        // = 5+12+21+32 = 70 in the low halves.
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0078_0234);
        put(&mut prog, &mut a, 0x0078_0454);
        put(&mut prog, &mut a, 0x0078_0674);
        put(&mut prog, &mut a, 0x0078_0894);
        let end = a;
        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000;
        cpu.set_sreg(SR_ACCLO, 0);
        cpu.set_sreg(SR_ACCHI, 0);
        let xs = [1u32, 2, 3, 4];
        let ys = [5u32, 6, 7, 8];
        for i in 0..4 {
            cpu.set_reg(2 + 2 * i as u32, xs[i]);
            cpu.set_reg(3 + 2 * i as u32, ys[i]);
        }
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.sreg(SR_ACCLO), 70, "dot product");
        assert_eq!(cpu.sreg(SR_ACCHI), 0, "no overflow");
    }

    #[test]
    fn bool_all_any() {
        // ALL4 b2, b0 = 0x009020: BR2 <- AND of BR[0..3].
        let (cpu, _) = run1(0x009020, |cpu, _| {
            for b in 0..4 {
                cpu.set_br(b, true);
            }
        });
        assert!(cpu.br(2), "all four set");
        let (cpu, _) = run1(0x009020, |cpu, _| {
            cpu.set_br(0, true);
            cpu.set_br(1, true);
            cpu.set_br(3, true);
        });
        assert!(!cpu.br(2), "bit 2 clear");
        // ANY8 b7, b8 = 0x00a870: BR7 <- OR of BR[8..15].
        let (cpu, _) = run1(0x00a870, |cpu, _| {
            cpu.set_br(12, true);
        });
        assert!(cpu.br(7), "one of eight set");
        let (cpu, _) = run1(0x00a870, |_, _| {});
        assert!(!cpu.br(7), "none set");
        // ALL8 b9, b8 word 0x00b890 needs all of BR[8..15].
        let (cpu, _) = run1(0x00b890, |cpu, _| {
            for b in 8..16 {
                cpu.set_br(b, true);
            }
        });
        assert!(cpu.br(9), "all eight set");
    }

    #[test]
    fn dsp_load_store() {
        // S32RI a2, a3, 4 = 0x01f322: plain word store like S32I.
        let (cpu, mut bus) = run1(0x01f322, |cpu, _| {
            cpu.set_reg(2, 0xA5A5_A5A5);
            cpu.set_reg(3, 0x3000);
        });
        assert_eq!(bus.read32(0x3004), 0xA5A5_A5A5, "s32ri stored");
        assert_eq!(cpu.reg(2), 0xA5A5_A5A5, "data reg preserved");
        // LDDR32.P a2 = 0x0072e0: DDR <- mem64[AR2], AR2 += 8.
        let (cpu, _) = run1(0x0072e0, |cpu, bus| {
            cpu.set_reg(2, 0x4000);
            bus.write32(0x4000, 0x1111_1111);
            bus.write32(0x4004, 0x2222_2222);
        });
        assert_eq!(cpu.ddr, 0x2222_2222_1111_1111, "ddr loaded LE");
        // SDDR32.P a3 = 0x0073f0 tested against the loaded DDR below.
        assert_eq!(cpu.reg(2), 0x4008, "postupdate by 8");
        // SDDR32.P a3 = 0x0073f0: mem64[AR3] <- DDR, AR3 += 8.
        let (cpu, mut bus) = run1(0x0073f0, |cpu, _| {
            cpu.ddr = 0x3333_3333_4444_4444;
            cpu.set_reg(3, 0x5000);
        });
        assert_eq!(bus.read32(0x5000), 0x4444_4444, "low half stored LE");
        assert_eq!(bus.read32(0x5004), 0x3333_3333, "high half stored LE");
        assert_eq!(cpu.reg(3), 0x5008, "postupdate by 8");
    }

    #[test]
    fn windowed_call_entry_retw() {
        // call4/entry/retw: PS.WOE set first, then a windowed call chain
        // that mutates the caller's a8/a9/a5 through the callee window.
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x0000_A422); // movi a2, 0x400
        put(&mut prog, &mut a, 0x0011_2280); // slli a2, a2, 8    ; a2 = 0x40000 (WOE)
        put(&mut prog, &mut a, 0x0013_E620); // wsr.ps a2
        put(&mut prog, &mut a, 0x0060_A012); // movi a1, 0x60     ; caller sp
        let call_pc = a;
        put(&mut prog, &mut a, 0x0000_0095); // call4 (offset fixed below)
        put(&mut prog, &mut a, 0x0007_A032); // movi a3, 7        ; after return
        put(&mut prog, &mut a, 0x0000_0446); // j done (offset fixed below)
        put(&mut prog, &mut a, 0x0000_20F0); // nop               ; not reached
        let func = a;
        put(&mut prog, &mut a, 0x0000_4136); // entry a1, 0x20   ; imm12 = N>>3
        put(&mut prog, &mut a, 0x0000_6102); // s32i a0, a1, 0    ; save return addr (callee a0) at mem[0x40]
        put(&mut prog, &mut a, 0x0001_C442); // addi a4, a4, 1    ; caller a8 = 1
        put(&mut prog, &mut a, 0x0002_C552); // addi a5, a5, 2    ; caller a9 = 2
        put(&mut prog, &mut a, 0x0004_C112); // addi a1, a1, 4    ; caller a5 = 0x44
        put(&mut prog, &mut a, 0x0000_0090); // retw
        let done = a;
        put(&mut prog, &mut a, 0x0000_A2B2); // movi a11, 0x200
        put(&mut prog, &mut a, 0x0001_6B82); // s32i a8, a11, 4   ; callee a4 result
        put(&mut prog, &mut a, 0x0002_6B92); // s32i a9, a11, 8   ; callee a5 result
        put(&mut prog, &mut a, 0x0003_6B52); // s32i a5, a11, 12  ; callee a1 result
        put(&mut prog, &mut a, 0x0004_6B32); // s32i a3, a11, 16  ; post-call code
        let end = a;

        // Patch call4/j offsets now that labels are known.
        let call_off = (func.wrapping_sub(4).wrapping_sub(call_pc & !3)) >> 2;
        prog[call_pc.checked_sub(0x4000_1000).unwrap() as usize / 3].1 =
            0x0000_0015 | (call_off << 6);
        let j_off = done.wrapping_sub(4).wrapping_sub(call_pc + 3 + 3);
        prog[(call_pc + 6 - 0x4000_1000) as usize / 3].1 = 0x0000_0006 | (j_off << 6);

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);

        assert_eq!(
            bus.read32(0x40),
            call_pc + 3,
            "call4 return addr saved by callee"
        );
        assert_eq!(bus.read32(0x204), 1, "callee a4 == caller a8");
        assert_eq!(bus.read32(0x208), 2, "callee a5 == caller a9");
        assert_eq!(bus.read32(0x20c), 0x44, "callee a1 == caller a5");
        assert_eq!(bus.read32(0x210), 7, "code after call4");
        assert_eq!(cpu.sreg(SR_WINDOW_START), 1, "window 0 only");
        assert_eq!(cpu.windowbase(), 0, "window base restored");
        assert_eq!(cpu.sreg(SR_PS), 0x50000, "PS: WOE | CALLINC=1");
    }

    #[test]
    fn call0_preserves_callinc() {
        // QEMU translate_call0/translate_callx0 do NOT write PS.CALLINC
        // (only the windowed CALL4/8/12 + CALLX4/8/12 deposit it via
        // gen_callw_slot).  The ESP-IDF level-1 vector does `call0
        // _xt_user_exc` and _xt_lowint1 saves `rsr.ps` into the task
        // frame — if call0 cleared CALLINC, an interrupt taken right
        // after a task dispatch would resume the task with CALLINC=0
        // and its `entry` would fail to rotate (a2 = 0 -> callx8 0).
        // Regression for the ipc0 crash in esp32s3_hello.
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        // Layout constraints (real silicon, QEMU): CALL/CALL4 targets
        // resolve to (pc&~3)+4+off<<2 (always 4-aligned), and RET masks
        // a0&~3, so calls must sit at pc ≡ 1 (mod 4) so pc+3 ≡ 0 (mod 4).
        // The store base must be a4-free AND a5-free: call4 rotates the
        // window, so the callee's a1 lands in phys[5] (caller's a5) and
        // a4 holds the call4 return slot — only a0..a3 survive a call4.
        put(&mut prog, &mut a, 0x0006_A022); // movi a2, 6
        put(&mut prog, &mut a, 0x0011_2200); // slli a2, a2, 16    ; a2 = 0x60000 (WOE|CALLINC=2)
        put(&mut prog, &mut a, 0x0013_E620); // wsr.ps a2          ; WSR = op2=1 (0x13E)
        put(&mut prog, &mut a, 0x0000_A032); // movi a3, 0         ; store base (preserved)
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0003_E620); // rsr.ps a2          ; RSR = op2=0 (0x03E)
        let call0_pc = a;
        put(&mut prog, &mut a, 0x0000_0005); // call0 (off patched); pc ≡ 1 mod 4
        put(&mut prog, &mut a, 0x0010_6322); // s32i a2, a3, 0x40  ; mem[0x40] = ps after call0
        put(&mut prog, &mut a, 0x0000_0041); // l32r a4, f1 (imm16 patched)
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0003_E620); // rsr.ps a2
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        let _callx0_pc = a;
        put(&mut prog, &mut a, 0x0000_04C0); // callx0 a4          ; pc ≡ 1 mod 4
        put(&mut prog, &mut a, 0x0011_6322); // s32i a2, a3, 0x44  ; mem[0x44] = ps after callx0
        put(&mut prog, &mut a, 0x0060_A012); // movi a1, 0x60      ; sp for the call4 chain
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0003_E620); // rsr.ps a2
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        let call4_pc = a;
        put(&mut prog, &mut a, 0x0000_0015); // call4 (off patched); pc ≡ 1 mod 4
        put(&mut prog, &mut a, 0x0003_E620); // rsr.ps a2
        put(&mut prog, &mut a, 0x0012_6322); // s32i a2, a3, 0x48  ; mem[0x48] = ps after call4
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        let j_end_pc = a;
        put(&mut prog, &mut a, 0x0000_0006); // j end (off patched)
        let f2 = a;
        put(&mut prog, &mut a, 0x0000_4136); // f2: entry a1, 0x20  ; rotate by CALLINC=1
        put(&mut prog, &mut a, 0x0000_0090); //     retw             ; 4-aligned target
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        let f1 = a;
        put(&mut prog, &mut a, 0x0000_0080); // f1: ret            ; 4-aligned target
        let end = a;
        // l32r literal BEFORE the code (l32r target = ((0xFFFF0000|imm16)
        // << 2) + base is always a backward reference — QEMU uimm16x4).
        prog.push((0x4000_0FFC, f1)); // 4 bytes, 4-aligned

        // Patch call targets (call offset = (target - 4 - (pc & ~3)) >> 2).
        let c0 = f1.wrapping_sub(4).wrapping_sub(call0_pc & !3) >> 2;
        prog[(call0_pc - 0x4000_1000) as usize / 3].1 = 0x05 | (c0 << 6);
        let c4 = f2.wrapping_sub(4).wrapping_sub(call4_pc & !3) >> 2;
        prog[(call4_pc - 0x4000_1000) as usize / 3].1 = 0x15 | (c4 << 6);
        // j end from 0x51: off = end - pc - 4.
        prog[(j_end_pc - 0x4000_1000) as usize / 3].1 =
            0x06 | (end.wrapping_sub(4).wrapping_sub(j_end_pc) << 6);
        // l32r a4 at 0x1b: base = (0x4000101b + 3) & ~3 = 0x4000101c.
        let imm16 = (0x4000_0FFCu32.wrapping_sub(0x4000_101c) >> 2) & 0xFFFF;
        prog[9].1 = (imm16 << 8) | 0x41; // l32r: imm16 in bits [23:8]

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);

        assert_eq!(bus.read32(0x40), 0x60000, "call0 leaves CALLINC alone");
        assert_eq!(bus.read32(0x44), 0x60000, "callx0 leaves CALLINC alone");
        assert_eq!(bus.read32(0x48), 0x50000, "call4 writes CALLINC=1");
        assert_eq!(cpu.sreg(SR_PS), 0x50000, "PS after call4");
    }

    #[test]
    fn exception_vectors_and_rfe() {
        // ill -> kernel vector (VECBASE + 0x300); handler rewrites EPC1
        // and returns with rfe.
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0001_A022); // movi a2, 1
        let ill_pc = a;
        put(&mut prog, &mut a, 0x0000_0000); // ill
        put(&mut prog, &mut a, 0x0037_A032); // movi a3, 55       ; return target
        put(&mut prog, &mut a, 0x0000_6642); // s32i a4, a6, 0    ; mem[0] = 8
        let end = a;
        let mut h = 0x4000_0300u32;
        put(&mut prog, &mut h, 0x0040_A042); // movi a4, 0x40     ; handler
        put(&mut prog, &mut h, 0x0001_4480); // slli a4, a4, 24   ; 0x40 << 24 = 0x40000000
        put(&mut prog, &mut h, 0x0006_C442); // addi a4, a4, 6    ; EPC1 = 0x40000006
        put(&mut prog, &mut h, 0x0013_B140); // wsr.epc1 a4
        put(&mut prog, &mut h, 0x0000_3000); // rfe

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000; // programs live at 0x40000000
        assert!(matches!(cpu.step(&mut bus), StepResult::Ok));
        assert_eq!(cpu.pc, ill_pc, "reached ill");
        assert!(matches!(
            cpu.step(&mut bus),
            StepResult::Exception {
                cause: ILLEGAL_INSTRUCTION_CAUSE
            }
        ));
        assert_eq!(cpu.sreg(SR_EPC1), ill_pc, "EPC1 = faulting pc");
        assert_eq!(cpu.sreg(SR_EXCCAUSE), ILLEGAL_INSTRUCTION_CAUSE);
        assert_ne!(cpu.sreg(SR_PS) & PS_EXCM, 0, "EXCM set");
        assert_eq!(cpu.pc, 0x4000_0300, "kernel vector");

        assert!(matches!(cpu.step(&mut bus), StepResult::Ok)); // movi a4, 0x400
        assert!(matches!(cpu.step(&mut bus), StepResult::Ok)); // slli
        assert!(matches!(cpu.step(&mut bus), StepResult::Ok)); // addi
        assert!(matches!(cpu.step(&mut bus), StepResult::Ok)); // wsr.epc1 a4
        assert!(matches!(cpu.step(&mut bus), StepResult::Ok)); // rfe
        assert_eq!(cpu.pc, 0x4000_0006, "rfe returned via rewritten EPC1");
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(3), 55, "returned past fault");
        assert_eq!(cpu.sreg(SR_PS) & PS_EXCM, 0, "rfe cleared EXCM");
        assert_eq!(cpu.sreg(SR_EPC1), 0x4000_0006, "handler rewrote EPC1");
        assert_eq!(bus.read32(0), 0x4000_0006, "handler ran");
    }

    #[test]
    fn load_store_widths() {
        // All load/store widths including unaligned access (ESP32-S3 has
        // XCHAL_UNALIGNED_LOAD_HW=1: no alignment exceptions).
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0000_A122); // movi a2, 0x100
        put(&mut prog, &mut a, 0x00FF_AF32); // movi a3, -1
        put(&mut prog, &mut a, 0x0000_4232); // s8i a3, a2, 0
        put(&mut prog, &mut a, 0x0001_5232); // s16i a3, a2, 2    ; unaligned
        put(&mut prog, &mut a, 0x0000_0242); // l8ui a4, a2, 0
        put(&mut prog, &mut a, 0x0001_1252); // l16ui a5, a2, 2
        put(&mut prog, &mut a, 0x0001_9262); // l16si a6, a2, 2
        put(&mut prog, &mut a, 0x0055_A072); // movi a7, 0x55
        put(&mut prog, &mut a, 0x0001_4272); // s8i a7, a2, 1
        put(&mut prog, &mut a, 0x0001_0282); // l8ui a8, a2, 1
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000; // programs live at 0x40000000
        run(&mut cpu, &mut bus, end);

        assert_eq!(cpu.reg(4), 0xff, "l8ui");
        assert_eq!(cpu.reg(5), 0xffff, "l16ui");
        assert_eq!(cpu.reg(6), 0xffff_ffff, "l16si sign-extended");
        assert_eq!(cpu.reg(8), 0x55, "l8ui overwritten byte");
        assert_eq!(bus.read8(0x100), 0xff);
        assert_eq!(bus.read8(0x101), 0x55);
        assert_eq!(bus.read16(0x102), 0xffff);
    }

    #[test]
    fn interrupt_preemption_level4_takes_in_level3_handler() {
        // Main enables line 22 (level 3) and line 24 (level 4).  A level-3
        // take runs its handler with PS.INTLEVEL = 3 | EXCM; a level-4 line
        // then preempts (cintlevel = max(3, 3) = 3 < 4).  rfi 4 returns to the
        // level-3 handler, rfi 3 returns to the main program.
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0001_A032); // movi a3, 1
        put(&mut prog, &mut a, 0x0001_3380); // slli a3, a3, 24     ; 0x1000000 (line 24, L4)
        put(&mut prog, &mut a, 0x0080_A022); // movi a2, 0x80
        put(&mut prog, &mut a, 0x0011_2210); // slli a2, a2, 15     ; 0x400000 (line 22, L3)
        put(&mut prog, &mut a, 0x0020_2230); // or a2, a2, a3       ; 0x1400000
        put(&mut prog, &mut a, 0x0013_E420); // wsr intenable a2
        put(&mut prog, &mut a, 0x0000_20F0); // nop                ; EPC3 = 0x12
        put(&mut prog, &mut a, 0x0000_20F0); // nop
        put(&mut prog, &mut a, 0x0007_A032); // movi a3, 7
        let end = a;
        let mut h3 = 0x4000_01C0u32; // level 3 vector
        put(&mut prog, &mut h3, 0x0033_A342); // movi a4, 0x333
        put(&mut prog, &mut h3, 0x0000_20F0); // nop
        put(&mut prog, &mut h3, 0x0000_20F0); // nop                ; EPC4 = rfi's address
        put(&mut prog, &mut h3, 0x0000_3310); // rfi 3
        let mut h4 = 0x4000_0200u32; // level 4 vector
        put(&mut prog, &mut h4, 0x0044_A452); // movi a5, 0x444
        put(&mut prog, &mut h4, 0x0000_3410); // rfi 4

        let mut bus = IntBus::load(&prog, 1 << 22);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000; // programs live at 0x40000000
        for _ in 0..6 {
            cpu.step(&mut bus); // 5 builders + wsr (take at end)
        }
        assert_eq!(cpu.sreg(SR_INTENABLE), 0x140_0000, "intenable");
        assert_eq!(cpu.pc, 0x4000_01C0, "level 3 vector");
        assert_eq!(cpu.sreg(SR_PS), 0x13, "PS: INTLEVEL 3 | EXCM");
        for _ in 0..2 {
            cpu.step(&mut bus); // movi a4, nop
        }
        bus.lines = (1 << 22) | (1 << 24);
        cpu.step(&mut bus); // nop -> level 4 preempts (cintlevel 3)
        assert_eq!(cpu.pc, 0x4000_0200, "level 4 vector");
        assert_eq!(cpu.sreg(SR_PS), 0x14, "PS: INTLEVEL 4 | EXCM");
        assert_eq!(cpu.sreg(SR_EPC4), 0x4000_01C9, "EPC4 = inside L3 handler");
        assert_eq!(cpu.sreg(SR_EPS4), 0x13, "EPS4 = L3 handler PS");
        cpu.step(&mut bus); // movi a5
        bus.lines = 1 << 22; // L4 handler "clears" its peripheral (INT_CLR)
        cpu.step(&mut bus); // rfi 4 -> back into L3 handler
        assert_eq!(cpu.pc, 0x4000_01C9, "rfi 4 resumed L3 handler");
        assert_eq!(cpu.sreg(SR_PS), 0x13, "rfi 4 restored L3 PS");
        bus.lines = 0; // L3 handler "clears" its peripheral (INT_CLR)
        cpu.step(&mut bus); // rfi 3 -> back to main
        assert_eq!(cpu.pc, 0x4000_0012, "rfi 3 resumed main");
        assert_eq!(cpu.sreg(SR_PS), 0, "rfi 3 restored PS");
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(3), 7, "main finished");
        assert_eq!(cpu.reg(4), 0x333, "L3 handler marker");
        assert_eq!(cpu.reg(5), 0x444, "L4 handler marker");
    }

    #[test]
    fn interrupt_level2_take_and_rfi() {
        // wsr intenable (bit 19 = line 19, level 2); the asserted line 19
        // is taken at the end of the wsr itself: EPC2/EPS2 capture the
        // nop, PS = INTLEVEL 2 | EXCM, pc = VECBASE + 0x180.  The handler
        // disables the line and returns with rfi 2.
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0008_A022); // movi a2, 8
        put(&mut prog, &mut a, 0x0011_2200); // slli a2, a2, 16     ; 0x80000 (bit 19)
        put(&mut prog, &mut a, 0x0013_E420); // wsr intenable a2   ; take at its end
        put(&mut prog, &mut a, 0x0000_20F0); // nop                ; EPC2 = 9
        put(&mut prog, &mut a, 0x0007_A032); // movi a3, 7
        let end = a;
        let mut h = 0x4000_0180u32; // level 2 vector
        put(&mut prog, &mut h, 0x0000_A042); // movi a4, 0
        put(&mut prog, &mut h, 0x0013_E440); // wsr intenable a4    ; mask the line
        put(&mut prog, &mut h, 0x0000_3210); // rfi 2

        let mut bus = IntBus::load(&prog, 1 << 19);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000; // programs live at 0x40000000
        for _ in 0..3 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.pc, 0x4000_0180, "level 2 vector");
        assert_eq!(cpu.sreg(SR_EPC2), 0x4000_0009, "EPC2 = next instruction");
        assert_eq!(cpu.sreg(SR_EPS2), 0, "EPS2 = old PS");
        assert_eq!(cpu.sreg(SR_PS), 0x12, "PS: INTLEVEL 2 | EXCM");
        assert_eq!(cpu.sreg(SR_EPC1), 0, "EPC1 untouched");
        assert_eq!(cpu.sreg(SR_EXCCAUSE), 0, "not an exception");

        for _ in 0..3 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.pc, 0x4000_0009, "rfi 2 returned to EPC2");
        assert_eq!(cpu.sreg(SR_PS), 0, "rfi restored PS");
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(3), 7, "returned past the interrupt");
    }

    #[test]
    fn interrupt_masking_levels_and_nmi() {
        // Line 19 (level 2): masked by PS.INTLEVEL=2, and by PS.EXCM
        // (cintlevel = max(INTLEVEL, EXCM_LEVEL=3)); line 15 (level 3) is
        // also masked by EXCM; line 24 (level 4) beats it.
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0000_20F0); // nop (line 19 pending)
        put(&mut prog, &mut a, 0x0000_20F0); // nop (lines 19+15 pending)
        put(&mut prog, &mut a, 0x0000_20F0); // nop (line 24 added: taken)
        let mut h = 0x4000_0200u32; // level 4 vector
        put(&mut prog, &mut h, 0x0000_A042); // movi a4, 0
        put(&mut prog, &mut h, 0x0013_E440); // wsr intenable a4
        put(&mut prog, &mut h, 0x0000_3410); // rfi 4
        let mut bus = IntBus::load(&prog, 1 << 19);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000; // programs live at 0x40000000
        cpu.set_sreg(SR_INTENABLE, (1 << 19) | (1 << 15) | (1 << 24));
        cpu.set_sreg(SR_PS, 2); // INTLEVEL 2
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0x4000_0003, "masked by INTLEVEL 2");
        bus.lines = (1 << 19) | (1 << 15);
        cpu.set_sreg(SR_PS, PS_EXCM); // EXCM, INTLEVEL 0
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0x4000_0006, "masked by EXCM (cintlevel 3)");
        bus.lines |= 1 << 24;
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0x4000_0200, "level 4 beats EXCM_LEVEL 3");
        assert_eq!(cpu.sreg(SR_EPC4), 0x4000_0009, "EPC4 = next nop");
        assert_eq!(cpu.sreg(SR_PS), 0x14, "PS: INTLEVEL 4 | EXCM");

        // NMI (line 14) bypasses INTENABLE and INTLEVEL.
        let mut prog2 = Vec::new();
        let mut b = 0x4000_1000u32;
        put(&mut prog2, &mut b, 0x0004_A022); // movi a2, 4
        put(&mut prog2, &mut b, 0x0011_2240); // slli a2, a2, 12     ; 0x4000 (bit 14)
        put(&mut prog2, &mut b, 0x0013_E220); // wsr intset a2       ; sticky NMI
        let after = b;
        put(&mut prog2, &mut b, 0x0000_20F0); // nop                ; interrupted target
        let mut h2 = 0x4000_02C0u32; // NMI vector
        put(&mut prog2, &mut h2, 0x0000_20F0); // nop (handler body)
        let mut bus2 = IntBus::load(&prog2, 0); // no external lines
        let mut cpu2 = Cpu::new(0);
        cpu2.pc = 0x4000_1000;
        for _ in 0..3 {
            cpu2.step(&mut bus2);
        }
        assert_eq!(cpu2.pc, 0x4000_02C0, "NMI vector (INTENABLE = 0)");
        assert_eq!(cpu2.sreg(SR_EPC7), after, "EPC7 = next instruction");
        assert_eq!(cpu2.sreg(SR_PS), 0x17, "PS: NMI level 7 | EXCM");
        assert_eq!(
            cpu2.sreg(SR_INTSET) & (1 << 14),
            0,
            "NMI sticky bit cleared on take"
        );
        cpu2.step(&mut bus2); // handler nop; no re-take
        assert_eq!(cpu2.pc, 0x4000_02C3, "no re-take after NMI cleared");
    }

    #[test]
    fn interrupt_level1_is_kernel_exception() {
        // Level-1 lines have no vector: delivered as a kernel exception
        // with EXCCAUSE 4 (QEMU handle_interrupt else branch).
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0001_A022); // movi a2, 1
        put(&mut prog, &mut a, 0x0013_E420); // wsr intenable a2 (bit 0)
        put(&mut prog, &mut a, 0x0000_20F0); // nop                ; interrupted target
        let mut bus = IntBus::load(&prog, 1 << 0);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000; // programs live at 0x40000000
        cpu.step(&mut bus); // movi
        cpu.step(&mut bus); // wsr intenable -> level 1 pending
        assert_eq!(cpu.pc, 0x4000_0300, "kernel vector");
        assert_eq!(cpu.sreg(SR_EXCCAUSE), LEVEL1_INTERRUPT_CAUSE);
        assert_eq!(cpu.sreg(SR_EPC1), 0x4000_0006, "EPC1 = next instruction");
        assert_ne!(cpu.sreg(SR_PS) & PS_EXCM, 0, "EXCM set");
        assert_eq!(cpu.sreg(SR_PS) & PS_INTLEVEL, 0, "INTLEVEL untouched");
    }

    #[test]
    fn wsr_intset_intclear_and_rsr_interrupt() {
        // Sticky INTSET via wsr; rsr.interrupt reflects it; intclear
        // removes it.  No take: INTENABLE stays 0.
        let mut prog = Vec::new();
        let mut a = 0x4000_0000u32;
        put(&mut prog, &mut a, 0x0008_A022); // movi a2, 8
        put(&mut prog, &mut a, 0x0011_2280); // slli a2, a2, 8        ; 0x800 (bit 11, level 3)
        put(&mut prog, &mut a, 0x0013_E220); // wsr intset a2
        put(&mut prog, &mut a, 0x0003_E230); // rsr interrupt a3
        put(&mut prog, &mut a, 0x0013_E320); // wsr intclear a2
        put(&mut prog, &mut a, 0x0003_E240); // rsr interrupt a4
        let end = a;
        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_0000; // programs live at 0x40000000
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(3), 0x800, "rsr.interrupt reflects wsr.intset");
        assert_eq!(cpu.reg(4), 0, "intclear visible to rsr.interrupt");
        assert_eq!(cpu.sreg(SR_INTSET), 0, "wsr.intclear removed the bit");
        assert_eq!(cpu.pc, end, "no interrupt taken (INTENABLE = 0)");
    }

    #[test]
    fn prid_reads_core_id() {
        // PRID (SR 235) carries the per-core strapping; the ESP32-S3 ROM
        // compares against 0xCDCD (core 0) / 0xABAB (core 1) and the app
        // derives the core index as (PRID >> 13) & 1.
        // rsr a2, PRID = 0x0003_EB20 (t=2); rsr a3, PRID = 0x0003_EB30.
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x0003_EB20); // rsr a2, PRID
        put(&mut prog, &mut a, 0x0003_EB30); // rsr a3, PRID
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu0 = Cpu::new(0);
        cpu0.pc = 0x4000_1000;
        run(&mut cpu0, &mut bus, end);
        assert_eq!(cpu0.reg(2), 0xCDCD, "core 0 PRID");
        assert_eq!(cpu0.reg(3), 0xCDCD, "core 0 PRID");
        assert_eq!((cpu0.reg(2) >> 13) & 1, 0, "core 0 index bit");

        let mut bus = RamBus::load(&prog);
        let mut cpu1 = Cpu::new(1);
        cpu1.pc = 0x4000_1000;
        run(&mut cpu1, &mut bus, end);
        assert_eq!(cpu1.reg(2), 0xABAB, "core 1 PRID");
        assert_eq!(cpu1.reg(3), 0xABAB, "core 1 PRID");
        assert_eq!((cpu1.reg(2) >> 13) & 1, 1, "core 1 index bit");
    }

    #[test]
    fn rsr_windowbase_windowstart_roundtrip() {
        // `sr_of` needs explicit WINDOWBASE/WINDOWSTART arms: without them
        // both RSRs fall through to `_ => 0` and read LBEG (SR 0), which
        // poisons xthal_window_spill_nw's ws computation (MicroPython boot
        // died at ~7.47M insns with ws=0x4038, wild sp, ret-to-heap).
        // rsr a2, WINDOWBASE = 0x0003_4820; wsr WINDOWBASE a2 = 0x0013_4820;
        // rsr a3, WINDOWSTART = 0x0003_4930; wsr WINDOWSTART a3 = 0x0013_4930.
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x000C_A022); // movi a2, 12 (fitting wb)
        put(&mut prog, &mut a, 0x0013_4820); // wsr WINDOWBASE, a2
        put(&mut prog, &mut a, 0x0003_4830); // rsr a3, WINDOWBASE
        put(&mut prog, &mut a, 0x00A0_A242); // movi a4, 0x2A0
        put(&mut prog, &mut a, 0x0013_4940); // wsr WINDOWSTART, a4
        put(&mut prog, &mut a, 0x0003_4950); // rsr a5, WINDOWSTART
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(3), 12, "rsr.windowbase reads back wsr value");
        assert_eq!(cpu.reg(5), 0x2A0, "rsr.windowstart reads back wsr value");
    }

    // Single-precision FPU tests. FP RRR word layout (verified against
    // real S3 firmware bytes, e.g. wfr f1,a2 = 0xFA1250):
    //   word = (op2<<20)|(op1<<16)|(r<<12)|(s<<8)|(t<<4)|op0, op0=0.
    // FP class op1=10 (arith/convert/move), op1=11 (compares/cond moves);
    // op2 selects the op (ADD=0 SUB=1 MUL=2 MADD=4 MSUB=5 FLOAT=12
    // UFLOAT=13 UTRUNC=14 ROUND=8 TRUNC=9 FLOOR=10 CEIL=11; two-operand
    // class op2=15 with t-ext MOV=0 ABS=1 CONST=3 RFR=4 WFR=5 NEG=6;
    // compares op1=11 op2: UN=1 OEQ=2 UEQ=3 OLT=4 ULT=5 OLE=6 ULE=7).
    // movi at,imm: (imm[7:0]<<16)|(0xA<<12)|(imm[11:8]<<8)|(at<<4)|2.
    // slli at,as,n: (sa4<<20)|(1<<16)|(at<<12)|(as<<8)|((sa&15)<<4), sa=(32-n)&31.
    #[test]
    fn fpu_moves_and_arith() {
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x00F8_A322); // movi a2, 0x3F8
        put(&mut prog, &mut a, 0x0001_22C0); // slli a2, a2, 20   ; a2 = 1.0
        put(&mut prog, &mut a, 0x0000_A432); // movi a3, 0x400
        put(&mut prog, &mut a, 0x0001_33C0); // slli a3, a3, 20   ; a3 = 2.0
        put(&mut prog, &mut a, 0x00FA_1250); // wfr f1, a2
        put(&mut prog, &mut a, 0x00FA_2350); // wfr f2, a3
        put(&mut prog, &mut a, 0x000A_3120); // add.s f3, f1, f2  ; 3.0
        put(&mut prog, &mut a, 0x00FA_4340); // rfr a4, f3
        put(&mut prog, &mut a, 0x001A_3120); // sub.s f3, f1, f2  ; -1.0
        put(&mut prog, &mut a, 0x00FA_5340); // rfr a5, f3
        put(&mut prog, &mut a, 0x002A_3120); // mul.s f3, f1, f2  ; 2.0
        put(&mut prog, &mut a, 0x00FA_6340); // rfr a6, f3
        put(&mut prog, &mut a, 0x00FA_4100); // mov.s f4, f1
        put(&mut prog, &mut a, 0x00FA_4110); // abs.s f4, f1      ; (f4 = 1.0)
        put(&mut prog, &mut a, 0x00FA_4160); // neg.s f4, f1      ; f4 = -1.0
        put(&mut prog, &mut a, 0x00FA_7440); // rfr a7, f4
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(4), 0x4040_0000, "1.0 + 2.0 = 3.0");
        assert_eq!(cpu.reg(5), 0xBF80_0000, "1.0 - 2.0 = -1.0");
        assert_eq!(cpu.reg(6), 0x4000_0000, "1.0 * 2.0 = 2.0");
        assert_eq!(cpu.reg(7), 0xBF80_0000, "neg(1.0) = -1.0");
    }

    #[test]
    fn fpu_madd_msub_const() {
        // MADD_S/MSUB_S are fused (QEMU float32_muladd); CONST_S table =
        // [0.0, 1.0, 2.0, 0.5], imm = s field (QEMU translate_const_s).
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x00FA_1130); // const.s f1, 1      ; 1.0
        put(&mut prog, &mut a, 0x00FA_2330); // const.s f2, 3      ; 0.5
        put(&mut prog, &mut a, 0x00FA_3230); // const.s f3, 2      ; 2.0
        put(&mut prog, &mut a, 0x004A_1230); // madd.s f1, f2, f3  ; 1+0.5*2=2
        put(&mut prog, &mut a, 0x00FA_4140); // rfr a4, f1
        put(&mut prog, &mut a, 0x005A_5230); // msub.s f5, f2, f3  ; f5=0: 0-1
        put(&mut prog, &mut a, 0x00FA_6540); // rfr a6, f5
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(4), 0x4000_0000, "madd(1, 0.5, 2) = 2.0");
        assert_eq!(cpu.reg(6), 0xBF80_0000, "msub(0, 0.5, 2) = -1.0");
        assert_eq!(cpu.freg(2).to_bits(), 0x3F00_0000, "const.s 3 = 0.5");
        assert_eq!(cpu.freg(3).to_bits(), 0x4000_0000, "const.s 2 = 2.0");
    }

    #[test]
    fn fpu_converts() {
        // FLOAT_S/UFLOAT_S/TRUNC_S (+edges per the ISA RM TRUNC.S page:
        // +ovf/+inf/NaN -> 0x7FFFFFFF, -ovf/-inf -> 0x80000000) and the
        // UTRUNC_S edges (NaN/+ovf -> 0xFFFFFFFF, negative -> 0x80000000).
        // trunc.s ar,fs,t: op2=9; utrunc.s: op2=14.
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x0005_A022); // movi a2, 5
        put(&mut prog, &mut a, 0x00CA_1200); // float.s f1, a2, 0  ; 5.0
        put(&mut prog, &mut a, 0x00FA_7140); // rfr a7, f1
        put(&mut prog, &mut a, 0x009A_8100); // trunc.s a8, f1, 0  ; 5
        put(&mut prog, &mut a, 0x00F8_A722); // movi a2, 0x7F8
        put(&mut prog, &mut a, 0x0001_22C0); // slli a2, a2, 20   ; +inf
        put(&mut prog, &mut a, 0x00FA_1250); // wfr f1, a2
        put(&mut prog, &mut a, 0x009A_3100); // trunc.s a3, f1, 0  ; +inf -> MAX
        put(&mut prog, &mut a, 0x00EA_4100); // utrunc.s a4, f1, 0 ; +inf -> ~0
        put(&mut prog, &mut a, 0x00FA_1160); // neg.s f1, f1       ; -inf
        put(&mut prog, &mut a, 0x009A_5100); // trunc.s a5, f1, 0  ; -inf -> MIN
        put(&mut prog, &mut a, 0x00EA_6100); // utrunc.s a6, f1, 0 ; neg -> 0x80000000
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(7), 0x40A0_0000, "float(5) = 5.0");
        assert_eq!(cpu.reg(4), 0xFFFF_FFFF, "utrunc(+inf) = ~0");
        assert_eq!(cpu.reg(8), 5, "trunc(5.0) = 5");
        assert_eq!(cpu.reg(3), 0x7FFF_FFFF, "trunc(+inf) saturates");
        assert_eq!(cpu.reg(5), 0x8000_0000, "trunc(-inf) = MIN");
        assert_eq!(cpu.reg(6), 0x8000_0000, "utrunc(negative) = 0x80000000");
    }

    #[test]
    fn fpu_compares_and_cond_moves() {
        // Compares write BR bit r (QEMU translate_compare_s): ordered ops
        // false on NaN, unordered ops true, UN tests NaN-ness. MOVT/MOVF
        // test BR bit t; MOVEQZ tests AR[t] == 0.
        // oeq.s b2,f1,f2: (2<<20)|(11<<16)|(2<<12)|(1<<8)|(2<<4).
        // olt.s b3: op2=4; ult.s b4: op2=5; un.s b5: op2=1.
        // movt.s f3,f1,2: (13<<20)|(11<<16)|(3<<12)|(1<<8)|(2<<4).
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x00F8_A322); // movi a2, 0x3F8
        put(&mut prog, &mut a, 0x0001_22C0); // slli a2, a2, 20   ; 1.0
        put(&mut prog, &mut a, 0x00FA_1250); // wfr f1, a2
        put(&mut prog, &mut a, 0x00FA_2250); // wfr f2, a2        ; f2 = 1.0
        put(&mut prog, &mut a, 0x002B_2120); // oeq.s b2, f1, f2  ; true
        put(&mut prog, &mut a, 0x004B_3120); // olt.s b3, f1, f2  ; false
        put(&mut prog, &mut a, 0x00DB_3120); // movt.s f3, f1, 2  ; taken
        put(&mut prog, &mut a, 0x00FA_6340); // rfr a6, f3
        put(&mut prog, &mut a, 0x0080_A032); // movi a3, 0x80
        put(&mut prog, &mut a, 0x0001_3380); // slli a3, a3, 24   ; 0x80000000
        put(&mut prog, &mut a, 0x0020_2230); // or a2, a2, a3     ; a2 = -1.0 bits
        put(&mut prog, &mut a, 0x00FA_1250); // wfr f1, a2        ; f1 = -1.0
        put(&mut prog, &mut a, 0x004B_4120); // olt.s b4, f1, f2  ; -1 < 1 true
        put(&mut prog, &mut a, 0x001B_5120); // un.s b5, f1, f2   ; false (ordered)
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert!(cpu.br(2), "1.0 == 1.0 sets BR[2]");
        assert!(!cpu.br(3), "1.0 < 1.0 clears BR[3]");
        assert!(cpu.br(4), "-1.0 < 1.0 sets BR[4]");
        assert!(!cpu.br(5), "ordered pair clears UN bit");
        assert_eq!(cpu.reg(6), 0x3F80_0000, "movt took the move");
    }

    #[test]
    fn fpu_load_store_and_branches() {
        // LSI/SSI round-trip (RRI8 FP class op0=3, r = 0/4 selector;
        // dest/base/imm = t/s/imm8<<2) and BT/BF on a compare result
        // (b0=0x76, b1=(bool#<<4)|r with r=0 BF / 1 BT, target=pc+4+imm8).
        const DATA: u32 = 0x4000_2000;
        let mut prog = Vec::new();
        prog.push((DATA, 0x4040_0000)); // 3.0f bits
        let mut a = 0x4000_1000u32;
        // a2 = DATA (0x40002000): movi 0x400 + slli 20 -> 0x40000000,
        // then add low half via overlapping movi/or: 0x2000 =
        // movi a3,0x20... 0x20 fits: movi a3, 0x20; slli a3, a3, 8? no
        // slli-by-8: field (32-8)=24 -> 0x180. Simpler: movi a3,0x2000?
        // 0x2000 > 0x7FF, no. Use: movi a3, 0x20; slli a3, a3, 8.
        put(&mut prog, &mut a, 0x0000_A422); // movi a2, 0x400
        put(&mut prog, &mut a, 0x0001_22C0); // slli a2, a2, 20   ; 0x40000000
        put(&mut prog, &mut a, 0x0020_A032); // movi a3, 0x20
        put(&mut prog, &mut a, 0x0011_3380); // slli a3, a3, 8    ; 0x2000
        put(&mut prog, &mut a, 0x0020_2230); // or a2, a2, a3     ; 0x40002000
        put(&mut prog, &mut a, 0x0000_0213); // lsi f1, a2, 0     ; 3.0
        put(&mut prog, &mut a, 0x00FA_4140); // rfr a4, f1
        put(&mut prog, &mut a, 0x0001_4213); // ssi f1, a2, 4     ; MEM[+4] = 3.0
        put(&mut prog, &mut a, 0x0001_0223); // lsi f2, a2, 4     ; reload
        put(&mut prog, &mut a, 0x00FA_5240); // rfr a5, f2
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(2), DATA, "address materialized");
        assert_eq!(cpu.reg(4), 0x4040_0000, "lsi loaded 3.0");
        assert_eq!(cpu.reg(5), 0x4040_0000, "ssi+lsi round-trip");
    }

    #[test]
    fn fpu_bt_bf_condmove() {
        // BT/BF branch on BR bit s (b0=0x76, b1=(r<<4)|s with r=0 BF /
        // 1 BT, target = pc+4+sext8); MOVEQZ/MOVNEZ test AR[t].
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x00F8_A322); // movi a2, 0x3F8
        put(&mut prog, &mut a, 0x0001_22C0); // slli a2, a2, 20   ; 1.0
        put(&mut prog, &mut a, 0x00FA_1250); // wfr f1, a2
        put(&mut prog, &mut a, 0x00FA_2250); // wfr f2, a2
        put(&mut prog, &mut a, 0x002B_2120); // oeq.s b2, f1, f2  ; true
        put(&mut prog, &mut a, 0x0002_1276); // bt 2, +2 -> T1 (T1 = bt_pc+4+2)
        put(&mut prog, &mut a, 0x0011_A042); // movi a4, 0x11 (skipped)
        put(&mut prog, &mut a, 0x0022_A052); // T1: movi a5, 0x22
        put(&mut prog, &mut a, 0x0002_0276); // bf 2, +2 -> T3 (not taken)
        put(&mut prog, &mut a, 0x0033_A062); // movi a6, 0x33
        put(&mut prog, &mut a, 0x0044_A072); // T3: movi a7, 0x44
        put(&mut prog, &mut a, 0x00FA_5230); // const.s f5, 2     ; 2.0
        put(&mut prog, &mut a, 0x008B_4100); // moveqz.s f4, f1, a0; a0=0: move
        put(&mut prog, &mut a, 0x009B_5100); // movnez.s f5, f1, a0; no move
        put(&mut prog, &mut a, 0x00FA_8440); // rfr a8, f4
        put(&mut prog, &mut a, 0x00FA_9540); // rfr a9, f5
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(4), 0, "bt taken skips the marker");
        assert_eq!(cpu.reg(5), 0x22, "bt lands on T1");
        assert_eq!(cpu.reg(6), 0x33, "bf not taken falls through");
        assert_eq!(cpu.reg(7), 0x44, "reaches T3");
        assert_eq!(cpu.reg(8), 0x3F80_0000, "moveqz moved (a0 == 0)");
        assert_eq!(cpu.reg(9), 0x4000_0000, "movnez kept 2.0 (a0 == 0)");
    }

    #[test]
    fn fpu_ar_movf_movt_test_br_bits() {
        // AR MOVF/MOVT move on BR[bt] clear/set (ISA RM Boolean Option),
        // NOT on an AR bit. BR2=1 (oeq.s true) with a2 bit2=0, BR3=0
        // (olt.s false) with a3 bit3=1, so any AR-bit reading fails.
        // Encodings: FP compare op0=0 op1=11 op2=2/4 (r=BR bit, s/t=FR);
        // AR movf/movt op0=0 op1=3 op2=12/13 (r=dest, s=src, t=bit).
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x00F8_A322); // movi a2, 0x3F8
        put(&mut prog, &mut a, 0x0001_22C0); // slli a2, a2, 20   ; 1.0 (bit2=0)
        put(&mut prog, &mut a, 0x00FA_1250); // wfr f1, a2
        put(&mut prog, &mut a, 0x00FA_2250); // wfr f2, a2
        put(&mut prog, &mut a, 0x002B_2120); // oeq.s b2, f1, f2  ; BR2=1
        put(&mut prog, &mut a, 0x004B_3120); // olt.s b3, f1, f2  ; BR3=0
        put(&mut prog, &mut a, 0x0008_A032); // movi a3, 8        ; bit3=1
        put(&mut prog, &mut a, 0x0055_A052); // movi a5, 0x55
        put(&mut prog, &mut a, 0x00AA_A062); // movi a6, 0xAA
        put(&mut prog, &mut a, 0x00BB_A072); // movi a7, 0xBB
        put(&mut prog, &mut a, 0x00DD_A082); // movi a8, 0xDD
        put(&mut prog, &mut a, 0x00CC_A092); // movi a9, 0xCC
        put(&mut prog, &mut a, 0x00C3_6520); // movf a6, a5, 2    ; skip
        put(&mut prog, &mut a, 0x00D3_7520); // movt a7, a5, 2    ; move
        put(&mut prog, &mut a, 0x00C3_8530); // movf a8, a5, 3    ; move
        put(&mut prog, &mut a, 0x00D3_9530); // movt a9, a5, 3    ; skip
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(6), 0xAA, "movf skips on BR2=1");
        assert_eq!(cpu.reg(7), 0x55, "movt takes on BR2=1");
        assert_eq!(cpu.reg(8), 0x55, "movf takes on BR3=0");
        assert_eq!(cpu.reg(9), 0xCC, "movt skips on BR3=0");
    }

    #[test]
    fn fpu_floor_ceil_round_ufloat() {
        // FLOOR/CEIL/ROUND_S (op2=10/11/8) and UFLOAT_S (op2=13).
        // 2.5 = 0x40200000 via movi 0x402 + slli 20.
        let mut prog = Vec::new();
        let mut a = 0x4000_1000u32;
        put(&mut prog, &mut a, 0x0002_A432); // movi a3, 0x402
        put(&mut prog, &mut a, 0x0001_33C0); // slli a3, a3, 20   ; 2.5
        put(&mut prog, &mut a, 0x00FA_1350); // wfr f1, a3
        put(&mut prog, &mut a, 0x00AA_9100); // floor.s a9, f1, 0 ; 2
        put(&mut prog, &mut a, 0x00BA_A100); // ceil.s a10, f1, 0 ; 3
        put(&mut prog, &mut a, 0x008A_B100); // round.s a11, f1, 0; 2 (ties-even)
        put(&mut prog, &mut a, 0x0005_A022); // movi a2, 5
        put(&mut prog, &mut a, 0x00DA_2200); // ufloat.s f2, a2, 0; 5.0
        put(&mut prog, &mut a, 0x00FA_6240); // rfr a6, f2
        let end = a;

        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, end);
        assert_eq!(cpu.reg(9), 2, "floor(2.5) = 2");
        assert_eq!(cpu.reg(10), 3, "ceil(2.5) = 3");
        assert_eq!(cpu.reg(11), 2, "round(2.5) = 2 ties-even");
        assert_eq!(cpu.reg(6), 0x40A0_0000, "ufloat(5) = 5.0");
    }

    // libgcc software-divide/square-root sequences through the FPU.
    // The divide/square-root step ops (DIV0/NEXP01/MADDN/ADDEXP/ADDEXPM/
    // DIVN/SQRT0/RECIP0/RSQRT0) are NOPs (QEMU parity, commit f8c6137);
    // the sequences collapse correctly because MKDADJ/MKSADJ perform the
    // true divide/sqrt and ADDEXPM moves it to the final register that
    // DIVN leaves untouched.  Encodings below reproduce real toolchain
    // bytes exactly (e.g. wfr f1,a2 = 0xFA1250, divn.s f0,f1,f3 =
    // 0x7A0130, maddn.s f3,f2,f2 = 0x6A3220, all matching disassembled
    // ESP32-S3 libgcc/firmware).  Expected results are computed by host
    // f32 division/sqrt (correctly rounded IEEE).
    //
    // Machine-generated words (op1=10 arith class op2: ADD=0 MADD=4
    // MSUB=5 MADDN=6 DIVN=7; op2=15 two-op class t-ext: MOV=0 CONST=3
    // NEG=6 DIV0=7 SQRT0=9 NEXP01=11 MKSADJ=12 MKDADJ=13 ADDEXP=14
    // ADDEXPM=15; WFR t=5 RFR t=4; word=(op2<<20)|(op1<<16)|(r<<12)|
    // (s<<8)|(t<<4); LSI/SSI op0=3 class; l32i/or/movi/slli per the
    // header notes above fpu_moves_and_arith).
    #[allow(dead_code)]
    fn fp_seq_run(x_bits: u32, y_bits: u32, seq: &[u32]) -> u32 {
        const D0: u32 = 0x4000_2000;
        let mut prog = alloc::vec![(D0, x_bits), (D0 + 4, y_bits)];
        let mut a = 0x4000_1000u32;
        let emit = |w: u32, p: &mut Vec<(u32, u32)>, a: &mut u32| {
            p.push((*a, w));
            *a += crate::generated::insn_len((w & 0xff) as u8);
        };
        emit(0x0000_A422, &mut prog, &mut a); // movi a2, 0x400
        emit(0x0001_22C0, &mut prog, &mut a); // slli a2, a2, 20
        emit(0x0020_A032, &mut prog, &mut a); // movi a3, 0x20
        emit(0x0011_3380, &mut prog, &mut a); // slli a3, a3, 8
        emit(0x0020_2230, &mut prog, &mut a); // or a2, a2, a3 -> D0
        emit(0x0020_4200, &mut prog, &mut a); // or a4, a2, a0 -> base copy
        emit(0x0000_2422, &mut prog, &mut a); // l32i a2, a4, 0
        emit(0x0001_2432, &mut prog, &mut a); // l32i a3, a4, 4
        for w in seq {
            emit(*w, &mut prog, &mut a);
        }
        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        run(&mut cpu, &mut bus, a);
        cpu.reg(4)
    }

    #[test]
    fn fpu_libgcc_div_sequence() {
        // Exact libgcc __divsf3 instruction sequence (wfr/div0/nexp01/
        // const/maddn/mov/neg/mkdadj/addexpm/addexp/divn/rfr).
        const SEQ: [u32; 28] = [
            0x00FA1250, 0x00FA2350, 0x00FA3270, 0x00FA42B0, 0x00FA5130, 0x006A5430, 0x00FA6300,
            0x00FA7200, 0x00FA21B0, 0x006A6560, 0x00FA5130, 0x00FA0030, 0x00FA8260, 0x006A5460,
            0x006A0830, 0x00FA71D0, 0x006A6560, 0x006A8400, 0x00FA3130, 0x006A3460, 0x006A0860,
            0x00FA2260, 0x006A6360, 0x006A2400, 0x00FA07F0, 0x00FA67E0, 0x007A0260, 0x00FA4040,
        ];
        for (x, y) in [
            (1.0f32, 3.0f32),
            (2.0, 3.0),
            (7.0, 2.0),
            (1.0, 10.0),
            (-1.0, 3.0),
            (1.0, 0.0),
            (-1.0, 0.0),
            (0.0, 5.0),
            (100.0, 0.25),
            (0.1, 0.3),
        ] {
            let got = fp_seq_run(x.to_bits(), y.to_bits(), &SEQ);
            assert_eq!(got, (x / y).to_bits(), "{x}/{y}");
        }
        // 0/0 is NaN (payload-insensitive).
        let nan = fp_seq_run(0.0f32.to_bits(), 0.0f32.to_bits(), &SEQ);
        assert!(f32::from_bits(nan).is_nan(), "0/0 = NaN, got {nan:#x}");
    }

    #[test]
    fn fpu_libgcc_sqrt_sequence() {
        // Exact libgcc __ieee754_sqrtf sequence (second operand ignored).
        const SEQ: [u32; 31] = [
            0x00FA1250, 0x00FA2190, 0x00FA3030, 0x006A3220, 0x00FA41B0, 0x00FA0330, 0x00FA40E0,
            0x006A0340, 0x00FA31B0, 0x00FA5360, 0x006A2020, 0x00FA0030, 0x00FA6030, 0x00FA7030,
            0x006A0520, 0x006A6240, 0x00FA4330, 0x006A7420, 0x006A3000, 0x006A4620, 0x00FA2760,
            0x006A0320, 0x006A7470, 0x00FA21C0, 0x00FA11B0, 0x006A1000, 0x00FA3760, 0x00FA02F0,
            0x00FA32E0, 0x007A0130, 0x00FA4040,
        ];
        for x in [2.0f32, 0.25, 1.0, 100.0, 0.0, f32::INFINITY, 0.5] {
            let got = fp_seq_run(x.to_bits(), 0, &SEQ);
            assert_eq!(got, x.sqrt().to_bits(), "sqrt({x})");
        }
        let nan = fp_seq_run((-1.0f32).to_bits(), 0, &SEQ);
        assert!(f32::from_bits(nan).is_nan(), "sqrt(-1) = NaN");
    }

    #[test]
    fn ee_dsp_instruction_is_unimplemented() {
        // An unimplemented TIE/DSP (`ee.*`) instruction must halt the core with
        // StepResult::Unimplemented rather than mis-executing or raising a
        // spurious illegal-instruction exception.
        // ee.srs.accx a1, a2, 0 = 24 11 7e: decodes but has no executor
        // (its shift-AR operand is not encoded), so it still traps loud.
        let prog = [(0x4000_1000u32, 0x007E_1124u32)];
        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        let r = cpu.step(&mut bus);
        assert_eq!(r, StepResult::Unimplemented("opcode"));
    }

    /// Run one ee.* word at 0x4000_1000 after `setup`, return cpu.
    fn ee_run1(word: u32, setup: impl FnOnce(&mut Cpu)) -> Cpu {
        let mut bus = RamBus::load(&[(0x4000_1000, word)]);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        setup(&mut cpu);
        run(&mut cpu, &mut bus, 0x4000_1003);
        cpu
    }

    /// Run one ee.* word (3- or 4-byte `len`) with data-memory `setup`.
    fn ee_run_mem(word: u32, len: u32, setup: impl FnOnce(&mut Cpu, &mut RamBus)) -> (Cpu, RamBus) {
        let mut bus = RamBus::load(&[(0x4000_1000, word)]);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        setup(&mut cpu, &mut bus);
        run(&mut cpu, &mut bus, 0x4000_1000 + len);
        (cpu, bus)
    }

    #[test]
    fn ee_vadds_s8_saturates_asymmetric() {
        // ee.vadds.s8 q2, q0, q1 = 84 08 9e
        let cpu = ee_run1(0x009E_0884, |c| {
            c.qregs[0] = [100; 16];
            c.qregs[1] = [50; 16];
        });
        assert_eq!(cpu.qregs[2], [127; 16]);
        // Negative overflow clamps to -0x7f (-127), NOT -128.
        let cpu = ee_run1(0x009E_0884, |c| {
            c.qregs[0] = [156; 16]; // -100
            c.qregs[1] = [206; 16]; // -50
        });
        assert_eq!(cpu.qregs[2], [129; 16]); // -127
    }

    #[test]
    fn ee_vsubs_s16_saturates() {
        // ee.vsubs.s16 q3, q1, q0 = d4 a1 9e
        let cpu = ee_run1(0x009E_A1D4, |c| {
            c.qregs[1] = [
                0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03,
                0xE8, 0x03,
            ]; // 1000 x8
            c.qregs[0] = [
                0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01, 0x2C, 0x01,
                0x2C, 0x01,
            ]; // 300 x8
        });
        for i in 0..8 {
            let lo = cpu.qregs[3][2 * i];
            let hi = cpu.qregs[3][2 * i + 1];
            assert_eq!((lo as u16) | ((hi as u16) << 8), 700, "lane {i}");
        }
        // -30000 - 3000 = -33000 clamps to -32767 (0x8001).
        let cpu = ee_run1(0x009E_A1D4, |c| {
            c.qregs[1] = [
                0xD0, 0x8A, 0xD0, 0x8A, 0xD0, 0x8A, 0xD0, 0x8A, 0xD0, 0x8A, 0xD0, 0x8A, 0xD0, 0x8A,
                0xD0, 0x8A,
            ]; // -30000
            c.qregs[0] = [
                0xB8, 0x0B, 0xB8, 0x0B, 0xB8, 0x0B, 0xB8, 0x0B, 0xB8, 0x0B, 0xB8, 0x0B, 0xB8, 0x0B,
                0xB8, 0x0B,
            ]; // 3000
        });
        for i in 0..8 {
            let lo = cpu.qregs[3][2 * i];
            let hi = cpu.qregs[3][2 * i + 1];
            assert_eq!((lo as u16) | ((hi as u16) << 8), 0x8001, "lane {i}");
        }
    }

    #[test]
    fn ee_vmul_s8_shifts_by_sar() {
        // ee.vmul.s8 q0, q1, q2 = 94 31 8e, SAR = 2: (8*4)>>2 = 8.
        let cpu = ee_run1(0x008E_3194, |c| {
            c.qregs[1] = [8; 16];
            c.qregs[2] = [4; 16];
            c.set_sreg(crate::cpu::SR_SAR, 2);
        });
        assert_eq!(cpu.qregs[0], [8; 16]);
    }

    #[test]
    fn ee_vmin_vmax_select_lanes() {
        // ee.vmin.s32 q1, q0, q3 = 64 b8 8e
        let cpu = ee_run1(0x008E_B864, |c| {
            // q0 lanes [5, -1, 100, 156], q3 lanes [3, 7, -200, 0].
            c.qregs[0] = [
                5, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 100, 0, 0, 0, 156, 0, 0, 0,
            ];
            c.qregs[3] = [3, 0, 0, 0, 7, 0, 0, 0, 0x38, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0];
        });
        let w = |q: &[u8; 16], i: usize| {
            u32::from_le_bytes([q[4 * i], q[4 * i + 1], q[4 * i + 2], q[4 * i + 3]])
        };
        assert_eq!(w(&cpu.qregs[1], 0), 3);
        assert_eq!(w(&cpu.qregs[1], 1) as i32, -1);
        assert_eq!(w(&cpu.qregs[1], 2) as i32, -200);
        assert_eq!(w(&cpu.qregs[1], 3), 0);
        // ee.vmax.s16 q2, q1, q0 = 24 21 9e: max(-5, 3) = 3.
        let cpu = ee_run1(0x009E_2124, |c| {
            c.qregs[1] = [
                0xFB, 0xFF, 0xFB, 0xFF, 0xFB, 0xFF, 0xFB, 0xFF, 0xFB, 0xFF, 0xFB, 0xFF, 0xFB, 0xFF,
                0xFB, 0xFF,
            ]; // -5
            c.qregs[0] = [3, 0, 3, 0, 3, 0, 3, 0, 3, 0, 3, 0, 3, 0, 3, 0]; // 3
        });
        for i in 0..8 {
            assert_eq!(cpu.qregs[2][2 * i], 3);
            assert_eq!(cpu.qregs[2][2 * i + 1], 0);
        }
    }

    #[test]
    fn ee_vcmp_produces_lane_masks() {
        // ee.vcmp.eq.s8 q0, q1, q1 (same reg => all equal) = b4 09 8e
        let cpu = ee_run1(0x008E_09B4, |c| {
            c.qregs[1] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        });
        assert_eq!(cpu.qregs[0], [0xFF; 16]);
        // ee.vcmp.lt.s16 q2, q0, q1: 1 < 2 => all set = f4 08 9e
        let cpu = ee_run1(0x009E_08F4, |c| {
            c.qregs[0] = [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0];
            c.qregs[1] = [2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0];
        });
        assert_eq!(cpu.qregs[2], [0xFF; 16]);
        // ee.vcmp.gt.s8 q2, q0, q1: 6 > 5 => set = e4 08 9e
        let cpu = ee_run1(0x009E_08E4, |c| {
            c.qregs[0] = [6; 16];
            c.qregs[1] = [5; 16];
        });
        assert_eq!(cpu.qregs[2], [0xFF; 16]);
    }

    #[test]
    fn ee_logic_ops() {
        // andq/orq/xorq q0,q1,q2; notq q0,q1.
        let (a, b) = ([0xF0; 16], [0xCC; 16]);
        let cpu = ee_run1(0x00CD_3414, |c| {
            c.qregs[1] = a;
            c.qregs[2] = b;
        });
        assert_eq!(cpu.qregs[0], [0xC0; 16]);
        let cpu = ee_run1(0x00CD_7414, |c| {
            c.qregs[1] = a;
            c.qregs[2] = b;
        });
        assert_eq!(cpu.qregs[0], [0xFC; 16]);
        let cpu = ee_run1(0x00CD_3514, |c| {
            c.qregs[1] = a;
            c.qregs[2] = b;
        });
        assert_eq!(cpu.qregs[0], [0x3C; 16]);
        let cpu = ee_run1(0x00CD_7F14, |c| {
            c.qregs[1] = a;
        });
        assert_eq!(cpu.qregs[0], [0x0F; 16]);
    }

    #[test]
    fn ee_vzip_vunzip_round_trip() {
        // ee.vzip.16 q0, q5 = b4 83 ec
        let mut q0 = [0u8; 16];
        let mut q5 = [0u8; 16];
        for i in 0..16 {
            q0[i] = i as u8;
            q5[i] = 100 + i as u8;
        }
        let cpu = ee_run1(0x00EC_83B4, |c| {
            c.qregs[0] = q0;
            c.qregs[5] = q5;
        });
        // vzip.16 interleaves 16-bit units: q0 gets sa.u16[0..4]/sb.u16[0..4].
        for i in 0..4 {
            assert_eq!(cpu.qregs[0][4 * i], i as u8 * 2);
            assert_eq!(cpu.qregs[0][4 * i + 1], i as u8 * 2 + 1);
            assert_eq!(cpu.qregs[0][4 * i + 2], 100 + i as u8 * 2);
            assert_eq!(cpu.qregs[0][4 * i + 3], 100 + i as u8 * 2 + 1);
            assert_eq!(cpu.qregs[5][4 * i], 8 + i as u8 * 2);
            assert_eq!(cpu.qregs[5][4 * i + 1], 8 + i as u8 * 2 + 1);
            assert_eq!(cpu.qregs[5][4 * i + 2], 108 + i as u8 * 2);
            assert_eq!(cpu.qregs[5][4 * i + 3], 108 + i as u8 * 2 + 1);
        }
        // ee.vunzip.8 q1, q2 = a4 13 dc inverts a byte interleave.
        let cpu = ee_run1(0x00DC_13A4, |c| {
            c.qregs[1] = [
                0, 100, 1, 101, 2, 102, 3, 103, 4, 104, 5, 105, 6, 106, 7, 107,
            ];
            c.qregs[2] = [
                8, 108, 9, 109, 10, 110, 11, 111, 12, 112, 13, 113, 14, 114, 15, 115,
            ];
        });
        for i in 0..16 {
            assert_eq!(cpu.qregs[1][i], i as u8, "q1[{i}]");
            assert_eq!(cpu.qregs[2][i], 100 + i as u8, "q2[{i}]");
        }
    }

    #[test]
    fn ee_cmul_s16_complex_product() {
        // ee.cmul.s16 q0, q1, q2, 1 = 14 11 8e: (3+4i)(1+0i)>>1 = (1,2).
        let cpu = ee_run1(0x008E_1114, |c| {
            c.qregs[1] = [3, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            c.qregs[2] = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        });
        assert_eq!(cpu.qregs[0][0], 1);
        assert_eq!(cpu.qregs[0][1], 0);
        assert_eq!(cpu.qregs[0][2], 2);
        assert_eq!(cpu.qregs[0][3], 0);
        for i in 4..16 {
            assert_eq!(cpu.qregs[0][i], 0);
        }
    }

    #[test]
    fn ee_zero_and_movi_move_data() {
        // ee.zero.q q3 = a4 ff dd
        let cpu = ee_run1(0x00DD_FFA4, |c| {
            c.qregs[3] = [0xAA; 16];
        });
        assert_eq!(cpu.qregs[3], [0; 16]);
        // ee.movi.32.q q1, a2, 1 = 24 b6 cd
        let cpu = ee_run1(0x00CD_B624, |c| {
            c.set_reg(2, 0xDEAD_BEEF);
            c.qregs[1] = [0; 16];
        });
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][4..8].try_into().unwrap()),
            0xDEAD_BEEF
        );
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][0..4].try_into().unwrap()),
            0
        );
    }

    #[test]
    fn ee_vrelu_vprelu_predicated_scale() {
        // ee.vrelu.s8 q1, a2, a3 = 34 d2 cd: lanes (-4,5,-128,0), mult 2, shr 1.
        let cpu = ee_run1(0x00CD_D234, |c| {
            c.qregs[1] = [252, 5, 128, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]; // -4,5,-128,0,...
            c.set_reg(2, 2);
            c.set_reg(3, 1);
        });
        assert_eq!(cpu.qregs[1][0], 252); // -4 unchanged
        assert_eq!(cpu.qregs[1][1], 5);
        assert_eq!(cpu.qregs[1][2], 128); // -128 unchanged
        assert_eq!(cpu.qregs[1][3], 0);
        // ee.vprelu.s8 q1, q2, q3, a4 = 44 ba 8c: q2=(-4,6), q3=(3,3), shr 1.
        let cpu = ee_run1(0x008C_BA44, |c| {
            c.qregs[2] = [252, 6, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14];
            c.qregs[3] = [3; 16];
            c.set_reg(4, 1);
        });
        assert_eq!(cpu.qregs[1][0], 250); // (-4*3)>>1 = -6
        assert_eq!(cpu.qregs[1][1], 6); // positive passes through
    }

    #[test]
    fn ee_vsl_32_shifts_left() {
        // ee.vsl.32 q1, q2 = 14 3f dd: shift LEFT by SAR.
        let cpu = ee_run1(0x00DD_3F14, |c| {
            c.qregs[2] = [0, 0, 0, 0x80, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // 0x80000000, 1
            c.set_sreg(crate::cpu::SR_SAR, 1);
        });
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][0..4].try_into().unwrap()),
            0
        );
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][4..8].try_into().unwrap()),
            2
        );
    }

    #[test]
    fn ee_vsmulas_s8_accumulates_scalar_product() {
        // ee.vsmulas.s8.qacc q0, q1, 0 = 44 08 8e: scalar q1.s8[0] = 2.
        let cpu = ee_run1(0x008E_0844, |c| {
            c.qregs[0] = [1; 16];
            c.qregs[1] = [2; 16];
        });
        for acc in 0..2 {
            for i in 0..8 {
                assert_eq!(crate::ee::acc_s20(&cpu, acc, i), 2, "acc{acc}[{i}]");
            }
        }
    }

    #[test]
    fn ee_srcmb_s16_shifts_qacc_into_q() {
        // ee.srcmb.s16.qacc q1, a2, 1 = 24 f6 cd: QACC = 0x1000, shr 4.
        let cpu = ee_run1(0x00CD_F624, |c| {
            for acc in 0..2 {
                for i in 0..4 {
                    crate::ee::set_acc_s40(c, acc, i, 0x1000);
                }
            }
            c.set_reg(2, 4);
        });
        for i in 0..8 {
            let v = (cpu.qregs[1][2 * i] as u16) | ((cpu.qregs[1][2 * i + 1] as u16) << 8);
            assert_eq!(v, 0x100, "lane {i}");
        }
        for acc in 0..2 {
            for i in 0..4 {
                assert_eq!(crate::ee::acc_s40(&cpu, acc, i), 0x100);
            }
        }
    }

    #[test]
    fn ee_vmulas_accumulates_dot_products() {
        // ee.vmulas.s8.accx q0, q1 = c4 08 1a: dot(ones, ones) = 16.
        let cpu = ee_run1(0x001A_08C4, |c| {
            c.qregs[0] = [1; 16];
            c.qregs[1] = [1; 16];
        });
        assert_eq!(cpu.accx, 16);
        // ee.vmulas.s16.qacc q2, q3 = 84 3a 1a: 100*100 x4 per half.
        let cpu = ee_run1(0x001A_3A84, |c| {
            c.qregs[2] = [
                100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0,
            ];
            c.qregs[3] = [
                100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0,
            ];
        });
        for acc in 0..2 {
            for i in 0..4 {
                assert_eq!(crate::ee::acc_s40(&cpu, acc, i), 10_000, "acc{acc}[{i}]");
            }
        }
    }

    #[test]
    fn ee_bitrev_folds_reversed_indices() {
        // ee.bitrev q1, a2 = 24 fb cd, fft_width 3, base 0.
        let cpu = ee_run1(0x00CD_FB24, |c| {
            c.fft_width = 3;
            c.set_reg(2, 0);
        });
        assert_eq!(
            cpu.qregs[1],
            [0, 0, 4, 0, 2, 0, 6, 0, 4, 0, 5, 0, 6, 0, 7, 0]
        );
        assert_eq!(cpu.reg(2), 8);
    }

    #[test]
    fn ee_vmulas_fused_mac_then_load() {
        // ee.vmulas.s8.accx.ld.ip q0, a2, 16, q3, q4 = 2e e1 02 f0:
        // MAC first (accx = 1*2*16 = 32), then load, then +16.
        let (cpu, _) = ee_run_mem(0xF002_E12E, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.qregs[3] = [1; 16];
            c.qregs[4] = [2; 16];
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 3);
            }
        });
        assert_eq!(cpu.accx, 32);
        assert_eq!(cpu.qregs[0], [3; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2010);
        // ee.vmulas.s8.accx.ld.xp q0, a2, a3, q3, q4 = 2e e3 12 f0.
        let (cpu, _) = ee_run_mem(0xF012_E32E, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.set_reg(3, 32);
            c.qregs[3] = [1; 16];
            c.qregs[4] = [2; 16];
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 5);
            }
        });
        assert_eq!(cpu.accx, 32);
        assert_eq!(cpu.qregs[0], [5; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2020);
        // ee.vmulas.s16.qacc.ld.ip q0, a2, 16, q3, q4 = 2e e1 01 f0.
        let (cpu, _) = ee_run_mem(0xF001_E12E, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            for i in 0..8 {
                c.qregs[3][2 * i] = 100;
                c.qregs[3][2 * i + 1] = 0;
                c.qregs[4][2 * i] = 100;
                c.qregs[4][2 * i + 1] = 0;
                b.write8(0x4000_2000 + i as u32, 7);
            }
            for i in 8..16 {
                b.write8(0x4000_2000 + i as u32, 7);
            }
        });
        for acc in 0..2 {
            for i in 0..4 {
                assert_eq!(crate::ee::acc_s40(&cpu, acc, i), 10_000, "acc{acc}[{i}]");
            }
        }
        assert_eq!(cpu.qregs[0], [7; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_vmulas_ldbc_mac_then_broadcast() {
        // ee.vmulas.s8.qacc.ldbc.incp q0, a2, q3, q4 = 24 43 a7:
        // MAC first (lanes = 1*2 = 2), then broadcast mem8, then +1.
        let (cpu, _) = ee_run_mem(0xA74324, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.qregs[3] = [1; 16];
            c.qregs[4] = [2; 16];
            b.write8(0x4000_2000, 9);
        });
        for acc in 0..2 {
            for i in 0..8 {
                assert_eq!(crate::ee::acc_s20(&cpu, acc, i), 2, "acc{acc}[{i}]");
            }
        }
        assert_eq!(cpu.qregs[0], [9; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2001);
    }

    #[test]
    fn ee_mov_qacc_widens_lanes() {
        // ee.mov.s16.qacc q2 = 24 7f dd with q2 = -1 lanes.
        let cpu = ee_run1(0x00DD_7F24, |c| {
            c.qregs[2] = [0xFF; 16];
        });
        for acc in 0..2 {
            for i in 0..4 {
                assert_eq!(crate::ee::acc_s40(&cpu, acc, i), -1, "acc{acc}[{i}]");
            }
        }
        // ee.mov.u8.qacc q2 = 74 7f dd: zero-extended bytes.
        let cpu = ee_run1(0x00DD_7F74, |c| {
            c.qregs[2] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        });
        for i in 0..8 {
            assert_eq!(crate::ee::acc_s20(&cpu, 0, i), (i + 1) as i64);
            assert_eq!(crate::ee::acc_s20(&cpu, 1, i), (i + 9) as i64);
        }
    }

    #[test]
    fn ee_gpio_latch_round_trip() {
        // ee.set_bit_gpio_out 3 = 34 40 75; wr_mask; get_gpio_in.
        let cpu = ee_run1(0x0075_4034, |_| {});
        assert_eq!(cpu.tie_gpio, 3);
        // ee.wr_mask_gpio_out a2, a3 = 24 43 72: gpio = (gpio & ~mask) | (data & mask).
        let cpu = ee_run1(0x0072_4324, |c| {
            c.tie_gpio = 3;
            c.set_reg(2, 0xF);
            c.set_reg(3, 0xA);
        });
        assert_eq!(cpu.tie_gpio, 0xA);
        // ee.get_gpio_in a2 = 24 08 65 reads back the latch.
        let cpu = ee_run1(0x0065_0824, |c| {
            c.tie_gpio = 0xA;
        });
        assert_eq!(cpu.reg(2), 0xA);
    }

    #[test]
    fn ee_slci_srci_slide_pairs() {
        // ee.slci.2q q0, q1, 3 = 34 16 cc (shift 4 left).
        let (cpu, _) = ee_run_mem(0xCC1634, 3, |c, _| {
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[0] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
        });
        assert_eq!(
            cpu.qregs[1],
            [0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        );
        assert_eq!(
            cpu.qregs[0],
            [
                12, 13, 14, 15, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111
            ]
        );
        // ee.srci.2q q0, q1, 3 = 34 1a cc (shift 4 right).
        let (cpu, _) = ee_run_mem(0xCC1A34, 3, |c, _| {
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[0] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
        });
        assert_eq!(
            cpu.qregs[1],
            [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 100, 101, 102, 103]
        );
        assert_eq!(
            cpu.qregs[0],
            [
                104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 0, 0, 0, 0
            ]
        );
    }

    #[test]
    fn ee_src_q_funnel_shifts() {
        // ee.src.q q0, q1, q2 = 04 13 dc, sar_byte 4.
        let (cpu, _) = ee_run_mem(0xDC1304, 3, |c, _| {
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[2] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
            c.sar_byte = 4;
        });
        assert_eq!(
            cpu.qregs[0],
            [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 100, 101, 102, 103]
        );
        // ee.src.q.qup q0, q1, q2 = 04 17 dc also copies q2 over q1.
        let (cpu, _) = ee_run_mem(0xDC1704, 3, |c, _| {
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[2] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
            c.sar_byte = 4;
        });
        assert_eq!(
            cpu.qregs[0],
            [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 100, 101, 102, 103]
        );
        assert_eq!(
            cpu.qregs[1],
            [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115
            ]
        );
    }

    #[test]
    fn ee_vldbc_broadcasts_memory() {
        // ee.vldbc.8 q2, a3 = 34 3b dd.
        let (cpu, _) = ee_run_mem(0xDD3B34, 3, |c, b| {
            c.set_reg(3, 0x4000_2000);
            b.write8(0x4000_2000, 0xAB);
        });
        assert_eq!(cpu.qregs[2], [0xAB; 16]);
        // ee.vldbc.8.ip q1, a2, 16 = 24 90 c5.
        let (cpu, _) = ee_run_mem(0xC59024, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            b.write8(0x4000_2000, 0xCD);
        });
        assert_eq!(cpu.qregs[1], [0xCD; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_vld_vst_128_round_trip() {
        // ee.vld.128.ip q4, a2, 16 = 24 01 a3.
        let (cpu, _) = ee_run_mem(0xA30124, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, i as u32 + 1);
            }
        });
        for i in 0..16 {
            assert_eq!(cpu.qregs[4][i], i as u8 + 1);
        }
        assert_eq!(cpu.reg(2), 0x4000_2010);
        // ee.vst.128.ip q4, a2, 16 = 24 01 aa stores it back elsewhere.
        let (cpu, mut bus) = ee_run_mem(0xAA0124, 3, |c, _| {
            c.set_reg(2, 0x4000_3000);
            for i in 0..16 {
                c.qregs[4][i] = 16 - i as u8;
            }
        });
        for i in 0..16 {
            assert_eq!(bus.read8(0x4000_3000 + i as u32), 16 - i as u32);
        }
        assert_eq!(cpu.reg(2), 0x4000_3010);
        // ee.vld.128.xp q1, a2, a3 = 24 a3 8d adds AR.
        let (cpu, _) = ee_run_mem(0x8DA324, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.set_reg(3, 32);
            b.write32(0x4000_2000, 0x1111_1111);
            b.write32(0x4000_2004, 0x2222_2222);
            b.write32(0x4000_2008, 0x3333_3333);
            b.write32(0x4000_200C, 0x4444_4444);
        });
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][0..4].try_into().unwrap()),
            0x1111_1111
        );
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][12..16].try_into().unwrap()),
            0x4444_4444
        );
        assert_eq!(cpu.reg(2), 0x4000_2020);
    }

    #[test]
    fn ee_vld_vst_64_halves() {
        // ee.vld.h.64.ip q1, a2, 8 = 24 81 88 loads the high half only.
        let (cpu, _) = ee_run_mem(0x888124, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.qregs[1] = [0xAA; 16];
            b.write32(0x4000_2000, 0x1111_1111);
            b.write32(0x4000_2004, 0x2222_2222);
        });
        assert_eq!(&cpu.qregs[1][..8], &[0xAA; 8]);
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][8..12].try_into().unwrap()),
            0x1111_1111
        );
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][12..16].try_into().unwrap()),
            0x2222_2222
        );
        assert_eq!(cpu.reg(2), 0x4000_2008);
        // ee.vst.l.64.ip q1, a2, 8 = 24 81 84 stores the low half only.
        let (cpu, mut bus) = ee_run_mem(0x848124, 3, |c, _| {
            c.set_reg(2, 0x4000_3000);
            c.qregs[1] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
        });
        for i in 0..8 {
            assert_eq!(bus.read8(0x4000_3000 + i as u32), i as u32 + 1);
        }
        assert_eq!(cpu.reg(2), 0x4000_3008);
    }

    #[test]
    fn ee_ldxq_stxq_indexed_lanes() {
        // ee.ldxq.32 q1, q2, a3, 0, 1 = 3e 8d f1 e0: offset 5*4-4 = 16.
        let (cpu, _) = ee_run_mem(0xE0F1_8D3E, 4, |c, b| {
            c.set_reg(3, 0x4000_2000);
            c.qregs[2] = [0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // s16[1] = 5
            b.write32(0x4000_2010, 0x1234_5678);
        });
        assert_eq!(
            u32::from_le_bytes(cpu.qregs[1][0..4].try_into().unwrap()),
            0x1234_5678
        );
        // ee.stxq.32 q0, q1, a2, 3, 2 = 2e 50 08 e7: offset 4*4-4 = 12.
        let (_, mut bus) = ee_run_mem(0xE708_502E, 4, |c, _| {
            c.set_reg(2, 0x4000_2000);
            c.qregs[0] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xDD, 0xCC, 0xBB, 0xAA]; // u32[3]
            c.qregs[1] = [0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // s16[2] = 4
        });
        assert_eq!(bus.read32(0x4000_200C), 0xAABB_CCDD);
    }

    #[test]
    fn ee_accx_spill_fill_masks_44_bits() {
        // ee.st.accx.ip a2, 8 = 24 01 02.
        let (_, mut bus) = ee_run_mem(0x020124, 3, |c, _| {
            c.set_reg(2, 0x4000_2000);
            c.accx = 0x0123_4567_89AB;
        });
        assert_eq!(bus.read32(0x4000_2000), 0x4567_89AB);
        assert_eq!(bus.read32(0x4000_2004), 0x0000_0123);
        // ee.ld.accx.ip a4, 0 = 44 00 0e masks to 44 bits (no postupdate).
        let (cpu, _) = ee_run_mem(0x0E0044, 3, |c, b| {
            c.set_reg(4, 0x4000_2000);
            b.write32(0x4000_2000, 0xFFFF_FFFF);
            b.write32(0x4000_2004, 0xFFFF_FFFF);
        });
        assert_eq!(cpu.accx, 0x0FFF_FFFF_FFFF);
        assert_eq!(cpu.reg(4), 0x4000_2000);
    }

    #[test]
    fn ee_qacc_spill_fill() {
        // ee.ld.qacc_h.h.32.ip a4, 4 = 44 01 1e loads the top word.
        let (cpu, _) = ee_run_mem(0x1E0144, 3, |c, b| {
            c.set_reg(4, 0x4000_2000);
            b.write32(0x4000_2000, 0xDEAD_BEEF);
        });
        assert_eq!(crate::ee::qacc_word(&cpu.accq[1], 4), 0xDEAD_BEEF);
        assert_eq!(cpu.reg(4), 0x4000_2004);
        // ee.st.qacc_h.h.32.ip a2, 4 = 24 01 12 stores it back.
        let (_, mut bus) = ee_run_mem(0x120124, 3, |c, _| {
            c.set_reg(2, 0x4000_3000);
            crate::ee::set_qacc_word(&mut c.accq[1], 4, 0xCAFE_BABE);
        });
        assert_eq!(bus.read32(0x4000_3000), 0xCAFE_BABE);
        // ee.ld.qacc_h.l.128.ip a4, 16 = 44 01 06 fills bytes 0..16.
        let (cpu, _) = ee_run_mem(0x060144, 3, |c, b| {
            c.set_reg(4, 0x4000_2000);
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, i as u32 + 1);
            }
        });
        assert_eq!(
            &cpu.accq[1][..16],
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        );
        assert_eq!(cpu.reg(4), 0x4000_2010);
        // ee.st.qacc_h.l.128.ip a2, 16 = 24 01 0d spills bytes 0..16.
        let (_, mut bus) = ee_run_mem(0x0D0124, 3, |c, _| {
            c.set_reg(2, 0x4000_3000);
            c.accq[1] = [0x55; 20];
        });
        for i in 0..16 {
            assert_eq!(bus.read8(0x4000_3000 + i as u32), 0x55);
        }
    }

    #[test]
    fn ee_ldqa_widens_into_qacc() {
        // ee.ldqa.s16.128.ip a4, 16 = 44 01 01: -1 lanes sign-extend.
        let (cpu, _) = ee_run_mem(0x010144, 3, |c, b| {
            c.set_reg(4, 0x4000_2000);
            for i in 0..8 {
                b.write16(0x4000_2000 + 2 * i as u32, 0xFFFF);
            }
        });
        for acc in 0..2 {
            for i in 0..4 {
                assert_eq!(crate::ee::acc_s40(&cpu, acc, i), -1, "acc{acc}[{i}]");
            }
        }
        assert_eq!(cpu.reg(4), 0x4000_2010);
    }

    #[test]
    fn ee_ldf_stf_swap_halves() {
        // ee.ldf.64.xp f0, f1, a4, a5 = 40 05 16: mem[0]->f1, mem[4]->f0.
        let (cpu, _) = ee_run_mem(0x160540, 3, |c, b| {
            c.set_reg(4, 0x4000_2000);
            c.set_reg(5, 8);
            b.write32(0x4000_2000, 1.0f32.to_bits());
            b.write32(0x4000_2004, 2.0f32.to_bits());
        });
        assert_eq!(cpu.freg(1), 1.0);
        assert_eq!(cpu.freg(0), 2.0);
        assert_eq!(cpu.reg(4), 0x4000_2008);
        // ee.stf.64.ip f0, f1, a2, 8 = 2e 07 10 e0 stores f1 then f0.
        let (_, mut bus) = ee_run_mem(0xE010_072E, 4, |c, _| {
            c.set_reg(2, 0x4000_3000);
            c.set_freg(1, 3.0);
            c.set_freg(0, 4.0);
        });
        assert_eq!(bus.read32(0x4000_3000), 3.0f32.to_bits());
        assert_eq!(bus.read32(0x4000_3004), 4.0f32.to_bits());
    }

    #[test]
    fn ee_ua_state_spill_fill() {
        // ee.ld.ua_state.ip a4, 16 = 44 01 10.
        let (cpu, _) = ee_run_mem(0x100144, 3, |c, b| {
            c.set_reg(4, 0x4000_2000);
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 0x10 + i as u32);
            }
        });
        assert_eq!(
            cpu.ua_state,
            [
                0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
                0x1E, 0x1F
            ]
        );
        assert_eq!(cpu.reg(4), 0x4000_2010);
        // ee.st.ua_state.ip a2, 0 = 24 00 1c (no postupdate).
        let (cpu2, mut bus) = ee_run_mem(0x1C0024, 3, |c, _| {
            c.set_reg(2, 0x4000_3000);
            c.ua_state = [0xAA; 16];
        });
        for i in 0..16 {
            assert_eq!(bus.read8(0x4000_3000 + i as u32), 0xAA);
        }
        assert_eq!(cpu2.reg(2), 0x4000_3000);
    }

    #[test]
    fn ee_ld_usar_snapshots_address_nibble() {
        // ee.ld.128.usar.ip q1, a2, 16 = 24 81 81 with unaligned a2.
        let (cpu, _) = ee_run_mem(0x818124, 3, |c, b| {
            c.set_reg(2, 0x4000_2003);
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, i as u32);
            }
        });
        for i in 0..16 {
            assert_eq!(cpu.qregs[1][i], i as u8);
        }
        assert_eq!(cpu.sar_byte, 3);
        assert_eq!(cpu.reg(2), 0x4000_2013);
    }

    #[test]
    fn ee_vldhbc_broadcasts_pairs() {
        // ee.vldhbc.16.incp q0, q1, a2 = 24 12 cc: two pair-broadcasts, +16.
        let (cpu, _) = ee_run_mem(0xCC1224, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            for i in 0..8 {
                b.write8(0x4000_2000 + i as u32, i as u32 + 1);
                b.write8(0x4000_2008 + i as u32, 0x10 + i as u32);
            }
        });
        assert_eq!(
            cpu.qregs[0],
            [1, 2, 1, 2, 3, 4, 3, 4, 5, 6, 5, 6, 7, 8, 7, 8]
        );
        assert_eq!(
            cpu.qregs[1],
            [
                0x10, 0x11, 0x10, 0x11, 0x12, 0x13, 0x12, 0x13, 0x14, 0x15, 0x14, 0x15, 0x16, 0x17,
                0x16, 0x17
            ]
        );
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_ams_butterfly_then_load() {
        // ee.fft.ams.s16.ld.incp q0, a2, q2, q4, q0, q0, q0, 0 = 2e 10 00 d1.
        // AMS on (qx=q0,qy=q0,qm=q0) lanes 2,3, then load into q0.
        // Alias probes: q1 (qx|1) holds sentinel 77 (HW must use q0);
        // q3 (qz|1) stays zero (HW must write q2, not q3).
        let (cpu, _) = ee_run_mem(0xD100_102E, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.qregs[0] = [0, 0, 0, 0, 3, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // s16[2]=3, s16[3]=1
            c.qregs[1] = [77; 16];
            c.qregs[3] = [0; 16];
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 20 + i as u32);
            }
        });
        let s16 = |q: &[u8; 16], i: usize| i16::from_le_bytes([q[2 * i], q[2 * i + 1]]);
        // temp0=6, temp1=0, temp2=-2, temp3=6.
        assert_eq!(s16(&cpu.qregs[2], 2), 4);
        assert_eq!(s16(&cpu.qregs[2], 3), 6);
        assert_eq!(s16(&cpu.qregs[4], 2), 8);
        assert_eq!(s16(&cpu.qregs[4], 3), 6);
        assert_eq!(cpu.qregs[3], [0; 16]);
        for i in 0..16 {
            assert_eq!(cpu.qregs[0][i], 20 + i as u8);
        }
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_dsp_dot_product_vehicle() {
        // End-to-end DSP path: two post-increment 128-bit loads feed a
        // vector MAC. A = B = [1..16], dot = sum(i^2) = 1496.
        //   vld.128.ip q0, a2, 16 = 24 01 83
        //   vld.128.ip q1, a2, 16 = 24 81 83
        //   vmulas.s8.accx q0, q1 = c4 08 1a
        let prog = [
            (0x4000_1000u32, 0x0083_0124u32),
            (0x4000_1003u32, 0x0083_8124u32),
            (0x4000_1006u32, 0x001A_08C4u32),
        ];
        let mut bus = RamBus::load(&prog);
        let mut cpu = Cpu::new(0);
        cpu.pc = 0x4000_1000;
        cpu.set_reg(2, 0x4000_2000);
        for i in 0..16 {
            bus.write8(0x4000_2000 + i as u32, i as u32 + 1);
            bus.write8(0x4000_2010 + i as u32, i as u32 + 1);
        }
        run(&mut cpu, &mut bus, 0x4000_1009);
        assert_eq!(cpu.accx, 1496);
        assert_eq!(cpu.reg(2), 0x4000_2020);
    }

    #[test]
    fn ee_vldbc_width_variants() {
        // ee.vldbc.16 q2, a3 = 34 73 dd.
        let (cpu, _) = ee_run_mem(0xDD7334, 3, |c, b| {
            c.set_reg(3, 0x4000_2000);
            b.write16(0x4000_2000, 0xABCD);
        });
        for i in 0..8 {
            assert_eq!(
                (cpu.qregs[2][2 * i] as u16) | ((cpu.qregs[2][2 * i + 1] as u16) << 8),
                0xABCD
            );
        }
        // ee.vldbc.32 q4, a3 = 34 77 ed.
        let (cpu, _) = ee_run_mem(0xED7734, 3, |c, b| {
            c.set_reg(3, 0x4000_2000);
            b.write32(0x4000_2000, 0x1234_5678);
        });
        for i in 0..4 {
            assert_eq!(
                u32::from_le_bytes(cpu.qregs[4][4 * i..4 * i + 4].try_into().unwrap()),
                0x1234_5678
            );
        }
        // ee.vldbc.16.ip q1, a2, 16 = 24 88 85.
        let (cpu, _) = ee_run_mem(0x858824, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            b.write16(0x4000_2000, 0xBEEF);
        });
        for i in 0..8 {
            assert_eq!(
                (cpu.qregs[1][2 * i] as u16) | ((cpu.qregs[1][2 * i + 1] as u16) << 8),
                0xBEEF
            );
        }
        assert_eq!(cpu.reg(2), 0x4000_2010);
        // ee.vldbc.32.ip q1, a2, 16 = 24 84 82.
        let (cpu, _) = ee_run_mem(0x828424, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            b.write32(0x4000_2000, 0xDEAD_BEEF);
        });
        for i in 0..4 {
            assert_eq!(
                u32::from_le_bytes(cpu.qregs[1][4 * i..4 * i + 4].try_into().unwrap()),
                0xDEAD_BEEF
            );
        }
        assert_eq!(cpu.reg(2), 0x4000_2010);
        // ee.vldbc.8.xp q1, a2, a3 = 24 d3 8d.
        let (cpu, _) = ee_run_mem(0x8DD324, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.set_reg(3, 16);
            b.write8(0x4000_2000, 0x55);
        });
        assert_eq!(cpu.qregs[1], [0x55; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2010);
        // ee.vldbc.16.xp q1, a2, a3 = 24 c3 8d.
        let (cpu, _) = ee_run_mem(0x8DC324, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.set_reg(3, 16);
            b.write16(0x4000_2000, 0xCAFE);
        });
        for i in 0..8 {
            assert_eq!(
                (cpu.qregs[1][2 * i] as u16) | ((cpu.qregs[1][2 * i + 1] as u16) << 8),
                0xCAFE
            );
        }
        assert_eq!(cpu.reg(2), 0x4000_2010);
        // ee.vldbc.32.xp q1, a2, a3 = 24 93 8d.
        let (cpu, _) = ee_run_mem(0x8D9324, 3, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.set_reg(3, 16);
            b.write32(0x4000_2000, 0x0BAD_F00D);
        });
        for i in 0..4 {
            assert_eq!(
                u32::from_le_bytes(cpu.qregs[1][4 * i..4 * i + 4].try_into().unwrap()),
                0x0BAD_F00D
            );
        }
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_ld_usar_xp_adds_register() {
        // ee.ld.128.usar.xp q1, a2, a3 = 24 83 8d.
        let (cpu, _) = ee_run_mem(0x8D8324, 3, |c, b| {
            c.set_reg(2, 0x4000_2005);
            c.set_reg(3, 11);
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 0x20 + i as u32);
            }
        });
        for i in 0..16 {
            assert_eq!(cpu.qregs[1][i], 0x20 + i as u8);
        }
        assert_eq!(cpu.sar_byte, 5);
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_slcxxp_srcxxp_indexed_slides() {
        // ee.slcxxp.2q q0, q1, a2, a3 = 24 13 86 (shift 4 left, a2 += 8).
        let (cpu, _) = ee_run_mem(0x861324, 3, |c, _| {
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[0] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
            c.set_reg(2, 3);
            c.set_reg(3, 8);
        });
        assert_eq!(
            cpu.qregs[1],
            [0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        );
        assert_eq!(
            cpu.qregs[0],
            [
                12, 13, 14, 15, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111
            ]
        );
        assert_eq!(cpu.reg(2), 11);
        // ee.srcxxp.2q q0, q1, a2, a3 = 24 13 c6 (shift 4 right).
        let (cpu, _) = ee_run_mem(0xC61324, 3, |c, _| {
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[0] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
            c.set_reg(2, 3);
            c.set_reg(3, 8);
        });
        assert_eq!(
            cpu.qregs[1],
            [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 100, 101, 102, 103]
        );
        assert_eq!(
            cpu.qregs[0],
            [
                104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 0, 0, 0, 0
            ]
        );
        assert_eq!(cpu.reg(2), 11);
    }

    #[test]
    fn ee_fused_valu_load_then_add() {
        // ee.vadds.s8.ld.incp q0, a2, q3, q4, q5 = 2f 2c 93 e0.
        let (cpu, _) = ee_run_mem(0xE093_2C2F, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 10);
            }
            c.qregs[4] = [1; 16];
            c.qregs[5] = [2; 16];
        });
        assert_eq!(cpu.qregs[0], [10; 16]);
        assert_eq!(cpu.qregs[3], [3; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_fused_valu_store_then_add() {
        // ee.vadds.s8.st.incp q0, a2, q3, q4, q5 = 2f 22 8b e4.
        let (cpu, mut bus) = ee_run_mem(0xE48B_222F, 4, |c, _| {
            c.set_reg(2, 0x4000_3000);
            c.qregs[0] = [7; 16];
            c.qregs[4] = [1; 16];
            c.qregs[5] = [2; 16];
        });
        for i in 0..16 {
            assert_eq!(bus.read8(0x4000_3000 + i as u32), 7);
        }
        assert_eq!(cpu.qregs[3], [3; 16]);
        assert_eq!(cpu.reg(2), 0x4000_3010);
    }

    #[test]
    fn ee_fused_vmul_load_then_multiply() {
        // ee.vmul.s8.ld.incp q0, a2, q3, q4, q5 = 2f 2c c3 e0, SAR = 1.
        let (cpu, _) = ee_run_mem(0xE0C3_2C2F, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 4);
            }
            c.qregs[4] = [3; 16];
            c.qregs[5] = [2; 16];
            c.set_sreg(crate::cpu::SR_SAR, 1);
        });
        assert_eq!(cpu.qregs[0], [4; 16]);
        assert_eq!(cpu.qregs[3], [3; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_r2bf_butterfly_sums_and_differences() {
        // ee.fft.r2bf.s16 q0, q1, q2, q3, 0 = 64 14 cc.
        let (cpu, _) = ee_run_mem(0xCC1464, 3, |c, _| {
            for i in 0..8 {
                c.qregs[2][2 * i] = i as u8;
                c.qregs[2][2 * i + 1] = 0;
                c.qregs[3][2 * i] = 10 + i as u8;
                c.qregs[3][2 * i + 1] = 0;
            }
        });
        let s16 = |q: &[u8; 16], i: usize| i16::from_le_bytes([q[2 * i], q[2 * i + 1]]);
        assert_eq!(
            [
                s16(&cpu.qregs[0], 0),
                s16(&cpu.qregs[0], 1),
                s16(&cpu.qregs[0], 2),
                s16(&cpu.qregs[0], 3)
            ],
            [4, 6, 8, 10]
        );
        assert_eq!(
            [
                s16(&cpu.qregs[0], 4),
                s16(&cpu.qregs[0], 5),
                s16(&cpu.qregs[0], 6),
                s16(&cpu.qregs[0], 7)
            ],
            [-4, -4, -4, -4]
        );
        assert_eq!(
            [
                s16(&cpu.qregs[1], 0),
                s16(&cpu.qregs[1], 1),
                s16(&cpu.qregs[1], 2),
                s16(&cpu.qregs[1], 3)
            ],
            [24, 26, 28, 30]
        );
        assert_eq!(
            [
                s16(&cpu.qregs[1], 4),
                s16(&cpu.qregs[1], 5),
                s16(&cpu.qregs[1], 6),
                s16(&cpu.qregs[1], 7)
            ],
            [-4, -4, -4, -4]
        );
        // sel2 = 1 reroutes lanes: ee.fft.r2bf.s16 q0, q1, q2, q3, 1 = 64 15 cc.
        let (cpu, _) = ee_run_mem(0xCC1564, 3, |c, _| {
            for i in 0..8 {
                c.qregs[2][2 * i] = i as u8;
                c.qregs[2][2 * i + 1] = 0;
                c.qregs[3][2 * i] = 10 + i as u8;
                c.qregs[3][2 * i + 1] = 0;
            }
        });
        assert_eq!(
            [
                s16(&cpu.qregs[0], 0),
                s16(&cpu.qregs[0], 1),
                s16(&cpu.qregs[0], 2),
                s16(&cpu.qregs[0], 3)
            ],
            [2, 4, 10, 12]
        );
        assert_eq!(
            [
                s16(&cpu.qregs[0], 4),
                s16(&cpu.qregs[0], 5),
                s16(&cpu.qregs[0], 6),
                s16(&cpu.qregs[0], 7)
            ],
            [-2, -2, -2, -2]
        );
        assert_eq!(
            [
                s16(&cpu.qregs[1], 0),
                s16(&cpu.qregs[1], 1),
                s16(&cpu.qregs[1], 2),
                s16(&cpu.qregs[1], 3)
            ],
            [22, 24, 30, 32]
        );
    }

    #[test]
    fn ee_src_q_ld_slides_then_loads() {
        // ee.src.q.ld.ip q0, a2, 16, q1, q2 = 2e 41 20 e0: qup(q1,q2)
        // by sar_byte=4 first, then load into q0, then +16.
        let (cpu, _) = ee_run_mem(0xE020_412E, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.sar_byte = 4;
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[2] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 50 + i as u32);
            }
        });
        assert_eq!(
            cpu.qregs[1],
            [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 100, 101, 102, 103]
        );
        for i in 0..16 {
            assert_eq!(cpu.qregs[0][i], 50 + i as u8);
        }
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }

    #[test]
    fn ee_srcq_stores_slide() {
        // ee.srcq.128.st.incp q1, q2, a3 = 34 1e dc, sar_byte=4.
        let (cpu, mut bus) = ee_run_mem(0xDC1E34, 3, |c, _| {
            c.set_reg(3, 0x4000_3000);
            c.sar_byte = 4;
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[2] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
        });
        for i in 0..16 {
            let want = if i < 12 {
                4 + i as u32
            } else {
                100 + (i - 12) as u32
            };
            assert_eq!(bus.read8(0x4000_3000 + i as u32), want, "byte {i}");
        }
        assert_eq!(cpu.reg(3), 0x4000_3010);
    }

    #[test]
    fn ee_r2bf_st_differences_and_shifted_sums() {
        // ee.fft.r2bf.s16.st.incp q0, q2, q3, a2, 0 = 2e 84 38 e8.
        // qy=q3 (odd) executes as qy=q1 (masked); q1 holds decoys.
        let (cpu, mut bus) = ee_run_mem(0xE838_842E, 4, |c, _| {
            c.set_reg(2, 0x4000_3000);
            for i in 0..8 {
                c.qregs[2][2 * i] = i as u8;
                c.qregs[2][2 * i + 1] = 0;
                c.qregs[1][2 * i] = 10 + i as u8;
                c.qregs[1][2 * i + 1] = 0;
                c.qregs[3][2 * i] = 20 + i as u8;
                c.qregs[3][2 * i + 1] = 0;
            }
        });
        let s16 = |q: &[u8; 16], i: usize| i16::from_le_bytes([q[2 * i], q[2 * i + 1]]);
        for i in 0..8 {
            assert_eq!(s16(&cpu.qregs[0], i), -10, "diff {i}");
        }
        for i in 0..8 {
            let want = if i < 4 {
                10 + 2 * i as u32
            } else {
                18 + 2 * (i - 4) as u32
            };
            let got = bus.read16(0x4000_3000 + 2 * i as u32);
            assert_eq!(got, want, "sum {i}");
        }
        assert_eq!(cpu.reg(2), 0x4000_3010);
    }
    #[test]
    fn ee_cmul_ld_pair_multiply_then_load() {
        // ee.fft.cmul.s16.ld.xp q0, a2, a3, q1, q2, q3, 0 = 2e 83 31 dc,
        // SAR = 1: (3+4i)(1+0i)>>1 = (1,2) into q1[0,1], then load.
        let (cpu, _) = ee_run_mem(0xDC31_832E, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.set_reg(3, 16);
            c.set_sreg(crate::cpu::SR_SAR, 1);
            c.qregs[2] = [3, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            c.qregs[3] = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 30 + i as u32);
            }
        });
        assert_eq!(cpu.qregs[1][0], 1);
        assert_eq!(cpu.qregs[1][1], 0);
        assert_eq!(cpu.qregs[1][2], 2);
        assert_eq!(cpu.qregs[1][3], 0);
        for i in 0..16 {
            assert_eq!(cpu.qregs[0][i], 30 + i as u8);
        }
        assert_eq!(cpu.reg(2), 0x4000_2010);
    }
    #[test]
    fn ee_src_q_ld_xp_adds_register() {
        // ee.src.q.ld.xp q0, a2, a3, q1, q2 = 2e 43 20 e8.
        let (cpu, _) = ee_run_mem(0xE820_432E, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.set_reg(3, 32);
            c.sar_byte = 4;
            c.qregs[1] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[2] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 60 + i as u32);
            }
        });
        assert_eq!(
            cpu.qregs[1],
            [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 100, 101, 102, 103]
        );
        for i in 0..16 {
            assert_eq!(cpu.qregs[0][i], 60 + i as u8);
        }
        assert_eq!(cpu.reg(2), 0x4000_2020);
    }
    #[test]
    fn ee_cmul_st_shifted_sums_then_store() {
        // ee.fft.cmul.s16.st.xp q0, q2, q4, a2, a3, 0, 0, 0 = 2e 03 24 a8.
        // low = qv[0..4], high = [qv[4],qv[5],cmul-pair], then a2 += a3.
        let (cpu, mut bus) = ee_run_mem(0xA824_032E, 4, |c, _| {
            c.set_reg(2, 0x4000_3000);
            c.set_reg(3, 16);
            c.qregs[0] = [3, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            c.qregs[2] = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            c.qregs[4] = [7, 0, 8, 0, 9, 0, 10, 0, 11, 0, 12, 0, 0, 0, 0, 0];
        });
        for (i, want) in [7u32, 8, 9, 10, 11, 12, 3, 4].iter().enumerate() {
            assert_eq!(bus.read16(0x4000_3000 + 2 * i as u32), *want, "lane {i}");
        }
        assert_eq!(cpu.reg(2), 0x4000_3010);
    }
    #[test]
    fn ee_vst_decp_reversed_halves_then_decrement() {
        // ee.fft.vst.r32.decp q0, a2, 0 = 24 33 cd: mem = [u32[3],u32[2],
        // u32[1],u32[0]], then a2 -= 16.
        let (cpu, mut bus) = ee_run_mem(0xCD3324, 3, |c, _| {
            c.set_reg(2, 0x4000_3000);
            c.qregs[0] = [1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0];
        });
        for (i, want) in [4u32, 3, 2, 1].iter().enumerate() {
            assert_eq!(bus.read32(0x4000_3000 + 4 * i as u32), *want, "lane {i}");
        }
        assert_eq!(cpu.reg(2), 0x4000_2FF0);
    }
    #[test]
    fn ee_vmulas_qup_mac_load_then_slide() {
        // ee.vmulas.s8.accx.ld.ip.qup q0, a2, 16, q3, q4, q5, q6
        // = 2e e1 56 20: MAC (accx=32), load q0, +16, slide.
        // qs0=q5 executes as q4 ([1:0] masked); q4 holds the source,
        // q5 stays untouched (proves the masking direction).
        let (cpu, _) = ee_run_mem(0x2056_E12E, 4, |c, b| {
            c.set_reg(2, 0x4000_2000);
            c.sar_byte = 4;
            c.qregs[3] = [1; 16];
            c.qregs[4] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
            c.qregs[5] = [77; 16];
            c.qregs[6] = [
                100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
            ];
            // NOTE q4 doubles as MAC source qy AND slide source (distinct
            // lanes used: MAC reads all, slide reads all — set once).
            c.qregs[4] = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2];
            for i in 0..16 {
                b.write8(0x4000_2000 + i as u32, 9);
            }
        });
        assert_eq!(cpu.accx, 32);
        assert_eq!(cpu.qregs[0], [9; 16]);
        assert_eq!(cpu.reg(2), 0x4000_2010);
        assert_eq!(cpu.qregs[5], [77; 16]);
        assert_eq!(
            cpu.qregs[4],
            [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 100, 101, 102, 103]
        );
    }
    #[test]
    fn ee_ams_st_shifted_halves_then_store() {
        // ee.fft.ams.s16.st.incp q0, q1, a2, a3, q2, q3, q4, 0
        // = 2e e3 0a a0: low=[as0>>1,qv>>1], high=[qv>>1], qz1 side
        // effects on lanes 6,7, then a3 += 16.
        let (cpu, mut bus) = ee_run_mem(0xA00A_E32E, 4, |c, _| {
            c.set_reg(2, 0x0002_0001);
            c.set_reg(3, 0x4000_3000);
            c.qregs[0] = [10, 0, 11, 0, 12, 0, 13, 0, 14, 0, 15, 0, 16, 0, 17, 0];
            c.qregs[2] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1, 0];
            c.qregs[3] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 0];
            c.qregs[4] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 0];
        });
        for (i, want) in [0u32, 1, 5, 5, 6, 6, 7, 7].iter().enumerate() {
            assert_eq!(bus.read16(0x4000_3000 + 2 * i as u32), *want, "lane {i}");
        }
        let s16 = |q: &[u8; 16], i: usize| i16::from_le_bytes([q[2 * i], q[2 * i + 1]]);
        assert_eq!(s16(&cpu.qregs[1], 6), 3);
        assert_eq!(s16(&cpu.qregs[1], 7), 9);
        assert_eq!(cpu.reg(3), 0x4000_3010);
    }
}
