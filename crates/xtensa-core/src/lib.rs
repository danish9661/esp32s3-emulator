#![no_std]

#[cfg(test)]
extern crate std;

pub mod bus;
pub mod cpu;
mod exec;
pub mod generated;

pub use bus::Bus;
pub use cpu::{Cpu, StepResult};

#[cfg(test)]
mod tests {
    use crate::generated::*;

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
        // ISA RM L32R: sext16(imm16) << 2 + ((pc+3) & ~3). (QEMU's C emits
        // the tensilica idiom `((0xffff<<16)|imm16)<<2`, which equals sext
        // only when bit 15 of imm16 is set; forward l32r would be wrong.)
        assert_eq!(
            o[1].value,
            0x48d0u32.wrapping_add((0x4000_0100u32 + 3) & !3)
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
    fn run(cpu: &mut Cpu, bus: &mut RamBus, end: u32) {
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
        let mut cpu = Cpu::new();
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
        let mut cpu = Cpu::new();
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
        let mut cpu = Cpu::new();
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
        let mut cpu = Cpu::new();
        run(&mut cpu, &mut bus, end);

        assert_eq!(cpu.reg(4), 0xff, "l8ui");
        assert_eq!(cpu.reg(5), 0xffff, "l16ui");
        assert_eq!(cpu.reg(6), 0xffff_ffff, "l16si sign-extended");
        assert_eq!(cpu.reg(8), 0x55, "l8ui overwritten byte");
        assert_eq!(bus.read8(0x100), 0xff);
        assert_eq!(bus.read8(0x101), 0x55);
        assert_eq!(bus.read16(0x102), 0xffff);
    }
}
