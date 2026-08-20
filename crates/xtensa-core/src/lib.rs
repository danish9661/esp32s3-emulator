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
        let callx0_pc = a;
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
}
