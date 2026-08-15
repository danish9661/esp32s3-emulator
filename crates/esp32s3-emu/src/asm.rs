//! Minimal hand-assembler for the Xtensa LX7 ISA, used to build the boot ROM
//! stub and hand-assembled firmware images.
//!
//! Every encoding below is verified against `xtensa-core::generated` (the
//! QEMU-derived decoder) and the cpu_tests/machine_tests binaries — see the
//! AGENTS.md status log (2026-08-15) for the byte-level gotchas:
//! - little-endian word = `b0 | b1<<8 | b2<<16` (24-bit) / `b0 | b1<<8` (16-bit)
//! - RRI8 (op0=2) opcode selector lives in r=[15:12] (MOVI=0xA, ADDI=0xC, ...)
//! - MOVI immediate = sext12({insn[11:8]<<8 | insn[23:16]}); r=[15:12] is the
//!   opcode selector, the value never touches it
//! - SLLI: op1=1, op2=[23:20]=sal[4], sal={bit20, insn[7:4]}, shift=32-sal
//! - l16ui/s16i and l32i/s32i offsets are in halfwords/words respectively
//! - l32r: imm16 = (target - ((pc+3)&~3)) >> 2, low 16 bits

use alloc::vec::Vec;

/// Instruction stream builder with pc tracking and l32r backpatching.
pub struct Asm {
    bytes: Vec<u8>,
    base_pc: u32,
}

impl Asm {
    /// New stream that will be loaded at `base_pc` (needed for pc-relative
    /// encodings: l32r, j, branches, call0).
    pub fn new(base_pc: u32) -> Self {
        Self {
            bytes: Vec::new(),
            base_pc,
        }
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Mutable access to the emitted bytes (for patching raw literals).
    pub fn bytes_mut(&mut self) -> &mut Vec<u8> {
        &mut self.bytes
    }

    /// PC of the next instruction to be emitted.
    pub fn pc(&self) -> u32 {
        self.base_pc + self.bytes.len() as u32
    }

    /// Current byte offset (for patch_l32r).
    pub fn offset(&self) -> usize {
        self.bytes.len()
    }

    /// Append a little-endian instruction word.
    fn insn(&mut self, word: u32, len: u32) {
        for i in 0..len {
            self.bytes.push(((word >> (8 * i)) & 0xFF) as u8);
        }
    }

    // ── 24-bit instructions (RRI8: op0=2, selector r=[15:12]) ────────────────

    /// movi t, sext12(v) — v in [-2048, 2047].
    pub fn movi(&mut self, t: u8, v: i32) {
        let v = v & 0xFFF;
        self.insn(
            ((v as u32 & 0xFF) << 16)
                | (0xA << 12)
                | ((((v as u32) >> 8) & 0xF) << 8)
                | ((t as u32) << 4)
                | 2,
            3,
        );
    }

    /// addi t, s, sext8(v).
    pub fn addi(&mut self, t: u8, s: u8, v: i32) {
        self.insn(
            ((v as u32 & 0xFF) << 16) | (0xC << 12) | ((s as u32) << 8) | ((t as u32) << 4) | 2,
            3,
        );
    }

    /// addmi t, s, sext8(v)<<8.
    pub fn addmi(&mut self, t: u8, s: u8, v: i32) {
        self.insn(
            ((v as u32 & 0xFF) << 16) | (0xD << 12) | ((s as u32) << 8) | ((t as u32) << 4) | 2,
            3,
        );
    }

    /// slli t, s, shift (1..=31). t=[15:12], s=[11:8], sal={bit20, bits[7:4]}
    /// (verified: `slli a2,a2,24` = 0x0001_2280 in machine_tests).
    pub fn slli(&mut self, t: u8, s: u8, shift: u32) {
        let sal = 32 - shift;
        self.insn(
            ((sal >> 4) << 20)
                | (1 << 16)
                | ((t as u32) << 12)
                | ((s as u32) << 8)
                | ((sal & 0xF) << 4),
            3,
        );
    }

    /// Load `v` (any 32-bit) into `t` using movi/slli/addmi/addi.
    pub fn li(&mut self, t: u8, v: i32) {
        if (-2048..=2047).contains(&v) {
            self.movi(t, v);
            return;
        }
        let hi = v >> 12; // arithmetic shift
        let lo = v & 0xFFF;
        self.li(t, hi);
        self.slli(t, t, 12);
        let lo_byte = lo & 0xFF;
        let hi_nib = (lo >> 8) & 0xF;
        let (lo8, hi8) = if lo_byte >= 0x80 {
            (lo_byte - 256, hi_nib + 1)
        } else {
            (lo_byte, hi_nib)
        };
        if hi8 != 0 {
            self.addmi(t, t, hi8);
        }
        if lo8 != 0 {
            self.addi(t, t, lo8);
        }
    }

    // ── loads/stores (offsets in BYTES; RRI8 encodes word/halfword counts) ───

    /// l8ui t, s, off.
    pub fn l8ui(&mut self, t: u8, s: u8, off: u32) {
        self.insn(
            ((off & 0xFF) << 16) | ((s as u32) << 8) | ((t as u32) << 4) | 2,
            3,
        );
    }

    /// l16ui t, s, off (halfword count = off/2).
    pub fn l16ui(&mut self, t: u8, s: u8, off: u32) {
        self.insn(
            (((off >> 1) & 0xFF) << 16) | (1 << 12) | ((s as u32) << 8) | ((t as u32) << 4) | 2,
            3,
        );
    }

    /// l32i t, s, off (word count = off/4).
    pub fn l32i(&mut self, t: u8, s: u8, off: u32) {
        self.insn(
            (((off >> 2) & 0xFF) << 16) | (2 << 12) | ((s as u32) << 8) | ((t as u32) << 4) | 2,
            3,
        );
    }

    /// s8i t, s, off.
    pub fn s8i(&mut self, t: u8, s: u8, off: u32) {
        self.insn(
            ((off & 0xFF) << 16) | (4 << 12) | ((s as u32) << 8) | ((t as u32) << 4) | 2,
            3,
        );
    }

    /// s16i t, s, off (halfword count = off/2).
    pub fn s16i(&mut self, t: u8, s: u8, off: u32) {
        self.insn(
            (((off >> 1) & 0xFF) << 16) | (5 << 12) | ((s as u32) << 8) | ((t as u32) << 4) | 2,
            3,
        );
    }

    /// s32i t, s, off (word count = off/4).
    pub fn s32i(&mut self, t: u8, s: u8, off: u32) {
        self.insn(
            (((off >> 2) & 0xFF) << 16) | (6 << 12) | ((s as u32) << 8) | ((t as u32) << 4) | 2,
            3,
        );
    }

    // ── control flow ──────────────────────────────────────────────────────────

    /// l32r t, target — emits with a placeholder; call `patch_l32r` once the
    /// literal address is known.
    pub fn l32r(&mut self, t: u8) -> usize {
        let at = self.bytes.len();
        self.insn(((t as u32) << 4) | 1, 3);
        at
    }

    /// Patch an l32r emitted by `l32r(t)` to point at `target`.
    pub fn patch_l32r(&mut self, at: usize, target: u32) {
        let pc = self.base_pc + at as u32;
        let base = (pc + 3) & !3;
        let imm16 = ((target as i64 - base as i64) >> 2) as u32 & 0xFFFF;
        self.bytes[at + 1] = (imm16 & 0xFF) as u8;
        self.bytes[at + 2] = ((imm16 >> 8) & 0xFF) as u8;
    }

    /// j target (18-bit signed offset).
    pub fn j(&mut self, target: u32) {
        let off = (target as i64 - self.pc() as i64 - 4) & 0x3FFFF;
        self.insn(((off as u32) << 6) | 6, 3);
    }

    /// jx t (t in [11:8] per generated.rs).
    pub fn jx(&mut self, t: u8) {
        self.insn(((t as u32) << 8) | (2 << 6) | (2 << 4), 3);
    }

    /// callx0 t — pc+4 -> a0, jump reg(t).
    pub fn callx0(&mut self, t: u8) {
        self.insn(((t as u32) << 8) | (3 << 6), 3);
    }

    /// ret (a0 -> pc). RRR op0=0 op1=0 op2=0 r=0 m=2 n=0 (t-field = 0x8;
    /// 0xD would decode as CALLX4).
    pub fn ret(&mut self) {
        // RRR op0=0 op1=0 op2=0 r=0 m=2 n=0 (t-field = 0x8; 0xD would
        // decode as CALLX4).
        self.insn(2 << 6, 3);
    }

    /// rsil t, level (RRR op0=0 op1=0 op2=0 r=6; s=[11:8] = level,
    /// t=[7:4] = dest; ISA RM "RSIL"; verified against generated.rs).
    pub fn rsil(&mut self, t: u8, level: u32) {
        self.insn((6 << 12) | ((level & 0xF) << 8) | ((t as u32) << 4), 3);
    }

    /// rfi level (RRR op0=0 op1=0 op2=0 r=3 t=1; s=[11:8] = level).
    pub fn rfi(&mut self, level: u32) {
        self.insn((3 << 12) | ((level & 0xF) << 8) | (1 << 4), 3);
    }

    /// rsr t, sr (RRR op0=0 op1=3 op2=0; sr = {r[15:12]<<4 | s[11:8]}).
    pub fn rsr(&mut self, t: u8, sr: u32) {
        self.insn(
            (3 << 16) | (((sr >> 4) & 0xF) << 12) | ((sr & 0xF) << 8) | ((t as u32) << 4),
            3,
        );
    }

    /// wsr sr, t (RRR op0=0 op1=3 op2=1; source = t=[7:4]).
    pub fn wsr(&mut self, sr: u32, t: u8) {
        self.insn(
            (1 << 20)
                | (3 << 16)
                | (((sr >> 4) & 0xF) << 12)
                | ((sr & 0xF) << 8)
                | ((t as u32) << 4),
            3,
        );
    }

    /// beqz s, target (12-bit signed offset).
    pub fn beqz(&mut self, s: u8, target: u32) {
        let off = (target as i64 - self.pc() as i64 - 4) & 0xFFF;
        self.insn(((off as u32) << 12) | ((s as u32) << 8) | (1 << 4) | 6, 3);
    }

    /// bnez s, target (12-bit signed offset).
    pub fn bnez(&mut self, s: u8, target: u32) {
        let off = (target as i64 - self.pc() as i64 - 4) & 0xFFF;
        self.insn(
            ((off as u32) << 12) | ((s as u32) << 8) | (1 << 6) | (1 << 4) | 6,
            3,
        );
    }

    // ── 16-bit (inst16b) ───────────────────────────────────────────────────────

    /// movi.n s, imm7 (0..=0x7F; imm7 = {z, n[5:4], r[15:12]} per
    /// inst16b decode; z lives in bit [6:4]'s top bit, not bit 20).
    pub fn movi_n(&mut self, s: u8, imm7: u32) {
        let r = imm7 & 0xF;
        let n = (imm7 >> 4) & 0x7;
        self.insn((r << 12) | ((s as u32) << 8) | (n << 4) | 0xC, 2);
    }

    /// 2-byte padding (nop.n is not decoded by our decoder — see AGENTS.md).
    pub fn pad2(&mut self) {
        self.movi_n(15, 0);
    }

    /// Append a raw 32-bit literal (l32r pool entry), little-endian.
    pub fn lit(&mut self, word: u32) {
        self.insn(word, 4);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use xtensa_core::generated::{Opcode, Opnd, decode_inst, fld_inst, opnds};
    use xtensa_core::{Bus, Cpu, StepResult};

    struct RamBus(Vec<u8>);

    impl RamBus {
        fn load(bytes: &[u8]) -> Self {
            let mut v = std::vec![0u8; 0x1000];
            v[..bytes.len()].copy_from_slice(bytes);
            Self(v)
        }
        fn idx(&self, addr: u32) -> usize {
            (addr - 0x4000_0000) as usize
        }
    }

    impl Bus for RamBus {
        fn read8(&mut self, addr: u32) -> u32 {
            self.0[self.idx(addr)] as u32
        }
        fn read16(&mut self, addr: u32) -> u32 {
            let i = self.idx(addr);
            u16::from_le_bytes([self.0[i], self.0[i + 1]]) as u32
        }
        fn read32(&mut self, addr: u32) -> u32 {
            let i = self.idx(addr);
            u32::from_le_bytes([self.0[i], self.0[i + 1], self.0[i + 2], self.0[i + 3]])
        }
        fn write8(&mut self, addr: u32, val: u32) {
            let i = self.idx(addr);
            self.0[i] = val as u8;
        }
        fn write16(&mut self, addr: u32, val: u32) {
            let i = self.idx(addr);
            self.0[i] = val as u8;
            self.0[i + 1] = (val >> 8) as u8;
        }
        fn write32(&mut self, addr: u32, val: u32) {
            let i = self.idx(addr);
            for (j, b) in val.to_le_bytes().into_iter().enumerate() {
                self.0[i + j] = b;
            }
        }
    }

    fn run(cpu: &mut Cpu, bus: &mut RamBus, end: u32, max: usize) {
        for _ in 0..max {
            if cpu.pc >= end {
                return;
            }
            match cpu.step(bus) {
                StepResult::Ok => {}
                r => panic!("unexpected step result: {r:?}"),
            }
        }
        panic!("did not reach pc {end:#x}");
    }

    #[test]
    fn known_encodings_match_machine_tests() {
        let mut a = Asm::new(0x4000_0000);
        a.movi(2, 0x60);
        assert_eq!(a.bytes(), &[0x22, 0xA0, 0x60], "movi a2,0x60");
        a.movi(3, -1);
        assert_eq!(a.bytes()[3..], [0x32, 0xAF, 0xFF], "movi a3,-1");
        a.slli(2, 2, 24);
        assert_eq!(a.bytes()[6..], [0x80, 0x22, 0x01], "slli a2,a2,24");
        a.slli(4, 4, 12);
        assert_eq!(a.bytes()[9..], [0x40, 0x44, 0x11], "slli a4,a4,12");
        a.addi(3, 3, 1);
        assert_eq!(a.bytes()[12..], [0x32, 0xC3, 0x01], "addi a3,a3,1");
        a.s32i(4, 5, 4);
        assert_eq!(a.bytes()[15..], [0x42, 0x65, 0x01], "s32i a4,a5,1");
        a.l8ui(8, 2, 1);
        assert_eq!(a.bytes()[18..], [0x82, 0x02, 0x01], "l8ui a8,a2,1");
        a.movi_n(4, 5);
        assert_eq!(a.bytes()[21..], [0x0C, 0x54], "movi.n a4,5");
        a.movi_n(3, 16);
        assert_eq!(a.bytes()[23..], [0x1C, 0x03], "movi.n a3,16");
    }

    #[test]
    fn decoder_roundtrip() {
        let mut a = Asm::new(0x4000_0100);
        a.movi(2, -100);
        a.slli(5, 5, 3);
        a.l16ui(6, 2, 6);
        a.s16i(7, 2, 4);
        a.callx0(3);
        a.jx(4);
        a.ret();
        a.beqz(5, 0x4000_0120);
        a.bnez(6, 0x4000_00F0);
        a.j(0x4000_0200);
        for w in a.bytes.chunks_exact(3) {
            let word = u32::from_le_bytes([w[0], w[1], w[2], 0]);
            let op = decode_inst(word).expect("should decode");
            let _ = opnds(op, word, 0);
            let _ = fld_inst::op0(word);
        }
        let w = u32::from_le_bytes([a.bytes[0], a.bytes[1], a.bytes[2], 0]);
        assert_eq!(decode_inst(w), Some(Opcode::OPCODE_MOVI));
        assert_eq!(opnds(Opcode::OPCODE_MOVI, w, 0)[1], Opnd::imm(0xFFFF_FF9C));
    }

    #[test]
    fn sr_and_interrupt_encodings_roundtrip() {
        // wsr intenable (sr 228 = 0xE4), wsr intset (226), wsr intclear
        // (227), rsil a4,0, rfi 3, rsr interrupt (decodes to SR 226).
        let mut a = Asm::new(0x4000_0100);
        a.wsr(228, 4);
        a.wsr(226, 4);
        a.wsr(227, 4);
        a.rsil(4, 0);
        a.rfi(3);
        a.rsr(5, 226);
        for w in a.bytes.chunks_exact(3) {
            let word = u32::from_le_bytes([w[0], w[1], w[2], 0]);
            assert!(decode_inst(word).is_some(), "word {word:#010x} decodes");
            let op = decode_inst(word).unwrap();
            let _ = opnds(op, word, 0);
        }
        let word = |i: usize| u32::from_le_bytes([a.bytes[i], a.bytes[i + 1], a.bytes[i + 2], 0]);
        assert_eq!(decode_inst(word(0)), Some(Opcode::OPCODE_WSR_INTENABLE));
        assert_eq!(decode_inst(word(3)), Some(Opcode::OPCODE_WSR_INTSET));
        assert_eq!(decode_inst(word(6)), Some(Opcode::OPCODE_WSR_INTCLEAR));
        assert_eq!(decode_inst(word(9)), Some(Opcode::OPCODE_RSIL));
        assert_eq!(decode_inst(word(12)), Some(Opcode::OPCODE_RFI));
        assert_eq!(decode_inst(word(15)), Some(Opcode::OPCODE_RSR_INTERRUPT));
        // Operand extraction: rsil dest = t, level = s; rfi level = s.
        assert_eq!(
            opnds(Opcode::OPCODE_RSIL, word(9), 0)[0],
            Opnd::reg(4),
            "rsil dest a4"
        );
        assert_eq!(opnds(Opcode::OPCODE_RSIL, word(9), 0)[1], Opnd::imm(0));
        assert_eq!(opnds(Opcode::OPCODE_RFI, word(12), 0)[0], Opnd::imm(3));
    }

    #[test]
    fn li_builds_constants() {
        let mut a = Asm::new(0x4000_0000);
        a.li(1, 0x3FC8_8000);
        a.li(2, 0x3C01_0000);
        a.li(3, 0x6000_0000);
        let mut cpu = Cpu::new();
        let mut bus = RamBus::load(a.bytes());
        run(&mut cpu, &mut bus, a.pc(), 64);
        assert_eq!(cpu.reg(1), 0x3FC8_8000);
        assert_eq!(cpu.reg(2), 0x3C01_0000);
        assert_eq!(cpu.reg(3), 0x6000_0000);
    }

    #[test]
    fn l32r_backpatch() {
        let mut a = Asm::new(0x4000_0000);
        let p1 = a.l32r(2);
        let p2 = a.l32r(3);
        a.pad2(); // 4-align the literal pool (l32r base = (pc+3)&~3)
        let pool = a.offset();
        a.lit(0x4000_0000 + pool as u32);
        a.lit(0x4000_0000 + pool as u32 + 4);
        a.patch_l32r(p1, 0x4000_0000 + pool as u32);
        a.patch_l32r(p2, 0x4000_0000 + pool as u32 + 4);
        let mut cpu = Cpu::new();
        let mut bus = RamBus::load(a.bytes());
        // Stop before the pool so the literals are never executed.
        run(&mut cpu, &mut bus, 0x4000_0000 + pool as u32, 16);
        assert_eq!(cpu.reg(2), 0x4000_0000 + pool as u32);
        assert_eq!(cpu.reg(3), 0x4000_0000 + pool as u32 + 4);
    }
}
