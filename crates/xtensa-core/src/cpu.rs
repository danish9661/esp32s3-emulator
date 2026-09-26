//! ESP32-S3 Xtensa LX7 CPU: register file, windowed registers, exceptions,
//! step loop.
//!
//! Reference: QEMU espressif/qemu `target/xtensa` (GPLv2):
//!   - `win_helper.c`: window overflow/underflow, retw, entry, movsp helpers
//!   - `translate.c`: per-opcode translation semantics
//!   - `cpu.h`: SR numbers, PS field layout
//!   - `core-esp32s3/core-isa.h`: LX7 configuration (64 ARs, little-endian,
//!     windowed, loops)
//!
//! Register-file model (identical semantics to QEMU's phys_regs/regs view):
//! there is one physical 64-entry array; logical register aN maps to
//! `phys[(wb * 4 + N) & 63]` where wb = WINDOW_BASE.  QEMU instead keeps a
//! pre-rotated 16-register view (`env->regs[0..15]`) that is re-rotated at
//! translation-block boundaries; the two are equivalent because every
//! rotation in QEMU is a multiple of 4 registers (ISA RM, windowed
//! registers).

use crate::bus::Bus;
use crate::exec::{self, Outcome};
use crate::generated::{Opcode, decode_inst, decode_inst16a, decode_inst16b, insn_len, opnds};

/// One decode-cache entry: fetch key (`pc`, `raw`) plus the decoded
/// `opcode`, its operands, instruction length, and the precomputed
/// windowed-register operand mask for the overflow check.
type DecodeEntry = (
    u32,
    u32,
    Option<crate::generated::Opcode>,
    [crate::generated::Opnd; 8],
    u32,
    u32,
);

// Special register numbers (QEMU cpu.h "SR enum").  ESP32-S3 has no NDEPC,
// so double exceptions reuse EPC1.
pub const SR_LBEG: u32 = 0;
pub const SR_LEND: u32 = 1;
pub const SR_LCOUNT: u32 = 2;
pub const SR_SAR: u32 = 3;
pub const SR_BR: u32 = 4;
pub const SR_LITBASE: u32 = 5;
pub const SR_SCOMPARE1: u32 = 12;
pub const SR_ACCLO: u32 = 16;
pub const SR_ACCHI: u32 = 17;
pub const SR_M0: u32 = 32;
pub const SR_M1: u32 = 33;
pub const SR_M2: u32 = 34;
pub const SR_M3: u32 = 35;
pub const SR_PREFCTL: u32 = 40;
pub const SR_MISC0: u32 = 48;
pub const SR_MISC1: u32 = 49;
pub const SR_MISC2: u32 = 50;
pub const SR_MISC3: u32 = 51;
pub const SR_WINDOW_BASE: u32 = 72;
pub const SR_WINDOW_START: u32 = 73;
pub const SR_PTEVADDR: u32 = 83;
pub const SR_MMID: u32 = 89;
pub const SR_RASID: u32 = 90;
pub const SR_ITLBCFG: u32 = 91;
pub const SR_DTLBCFG: u32 = 92;
pub const SR_ERACCESS: u32 = 95;
pub const SR_IBREAKENABLE: u32 = 96;
pub const SR_MEMCTL: u32 = 97;
pub const SR_CACHEATTR: u32 = 98;
pub const SR_ATOMCTL: u32 = 99;
pub const SR_DDR: u32 = 104;
pub const SR_MEPC: u32 = 106;
pub const SR_MEPS: u32 = 107;
pub const SR_MESAVE: u32 = 108;
pub const SR_MESR: u32 = 109;
pub const SR_MECR: u32 = 110;
pub const SR_MEVADDR: u32 = 111;
pub const SR_IBREAKA0: u32 = 128;
pub const SR_IBREAKA1: u32 = 129;
pub const SR_DBREAKA0: u32 = 144;
pub const SR_DBREAKA1: u32 = 145;
pub const SR_DBREAKC0: u32 = 160;
pub const SR_DBREAKC1: u32 = 161;
pub const SR_CONFIGID0: u32 = 176;
pub const SR_EPC1: u32 = 177;
pub const SR_EPC2: u32 = 178;
pub const SR_EPC3: u32 = 179;
pub const SR_EPC4: u32 = 180;
pub const SR_EPC5: u32 = 181;
pub const SR_EPC6: u32 = 182;
pub const SR_EPC7: u32 = 183;
pub const SR_DEPC: u32 = 192;
pub const SR_EPS2: u32 = 194;
pub const SR_EPS3: u32 = 195;
pub const SR_EPS4: u32 = 196;
pub const SR_EPS5: u32 = 197;
pub const SR_EPS6: u32 = 198;
pub const SR_EPS7: u32 = 199;
pub const SR_CONFIGID1: u32 = 208;
pub const SR_EXCSAVE1: u32 = 209;
pub const SR_EXCSAVE2: u32 = 210;
pub const SR_EXCSAVE3: u32 = 211;
pub const SR_EXCSAVE4: u32 = 212;
pub const SR_EXCSAVE5: u32 = 213;
pub const SR_EXCSAVE6: u32 = 214;
pub const SR_EXCSAVE7: u32 = 215;
pub const SR_CPENABLE: u32 = 224;
pub const SR_INTERRUPT: u32 = 225;
pub const SR_INTSET: u32 = 226;
pub const SR_INTCLEAR: u32 = 227;
pub const SR_INTENABLE: u32 = 228;
pub const SR_PS: u32 = 230;
pub const SR_VECBASE: u32 = 231;
pub const SR_EXCCAUSE: u32 = 232;
pub const SR_DEBUGCAUSE: u32 = 233;
pub const SR_CCOUNT: u32 = 234;
pub const SR_PRID: u32 = 235;
pub const SR_ICOUNT: u32 = 236;
pub const SR_ICOUNTLEVEL: u32 = 237;
pub const SR_EXCVADDR: u32 = 238;
pub const SR_CCOMPARE0: u32 = 240;
pub const SR_CCOMPARE1: u32 = 241;
pub const SR_CCOMPARE2: u32 = 242;

// User SR space (RUR/WUR).  THREADPTR is a distinct physical register from
// VECBASE even though both are numbered 231 (QEMU cpu.h has separate enums).
pub const UR_THREADPTR: u32 = 231;
pub const UR_SAR_BYTE: u32 = 232;
pub const UR_FCR: u32 = 233;
pub const UR_FSR: u32 = 234;
pub const UR_FFT_BIT_WIDTH: u32 = 235;
pub const UR_ACCX_0: u32 = 236;
pub const UR_ACCX_1: u32 = 237;
pub const UR_QACC_H_0: u32 = 238;
pub const UR_QACC_H_1: u32 = 239;
pub const UR_QACC_H_2: u32 = 240;
pub const UR_QACC_H_3: u32 = 241;
pub const UR_QACC_H_4: u32 = 242;
pub const UR_QACC_L_0: u32 = 243;
pub const UR_QACC_L_1: u32 = 244;
pub const UR_QACC_L_2: u32 = 245;
pub const UR_QACC_L_3: u32 = 246;
pub const UR_QACC_L_4: u32 = 247;
pub const UR_UA_STATE_0: u32 = 248;
pub const UR_UA_STATE_1: u32 = 249;
pub const UR_UA_STATE_2: u32 = 250;
pub const UR_UA_STATE_3: u32 = 251;
pub const UR_GPIO_OUT: u32 = 252;

// PS fields (QEMU cpu.h; ISA RM "Processor State (PS) Register").
pub const PS_INTLEVEL: u32 = 0xf;
pub const PS_EXCM: u32 = 0x10;
pub const PS_UM: u32 = 0x20;
pub const PS_RING: u32 = 0xc0;
pub const PS_RING_SHIFT: u32 = 6;
pub const PS_OWB: u32 = 0xf00;
pub const PS_OWB_SHIFT: u32 = 8;
pub const PS_CALLINC: u32 = 0x30000;
pub const PS_CALLINC_SHIFT: u32 = 16;
pub const PS_WOE: u32 = 0x40000;

// EXCCAUSE values (ISA RM, "EXCCAUSE Register").
pub const ILLEGAL_INSTRUCTION_CAUSE: u32 = 0;
pub const SYSCALL_CAUSE: u32 = 1;
pub const INSTRUCTION_FETCH_ERROR_CAUSE: u32 = 2;
pub const LOAD_STORE_ERROR_CAUSE: u32 = 3;
pub const LEVEL1_INTERRUPT_CAUSE: u32 = 4;
pub const ALLOCA_CAUSE: u32 = 5;
pub const INTEGER_DIVIDE_BY_ZERO_CAUSE: u32 = 6;
pub const PC_VALUE_ERROR_CAUSE: u32 = 7;
pub const PRIVILEGED_CAUSE: u32 = 8;
pub const LOAD_STORE_ALIGNMENT_CAUSE: u32 = 9;
pub const WINDOW_OVERFLOW4_CAUSE: u32 = 32;
pub const WINDOW_UNDERFLOW4_CAUSE: u32 = 33;
pub const WINDOW_OVERFLOW8_CAUSE: u32 = 34;
pub const WINDOW_UNDERFLOW8_CAUSE: u32 = 35;
pub const WINDOW_OVERFLOW12_CAUSE: u32 = 36;
pub const WINDOW_UNDERFLOW12_CAUSE: u32 = 37;

// Exception vector offsets from VECBASE (ESP32-S3 core-isa.h:
// XCHAL_WINDOW_VECTORS_VADDR 0x40000000, KERNEL 0x40000300, USER 0x40000340,
// DOUBLE 0x400003C0; VECBASE reset value 0x40000000).
pub const VEC_OF4: u32 = 0x000;
pub const VEC_UF4: u32 = 0x040;
pub const VEC_OF8: u32 = 0x080;
pub const VEC_UF8: u32 = 0x0c0;
pub const VEC_OF12: u32 = 0x100;
pub const VEC_UF12: u32 = 0x140;
pub const VEC_KERNEL: u32 = 0x300;
pub const VEC_USER: u32 = 0x340;
pub const VEC_DOUBLE: u32 = 0x3c0;

// Interrupt configuration (ESP32-S3 core-isa.h: XCHAL_NUM_INTLEVELS = 6,
// XCHAL_EXCM_LEVEL = 3, XCHAL_NUM_EXTINTERRUPTS = 26; the 32 interrupt lines
// are grouped into 6 levels + NMI as "level 7", which is how QEMU's
// exc_helper.c treats them).
/// Highest non-NMI interrupt level delivered to a vector; NMI is level 7.
pub const INT_NUM_LEVELS: u32 = 6;
/// Level used for cintlevel while PS.EXCM is set (XCHAL_EXCM_LEVEL).
pub const EXCM_LEVEL: u32 = 3;
/// Lines per level, index = level 0..=7 (XCHAL_INTLEVEL1..7_MASK).
pub const INT_LEVEL_MASKS: [u32; 8] = [
    0,
    0x0006_37FF, // level 1: lines 0-10, 12, 13, 17, 18
    0x0038_0000, // level 2: lines 19-21
    0x28C0_8800, // level 3: lines 11, 15, 22, 23, 27, 29
    0x5300_0000, // level 4: lines 24, 25, 28, 30
    0x8401_0000, // level 5: lines 16, 26, 31
    0,           // level 6: no lines
    0x0000_4000, // level 7 (NMI): line 14
];
/// Interrupt vector offsets from VECBASE for levels 2..=7
/// (XCHAL_INTLEVEL2..7_VECOFS).  Level 1 has no vector: it is delivered
/// as a kernel/user/double exception (EXCCAUSE = LEVEL1_INTERRUPT_CAUSE).
pub const INT_VEC_OFFSETS: [u32; 8] = [0, 0, 0x180, 0x1C0, 0x200, 0x240, 0x280, 0x2C0];
/// NMI line (bit 14; XCHAL_INTLEVEL7_MASK) and the level QEMU uses for it.
pub const NMI_LINE: u32 = 14;
pub const NMI_LEVEL: u32 = 7;

/// Reset vector address (ESP32-S3 core-isa.h XCHAL_RESET_VECTOR_PADDR =
/// 0x40000400 — the window vectors own 0x40000000-0x17F, the kernel/user
/// vectors 0x300/0x340, and the ROM's reset code starts at 0x40000400;
/// QEMU `xtensa_cpu_reset` sets env->pc = XCHAL_RESET_VECTOR_PADDR).
pub const RESET_VECTOR: u32 = 0x4000_0400;
/// VECBASE reset value: the window vectors live at 0x40000000
/// (XCHAL_WINDOW_VECTORS_VADDR; core-isa.h) — NOT the reset vector.
pub const VECBASE_RESET: u32 = 0x4000_0000;

pub struct Cpu {
    pub pc: u32,
    /// Core ID (0 or 1): selects the interrupt-matrix column in
    /// `Bus::int_pending` and seeds the PRID special register (the real
    /// PRID is a read-only strapping of the core number, ISA RM PRID;
    /// the ESP32-S3 ROM's reset vector compares PRID against 0xCDCD
    /// (core 0, `_start` stack select) and 0xABAB (core 1, the
    /// APP-CPU fastboot check) — QEMU esp32s3.c uses the same values).
    core_id: usize,
    phys: [u32; 64],
    /// Single-precision floating-point register file: 16 × 32-bit raw IEEE
    /// bits (ISA RM 4.3.11.2). NOT windowed — global like the special
    /// registers (QEMU `f` array). Reset state is undefined on silicon;
    /// zeroed here.
    fpregs: [u32; 16],
    sregs: [u32; 256],
    user_sregs: [u32; 256],
    /// Debug/OCD doubleword register (LDDR32.P/SDDR32.P postupdate memory
    /// moves through it; ISA RM "Processor State" DDR — not a numbered
    /// special register, so it lives outside `sregs`). Reset 0.
    pub ddr: u64,
    /// TIE vector register file (ee.* DSP/AI units): 8 × 128-bit Q
    /// registers, byte-addressable (QEMU `Q_reg`: s8/u8/s16/u16/s32/u32
    /// lane views). Reset 0.
    pub qregs: [[u8; 16]; 8],
    /// TIE 40-bit saturating accumulator (vmulas target; QEMU `ACCX`).
    /// Signed ops saturate to ±0x7FFFFFFFFF, unsigned to [0, 0xFFFFFFFFFF].
    pub accx: i64,
    /// TIE quarter-round accumulators: 2 × 20 bytes (8×20-bit lanes for
    /// 8-bit ops, 4×40-bit lanes for 16-bit ops; QEMU `ACCQ`).
    pub accq: [[u8; 20]; 2],
    /// TIE unaligned-access staging register (LD_UA/ST_UA).
    pub ua_state: [u8; 16],
    /// TIE SAR byte, FFT width and GPIO-out state (UR_SAR_BYTE,
    /// UR_FFT_BIT_WIDTH, UR_GPIO_OUT user registers).
    pub sar_byte: u8,
    pub fft_width: u8,
    pub tie_gpio: u32,
    pub(crate) windowbase_next: Option<u32>,
    pub icount: u64,
    /// Length in bytes of the most recently executed instruction (set by
    /// `step_one` on every path that reaches fetch). Lets block drivers
    /// verify pc advance without re-fetching the length.
    last_len: u32,
    /// Raw instruction word of the most recently executed instruction (set
    /// alongside `last_len`). Lets TIE executors decode operands directly
    /// from the word, since the generated `opnds` arms for `ee.*` are
    /// placeholder stubs.
    last_raw: u32,
    /// Debug counters for interrupt-delivery diagnosis (run_flash probes).
    pub dbg_irq_taken: u64,
    pub dbg_irq_skipped_level0: u64,
    /// Decode cache for WASM/native speed: direct-mapped cache of
    /// `pc -> (raw, opc, opnds, len, wmask)` to avoid the `decode_inst` match
    /// AND the `opnds` match per step. Hot loops (boot copy, FreeRTOS) hit
    /// 95%+ of the time; `opnds()` is the larger of the two matches, so
    /// caching it is the bigger win. `wmask` is the precomputed windowed-
    /// register operand mask for the overflow check.
    decode_cache: alloc::boxed::Box<[DecodeEntry; 8192]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepResult {
    Ok,
    /// An exception was raised; EPC1/EXCCAUSE/PS and PC are already set.
    Exception {
        cause: u32,
    },
    Unimplemented(&'static str),
}

impl Cpu {
    /// Create a CPU for core `core_id` (0 or 1 on the ESP32-S3).
    pub fn new(core_id: usize) -> Self {
        // Reset state: vectors at 0x4000_0000, window 0 active (the reset
        // boot code on real silicon sets WINDOWSTART=1 before the first
        // windowed call).
        let mut cpu = Cpu {
            pc: RESET_VECTOR,
            core_id,
            phys: [0; 64],
            fpregs: [0; 16],
            sregs: [0; 256],
            user_sregs: [0; 256],
            ddr: 0,
            qregs: [[0; 16]; 8],
            accx: 0,
            accq: [[0; 20]; 2],
            ua_state: [0; 16],
            sar_byte: 0,
            fft_width: 0,
            tie_gpio: 0,
            windowbase_next: None,
            icount: 0,
            last_len: 0,
            last_raw: 0,
            dbg_irq_taken: 0,
            dbg_irq_skipped_level0: 0,
            decode_cache: {
                let mut v = alloc::vec::Vec::with_capacity(8192);
                v.resize_with(8192, || {
                    (0, 0, None, [crate::generated::Opnd::imm(0); 8], 0, 0)
                });
                v.into_boxed_slice().try_into().unwrap()
            },
        };
        cpu.sregs[SR_VECBASE as usize] = VECBASE_RESET;
        cpu.sregs[SR_WINDOW_START as usize] = 1;
        // PRID strapping: 0xCDCD (core 0) / 0xABAB (core 1) — the exact
        // values the ESP32-S3 ROM compares against (reset vector 0x40045A,
        // _start 0x40034C0B); the app derives its core index as
        // (PRID >> 13) & 1 (core_intr_matrix_clear, xPortEnterCriticalTimeout).
        cpu.sregs[SR_PRID as usize] = if core_id == 1 { 0xABAB } else { 0xCDCD };
        cpu
    }

    // --- register file -----------------------------------------------------

    /// Length in bytes of the last instruction `step_one` executed
    /// (for block drivers verifying fall-through advance).
    #[inline]
    pub fn last_len(&self) -> u32 {
        self.last_len
    }

    /// Raw word of the most recently executed instruction (see `last_raw`).
    #[inline]
    pub fn last_raw(&self) -> u32 {
        self.last_raw
    }

    #[inline]
    pub fn windowbase(&self) -> u32 {
        self.sregs[SR_WINDOW_BASE as usize] & 0xf
    }

    /// Direct WINDOWBASE write (host-callback frontend — same field CALL8
    /// / ENTRY / RETW / WSR_WINDOWBASE write; QEMU `win_helper.c` parity).
    /// Needed to synthesize a call frame around a firmware callback the
    /// host invokes (see `Esp32S3::run_espnow_callback`).
    #[inline]
    pub fn set_windowbase(&mut self, wb: u32) {
        self.sregs[SR_WINDOW_BASE as usize] = wb & 0xf;
    }

    /// Logical register aN (ISA RM: aN = phys[(WINDOW_BASE*4 + N) & 63]).
    #[inline]
    pub fn reg(&self, n: u32) -> u32 {
        self.phys[((self.windowbase() * 4 + n) & 63) as usize]
    }

    #[inline]
    pub fn set_reg(&mut self, n: u32, v: u32) {
        let i = ((self.windowbase() * 4 + n) & 63) as usize;
        self.phys[i] = v;
    }

    /// Floating-point register fN as f32 (raw bits via u32::from_bits).
    #[inline]
    pub fn freg(&self, n: u32) -> f32 {
        f32::from_bits(self.fpregs[(n & 15) as usize])
    }

    /// Write floating-point register fN (stored as raw bits).
    #[inline]
    pub fn set_freg(&mut self, n: u32, v: f32) {
        self.fpregs[(n & 15) as usize] = v.to_bits();
    }

    /// Boolean register bit (ISA RM Boolean Option): the 16 FP-compare
    /// result bits live in SR_BR (QEMU `br` — compares set/clear one bit).
    #[inline]
    pub fn br(&self, b: u32) -> bool {
        self.sregs[SR_BR as usize] >> (b & 15) & 1 != 0
    }

    /// Set/clear one boolean bit, preserving the other 15.
    #[inline]
    pub fn set_br(&mut self, b: u32, v: bool) {
        let bit = 1u32 << (b & 15);
        if v {
            self.sregs[SR_BR as usize] |= bit;
        } else {
            self.sregs[SR_BR as usize] &= !bit;
        }
    }

    #[inline]
    pub fn sreg(&self, n: u32) -> u32 {
        self.sregs[n as usize]
    }

    #[inline]
    pub fn set_sreg(&mut self, n: u32, v: u32) {
        self.sregs[n as usize] = v;
    }

    #[inline]
    pub fn user_sreg(&self, n: u32) -> u32 {
        self.user_sregs[n as usize]
    }

    #[inline]
    pub fn set_user_sreg(&mut self, n: u32, v: u32) {
        self.user_sregs[n as usize] = v;
    }

    #[inline]
    pub fn ps(&self) -> u32 {
        self.sregs[SR_PS as usize]
    }

    /// WINDOWSTART replicated to 16 bits: `ws | (ws << (nareg / 4))`
    /// (QEMU xtensa_replicate_windowstart, cpu.h).
    #[inline]
    fn windowstart_replicated(&self) -> u32 {
        let ws = self.sregs[SR_WINDOW_START as usize];
        ws | (ws << 16)
    }

    /// Number of active window units above the current base, capped at 3
    /// (QEMU: `ctz32(windowstart >> (wb + 1)) | 0x8` in
    /// xtensa_tr_init_disas_context, translate.c).
    #[inline]
    pub(crate) fn window(&self) -> u32 {
        let ws = self.windowstart_replicated() >> (self.windowbase() + 1);
        (ws | 0x8).trailing_zeros()
    }

    /// Rotate the window by `delta` window units (multiples of 4 registers).
    /// Physical regs are not moved; only WINDOW_BASE changes (equivalent to
    /// QEMU's xtensa_rotate_window, which rotates a view array).
    #[inline]
    pub(crate) fn rotate(&mut self, delta: i32) {
        let wb = (self.windowbase() as i32 + delta) & 0xf;
        self.sregs[SR_WINDOW_BASE as usize] = wb as u32;
    }

    /// Apply a deferred window-base change (QEMU defers rotations to
    /// translation-block boundaries via `windowbase_next`; we defer to the
    /// end of the instruction, which is equivalent in an interpreter).
    #[inline]
    fn sync_windowbase(&mut self) {
        if let Some(wb) = self.windowbase_next.take() {
            self.sregs[SR_WINDOW_BASE as usize] = wb & 0xf;
        }
    }

    /// Window overflow (QEMU HELPER(window_check), win_helper.c): rotate to
    /// the first free unit above the active chain, save OWB, take the
    /// WINDOW_OVERFLOW4/8/12 vector.  Returns the EXCCAUSE value.
    pub(crate) fn window_overflow(&mut self, pc: u32) -> u32 {
        let wb_old = self.windowbase();
        let ws = self.windowstart_replicated() >> (wb_old + 1);
        let n = ws.trailing_zeros() + 1;

        self.rotate(n as i32);
        self.sregs[SR_PS as usize] =
            (self.sregs[SR_PS as usize] & !PS_OWB) | (wb_old << PS_OWB_SHIFT) | PS_EXCM;
        self.sregs[SR_EPC1 as usize] = pc;
        let cause = match (ws >> n).trailing_zeros() {
            0 => WINDOW_OVERFLOW4_CAUSE,
            1 => WINDOW_OVERFLOW8_CAUSE,
            _ => WINDOW_OVERFLOW12_CAUSE,
        };
        self.sregs[SR_EXCCAUSE as usize] = cause;
        self.pc = self.sregs[SR_VECBASE as usize]
            + VEC_OF4
            + ((cause - WINDOW_OVERFLOW4_CAUSE) / 2) * 0x80;
        cause
    }

    /// Window underflow (QEMU HELPER(test_underflow_retw), win_helper.c):
    /// rotate back, save OWB, take the WINDOW_UNDERFLOW4/8/12 vector.
    pub(crate) fn window_underflow(&mut self, pc: u32, n: u32) -> u32 {
        let wb_old = self.windowbase();
        self.rotate(-(n as i32));
        self.sregs[SR_PS as usize] =
            (self.sregs[SR_PS as usize] & !PS_OWB) | (wb_old << PS_OWB_SHIFT) | PS_EXCM;
        self.sregs[SR_EPC1 as usize] = pc;
        let cause = match n {
            1 => WINDOW_UNDERFLOW4_CAUSE,
            2 => WINDOW_UNDERFLOW8_CAUSE,
            _ => WINDOW_UNDERFLOW12_CAUSE,
        };
        self.sregs[SR_EXCCAUSE as usize] = cause;
        self.pc = self.sregs[SR_VECBASE as usize] + VEC_UF4 + (n - 1) * 0x80;
        cause
    }

    /// Generic exception (QEMU HELPER(exception_cause), exc_helper.c).
    /// ESP32-S3 has no NDEPC: EPC1 is always used, even for double
    /// exceptions.
    pub(crate) fn raise_cause(&mut self, pc: u32, cause: u32) {
        self.sregs[SR_EPC1 as usize] = pc;
        let vec = if self.sregs[SR_PS as usize] & PS_EXCM != 0 {
            VEC_DOUBLE
        } else if self.sregs[SR_PS as usize] & PS_UM != 0 {
            VEC_USER
        } else {
            VEC_KERNEL
        };
        self.sregs[SR_EXCCAUSE as usize] = cause;
        self.sregs[SR_PS as usize] |= PS_EXCM;
        self.pc = self.sregs[SR_VECBASE as usize] + vec;
    }

    /// Effective interrupt status: software-sticky INTSET bits ORed with
    /// the interrupt lines asserted by the SoC (QEMU keeps the live line
    /// state directly in INTSET via xtensa_irq; we keep them separate).
    #[inline]
    pub fn intset_live<B: Bus>(&self, bus: &mut B) -> u32 {
        self.sregs[SR_INTSET as usize] | bus.int_pending(self.core_id)
    }

    /// Interrupt dispatch at the instruction boundary (QEMU
    /// check_interrupts + handle_interrupt, exc_helper.c).  Returns true
    /// if an interrupt was taken.
    ///
    /// Semantics (all verified against exc_helper.c:155-200):
    /// - The pending level is the highest one with a bit in
    ///   `INT_LEVEL_MASKS[level] & INTSET & INTENABLE`; the NMI (level 7,
    ///   line 14) bypasses INTENABLE and the cintlevel comparison.
    /// - cintlevel = PS.INTLEVEL, raised to EXCM_LEVEL (3) while PS.EXCM
    ///   is set; the interrupt is only taken if level > cintlevel.
    /// - Level 1: delivered as a kernel/user exception with EXCCAUSE =
    ///   LEVEL1_INTERRUPT_CAUSE, EPC1 = pc, PS.EXCM set.
    /// - Level 2..6: EPC[level] = pc, EPS[level] = old PS,
    ///   PS = (PS & ~INTLEVEL) | level | EXCM, pc = VECBASE + vector.
    /// - NMI: same as level 2..6 plus its sticky INTSET bit is cleared.
    ///
    /// Public so block-at-a-time drivers (machine `step_fast`) can execute
    /// several instructions via `step_one` and dispatch interrupts once at
    /// the block boundary, mirroring QEMU's TB-granularity delivery.
    pub fn check_interrupts<B: Bus>(&mut self, bus: &mut B) -> bool {
        let intenable = self.sregs[SR_INTENABLE as usize];
        let intset_sw = self.sregs[SR_INTSET as usize];
        // Fast path: if no interrupts are globally enabled and no software
        // interrupt bits are set, skip the expensive int_pending scan entirely.
        // During early boot (before FreeRTOS enables interrupts) this skips
        // ~22 peripheral register reads per instruction.
        if intenable == 0 && intset_sw == 0 {
            self.dbg_irq_skipped_level0 += 1;
            return false;
        }
        let intset = intset_sw | bus.int_pending(self.core_id);
        let level = if intset & (1 << NMI_LINE) != 0 {
            NMI_LEVEL
        } else {
            let mut l = 0;
            for i in (1..=INT_NUM_LEVELS).rev() {
                if INT_LEVEL_MASKS[i as usize] & intset & intenable != 0 {
                    l = i;
                    break;
                }
            }
            l
        };
        if level == 0 {
            self.dbg_irq_skipped_level0 += 1;
            return false;
        }
        let ps = self.sregs[SR_PS as usize];
        let cintlevel = if ps & PS_EXCM != 0 {
            (ps & PS_INTLEVEL).max(EXCM_LEVEL)
        } else {
            ps & PS_INTLEVEL
        };
        if level != NMI_LEVEL && level <= cintlevel {
            return false;
        }
        self.dbg_irq_taken += 1;
        let pc = self.pc;
        if level == 1 {
            self.sregs[SR_EXCCAUSE as usize] = LEVEL1_INTERRUPT_CAUSE;
            self.sregs[SR_EPC1 as usize] = pc;
            self.sregs[SR_PS as usize] |= PS_EXCM;
            let vec = if ps & PS_UM != 0 {
                VEC_USER
            } else {
                VEC_KERNEL
            };
            self.pc = self.sregs[SR_VECBASE as usize] + vec;
        } else {
            self.sregs[(SR_EPC1 + level - 1) as usize] = pc;
            self.sregs[(SR_EPS2 + level - 2) as usize] = ps;
            self.sregs[SR_PS as usize] = (ps & !PS_INTLEVEL) | level | PS_EXCM;
            if level == NMI_LEVEL {
                // The NMI is edge-like on this core: taking it clears its
                // sticky INTSET bit (QEMU handle_interrupt nmi branch).
                self.sregs[SR_INTSET as usize] &= !(1 << NMI_LINE);
            }
            self.pc = self.sregs[SR_VECBASE as usize] + INT_VEC_OFFSETS[level as usize];
        }
        true
    }

    /// Execute one instruction.  Returns a StepResult describing what
    /// happened (see StepResult).
    #[inline(always)]
    pub fn step<B: Bus>(&mut self, bus: &mut B) -> StepResult {
        let r = self.step_one(bus);
        if !matches!(r, StepResult::Ok) {
            return r;
        }
        // Interrupt dispatch at the instruction boundary (QEMU runs
        // check_interrupts before the next translation block).  pc has
        // already advanced to the next instruction, so EPC[level] /
        // EPS[level] let the handler return with RFE/RFI to the
        // instruction after the one that took the interrupt.
        self.check_interrupts(bus);
        StepResult::Ok
    }

    /// Execute one instruction WITHOUT the trailing interrupt check.
    /// Identical to `step` otherwise (CCOUNT advance, fetch, decode,
    /// window-overflow check, execute, loop-end check). Public so
    /// block-at-a-time drivers can run several instructions back-to-back
    /// and dispatch interrupts once at the block boundary (QEMU TB
    /// granularity); the caller must invoke `check_interrupts` afterwards
    /// or interrupts would never deliver.
    #[inline(always)]
    pub fn step_one<B: Bus>(&mut self, bus: &mut B) -> StepResult {
        // CCOUNT (SR 234) advances one cycle per instruction on real
        // silicon — the boot ROM's delays (0x40041A76: `rsr.ccount; sub;
        // bltu`) spin on it and would loop forever with a frozen counter.
        self.sregs[SR_CCOUNT as usize] = self.sregs[SR_CCOUNT as usize].wrapping_add(1);
        let pc = self.pc;
        // Single read32 for fetch, extract b0 and len — faster than read8+read16/32
        let raw_full = bus.read32(pc);
        let b0 = (raw_full & 0xFF) as u8;
        let len = insn_len(b0);
        let raw = if len == 2 {
            raw_full & 0xFFFF
        } else {
            raw_full
        };

        // Decode cache: direct-mapped 8k entries, keyed by pc+raw.
        // Hot loops (boot copy, FreeRTOS) hit 95%+ of the time, avoiding both
        // large matches (`decode_inst` and the even larger `opnds`). The
        // window-operand mask is precomputed at fill time.
        let idx = (pc as usize) & 0x1FFF;
        let (cached_pc, cached_raw, cached_opc, cached_opnds, cached_len, cached_wmask) =
            &self.decode_cache[idx];
        let (opc, o, len, wmask) = if *cached_pc == pc && *cached_raw == raw {
            (*cached_opc, *cached_opnds, *cached_len, *cached_wmask)
        } else {
            let (opc, insn) = match len {
                2 => {
                    let opc = if b0 & 0xf <= 11 {
                        decode_inst16a(raw)
                    } else {
                        decode_inst16b(raw)
                    };
                    (opc, raw)
                }
                _ => (decode_inst(raw), raw),
            };
            let opnds = opnds(opc.unwrap_or(Opcode::OPCODE_ILL), insn, pc);
            let mut wmask = 0u32;
            for op in opnds.iter() {
                if op.is_reg {
                    wmask |= 1u32 << (op.value & 31);
                }
            }
            self.decode_cache[idx] = (pc, raw, opc, opnds, len, wmask);
            (opc, opnds, len, wmask)
        };
        self.last_len = len;
        self.last_raw = raw;

        let opc = match opc {
            Some(o) => o,
            None => {
                self.raise_cause(pc, ILLEGAL_INSTRUCTION_CAUSE);
                return StepResult::Exception {
                    cause: ILLEGAL_INSTRUCTION_CAUSE,
                };
            }
        };

        // Generic window-overflow check: QEMU ORs 1<<v for every AR
        // register operand (visible and hidden) of the instruction and
        // raises WINDOW_OVERFLOWx if (highest bit)/4 > active window units.
        // `wmask` is precomputed at decode-cache fill time. The check is
        // ACTIVE only with WOE set and EXCM clear (QEMU
        // xtensa_get_tb_cpu_state: `(PS & (WOE|EXCM)) == WOE`); the
        // exception vectors run with EXCM, and the raw s32e/l32e/rfwo
        // window-handler code must not re-trigger the overflow.
        if wmask != 0 && self.sregs[SR_PS as usize] & (PS_WOE | PS_EXCM) == PS_WOE {
            let r = 31 - wmask.leading_zeros();
            if r / 4 > self.window() {
                let cause = self.window_overflow(pc);
                return StepResult::Exception { cause };
            }
        }

        match exec::execute(self, bus, opc, &o, len) {
            Outcome::Seq => {
                let next_pc = pc.wrapping_add(len);
                // Zero-overhead loop check (QEMU gen_check_loop_end): when
                // the sequential continuation reaches LEND and LCOUNT != 0,
                // decrement and jump to LBEG.  Taken branches/jumps do NOT
                // trigger the check.
                if next_pc == self.sregs[SR_LEND as usize] && self.sregs[SR_LCOUNT as usize] != 0 {
                    self.sregs[SR_LCOUNT as usize] -= 1;
                    self.pc = self.sregs[SR_LBEG as usize];
                } else {
                    self.pc = next_pc;
                }
            }
            Outcome::Jump(t) => {
                self.pc = t;
            }
            Outcome::Exception(cause) => {
                return StepResult::Exception { cause };
            }
            Outcome::Unimplemented => {
                return StepResult::Unimplemented("opcode");
            }
        }
        self.sync_windowbase();
        self.icount += 1;
        StepResult::Ok
    }
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new(0)
    }
}
