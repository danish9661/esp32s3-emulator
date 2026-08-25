//! ESP32-S3 ULP-RISC-V control/status register block.
//!
//! The ULP-RISC-V coprocessor lives at `DR_REG_ULP_RISCV_BASE = 0x6000_8100`,
//! i.e. offset `0x100` of the `0x6000_8000` page (the page also holds
//! RTC_CNTL at 0x000 and RTC_IO at 0x400). Per the ULP-RISC-V `ulp_riscv_dev_t`
//! layout the block covers roughly `0x100..0x200` of that page: `core` (0x100,
//! start/reset/halt bits), `ocp` (0x104), `debug` (0x108, halted flag), and the
//! 16 general-purpose `reg` slots (0x10C..0x1FC, used by the ULP program to
//! communicate results back to the main CPU).
//!
//! Modeled as a register store. **Execution of ULP programs is NOT modeled**
//! (that requires a second RISC-V core); only the register interface is
//! emulated, so firmware can configure/start the ULP and poll status. This is
//! sufficient to validate the register path via direct pokes and matches the
//! documented P5 validation pattern.

pub const ULP_BASE: u32 = 0x6000_8100;

// Region of the 0x6000_8000 page carved out for ULP (off 0x100..0x200).
pub const ULP_OFF_START: u32 = 0x100;
pub const ULP_OFF_END: u32 = 0x200;

const REG_COUNT: usize = (ULP_OFF_END - ULP_OFF_START) as usize / 4;

pub struct Ulp {
    regs: [u32; REG_COUNT],
}

impl Default for Ulp {
    fn default() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
        }
    }
}

impl Ulp {
    pub fn new() -> Self {
        Self::default()
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
    }
}
