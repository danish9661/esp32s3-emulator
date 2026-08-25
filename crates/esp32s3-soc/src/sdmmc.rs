//! ESP32-S3 SD/MMC host controller (DesignWare MMC, `DR_REG_SDMMC_BASE`).
//!
//! Base `DR_REG_SDMMC_BASE = 0x6002_8000`. The controller is a Synopsys
//! DesignWare MMC host: `CTRL` (0x00), `PWREN` (0x04), `CLKDIV` (0x08),
//! `CLKENA` (0x10), `CTYPE` (0x18), `BLKSIZ` (0x1C), `BYTCNT` (0x20),
//! `CMDARG` (0x28), `CMD` (0x2C), `RESP0..3` (0x30..0x3C), `MINTSTS` (0x40),
//! `RINTSTS` (0x44), `STATUS` (0x48), `CARDTHRCTL` (0x50), ... up to ~0x400.
//! The command/response state machine (DMA, card detection, command issue) is
//! **NOT modeled** — that requires driving a real SD card. The block is modeled
//! as a register store so firmware can configure the controller and poll status,
//! which is enough to validate the register path via direct pokes (the same
//! pattern used for the I2C/TWAI peripheral validation).

pub const SDMMC_BASE: u32 = 0x6002_8000;

// Cover the full controller register window (0x400 bytes).
const REG_COUNT: usize = 0x400 / 4;

pub struct Sdmmc {
    regs: [u32; REG_COUNT],
}

impl Default for Sdmmc {
    fn default() -> Self {
        Self {
            regs: [0u32; REG_COUNT],
        }
    }
}

impl Sdmmc {
    pub fn new() -> Self {
        Self::default()
    }

    fn idx(&self, offset: u32) -> usize {
        ((offset & 0xFFF) / 4) as usize
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
