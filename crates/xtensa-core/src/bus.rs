//! Memory access interface implemented by the SoC layer.
//!
//! The CPU core is SoC-agnostic: every load/store/fetch goes through this
//! trait (ESP32-S3 Technical Reference Manual, memory map chapter).

pub trait Bus {
    fn read8(&mut self, addr: u32) -> u32;
    fn read16(&mut self, addr: u32) -> u32;
    fn read32(&mut self, addr: u32) -> u32;
    fn write8(&mut self, addr: u32, val: u32);
    fn write16(&mut self, addr: u32, val: u32);
    fn write32(&mut self, addr: u32, val: u32);
}
