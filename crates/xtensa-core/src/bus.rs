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

    /// CPU interrupt lines currently asserted by the SoC (bit n = line n
    /// high).  The CPU ORs this into its INTSET state when checking for
    /// pending interrupts and when reading `rsr.interrupt` (QEMU: the
    /// INTC's qemu_irqs drive the CPU's extint inputs, which set the
    /// corresponding INTSET bits — exc_helper.c check_interrupts /
    /// translate.c xtensa_irq).
    fn int_pending(&mut self) -> u32 {
        0
    }
}
