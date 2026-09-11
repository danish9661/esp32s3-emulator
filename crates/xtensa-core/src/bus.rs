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

    /// CPU interrupt lines currently asserted by the SoC for CPU `cpu`
    /// (bit n = line n high).  The CPU ORs this into its INTSET state when
    /// checking for pending interrupts and when reading `rsr.interrupt`
    /// (QEMU: the INTC's qemu_irqs drive each CPU's extint inputs, which
    /// set the corresponding INTSET bits — exc_helper.c check_interrupts /
    /// translate.c xtensa_irq).  The interrupt matrix is per-CPU
    /// (ESP32-S3 TRM, interrupt matrix: one core_0/core_1 map per source).
    fn int_pending(&mut self, _cpu: usize) -> u32 {
        0
    }

    /// Dedicated-GPIO input channels currently seen by the CPU (bit c =
    /// channel c level). The ESP32-S3 wires the `ee.get_gpio_in` TIE
    /// instruction to the GPIO-matrix CORE1_GPIO_IN0..7 inputs (signals
    /// 129..131, 252..255, 54); the SoC resolves them against pad levels.
    /// Defaults to 0 (no pins routed, like an unconnected matrix input).
    fn dedic_gpio_in(&mut self) -> u32 {
        0
    }
}
