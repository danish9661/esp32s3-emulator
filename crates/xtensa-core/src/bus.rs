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

    /// Atomic 32-bit compare-and-swap for `s32c1i` (ISA RM "Conditional
    /// Store"): if `*addr == compare`, set `*addr = val`. Returns the OLD
    /// word. The whole read-compare-write is ONE bus transaction — no
    /// `tick_timers` runs inside it, so a core-0 timer ISR can never slip
    /// between the read and the write and steal a lock word both cores
    /// raced on (that interleaving corrupts `portMUX` spinlocks: core 0's
    /// separated read32/write32 lets core 1's ISR write land first, then
    /// core 0 overwrites it and both cores own the lock). The default
    /// read-then-write is correct only for single-threaded test buses.
    fn cas32(&mut self, addr: u32, compare: u32, val: u32) -> u32 {
        let old = self.read32(addr);
        if old == compare {
            self.write32(addr, val);
        }
        old
    }

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
