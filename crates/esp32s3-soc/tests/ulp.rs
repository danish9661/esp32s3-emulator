//! ULP-RISC-V register-block model tests.

use esp32s3_soc::ulp::ULP_BASE;

#[test]
fn core_register_round_trips() {
    let mut d = esp32s3_soc::ulp::Ulp::new();
    d.write32(ULP_BASE + 0x00, 0xDEAD_BEEF); // core
    d.write32(ULP_BASE + 0x04, 0x1234_5678); // ocp
    assert_eq!(d.read32(ULP_BASE + 0x00), 0xDEAD_BEEF);
    assert_eq!(d.read32(ULP_BASE + 0x04), 0x1234_5678);
}

#[test]
fn general_registers_round_trip() {
    let mut d = esp32s3_soc::ulp::Ulp::new();
    // General-purpose result registers live within the block (off 0x0C..0xFC).
    d.write32(ULP_BASE + 0x0C, 0xAB);
    d.write32(ULP_BASE + 0xFC, 0xCD);
    assert_eq!(d.read32(ULP_BASE + 0x0C), 0xAB);
    assert_eq!(d.read32(ULP_BASE + 0xFC), 0xCD);
}

#[test]
fn unwritten_register_reads_zero() {
    let mut d = esp32s3_soc::ulp::Ulp::new();
    // An un-written register reads back as 0 (software-defined default).
    assert_eq!(d.read32(ULP_BASE + 0x100), 0);
    assert_eq!(d.read32(ULP_BASE + 0x108), 0);
}
