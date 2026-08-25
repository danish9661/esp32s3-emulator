//! SD/MMC host controller (DesignWare MMC) register model tests.

use esp32s3_soc::sdmmc::SDMMC_BASE;

#[test]
fn ctrl_register_round_trips() {
    let mut d = esp32s3_soc::sdmmc::Sdmmc::new();
    d.write32(SDMMC_BASE + 0x00, 0x000F_0001); // CTRL
    assert_eq!(d.read32(SDMMC_BASE + 0x00), 0x000F_0001);
}

#[test]
fn cmd_and_resp_round_trip() {
    let mut d = esp32s3_soc::sdmmc::Sdmmc::new();
    d.write32(SDMMC_BASE + 0x2C, 0x8020_0000); // CMD
    d.write32(SDMMC_BASE + 0x30, 0xCAFE_BEEF); // RESP0
    assert_eq!(d.read32(SDMMC_BASE + 0x2C), 0x8020_0000);
    assert_eq!(d.read32(SDMMC_BASE + 0x30), 0xCAFE_BEEF);
}

#[test]
fn unwritten_register_reads_zero() {
    let mut d = esp32s3_soc::sdmmc::Sdmmc::new();
    assert_eq!(d.read32(SDMMC_BASE + 0x48), 0); // STATUS
}
