//! EFUSE unit tests: modeled factory MAC in the read-data registers, read_cmd
//! handshake is a no-op, STATUS reports idle.

use esp32s3_soc::efuse::Efuse;

// Factory MAC 0x112233445566 is stored byte-reversed per word (block bit[0:8)
// is a word's LSB, and the driver maps block bit[0:8) -> MAC byte[0]).
const MAC_LO: u32 = 0x4433_2211;
const MAC_HI: u32 = 0x0000_6655;

#[test]
fn modeled_mac_in_sys_part1_and_spi_sys() {
    let mut e = Efuse::new();
    // RD_SYS_PART1_DATA0..1 (block 1 factory MAC).
    assert_eq!(e.read32(0x5C), MAC_LO);
    assert_eq!(e.read32(0x60), MAC_HI);
    // RD_MAC_SPI_SYS_0..1 (identical SPI-boot MAC).
    assert_eq!(e.read32(0x44), MAC_LO);
    assert_eq!(e.read32(0x48), MAC_HI);
}

#[test]
fn status_reports_idle() {
    let mut e = Efuse::new();
    // EFUSE_STATUS_REG (0x1D0) state field reads idle (0) so the driver's
    // read-done poll exits immediately.
    assert_eq!(e.read32(0x1D0), 0);
}

#[test]
fn read_cmd_is_noop_and_preserves_mac() {
    let mut e = Efuse::new();
    // Issuing read_cmd must not clobber the modeled MAC.
    e.write32(0x1D4, 1);
    assert_eq!(e.read32(0x5C), MAC_LO);
    assert_eq!(e.read32(0x60), MAC_HI);
    // Writes to the read-only data registers are dropped.
    e.write32(0x5C, 0xDEAD_BEEF);
    assert_eq!(e.read32(0x5C), MAC_LO);
    // Random read of an unmodeled register returns 0.
    assert_eq!(e.read32(0x200), 0);
}
