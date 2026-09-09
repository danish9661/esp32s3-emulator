//! LP_UART (low-power UART) register model tests.

use esp32s3_soc::lp_uart::{LP_UART_BASE, LpUart};

#[test]
fn fifo_and_conf_registers_round_trip() {
    let mut d = LpUart::new();
    d.write32(LP_UART_BASE, 0x0000_00AB); // FIFO
    d.write32(LP_UART_BASE + 0x20, 0x1234_5678); // CONF0
    assert_eq!(d.read32(LP_UART_BASE), 0x0000_00AB);
    assert_eq!(d.read32(LP_UART_BASE + 0x20), 0x1234_5678);
}

#[test]
fn clkdiv_register_round_trips() {
    let mut d = LpUart::new();
    d.write32(LP_UART_BASE + 0x14, 0x00AA_00BB); // CLKDIV
    assert_eq!(d.read32(LP_UART_BASE + 0x14), 0x00AA_00BB);
}

#[test]
fn unwritten_register_reads_zero() {
    let mut d = LpUart::new();
    assert_eq!(d.read32(LP_UART_BASE + 0x04), 0); // INT_RAW
}
