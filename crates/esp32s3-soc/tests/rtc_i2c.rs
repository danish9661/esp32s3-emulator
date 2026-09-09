//! RTC_I2C (LP/I2C) register model tests.

use esp32s3_soc::rtc_i2c::{RTC_I2C_BASE, RtcI2c};

#[test]
fn scl_timing_registers_round_trip() {
    let mut d = RtcI2c::new();
    d.write32(RTC_I2C_BASE, 0x0000_0032); // I2C_SCL_LOW
    d.write32(RTC_I2C_BASE + 0x04, 0x0000_0064); // I2C_SCL_HIGH
    assert_eq!(d.read32(RTC_I2C_BASE), 0x0000_0032);
    assert_eq!(d.read32(RTC_I2C_BASE + 0x04), 0x0000_0064);
}

#[test]
fn ctrl_register_round_trips() {
    let mut d = RtcI2c::new();
    d.write32(RTC_I2C_BASE + 0x0C, 0x00FF_00AA); // I2C_CTRL
    assert_eq!(d.read32(RTC_I2C_BASE + 0x0C), 0x00FF_00AA);
}

#[test]
fn unwritten_register_reads_zero() {
    let mut d = RtcI2c::new();
    assert_eq!(d.read32(RTC_I2C_BASE + 0x08), 0); // I2C_MS_DELAY
}
