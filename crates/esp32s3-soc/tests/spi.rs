//! GPSPI master model unit tests (USR transfers, clock divider, MISO).

use esp32s3_soc::spi::*;

/// Configures `spi` for an 8-bit MOSI transfer of `byte` at 2 APB cycles
/// per bit.
fn setup_mosi(spi: &mut Spi, byte: u8) {
    spi.write32(SPI_CLOCK, 0x1000); // clkdiv_pre=0, clkcnt_n=1 -> 2 cyc/bit
    spi.write32(SPI_MS_DLEN, 7);
    spi.write32(SPI_DATA_BUF, u32::from(byte) << 24);
    spi.write32(SPI_USER, 1 << 27); // usr_mosi
    spi.write32(SPI_CLK_GATE, 1);
}

/// Samples (clock, MOSI) and advances one APB cycle.
fn sample(spi: &mut Spi) -> (u32, u32) {
    let (ck, d) = (spi.signal_level(101), spi.signal_level(103));
    spi.tick(1);
    (ck, d)
}

/// An 8-bit MOSI transfer shifts the byte out MSB first with one clock
/// pulse per bit and self-clears CMD.usr afterwards.
#[test]
fn mosi_byte_shifts_msb_first() {
    let mut s = Spi::new(0);
    setup_mosi(&mut s, 0xA5);
    s.write32(SPI_CMD, 1 << 24);
    let expected = 0xA5u8;
    let mut bits = 0u32;
    for i in 0..8 {
        let exp_d = u32::from((expected >> (7 - i)) & 1);
        assert_eq!(sample(&mut s), (0, exp_d), "slot {i} low half");
        let (ck, d) = sample(&mut s);
        assert_eq!(ck, 1, "slot {i} high half");
        bits = (bits << 1) | d;
    }
    assert_eq!(bits, 0xA5);
    // Clock returns to idle low after the 16th cycle.
    assert_eq!(s.signal_level(101), 0);
    assert_eq!(s.read32(SPI_CMD) & (1 << 24), 0, "usr self-clears");
    assert_eq!(s.signal_level(103), 0, "MOSI idles at d_pol=0");
}

/// The SPI clock divider: 3 APB cycles per bit with clkdiv_pre=1,
/// clkcnt_n=1 (freq = sys/(pre+1)/(n+1), TRM SPI_CLOCK).
#[test]
fn clock_divider_prescaler() {
    let mut s = Spi::new(0);
    s.write32(SPI_CLOCK, (1 << 18) | (1 << 12)); // pre=1, n=1 -> 3 cyc/bit
    s.write32(SPI_MS_DLEN, 7);
    s.write32(SPI_DATA_BUF, 0x80 << 24);
    s.write32(SPI_USER, 1 << 27);
    s.write32(SPI_CLK_GATE, 1);
    s.write32(SPI_CMD, 1 << 24);
    let mut pulses = 0u32;
    let mut high = 0u32;
    for _ in 0..32 {
        if s.signal_level(101) == 1 {
            high += 1;
        }
        let prev = s.signal_level(101);
        s.tick(1);
        if s.signal_level(101) == 1 && prev == 0 {
            pulses += 1;
        }
    }
    // 4-cycle slots (pre=1, n=1): two 2-cycle clock periods per bit slot.
    assert_eq!(pulses, 16, "two clock pulses per bit");
    assert_eq!(high, 16);
}

/// A MISO-only read transaction samples zeros into the buffer
/// (no device on the bus) left-aligned MSB first.
#[test]
fn miso_read_returns_zeros() {
    let mut s = Spi::new(0);
    s.write32(SPI_CLOCK, 0x1000);
    s.write32(SPI_MS_DLEN, 15); // 16 bits
    s.write32(SPI_USER, 1 << 28); // usr_miso
    s.write32(SPI_CLK_GATE, 1);
    s.write32(SPI_CMD, 1 << 24);
    for _ in 0..64 {
        s.tick(1);
    }
    assert_eq!(s.read32(SPI_DATA_BUF), 0, "undriven MISO reads 0");
    assert_eq!(s.read32(SPI_CMD) & (1 << 24), 0);
}

/// clk_equ_sysclk makes the bit rate one APB cycle per bit.
#[test]
fn equ_sysclk_fast_clock() {
    let mut s = Spi::new(0);
    s.write32(SPI_CLOCK, 1 << 31);
    s.write32(SPI_MS_DLEN, 7);
    s.write32(SPI_DATA_BUF, 0xFF << 24);
    s.write32(SPI_USER, 1 << 27);
    s.write32(SPI_CLK_GATE, 1);
    s.write32(SPI_CMD, 1 << 24);
    s.tick(1);
    assert_eq!(s.signal_level(101), 1, "second cycle of 1-cycle slot");
    for _ in 0..16 {
        s.tick(1);
    }
    assert_eq!(s.read32(SPI_CMD) & (1 << 24), 0, "done after 8 cycles");
}

/// The module clock gate must be enabled or the transfer never runs.
#[test]
fn clk_gate_halts_transfer() {
    let mut s = Spi::new(0);
    s.write32(SPI_CLOCK, 0x1000);
    s.write32(SPI_MS_DLEN, 7);
    s.write32(SPI_DATA_BUF, 0x55 << 24);
    s.write32(SPI_USER, 1 << 27);
    s.write32(SPI_CMD, 1 << 24);
    assert_eq!(sample(&mut s), (0, 0), "no clock without clk_en");
    assert_eq!(s.read32(SPI_CMD) & (1 << 24), 1 << 24, "usr stays set");
    s.write32(SPI_CLK_GATE, 1);
    assert_eq!(s.signal_level(101), 0, "clock runs once gated");
    s.tick(1);
    assert_eq!(s.signal_level(101), 1);
}
