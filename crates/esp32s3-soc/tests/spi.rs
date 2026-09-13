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
    // pre=1, n=1: one 4-tick SPI clock per bit slot (TRM SPI_CLOCK:
    // spi_clk = system/(clkdiv_pre+1)/(clkcnt_n+1)), low for
    // (n-h)*(pre+1) = 2 ticks of each cycle.
    assert_eq!(pulses, 8, "one clock pulse per bit");
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

/// A MOSI transfer exposes the MCU's byte stream via `take_last_tx` for the
/// host to inspect, and an injected MISO byte lands in the data buffer
/// (Wokwi-style virtual-SPI round trip).
#[test]
fn mosi_transfer_exposes_tx_and_injected_miso() {
    let mut s = Spi::new(0);
    setup_mosi(&mut s, 0xA5);
    s.write32(SPI_USER, (1 << 27) | (1 << 28)); // usr_mosi + usr_miso
    s.inject_miso(&[0x3C]); // virtual device's response
    s.write32(SPI_CMD, 1 << 24);
    for _ in 0..200 {
        sample(&mut s);
        if s.read32(SPI_CMD) & (1 << 24) == 0 {
            break;
        }
    }
    assert_eq!(s.read32(SPI_CMD) & (1 << 24), 0, "usr self-clears");
    // The host reads back exactly what the firmware transmitted.
    assert_eq!(s.take_last_tx(), Some(vec![0xA5]));
    // And the injected MISO byte is shifted into the buffer MSB-first.
    assert_eq!(s.read32(SPI_DATA_BUF) >> 24, 0x3C);
    // No leftover transfer pending.
    assert_eq!(s.take_last_tx(), None);
}

/// Slave mode: a host-driven master-write captures bytes into the data
/// buffer, records SLAVE1.data_bitlen, and latches trans_done.
#[test]
fn slave_inject_write_captures_rx_and_raises_done() {
    let mut s = Spi::new(0);
    s.write32(SPI_SLAVE, 1 << 26); // slave_mode
    assert!(s.is_slave());
    s.slave_inject_write(&[0x11, 0x22]);
    assert_eq!(s.read32(SPI_DATA_BUF), 0x1122_0000);
    assert_eq!(s.read32(SPI_SLAVE1) & 0x3FFFF, 16);
    assert_eq!(
        s.read32(SPI_INT_RAW) & (1 << 12),
        1 << 12,
        "trans_done latched"
    );
    // INT_CLR clears it (driver handshake).
    s.write32(SPI_INT_CLR, 1 << 12);
    assert_eq!(s.read32(SPI_INT_RAW) & (1 << 12), 0);
}

/// Slave mode: a host-driven master-read returns the firmware-preloaded TX
/// bytes and records the bitlen.
#[test]
fn slave_take_read_returns_preloaded_tx() {
    let mut s = Spi::new(0);
    s.write32(SPI_SLAVE, 1 << 26);
    s.write32(SPI_DATA_BUF, 0xA5C3_0000);
    let got = s.slave_take_read(2);
    assert_eq!(got, vec![0xA5, 0xC3]);
    assert_eq!(s.read32(SPI_SLAVE1) & 0x3FFFF, 16);
    assert_eq!(
        s.read32(SPI_INT_RAW) & (1 << 12),
        1 << 12,
        "trans_done latched"
    );
}

/// CMD.usr does not start a master transaction in slave mode, and the
/// slave host calls are inert in master mode.
#[test]
fn slave_mode_gates_master_trigger() {
    let mut s = Spi::new(0);
    s.write32(SPI_SLAVE, 1 << 26);
    setup_mosi(&mut s, 0xA5);
    s.write32(SPI_CMD, 1 << 24);
    for _ in 0..32 {
        s.tick(1);
    }
    // No master transaction ran: usr still set, no trans_done, no MOSI event.
    assert_eq!(s.read32(SPI_CMD) & (1 << 24), 1 << 24);
    assert_eq!(s.read32(SPI_INT_RAW) & (1 << 12), 0);

    let mut m = Spi::new(0);
    m.slave_inject_write(&[0xFF]);
    assert_eq!(m.read32(SPI_INT_RAW) & (1 << 12), 0, "inert in master mode");
    assert!(m.slave_take_read(1).is_empty());
}

/// DMA-backed transfer: GDMA-fed bytes shift out on MOSI, MISO (zeros with
/// no device) lands in dma_rx, trans_done latches, CMD.usr clears.
#[test]
fn dma_transfer_shifts_fed_bytes_and_captures_rx() {
    let mut s = Spi::new(0);
    s.write32(SPI_CLOCK, 0x1000); // 2 cyc/bit
    s.write32(SPI_MS_DLEN, 31);
    s.write32(SPI_USER, (1 << 27) | (1 << 28) | 1); // mosi+miso+doutdin
    s.write32(SPI_CLK_GATE, 1);
    s.spi_dma_feed(&[0xA5, 0x3C, 0xF0, 0x0F]);
    s.dma_trigger();
    for _ in 0..64 {
        s.tick(1);
    }
    assert_eq!(s.read32(SPI_CMD) & (1 << 24), 0, "usr clears");
    assert_eq!(s.read32(SPI_INT_RAW) & (1 << 12), 1 << 12, "trans_done");
    assert_eq!(s.dma_rx_word(0), 0, "MISO zeros with no device");
    // MOSI event stream carries the fed bytes (doutdin halves data_bits).
    let tx = s.take_last_tx().unwrap();
    assert_eq!(&tx[..2], &[0xA5, 0x3C]);
}

/// DMA transfer honors injected MISO into dma_rx.
#[test]
fn dma_transfer_captures_injected_miso() {
    let mut s = Spi::new(0);
    s.write32(SPI_CLOCK, 0x1000);
    s.write32(SPI_MS_DLEN, 15);
    s.write32(SPI_USER, (1 << 27) | (1 << 28) | 1);
    s.write32(SPI_CLK_GATE, 1);
    s.inject_miso(&[0x5A]);
    s.spi_dma_feed(&[0xFF, 0x00]);
    s.dma_trigger();
    for _ in 0..32 {
        s.tick(1);
    }
    assert_eq!(s.dma_rx_word(0) & 0xFF, 0x5A);
}

/// DMA transfers longer than the 64-byte data buffer work (sized buffer).
#[test]
fn dma_transfer_beyond_data_buffer() {
    let mut s = Spi::new(0);
    s.write32(SPI_CLOCK, 0x1000);
    s.write32(SPI_MS_DLEN, 639); // 640 bits
    s.write32(SPI_USER, (1 << 27) | (1 << 28) | 1);
    s.write32(SPI_CLK_GATE, 1);
    s.spi_dma_feed(&[0xA5; 80]);
    s.dma_trigger();
    for _ in 0..1280 {
        s.tick(1);
    }
    assert_eq!(s.read32(SPI_INT_RAW) & (1 << 12), 1 << 12, "trans_done");
    assert_eq!(s.dma_rx_word(76), 0);
}

/// Slave DMA enables gate on slave_mode + DMA_CONF bits; completion records
/// SLAVE1 bitlen with the RD/WR DMA-done latches (not trans_done).
#[test]
fn slave_dma_flags_and_done_bits() {
    let mut s = Spi::new(0);
    assert!(!s.slave_dma_rx_enabled());
    s.write32(SPI_SLAVE, 1 << 26);
    assert!(!s.slave_dma_rx_enabled() && !s.slave_dma_tx_enabled());
    s.write32(0x30, (1 << 25) | (1 << 26)); // dma_conf rx+tx ena
    assert!(s.slave_dma_rx_enabled() && s.slave_dma_tx_enabled());
    s.slave_dma_done(32, true);
    assert_eq!(s.read32(SPI_SLAVE1) & 0x3FFFF, 32);
    assert_eq!(s.read32(SPI_INT_RAW) & (1 << 9), 1 << 9, "WR_DMA_DONE");
    assert_eq!(s.read32(SPI_INT_RAW) & (1 << 12), 0, "no trans_done");
    s.slave_dma_done(16, false);
    assert_eq!(s.read32(SPI_INT_RAW) & (1 << 8), 1 << 8, "RD_DMA_DONE");
    s.write32(SPI_INT_CLR, (1 << 8) | (1 << 9));
    assert_eq!(s.read32(SPI_INT_RAW) & ((1 << 8) | (1 << 9)), 0);
}

/// Quad-mode decode from SPI_CTRL FREAD/FCMD/FADDR bits (spi_reg.h):
/// quad wins over dual, dual over single, default single.
#[test]
fn quad_mode_decodes_ctrl_bits() {
    let mut s = Spi::new(0);
    assert_eq!(s.quad_mode(), 0, "single by default");
    // CTRL @ 0x08: FREAD_DUAL[14], FREAD_QUAD[15], FCMD/FADDR mirrors.
    s.write32(SPI_CTRL, 1 << 14);
    assert_eq!(s.quad_mode(), 1, "dual");
    s.write32(SPI_CTRL, (1 << 14) | (1 << 15));
    assert_eq!(s.quad_mode(), 2, "quad wins over dual");
    s.write32(SPI_CTRL, 1 << 6); // FADDR_QUAD alone
    assert_eq!(s.quad_mode(), 2, "addr-phase quad counts");
}

/// Fake quad device: provisioned pattern serves MISO when a wide mode is
/// programmed (address-selected window), unprovisioned stays zeros, and
/// single-line mode never consults the store.
#[test]
fn quad_fake_device_serves_pattern_and_round_trips() {
    let mut s = Spi::new(0);
    let pat: Vec<u8> = (0..64u8).collect();
    s.quad_fake_provision(&pat);
    // 4-byte MISO read at addr 0x10 with FREAD_QUAD -> pattern[0x10..].
    s.write32(SPI_CLOCK, 0x1000);
    s.write32(SPI_ADDR, 0x10);
    s.write32(SPI_USER1, 23 << 27); // 24-bit address
    s.write32(SPI_MS_DLEN, 31); // 32 data bits
    s.write32(SPI_USER, (1 << 30) | (1 << 28)); // usr_addr + usr_miso
    s.write32(SPI_CTRL, 1 << 15); // FREAD_QUAD
    s.write32(SPI_CLK_GATE, 1);
    s.write32(SPI_CMD, 1 << 24);
    for _ in 0..256 {
        s.tick(1);
    }
    assert_eq!(s.read32(SPI_DATA_BUF), 0x10111213, "quad window at addr");
    // MOSI write at addr 0x20 commits into the store; read-back matches.
    s.write32(SPI_DATA_BUF, 0xDEADBEEF);
    s.write32(SPI_USER, (1 << 30) | (1 << 27)); // usr_addr + usr_mosi
    s.write32(SPI_CMD, 1 << 24);
    for _ in 0..256 {
        s.tick(1);
    }
    s.write32(SPI_USER, (1 << 30) | (1 << 28));
    s.write32(SPI_CMD, 1 << 24);
    for _ in 0..256 {
        s.tick(1);
    }
    assert_eq!(s.read32(SPI_DATA_BUF), 0xDEADBEEF, "write/read-back");
}

/// Sequential half-duplex (mosi+miso WITHOUT doutdin): the 8-bit MOSI byte
/// shifts out first, then the MISO byte shifts in — the Arduino polling
/// path (spiTransferByteNL programs usr_mosi|usr_miso, doutdin clear).
/// Proven by the SDSPI probe: USER=0x18000001 must return the injected
/// MISO byte, not zeros.
#[test]
fn sequential_halfduplex_miso_returns_injected_byte() {
    let mut s = Spi::new(0);
    s.write32(SPI_CLOCK, 0x1000); // 2 cyc/bit
    s.write32(SPI_MS_DLEN, 7); // d = 8 bits per direction
    s.write32(SPI_DATA_BUF, 0x40 << 24); // MOSI byte (CMD0-ish)
    s.write32(SPI_USER, (1 << 27) | (1 << 28)); // mosi+miso, NO doutdin
    s.write32(SPI_CLK_GATE, 1);
    s.inject_miso(&[0x01]); // idle-high response
    s.write32(SPI_CMD, 1 << 24);
    for _ in 0..64 {
        s.tick(1);
    }
    assert_eq!(s.read32(SPI_CMD) & (1 << 24), 0, "usr clears");
    assert_eq!(s.read32(SPI_DATA_BUF) >> 24, 0x01, "MISO byte lands");
    // MOSI event stream is the first d bits (the byte we sent).
    let tx = s.take_last_tx().unwrap();
    assert_eq!(tx, vec![0x40], "MOSI first half");
}
