//! I2C master model unit tests (command list, SCL/SDA waveform, NACK,
//! clock divider).

use esp32s3_soc::i2c::*;
use esp32s3_soc::soc::{EVT_I2C_READ, EVT_I2C_START, EVT_I2C_STOP, EVT_I2C_WRITE};

/// Configures `i2c` for a fast master: low = 2 APB cycles, high = 1 APB
/// cycle per SCL pulse (scl_low_period=1, scl_high_period=1, div=1).
fn setup_master(i2c: &mut I2c) {
    i2c.write32(I2C_CTR, 1 << 4); // ms_mode
    i2c.write32(I2C_SCL_LOW_PERIOD, 1);
    i2c.write32(I2C_SCL_HIGH_PERIOD, 1);
    i2c.write32(I2C_SCL_START_HOLD, 0);
    i2c.write32(I2C_SCL_STOP_HOLD, 0);
    i2c.write32(I2C_SCL_STOP_SETUP, 0);
}

/// Samples (SCL, SDA) on I2CEXT0 signals 89/90 and advances one APB cycle.
fn sample(i2c: &mut I2c) -> (u32, u32) {
    let (scl, sda) = (i2c.signal_level(89), i2c.signal_level(90));
    i2c.tick(1);
    (scl, sda)
}

/// A master write to a nonexistent device: START, address byte 0xA0,
/// NACK, data byte 0xAA, NACK, STOP, END — with the SCL/SDA waveform
/// (18 pulses) and the STOP condition.
#[test]
fn master_write_shifts_addr_and_nacks() {
    let mut i2c = I2c::new(0);
    setup_master(&mut i2c);
    i2c.write32(I2C_DATA, 0xA0); // addr 0x50, write
    i2c.write32(I2C_DATA, 0xAA);
    i2c.write32(I2C_COMD, 6 << 11); // RSTART
    i2c.write32(I2C_COMD + 4, (1 << 11) | 1); // WRITE 1 byte
    i2c.write32(I2C_COMD + 8, (1 << 11) | 1); // WRITE 1 byte
    i2c.write32(I2C_COMD + 12, 2 << 11); // STOP
    i2c.write32(I2C_COMD + 16, 4 << 11); // END
    assert_eq!(i2c.signal_level(89), 1, "no activity before trans_start");
    assert_eq!(i2c.read32(I2C_COMD + 16) & (1 << 31), 0, "not started");
    // Sample the idle state, then trigger — the START condition needs the
    // preceding (1,1) sample (SDA falling while SCL high) to be detected
    // against the STOP's SDA dip.
    let mut samples = Vec::new();
    samples.push(sample(&mut i2c));
    i2c.write32(I2C_CTR, (1 << 4) | (1 << 5)); // trans_start
    for _ in 0..100 {
        samples.push(sample(&mut i2c));
    }
    // START: SDA falls while SCL high, then SCL falls.
    let start = samples
        .windows(2)
        .position(|w| w[0] == (1, 1) && w[1] == (1, 0))
        .expect("START condition");
    assert_eq!(samples[start + 2], (0, 1), "SCL falls, SDA = first bit");
    // Bits at each SCL rising edge (low=2, high=1 -> edge every 3 cycles).
    let mut bits = Vec::new();
    for k in start + 1..samples.len() {
        if samples[k].0 == 1 && samples[k - 1].0 == 0 {
            bits.push(samples[k].1);
        }
    }
    assert_eq!(bits.len(), 18);
    let expected = [1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1];
    for (k, e) in expected.iter().enumerate() {
        assert_eq!(bits[k], *e, "pulse {k}");
    }
    // STOP: SDA rises while SCL high.
    let last_edge = (start + 1..samples.len())
        .filter(|&k| samples[k].0 == 1 && samples[k - 1].0 == 0)
        .nth(17)
        .unwrap();
    let stop = samples[last_edge + 1..]
        .windows(2)
        .position(|w| w[0] == (1, 0) && w[1] == (1, 1))
        .expect("STOP condition");
    assert!(stop > 0);
    for s in samples.iter().skip(last_edge + 1 + stop + 2) {
        assert_eq!(*s, (1, 1), "idle high after STOP");
    }
    // NACK latched; all commands done; trans_complete raw raised.
    assert_eq!(i2c.read32(I2C_SR) & 1, 1, "resp_rec = NACK");
    assert_eq!(i2c.read32(I2C_SR) & (1 << 4), 0, "bus idle");
    for off in [0, 4, 8, 12, 16] {
        assert_ne!(i2c.read32(I2C_COMD + off) & (1 << 31), 0, "comd {off} done");
    }
    assert_ne!(i2c.read32(I2C_INT_RAW) & (1 << 7), 0, "trans_complete");
}

/// A master READ shifts in 1s (no device) and pushes the bytes into the
/// RX FIFO, sending the comd.ack_val level after each byte.
#[test]
fn master_read_shifts_ones_into_fifo() {
    let mut i2c = I2c::new(0);
    setup_master(&mut i2c);
    i2c.write32(I2C_COMD, 6 << 11); // RSTART
    i2c.write32(I2C_COMD + 4, (3 << 11) | 2); // READ 2 bytes
    i2c.write32(I2C_COMD + 8, 2 << 11); // STOP
    i2c.write32(I2C_COMD + 12, 4 << 11); // END
    let mut samples = Vec::new();
    samples.push(sample(&mut i2c));
    i2c.write32(I2C_CTR, (1 << 4) | (1 << 5));
    for _ in 0..140 {
        samples.push(sample(&mut i2c));
    }
    let start = samples
        .windows(2)
        .position(|w| w[0] == (1, 1) && w[1] == (1, 0))
        .expect("START condition");
    let mut bits = Vec::new();
    for k in start + 1..samples.len() {
        if samples[k].0 == 1 && samples[k - 1].0 == 0 {
            bits.push(samples[k].1);
        }
    }
    // 8 data + ACK0 + 8 data + ACK0 = 18 pulses, all data bits 1.
    assert_eq!(bits.len(), 18);
    let expected = [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0];
    for (k, e) in expected.iter().enumerate() {
        assert_eq!(bits[k], *e, "pulse {k}");
    }
    // Both bytes land in the RX FIFO as 0xFF (undriven line).
    assert_eq!(i2c.read32(I2C_SR) >> 8 & 0x3F, 2, "rx fifo count");
    assert_eq!(i2c.read32(I2C_DATA), 0xFF);
    assert_eq!(i2c.read32(I2C_DATA), 0xFF);
    assert_eq!(i2c.read32(I2C_DATA), 0, "empty fifo reads 0");
}

/// A master READ with an injected slave byte delivers it into the RX FIFO
/// and emits EVT_I2C_START / EVT_I2C_READ / EVT_I2C_STOP events (Wokwi-style
/// virtual-I2C round trip).
#[test]
fn read_with_injected_rx_delivers_byte_and_emits_events() {
    let mut i2c = I2c::new(0);
    setup_master(&mut i2c);
    i2c.inject_rx(&[0x57]); // virtual device's response
    i2c.write32(I2C_COMD, 6 << 11); // RSTART
    i2c.write32(I2C_COMD + 4, (3 << 11) | 1); // READ 1 byte
    i2c.write32(I2C_COMD + 8, 2 << 11); // STOP
    i2c.write32(I2C_COMD + 12, 4 << 11); // END
    i2c.write32(I2C_CTR, (1 << 4) | (1 << 5)); // trans_start
    let mut evs = Vec::new();
    for _ in 0..100 {
        i2c.tick(1);
        evs.extend(i2c.drain_events());
    }
    assert_eq!(i2c.read32(I2C_SR) >> 8 & 0x3F, 1, "rx fifo count");
    assert_eq!(i2c.read32(I2C_DATA), 0x57, "injected byte");
    assert!(evs.iter().any(|e| e.kind == EVT_I2C_START && e.a == 0));
    assert!(
        evs.iter()
            .any(|e| e.kind == EVT_I2C_READ && e.a == 0 && e.b == 0x57)
    );
    assert!(evs.iter().any(|e| e.kind == EVT_I2C_STOP && e.a == 0));
}

/// A master WRITE emits EVT_I2C_WRITE events carrying each transmitted byte.
#[test]
fn write_emits_write_events() {
    let mut i2c = I2c::new(0);
    setup_master(&mut i2c);
    i2c.write32(I2C_DATA, 0xA0); // addr 0x50, write
    i2c.write32(I2C_DATA, 0xAA);
    i2c.write32(I2C_COMD, 6 << 11); // RSTART
    i2c.write32(I2C_COMD + 4, (1 << 11) | 1); // WRITE 1 byte
    i2c.write32(I2C_COMD + 8, (1 << 11) | 1); // WRITE 1 byte
    i2c.write32(I2C_COMD + 12, 2 << 11); // STOP
    i2c.write32(I2C_COMD + 16, 4 << 11); // END
    i2c.write32(I2C_CTR, (1 << 4) | (1 << 5)); // trans_start
    let mut evs = Vec::new();
    for _ in 0..100 {
        i2c.tick(1);
        evs.extend(i2c.drain_events());
    }
    assert!(
        evs.iter()
            .any(|e| e.kind == EVT_I2C_WRITE && e.a == 0 && e.b == 0xA0)
    );
    assert!(
        evs.iter()
            .any(|e| e.kind == EVT_I2C_WRITE && e.a == 0 && e.b == 0xAA)
    );
}

/// clk_conf.sclk_div_num scales every timing register (module clock =
/// APB / (div+1)): with div=1 the SCL pulse is 6 APB cycles.
#[test]
fn sclk_div_scales_period() {
    let mut i2c = I2c::new(0);
    i2c.write32(I2C_CTR, 1 << 4);
    i2c.write32(I2C_CLK_CONF, 1); // sclk_div_num = 1 -> div 2
    i2c.write32(I2C_SCL_LOW_PERIOD, 1); // low = (1+1)*2 = 4
    i2c.write32(I2C_SCL_HIGH_PERIOD, 1); // high = 1*2 = 2
    i2c.write32(I2C_SCL_START_HOLD, 0);
    i2c.write32(I2C_SCL_STOP_HOLD, 0);
    i2c.write32(I2C_SCL_STOP_SETUP, 0);
    i2c.write32(I2C_DATA, 0x80);
    i2c.write32(I2C_COMD, 6 << 11);
    i2c.write32(I2C_COMD + 4, (1 << 11) | 1);
    i2c.write32(I2C_COMD + 8, 2 << 11);
    i2c.write32(I2C_COMD + 12, 4 << 11);
    i2c.write32(I2C_CTR, (1 << 4) | (1 << 5));
    // START hold (1) + 9 pulses * 6 cycles.
    let mut edges = Vec::new();
    let mut prev = i2c.signal_level(89);
    let mut t = 0u32;
    while edges.len() < 3 {
        i2c.tick(1);
        t += 1;
        let cur = i2c.signal_level(89);
        if cur == 1 && prev == 0 {
            edges.push(t);
        }
        prev = cur;
    }
    assert_eq!(edges[1] - edges[0], 6, "6 APB cycles per pulse");
    assert_eq!(edges[2] - edges[1], 6);
}

/// A WRITE issued with an empty TX FIFO must still drive the bus and, with
/// no slave present, the SDA line is released high.  On real ESP32-S3
/// silicon the INT_NACK interrupt is NOT latched for the address byte's ACK
/// cycle during master-transmit — only for data bytes during master-receive.
/// The END command latches INT_END_DETECT + INT_TRANS_COMPLETE so the
/// esp-idf master ISR takes the TRANS_COMPLETE (msg=1) path.
#[test]
fn write_with_empty_fifo_nacks() {
    let mut i2c = I2c::new(0);
    setup_master(&mut i2c);
    i2c.write32(I2C_COMD, (1 << 11) | 1);
    i2c.write32(I2C_CTR, (1 << 4) | (1 << 5));
    // Advance until the command completes; record whether SCL ever toggled.
    let mut scl_low_seen = false;
    for _ in 0..256 {
        if i2c.signal_level(89) == 0 {
            scl_low_seen = true;
        }
        i2c.tick(1);
        if i2c.read32(I2C_COMD) & (1 << 31) != 0 {
            break;
        }
    }
    assert!(scl_low_seen, "empty-FIFO WRITE must still drive SCL");
    assert_ne!(i2c.read32(I2C_COMD) & (1 << 31), 0, "comd done");
    // END clears INT_NACK so the ISR takes the TRANS_COMPLETE path.
    assert_eq!(
        i2c.read32(I2C_INT_RAW) & (1 << 10),
        0,
        "END clears nack_int_raw"
    );
    assert_ne!(
        i2c.read32(I2C_INT_RAW) & (1 << 7),
        0,
        "END sets trans_complete"
    );
    assert_ne!(
        i2c.read32(I2C_INT_RAW) & (1 << 3),
        0,
        "END sets end_detect"
    );
}

/// FIFO reset bits clear the pointers.
#[test]
fn fifo_reset_clears_counters() {
    let mut i2c = I2c::new(0);
    i2c.write32(I2C_DATA, 0x11);
    i2c.write32(I2C_DATA, 0x22);
    assert_eq!(i2c.read32(I2C_SR) >> 18 & 0x3F, 2, "tx fifo count");
    i2c.write32(I2C_FIFO_CONF, 1 << 13); // tx_fifo_rst
    assert_eq!(i2c.read32(I2C_SR) >> 18 & 0x3F, 0, "tx fifo cleared");
}
