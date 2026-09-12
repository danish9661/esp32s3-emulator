//! UART unit tests: RXFIFO_TOUT fires after the programmed idle period with
//! data pending (and only then).

use esp32s3_soc::uart::*;

/// CONF1 with rx_tout_en + MEM_CONF with rx_tout_thrhd = 10 bit-times;
/// CLKDIV divisor 100 (1 tick/bit => timeout after ~1000 ticks).
fn setup_tout(u: &mut Uart) {
    u.write32(UART_CLKDIV, 100);
    u.write32(UART_MEM_CONF, 10 << 17);
    u.write32(UART_CONF1, 1 << 23);
}

/// With tout enabled and a byte pending, TOUT latches after the idle period.
#[test]
fn tout_fires_after_idle_period() {
    let mut u = Uart::new();
    setup_tout(&mut u);
    u.inject_rx(b'Z');
    assert_eq!(u.read32(UART_INT_RAW) & (1 << 8), 0, "not yet");
    u.tick(999);
    assert_eq!(u.read32(UART_INT_RAW) & (1 << 8), 0, "still early");
    u.tick(1);
    assert_eq!(u.read32(UART_INT_RAW) & (1 << 8), 1 << 8, "TOUT latched");
    // INT_CLR clears it; with data still pending it re-arms and refires.
    u.write32(UART_INT_CLR, 1 << 8);
    assert_eq!(u.read32(UART_INT_RAW) & (1 << 8), 0);
    u.tick(1000);
    assert_eq!(u.read32(UART_INT_RAW) & (1 << 8), 1 << 8, "re-armed");
}

/// A fresh byte resets the idle counter.
#[test]
fn tout_idle_resets_on_new_byte() {
    let mut u = Uart::new();
    setup_tout(&mut u);
    u.inject_rx(b'A');
    u.tick(900);
    u.inject_rx(b'B');
    u.tick(900);
    assert_eq!(
        u.read32(UART_INT_RAW) & (1 << 8),
        0,
        "900 < 1000 after the reset"
    );
    u.tick(100);
    assert_eq!(u.read32(UART_INT_RAW) & (1 << 8), 1 << 8);
}

/// Disabled timeout (reset default) never fires, however long data waits.
#[test]
fn tout_disabled_is_inert() {
    let mut u = Uart::new();
    u.write32(UART_CLKDIV, 100);
    u.inject_rx(b'Z');
    u.tick(100_000);
    assert_eq!(u.read32(UART_INT_RAW) & (1 << 8), 0);
}

/// Draining the FIFO stops the counter (nothing pending, fast path).
#[test]
fn tout_idle_with_empty_fifo() {
    let mut u = Uart::new();
    setup_tout(&mut u);
    u.inject_rx(b'Z');
    assert_eq!(u.read32(UART_FIFO), u32::from(b'Z'));
    u.tick(100_000);
    assert_eq!(u.read32(UART_INT_RAW) & (1 << 8), 0);
}

/// RXFIFO_FULL is gated on the CONF1 threshold: a 1-byte burst with the
/// reset threshold (96) does not latch FULL.
#[test]
fn full_gated_on_threshold() {
    let mut u = Uart::new();
    u.inject_rx(b'Z');
    assert_eq!(u.read32(UART_INT_RAW) & 1, 0, "below threshold");
    // Programming the threshold to 1 latches FULL with data pending.
    u.write32(UART_CONF1, 1);
    assert_eq!(u.read32(UART_INT_RAW) & 1, 1, "FULL latched");
    // Draining drops it again (level-style).
    assert_eq!(u.read32(UART_FIFO), u32::from(b'Z'));
    assert_eq!(u.read32(UART_INT_RAW) & 1, 0);
}

/// A burst reaching the programmed threshold latches FULL on arrival.
#[test]
fn full_fires_at_threshold() {
    let mut u = Uart::new();
    u.write32(UART_CONF1, 3);
    u.inject_rx(b'a');
    u.inject_rx(b'b');
    assert_eq!(u.read32(UART_INT_RAW) & 1, 0);
    u.inject_rx(b'c');
    assert_eq!(u.read32(UART_INT_RAW) & 1, 1);
}

/// RX FIFO caps at the 128-byte hardware depth: longer bursts drop the
/// excess (overrun) instead of reporting a count the REPL ISR would copy
/// past its 128B stack buffer (150B bursts crashed MicroPython).
#[test]
fn rx_fifo_caps_at_hardware_depth() {
    let mut u = Uart::new();
    for _ in 0..200 {
        u.inject_rx(b'A');
    }
    let status = u.read32(UART_STATUS);
    assert_eq!(status & 0x3FF, 128, "count saturates at depth");
    for _ in 0..128 {
        assert_eq!(u.read32(UART_FIFO) as u8, b'A');
    }
    assert_eq!(u.read32(UART_FIFO), 0, "overrun bytes were dropped");
}

/// RS485 echo (RS485_CONF rs485_en + rs485tx_rx_en): a TX byte loops back
/// into the RX FIFO; with only rs485_en (receiver muted during TX) or in
/// normal mode nothing echoes.
#[test]
fn rs485_echo_needs_en_and_tx_rx_en() {
    let mut u = Uart::new();
    // Normal mode: TX never echoes.
    u.write32(UART_FIFO, 0x5A);
    assert_eq!(u.read32(UART_FIFO), 0, "no echo in normal mode");
    // RS485 without the echo bit: receiver muted during TX.
    u.write32(UART_RS485_CONF, 1 << 0);
    u.write32(UART_FIFO, 0x5A);
    assert_eq!(u.read32(UART_FIFO), 0, "muted without tx_rx_en");
    // RS485 + echo bit: TX byte loops back.
    u.write32(UART_RS485_CONF, (1 << 0) | (1 << 3));
    u.write32(UART_FIFO, 0x5A);
    assert_eq!(u.read32(UART_FIFO), 0x5A, "echo byte");
    assert_eq!(u.read32(UART_FIFO), 0, "FIFO drained");
}

/// Hardware flow control (CONF0 TX_FLOW_EN[15] / RX_FLOW_EN[22],
/// uart_reg.h): with TX flow on and CTS high the transmitter holds bytes
/// (EMPTY/DONE low, TXFIFO_CNT live); a CTS drop flushes them. CTS_CHG
/// latches on every CTS edge.
#[test]
fn tx_flow_control_holds_and_flushes_on_cts() {
    let mut u = Uart::new();
    // TX_FLOW_EN + empty threshold 1 (EMPTY reads low once a byte is
    // held); clear the reset EMPTY/DONE latches first.
    u.write32(UART_CONF0, 1 << 15);
    u.write32(UART_CONF1, 96 | (1 << 10));
    u.write32(UART_INT_CLR, 0xFFFF);
    // Default CTS = pull-high (stop).
    u.write32(UART_FIFO, 0x41);
    u.write32(UART_FIFO, 0x42);
    assert!(u.take_tx().is_empty(), "held while CTS high");
    let st = u.read32(UART_STATUS);
    assert_eq!((st >> 16) & 0x3FF, 2, "TXFIFO_CNT live, st={st:#x}");
    assert_eq!(st & (1 << 14), 1 << 14, "CTSn high, st={st:#x}");
    assert_eq!(u.read32(UART_INT_RAW) & INT_TXFIFO_EMPTY, 0, "EMPTY low");
    assert_eq!(u.read32(UART_INT_RAW) & INT_TX_DONE, 0, "DONE low");
    // CTS drop (go): flush + DONE latch + CTS_CHG edge.
    u.set_cts(0);
    assert_eq!(u.take_tx(), vec![0x41, 0x42], "flushed on CTS go");
    assert_eq!(u.read32(UART_STATUS) & (1 << 14), 0, "CTSn low");
    assert_ne!(u.read32(UART_INT_RAW) & INT_CTS_CHG, 0, "CTS_CHG latched");
    assert_ne!(u.read32(UART_INT_RAW) & INT_TX_DONE, 0, "DONE on flush");
}

/// RX flow control drives RTSn (STATUS[30], U_RTSn signal) from the RX
/// level against MEM_CONF RX_FLOW_THRHD[16:7]: ready (0) below, stop (1)
/// at/above. Disabled flow idles RTSn high.
#[test]
fn rx_flow_control_drives_rtsn_from_level() {
    let mut u = Uart::new();
    assert_eq!(u.rts_level(), 1, "RTSn idle high without flow");
    u.write32(UART_CONF0, 1 << 22);
    u.write32(UART_MEM_CONF, 4 << 7); // threshold 4
    assert_eq!(u.rts_level(), 0, "RTSn ready while empty");
    u.inject_rx(b'A');
    u.inject_rx(b'B');
    u.inject_rx(b'C');
    assert_eq!(u.rts_level(), 0, "ready below threshold");
    u.inject_rx(b'D');
    assert_eq!(u.rts_level(), 1, "stop at threshold");
    assert_eq!(u.read32(UART_STATUS) & (1 << 30), 1 << 30, "RTSn in STATUS");
}

/// The sticky RX edge survives a FIFO drain (sleep-entry FIFO reset): an
/// injected-then-read byte still reports its edge exactly once.
#[test]
fn rx_edge_sticky_until_taken() {
    let mut u = Uart::new();
    assert!(!u.take_rx_edge(), "no edge at reset");
    u.inject_rx(b'Z');
    assert!(u.rx_pending());
    assert_eq!(u.read32(UART_FIFO), u32::from(b'Z'));
    assert!(!u.rx_pending(), "FIFO drained");
    assert!(u.take_rx_edge(), "edge survives the drain");
    assert!(!u.take_rx_edge(), "edge consumed once");
}

/// LOOPBACK (CONF0 bit 14, uart_reg.h loopback test mode): transmitted
/// bytes re-enter the receiver unconditionally (unlike the RS485 echo,
/// no mode bit needed).
#[test]
fn loopback_feeds_tx_into_rx() {
    let mut u = Uart::new();
    u.write32(UART_CONF0, 1 << 14);
    u.write32(UART_FIFO, 0xA5);
    assert_eq!(u.read32(UART_FIFO), 0xA5, "looped back");
    assert_eq!(u.read32(UART_FIFO), 0, "FIFO drained");
}
