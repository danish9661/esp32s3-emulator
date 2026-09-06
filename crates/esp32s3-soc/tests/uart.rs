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
