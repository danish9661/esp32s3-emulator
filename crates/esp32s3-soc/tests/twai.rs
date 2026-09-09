//! TWAI unit tests: reset mode, acceptance-filter config, self-test loopback,
//! RX release, and the interrupt handshake.

use esp32s3_soc::twai::*;

const MODE: u32 = 0x00;
const CMD: u32 = 0x04;
const STATUS: u32 = 0x08;
const IER: u32 = 0x10;

#[test]
fn powers_up_in_reset_mode_with_tx_free() {
    let mut t = Twai::new();
    assert_eq!(t.read32(MODE) & 1, 1, "rm must be 1 after reset");
    // tbs (bit2) | tcs (bit3)
    assert_eq!(t.read32(STATUS) & 0x0C, 0x0C, "TX buffer free + complete");
}

#[test]
fn acceptance_filter_configured_only_in_reset_mode() {
    let mut t = Twai::new();
    // In reset mode, 0x40..0x4C are ACR, 0x50..0x5C are AMR.
    t.write32(0x40, 0x11);
    t.write32(0x50, 0x22);
    assert_eq!(t.read32(0x40), 0x11, "ACR readable in reset mode");
    assert_eq!(t.read32(0x50), 0x22, "AMR readable in reset mode");
    // Leave reset (operational). The same offsets now address the TX buffer.
    t.write32(MODE, 0);
    t.write32(0x40, 0xAB);
    t.write32(0x50, 0xCD);
    assert_eq!(
        t.read32(0x40),
        0xAB,
        "0x40 is TX buffer[0] in operational mode"
    );
    assert_eq!(
        t.read32(0x50),
        0xCD,
        "0x50 is TX buffer[4] in operational mode"
    );
}

#[test]
fn self_test_loopback_receives_transmitted_frame() {
    let mut t = Twai::new();
    // Enter reset mode and program accept-all (AMR all 0xFF), then leave reset
    // in self-test mode (stm = bit2), rm = 0.
    t.write32(MODE, 1);
    for off in [0x50u32, 0x54, 0x58, 0x5C] {
        t.write32(off, 0xFF);
    }
    t.write32(MODE, 1 << 2);
    // Load a 13-byte frame into the TX buffer (operational mode).
    let tx: [u8; 13] = [
        0x08, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x01, 0x23, 0x45, 0x67,
    ];
    for (i, b) in tx.iter().enumerate() {
        t.write32(0x40 + (i as u32) * 4, *b as u32);
    }
    // Issue transmission request.
    t.write32(CMD, 1);
    // Status: TX complete (tcs) + RX buffer full (rbs).
    let st = t.read32(STATUS);
    assert_ne!(st & (1 << 3), 0, "tcs: transmission complete");
    assert_ne!(st & (1 << 0), 0, "rbs: frame in RX buffer");
    // Read back the RX buffer and compare to what was transmitted.
    let mut rx = [0u8; 13];
    for (slot, b) in rx.iter_mut().enumerate() {
        *b = t.read32(0x40 + (slot as u32) * 4) as u8;
    }
    assert_eq!(rx, tx, "looped-back frame must equal transmitted frame");
}

#[test]
fn transmit_asserts_ti_and_ri_then_release_clears_ri() {
    let mut t = Twai::new();
    t.write32(MODE, 1 << 2);
    t.write32(IER, (1 << 0) | (1 << 1)); // enable receive + transmit interrupts
    t.write32(0x40, 0x08);
    t.write32(CMD, 1);
    assert!(t.int_pending(), "TI and/or RI should be pending");
    assert_ne!(t.int_st() & ((1 << 0) | (1 << 1)), 0, "RI and TI set");
    // Release the RX buffer: clears rbs and RI.
    t.write32(CMD, 1 << 2);
    assert_eq!(t.read32(STATUS) & (1 << 0), 0, "rbs cleared after RRB");
    assert_eq!(t.int_st() & (1 << 0), 0, "RI cleared after RRB");
}

#[test]
fn accept_all_mask_passes_every_frame() {
    let mut t = Twai::new();
    // Enter reset mode, program the acceptance filter to accept-all (AMR all
    // 0xFF = don't-care every bit), then leave reset in self-test mode.
    t.write32(MODE, 1);
    for off in [0x50u32, 0x54, 0x58, 0x5C] {
        t.write32(off, 0xFF);
    }
    t.write32(MODE, 1 << 2);
    // Transmit a frame with a non-zero header.
    let tx: [u8; 13] = [
        0x04, 0xFF, 0x00, 0xAA, 0x01, 0x02, 0x03, 0x04, 0, 0, 0, 0, 0,
    ];
    for (i, b) in tx.iter().enumerate() {
        t.write32(0x40 + (i as u32) * 4, *b as u32);
    }
    t.write32(CMD, 1);
    assert_ne!(
        t.read32(STATUS) & (1 << 0),
        0,
        "accept-all filter lets the frame through to RX"
    );
}
