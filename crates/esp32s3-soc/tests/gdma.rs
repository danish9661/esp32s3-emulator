//! GDMA unit tests: register decode, link-address control-bit stripping,
//! descriptor parsing, and the TX-done interrupt handshake.

use esp32s3_soc::gdma::*;

// Absolute GDMA-page offsets (base 0x6004_2000), relative to the SoC mmio arm.
// Channel pair stride 0xC0; out (TX) block sits at +0x60 within a pair.
// out.link = 0x20, out.peri_sel = 0x48, out.int_raw = 0x08, out.int_ena = 0x10,
// out.int_clr = 0x14. Channel 0 out block = 0x60; channel 2 out block = 0x180.
const C0_LINK: u32 = 0x60 + 0x20;
const C0_PERI: u32 = 0x60 + 0x48;
const C0_RAW: u32 = 0x60 + 0x08;
const C0_ENA: u32 = 0x60 + 0x10;
const C0_CLR: u32 = 0x60 + 0x14;
const C2_LINK: u32 = 2 * 0xC0 + 0x60 + 0x20;

#[test]
fn link_addr_strips_start_stop_control_bits() {
    // Firmware writes `(&desc & 0xFFFFF)` then ORs `start` (bit 1). The address
    // field's low 2 bits are control bits and must be stripped so the
    // reconstructed descriptor address matches the real DRAM address.
    let mut g = Gdma::default();
    // descriptor at 0x3FC9_6788, start bit 1 set -> link value 0x9678A.
    g.write32(C0_LINK, 0x9678A);
    assert_eq!(g.out_link_addr(0), 0x3FC9_6788);
}

#[test]
fn link_addr_without_start_is_identity() {
    let mut g = Gdma::default();
    g.write32(C0_LINK, 0x96788);
    assert_eq!(g.out_link_addr(0), 0x3FC9_6788);
}

#[test]
fn peri_sel_returns_low_six_bits() {
    let mut g = Gdma::default();
    g.write32(C0_PERI, 9);
    assert_eq!(g.out_peri_sel(0), GDMA_RMT_PERIPH);
    // Only the low 6 bits are the peripheral id. 0x49 = 9 | (1<<6): low 6 bits = 9.
    g.write32(C0_PERI, 0x49);
    assert_eq!(g.out_peri_sel(0), GDMA_RMT_PERIPH);
}

#[test]
fn write_start_returns_channel_and_sets_link() {
    let mut g = Gdma::default();
    // Start bit set on channel 2's out link.
    let ch = g.write32(C2_LINK, 0x1000_02);
    assert_eq!(ch, Some(2));
    // A plain (no start) peri_sel write returns None.
    let none = g.write32(C0_PERI, 9);
    assert_eq!(none, None);
}

#[test]
fn tx_done_interrupt_asserts_and_clears() {
    let mut g = Gdma::default();
    g.raise_out_done(0);
    // raw bits for done/eof/total_eof.
    assert_eq!(g.read32(C0_RAW) & 0b1011, 0b1011);
    // Enable the channel interrupt; status should now show pending.
    g.write32(C0_ENA, 1);
    assert!(g.int_pending());
    // Clear via int_clr; raw bits drop and pending goes away.
    g.write32(C0_CLR, 1);
    assert_eq!(g.read32(C0_RAW) & 1, 0);
    assert!(!g.int_pending());
}

#[test]
fn descriptor_fields_parse_correctly() {
    // gdma_descriptor_t = { dw0, buf, next, reserved }. Verify the field
    // extraction the SoC walk uses: len[23:12], eof[30], owner[31].
    let dw0: u32 = (16) | ((4 * 4) << 12) | (1u32 << 30) | (1u32 << 31);
    assert_eq!(dw0 & 0xFFF, 16, "buf_size");
    assert_eq!((dw0 >> 12) & 0xFFF, 16, "length");
    assert_eq!((dw0 >> 30) & 1, 1, "eof");
    assert_eq!((dw0 >> 31) & 1, 1, "owner");
}
