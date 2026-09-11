//! Flash-encryption fixture tests (public Soc API): eFuse gate, uniform
//! XTS region encryption, decrypted-view round-trip, erased-bypass.

use esp32s3_soc::soc::Soc;

const KEY: [u8; 32] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
];

#[test]
fn fresh_device_is_plaintext() {
    let s = Soc::new();
    assert!(!s.flash_enc_enabled());
}

#[test]
fn provision_enables_gate_and_round_trips() {
    let mut s = Soc::new();
    s.flashenc_provision(&KEY);
    assert!(s.flash_enc_enabled());
    // Uniform-encrypt 64 bytes of backing, then read back the plaintext
    // view: every block round-trips.
    let mut img = [0xFFu8; 64];
    img[0..16].copy_from_slice(b"0123456789abcdef");
    img[32..48].copy_from_slice(b"QUEENOFSAWSSAWNE"); // arbitrary
    s.load_flash_image(0, &img);
    s.flashenc_encrypt_region(0, 64);
    let raw = s.flash_image();
    assert_ne!(&raw[0..16], &img[0..16], "ciphertext differs");
    let view = s.flash_image_decrypted();
    assert_eq!(&view[0..64], &img[..], "decrypted view matches");
}

#[test]
fn erased_blocks_read_ff_through_decrypt() {
    let mut s = Soc::new();
    s.flashenc_provision(&KEY);
    s.load_flash_image(0, &[0xFFu8; 64]);
    // Never-encrypted erased backing reads FF (erased bypass).
    let view = s.flash_image_decrypted();
    assert!(view[0..64].iter().all(|&b| b == 0xFF));
    // Encrypting erased blocks keeps them reading FF (round-trip).
    s.flashenc_encrypt_region(0, 64);
    let view = s.flash_image_decrypted();
    assert!(view[0..64].iter().all(|&b| b == 0xFF));
}
