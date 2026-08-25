//! RNG (WDEV_RND) register model tests.

use esp32s3_soc::rng::Rng;

const DATA: u32 = 0x6003_507C;

#[test]
fn data_register_returns_varying_values() {
    let mut d = Rng::new();
    let a = d.read32(DATA);
    let b = d.read32(DATA);
    let c = d.read32(DATA);
    assert_ne!(a, b, "consecutive RNG reads must differ");
    assert_ne!(b, c, "consecutive RNG reads must differ");
}

#[test]
fn data_register_is_deterministic_for_a_seed() {
    // A fresh Rng yields the same sequence every time (reproducible tests).
    let mut a = Rng::new();
    let mut b = Rng::new();
    assert_eq!(a.read32(DATA), b.read32(DATA));
    assert_eq!(a.read32(DATA), b.read32(DATA));
    assert_eq!(a.read32(DATA), b.read32(DATA));
}

#[test]
fn non_data_registers_store_writes() {
    let mut d = Rng::new();
    d.write32(0x6003_5010, 0xBEEF);
    assert_eq!(d.read32(0x6003_5010), 0xBEEF);
    // The data register is unaffected by unrelated writes.
    let _ = d.read32(DATA);
    assert_eq!(d.read32(0x6003_5010), 0xBEEF);
}
