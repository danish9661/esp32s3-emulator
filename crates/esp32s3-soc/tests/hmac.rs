//! HMAC peripheral known-answer tests.  Expected digests computed with
//! Python `hmac.new(key, msg, sha256).hexdigest()` (see hmac.rs).
#![allow(clippy::identity_op)]

use esp32s3_soc::efuse::Efuse;
use esp32s3_soc::hmac::Hmac;

const SET_PARA_PURPOSE: u32 = 0x44;
const SET_PARA_KEY: u32 = 0x48;
const SET_PARA_FINISH: u32 = 0x4c;
const SET_MESSAGE_ONE: u32 = 0x50;
const SET_START: u32 = 0x40;
const WDATA: u32 = 0x80;
const RDATA: u32 = 0xc0;

fn set_key(efuse: &mut Efuse, id: usize, word: u32) {
    // All 8 words of key block `id` set to `word` (constant-byte key).
    for i in 0..8usize {
        efuse.write32(0x9c + id as u32 * 0x20 + i as u32 * 4, word);
    }
}

fn write_block(h: &mut Hmac, bytes: &[u8]) {
    for chunk in bytes.chunks(4) {
        let mut w = 0u32;
        for (j, b) in chunk.iter().enumerate() {
            w |= (*b as u32) << (j * 8);
        }
        h.write32(WDATA, w);
    }
}

fn feed(h: &mut Hmac, key: &[u8; 32], msg: &[u8]) {
    // Driver assembles the inner SHA input: (key ^ ipad) || padded_message.
    let mut inner = Vec::new();
    for i in 0..64usize {
        let kb = if i < key.len() { key[i] } else { 0 };
        inner.push(kb ^ 0x36);
    }
    let mut m = msg.to_vec();
    m.push(0x80);
    while m.len() % 64 != 56 {
        m.push(0);
    }
    // SHA length is the total inner-message length: (key ^ ipad) is 64 bytes.
    let bit_len = ((64 + msg.len()) as u64).wrapping_mul(8);
    m.extend_from_slice(&bit_len.to_be_bytes());
    inner.extend_from_slice(&m);
    write_block(h, &inner);
    // Single (already-padded) block.
    h.write32(SET_MESSAGE_ONE, 1);
}

fn digest(h: &Hmac) -> [u8; 32] {
    let mut out = [0u8; 32];
    for i in 0..8usize {
        let w = h.read32(RDATA + i as u32 * 4);
        let b = i * 4;
        out[b..b + 4].copy_from_slice(&w.to_be_bytes());
    }
    out
}

fn run(efuse: &mut Efuse, key_id: usize, key_word: u32, msg: &[u8]) -> [u8; 32] {
    set_key(efuse, key_id, key_word);
    let key = [key_word as u8; 32];
    let mut h = Hmac::new();
    h.write32(SET_PARA_PURPOSE, 8); // HMAC_KEY_PURPOSE_UP
    h.write32(SET_PARA_KEY, key_id as u32);
    h.write32(SET_PARA_FINISH, 1);
    h.fetch_key(efuse);
    feed(&mut h, &key, msg);
    h.write32(SET_START, 1);
    digest(&h)
}

fn hex_eq(d: &[u8; 32], hex: &str) -> bool {
    let mut want = [0u8; 32];
    let mut ok = hex.len() == 64;
    for i in 0..32usize {
        if !ok {
            break;
        }
        let hi = (hex.as_bytes()[i * 2] as char).to_digit(16);
        let lo = (hex.as_bytes()[i * 2 + 1] as char).to_digit(16);
        match (hi, lo) {
            (Some(h), Some(l)) => want[i] = (h * 16 + l) as u8,
            _ => ok = false,
        }
    }
    ok && &want == d
}

#[test]
fn hmac_zero_key_hello() {
    let mut efuse = Efuse::new();
    let d = run(&mut efuse, 0, 0, b"hello");
    assert!(
        hex_eq(
            &d,
            "4352b26e33fe0d769a8922a6ba29004109f01688e26acc9e6cb347e5a5afc4da"
        ),
        "got {:?}",
        d
    );
}

#[test]
fn hmac_0b_key_hi_there() {
    let mut efuse = Efuse::new();
    let d = run(&mut efuse, 0, 0x0b0b0b0b, b"Hi There");
    assert!(
        hex_eq(
            &d,
            "198a607eb44bfbc69903a0f1cf2bbdc5ba0aa3f3d9ae3c1c7a3b1696a0b68cf7"
        ),
        "got {:?}",
        d
    );
}

#[test]
fn hmac_aa_key_phrase() {
    let mut efuse = Efuse::new();
    let d = run(&mut efuse, 2, 0xaaaaaaaa, b"what do ya want for nothing?");
    assert!(
        hex_eq(
            &d,
            "40f7684c3cbd90ba46f70247ca1d7cc692d673f434b66926a93c7f224ec74a5e"
        ),
        "got {:?}",
        d
    );
}

#[test]
fn hmac_multiblock_message() {
    // > 64 bytes so two 512-bit blocks are written (exercises SET_MESSAGE_ING).
    let mut efuse = Efuse::new();
    let msg =
        b"The quick brown fox jumps over the lazy dog and then keeps on running through the field.";
    let d = run(&mut efuse, 0, 0, msg);
    // Reference computed with python3 hmac.new(bytes(32), msg, sha256).
    assert!(
        hex_eq(
            &d,
            "3073df7ccc76360774ad3a77df0cc6e0c8b89d8e53d0822bb4c83f7ad436fb61"
        ),
        "got {:?}",
        d
    );
}
