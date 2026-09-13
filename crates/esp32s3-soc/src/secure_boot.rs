//! ESP32-S3 Secure Boot v2 signature-block verification (offline).
//!
//! Layout per `espsecure/__init__.py` (esptool 5.3.1, Apache-2.0 — host
//! tooling, not firmware): the signed image is data padded to a 4 KB
//! boundary plus one 4 KB signature sector holding up to three 1216-byte
//! blocks. Block 0 layout (little-endian, `validate_signature_block` +
//! `verify_signature_v2`):
//! ```text
//! magic[0]=0xE7, version[1] (2=RSA, 3=ECDSA), sha[2] (0=SHA256,1=SHA384),
//! digest[4..36] = SHA256(image_without_sig_sector) (or 48 B at [4..52]),
//! curve_id[36] (1=P192,2=P256,3=P384), pubkey[37..101] (raw x||y),
//! r[101..133] + s[133..165] (little-endian each), crc32(block[..1196]).
//! ```
//! `verify_image` checks the CRC, the digest and the ECDSA P-256 signature
//! through the [`crate::ecdsa`] model (register pokes: QX/QY/Z/R/S +
//! START, poll RESULT) — the same accelerator real ROM verification uses.
//! RSA blocks and SHA384/P-192/P-384 curves report `Unsupported` (the
//! model covers P-256/SHA256; espsecure generates those by default).
//!
//! Validated by `sbv2_verify_espsecure_fixture` (a real `espsecure.py`
//! ECDSA-P256 signed blob) — the offline signing pipeline the ROM stub's
//! fail-closed gate stands in for.

use crate::ecdsa::Ecdsa;

/// Why a signature block was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sbv2Verdict {
    /// Block N verified (ECDSA P-256, digest matches, RESULT=1).
    Valid,
    /// CRC mismatch or bad magic/version (block absent).
    Invalid,
    /// RSA block or non-P256/SHA256 parameters (model coverage).
    Unsupported,
}

/// Verify the Secure Boot v2 signature sector of `image` (data ++ 4 KB
/// signature sector, exactly what `espsecure.py sign-data` emits).
/// Returns the verdict for signature block 0.
pub fn verify_image(image: &[u8]) -> Sbv2Verdict {
    const SECTOR: usize = 4096;
    const BLOCK: usize = 1216;
    if image.len() < 2 * SECTOR || !image.len().is_multiple_of(SECTOR) {
        return Sbv2Verdict::Invalid;
    }
    let (data, sigsec) = image.split_at(image.len() - SECTOR);
    let blk = &sigsec[..BLOCK];
    if blk[0] != 0xE7 || (blk[1] != 2 && blk[1] != 3) {
        return Sbv2Verdict::Invalid;
    }
    if blk[1] != 3 {
        return Sbv2Verdict::Unsupported; // RSA
    }
    if blk[2] != 0 {
        return Sbv2Verdict::Unsupported; // SHA384
    }
    // CRC32 over block[..1196] (zlib CRC, same as Python).
    if crc32(&blk[..1196]) != u32::from_le_bytes([blk[1196], blk[1197], blk[1198], blk[1199]]) {
        return Sbv2Verdict::Invalid;
    }
    // Digest over the image without the signature sector.
    let digest = crate::hmac::sha256(data);
    if digest[..] != blk[4..36] {
        return Sbv2Verdict::Invalid;
    }
    if blk[36] != 2 {
        return Sbv2Verdict::Unsupported; // P-192/P-384
    }
    // P-256 verify through the ECDSA model. Limb conventions (proven by a
    // full 3-axis matrix + python twin, Sept 2026): the model is
    // LE-limb-native; espsecure stores Q/R/S as LE bytes (LE words are the
    // true limbs), but the DIGEST is a raw hash whose integer is big-endian
    // (like `Prehashed` in the tool) — so Z alone needs the word-reversed
    // (BE-word) packing. Feeding Z as LE words verifies nothing (RESULT=0
    // on a genuine signature); feeding it BE-word verifies (RESULT=1).
    let le = |b: &[u8]| -> [u32; 8] {
        let mut w = [0u32; 8];
        for (i, c) in b.chunks(4).enumerate().take(8) {
            w[i] = u32::from_le_bytes([c[0], c[1], c[2], c[3]]);
        }
        w
    };
    let bew = |b: &[u8]| -> [u32; 8] {
        let n = b.len() / 4;
        let mut w = [0u32; 8];
        for i in 0..n {
            let p = &b[(n - 1 - i) * 4..];
            w[i] =
                ((p[0] as u32) << 24) | ((p[1] as u32) << 16) | ((p[2] as u32) << 8) | p[3] as u32;
        }
        w
    };
    let mut e = Ecdsa::new();
    // CONF: work_mode=verify(0), ecc_curve=P-256(1<<2).
    e.write32(0x00, 1 << 2);
    for (i, w) in le(&blk[37..69]).iter().enumerate() {
        e.write32(0x80 + ((5 * 8 + i) * 4) as u32, *w); // QX
    }
    for (i, w) in le(&blk[69..101]).iter().enumerate() {
        e.write32(0x80 + ((6 * 8 + i) * 4) as u32, *w); // QY
    }
    let mut z = [0u8; 32];
    z.copy_from_slice(&digest[..32]);
    for (i, w) in bew(&z).iter().enumerate() {
        e.write32(0x80 + ((9 * 8 + i) * 4) as u32, *w); // Z (BE-word packed)
    }
    for (i, w) in le(&blk[101..133]).iter().enumerate() {
        e.write32(0x80 + ((10 * 8 + i) * 4) as u32, *w); // R
    }
    for (i, w) in le(&blk[133..165]).iter().enumerate() {
        e.write32(0x80 + ((11 * 8 + i) * 4) as u32, *w); // S
    }
    e.write32(0x04, 1); // START
    if e.read32(0x18) == 1 {
        Sbv2Verdict::Valid
    } else {
        Sbv2Verdict::Invalid
    }
}

/// zlib CRC32 (IEEE, init 0xFFFF_FFFF, xorout) — matches Python's.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    crc ^ 0xFFFF_FFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    // Real `espsecure.py` ECDSA-P256 signed blob (9 B payload padded to
    // 4 KB + 4 KB signature sector, digest + signature verified against
    // the tool's own `verify-signature`): must report Valid through the
    // ECDSA model.
    const SIGNED: &[u8] = include_bytes!("../tools/sbtest_data_signed.bin");

    #[test]
    fn sbv2_verify_espsecure_fixture() {
        assert_eq!(verify_image(SIGNED), Sbv2Verdict::Valid);
    }

    #[test]
    fn sbv2_tampered_image_rejected() {
        let mut img = SIGNED.to_vec();
        img[0] ^= 0xFF; // flip a data byte: digest mismatch
        assert_eq!(verify_image(&img), Sbv2Verdict::Invalid);
    }

    #[test]
    fn sbv2_crc_mismatch_rejected() {
        let mut img = SIGNED.to_vec();
        let n = img.len();
        img[n - 4096 + 100] ^= 0x01; // flip inside block pre-CRC area
        assert_eq!(verify_image(&img), Sbv2Verdict::Invalid);
    }
}
