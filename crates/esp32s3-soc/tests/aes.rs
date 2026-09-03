//! AES unit tests: the AES-128/192/256 ECB cipher is exercised through the
//! register-level model (key + TEXT_IN writes, `AES_MODE` select, `AES_TRIGGER`
//! transform, `AES_TEXT_OUT` readback), asserting the FIPS-197 test vectors.

use esp32s3_soc::aes::Aes;

const AES_MODE: u32 = 0x40;
const AES_TRIGGER: u32 = 0x48;
const AES_TEXT_IN_BASE: u32 = 0x20;
const AES_TEXT_OUT_BASE: u32 = 0x30;

/// Run one AES-ECB block: `key`/`pt` are 16/24/32 bytes, `decrypt` selects
/// direction. Returns the 16 ciphertext bytes as four LE words.
fn run_ecb(key: &[u8], pt: &[u8; 16], decrypt: bool) -> [u32; 4] {
    assert!(key.len() == 16 || key.len() == 24 || key.len() == 32);
    let mut a = Aes::new();
    // Key: 4/6/8 words, stored LSB-first per 32-bit word.
    for (i, chunk) in key.chunks(4).enumerate() {
        let mut w = [0u8; 4];
        w[..chunk.len()].copy_from_slice(chunk);
        let val =
            (w[0] as u32) | ((w[1] as u32) << 8) | ((w[2] as u32) << 16) | ((w[3] as u32) << 24);
        a.write32((i as u32) * 4, val);
    }
    // Mode: 0/1/2 = AES-128/192/256 encrypt, +4 = decrypt.
    let nk = (key.len() / 8) as u32 - 2; // 16->0, 24->1, 32->2
    let mode = if decrypt { nk + 4 } else { nk };
    a.write32(AES_MODE, mode);
    // Plaintext: 4 words LSB-first.
    for i in 0..4 {
        let mut w = [0u8; 4];
        w.copy_from_slice(&pt[i * 4..i * 4 + 4]);
        let val =
            (w[0] as u32) | ((w[1] as u32) << 8) | ((w[2] as u32) << 16) | ((w[3] as u32) << 24);
        a.write32(AES_TEXT_IN_BASE + (i as u32) * 4, val);
    }
    a.write32(AES_TRIGGER, 1);
    let mut out = [0u32; 4];
    for i in 0..4 {
        out[i] = a.read32(AES_TEXT_OUT_BASE + (i as u32) * 4);
    }
    out
}

/// Reverse the 4 LE words into the canonical byte-string form for comparison.
fn words_to_hex(words: [u32; 4]) -> String {
    let mut s = String::new();
    for w in words.iter() {
        for b in 0..4 {
            s.push_str(&format!("{:02x}", (w >> (b * 8)) & 0xFF));
        }
    }
    s
}

#[test]
fn aes128_ecb_encrypt_fips_vector() {
    // FIPS-197 ECB-AES128 example: key=000102..0f, pt=001122..eeff.
    let key = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
    let pt = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
        0xff,
    ];
    let ct = run_ecb(&key, &pt, false);
    assert_eq!(
        words_to_hex(ct),
        "69c4e0d86a7b0430d8cdb78070b4c55a",
        "AES-128 ECB encrypt must match FIPS-197"
    );
}

#[test]
fn aes128_ecb_decrypt_roundtrip() {
    let key = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
    let pt = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
        0xff,
    ];
    let ct = run_ecb(&key, &pt, false);
    // Decrypt the ciphertext: must recover the plaintext.
    let mut ct_bytes = [0u8; 16];
    for i in 0..4 {
        for b in 0..4 {
            ct_bytes[i * 4 + b] = ((ct[i] >> (b * 8)) & 0xFF) as u8;
        }
    }
    let pt2 = run_ecb(&key, &ct_bytes, true);
    assert_eq!(words_to_hex(pt2), "00112233445566778899aabbccddeeff");
}

#[test]
fn aes256_ecb_encrypt_fips_vector() {
    // FIPS-197 ECB-AES256 example.
    let key = [
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
        0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
        0x1e, 0x1f,
    ];
    let pt = [
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
        0xff,
    ];
    let ct = run_ecb(&key, &pt, false);
    assert_eq!(
        words_to_hex(ct),
        "8ea2b7ca516745bfeafc49904b496089",
        "AES-256 ECB encrypt must match FIPS-197"
    );
}

#[test]
fn aes_state_reads_done_after_transform() {
    let key = [0u8; 16];
    let mut a = Aes::new();
    for i in 0..4 {
        a.write32((i as u32) * 4, 0);
    }
    a.write32(AES_MODE, 0);
    a.write32(AES_TEXT_IN_BASE, 0);
    a.write32(AES_TRIGGER, 1);
    // AES_STATE (0x4c) reads 2 (DONE) after a synchronous transform.
    assert_eq!(a.read32(0x4c) & 0x3, 2);
}

const AES_BLOCK_MODE: u32 = 0x94;
const AES_IV_BASE: u32 = 0x50;

fn hex_bytes(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

fn to_hex(bs: &[u8]) -> String {
    bs.iter().map(|b| format!("{b:02x}")).collect()
}

/// Drive a multi-block message through the register interface the way the
/// DMA engine does per block: program key/mode/block-mode/IV once, then for
/// each 16-byte chunk write TEXT_IN, TRIGGER, and read TEXT_OUT back.
/// Chaining state must persist across the separate transform() calls.
fn run_chained(key: &[u8], mode_reg: u32, block_mode: u32, iv: &[u8; 16], pt: &[u8]) -> Vec<u8> {
    assert_eq!(pt.len() % 16, 0);
    let mut a = Aes::new();
    for (i, chunk) in key.chunks(4).enumerate() {
        let mut w = [0u8; 4];
        w[..chunk.len()].copy_from_slice(chunk);
        a.write32(
            (i as u32) * 4,
            (w[0] as u32) | ((w[1] as u32) << 8) | ((w[2] as u32) << 16) | ((w[3] as u32) << 24),
        );
    }
    a.write32(AES_MODE, mode_reg);
    a.write32(AES_BLOCK_MODE, block_mode);
    for i in 0..4 {
        let w = (iv[i * 4] as u32)
            | ((iv[i * 4 + 1] as u32) << 8)
            | ((iv[i * 4 + 2] as u32) << 16)
            | ((iv[i * 4 + 3] as u32) << 24);
        a.write32(AES_IV_BASE + (i as u32) * 4, w);
    }
    let mut out = Vec::new();
    for blk in pt.chunks(16) {
        for i in 0..4 {
            let w = (blk[i * 4] as u32)
                | ((blk[i * 4 + 1] as u32) << 8)
                | ((blk[i * 4 + 2] as u32) << 16)
                | ((blk[i * 4 + 3] as u32) << 24);
            a.write32(AES_TEXT_IN_BASE + (i as u32) * 4, w);
        }
        a.write32(AES_TRIGGER, 1);
        for i in 0..4 {
            let w = a.read32(AES_TEXT_OUT_BASE + (i as u32) * 4);
            out.extend_from_slice(&w.to_le_bytes());
        }
    }
    out
}

const CHAIN_KEY: &str = "2b7e151628aed2a6abf7158809cf4f3c";
const CHAIN_IV: &str = "000102030405060708090a0b0c0d0e0f";
const CHAIN_PT: &str = "6bc1bee22e409f96e93d7e117393172a6e7e9a53c57e5c7ae5de90f1a1ad576e3d529ea9dda0d0059c19774e6a76008845de27fa560d14f043bd1788a07468f8";

#[test]
fn aes128_cbc_multiblock_kat() {
    let key = hex_bytes(CHAIN_KEY);
    let iv: [u8; 16] = hex_bytes(CHAIN_IV).try_into().unwrap();
    let pt = hex_bytes(CHAIN_PT);
    // Reference generated with PyCryptodome (FIPS block-1 anchor verified).
    let ct = run_chained(&key, 0, 1, &iv, &pt);
    assert_eq!(
        to_hex(&ct),
        "7649abac8119b246cee98e9b12e9197d57c71400a906fdcc09c2e6c7361f35af796a0e99e00c09a298b33288703d20680f2c8ee0906c92619019936e05261ca4",
        "AES-128-CBC encrypt KAT"
    );
    // Decrypt round-trips through the same chaining path.
    let rt = run_chained(&key, 4, 1, &iv, &ct);
    assert_eq!(rt, pt, "AES-128-CBC decrypt roundtrip");
}

#[test]
fn aes128_ctr_multiblock_kat() {
    let key = hex_bytes(CHAIN_KEY);
    // NIST SP 800-38A F.5 counter (BE counter in the low word region).
    let iv: [u8; 16] = hex_bytes("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff")
        .try_into()
        .unwrap();
    let pt = hex_bytes(CHAIN_PT);
    let ct = run_chained(&key, 0, 3, &iv, &pt);
    assert_eq!(
        to_hex(&ct),
        "874d6191b620e3261bef6864990db6ce5855e66fa20d0d19fd7ee7265dfd24c0577e5dd1a529e74a22adbf557dcc6cccad421e65a6fc8c3697b72653b518c306",
        "AES-128-CTR encrypt KAT (pins the INC32 counter order)"
    );
    let rt = run_chained(&key, 0, 3, &iv, &ct);
    assert_eq!(rt, pt, "AES-128-CTR decrypt roundtrip");
}

#[test]
fn aes128_cfb_ofb_multiblock_kat() {
    let key = hex_bytes(CHAIN_KEY);
    let iv: [u8; 16] = hex_bytes(CHAIN_IV).try_into().unwrap();
    let pt = hex_bytes(CHAIN_PT);
    let ct128 = run_chained(&key, 0, 5, &iv, &pt);
    assert_eq!(
        to_hex(&ct128),
        "3b3fd92eb72dad20333449f8e83cfb4a08f555337bce59d9b68a32f07b1e3cb4cd6d1c902bf23f4b3af5452a46877dad46cf817b9c27bab1ee0652280f8f57f2",
        "AES-128-CFB128 KAT"
    );
    assert_eq!(
        run_chained(&key, 4, 5, &iv, &ct128),
        pt,
        "AES-128-CFB128 decrypt roundtrip"
    );
    let ct8 = run_chained(&key, 0, 4, &iv, &pt);
    assert_eq!(
        to_hex(&ct8),
        "3b79424c9c0dd436bace9e0ed4586a4ff2c049aa25a347c335bb6d0f3369857c31496c905cc283cebdb3b2eb5bdef793faa8b398ee50c8c16a6b7d912d490962",
        "AES-128-CFB8 KAT"
    );
    assert_eq!(
        run_chained(&key, 4, 4, &iv, &ct8),
        pt,
        "AES-128-CFB8 decrypt roundtrip"
    );
    let cto = run_chained(&key, 0, 2, &iv, &pt);
    assert_eq!(
        to_hex(&cto),
        "3b3fd92eb72dad20333449f8e83cfb4ab7da4089cdec7fe58e55ad87214c011a9ada87f1e2a3d8e23aa641ff521cbfab830d66977f1b489f8833462a87cef1b6",
        "AES-128-OFB KAT"
    );
    assert_eq!(
        run_chained(&key, 0, 2, &iv, &cto),
        pt,
        "AES-128-OFB decrypt roundtrip"
    );
}

#[test]
fn aes128_cbc_bulk_dma_single_transform() {
    // One transform() call over a 4-block staged buffer (the GDMA path),
    // as opposed to per-block triggers: same chained result.
    use esp32s3_soc::aes::Aes as AesDirect;
    let key = hex_bytes(CHAIN_KEY);
    let iv: [u8; 16] = hex_bytes(CHAIN_IV).try_into().unwrap();
    let pt = hex_bytes(CHAIN_PT);
    let mut a = AesDirect::new();
    for (i, chunk) in key.chunks(4).enumerate() {
        let mut w = [0u8; 4];
        w.copy_from_slice(chunk);
        a.write32(
            (i as u32) * 4,
            (w[0] as u32) | ((w[1] as u32) << 8) | ((w[2] as u32) << 16) | ((w[3] as u32) << 24),
        );
    }
    a.write32(AES_MODE, 0);
    a.write32(AES_BLOCK_MODE, 1);
    for i in 0..4 {
        let w = (iv[i * 4] as u32)
            | ((iv[i * 4 + 1] as u32) << 8)
            | ((iv[i * 4 + 2] as u32) << 16)
            | ((iv[i * 4 + 3] as u32) << 24);
        a.write32(AES_IV_BASE + (i as u32) * 4, w);
    }
    for &b in &pt {
        a.feed_text_in_byte(b);
    }
    a.transform();
    // Stream the output through the FIFO-port reader (plain MMIO reads
    // past 0x3C would hit MODE/STATE/IV registers, not ciphertext).
    let mut out = Vec::new();
    for k in (0..64).step_by(4) {
        out.extend_from_slice(&a.out_word_at(k as u32).to_le_bytes());
    }
    assert_eq!(
        to_hex(&out),
        "7649abac8119b246cee98e9b12e9197d57c71400a906fdcc09c2e6c7361f35af796a0e99e00c09a298b33288703d20680f2c8ee0906c92619019936e05261ca4",
        "bulk-fed CBC matches per-block triggering"
    );
}
