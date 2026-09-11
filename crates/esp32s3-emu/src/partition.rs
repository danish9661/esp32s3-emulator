//! ESP-IDF partition table (host-side parse; the boot path will use it once
//! we boot real IDF images).
//!
//! Format (esp_partition.h, esp_partition_info_t): a 32-byte entry array at
//! flash offset 0x8000. Entry: magic u16 (0x50AA), type u8, subtype u8,
//! offset u32, size u32, label [u8; 16], flags u32. The table ends at the
//! first entry with magic 0xEBEB, or an MD5 checksum entry (type 0xFF) whose
//! label field holds the 16-byte MD5.
//
// NOTE: the entry magic is 0x50AA (bytes AA 50 on flash) — verified against
// real arduino-cli images. An earlier 0xAA50 read only worked because the
// synthetic tests wrote the same swapped bytes; real tables never parsed,
// so OTA selection always fell back to the factory slot.

use alloc::vec::Vec;

/// Partition table flash offset (partition table occupies 0x8000..0x9000).
pub const PARTITION_TABLE_OFFSET: u32 = 0x8000;
const PARTITION_MAGIC: u16 = 0x50AA;
const PARTITION_END_MAGIC: u16 = 0xEBEB;
const MD5_ENTRY_TYPE: u8 = 0xFF;
const MAX_ENTRIES: usize = 64; // 32 bytes * 64 = 2 KB table limit

/// One parsed partition entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Partition {
    pub ty: u8,
    pub subtype: u8,
    pub offset: u32,
    pub size: u32,
    pub label: [u8; 16],
}

/// Parse the partition table at PARTITION_TABLE_OFFSET within `flash`.
pub fn parse_partition_table(flash: &[u8]) -> Option<Vec<Partition>> {
    let base = PARTITION_TABLE_OFFSET as usize;
    if flash.len() < base + 32 {
        return None;
    }
    if u16::from_le_bytes([flash[base], flash[base + 1]]) != PARTITION_MAGIC {
        return None;
    }
    let mut out = Vec::new();
    for i in 0..MAX_ENTRIES {
        let e = &flash[base + i * 32..base + (i + 1) * 32];
        let magic = u16::from_le_bytes([e[0], e[1]]);
        if magic == PARTITION_END_MAGIC || e[2] == MD5_ENTRY_TYPE {
            break;
        }
        let mut label = [0u8; 16];
        label.copy_from_slice(&e[12..28]);
        out.push(Partition {
            ty: e[2],
            subtype: e[3],
            offset: u32::from_le_bytes([e[4], e[5], e[6], e[7]]),
            size: u32::from_le_bytes([e[8], e[9], e[10], e[11]]),
            label,
        });
    }
    Some(out)
}

/// Partition types (esp_partition.h).
pub const PTYPE_APP: u8 = 0x00;
pub const PTYPE_DATA: u8 = 0x01;
/// App partition subtypes.
pub const APP_SUBTYPE_FACTORY: u8 = 0x00;
pub const APP_SUBTYPE_OTA_MIN: u8 = 0x10; // ota_0 = 0x10, ota_1 = 0x11, ...
/// Data partition subtype for the OTA selection record (esp_partition.h
/// ESP_PARTITION_SUBTYPE_DATA_OTA). Only a fallback: real tables are found
/// by the "otadata" label (whose subtype is also 0x00).
pub const DATA_SUBTYPE_OTA: u8 = 0x00;

fn label_matches(lbl: &[u8; 16], s: &[u8]) -> bool {
    lbl.len() >= s.len() && lbl[..s.len()] == *s && lbl[s.len()..].iter().all(|&b| b == 0)
}

pub fn find_partition(parts: &[Partition], ty: u8, subtype: u8) -> Option<&Partition> {
    parts.iter().find(|p| p.ty == ty && p.subtype == subtype)
}

pub fn find_partition_by_label<'a>(parts: &'a [Partition], label: &[u8]) -> Option<&'a Partition> {
    parts.iter().find(|p| label_matches(&p.label, label))
}

/// OTA selection entry (`ota_select_entry_t`, esp_ota_ops, 32 bytes):
/// ota_seq u32 @ +0, seq_label [u8; 20] @ +4, ota_state u32 @ +24,
/// crc u32 @ +28. Ground truth, all verified against real firmware.
/// CRC rule: `bootloader_common_ota_select_crc` equals
/// `esp_rom_crc32_le(-1, seq, 4)` with xor-out. Live vectors: seq 1 gives
/// 0x4743989A and seq 2 gives 0x55F63774 (pinned by
/// `ota_seq_crc_known_answers`). Validity rule:
/// `bootloader_common_ota_select_invalid` rejects seq == 0xFFFFFFFF and
/// ota_state 3 (INVALID) / 4 (ABORTED). Layout rule: the otadata partition
/// (0x2000 bytes) spans TWO sectors (base, base + 0x1000); the driver
/// erases + rewrites the second one on update (live trace showed
/// `SE @ 0xF000` with the entry at 0xF000). Each sector holds two entries
/// (at +0 and +32). The bootloader boots the CRC-valid entry with the
/// highest ota_seq; the slot is (seq - 1) % ota_count (verified: pristine
/// seq 1 boots ota_0, post-update seq 2 boots ota_1).
const OTA_ENTRY_SIZE: usize = 32;
const OTA_SECTOR_SIZE: usize = 0x1000;
const OTA_SEQ_ERASED: u32 = 0xFFFF_FFFF;

/// CRC32-ISO-HDLC over the 4-byte ota_seq, matching `esp_rom_crc32_le(-1,
/// ...)`: the ROM pre-xors its init arg, so the raw register starts at 0
/// with the standard xor-out (verified: seq 1 -> 0x4743989A, seq 2 ->
/// 0x55F63774).
pub(crate) fn ota_seq_crc(seq: u32) -> u32 {
    let mut crc = 0u32;
    for b in seq.to_le_bytes() {
        crc ^= b as u32;
        for _ in 0..8 {
            let m = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    crc ^ 0xFFFF_FFFF
}

/// Inspect the `otadata` partition and return the flash offset of the app
/// partition the bootloader would select.  Returns `None` when there is no
/// OTA data partition or no valid slot — the caller should then fall back to
/// the factory app slot.
pub fn select_ota_boot_offset(flash: &[u8]) -> Option<u32> {
    let parts = parse_partition_table(flash)?;
    let ota = find_partition_by_label(&parts, b"otadata")
        .or_else(|| find_partition(&parts, PTYPE_DATA, DATA_SUBTYPE_OTA))?;
    let ota_count = parts
        .iter()
        .filter(|p| p.ty == PTYPE_APP && p.subtype >= APP_SUBTYPE_OTA_MIN)
        .count() as u32;
    if ota_count == 0 {
        return None;
    }
    let base = ota.offset as usize;
    let mut best: Option<u32> = None;
    for sector in [0usize, OTA_SECTOR_SIZE] {
        for e in [0usize, OTA_ENTRY_SIZE] {
            let o = base + sector + e;
            if o + OTA_ENTRY_SIZE > flash.len() || o + OTA_ENTRY_SIZE > base + ota.size as usize {
                continue;
            }
            let seq = u32::from_le_bytes([flash[o], flash[o + 1], flash[o + 2], flash[o + 3]]);
            if seq == OTA_SEQ_ERASED {
                continue;
            }
            let state =
                u32::from_le_bytes([flash[o + 24], flash[o + 25], flash[o + 26], flash[o + 27]]);
            if state == 3 || state == 4 {
                continue;
            }
            let crc =
                u32::from_le_bytes([flash[o + 28], flash[o + 29], flash[o + 30], flash[o + 31]]);
            if ota_seq_crc(seq) != crc {
                continue;
            }
            if best.is_none_or(|b| seq > b) {
                best = Some(seq);
            }
        }
    }
    let slot = (best?.wrapping_sub(1) % ota_count) as u8;
    let app = parts
        .iter()
        .find(|p| p.ty == PTYPE_APP && p.subtype == APP_SUBTYPE_OTA_MIN + slot)?;
    Some(app.offset)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(ty: u8, subtype: u8, offset: u32, size: u32, label: &[u8; 16]) -> Vec<u8> {
        let mut e = Vec::new();
        e.extend_from_slice(&PARTITION_MAGIC.to_le_bytes());
        e.push(ty);
        e.push(subtype);
        e.extend_from_slice(&offset.to_le_bytes());
        e.extend_from_slice(&size.to_le_bytes());
        e.extend_from_slice(label);
        e.extend_from_slice(&[0, 0, 0, 0]);
        e
    }

    #[test]
    fn parses_partition_table() {
        let mut flash = std::vec![0u8; 0x9000];
        let app = entry(0, 0x10, 0x10000, 0x200000, b"factory\0\0\0\0\0\0\0\0\0");
        let nvs = entry(1, 2, 0x9000, 0x6000, b"nvs\0\0\0\0\0\0\0\0\0\0\0\0\0");
        flash[0x8000..0x8000 + app.len()].copy_from_slice(&app);
        flash[0x8000 + 32..0x8000 + 64].copy_from_slice(&nvs);
        let term = entry(0, 0, 0, 0, &[0; 16]);
        flash[0x8000 + 64..0x8000 + 96].copy_from_slice(&term);
        flash[0x8000 + 64] = 0xEB;
        flash[0x8000 + 65] = 0xEB;

        let parts = parse_partition_table(&flash).expect("magic should match");
        assert_eq!(parts.len(), 2);
        assert_eq!(parts[0].ty, 0);
        assert_eq!(parts[0].subtype, 0x10);
        assert_eq!(parts[0].offset, 0x10000);
        assert_eq!(parts[0].size, 0x200000);
        assert_eq!(&parts[0].label, b"factory\0\0\0\0\0\0\0\0\0");
        assert_eq!(parts[1].ty, 1);
        assert_eq!(parts[1].offset, 0x9000);
    }

    #[test]
    fn md5_entry_ends_table() {
        let mut flash = std::vec![0u8; 0x9000];
        let app = entry(0, 0x10, 0x10000, 0x200000, b"factory\0\0\0\0\0\0\0\0\0");
        flash[0x8000..0x8000 + 32].copy_from_slice(&app);
        let mut md5 = entry(0xFF, 0, 0, 0, b"0123456789abcdef");
        md5[0] = 0xAA;
        md5[1] = 0x50;
        flash[0x8000 + 32..0x8000 + 64].copy_from_slice(&md5);

        let parts = parse_partition_table(&flash).expect("magic should match");
        assert_eq!(parts.len(), 1);
    }

    #[test]
    fn bad_magic_is_none() {
        let flash = std::vec![0u8; 0x9000];
        assert_eq!(parse_partition_table(&flash), None);
    }

    // Build a minimal partition table (no MD5 terminator) at 0x8000.
    fn table_with(parts: &[(u8, u8, u32, u32, &[u8; 16])]) -> std::vec::Vec<u8> {
        let mut flash = std::vec![0u8; 0x1_0000];
        flash[0x8000] = 0xAA;
        flash[0x8001] = 0x50;
        for (i, (ty, sub, off, sz, lbl)) in parts.iter().enumerate() {
            let e = 0x8000 + i * 32;
            flash[e] = 0xAA;
            flash[e + 1] = 0x50;
            flash[e + 2] = *ty;
            flash[e + 3] = *sub;
            flash[e + 4..e + 8].copy_from_slice(&off.to_le_bytes());
            flash[e + 8..e + 12].copy_from_slice(&sz.to_le_bytes());
            flash[e + 12..e + 28].copy_from_slice(*lbl);
        }
        flash[0x8000 + parts.len() * 32] = 0xEB;
        flash[0x8000 + parts.len() * 32 + 1] = 0xEB;
        flash
    }

    const FACTORY: [u8; 16] = *b"factory\0\0\0\0\0\0\0\0\0";
    const OTA0: [u8; 16] = *b"ota_0\0\0\0\0\0\0\0\0\0\0\0";
    const OTA1: [u8; 16] = *b"ota_1\0\0\0\0\0\0\0\0\0\0\0";
    const OTADATA: [u8; 16] = *b"otadata\0\0\0\0\0\0\0\0\0";

    /// Write a valid 32-byte otadata entry (seq + erased label + state +
    /// correct CRC) at flash offset `off`.
    fn ota_entry(flash: &mut [u8], off: usize, seq: u32, state: u32) {
        flash[off..off + 4].copy_from_slice(&seq.to_le_bytes());
        for b in flash[off + 4..off + 24].iter_mut() {
            *b = 0xFF;
        }
        flash[off + 24..off + 28].copy_from_slice(&state.to_le_bytes());
        flash[off + 28..off + 32].copy_from_slice(&ota_seq_crc(seq).to_le_bytes());
    }

    fn ota_table() -> Vec<u8> {
        table_with(&[
            (0x00, 0x00, 0x10000, 0x200000, &FACTORY),
            (0x00, 0x10, 0x10000, 0x200000, &OTA0),
            (0x00, 0x11, 0x200000, 0x100000, &OTA1),
            (0x01, 0x39, 0xe000, 0x2000, &OTADATA),
        ])
    }

    #[test]
    fn ota_seq_crc_known_answers() {
        // Live otadata from real firmware (verified against the ROM CRC).
        assert_eq!(ota_seq_crc(1), 0x4743_989A);
        assert_eq!(ota_seq_crc(2), 0x55F6_3774);
    }

    #[test]
    fn ota_selects_highest_valid_slot() {
        let mut flash = ota_table();
        // Sector 0: slot0 seq=5, slot1 seq=3 (both valid).
        ota_entry(&mut flash, 0xe000, 5, 0xFFFF_FFFF);
        ota_entry(&mut flash, 0xe020, 3, 0xFFFF_FFFF);
        // (5-1)%2 = 0 -> ota_0.
        assert_eq!(select_ota_boot_offset(&flash), Some(0x10000));
    }

    #[test]
    fn ota_selects_slot1_when_higher() {
        let mut flash = ota_table();
        ota_entry(&mut flash, 0xe000, 1, 0xFFFF_FFFF);
        ota_entry(&mut flash, 0xe020, 2, 0);
        // (2-1)%2 = 1 -> ota_1.
        assert_eq!(select_ota_boot_offset(&flash), Some(0x200000));
    }

    #[test]
    fn ota_selects_slot1_from_second_sector() {
        // The real post-update state: pristine seq 1 in sector 0, new seq 2
        // in sector 1 (the driver erases + rewrites 0xF000 on update).
        let mut flash = ota_table();
        ota_entry(&mut flash, 0xe000, 1, 0xFFFF_FFFF);
        ota_entry(&mut flash, 0xf000, 2, 0);
        assert_eq!(select_ota_boot_offset(&flash), Some(0x200000));
    }

    #[test]
    fn ota_rejects_bad_crc_and_aborted_state() {
        let mut flash = ota_table();
        ota_entry(&mut flash, 0xe000, 9, 0xFFFF_FFFF);
        flash[0xe000 + 28] ^= 0xFF; // corrupt the CRC
        ota_entry(&mut flash, 0xe020, 7, 3); // INVALID state
        ota_entry(&mut flash, 0xf000, 5, 4); // ABORTED state
        ota_entry(&mut flash, 0xf020, 2, 0); // only valid entry
        assert_eq!(select_ota_boot_offset(&flash), Some(0x200000));
    }

    #[test]
    fn ota_none_when_no_valid_slot() {
        let mut flash = ota_table();
        for b in flash[0xe000..0xe000 + 0x2000].iter_mut() {
            *b = 0xFF; // erased otadata: no valid entry
        }
        assert_eq!(select_ota_boot_offset(&flash), None);
    }

    #[test]
    fn parses_real_arduino_partition_table() {
        // Exact partition table bytes from a real arduino-cli merged image
        // (default.csv: nvs/otadata/app0/app1/spiffs/coredump + terminator).
        // Regression test: the entry magic is 0x50AA (bytes AA 50); an
        // earlier 0xAA50 constant silently rejected every real table.
        let hex = concat!(
            "aa50010200900000005000006e76730000000000000000000000000000000000",
            "aa50010000e00000002000006f74616461746100000000000000000000000000",
            "aa50001000000100000014006170703000000000000000000000000000000000",
            "aa50001100001500000014006170703100000000000000000000000000000000",
            "aa50018200002900000016007370696666730000000000000000000000000000",
            "aa50010300003f0000000100636f726564756d70000000000000000000000000",
            "ebebffffffffffffffffffffffffffff972dae2ff872a0142d60bad124c0666b",
        );
        let mut raw = std::vec![0u8; 224];
        for (i, b) in raw.iter_mut().enumerate() {
            *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap();
        }
        let mut flash = std::vec![0u8; 0x9000];
        flash[0x8000..0x8000 + 224].copy_from_slice(&raw);
        let parts = parse_partition_table(&flash).expect("real table parses");
        assert_eq!(parts.len(), 6);
        let ota = find_partition_by_label(&parts, b"otadata").expect("otadata found");
        assert_eq!((ota.offset, ota.size), (0xe000, 0x2000));
        // NOTE: real otadata subtype is 0x00, not 0x39 — the label carries it.
        assert_eq!(ota.subtype, 0x00);
        let app1 = find_partition(&parts, PTYPE_APP, 0x11).expect("app1 found");
        assert_eq!(app1.offset, 0x150000);
    }

    #[test]
    fn ota_none_when_no_otadata_partition() {
        let flash = table_with(&[(0x00, 0x00, 0x10000, 0x200000, &FACTORY)]);
        assert_eq!(select_ota_boot_offset(&flash), None);
    }
}
