//! ESP-IDF partition table (host-side parse; the boot path will use it once
//! we boot real IDF images).
//!
//! Format (esp_partition.h, esp_partition_info_t): a 32-byte entry array at
//! flash offset 0x8000. Entry: magic u16 (0xAA50), type u8, subtype u8,
//! offset u32, size u32, label [u8; 16], flags u32. The table ends at the
//! first entry with magic 0xEBEB, or an MD5 checksum entry (type 0xFF) whose
//! label field holds the 16-byte MD5.

use alloc::vec::Vec;

/// Partition table flash offset (partition table occupies 0x8000..0x9000).
pub const PARTITION_TABLE_OFFSET: u32 = 0x8000;
const PARTITION_MAGIC: u16 = 0xAA50;
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
/// Data partition subtype for the OTA selection record.
pub const DATA_SUBTYPE_OTA: u8 = 0x39;

fn label_matches(lbl: &[u8; 16], s: &[u8]) -> bool {
    lbl.len() >= s.len() && lbl[..s.len()] == *s && lbl[s.len()..].iter().all(|&b| b == 0)
}

pub fn find_partition(parts: &[Partition], ty: u8, subtype: u8) -> Option<&Partition> {
    parts.iter().find(|p| p.ty == ty && p.subtype == subtype)
}

pub fn find_partition_by_label<'a>(parts: &'a [Partition], label: &[u8]) -> Option<&'a Partition> {
    parts.iter().find(|p| label_matches(&p.label, label))
}

/// OTA selection record (`ota_data_t`, esp_ota_ops): two 32-byte entries,
/// each a `u32 ota_seq` (bit 31 = valid, low 16 bits = monotonic sequence)
/// followed by a 24-byte label.  The 2nd-stage bootloader boots the valid
/// entry with the highest sequence (ties resolve to slot 0).
const OTA_SEQ_VALID: u32 = 0x8000_0000;

/// Inspect the `otadata` partition and return the flash offset of the app
/// partition the bootloader would select.  Returns `None` when there is no
/// OTA data partition or no valid slot — the caller should then fall back to
/// the factory app slot.
pub fn select_ota_boot_offset(flash: &[u8]) -> Option<u32> {
    let parts = parse_partition_table(flash)?;
    let ota = find_partition_by_label(&parts, b"otadata")
        .or_else(|| find_partition(&parts, PTYPE_DATA, DATA_SUBTYPE_OTA))?;
    let base = ota.offset as usize;
    if base + 64 > flash.len() {
        return None;
    }
    let seq0 = u32::from_le_bytes([
        flash[base],
        flash[base + 1],
        flash[base + 2],
        flash[base + 3],
    ]);
    let seq1 = u32::from_le_bytes([
        flash[base + 32],
        flash[base + 33],
        flash[base + 34],
        flash[base + 35],
    ]);
    let v0 = (seq0 & OTA_SEQ_VALID) != 0;
    let v1 = (seq1 & OTA_SEQ_VALID) != 0;
    if !v0 && !v1 {
        return None;
    }
    let slot = if v0 && v1 {
        if (seq1 & 0xFFFF) > (seq0 & 0xFFFF) {
            1
        } else {
            0
        }
    } else if v0 {
        0
    } else {
        1
    };
    let app = parts
        .iter()
        .find(|p| p.ty == PTYPE_APP && p.subtype == APP_SUBTYPE_OTA_MIN + slot as u8)?;
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
        md5[0] = 0x50;
        md5[1] = 0xAA;
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
        flash[0x8000] = 0x50;
        flash[0x8001] = 0xAA;
        for (i, (ty, sub, off, sz, lbl)) in parts.iter().enumerate() {
            let e = 0x8000 + i * 32;
            flash[e] = 0x50;
            flash[e + 1] = 0xAA;
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

    #[test]
    fn ota_selects_highest_valid_slot() {
        let mut flash = table_with(&[
            (0x00, 0x00, 0x10000, 0x200000, &FACTORY),
            (0x00, 0x10, 0x10000, 0x200000, &OTA0),
            (0x00, 0x11, 0x200000, 0x100000, &OTA1),
            (0x01, 0x39, 0xe000, 0x2000, &OTADATA),
        ]);
        // otadata at 0xe000: slot0 seq=0x80000005 (valid), slot1 seq=0x80000003.
        flash[0xe000..0xe004].copy_from_slice(&0x8000_0005u32.to_le_bytes());
        flash[0xe020..0xe024].copy_from_slice(&0x8000_0003u32.to_le_bytes());
        assert_eq!(select_ota_boot_offset(&flash), Some(0x10000));
    }

    #[test]
    fn ota_selects_slot1_when_higher() {
        let mut flash = table_with(&[
            (0x00, 0x00, 0x10000, 0x200000, &FACTORY),
            (0x00, 0x10, 0x10000, 0x200000, &OTA0),
            (0x00, 0x11, 0x200000, 0x100000, &OTA1),
            (0x01, 0x39, 0xe000, 0x2000, &OTADATA),
        ]);
        flash[0xe000..0xe004].copy_from_slice(&0u32.to_le_bytes()); // slot0 invalid
        flash[0xe020..0xe024].copy_from_slice(&0x8000_0001u32.to_le_bytes()); // slot1 valid
        assert_eq!(select_ota_boot_offset(&flash), Some(0x200000));
    }

    #[test]
    fn ota_none_when_no_valid_slot() {
        let mut flash = table_with(&[
            (0x00, 0x00, 0x10000, 0x200000, &FACTORY),
            (0x00, 0x10, 0x10000, 0x200000, &OTA0),
            (0x00, 0x11, 0x200000, 0x100000, &OTA1),
            (0x01, 0x39, 0xe000, 0x2000, &OTADATA),
        ]);
        flash[0xe000..0xe004].copy_from_slice(&0u32.to_le_bytes());
        flash[0xe020..0xe024].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(select_ota_boot_offset(&flash), None);
    }

    #[test]
    fn ota_none_when_no_otadata_partition() {
        let flash = table_with(&[(0x00, 0x00, 0x10000, 0x200000, &FACTORY)]);
        assert_eq!(select_ota_boot_offset(&flash), None);
    }
}
