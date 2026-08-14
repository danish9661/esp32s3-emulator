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
}
