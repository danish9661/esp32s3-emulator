//! Hybrid-sync mapping KAT: the bulk sram/iram0 accessors used for
//! wasmtime<->Soc state sync must agree byte-for-byte with the Bus for
//! every address the transcribed direct-memory legs can touch (DRAM,
//! SRAM0 window, DIRAM alias window). A mapping slip here corrupts guest
//! RAM silently and only manifests as a far-future ILLEGAL (proven live:
//! hybrid died at 0x40374300 after 13 clean slices).

use esp32s3_soc::memmap::{
    DIRAM_DATA_BASE, DIRAM_INST_BASE, DRAM_BASE, DRAM_SIZE, IRAM_BASE, SRAM0_SIZE,
};
use esp32s3_soc::Soc;
use xtensa_core::Bus;

fn bulk_read(soc: &Soc, addr: u32) -> u8 {
    if (DRAM_BASE..DRAM_BASE + DRAM_SIZE).contains(&addr) {
        soc.sram_bytes()[(addr - DRAM_BASE) as usize]
    } else if (IRAM_BASE..IRAM_BASE + SRAM0_SIZE).contains(&addr) {
        soc.iram0_bytes()[(addr - IRAM_BASE) as usize]
    } else {
        // DIRAM alias window.
        let o = addr.wrapping_sub(DIRAM_INST_BASE);
        soc.sram_bytes()[(DIRAM_DATA_BASE - DRAM_BASE + o) as usize]
    }
}

#[test]
fn bulk_accessors_match_bus_across_regions() {
    let mut soc = Soc::new();
    // Distinctive pattern per region through the BUS (ground truth).
    let probes: &[(u32, u8)] = &[
        (DRAM_BASE, 0x11),
        (DRAM_BASE + 0x12345, 0x22),
        (DRAM_BASE + DRAM_SIZE - 1, 0x33),
        (IRAM_BASE, 0x44),
        (IRAM_BASE + SRAM0_SIZE - 1, 0x55),
        (DIRAM_INST_BASE, 0x66),
        (DIRAM_INST_BASE + 0x1234, 0x77),
    ];
    for &(addr, val) in probes {
        soc.write8(addr, val as u32);
    }
    for &(addr, val) in probes {
        assert_eq!(soc.read8(addr), val as u32, "bus roundtrip at {addr:#x}");
        assert_eq!(bulk_read(&soc, addr), val, "bulk mismatch at {addr:#x}");
    }
    // Reverse direction: bulk write, bus read.
    for &(addr, _) in probes {
        if (DRAM_BASE..DRAM_BASE + DRAM_SIZE).contains(&addr) {
            soc.sram_bytes_mut()[(addr - DRAM_BASE) as usize] = 0xA5;
        } else if (IRAM_BASE..IRAM_BASE + SRAM0_SIZE).contains(&addr) {
            soc.iram0_bytes_mut()[(addr - IRAM_BASE) as usize] = 0xA5;
        } else {
            let o = addr.wrapping_sub(DIRAM_INST_BASE);
            soc.sram_bytes_mut()[(DIRAM_DATA_BASE - DRAM_BASE + o) as usize] = 0xA5;
        }
        assert_eq!(soc.read8(addr), 0xA5u32, "reverse mismatch at {addr:#x}");
    }
}
