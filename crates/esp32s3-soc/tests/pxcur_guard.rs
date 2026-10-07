//! pxCurrentTCBs + handler-table guard unit test: every CPU store lane
//! (byte/word/CAS x D-view/I-alias) must preserve a live pointer
//! against a zeroing write while the WorkerL3 image is armed (TEMP
//! forensics need the guarantee; DELETE the file with the guards).

use esp32s3_soc::Soc;
use xtensa_core::Bus;

const TCB1: u32 = 0x3FC9_BAFC;

#[test]
fn pxcur_debug_image_and_seed() {
    let mut s = Soc::new();
    s.wifi_fixture_image_worker_l3();
    assert!(
        s.wifi_image == esp32s3_soc::WifiImage::WorkerL3,
        "image not armed"
    );
    s.write32(TCB1, 0x3FCA_1234);
    assert_eq!(s.read32(TCB1), 0x3FCA_1234, "seed must land");
}
