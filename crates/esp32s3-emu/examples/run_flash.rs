//! Host runner for real firmware flash images (Arduino CLI / ESP-IDF merged
//! images).  Usage: `cargo run -p esp32s3-emu --example run_flash -- <merged.bin>`.
//!
//! Boots the image via `Esp32S3::boot_from_flash`, steps it, and prints UART0
//! console output plus a stall/exception report so real-firmware boot can be
//! validated without QEMU.

use std::env;
use std::fs;
use std::time::Instant;

use esp32s3_emu::Esp32S3;
use xtensa_core::cpu::SR_EPC1;
use xtensa_core::{Bus, StepResult};

/// Describe the faulting instruction behind an Unimplemented trap (these
/// are ee.* DSP/TIE extensions: recognized by the decoder but without an
/// execution model).
fn unimp_detail(m: &mut Esp32S3, core: usize) -> String {
    use xtensa_core::generated::{decode_inst, decode_inst16a, decode_inst16b, insn_len};
    let pc = m.cpu[core].pc;
    let raw = m.soc.read32(pc);
    let b0 = (raw & 0xFF) as u8;
    let opc = match insn_len(b0) {
        2 => {
            let w = raw & 0xFFFF;
            if b0 & 0xf <= 11 {
                decode_inst16a(w)
            } else {
                decode_inst16b(w)
            }
        }
        _ => decode_inst(raw),
    };
    match opc {
        Some(o) if xtensa_core::ee::is_ee_opcode(o) => {
            // TIE/DSP trap: name the hardware unit, not just the mnemonic.
            format!(
                "{} [ee:{}] (raw {raw:#010x})",
                o.name(),
                xtensa_core::ee::ee_family(o)
            )
        }
        Some(o) => format!("{} (raw {raw:#010x})", o.name()),
        None => format!("undecodable (raw {raw:#010x})"),
    }
}

fn main() {
    let path = env::args().nth(1).expect("usage: run_flash <flash image>");
    let flash = fs::read(&path).expect("read flash image");

    let mut m = Esp32S3::new();
    // Secure-boot fixture: SECURE_BOOT_EN=1 burns eFuse SECURE_BOOT_EN
    // (BLK0 word 5 = REPEAT_DATA4 bit 20) through the real PGM path before
    // boot, so `boot_from_flash` verifies the app region's signature sector
    // (fail-closed: unsigned/bad-signature boots park the CPUs with no
    // output). Pair with a genuinely `espsecure.py sign-data`-signed image
    // (see tools/sketches/esp32s3_secure_boot/) for the allow path.
    if env::var("SECURE_BOOT_EN").is_ok() {
        const EFUSE_BASE: u32 = 0x6000_7000;
        for (i, w) in [0u32, 0, 0, 0, 0, 1 << 20, 0, 0].iter().enumerate() {
            m.soc.write32(EFUSE_BASE + (i as u32) * 4, *w);
        }
        m.soc.write32(EFUSE_BASE + 0x1D4, 0x2); // PGM bit, BLK_NUM 0
        println!("[host] SECURE_BOOT_EN burned (fail-closed gate armed)");
    }
    // Flash-encryption fixture: FLASHENC_KEY=<64 hex> provisions the eFuse
    // XTS key + crypt count, then uniformly encrypts the whole image
    // (every 16-byte block at its absolute offset, like esptool) before
    // boot. The firmware boots and runs on decrypted plaintext.
    let flashenc: Option<Vec<u8>> = env::var("FLASHENC_KEY").ok().map(|hex| {
        assert_eq!(hex.len(), 64, "FLASHENC_KEY must be 64 hex chars");
        (0..32)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex key"))
            .collect()
    });
    if let Some(key) = &flashenc {
        let mut key32 = [0u8; 32];
        key32.copy_from_slice(key);
        m.soc.flashenc_provision(&key32);
        println!("[host] flash encryption provisioned");
    }
    let boot_image: Vec<u8>;
    let boot_ref: &[u8] = if flashenc.is_some() {
        assert_eq!(flash.len() % 16, 0, "image length must be 16-byte aligned");
        // Encrypt a copy through the backing (provisioned above), then boot
        // the ciphertext. load+encrypt here mirrors the factory flow.
        m.soc.load_flash_image(0, &flash);
        m.soc.flashenc_encrypt_region(0, flash.len() as u32);
        boot_image = m.soc.flash_image().to_vec();
        println!("[host] flash image encrypted ({} bytes)", boot_image.len());
        &boot_image
    } else {
        &flash
    };
    m.boot_from_flash(boot_ref);

    // RNG reseed: RNG_SEED=<u32> varies the esp_random() stream across
    // runs (deterministic within a run; default seed keeps tests stable).
    if let Ok(seed) = env::var("RNG_SEED")
        && let Ok(seed) = seed.parse::<u32>()
    {
        m.soc.rng_reseed(seed);
        println!("[host] reseeded RNG ({seed:#x})");
    }

    // ADC injection for sketches doing analogRead: ADC_INJECT_MV=<mv>
    // applies the voltage to ADC1 channel 3 (GPIO4).
    if let Ok(mv) = env::var("ADC_INJECT_MV")
        && let Ok(mv) = mv.parse::<u32>()
    {
        m.soc.adc_inject_voltage(0, 3, mv);
        println!("[host] injected {mv} mV on ADC1_CH3 (GPIO4)");
    }

    // TSENS injection for temperatureRead: TEMP_RAW=<0..255> sets the
    // SENS_TSENS_OUT DAC code the firmware reads.
    if let Ok(raw) = env::var("TEMP_RAW")
        && let Ok(raw) = raw.parse::<u8>()
    {
        m.soc.tsens_inject(raw);
        println!("[host] injected TSENS raw={raw}");
    }
    // Celsius shorthand: TEMP_C=<c> inverts the driver-observed middle
    // line T = 0.4375*raw - 21 (blank-eFuse cal; ends are nonlinear).
    if let Ok(c) = env::var("TEMP_C")
        && let Ok(c) = c.parse::<f32>()
    {
        let raw = (((c + 21.0) * 16.0 / 7.0).round() as i32).clamp(0, 255) as u8;
        m.soc.tsens_inject(raw);
        println!("[host] injected TSENS {c}C as raw={raw}");
    }

    // Touch injection for sketches doing touchRead: TOUCH_INJECT=<pad>:<val>
    // sets the counter touch pad 1..=14 reports (falls when touched).
    if let Ok(spec) = env::var("TOUCH_INJECT")
        && let Some((pad, val)) = spec.split_once(':')
        && let (Ok(pad), Ok(val)) = (pad.parse::<usize>(), val.parse::<u32>())
    {
        m.soc.touch_inject(pad, val);
        println!("[host] injected touch pad {pad} = {val}");
    }

    // Fake temperature devices (tempdev-sketch support): I2C_TEMP_RX=<hex>
    // pre-injects the I2C thermometer read bytes (e.g. 1900 = TMP102
    // 25.00 C), SPI_TEMP_RX=<hex> the next SPI thermometer frame bytes
    // (e.g. 0320 = MAX6675 25.00 C). Same hooks the virtual-demo browser
    // harness uses (i2c_inject_rx / spi_inject_miso).
    fn parse_hex_bytes(s: &str) -> Vec<u8> {
        let h: Vec<char> = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();
        (0..h.len() / 2)
            .map(|i| {
                let pair: String = [h[2 * i], h[2 * i + 1]].iter().collect();
                u8::from_str_radix(&pair, 16).unwrap_or(0)
            })
            .collect()
    }
    if let Ok(hex) = env::var("I2C_TEMP_RX") {
        let b = parse_hex_bytes(&hex);
        m.soc.i2c_inject_rx(0, &b);
        println!("[host] injected I2C temp bytes {b:02x?}");
    }
    if let Ok(hex) = env::var("SPI_TEMP_RX") {
        let b = parse_hex_bytes(&hex);
        m.soc.spi_inject_miso(0, &b);
        println!("[host] injected SPI temp bytes {b:02x?}");
    }

    // Brown-out injection for BOD sketches: BOD_INJECT=1 holds the
    // low-voltage condition so an enabled detector trips (interrupt
    // after int_wait, chip reset after rst_wait with rst_ena).
    if env::var("BOD_INJECT").is_ok() {
        m.soc.bod_inject(true);
        println!("[host] injected brownout condition");
    }

    // PSRAM density override for 16 MB validation: PSRAM_MR2=<n> sets the
    // MR2 reset default (3 = 64 Mb / 8 MB, 5 = 128 Mb / 16 MB) before the
    // OPI sizing path runs at boot.
    if let Ok(mr2) = env::var("PSRAM_MR2")
        && let Ok(mr2) = mr2.parse::<u8>()
    {
        m.soc.psram_set_mr2(mr2);
        println!("[host] PSRAM MR2 override ={mr2}");
    }

    // UART1 RX injection (echo-sketch support): UART_INJECT=<text> is
    // pushed into UART1 RX as soon as the console shows the RXREADY marker.
    let uart1_inject: Option<Vec<u8>> = env::var("UART_INJECT").ok().map(|s| s.into_bytes());
    let mut uart1_injected = false;
    // UART0 RX injection (REPL experiment): UART0_INJECT=<text> is pushed
    // into UART0 RX once the console shows UART0_MARKER (MicroPython's
    // REPL may listen on UART0 rather than USB-CDC depending on the
    // board's console configuration).
    let uart0_inject: Option<Vec<u8>> = env::var("UART0_INJECT")
        .ok()
        .map(|s| s.replace("\\n", "\n").replace("\\r", "\r").into_bytes());
    let uart0_marker: Vec<u8> = env::var("UART0_MARKER")
        .ok()
        .map(|s| s.into_bytes())
        .unwrap_or_else(|| b">>> ".to_vec());
    let mut uart0_injected = false;
    // USB-Serial-JTAG RX injection (REPL experiment): USB_INJECT=<text> is
    // pushed into the USB CDC RX FIFO once the console shows USB_MARKER
    // (default: the MicroPython post-PSRAM boot line).
    let usb_inject: Option<Vec<u8>> = env::var("USB_INJECT")
        .ok()
        .map(|s| s.replace("\\n", "\n").replace("\\r", "\r").into_bytes());
    let usb_marker: Vec<u8> = env::var("USB_MARKER")
        .ok()
        .map(|s| s.into_bytes())
        .unwrap_or_else(|| b"continuing without it".to_vec());
    let mut usb_injected = false;

    // SPI-slave host exchange (slave-sketch support): SPI_SLAVE_XCHG=1 drives
    // both halves when the app prints its markers — a master-write-to-slave
    // ([0x11, 0x22, 0x33]) on "SPI SLAVE READY", then a 2-byte
    // master-read-from-slave (expecting the preloaded [0xA5, 0xC3]) on
    // "SPI SLAVE TX-REQ".
    let spi_slave_xchg = env::var("SPI_SLAVE_XCHG").is_ok();
    let mut spi_slave_wrote = false;
    let mut spi_slave_read = false;
    // Slave-DMA halves (separate markers, same env flag).
    let mut spi_slave_dma_wrote = false;
    let mut spi_slave_dma_read = false;

    // I2C-slave host exchange (slave-sketch support): I2C_SLAVE_XCHG=1 drives
    // both halves when the app prints its markers — a master-write-to-slave
    // (addr 0x42, [0x11, 0x22]) on "I2C SLAVE READY", then a 1-byte
    // master-read-from-slave (expecting the preloaded [0xA5]) on
    // "I2C SLAVE TX-REQ".
    let i2c_slave_xchg = env::var("I2C_SLAVE_XCHG").is_ok();
    let mut i2c_slave_wrote = false;
    let mut i2c_slave_read = false;

    // Fake quad-SPI device store (quaddev-sketch support): SPI_QUADDEV=1
    // provisions a 256-byte incrementing pattern (byte i = i) on GPSPI2
    // before boot, matching the sketch's expected address windows.
    if env::var("SPI_QUADDEV").is_ok() {
        let pat: Vec<u8> = (0..=255u16).map(|i| i as u8).collect();
        m.soc.spi_quad_fake_provision(0, &pat);
        println!("[host] provisioned fake quad-SPI device on GPSPI2");
    }

    // SPI-mode SD card (sdspi-sketch support): SPI_SDSPI=1 attaches the
    // in-model card on GPSPI2 sharing the SDMMC FAT16 image, so the
    // Arduino `SD` library mounts the same volume the SDMMC path uses.
    if env::var("SPI_SDSPI").is_ok() {
        m.soc.spi_sdspi_attach_sdmmc_image(0);
        println!("[host] attached SDSPI card on GPSPI2 (SDMMC image)");
    }

    // Step budget in INSTRUCTIONS (`step_fast` executes whole blocks and
    // reports how many instructions ran): one old loop iteration stepped a
    // single instruction per core (2/cross-core pair), so the old 48M-step
    // default ~= 96M instructions.  The loop also exits early when the
    // firmware has been idle (no UART output AND no PC change) for 2M
    // consecutive steps — indicates setup() is done and the firmware is in
    // its main loop or halted.
    let max_insns: usize = env::var("STEPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(96_000_000);
    let t0 = Instant::now();
    let mut uart_buf: Vec<u8> = Vec::new();
    let mut last_pc = 0u32;
    let mut last_uart_len = 0usize;
    let mut stuck = 0u32;
    let mut idle_steps: usize = 0;
    let mut executed: u64 = 0;
    let mut i: usize = 0; // macro-step counter (diagnostics only)

    loop {
        if executed >= max_insns as u64 {
            break;
        }
        let (r, r1, n) = m.step_fast();
        executed += n as u64;
        i += 1;
        let pc = m.cpu[0].pc;
        // --- Exception handling ---
        if let StepResult::Exception { cause } = r {
            if (32..=37).contains(&cause) {
                continue; // Window overflow/underflow — normal.
            }
            if cause == 5 {
                // ALLOCA (movsp stack-guard spill request) — normal,
                // firmware-handled VM event like window spills: the
                // handler spills windows and resumes past the movsp.
                // Aborting here killed healthy MicroPython REPL runs.
                continue;
            }
            if cause == 1 && env::var("SYSCALL_CONTINUE").is_ok() {
                // Let the firmware's own exception vector handle syscalls
                // (raise_cause already vectored; e.g. MicroPython issues
                // syscalls its kernel handles).
                continue;
            }
            if cause == 0 {
                // Illegal instruction — likely the ESP-IDF panic `ill`.
                // PANIC_CONTINUE lets the firmware's own handler run it
                // (panic_abort ends in a deliberate `ill`; the coredump
                // sketch needs the reboot it triggers).
                if env::var("PANIC_CONTINUE").is_ok() {
                    continue;
                }
                let epc = m.cpu[0].sreg(SR_EPC1);
                println!(
                    "\n>> ILLEGAL at step {i}, pc {:#010x}, EPC1 {:#010x}, wb {}, b0={:#04x}",
                    pc,
                    epc,
                    m.cpu[0].windowbase(),
                    m.soc.read8(epc)
                );
                break;
            }
            let epc = m.cpu[0].sreg(SR_EPC1);
            println!(
                "\n== step {i}: exception(cause={cause}) at pc {:#010x}; EPC1 {:#010x}, a0={:#x} a1={:#x} a2={:#x}",
                pc,
                epc,
                m.cpu[0].reg(0),
                m.cpu[0].reg(1),
                m.cpu[0].reg(2),
            );
            break;
        }
        if let StepResult::Exception { cause } = r1 {
            if cause != 0 && (32..=37).contains(&cause) {
                continue;
            }
            if cause == 5 {
                continue; // ALLOCA spill request (see core0 arm).
            }
            if cause == 1 && env::var("SYSCALL_CONTINUE").is_ok() {
                continue; // Same as above (core1 syscalls).
            }
            if cause == 0 && env::var("PANIC_CONTINUE").is_ok() {
                continue; // Deliberate panic `ill` (see core0 arm).
            }
            println!(
                "\n== step {i}: core1 exception(cause={cause}) at pc {:#010x}; EPC1 {:#010x}",
                m.cpu[1].pc,
                m.cpu[1].sreg(SR_EPC1),
            );
            break;
        }
        // Unimplemented instructions (ee.* DSP/TIE extensions: decoded but
        // with no execution model) trap LOUDLY instead of hanging: the pc
        // is frozen on the faulting op, so ignoring the result would spin
        // forever. A dynamic audit (2026-09-03: 24 sketches x 96M insns,
        // both cores) shows zero executions, so this never fires today.
        if matches!(r, StepResult::Unimplemented(_)) {
            let pc = m.cpu[0].pc;
            let detail = unimp_detail(&mut m, 0);
            println!("\n>> UNIMPLEMENTED core0 at pc {pc:#010x}: {detail}",);
            break;
        }
        if matches!(r1, StepResult::Unimplemented(_)) {
            let pc = m.cpu[1].pc;
            let detail = unimp_detail(&mut m, 1);
            println!("\n>> UNIMPLEMENTED core1 at pc {pc:#010x}: {detail}",);
            break;
        }

        // --- UART output (every step): the drain must stay per-step.
        // `take_uart_tx(0)` merges the UART0 and USB-Serial-JTAG FIFOs and
        // its ROM-doubling dedup pairs each UART byte with its USB twin in
        // the same drain call. Both cores print concurrently during boot,
        // so any batching wider than a step interleaves the two streams
        // across drain windows (`out != usb`) and the single-byte
        // cross-call state lets doubled bytes through — batching this was
        // tried and corrupted the early-boot log line. The per-step drain
        // itself is cheap (empty Vec handoffs + a DRAM fast-path read).
        let tx = m.take_uart_tx(0);
        let tx1 = m.take_uart_tx(1);
        if !tx1.is_empty() {
            uart_buf.extend_from_slice(&tx1);
        }
        if !tx.is_empty() {
            uart_buf.extend_from_slice(&tx);
        }

        // UART1 RX injection: once the app prints the ready marker, push
        // the host payload into UART1 RX (the echo sketch reads it back).
        // Gated on new bytes: the buffer only changes when a drain above
        // appended, so scanning every step is wasted O(buffer) work.
        if !uart1_injected
            && (!tx.is_empty() || !tx1.is_empty())
            && let Some(bytes) = &uart1_inject
            && uart_buf.windows(b"RXREADY".len()).any(|w| w == b"RXREADY")
        {
            for &b in bytes {
                m.soc.uart_inject_rx(1, b);
            }
            println!(
                "[host] injected {:?} into UART1 RX",
                String::from_utf8_lossy(bytes)
            );
            uart1_injected = true;
        }

        // UART0 RX injection: same pattern with a configurable marker.
        if !uart0_injected
            && (!tx.is_empty() || !tx1.is_empty())
            && let Some(bytes) = &uart0_inject
            && uart_buf
                .windows(uart0_marker.len())
                .any(|w| w == uart0_marker.as_slice())
        {
            for &b in bytes {
                m.soc.uart_inject_rx(0, b);
            }
            println!(
                "[host] injected {:?} into UART0 RX",
                String::from_utf8_lossy(bytes)
            );
            uart0_injected = true;
        }

        // USB-CDC RX injection: same pattern with a configurable marker.
        if !usb_injected
            && (!tx.is_empty() || !tx1.is_empty())
            && let Some(bytes) = &usb_inject
            && uart_buf
                .windows(usb_marker.len())
                .any(|w| w == usb_marker.as_slice())
        {
            for &b in bytes {
                m.soc.usb_inject_rx(b);
            }
            println!(
                "[host] injected {:?} into USB CDC RX",
                String::from_utf8_lossy(bytes)
            );
            usb_injected = true;
        }

        // SPI-slave host exchange: marker-driven, one shot per half.
        if spi_slave_xchg && (!tx.is_empty() || !tx1.is_empty()) {
            if !spi_slave_wrote
                && uart_buf
                    .windows(b"SPI SLAVE READY".len())
                    .any(|w| w == b"SPI SLAVE READY")
            {
                m.soc.spi_slave_inject_write(0, &[0x11, 0x22, 0x33]);
                println!("[host] SPI slave master-write [11 22 33]");
                spi_slave_wrote = true;
            }
            if spi_slave_wrote
                && !spi_slave_read
                && uart_buf
                    .windows(b"SPI SLAVE TX-REQ".len())
                    .any(|w| w == b"SPI SLAVE TX-REQ")
            {
                let got = m.soc.spi_slave_take_read(0, 2);
                println!("[host] SPI slave master-read -> {got:02x?}");
                assert_eq!(got, vec![0xA5, 0xC3], "slave TX preload mismatch");
                spi_slave_read = true;
            }
            // Slave-DMA halves: the model routes through the GDMA links
            // when the sketch enables DMA_CONF rx/tx (same env flag).
            if !spi_slave_dma_wrote
                && uart_buf
                    .windows(b"SPI SLAVE DMA READY".len())
                    .any(|w| w == b"SPI SLAVE DMA READY")
            {
                m.soc.spi_slave_inject_write(0, &[0xDE, 0xAD, 0xBE, 0xEF]);
                println!("[host] SPI slave DMA master-write [de ad be ef]");
                spi_slave_dma_wrote = true;
            }
            if spi_slave_dma_wrote
                && !spi_slave_dma_read
                && uart_buf
                    .windows(b"SPI SLAVE DMA TX-REQ".len())
                    .any(|w| w == b"SPI SLAVE DMA TX-REQ")
            {
                let got = m.soc.spi_slave_take_read(0, 4);
                println!("[host] SPI slave DMA master-read -> {got:02x?}");
                assert_eq!(got, vec![0x12, 0x34, 0x56, 0x78], "slave DMA TX mismatch");
                spi_slave_dma_read = true;
            }
        }

        // I2C-slave host exchange: marker-driven, one shot per half.
        if i2c_slave_xchg && (!tx.is_empty() || !tx1.is_empty()) {
            if !i2c_slave_wrote
                && uart_buf
                    .windows(b"I2C SLAVE READY".len())
                    .any(|w| w == b"I2C SLAVE READY")
            {
                m.soc.i2c_slave_inject_write(0, 0x42, &[0x11, 0x22]);
                println!("[host] I2C slave master-write [11 22]");
                i2c_slave_wrote = true;
            }
            if i2c_slave_wrote
                && !i2c_slave_read
                && uart_buf
                    .windows(b"I2C SLAVE TX-REQ".len())
                    .any(|w| w == b"I2C SLAVE TX-REQ")
            {
                let got = m.soc.i2c_slave_take_read(0, 0x42, 1);
                println!("[host] I2C slave master-read -> {got:02x?}");
                assert_eq!(got, vec![0xA5], "slave TX preload mismatch");
                i2c_slave_read = true;
            }
        }

        // --- Stall / idle detection ---
        // (Skipped while fast-forwarding deep sleep: the CPUs are halted by
        // design for the whole sleep, however long the ULP program runs.)
        if m.is_asleep() {
            idle_steps = 0;
            stuck = 0;
        }
        let uart_len = uart_buf.len();
        if pc == last_pc && uart_len == last_uart_len {
            idle_steps += 1;
            stuck += 1;
            // 1M macro-steps: the ROM stub loader copies DRAM segments
            // byte-wise (~265k iterations for a big .rodata image like
            // MicroPython's), and block-at-a-time execution samples the
            // same copy-loop pc every macro-step — 200k tripped mid-copy.
            if stuck == 1_000_000 {
                println!(
                    "\n== STUCK: core0 pc {:#010x} unchanged for 1M steps at step {i} (a0={:#x} a1={:#x} a2={:#x}) ==",
                    pc,
                    m.cpu[0].reg(0),
                    m.cpu[0].reg(1),
                    m.cpu[0].reg(2),
                );
                break;
            }
            // Early-exit: if both cores are idle (no PC change + no new UART
            // output) for 2M steps, the firmware's setup() has finished and
            // the main loop is spinning — we've seen all the meaningful output.
            // IDLE_STEPS overrides (slow boots like MicroPython need more).
            let idle_limit: usize = env::var("IDLE_STEPS")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(2_000_000);
            if idle_steps >= idle_limit {
                println!("\n== IDLE: no output or PC change for 2M steps at step {i} ==");
                break;
            }
        } else {
            idle_steps = 0;
            stuck = 0;
        }
        last_pc = pc;
        last_uart_len = uart_buf.len();
    }

    // --- Final drain: grab any remaining UART/USB-Serial TX bytes ---
    {
        let tx1 = m.take_uart_tx(1);
        uart_buf.extend_from_slice(&tx1);
        let tx = m.take_uart_tx(0);
        uart_buf.extend_from_slice(&tx);
    }

    // --- Final report (MIPS = executed instructions/sec) ---
    let elapsed = t0.elapsed();
    let mips = executed as f64 / elapsed.as_secs_f64() / 1_000_000.0;
    println!(
        "\n== end: core0 pc {:#010x}, core1 pc {:#010x}, {} insns ({} macro-steps) in {:.2}s ({:.1} MIPS) ==",
        m.cpu[0].pc,
        m.cpu[1].pc,
        executed,
        i,
        elapsed.as_secs_f64(),
        mips
    );
    for c in 0..2 {
        let cpu = &m.cpu[c];
        println!(
            "== core{c}: a0={:#010x} a2={:#010x} sp={:#010x} wb={} ps={:#x}",
            cpu.reg(0),
            cpu.reg(2),
            cpu.reg(1),
            cpu.windowbase(),
            cpu.ps()
        );
    }
    println!(
        "== uart bytes ({}): {:?}",
        uart_buf.len(),
        String::from_utf8_lossy(&uart_buf)
    );
    println!(
        "== irq taken core0={} core1={}",
        m.cpu[0].dbg_irq_taken, m.cpu[1].dbg_irq_taken,
    );
    println!(
        "== int_pending(0)={:#x} int_pending(1)={:#x}",
        m.soc.int_pending(0),
        m.soc.int_pending(1),
    );
}
