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
    m.boot_from_flash(&flash);

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

    // I2C-slave host exchange (slave-sketch support): I2C_SLAVE_XCHG=1 drives
    // both halves when the app prints its markers — a master-write-to-slave
    // (addr 0x42, [0x11, 0x22]) on "I2C SLAVE READY", then a 1-byte
    // master-read-from-slave (expecting the preloaded [0xA5]) on
    // "I2C SLAVE TX-REQ".
    let i2c_slave_xchg = env::var("I2C_SLAVE_XCHG").is_ok();
    let mut i2c_slave_wrote = false;
    let mut i2c_slave_read = false;

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
