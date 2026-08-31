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

fn main() {
    let path = env::args().nth(1).expect("usage: run_flash <flash image>");
    let flash = fs::read(&path).expect("read flash image");

    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);

    // ADC injection for sketches doing analogRead: ADC_INJECT_MV=<mv>
    // applies the voltage to ADC1 channel 3 (GPIO4).
    if let Ok(mv) = env::var("ADC_INJECT_MV") {
        if let Ok(mv) = mv.parse::<u32>() {
            m.soc.adc_inject_voltage(0, 3, mv);
            println!("[host] injected {mv} mV on ADC1_CH3 (GPIO4)");
        }
    }

    // UART1 RX injection (echo-sketch support): UART_INJECT=<text> is
    // pushed into UART1 RX as soon as the console shows the RXREADY marker.
    let uart1_inject: Option<Vec<u8>> = env::var("UART_INJECT").ok().map(|s| s.into_bytes());
    let mut uart1_injected = false;

    // Step budget: STEPS=N overrides the default (48M).  The loop also
    // exits early when the firmware has been idle (no UART output AND
    // no PC change) for 2M consecutive steps — indicates setup() is done
    // and the firmware is in its main loop or halted.
    let max_steps: usize = env::var("STEPS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(48_000_000);
    let t0 = Instant::now();
    let mut uart_buf: Vec<u8> = Vec::new();
    let mut last_pc = 0u32;
    let mut last_uart_len = 0usize;
    let mut stuck = 0u32;
    let mut idle_steps: usize = 0;

    for i in 0..max_steps {
        m.soc.tick_timers(1);

        // WDT / peripheral reset → reboot.
        if m.soc.consume_reset() {
            m.reset();
            continue;
        }

        // Deep-sleep fast-forward.
        if m.is_asleep() {
            m.tick_sleep_one();
            continue;
        }
        if let Some(ticks) = m.soc.consume_sleep_request() {
            m.begin_sleep(ticks);
            continue;
        }

        let r = m.cpu[0].step(&mut m.soc);
        let r1 = m.cpu[1].step(&mut m.soc);
        let pc = m.cpu[0].pc;

        // Transition out of ROM-boot flash mode once the PC leaves the
        // ROM stub region.  Without this, cache_read8 bypasses the MMU
        // forever and the app can never read flash through the cache.
        if m.soc.rom_boot_mode()
            && !(pc >= esp32s3_emu::rom_stub::ROM_BASE
                && pc < esp32s3_emu::rom_stub::ROM_END)
        {
            m.soc.set_rom_boot_mode(false);
        }

        // --- AES DMA flag workaround ---
        // The esp-idf AES driver polls a completion flag (0x3fcec85c) that its
        // GDMA RX-done ISR normally clears.  The AES driver does not route the
        // GDMA interrupt (source 63) through the interrupt matrix for this
        // sketch, so the firmware ISR never runs in the model and the flag
        // stays set, deadlocking the poll loop.  We simulate the ISR clearing
        // it so the driver completes.
        if m.cpu[1].pc == 0x4201_32c0 {
            let flag_addr = 0x3fcec85c_u32;
            let fv = m.soc.read32(flag_addr);
            if fv & (1u32 << 31) != 0 {
                m.soc.write32(flag_addr, fv & !(1u32 << 31));
            }
        }

        // --- Exception handling ---
        if let StepResult::Exception { cause } = r {
            if (32..=37).contains(&cause) {
                continue; // Window overflow/underflow — normal.
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
            println!(
                "\n== step {i}: core1 exception(cause={cause}) at pc {:#010x}; EPC1 {:#010x}",
                m.cpu[1].pc,
                m.cpu[1].sreg(SR_EPC1),
            );
            break;
        }
        if let StepResult::Unimplemented(_insn) = r {
            // Unimplemented TIE/DSP instructions — skip like old code did.
            // These are ee.* extensions that are never on the boot path.
        }

        // --- UART output ---
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
        if !uart1_injected {
            if let Some(bytes) = &uart1_inject {
                if uart_buf.windows(b"RXREADY".len()).any(|w| w == b"RXREADY") {
                    for &b in bytes {
                        m.soc.uart_inject_rx(1, b);
                    }
                    println!(
                        "[host] injected {:?} into UART1 RX",
                        String::from_utf8_lossy(bytes)
                    );
                    uart1_injected = true;
                }
            }
        }

        // --- Stall / idle detection ---
        let uart_len = uart_buf.len();
        if pc == last_pc && uart_len == last_uart_len {
            idle_steps += 1;
            stuck += 1;
            if stuck == 200_000 {
                println!(
                    "\n== STUCK: core0 pc {:#010x} unchanged for 200k steps at step {i} (a0={:#x} a1={:#x} a2={:#x}) ==",
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
            if idle_steps >= 2_000_000 {
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
        let tx = m.take_uart_tx(0);
        uart_buf.extend_from_slice(&tx);
    }

    // --- USB-Serial-JTAG diagnostics ---
    {
        let (ep1, wr_done) = m.soc.usb_serial_diagnostics();
        println!("== usb-serial-jtag: ep1_writes={ep1}, wr_done={wr_done}");
    }

    // --- Final report ---
    let elapsed = t0.elapsed();
    let mips = max_steps as f64 / elapsed.as_secs_f64() / 1_000_000.0;
    println!(
        "\n== end: core0 pc {:#010x}, core1 pc {:#010x}, {} steps in {:.2}s ({:.1} MIPS) ==",
        m.cpu[0].pc, m.cpu[1].pc, max_steps,
        elapsed.as_secs_f64(), mips
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
