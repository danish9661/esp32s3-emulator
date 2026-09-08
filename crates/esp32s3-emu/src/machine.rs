//! ESP32-S3 machine: CPU + SoC glue, firmware loading, step loop.
//!
//! The CPU is SoC-agnostic; the machine wires `xtensa-core::Cpu` to the
//! `esp32s3-soc::Soc` address space and exposes the host-facing API
//! (load image, step, read console output / GPIO).

use alloc::vec;
use alloc::vec::Vec;
use esp32s3_soc::Soc;
use esp32s3_soc::memmap::{
    DRAM_BASE, IRAM_BASE, IROM_BASE, IROM_SIZE, ROM_DATA_BASE, ROM_DATA_SIZE, RTC_FAST_BASE,
    RTC_FAST_SIZE, RTC_SLOW_BASE, RTC_SLOW_SIZE, SRAM_BYTES,
};
use xtensa_core::{Bus, Cpu, StepResult};

use crate::rom_stub;
use crate::rom_stub::HOST_PRINTF;

pub struct Esp32S3 {
    /// Both ESP32-S3 LX7 cores.  Core 1 is gated at reset by the ROM stub
    /// (rom_stub.rs: PRID check) until core 0 releases it, mirroring the
    /// real ROM's APP CPU boot flow (QEMU esp32s3.c runs both CPUs and lets
    /// the ROM gate CPU1 — no release register is modeled).
    pub cpu: [Cpu; 2],
    pub soc: Soc,
    /// Last flash image passed to `boot_from_flash`, retained so a WDT/system
    /// reset can re-run the boot sequence.
    flash: Vec<u8>,
    /// True while the machine is fast-forwarding a deep-sleep period (CPU
    /// halted, no instructions executed).
    asleep: bool,
    /// Remaining steps to fast-forward while `asleep`.
    sleep_remaining: u64,
    /// Dedup state for the ROM console doubling bug: the ROM's putc at
    /// 0x40043CE8 writes each char to BOTH UART0 (0x60000000) and
    /// USB-Serial-JTAG (0x60038000).  Merging both FIFOs doubles every char.
    /// Track the last emitted byte so the UART duplicate can be dropped.
    last_console_byte: Option<u8>,
    last_console_was_usb: bool,
    /// Global tick parity for `step_fast`: toggled before every fast-block
    /// op so peripheral time advances once per two ops (the single-step
    /// ratio), shared by both cores' loops within a macro-step.
    fast_tick: bool,
}

impl Esp32S3 {
    pub fn new() -> Self {
        Self {
            cpu: [Cpu::new(0), Cpu::new(1)],
            soc: Soc::new(),
            flash: Vec::new(),
            asleep: false,
            sleep_remaining: 0,
            last_console_byte: None,
            last_console_was_usb: false,
            fast_tick: false,
        }
    }

    /// Execute one instruction on each core.  Timers advance one cycle per
    /// step so they make progress in host-driven execution (refined in P5).
    /// The two cores are serialized core0-then-core1 within a step; real
    /// silicon runs them simultaneously, but the fixed order keeps timer
    /// ticks and per-core instruction counts identical to the single-core
    /// behavior the machine tests were written against.
    pub fn step(&mut self) -> StepResult {
        self.soc.tick_timers(1);
        // A device (e.g. a WDT stage with a reset action) may have requested a
        // hard reset while the timers advanced; reboot before executing more.
        if self.soc.consume_reset() {
            self.reset();
            return StepResult::Ok;
        }
        // Deep-sleep fast-forward: while asleep the CPU is halted.  Count down
        // the captured sleep period then wake (reboot with the timer cause).
        if self.asleep {
            if self.sleep_remaining == 0 {
                self.wake();
            } else {
                self.sleep_remaining -= 1;
            }
            return StepResult::Ok;
        }
        // Firmware requested a deep-sleep this step: enter it and skip the CPU.
        if let Some(ticks) = self.soc.consume_sleep_request() {
            self.asleep = true;
            self.sleep_remaining = ticks.max(1);
            return StepResult::Ok;
        }
        let r = self.cpu[0].step(&mut self.soc);
        if self.soc.rom_boot_mode()
            && !(self.cpu[0].pc >= rom_stub::ROM_BASE && self.cpu[0].pc < rom_stub::ROM_END)
        {
            // The ROM stub's flash reads (via the data window) must bypass the
            // MMU; once core 0 jumps into the app, the app's window reads go
            // through the MMU again (see `Soc::rom_boot_mode`).
            self.soc.set_rom_boot_mode(false);
        }
        self.cpu[1].step(&mut self.soc);
        r
    }

    /// Advance both cores by one fast block each (block-at-a-time execution).
    /// Returns `(core0_result, core1_result, instructions_executed)`.
    ///
    /// Each core runs its cached straight-line block (`Soc::fast_len_for`,
    /// branch op inclusive). Peripheral time advances INSIDE the op loop via
    /// `fast_maybe_tick` — one tick per two ops globally, exactly the
    /// single-step ratio — so firmware-observed peripheral state stays near
    /// identical to single-stepping (a bulk tick upfront was tried and broke
    /// MCPWM duty sampling: frozen mid-block time aliases the firmware's pin
    /// sampling). CCOUNT stays per-instruction exact (delay loops are safe),
    /// and interrupts dispatch once per core at its block end (QEMU TB
    /// granularity, ≤16 instructions late). `step` is untouched and remains
    /// the precise single-step path the machine tests use.
    pub fn step_fast(&mut self) -> (StepResult, StepResult, u32) {
        let llen = self.soc.fast_len_for(0, self.cpu[0].pc);
        let flen = self.soc.fast_len_for(1, self.cpu[1].pc);
        let (Some(llen), Some(flen)) = (llen, flen) else {
            // Undecodable pc (raises ILLEGAL below): single-step both cores
            // with the exact single-step plumbing for one tick.
            self.soc.tick_timers(1);
            if self.soc.consume_reset() {
                self.reset();
                return (StepResult::Ok, StepResult::Ok, 0);
            }
            if self.asleep {
                if self.sleep_remaining == 0 {
                    self.wake();
                } else {
                    self.sleep_remaining -= 1;
                }
                return (StepResult::Ok, StepResult::Ok, 0);
            }
            if let Some(ticks) = self.soc.consume_sleep_request() {
                self.asleep = true;
                self.sleep_remaining = ticks.max(1);
                return (StepResult::Ok, StepResult::Ok, 0);
            }
            let r0 = self.cpu[0].step(&mut self.soc);
            if self.soc.rom_boot_mode()
                && !(self.cpu[0].pc >= rom_stub::ROM_BASE && self.cpu[0].pc < rom_stub::ROM_END)
            {
                self.soc.set_rom_boot_mode(false);
            }
            let r1 = self.cpu[1].step(&mut self.soc);
            return (r0, r1, 2);
        };
        // Deep-sleep plumbing mirrors `step` (per macro-step; entry mid-block
        // takes effect here, ≤16 instructions late).
        if self.asleep {
            if self.sleep_remaining == 0 {
                self.wake();
            } else {
                self.sleep_remaining -= 1;
            }
            return (StepResult::Ok, StepResult::Ok, 0);
        }
        if let Some(ticks) = self.soc.consume_sleep_request() {
            self.asleep = true;
            self.sleep_remaining = ticks.max(1);
            return (StepResult::Ok, StepResult::Ok, 0);
        }
        let (r0, n0) = self.run_fast_core(0, llen);
        let (r1, n1) = self.run_fast_core(1, flen);
        if self.soc.rom_boot_mode()
            && !(self.cpu[0].pc >= rom_stub::ROM_BASE && self.cpu[0].pc < rom_stub::ROM_END)
        {
            // The ROM stub's flash reads (via the data window) must bypass the
            // MMU; once core 0 jumps into the app, the app's window reads go
            // through the MMU again (see `Soc::rom_boot_mode`).
            self.soc.set_rom_boot_mode(false);
        }
        (r0, r1, n0 + n1)
    }

    /// One global timer tick per two executed ops (see `step_fast`): called
    /// before every op in the fast-block loop. Preserves the single-step
    /// tick-per-two-instructions ratio while keeping peripheral time gradual
    /// inside blocks. A WDT/system reset raised by the tick reboots
    /// immediately (the block is abandoned).
    #[inline]
    fn fast_maybe_tick(&mut self) -> bool {
        self.fast_tick = !self.fast_tick;
        if !self.fast_tick {
            return false;
        }
        self.soc.tick_timers(1);
        if self.soc.consume_reset() {
            self.reset();
            return true;
        }
        false
    }

    /// Run up to `len` instructions on `core` via `step_one` (no per-op
    /// interrupt dispatch), stopping early on a reset, any non-Ok result, or
    /// pc deviation (taken branch at the block end, or an unexpected flow
    /// change — the cached entry only stores the run length, so any
    /// deviation ends the run). Dispatches interrupts once at the end.
    /// Returns (result, instructions_executed).
    fn run_fast_core(&mut self, core: usize, len: u8) -> (StepResult, u32) {
        let mut n = 0u32;
        for _ in 0..len {
            if self.fast_maybe_tick() {
                return (StepResult::Ok, n);
            }
            let pc0 = self.cpu[core].pc;
            let r = self.cpu[core].step_one(&mut self.soc);
            // `step_one` records the fetched length even on exception paths,
            // so no re-fetch is needed to verify fall-through advance.
            let elen = self.cpu[core].last_len();
            n += 1;
            match r {
                StepResult::Ok => {
                    let pc = self.cpu[core].pc;
                    // Fall-through continues the block. A pc matching LBEG is
                    // the zero-overhead-loop wrap (taken branches end cached
                    // runs, so any other deviation ends this run).
                    if pc != pc0.wrapping_add(elen)
                        && pc != self.cpu[core].sreg(xtensa_core::cpu::SR_LBEG)
                    {
                        break;
                    }
                }
                other => return (other, n),
            }
        }
        self.cpu[core].check_interrupts(&mut self.soc);
        (StepResult::Ok, n)
    }

    /// Re-run the boot sequence from the last loaded flash image.  Used when a
    /// peripheral (WDT) triggers a system reset.
    pub fn reset(&mut self) {
        self.cpu = [Cpu::new(0), Cpu::new(1)];
        self.soc = Soc::new();
        self.asleep = false;
        self.sleep_remaining = 0;
        self.last_console_byte = None;
        self.last_console_was_usb = false;
        let f = self.flash.clone();
        self.boot_from_flash(&f);
    }

    /// True while the machine is fast-forwarding a deep-sleep period.
    pub fn is_asleep(&self) -> bool {
        self.asleep
    }

    /// Remaining steps to fast-forward while in deep-sleep.
    pub fn sleep_remaining(&self) -> u64 {
        self.sleep_remaining
    }

    /// Fast-forward up to `max_steps` of a deep-sleep period. Returns the
    /// number of steps consumed (may be less than `max_steps` if the sleep
    /// period ended). After this, `is_asleep()` will be false.
    pub fn fast_forward_sleep(&mut self, max_steps: u64) -> u64 {
        let skip = self.sleep_remaining.min(max_steps);
        self.sleep_remaining -= skip;
        if self.sleep_remaining == 0 {
            self.wake();
        }
        skip
    }

    /// Enter deep-sleep for `ticks` (slow-clock) steps; the CPU halts until
    /// the period elapses, then the machine reboots with the wakeup cause set.
    pub fn begin_sleep(&mut self, ticks: u64) {
        self.asleep = true;
        self.sleep_remaining = ticks.max(1);
    }

    /// Advance one step of a deep-sleep fast-forward; wakes when the period
    /// elapses.  Mirrors `Esp32S3::step` for host runners that drive the cores
    /// manually (e.g. `run_flash`).
    pub fn tick_sleep_one(&mut self) {
        if self.sleep_remaining == 0 {
            self.wake();
        } else {
            self.sleep_remaining -= 1;
        }
    }

    /// Reboot after a deep-sleep period, recording a timer wakeup cause so
    /// `esp_sleep_get_wakeup_cause()` returns `ESP_SLEEP_WAKEUP_TIMER`.
    fn wake(&mut self) {
        self.reset();
        self.soc.set_sleep_wakeup_cause(1 << 3); // RTC_TIMER_TRIG_EN
        // Deep-sleep reset reason: `esp_sleep_get_wakeup_cause` only reads
        // the wakeup-cause register when the PRO reason is DEEPSLEEP (5).
        self.soc.set_reset_cause(
            esp32s3_soc::rtc::RESET_CAUSE_DEEPSLEEP,
            esp32s3_soc::rtc::RESET_CAUSE_DEEPSLEEP,
        );
        self.asleep = false;
        self.sleep_remaining = 0;
    }

    /// Load a raw firmware image at `addr` (DRAM, IRAM, RTC, ROM-data or
    /// IROM window).
    pub fn load_image(&mut self, addr: u32, bytes: &[u8]) {
        for (i, b) in bytes.iter().enumerate() {
            let a = addr + i as u32;
            if (DRAM_BASE..DRAM_BASE + SRAM_BYTES as u32).contains(&a)
                || (IRAM_BASE..IRAM_BASE + SRAM_BYTES as u32).contains(&a)
                || (ROM_DATA_BASE..ROM_DATA_BASE + ROM_DATA_SIZE).contains(&a)
                || (RTC_FAST_BASE..RTC_FAST_BASE + RTC_FAST_SIZE).contains(&a)
                || (RTC_SLOW_BASE..RTC_SLOW_BASE + RTC_SLOW_SIZE).contains(&a)
            {
                self.soc.write8(a, *b as u32);
            } else if (IROM_BASE..IROM_BASE + IROM_SIZE).contains(&a) {
                // The Bus write path treats IROM as read-only; the ROM
                // storage is written directly when hosting loads it.
                self.soc.irom_mut()[(a - IROM_BASE) as usize] = *b;
            }
        }
    }

    /// Install the boot ROM stub and a full flash image, then reset the CPU
    /// to the ROM reset vector. The ROM stub (rom_stub.rs) loads the app
    /// image from flash offset `APP_FLASH_OFFSET` and jumps to its entry.
    pub fn boot_from_flash(&mut self, flash: &[u8]) {
        self.flash = flash.to_vec();
        self.soc.load_flash_image(0, flash);
        // Pick the app slot: OTA images select ota_0/ota_1 via the otadata
        // partition; non-OTA images fall back to the factory slot at
        // APP_FLASH_OFFSET (0x10000).
        let app_off =
            crate::partition::select_ota_boot_offset(flash).unwrap_or(rom_stub::APP_FLASH_OFFSET);
        // Pre-map the app's flash-mapped segments (.flash.text/.flash.rodata)
        // in the cache MMU — the real 2nd-stage bootloader maps them instead
        // of copying (the ROM stub's copy loop cannot write the read-only
        // windows).
        self.soc.map_app_flash_segments(app_off);
        // The stub's own flash reads must NOT go through that MMU (it reads
        // the image the way the real ROM reads flash — via SPI, MMU-free).
        self.soc.set_rom_boot_mode(true);
        let rom = rom_stub::rom_image();
        self.load_image(rom_stub::ROM_BASE, &rom);
        self.load_rom_data();
        // ROM data: the ROM layout struct + its pointer in RTC fast memory
        // (esp32s3.rom.ld maps ets_rom_layout_p = 0x3FF1FFFC); the app's
        // heap init reads layout->dram0_rtos_reserved_start via it.  The
        // S3's ets_rom_layout_t starts with magic then dram0_rtos_reserved_*
        // (NOT the ESP32-classic dram0_stack0_* ordering) and the values
        // are the HIGH-DRAM ROM area (Zephyr soc/espressif/esp32s3/memory.h:
        // PRO stack 0x3FCE9710-0x3FCEB710, APP stack 0x3FCEB710-0x3FCED710,
        // ROM .bss/.data 0x3FCED710-0x3FCF0000).  heap_caps_init's overlap
        // check aborts unless dram0_rtos_reserved_start >= the app's bss end
        // (0x3FC982C8) — the old value 0x3FC880A0 overlapped the app's
        // IRAM-alias reserved region {0x3FC84000, 0x3FC92F00}.
        self.soc.write32(rom_stub::ROM_LAYOUT, 0xC5A5_C5A5); // magic
        self.soc.write32(rom_stub::ROM_LAYOUT + 4, 0x3FCE_9710); // dram0_rtos_reserved_start
        self.soc.write32(rom_stub::ROM_LAYOUT + 8, 0x3FCF_0000); // dram0_rtos_reserved_end
        self.soc.write32(rom_stub::ROM_LAYOUT + 24, 0x3FCE_9710); // dram0_stack0_start_addr
        self.soc.write32(rom_stub::ROM_LAYOUT + 28, 0x3FCE_B710); // dram0_stack0_end_addr
        self.soc.write32(rom_stub::ROM_LAYOUT + 32, 0x3FCE_B710); // dram0_stack1_start_addr
        self.soc.write32(rom_stub::ROM_LAYOUT + 36, 0x3FCE_D710); // dram0_stack1_end_addr
        self.soc.write32(0x3FF1_FFFC, rom_stub::ROM_LAYOUT);
        // Reset pc = 0x40000400 (XCHAL_RESET_VECTOR_PADDR), not 0x40000000
        // — the window vectors own the bottom of the ROM (core-isa.h).
        self.cpu[0].pc = xtensa_core::cpu::RESET_VECTOR;
    }

    /// Load the real ROM's data state — rodata tables in RTC fast memory
    /// (0x3FF18000, .rodata @ 0x3FF18C00), .data init values in high DRAM
    /// (0x3FCD7E00-0x3FCF0000: xtos tables, spi_flash driver data, PRO/APP
    /// stacks, shared buffers) and the console-init flags the ROM bootloader
    /// leaves behind (putc1 = uart_tx_one_char @ 0x40000648 + the uart0
    /// ready byte at 0x3FCEFFB8 — without them the real ets_printf no-ops on
    /// [0x3FCEF750] and uart_tx_one_char on [0x3FCEFFB8]; the ROM console
    /// then emits via the USB-Serial-JTAG FIFO @ 0x60038000, captured in
    /// take_uart_tx(0)).  Called by `boot_from_flash` and the ROM-call tests.
    pub fn load_rom_data(&mut self) {
        self.load_image(0x3FF1_8000, rom_stub::rom_rodata_blob());
        self.load_image(0x3FCD_7E00, rom_stub::rom_data_blob());
        // NB: _putc1/_putc2 (0x3FCEF754/0x3FCEF750) are BSS — the real ROM's
        // boot_prepare installs putc1 = ets_write_char_uart via
        // ets_install_uart_printf and leaves putc2 = 0; writing putc2 here
        // makes ets_write_char emit every char TWICE.
        //
        // The boot glue's loader does not run the real boot_prepare, so
        // putc1 stays 0 here — the real ets_printf (0x4004423C, reached via
        // the 0x400005D0 __call_ets_printf wrapper) early-returns and the
        // console stays silent until the APP calls esp_rom_install_uart_
        // printf (which sets putc1 itself — the real firmware boot works
        // without this write; bare-metal tests must install putc1 first).
        // Mirror the bootloader's state instead: putc1 = the real ROM's
        // uart_tx_one_char @ 0x40048C30 (ESP32-S3 ROM writes to the
        // USB-Serial-JTAG FIFO, NOT UART0 — see soc.rs comment).  The
        // older 0x40000648 version targets UART0 and would double all
        // console output (rom_puts also feeds USB-Serial-JTAG).
        // The boot ROM's console (uart_tx_one_char) emits via the
        // USB-Serial-JTAG FIFO, not UART0 — merge it into UART0's stream
        // so console output lands in the same place the app's Serial
        // prints go.
        self.soc.write32(0x3FCE_F754, 0x4004_8C30); // _putc1 = uart_tx_one_char (USB-Serial-JTAG)
        self.soc.write32(0x3FCE_FFB8, 1); // uart0 tx enabled
    }

    /// Bytes emitted by UART `n` since the last call (console output).
    /// Also drains the ROM `ets_printf` mailbox (rom_stub::HOST_PRINTF):
    /// formats the pending message and emits it via UART0.
    pub fn take_uart_tx(&mut self, n: usize) -> Vec<u8> {
        // Fast path: nothing pending in any console source — return without
        // touching the merge/dedup state. The full paired drain below runs
        // only when bytes exist, so its ROM-doubling pairing semantics are
        // unchanged (batching the drain itself was tried and corrupts the
        // doubled early-boot log because both cores print concurrently).
        // With all sources empty the full path would also emit nothing and
        // leave the dedup state untouched, so this is behavior-identical.
        if n == 0 {
            if !self.soc.uart_tx_pending(0)
                && !self.soc.usb_tx_pending()
                && self.soc.read32(HOST_PRINTF) == 0
            {
                return Vec::new();
            }
        } else if !self.soc.uart_tx_pending(n) {
            return Vec::new();
        }
        let mut out = self.soc.take_uart_tx(n);
        if n == 0 {
            let usb = self.soc.take_usb_serial_tx();
            // Dedup the ROM doubling bug: the ROM's putc (0x40043CE8) writes
            // each char to BOTH UART0 and USB.  When merging, the UART copy is
            // a duplicate of the previous USB byte.  Drop it.
            let mut filtered_usb = Vec::new();
            let mut filtered_uart = Vec::new();
            // First handle same-call dedup: if both FIFOs have identical content,
            // the UART copy is a duplicate — keep USB only.
            if !usb.is_empty() && out == usb {
                filtered_uart.clear();
                filtered_usb = usb;
            } else {
                // Cross-call dedup: check each uart byte against last emitted
                for &b in &out {
                    if self.last_console_was_usb && self.last_console_byte == Some(b) {
                        // UART duplicate of previous USB — drop
                    } else {
                        filtered_uart.push(b);
                    }
                }
                for &b in &usb {
                    // USB bytes are never deduped (they are the primary)
                    filtered_usb.push(b);
                }
            }
            out = filtered_uart;
            // Update dedup state from what we actually emit
            if let Some(&last) = filtered_usb.last() {
                self.last_console_byte = Some(last);
                self.last_console_was_usb = true;
            } else if let Some(&last) = out.last() {
                self.last_console_byte = Some(last);
                self.last_console_was_usb = false;
            }
            // Merge: USB first (primary), then UART (filtered), then host
            let mut merged = filtered_usb;
            merged.extend_from_slice(&out);
            out = merged;
        }
        if n == 0 && self.soc.read32(HOST_PRINTF) != 0 {
            let msg = self.format_host_printf();
            out.extend_from_slice(&msg);
            // Host printf bytes update dedup state too (treated as non-USB)
            if let Some(&last) = out.last() {
                self.last_console_byte = Some(last);
                self.last_console_was_usb = false;
            }
        }
        out
    }

    /// Diagnostic: drain each console source separately (uart0, usb, host_printf).
    pub fn take_uart_tx_split(&mut self) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let uart0 = self.soc.take_uart_tx(0);
        let usb = self.soc.take_usb_serial_tx();
        let host = if self.soc.read32(HOST_PRINTF) != 0 {
            self.format_host_printf()
        } else {
            Vec::new()
        };
        (uart0, usb, host)
    }

    /// Format the pending `ets_printf` message (host printf mailbox) and
    /// clear the mailbox.  `%d %u %x %X %p %s %c %%` with `-`/`0` flags,
    /// decimal width and `l`/`h`/`z` length prefixes are supported — the
    /// ESP-IDF panic/assert messages use these; `%f` and friends print as
    /// `%f` verbatim.
    fn format_host_printf(&mut self) -> Vec<u8> {
        let fmt = self.soc.read32(HOST_PRINTF);
        let mut args = [0u32; 5];
        for (i, a) in args.iter_mut().enumerate() {
            *a = self.soc.read32(HOST_PRINTF + 4 + 4 * i as u32);
        }
        self.soc.write32(HOST_PRINTF, 0);
        let mut out = Vec::new();
        let mut ai = 0usize;
        let mut p = fmt;
        loop {
            let c = self.soc.read8(p) as u8;
            if c == 0 {
                break;
            }
            p += 1;
            if c != b'%' {
                out.push(c);
                continue;
            }
            // flags
            let mut left = false;
            let mut zero = false;
            loop {
                match self.soc.read8(p) as u8 {
                    b'-' => left = true,
                    b'0' => zero = true,
                    _ => break,
                }
                p += 1;
            }
            // width (digits)
            let mut width = 0usize;
            loop {
                let d = self.soc.read8(p) as u8;
                if !d.is_ascii_digit() {
                    break;
                }
                width = width * 10 + (d - b'0') as usize;
                p += 1;
            }
            // precision digits are consumed but ignored (rare in IDF logs)
            if self.soc.read8(p) as u8 == b'.' {
                p += 1;
                loop {
                    let d = self.soc.read8(p) as u8;
                    if !d.is_ascii_digit() {
                        break;
                    }
                    p += 1;
                }
            }
            // length prefixes
            while let b'l' | b'h' | b'L' | b'z' | b'j' | b't' = self.soc.read8(p) as u8 {
                p += 1;
            }
            let conv = self.soc.read8(p) as u8;
            p += 1;
            if conv == b'%' {
                out.push(b'%');
                continue;
            }
            let arg = if ai < args.len() {
                let v = args[ai];
                ai += 1;
                v
            } else {
                0
            };
            let field: Vec<u8> = match conv {
                b's' => {
                    let mut sp = arg;
                    let mut s = Vec::new();
                    loop {
                        let b = self.soc.read8(sp) as u8;
                        if b == 0 {
                            break;
                        }
                        s.push(b);
                        sp += 1;
                    }
                    s
                }
                b'c' => vec![arg as u8],
                b'd' | b'i' => {
                    let v = arg as i32;
                    if v < 0 {
                        let mut t = vec![b'-'];
                        t.extend(itoa(v.wrapping_neg() as u32));
                        t
                    } else {
                        itoa(v as u32)
                    }
                }
                b'u' => itoa(arg),
                b'x' | b'X' | b'p' => {
                    let mut h = if conv == b'p' {
                        b"0x".to_vec()
                    } else {
                        Vec::new()
                    };
                    h.extend(hex(arg, conv == b'X'));
                    h
                }
                b'o' => oct(arg),
                _ => vec![b'%', conv],
            };
            // width padding (zero-flag only makes sense for numeric fields)
            let pad = width.saturating_sub(field.len());
            if pad > 0 && !left {
                for _ in 0..pad {
                    out.push(if zero { b'0' } else { b' ' });
                }
            }
            out.extend_from_slice(&field);
            if pad > 0 && left {
                out.extend(core::iter::repeat_n(b' ', pad));
            }
        }
        out
    }

    /// Output-pin state (host LED visualization).
    pub fn gpio_output(&self) -> u32 {
        self.soc.gpio_output()
    }
}

/// Decimal digits of `n` (no sign).
fn itoa(n: u32) -> Vec<u8> {
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    let mut n = n;
    loop {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        if n == 0 {
            break;
        }
    }
    buf[i..].to_vec()
}

/// Lower/upper hex digits of `n`.
fn hex(n: u32, upper: bool) -> Vec<u8> {
    let table: &[u8; 16] = if upper {
        b"0123456789ABCDEF"
    } else {
        b"0123456789abcdef"
    };
    let mut buf = [0u8; 8];
    let mut i = buf.len();
    let mut n = n;
    loop {
        i -= 1;
        buf[i] = table[(n & 0xF) as usize];
        n >>= 4;
        if n == 0 {
            break;
        }
    }
    buf[i..].to_vec()
}

/// Octal digits of `n`.
fn oct(n: u32) -> Vec<u8> {
    let mut buf = [0u8; 11];
    let mut i = buf.len();
    let mut n = n;
    loop {
        i -= 1;
        buf[i] = b'0' + (n & 7) as u8;
        n >>= 3;
        if n == 0 {
            break;
        }
    }
    buf[i..].to_vec()
}

impl Default for Esp32S3 {
    fn default() -> Self {
        Self::new()
    }
}
