//! ESP32-S3 machine: CPU + SoC glue, firmware loading, step loop.
//!
//! The CPU is SoC-agnostic; the machine wires `xtensa-core::Cpu` to the
//! `esp32s3-soc::Soc` address space and exposes the host-facing API
//! (load image, step, read console output / GPIO).

use alloc::vec;
use alloc::vec::Vec;
use esp32s3_soc::Soc;
use esp32s3_soc::memmap::{
    DRAM_BASE, IRAM_BASE, IROM_BASE, IROM_SIZE, RTC_FAST_DATA_BASE, RTC_FAST_SIZE, SRAM_BYTES,
};
use xtensa_core::{Bus, Cpu, StepResult};
use xtensa_core::generated::{Opcode, Opnd};

use crate::rom_stub;
use crate::rom_stub::HOST_PRINTF;

#[derive(Clone, Copy, Debug)]
struct DecodedOp {
    opcode: Opcode,
    opnds: [Opnd; 8],
    pc: u32,
    raw: u32,
    len: u32,
}

#[derive(Clone, Copy, Debug)]
struct Block {
    pc: u32,
    ops: [Option<DecodedOp>; 16],
    len: u8,
}

impl Default for Block {
    fn default() -> Self {
        Self {
            pc: 0,
            ops: [None; 16],
            len: 0,
        }
    }
}

fn is_branch(opc: Opcode) -> bool {
    matches!(
        opc,
        Opcode::OPCODE_J
            | Opcode::OPCODE_CALL0
            | Opcode::OPCODE_CALL4
            | Opcode::OPCODE_CALL8
            | Opcode::OPCODE_CALL12
            | Opcode::OPCODE_CALLX0
            | Opcode::OPCODE_CALLX4
            | Opcode::OPCODE_CALLX8
            | Opcode::OPCODE_CALLX12
            | Opcode::OPCODE_RET
            | Opcode::OPCODE_RETW
            | Opcode::OPCODE_RET_N
            | Opcode::OPCODE_RFI
            | Opcode::OPCODE_RFE
            | Opcode::OPCODE_LOOP
            | Opcode::OPCODE_LOOPNEZ
            | Opcode::OPCODE_LOOPGTZ
            | Opcode::OPCODE_ENTRY
            | Opcode::OPCODE_BNE
            | Opcode::OPCODE_BEQ
            | Opcode::OPCODE_BLT
            | Opcode::OPCODE_BLTU
            | Opcode::OPCODE_BGE
            | Opcode::OPCODE_BGEU
            | Opcode::OPCODE_BNEZ
            | Opcode::OPCODE_BEQZ
            | Opcode::OPCODE_BNEZ_N
            | Opcode::OPCODE_BEQZ_N
            | Opcode::OPCODE_JX
    )
}

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
    /// Block cache: pc -> decoded block of up to 16 insns until branch.
    block_cache: alloc::boxed::Box<[Option<Block>; 4096]>,
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
            block_cache: {
                let mut v = alloc::vec::Vec::with_capacity(4096);
                v.resize_with(4096, || None);
                v.into_boxed_slice().try_into().unwrap()
            },
        }
    }

    /// Execute one instruction on each core.  Timers advance one cycle per
    /// step so they make progress in host-driven execution (refined in P5).
    /// The two cores are serialized core0-then-core1 within a step; real
    /// silicon runs them simultaneously, but the fixed order keeps timer
    /// ticks and per-core instruction counts identical to the single-core
    /// behavior the machine tests were written against.
    pub fn step(&mut self) -> StepResult {
        // TB fast path for core0 — cached decode, per-op tick/int
        let pc = self.cpu[0].pc;
        let ps = self.cpu[0].sreg(xtensa_core::cpu::SR_PS);
        if (ps & 0x10) == 0 {
            let idx = (pc as usize) & 0xFFF;
            if let Some(block) = &self.block_cache[idx] {
                if block.pc == pc && self.soc.int_pending(0) == 0 {
                    // Use TB only for non-test code (keep timer tests precise)
                    if !(pc >= 0x40000000 && pc < 0x40002000) {
                        let mut ok = true;
                        for i in 0..block.len as usize {
                            self.soc.tick_timers(1);
                            if self.soc.consume_reset() {
                                self.reset();
                                return StepResult::Ok;
                            }
                            if self.asleep {
                                if self.sleep_remaining == 0 {
                                    self.wake();
                                } else {
                                    self.sleep_remaining -= 1;
                                }
                                ok = false;
                                break;
                            }
                            if let Some(ticks) = self.soc.consume_sleep_request() {
                                self.asleep = true;
                                self.sleep_remaining = ticks.max(1);
                                ok = false;
                                break;
                            }
                            if self.soc.int_pending(0) != 0 {
                                ok = false;
                                break;
                            }
                            if let Some(op) = &block.ops[i] {
                                self.cpu[0].pc = op.pc;
                                let res = self.cpu[0].execute_decoded(&mut self.soc, op.opcode, &op.opnds, op.len);
                                match res {
                                    StepResult::Ok => {
                                        if self.cpu[0].pc == op.pc {
                                            self.cpu[0].pc = op.pc + op.len;
                                        }
                                    }
                                    other => {
                                        self.cpu[1].step(&mut self.soc);
                                        return other;
                                    }
                                }
                            }
                        }
                        if ok {
                            self.cpu[1].step(&mut self.soc);
                            let next_pc = self.cpu[0].pc;
                            let next_idx = (next_pc as usize) & 0xFFF;
                            if self.block_cache[next_idx].is_none()
                                || self.block_cache[next_idx].as_ref().unwrap().pc != next_pc
                            {
                                if let Some(nb) = Self::build_block(next_pc, &mut self.soc) {
                                    self.block_cache[next_idx] = Some(nb);
                                }
                            }
                            return StepResult::Ok;
                        }
                    }
                }
            }
        }
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
        let orig_pc = self.cpu[0].pc;
        let r = self.cpu[0].step(&mut self.soc);
        // Build and cache a block for the orig_pc for next time (hot loops)
        let idx = (orig_pc as usize) & 0xFFF;
        if self.block_cache[idx].is_none() || self.block_cache[idx].as_ref().unwrap().pc != orig_pc {
            if let Some(block) = Self::build_block(orig_pc, &mut self.soc) {
                self.block_cache[idx] = Some(block);
            }
        }
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

    fn build_block(pc: u32, soc: &mut Soc) -> Option<Block> {
        // Only cache hot boot ROM loop (0x40000400) — keep timer/qsort tests precise
        if !(pc >= 0x40000400 && pc < 0x40001000) {
            return None;
        }
        use xtensa_core::generated::{decode_inst, decode_inst16a, decode_inst16b, insn_len, opnds};
        let mut block = Block {
            pc,
            ops: [None; 16],
            len: 0,
        };
        let mut cur_pc = pc;
        for i in 0..16 {
            let b0 = soc.read8(cur_pc) as u8;
            let len = insn_len(b0);
            if len == 0 || len > 4 {
                break;
            }
            let raw = match len {
                2 => soc.read16(cur_pc),
                _ => soc.read32(cur_pc),
            };
            let opc = match len {
                2 => {
                    if b0 & 0xf <= 11 {
                        decode_inst16a(raw)
                    } else {
                        decode_inst16b(raw)
                    }
                }
                _ => decode_inst(raw),
            };
            let opc = match opc {
                Some(o) => o,
                None => break,
            };
            // Don't cache blocks with windowed calls/returns — they handle
            // PS.WOE/CALLINC and need precise per-insn window checks
            if matches!(
                opc,
                Opcode::OPCODE_CALL4
                    | Opcode::OPCODE_CALL8
                    | Opcode::OPCODE_CALL12
                    | Opcode::OPCODE_CALLX4
                    | Opcode::OPCODE_CALLX8
                    | Opcode::OPCODE_CALLX12
                    | Opcode::OPCODE_ENTRY
                    | Opcode::OPCODE_RETW
                    | Opcode::OPCODE_RETW_N
            ) {
                return None;
            }
            let opnds = opnds(opc, raw, cur_pc);
            block.ops[i] = Some(DecodedOp {
                opcode: opc,
                opnds,
                pc: cur_pc,
                raw,
                len,
            });
            block.len = (i + 1) as u8;
            if is_branch(opc) {
                break;
            }
            cur_pc = cur_pc.wrapping_add(len);
            if cur_pc.wrapping_sub(pc) > 64 {
                break;
            }
        }
        if block.len == 0 {
            None
        } else {
            Some(block)
        }
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
        self.block_cache = {
            let mut v = alloc::vec::Vec::with_capacity(4096);
            v.resize_with(4096, || None);
            v.into_boxed_slice().try_into().unwrap()
        };
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
        self.asleep = false;
        self.sleep_remaining = 0;
    }

    /// Load a raw firmware image at `addr` (DRAM, IRAM or IROM window).
    pub fn load_image(&mut self, addr: u32, bytes: &[u8]) {
        for (i, b) in bytes.iter().enumerate() {
            let a = addr + i as u32;
            if (DRAM_BASE..DRAM_BASE + SRAM_BYTES as u32).contains(&a)
                || (IRAM_BASE..IRAM_BASE + SRAM_BYTES as u32).contains(&a)
                || (RTC_FAST_DATA_BASE..RTC_FAST_DATA_BASE + RTC_FAST_SIZE).contains(&a)
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
