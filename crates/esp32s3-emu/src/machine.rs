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
    /// Secure-boot rejection latch (fail-closed): set by `boot_from_flash`
    /// when eFuse SECURE_BOOT_EN is set. With a signed bootloader image the
    /// ROM path verifies the app region via `secure_boot::verify_image`
    /// (ECDSA-P256 through the ECDSA model); without one — or when the
    /// signature fails — the boot is refused and both CPUs park (no ticks,
    /// no output) instead of insecurely booting. Cleared on the next
    /// allowed boot. `boot_deny_reason` records which check refused.
    boot_denied: bool,
    /// Why the last boot was refused (`None` when allowed): "unsigned" (no
    /// signature sector) or "bad-signature" (present but invalid).
    boot_deny_reason: Option<&'static str>,
    /// Remaining steps to fast-forward while `asleep`.
    sleep_remaining: u64,
    /// Light (resume, clocks gated) vs deep (reboot) sleep for the current
    /// `asleep` period. Captured from the SLEEP_EN-time DIG_PWC PD config.
    sleep_light: bool,
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
            boot_denied: false,
            boot_deny_reason: None,
            sleep_remaining: 0,
            sleep_light: false,
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
        if self.boot_denied {
            return StepResult::Ok;
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
            // A watched ULP halting mid-sleep wakes immediately (ULP cause).
            self.sleep_tick();
            return StepResult::Ok;
        }
        // Firmware requested a sleep this step: enter it and skip the CPU.
        // Deep sleep reboots on wake; light sleep resumes in place (the
        // kind comes from the SLEEP_EN-time DIG_PWC power-down config).
        if let Some((ticks, deep)) = self.soc.consume_sleep_request() {
            self.asleep = true;
            self.sleep_light = !deep;
            self.sleep_remaining = ticks.max(1);
            return StepResult::Ok;
        }
        // Host-pool free interception (WiFi fixture support — same as the
        // `run_fast_core` hook below; `step` is the precise single-step path
        // the probes use, so it needs the identical skip). The call site is
        // `call8 _ZdlPvj` (callee arg a2 = caller a10 by the windowed ABI:
        // CALL8 rotates wb by 2, so callee-a2 aliases caller-a10 — read the
        // CALLER's a10 BEFORE stepping, while wb still names it). The call
        // op's own pc is per-image (`Soc::wifi_delete_call_site` — the
        // Arduino NetworkEvents code links elsewhere per sketch).
        for c in 0..2 {
            if self.cpu[c].pc == self.soc.wifi_delete_call_site()
                && self.soc.wifi_ard_free(self.cpu[c].reg(10))
            {
                self.cpu[c].pc = self.cpu[c].pc.wrapping_add(3);
                // Advance past the call WITHOUT executing it: the callee
                // frame was never entered, so no return address was pushed
                // and no window rotation happened — execution continues at
                // the caller's next instruction with a0/ra intact.
                // (The intercepted free is a leak-by-design no-op; the
                // firmware's delete-path return value is unused.)
                return StepResult::Ok;
            }
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
        self.sync_dedic_out();
        r
    }

    /// Mirror both CPUs' dedicated-GPIO output latches (`tie_gpio`, written
    /// by the `ee.*gpio_out`/`wur.gpio_out` TIE instructions) into the SoC
    /// so the CORE1_GPIO_OUT matrix signals drive pads every step.
    fn sync_dedic_out(&mut self) {
        for c in 0..2 {
            let bits = self.cpu[c].tie_gpio;
            self.soc.set_dedic_out(c, bits);
        }
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
        if self.boot_denied {
            return (StepResult::Ok, StepResult::Ok, 0);
        }
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
                // Peripherals (notably the ULP coprocessor and RTC clock)
                // keep running in deep sleep; tick before checking events.
                self.soc.tick_timers(1);
                self.sleep_tick();
                return (StepResult::Ok, StepResult::Ok, 0);
            }
            if let Some((ticks, deep)) = self.soc.consume_sleep_request() {
                self.asleep = true;
                self.sleep_light = !deep;
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
            self.sync_dedic_out();
            return (r0, r1, 2);
        };
        // Deep-sleep plumbing mirrors `step` (per macro-step; entry mid-block
        // takes effect here, ≤16 instructions late).
        if self.asleep {
            // Peripherals keep running in deep sleep (see above).
            self.soc.tick_timers(1);
            self.sleep_tick();
            return (StepResult::Ok, StepResult::Ok, 0);
        }
        if let Some((ticks, deep)) = self.soc.consume_sleep_request() {
            self.asleep = true;
            self.sleep_light = !deep;
            self.sleep_remaining = ticks.max(1);
            return (StepResult::Ok, StepResult::Ok, 0);
        }
        let (r0, n0) = self.run_fast_core(0, llen);
        let (r1, n1) = self.run_fast_core(1, flen);
        // Self-contained Wi-Fi fixture engine (browser/bridge path): drive
        // one step with post-step pcs (same sampling point run_flash uses
        // after its `step_fast`). No-op while no fixture is armed. Handles
        // the records-check a10 = ESP_OK force itself (the Soc cannot see
        // CPU regs): when the engine staged records this poll, force the
        // trapping core's return value.
        let pc0 = self.cpu[0].pc;
        let pc1 = self.cpu[1].pc;
        let records_pc = self.soc.wifi_fixture_records_pc();
        let staged_before = self.soc.wifi_fixture_records_staged();
        // The engine's ESP-NOW legs need the UART `sent 1` marker, which
        // lives in host-side console state the SoC cannot see: pass a
        // cheap snapshot (the merged UART0+USB stream, same bytes
        // run_flash greps). Snapshot BEFORE the drain below so the poll
        // sees the same bytes the host just observed.
        let uart_snap = self.soc.console_snapshot();
        self.soc.wifi_fixture_poll(pc0, pc1, &uart_snap);
        if self.soc.wifi_fixture_records_staged() && !staged_before {
            for c in 0..2 {
                if (c == 0 && pc0 == records_pc) || (c == 1 && pc1 == records_pc) {
                    self.cpu[c].set_reg(10, 0); // a10 = ESP_OK
                }
            }
        }
        // Run any ESP-NOW callback the poll staged, in-firmware on core 1
        // (parks in IDLE — always safe; same core run_flash uses).
        while self.soc.wifi_espnow_call_pending() {
            self.run_espnow_callback(1);
        }
        if self.soc.rom_boot_mode()
            && !(self.cpu[0].pc >= rom_stub::ROM_BASE && self.cpu[0].pc < rom_stub::ROM_END)
        {
            // The ROM stub's flash reads (via the data window) must bypass the
            // MMU; once core 0 jumps into the app, the app's window reads go
            // through the MMU again (see `Soc::rom_boot_mode`).
            self.soc.set_rom_boot_mode(false);
        }
        self.sync_dedic_out();
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
            // Fixture-engine pre-op sample (browser/bridge path): the
            // engine's post-step poll (after both blocks, same point
            // run_flash uses) observes a transient CALLEE-ENTRY pc only
            // when a macro-step happens to END exactly on it; the per-op
            // sample here observes EVERY pc the core executes, so arming
            // here is airtight where post-step is luck. The post-step
            // poll stays for steady-state legs (scan_start spins,
            // records_check traps).
            self.soc.wifi_fixture_poll_pre(pc0);
            let _ = pc0;
            // Host-pool free interception (WiFi fixture support — same as
            // the `step` hook above): skip the `_ZdlPvj` call for host-pool
            // pointers (leak-by-design no-op; caller arg a2 = caller a10).
            // Call-site pc is per-image (`Soc::wifi_delete_call_site`).
            // All other frees run unmodified.
            // GATE (proven live on worker 2026-09-28): the site pc alone
            // is NOT sufficient — union hook pcs collide with live callees
            // on relinked images (e.g. STA get_ip_info pcs land inside the
            // worker's `esp_netif_update_default_netif_lwip`). The
            // `wifi_ard_free` pointer check is the real discriminator
            // (host-pool range — firmware heap pointers never match); the
            // pc check only selects the call op. Both must hold: pc ==
            // this image's site AND ptr in pool. (The TX-tap arm below is
            // likewise gated on `wifi_image`, same discipline.)
            if pc0 == self.soc.wifi_delete_call_site()
                && self.soc.wifi_ard_free(self.cpu[core].reg(10))
            {
                self.cpu[core].pc = pc0.wrapping_add(3);
                n += 1;
                continue;
            }
            // WiFi insider hooks (fixture support — the closed
            // lwIP/net80211 stack has no live state). Hook TABLE (all
            // CALLEE entries; caller args are windowed — callee a2/a3 =
            // caller a10/a11 — read BEFORE `step_one` while wb still
            // names the caller; hooks fire only while staged, else the
            // call runs and fails soft like silicon):
            // - `esp_netif_get_ip_info` → staged fixture LAN
            // - `esp_wifi_sta_get_ap_info` → staged AP record
            // - `esp_wifi_get_config` (ifx==1) → staged AP config
            // - `esp_wifi_set_config` (ifx==1) → capture AP config
            // - `esp_wifi_ap_get_sta_list` → staged station count (0)
            // - `esp_wifi_disconnect` → ESP_OK + disconnect latch
            // (Single-step `step` never reaches here — run_flash drives
            // `step_fast` exclusively; the `step` hook above covers only
            // the host-pool free for probe use.)
            // NOTE: these pcs are CALLEE entries, so "skip" means fake-return
            // to the caller (NOT pc+3, which would land mid-callee): the
            // windowed CALL stored the return address in caller a8 (= future
            // callee a0 — same phys slot after the ENTRY rotation, which we
            // skip), encoded as (callinc<<30)|(addr & 0x3fffffff). Emulate
            // RETW's jump ((pc & 0xc0000000)|(a0 & 0x3fffffff)) using caller
            // a8 read BEFORE stepping (wb still names the caller), and stage
            // the ESP_OK return in caller a10 (= future callee a2, which the
            // caller reads as its return value).
            //
            // IMAGE CAVEAT: the linked addresses DIFFER per sketch (nm on
            // each sketch ELF — every image links the closed libs
            // elsewhere). The table below is the UNION over all six
            // hookable entry points (scan/STA/AP/ESP-NOW/worker images
            // plus the worker's post-static-IP relink); at most one entry
            // matches per image (addresses are unique per image — no
            // cross-image aliasing possible since a run boots exactly one
            // image). WARNING: a hook pc that is a REAL function entry on
            // one image but mid-function garbage on another MISFIRES
            // (proven live on worker: union pcs 0x4202e77c/98/04/a30/c8/58
            // all land inside `esp_netif_update_default_netif_lwip` /
            // `esp_netif_dhcps_option_api` / `esp_netif_get_mac` — a
            // mid-function fire with garbage regs would corrupt the real
            // call. The `wifi_hook_*` STAGED guard is what makes this
            // safe: the skip only fires while staged (post-GOT_IP); an
            // unstaged pc match falls through to the real function. The
            // worker's true `esp_netif_get_ip_info` is 0x4202e890).
            // GATED latitudes: ONLY true function entries may appear here.
            // Union pcs that are mid-function on ANY image were removed
            // 2026-09-28 (proven live: STA/scan/AP/espnow get_ip_info pcs
            // land inside the worker's `esp_netif_update_default_netif_lwip`
            // / `esp_netif_dhcps_option_api` / `esp_netif_get_mac` — firing
            // there would run the skip mid-function with garbage regs).
            // True entries (nm-verified per image): STA 0x4202e77c, scan
            // 0x4202e804, AP 0x4202e838, ESP-NOW 0x4202ea9c, worker
            // 0x4202e890.
            // PER-IMAGE GATE (proven live on worker 2026-09-28): the
            // union table above is UNSOUND across relinks (a stale pc
            // lands mid-function on another image and fires with garbage
            // regs — worker wedged at `esp_netif_update_default_netif_lwip
            // +0xa4` == scan 0x4202e804 with the hook clobbering the
            // caller's real out pointer). The skip therefore fires iff
            // (pc == this run's image true entry AND staged): the image
            // gate selects the entry, the staged guard selects the phase.
            // The staged check alone is insufficient (staged data exists
            // post-GOT_IP while unrelated code paths coincidentally hit
            // stale pcs); the pc check alone is insufficient (stale pcs
            // hit live code on relinked images).
            if pc0 == self.soc.wifi_hook_get_ip_info_pc() {
                // WINDOWED-ABI CONTRACT (proven live on worker 2026-09-28
                // via the EPC1=0x7fcb1834 ILLEGAL): the hook fires at the
                // CALLEE entry, where the window has NOT rotated yet (ENTRY
                // is the callee's first op and it has not executed).
                // Caller and callee therefore share one window here: the
                // caller's a2 (2nd arg slot) IS the callee's a2. The real
                // `esp_netif_get_ip_info(esp_netif, out)` is a 2-arg
                // call, so arg1 (out ptr) = caller-a3 = reg(11), NOT
                // reg(10) (which is the caller's a2 = the netif pointer,
                // 0x3fcb1834 -- writing 12 bytes there then EPC1-ing into
                // it is exactly the observed crash). The ap_info arm below
                // is a 1-arg call, so arg0 = caller-a2 = reg(10) -- both
                // arms read the CALLER's arg slot for `out`, which differs
                // by arity. RET-CONTRACT: the callers
                // (`localIP`/`subnetMask`/`gatewayIP`/`broadcastIP`) test
                // the RETURN VALUE (`bnez a10`, nonzero = error =>
                // default-construct 0.0.0.0). The skip only fires while
                // staged (post-GOT_IP); `wifi_hook_get_ip_info` returns
                // false unstaged and the real function runs (fails soft
                // like silicon, 0.0.0.0).
                let out = self.cpu[core].reg(11);
                let ra = self.cpu[core].reg(8);
                if self.soc.wifi_hook_get_ip_info(out) {
                    self.cpu[core].set_reg(10, 0);
                    self.cpu[core].pc = (pc0 & 0xc000_0000) | (ra & 0x3fff_ffff);
                    n += 1;
                    continue;
                }
            // True entries only (same mid-function rule as above): STA
            // 0x42064118, scan 0x42064130, AP 0x42064164, ESP-NOW
            // 0x4206a298, worker 0x420641bc.
            } else if pc0 == self.soc.wifi_hook_get_ap_info_pc() {
                let out = self.cpu[core].reg(10);
                let ra = self.cpu[core].reg(8);
                if self.soc.wifi_hook_get_ap_info(out) {
                    self.cpu[core].set_reg(10, 0);
                    self.cpu[core].pc = (pc0 & 0xc000_0000) | (ra & 0x3fff_ffff);
                    n += 1;
                    continue;
                }
            // True entries only (capture-only arm, but keep the union
            // exact anyway): STA 0x42063ec0, scan 0x42063ed8, AP
            // 0x42063f0c, ESP-NOW 0x42069f98, worker 0x42063f64.
            } else if pc0 == self.soc.wifi_hook_get_config_pc() {
                // STALE-MEMBER HAZARD (proven live on worker): union pcs
                // from older images can land mid-function on a relinked
                // image (e.g. STA 0x42063ec0 == worker
                // `esp_wifi_clear_ap_list+0x10`). This arm is
                // capture-only (no skip), so a misfire only mirrors
                // staged bytes on ifx==1 — harmless.

                // NOTE: get_config is a WRITE-ONLY side effect like
                // set_config (no fake-RETW skip): the closed driver's own
                // store update + canary live in the caller's frame, and
                // skipping corrupts it. The staged bytes now mirror the
                // caller's buffer, so the caller's own return path
                // observes them unmodified.
                let ifx = self.cpu[core].reg(10);
                let out = self.cpu[core].reg(11);
                self.soc.wifi_hook_ap_get_config(ifx, out);
            // True entries only: STA 0x42063e58, scan 0x42063e70, AP
            // 0x42063ea4, ESP-NOW 0x42069f30, worker 0x42063efc.
            } else if pc0 == self.soc.wifi_hook_set_config_pc() {
                // STALE-MEMBER HAZARD: same as get_config above (union
                // pcs can land mid-function on a relinked image) — safe
                // here because this arm is capture-only (no skip).
                // NOTE: set_config is DELIBERATELY never skipped (no
                // fake-RETW): the hook only CAPTURES the firmware's own
                // config into the staged store and lets the call run. A
                // skip here would bypass the closed driver's own store
                // update (the canary lives in the caller's frame, which
                // the skip's window surgery corrupts). Capture is
                // read-only w.r.t. CPU state.
                let ifx = self.cpu[core].reg(10);
                let src = self.cpu[core].reg(11);
                self.soc.wifi_hook_ap_set_config(ifx, src);
            // True entries only: STA 0x42063f04, scan 0x42063f1c, AP
            // 0x42063f50, ESP-NOW 0x42069fdc, worker 0x42063fa8.
            } else if pc0 == self.soc.wifi_hook_ap_sta_list_pc() {
                let out = self.cpu[core].reg(10);
                let ra = self.cpu[core].reg(8);
                if self.soc.wifi_hook_ap_sta_list(out) {
                    self.cpu[core].set_reg(10, 0);
                    self.cpu[core].pc = (pc0 & 0xc000_0000) | (ra & 0x3fff_ffff);
                    n += 1;
                    continue;
                }
            // True entries only: STA 0x4203c7d0, scan 0x4203c858, AP
            // 0x4203c88c, ESP-NOW 0x4203caf0, worker 0x4203c8e4.
            // WORKER EXEMPTION (proven live 2026-09-28): the worker's
            // closed `esp_wifi_connect` path calls `esp_wifi_disconnect`
            // internally on retry — skipping it there reports a success
            // the firmware never earned and posts a DISCONNECTED the
            // association never had, parking the run at `esp_wifi_connect`
            // forever. The worker sketch never calls `WiFi.disconnect()`
            // itself, so the hook simply never fires on this image.
            } else if pc0 == self.soc.wifi_hook_disconnect_pc()
                && self.soc.wifi_image != esp32s3_soc::WifiImage::Worker
                && self.soc.wifi_image != esp32s3_soc::WifiImage::WorkerL3
                && self.soc.wifi_hook_disconnect()
            {
                let ra = self.cpu[core].reg(8);
                self.cpu[core].set_reg(10, 0);
                self.cpu[core].pc = (pc0 & 0xc000_0000) | (ra & 0x3fff_ffff);
                self.soc.wifi_notify_disconnect();
                n += 1;
                continue;
            }
            // Ethernet TX capture (live-IP backhaul tap): the linked
            // `esp_netif_transmit(esp_netif, data, len)` callee is lwIP's
            // single egress point (DHCP/ARP/IP/ICMP/UDP/TCP all leave
            // here). Pcs are nm per sketch ELF — scan 0x4202e560, STA
            // 0x4202e4d8, AP 0x4202e594, ESP-NOW 0x4202e7f8,
            // test-worker-net 0x4202e5f0 — and the hook fires ONLY for
            // the pc matching this run's `wifi_image` (gated below):
            // the images link the closed libs at different addresses,
            // so a raw union would misfire (proven live 2026-09-27: on
            // the STA image the AP pc 0x4202e594 is
            // `esp_netif_dhcpc_start`, whose regs held zeros/garbage —
            // the "1600B zeros" pcap trial). The call RUNS unmodified
            // (capture is read-only w.r.t. CPU state — same class as the
            // set_config capture hook); the frame bytes land in
            // `pending_net_tx` + an EVT_NET_FRAME event for the host
            // (gateway bridge + pcap) to drain. Caller args are windowed
            // (callee a3/a4 = caller a11/a12), read BEFORE `step_one`
            // while wb still names the caller. LIVE since the worker
            // sketch drives the real stack (proven: 2 frames to pcap —
            // ARP 0x0806 + IPv4 0x0800 — via direct `esp_netif_transmit`
            // calls; the fixture sketches still never call it — their
            // closed DHCP/client stack has no live netif state).
            {
                use esp32s3_soc::WifiImage;
                let want = match self.soc.wifi_image {
                    WifiImage::Sta => 0x4202_e4d8,
                    WifiImage::Ap => 0x4202_e594,
                    WifiImage::Scan => 0x4202_e560,
                    WifiImage::EspNow => 0x4202_e7f8,
                    WifiImage::Worker => 0x4202_e60c,
                    WifiImage::WorkerL3 => 0x4202_f924,
                };
                if pc0 == want {
                    let data = self.cpu[core].reg(11);
                    let len = self.cpu[core].reg(12);
                    self.soc.net_capture_tx(data, len);
                }
            }
            // Ethernet RX injection (gateway→board replies: ARP/DHCP/
            // IPv6/gVisor returns staged by the host via `net_inject_rx`
            // from the NET_GW TCP leg). Entry hook on the linked
            // `esp_netif_receive(esp_netif, buffer, len, eb)` callee
            // (worker image 0x4202e654; other images use their own
            // linked pcs, same wifi_image gate as the TX tap). When a
            // frame is staged, its bytes are copied into the firmware's
            // own `buffer` (caller a11 — the closed stack's pbuf-backed
            // receive buffer, so the frame flows into lwIP unmodified)
            // and the call is fake-returned with ESP_OK (caller a10 =
            // 0); the scratch copy in `WIFI_SCRATCH` + `net_rx_last_*`
            // stays observable for tests. When the FIFO is empty the
            // call runs unmodified (silicon with no packet waiting —
            // the closed stack drops it). Caller args windowed, read
            // BEFORE `step_one` like the TX tap.
            {
                use esp32s3_soc::WifiImage;
                let want_rx = match self.soc.wifi_image {
                    WifiImage::Sta => 0x4202_e53c,
                    WifiImage::Ap => 0x4202_e5f8,
                    WifiImage::Scan => 0x4202_e5c4,
                    WifiImage::EspNow => 0x4202_e85c,
                    WifiImage::Worker => 0x4202_e670,
                    WifiImage::WorkerL3 => 0x4202_f988,
                };
                if pc0 == want_rx
                    && let Some(frame) = self.soc.net_take_rx()
                {
                    use xtensa_core::Bus as _Bus;
                    let buf = self.cpu[core].reg(11);
                    let cap = self.cpu[core].reg(12) as usize;
                    let m = frame.len().min(cap).min(1600);
                    for (k, b) in frame.iter().take(m).enumerate() {
                        self.soc.write8(buf + k as u32, *b as u32);
                    }
                    // Mirror into scratch for test observability
                    // (same bytes, same layout as `net_rx_stage`).
                    self.soc.net_rx_stage(&frame[..m]);
                    let ra = self.cpu[core].reg(8);
                    self.cpu[core].set_reg(10, 0); // ESP_OK
                    self.cpu[core].pc = (pc0 & 0xc000_0000) | (ra & 0x3fff_ffff);
                    n += 1;
                    continue;
                }
            }
            // BLE VHCI TX capture (NimBLE host→controller path): the linked
            // `esp_vhci_host_send_packet(data, len)` callee (BLE image only,
            // 0x42025f90 — nm on the esp32s3_ble ELF; sibling images never
            // link libbt so no gate is needed) takes NimBLE's H4 HCI frame
            // (type 0x01 CMD / 0x02 ACL + payload). Caller args are
            // windowed (callee a2/a3 = caller a10/a11), read BEFORE
            // `step_one` while wb still names the caller — same discipline
            // as the TX-tap arm above. Capture is read-only w.r.t. CPU
            // state: the call RUNS unmodified (the closed
            // `API_vhci_host_send_packet` descends into function-pointer
            // tables that read 0 on the emulator and return an error the
            // NimBLE TX path tolerates — `ble_hci_trans_hs_cmd_tx` frees
            // the mbuf either way); the frame bytes land in `pending_tx`
            // + an EVT_BLE_HCI event for the host (Bumble bridge) to
            // drain via `bt_hci_take_tx`.
            //
            // SEMAPHORE NOTE (proven live: only ONE `BLE TX` line ever
            // fires): NimBLE's `ble_hci_trans_hs_cmd_tx` takes
            // `vhci_send_sem` (0x3fc9d6d0, BLE image only) before EVERY
            // send with a ~2s (`0x7d0`-tick) timeout, and NOTHING on the
            // emulator ever gives it — silicon's controller gives it via
            // `controller_rcv_pkt_ready` after each TX completes (see
            // esp_nimble_hci.c; `notify_host_send_available` wakes the
            // same semaphore). The Host → `host_rcv_pkt` reply path does
            // NOT unblock it (the Command Complete event only flows into
            // the NimBLE event queue, which the stalled sender task never
            // drains). So the tap gives the semaphore here, once per
            // captured packet: `vhci_send_sem` counts INITIALLY 1 (set by
            // `esp_nimble_hci_init`), each send takes 1, each completion
            // gives 1 back — one give per captured TX is exactly the
            // controller's contract. The give itself runs IN FIRMWARE
            // (same call8-frame synthesis as `run_ble_host_recv`, on the
            // CURRENT core, callee `controller_rcv_pkt_ready` at
            // 0x4200507c which gives iff the handle is nonzero).
            //
            // ROM-LOOPBACK NOTE (proven live 2026-09-30: the REAL ROM
            // binary at 0x4002dd10/0x4002ded8 implements a controller
            // loopback — it takes the host's TX command and synthesizes
            // the Command Complete into 0x3fcacfd6 itself, then calls
            // `host_rcv_pkt` with its OWN buffer — no host/bridge needed
            // for basic commands). The tap therefore captures READ-ONLY
            // (bridge observability) and runs NOTHING in firmware: no
            // sem give (`run_ble_send_ready`), no ack deliver, no pool
            // top-up. All three were proven harmful or redundant:
            // - the give pumped `vhci_send_sem` the ROM loopback already
            //   manages (double-completion → stale acks);
            // - `ble_ack_deliver_at` overwrote the live TX mbuf with a
            //   STALE bridge reply (previous command's CC → opcode
            //   mismatch → `HCI process ack returned 12`);
            // - `ble_pool_top_up` linked the checked-out mbuf as free
            //   while live (double-ownership vs the ROM loopback's own
            //   alloc/free pairing → `assert failed: 0x42014482`).
            // The helpers stay (unit-tested, wired for a future external-
            // controller mode) but the tap does not call them.
            if pc0 == 0x4202_5f90 {
                let data = self.cpu[core].reg(10);
                let len = self.cpu[core].reg(11);
                self.soc.bt_hci_capture_tx(data, len);
            }
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

    /// Run one staged host-context callback invocation IN FIRMWARE on
    /// `core` (host frontend — the machine owns the CPUs, the SoC only
    /// stages; see `Soc::wifi_espnow_take_call`). Saves the core's full
    /// windowed state, calls the closed driver's registered wrapper with
    /// the windowed-ABI args staged in the scratch window, runs it to
    /// `retw`, then restores everything except the callback's own memory
    /// writes (the sketch-visible `sent_ok`/`got_rx` flags + peer
    /// dispatch).
    ///
    /// Window discipline (ISA RM Ch.4, verified against exec.rs CALL8):
    /// the caller (host) synthesizes a call8 frame — rotate wb by 2 (the
    /// ENTRY will rotate back on return), stash the return pc in the
    /// caller's a8 slot (callee-a0 alias), stage args in caller
    /// a10/a11/a12 (callee-a2/a3/a4 aliases), jump to the entry. The
    /// callback's ENTRY rotates wb back; its `retw` rotates forward to
    /// the host frame and lands at the stashed return pc, where the host
    /// detects completion and restores the saved state.
    ///
    /// Shared by the ESP-NOW fixture legs AND the BLE VHCI recv path
    /// (see `run_ble_host_recv`: same call8-frame synthesis, 2-arg
    /// `host_rcv_pkt(data, len)` form).
    pub fn run_espnow_callback(&mut self, core: usize) -> bool {
        let Some((entry, a2, a3)) = self.soc.wifi_espnow_take_call() else {
            return false;
        };
        let a4 = self.soc.wifi_espnow_take_a4();
        let cpu = &mut self.cpu[core];
        let saved_pc = cpu.pc;
        let saved_wb = cpu.windowbase();
        let saved_a0 = cpu.reg(0);
        let saved_a1 = cpu.reg(1);
        let saved_a6 = cpu.reg(6);
        let saved_a7 = cpu.reg(7);
        let saved_a8 = cpu.reg(8);
        let saved_a10 = cpu.reg(10);
        let saved_a11 = cpu.reg(11);
        let saved_a12 = cpu.reg(12);
        let saved_a13 = cpu.reg(13);
        // Synthesize the call8 frame: wb+2 (ENTRY rotates back), return
        // address 0x4000_0000 (unmapped ROM hole — never executed, only
        // compared), args staged. a6/a7 also saved: the wrapper's
        // vtable dispatch (`l32i a8,[a6,8]` + `callx8`) runs in the
        // CURRENT window, so a live a6 would route the call through a
        // garbage vtable (proven live: handler ENTRY saw a3=2/a4=0).
        let wb = (saved_wb + 2) & 0xf;
        cpu.set_windowbase(wb);
        cpu.pc = entry;
        cpu.set_reg(8, 0x4000_0000);
        cpu.set_reg(10, a2);
        cpu.set_reg(11, a3);
        // Direct-vtable `onReceive` is a 4-arg method
        // (this, data, len, bcast): len + bcast ride in a4/a5 from the
        // staged frame; wrapper calls ignore a4/a5 (a2/a3 suffice).
        if let Some((len, bcast)) = a4 {
            cpu.set_reg(12, len);
            cpu.set_reg(13, bcast);
        } else {
            cpu.set_reg(12, 0);
        }
        // Run until the callback returns to the host return pc (bounded:
        // the wrapper is a few dozen instructions; a stuck callback
        // restores and reports false rather than hanging the harness).
        for _ in 0..10_000 {
            self.cpu[core].step_one(&mut self.soc);
            if self.cpu[core].pc == 0x4000_0000 {
                break;
            }
        }
        let ok = self.cpu[core].pc == 0x4000_0000;
        let cpu = &mut self.cpu[core];
        cpu.pc = saved_pc;
        cpu.set_reg(0, saved_a0);
        cpu.set_reg(1, saved_a1);
        cpu.set_reg(6, saved_a6);
        cpu.set_reg(7, saved_a7);
        cpu.set_reg(8, saved_a8);
        cpu.set_reg(10, saved_a10);
        cpu.set_reg(11, saved_a11);
        cpu.set_reg(12, saved_a12);
        cpu.set_reg(13, saved_a13);
        // Apply the restored wb immediately (no step boundary before the
        // harness resumes — a stale wb would misname every reg).
        cpu.set_windowbase(saved_wb);
        ok
    }

    /// Run the registered BLE VHCI `notify_host_recv` callback IN FIRMWARE
    /// on `core` with one staged controller→host HCI packet (host
    /// frontend — the Bumble bridge reply path; the packet bytes were
    /// staged by the host via `Soc::bt_hci_inject_rx`).
    ///
    /// Same call8-frame synthesis as `run_espnow_callback` (2-arg form):
    /// `host_rcv_pkt(data, len)` = NimBLE's `ble_transport_to_hs_evt_impl`
    /// path via `host_rcv_pkt` at 0x420050a0 (BLE image only — nm on the
    /// esp32s3_ble ELF; sibling images never link NimBLE so the hook
    /// address never matches). The packet bytes are copied into the host
    /// scratch window first (`Soc::bt_hci_stage_rx` at
    /// `WIFI_SCRATCH + 0x2500`, past the net-RX slots — never heap, never
    /// the TLSF pool: `host_rcv_pkt` only READS the bytes into an mbuf it
    /// allocates itself, same class as the `WIFI_SCRATCH` fixture
    /// window). `entry` is the firmware's `notify_host_recv` pointer
    /// (read once from the `vhci_host_cb` rodata by the host); while
    /// undiscovered the call is skipped and the packet stays queued
    /// (level, not edge — silicon with an unregistered callback drops
    /// it, but retrying is harmless and covers the
    /// register-then-reply race).
    ///
    /// PARKED (see the run_flash BLE leg note): driving the reply into
    /// the firmware via a synthetic `host_rcv_pkt` call walks the mbuf
    /// pool free path with a block the pool does not recognize (`assert
    /// failed: 0x42014482` in `os_memblock_from(pool_cmd)`,
    /// panic_abort EPC1=0x4037fdc4 — proven live via pb5: the plain
    /// `host_rcv_pkt` firmware path asserts with NO hook and NO bridge
    /// traffic). The reply must enter through the firmware's OWN VHCI
    /// poll (`host_rcv_pkt` at 0x420050a0, called by the controller glue
    /// with its own buffer at 0x3fcacfd6) — until the controller-glue
    /// poll is modeled (the RWBLE ISR path that hands the firmware its
    /// own buffer), this stays parked and UNCALLED (kept for the day the
    /// poll lands; the overflow-aware loop + full-phys save + INTENABLE
    /// mask below are all verified correct — the failure is the FOREIGN
    /// buffer, not the synthesis).
    ///
    /// Returns true when a packet was delivered (callback ran to `retw`);
    /// false when idle (no packet, or no callback yet).
    #[allow(dead_code)]
    pub fn run_ble_host_recv(&mut self, core: usize, entry: u32) -> bool {
        if !(0x4000_0000..0x4240_0000).contains(&entry) {
            // No callback yet — idle WITHOUT consuming the packet (level,
            // not edge: the FIFO still holds it, so the next step retries
            // once the callback is discovered). NOTE: no event is queued
            // or drained here — `bt_hci_inject_rx` already queued the RX
            // event at stage time.
            return false;
        }
        let Some((buf, len)) = self.soc.bt_hci_stage_rx() else {
            return false;
        };
        let cpu = &mut self.cpu[core];
        let saved_pc = cpu.pc;
        let saved_wb = cpu.windowbase();
        // Save the FULL physical window file (all 64 regs): the
        // synthetic call rotates wb+2 and runs thousands of firmware
        // instructions (allocators, queues, window spills), clobbering
        // physical registers across MULTIPLE windows. Saving only the
        // current view (proven live: a0-a15 of one window) misses the
        // caller's spilled window — the restore then resumes with a
        // clobbered a1 (stack pointer — observed 0xffffff05 post-hook
        // vs 0x3fcaec30 pre-hook), so the very next `entry` spills
        // through a wild sp and dies with EPC1 at the resumed pc
        // (0x420050ba). Cost: 64 u32 copies per delivered packet
        // (packets are rare). `phys_regs` exposes the raw file; the
        // restore writes it back verbatim (wb restored after, so the
        // view names the same window again).
        let saved_phys = *cpu.phys_regs();
        // Mask interrupts across the synthetic call (proven live: the
        // callee's `entry a1,32` takes a WindowOverflow vector, and a
        // pending level-1 line arriving MID-call vectors into the real
        // kernel queue with a HOST-saved wb — on return the restore
        // writes the wrong window (a3 garbage → EPC1+illegal inside
        // `host_rcv_pkt`, EPC1=0x420050ba). Silicon runs the VHCI
        // callback in interrupt context with the line already claimed;
        // INTENABLE=0 is the faithful equivalent, restored below.
        // (Same hazard class as `run_espnow_callback` — its wrappers are
        // leaf enough to usually survive, but the mask is correct for
        // both; the espnow path is left untouched per minimal-diff.)
        let saved_ie = cpu.sreg(xtensa_core::cpu::SR_INTENABLE);
        cpu.set_sreg(xtensa_core::cpu::SR_INTENABLE, 0);
        let wb = (saved_wb + 2) & 0xf;
        cpu.set_windowbase(wb);
        cpu.pc = entry;
        cpu.set_reg(8, 0x4000_0000);
        cpu.set_reg(10, buf);
        cpu.set_reg(11, len);
        for _ in 0..10_000 {
            // Window overflow/underflow (causes 32..=37) is NORMAL
            // control flow for a call this deep (proven live:
            // `ble_transport_alloc_evt → os_memblock_get` takes
            // WindowOverflow8 mid-call — the vector spills windows and
            // resumes the caller). The run only ends at the fake-RETW
            // pc; any OTHER non-Ok result aborts the call (state still
            // restored below). NOTE: the loop must NOT break on
            // overflow — breaking leaves pc at the vector
            // (0x40374080), `ok` reads false, and worse, the restore
            // then resumes the firmware INSIDE the overflow handler
            // state. `step_one` already vectored correctly; just keep
            // stepping (same discipline as `step_fast`'s block loop,
            // which only ends runs on pc deviation, not on overflow
            // exceptions).
            let r = self.cpu[core].step_one(&mut self.soc);
            if self.cpu[core].pc == 0x4000_0000 {
                break;
            }
            if !matches!(r, StepResult::Ok | StepResult::Exception { cause: 32..=37 }) {
                break;
            }
        }
        let ok = self.cpu[core].pc == 0x4000_0000;
        let cpu = &mut self.cpu[core];
        cpu.pc = saved_pc;
        *cpu.phys_regs_mut() = saved_phys;
        cpu.set_sreg(xtensa_core::cpu::SR_INTENABLE, saved_ie);
        cpu.set_windowbase(saved_wb);
        ok
    }

    /// Run the emulator-side equivalent of the controller's TX-done
    /// signal: `controller_rcv_pkt_ready` (0x4200507c, BLE image only —
    /// nm on the esp32s3_ble ELF) gives `vhci_send_sem` iff the handle
    /// is nonzero. Same 0-arg call8-frame synthesis as `run_ble_host_recv`
    /// (fake-RETW at 0x4000_0000); the callee's `beqz` skips the give
    /// when unregistered, so this is safe to call unconditionally from
    /// the TX-tap arm.
    ///
    /// PARKED with the rest of the firmware-synthesis BLE path (see the
    /// tap note): the ROM loopback already manages `vhci_send_sem`, and
    /// an extra give double-completes TX (stale acks → opcode mismatch).
    #[allow(dead_code)]
    fn run_ble_send_ready(&mut self, core: usize) {
        const READY: u32 = 0x4200_507c;
        const RETPC: u32 = 0x4000_0000;
        let cpu = &mut self.cpu[core];
        let saved_pc = cpu.pc;
        let saved_wb = cpu.windowbase();
        let saved_a0 = cpu.reg(0);
        let saved_a1 = cpu.reg(1);
        let saved_a8 = cpu.reg(8);
        let wb = (saved_wb + 2) & 0xf;
        cpu.set_windowbase(wb);
        cpu.pc = READY;
        cpu.set_reg(8, RETPC);
        for _ in 0..10_000 {
            self.cpu[core].step_one(&mut self.soc);
            if self.cpu[core].pc == RETPC {
                break;
            }
        }
        let cpu = &mut self.cpu[core];
        cpu.pc = saved_pc;
        cpu.set_reg(0, saved_a0);
        cpu.set_reg(1, saved_a1);
        cpu.set_reg(8, saved_a8);
        cpu.set_windowbase(saved_wb);
    }

    /// Re-run the boot sequence from the last loaded flash image.  Used when a
    /// peripheral (WDT) triggers a system reset.
    pub fn reset(&mut self) {
        self.cpu = [Cpu::new(0), Cpu::new(1)];
        // Silicon flash is non-volatile: preserve MEMSPI program/erase
        // writes (OTA updates, NVS) across the reboot by snapshotting the
        // live backing store (same length as the original image) instead of
        // rebooting from the pristine image, which would lose them.
        let n = self.flash.len();
        self.flash = self.soc.flash_image()[..n].to_vec();
        // eFuse is OTP (never wiped by reset): preserve it so encrypted
        // devices keep their key across reboots.
        let efuse = self.soc.efuse_snapshot();
        // PSRAM retains across CPU/system resets (only deep sleep powers
        // the octal-RAM array down — see wake(), which zeroes it back).
        let psram = self.soc.psram_snapshot();
        // RTC domain (slow/fast memory, ULP + touch state) survives CPU
        // resets on silicon — only a power-down loses it.
        let rtc = self.soc.snapshot_rtc();
        // Wi-Fi fixture DRAM-live state (staged records/IP/config, read
        // counters, disconnect latch, staged ESP-NOW calls, engine latch
        // bits) survives CPU/system resets like DRAM does — without it a
        // mid-run WDT reset wipes the STA association / ESP-NOW legs and
        // the post-reboot firmware hangs re-waiting (proven live:
        // wifi_sta lost GOT_IP/DONE across an interrupt-WDT reset).
        let wifi_fixture = self.soc.snapshot_wifi_fixture_runtime();
        let wifi_image = self.soc.wifi_image;
        let wifi_layout = self.soc.wifi_layout_cells();
        self.soc = Soc::new();
        self.soc.restore_efuse(efuse);
        self.soc.restore_psram(psram);
        self.soc.restore_rtc(rtc);
        self.soc.restore_wifi_fixture_runtime(wifi_fixture);
        self.soc.wifi_image = wifi_image;
        self.soc.wifi_layout_restore(wifi_layout);
        self.asleep = false;
        self.boot_denied = false;
        self.boot_deny_reason = None;
        self.sleep_remaining = 0;
        self.sleep_light = false;
        self.last_console_byte = None;
        self.last_console_was_usb = false;
        let f = self.flash.clone();
        self.boot_from_flash(&f);
    }

    /// True while the machine is fast-forwarding a deep-sleep period.
    pub fn is_asleep(&self) -> bool {
        self.asleep
    }

    /// True when the last `boot_from_flash` was refused for secure boot
    /// (eFuse SECURE_BOOT_EN set): both CPUs are parked with no output.
    pub fn secure_boot_rejected(&self) -> bool {
        self.boot_denied
    }

    /// Why the last boot was refused (`None` when allowed): "unsigned" (no
    /// signature sector found at the app image end) or "bad-signature"
    /// (sector present but CRC/digest/ECDSA check failed).
    pub fn secure_boot_deny_reason(&self) -> Option<&'static str> {
        self.boot_deny_reason
    }

    /// Secure-boot verdict for the app region of `flash` (the bytes the
    /// ROM would verify: from the OTA-selected slot to the end of flash).
    /// Unsigned images (no 0xE7 magic) report `Invalid`; the caller maps
    /// that to the "unsigned" deny reason.
    fn secure_boot_app_verdict(&self, flash: &[u8]) -> esp32s3_soc::secure_boot::Sbv2Verdict {
        use esp32s3_soc::secure_boot::{Sbv2Verdict, verify_image};
        // Mirror the slot selection boot_from_flash applies below (OTA
        // otadata or the factory offset), honoring flash encryption the
        // same way.
        let decrypted;
        let view: &[u8] = if self.soc.flash_enc_enabled() {
            decrypted = self.soc.flash_image_decrypted();
            &decrypted
        } else {
            flash
        };
        let app_off =
            crate::partition::select_ota_boot_offset(view).unwrap_or(rom_stub::APP_FLASH_OFFSET);
        let region = match view.get(app_off as usize..) {
            Some(r) => r,
            None => return Sbv2Verdict::Invalid,
        };
        verify_image(region)
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
        self.sleep_light = false;
        self.sleep_remaining = ticks.max(1);
    }

    /// Enter light-sleep for `ticks` steps (host-test helper mirroring
    /// `begin_sleep`); the CPU halts, then resumes in place with the
    /// stashed wakeup cause applied and no reboot.
    pub fn begin_light_sleep(&mut self, ticks: u64) {
        self.asleep = true;
        self.sleep_light = true;
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

    /// Advance one deep-sleep tick: a watched ULP halting mid-sleep or the
    /// captured period running out wakes (reboots with the stashed cause).
    fn sleep_tick(&mut self) {
        if self.soc.sleep_ulp_fired() || self.sleep_remaining == 0 {
            self.wake();
        } else {
            self.sleep_remaining -= 1;
        }
    }

    /// Reboot after a deep-sleep period, recording the evaluated wakeup
    /// cause (timer / EXT0 / EXT1 / touch / ULP) so
    /// `esp_sleep_get_wakeup_cause()` returns it, plus the EXT1 triggering
    /// pads. RTC slow/fast memory and ULP state are retained across the
    /// reboot, like silicon. A light sleep instead resumes in place (no
    /// reset: DRAM, CPU state and the reset reason are untouched) after
    /// applying the same wakeup cause.
    fn wake(&mut self) {
        let cause = self.soc.take_sleep_cause();
        let ext1 = self.soc.take_sleep_ext1();
        let cause = if cause == 0 {
            // Should not happen (consume always stashes something), but keep
            // the old timer default rather than reporting UNDEFINED.
            esp32s3_soc::rtc::CAUSE_TIMER
        } else {
            cause
        };
        if self.sleep_light {
            self.soc.set_sleep_wakeup_cause(cause);
            self.soc.set_sleep_wakeup_int();
            self.soc.set_ext1_status(ext1);
            self.asleep = false;
            self.sleep_remaining = 0;
            return;
        }
        let retain = self.soc.snapshot_rtc();
        let gpio_hold = self.soc.snapshot_gpio_hold();
        self.reset();
        self.soc.set_sleep_wakeup_cause(cause);
        self.soc.set_ext1_status(ext1);
        self.soc.restore_rtc(retain);
        // Digital-pad hold (DIG_PAD_HOLD): held pads keep driving across
        // the reboot like silicon; unheld pads reset with the digital core.
        self.soc.restore_gpio_hold(gpio_hold);
        // Deep sleep powers the PSRAM array down: contents are lost
        // (silicon does not retain), so wipe the snapshot-restored copy
        // back to erased zeros. Light sleep / WDT / software resets keep
        // PSRAM (handled in reset()).
        self.soc.psram_wipe();
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
        // Secure-boot enforcement: with SECURE_BOOT_EN burned, the mask-ROM
        // verifies the bootloader/app signature before loading anything.
        // The app region (from the OTA-selected slot to the end of flash)
        // is verified via `secure_boot::verify_image` — the offline port of
        // the ROM's espsecure check, through the ECDSA model. Unsigned or
        // bad-signature images are refused (parked CPUs, no output);
        // default eFuse (all zero) boots normally.
        if self.soc.secure_boot_enabled() {
            let verdict = self.secure_boot_app_verdict(flash);
            if verdict != esp32s3_soc::secure_boot::Sbv2Verdict::Valid {
                self.boot_denied = true;
                self.boot_deny_reason = Some(match verdict {
                    esp32s3_soc::secure_boot::Sbv2Verdict::Unsupported => "bad-signature",
                    _ => "unsigned",
                });
                return;
            }
            // Verified: fall through and boot the image.
        }
        self.boot_denied = false;
        self.boot_deny_reason = None;
        self.flash = flash.to_vec();
        self.soc.load_flash_image(0, flash);
        // Encrypted devices: parse/map from a decrypted view (the ROM
        // bootloader reads through decrypting HW; the stub loader + XIP do
        // the same at runtime via flash_byte/MEMSPI). The raw backing stays
        // ciphertext (persisted across resets like silicon).
        let decrypted;
        let view: &[u8] = if self.soc.flash_enc_enabled() {
            decrypted = self.soc.flash_image_decrypted();
            &decrypted
        } else {
            flash
        };
        // Pick the app slot: OTA images select ota_0/ota_1 via the otadata
        // partition; non-OTA images fall back to the factory slot at
        // APP_FLASH_OFFSET (0x10000).
        let app_off =
            crate::partition::select_ota_boot_offset(view).unwrap_or(rom_stub::APP_FLASH_OFFSET);
        // Pre-map the app's flash-mapped segments (.flash.text/.flash.rodata)
        // in the cache MMU — the real 2nd-stage bootloader maps them instead
        // of copying (the ROM stub's copy loop cannot write the read-only
        // windows).
        self.soc.map_app_flash_segments(view, app_off);
        // The stub's own flash reads must NOT go through that MMU (it reads
        // the image the way the real ROM reads flash — via SPI, MMU-free).
        self.soc.set_rom_boot_mode(true);
        let rom = rom_stub::rom_image();
        self.load_image(rom_stub::ROM_BASE, &rom);
        self.load_rom_data();
        // ROM 1st-stage flash detection: on silicon the mask-ROM boot runs
        // detect_spi_flash_chip (RDID) before the bootloader and stores the
        // physical size in the legacy chip struct's chip_size field
        // (0x3FCEF6A8, read as a5+4 by esp_flash_init_default_chip, which
        // then stores it unconditionally into default_chip->size at
        // 0x420069af).  Our stub loader skips the ROM 1st stage, so the
        // field keeps the snapshot blob's baked-in 0x200000 (2 MB) while the
        // emulated chip is the full image (esptool merge_bin pads to the
        // physical size, and the bootloader header byte 3 agrees).  The
        // stale 2 MB makes esp_ota_begin fail with 0x102
        // (ESP_ERR_INVALID_ARG): esp_partition_erase_range rejects ota_1
        // (0x150000+0x140000) as out of bounds.  Patch the field with the
        // image size, i.e. the state the real ROM boot leaves behind.
        self.soc.write32(0x3FCE_F6A8, flash.len() as u32); // legacy chip_size = physical flash size
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
            //
            // Scoped to ROM boot mode ONLY (the stub clears it once core 0
            // jumps into the app): the cross-call arm drops ANY UART byte
            // equal to the trailing USB byte, which corrupted app-stage
            // output — a sketch marker ("UART-SLEEP-ARMED") lost its first
            // byte to a coincidental boot-log collision, defeating marker
            // injection and hanging the sleep harness with zero output.
            // App firmware prints single-console; nothing to dedup there.
            let mut filtered_usb = Vec::new();
            let mut filtered_uart = Vec::new();
            // First handle same-call dedup: if both FIFOs have identical content,
            // the UART copy is a duplicate — keep USB only.
            if self.soc.rom_boot_mode() && !usb.is_empty() && out == usb {
                filtered_uart.clear();
                filtered_usb = usb;
            } else if self.soc.rom_boot_mode() {
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
            } else {
                filtered_uart = out;
                filtered_usb = usb;
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
