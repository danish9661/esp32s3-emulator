//! TEMP-HYBRID (S4 hybrid engine driver): runs real firmware with core0
//! alternating between transpiled slices (wasmtime static modules built by
//! static_func.py) and the interpreter; core1 is always interpreted.
//! Shared state syncs both ways at every slice boundary (AR + wb/ws/ps +
//! SAR/SCOMPARE1/EPC1 + full SRAM/IRAM0; ROM is read-only and never synced;
//! RTC/PSRAM/MMIO stay Soc-owned behind imports). The machine lives inside
//! the wasmtime Store so imports reach it directly (no raw pointers).
//! Usage: hybrid_run <module.wasm> <report.json> <flash.merged.bin> <max_insns>
//! (binary name stays transpile_run: the example target was not renamed).
//! Prints the UART stream plus `== hybrid end` with insns/wall/MIPS and the
//! transpiled/interp split. The S4 measurement vehicle (NOT the battery;
//! sketches stay on run_flash).
//!
//! Design limits (documented approximations, each validated by UART/state
//! match on hello; revisit with measurements, not guesses):
//! - Slice quantum: BUDGET_K dispatches (~25k est insns). Core1 advances the
//!   same est count interpreted per slice; timers tick est/2 (the model's
//!   1-per-2 ratio). Cross-core races inside a slice are possible in theory;
//!   slices are short and validation (UART identity) decides adequacy.
//! - Slices are skipped while a core0 IRQ is pending (the interpreter
//!   delivers it, possibly INTO transcribed code on the next slice).
//! - WDT reset is consumed (m.reset); sleep is NOT modeled here (hello never
//!   sleeps — sleep sketches stay on run_flash).
//! - Dedic-GPIO mirroring and wifi fixture hooks (private to machine.rs)
//!   are skipped on slice boundaries (no wifi/dedic traffic in hello boot).
//! - CCOUNT is never synced (covered code cannot read it: rsr_ccount traps
//!   to the interpreter by design, so delay loops always interpret).

use esp32s3_emu::Esp32S3;
use esp32s3_soc::memmap::{
    DIRAM_DATA_BASE, DRAM_BASE, DRAM_SIZE, IRAM_BASE, SRAM0_SIZE,
};
use std::collections::HashMap;
use std::time::Instant;
use wasmtime::{Caller, Engine, Extern, Instance, Linker, Module, Store, Val};
use xtensa_core::{
    Bus as _,
    StepResult,
    cpu::{SR_EPC1, SR_PS, SR_SAR, SR_SCOMPARE1, SR_WINDOW_START},
};

// Module-memory layout mirrors (must match windiff.py exactly).
const M_DRAM_OFF: usize = 0x10000;
const M_IRAM_OFF: usize = 0x90000;
const DIRAM_SIZE: usize = 0x68000;
const MB_REASON: usize = 0x310;
const MB_PC: usize = 0x318;
const MB_NEXT: usize = 0x31C;
const DISP_CTR: usize = 0x320;
const SHADOW_BASE: usize = 0x200000;
const SAR_P: usize = 0x300;
const SCOMP_P: usize = 0x304;
const EPC1_P: usize = 0x308;
const HALT: u32 = 0xFFFFFFFF;
// Estimated Xtensa insns per dispatch (slice accounting only).
const EST_PER_DISPATCH: u64 = 6;

struct Hybrid {
    m: Esp32S3,
    traps: Vec<u32>,
    trap_pc: u32,
    last_ctr: u64,
    uart: Vec<u8>,
    transpiled: u64,
    interp: u64,
    slices: u64,
}

fn rd32(mem: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([mem[off], mem[off + 1], mem[off + 2], mem[off + 3]])
}

fn set_global(store: &mut Store<Hybrid>, inst: &Instance, name: &str, v: u32) {
    let g = inst.get_global(&mut *store, name).expect("global export");
    g.set(&mut *store, Val::I32(v as i32)).expect("global set");
}

/// Sync machine -> module before a slice (fresh root: shadow reset + HALT).
fn sync_to_wasm(store: &mut Store<Hybrid>, inst: &Instance, seg: u32) {
    // Snapshot machine state into owned copies first (borrow discipline:
    // the Store borrow ends before the memory borrow begins).
    let (phys, wb, ws, ps, sar, sc, epc1, sram, iram0) = {
        let h = store.data_mut();
        let c = &h.m.cpu[0];
        (
            *c.phys_regs(),
            c.windowbase(),
            c.sreg(SR_WINDOW_START),
            c.sreg(SR_PS),
            c.sreg(SR_SAR),
            c.sreg(SR_SCOMPARE1),
            c.sreg(SR_EPC1),
            h.m.soc.sram_bytes().to_vec(),
            h.m.soc.iram0_bytes().to_vec(),
        )
    };
    let mem = inst.get_memory(&mut *store, "mem").expect("mem export");
    {
        let data = mem.data_mut(&mut *store);
        for (i, w) in phys.iter().enumerate() {
            data[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        data[SAR_P..SAR_P + 4].copy_from_slice(&sar.to_le_bytes());
        data[SCOMP_P..SCOMP_P + 4].copy_from_slice(&sc.to_le_bytes());
        data[EPC1_P..EPC1_P + 4].copy_from_slice(&epc1.to_le_bytes());
        data[M_DRAM_OFF..M_DRAM_OFF + sram.len()].copy_from_slice(&sram);
        data[M_IRAM_OFF..M_IRAM_OFF + iram0.len()].copy_from_slice(&iram0);
        let diram_off = (DIRAM_DATA_BASE - DRAM_BASE) as usize;
        let dst = M_IRAM_OFF + SRAM0_SIZE as usize;
        data[dst..dst + DIRAM_SIZE].copy_from_slice(&sram[diram_off..diram_off + DIRAM_SIZE]);
        // Fresh shadow root (fallback resumes never inherit a stale stack:
        // the interpreter may have returned past the transpiled caller).
        data[SHADOW_BASE..SHADOW_BASE + 4].copy_from_slice(&HALT.to_le_bytes());
    }
    set_global(store, inst, "wb", wb);
    set_global(store, inst, "ws", ws);
    set_global(store, inst, "ps", ps);
    // Shadow holds one pushed HALT (ssp=4 = bytes used, matching the
    // run-prologue ssp_push; ssp=0 means EMPTY and ssp_pop traps on it —
    // proven by the first hybrid slice dying in the retw guard).
    set_global(store, inst, "ssp", 4);
    set_global(store, inst, "start", seg);
}

/// Sync module -> machine after a slice. Returns (reason, aux) where aux is
/// MB_NEXT (reason 2), MB_PC (reason 0 fall-through), or trap pc (reason 1).
fn get_global_u32(store: &mut Store<Hybrid>, inst: &Instance, name: &str) -> u32 {
    match inst.get_global(&mut *store, name).expect("global export").get(&mut *store) {
        Val::I32(v) => v as u32,
        _ => panic!("global type"),
    }
}

fn sync_to_soc(store: &mut Store<Hybrid>, inst: &Instance) -> (u32, u32) {
    let (wb, ws, ps) = (
        get_global_u32(store, inst, "wb"),
        get_global_u32(store, inst, "ws"),
        get_global_u32(store, inst, "ps"),
    );
    let mem = inst.get_memory(&mut *store, "mem").expect("mem export");
    let data = mem.data(&*store);
    let mut phys = [0u32; 64];
    for (i, w) in phys.iter_mut().enumerate() {
        *w = rd32(data, i * 4);
    }
    let (sar, sc, epc1) = (rd32(data, SAR_P), rd32(data, SCOMP_P), rd32(data, EPC1_P));
    let (reason, aux) = (rd32(data, MB_REASON), rd32(data, MB_NEXT));
    let mb_pc = rd32(data, MB_PC);
    let sram = data[M_DRAM_OFF..M_DRAM_OFF + DRAM_SIZE as usize].to_vec();
    let iram0 = data[M_IRAM_OFF..M_IRAM_OFF + SRAM0_SIZE as usize].to_vec();
    let diram = data[M_IRAM_OFF + SRAM0_SIZE as usize..M_IRAM_OFF + SRAM0_SIZE as usize + DIRAM_SIZE]
        .to_vec();
    let (aux_pc, trap_pc) = (if reason == 0 { mb_pc } else { aux }, store.data().trap_pc);
    {
        let h = store.data_mut();
        *h.m.cpu[0].phys_regs_mut() = phys;
        h.m.cpu[0].set_windowbase(wb);
        h.m.cpu[0].set_sreg(SR_WINDOW_START, ws);
        h.m.cpu[0].set_sreg(SR_PS, ps);
        h.m.cpu[0].set_sreg(SR_SAR, sar);
        h.m.cpu[0].set_sreg(SR_SCOMPARE1, sc);
        h.m.cpu[0].set_sreg(SR_EPC1, epc1);
        h.m.soc.sram_bytes_mut().copy_from_slice(&sram);
        h.m.soc.iram0_bytes_mut().copy_from_slice(&iram0);
        let diram_off = (DIRAM_DATA_BASE - DRAM_BASE) as usize;
        h.m.soc.sram_bytes_mut()[diram_off..diram_off + DIRAM_SIZE].copy_from_slice(&diram);
    }
    if reason == 1 {
        (reason, trap_pc)
    } else {
        (reason, aux_pc)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mod_bytes = std::fs::read(&args[1]).expect("read wasm module");
    let rep: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&args[2]).expect("read report")).expect("parse report");
    let flash = std::fs::read(&args[3]).expect("read flash image");
    let max_insns: u64 = args[4].parse().expect("max_insns");
    let mut segmap = HashMap::<u32, u32>::new();
    for (k, v) in rep["segmap"].as_object().expect("segmap").iter() {
        segmap.insert(k.parse::<u32>().expect("pc"), v.as_u64().expect("seg") as u32);
    }
    let segpcs: Vec<u32> = rep["segpcs"]
        .as_array()
        .expect("segpcs")
        .iter()
        .map(|v| v.as_u64().expect("pc") as u32)
        .collect();
    // IRAM BASE sanity (layout contract with windiff.py).
    assert_eq!(IRAM_BASE, 0x4037_0000);
    assert_eq!(DRAM_BASE, 0x3FC8_0000);
    assert_eq!(DRAM_SIZE, 0x80000);

    let engine = Engine::default();
    let module = Module::new(&engine, &mod_bytes).expect("compile module");
    let mut store = Store::new(
        &engine,
        Hybrid {
            m: Esp32S3::new(),
            traps: Vec::new(),
            trap_pc: 0,
            last_ctr: 0,
            uart: Vec::new(),
            transpiled: 0,
            interp: 0,
            slices: 0,
        },
    );
    store.data_mut().m.boot_from_flash(&flash);
    let mut linker = Linker::new(&engine);
    linker
        .func_wrap("env", "trap", |mut caller: Caller<'_, Hybrid>, pc: u32| {
            caller.data_mut().traps.push(pc);
            caller.data_mut().trap_pc = pc;
            // Mailbox for the post-slice protocol (reason 1 = interp from pc).
            if let Some(Extern::Memory(mem)) = caller.get_export("mem") {
                let mut b = [0u8; 8];
                b[0..4].copy_from_slice(&1u32.to_le_bytes());
                b[4..8].copy_from_slice(&pc.to_le_bytes());
                let _ = mem.write(&mut caller, MB_REASON, &b);
            }
        })
        .expect("trap import");
    linker
        .func_wrap("env", "soc_read32", |mut caller: Caller<'_, Hybrid>, addr: u32| -> u32 {
            caller.data_mut().m.soc.read32(addr)
        })
        .expect("read import");
    linker
        .func_wrap("env", "soc_write32", |mut caller: Caller<'_, Hybrid>, addr: u32, val: u32| {
            caller.data_mut().m.soc.write32(addr, val);
        })
        .expect("write import");
    linker
        .func_wrap(
            "env",
            "soc_cas32",
            |mut caller: Caller<'_, Hybrid>, addr: u32, compare: u32, val: u32| -> u32 {
                caller.data_mut().m.soc.cas32(addr, compare, val)
            },
        )
        .expect("cas import");
    let instance = linker.instantiate(&mut store, &module).expect("instantiate");
    let resume = instance
        .get_typed_func::<(), ()>(&mut store, "resume")
        .expect("resume export");

    let t0 = Instant::now();
    let mut total: u64 = 0;
    let mut boot_ok = false;
    // Sleep is out of scope for hybrid (hello never sleeps): on the first
    // sleep request, fall back to pure step_fast permanently (it owns the
    // sleep/wake machinery; eating the request here would hang the guest).
    let mut slow_path = false;
    loop {
        if total >= max_insns {
            break;
        }
        if store.data_mut().m.soc.consume_reset() {
            store.data_mut().m.reset();
            continue;
        }
        if store.data_mut().m.soc.consume_sleep_request().is_some() {
            println!("!! hybrid: sleep requested (out of scope) — pure interp from here");
            slow_path = true;
        }
        let pc = store.data_mut().m.cpu[0].pc;
        let irq_pending = store.data_mut().m.soc.int_pending(0) != 0;
        if !slow_path && !irq_pending {
            if let Some(&seg) = segmap.get(&pc) {
                // ---- Transpiled slice (core0); core1 + timers catch up. ----
                if std::env::var("HYBRID_STATETRACE").is_ok() {
                    let h = store.data();
                    eprintln!(
                        "HYBRID slice at {:#x}: wb={} ws={:#x} ps={:#x} a0={:#x} a1={:#x} a2={:#x}",
                        pc,
                        h.m.cpu[0].windowbase(),
                        h.m.cpu[0].sreg(SR_WINDOW_START),
                        h.m.cpu[0].sreg(SR_PS),
                        h.m.cpu[0].reg(0),
                        h.m.cpu[0].reg(1),
                        h.m.cpu[0].reg(2),
                    );
                }
                sync_to_wasm(&mut store, &instance, seg);
                if let Err(e) = resume.call(&mut store, ()) {
                    println!("\n!! resume trap: {e:?} at pc {pc:#x}");
                    break;
                }
                let mem = instance.get_memory(&mut store, "mem").expect("mem");
                let data = mem.data(&store);
                let disp = rd32(data, DISP_CTR) as u64 - store.data().last_ctr;
                store.data_mut().last_ctr = rd32(data, DISP_CTR) as u64;
                let est = disp * EST_PER_DISPATCH;
                store.data_mut().m.soc.tick_timers(est / 2);
                // Core1 advances the same window interpreted.
                {
                    let h = store.data_mut();
                    let (cpu, soc) = (&mut h.m.cpu, &mut h.m.soc);
                    for _ in 0..est {
                        let _ = cpu[1].step(soc);
                    }
                }
                let (reason, aux) = sync_to_soc(&mut store, &instance);
                {
                    let h = store.data_mut();
                    h.uart.extend(h.m.take_uart_tx(0));
                    h.uart.extend(h.m.take_uart_tx(1));
                    h.transpiled += est;
                    h.interp += est;
                    h.slices += 1;
                    total += 2 * est;
                }
                match reason {
                    1 => {
                        // Trap: interpret from the trapped pc.
                        store.data_mut().m.cpu[0].pc = aux;
                    }
                    2 => {
                        // Budget: continue at the seg's entry pc.
                        let idx = aux as usize;
                        if idx >= segpcs.len() {
                            println!("\n!! MB_NEXT {idx} out of range");
                            break;
                        }
                        store.data_mut().m.cpu[0].pc = segpcs[idx];
                    }
                    _ => {
                        // Done: fall-through MB_PC or retw RA (full a0).
                        if aux == HALT {
                            let ra = store.data_mut().m.cpu[0].reg(0);
                            store.data_mut().m.cpu[0].pc = ra;
                        } else {
                            store.data_mut().m.cpu[0].pc = aux;
                        }
                    }
                }
                if store.data().uart.windows(7).any(|w| w == b"boot OK") {
                    boot_ok = true;
                    break;
                }
                continue;
            }
        }
        // ---- Interpreted phase (both cores, hooks, sleep, exceptions). ----
        let (r0, _r1, n) = store.data_mut().m.step_fast();
        store.data_mut().interp += n as u64;
        total += n as u64;
        let pc0 = store.data().m.cpu[0].pc;
        if let StepResult::Exception { cause } = r0 {
            if !(32..=37).contains(&cause) && cause != 5 {
                println!("\n!! core0 exception cause={cause} at pc {pc0:#010x}");
                break;
            }
        }
        {
            let h = store.data_mut();
            h.uart.extend(h.m.take_uart_tx(0));
            h.uart.extend(h.m.take_uart_tx(1));
            if h.uart.windows(7).any(|w| w == b"boot OK") {
                boot_ok = true;
                break;
            }
        }
    }
    let wall = t0.elapsed().as_secs_f64();
    let h = store.data();
    println!("== hybrid uart bytes ({}): {:?}", h.uart.len(), String::from_utf8_lossy(&h.uart));
    println!(
        "== hybrid end: {total} insns in {wall:.2}s ({:.1} MIPS) [transpiled {} + interp {}] slices={} traps={} boot_ok={boot_ok}",
        total as f64 / wall / 1e6,
        h.transpiled,
        h.interp,
        h.slices,
        h.traps.len(),
    );
}
