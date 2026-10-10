//! TEMP-S4BRIDGE (S4 wasmtime + live-Soc integration proof).
//! Runs a transpiled module (see tools/transpile_spike/s4bridge.py) with
//! MMIO imports served by a REAL Soc: direct RAM ops + UART0 FIFO write
//! through the import boundary land observably in Soc state.
//! Usage: transpile_run <module.wasm>

use esp32s3_soc::Soc;
use wasmtime::{Caller, Engine, Linker, Module, Store};
use xtensa_core::Bus;

struct Host {
    soc: Soc,
    traps: Vec<u32>,
    polls: u32,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let bytes = std::fs::read(&args[1]).expect("read wasm module");
    let engine = Engine::default();
    let module = Module::new(&engine, &bytes).expect("compile module");
    let mut store = Store::new(
        &engine,
        Host {
            soc: Soc::new(),
            traps: Vec::new(),
            polls: 0,
        },
    );
    {
        let mut linker = Linker::new(&engine);
        linker
            .func_wrap(
                "env",
                "trap",
                |mut caller: Caller<'_, Host>, pc: u32| {
                    caller.data_mut().traps.push(pc);
                },
            )
            .expect("trap import");
        linker
            .func_wrap(
                "env",
                "soc_read32",
                |mut caller: Caller<'_, Host>, addr: u32| -> u32 {
                    caller.data_mut().soc.read32(addr)
                },
            )
            .expect("read import");
        linker
            .func_wrap(
                "env",
                "soc_write32",
                |mut caller: Caller<'_, Host>, addr: u32, val: u32| {
                    caller.data_mut().soc.write32(addr, val);
                },
            )
            .expect("write import");
        linker
            .func_wrap("env", "poll_irq", |mut caller: Caller<'_, Host>| -> u32 {
                caller.data_mut().polls += 1;
                0
            })
            .expect("poll import");
        let instance = linker
            .instantiate(&mut store, &module)
            .expect("instantiate");
        let run = instance
            .get_typed_func::<(), i64>(&mut store, "run")
            .expect("run export");
        let got = run.call(&mut store, ()).expect("call run");
        // Module-memory reads scoped: the view borrows `store`.
        let (w8, w12) = {
            let mem = instance.get_memory(&mut store, "mem").expect("mem export");
            let data = mem.data(&store);
            let w = |o: usize| {
                u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]])
            };
            (w(8), w(12))
        };
        assert_eq!(w8, 0x48, "ram direct store");
        let oracle = store.data_mut().soc.read32(0x6000_001C);
        assert_eq!(w12, oracle, "mmio read == live oracle");
        assert_eq!(got, ((oracle as i64) << 32) | 0x48, "return pack");
        assert!(store.data().traps.is_empty(), "no traps");
        let uart = store.data_mut().soc.take_uart_tx(0);
        assert_eq!(uart, vec![0x48u8], "uart fifo via import");
        println!("S4BRIDGE PASS run={} status={:#x} uart={:02x?}", got, oracle, uart);
    }
}
