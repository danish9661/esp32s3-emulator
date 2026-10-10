//! TEMP-DIFF (S4c-1 differential transpiler oracle; DELETE after).
//! Boots a flash image, single-steps the machine, and dumps a JSON trace
//! of core0: per-step (pc, raw, opcode, operands), initial/final visible
//! regs + windowbase, and touched DRAM words (before/after).
//! Usage: diff_dump <image> <steps> [skip]

use esp32s3_emu::Esp32S3;
use xtensa_core::generated::{decode_inst, decode_inst16a, decode_inst16b, insn_len, opnds};
use xtensa_core::Bus;

fn esc(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).cloned().unwrap_or_default();
    // TEMP-SFUNC (S4 static pipeline probe): `diff_dump <image> static
    // <entry-hex> <nbytes>` boots the image (segments mapped) and decodes
    // linearly from entry without executing. Stops at first retw/ret/j
    // (leaf boundary) or nbytes. Same per-insn JSON shape as the trace.
    if args.get(2).map(|s| s.as_str()) == Some("static") {
        let entry = args
            .get(3)
            .and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok())
            .unwrap_or(0x4000_0000);
        let nbytes: u32 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(256);
        let flash = std::fs::read(&path).expect("read flash image");
        let mut m = Esp32S3::new();
        m.boot_from_flash(&flash);
        // App DRAM/IRAM segments land via the ROM stub's copy loop at
        // runtime (fresh-boot RAM reads zero, proven by 0x403769ac). Step
        // until the loader jumps out of the stub (app entry) with a cap;
        // post-loader RAM == image segment bytes (verbatim copy), so this
        // matches what a load-time image-segment reader would decode.
        // (Production S4 reads image segments directly; this probe steps.)
        for _ in 0..20_000_000 {
            if m.cpu[0].pc >= 0x4030_0000 {
                break;
            }
            let _ = m.step();
        }
        println!("{{\"sfunc\": {{\"entry\":{}, \"insns\": [", entry);
        let mut pc = entry;
        let end = entry.wrapping_add(nbytes);
        let mut first = true;
        while pc < end {
            let raw_full = m.soc.read32(pc);
            let b0 = (raw_full & 0xFF) as u8;
            let len = insn_len(b0);
            let raw = if len == 2 { raw_full & 0xFFFF } else { raw_full };
            let opc = match len {
                2 if b0 & 0xf <= 11 => decode_inst16a(raw),
                2 => decode_inst16b(raw),
                _ => decode_inst(raw),
            };
            let (name, ops) = match opc {
                Some(o) => {
                    let os = opnds(o, raw, pc);
                    let s: Vec<String> = os
                        .iter()
                        .map(|x| {
                            format!(
                                "{{\"v\":{},\"r\":{},\"vis\":{}}}",
                                x.value, x.is_reg as u8, x.visible as u8
                            )
                        })
                        .collect();
                    (o.name().to_string(), s.join(","))
                }
                None => ("<ill>".to_string(), String::new()),
            };
            print!(
                "{}{{\"pc\":{},\"len\":{},\"raw\":{},\"opc\":\"{}\",\"opnds\":[{}]}}",
                if first { "" } else { "," },
                pc,
                len,
                raw,
                esc(&name),
                ops
            );
            first = false;
            println!();
            if matches!(name.as_str(), "retw" | "retw_n" | "ret" | "j") {
                break;
            }
            pc = pc.wrapping_add(len as u32);
        }
        println!("]}}}}");
        return;
    }
    let steps: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(60);
    let skip_arg = args.get(3).cloned().unwrap_or_default();
    let flash = std::fs::read(&path).expect("read flash image");
    let watch: u32 = args
        .get(4)
        .and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or(0);
    let mut m = Esp32S3::new();
    m.boot_from_flash(&flash);
    // Skip phase: count, func:<entry> (scan to entry, S4 static pairing),
    // or watch-break. TEMP-DIFF scaffolding.
    let func_entry: Option<u32> = skip_arg
        .strip_prefix("func:")
        .and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok());
    let skip: usize = if func_entry.is_none() {
        skip_arg.parse().unwrap_or(0)
    } else {
        0
    };
    if let Some(entry) = func_entry {
        // Scan (cap 200M) until core0 pc == entry; the trace below then
        // captures that dynamic execution for static-vs-dynamic pairing.
        // Loud fail if never reached (wrong entry for this image).
        let mut found = false;
        for _ in 0..200_000_000 {
            if m.cpu[0].pc == entry {
                found = true;
                break;
            }
            let _ = m.step();
        }
        assert!(found, "TEMP-DIFF: func entry {:#x} never reached", entry);
    }
    // Dynamic function discovery: histogram of executed ENTRY pcs.
    use std::collections::HashMap;
    let mut funcs: HashMap<u32, usize> = HashMap::new();
    for _ in 0..skip {
        let pc = m.cpu[0].pc;
        // Peek the opcode cheaply: ENTRY = RRR w/ op2 field 14? Decode fully.
        let raw_full = m.soc.read32(pc);
        let b0 = (raw_full & 0xFF) as u8;
        let len = insn_len(b0);
        let raw = if len == 2 { raw_full & 0xFFFF } else { raw_full };
        let is_entry = match len {
            2 if b0 & 0xf <= 11 => decode_inst16a(raw),
            2 => decode_inst16b(raw),
            _ => decode_inst(raw),
        }
        .map(|o| o.name() == "entry")
        .unwrap_or(false);
        if is_entry {
            *funcs.entry(pc).or_insert(0) += 1;
        }
        if watch != 0 && pc == watch {
            break;
        }
        let _ = m.step();
    }
    // PSRAM physical snapshot (the s8i-to-flash-window question: PSRAM-mapped
    // writes land here, flash-mapped are dropped — the diff tells them apart).
    let psram_before = m.soc.psram_snapshot();
    // Snapshot DRAM before (wordwise over the 512KB range).
    const DRAM: u32 = 0x3FC8_0000;
    const WORDS: usize = 0x80000 / 4;
    let mut before = Vec::with_capacity(WORDS);
    for i in 0..WORDS {
        before.push(m.soc.read32(DRAM + i as u32 * 4));
    }
    let mut regs0 = [0u32; 16];
    for r in 0..16 {
        regs0[r as usize] = m.cpu[0].reg(r);
    }
    let wb0 = m.cpu[0].windowbase();
    // Entry PS/WINDOWSTART for static ENTRY (MUST be pre-trace: post-trace
    // reads reflect span-end windows, proven by the 0x15-vs-0x55 incident).
    let ps0 = m.cpu[0].sreg(230);
    let ws0 = m.cpu[0].sreg(73);
    // Full phys file BEFORE the span (execution reads outside the start
    // window: saved ras/spills. Cloned pre-loop: the stepping loop needs
    // &mut m, so the borrow cannot be held).
    let phys0 = *m.cpu[0].phys_regs();
    // (Trace records print inline; oracle reads capture into reads_log.)
    // Oracle reads captured INLINE right after each step (exact even when
    // a later step overwrites the addr — post-hoc re-reads are unsound).
    let mut reads_log: Vec<(u32, u32, u32)> = Vec::new();
    println!("{{\"steps\": [");
    for i in 0..steps {
        let pc = m.cpu[0].pc;
        let raw_full = m.soc.read32(pc);
        let b0 = (raw_full & 0xFF) as u8;
        let len = insn_len(b0);
        let raw = if len == 2 { raw_full & 0xFFFF } else { raw_full };
        let opc = match len {
            2 if b0 & 0xf <= 11 => decode_inst16a(raw),
            2 => decode_inst16b(raw),
            _ => decode_inst(raw),
        };
        let (name, ops, opv) = match opc {
            Some(o) => {
                let os = opnds(o, raw, pc);
                let s: Vec<String> = os
                    .iter()
                    .map(|x| {
                        format!(
                            "{{\"v\":{},\"r\":{},\"vis\":{}}}",
                            x.value, x.is_reg as u8, x.visible as u8
                        )
                    })
                    .collect();
                let v: Vec<(u32, bool, bool)> =
                    os.iter().map(|x| (x.value, x.is_reg, x.visible)).collect();
                (o.name().to_string(), s.join(","), v)
            }
            None => ("<ill>".to_string(), String::new(), Vec::new()),
        };
        let mut rr = [0u32; 16];
        for r in 0..16 {
            rr[r as usize] = m.cpu[0].reg(r);
        }
        // PS + WINDOWSTART per step (windowed spans: ENTRY reads CALLINC,
        // RETW checks WOE/ws, RSIL/WSR mutate PS).
        let ps = m.cpu[0].sreg(230);
        let ws = m.cpu[0].sreg(73);
        let wb = m.cpu[0].windowbase();
        print!(
            "{}{{\"pc\":{},\"len\":{},\"raw\":{},\"opc\":\"{}\",\"opnds\":[{}],\"regs\":{:?},\"ps\":{},\"ws\":{},\"wb\":{}}}",
            if i == 0 { "" } else { "," },
            pc,
            len,
            raw,
            esc(&name),
            ops,
            rr,
            ps,
            ws,
            wb
        );
        println!();
        // Inline oracle capture for loads (BEFORE any later step can
        // overwrite the addr): effective addr from pre-step regs/fields,
        // value read post-step. MMIO addrs refused (import boundary).
        if name.as_str() == "l32r" {
            let eff = opv[1].0;
            if (0x6000_0000..0x6010_0000).contains(&eff) {
                // Marker (width 8): Python fails loudly (import boundary).
                reads_log.push((eff, 0, 8));
            } else {
                reads_log.push((eff, m.soc.read32(eff), 4));
            }
        } else {
            let lw = match name.as_str() {
                "l8ui" => Some((1u32, opv[1].0, opv[2].0)),
                "l32i" | "l32i_n" => Some((4u32, opv[1].0, opv[2].0)),
                _ => None,
            };
            if let Some((width, base, off)) = lw {
                if !(opv[1].1 && !opv[2].1) {
                    panic!("TEMP-DIFF: unexpected load shape at {:#x}", pc);
                }
                let eff = rr[base as usize].wrapping_add(off);
                if (0x6000_0000..0x6010_0000).contains(&eff) {
                    // Marker (width 8): Python fails loudly (import boundary).
                    reads_log.push((eff, 0, 8));
                    continue;
                }
                let val = if width == 1 {
                    m.soc.read8(eff)
                } else {
                    m.soc.read32(eff)
                };
                reads_log.push((eff, val, width));
            }
        }
        let _ = m.step();
    }
    println!("],");
    let mut regs1 = [0u32; 16];
    for r in 0..16 {
        regs1[r as usize] = m.cpu[0].reg(r);
    }
    let wb1 = m.cpu[0].windowbase();
    println!("\"regs0\": {:?},", regs0);
    println!("\"phys0\": {:?},", phys0.as_slice());
    // Entry PS/WINDOWSTART (static ENTRY needs the caller's CALLINC).
    println!("\"ps0\": {}, \"ws0\": {},", ps0, ws0);
    // Entry-pc histogram (dynamic function discovery).
    let mut hist: Vec<(u32, usize)> = funcs.into_iter().collect();
    hist.sort_by_key(|(_, c)| core::cmp::Reverse(*c));
    hist.truncate(25);
    print!("\"funcs\": [");
    for (i, (pc, c)) in hist.iter().enumerate() {
        if i > 0 {
            print!(",");
        }
        print!("[{}, {}]", pc, c);
    }
    println!("],");
    println!("\"regs1\": {:?},", regs1);
    println!("\"wb0\": {}, \"wb1\": {},", wb0, wb1);
    println!("\"ps1\": {}, \"ws1\": {},", m.cpu[0].sreg(230), m.cpu[0].sreg(73));
    print!("\"mem\": [");
    let mut first = true;
    for i in 0..WORDS {
        let after = m.soc.read32(DRAM + i as u32 * 4);
        if after != before[i] {
            if !first {
                print!(",");
            }
            first = false;
            print!(
                "[{}, {}, {}]",
                DRAM + i as u32 * 4,
                before[i],
                after
            );
        }
    }
    // Oracle reads were captured inline during stepping (exact even
    // under later overwrites); just print them.
    print!("], \"reads\": [");
    let mut rfirst = true;
    for (eff, val, width) in reads_log.iter() {
        if !rfirst {
            print!(",");
        }
        rfirst = false;
        print!("[{}, {}, {}]", eff, val, width);
    }
    // PSRAM physical diff (words): distinguishes PSRAM-mapped flash-window
    // writes (land here) from flash-mapped drops (no trace anywhere).
    print!("], \"psram\": [");
    let psram_after = m.soc.psram_snapshot();
    let mut pfirst = true;
    let n = psram_before.len().min(psram_after.len()) / 4;
    for i in 0..n {
        let b = u32::from_le_bytes([
            psram_before[4 * i],
            psram_before[4 * i + 1],
            psram_before[4 * i + 2],
            psram_before[4 * i + 3],
        ]);
        let a = u32::from_le_bytes([
            psram_after[4 * i],
            psram_after[4 * i + 1],
            psram_after[4 * i + 2],
            psram_after[4 * i + 3],
        ]);
        if a != b {
            if !pfirst {
                print!(",");
            }
            pfirst = false;
            print!("[{}, {}, {}]", 4 * i, b, a);
        }
    }
    // TEMP-SFUNC static preload: full DRAM + IRAM + ROM snapshots (the
    // static emitter identity-loads them into module memory; only func:
    // mode needs them, but printing unconditionally keeps one code path —
    // traces stay small because most words print once here, not per step).
    // Sizes: DRAM/IRAM 512KB, IROM 384KB (wordwise via the Bus).
    print!("], \"fulldram\": [");
    for i in 0..0x80000 / 4 {
        if i > 0 {
            print!(",");
        }
        print!("{}", m.soc.read32(0x3FC8_0000 + i as u32 * 4));
    }
    print!("], \"fulliram\": [");
    for i in 0..0x80000 / 4 {
        if i > 0 {
            print!(",");
        }
        print!("{}", m.soc.read32(0x4037_0000 + i as u32 * 4));
    }
    print!("], \"fullrom\": [");
    for i in 0..0x60000 / 4 {
        if i > 0 {
            print!(",");
        }
        print!("{}", m.soc.read32(0x4000_0000 + i as u32 * 4));
    }
    println!("]}}");
}
