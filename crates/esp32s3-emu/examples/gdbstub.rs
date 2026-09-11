//! GDB remote-serial-protocol stub for the ESP32-S3 emulator (host tool).
//!
//! Usage: `gdbstub <firmware.merged.bin> [port=3333]`, then in GDB:
//! `target remote :3333`. Single-threaded, core 0, whole-machine lockstep
//! (both Xtensa cores advance together; breakpoints watch core 0's pc).
//! Registers are a custom 19×u32 layout (`a0..a15` logical window regs,
//! `pc`, `sar`, `windowbase`) described via `qXfer:features:read`
//! (target.xml), so stock GDB works without Xtensa support (disassembly
//! and backtraces are limited; break/step/inspect are fully functional).
//! Software breakpoints only (tracked addresses, no code patching):
//! `Z0`/`z0` (and `Z1`/`z1` treated the same). `c` runs with a step
//! budget and Ctrl-C (0x03) interrupt; `s` single-steps the machine once.
//! UART output goes to this process's stdout (like run_flash).

use esp32s3_emu::Esp32S3;
use std::env;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use xtensa_core::{Bus, cpu::SR_SAR};

/// Custom GDB target description (19 little-endian u32 regs).
const TARGET_XML: &str = r#"<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target><feature name="org.esp32s3.emu.cpu">
<reg name="a0" bitsize="32"/><reg name="a1" bitsize="32"/><reg name="a2" bitsize="32"/><reg name="a3" bitsize="32"/><reg name="a4" bitsize="32"/><reg name="a5" bitsize="32"/><reg name="a6" bitsize="32"/><reg name="a7" bitsize="32"/><reg name="a8" bitsize="32"/><reg name="a9" bitsize="32"/><reg name="a10" bitsize="32"/><reg name="a11" bitsize="32"/><reg name="a12" bitsize="32"/><reg name="a13" bitsize="32"/><reg name="a14" bitsize="32"/><reg name="a15" bitsize="32"/><reg name="pc" bitsize="32" type="code_ptr"/><reg name="sar" bitsize="32"/><reg name="windowbase" bitsize="32"/>
</feature></target>"#;

/// Steps a `c` runs before giving up with TRAP (Ctrl-C interrupts sooner).
const CONT_BUDGET: u64 = 500_000_000;

fn hex(v: u8) -> u8 {
    if v < 10 { b'0' + v } else { b'a' + v - 10 }
}

fn hexval(b: u8) -> u8 {
    match b {
        b'0'..=b'9' => b - b'0',
        b'a'..=b'f' => b - b'a' + 10,
        b'A'..=b'F' => b - b'A' + 10,
        _ => 0,
    }
}

fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |a, b| a.wrapping_add(*b))
}

/// Read one RSP packet body (without `$...#cs`). Returns None on Ctrl-C.
/// `pending` carries a byte already read (e.g. a `$` mistaken for an ack).
fn read_packet(s: &mut TcpStream, buf: &mut Vec<u8>, pending: &mut Option<u8>) -> Option<Vec<u8>> {
    buf.clear();
    let mut b = [0u8; 1];
    // Consume a pending byte first.
    if let Some(p) = pending.take() {
        if p == 0x03 {
            return None;
        }
        if p != b'$' {
            // Stray ack/byte: keep scanning for the next packet.
            loop {
                if s.read_exact(&mut b).is_err() {
                    return None;
                }
                if b[0] == 0x03 {
                    return None;
                }
                if b[0] == b'$' {
                    break;
                }
            }
        }
    } else {
        loop {
            if s.read_exact(&mut b).is_err() {
                return None;
            }
            if b[0] == 0x03 {
                return None;
            }
            if b[0] == b'$' {
                break;
            }
        }
    }
    let mut body = Vec::new();
    loop {
        if s.read_exact(&mut b).is_err() {
            return None;
        }
        if b[0] == b'#' {
            break;
        }
        // RLE (`*`) and binary escapes are not produced by our replies;
        // requests using them get a best-effort raw treatment below.
        body.push(b[0]);
    }
    let mut cs = [0u8; 2];
    if s.read_exact(&mut cs).is_err() {
        return None;
    }
    let _ = s.write_all(b"+");
    Some(body)
}

fn send(s: &mut TcpStream, body: &[u8], pending: &mut Option<u8>) {
    let mut out = Vec::with_capacity(body.len() + 4);
    out.push(b'$');
    out.extend_from_slice(body);
    out.push(b'#');
    let cs = checksum(body);
    out.push(hex(cs >> 4));
    out.push(hex(cs & 15));
    let _ = s.write_all(&out);
    // Best-effort ack read: a `$` here means the client already pipelined
    // its next packet (e.g. it skips acks); stash it for read_packet.
    // Anything else (ack/NAK/timeout) is ignored.
    let _ = s.set_read_timeout(Some(std::time::Duration::from_millis(20)));
    let mut b = [0u8; 1];
    if s.read_exact(&mut b).is_ok() && b[0] != b'+' {
        *pending = Some(b[0]);
    }
    let _ = s.set_read_timeout(Some(std::time::Duration::from_millis(100)));
}

fn send_str(s: &mut TcpStream, pending: &mut Option<u8>, body: &str) {
    send(s, body.as_bytes(), pending);
}

fn regs_hex(m: &Esp32S3) -> String {
    let mut r = String::with_capacity(19 * 8);
    for i in 0..16u32 {
        r.push_str(&format!("{:08x}", m.cpu[0].reg(i)));
    }
    r.push_str(&format!("{:08x}", m.cpu[0].pc));
    r.push_str(&format!("{:08x}", m.cpu[0].sreg(SR_SAR)));
    r.push_str(&format!("{:08x}", m.cpu[0].windowbase()));
    r
}

fn parse_hex(s: &[u8]) -> u32 {
    let mut v = 0u32;
    for &b in s {
        v = v.wrapping_shl(4) | hexval(b) as u32;
    }
    v
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: gdbstub <firmware.merged.bin> [port=3333]");
        std::process::exit(1);
    }
    let port: u16 = args.get(2).and_then(|p| p.parse().ok()).unwrap_or(3333);
    let img = std::fs::read(&args[1]).expect("read firmware");
    let mut m = Esp32S3::new();
    m.boot_from_flash(&img);
    let listener = TcpListener::bind(("127.0.0.1", port)).expect("bind");
    eprintln!("[gdbstub] listening on 127.0.0.1:{port} (core 0, lockstep)");
    let (mut s, _) = listener.accept().expect("accept");
    s.set_read_timeout(Some(std::time::Duration::from_millis(100)))
        .ok();
    let mut breaks: Vec<u32> = Vec::new();
    let mut buf = Vec::new();
    let mut pending: Option<u8> = None;
    // GDB opens stopped; report the reset vector as the initial stop.
    send_str(&mut s, &mut pending, "S05");
    loop {
        // Drain UART to our stdout (like run_flash) so firmware output is
        // visible alongside the debug session.
        for b in m.take_uart_tx(0).into_iter().chain(m.take_uart_tx(1)) {
            print!("{}", b as char);
        }
        let _ = std::io::stdout().flush();
        let pkt = match read_packet(&mut s, &mut buf, &mut pending) {
            Some(p) => p,
            None => {
                send_str(&mut s, &mut pending, "S02");
                continue;
            }
        };
        if pkt.is_empty() {
            continue;
        }
        match pkt[0] {
            b'?' => send_str(&mut s, &mut pending, "S05"),
            b'g' => send_str(&mut s, &mut pending, &regs_hex(&m)),
            b'p' => {
                let n = parse_hex(&pkt[1..]) as usize;
                let v = match n {
                    0..=15 => m.cpu[0].reg(n as u32),
                    16 => m.cpu[0].pc,
                    17 => m.cpu[0].sreg(SR_SAR),
                    18 => m.cpu[0].windowbase(),
                    _ => 0,
                };
                send_str(&mut s, &mut pending, &format!("{v:08x}"));
            }
            b'P' => {
                if let Some(eq) = pkt.iter().position(|&b| b == b'=') {
                    let n = parse_hex(&pkt[1..eq]) as usize;
                    let mut v = 0u32;
                    for &b in &pkt[eq + 1..] {
                        v = v.wrapping_shl(4) | hexval(b) as u32;
                    }
                    match n {
                        0..=15 => m.cpu[0].set_reg(n as u32, v),
                        16 => m.cpu[0].pc = v,
                        17 => m.cpu[0].set_sreg(SR_SAR, v),
                        _ => {}
                    }
                    send_str(&mut s, &mut pending, "OK");
                } else {
                    send_str(&mut s, &mut pending, "E01");
                }
            }
            b'm' => {
                // mADDR,LEN (both hex).
                let parts: Vec<&[u8]> = pkt[1..].split(|&b| b == b',').collect();
                if parts.len() == 2 {
                    let addr = parse_hex(parts[0]);
                    let len = parse_hex(parts[1]).min(4096) as usize;
                    let mut r = String::with_capacity(len * 2);
                    for i in 0..len {
                        r.push_str(&format!(
                            "{:02x}",
                            m.soc.read8(addr.wrapping_add(i as u32)) as u8
                        ));
                    }
                    send_str(&mut s, &mut pending, &r);
                } else {
                    send_str(&mut s, &mut pending, "E01");
                }
            }
            b'M' => {
                // MADDR,LEN:HEX (with optional RLE `*`).
                if let (Some(comma), Some(colon)) = (
                    pkt.iter().position(|&b| b == b','),
                    pkt.iter().position(|&b| b == b':'),
                ) {
                    let addr = parse_hex(&pkt[1..comma]);
                    let mut data: Vec<u8> = Vec::new();
                    let mut it = pkt[colon + 1..].iter().peekable();
                    while let Some(&b) = it.next() {
                        if b == b'*' {
                            // RLE repeat count (count byte follows, base 29).
                            if let (Some(&c), true) = (it.next(), !data.is_empty()) {
                                let n = c.wrapping_sub(29).wrapping_add(1) as usize;
                                let last = *data.last().unwrap();
                                data.extend(std::iter::repeat_n(last, n.min(4096)));
                            }
                        } else {
                            let hi = hexval(b);
                            if let Some(&lo) = it.next() {
                                data.push(hi << 4 | hexval(lo));
                            }
                        }
                    }
                    for (i, b) in data.iter().enumerate().take(4096) {
                        m.soc.write8(addr.wrapping_add(i as u32), *b as u32);
                    }
                    send_str(&mut s, &mut pending, "OK");
                } else {
                    send_str(&mut s, &mut pending, "E01");
                }
            }
            b'c' => {
                if pkt.len() > 1
                    && let Ok(a) =
                        u32::from_str_radix(std::str::from_utf8(&pkt[1..]).unwrap_or(""), 16)
                {
                    m.cpu[0].pc = a;
                }
                let mut left = CONT_BUDGET;
                let stop = loop {
                    if breaks.contains(&m.cpu[0].pc) {
                        break "S05";
                    }
                    if left == 0 {
                        break "S05";
                    }
                    // Poll for Ctrl-C between step batches.
                    m.step_fast();
                    for b in m.take_uart_tx(0).into_iter().chain(m.take_uart_tx(1)) {
                        print!("{}", b as char);
                    }
                    left -= 1;
                    // Breakpoint check each macro-step is coarse (a block
                    // can step over one); single-step `s` is exact.
                    if breaks.contains(&m.cpu[0].pc) {
                        break "S05";
                    }
                    // Non-blocking Ctrl-C probe.
                    s.set_nonblocking(true).ok();
                    let mut b = [0u8; 1];
                    let intr = matches!(s.read_exact(&mut b), Ok(()) if b[0] == 0x03);
                    s.set_nonblocking(false).ok();
                    let _ = s.set_read_timeout(Some(std::time::Duration::from_millis(100)));
                    if intr {
                        break "S02";
                    }
                };
                send_str(&mut s, &mut pending, stop);
            }
            b's' => {
                if pkt.len() > 1
                    && let Ok(a) =
                        u32::from_str_radix(std::str::from_utf8(&pkt[1..]).unwrap_or(""), 16)
                {
                    m.cpu[0].pc = a;
                }
                m.step();
                for b in m.take_uart_tx(0).into_iter().chain(m.take_uart_tx(1)) {
                    print!("{}", b as char);
                }
                send_str(&mut s, &mut pending, "S05");
            }
            b'Z' | b'z' => {
                // Z0/Z1 (and kind): software breakpoint add/remove.
                let parts: Vec<&[u8]> = pkt[1..].split(|&b| b == b',').collect();
                if parts.len() >= 2 && (pkt[1] == b'0' || pkt[1] == b'1') {
                    let addr = parse_hex(parts[1]);
                    if pkt[0] == b'Z' {
                        if !breaks.contains(&addr) {
                            breaks.push(addr);
                        }
                    } else {
                        breaks.retain(|&a| a != addr);
                    }
                    send_str(&mut s, &mut pending, "OK");
                } else {
                    send_str(&mut s, &mut pending, "");
                }
            }
            b'H' => send_str(&mut s, &mut pending, "OK"),
            b'q' => {
                if pkt.starts_with(b"qSupported") {
                    send_str(&mut s, &mut pending, "PacketSize=4000;qXfer:features:read+");
                } else if pkt.starts_with(b"qfThreadInfo") {
                    send_str(&mut s, &mut pending, "m1");
                } else if pkt.starts_with(b"qsThreadInfo") {
                    send_str(&mut s, &mut pending, "l");
                } else if pkt.starts_with(b"qAttached") {
                    send_str(&mut s, &mut pending, "1");
                } else if pkt.starts_with(b"qXfer:features:read:target.xml:") {
                    // qXfer:features:read:target.xml:OFF,LEN
                    let rest = &pkt["qXfer:features:read:target.xml:".len()..];
                    let parts: Vec<&[u8]> = rest.split(|&b| b == b',').collect();
                    if parts.len() == 2 {
                        let off =
                            usize::from_str_radix(std::str::from_utf8(parts[0]).unwrap_or("0"), 16)
                                .unwrap_or(0);
                        let len =
                            usize::from_str_radix(std::str::from_utf8(parts[1]).unwrap_or("0"), 16)
                                .unwrap_or(0);
                        let xml = TARGET_XML.as_bytes();
                        if off >= xml.len() {
                            send_str(&mut s, &mut pending, "l");
                        } else {
                            let end = (off + len).min(xml.len());
                            let mut r = String::from(if end >= xml.len() { "l" } else { "m" });
                            r.push_str(&String::from_utf8_lossy(&xml[off..end]));
                            send_str(&mut s, &mut pending, &r);
                        }
                    } else {
                        send_str(&mut s, &mut pending, "E01");
                    }
                } else if pkt.starts_with(b"qC") {
                    send_str(&mut s, &mut pending, "QC1");
                } else {
                    send_str(&mut s, &mut pending, "");
                }
            }
            b'v' => send_str(&mut s, &mut pending, ""),
            b'D' => {
                send_str(&mut s, &mut pending, "OK");
                return;
            }
            b'k' => return,
            b'!' => send_str(&mut s, &mut pending, "OK"),
            _ => send_str(&mut s, &mut pending, ""),
        }
    }
}
