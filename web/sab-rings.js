// Shared SAB transport for the S3 Lab emulation worker (see emu-worker.js).
//
// Two transports exist between the emulation worker and the UI thread:
//   - postMessage (default): frame results ride as transferable buffers.
//     Works everywhere, no special headers.
//   - SAB shared-ring (opt-in `transport: 'sab'`): the worker writes UART
//     bytes + counters into a SharedArrayBuffer; the UI thread polls it with
//     rAF (zero postMessage/copy per frame). Requires crossOriginIsolated
//     (COOP/COEP — see tools/serve.py); without it the worker reports
//     `sabUnsupported` and the UI falls back to postMessage.
//
// Layout (all u32, little-endian platform order):
//   CTRL[0] WR_SEQ   — bumped per published frame
//   CTRL[1] RD_SEQ   — UI ack of the last consumed frame
//   CTRL[2] GPIO     — latest gpio_output() mask
//   CTRL[3] STEPS_LO — total instructions executed (low 32)
//   CTRL[4] STEPS_HI — total instructions executed (high 32)
//   CTRL[5] MIPS_X10 — smoothed MIPS * 10
//   CTRL[6] DROPPED  — UART bytes dropped while the ring was full
//   CTRL[7] PC       — core-0 pc of the last frame
//   CTRL[8] UART_HEAD / CTRL[9] UART_TAIL — ring byte offsets
// UART ring follows the control block (64 KiB).

export const CTRL_LEN = 10;
export const UART_RING = 65536;
export const SAB_BYTES = CTRL_LEN * 4 + UART_RING;

export const C = {
  WR: 0, RD: 1, GPIO: 2, STEPS_LO: 3, STEPS_HI: 4,
  MIPS_X10: 5, DROPPED: 6, PC: 7, UHEAD: 8, UTAIL: 9,
};

export function supported() {
  try {
    return (
      typeof SharedArrayBuffer !== 'undefined' &&
      typeof Atomics !== 'undefined' &&
      self.crossOriginIsolated === true &&
      new SharedArrayBuffer(8).byteLength === 8
    );
  } catch {
    return false;
  }
}

export function createRings() {
  const sab = new SharedArrayBuffer(SAB_BYTES);
  return { sab, ctrl: new Uint32Array(sab, 0, CTRL_LEN), uart: new Uint8Array(sab, CTRL_LEN * 4) };
}

export function wrapRings(sab) {
  return { sab, ctrl: new Uint32Array(sab, 0, CTRL_LEN), uart: new Uint8Array(sab, CTRL_LEN * 4) };
}

// Writer side (emulation worker). Overwrites oldest bytes when full and
// counts them in DROPPED (same drop-not-block discipline as the 128-byte
// hardware FIFOs the inject paths model).
export function ringWriteUart(rings, bytes) {
  const { ctrl, uart } = rings;
  let head = Atomics.load(ctrl, C.UHEAD);
  const tail = Atomics.load(ctrl, C.UTAIL);
  let dropped = 0;
  for (let i = 0; i < bytes.length; i++) {
    const next = (head + 1) % UART_RING;
    if (next === Atomics.load(ctrl, C.UTAIL)) {
      dropped++;
      continue;
    }
    uart[head] = bytes[i];
    head = next;
  }
  Atomics.store(ctrl, C.UHEAD, head);
  if (dropped) Atomics.add(ctrl, C.DROPPED, dropped);
}

// Reader side (UI thread). Returns up to maxBytes newly arrived bytes.
export function ringReadUart(rings, maxBytes = 16384) {
  const { ctrl, uart } = rings;
  let tail = Atomics.load(ctrl, C.UTAIL);
  const head = Atomics.load(ctrl, C.UHEAD);
  let n = (head - tail + UART_RING) % UART_RING;
  if (n > maxBytes) n = maxBytes;
  const out = new Uint8Array(n);
  for (let i = 0; i < n; i++) out[i] = uart[(tail + i) % UART_RING];
  Atomics.store(ctrl, C.UTAIL, (tail + n) % UART_RING);
  return out;
}
