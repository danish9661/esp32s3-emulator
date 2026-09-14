#!/usr/bin/env node
// CI browser-bundle check: rebuilds web/pkg from current Rust sources and
// boots the hello firmware in-wasm to `boot OK`.
//
// Why this exists: web/pkg/ is gitignored and rebuilt by hand, so it rots
// silently (2026-09-14: 13h stale, predating all speed work). The battery
// never touches it (NODE harnesses build their own nodejs-target pkg into
// tools/.virtual_demo_pkg/). This check closes that gap for the web target.
//
// Steps:
//   1. wasm-pack build crates/wasm-bridge --target web --out-dir <tmp>
//      (NOT web/pkg: never clobber a developer's local bundle in CI;
//      correctness of the build output is what we assert, plus freshness
//      of the checked-in web/ sources against the local web/pkg).
//   2. API-compat: every `emu.*` call in web/main.js must exist in the
//      freshly built wasm_bridge.d.ts.
//   3. Freshness: no .rs file under crates/ may be newer than
//      web/pkg/wasm_bridge_bg.wasm (warn -> FAIL: bundle is stale).
//   4. Boot check: build a second nodejs-target pkg and boot
//      tools/sketches/esp32s3_hello/esp32s3_hello.merged.bin in-wasm,
//      asserting `boot OK` in UART output.
//
// Usage: node tools/web_bundle_check.mjs [--keep-tmp]
// Exit 0 on PASS, 1 with diagnostics on FAIL.
import { execFileSync } from 'node:child_process';
import { existsSync, statSync, readFileSync, readdirSync, mkdtempSync, rmSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import os from 'node:os';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const KEEP = process.argv.includes('--keep-tmp');
let fails = 0;
const fail = (m) => { console.log(`FAIL ${m}`); fails++; };
const pass = (m) => console.log(`PASS ${m}`);

// --- 1. fresh web-target build into a temp dir ---
const tmp = mkdtempSync(join(os.tmpdir(), 'web-bundle-check-'));
try {
  execFileSync('wasm-pack', ['build', 'crates/wasm-bridge', '--target', 'web', '--out-dir', tmp], {
    cwd: root, stdio: 'pipe',
  });
  pass('wasm-pack web build from current sources');
} catch (e) {
  fail(`wasm-pack web build failed:\n${(e.stderr || e.stdout || e.message || '').toString().slice(0, 2000)}`);
  process.exit(1);
}

// --- 2. API-compat: every emu.* call in main.js exists in the fresh d.ts ---
const main = readFileSync(join(root, 'web', 'main.js'), 'utf8');
const dts = readFileSync(join(tmp, 'wasm_bridge.d.ts'), 'utf8');
const calls = [...new Set([...main.matchAll(/emu\.(\w+)/g)].map((m) => m[1]))];
const missing = calls.filter((c) => !dts.includes(c));
if (missing.length === 0) pass(`API-compat (${calls.length} emu.* calls resolve in d.ts)`);
else fail(`API-compat missing from d.ts: ${missing.join(', ')}`);

// --- 3. freshness: local web/pkg must not predate any crate source ---
const pkgWasm = join(root, 'web', 'pkg', 'wasm_bridge_bg.wasm');
if (!existsSync(pkgWasm)) {
  fail('web/pkg/wasm_bridge_bg.wasm missing (run: wasm-pack build crates/wasm-bridge --target web --out-dir ../../web/pkg)');
} else {
  const built = statSync(pkgWasm).mtimeMs;
  let out = '';
  try {
    out = execFileSync('find', ['crates', '-name', '*.rs', '-newer', pkgWasm], { cwd: root, encoding: 'utf8' });
  } catch { /* find errors -> treat as stale */ out = 'find-error'; }
  if (out.trim().length === 0) pass('web/pkg fresh vs crates/');
  else fail(`web/pkg stale: ${out.trim().split('\n').slice(0, 5).join(', ')} newer than the bundle (rebuild web/pkg)`);
}

// --- 4. boot check: nodejs-target pkg boots hello to `boot OK` ---
const nodePkg = mkdtempSync(join(os.tmpdir(), 'web-bundle-node-'));
try {
  execFileSync('wasm-pack', ['build', 'crates/wasm-bridge', '--target', 'nodejs', '--out-dir', nodePkg], {
    cwd: root, stdio: 'pipe',
  });
} catch (e) {
  fail(`wasm-pack nodejs build failed:\n${(e.stderr || e.stdout || e.message || '').toString().slice(0, 2000)}`);
  process.exit(1);
}
const { Emulator } = await import(join(nodePkg, 'wasm_bridge.js'));
const flash = new Uint8Array(readFileSync(join(root, 'tools', 'sketches', 'esp32s3_hello', 'esp32s3_hello.merged.bin')));
const emu = new Emulator();
emu.load_flash(flash);
let out = '';
let n = 0;
for (let i = 0; i < 5000 && n < 200_000_000; i++) {
  n += emu.step_batch(40000);
  out += Buffer.from(emu.uart_read()).toString();
  if (out.includes('boot OK')) break;
}
if (out.includes('boot OK')) pass(`in-wasm hello boot OK (${n} insns)`);
else fail(`in-wasm hello boot missing boot OK (tail: ${JSON.stringify(out.slice(-300))})`);

if (!KEEP) { rmSync(tmp, { recursive: true, force: true }); rmSync(nodePkg, { recursive: true, force: true }); }
else console.log(`kept tmp dirs: ${tmp} ${nodePkg}`);
console.log(fails === 0 ? '== web bundle check: ALL PASS ==' : `== web bundle check: ${fails} FAILs ==`);
process.exit(fails === 0 ? 0 : 1);
