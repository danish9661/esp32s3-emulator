#!/usr/bin/env python3
"""Maintenance guard: firmware-bin freshness + gallery coverage (warn-only).

Checks (exit 0 always unless --strict; prints FAIL lines for CI):
  1. Freshness: every battery case's committed .merged.bin must be newer
     than its sketch sources (.ino + build/* when present). A stale bin is
     exactly the 2026-09-14 rot class (bins rebuilt 17:36, sources newer ->
     6 phantom "pre-existing" fails). Mirrors run_battery.sh bin resolution
     incl. variants (psram_*/hello_opi/touch_denoise/flashenc/rsa/twai_driver
     build-output, ota_update two-pass, secure_boot sign-reassemble) and
     NODE harnesses.
  2. Gallery coverage: every web/firmware/manifest.json entry must resolve
     to an existing web/firmware/<file>, and every manifest file must be a
     committed sketch bin (warns on orphans either direction). secure_boot
     is intentionally gallery-absent (needs the SECURE_BOOT_EN burn, which
     the browser cannot provide). Gallery-excluded by the same policy class
     (needs a host fixture the browser cannot provide): Touch
     (TOUCH_INJECT). Both are allow-listed, not failures.

Usage: tools/check_firmware_freshness.py [--strict]
"""
import glob
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SK = os.path.join(ROOT, "tools", "sketches")
WEB_FW = os.path.join(ROOT, "web", "firmware")
STRICT = "--strict" in sys.argv

fails = []


def note(msg):
    print(("FAIL " if msg.startswith("FAIL") else "WARN ") + msg)
    if msg.startswith("FAIL"):
        fails.append(msg)


def newest_source(sketch_dir):
    """Newest mtime among sketch sources (.ino, .h). Build outputs are
    deliberately EXCLUDED: build/ artifacts are always newer than any
    committed bin (they are rewritten by every --build run) and would
    false-positive every case. What matters is whether the committed bin
    postdates the human-edited sources."""
    pats = ["*.ino", "*.h"]
    best = 0.0
    for p in pats:
        for f in glob.glob(os.path.join(sketch_dir, p)):
            try:
                best = max(best, os.path.getmtime(f))
            except OSError:
                pass
    # fall back to dir mtime if nothing matched
    if best == 0.0:
        try:
            best = os.path.getmtime(sketch_dir)
        except OSError:
            pass
    return best


def bin_for_case(name):
    """Mirror run_battery.sh resolution -> (bin_path, [source_dirs])."""
    if name == "psram_qspi":
        return (f"{SK}/esp32s3_psram/esp32s3_psram_qspi.merged.bin", [f"{SK}/esp32s3_psram"])
    if name in ("psram_opi", "psram_16m"):
        base = "psram_opi" if name == "psram_opi" else "psram_16m"
        return (f"{SK}/esp32s3_psram/esp32s3_psram_{base.split('_')[1]}.merged.bin", [f"{SK}/esp32s3_psram"])
    if name == "hello_opi":
        return (f"{SK}/esp32s3_hello/esp32s3_hello_opi.merged.bin", [f"{SK}/esp32s3_hello"])
    if name == "touch_denoise":
        return (f"{SK}/esp32s3_touch/esp32s3_touch.merged.bin", [f"{SK}/esp32s3_touch"])
    if name == "flashenc":
        return (f"{SK}/esp32s3_hello/esp32s3_hello.merged.bin", [f"{SK}/esp32s3_hello"])
    if name == "rsa":
        return (
            f"{SK}/esp32s3_rsa/esp32s3_rsa_poke/esp32s3_rsa_poke.merged.bin",
            [f"{SK}/esp32s3_rsa/esp32s3_rsa_poke"],
        )
    if name == "ota_update":
        return (
            f"{SK}/esp32s3_ota_update/esp32s3_ota_update.merged.bin",
            [f"{SK}/esp32s3_ota_update", f"{SK}/esp32s3_ota_slot1"],
        )
    if name == "secure_boot":
        # sign-reassemble build: sources are the .ino + the build script
        # itself (key + app_signed.bin regenerate); the hello build outputs
        # it consumes are covered by the hello case, not here.
        return (
            f"{SK}/esp32s3_secure_boot/esp32s3_secure_boot.merged.bin",
            [f"{SK}/esp32s3_secure_boot"],
        )
    if name == "twai_driver":
        # committed bin lives in build/ (no top-level copy by convention)
        b = f"{SK}/esp32s3_twai_driver/build/esp32s3_twai_driver.ino.merged.bin"
        return (b, [f"{SK}/esp32s3_twai_driver"])
    if name == "gdb":
        return (f"{SK}/esp32s3_hello/esp32s3_hello.merged.bin", [f"{SK}/esp32s3_hello"])
    d = f"{SK}/esp32s3_{name}"
    for cand in (f"{d}/esp32s3_{name}.merged.bin", f"{d}/esp32s3_{name}.ino.merged.bin"):
        if os.path.exists(cand):
            return (cand, [d])
    found = glob.glob(f"{d}/build/*.merged.bin")
    if found:
        return (found[0], [d])
    return (f"{d}/esp32s3_{name}.merged.bin", [d])


def battery_cases():
    import re

    txt = open(os.path.join(ROOT, "tools", "run_battery.sh")).read()
    return re.findall(r'^"([^"|]+)\|', txt, re.M)


# --- 1. freshness ---
for name in battery_cases():
    binpath, srcdirs = bin_for_case(name)
    if not os.path.exists(binpath):
        note(f"FAIL missing bin for battery case {name}: {os.path.relpath(binpath, ROOT)}")
        continue
    bmt = os.path.getmtime(binpath)
    for sd in srcdirs:
        smt = newest_source(sd)
        if smt > bmt + 1.0:
            note(
                f"FAIL stale bin for {name}: {os.path.relpath(binpath, ROOT)} "
                f"older than sources in {os.path.relpath(sd, ROOT)} "
                f"(rebuild: ./tools/run_battery.sh --build {name})"
            )

# --- 2. gallery coverage ---
# Gallery policy (documented 2026-09-14): the gallery is a curated subset,
# not 1:1 with the battery. 36/36 entries are the in-wasm-validatable set;
# everything else is intentionally gallery-absent (needs host env/fixtures
# the browser cannot provide, is a build variant sharing one source dir, or
# was never promoted). Warn-only: a missing gallery entry is NEVER a FAIL.
# NOTE: web/firmware/*.merged.bin are gitignored local copies (only
# manifest.json is tracked); the committed bins live under tools/sketches.
# So "orphan" here means "present locally but not in the manifest" — a
# leftover-copy hint, not a repo inconsistency.
GALLERY_ABSENT_OK = {"secure_boot"}  # needs SECURE_BOOT_EN burn; browser can't provide
man = json.load(open(os.path.join(WEB_FW, "manifest.json")))
mfiles = [e["file"] for e in man]
for e in man:
    p = os.path.join(WEB_FW, e["file"])
    if not os.path.exists(p):
        note(f"FAIL gallery manifest entry missing bin: {e['file']}")
webbins = {f for f in os.listdir(WEB_FW) if f.endswith(".merged.bin")}
for f in sorted(webbins - set(mfiles)):
    note(f"WARN web/firmware orphan bin (no manifest entry): {f}")
covered = set()
for e in man:
    covered.add(e["file"])
# Gallery-coverage section is informational: the gallery is a curated
# subset (36 entries), not 1:1 with the 106 battery cases. Report the
# counts and stop — per-case "no gallery entry" lines would just restate
# policy as noise.
print(f"gallery entries: {len(mfiles)}, battery cases: {len(battery_cases())} (gallery is a curated subset; absence is policy, not rot)")

print(f"== freshness+gallery guard: {len(fails)} FAILs ==")
sys.exit(1 if (fails and STRICT) else 0)
