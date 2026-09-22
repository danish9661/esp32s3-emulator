#!/usr/bin/env python3
"""Sync gallery firmware bins into web/firmware/ from tools/sketches.

Reads web/firmware/manifest.json; for each unique gallery file, resolves the
source binary under tools/sketches (mirroring run_battery.sh /
check_firmware_freshness.py resolution, incl. variants) and copies it into
web/firmware/.

Why this exists: web/firmware/*.merged.bin are gitignored local copies (only
manifest.json is tracked); the committed bins live under tools/sketches (a
curated subset is force-added, the rest rebuild via arduino-cli). A fresh
checkout (CI, Pages) therefore has an empty web/firmware/ and the gallery
404s. This script repopulates it — locally and in the Pages workflow.

Usage:
  tools/sync_gallery.py           # copy-only (uses committed/present bins)
  tools/sync_gallery.py --build   # compile missing sketches via arduino-cli
  tools/sync_gallery.py --check   # report only; exit 1 if anything missing

Exit 0 always unless --check and something is unresolvable.
"""
import glob
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SK = os.path.join(ROOT, "tools", "sketches")
WEB_FW = os.path.join(ROOT, "web", "firmware")
BUILD = "--build" in sys.argv
CHECK = "--check" in sys.argv

# Gallery files needing non-default builds: gallery file ->
# (sketch_dir, fqbn_suffix, build_inobin). Default (absent here) is
# dir tools/sketches/<base>, fqbn esp32:esp32:esp32s3,
# inobin <base>.ino.merged.bin. ota_update uses its two-pass script;
# twai_driver's committed bin lives in build/ by convention.
BUILD_SPECIALS = {
    "esp32s3_psram_opi.merged.bin": ("esp32s3_psram", ":PSRAM=opi", "esp32s3_psram.ino.merged.bin"),
    "esp32s3_ota_update.merged.bin": ("__ota_script__", "", ""),
    "esp32s3_twai_driver.merged.bin": ("esp32s3_twai_driver", "", "esp32s3_twai_driver.ino.merged.bin"),
}

missing = []


def note(msg):
    print(("FAIL " if msg.startswith("FAIL") else "WARN ") + msg)
    if msg.startswith("FAIL"):
        missing.append(msg)


def find_present(gfile):
    """Resolve an already-built bin: exact top-level copy anywhere under
    tools/sketches first (covers variants like psram_opi whose dir
    esp32s3_psram differs from the file base esp32s3_psram_opi), then any
    build/ output, mirroring run_battery.sh fallback order."""
    # Exact filename match anywhere (top-level copies + build outputs).
    cands = sorted(glob.glob(os.path.join(SK, "*", gfile)))
    if cands:
        return cands[0]
    cands = sorted(glob.glob(os.path.join(SK, "*", "*", gfile)))
    if cands:
        return cands[0]
    base = gfile[: -len(".merged.bin")]
    cands = sorted(glob.glob(os.path.join(SK, base, "build", "*.merged.bin")))
    if cands:
        return cands[0]
    cands = sorted(glob.glob(os.path.join(SK, "*", "build", gfile)))
    if cands:
        return cands[0]
    return None


def build_gallery_file(gfile):
    """Compile the sketch for a gallery file via arduino-cli (or its
    special build script). Returns the built bin path, or None."""
    if shutil.which("arduino-cli") is None:
        print(f"SKIP build {gfile} (arduino-cli missing)")
        return None
    if gfile in BUILD_SPECIALS and BUILD_SPECIALS[gfile][0] == "__ota_script__":
        r = subprocess.run(
            [os.path.join(ROOT, "tools", "build_ota.sh")],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        if r.returncode != 0:
            print(f"FAIL build {gfile} (build_ota.sh):\n{r.stderr[-2000:]}")
            return None
        return os.path.join(SK, "esp32s3_ota_update", gfile)
    if gfile in BUILD_SPECIALS:
        subdir, fqbn_suffix, inobin = BUILD_SPECIALS[gfile]
    else:
        base = gfile[: -len(".merged.bin")]
        subdir, fqbn_suffix, inobin = base, "", base + ".ino.merged.bin"
    srcdir = os.path.join(SK, subdir)
    if not os.path.isdir(srcdir):
        print(f"FAIL build {gfile} (no sketch dir {srcdir})")
        return None
    fqbn = "esp32:esp32:esp32s3" + fqbn_suffix
    builddir = os.path.join(srcdir, "build")
    r = subprocess.run(
        ["arduino-cli", "compile", "--fqbn", fqbn, "--build-path", builddir, srcdir],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    if r.returncode != 0:
        print(f"FAIL build {gfile} (compile):\n{r.stderr[-2000:]}")
        return None
    built = os.path.join(builddir, inobin)
    if not os.path.exists(built):
        print(f"FAIL build {gfile} (no output {built})")
        return None
    # Persist a top-level copy where the battery/freshness guard expects it
    # (except twai_driver, whose convention keeps the bin in build/).
    if gfile != "esp32s3_twai_driver.merged.bin":
        shutil.copyfile(built, os.path.join(srcdir, gfile))
    return built


man = json.load(open(os.path.join(WEB_FW, "manifest.json")))
files = list(dict.fromkeys(e["file"] for e in man))
os.makedirs(WEB_FW, exist_ok=True)
synced = 0
for gfile in files:
    src = find_present(gfile)
    if src is None and BUILD:
        src = build_gallery_file(gfile)
    if src is None:
        note(f"FAIL gallery file unresolvable: {gfile}")
        continue
    dst = os.path.join(WEB_FW, gfile)
    if CHECK:
        continue
    shutil.copyfile(src, dst)
    synced += 1

print(f"== gallery sync: {synced}/{len(files)} files in web/firmware/ ==")
if CHECK and missing:
    sys.exit(1)
