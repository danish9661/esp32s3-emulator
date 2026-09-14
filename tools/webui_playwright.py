"""Playwright end-to-end test for the ESP32-S3 web UI.

Serves web/ over HTTP, loads index.html in Chromium, and asserts:
  1. hello firmware boots to 'boot OK' after Run
  2. #mips meter shows '<N> MIPS' while running
  3. serial input box echoes typed text ('>> ...' terminal echo)
  4. uart_echo firmware round-trips a byte injected via UART1
     (firmware prints "[uart1] rx ..."), proving usb/uart_inject_rx works
  5. sdspi gallery entry boots in-wasm to 'SDSPI FAT READ PASS'
     (proves the `"sdspi": true` manifest flag wires the virtual SD card
     through the new `spi_sdspi_attach_sdmmc_image` bridge call)
  6. touch gallery entry reads the injected pad counter in-wasm
     ('TOUCH PASS' — proves the `"touch": "3:1877"` manifest flag wires
     TOUCH_INJECT through the new `touch_inject` bridge call)

Fails loudly on any page error. Exits 0 on PASS, 1 on FAIL.
"""
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path("/home/danish1075/Documents/esp32 s3 emu")
WEB = ROOT / "web"
PORT = 8129

server = subprocess.Popen(
    [sys.executable, "-m", "http.server", str(PORT), "--bind", "127.0.0.1"],
    cwd=str(WEB),
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
)
time.sleep(1.5)

from playwright.sync_api import sync_playwright

failures = []


def check(name, cond, extra=""):
    tag = "PASS" if cond else "FAIL"
    print(f"{tag} {name} {extra}".rstrip(), flush=True)
    if not cond:
        failures.append(name)


try:
    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--use-gl=swiftshader"])
        page = browser.new_page()
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)))

        page.goto(f"http://127.0.0.1:{PORT}/index.html")
        # wasm ready + auto-loaded hello firmware enables Run
        page.wait_for_selector("#run:not([disabled])", timeout=30000)
        check("wasm-ready-run-enabled", True)
        # gallery populated from manifest
        page.wait_for_function(
            "() => document.getElementById('gallery').options.length > 1",
            timeout=15000,
        )
        n_opts = page.eval_on_selector("#gallery", "el => el.options.length")
        check("gallery-populated", n_opts > 5, f"({n_opts} options)")
        # serial input + mips elements exist
        check("serial-input-present", page.is_visible("#serialInput"))
        check("serial-send-present", page.is_visible("#serialSend"))
        check("serial-port-present", page.is_visible("#serialPort"))
        check("mips-present", page.locator("#mips").count() == 1)

        # --- Test 1: hello boots ---
        page.click("#run")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('boot OK')",
                timeout=180000,
            )
            check("hello-boot-OK", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-500)")
            check("hello-boot-OK", False, f"(timeout; tail={tail!r})")

        # --- Test 2: MIPS meter shows while running ---
        try:
            page.wait_for_function(
                "() => /[0-9]+\\.[0-9] MIPS/.test(document.getElementById('mips').textContent)",
                timeout=15000,
            )
            mips = page.eval_on_selector("#mips", "el => el.textContent")
            check("mips-meter", True, f"({mips})")
        except Exception:
            mips = page.eval_on_selector("#mips", "el => el.textContent")
            check("mips-meter", False, f"(got {mips!r})")

        # --- Test 3: serial input terminal echo (no firmware needed) ---
        page.fill("#serialInput", "hi from test")
        page.click("#serialSend")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('hi from test')",
                timeout=5000,
            )
            check("serial-echo", True)
        except Exception:
            check("serial-echo", False, "(typed text never echoed)")

        # --- Test 4: uart_echo round-trip via UART1 ---
        page.click("#stop")
        page.select_option("#gallery", value="./firmware/esp32s3_uart_echo.merged.bin")
        page.wait_for_function(
            "() => !document.getElementById('run').disabled",
            timeout=120000,
        )
        page.click("#run")
        page.wait_for_function(
            "() => document.getElementById('console').textContent.includes('RXREADY')",
            timeout=180000,
        )
        check("uart-echo-ready", True)
        page.select_option("#serialPort", value="1")
        page.fill("#serialInput", "Z")
        page.click("#serialSend")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('[uart1] rx')",
                timeout=120000,
            )
            check("uart1-roundtrip", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-800)")
            check("uart1-roundtrip", False, f"(tail={tail!r})")

        real_errors = [e for e in errors if "favicon" not in e.lower()]
        check("no-page-errors", len(real_errors) == 0, f"({real_errors[:3]!r})" if real_errors else "")

        # --- Test 5: sdspi gallery entry mounts the virtual SD card ---
        page.click("#stop")
        page.select_option("#gallery", value="./firmware/esp32s3_sdspi.merged.bin")
        page.wait_for_function(
            "() => !document.getElementById('run').disabled",
            timeout=120000,
        )
        page.click("#run")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('SDSPI FAT READ PASS')",
                timeout=240000,
            )
            check("sdspi-inwasm-mount", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-800)")
            check("sdspi-inwasm-mount", False, f"(tail={tail!r})")

        # --- Test 6: touch gallery entry reads the injected pad counter ---
        # (proves the `"touch": "3:1877"` manifest flag wires TOUCH_INJECT
        # through the new `touch_inject` bridge call)
        page.click("#stop")
        page.select_option("#gallery", value="./firmware/esp32s3_touch.merged.bin")
        page.wait_for_function(
            "() => !document.getElementById('run').disabled",
            timeout=120000,
        )
        page.click("#run")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('TOUCH PASS')",
                timeout=240000,
            )
            check("touch-inwasm-read", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-800)")
            check("touch-inwasm-read", False, f"(tail={tail!r})")

        real_errors = [e for e in errors if "favicon" not in e.lower()]
        check("no-page-errors-final", len(real_errors) == 0, f"({real_errors[:3]!r})" if real_errors else "")
        browser.close()
finally:
    server.terminate()
    server.wait()

print("== webui playwright: %s ==" % ("ALL PASS" if not failures else f"FAILURES {failures}"))
sys.exit(1 if failures else 0)
