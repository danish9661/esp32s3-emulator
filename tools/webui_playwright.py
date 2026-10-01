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
  7. MicroPython REPL preset boots the bundled stock image in-wasm
     (`>>> ` banner + `print(6*7)` -> `42` over UART0 — proves the ▶ REPL
     button + vfs-partition pad + UART0 flip; image committed at
     tools/firmware/, served same-origin)
  8. wifi_ap gallery entry boots in-wasm to 'WIFI AP DONE' (proves the
     `"wifi_ap"` manifest object wires the SoftAP fixture through the new
     `wifi_ap_fixture` bridge call)
  9. espnow gallery entry boots in-wasm to 'WIFI ESPNOW DONE' (proves the
     `"espnow": true` manifest flag wires the virtual-peer loopback
     through the new `wifi_espnow_fixture` bridge call)
 10. test_worker_net gallery entry boots in-wasm to 'WORKER NET DONE'
     (proves the `"wifi_worker"` manifest string wires the worker STA
     fixture through `wifi_worker_fixture`; empty-FIFO verdicts expected)
 11. Live-IP panel fails soft with no gateway (no page error)
 12. test_worker_l3 gallery entry boots in-wasm to 'WORKER L3 DONE'
     (proves the `"wifi_worker_l3"` manifest string wires the L3 worker
     fixture through the new `wifi_worker_l3_fixture` bridge call)
 13. ble gallery entry boots in-wasm to 'BLE DONE' (proves the gallery
     entry loads; the ROM controller loopback answers basic commands
     locally, no bridge needed)
 14. BLE panel (Connect with no gateway/bridge fails soft, no page error)

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

        # --- Test 8: wifi_ap gallery entry boots in-wasm to WIFI AP DONE ---
        # (proves the `"wifi_ap"` manifest object wires the SoftAP fixture
        # through the new `wifi_ap_fixture` bridge call)
        page.click("#stop")
        page.select_option("#gallery", value="./firmware/esp32s3_wifi_ap.merged.bin")
        page.wait_for_function(
            "() => !document.getElementById('run').disabled",
            timeout=120000,
        )
        page.click("#run")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('WIFI AP DONE')",
                timeout=240000,
            )
            check("wifi-ap-inwasm-done", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-800)")
            check("wifi-ap-inwasm-done", False, f"(tail={tail!r})")

        # --- Test 9: espnow gallery entry boots in-wasm to WIFI ESPNOW DONE ---
        # (proves the `"espnow": true` manifest flag wires the virtual-peer
        # loopback through the new `wifi_espnow_fixture` bridge call)
        page.click("#stop")
        page.select_option("#gallery", value="./firmware/esp32s3_espnow.merged.bin")
        page.wait_for_function(
            "() => !document.getElementById('run').disabled",
            timeout=120000,
        )
        page.click("#run")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('WIFI ESPNOW DONE')",
                timeout=240000,
            )
            check("espnow-inwasm-done", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-800)")
            check("espnow-inwasm-done", False, f"(tail={tail!r})")

        # --- Test 10: test_worker_net gallery entry boots in-wasm to DONE ---
        # (proves the `"wifi_worker"` manifest string wires the worker STA
        # fixture through the new `wifi_worker_fixture` bridge call; the TX
        # tap fires in-wasm too — frames stay local without a gateway, so
        # the empty-FIFO verdicts -13/-14 are expected, same as headless)
        page.click("#stop")
        page.select_option("#gallery", value="./firmware/esp32s3_test_worker_net.merged.bin")
        page.wait_for_function(
            "() => !document.getElementById('run').disabled",
            timeout=120000,
        )
        page.click("#run")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('WORKER NET DONE')",
                timeout=240000,
            )
            check("worker-inwasm-done", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-800)")
            check("worker-inwasm-done", False, f"(tail={tail!r})")

        # --- Test 11: Live-IP panel wires the gateway bridge without errors ---
        # (proves the new panel exists, Connect with no gateway fails soft
        # into the disconnected status — no page error — and TX frames still
        # flow locally: the worker entry above already proved the tap fires;
        # a live gateway E2E is covered headlessly by run_flash NET_GW)
        try:
            check("gw-url-present", page.is_visible("#gwUrl"))
            check("gw-connect-present", page.is_visible("#gwConnect"))
            check("gw-status-present", page.locator("#gwStatus").count() == 1)
            page.fill("#gwUrl", "ws://127.0.0.1:59999/api/network-gateway?sessionId=browser")
            page.click("#gwConnect")
            page.wait_for_function(
                "() => document.getElementById('gwStatus').textContent.includes('disconnected') || document.getElementById('gwStatus').textContent.includes('connecting') || document.getElementById('gwStatus').textContent.includes('error') || document.getElementById('gwStatus').textContent.includes('failed')",
                timeout=15000,
            )
            check("gw-connect-fails-soft", True)
        except Exception:
            tail = page.eval_on_selector("#gwStatus", "el => el.textContent.slice(-200)")
            check("gw-connect-fails-soft", False, f"(tail={tail!r})")

        # --- Test 12: test_worker_l3 gallery entry boots in-wasm to DONE ---
        # (proves the `"wifi_worker_l3"` manifest string wires the L3 worker
        # fixture through the new `wifi_worker_l3_fixture` bridge call;
        # same empty-FIFO discipline as the net worker — no gateway)
        page.click("#stop")
        page.select_option("#gallery", value="./firmware/esp32s3_test_worker_l3.merged.bin")
        page.wait_for_function(
            "() => !document.getElementById('run').disabled",
            timeout=120000,
        )
        page.click("#run")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('WORKER L3 DONE')",
                timeout=240000,
            )
            check("worker-l3-inwasm-done", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-800)")
            check("worker-l3-inwasm-done", False, f"(tail={tail!r})")

        # --- Test 13: ble gallery entry boots in-wasm to BLE DONE ---
        # (proves the gallery entry loads and the ROM controller loopback
        # answers the init commands locally — no bridge needed for DONE)
        page.click("#stop")
        page.select_option("#gallery", value="./firmware/esp32s3_ble.merged.bin")
        page.wait_for_function(
            "() => !document.getElementById('run').disabled",
            timeout=120000,
        )
        page.click("#run")
        try:
            page.wait_for_function(
                "() => document.getElementById('console').textContent.includes('BLE DONE')",
                timeout=240000,
            )
            check("ble-inwasm-done", True)
        except Exception:
            tail = page.eval_on_selector("#console", "el => el.textContent.slice(-800)")
            check("ble-inwasm-done", False, f"(tail={tail!r})")

        # --- Test 14: BLE panel wires the bridge without errors ---
        # (proves the new panel exists and Connect with no gateway/bridge
        # fails soft into the disconnected status — no page error)
        try:
            check("ble-url-present", page.is_visible("#bleUrl"))
            check("ble-connect-present", page.is_visible("#bleConnect"))
            check("ble-status-present", page.locator("#bleStatus").count() == 1)
            page.fill("#bleUrl", "ws://127.0.0.1:59999/api/ble-gateway")
            page.click("#bleConnect")
            page.wait_for_function(
                "() => document.getElementById('bleStatus').textContent.includes('disconnected') || document.getElementById('bleStatus').textContent.includes('connecting') || document.getElementById('bleStatus').textContent.includes('error') || document.getElementById('bleStatus').textContent.includes('failed')",
                timeout=15000,
            )
            check("ble-connect-fails-soft", True)
        except Exception:
            tail = page.eval_on_selector("#bleStatus", "el => el.textContent.slice(-200)")
            check("ble-connect-fails-soft", False, f"(tail={tail!r})")

        # --- Test 7: MicroPython REPL preset (bundled same-origin image) ---
        # (proves the ▶ REPL button + vfs-partition pad + UART0 flip: load
        # the committed tools/firmware/ stock MicroPython .bin, pad to 3
        # MiB with the littlefs vfs record + MD5 exactly like
        # tools/micropython_repl.sh, boot to `>>> `, then evaluate
        # `print(6*7)` -> `42` over UART0)
        import shutil
        mp_src = ROOT / "tools" / "firmware" / "ESP32_GENERIC_S3-20260824-v1.29.0.bin"
        if not mp_src.exists():
            # legacy cache dir (pre-bundle harness downloads)
            mp_src = ROOT / "tools" / ".micropython" / "ESP32_GENERIC_S3-20260824-v1.29.0.bin"
        if not mp_src.exists():
            check("micropython-image-present", False, "(missing tools/firmware/ESP32_GENERIC_S3-20260824-v1.29.0.bin)")
        else:
            shutil.copyfile(mp_src, WEB / "mp_test.bin")
            try:
                page.click("#stop")
                page.fill("#mpUrl", f"http://127.0.0.1:{PORT}/mp_test.bin")
                page.click("#mpLoad")
                page.wait_for_function(
                    "() => document.getElementById('status').textContent.includes('MicroPython ready')",
                    timeout=60000,
                )
                check("micropython-preset-load", True)
                check("micropython-uart0-flip", page.eval_on_selector("#serialPort", "el => el.value") == "0")
                page.click("#run")
                try:
                    page.wait_for_function(
                        "() => document.getElementById('console').textContent.includes('>>> ')",
                        timeout=600000,
                    )
                    check("micropython-repl-banner", True)
                except Exception:
                    tail = page.eval_on_selector("#console", "el => el.textContent.slice(-400)")
                    check("micropython-repl-banner", False, f"(tail={tail!r})")
                page.fill("#serialInput", "print(6*7)")
                page.click("#serialSend")
                try:
                    page.wait_for_function(
                        "() => document.getElementById('console').textContent.includes('42')",
                        timeout=120000,
                    )
                    check("micropython-eval-42", True)
                except Exception:
                    tail = page.eval_on_selector("#console", "el => el.textContent.slice(-400)")
                    check("micropython-eval-42", False, f"(tail={tail!r})")
            finally:
                (WEB / "mp_test.bin").unlink(missing_ok=True)

        real_errors = [e for e in errors if "favicon" not in e.lower()]
        check("no-page-errors-final", len(real_errors) == 0, f"({real_errors[:3]!r})" if real_errors else "")
        browser.close()
finally:
    server.terminate()
    server.wait()

print("== webui playwright: %s ==" % ("ALL PASS" if not failures else f"FAILURES {failures}"))
sys.exit(1 if failures else 0)
