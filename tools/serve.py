#!/usr/bin/env python3
"""Serve web/ with COOP/COEP headers for SAB development.

SharedArrayBuffer (the worker<->UI zero-copy ring transport, web/sab-rings.js)
requires a cross-origin-isolated page: COOP + COEP headers. Plain
`python3 -m http.server` does not send them, so the Transport selector can
only offer postMessage there (auto-fallback — the page still works, just
without the SAB path).

Usage: python3 tools/serve.py [port, default 8129]
Serves web/ at http://127.0.0.1:<port>/ with:
  Cross-Origin-Opener-Policy: same-origin
  Cross-Origin-Embedder-Policy: require-corp
Then open index.html and set Transport = SAB (or Auto).
"""

import http.server
import sys
from functools import partial
from pathlib import Path

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8129
ROOT = Path(__file__).resolve().parent.parent / "web"


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        super().end_headers()


if __name__ == "__main__":
    http.server.ThreadingHTTPServer(
        ("127.0.0.1", PORT), partial(Handler, directory=str(ROOT))
    ).serve_forever()
