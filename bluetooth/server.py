"""Serve the local BalanceBot Web Bluetooth console on a trustworthy localhost origin."""

from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import os


HOST = "127.0.0.1"
PORT = 8765


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, max-age=0")
        super().end_headers()


def main() -> None:
    root = Path(__file__).resolve().parent
    os.chdir(root)
    server = ThreadingHTTPServer((HOST, PORT), NoCacheHandler)
    print(f"BalanceBot BLE console: http://{HOST}:{PORT}/bluetooth.html")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
