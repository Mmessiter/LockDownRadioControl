#!/usr/bin/env python3
"""
Dev firmware server for LDRC RXV2.

Drop your RXV2-*.bin files in this folder. Run this script. The chip's
/firmware page picks them up automatically.

Usage:
    python3 firmware_server.py            # serves on port 8000
    python3 firmware_server.py 9001       # custom port

Then on the chip's /firmware page, set the manifest URL to:
    http://<your-mac-ip-or-hostname>:8000/manifest.json
"""
import http.server
import json
import os
import re
import socket
import socketserver
import sys
from pathlib import Path

HERE = Path(__file__).parent.resolve()
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000


def parse_version(name: str):
    m = re.match(r"RXV2-(\d+)\.(\d+)\.(\d+)(?:-(.+))?", name)
    if not m:
        return (0, 0, 0, "")
    return (int(m.group(1)), int(m.group(2)), int(m.group(3)), m.group(4) or "")


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def do_GET(self):
        if self.path == "/manifest.json":
            self._send_manifest()
            return
        super().do_GET()

    def _send_manifest(self):
        bins = sorted(HERE.glob("*.bin"))
        versions = []
        for b in bins:
            name = b.stem  # strip .bin
            ver = parse_version(name)
            versions.append({
                "name": name,
                "url": f"/{b.name}",
                "size": b.stat().st_size,
                "mtime": int(b.stat().st_mtime),
                "ver": ver,
            })
        versions.sort(key=lambda v: v["ver"], reverse=True)
        for v in versions:
            v.pop("ver")
        body = json.dumps({"versions": versions}, indent=2).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def my_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()


if __name__ == "__main__":
    os.chdir(HERE)
    ip = my_lan_ip()
    print(f"Serving {HERE}")
    print(f"Manifest:   http://{ip}:{PORT}/manifest.json")
    print(f"On chip:    set manifest URL to that ^")
    print(f"Drop new RXV2-*.bin files in this folder; chip will see them on next reload.")
    print()
    with socketserver.ThreadingTCPServer(("", PORT), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nbye")
