#!/usr/bin/env python3
"""
Preserve the receiver's Rotorflight backups across a `pio run -t uploadfs`.

The LittleFS partition is replaced wholesale by `uploadfs`, so every
file the chip has accumulated under /backups/ is wiped. The user has to
manually re-create their tuning snapshots — painful.

Use this in two steps around uploadfs:

    python3 preserve_backups.py pull    # before uploadfs
    pio run -e xiao_s3_ota -t uploadfs
    python3 preserve_backups.py push    # after the chip reboots

Or run `dev/uploadfs_safe.sh` which wraps the whole dance.

Local cache lives at dev/.backup-cache/<chip-host>/<name>.json so
multiple receivers can be served from the same dev tree.
"""
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import os
# Target host: CHIP_HOST env var (set by uploadfs_safe.sh from PLATFORMIO_UPLOAD_PORT)
# falls back to the legacy default. Stops backups being pulled from the wrong chip.
CHIP_HOST   = os.environ.get("CHIP_HOST", "LDRC_RX.local")
CHIP_BASE   = f"http://{CHIP_HOST}"
CACHE_ROOT  = Path(__file__).resolve().parent / ".backup-cache" / CHIP_HOST
REQ_TIMEOUT = 8.0


def fetch(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=REQ_TIMEOUT) as r:
        return r.read()


def wait_for_chip(max_wait: float = 60.0) -> bool:
    """Poll /api/state.json until the chip answers (post-reboot grace)."""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        try:
            fetch(f"{CHIP_BASE}/api/state.json")
            return True
        except (urllib.error.URLError, TimeoutError, OSError):
            time.sleep(2.0)
    return False


def pull() -> int:
    if not wait_for_chip(15):
        print(f"[preserve] chip {CHIP_HOST} unreachable — no backups pulled", file=sys.stderr)
        return 1
    try:
        listing = json.loads(fetch(f"{CHIP_BASE}/api/backup/list"))
    except Exception as e:
        print(f"[preserve] /api/backup/list failed: {e}", file=sys.stderr)
        return 1
    names = listing.get("names", [])
    if not names:
        print("[preserve] chip has no backups — nothing to preserve")
        return 0

    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    saved = 0
    for n in names:
        try:
            body = fetch(f"{CHIP_BASE}/api/backup/load?name={urllib.parse.quote(n)}")
            (CACHE_ROOT / f"{n}.json").write_bytes(body)
            saved += 1
            print(f"[preserve] pulled '{n}' ({len(body)} bytes)")
        except Exception as e:
            print(f"[preserve] WARNING: pulling '{n}' failed: {e}", file=sys.stderr)
    print(f"[preserve] {saved}/{len(names)} backups cached under {CACHE_ROOT}")
    return 0 if saved == len(names) else 1


def push() -> int:
    if not CACHE_ROOT.is_dir():
        print(f"[preserve] no cache at {CACHE_ROOT} — nothing to restore")
        return 0
    files = sorted(CACHE_ROOT.glob("*.json"))
    if not files:
        print(f"[preserve] cache empty — nothing to restore")
        return 0
    # The chip needs an extra moment after uploadfs reboot for the
    # web server to actually accept connections — wait longer here.
    if not wait_for_chip(90):
        print(f"[preserve] chip {CHIP_HOST} unreachable after wait — backups NOT restored", file=sys.stderr)
        return 1
    restored = 0
    for path in files:
        name = path.stem
        body = path.read_bytes()
        url  = f"{CHIP_BASE}/api/backup/save?name={urllib.parse.quote(name)}"
        req  = urllib.request.Request(url, data=body, method="POST",
                                      headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=REQ_TIMEOUT) as r:
                if r.status == 200:
                    restored += 1
                    print(f"[preserve] pushed '{name}' ({len(body)} bytes)")
                else:
                    print(f"[preserve] WARNING: '{name}' returned HTTP {r.status}", file=sys.stderr)
        except Exception as e:
            print(f"[preserve] WARNING: pushing '{name}' failed: {e}", file=sys.stderr)
    print(f"[preserve] {restored}/{len(files)} backups restored")
    return 0 if restored == len(files) else 1


def main() -> int:
    if len(sys.argv) != 2 or sys.argv[1] not in ("pull", "push"):
        print(__doc__.strip(), file=sys.stderr)
        return 2
    return pull() if sys.argv[1] == "pull" else push()


if __name__ == "__main__":
    sys.exit(main())
