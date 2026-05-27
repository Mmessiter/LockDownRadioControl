"""
PlatformIO post-build script: copy every successful firmware build into the
dev/ folder (alongside this file) with the FW_VERSION string as the filename.

Also mirrors each build into the runtime directory the auto-launched dev
firmware server reads from. We can't run that server out of ~/Documents
because macOS TCC blocks launchd-spawned processes from reading
~/Documents — so the LaunchAgent serves from /Users/Shared/rxv2-firmware-server/
instead and we keep it in sync from here.

Wired into platformio.ini via:
    extra_scripts = post:dev/archive_firmware.py
"""
import re
import shutil
from pathlib import Path

Import("env", "projenv")  # type: ignore  # injected by PlatformIO

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))   # type: ignore
HERE        = PROJECT_DIR / "dev"
RUNTIME_DIR = Path("/Users/Shared/rxv2-firmware-server")
DEFS_H      = PROJECT_DIR / "src" / "1Defs.h"
VERSION_RE  = re.compile(r'FW_VERSION\s*=\s*"([^"]+)"')


def archive(source, target, env):
    src = Path(str(target[0]))   # the just-built .bin
    if not src.exists():
        return
    text = DEFS_H.read_text(errors="ignore")
    m = VERSION_RE.search(text)
    if not m:
        print("[archive] FW_VERSION not found in 1Defs.h — skipping")
        return
    version = m.group(1)
    dest = HERE / f"{version}.bin"
    shutil.copy2(src, dest)
    print(f"[archive] {dest.name} ({dest.stat().st_size} bytes)")

    # Also mirror into the LaunchAgent's runtime directory if it exists.
    # Skips silently when the agent isn't installed (CI / fresh checkouts).
    if RUNTIME_DIR.is_dir():
        runtime_dest = RUNTIME_DIR / dest.name
        shutil.copy2(src, runtime_dest)
        print(f"[archive] mirrored → {runtime_dest}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", archive)  # type: ignore
