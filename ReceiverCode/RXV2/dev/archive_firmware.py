"""
PlatformIO post-build script: copy every successful firmware build into the
dev/ folder (alongside this file) with the FW_VERSION string as the filename.
The dev firmware server picks it up automatically.

Wired into platformio.ini via:
    extra_scripts = post:dev/archive_firmware.py
"""
import re
import shutil
from pathlib import Path

Import("env", "projenv")  # type: ignore  # injected by PlatformIO

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))   # type: ignore
HERE        = PROJECT_DIR / "dev"
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


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", archive)  # type: ignore
