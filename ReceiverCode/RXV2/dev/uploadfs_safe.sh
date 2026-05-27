#!/usr/bin/env bash
# Wrap `pio run -t uploadfs` with a backup preserve dance.
#
# The LittleFS partition gets wiped during uploadfs, taking every
# Rotorflight backup the user has built up with it. This script:
#
#   1. Pulls every /backups/*.json off the chip into dev/.backup-cache/
#   2. Runs pio run -e xiao_s3_ota -t uploadfs
#   3. Waits for the chip to reboot
#   4. Pushes the cached backups back to the chip
#
# Run from anywhere — paths are anchored to the script's own location.

set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$HERE/.." && pwd)"

ENV="${PIO_ENV:-xiao_s3_ota}"

echo "==> Preserving backups from the chip"
python3 "$HERE/preserve_backups.py" pull || \
    echo "==> WARNING: backup pull had errors — continuing anyway"

echo "==> Running pio run -e $ENV -t uploadfs"
(cd "$PROJECT_DIR" && pio run -e "$ENV" -t uploadfs)

echo "==> Restoring backups to the chip"
python3 "$HERE/preserve_backups.py" push

echo "==> Done."
