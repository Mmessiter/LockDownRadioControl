# RXV2 public release mirror (messiter.com)

This directory is the FTP-upload staging area for the public RXV2
firmware mirror at `https://messiter.com/rxv2/release/`. The chip's
`/api/firmware/check` endpoint fetches `manifest.json` here in addition
to the user-configured local dev server, so any receiver with internet
access can install official releases without the dev Mac being on the
LAN.

## Layout

```
public_html/
└── rxv2/
    └── release/
        ├── manifest.json          # version list — newest entries first
        └── v0.9.33/
            └── firmware.bin       # the .bin matching the entry's `name`
```

Each version entry in `manifest.json` is:
```json
{
  "name":  "RXV2-x.y.z-tag",                              // matches FW_VERSION
  "url":   "https://messiter.com/rxv2/release/vX.Y.Z/firmware.bin",
  "notes": "Optional one-paragraph human-readable change summary."
}
```

The chip's `firmware.html` page parses `name` (`RXV2-(\d+)\.(\d+)\.(\d+)`)
to decide whether each entry is newer / current / older than what's
running, and resolves `url` against the manifest's location (so it
accepts both absolute https URLs and relative paths).

## Release workflow

1. Build the firmware: `pio run -e xiao_s3_ota`.
2. Copy `.pio/build/xiao_s3_ota/firmware.bin` into a new `vX.Y.Z/`
   directory under `public_html/rxv2/release/`.
3. Prepend a new entry to `manifest.json` with the matching `name`,
   `url`, and a one-paragraph `notes` summary.
4. Upload `public_html/rxv2/` to messiter.com via FTP, preserving the
   directory layout (FileZilla: drag-drop the `public_html` contents
   into the site root).

The receiver picks the new release up on the next visit to its
`/firmware` page — no separate "publish" step on the chip.
