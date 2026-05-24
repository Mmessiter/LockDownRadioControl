# RXV2 PCB — KiCad starter project

Starter files for laying out the RXV2 receiver in [KiCad 9](https://www.kicad.org/download/).
Two board variants share the same symbol library:

- **`RXV2-single/`** — production single-radio board (1 × E01-ML01SP4)
- **`RXV2-dual/`** — dual-radio board with swap-on-loss redundancy (2 × E01-ML01SP4)

The firmware in this repo (`src/`) auto-detects which variant it's running on,
so the same `.bin` works on either.

## What's in here

```
kicad/
├── README.md                ← this file
├── DESIGN_NOTES.md          ← layout/RF/decoupling guidance — read before laying out
├── symbols/
│   └── RXV2.kicad_sym       ← custom symbols for XIAO ESP32-S3 and E01-ML01SP4 module
├── RXV2-single/
│   ├── RXV2-single.kicad_pro
│   ├── RXV2-single.kicad_sch    ← starter schematic (open in Eeschema)
│   ├── SCHEMATIC.md             ← textual schematic spec — authoritative reference
│   └── BOM.md                   ← parts list with part numbers + datasheet links
└── RXV2-dual/
    ├── RXV2-dual.kicad_pro
    ├── RXV2-dual.kicad_sch
    ├── SCHEMATIC.md
    └── BOM.md
```

## Open in KiCad

1. Install **KiCad 9** (free, runs on Windows / macOS / Linux).
2. **File → Open Project** → pick either `RXV2-single/RXV2-single.kicad_pro` or
   `RXV2-dual/RXV2-dual.kicad_pro`.
3. KiCad will offer to associate the custom symbol library
   (`symbols/RXV2.kicad_sym`) with the project automatically. If it doesn't:
   **Preferences → Manage Symbol Libraries → Project tab → Add**, point to
   `symbols/RXV2.kicad_sym`, nickname `RXV2`.
4. Double-click the `.kicad_sch` to open the schematic in Eeschema.

## If the schematic doesn't look right

The `SCHEMATIC.md` files in each subfolder are the **authoritative reference**.
They list every component, every net, and every pin connection in plain
English. If anything in the `.kicad_sch` looks wrong or KiCad complains, work
from `SCHEMATIC.md` and draw the schematic yourself — Eeschema's graphical
editor is the friendliest path for someone new to KiCad anyway.

That's also the recommended **learning path**: open a blank schematic, place
the symbols from the `RXV2` library, wire them up by following the
component-by-component spec in `SCHEMATIC.md`. You'll touch every wire and
learn the tool properly.

## Reading order

1. `DESIGN_NOTES.md` — what the board needs to do well (RF, decoupling,
   thermals, mechanical) — context before you place anything.
2. `RXV2-single/SCHEMATIC.md` — start here; simpler board.
3. `RXV2-single/BOM.md` — parts to order while you're laying out.
4. `RXV2-dual/SCHEMATIC.md` — once you're comfortable with the single, the
   dual adds one extra module + two CE/CSN lines.
