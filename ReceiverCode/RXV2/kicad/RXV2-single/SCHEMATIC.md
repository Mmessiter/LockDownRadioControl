# RXV2 single-radio — schematic specification

Build this in Eeschema by placing the components below and connecting them
per the netlist table. The custom `RXV2` symbol library (configured in
`sym-lib-table`) provides the XIAO and E01-ML01SP4 symbols; everything else
is in KiCad's stock libraries.

## Components

| Ref | Symbol | Value | Footprint | Notes |
|---|---|---|---|---|
| **U1** | `RXV2:XIAO_ESP32_S3` | XIAO_ESP32_S3 | Seeed XIAO module (14-pad castellated) | The microcontroller. |
| **U2** | `RXV2:E01_ML01SP4` | E01-ML01SP4 | EBYTE module footprint | Radio 1 — primary. |
| **C1** | `Device:C` | 100nF | `Capacitor_SMD:C_0603_1608Metric` | HF decoupling at U2 VCC. **Place ≤ 2 mm from U2's VCC pin.** |
| **C2** | `Device:C` | 10µF | `Capacitor_SMD:C_0805_2012Metric` (or 1206) | Bulk decoupling at U2 VCC. Same proximity rule as C1. |
| **J1** | `Connector_Generic:Conn_01x04` | Conn_01x04 | `Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical` | FC connector (or your preferred 4-pin layout). |
| **J2** | `Connector_Generic:Conn_01x04` | Conn_01x04 | `Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical` | Optional spare-pad header — D0, D1, D4, D9. Mark `DNP` if not populating. |

## Power nets

| Net | Source | Sinks |
|---|---|---|
| **+3V3** | U1 pin 3 (3V3) | U2 pin 2 (VCC_3V3), C1 (+), C2 (+), J2 pin 2 (optional) |
| **GND** | U1 pin 2 (GND) | U2 pin 1 (GND), C1 (-), C2 (-), J1 pin 1, J2 pin 1 |
| **+5V** | U1 pin 1 (5V) | J1 pin 2 (FC 5V rail, optional input from BEC) |

Add a `PWR_FLAG` on each of `+3V3`, `+5V`, and `GND` so KiCad's ERC doesn't
complain about un-driven power nets.

## Signal nets

| Net | XIAO pin | nRF24 pin (U2) | Other | Notes |
|---|---|---|---|---|
| `SPI_SCK` | D8 (pin 6) | SCK (pin 5) | — | Shared SPI clock. |
| `SPI_MISO` | D7 (pin 7) | MISO (pin 7) | — | Shared SPI MISO. **Must not be D9** — D9 is the C3 BOOT strap (not used on S3 but PCB stays portable). |
| `SPI_MOSI` | D10 (pin 4) | MOSI (pin 6) | — | Shared SPI MOSI. |
| `RF1_CE` | D2 (pin 10) | CE (pin 3) | — | Radio 1 chip-enable. |
| `RF1_CSN` | D3 (pin 11) | CSN (pin 4) | — | Radio 1 chip-select. |
| `FC_TX` | D6 (pin 14) | — | J1 pin 3 | RC output to flight controller. |
| `FC_RX` | D5 (pin 13) | — | J1 pin 4 | Telemetry input from flight controller. |
| `IRQ_NC` | — | IRQ (pin 8) | (No connect) | Mark with a `NC` flag — firmware masks all IRQs. |

## Spare pads (route to J2 header if desired)

| XIAO pin | Net | Use |
|---|---|---|
| D0 (pin 8) | `SPARE_D0` | ADC1_CH0 capable on S3 (GPIO 1). |
| D1 (pin 9) | `SPARE_D1` | Plain GPIO (GPIO 2). |
| D4 (pin 12) | `SPARE_D4` | ADC1_CH4 capable on S3 (GPIO 5). Best of the spares for sensors. |
| D9 (pin 5) | `SPARE_D9` | Plain GPIO on S3 (GPIO 8). Was forbidden on C3 (BOOT strap); now safe. |

You can route any subset of these to J2. If you don't populate J2, mark it
DNP so the BOM excludes it.

## ERC checklist (after wiring)

- All `PWR_FLAG`s placed (one each on `+3V3`, `+5V`, `GND`).
- `NC` flag on U2's IRQ pin.
- No unconnected pins on U1 except those you've intentionally left for
  future expansion — for those, place `NC` flags too.
- Run **Schematic → Inspect → Electrical Rules Checker**. Fix any errors
  before going to PCB.

## Step-by-step build order in Eeschema

1. **Open** `RXV2-single.kicad_sch`. You'll get a blank A4 sheet.
2. **Place U1** — press `A`, search "XIAO_ESP32_S3" in the RXV2 library,
   place near the centre-left.
3. **Place U2** — press `A`, search "E01_ML01SP4", place to the right of U1.
4. **Place power flags** — press `A`, search `PWR_FLAG`. Place three.
5. **Place caps** — press `A`, search `C`. Place C1 and C2 near U2.
6. **Place connectors** — press `A`, search `Conn_01x04`. Place J1 and J2.
7. **Wire it up** — press `W` and follow the net tables above. Use **global
   labels** (`Place → Global Label`, shortcut `Ctrl+L`) on long-distance
   connections (SPI, FC_TX/RX) to avoid spaghetti. Power nets can use
   `+3V3`, `+5V`, `GND` from the Power library.
8. **Annotate** — `Tools → Annotate Schematic` (auto-assigns refs U1, U2...).
9. **Assign footprints** — `Tools → Assign Footprints`. Set each per the
   table.
10. **Run ERC** — fix all errors.
11. **Generate netlist** — needed before you can lay out the PCB.
12. **Switch to PCB editor** — `Tools → Switch to PCB Editor` or open the
   `.kicad_pcb` file (KiCad creates it automatically on first open).

Layout guidance is in `../DESIGN_NOTES.md` — read it before placing
components on the PCB.
