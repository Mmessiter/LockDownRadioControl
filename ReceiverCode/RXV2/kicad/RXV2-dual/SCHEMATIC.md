# RXV2 dual-radio — schematic specification

Same as the single-radio board plus a second E01-ML01SP4 module that shares
the SPI bus but has its own CE/CSN lines on D0/D1. The firmware
auto-detects the second radio at boot.

## Components

| Ref | Symbol | Value | Footprint | Notes |
|---|---|---|---|---|
| **U1** | `RXV2:XIAO_ESP32_S3` | XIAO_ESP32_S3 | Seeed XIAO module (14-pad castellated) | The microcontroller. |
| **U2** | `RXV2:E01_ML01SP4` | E01-ML01SP4 | EBYTE module footprint | Radio 1 — primary. |
| **U3** | `RXV2:E01_ML01SP4` | E01-ML01SP4 | EBYTE module footprint | Radio 2 — hot standby. |
| **C1** | `Device:C` | 100nF | `Capacitor_SMD:C_0603_1608Metric` | HF decoupling at U2 VCC. ≤ 2 mm. |
| **C2** | `Device:C` | 10µF | `Capacitor_SMD:C_0805_2012Metric` | Bulk decoupling at U2 VCC. |
| **C3** | `Device:C` | 100nF | `Capacitor_SMD:C_0603_1608Metric` | HF decoupling at U3 VCC. ≤ 2 mm. |
| **C4** | `Device:C` | 10µF | `Capacitor_SMD:C_0805_2012Metric` | Bulk decoupling at U3 VCC. |
| **J1** | `Connector_Generic:Conn_01x04` | Conn_01x04 | `Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical` | FC connector. |
| **J2** | `Connector_Generic:Conn_01x02` | Conn_01x02 | `Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical` | Optional spare-pad header — D4, D9. Mark DNP if not populating. |

## Power nets

| Net | Source | Sinks |
|---|---|---|
| **+3V3** | U1 pin 3 (3V3) | U2 pin 2, U3 pin 2, C1 (+), C2 (+), C3 (+), C4 (+) |
| **GND** | U1 pin 2 (GND) | U2 pin 1, U3 pin 1, C1 (-), C2 (-), C3 (-), C4 (-), J1 pin 1, J2 pin 1 |
| **+5V** | U1 pin 1 (5V) | J1 pin 2 |

Add `PWR_FLAG` on `+3V3`, `+5V`, and `GND`.

## Signal nets

| Net | XIAO pin | nRF24 pin | Other | Notes |
|---|---|---|---|---|
| `SPI_SCK` | D8 (pin 6) | U2 SCK (pin 5), U3 SCK (pin 5) | — | Shared SPI clock. |
| `SPI_MISO` | D7 (pin 7) | U2 MISO (pin 7), U3 MISO (pin 7) | — | Shared SPI MISO. |
| `SPI_MOSI` | D10 (pin 4) | U2 MOSI (pin 6), U3 MOSI (pin 6) | — | Shared SPI MOSI. |
| `RF1_CE` | D2 (pin 10) | U2 CE (pin 3) | — | Radio 1 chip-enable. |
| `RF1_CSN` | D3 (pin 11) | U2 CSN (pin 4) | — | Radio 1 chip-select. |
| `RF2_CE` | D0 (pin 8) | U3 CE (pin 3) | — | **Radio 2 chip-enable.** |
| `RF2_CSN` | D1 (pin 9) | U3 CSN (pin 4) | — | **Radio 2 chip-select.** |
| `FC_TX` | D6 (pin 14) | — | J1 pin 3 | RC output. |
| `FC_RX` | D5 (pin 13) | — | J1 pin 4 | Telemetry input. |
| `IRQ_NC_1` | — | U2 IRQ (pin 8) | NC flag | Mark with `NC`. |
| `IRQ_NC_2` | — | U3 IRQ (pin 8) | NC flag | Mark with `NC`. |

## Spare pads (route to J2 if desired)

| XIAO pin | Net | Use |
|---|---|---|
| D4 (pin 12) | `SPARE_D4` | ADC1_CH4 capable on S3. Useful for analog sensors. |
| D9 (pin 5) | `SPARE_D9` | Plain GPIO on S3 (was forbidden on C3). |

On the dual board these are the only true spares — D0/D1 are now Radio 2.

## Layout note specific to dual-radio

Place U2 and U3 with their **antenna ends pointing in different directions**
— either opposite (180°) or perpendicular (90°). Don't put them parallel
and close together; their antennas will couple/shadow each other on
certain orientations and you lose the diversity benefit of having two
radios.

Other than that, follow the single-radio layout guidance — `../DESIGN_NOTES.md`.

## ERC checklist

- `PWR_FLAG`s on all three power nets.
- `NC` flags on both U2 and U3 IRQ pins.
- No dangling pins anywhere.

## Step-by-step build

Same as the single-radio version, with these additions in step 3:

3a. **Place U3** below or beside U2 (you'll move them later in PCB).
3b. **Place C3 and C4** near U3.
3c. **Wire RF2_CE (D0 → U3 CE)** and **RF2_CSN (D1 → U3 CSN)**.
3d. **Branch the SPI bus** — each of SCK / MISO / MOSI connects to both
    U2 and U3. In Eeschema this is just a junction (`J` to add a junction
    explicitly, or it auto-creates when two wires cross).
