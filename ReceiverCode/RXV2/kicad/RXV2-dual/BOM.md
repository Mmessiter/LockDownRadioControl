# RXV2 dual-radio — Bill of Materials

Quantities are **per board**. Same as the single-radio BOM plus one extra
E01-ML01SP4 module and its decoupling pair.

| Ref | Qty | Part | Package | Mouser / LCSC | Datasheet | Why this part |
|---|---|---|---|---|---|---|
| **U1** | 1 | Seeed XIAO ESP32-S3 | XIAO 14-pad castellated | Seeed direct, [Mouser 713-113991114](https://www.mouser.com/ProductDetail/713-113991114) | [wiki.seeedstudio.com/xiao_esp32s3_getstarted](https://wiki.seeedstudio.com/xiao_esp32s3_getstarted/) | Production chip. |
| **U2, U3** | **2** | EBYTE E01-ML01SP4 | SMD nRF24L01+PA+LNA | LCSC [C2837987](https://lcsc.com/product-detail/C2837987.html) | [ebyte.com/en/product-view-news.html?id=159](https://www.ebyte.com/en/product-view-news.html?id=159) | Primary + hot-standby radio. Firmware auto-detects U3 at boot. |
| **C1, C3** | **2** | 100 nF ceramic, 50 V, X7R | 0603 | Murata GRM188R71H104KA93, LCSC C14663 | — | HF decoupling — one per radio, ≤ 2 mm from VCC. |
| **C2, C4** | **2** | 10 µF ceramic, 16 V, X5R | 0805 | Murata GRM21BR61C106KE15, LCSC C15850 | — | Bulk decoupling — one per radio. |
| **J1** | 1 | JST-SH 4-pin top-entry | JST_SH_BM04B-SRSS-TB | Mouser 798-BM04B-SRSS-TB | — | FC connector. |
| **J2** | 0–1 | 2-pin 2.54 mm pin header | PinHeader_1x02_P2.54mm | Generic | — | **Optional** spare-pad header (D4, D9). Mark DNP if not populating. |

## Approximate cost

| | Each |
|---|---|
| XIAO ESP32-S3 | ~$7.50 |
| 2 × E01-ML01SP4 | ~$10–12 |
| C1–C4, J1 | <$1.50 combined |
| **PCB fab** | ~$0.50 each |
| **Board BOM** | **~$19–21** |

For a batch of 20 dual-radio boards: ~**$400** in parts + $5 in PCB fab.

## Antenna placement (critical for dual)

The whole point of two radios is **antenna diversity** — when one antenna
is in a polarisation null (model nose-down, antenna line-of-sight blocked,
etc.) the other is in a different orientation and still hears the TX.
The firmware swaps between radios on packet loss.

For this to actually work the two antennas must be in **different
orientations**. Either:

- **180° opposed** — antennas point in opposite directions along one PCB axis
- **90° perpendicular** — antennas at right angles to each other

Don't place them parallel and close together. They'll shadow each other
and you lose all the diversity benefit (and waste an entire radio module).

If you're using external antennas via IPEX, this is even easier — mount
the two whips at 90° via flex coax.

## Notes on populating partially

If you build the dual-radio PCB but only populate U2 (leave U3 + C3 + C4
empty), the firmware will detect this at boot (`radio2.begin()` returns
false) and run in single-radio mode. So the dual-radio PCB is a strict
superset of the single — you can stuff fewer boards as singles if you run
out of E01-ML01SP4 modules during a build session.
