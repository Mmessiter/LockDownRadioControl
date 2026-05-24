# RXV2 single-radio — Bill of Materials

Quantities are **per board**. All passive packages are SMD.

| Ref | Qty | Part | Package | Mouser / LCSC | Datasheet | Why this part |
|---|---|---|---|---|---|---|
| **U1** | 1 | Seeed XIAO ESP32-S3 | XIAO 14-pad castellated, 21 × 17.5 mm | Seeed direct, [Mouser 713-113991114](https://www.mouser.com/ProductDetail/713-113991114) | [wiki.seeedstudio.com/xiao_esp32s3_getstarted](https://wiki.seeedstudio.com/xiao_esp32s3_getstarted/) | Production chip. Dual-core 240 MHz, 512 KB SRAM, 4 MB flash, USB-C. |
| **U2** | 1 | EBYTE E01-ML01SP4 | SMD nRF24L01+PA+LNA module | LCSC [C2837987](https://lcsc.com/product-detail/C2837987.html), AliExpress | [ebyte.com/en/product-view-news.html?id=159](https://www.ebyte.com/en/product-view-news.html?id=159) | 100 mW PA + LNA, ~1 km range with whip. SMD version of the through-hole DP5 used in prototype. |
| **C1** | 1 | 100 nF ceramic, 50 V, X7R | 0603 | Murata GRM188R71H104KA93 (Mouser 81-GRM188R71H104KA93D), LCSC C14663 | — | HF decoupling. Place within 2 mm of U2 VCC pin. |
| **C2** | 1 | 10 µF ceramic, 16 V, X5R | 0805 (or 1206) | Murata GRM21BR61C106KE15 (Mouser 81-GRM21BR61C106KE5L), LCSC C15850 | — | Bulk decoupling. Same proximity rule as C1. Tantalum (e.g. AVX TAJ) also fine if you prefer. |
| **J1** | 1 | JST-SH 4-pin top-entry | JST_SH_BM04B-SRSS-TB | JST [BM04B-SRSS-TB](https://www.mouser.com/ProductDetail/JST/BM04B-SRSS-TB-LF-SN) (Mouser 798-BM04B-SRSS-TB) | [jst.com](https://www.jst-mfg.com/product/pdf/eng/eSH.pdf) | FC connector. Matches CRSF cable order: GND, 5V, RC-out, telemetry-in. |
| **J2** | 0–1 | 4-pin 2.54 mm pin header | PinHeader_1x04_P2.54mm | Generic | — | **Optional** spare-pad header (D0, D1, D4, D9). Mark DNP if not populating. |

## Approximate cost

| | Each |
|---|---|
| XIAO ESP32-S3 | ~$7.50 |
| E01-ML01SP4 | ~$5–6 |
| C1, C2, J1 | <$1 combined |
| **PCB fab (10 pcs at JLCPCB)** | ~$5/10 = $0.50 each |
| **Board BOM** | **~$13–14** |

For a batch of 20 boards: ~**$280** in parts + $5 in PCB fab.

## Antenna

The E01-ML01SP4 has an onboard PCB antenna and an optional IPEX (U.FL)
connector for an external antenna. For most installations the onboard
antenna is sufficient at 250 kbps with PA enabled; if you want extended
range, fit a 2.4 GHz whip via IPEX.

## Solder paste / assembly notes

- The XIAO is hand-solderable via the castellated pads.
- The E01-ML01SP4 is also castellated — hand-solder with a fine tip and
  flux paste.
- Passives are reflow-friendly; 0603 is generous for hand work too.

If outsourcing assembly (JLCPCB SMT, PCBWay, etc.), the XIAO and
E01-ML01SP4 are usually marked "do not assemble" — they're either hand-soldered
or supplied as customer-fitted. JLCPCB will assemble C1, C2, J1.
