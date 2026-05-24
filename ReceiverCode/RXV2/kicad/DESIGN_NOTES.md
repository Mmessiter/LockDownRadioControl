# RXV2 PCB design notes

Read this before you start placing components. These are the few things that
genuinely matter for a board this small. Most of the rest is straightforward
two-layer routing.

## Mechanical envelope

- **Recommended outline**: ~25 × 35 mm. Smaller is fine for single-radio;
  dual needs a little more room.
- **XIAO module orientation**: keep the USB-C end at one edge of the PCB and
  **leave that edge unobstructed** — you'll need clear USB access for first
  flashing, OTA failure recovery, and serial debug.
- **Mounting holes**: 2.5 mm holes at the corners are typical for small RX
  boards. If you don't have a mechanical constraint, skip them and use
  servo-tape mounting.
- **Bottom-side components**: avoid. Keep one side clean for mounting tape.

## Power

- The XIAO has its own 3V3 LDO that can source ~700 mA — easily enough for
  one or two E01-ML01SP4 modules (peak ~120 mA per module during TX).
- **Input**: either USB-C (via the XIAO) **or** 5 V on the XIAO's `5V` pad
  (from a BEC). Don't apply both at once unless you've added diode protection
  — the XIAO doesn't have a power MUX on its 5V pad.
- The board doesn't need its own regulator. The XIAO is the regulator.

## Decoupling — non-negotiable

For **each** nRF24 module, place these as close to the module's VCC/GND pins
as physically possible (within 2 mm if you can):

- **10 µF** — bulk decoupling. Tantalum (TAJA106M016R or similar) or X5R/X7R
  MLCC (0805 or 1206). The MLCC is fine and smaller.
- **100 nF** — high-frequency decoupling. X7R MLCC, 0603 package.

The E01-ML01SP4 module *does* have onboard decoupling, but the trace from
the XIAO's 3V3 regulator to the module is several centimetres and induces
enough inductance to cause TX-current sag. The bulk cap eliminates that.

## RF layout — the only RF-critical part

1. **Antenna keep-out zone** — no copper (signal traces, ground pour, or
   silkscreen) within ~5 mm of the antenna end of the E01-ML01SP4 module.
   The module datasheet shows the antenna feed area; treat the area beyond
   it (off the PCB) as the radiating volume.
2. **Ground pour** under everything *except* the antenna keep-out zone.
   Stitch the two layers' grounds together with vias every ~5 mm.
3. **SPI traces** (SCK / MISO / MOSI) — keep them tight together but away
   from the antenna. They're 4 MHz; not RF-critical themselves, but you
   don't want them coupling into the antenna.
4. **CE / CSN traces** — slow digital, no constraint other than reasonable
   length.

For the **dual-radio** variant: place the two E01-ML01SP4 modules with
their antennas pointing in **opposite directions** or **90° apart**. This
gives polarisation diversity. Don't put them parallel and close — they'll
shadow each other on certain orientations.

## SPI bus length

The shared SPI bus runs from the XIAO to one (or two) radio modules. At
4 MHz this is forgiving — keep it under ~5 cm total and you'll have no
signal integrity issues. Don't snake it; straight runs are easier to debug.

## Connectors

- **FC connector** (recommended): JST-SH 4-pin or compatible. Pinout chosen
  to match the standard CRSF cable order so off-the-shelf cables work:
  `GND, 5V, D6 (RC out), D5 (telemetry in)`. If you'd rather use a 3-pin
  servo connector for SBUS-only use cases, omit D5.
- **Spare-pad header** (optional): break out the free GPIO pads to a
  2.54 mm header — useful for future sensors / debug. Single-radio variant
  has D0, D1, D4, D9 free; dual has D4, D9.

## Silkscreen and identification

- Print the firmware-required pad labels on the silkscreen: `D2 CE1`,
  `D3 CSN1`, `D8 SCK`, `D7 MISO`, `D10 MOSI`, etc. — anyone debugging
  later (including you in six months) will thank you.
- Print the **board version** and **single/dual variant** on the silkscreen
  so you don't mix them up in a parts drawer.

## Layer count and stack-up

Two layers is enough. Standard fab stack-up (JLCPCB, PCBWay, Aisler):

- **Layer count**: 2
- **Thickness**: 1.6 mm (standard); 0.8 mm if you want lighter
- **Copper weight**: 1 oz (default)
- **Soldermask**: any colour; matte black looks pro on flying gear
- **Surface finish**: HASL (cheap) is fine; ENIG (gold) if you can spare ~$5
- **Silkscreen**: white on dark mask; black on green/white mask

## What NOT to put on the board

- A separate 3V3 regulator (XIAO has one)
- A reset button (XIAO has one on its own; use it)
- An external user LED on D4 (firmware uses the XIAO's onboard LED now —
  D4 is freed as a spare ADC pin)
- A boot-mode strap circuit (XIAO handles it)
- Pull-ups on SPI or CSN (firmware sets internal pulls where needed)

Keep it minimal. Less to go wrong, smaller board, cheaper assembly.
