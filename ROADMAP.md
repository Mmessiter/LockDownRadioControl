# LDRC Roadmap — where we are

_Last updated: 2026-05-26_

## Right now: **Receiver V2 (RXV2)**

We are building **RXV2**, the next-generation receiver.

- **Hardware:** Seeed XIAO **ESP32-S3** + EBYTE ML01DP5 / SP4 (nRF24L01+ clone). C3 build envs retained only for the original prototypes — production is S3.
- **Compatibility:** RXV2 talks to the **existing v1 transmitter** unchanged. The v1 TX (Teensy 4.1, firmware 2.5.x L/R) must not require recompilation to bind and fly an RXV2. Every RXV2 protocol decision is gated on "does this still work with an un-modified v1 TX?".
- **Source tree:** `ReceiverCode/RXV2/` (PlatformIO, default env `xiao_s3_ota`).
- **OTA upload:** `pio run -e xiao_s3_ota -t upload` — receiver must be on WiFi and reachable at `LDRC_RX.local:3232`. If mDNS fails, a power-cycle on the receiver usually fixes it.

## Hard rules while RXV2 is in flight

1. **Don't edit `TransmitterCode/`** to fix an RXV2 problem. The v1 TX is the fixed reference; the RXV2 conforms to it. Treat the TX side as read-only unless Malcolm explicitly says otherwise.
2. **Ack-payload layout** must mirror v1 (`ReceiverCode/src/utilities/radio.h::LoadAckPayload`). Slot 0 = version, slot 1 = packet count, etc. New telemetry items go into unused slots, not by remapping existing ones.
3. **Ack slots 0/1 use a bind-protect window — don't break it either way.** The v1 TX's `ParseAckPayload` runs `GetModelsMacAddress` on **every** packet while `!ModelMatched && !LedWasGreen` (transceiver.h:1224), reading slot 0 → `ModelsMacUnion.Val32[0]` and slot 1 → `Val32[1]`. The MAC-ack phase (first 20 acks via `macAcksSent < MAC_ACK_THRESHOLD`) is **not** sufficient on its own — the telemetry rotation must also carry the real MAC on slots 0/1 until the TX has resolved its pre-match state. We can't observe that state from the RX, so the working design (in `ReceiverCode/RXV2/src/Radio.h`) uses a 5-second continuous-traffic timer: while traffic is fresh-but-young (<5 s since last >500 ms gap), slots 0/1 carry `boardMac[0..3]`/`boardMac[4..7]`; after that window they carry firmware version + `rx.packets` (mirroring v1 `LoadAckPayload`). **Do NOT "simplify" this to either extreme**: "MAC forever" leaves the TX's RX-firmware field showing garbage (Malcolm hates it); "version forever" corrupts the saved model ID at bind. Both have been tried — both are wrong. This bug has bitten 3+ times; the windowed solution is the settled answer (see `feedback_rxv2_slot0_mac` memory).
4. Builds must succeed under PlatformIO; OTA artifacts are auto-archived by `dev/archive_firmware.py`.

## Next, once RXV2 is settled: **Transmitter V2 (TXV2)**

Not started. Will be a separate effort — likely a new MCU + display, dropping the Teensy + Nextion combo. Out of scope until Malcolm declares RXV2 finished.

## Where the detail lives

- `~/.claude/projects/-Users-malcolmmessiter-Documents-GitHub-LockDownRadioControl/memory/` — auto-loaded notes (RXV2 design, chip plan, bind handshake, MSP bridge plan, RF version pinning, etc.).
- `ReceiverCode/RXV2/src/` — RXV2 firmware modules.
- `ReceiverCode/src/utilities/radio.h` — v1 receiver, the reference protocol implementation.
- `TransmitterCode/include/transceiver.h` — v1 TX ack-payload parsing (read-only while RXV2 is in flight).
