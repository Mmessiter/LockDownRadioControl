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
3. **MAC delivery** uses the dedicated MAC-ack phase (first 20 acks via `macAcksSent < MAC_ACK_THRESHOLD`), exactly as v1 does. The telemetry rotation does **not** carry MAC bytes.
4. Builds must succeed under PlatformIO; OTA artifacts are auto-archived by `dev/archive_firmware.py`.

## Next, once RXV2 is settled: **Transmitter V2 (TXV2)**

Not started. Will be a separate effort — likely a new MCU + display, dropping the Teensy + Nextion combo. Out of scope until Malcolm declares RXV2 finished.

## Where the detail lives

- `~/.claude/projects/-Users-malcolmmessiter-Documents-GitHub-LockDownRadioControl/memory/` — auto-loaded notes (RXV2 design, chip plan, bind handshake, MSP bridge plan, RF version pinning, etc.).
- `ReceiverCode/RXV2/src/` — RXV2 firmware modules.
- `ReceiverCode/src/utilities/radio.h` — v1 receiver, the reference protocol implementation.
- `TransmitterCode/include/transceiver.h` — v1 TX ack-payload parsing (read-only while RXV2 is in flight).
