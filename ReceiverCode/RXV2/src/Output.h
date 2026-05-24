// LockDownRadioControl — RXV2  ::  Output.h
//
// All RC-output protocol code: frame builders (SBUS, CRSF, IBUS, FBUS, PPM),
// UART/RMT configuration, and the periodic sbusTick() that drives whichever
// protocol is active.
//
// Same pin (D6) for all of them — the chip reconfigures it as either an
// inverted/non-inverted UART or as an RMT pulse generator at boot, depending
// on the user's saved protocol choice.
//
//*********************************************************************

#ifndef _SRC_OUTPUT_H
#define _SRC_OUTPUT_H

#include "1Defs.h"

//*********************************************************************
//  Protocol name / description (for UI)
//*********************************************************************

inline const char* protocolName(Protocol p) {
    switch (p) {
        case PROTO_SBUS:  return "SBUS";
        case PROTO_CRSF:  return "CRSF";
        case PROTO_IBUS:  return "IBUS";
        case PROTO_PPM:   return "PPM";
        case PROTO_FBUS:  return "FBUS";
        case PROTO_IBUS2: return "IBUS2";
    }
    return "?";
}

//*********************************************************************

inline const char* protocolDesc(Protocol p) {
    switch (p) {
        case PROTO_SBUS:  return "FrSky/Futaba, 100 kbaud 8E2 inverted, ~71 Hz";
        case PROTO_CRSF:  return "Crossfire/ELRS, 420 kbaud 8N1, ~250 Hz";
        case PROTO_IBUS:  return "FlySky, 115200 8N1, ~140 Hz";
        case PROTO_PPM:   return "Single-pin pulse train, 8 ch, ~45 Hz (RMT)";
        case PROTO_FBUS:  return "FrSky FBUS, 115200 8N1 inverted, RC frames only (no FC telemetry yet)";
        case PROTO_IBUS2: return "FlySky IBUS2 servo path, 115200 8N1 — same RC frames as IBUS, separate menu entry";
    }
    return "";
}

//*********************************************************************
//  SBUS frame (FrSky / Futaba)
//*********************************************************************
// 25 bytes: start byte 0x0F, 16 channels × 11 bits packed, flags, end byte.

inline void buildSbusFrame(bool frameLost, bool failsafe) {
    memset(sbusFrame, 0, sizeof(sbusFrame));
    sbusFrame[0] = 0x0F;

    uint32_t bits     = 0;
    uint8_t  bitCount = 0;
    uint8_t  idx      = 1;
    for (uint8_t c = 0; c < 16; ++c) {
        // Mirror v1 MapToSBUS: map(value, 500, 2500, 0, 2047)
        long v = (long)channelMicros[c] - 500;
        long s = (v * 2047L) / 2000L;
        if (s < 0)    s = 0;
        if (s > 2047) s = 2047;
        bits |= ((uint32_t)s & 0x07FFu) << bitCount;
        bitCount += 11;
        while (bitCount >= 8) {
            sbusFrame[idx++] = (uint8_t)(bits & 0xFFu);
            bits >>= 8;
            bitCount -= 8;
        }
    }
    sbusFrame[23] = (frameLost ? 0x04 : 0) | (failsafe ? 0x08 : 0);
    sbusFrame[24] = 0x00;
}

//*********************************************************************
//  CRSF CRC-8 (poly 0xD5)
//*********************************************************************

inline uint8_t crsfCrc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

//*********************************************************************
//  CRSF "packed RC channels" frame (type 0x16)
//*********************************************************************
// 26 bytes total. CRSF channel value 172 = 988 us, 992 = 1500 us, 1811 = 2012 us
// — same 11-bit container as SBUS but shifted/centred.

inline void buildCrsfFrame() {
    crsfFrame[0] = 0xC8;                                 // dest: flight controller
    crsfFrame[1] = 24;                                   // length of (type + payload + crc)
    crsfFrame[2] = 0x16;                                 // type: packed RC channels

    uint32_t bits     = 0;
    uint8_t  bitCount = 0;
    uint8_t  idx      = 3;
    for (uint8_t c = 0; c < 16; ++c) {
        long v       = (long)channelMicros[c] - 1500;    // centre at zero
        long crsfval = (v * 819L) / 500L + 992L;         // 819/500 ≈ 1.638
        if (crsfval < 172)  crsfval = 172;
        if (crsfval > 1811) crsfval = 1811;
        bits |= ((uint32_t)crsfval & 0x07FFu) << bitCount;
        bitCount += 11;
        while (bitCount >= 8) {
            crsfFrame[idx++] = (uint8_t)(bits & 0xFFu);
            bits >>= 8;
            bitCount -= 8;
        }
    }
    crsfFrame[25] = crsfCrc8(&crsfFrame[2], 23);
}

//*********************************************************************
//  FBUS RC frame (FrSky FBUS, control / channel data)  -- dormant
//*********************************************************************
// Best-effort encoding based on common public references — not yet verified
// against a real FC.
//   [0]  length = 0x18 (24 bytes following: header + 23 payload bytes)
//   [1]  header = 0xFF (control / RC frame type)
//   [2..23]  22 bytes of 16 channels × 11 bits packed (same as SBUS/CRSF)
//   [24] flags (frame_lost = bit2, failsafe = bit3)
//   [25] CRC8 over bytes [1..24], poly 0xD5

inline void buildFbusFrame(bool frameLost, bool failsafe) {
    memset(fbusFrame, 0, sizeof(fbusFrame));
    fbusFrame[0] = 0x18;
    fbusFrame[1] = 0xFF;

    uint32_t bits     = 0;
    uint8_t  bitCount = 0;
    uint8_t  idx      = 2;
    for (uint8_t c = 0; c < 16; ++c) {
        long v = (long)channelMicros[c] - 500;
        long s = (v * 2047L) / 2000L;
        if (s < 0)    s = 0;
        if (s > 2047) s = 2047;
        bits |= ((uint32_t)s & 0x07FFu) << bitCount;
        bitCount += 11;
        while (bitCount >= 8) {
            fbusFrame[idx++] = (uint8_t)(bits & 0xFFu);
            bits >>= 8;
            bitCount -= 8;
        }
    }
    fbusFrame[24] = (frameLost ? 0x04 : 0) | (failsafe ? 0x08 : 0);
    fbusFrame[25] = crsfCrc8(&fbusFrame[1], 24);
}

//*********************************************************************
//  IBUS frame (FlySky)
//*********************************************************************
// 32 bytes: header 0x20 0x40, 14 channels × uint16_t little-endian (1000-2000 µs),
// 16-bit checksum = 0xFFFF - sum of preceding bytes.

inline void buildIbusFrame() {
    ibusFrame[0] = 0x20;
    ibusFrame[1] = 0x40;
    for (uint8_t c = 0; c < 14; ++c) {
        uint16_t v = channelMicros[c];
        if (v < 1000) v = 1000;
        if (v > 2000) v = 2000;
        ibusFrame[2 + c * 2]     = (uint8_t)(v & 0xFFu);
        ibusFrame[2 + c * 2 + 1] = (uint8_t)((v >> 8) & 0xFFu);
    }
    uint16_t cksum = 0xFFFF;
    for (uint8_t i = 0; i < 30; ++i) cksum -= ibusFrame[i];
    ibusFrame[30] = (uint8_t)(cksum & 0xFFu);
    ibusFrame[31] = (uint8_t)((cksum >> 8) & 0xFFu);
}

//*********************************************************************
//  PPM RMT items (8 channels)
//*********************************************************************
// Positive PPM (ppmInverted=false): idle LOW, 300 µs HIGH sync pulse leads each
//   channel slot, then LOW for remainder.
// Negative PPM (ppmInverted=true): idle HIGH, 300 µs LOW sync, HIGH for remainder.
// Final entry: long pulse at idle level to fill the 22.5 ms frame.

inline void buildPpmRmtItems() {
    uint8_t sync_lvl = ppmInverted ? 0 : 1;
    uint8_t idle_lvl = ppmInverted ? 1 : 0;
    uint32_t total = 0;
    for (uint8_t c = 0; c < PPM_CHANNELS; ++c) {
        uint16_t v = channelMicros[c];
        if (v < 1000) v = 1000;
        if (v > 2000) v = 2000;
        ppmItems[c].duration0 = PPM_SYNC_US;
        ppmItems[c].level0    = sync_lvl;
        ppmItems[c].duration1 = v - PPM_SYNC_US;
        ppmItems[c].level1    = idle_lvl;
        total += v;
    }
    uint32_t gap = (total < PPM_FRAME_US) ? (PPM_FRAME_US - total) : 1000;
    if (gap > 30000) gap = 30000;
    ppmItems[PPM_CHANNELS].duration0 = (uint16_t)gap;
    ppmItems[PPM_CHANNELS].level0    = idle_lvl;
    ppmItems[PPM_CHANNELS].duration1 = 0;
    ppmItems[PPM_CHANNELS].level1    = idle_lvl;
}

//*********************************************************************
//  Configure output driver from saved protocol selection
//*********************************************************************
// Called once from setup() after loading NVS. Brings up UART or RMT on D6 (and
// D5 for the bidirectional protocols) per the chosen protocol.

inline void configureOutputDriver(Protocol p) {
    if (p == PROTO_PPM) {
        // arduino-esp32 v2 RMT API: rmtInit(pin, tx_not_rx, memsize). tx_not_rx=true → TX mode.
        // RMT_MEM_128 gives plenty of headroom (32 items, we use 9).
        ppmRmt = rmtInit(PIN_SBUS_TX, /*tx_not_rx=*/true, RMT_MEM_128);
        if (ppmRmt) {
            rmtSetTick(ppmRmt, 1000.0f);
            ppmRmtReady = true;
            Serial.printf("[out] PPM via RMT on GPIO %d\n", PIN_SBUS_TX);
        } else {
            Serial.println("[out] PPM RMT init failed");
            ppmRmtReady = false;
        }
        return;
    }
    switch (p) {
        case PROTO_SBUS:
            // TX-only — SBUS has no telemetry path back from the FC.
            Serial1.begin(100000, SERIAL_8E2, -1, PIN_SBUS_TX, true);
            Serial.printf("[out] SBUS inverted UART on GPIO %d (100 kbaud 8E2)\n", PIN_SBUS_TX);
            break;
        case PROTO_CRSF:
            // Bidirectional — D6 sends RC, D5 receives FC telemetry + (future) MSP responses.
            Serial1.begin(420000, SERIAL_8N1, PIN_FC_RX, PIN_SBUS_TX, false);
            Serial.printf("[out] CRSF UART tx=D6 rx=D5 (420 kbaud 8N1)\n");
            break;
        case PROTO_IBUS:
            // TX-only IBUS — no sensor polling here; pick IBUS2 if you want telemetry.
            Serial1.begin(115200, SERIAL_8N1, -1, PIN_SBUS_TX, false);
            Serial.printf("[out] IBUS UART on GPIO %d (115200 8N1)\n", PIN_SBUS_TX);
            break;
        case PROTO_FBUS:
            // FBUS is single-wire half-duplex: tie D6 and D5 together externally to the FC's
            // FBUS pad. We transmit on D6 and listen on D5 for the FC's downlink queries.
            Serial1.begin(115200, SERIAL_8N1, PIN_FC_RX, PIN_SBUS_TX, true);
            Serial.printf("[out] FBUS UART tx=D6 rx=D5 (115200 8N1 inverted)\n");
            break;
        case PROTO_IBUS2:
            // FlySky IBUS2: servo bus on D6 (TX), sensor bus on D5 (RX). The FC polls D5
            // for sensor data; we respond by writing the response back via the same UART.
            Serial1.begin(115200, SERIAL_8N1, PIN_FC_RX, PIN_SBUS_TX, false);
            Serial.printf("[out] IBUS2 UART tx=D6 rx=D5 (115200 8N1)\n");
            break;
        default: break;
    }
}

//*********************************************************************
//  Period selector per protocol
//*********************************************************************

inline uint32_t protocolPeriodMs(Protocol p) {
    switch (p) {
        case PROTO_SBUS:  return SBUS_PERIOD_MS;
        case PROTO_CRSF:  return CRSF_PERIOD_MS;
        case PROTO_IBUS:  return IBUS_PERIOD_MS;
        case PROTO_PPM:   return PPM_PERIOD_MS;
        case PROTO_FBUS:  return FBUS_PERIOD_MS;
        case PROTO_IBUS2: return IBUS_PERIOD_MS;
    }
    return SBUS_PERIOD_MS;
}

//*********************************************************************
//  Periodic output tick — drives whichever protocol is active
//*********************************************************************
// Name is "sbusTick" for historical reasons; it actually dispatches to the
// selected protocol's frame builder + write.

inline void sbusTick() {
    // Suspend RC frame transmission while the MSP bridge has a client connected.
    // The user is configuring (TX is off, we're not flying); the FC's CRSF UART
    // is being driven by Configurator over the bridge instead. Sending RC frames
    // here would interleave with MSP traffic and corrupt both directions.
    if (mspBridgeActive) return;

    if ((uint32_t)(millis() - lastSbusMs) < protocolPeriodMs(currentProtocol)) return;
    lastSbusMs = millis();

    uint32_t age      = millis() - lastChannelDataMs;
    bool     frameLost = age > 100;
    bool     failsafe  = age > 500;

    switch (currentProtocol) {
        case PROTO_SBUS:
            buildSbusFrame(frameLost, failsafe);
            Serial1.write(sbusFrame, sizeof(sbusFrame));
            break;
        case PROTO_CRSF:
            // CRSF has no explicit frame-lost bit — the FC infers loss from absence of frames.
            (void)frameLost; (void)failsafe;
            buildCrsfFrame();
            Serial1.write(crsfFrame, sizeof(crsfFrame));
            break;
        case PROTO_IBUS:
        case PROTO_IBUS2:
            (void)frameLost; (void)failsafe;
            buildIbusFrame();
            Serial1.write(ibusFrame, sizeof(ibusFrame));
            break;
        case PROTO_FBUS:
            buildFbusFrame(frameLost, failsafe);
            Serial1.write(fbusFrame, sizeof(fbusFrame));
            break;
        case PROTO_PPM:
            if (ppmRmtReady && ppmRmt) {
                buildPpmRmtItems();
                // Non-blocking. Each frame is ~22 ms; PPM_PERIOD_MS is 25 ms so the
                // previous transmission has plenty of time to complete before the next
                // write reuses the buffer. If hardware is still busy we skip this frame.
                rmtWrite(ppmRmt, ppmItems, PPM_CHANNELS + 1);
            }
            break;
    }
    sbusFramesOut++;
}

#endif // _SRC_OUTPUT_H
