// LockDownRadioControl — RXV2  ::  Telemetry.h
//
// Inbound telemetry parsers for the bidirectional output protocols. When CRSF,
// FBUS, or IBUS2 is selected, the chip listens on D5 for frames coming back
// from the flight controller. Parsed values land in fcTelem (declared in
// 1Defs.h) and are surfaced on /diagnostics.
//
// FBUS / IBUS2 parsers are present but currently dormant — they're kept so
// the code path still compiles, and so we can resurrect them with one
// PROTO_MAX bump in 1Defs.h.
//
//*********************************************************************

#ifndef _SRC_TELEMETRY_H
#define _SRC_TELEMETRY_H

#include "1Defs.h"
#include "Output.h"      // crsfCrc8()

//*********************************************************************
//  Raw byte capture (every byte from D5, regardless of protocol)
//*********************************************************************

inline void captureRawFcByte(uint8_t b) {
    fcRawRing[fcRawHead] = b;
    fcRawHead = (uint16_t)((fcRawHead + 1) % FC_RAW_RING_SIZE);
    if (fcRawCount < FC_RAW_RING_SIZE) fcRawCount++;
    fcRawTotal++;
}

//*********************************************************************
//  CRSF inbound parser — frame decoder
//*********************************************************************
// buf[0] device address, buf[1] length (= type + payload + crc), buf[2] type,
// buf[3..total-2] payload, buf[total-1] CRC over [2..total-2].

inline void parseCrsfFrame(const uint8_t* buf, uint8_t total) {
    if (total < 4) return;
    uint8_t crc = crsfCrc8(&buf[2], (uint8_t)(total - 3));
    if (crc != buf[total - 1]) { fcTelem.crcErrors++; return; }

    uint8_t        type     = buf[2];
    const uint8_t* p        = &buf[3];
    uint8_t        plLen    = (uint8_t)(total - 4);

    switch (type) {
        case 0x14:   // CRSF_FRAMETYPE_LINK_STATISTICS
            if (plLen >= 10) {
                fcTelem.fcUplinkRssi = -(int8_t)p[0];       // dBm
                fcTelem.fcUplinkLq   = p[2];
                fcTelem.fcUplinkSnr  = (int8_t)p[3];
            }
            break;
        case 0x08:   // CRSF_FRAMETYPE_BATTERY_SENSOR
            if (plLen >= 8) {
                uint16_t v  = ((uint16_t)p[0] << 8) | p[1];       // 0.1 V
                uint16_t i  = ((uint16_t)p[2] << 8) | p[3];       // 0.1 A
                uint32_t mah = ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 8) | p[6];
                fcTelem.fcBattVolts = v * 0.1f;
                fcTelem.fcBattAmps  = i * 0.1f;
                fcTelem.fcBattMah   = mah;
                fcTelem.fcBattPct   = p[7];
            }
            break;
        case 0x1E:   // CRSF_FRAMETYPE_ATTITUDE
            if (plLen >= 6) {
                fcTelem.attitudePitch = (int16_t)(((uint16_t)p[0] << 8) | p[1]);
                fcTelem.attitudeRoll  = (int16_t)(((uint16_t)p[2] << 8) | p[3]);
                fcTelem.attitudeYaw   = (int16_t)(((uint16_t)p[4] << 8) | p[5]);
            }
            break;
        case 0x21: {  // CRSF_FRAMETYPE_FLIGHT_MODE (null-terminated string)
            uint8_t copyLen = plLen;
            if (copyLen > sizeof(fcTelem.flightMode) - 1) copyLen = sizeof(fcTelem.flightMode) - 1;
            memcpy(fcTelem.flightMode, p, copyLen);
            fcTelem.flightMode[copyLen] = '\0';
            break;
        }
        case 0x7B:   // CRSF_FRAMETYPE_MSP_RSP — MSP response from FC
            // p[] starts at the byte after the type, i.e. dest|src|status|mspBody
            // plLen is the payload length (excluding type and CRC).
            // The MSP parser in MspFc.h decodes dest/src/status and the MSP body.
            mspParseResponse(p, plLen);
            break;
        default: break;
    }
    fcTelem.framesParsed++;
    fcTelem.lastFrameMs = millis();
    fcTelem.valid       = true;
}

//*********************************************************************
//  CRSF inbound parser — byte-by-byte state machine
//*********************************************************************

inline void parseCrsfIncoming(uint8_t b) {
    fcTelem.bytesIn++;
    if (fcRxLen == 0) {
        if (b == 0xC8 || b == 0xEA || b == 0xEC || b == 0xEE) {
            fcRxBuf[0] = b;
            fcRxLen    = 1;
        }
        return;
    }
    if (fcRxLen == 1) {
        if (b >= 2 && b <= 62) {
            fcRxBuf[1] = b;
            fcRxLen    = 2;
        } else {
            fcRxLen = 0;
        }
        return;
    }
    fcRxBuf[fcRxLen++] = b;
    uint8_t total = (uint8_t)(fcRxBuf[1] + 2);
    if (fcRxLen >= total || fcRxLen >= sizeof(fcRxBuf)) {
        parseCrsfFrame(fcRxBuf, fcRxLen);
        fcRxLen = 0;
    }
}

//*********************************************************************
//  FBUS inbound parser  -- scaffold, dormant
//*********************************************************************
// FBUS frames from FC are S.Port-style poll/downlink. Public spec is fuzzy
// across firmware versions; for now we count bytes only — full implementation
// pending a confirmed wire-capture against a real FC.

inline void parseFbusIncoming(uint8_t b) {
    fcTelem.bytesIn++;
    (void)b;
}

//*********************************************************************
//  IBUS2 sensor protocol — checksum helper
//*********************************************************************
// len = total frame size including the 2 checksum bytes at the end.

inline void ibusFillChecksum(uint8_t* buf, uint8_t len) {
    uint16_t s = 0xFFFF;
    for (uint8_t i = 0; i < (uint8_t)(len - 2); ++i) s -= buf[i];
    buf[len - 2] = (uint8_t)(s & 0xFF);
    buf[len - 1] = (uint8_t)((s >> 8) & 0xFF);
}

//*********************************************************************
//  IBUS2 sensor protocol — poll/response state machine
//*********************************************************************
// FlySky IBUS-Telemetry. FC sends a 4-byte poll on the sensor wire; we
// respond with a matching frame. Commands recognised:
//   0x80 + slot  DISCOVER (echo back the 4-byte query)
//   0x90 + slot  TYPE     (sensor type + payload length)
//   0xA0 + slot  VALUE    (sensor data)
// Checksum: 16-bit little-endian = 0xFFFF - sum of preceding bytes.

inline void parseIbus2Incoming(uint8_t b) {
    fcTelem.bytesIn++;

    if (ibusRxIdx == 0) {
        if (b == 0x04) { ibusRxBuf[0] = b; ibusRxIdx = 1; }    // length byte (always 4 for poll)
        return;
    }
    ibusRxBuf[ibusRxIdx++] = b;
    if (ibusRxIdx < 4) return;

    // Full 4-byte poll received — validate checksum.
    uint16_t expected = 0xFFFF;
    expected -= ibusRxBuf[0];
    expected -= ibusRxBuf[1];
    uint16_t got = (uint16_t)ibusRxBuf[2] | ((uint16_t)ibusRxBuf[3] << 8);
    if (expected != got) {
        fcTelem.crcErrors++;
        ibusRxIdx = 0;
        return;
    }

    uint8_t cmd     = ibusRxBuf[1];
    uint8_t cmdType = cmd & 0xF0;
    uint8_t slot    = cmd & 0x0F;
    ibusRxIdx = 0;

    fcTelem.framesParsed++;
    fcTelem.lastFrameMs = millis();
    fcTelem.valid       = true;

    if (slot < 1 || slot > IBUS_NUM_SLOTS) return;
    const IbusSlot& sl = IBUS_SLOTS[slot - 1];

    uint8_t resp[12] = {0};
    switch (cmdType) {
        case 0x80:   // DISCOVER → echo
            resp[0] = 0x04; resp[1] = cmd;
            ibusFillChecksum(resp, 4);
            Serial1.write(resp, 4);
            fcTelem.responsesSent++;
            break;
        case 0x90:   // TYPE → 6-byte reply
            resp[0] = 0x06; resp[1] = cmd;
            resp[2] = sl.type;
            resp[3] = sl.dataLen;
            ibusFillChecksum(resp, 6);
            Serial1.write(resp, 6);
            fcTelem.responsesSent++;
            break;
        case 0xA0: {   // VALUE → sensor data
            uint8_t total = (uint8_t)(2 + sl.dataLen + 2);
            resp[0] = total;
            resp[1] = cmd;
            if (slot == 1) {
                // External voltage in 0.01 V units — from FC CRSF telemetry; 0 if none.
                uint16_t v = (uint16_t)((fcTelem.valid ? fcTelem.fcBattVolts : 0.0f) * 100.0f);
                resp[2] = (uint8_t)(v & 0xFF);
                resp[3] = (uint8_t)((v >> 8) & 0xFF);
            } else if (slot == 2) {
                // RF link quality: 0..100 based on time since last RF packet.
                uint32_t age = rx.lastMillis ? (millis() - rx.lastMillis) : 1000UL;
                if (age > 1000) age = 1000;
                uint16_t lq  = (uint16_t)((1000 - age) / 10);
                resp[2] = (uint8_t)(lq & 0xFF);
                resp[3] = (uint8_t)((lq >> 8) & 0xFF);
            }
            ibusFillChecksum(resp, total);
            Serial1.write(resp, total);
            fcTelem.responsesSent++;
            break;
        }
        default: break;
    }
}

//*********************************************************************
//  Top-level RX pump — drain Serial1 every loop()
//*********************************************************************
// Captures every byte to the raw ring (for /diagnostics) and dispatches to the
// per-protocol parser if there is one for the current protocol.

inline void protocolRx() {
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        captureRawFcByte(b);
        mspBridgeOnFcByte(b);     // forward to TCP if MSP bridge has a client
        switch (currentProtocol) {
            case PROTO_CRSF:  parseCrsfIncoming(b);  break;
            case PROTO_FBUS:  parseFbusIncoming(b);  break;
            case PROTO_IBUS2: parseIbus2Incoming(b); break;
            default: break;
        }
    }
}

#endif // _SRC_TELEMETRY_H
