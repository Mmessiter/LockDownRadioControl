// LockDownRadioControl — RXV2  ::  MspBridge.h
//
// Phase 1 wireless MSP bridge.
//
// Listens on TCP port 5760 (the Rotorflight/Betaflight standard). When a
// client connects and the receiver is in CRSF mode, every byte received
// from the client is forwarded straight to Serial1 (D6 → FC), and every
// byte arriving from the FC on Serial1 (D5) is forwarded straight back to
// the TCP client. RC frame transmission on D6 is suspended while a client
// is connected — you're configuring, not flying.
//
// Use case: open Rotorflight Configurator on Mac/iPad, pick "TCP", enter
// `ldrc_rx.local:5760`, connect — full wireless MSP access. No TX needed.
//
// This is a transparent byte pipe. If Configurator expects raw MSP and the
// FC expects MSP-over-CRSF (or vice-versa), we'll need to add MSP frame
// awareness in a follow-up — but try this first; it might just work.
//
//*********************************************************************

#ifndef _SRC_MSPBRIDGE_H
#define _SRC_MSPBRIDGE_H

#include "1Defs.h"

//*********************************************************************
//  Start the TCP server (called after WiFi is up + only in CRSF mode)
//*********************************************************************

inline void mspBridgeStart() {
    if (mspBridgeStarted) return;
    if (currentProtocol != PROTO_CRSF) {
        // The bridge only makes sense when D5/D6 are in bidirectional CRSF
        // mode. For SBUS/IBUS/PPM there's no return path to the FC.
        Serial.println("[msp] bridge NOT started — output protocol is not CRSF");
        return;
    }
    mspBridge.begin();
    mspBridge.setNoDelay(true);
    mspBridgeStarted = true;
    Serial.printf("[msp] bridge listening on tcp/%u\n", (unsigned)MSP_BRIDGE_PORT);
    events.add("MSP bridge listening :5760");
}

//*********************************************************************
//  Hook called from Telemetry.h whenever a byte arrives from the FC on D5
//*********************************************************************
// Adds zero overhead when no client is connected. When a client is
// connected, forwards the byte to the TCP socket so Configurator can
// parse the FC's MSP response (or any CRSF telemetry — Configurator will
// ignore frame types it doesn't care about).

inline void mspBridgeOnFcByte(uint8_t b) {
    if (!mspBridgeActive) return;
    if (!mspClient || !mspClient.connected()) return;
    mspClient.write(b);
    mspBridgeBytesIn++;
}

//*********************************************************************
//  Main poll — accepts clients, pumps TCP -> Serial1 bytes
//*********************************************************************
// Call once per loop(). Cheap when nothing's connecting.
// Outbound bytes (FC -> TCP) are handled inline in protocolRx() via
// mspBridgeOnFcByte() — they share the existing Serial1 read path so we
// don't double-consume.

inline void mspBridgePoll() {
    if (!mspBridgeStarted) return;

    // Accept new connections — but only one client at a time.
    if (mspBridge.hasClient()) {
        WiFiClient incoming = mspBridge.accept();
        if (!mspClient || !mspClient.connected()) {
            mspClient = incoming;
            mspClient.setNoDelay(true);
            mspBridgeActive    = true;
            mspBridgeConnectMs = millis();
            mspBridgeConnections++;
            char buf[64];
            snprintf(buf, sizeof(buf), "MSP client connected from %s",
                     mspClient.remoteIP().toString().c_str());
            events.add(buf);
            Serial.printf("[msp] %s\n", buf);
        } else {
            // Already have a client — reject the newcomer.
            incoming.stop();
            Serial.println("[msp] rejected 2nd client (only one at a time)");
        }
    }

    // Handle disconnect.
    if (mspBridgeActive && (!mspClient || !mspClient.connected())) {
        uint32_t durationMs = millis() - mspBridgeConnectMs;
        char buf[80];
        snprintf(buf, sizeof(buf), "MSP client gone (%u s, %u B in / %u B out)",
                 (unsigned)(durationMs / 1000),
                 (unsigned)mspBridgeBytesIn, (unsigned)mspBridgeBytesOut);
        events.add(buf);
        Serial.printf("[msp] %s\n", buf);
        mspClient.stop();
        mspBridgeActive = false;
    }

    // Pump TCP -> Serial1 (toward FC). Single read per poll, batch up to 256B.
    if (mspBridgeActive && mspClient && mspClient.connected()) {
        uint8_t buf[256];
        int     n = 0;
        while (mspClient.available() && n < (int)sizeof(buf)) {
            int b = mspClient.read();
            if (b < 0) break;
            buf[n++] = (uint8_t)b;
        }
        if (n > 0) {
            Serial1.write(buf, n);
            mspBridgeBytesOut += n;
        }
    }
}

#endif // _SRC_MSPBRIDGE_H
