// LockDownRadioControl — RXV2  ::  MspFc.h
//
// Active FC discovery via MSP-over-CRSF. At boot (and periodically thereafter
// while not detected), sends MSP_FC_VARIANT + MSP_FC_VERSION requests wrapped
// in CRSF type 0x7A frames. Parses CRSF type 0x7B responses to extract MSP
// payloads. Result lands in fcInfo (declared in 1Defs.h) and is surfaced on
// /api/state.json so the home page JS can conditionally show the
// "Rotorflight config" button when version ≥ 2.2 is detected.
//
// MSP-over-CRSF wire format used here (per BetaFlight / ELRS convention):
//
//   sync(C8) length 7A dest(C8) src(EA) status(10) {MSP v1 body} crc8
//
//   - status byte 0x10 = start-of-frame, sequence 0, MSP v1
//   - MSP v1 body for request: size(1) function(1) payload(N) checksum(1)
//   - MSP v1 body for response: same shape, function echoed back
//
// We send everything on Serial1 (which Output.h has already configured as
// CRSF at 420 kbaud 8N1 on D6/D5) and receive responses through the existing
// CRSF parser in Telemetry.h.
//
//*********************************************************************

#ifndef _SRC_MSPFC_H
#define _SRC_MSPFC_H

#include "1Defs.h"
#include "Output.h"      // crsfCrc8()

//*********************************************************************
//  MSP function codes we care about
//*********************************************************************

constexpr uint8_t MSP_API_VERSION    = 1;     // 3 bytes: protoVer, apiMajor, apiMinor
constexpr uint8_t MSP_FC_VARIANT     = 2;     // 4 ASCII bytes (e.g. "RTFL")
constexpr uint8_t MSP_FC_VERSION     = 3;     // 3 bytes: major, minor, patch

// Rotorflight tuning commands (codes from v1's Nexus.h)
constexpr uint8_t MSP_RC_TUNING      = 111;
constexpr uint8_t MSP_PID            = 112;
constexpr uint8_t MSP_SET_RC_TUNING  = 204;
constexpr uint8_t MSP_SET_PID        = 202;
constexpr uint8_t MSP_PID_PROFILE    = 94;
constexpr uint8_t MSP_SET_PID_PROFILE = 95;
constexpr uint8_t MSP_SELECT_SETTING = 210;
constexpr uint8_t MSP_GOVERNOR_CONFIG = 142;
constexpr uint8_t MSP_SET_GOVERNOR_CONFIG = 143;
constexpr uint8_t MSP_GOVERNOR_PROFILE = 148;
constexpr uint8_t MSP_SET_GOVERNOR_PROFILE = 149;
constexpr uint8_t MSP_EEPROM_WRITE   = 250;

//*********************************************************************
//  Sync-wait state for mspRequestAndWait()
//*********************************************************************

inline volatile uint8_t  mspWaitFunction = 0xFF;     // 0xFF = nothing pending
inline uint8_t           mspWaitRespBuf[256] = {0};
inline volatile uint16_t mspWaitRespLen = 0;
inline volatile bool     mspWaitRespReady = false;

//*********************************************************************
//  CRSF address constants
//*********************************************************************

constexpr uint8_t CRSF_ADDR_FC      = 0xC8;
constexpr uint8_t CRSF_ADDR_HANDSET = 0xEA;
constexpr uint8_t CRSF_TYPE_MSP_REQ = 0x7A;
constexpr uint8_t CRSF_TYPE_MSP_RSP = 0x7B;

//*********************************************************************
//  Build + send an MSP v1 request, wrapped in CRSF type 0x7A
//*********************************************************************

inline void mspSendRequest(uint8_t function, const uint8_t* payload = nullptr, uint8_t payloadLen = 0) {
    // MSP v1 body inside CRSF is just: size(1) function(1) payload(N).
    // The inner XOR checksum is NOT transmitted — CRSF's CRC8 protects it.
    uint8_t mspBody[64];
    if (payloadLen > sizeof(mspBody) - 2) return;
    mspBody[0] = payloadLen;
    mspBody[1] = function;
    for (uint8_t i = 0; i < payloadLen; ++i) mspBody[2 + i] = payload[i];
    uint8_t mspBodyLen = 2 + payloadLen;

    // CRSF wrapper. Status byte 0x30 = SoF (bit 4) + MSP version 1 (bits 6-5 = 01).
    // length covers: type + dest + src + status + mspBody + crc.
    uint8_t crsf[80];
    crsf[0] = CRSF_ADDR_FC;                  // sync
    crsf[1] = 4 + mspBodyLen + 1;            // length byte
    crsf[2] = CRSF_TYPE_MSP_REQ;
    crsf[3] = CRSF_ADDR_FC;                  // dest = FC
    crsf[4] = CRSF_ADDR_HANDSET;             // src  = handset (us)
    crsf[5] = 0x30;                          // status: SoF + version v1
    memcpy(&crsf[6], mspBody, mspBodyLen);
    uint8_t crsfTotal = 6 + mspBodyLen;      // up to and including last MSP byte
    crsf[crsfTotal] = crsfCrc8(&crsf[2], (uint8_t)(crsfTotal - 2));
    Serial1.write(crsf, (size_t)(crsfTotal + 1));
}

//*********************************************************************
//  Parse a CRSF MSP response (type 0x7B) payload — called from Telemetry.h
//*********************************************************************
// `body` points to the CRSF frame's payload (the bytes after type), `bodyLen`
// is the count from dest through last MSP byte (exclusive of CRC — caller has
// already verified that). Layout: dest(1) src(1) status(1) mspBody(N).

inline void mspParseResponse(const uint8_t* body, uint8_t bodyLen) {
    if (bodyLen < 3 + 2) return;                     // dest+src+status + minimal MSP (size+func)
    // body[0] = dest, body[1] = src, body[2] = status
    const uint8_t* msp = &body[3];
    uint8_t mspLen = (uint8_t)(bodyLen - 3);
    if (mspLen < 2) return;
    uint8_t size = msp[0];
    uint8_t func = msp[1];
    if ((uint16_t)(2 + size) > mspLen) return;       // payload doesn't fit (no inner CRC)
    const uint8_t* payload = &msp[2];

    fcInfo.lastResponseMs = millis();

    // If a synchronous request is waiting for this function code, capture it.
    if (func == mspWaitFunction && !mspWaitRespReady) {
        mspWaitRespLen = size;
        if (size > 0 && size <= sizeof(mspWaitRespBuf)) {
            memcpy(mspWaitRespBuf, payload, size);
        }
        mspWaitRespReady = true;
    }

    switch (func) {
        case MSP_FC_VARIANT:
            if (size >= 4) {
                memcpy(fcInfo.variant, payload, 4);
                fcInfo.variant[4] = '\0';
                fcInfo.detected = true;
            }
            break;
        case MSP_FC_VERSION:
            if (size >= 3) {
                fcInfo.fwMajor = payload[0];
                fcInfo.fwMinor = payload[1];
                fcInfo.fwPatch = payload[2];
                fcInfo.versionKnown = true;
            }
            break;
        case MSP_API_VERSION:
            if (size >= 3) {
                fcInfo.mspProto = payload[0];
                fcInfo.apiMajor = payload[1];
                fcInfo.apiMinor = payload[2];
            }
            break;
        default:
            break;
    }
}

//*********************************************************************
//  Synchronous MSP request: send and wait for response
//*********************************************************************
// Used by HTTP request handlers to talk to the FC. Blocks for up to
// timeoutMs while pumping Serial1 → CRSF parser → mspParseResponse, which
// will set mspWaitRespReady=true if the matching function code comes back.

inline bool mspRequestAndWait(uint8_t function, const uint8_t* req, uint8_t reqLen,
                              uint8_t* outBuf, uint16_t* outLen, uint32_t timeoutMs) {
    extern void protocolRx();   // defined in Telemetry.h
    mspWaitFunction  = function;
    mspWaitRespReady = false;
    mspWaitRespLen   = 0;
    mspSendRequest(function, req, reqLen);
    uint32_t deadline = millis() + timeoutMs;
    while (!mspWaitRespReady && (int32_t)(deadline - millis()) > 0) {
        protocolRx();
        delay(1);
    }
    bool ok = mspWaitRespReady;
    mspWaitFunction = 0xFF;
    if (ok) {
        if (outLen) *outLen = mspWaitRespLen;
        if (outBuf && mspWaitRespLen > 0) memcpy(outBuf, mspWaitRespBuf, mspWaitRespLen);
    }
    return ok;
}

//*********************************************************************
//  Periodic probe — call from loop(), runs at low rate
//*********************************************************************
// While the FC is undetected, send MSP_FC_VARIANT + MSP_FC_VERSION +
// MSP_API_VERSION every PROBE_INTERVAL_MS. Once detected, slow down to a
// heartbeat that confirms the FC is still alive. If responses stop for
// PROBE_TIMEOUT_MS we mark detected=false again.

constexpr uint32_t PROBE_INTERVAL_MS    = 1000;   // while seeking
constexpr uint32_t PROBE_HEARTBEAT_MS   = 5000;   // once detected
constexpr uint32_t PROBE_TIMEOUT_MS     = 10000;  // declare FC lost after this

inline void mspFcPoll() {
    // Only meaningful in CRSF mode (D6 is wired as CRSF UART to FC).
    if (currentProtocol != PROTO_CRSF) return;
    // Don't fight the bridge — if a Configurator client is talking to the FC
    // we'd just confuse both sides.
    if (mspBridgeActive) return;
    // Don't fight a synchronous /api/msp request that's mid-wait — sending
    // a competing probe causes the FC to interleave two responses, often
    // making the sync request time out and the page see "Read failed".
    if (mspWaitFunction != 0xFF) return;
    // Don't probe while RC frames are being transmitted to a live TX session,
    // unless dev mode keeps wifi on anyway. Probing during active flight would
    // briefly compete for Serial1 bandwidth.
    bool flying = (rx.lastMillis != 0) && ((uint32_t)(millis() - rx.lastMillis) < 500);
    if (flying && !DEV_KEEP_WIFI) return;

    uint32_t now = millis();
    uint32_t interval = fcInfo.detected ? PROBE_HEARTBEAT_MS : PROBE_INTERVAL_MS;
    if ((uint32_t)(now - fcInfo.lastProbeMs) < interval) return;
    fcInfo.lastProbeMs = now;

    // Cycle through the three requests on successive probes so we eventually
    // get all three pieces of info even if some responses are dropped.
    static uint8_t which = 0;
    switch (which++ % 3) {
        case 0: mspSendRequest(MSP_FC_VARIANT);  break;
        case 1: mspSendRequest(MSP_FC_VERSION);  break;
        case 2: mspSendRequest(MSP_API_VERSION); break;
    }
    fcInfo.probesSent++;

    // Detect timeout — if we were detected but responses have stopped.
    if (fcInfo.detected && fcInfo.lastResponseMs != 0 &&
        (uint32_t)(now - fcInfo.lastResponseMs) > PROBE_TIMEOUT_MS) {
        fcInfo.detected     = false;
        fcInfo.versionKnown = false;
        events.add("FC lost — no MSP response in 10s");
    }
}

//*********************************************************************
//  Convenience: is this a Rotorflight 2.2+ FC?
//*********************************************************************

// Map MSP API version → Rotorflight major.minor (per v1's Nexus.h mapping).
// API 12.8 == Rotorflight 1.x ; API 12.9 == Rotorflight 2.3.
// (2.2 maps to API 12.9 too in some builds — refine when we see one.)
inline uint8_t rotorflightMajor() {
    if (fcInfo.apiMajor == 12 && fcInfo.apiMinor == 8) return 1;
    if (fcInfo.apiMajor == 12 && fcInfo.apiMinor >= 9) return 2;
    return 0;
}
inline uint8_t rotorflightMinor() {
    // Best-guess: API 12.9 == RF 2.3 for now. Refine when we get RF 2.2 traffic.
    if (fcInfo.apiMajor == 12 && fcInfo.apiMinor == 9) return 3;
    return 0;
}

inline bool fcIsRotorflightConfigCapable() {
    // Strict check: only if MSP probe confirmed Rotorflight 2.2+.
    if (fcInfo.detected && fcInfo.versionKnown &&
        strncmp(fcInfo.variant, "RTFL", 4) == 0 &&
        rotorflightMajor() >= 2) {
        return true;
    }
    // Fallback: in CRSF mode, if telemetry is actively flowing from the FC,
    // assume the user knows what they're connected to and surface the button.
    // (MSP-over-CRSF probe isn't reliable across all FC configurations.)
    if (currentProtocol == PROTO_CRSF &&
        fcTelem.framesParsed > 5 &&
        fcTelem.lastFrameMs != 0 &&
        (uint32_t)(millis() - fcTelem.lastFrameMs) < 2000) {
        return true;
    }
    return false;
}

#endif // _SRC_MSPFC_H
