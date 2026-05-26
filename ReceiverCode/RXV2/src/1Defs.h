// LockDownRadioControl — RXV2  ::  1Defs.h
//
// This is the single global-storage header for the receiver. It is named with a
// numeric prefix so that #include order forces it FIRST — every other module
// header in the project assumes the types, enums, constants, and global objects
// declared here already exist.
//
// Mirrors the v1 receiver convention (utilities/1Definitions.h).
//
//*********************************************************************

#ifndef _SRC_1DEFS_H
#define _SRC_1DEFS_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <esp_ota_ops.h>
#include <esp32-hal-rmt.h>
#include <Update.h>
#include <HTTPClient.h>

//*********************************************************************
//  Firmware version
//*********************************************************************

constexpr const char* FW_VERSION = "RXV2-0.9.31-revert-slot0";

//*********************************************************************
//  WiFi / network defaults
//*********************************************************************
// WiFi credentials live ONLY in NVS — never compiled into the firmware. On a
// fresh chip (no NVS entry) the boot flow goes directly to AP mode so the user
// can enter their network details via the web UI.

constexpr const char* WIFI_DEFAULT_SSID     = "";
constexpr const char* WIFI_DEFAULT_PASSWORD = "";
constexpr const char* AP_SSID               = "LDRC_RX";    // fallback AP if STA fails
constexpr const char* OTA_HOSTNAME          = "LDRC_RX";    // -> http://LDRC_RX.local/
constexpr const char* OTA_PASSWORD          = nullptr;      // set to a string to require auth

enum NetMode : uint8_t {
    NET_INIT,
    NET_WAITING_RF,           // initial 10 s window
    NET_NO_WIFI,              // TX found in boot window — or user disabled WiFi
    NET_WIFI_CONNECTING,
    NET_WIFI_UP,
    NET_AP
};
inline NetMode  netMode       = NET_INIT;
inline uint32_t netStateStart = 0;
inline bool     otaStarted    = false;

constexpr uint32_t RF_WINDOW_MS         = 10000;   // boot window before falling through to WiFi
constexpr uint32_t WIFI_CONNECT_MS      = 12000;   // STA timeout before giving up and going to AP

// *** DEV FLAG — set to false before shipping ***
// When true: skip the RF-discovery boot window and go straight to WiFi STA,
// even if the TX is on at boot. Lets us iterate without having to power-cycle
// the TX before every reflash. Production behaviour (false) is: if a TX packet
// arrives in the first 10 s, stay RF-only for the session.
constexpr bool     DEV_KEEP_WIFI         = true;

//*********************************************************************
//  Pin map (XIAO ESP32-C3 / -S3, same pinout in either footprint)
//*********************************************************************
// PCB pads, GPIO numbers, and what they connect to. See PINOUT.md for the
// definitive table that goes with the PCB design.
//
//   D2  (GPIO 4)  -> Radio1 CE        D7  (GPIO 20) -> SPI MISO (shared)
//   D3  (GPIO 5)  -> Radio1 CSN       D8  (GPIO 8)  -> SPI SCK  (shared)
//   D0  (GPIO 2)  -> Radio2 CE        D10 (GPIO 10) -> SPI MOSI (shared)
//   D1  (GPIO 3)  -> Radio2 CSN       D4  (GPIO 6)  -> Radio3 CE  (triple-radio PCB only)
//                                     D9  (GPIO 9)  -> Radio3 CSN (triple-radio PCB only, S3 only)
//   LED -> LED_BUILTIN  (XIAO onboard, freed D4 from heartbeat duty)
//   D5  (GPIO 7)  -> UART RX (FC telemetry in — CRSF / FBUS / IBUS2 sensor)
//   D6  (GPIO 21) -> UART/RMT TX (RC out — SBUS / CRSF / IBUS / PPM)
//
// Note: Radio3 uses D9, which on the older ESP32-C3 is the BOOT strap pin (see
// next note). On the S3 (production target) D9 = GPIO 9 is a plain GPIO with
// no boot-strap meaning. The triple-radio PCB variant is therefore S3-only.
//
// Why MISO is NOT on D9 (GPIO 9):
//   GPIO 9 on the ESP32-C3 doubles as the BOOT strap pin sampled at every
//   reset. If LOW at reset, the chip enters USB-download mode instead of
//   running flash. With the nRF24's MISO output wired there, transient LOW
//   levels during the module's power-on-reset can drag GPIO 9 down at exactly
//   the wrong moment, making cold-boot unreliable. Routing MISO through the
//   GPIO matrix to D7 (GPIO 20) removes the conflict.

// Use the symbolic D-named pad constants (defined in the Seeed variant headers)
// rather than raw GPIO numbers, because the underlying GPIO assignments differ
// between the XIAO ESP32-C3 and ESP32-S3 even though the physical pad labels
// (D0..D10) are identical. Example: D6 = GPIO 21 on C3, GPIO 43 on S3.
// With Dx names the same firmware binary builds correctly for either chip.

// Heartbeat goes to the XIAO module's onboard user LED (active-LOW). On S3 the
// onboard LED is GPIO 21 (LED_BUILTIN from the Seeed variant). This frees the
// D4 pad on the PCB as a fully-free GPIO (ADC1_CH4 on S3) for future use.
constexpr uint8_t LED_PIN      = LED_BUILTIN;
constexpr uint8_t PIN_NRF_CE   = D2;   // Radio1 CE
constexpr uint8_t PIN_NRF_CSN  = D3;   // Radio1 CSN
constexpr uint8_t PIN_NRF_CE2  = D0;   // Radio2 CE   (dual-radio PCB or higher)
constexpr uint8_t PIN_NRF_CSN2 = D1;   // Radio2 CSN  (dual-radio PCB or higher)
constexpr uint8_t PIN_NRF_CE3  = D4;   // Radio3 CE   (triple-radio PCB only; S3 only)
constexpr uint8_t PIN_NRF_CSN3 = D9;   // Radio3 CSN  (triple-radio PCB only; S3 only)
constexpr int8_t  PIN_SPI_SCK  = D8;   // shared
constexpr int8_t  PIN_SPI_MISO = D7;   // shared — kept OFF D9 because D9 is the C3 BOOT strap
constexpr int8_t  PIN_SPI_MOSI = D10;  // shared
constexpr int8_t  PIN_SBUS_TX  = D6;   // RC output (UART/RMT)
constexpr int8_t  PIN_FC_RX    = D5;   // FC telemetry RX (UART)

constexpr uint32_t NRF_SPI_HZ = 4000000;     // 4 MHz — conservative, eliminates marginal timing

//*********************************************************************
//  v1 protocol constants (mirror v1's utilities/1Definitions.h)
//*********************************************************************

constexpr uint8_t V1_DEFAULT_PIPE[5] = { 0x23, 0x94, 0x3e, 0xbe, 0xb7 };
constexpr uint8_t V1_RECOVERY_CH     = 82;     // FHSS_Recovery_Channels[2] — v1 starts Reconnect here
constexpr uint8_t V1_PIPE_NUMBER     = 1;

// v1 FHSS hop table — UK-friendly channels, in hop order.
constexpr uint8_t FHSS_CHANNELS[83] = {
    51, 28, 24, 61, 64, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13,  2,
    34, 47, 20, 16, 72, 35, 57, 45, 29, 75,  3, 41, 62, 11,  9, 77, 37,  8, 31, 36,
    18, 17, 50, 78, 73, 30, 79,  6, 23, 40, 54, 12, 80, 53, 22,  1, 74, 39, 58, 63,
    70, 52, 42, 25, 43, 26, 14, 38, 48, 68, 33, 27, 60, 44, 46, 56,  7, 81,  5, 65,
     4, 10,  0
};
constexpr uint8_t  HOP_TIME_MS         = 8;   // v1 HOPTIME → ~100 Hz FHSS
constexpr uint8_t  MAC_ACK_THRESHOLD   = 20;  // matches v1: MAC in first 20 acks, then telemetry rotation
constexpr uint8_t  MAX_TELEMETRY_ITEM  = 36;  // bumped from 35 to make room for case 36 = RX3 active time

// v1 channel 82 lives at index 14 of FHSS_CHANNELS. We bind there, then increment.
constexpr uint8_t  CHAN82_INDEX        = 14;

// RXV2 version stamp (RX board uniquely identifies itself to v1 TX).
constexpr uint8_t  RXV2_THIS_RADIO     = 1;
constexpr uint8_t  RXV2_V_MAJOR        = 2;
constexpr uint8_t  RXV2_V_MINOR        = 5;
constexpr uint8_t  RXV2_V_MINIMUS      = 6;       // must match TX's required version exactly — TX rejects any other value
constexpr char     RXV2_V_EXTRA        = 'R';     // R = RXV2 prototype
constexpr uint32_t RXV2_RECEIVER_TYPE  = 0x52580200;   // 'RX' 0x02 0x00 — sentinel

//*********************************************************************
//  Output protocol enum + persistence keys
//*********************************************************************
// FBUS and IBUS2 enum values are kept (so their parsers and case branches still
// compile) but are not user-selectable — PROTO_MAX gates the menu and the save
// validator. To re-enable them in the UI, raise PROTO_MAX.

enum Protocol : uint8_t {
    PROTO_SBUS  = 0,
    PROTO_CRSF  = 1,
    PROTO_IBUS  = 2,
    PROTO_PPM   = 3,
    PROTO_FBUS  = 4,
    PROTO_IBUS2 = 5,
};
constexpr uint8_t PROTO_MAX     = PROTO_PPM;
constexpr uint8_t PROTO_DEFAULT = PROTO_SBUS;

inline Protocol  currentProtocol = (Protocol)PROTO_DEFAULT;
inline bool      ppmInverted     = false;

//*********************************************************************
//  NVS keys (Preferences namespace = "rxv2")
//*********************************************************************

constexpr const char* NVS_NAMESPACE      = "rxv2";
constexpr const char* NVS_KEY_PIPE       = "pipe";
constexpr const char* NVS_KEY_SSID       = "ssid";
constexpr const char* NVS_KEY_PASS       = "pass";
constexpr const char* NVS_KEY_BOARD_ID   = "board_id";   // 6-byte board ID; captured first boot, never changes
constexpr const char* NVS_KEY_BOOT_COUNT = "qbc";        // quick-boot counter for escape hatch
constexpr const char* NVS_KEY_PROTO      = "proto";
constexpr const char* NVS_KEY_PPM_INV    = "ppm_inv";
constexpr const char* NVS_KEY_FW_MANIFEST = "fwurl";   // URL of dev firmware server's manifest.json

constexpr uint8_t     QUICK_BOOT_THRESHOLD = 3;
constexpr uint32_t    QUICK_BOOT_RESET_MS  = 5000;

inline Preferences prefs;
inline bool forceWifiMode = false;

//*********************************************************************
//  Channel-data periods + frame buffers
//*********************************************************************

constexpr uint32_t SBUS_PERIOD_MS = 14;     // ~71 Hz
constexpr uint32_t CRSF_PERIOD_MS = 4;      // ~250 Hz
constexpr uint32_t IBUS_PERIOD_MS = 7;      // ~140 Hz
constexpr uint32_t PPM_PERIOD_MS  = 25;     // 40 Hz — leaves 2-3 ms over the ~22 ms frame
constexpr uint32_t FBUS_PERIOD_MS = 9;      // ~111 Hz

inline uint16_t channelMicros[16];                       // initialised in setup() to 1500us
inline uint32_t lastChannelDataMs = 0;
inline uint8_t  sbusFrame[25];
inline uint8_t  crsfFrame[26];
inline uint8_t  ibusFrame[32];
inline uint8_t  fbusFrame[26];
inline uint32_t lastSbusMs    = 0;
inline uint32_t sbusFramesOut = 0;          // legacy name — counts ALL output frames

// PPM via RMT
constexpr uint32_t PPM_SYNC_US     = 300;
constexpr uint32_t PPM_FRAME_US    = 22500;
constexpr uint8_t  PPM_CHANNELS    = 8;
inline rmt_data_t  ppmItems[PPM_CHANNELS + 1];
inline bool        ppmRmtReady = false;
inline rmt_obj_t*  ppmRmt      = nullptr;

//*********************************************************************
//  RX statistics, board MAC, bind state
//*********************************************************************

struct RxStats {
    uint32_t packets       = 0;
    uint32_t lastMillis    = 0;
    uint8_t  lastPayload   = 0;
    uint8_t  lastBytes[32] = {0};
    uint8_t  maxPayload    = 0;
    uint8_t  maxBytes[32]  = {0};
    uint32_t acksWritten   = 0;
};
inline RxStats rx;

inline uint8_t boardMac[8] = {0};      // v1 sends our 8-byte board ID in early acks

struct BindStateT {
    bool     bound          = false;
    uint8_t  pipe[5]        = {0};
    uint32_t boundMillis    = 0;
    uint32_t attempts       = 0;
};
inline BindStateT bindState;

//*********************************************************************
//  Dual-radio state
//*********************************************************************
// Both radios share SCK/MOSI/MISO, each with its own CE/CSN. Radio1 is primary;
// Radio2 is optional (probed at boot). Both configured identically so we can
// swap CE between them on packet loss — same pattern as v1 on the Teensy.

inline RF24 radio1(PIN_NRF_CE,  PIN_NRF_CSN,  NRF_SPI_HZ);
inline RF24 radio2(PIN_NRF_CE2, PIN_NRF_CSN2, NRF_SPI_HZ);
inline RF24 radio3(PIN_NRF_CE3, PIN_NRF_CSN3, NRF_SPI_HZ);
inline RF24* radios[3]    = { &radio1, &radio2, &radio3 };
inline RF24* currentRadio = &radio1;

// Independent per-slot presence — probed at boot. radioPresent[i] is true
// iff the chip at slot i+1 responds on SPI. swapRadios() rotates only
// through present slots, so a missing or failed radio is skipped.
inline bool     radioPresent[3]      = { false, false, false };
inline uint8_t  numRadiosPresent     = 0;        // 0..3
inline uint8_t  activeRadioIdx       = 1;       // for UI display: 1, 2, or 3
inline uint32_t radioSwaps           = 0;
inline uint32_t lastRadioSwapMs      = 0;

// Per-radio active-time accumulators. radioActiveMs[i] counts ms that slot
// i+1 has spent as the currently-listening radio (across all swaps). The
// time accumulating for the active radio right now is held separately in
// radioActiveStartMs and is added in by radioElapsedSec() when read.
inline uint32_t radioActiveMs[3]     = { 0, 0, 0 };
inline uint32_t radioActiveStartMs   = 0;

constexpr uint32_t RADIO_SWAP_PACKET_TIMEOUT_MS = 200;
constexpr uint32_t RADIO_SWAP_COOLDOWN_MS       = 200;

//*********************************************************************
//  Ack-payload rotation state
//*********************************************************************

constexpr uint8_t ACK_PAYLOAD_BYTES = 6;     // v1 PAYLOADSIZE
inline uint8_t  ackByteZero    = 1;
// Run the MAC broadcast phase like v1 does — the v1 TX uses ack[0]==0 / ack[0]==1
// frames to capture the receiver's 8-byte MAC and decide bind success. With this
// initialised at 0 (not MAC_ACK_THRESHOLD), the first 20 acks send the real
// chip boardMac in two halves so the TX recognises the receiver and exits bind.
// (The earlier "skip MAC phase" workaround was for an old Model-ID stability
// issue that's since been fixed by NVS-caching the board MAC.)
inline uint32_t macAcksSent    = 0;
inline uint8_t  telemetryItem  = 0;
inline uint8_t  nextChannelIdx = CHAN82_INDEX;
inline uint32_t lastHopMs      = 0;
inline bool     hopPending     = false;
inline bool     fhssEnabled    = false;      // off until link is proven on a fixed channel

//*********************************************************************
//  Radio self-test result
//*********************************************************************

struct RadioSelfTest {
    bool    ran           = false;
    bool    beginOk       = false;
    bool    chipConnected = false;
    bool    channelOk     = false;
    bool    dataRateOk    = false;
    uint8_t channelRead   = 0xFF;
    uint8_t dataRateRead  = 0xFF;
    const char* verdict   = "not run";
};
inline RadioSelfTest rfTest;

//*********************************************************************
//  FC telemetry (CRSF inbound)
//*********************************************************************

struct FcTelem {
    bool     valid          = false;
    float    fcBattVolts    = 0.0f;
    float    fcBattAmps     = 0.0f;
    uint32_t fcBattMah      = 0;
    uint8_t  fcBattPct      = 0;
    int8_t   fcUplinkRssi   = 0;
    uint8_t  fcUplinkLq     = 0;
    int8_t   fcUplinkSnr    = 0;
    int16_t  attitudePitch  = 0;        // mrad
    int16_t  attitudeRoll   = 0;
    int16_t  attitudeYaw    = 0;
    char     flightMode[16] = {0};
    uint32_t framesParsed   = 0;
    uint32_t bytesIn        = 0;
    uint32_t crcErrors      = 0;
    uint32_t responsesSent  = 0;
    uint32_t lastFrameMs    = 0;
};
inline FcTelem fcTelem;

// Raw byte ring buffer — captures every byte received on D5 regardless of
// protocol. Lets us reverse-engineer FC traffic in modes where our parser is
// wrong or absent. Surfaced on /diagnostics as the "Recent raw" hex line.
constexpr size_t FC_RAW_RING_SIZE = 128;
inline uint8_t  fcRawRing[FC_RAW_RING_SIZE] = {0};
inline uint16_t fcRawHead   = 0;
inline uint16_t fcRawCount  = 0;
inline uint32_t fcRawTotal  = 0;

// Shared 128-byte byte-stream buffer used by all per-protocol parsers (one
// protocol is active at a time, so they don't collide).
inline uint8_t  fcRxBuf[128];
inline uint8_t  fcRxLen = 0;

// IBUS2 sensor protocol
struct IbusSlot {
    uint8_t type;       // FlySky sensor type code
    uint8_t dataLen;    // payload length in bytes (2 or 4)
};
constexpr IbusSlot IBUS_SLOTS[] = {
    { 0x03, 2 },   // slot 1: external voltage in 0.01 V units
    { 0x09, 2 },   // slot 2: RF link quality 0..100
};
constexpr uint8_t IBUS_NUM_SLOTS = sizeof(IBUS_SLOTS) / sizeof(IBUS_SLOTS[0]);
inline uint8_t ibusRxBuf[4];
inline uint8_t ibusRxIdx = 0;

//*********************************************************************
//  Event log (Black-box page)
//*********************************************************************
// ~2.5 KB in-RAM ring of textual events. Useful for "what happened in the last
// few seconds" debugging without serial.

struct EventLog {
    static constexpr size_t SIZE     = 32;
    static constexpr size_t MSG_LEN  = 80;
    char     msgs[SIZE][MSG_LEN] = {};
    uint32_t when[SIZE]          = {};
    size_t   head                = 0;
    size_t   count               = 0;

    void add(const char* msg) {
        when[head] = millis();
        strncpy(msgs[head], msg, MSG_LEN - 1);
        msgs[head][MSG_LEN - 1] = '\0';
        head = (head + 1) % SIZE;
        if (count < SIZE) count++;
    }
};
inline EventLog events;

//*********************************************************************
//  Web server, boot millis, build-days
//*********************************************************************

inline WebServer server(80);
inline uint32_t  bootMillis = 0;
inline uint32_t  buildDays  = 0;     // days since 2020-01-01, populated in setup()
inline bool      httpServerStarted = false;
inline volatile bool flyArmRequested = false;
inline bool      littleFsMounted = false;

//*********************************************************************
//  MSP bridge (TCP server on port 5760 → CRSF UART → FC)
//*********************************************************************
// When CRSF is the selected output protocol and a Rotorflight Configurator
// (or any MSP client) connects to port 5760, we become a transparent byte
// pipe: TCP bytes get written to Serial1 (D6 → FC), and bytes arriving on
// Serial1 from the FC (D5) get forwarded to the TCP client in addition to
// the local telemetry parser. RC frame transmission on D6 is suspended
// while a client is connected (you're configuring, not flying).
//
// Port 5760 is the Rotorflight / Betaflight standard for TCP-MSP.

constexpr uint16_t MSP_BRIDGE_PORT = 5760;

inline WiFiServer mspBridge(MSP_BRIDGE_PORT);
inline WiFiClient mspClient;
inline bool       mspBridgeStarted = false;     // server listening
inline bool       mspBridgeActive  = false;     // a client is connected RIGHT NOW
inline uint32_t   mspBridgeBytesIn  = 0;        // FC -> TCP (telemetry/MSP responses)
inline uint32_t   mspBridgeBytesOut = 0;        // TCP -> FC (MSP requests)
inline uint32_t   mspBridgeConnectMs = 0;       // when current client connected
inline uint32_t   mspBridgeConnections = 0;     // total connections since boot

//*********************************************************************
//  FC detection (MSP-over-CRSF probe)
//*********************************************************************

struct FcInfo {
    bool     detected         = false;
    bool     versionKnown     = false;
    char     variant[5]       = {0};     // e.g. "RTFL" / "BTFL" / "INAV"
    uint8_t  fwMajor          = 0;       // firmware semver (e.g. 2.3.0)
    uint8_t  fwMinor          = 0;
    uint8_t  fwPatch          = 0;
    uint8_t  mspProto         = 0;       // MSP protocol version
    uint8_t  apiMajor         = 0;       // MSP API version (separate from fw version)
    uint8_t  apiMinor         = 0;
    uint32_t probesSent       = 0;
    uint32_t lastProbeMs      = 0;
    uint32_t lastResponseMs   = 0;
};
inline FcInfo fcInfo;

//*********************************************************************
//  Cross-module forward declarations
//*********************************************************************
// Functions defined in one module header but called from another. Listed here
// so include-order between module headers doesn't matter.

void     loadNextAck();
void     decompress(uint16_t* out, const uint16_t* in, uint8_t outSize);
uint8_t  decompressedSize(uint8_t payloadBytes);
void     startApMode();
void     disableWifi();
void     saveBindToNvs();
void     clearBindNvs();
void     loadBindFromNvs();
String   getEffectiveSsid();
String   getEffectivePass();
bool     wifiCredsAreCustom();
const char* protocolName(Protocol p);
const char* protocolDesc(Protocol p);
const char* netModeName();
String   uptimeString();
void     mspBridgeStart();
void     mspBridgePoll();
void     mspBridgeOnFcByte(uint8_t b);
void     mspParseResponse(const uint8_t* body, uint8_t bodyLen);
void     mspFcPoll();
bool     fcIsRotorflightConfigCapable();
void     ledOn();
void     ledOff();

//*********************************************************************
//  Small helpers used everywhere
//*********************************************************************

// Pack a uint32 into bytes [1..4] of an ack-payload buffer, little-endian
// (matches v1's SendIntToAckPayload which uses a union { uint32_t; uint8_t[4]; }).
inline void packU32(uint8_t* ack, uint32_t v) {
    ack[1] = (uint8_t)(v       & 0xff);
    ack[2] = (uint8_t)((v >> 8)  & 0xff);
    ack[3] = (uint8_t)((v >> 16) & 0xff);
    ack[4] = (uint8_t)((v >> 24) & 0xff);
}
inline void packF32(uint8_t* ack, float f) {
    uint32_t v;
    memcpy(&v, &f, sizeof(v));
    packU32(ack, v);
}

#endif // _SRC_1DEFS_H
