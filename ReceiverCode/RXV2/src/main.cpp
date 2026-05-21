// LockDownRadioControl — Receiver V2 bootstrap
//
// First-stage firmware for the XIAO ESP32-C3 in the tiny RX.
// Brings up WiFi, mDNS, ArduinoOTA and a small status web server so all
// subsequent firmware can be pushed wirelessly from the dev machine.
//
// Final RX behaviour (not yet here): if no TX seen for 30 s, enable WiFi.
// If a bound TX is heard, disable WiFi and run the RF24 link.
//
// XIAO ESP32-C3 <-> EBYTE E01-ML01DP5 wiring (agreed pin map, revised 2026-05-21):
//   VCC   -> 3V3 pad                 (3.3 V only, never 5 V)
//   GND   -> GND pad
//   CE    -> D2  (GPIO 4)
//   CSN   -> D3  (GPIO 5)
//   SCK   -> D8  (GPIO 8)            shares with XIAO user LED, acceptable
//   MISO  -> D7  (GPIO 20)           moved off D9 — see note below
//   MOSI  -> D10 (GPIO 10)
//   IRQ   -> NC                      v1 masks all IRQs; we will too
//
// SBUS OUT:
//   D6  (GPIO 21) — inverted UART TX at 100 kbaud, 8E2 — connect to SBUS input on
//                   simulator / future flight-controller. 16 channels @ ~71 Hz.
//
// RX BATTERY VOLTAGE SENSE:
//   D0  (GPIO 2)  — ADC1 channel. External divider sized for up to 12S LiPo (50.4 V max):
//                   V_batt ─[22 kΩ]─┬─[D0]
//                                   └─[1.2 kΩ]─ GND
//                   (and a 100 nF cap from D0 to GND, close to the XIAO).
//
// Status LED on D4 (GPIO 6).
//
// Why MISO is NOT on D9 (GPIO 9):
//   GPIO 9 on the ESP32-C3 doubles as the BOOT strap pin sampled at every reset.
//   If LOW at reset, the chip enters USB-download mode instead of running flash.
//   With the nRF24's MISO output wired there, transient LOW levels during the
//   module's power-on-reset can drag GPIO 9 down at exactly the wrong moment,
//   making cold-boot unreliable. Routing MISO through the GPIO matrix to D7
//   (GPIO 20) removes the conflict; D7 has no strap function.

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>
#include <Preferences.h>
#include <esp_ota_ops.h>

namespace {

// WiFi credentials live only in NVS — never compiled into the firmware. On a fresh
// chip (no NVS entry), the boot flow goes directly to AP mode so the user can enter
// their network details via the web UI.
constexpr const char* WIFI_DEFAULT_SSID     = "";
constexpr const char* WIFI_DEFAULT_PASSWORD = "";
constexpr const char* AP_SSID               = "LDRC_RX";    // fallback AP if STA fails
constexpr const char* OTA_HOSTNAME          = "LDRC_RX";    // -> http://LDRC_RX.local/ (matches the AP SSID for consistency)
constexpr const char* OTA_PASSWORD          = nullptr;      // set to a string to require auth

// Network state machine. The chip listens for the TX for ~10 s on boot; if a packet
// arrives in that window we skip WiFi entirely for this session (RF-only flight mode).
// Otherwise we try the saved/default STA WiFi, and finally fall back to an AP so the
// user can re-enter credentials. The user can also force WiFi off mid-session via /fly_arm.
enum NetMode : uint8_t {
    NET_INIT,
    NET_WAITING_RF,           // initial 10 s window
    NET_NO_WIFI,              // TX found in boot window — or user disabled WiFi
    NET_WIFI_CONNECTING,
    NET_WIFI_UP,
    NET_AP
};
NetMode  netMode       = NET_INIT;
uint32_t netStateStart = 0;
bool     otaStarted    = false;

constexpr uint32_t RF_WINDOW_MS         = 10000;   // boot window before falling through to WiFi
constexpr uint32_t WIFI_CONNECT_MS      = 12000;   // STA timeout before giving up and going to AP

const char* netModeName() {
    switch (netMode) {
        case NET_INIT:             return "init";
        case NET_WAITING_RF:       return "waiting for TX";
        case NET_NO_WIFI:          return "RF-only (no WiFi)";
        case NET_WIFI_CONNECTING:  return "connecting WiFi";
        case NET_WIFI_UP:          return "WiFi up";
        case NET_AP:               return "AP mode";
    }
    return "?";
}

constexpr const char* FW_VERSION    = "RXV2-0.1.26-no-default-creds";

constexpr uint8_t LED_PIN      = 6;   // D4 / GPIO 6 — separate heartbeat LED
constexpr uint8_t PIN_NRF_CE   = 4;   // D2
constexpr uint8_t PIN_NRF_CSN  = 5;   // D3
constexpr int8_t  PIN_SPI_SCK  = 8;   // D8
constexpr int8_t  PIN_SPI_MISO = 20;  // D7 — moved off D9/BOOT strap (see header)
constexpr int8_t  PIN_SPI_MOSI = 10;  // D10
constexpr int8_t  PIN_SBUS_TX  = 21;  // D6 — inverted SBUS UART output
constexpr int8_t  PIN_VBATT    = 2;   // D0 — ADC1_CH2, RX battery voltage divider midpoint

// Voltage divider scaling: V_batt = V_adc × (R1 + R2) / R2.
// Default values for the recommended 22 kΩ / 1.2 kΩ divider:
constexpr float   VBATT_DIVIDER_R1 = 22000.0f;
constexpr float   VBATT_DIVIDER_R2 =  1200.0f;
constexpr float   VBATT_SCALE      = (VBATT_DIVIDER_R1 + VBATT_DIVIDER_R2) / VBATT_DIVIDER_R2;

constexpr uint32_t NRF_SPI_HZ = 4000000;     // 4 MHz — conservative, eliminates marginal-timing failures
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN, NRF_SPI_HZ);

// --- v1 protocol constants (mirror ReceiverCode/src/utilities/1Definitions.h) ---
constexpr uint8_t V1_DEFAULT_PIPE[5] = { 0x23, 0x94, 0x3e, 0xbe, 0xb7 };
constexpr uint8_t V1_RECOVERY_CH     = 82;     // FHSS_Recovery_Channels[2] — same channel v1 starts Reconnect on
constexpr uint8_t V1_PIPE_NUMBER     = 1;

// v1 FHSS hop table — UK-friendly channels, in hop order (mirrors FHSS_Channels[] in v1).
constexpr uint8_t FHSS_CHANNELS[83] = {
    51, 28, 24, 61, 64, 55, 66, 19, 76, 21, 59, 67, 15, 71, 82, 32, 49, 69, 13,  2,
    34, 47, 20, 16, 72, 35, 57, 45, 29, 75,  3, 41, 62, 11,  9, 77, 37,  8, 31, 36,
    18, 17, 50, 78, 73, 30, 79,  6, 23, 40, 54, 12, 80, 53, 22,  1, 74, 39, 58, 63,
    70, 52, 42, 25, 43, 26, 14, 38, 48, 68, 33, 27, 60, 44, 46, 56,  7, 81,  5, 65,
     4, 10,  0
};
constexpr uint8_t  HOP_TIME_MS         = 8;   // v1 HOPTIME → ~100 Hz FHSS
constexpr uint8_t  MAC_ACK_THRESHOLD   = 20;  // matches v1: MAC sent in first 20 acks, then telemetry rotation
constexpr uint8_t  MAX_TELEMETRY_ITEM  = 35;  // v1 MAX_TELEMETERY_ITEMS

// v1 channel 82 lives at index 14 of FHSS_CHANNELS. We bind there, then increment from there.
constexpr uint8_t  CHAN82_INDEX        = 14;

// RXV2 version stamp (RX board uniquely identifies itself to v1 TX)
constexpr uint8_t  RXV2_THIS_RADIO     = 1;
constexpr uint8_t  RXV2_V_MAJOR        = 2;
constexpr uint8_t  RXV2_V_MINOR        = 5;
constexpr uint8_t  RXV2_V_MINIMUS      = 6;       // matches v1 minimum that TX requires
constexpr char     RXV2_V_EXTRA        = 'R';     // R = RXV2 prototype (TX shows this as the trailing letter)
constexpr uint32_t RXV2_RECEIVER_TYPE  = 0x52580200;   // 'RX' 0x02 0x00 — sentinel

struct RxStats {
    uint32_t packets       = 0;
    uint32_t lastMillis    = 0;
    uint8_t  lastPayload   = 0;
    uint8_t  lastBytes[32] = {0};
    uint8_t  maxPayload    = 0;        // biggest payload seen (useful for spotting bind frames)
    uint8_t  maxBytes[32]  = {0};      // contents of that biggest packet
    uint32_t acksWritten   = 0;
};
RxStats rx;

// v1 sends its 8-byte board ID as the first 20-or-so ack payloads so the TX can
// recognise "which receiver am I bound to". ESP32 WiFi MAC is 6 bytes; pad to 8
// with zeros, mirroring v1's teensyMAC() helper.
uint8_t boardMac[8] = {0};

// Bind state — set when we extract a new 5-byte pipe address from a TX bind packet.
struct BindStateT {
    bool     bound          = false;
    uint8_t  pipe[5]        = {0};
    uint32_t boundMillis    = 0;
    uint32_t attempts       = 0;
};
BindStateT bindState;

// Small in-RAM event log for the Black-box page. ~2.5 KB total.
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
EventLog events;

// NVS-backed bind persistence — bound pipe survives reboots.
Preferences prefs;
constexpr const char* NVS_NAMESPACE = "rxv2";
constexpr const char* NVS_KEY_PIPE  = "pipe";

// WiFi credentials helpers — keys live alongside the bind pipe in the same NVS namespace.
constexpr const char* NVS_KEY_SSID       = "ssid";
constexpr const char* NVS_KEY_PASS       = "pass";

// Quick-boot escape hatch: 3 power cycles within 5 s force WiFi regardless of RF.
// Lets us recover access if the TX is on at boot and locks WiFi off.
constexpr const char* NVS_KEY_BOOT_COUNT = "qbc";
constexpr uint8_t     QUICK_BOOT_THRESHOLD = 3;
constexpr uint32_t    QUICK_BOOT_RESET_MS  = 5000;
bool forceWifiMode = false;

String getEffectiveSsid() {
    String s = prefs.isKey(NVS_KEY_SSID) ? prefs.getString(NVS_KEY_SSID, "") : "";
    return s.length() ? s : String(WIFI_DEFAULT_SSID);
}
String getEffectivePass() {
    String p = prefs.isKey(NVS_KEY_PASS) ? prefs.getString(NVS_KEY_PASS, "") : "";
    return p.length() ? p : String(WIFI_DEFAULT_PASSWORD);
}
bool wifiCredsAreCustom() {
    return prefs.isKey(NVS_KEY_SSID) && prefs.getString(NVS_KEY_SSID, "").length() > 0;
}

void loadBindFromNvs() {
    if (prefs.isKey(NVS_KEY_PIPE) && prefs.getBytesLength(NVS_KEY_PIPE) == 5) {
        prefs.getBytes(NVS_KEY_PIPE, bindState.pipe, 5);
        bindState.bound       = true;
        bindState.boundMillis = millis();
        Serial.printf("[bind] restored from NVS: %02X %02X %02X %02X %02X\n",
                      bindState.pipe[0], bindState.pipe[1], bindState.pipe[2],
                      bindState.pipe[3], bindState.pipe[4]);
    }
}

void saveBindToNvs() {
    prefs.putBytes(NVS_KEY_PIPE, bindState.pipe, 5);
    Serial.println("[bind] saved to NVS");
}

void clearBindNvs() {
    prefs.remove(NVS_KEY_PIPE);
    bindState.bound       = false;
    bindState.boundMillis = 0;
    memset(bindState.pipe, 0, sizeof(bindState.pipe));
    Serial.println("[bind] cleared NVS");
}

// Forward declarations — referenced before their definitions further down.
void loadNextAck();
void decompress(uint16_t* out, const uint16_t* in, uint8_t outSize);
uint8_t decompressedSize(uint8_t payloadBytes);
extern volatile bool flyArmRequested;
void startApMode();

// Compute "days since 2020-01-01" from the compiler's __DATE__ string (e.g. "May 21 2026").
// v1 reports the same quantity as telemetry item 35 (BuildAge); the TX subtracts the two
// and warns if they differ by more than a few days. Computed once at boot.
uint32_t computeBuildDays() {
    const char* d = __DATE__;                            // "Mmm dd yyyy"  (dd has a leading space if single digit)
    static const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    int month = 0;
    for (int i = 0; i < 12; ++i) {
        if (d[0] == months[i*3] && d[1] == months[i*3+1] && d[2] == months[i*3+2]) {
            month = i + 1;
            break;
        }
    }
    int day  = atoi(d + 4);
    int year = atoi(d + 7);

    auto isLeap = [](int y) {
        return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
    };
    static const int dim[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    uint32_t total = 0;
    for (int y = 2020; y < year; ++y) total += isLeap(y) ? 366 : 365;
    for (int m = 0; m < month - 1; ++m) {
        total += dim[m];
        if (m == 1 && isLeap(year)) total += 1;
    }
    total += day - 1;
    return total;
}
uint32_t buildDays = 0;                                  // populated in setup()

// --- Channel data + SBUS output ---
// v1 sends each channel as a 12-bit microsecond value (500..2500). We hold those
// values verbatim here; SBUS just remaps to 0..2047 before transmission.
uint16_t channelMicros[16];                              // initialised in setup() to 1500us
uint32_t lastChannelDataMs = 0;

constexpr uint32_t SBUS_PERIOD_MS = 14;                  // ~71 Hz frame rate
uint8_t  sbusFrame[25];
uint32_t lastSbusMs    = 0;
uint32_t sbusFramesOut = 0;

// --- RX battery voltage telemetry ---
float    vbattVolts    = 0.0f;        // filtered, in V — what we send to the TX via case 5
uint32_t vbattLastMv   = 0;           // last raw mV reading at the ADC pin (debug)
uint32_t lastVbattMs   = 0;

void readVbatt() {
    constexpr uint32_t VBATT_PERIOD_MS = 100;     // 10 Hz sample rate
    if ((uint32_t)(millis() - lastVbattMs) < VBATT_PERIOD_MS) return;
    lastVbattMs = millis();

    // analogReadMilliVolts uses the chip's burned-in calibration constants — much more
    // linear than raw analogRead, and reads in mV directly.
    uint32_t mv = analogReadMilliVolts(PIN_VBATT);
    vbattLastMv = mv;

    float instant = (mv / 1000.0f) * VBATT_SCALE;
    // Light IIR filter — 0.9 / 0.1 gives a settle time of ~0.5 s at 10 Hz sampling.
    if (vbattVolts == 0.0f) vbattVolts = instant;
    else                    vbattVolts = 0.9f * vbattVolts + 0.1f * instant;
}

// Decode a received packet into channelMicros[]. Mirrors v1's
// Decompress() + RearrangeTheChannels() + (no MapToSBUS — done at frame build time).
void decodeChannelData(const uint8_t* payload, uint8_t size) {
    if (size < 4) return;
    uint16_t mask = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    if (mask == 0) return;                               // parameter packet, not channel data

    uint16_t compressed[16] = {0};
    uint8_t  nData = size - 2;
    for (uint8_t i = 0; i + 1 < nData; i += 2) {
        compressed[i / 2] = (uint16_t)payload[2 + i] | ((uint16_t)payload[2 + i + 1] << 8);
    }
    uint16_t raw[24] = {0};
    decompress(raw, compressed, decompressedSize(size));

    uint8_t p = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (mask & (1u << i)) {
            channelMicros[i] = raw[p++];
        }
    }
    lastChannelDataMs = millis();
}

// Pack 16 channels of 11 bits each into the SBUS 25-byte frame.
void buildSbusFrame(bool frameLost, bool failsafe) {
    memset(sbusFrame, 0, sizeof(sbusFrame));
    sbusFrame[0] = 0x0F;                                 // start byte

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
    sbusFrame[24] = 0x00;                                // end byte
}

void sbusTick() {
    if ((uint32_t)(millis() - lastSbusMs) < SBUS_PERIOD_MS) return;
    lastSbusMs = millis();

    uint32_t age = millis() - lastChannelDataMs;
    bool frameLost = age > 100;                          // no data for >100ms
    bool failsafe  = age > 500;                          // no data for >500ms

    buildSbusFrame(frameLost, failsafe);
    Serial1.write(sbusFrame, sizeof(sbusFrame));
    sbusFramesOut++;
}

// 3:4 packed decompression — mirrors v1's Decompress(): 3 uint16_t input → 4 12-bit output.
void decompress(uint16_t* out, const uint16_t* in, uint8_t outSize) {
    uint8_t compressedSize = (outSize * 3) / 4;
    uint8_t p = 0;
    for (uint8_t i = 0; i + 2 < compressedSize; i += 3) {
        uint16_t w0 = in[i], w1 = in[i + 1], w2 = in[i + 2];
        out[p++] = w0 >> 4;
        out[p++] = ((w0 & 0x0F) << 8) | (w1 >> 8);
        out[p++] = ((w1 & 0xFF) << 4) | (w2 >> 12);
        out[p++] = w2 & 0x0FFF;
    }
}

// v1 GetDecompressedSize: number of 12-bit values we should expect, given the
// raw payload size (which is ChannelBitMask 2 bytes + 3:4 packed data).
uint8_t decompressedSize(uint8_t payloadBytes) {
    if (payloadBytes <= 2) return 0;
    uint8_t ds = (((uint16_t)(payloadBytes - 2) * 4) / 3) / 2;
    while (ds % 4) ++ds;
    return ds;
}

// If we're unbound and the packet looks like a bind frame (channels 1..5 carry
// the new 5-byte pipe address), extract the pipe and re-open the reading pipe.
// Mirrors v1's GetNewPipe() flow.
void tryBind(const uint8_t* payload, uint8_t size) {
    if (bindState.bound)   return;
    if (size < 10)    return;                    // bind frames are 10+ bytes (5 ch × 12 bits + bitmask + padding)
    uint16_t mask = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    if ((mask & 0x003E) != 0x003E) return;       // channels 1..5 must all be flagged as updated

    bindState.attempts++;

    // Pull the 3:4-packed data into uint16_t words.
    uint16_t compressed[16] = {0};
    uint8_t  nDataBytes = size - 2;
    for (uint8_t i = 0; i + 1 < nDataBytes; i += 2) {
        compressed[i / 2] = (uint16_t)payload[2 + i] | ((uint16_t)payload[2 + i + 1] << 8);
    }

    uint16_t raw[24] = {0};
    decompress(raw, compressed, decompressedSize(size));

    // Rearrange into received-per-channel order, then take channels 1..5.
    uint16_t channel[16] = {0};
    uint8_t p = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if (mask & (1u << i)) {
            channel[i] = raw[p++];
        }
    }

    // v1 GetNewPipe reverses the byte order when copying into the pipe address.
    for (uint8_t i = 0; i < 5; ++i) {
        bindState.pipe[4 - i] = (uint8_t)(channel[i + 1] & 0xff);
    }

    // Switch the radio's reading pipe to the new address. We stay on channel 82 for now.
    radio.stopListening();
    delayMicroseconds(150);
    radio.flush_tx();
    radio.flush_rx();
    radio.openReadingPipe(V1_PIPE_NUMBER, bindState.pipe);
    radio.startListening();
    delayMicroseconds(150);

    // Top up the ack-payload FIFO for the new pipe.
    loadNextAck();
    loadNextAck();
    loadNextAck();

    bindState.bound       = true;
    bindState.boundMillis = millis();
    saveBindToNvs();

    char buf[80];
    snprintf(buf, sizeof(buf), "Bound to TX, pipe %02X %02X %02X %02X %02X",
             bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4]);
    events.add(buf);

    Serial.printf("[bind] new pipe %02X %02X %02X %02X %02X (after %u attempts)\n",
                  bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4],
                  (unsigned)bindState.attempts);
}

// Pack a uint32 into bytes [1..4] of an ack-payload buffer, little-endian — matches
// v1's SendIntToAckPayload (which uses a union { uint32_t; uint8_t[4]; }).
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

constexpr uint8_t ACK_PAYLOAD_BYTES = 6;     // v1 PAYLOADSIZE
uint8_t  ackByteZero    = 1;                 // toggles 0/1 each ack — selects which MAC half goes out
uint32_t macAcksSent    = 0;                 // count of MAC-style acks already loaded
uint8_t  telemetryItem  = 0;                 // current rotating telemetry item
uint8_t  nextChannelIdx = CHAN82_INDEX;      // current FHSS index — both ends start here
uint32_t lastHopMs      = 0;
bool     hopPending     = false;             // we set HOP bit in last ack; do the actual stopListening/setChannel after
bool     fhssEnabled    = false;             // disable FHSS for now — keep link on channel 82 while we sort out the telemetry handshake

struct RadioSelfTest {
    bool    ran           = false;
    bool    beginOk       = false;     // radio.begin() returned true
    bool    chipConnected = false;     // isChipConnected() after init
    bool    channelOk     = false;     // write 76, read back 76
    bool    dataRateOk    = false;     // write 250 kbps, read back 250 kbps
    uint8_t channelRead   = 0xFF;
    uint8_t dataRateRead  = 0xFF;
    const char* verdict   = "not run";
};
RadioSelfTest rfTest;

const char* dataRateName(rf24_datarate_e d) {
    switch (d) {
        case RF24_1MBPS:   return "1 Mbps";
        case RF24_2MBPS:   return "2 Mbps";
        case RF24_250KBPS: return "250 kbps";
        default:           return "?";
    }
}

void runRadioSelfTest() {
    rfTest.ran = true;

    // CSN idle-high before SPI talks to it.
    pinMode(PIN_NRF_CSN, OUTPUT);
    digitalWrite(PIN_NRF_CSN, HIGH);
    pinMode(PIN_NRF_CE,  OUTPUT);
    digitalWrite(PIN_NRF_CE,  LOW);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);   // override variant defaults; route MISO off the BOOT-strap pin

    delay(50);                   // generous power-on-reset for the E01 module (datasheet says >100 ms ideally, but Vcc has been up since boot — 50 ms is plenty)

    rfTest.beginOk = radio.begin();
    if (rfTest.beginOk) {
        radio.printDetails();    // dump register state to serial for diagnosis
    } else {
        rfTest.verdict = "radio.begin() failed — see wiring checklist";
        Serial.println("[rf] begin() FAILED — chip not responding on SPI");
        return;
    }

    rfTest.chipConnected = radio.isChipConnected();

    // Channel register read/write
    radio.setChannel(76);
    delayMicroseconds(150);
    rfTest.channelRead = radio.getChannel();
    rfTest.channelOk   = (rfTest.channelRead == 76);

    // Data rate register read/write
    radio.setDataRate(RF24_250KBPS);
    delayMicroseconds(150);
    rf24_datarate_e dr = radio.getDataRate();
    rfTest.dataRateRead = static_cast<uint8_t>(dr);
    rfTest.dataRateOk   = (dr == RF24_250KBPS);

    bool allOk = rfTest.beginOk && rfTest.chipConnected
              && rfTest.channelOk && rfTest.dataRateOk;
    rfTest.verdict = allOk
        ? "PASS — radio responds, registers writable, ready to receive"
        : "FAIL — see details (likely a wiring fault)";

    Serial.printf("[rf] begin=%d chip=%d ch=%d dr=%d  %s\n",
                  rfTest.beginOk, rfTest.chipConnected,
                  rfTest.channelOk, rfTest.dataRateOk,
                  rfTest.verdict);
}

// Build the next ack payload — mirrors v1's LoadAckPayload().
//  First MAC_ACK_THRESHOLD acks carry our 8-byte board ID (so TX can identify us).
//  After that, we rotate items 0..35 (version, packet count, voltages, baro, GPS, etc.).
//  Items we don't have sensors for return zeros — v1 ignores zero values cleanly.
//  byte[0] also carries the FHSS HOP flag (high bit) when it's time to switch channel;
//  byte[5] carries the FHSS table index so the TX knows where we're going.
void loadNextAck() {
    uint8_t ack[ACK_PAYLOAD_BYTES] = {0};

    // FHSS hop decision — gated on a runtime flag. The v1 TX's hop-sync model is not
    // yet confirmed; first prove the link stays up on a fixed channel with the
    // telemetry rotation alone, then re-enable hopping.
    extern bool fhssEnabled;
    bool hopThisAck = false;
    if (fhssEnabled && macAcksSent >= MAC_ACK_THRESHOLD &&
        (uint32_t)(millis() - lastHopMs) >= HOP_TIME_MS) {
        nextChannelIdx = (nextChannelIdx + 1) % 83;
        hopThisAck = true;
    }

    if (macAcksSent < MAC_ACK_THRESHOLD) {
        // MAC ack format (v1 SendMacAddress)
        ackByteZero = ackByteZero ? 0 : 1;        // toggles 0,1,0,1 → which MAC half
        ack[0] = ackByteZero;                     // HOP bit stays clear during MAC phase
        uint8_t base = ackByteZero ? 4 : 0;
        ack[1] = boardMac[base + 0];
        ack[2] = boardMac[base + 1];
        ack[3] = boardMac[base + 2];
        ack[4] = boardMac[base + 3];
        macAcksSent++;
    } else {
        // Telemetry rotation (v1 LoadAckPayload switch, simplified — sensor-less items return zero)
        if (++telemetryItem > MAX_TELEMETRY_ITEM) telemetryItem = 0;
        ack[0] = telemetryItem;                   // HOP bit will be OR'd in below if hopping
        bool versionCase = false;
        switch (telemetryItem) {
            case 0:   // version — v1 puts the EXTRA ASCII char in byte[5], not the FHSS index
                ack[1] = RXV2_THIS_RADIO;
                ack[2] = RXV2_V_MAJOR;
                ack[3] = RXV2_V_MINOR;
                ack[4] = RXV2_V_MINIMUS;
                ack[5] = (uint8_t)RXV2_V_EXTRA;
                versionCase = true;
                break;
            case 1:   packU32(ack, rx.packets);                     break;  // SuccessfulPackets
            case 2:   packU32(ack, 0);                              break;  // RadioSwaps
            case 3:   packU32(ack, millis() / 1000);                break;  // RX1 uptime sec
            case 4:   packU32(ack, 0);                              break;  // RX2 uptime
            case 5: {                                                       // RX battery volts (v1 TX has a backward-compat
                // quirk: if it receives a value above 6S max it assumes the RX is 12S and was pre-halved,
                // so it multiplies by 2. We pre-halve here so the TX's ×2 produces the right number either way.)
                constexpr float V_6S_MAX = 25.2f;        // 6 × 4.2 V — threshold where v1 TX flips to 12S interpretation
                float vTx = (vbattVolts > V_6S_MAX) ? (vbattVolts * 0.5f) : vbattVolts;
                packF32(ack, vTx);
                break;
            }
            case 6:   packF32(ack, 0.0f);                           break;  // baro altitude
            case 7:   packF32(ack, 0.0f);                           break;  // baro temperature
            case 19:  packF32(ack, 0.0f);                           break;  // rate of climb
            case 23:  packU32(ack, RXV2_RECEIVER_TYPE);             break;  // Receiver_Type sentinel
            case 24:  packF32(ack, 0.0f);                           break;  // ESC temp
            case 31:  packU32(ack, 0);                              break;  // Rotorflight version (we don't have RF)
            case 35:  packU32(ack, buildDays);                      break;  // BuildAge in days since 2020-01-01
            // Items 8..18, 20..22, 25..30, 32..34 are sensor/RF-specific — leave as zero.
            default:  break;
        }
        if (hopThisAck) {
            ack[0] |= 0x80;        // HOP flag — TX hops to FHSS_CHANNELS[ack[5]] after this packet
            hopPending = true;
        }
        if (!versionCase) {
            ack[5] = nextChannelIdx;   // FHSS table index goes here for everything except version
        }
    }

    if (radio.writeAckPayload(V1_PIPE_NUMBER, ack, ACK_PAYLOAD_BYTES)) {
        rx.acksWritten++;
    }
}

// Configure the radio for v1-compatible listen on the bind pipe.
// Mirrors ConfigureRadio() in v1's ReceiverCode/src/utilities/radio.h.
void radioBeginListenV1() {
    if (!rfTest.beginOk) return;

    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.enableAckPayload();
    radio.setRetries(2, 2);
    radio.enableDynamicPayloads();
    radio.setAddressWidth(5);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    radio.maskIRQ(1, 1, 1);
    radio.setChannel(V1_RECOVERY_CH);
    // If NVS gave us a bound pipe, open it directly so the TX can pick the link up
    // without needing to re-bind through the DefaultPipe.
    const uint8_t* pipe = bindState.bound ? bindState.pipe : V1_DEFAULT_PIPE;
    radio.openReadingPipe(V1_PIPE_NUMBER, pipe);
    radio.startListening();
    Serial.printf("[rf] %s pipe %02X %02X %02X %02X %02X on channel %u\n",
                  bindState.bound ? "bound" : "default",
                  pipe[0], pipe[1], pipe[2], pipe[3], pipe[4], V1_RECOVERY_CH);
    (void)V1_DEFAULT_PIPE;       // referenced through the ternary above

    // Pre-load ack-payload FIFO so the very first incoming packet's ACK already
    // carries MAC bytes — otherwise the TX hears bare ACKs and stays in probe.
    loadNextAck();
    loadNextAck();
    loadNextAck();          // 3 slots in the chip's ack-payload FIFO

    lastHopMs = millis();   // FHSS timer starts now (won't actually hop until MAC phase ends)
}

void radioPoll() {
    uint8_t pipe = 0;
    if (radio.available(&pipe)) {
        uint8_t size = radio.getDynamicPayloadSize();
        if (size > 0 && size <= 32) {
            radio.read(rx.lastBytes, size);
            rx.lastPayload = size;
            if (size > rx.maxPayload) {
                rx.maxPayload = size;
                memcpy(rx.maxBytes, rx.lastBytes, size);
            }
        } else {
            radio.flush_rx();
            rx.lastPayload = 0;
        }
        rx.packets   += 1;
        rx.lastMillis = millis();

        // Check if this is a bind packet and act on it BEFORE writing the next ack —
        // tryBind switches pipes and pre-loads acks itself if it fires.
        if (!bindState.bound) {
            tryBind(rx.lastBytes, size);
        } else {
            // Once bound, every packet with non-zero mask is channel data; decode into channelMicros[].
            decodeChannelData(rx.lastBytes, size);
        }

        // Top the ack-payload FIFO back up — same call now picks MAC or telemetry phase.
        loadNextAck();

        // If we just told the TX to hop, hop ourselves now so we're both on the new channel
        // when the next packet flies. v1 does exactly this in UseReceivedData.
        if (hopPending) {
            hopPending = false;
            radio.stopListening();
            delayMicroseconds(100);
            radio.setChannel(FHSS_CHANNELS[nextChannelIdx]);
            radio.startListening();
            delayMicroseconds(100);
            lastHopMs = millis();
        }
    }
}

WebServer server(80);
uint32_t   bootMillis = 0;

void ledOn()  { digitalWrite(LED_PIN, LOW);  }
void ledOff() { digitalWrite(LED_PIN, HIGH); }

String uptimeString() {
    uint32_t s = (millis() - bootMillis) / 1000;
    uint32_t h = s / 3600; s %= 3600;
    uint32_t m = s / 60;   s %= 60;
    char buf[24];
    snprintf(buf, sizeof(buf), "%luh %lum %lus", (unsigned long)h, (unsigned long)m, (unsigned long)s);
    return String(buf);
}

// --- Shared page chrome -------------------------------------------------------------

// Common CSS — warm-flying-field palette. Stored in flash, served from every page.
const char PAGE_CSS[] PROGMEM = R"CSS(
*{box-sizing:border-box}
body{margin:0;min-height:100vh;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
  background:linear-gradient(180deg,#b8d8e8 0%,#f4e4c1 55%,#c8d8a8 100%);
  color:#2c3e50;padding:1.5em 1em 3em}
.container{max-width:500px;margin:0 auto}
h1{text-align:center;color:#2c3e50;font-weight:300;letter-spacing:.08em;font-size:1.9em;
  margin:.3em 0 .15em;text-shadow:0 1px 2px rgba(255,255,255,.5)}
.subtitle{text-align:center;color:#5d7a8c;margin:0 0 1.8em;font-size:.9em}
.btn{display:flex;align-items:center;width:100%;padding:1.1em 1.2em;margin:.7em 0;
  font-size:1.1em;text-decoration:none;color:#fff;border-radius:14px;border:0;
  box-shadow:0 3px 8px rgba(0,0,0,.12);transition:transform .08s,box-shadow .08s;cursor:pointer}
.btn:active{transform:translateY(2px);box-shadow:0 1px 3px rgba(0,0,0,.12)}
.btn .ico{font-size:1.55em;margin-right:.65em;line-height:1}
.btn-bind{background:#d4944a}
.btn-fw  {background:#7d9eb0}
.btn-diag{background:#5fa099}
.btn-bb  {background:#9aaf7d}
.btn-fly {background:#6cab5e;font-weight:600;font-size:1.3em;padding:1.4em 1.2em}
.btn-back{background:#7d9eb0;margin-top:1.5em}
.card{background:rgba(255,255,255,.65);border-radius:14px;padding:1.2em 1.4em;margin:.8em 0;
  box-shadow:0 2px 6px rgba(0,0,0,.06)}
.card h2{margin-top:0;font-weight:500;color:#3a5165}
dl{margin:0}dt{font-weight:600;margin-top:.6em;color:#3a5165}dd{margin-left:0;color:#2c3e50}
code{font-family:ui-monospace,SFMono-Regular,monospace;font-size:.92em;color:#5d3a3a}
.footer{text-align:center;margin-top:2em;color:#5d7a8c;font-size:.85em}
.footer a{color:#5d7a8c;text-decoration:none;margin:0 .5em}
.danger{background:#c97464}
.muted{color:#7a8b95;font-size:.88em}
.big{font-size:2.4em;font-weight:300;line-height:1.1;margin:.15em 0}
.status-card{text-align:center;padding:1.5em;border-radius:18px;margin:1em 0;
  box-shadow:0 4px 14px rgba(0,0,0,.08);transition:background .3s}
.status-card.up  {background:rgba(168,210,154,.85)}
.status-card.down{background:rgba(220,160,140,.85)}
.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:.4em;margin:1em 0}
.ch{background:rgba(255,255,255,.7);border-radius:8px;padding:.45em;text-align:center;font-size:.82em}
.ch .n{color:#7a8b95;font-size:.85em}
.ch .v{font-weight:600;font-size:1.05em}
)CSS";

String pageStart(const char* title) {
    String s;
    s.reserve(2200);
    s += F("<!doctype html><html><head>");
    s += F("<meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>");
    s += F("<title>"); s += title; s += F(" &middot; LDRC RX V2</title><style>");
    s += FPSTR(PAGE_CSS);
    s += F("</style></head><body><div class=container>");
    return s;
}

String pageEnd(bool homeLink = true) {
    String s;
    if (homeLink) {
        s += F("<a class='btn btn-back' href='/'><span class=ico>&#8617;</span>Return to menu</a>");
    }
    s += F("<div class=footer>");
    s += FW_VERSION;
    s += F("</div></div></body></html>");
    return s;
}

// Minimal HTML-attribute escaping. Used when pre-filling form values from NVS so
// passwords containing &, <, >, ", or ' don't break the input element.
String htmlAttr(const String& in) {
    String out;
    out.reserve(in.length() + 8);
    for (size_t i = 0; i < in.length(); ++i) {
        char c = in[i];
        switch (c) {
            case '&':  out += F("&amp;");  break;
            case '<':  out += F("&lt;");   break;
            case '>':  out += F("&gt;");   break;
            case '"':  out += F("&quot;"); break;
            case '\'': out += F("&#39;");  break;
            default:   out += c;           break;
        }
    }
    return out;
}

// --- Friendly front page -----------------------------------------------------------

void handleRoot() {
    String body = pageStart("Home");
    body += F("<h1>LDRC RX V2</h1>");
    body += F("<p class=subtitle id=subtitle>");
    if (netMode == NET_AP) {
        body += F("AP mode &mdash; join '"); body += AP_SSID; body += F("' &amp; set your WiFi");
    } else if (bindState.bound) {
        bool rfAlive = (rx.lastMillis != 0) && ((millis() - rx.lastMillis) < 500);
        body += rfAlive ? F("Bound &amp; receiving &mdash; ready to fly") : F("Bound, waiting for transmitter");
    } else {
        body += F("Unbound &mdash; tap <b>Start Bind</b> below");
    }
    body += F("</p>");

    body += F("<a class='btn btn-fly'  href='/fly'><span class=ico>&#9992;&#65039;</span>Fly now</a>");
    body += F("<a class='btn btn-bind' href='/bind'><span class=ico>&#128279;</span>Start Bind mode</a>");
    body += F("<a class='btn btn-fw'   href='/firmware'><span class=ico>&#128229;</span>Update or roll back firmware</a>");
    body += F("<a class='btn btn-diag' href='/diagnostics'><span class=ico>&#128270;</span>Advanced diagnostics</a>");
    body += F("<a class='btn btn-bb'   href='/blackbox'><span class=ico>&#128209;</span>Black box log</a>");
    if (netMode == NET_AP || netMode == NET_WIFI_UP) {
        body += F("<a class='btn btn-fw'   href='/wifi'><span class=ico>&#128246;</span>WiFi settings</a>");
    }

    body += pageEnd(false);
    server.send(200, "text/html", body);
}

// --- Diagnostics (the old detailed status, untouched) ------------------------------

void handleDiagnostics() {
    String body = pageStart("Diagnostics");
    body += F("<h1>Diagnostics</h1>");
    body += F("<p class=subtitle>Full receiver state &mdash; nothing prettified</p>");

    body += F("<div class=card><h2>Device</h2><dl>");
    body += F("<dt>Firmware</dt><dd>");   body += FW_VERSION;                    body += F("</dd>");
    body += F("<dt>Hostname</dt><dd>");   body += OTA_HOSTNAME;                  body += F(".local</dd>");
    body += F("<dt>IP</dt><dd>");         body += WiFi.localIP().toString();     body += F("</dd>");
    body += F("<dt>MAC</dt><dd>");        body += WiFi.macAddress();             body += F("</dd>");
    body += F("<dt>RSSI</dt><dd>");       body += WiFi.RSSI();                   body += F(" dBm</dd>");
    body += F("<dt>Uptime</dt><dd>");     body += uptimeString();                body += F("</dd>");
    body += F("<dt>Free heap</dt><dd>");  body += ESP.getFreeHeap();             body += F(" B</dd>");
    body += F("<dt>Chip</dt><dd>");       body += ESP.getChipModel();
                                          body += F(" rev ");
                                          body += ESP.getChipRevision();         body += F("</dd>");
    body += F("</dl></div>");

    body += F("<div class=card><h2>nRF24 self-test</h2><dl>");
    body += F("<dt>Verdict</dt><dd><b>");  body += rfTest.verdict;               body += F("</b></dd>");
    body += F("<dt>begin()</dt><dd>");     body += rfTest.beginOk       ? "ok" : "FAIL"; body += F("</dd>");
    body += F("<dt>isChipConnected()</dt><dd>"); body += rfTest.chipConnected ? "ok" : "FAIL"; body += F("</dd>");
    body += F("<dt>Channel r/w</dt><dd>"); body += rfTest.channelOk     ? "ok" : "FAIL";
                                            body += F(" (read back "); body += rfTest.channelRead; body += F(", expected 76)</dd>");
    body += F("<dt>Data rate r/w</dt><dd>"); body += rfTest.dataRateOk  ? "ok" : "FAIL";
                                            body += F(" (read back ");
                                            body += dataRateName(static_cast<rf24_datarate_e>(rfTest.dataRateRead));
                                            body += F(")</dd>");
    body += F("</dl><p><a href='/retest'>&#8635; Re-run self-test</a></p></div>");

    body += F("<div class=card><h2>RF link</h2><dl>");
    body += F("<dt>Packets received</dt><dd><b>"); body += rx.packets; body += F("</b></dd>");
    body += F("<dt>Ack-payloads written</dt><dd>"); body += rx.acksWritten; body += F("</dd>");
    body += F("<dt>MAC acks sent</dt><dd>"); body += macAcksSent; body += F(" / "); body += MAC_ACK_THRESHOLD; body += F("</dd>");
    body += F("<dt>Bind state</dt><dd>");
    if (bindState.bound) {
        char pipeBuf[24];
        snprintf(pipeBuf, sizeof(pipeBuf), "%02X %02X %02X %02X %02X",
                 bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4]);
        body += F("<b>bound</b> &mdash; pipe <code>");
        body += pipeBuf;
        body += F("</code>, ");
        body += (millis() - bindState.boundMillis) / 1000;
        body += F(" s ago");
    } else {
        body += F("unbound (DefaultPipe), ");
        body += bindState.attempts;
        body += F(" attempts seen");
    }
    body += F("</dd>");
    body += F("<dt>FHSS</dt><dd>"); body += (fhssEnabled ? "enabled" : "disabled");
    body += F(" &mdash; idx "); body += nextChannelIdx;
    body += F(" (channel "); body += FHSS_CHANNELS[nextChannelIdx]; body += F(")</dd>");
    body += F("<dt>Telemetry item</dt><dd>"); body += telemetryItem; body += F("</dd>");
    body += F("<dt>SBUS frames out</dt><dd>"); body += sbusFramesOut; body += F("</dd>");
    char vbBuf[40];
    snprintf(vbBuf, sizeof(vbBuf), "%.2f V (raw %u mV)", vbattVolts, (unsigned)vbattLastMv);
    body += F("<dt>RX battery (D0)</dt><dd>"); body += vbBuf; body += F("</dd>");
    body += F("<dt>Last channel data</dt><dd>");
    if (lastChannelDataMs == 0) body += F("never");
    else { body += (millis() - lastChannelDataMs); body += F(" ms ago"); }
    body += F("</dd>");
    char macBuf[24];
    snprintf(macBuf, sizeof(macBuf), "%02X %02X %02X %02X %02X %02X",
             boardMac[0], boardMac[1], boardMac[2], boardMac[3], boardMac[4], boardMac[5]);
    body += F("<dt>Board MAC</dt><dd><code>"); body += macBuf; body += F("</code></dd>");
    body += F("<dt>Last packet</dt><dd>");
    if (rx.lastMillis == 0) body += F("never");
    else {
        body += (millis() - rx.lastMillis); body += F(" ms ago, ");
        body += rx.lastPayload; body += F(" bytes");
    }
    body += F("</dd>");
    if (rx.lastPayload > 0) {
        char hex[120]; char* p = hex;
        for (uint8_t i = 0; i < rx.lastPayload && i < 32; ++i)
            p += snprintf(p, hex + sizeof(hex) - p, "%02X ", rx.lastBytes[i]);
        body += F("<dt>Last payload</dt><dd><code>"); body += hex; body += F("</code></dd>");
    }
    if (rx.maxPayload > 0) {
        char hex[120]; char* p = hex;
        for (uint8_t i = 0; i < rx.maxPayload && i < 32; ++i)
            p += snprintf(p, hex + sizeof(hex) - p, "%02X ", rx.maxBytes[i]);
        body += F("<dt>Largest payload</dt><dd><code>"); body += hex; body += F("</code> ("); body += rx.maxPayload; body += F(" bytes)</dd>");
    }
    body += F("</dl></div>");

    body += F("<div class=card><h2>Channels (&micro;s)</h2><div class=grid4 id=chgrid>");
    for (uint8_t i = 0; i < 16; ++i) {
        body += F("<div class=ch><div class=n>");
        body += (i + 1);
        body += F("</div><div class=v>");
        body += channelMicros[i];
        body += F("</div></div>");
    }
    body += F("</div></div>");

    // Live updater — polls /api/state.json a few times a second and updates the channel cells
    // plus the dynamic stat fields, without re-rendering the whole page.
    body += F("<script>"
        "async function tick(){try{"
        "const r=await fetch('/api/state.json',{cache:'no-store'});"
        "const s=await r.json();"
        "document.querySelectorAll('#chgrid .ch .v').forEach((e,i)=>{if(s.channels[i]!==undefined)e.textContent=s.channels[i];});"
        "const ch=document.getElementById('chstats');"
        "if(ch){ch.textContent=(s.last_pkt_ms>=0?(s.last_pkt_ms+' ms ago'):'never');}"
        "}catch(e){}setTimeout(tick,250);}tick();"
        "</script>");

    body += pageEnd();
    server.send(200, "text/html", body);
}

void handleRetest() {
    runRadioSelfTest();
    server.sendHeader("Location", "/diagnostics", true);
    server.send(303, "text/plain", "retested");
}

// --- Bind page -------------------------------------------------------------------

void handleBind() {
    String body = pageStart("Bind");
    body += F("<h1>Start Bind mode</h1>");

    body += F("<div class=card>");
    body += F("<p>This clears the saved pipe and reboots the receiver so it listens on the v1 <code>DefaultPipe</code> again.</p>");
    body += F("<p>Then put your transmitter into bind mode and wait a few seconds &mdash; the receiver will pick up the new pipe address and save it automatically.</p>");
    if (bindState.bound) {
        char pipeBuf[24];
        snprintf(pipeBuf, sizeof(pipeBuf), "%02X %02X %02X %02X %02X",
                 bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4]);
        body += F("<p class=muted>Current saved pipe: <code>");
        body += pipeBuf;
        body += F("</code></p>");
    } else {
        body += F("<p class=muted>Currently unbound &mdash; just power-cycle the TX in bind mode.</p>");
    }
    body += F("<form method='POST' action='/bind'>");
    body += F("<button class='btn danger' type='submit'><span class=ico>&#9888;&#65039;</span>Clear bind &amp; restart</button>");
    body += F("</form>");
    body += F("</div>");

    body += pageEnd();
    server.send(200, "text/html", body);
}

void handleBindDo() {
    clearBindNvs();
    String body = pageStart("Rebooting");
    body += F("<h1>Rebooting&hellip;</h1>");
    body += F("<div class=card><p>Bind cleared. The receiver is rebooting and will listen on DefaultPipe within ~5 seconds.</p>");
    body += F("<p><a href='/'>Back to home</a> (reload after the chip is back)</p></div>");
    body += pageEnd(false);
    server.send(200, "text/html", body);
    delay(400);
    ESP.restart();
}

// --- Firmware page (update + rollback) ------------------------------------------

void handleFirmware() {
    String body = pageStart("Firmware");
    body += F("<h1>Firmware</h1>");

    body += F("<div class=card><h2>Current</h2><dl>");
    body += F("<dt>Running</dt><dd>"); body += FW_VERSION; body += F("</dd>");
    body += F("<dt>Build date</dt><dd>"); body += __DATE__; body += F(" "); body += __TIME__; body += F("</dd>");

    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* other   = esp_ota_get_next_update_partition(NULL);
    if (running) { body += F("<dt>Running partition</dt><dd><code>"); body += running->label; body += F("</code></dd>"); }
    if (other)   { body += F("<dt>Other slot</dt><dd><code>"); body += other->label;
                   body += F("</code> (")  ; body += (other->size / 1024); body += F(" KB)</dd>"); }
    body += F("</dl></div>");

    body += F("<div class=card><h2>Update over the air</h2>");
    if (netMode == NET_AP) {
        body += F("<p><b>Not available in AP mode.</b> The Mac running PlatformIO can't reach <code>rxv2.local</code> while we're our own access point.</p>");
        body += F("<p>To update, either:</p><ul>");
        body += F("<li>Save your home/field WiFi on the <a href='/wifi'>WiFi page</a> and reboot &mdash; the receiver will join it and OTA becomes possible again, or</li>");
        body += F("<li>Connect the Mac to the <code>"); body += AP_SSID; body += F("</code> AP and push using IP <code>"); body += WiFi.softAPIP().toString(); body += F("</code>.</li></ul>");
    } else {
        body += F("<p>From the development Mac, with this receiver on WiFi:</p>");
        body += F("<p><code>cd ReceiverCode/RXV2<br>pio run -e xiao_ota -t upload</code></p>");
        body += F("<p class=muted>Push uses ArduinoOTA on port 3232 of <code>rxv2.local</code>. ~90 s fresh build, ~10 s cached.</p>");
    }
    body += F("</div>");

    body += F("<div class=card><h2>Roll back</h2>");
    body += F("<p>Swap to the other OTA slot and reboot. If that slot holds an older valid firmware, you're back on it; if it's empty or invalid, the bootloader returns to the current firmware automatically (rollback protection).</p>");
    body += F("<p class=muted>The ESP32-C3's default partition table only has two app slots (current + 1 backup). A deeper history (current + 2 backups, as discussed) would need a custom partition table and ~1.2 MB of LittleFS to stash an older binary &mdash; a future enhancement.</p>");
    body += F("<form method='POST' action='/rollback'>");
    body += F("<button class='btn danger' type='submit'><span class=ico>&#8634;</span>Roll back to previous slot</button>");
    body += F("</form></div>");

    body += F("<div class=card><h2>Escape hatch &mdash; force WiFi at boot</h2>");
    body += F("<p>The receiver normally stays WiFi-off when the TX is on (per the boot rule). If you're locked out of OTA because the TX is on, recover with this sequence:</p>");
    body += F("<ol><li>Power-cycle the receiver three times within about five seconds (e.g., quick unplug / replug / unplug / replug / unplug / replug)</li>");
    body += F("<li>On the third boot the chip skips the RF window and joins WiFi unconditionally</li>");
    body += F("<li>Push your update, then power-cycle once more to return to normal boot behaviour</li></ol>");
    body += F("<p class=muted>The counter resets automatically after 5 s of stable running, so a single power-cycle doesn't accumulate.</p>");
    body += F("</div>");

    body += pageEnd();
    server.send(200, "text/html", body);
}

void handleRollback() {
    const esp_partition_t* other = esp_ota_get_next_update_partition(NULL);
    if (!other) {
        server.send(500, "text/plain", "no other partition");
        return;
    }
    esp_err_t err = esp_ota_set_boot_partition(other);
    if (err != ESP_OK) {
        String msg = "rollback failed: ";
        msg += esp_err_to_name(err);
        server.send(500, "text/plain", msg);
        return;
    }
    String body = pageStart("Rolling back");
    body += F("<h1>Rolling back&hellip;</h1>");
    body += F("<div class=card><p>Switching boot to slot <code>");
    body += other->label;
    body += F("</code> and restarting. If that slot is invalid, the bootloader will bring you back to the current firmware automatically.</p>");
    body += F("<p><a href='/'>Back to home</a> (reload after the chip is back)</p></div>");
    body += pageEnd(false);
    server.send(200, "text/html", body);
    delay(400);
    ESP.restart();
}

// --- Fly now! page --------------------------------------------------------------

void handleFly() {
    String body = pageStart("Fly");
    body += F("<h1>Fly now</h1>");

    bool rfAlive = (rx.lastMillis != 0) && ((millis() - rx.lastMillis) < 500);
    body += F("<div id=statuscard class='status-card "); body += (rfAlive ? "up" : "down"); body += F("'>");
    body += F("<div class=label>RF link</div>");
    body += F("<div class=big id=rfstatus>");
    body += rfAlive ? F("Connected") : (bindState.bound ? F("Waiting") : F("Unbound"));
    body += F("</div>");
    body += F("<div class=muted id=rfdetail>");
    if (rfAlive)            { body += (millis() - rx.lastMillis); body += F(" ms ago"); }
    else if (bindState.bound) body += F("bound, no packets for now");
    else                      body += F("waiting for TX bind");
    body += F("</div></div>");

    body += F("<div class=card><div class=label>RX battery</div>");
    char vb[20];
    snprintf(vb, sizeof(vb), "%.2f V", vbattVolts);
    body += F("<div class=big id=vbatt>"); body += vb; body += F("</div></div>");

    body += F("<div class=card><h2>Stick positions</h2><div class=grid4 id=chgrid>");
    for (uint8_t i = 0; i < 16; ++i) {
        body += F("<div class=ch><div class=n>ch ");
        body += (i + 1);
        body += F("</div><div class=v>");
        body += channelMicros[i];
        body += F("</div></div>");
    }
    body += F("</div></div>");

    body += F("<form method='POST' action='/fly_arm' onsubmit=\"return confirm('Disable WiFi until next reboot?')\">");
    body += F("<button class='btn btn-fly' type='submit' style='background:#c97464'><span class=ico>&#128737;&#65039;</span>Disable WiFi &amp; fly</button>");
    body += F("</form>");
    body += F("<p class='muted' style='text-align:center'>SBUS keeps running with or without WiFi.</p>");

    // Live update script — keeps the page fresh without full reloads. Stops naturally
    // once WiFi is disabled (fetches just start failing).
    body += F("<script>"
        "async function tick(){try{"
        "const r=await fetch('/api/state.json',{cache:'no-store'});"
        "const s=await r.json();"
        "document.querySelectorAll('#chgrid .ch .v').forEach((e,i)=>{if(s.channels[i]!==undefined)e.textContent=s.channels[i];});"
        "const v=document.getElementById('vbatt');if(v)v.textContent=s.v.toFixed(2)+' V';"
        "const st=document.getElementById('rfstatus');"
        "const card=document.getElementById('statuscard');"
        "const det=document.getElementById('rfdetail');"
        "const alive=s.last_pkt_ms>=0&&s.last_pkt_ms<500;"
        "if(st){st.textContent=alive?'Connected':(s.bound?'Waiting':'Unbound');}"
        "if(card){card.classList.toggle('up',alive);card.classList.toggle('down',!alive);}"
        "if(det){det.textContent=alive?(s.last_pkt_ms+' ms ago'):(s.bound?'bound, no packets':'waiting for TX bind');}"
        "}catch(e){}setTimeout(tick,250);}tick();"
        "</script>");

    body += pageEnd();
    server.send(200, "text/html", body);
}

// --- Black box (event log) -----------------------------------------------------

void handleBlackbox() {
    String body = pageStart("Black box");
    body += F("<h1>Black box log</h1>");
    body += F("<div class=card>");
    if (events.count == 0) {
        body += F("<p class=muted>No events recorded yet.</p>");
    } else {
        body += F("<p class=muted>Most recent first. ");
        body += events.count; body += F(" event"); if (events.count != 1) body += F("s");
        body += F(".</p>");
        body += F("<dl>");
        // walk newest-to-oldest
        size_t n = events.count;
        size_t idx = (events.head + EventLog::SIZE - 1) % EventLog::SIZE;
        for (size_t k = 0; k < n; ++k) {
            char tsBuf[24];
            uint32_t s = events.when[idx] / 1000;
            uint32_t h = s / 3600; s %= 3600;
            uint32_t m = s / 60;   s %= 60;
            snprintf(tsBuf, sizeof(tsBuf), "%02u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);
            body += F("<dt>"); body += tsBuf; body += F("</dt><dd>"); body += events.msgs[idx]; body += F("</dd>");
            idx = (idx + EventLog::SIZE - 1) % EventLog::SIZE;
        }
        body += F("</dl>");
    }
    body += F("</div>");
    body += pageEnd();
    server.send(200, "text/html", body);
}

// --- JSON state endpoint used by /diagnostics and /fly for live updates --------

void handleApiState() {
    String j;
    j.reserve(700);
    char buf[64];
    j += '{';
    snprintf(buf, sizeof(buf), "\"v\":%.2f,\"raw_mv\":%u,", vbattVolts, (unsigned)vbattLastMv);
    j += buf;
    j += "\"packets\":";   j += rx.packets;
    j += ",\"last_pkt_ms\":";
    if (rx.lastMillis) j += (uint32_t)(millis() - rx.lastMillis);
    else               j += "-1";
    j += ",\"sbus_out\":"; j += sbusFramesOut;
    j += ",\"bound\":";    j += (bindState.bound ? "true" : "false");
    j += ",\"net_mode\":\""; j += netModeName(); j += "\"";
    j += ",\"rssi\":";
    if (netMode == NET_WIFI_UP) j += WiFi.RSSI();
    else                        j += "0";
    j += ",\"uptime_s\":"; j += (uint32_t)((millis() - bootMillis) / 1000);
    j += ",\"channels\":[";
    for (int i = 0; i < 16; ++i) {
        if (i) j += ',';
        j += channelMicros[i];
    }
    j += "]}";
    server.send(200, "application/json", j);
}

// --- WiFi credentials page ----------------------------------------------------

void handleWifi() {
    String body = pageStart("WiFi");
    body += F("<h1>WiFi settings</h1>");

    String ssid     = getEffectiveSsid();
    bool   custom   = wifiCredsAreCustom();

    body += F("<div class=card><h2>Status</h2><dl>");
    body += F("<dt>Mode</dt><dd>"); body += netModeName(); body += F("</dd>");
    body += F("<dt>Hostname</dt><dd>"); body += OTA_HOSTNAME; body += F(".local</dd>");
    body += F("<dt>SSID</dt><dd>");
    if (ssid.length() == 0) body += F("<i>(none saved)</i>");
    else                    { body += ssid; body += F(" "); body += (custom ? F("(saved in NVS)") : F("(empty default)")); }
    body += F("</dd>");
    if (netMode == NET_WIFI_UP) {
        body += F("<dt>IP</dt><dd>"); body += WiFi.localIP().toString(); body += F("</dd>");
        body += F("<dt>RSSI</dt><dd>"); body += WiFi.RSSI(); body += F(" dBm</dd>");
    } else if (netMode == NET_AP) {
        body += F("<dt>AP IP</dt><dd>"); body += WiFi.softAPIP().toString(); body += F(" (connect to '"); body += AP_SSID; body += F("')</dd>");
        body += F("<dt class=muted>Note</dt><dd class=muted>WiFi updates only work in STA mode &mdash; save credentials below so the next boot can join your network.</dd>");
    }
    body += F("</dl></div>");

    String savedPass = prefs.isKey(NVS_KEY_PASS) ? prefs.getString(NVS_KEY_PASS, "") : String("");

    body += F("<div class=card><h2>Set credentials</h2>");
    body += F("<p>Saved to NVS and used on next boot. Edit either field and tap save; the current values are pre-filled.</p>");
    body += F("<form method='POST' action='/wifi'>");
    body += F("<p><label>SSID<br><input name='ssid' type='text' required value='");
    body += htmlAttr(ssid);
    body += F("' style='width:100%;padding:.6em;font-size:1em;border-radius:8px;border:1px solid #b8c7d0'></label></p>");
    body += F("<p><label>Password<br><input name='pass' type='password' id='pwfield' value='");
    body += htmlAttr(savedPass);
    body += F("' style='width:100%;padding:.6em;font-size:1em;border-radius:8px;border:1px solid #b8c7d0'></label></p>");
    body += F("<p style='margin-top:-.3em'><label style='font-size:.92em;color:#3a5165'>");
    body += F("<input type='checkbox' onchange=\"document.getElementById('pwfield').type=this.checked?'text':'password'\" style='margin-right:.4em'>");
    body += F("Show password</label></p>");
    body += F("<button class='btn btn-fw' type='submit'><span class=ico>&#128190;</span>Save &amp; reboot</button>");
    body += F("</form></div>");

    if (custom) {
        body += F("<div class=card><h2>Forget saved credentials</h2>");
        body += F("<p class=muted>Clears the SSID/password from NVS. Next boot will use the compiled-in defaults again.</p>");
        body += F("<form method='POST' action='/wifi_reset'>");
        body += F("<button class='btn danger' type='submit'><span class=ico>&#9851;</span>Forget &amp; reboot</button>");
        body += F("</form></div>");
    }

    body += pageEnd();
    server.send(200, "text/html", body);
}

void handleWifiSet() {
    if (server.hasArg("ssid")) {
        prefs.putString(NVS_KEY_SSID, server.arg("ssid"));
    }
    if (server.hasArg("pass") && server.arg("pass").length() > 0) {
        prefs.putString(NVS_KEY_PASS, server.arg("pass"));
    }
    events.add("WiFi credentials saved");

    String body = pageStart("Saved");
    body += F("<h1>Saved &amp; rebooting</h1>");
    body += F("<div class=card><p>WiFi credentials saved. Receiver is rebooting and will attempt to join '");
    body += server.hasArg("ssid") ? server.arg("ssid") : getEffectiveSsid();
    body += F("' on next boot.</p></div>");
    body += pageEnd(false);
    server.send(200, "text/html", body);
    delay(500);
    ESP.restart();
}

void handleWifiReset() {
    prefs.remove(NVS_KEY_SSID);
    prefs.remove(NVS_KEY_PASS);
    events.add("WiFi NVS credentials wiped");

    String body = pageStart("Reset");
    body += F("<h1>WiFi credentials cleared</h1>");
    body += F("<div class=card><p>NVS wiped. Receiver is rebooting and will use the compiled-in defaults.</p></div>");
    body += pageEnd(false);
    server.send(200, "text/html", body);
    delay(500);
    ESP.restart();
}

// --- Fly arm (manual WiFi off) ----------------------------------------------

void handleFlyArm() {
    String body = pageStart("Flying");
    body += F("<h1>WiFi off &mdash; have a great flight!</h1>");
    body += F("<div class='card status-card up'>");
    body += F("<div class=big>&#9992;&#65039;</div>");
    body += F("<p>The receiver is now in <b>RF-only</b> mode. WiFi will return on the next power-cycle.</p>");
    body += F("<p class=muted>SBUS keeps streaming. This page will stop responding the moment WiFi shuts down (any second now).</p>");
    body += F("</div>");
    body += pageEnd(false);
    server.send(200, "text/html", body);
    flyArmRequested = true;       // disableWifi() runs from loop() after this response flushes
}

// --- Legacy small endpoints ----------------------------------------------------

void handleReboot() {
    String body = pageStart("Rebooting");
    body += F("<h1>Rebooting&hellip;</h1>");
    body += F("<div class=card><p>Back in ~5 s.</p></div>");
    body += pageEnd(false);
    server.send(200, "text/html", body);
    delay(400);
    ESP.restart();
}

void setupOTA() {
    if (otaStarted) return;
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (OTA_PASSWORD) ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA
        .onStart([]() {
            Serial.printf("[ota] start (%s)\n",
                          ArduinoOTA.getCommand() == U_FLASH ? "flash" : "fs");
            ledOn();
        })
        .onEnd([]() {
            Serial.println("\n[ota] complete");
            ledOff();
        })
        .onProgress([](unsigned int p, unsigned int t) {
            Serial.printf("[ota] %u%%\r", (p * 100) / t);
        })
        .onError([](ota_error_t e) {
            Serial.printf("[ota] error %u\n", e);
        });
    ArduinoOTA.begin();
    otaStarted = true;
    Serial.printf("[ota] ready as %s.local:3232\n", OTA_HOSTNAME);
}

void startMdnsAndOta() {
    if (!MDNS.begin(OTA_HOSTNAME)) {
        Serial.println("[mdns] failed");
    } else {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("[mdns] http://%s.local/\n", OTA_HOSTNAME);
    }
    setupOTA();
}

void startWifiStation() {
    String ssid = getEffectiveSsid();
    if (ssid.length() == 0) {
        // No SSID saved → there's nothing useful to try. Go straight to AP so the
        // user can enter credentials via the web UI on first boot.
        Serial.println("[wifi] no NVS SSID — going straight to AP mode");
        events.add("No SSID configured — AP mode");
        startApMode();
        return;
    }
    Serial.printf("[wifi] STA connecting to '%s'\n", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(ssid.c_str(), getEffectivePass().c_str());
    netMode       = NET_WIFI_CONNECTING;
    netStateStart = millis();

    char buf[80];
    snprintf(buf, sizeof(buf), "Trying STA WiFi '%s'", ssid.c_str());
    events.add(buf);
}

// Set true once server.begin() has been called — protects against double-init when
// transitioning between net modes (e.g. STA dropped then reconnected, or AP after STA).
bool httpServerStarted = false;

void startHttpServerIfNeeded() {
    if (httpServerStarted) return;
    server.begin();
    httpServerStarted = true;
    Serial.println("[http] listening on :80");
}

void onWifiConnected() {
    Serial.printf("[wifi] %s  RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    char buf[80];
    snprintf(buf, sizeof(buf), "WiFi up: %s", WiFi.localIP().toString().c_str());
    events.add(buf);
    startMdnsAndOta();
    startHttpServerIfNeeded();
    netMode = NET_WIFI_UP;
    ledOff();
}

void startApMode() {
    Serial.printf("[wifi] STA failed, starting AP '%s'\n", AP_SSID);
    WiFi.disconnect(true);
    delay(50);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID);
    delay(150);
    Serial.printf("[wifi] AP IP %s\n", WiFi.softAPIP().toString().c_str());
    char buf[80];
    snprintf(buf, sizeof(buf), "AP mode: %s at %s",
             AP_SSID, WiFi.softAPIP().toString().c_str());
    events.add(buf);
    if (MDNS.begin(OTA_HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
    }
    startHttpServerIfNeeded();
    netMode = NET_AP;
}

void disableWifi() {
    Serial.println("[wifi] turning off until reboot");
    if (otaStarted) {
        ArduinoOTA.end();
        otaStarted = false;
    }
    MDNS.end();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    netMode = NET_NO_WIFI;
    events.add("WiFi off (Fly mode)");
}

// Set by /fly_arm to defer the WiFi shutdown until after the response has been flushed.
volatile bool flyArmRequested = false;

void heartbeat() {
    // 50 ms blip every 2 s once we're online, so the LED proves we're alive.
    static uint32_t next = 0;
    static bool     on   = false;
    uint32_t now = millis();
    if (now >= next) {
        on = !on;
        if (on) { ledOn();  next = now + 50;   }
        else    { ledOff(); next = now + 1950; }
    }
}

} // namespace

void setup() {
    pinMode(LED_PIN, OUTPUT);
    ledOff();

    Serial.begin(115200);
    // Make USB-CDC writes non-blocking when no host is attached — otherwise
    // every printf can stall up to 20 s waiting for a reader, which delays
    // the entire boot sequence (radio self-test, WiFi, AP fallback all push back).
    Serial.setTxTimeoutMs(0);
    delay(200);
    Serial.printf("\n\n=== %s ===\n", FW_VERSION);
    bootMillis = millis();

    buildDays = computeBuildDays();
    Serial.printf("[id] build %s -> %u days since 2020-01-01\n", __DATE__, (unsigned)buildDays);

    // Centre all channels at 1500us so the SBUS frame is sane immediately at boot.
    for (uint8_t i = 0; i < 16; ++i) channelMicros[i] = 1500;

    // SBUS UART: 100 kbaud, 8E2, inverted. Frames start streaming failsafe values immediately.
    Serial1.begin(100000, SERIAL_8E2, -1, PIN_SBUS_TX, true);
    Serial.printf("[sbus] inverted UART on GPIO %d at 100 kbaud 8E2\n", PIN_SBUS_TX);

    events.add("Boot");

    prefs.begin(NVS_NAMESPACE, false);

    // Quick-boot escape hatch — count rapid reboots in NVS. Three within 5 s = force WiFi.
    {
        uint8_t cnt = prefs.isKey(NVS_KEY_BOOT_COUNT) ? prefs.getUChar(NVS_KEY_BOOT_COUNT, 0) : 0;
        cnt++;
        prefs.putUChar(NVS_KEY_BOOT_COUNT, cnt);
        if (cnt >= QUICK_BOOT_THRESHOLD) {
            forceWifiMode = true;
            prefs.putUChar(NVS_KEY_BOOT_COUNT, 0);
            Serial.printf("[boot] %u quick boots — forcing WiFi mode\n", cnt);
            events.add("Force-WiFi (triple power-cycle)");
        } else {
            Serial.printf("[boot] quick-boot counter: %u/%u\n", cnt, QUICK_BOOT_THRESHOLD);
        }
    }

    loadBindFromNvs();
    if (bindState.bound) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Restored bind %02X %02X %02X %02X %02X from NVS",
                 bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4]);
        events.add(buf);
    }

    // Use the underlying esp_read_mac so we can capture the board ID even before WiFi.begin().
    esp_read_mac(boardMac, ESP_MAC_WIFI_STA);
    Serial.printf("[id] board MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                  boardMac[0], boardMac[1], boardMac[2],
                  boardMac[3], boardMac[4], boardMac[5]);

    runRadioSelfTest();
    radioBeginListenV1();

    // Register HTTP routes now so the server is ready the instant any net mode brings up an interface.
    server.on("/", handleRoot);
    server.on("/fly", handleFly);
    server.on("/fly_arm", HTTP_POST, handleFlyArm);
    server.on("/diagnostics", handleDiagnostics);
    server.on("/bind", HTTP_GET,  handleBind);
    server.on("/bind", HTTP_POST, handleBindDo);
    server.on("/firmware", handleFirmware);
    server.on("/rollback", HTTP_POST, handleRollback);
    server.on("/blackbox", handleBlackbox);
    server.on("/wifi", HTTP_GET,  handleWifi);
    server.on("/wifi", HTTP_POST, handleWifiSet);
    server.on("/wifi_reset", HTTP_POST, handleWifiReset);
    server.on("/api/state.json", handleApiState);
    server.on("/retest", handleRetest);
    server.on("/reboot", handleReboot);
    server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
    // NB: server.begin() is deferred until WiFi STA or AP is up — calling it here
    // would touch LwIP before the WiFi stack has initialised it, panicking the chip
    // ("assert failed: tcpip_send_msg_wait_sem ... Invalid mbox").

    if (forceWifiMode) {
        // Escape hatch: skip RF window, go straight to WiFi STA so OTA is reachable.
        Serial.println("[net] force-WiFi requested, skipping RF window");
        startWifiStation();
    } else {
        // Normal: 10 s RF discovery window. The state machine in loop() decides what's next.
        netMode       = NET_WAITING_RF;
        netStateStart = millis();
        Serial.printf("[net] waiting up to %u ms for TX before bringing up WiFi\n",
                      (unsigned)RF_WINDOW_MS);
        events.add("Boot window: listening for TX...");
    }
}

void netStep() {
    switch (netMode) {
        case NET_WAITING_RF:
            // If a real packet has arrived, the TX is on — stay RF-only this session.
            if (rx.packets > 0) {
                Serial.println("[net] TX heard during boot window — staying RF-only");
                events.add("TX seen in 10s window — WiFi stays OFF");
                netMode = NET_NO_WIFI;
                return;
            }
            if ((uint32_t)(millis() - netStateStart) >= RF_WINDOW_MS) {
                Serial.println("[net] boot window timed out, trying WiFi");
                events.add("No TX in 10s — trying WiFi");
                startWifiStation();
            }
            break;

        case NET_WIFI_CONNECTING:
            if (WiFi.status() == WL_CONNECTED) {
                onWifiConnected();
            } else if ((uint32_t)(millis() - netStateStart) >= WIFI_CONNECT_MS) {
                Serial.println("[net] STA timed out, falling back to AP mode");
                events.add("STA failed — falling back to AP");
                startApMode();
            } else {
                // Blink fast while STA connecting
                static uint32_t lastBlink = 0;
                if (millis() - lastBlink > 150) {
                    lastBlink = millis();
                    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                }
            }
            break;

        case NET_WIFI_UP:
            // If WiFi drops, try to reconnect quietly (don't restart the chip).
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("[wifi] dropped, retrying STA");
                events.add("WiFi dropped, retrying");
                startWifiStation();
            }
            break;

        case NET_NO_WIFI:
        case NET_AP:
        case NET_INIT:
            // nothing to do
            break;
    }
}

void loop() {
    if (otaStarted) ArduinoOTA.handle();
    if (netMode == NET_WIFI_UP || netMode == NET_AP) {
        server.handleClient();
    }

    radioPoll();
    sbusTick();
    readVbatt();
    heartbeat();
    netStep();

    // After 5 s of stable running, clear the quick-boot counter so an isolated power-cycle
    // doesn't accumulate toward the 3-trip threshold.
    static bool quickBootReset = false;
    if (!quickBootReset && millis() > QUICK_BOOT_RESET_MS) {
        prefs.putUChar(NVS_KEY_BOOT_COUNT, 0);
        quickBootReset = true;
    }

    // Deferred WiFi-off so the response to /fly_arm completes before the radio dies.
    if (flyArmRequested) {
        flyArmRequested = false;
        delay(300);
        disableWifi();
    }
}
