// LockDownRadioControl — Receiver V2  ::  main.cpp
//
// This is the single compilation unit for the RXV2 firmware. It includes the
// per-subsystem headers (1Defs.h must always come first so globals are
// declared), then defines setup() / loop().
//
// Module layout (mirrors v1's single-cpp + many-h pattern):
//   1Defs.h        — pin map, version, enums, structs, all globals
//   Storage.h      — NVS bind + WiFi-credential helpers
//   Channels.h     — 16-channel decompression + decode
//   Output.h       — SBUS/CRSF/IBUS/PPM/FBUS frame builders + drivers
//   Telemetry.h    — CRSF/FBUS/IBUS2 inbound parsers
//   Radio.h        — nRF24L01+ self-test, dual-radio, bind, ack rotation
//   Network.h      — STA/AP state machine, OTA, mDNS, heartbeat
//   WebPages.h     — HTTP handlers + page chrome
//
// Web pages serve their shared stylesheet from LittleFS at /style.css; an
// embedded fallback exists in WebPages.h in case the filesystem doesn't mount.
// Edit data/style.css and push with `pio run -e xiao_ota -t uploadfs`.
//
//*********************************************************************

#include "1Defs.h"
#include "Storage.h"
#include "Channels.h"
#include "Output.h"
#include "Telemetry.h"
#include "Radio.h"
#include "Network.h"
#include "MspBridge.h"
#include "MspFc.h"
#include "WebPages.h"

//*********************************************************************
//  setup() — one-time boot sequence
//*********************************************************************

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

    // Centre all channels at 1500us so the output frame is sane immediately at boot.
    for (uint8_t i = 0; i < 16; ++i) channelMicros[i] = 1500;

    events.add("Boot");

    prefs.begin(NVS_NAMESPACE, false);

    //*****************************************************************
    // Mount LittleFS so web pages can serve assets from /data
    //*****************************************************************
    if (LittleFS.begin(true)) {     // formatOnFail=true — wipes & formats if corrupt
        littleFsMounted = true;
        Serial.printf("[fs] LittleFS mounted, %u/%u bytes used\n",
                      (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
    } else {
        littleFsMounted = false;
        Serial.println("[fs] LittleFS mount failed — using embedded fallback assets");
        events.add("LittleFS mount failed");
    }

    //*****************************************************************
    // Quick-boot escape hatch — three quick reboots forces WiFi + SBUS
    //*****************************************************************
    // Recovers cleanly from a protocol selection that's crashing the chip in
    // a reboot loop.
    {
        uint8_t cnt = prefs.isKey(NVS_KEY_BOOT_COUNT) ? prefs.getUChar(NVS_KEY_BOOT_COUNT, 0) : 0;
        cnt++;
        prefs.putUChar(NVS_KEY_BOOT_COUNT, cnt);
        if (cnt >= QUICK_BOOT_THRESHOLD) {
            forceWifiMode = true;
            prefs.putUChar(NVS_KEY_BOOT_COUNT, 0);
            prefs.putUChar(NVS_KEY_PROTO, (uint8_t)PROTO_SBUS);   // safety reset
            Serial.printf("[boot] %u quick boots — forcing WiFi + SBUS protocol\n", cnt);
            events.add("Recovery: forced WiFi + SBUS protocol");
        } else {
            Serial.printf("[boot] quick-boot counter: %u/%u\n", cnt, QUICK_BOOT_THRESHOLD);
        }
    }

    //*****************************************************************
    // Load saved output protocol (defaults to SBUS)
    //*****************************************************************
    {
        uint8_t p = prefs.isKey(NVS_KEY_PROTO) ? prefs.getUChar(NVS_KEY_PROTO, PROTO_DEFAULT) : PROTO_DEFAULT;
        if (p > PROTO_MAX) p = PROTO_DEFAULT;
        currentProtocol = (Protocol)p;
        ppmInverted = prefs.isKey(NVS_KEY_PPM_INV) ? (prefs.getUChar(NVS_KEY_PPM_INV, 0) != 0) : false;
        Serial.printf("[out] protocol = %s — %s  (PPM polarity: %s)\n",
                      protocolName(currentProtocol), protocolDesc(currentProtocol),
                      ppmInverted ? "negative (idle HIGH)" : "positive (idle LOW)");
    }
    configureOutputDriver(currentProtocol);

    //*****************************************************************
    // Bind state — restore from NVS if previously bound
    //*****************************************************************
    loadBindFromNvs();
    if (bindState.bound) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Restored bind %02X %02X %02X %02X %02X from NVS",
                 bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4]);
        events.add(buf);
    }

    //*****************************************************************
    // Board ID — captured once on first boot, persisted forever
    //*****************************************************************
    // Guarantees the MAC bytes we report to the TX during bind never change
    // between reboots, so the transmitter's auto-model-select can be left ON.
    if (prefs.isKey(NVS_KEY_BOARD_ID) && prefs.getBytesLength(NVS_KEY_BOARD_ID) == 6) {
        prefs.getBytes(NVS_KEY_BOARD_ID, boardMac, 6);
        Serial.println("[id] board MAC loaded from NVS (stable across reboots)");
    } else {
        esp_read_mac(boardMac, ESP_MAC_WIFI_STA);
        prefs.putBytes(NVS_KEY_BOARD_ID, boardMac, 6);
        Serial.println("[id] board MAC captured from chip and saved to NVS");
    }
    Serial.printf("[id] board MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                  boardMac[0], boardMac[1], boardMac[2],
                  boardMac[3], boardMac[4], boardMac[5]);

    //*****************************************************************
    // Radio bring-up
    //*****************************************************************
    runRadioSelfTest();
    detectAllRadios();       // probes slots 1/2/3 independently; sets radioPresent[]
    radioBeginListenV1();

    //*****************************************************************
    // Register HTTP routes (server.begin() is deferred to onWifiConnected()
    // or startApMode() — calling it before WiFi is up panics LwIP).
    //*****************************************************************
    registerWebRoutes();

    //*****************************************************************
    // Net state machine — RF discovery window unless forced to WiFi
    //*****************************************************************
    if (forceWifiMode) {
        Serial.println("[net] force-WiFi requested, skipping RF window");
        startWifiStation();
    } else if (DEV_KEEP_WIFI) {
        // Dev convenience: skip the RF-detect window so we can re-flash even
        // when the TX is on. Flip DEV_KEEP_WIFI to false in 1Defs.h before
        // shipping to restore the production "TX-on means RF-only" behaviour.
        Serial.println("[net] DEV_KEEP_WIFI=true — skipping RF window, going to WiFi");
        events.add("DEV mode: WiFi forced on at boot");
        startWifiStation();
    } else {
        netMode       = NET_WAITING_RF;
        netStateStart = millis();
        Serial.printf("[net] waiting up to %u ms for TX before bringing up WiFi\n",
                      (unsigned)RF_WINDOW_MS);
        events.add("Boot window: listening for TX...");
    }
}

//*********************************************************************
//  loop() — main service loop
//*********************************************************************

void loop() {
    if (otaStarted) ArduinoOTA.handle();
    if (netMode == NET_WIFI_UP || netMode == NET_AP) {
        server.handleClient();
    }

    radioPoll();
    protocolRx();          // pull any telemetry/MSP bytes the FC has sent back on D5
    mspBridgePoll();       // TCP/5760 ↔ FC for wireless Rotorflight config
    mspFcPoll();           // periodic FC-variant / FC-version discovery

    // Dual-radio redundancy: if we've not received a packet on the active
    // radio for a while AND a swap cooldown has elapsed AND we have a second
    // radio populated, try the other radio (v1 TryTheOtherTransceiver).
    if (numRadiosPresent >= 2 && bindState.bound && rx.lastMillis != 0 &&
        (uint32_t)(millis() - rx.lastMillis)     >= RADIO_SWAP_PACKET_TIMEOUT_MS &&
        (uint32_t)(millis() - lastRadioSwapMs)   >= RADIO_SWAP_COOLDOWN_MS) {
        swapRadios();
    }
    sbusTick();
    heartbeat();
    netStep();

    // Periodic free-heap snapshot to the event log so we can spot leaks
    // (every 60 s; logs only when value drops to track downward trend).
    {
        static uint32_t lastHeapLogMs = 0;
        static uint32_t lastHeap      = 0xFFFFFFFF;
        if ((uint32_t)(millis() - lastHeapLogMs) >= 60000) {
            lastHeapLogMs = millis();
            uint32_t now = ESP.getFreeHeap();
            // Log on first sample, or if heap has dropped by >2 KB since the last log.
            if (lastHeap == 0xFFFFFFFF || (lastHeap > now && (lastHeap - now) > 2048)) {
                char buf[64];
                snprintf(buf, sizeof(buf), "Free heap: %u B (was %u)",
                         (unsigned)now, (unsigned)lastHeap);
                events.add(buf);
                lastHeap = now;
            } else if (now > lastHeap) {
                lastHeap = now;   // recovered (e.g. closed connections freed) — update baseline silently
            }
        }
    }

    // After 5 s of stable running, clear the quick-boot counter so an isolated
    // power-cycle doesn't accumulate toward the 3-trip threshold.
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
