// LockDownRadioControl — RXV2  ::  Wifi.h
//
// WiFi state machine + ArduinoOTA + mDNS. The boot flow is:
//   1. Listen for the TX for ~10 s on a fixed channel.
//   2. If a packet arrives in that window, skip WiFi (RF-only flight mode).
//   3. Otherwise try saved STA credentials; on failure fall back to AP mode.
// The user can also force WiFi off mid-session via /fly_arm, or force WiFi on
// via the triple-power-cycle escape hatch in setup().
//
//*********************************************************************

#ifndef _SRC_WIFI_H
#define _SRC_WIFI_H

#include "1Defs.h"
#include "Storage.h"

//*********************************************************************
//  Net-mode name (for UI + serial)
//*********************************************************************

inline const char* netModeName() {
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

//*********************************************************************
//  LED helpers (used by both this module and Web/main)
//*********************************************************************
// Active-low: writing LOW turns the LED on.

inline void ledOn()  { digitalWrite(LED_PIN, LOW);  }
inline void ledOff() { digitalWrite(LED_PIN, HIGH); }

//*********************************************************************
//  Uptime as "Xh Ym Zs" (for /diagnostics)
//*********************************************************************

inline String uptimeString() {
    uint32_t s = (millis() - bootMillis) / 1000;
    uint32_t h = s / 3600; s %= 3600;
    uint32_t m = s / 60;   s %= 60;
    char buf[24];
    snprintf(buf, sizeof(buf), "%luh %lum %lus", (unsigned long)h, (unsigned long)m, (unsigned long)s);
    return String(buf);
}

//*********************************************************************
//  ArduinoOTA setup
//*********************************************************************
// Called once we have a working WiFi interface (STA or AP).

inline void setupOTA() {
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

//*********************************************************************
//  mDNS + OTA bring-up
//*********************************************************************

inline void startMdnsAndOta() {
    if (!MDNS.begin(OTA_HOSTNAME)) {
        Serial.println("[mdns] failed");
    } else {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("[mdns] http://%s.local/\n", OTA_HOSTNAME);
    }
    setupOTA();
}

//*********************************************************************
//  HTTP server start (deferred until WiFi is up)
//*********************************************************************
// server.begin() touches LwIP — calling it before WiFi STA or AP is up
// panics the chip ("assert failed: tcpip_send_msg_wait_sem ... Invalid mbox").

inline void startHttpServerIfNeeded() {
    if (httpServerStarted) return;
    server.begin();
    httpServerStarted = true;
    Serial.println("[http] listening on :80");
}

//*********************************************************************
//  STA connect attempt
//*********************************************************************

inline void startWifiStation() {
    String ssid = getEffectiveSsid();
    if (ssid.length() == 0) {
        // No SSID saved → go straight to AP so the user can enter credentials.
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

//*********************************************************************
//  STA connected handler
//*********************************************************************
// Disables WiFi modem sleep — without this the first request after any idle
// period takes ~100-200 ms while the radio wakes up, which feels like UI lag.
// ~20 mA cost is irrelevant when we're powered by USB or BEC.

inline void onWifiConnected() {
    WiFi.setSleep(false);

    Serial.printf("[wifi] %s  RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    char buf[80];
    snprintf(buf, sizeof(buf), "WiFi up: %s", WiFi.localIP().toString().c_str());
    events.add(buf);
    startMdnsAndOta();
    startHttpServerIfNeeded();
    mspBridgeStart();           // tcp/5760 — Configurator wireless MSP (CRSF mode only)
    netMode = NET_WIFI_UP;
    ledOff();
}

//*********************************************************************
//  AP fallback
//*********************************************************************

inline void startApMode() {
    Serial.printf("[wifi] STA failed, starting AP '%s'\n", AP_SSID);
    WiFi.disconnect(true);
    delay(50);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID);
    WiFi.setSleep(false);
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
    mspBridgeStart();
    netMode = NET_AP;
}

//*********************************************************************
//  Force WiFi off (Fly mode)
//*********************************************************************

inline void disableWifi() {
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

//*********************************************************************
//  Periodic state-machine step (call from loop())
//*********************************************************************

inline void netStep() {
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
            break;
    }
}

//*********************************************************************
//  Heartbeat LED — 50 ms blip every 2 s
//*********************************************************************

inline void heartbeat() {
    static uint32_t next = 0;
    static bool     on   = false;
    uint32_t now = millis();
    if (now >= next) {
        on = !on;
        if (on) { ledOn();  next = now + 50;   }
        else    { ledOff(); next = now + 1950; }
    }
}

//*********************************************************************
//  Compute "days since 2020-01-01" from __DATE__
//*********************************************************************
// v1 reports the same quantity as telemetry item 35 (BuildAge); the TX
// subtracts the two and warns if they differ by more than a few days.
// Computed once at boot.

inline uint32_t computeBuildDays() {
    const char* d = __DATE__;                            // "Mmm dd yyyy"
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

#endif // _SRC_WIFI_H
