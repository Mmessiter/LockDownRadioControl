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
    ArduinoOTA.setHostname(g_hostname.c_str());
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
    // From v0.9.51 the chip runs AP+STA simultaneously. The soft-AP
    // `LDRC_RX` is always live at 192.168.4.1 regardless of home WiFi
    // state, so the user can ALWAYS reach the chip — overnight router
    // hiccups, channel storms, ISP outages, none of it strands them in
    // a no-WiFi state any more. The web server, mDNS and MSP bridge
    // all come up immediately on the AP side; STA tries the home
    // network in the background and joins when it can.
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    // AP SSID is the user-friendly model name (or "RXV2-XXXX" default).
    // Phones in the area see "Goblin 700" instead of a generic
    // "LDRC_RX" and can tell receivers apart at a glance.
    WiFi.softAP(g_effectiveName.c_str());
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    Serial.printf("[wifi] soft-AP '%s' up at %s\n",
                  g_effectiveName.c_str(), WiFi.softAPIP().toString().c_str());

    // Bring the web server / mDNS up NOW on the AP side. Once STA
    // connects, the same handlers serve traffic at the LAN IP too —
    // startHttpServerIfNeeded() is idempotent.
    if (MDNS.begin(g_hostname.c_str())) {
        MDNS.addService("http", "tcp", 80);
    }
    setupOTA();
    startHttpServerIfNeeded();
    mspBridgeStart();

    String ssid = getEffectiveSsid();
    if (ssid.length() == 0) {
        // No NVS creds — stay AP-only. User can reach 192.168.4.1 to
        // set them up.
        Serial.println("[wifi] no NVS SSID — AP-only");
        events.add("No SSID — AP-only");
        netMode = NET_AP;
        return;
    }
    Serial.printf("[wifi] STA connecting to '%s' (AP stays up)\n", ssid.c_str());
    WiFi.setHostname(g_hostname.c_str());
    WiFi.begin(ssid.c_str(), getEffectivePass().c_str());
    netMode       = NET_WIFI_CONNECTING;
    netStateStart = millis();

    char buf[80];
    snprintf(buf, sizeof(buf), "Trying STA WiFi '%s' (AP also up)", ssid.c_str());
    events.add(buf);
}

//*********************************************************************
//  STA connected handler
//*********************************************************************
// Disables WiFi modem sleep — without this the first request after any idle
// period takes ~100-200 ms while the radio wakes up, which feels like UI lag.
// ~20 mA cost is irrelevant when we're powered by USB or BEC.

inline void onWifiConnected() {
    // STA joined home WiFi. AP + web server are already up from
    // startWifiStation() so there's nothing to bring up here — we just
    // log, reset counters, and transition state.
    Serial.printf("[wifi] STA %s  RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    char buf[80];
    snprintf(buf, sizeof(buf), "WiFi up: %s", WiFi.localIP().toString().c_str());
    events.add(buf);
    staAttempts = 0;
    netMode = NET_WIFI_UP;
    ledOff();
}

//*********************************************************************
//  AP fallback
//*********************************************************************

// Kept for the quick-boot escape hatch / "no SSID" boot path. Tears
// STA down and stays in AP-only mode. In normal operation we run
// AP+STA via startWifiStation() so this is rarely called.
inline void startApMode() {
    Serial.printf("[wifi] AP-only mode, '%s'\n", g_effectiveName.c_str());
    WiFi.disconnect(true);
    delay(50);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(g_effectiveName.c_str());
    WiFi.setSleep(false);
    delay(150);
    char buf[80];
    snprintf(buf, sizeof(buf), "AP-only mode: %s at %s",
             g_effectiveName.c_str(), WiFi.softAPIP().toString().c_str());
    events.add(buf);
    if (MDNS.begin(g_hostname.c_str())) {
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
        {
            // AP is already up from startWifiStation(), so we NEVER
            // fall back to AP-only from this state any more. Just
            // retry STA forever — the user can always reach the chip
            // via the AP side regardless of how STA is faring.
            wl_status_t s = WiFi.status();
            if (s == WL_CONNECTED) {
                onWifiConnected();
                break;
            }
            if ((uint32_t)(millis() - netStateStart) >= WIFI_CONNECT_MS) {
                staAttempts++;
                Serial.printf("[net] STA attempt %u timed out (status=%d), retrying — AP still up\n",
                              (unsigned)staAttempts, (int)s);
                char buf[64];
                snprintf(buf, sizeof(buf),
                         "STA retry %u (status=%d)",
                         (unsigned)staAttempts, (int)s);
                events.add(buf);
                // Restart the STA side only. AP stays up because we
                // pass `false` to WiFi.disconnect (don't turn WiFi off).
                WiFi.disconnect(false, true);   // disconnect STA, erase saved AP
                delay(200);
                WiFi.begin(getEffectiveSsid().c_str(),
                           getEffectivePass().c_str());
                netStateStart = millis();
                break;
            }
            // Still connecting — blink LED.
            static uint32_t lastBlink = 0;
            if (millis() - lastBlink > 150) {
                lastBlink = millis();
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            }
            break;
        }

        case NET_WIFI_UP:
        {
            // 3-second debounce. ESP32's WiFi.status() can transiently
            // report not-connected during background scans even when
            // the link is fine, so we only treat a drop as real after
            // the radio has been disconnected for several seconds.
            static uint32_t disconnectedSinceMs = 0;
            if (WiFi.status() == WL_CONNECTED) {
                disconnectedSinceMs = 0;
                break;
            }
            if (disconnectedSinceMs == 0) {
                disconnectedSinceMs = millis();
                break;
            }
            if (millis() - disconnectedSinceMs < 3000) break;
            // Genuinely dropped — go back to NET_WIFI_CONNECTING and
            // re-try STA. AP stays up throughout.
            Serial.println("[wifi] STA dropped, retrying (AP still up)");
            events.add("WiFi dropped, retrying");
            disconnectedSinceMs = 0;
            WiFi.disconnect(false, true);
            delay(200);
            WiFi.begin(getEffectiveSsid().c_str(),
                       getEffectivePass().c_str());
            netMode       = NET_WIFI_CONNECTING;
            netStateStart = millis();
            break;
        }

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
