// LockDownRadioControl — RXV2  ::  Storage.h
//
// NVS-backed persistence helpers. The Preferences object itself (`prefs`) is
// declared in 1Defs.h and opened in setup() before any of these functions are
// called. Keys all live in the same "rxv2" namespace.
//
//*********************************************************************

#ifndef _SRC_STORAGE_H
#define _SRC_STORAGE_H

#include "1Defs.h"

//*********************************************************************
//  Bind pipe persistence
//*********************************************************************

inline void loadBindFromNvs() {
    if (prefs.isKey(NVS_KEY_PIPE) && prefs.getBytesLength(NVS_KEY_PIPE) == 5) {
        prefs.getBytes(NVS_KEY_PIPE, bindState.pipe, 5);
        bindState.bound       = true;
        bindState.boundMillis = millis();
        Serial.printf("[bind] restored from NVS: %02X %02X %02X %02X %02X\n",
                      bindState.pipe[0], bindState.pipe[1], bindState.pipe[2],
                      bindState.pipe[3], bindState.pipe[4]);
    }
}

//*********************************************************************

inline void saveBindToNvs() {
    prefs.putBytes(NVS_KEY_PIPE, bindState.pipe, 5);
    Serial.println("[bind] saved to NVS");
}

//*********************************************************************

inline void clearBindNvs() {
    prefs.remove(NVS_KEY_PIPE);
    bindState.bound       = false;
    bindState.boundMillis = 0;
    memset(bindState.pipe, 0, sizeof(bindState.pipe));
    Serial.println("[bind] cleared NVS");
}

//*********************************************************************
//  WiFi credentials accessors
//*********************************************************************
// Effective = the value we'd actually use this boot. NVS wins if set; otherwise
// the compiled-in (empty) defaults.

inline String getEffectiveSsid() {
    String s = prefs.isKey(NVS_KEY_SSID) ? prefs.getString(NVS_KEY_SSID, "") : "";
    return s.length() ? s : String(WIFI_DEFAULT_SSID);
}

//*********************************************************************

inline String getEffectivePass() {
    String p = prefs.isKey(NVS_KEY_PASS) ? prefs.getString(NVS_KEY_PASS, "") : "";
    return p.length() ? p : String(WIFI_DEFAULT_PASSWORD);
}

//*********************************************************************

inline bool wifiCredsAreCustom() {
    return prefs.isKey(NVS_KEY_SSID) && prefs.getString(NVS_KEY_SSID, "").length() > 0;
}

#endif // _SRC_STORAGE_H
