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

//*********************************************************************
//  Model-name helpers
//*********************************************************************
// The receiver identifies itself to the home network and to the user's
// phone using a name. By default that name is "RXV2-XXXX" where XXXX
// is the last 4 hex digits of the board MAC, guaranteeing uniqueness.
// The user can override with a friendly name (e.g. "Goblin 700") via
// the front page; the friendly name then drives the soft-AP SSID, the
// mDNS hostname, and the page title across the receiver's web UI.

inline String getModelName() {
    return prefs.isKey(NVS_KEY_MODEL_NAME)
               ? prefs.getString(NVS_KEY_MODEL_NAME, "")
               : String();
}

inline bool nameIsCustom() {
    return getModelName().length() > 0;
}

inline String defaultName() {
    // Single fixed default ("RXV2") so a fresh chip is reachable at
    // a predictable hostname / AP SSID — RXV2.local from the phone,
    // "RXV2" in the WiFi network list. The front page then refuses
    // to let the user past until they've assigned a unique friendly
    // name, after which the hostname and AP SSID re-derive from it
    // (so multi-receiver setups stop colliding). Trying to be clever
    // with auto-generated MAC-suffix defaults made the default name
    // unmemorable, which is the opposite of what we wanted.
    return String("RXV2");
}

inline String effectiveName() {
    String n = getModelName();
    return n.length() ? n : defaultName();
}

// Sanitise a display name into a DNS-safe hostname (alphanumeric +
// hyphens, no leading/trailing hyphens, capped at 30 chars). Spaces
// and underscores collapse to hyphens; everything else is dropped.
inline String hostnameFromName(const String& src) {
    String out;
    out.reserve(src.length());
    for (size_t i = 0; i < src.length() && out.length() < 30; i++) {
        char c = src[i];
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9')) {
            out += c;
        } else if (c == '-' || c == ' ' || c == '_') {
            // Avoid runs of hyphens.
            if (!out.length() || out[out.length() - 1] != '-') out += '-';
        }
        // else: drop other chars (apostrophes, accents, emoji, etc.)
    }
    while (out.length() && out[out.length() - 1] == '-')
        out.remove(out.length() - 1);
    if (out.length() == 0) out = "RXV2";
    return out;
}

#endif // _SRC_STORAGE_H
