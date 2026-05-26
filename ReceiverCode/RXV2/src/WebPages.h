// LockDownRadioControl — RXV2  ::  WebPages.h
//
// HTTP request handlers. As of v0.6.0 the GET pages are static HTML files in
// data/ (served from LittleFS); each page fetches /api/state.json on load and
// populates placeholders via JS. POST handlers stay here in C++ because they
// send a one-shot confirmation and reboot the chip.
//
// Routes are registered in main.cpp setup() via registerWebRoutes() below so
// the server is ready the instant any net mode brings up an interface.
//
//*********************************************************************

#ifndef _SRC_WEBPAGES_H
#define _SRC_WEBPAGES_H

#include "1Defs.h"
#include "Storage.h"
#include "Output.h"        // protocolName(), protocolDesc()
#include "Network.h"       // netModeName(), uptimeString(), disableWifi()
#include "Radio.h"         // runRadioSelfTest(), dataRateName()

//*********************************************************************
//  Embedded fallback CSS (used only if LittleFS mount failed)
//*********************************************************************
// The canonical stylesheet lives in data/style.css — this is the rescue
// version so a totally-broken FS still produces a readable page.

inline const char PAGE_CSS_FALLBACK[] PROGMEM = R"CSS(
body{font-family:-apple-system,sans-serif;background:#f4e4c1;color:#2c3e50;padding:1em}
.container{max-width:500px;margin:0 auto}
.card{background:rgba(255,255,255,.65);border-radius:14px;padding:1.2em 1.4em;margin:.8em 0}
.btn{display:block;padding:1em;margin:.5em 0;background:#7d9eb0;color:#fff;text-decoration:none;border-radius:8px;text-align:center}
.muted{color:#7a8b95}
.footer{text-align:center;margin-top:2em;color:#5d7a8c;font-size:.85em}
)CSS";

//*********************************************************************
//  Embedded fallback home page (LittleFS broken / first-ever boot)
//*********************************************************************
// If we can't serve data/index.html, send this minimal recovery page that
// at least links to /wifi so the user can configure access.

inline const char FALLBACK_INDEX[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset=utf-8>
<title>LDRC RX V2 — recovery</title>
<style>body{font-family:-apple-system,sans-serif;background:#fff;color:#2c3e50;padding:1.5em;max-width:520px;margin:0 auto}
h1{color:#c97464}.card{background:#f4e4c1;border-radius:12px;padding:1em;margin:1em 0}
a{display:block;padding:.8em;margin:.5em 0;background:#7d9eb0;color:#fff;text-decoration:none;border-radius:8px;text-align:center}</style></head>
<body><h1>Recovery mode</h1>
<div class=card><p><b>LittleFS isn't mounted</b> &mdash; the web UI assets aren't available.
Re-flash the filesystem with <code>pio run -e xiao_ota -t uploadfs</code> from a development Mac.</p>
<p>Limited functions still work from this fallback:</p></div>
<a href="/wifi">WiFi settings (recovery form)</a>
<a href="/firmware">Firmware (OTA still works)</a>
<a href="/diagnostics">Diagnostics (JSON only)</a>
</body></html>
)HTML";

//*********************************************************************
//  Serve a file from LittleFS (returns false if not found / FS not mounted)
//*********************************************************************

inline bool serveLittleFsFile(const char* path, const char* mime) {
    if (!littleFsMounted) return false;
    if (!LittleFS.exists(path)) return false;
    File f = LittleFS.open(path, "r");
    if (!f) return false;
    server.streamFile(f, mime);
    f.close();
    return true;
}

//*********************************************************************
//  /style.css — try LittleFS first, fall back to embedded
//*********************************************************************

inline void handleStyleCss() {
    server.sendHeader("Cache-Control", "public, max-age=86400");
    if (serveLittleFsFile("/style.css", "text/css")) return;
    server.send_P(200, "text/css", PAGE_CSS_FALLBACK);
}

//*********************************************************************
//  /app.js — shared JS helpers used by every page
//*********************************************************************

inline void handleAppJs() {
    server.sendHeader("Cache-Control", "public, max-age=86400");
    if (serveLittleFsFile("/app.js", "application/javascript")) return;
    server.send(503, "text/plain", "/app.js not in LittleFS — uploadfs the data/ folder");
}

//*********************************************************************
//  Static-page handlers — each just streams the corresponding HTML file
//*********************************************************************

inline void handleRoot() {
    if (serveLittleFsFile("/index.html", "text/html")) return;
    server.send_P(200, "text/html", FALLBACK_INDEX);
}

inline void handleFly() {
    if (serveLittleFsFile("/fly.html", "text/html")) return;
    server.send(503, "text/plain", "/fly.html missing — uploadfs the data/ folder");
}

inline void handleDiagnostics() {
    if (serveLittleFsFile("/diagnostics.html", "text/html")) return;
    server.send(503, "text/plain", "/diagnostics.html missing — uploadfs the data/ folder");
}

inline void handleBlackbox() {
    if (serveLittleFsFile("/blackbox.html", "text/html")) return;
    server.send(503, "text/plain", "/blackbox.html missing — uploadfs the data/ folder");
}

inline void handleBind() {
    if (serveLittleFsFile("/bind.html", "text/html")) return;
    server.send(503, "text/plain", "/bind.html missing — uploadfs the data/ folder");
}

inline void handleFirmware() {
    if (serveLittleFsFile("/firmware.html", "text/html")) return;
    server.send(503, "text/plain", "/firmware.html missing — uploadfs the data/ folder");
}

inline void handleWifi() {
    if (serveLittleFsFile("/wifi.html", "text/html")) return;
    server.send(503, "text/plain", "/wifi.html missing — uploadfs the data/ folder");
}

inline void handleProtocolPage() {
    if (serveLittleFsFile("/protocol.html", "text/html")) return;
    server.send(503, "text/plain", "/protocol.html missing — uploadfs the data/ folder");
}

inline void handleRotorflight() {
    if (serveLittleFsFile("/rotorflight.html", "text/html")) return;
    server.send(503, "text/plain", "/rotorflight.html missing — uploadfs the data/ folder");
}

inline void handleRotorflightPid() {
    if (serveLittleFsFile("/rotorflight-pid.html", "text/html")) return;
    server.send(503, "text/plain", "/rotorflight-pid.html missing — uploadfs the data/ folder");
}

inline void handleRotorflightPidPlus() {
    if (serveLittleFsFile("/rotorflight-pidplus.html", "text/html")) return;
    server.send(503, "text/plain", "/rotorflight-pidplus.html missing — uploadfs the data/ folder");
}

inline void handleRotorflightRates() {
    if (serveLittleFsFile("/rotorflight-rates.html", "text/html")) return;
    server.send(503, "text/plain", "/rotorflight-rates.html missing — uploadfs the data/ folder");
}

inline void handleRotorflightGovProfile() {
    if (serveLittleFsFile("/rotorflight-gov-profile.html", "text/html")) return;
    server.send(503, "text/plain", "/rotorflight-gov-profile.html missing — uploadfs the data/ folder");
}

inline void handleRotorflightGovGlobal() {
    if (serveLittleFsFile("/rotorflight-gov-global.html", "text/html")) return;
    server.send(503, "text/plain", "/rotorflight-gov-global.html missing — uploadfs the data/ folder");
}

inline void handleRotorflightBackups() {
    if (serveLittleFsFile("/rotorflight-backups.html", "text/html")) return;
    server.send(503, "text/plain", "/rotorflight-backups.html missing — uploadfs the data/ folder");
}

//*********************************************************************
//  Backups — JSON snapshots of every editable Rotorflight parameter
//*********************************************************************
// Stored at /backups/<name>.json on LittleFS. Each file ~1.5 KB. The JS
// page owns the snapshot format; the chip is just a key/value store.
// Backup names: alnum, '-', '_', '.', max 32 chars. Hard cap 20 files.

constexpr const char* BACKUP_DIR = "/backups";
constexpr uint8_t     BACKUP_MAX = 20;

inline bool backupNameOk(const String& n) {
    if (n.length() == 0 || n.length() > 32) return false;
    for (size_t i = 0; i < n.length(); i++) {
        char c = n[i];
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
        if (!ok) return false;
    }
    return true;
}

inline String backupPath(const String& name) {
    String p = BACKUP_DIR;
    p += '/'; p += name; p += ".json";
    return p;
}

// GET /api/backup/list  → JSON: {"names":[...], "max":20, "free_bytes":N}
inline void handleBackupList() {
    if (!littleFsMounted) { server.send(500, "text/plain", "FS not mounted"); return; }
    if (!LittleFS.exists(BACKUP_DIR)) LittleFS.mkdir(BACKUP_DIR);
    File dir = LittleFS.open(BACKUP_DIR);
    String j = "{\"names\":[";
    bool first = true;
    if (dir) {
        File f = dir.openNextFile();
        while (f) {
            String n = f.name();
            // Strip directory path + ".json" suffix
            int slash = n.lastIndexOf('/');
            if (slash >= 0) n = n.substring(slash + 1);
            if (n.endsWith(".json")) n = n.substring(0, n.length() - 5);
            if (!first) j += ',';
            j += '"'; j += n; j += "\"";
            first = false;
            f.close();
            f = dir.openNextFile();
        }
        dir.close();
    }
    j += "],\"max\":"; j += BACKUP_MAX;
    j += ",\"free_bytes\":"; j += (uint32_t)(LittleFS.totalBytes() - LittleFS.usedBytes());
    j += "}";
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", j);
}

// GET /api/backup/load?name=X  → the JSON file's raw content
inline void handleBackupLoad() {
    if (!server.hasArg("name")) { server.send(400, "text/plain", "missing name"); return; }
    String name = server.arg("name");
    if (!backupNameOk(name)) { server.send(400, "text/plain", "bad name"); return; }
    String path = backupPath(name);
    if (!LittleFS.exists(path)) { server.send(404, "text/plain", "no such backup"); return; }
    File f = LittleFS.open(path, "r");
    if (!f) { server.send(500, "text/plain", "open failed"); return; }
    server.sendHeader("Cache-Control", "no-store");
    server.streamFile(f, "application/json");
    f.close();
}

// POST /api/backup/save?name=X  with JSON body  → 200 ok / 4xx error
inline void handleBackupSave() {
    if (!server.hasArg("name"))  { server.send(400, "text/plain", "missing name"); return; }
    if (!server.hasArg("plain")) { server.send(400, "text/plain", "missing body"); return; }
    String name = server.arg("name");
    if (!backupNameOk(name)) { server.send(400, "text/plain", "bad name"); return; }
    if (!LittleFS.exists(BACKUP_DIR)) LittleFS.mkdir(BACKUP_DIR);
    // Enforce hard cap. Allow overwrite of an existing name without counting it.
    String path = backupPath(name);
    bool replacing = LittleFS.exists(path);
    if (!replacing) {
        uint8_t count = 0;
        File dir = LittleFS.open(BACKUP_DIR);
        if (dir) {
            File f = dir.openNextFile();
            while (f) { count++; f.close(); f = dir.openNextFile(); }
            dir.close();
        }
        if (count >= BACKUP_MAX) {
            server.send(507, "text/plain", "backup limit reached — delete one first");
            return;
        }
    }
    File f = LittleFS.open(path, "w");
    if (!f) { server.send(500, "text/plain", "open for write failed"); return; }
    String body = server.arg("plain");
    f.print(body);
    f.close();
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain", "saved");
}

// POST /api/backup/delete?name=X
inline void handleBackupDelete() {
    if (!server.hasArg("name")) { server.send(400, "text/plain", "missing name"); return; }
    String name = server.arg("name");
    if (!backupNameOk(name)) { server.send(400, "text/plain", "bad name"); return; }
    String path = backupPath(name);
    if (!LittleFS.exists(path)) { server.send(404, "text/plain", "no such backup"); return; }
    if (!LittleFS.remove(path)) { server.send(500, "text/plain", "remove failed"); return; }
    server.send(200, "text/plain", "deleted");
}

//*********************************************************************
//  GET /api/msp?fn=N[&data=HEX]  — synchronous MSP request to the FC
//*********************************************************************
// Returns the response payload as a hex string. Empty body on no-data
// success. 504 if the FC doesn't reply within ~500 ms.

inline uint8_t hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return 0xFF;
}

inline void handleMspApi() {
    if (!server.hasArg("fn")) { server.send(400, "text/plain", "missing fn"); return; }
    uint8_t fn = (uint8_t)server.arg("fn").toInt();

    uint8_t reqBuf[128] = {0};
    uint8_t reqLen = 0;
    if (server.hasArg("data")) {
        String h = server.arg("data");
        if (h.length() % 2 != 0) { server.send(400, "text/plain", "data length odd"); return; }
        if (h.length() / 2 > sizeof(reqBuf)) { server.send(400, "text/plain", "data too long"); return; }
        for (size_t i = 0; i + 1 < h.length(); i += 2) {
            uint8_t hi = hexNibble(h[i]);
            uint8_t lo = hexNibble(h[i + 1]);
            if (hi == 0xFF || lo == 0xFF) { server.send(400, "text/plain", "bad hex"); return; }
            reqBuf[reqLen++] = (uint8_t)((hi << 4) | lo);
        }
    }

    uint8_t  respBuf[256];
    uint16_t respLen = 0;
    // 400 ms timeout: was 150 ms but read-failed too often. FC normally
    // responds in <50 ms, but if a periodic mspFcPoll probe queued behind
    // our request, the FC processes it first and ours can take longer.
    // (mspFcPoll is now blocked while a sync wait is in flight, but this
    // also handles the case where the user starts a new wait while a
    // probe response is mid-flight on the wire.)
    bool ok = mspRequestAndWait(fn, reqBuf, reqLen, respBuf, &respLen, 400);
    if (!ok) { server.send(504, "text/plain", "flight controller did not respond within 400 ms"); return; }

    String s;
    s.reserve(respLen * 2 + 4);
    char tmp[4];
    for (uint16_t i = 0; i < respLen; ++i) {
        snprintf(tmp, sizeof(tmp), "%02X", respBuf[i]);
        s += tmp;
    }
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain", s);
}

//*********************************************************************
//  POST /api/firmware/seturl — store manifest URL in NVS
//*********************************************************************

inline void handleFirmwareSetUrl() {
    if (!server.hasArg("url")) {
        server.send(400, "text/plain", "missing url");
        return;
    }
    String u = server.arg("url");
    u.trim();
    prefs.putString(NVS_KEY_FW_MANIFEST, u);
    events.add("Firmware manifest URL updated");
    server.send(200, "text/plain", "ok");
}

//*********************************************************************
//  POST /api/firmware/install?url=... — chip downloads .bin and applies
//*********************************************************************
// Synchronous — the HTTP response is returned only after the download
// completes (or fails), then the chip reboots.

inline void handleFirmwareInstall() {
    if (!server.hasArg("url")) {
        server.send(400, "text/plain", "missing url");
        return;
    }
    String url = server.arg("url");
    HTTPClient http;
    if (!http.begin(url)) {
        server.send(500, "text/plain", "http.begin failed");
        return;
    }
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "HTTP %d from server", code);
        http.end();
        server.send(502, "text/plain", msg);
        return;
    }
    int len = http.getSize();
    if (len <= 0) {
        http.end();
        server.send(502, "text/plain", "no content-length");
        return;
    }
    if (!Update.begin((size_t)len)) {
        http.end();
        server.send(500, "text/plain", Update.errorString());
        return;
    }
    WiFiClient* stream = http.getStreamPtr();
    size_t written = Update.writeStream(*stream);
    if (written != (size_t)len) {
        Update.end();
        http.end();
        char msg[80];
        snprintf(msg, sizeof(msg), "short write: %u / %d", (unsigned)written, len);
        server.send(500, "text/plain", msg);
        return;
    }
    if (!Update.end(true)) {
        http.end();
        server.send(500, "text/plain", Update.errorString());
        return;
    }
    http.end();
    events.add("Firmware installed via auto-update — rebooting");
    server.send(200, "text/plain", "ok — rebooting");
    delay(300);
    ESP.restart();
}

//*********************************************************************
//  Minimal POST-confirmation page (rendered inline, then chip reboots)
//*********************************************************************
// We don't load the full app shell here because the page lives ~500 ms before
// the chip resets. A tiny inline page is faster and survives even if
// LittleFS is broken.

inline String confirmPage(const char* title, const char* body) {
    String s;
    s.reserve(400);
    s += F("<!doctype html><html><head><meta charset=utf-8>");
    s += F("<meta name=viewport content='width=device-width,initial-scale=1'>");
    s += F("<title>"); s += title; s += F(" &middot; LDRC RX V2</title>");
    s += F("<link rel=stylesheet href='/style.css'>");
    s += F("</head><body><div class=container><h1>"); s += title; s += F("</h1>");
    s += F("<div class=card>"); s += body; s += F("</div>");
    s += F("<div class=footer>"); s += FW_VERSION; s += F("</div></div></body></html>");
    return s;
}

//*********************************************************************
//  POST /bind — clear bind and reboot
//*********************************************************************

inline void handleBindDo() {
    clearBindNvs();
    String body = confirmPage("Rebooting",
        "<p>Bind cleared. The receiver is rebooting and will listen on DefaultPipe within ~5 seconds.</p>"
        "<p><a href='/'>Back to home</a> (reload after the chip is back)</p>");
    server.send(200, "text/html", body);
    delay(400);
    ESP.restart();
}

//*********************************************************************
//  POST /rollback — switch OTA slot and reboot
//*********************************************************************

inline void handleRollback() {
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
    String msg = "<p>Switching boot to slot <code>";
    msg += other->label;
    msg += "</code> and restarting. If that slot is invalid, the bootloader will bring you back to the current firmware automatically.</p>"
           "<p><a href='/'>Back to home</a> (reload after the chip is back)</p>";
    server.send(200, "text/html", confirmPage("Rolling back", msg.c_str()));
    delay(400);
    ESP.restart();
}

//*********************************************************************
//  POST /wifi — save WiFi credentials and reboot
//*********************************************************************

inline void handleWifiSet() {
    if (server.hasArg("ssid")) {
        prefs.putString(NVS_KEY_SSID, server.arg("ssid"));
    }
    if (server.hasArg("pass") && server.arg("pass").length() > 0) {
        prefs.putString(NVS_KEY_PASS, server.arg("pass"));
    }
    events.add("WiFi credentials saved");

    String b = "<p>WiFi credentials saved. Receiver is rebooting and will attempt to join '";
    b += server.hasArg("ssid") ? server.arg("ssid") : getEffectiveSsid();
    b += "' on next boot.</p>";
    server.send(200, "text/html", confirmPage("Saved & rebooting", b.c_str()));
    delay(500);
    ESP.restart();
}

//*********************************************************************
//  POST /wifi_reset — wipe stored credentials and reboot
//*********************************************************************

inline void handleWifiReset() {
    prefs.remove(NVS_KEY_SSID);
    prefs.remove(NVS_KEY_PASS);
    events.add("WiFi NVS credentials wiped");
    server.send(200, "text/html", confirmPage("Reset",
        "<p>NVS wiped. Receiver is rebooting and will use the compiled-in defaults.</p>"));
    delay(500);
    ESP.restart();
}

//*********************************************************************
//  POST /protocol — save output protocol selection and reboot
//*********************************************************************

inline void handleProtocolSet() {
    if (server.hasArg("proto")) {
        long v = server.arg("proto").toInt();
        if (v >= PROTO_SBUS && v <= PROTO_MAX) {
            prefs.putUChar(NVS_KEY_PROTO, (uint8_t)v);
            char buf[60];
            snprintf(buf, sizeof(buf), "Output protocol set to %s", protocolName((Protocol)v));
            events.add(buf);
        }
    }
    prefs.putUChar(NVS_KEY_PPM_INV, server.hasArg("ppm_inv") ? 1 : 0);
    server.send(200, "text/html", confirmPage("Saved & rebooting",
        "<p>Output protocol updated. The receiver is rebooting to apply.</p>"));
    delay(500);
    ESP.restart();
}

//*********************************************************************
//  POST /fly_arm — disable WiFi until next reboot
//*********************************************************************

inline void handleFlyArm() {
    server.send(200, "text/html", confirmPage("Flying",
        "<p>The receiver is now in <b>RF-only</b> mode. WiFi will return on the next power-cycle.</p>"
        "<p class=muted>SBUS keeps streaming. This page will stop responding the moment WiFi shuts down (any second now).</p>"));
    flyArmRequested = true;       // disableWifi() runs from loop() after this response flushes
}

//*********************************************************************
//  GET /reboot — soft reset
//*********************************************************************

inline void handleReboot() {
    server.send(200, "text/html", confirmPage("Rebooting",
        "<p>Back in ~5 s.</p>"));
    delay(400);
    ESP.restart();
}

//*********************************************************************
//  GET /retest — re-run self-test, redirect to /diagnostics
//*********************************************************************

inline void handleRetest() {
    runRadioSelfTest();
    server.sendHeader("Location", "/diagnostics", true);
    server.send(303, "text/plain", "retested");
}

//*********************************************************************
//  /api/events.json — black-box event log, newest-first
//*********************************************************************

inline void handleApiEvents() {
    String j;
    j.reserve(events.count * 100 + 8);
    j += '[';
    if (events.count > 0) {
        size_t n = events.count;
        size_t idx = (events.head + EventLog::SIZE - 1) % EventLog::SIZE;
        for (size_t k = 0; k < n; ++k) {
            if (k) j += ',';
            j += "{\"t\":"; j += events.when[idx];
            j += ",\"msg\":\"";
            // Minimal JSON escaping
            const char* m = events.msgs[idx];
            for (; *m; ++m) {
                if (*m == '"' || *m == '\\') j += '\\';
                j += *m;
            }
            j += "\"}";
            idx = (idx + EventLog::SIZE - 1) % EventLog::SIZE;
        }
    }
    j += ']';
    server.send(200, "application/json", j);
}

//*********************************************************************
//  /api/state.json — comprehensive single endpoint for all pages
//*********************************************************************

inline void handleApiState() {
    String j;
    j.reserve(2200);
    char buf[96];

    // --- info ----------------------------------------------------------
    j += "{\"info\":{";
    j += "\"fw_version\":\""; j += FW_VERSION; j += "\"";
    j += ",\"build_date\":\""; j += __DATE__; j += ' '; j += __TIME__; j += "\"";
    j += ",\"hostname\":\""; j += OTA_HOSTNAME; j += "\"";
    j += ",\"ip\":\""; j += WiFi.localIP().toString(); j += "\"";
    j += ",\"mac\":\""; j += WiFi.macAddress(); j += "\"";
    j += ",\"rssi\":"; j += (netMode == NET_WIFI_UP ? (int)WiFi.RSSI() : 0);
    j += ",\"uptime_s\":"; j += (uint32_t)((millis() - bootMillis) / 1000);
    j += ",\"uptime_str\":\""; j += uptimeString(); j += "\"";
    j += ",\"free_heap\":"; j += ESP.getFreeHeap();
    j += ",\"chip\":\""; j += ESP.getChipModel(); j += "\"";
    j += ",\"chip_rev\":"; j += ESP.getChipRevision();
    j += ",\"littlefs\":"; j += (littleFsMounted ? "true" : "false");
    j += ",\"ap_ssid\":\""; j += AP_SSID; j += "\"";
    j += ",\"ap_ip\":\""; j += (netMode == NET_AP ? WiFi.softAPIP().toString() : String("")); j += "\"";

    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* other   = esp_ota_get_next_update_partition(NULL);
    j += ",\"partition_running\":\""; j += (running ? running->label : ""); j += "\"";
    j += ",\"partition_other\":\""; j += (other ? other->label : ""); j += "\"";
    j += ",\"partition_other_kb\":"; j += (other ? other->size / 1024 : 0);
    j += "}";

    // --- net ----------------------------------------------------------
    j += ",\"net\":{";
    j += "\"mode\":\""; j += netModeName(); j += "\"";
    j += ",\"ssid\":\""; j += getEffectiveSsid(); j += "\"";
    j += ",\"ssid_custom\":"; j += (wifiCredsAreCustom() ? "true" : "false");
    j += "}";

    // --- rf -----------------------------------------------------------
    j += ",\"rf\":{";
    snprintf(buf, sizeof(buf), "\"packets\":%u,\"acks_written\":%u,\"mac_acks_sent\":%u,\"mac_ack_threshold\":%u",
             (unsigned)rx.packets, (unsigned)rx.acksWritten, (unsigned)macAcksSent, (unsigned)MAC_ACK_THRESHOLD);
    j += buf;
    j += ",\"last_pkt_ms\":";
    if (rx.lastMillis) j += (uint32_t)(millis() - rx.lastMillis); else j += "-1";
    j += ",\"radios_count\":"; j += numRadiosPresent;
    j += ",\"radios_present\":["; j += (radioPresent[0] ? "true" : "false");
    j += ',';                     j += (radioPresent[1] ? "true" : "false");
    j += ',';                     j += (radioPresent[2] ? "true" : "false");
    j += "]";
    // Legacy field — kept for any older diagnostics page that reads it.
    j += ",\"radios_dual\":"; j += (numRadiosPresent >= 2 ? "true" : "false");
    j += ",\"active_radio\":"; j += activeRadioIdx;
    j += ",\"radio_swaps\":"; j += radioSwaps;
    j += ",\"fhss_enabled\":"; j += (fhssEnabled ? "true" : "false");
    j += ",\"fhss_idx\":"; j += nextChannelIdx;
    j += ",\"fhss_channel\":"; j += FHSS_CHANNELS[nextChannelIdx];
    j += ",\"telemetry_item\":"; j += telemetryItem;
    j += ",\"sbus_frames_out\":"; j += sbusFramesOut;
    j += ",\"last_channel_ms\":";
    if (lastChannelDataMs) j += (uint32_t)(millis() - lastChannelDataMs); else j += "-1";

    snprintf(buf, sizeof(buf), ",\"board_mac\":\"%02X%02X%02X%02X%02X%02X\"",
             boardMac[0], boardMac[1], boardMac[2], boardMac[3], boardMac[4], boardMac[5]);
    j += buf;

    // Last & max payload hex dumps (up to 32 bytes each)
    j += ",\"last_payload_len\":"; j += rx.lastPayload;
    j += ",\"last_payload\":\"";
    for (uint8_t i = 0; i < rx.lastPayload && i < 32; ++i) {
        snprintf(buf, sizeof(buf), "%02X ", rx.lastBytes[i]);
        j += buf;
    }
    j += "\"";
    j += ",\"max_payload_len\":"; j += rx.maxPayload;
    j += ",\"max_payload\":\"";
    for (uint8_t i = 0; i < rx.maxPayload && i < 32; ++i) {
        snprintf(buf, sizeof(buf), "%02X ", rx.maxBytes[i]);
        j += buf;
    }
    j += "\"";

    // Self-test
    j += ",\"self_test\":{";
    j += "\"verdict\":\""; j += rfTest.verdict; j += "\"";
    j += ",\"begin_ok\":"; j += (rfTest.beginOk ? "true" : "false");
    j += ",\"chip_connected\":"; j += (rfTest.chipConnected ? "true" : "false");
    j += ",\"channel_ok\":"; j += (rfTest.channelOk ? "true" : "false");
    j += ",\"data_rate_ok\":"; j += (rfTest.dataRateOk ? "true" : "false");
    j += ",\"channel_read\":"; j += rfTest.channelRead;
    j += ",\"data_rate_name\":\""; j += dataRateName(static_cast<rf24_datarate_e>(rfTest.dataRateRead)); j += "\"";
    j += "}}";

    // --- bind ---------------------------------------------------------
    j += ",\"bind\":{";
    j += "\"bound\":"; j += (bindState.bound ? "true" : "false");
    j += ",\"attempts\":"; j += bindState.attempts;
    snprintf(buf, sizeof(buf), ",\"pipe\":\"%02X %02X %02X %02X %02X\"",
             bindState.pipe[0], bindState.pipe[1], bindState.pipe[2], bindState.pipe[3], bindState.pipe[4]);
    j += buf;
    j += ",\"bound_s_ago\":";
    if (bindState.bound && bindState.boundMillis) j += (uint32_t)((millis() - bindState.boundMillis) / 1000);
    else j += "0";
    j += "}";

    // --- protocol -----------------------------------------------------
    j += ",\"protocol\":{";
    j += "\"current\":\""; j += protocolName(currentProtocol); j += "\"";
    j += ",\"ppm_inverted\":"; j += (ppmInverted ? "true" : "false");
    j += ",\"available\":[";
    // Display order — CRSF first (most-used), then the rest in enum order.
    static const Protocol DISPLAY_ORDER[] = { PROTO_CRSF, PROTO_SBUS, PROTO_IBUS, PROTO_PPM };
    for (size_t k = 0; k < sizeof(DISPLAY_ORDER) / sizeof(DISPLAY_ORDER[0]); ++k) {
        Protocol p = DISPLAY_ORDER[k];
        if (p > PROTO_MAX) continue;
        if (k > 0) j += ',';
        j += "{\"id\":"; j += (uint8_t)p;
        j += ",\"name\":\""; j += protocolName(p); j += "\"";
        j += ",\"desc\":\""; j += protocolDesc(p); j += "\"}";
    }
    j += "]}";

    // --- fc telemetry ------------------------------------------------
    j += ",\"fc\":{";
    j += "\"valid\":"; j += (fcTelem.valid ? "true" : "false");
    j += ",\"raw_total\":"; j += fcRawTotal;
    j += ",\"bytes\":"; j += fcTelem.bytesIn;
    j += ",\"frames\":"; j += fcTelem.framesParsed;
    j += ",\"crc_err\":"; j += fcTelem.crcErrors;
    j += ",\"responses\":"; j += fcTelem.responsesSent;
    j += ",\"last_frame_ms\":";
    if (fcTelem.lastFrameMs) j += (uint32_t)(millis() - fcTelem.lastFrameMs); else j += "-1";

    if (fcTelem.valid) {
        snprintf(buf, sizeof(buf), ",\"v\":%.2f,\"a\":%.2f,\"mah\":%u,\"pct\":%u",
                 fcTelem.fcBattVolts, fcTelem.fcBattAmps,
                 (unsigned)fcTelem.fcBattMah, (unsigned)fcTelem.fcBattPct);
        j += buf;
        snprintf(buf, sizeof(buf), ",\"rssi\":%d,\"lq\":%u,\"snr\":%d",
                 (int)fcTelem.fcUplinkRssi, (unsigned)fcTelem.fcUplinkLq, (int)fcTelem.fcUplinkSnr);
        j += buf;
        snprintf(buf, sizeof(buf), ",\"pitch\":%d,\"roll\":%d,\"yaw\":%d",
                 fcTelem.attitudePitch, fcTelem.attitudeRoll, fcTelem.attitudeYaw);
        j += buf;
        j += ",\"flight_mode\":\"";
        for (const char* p = fcTelem.flightMode; *p; ++p) {
            if (*p == '"' || *p == '\\') j += '\\';
            j += *p;
        }
        j += "\"";
    } else {
        j += ",\"v\":0,\"a\":0,\"mah\":0,\"pct\":0,\"rssi\":0,\"lq\":0,\"snr\":0";
        j += ",\"pitch\":0,\"roll\":0,\"yaw\":0,\"flight_mode\":\"\"";
    }

    // Raw byte ring dump (oldest-to-newest)
    j += ",\"raw_dump\":\"";
    {
        uint16_t start = (fcRawCount < FC_RAW_RING_SIZE) ? 0 : fcRawHead;
        for (uint16_t i = 0; i < fcRawCount; ++i) {
            uint16_t idx = (uint16_t)((start + i) % FC_RAW_RING_SIZE);
            snprintf(buf, sizeof(buf), "%02X ", fcRawRing[idx]);
            j += buf;
        }
    }
    j += "\"";
    j += "}";

    // --- firmware (auto-update) --------------------------------------
    j += ",\"fw\":{";
    j += "\"manifest_url\":\""; {
        String u = prefs.isKey(NVS_KEY_FW_MANIFEST) ? prefs.getString(NVS_KEY_FW_MANIFEST, "") : "";
        for (size_t i = 0; i < u.length(); ++i) {
            char c = u[i];
            if (c == '"' || c == '\\') j += '\\';
            j += c;
        }
    } j += "\"";
    j += "}";

    // --- msp bridge --------------------------------------------------
    j += ",\"msp\":{";
    j += "\"started\":"; j += (mspBridgeStarted ? "true" : "false");
    j += ",\"active\":"; j += (mspBridgeActive ? "true" : "false");
    j += ",\"port\":"; j += MSP_BRIDGE_PORT;
    j += ",\"bytes_in\":"; j += mspBridgeBytesIn;
    j += ",\"bytes_out\":"; j += mspBridgeBytesOut;
    j += ",\"connections\":"; j += mspBridgeConnections;
    if (mspBridgeActive) {
        j += ",\"client_ip\":\""; j += mspClient.remoteIP().toString(); j += "\"";
        j += ",\"connected_s\":"; j += (uint32_t)((millis() - mspBridgeConnectMs) / 1000);
    }
    j += "}";

    // --- fc detection (MSP-over-CRSF probe) --------------------------
    j += ",\"fcinfo\":{";
    j += "\"detected\":"; j += (fcInfo.detected ? "true" : "false");
    j += ",\"variant\":\""; j += fcInfo.variant; j += "\"";
    j += ",\"version_known\":"; j += (fcInfo.versionKnown ? "true" : "false");
    j += ",\"fw_major\":"; j += fcInfo.fwMajor;
    j += ",\"fw_minor\":"; j += fcInfo.fwMinor;
    j += ",\"fw_patch\":"; j += fcInfo.fwPatch;
    j += ",\"msp_proto\":"; j += fcInfo.mspProto;
    j += ",\"api_major\":"; j += fcInfo.apiMajor;
    j += ",\"api_minor\":"; j += fcInfo.apiMinor;
    j += ",\"probes_sent\":"; j += fcInfo.probesSent;
    j += ",\"last_response_ms\":";
    if (fcInfo.lastResponseMs) j += (uint32_t)(millis() - fcInfo.lastResponseMs); else j += "-1";
    j += ",\"rotorflight_capable\":"; j += (fcIsRotorflightConfigCapable() ? "true" : "false");
    j += ",\"rf_major\":"; j += rotorflightMajor();
    j += ",\"rf_minor\":"; j += rotorflightMinor();
    j += "}";

    // --- channels ----------------------------------------------------
    j += ",\"channels\":[";
    for (int i = 0; i < 16; ++i) {
        if (i) j += ',';
        j += channelMicros[i];
    }
    j += "]}";

    server.send(200, "application/json", j);
}

//*********************************************************************
//  Route registration (call from setup())
//*********************************************************************

inline void registerWebRoutes() {
    // Static GET pages (served from LittleFS)
    server.on("/",            handleRoot);
    server.on("/fly",         handleFly);
    server.on("/diagnostics", handleDiagnostics);
    server.on("/blackbox",    handleBlackbox);
    server.on("/bind",        HTTP_GET,  handleBind);
    server.on("/firmware",    handleFirmware);
    server.on("/wifi",        HTTP_GET,  handleWifi);
    server.on("/protocol",    HTTP_GET,  handleProtocolPage);
    server.on("/rotorflight",     handleRotorflight);
    server.on("/rotorflight-pid",     handleRotorflightPid);
    server.on("/rotorflight-pidplus", handleRotorflightPidPlus);
    server.on("/rotorflight-rates",        handleRotorflightRates);
    server.on("/rotorflight-gov-profile",  handleRotorflightGovProfile);
    server.on("/rotorflight-gov-global",   handleRotorflightGovGlobal);
    server.on("/rotorflight-backups",      handleRotorflightBackups);
    server.on("/api/backup/list",          HTTP_GET,  handleBackupList);
    server.on("/api/backup/load",          HTTP_GET,  handleBackupLoad);
    server.on("/api/backup/save",          HTTP_POST, handleBackupSave);
    server.on("/api/backup/delete",        HTTP_POST, handleBackupDelete);
    server.on("/api/msp",         HTTP_GET, handleMspApi);

    // Auto-update endpoints.
    server.on("/api/firmware/seturl",  HTTP_POST, handleFirmwareSetUrl);
    server.on("/api/firmware/install", HTTP_POST, handleFirmwareInstall);

    // Shared assets
    server.on("/style.css",   handleStyleCss);
    server.on("/app.js",      handleAppJs);

    // JSON APIs
    server.on("/api/state.json",  handleApiState);
    server.on("/api/events.json", handleApiEvents);

    // POSTs that reboot
    server.on("/bind",        HTTP_POST, handleBindDo);
    server.on("/rollback",    HTTP_POST, handleRollback);
    server.on("/wifi",        HTTP_POST, handleWifiSet);
    server.on("/wifi_reset",  HTTP_POST, handleWifiReset);
    server.on("/protocol",    HTTP_POST, handleProtocolSet);
    server.on("/fly_arm",     HTTP_POST, handleFlyArm);

    // Misc
    server.on("/retest",      handleRetest);
    server.on("/reboot",      handleReboot);
    server.on("/ping", [](){
        // Tiny endpoint to keep iOS's TCP/WiFi state warm. No allocations.
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "text/plain", "ok");
    });
    server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
}

#endif // _SRC_WEBPAGES_H
