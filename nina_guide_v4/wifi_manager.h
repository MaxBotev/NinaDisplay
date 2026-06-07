/*
 * wifi_manager.h
 * Captive-portal WiFi provisioning for ESP32.
 *
 * On startup:
 *   1. Tries to connect to saved credentials from NVS.
 *   2. If no saved credentials OR connection fails after timeout:
 *      - Starts AP "NINA-Display" (no password)
 *      - Spins up a web server on 192.168.4.1
 *      - Scans for nearby networks and serves a selection page
 *      - User connects phone to "NINA-Display", opens browser → auto-redirects
 *      - User picks network, enters password, hits Connect
 *      - Credentials saved to NVS, board reboots and connects
 *
 * Usage in setup():
 *   #include "wifi_manager.h"
 *   wifi_manager_begin();   // blocks until connected or AP mode started
 *   // After this, either WiFi.status()==WL_CONNECTED
 *   // or the web server is running in the background.
 *
 * Call wifi_manager_loop() from your main loop or a FreeRTOS task.
 */

#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_log.h>

static const char *WM_TAG       = "wifi_mgr";
static const char *WM_AP_SSID   = "NINA-Display";
static const char *WM_NVS_NS    = "wifi_cfg";
static const char *WM_NVS_SSID  = "ssid";
static const char *WM_NVS_PASS  = "pass";
static const int   WM_CONN_TIMEOUT_MS = 15000;
static const int   WM_DNS_PORT   = 53;
static const int   WM_HTTP_PORT  = 80;

static WebServer  wm_server(WM_HTTP_PORT);
static DNSServer  wm_dns;
static bool       wm_ap_active  = false;
static bool       wm_connected  = false;
static const char *WM_NVS_NINA = "nina_ip";

static String wm_load_nina_ip() {
  Preferences prefs;
  prefs.begin(WM_NVS_NS, true);
  String ip = prefs.getString(WM_NVS_NINA, "");
  prefs.end();
  return ip;
}

static void wm_save_nina_ip(const String &ip) {
  Preferences prefs;
  prefs.begin(WM_NVS_NS, false);
  prefs.putString(WM_NVS_NINA, ip);
  prefs.end();
  ESP_LOGI(WM_TAG, "Saved NINA IP: %s", ip.c_str());
}

// ── NVS helpers ───────────────────────────────────────────────────────────────
static bool wm_load_credentials(String &ssid, String &pass) {
  Preferences prefs;
  prefs.begin(WM_NVS_NS, true);
  ssid = prefs.getString(WM_NVS_SSID, "");
  pass = prefs.getString(WM_NVS_PASS, "");
  prefs.end();
  return ssid.length() > 0;
}

static void wm_save_credentials(const String &ssid, const String &pass) {
  Preferences prefs;
  prefs.begin(WM_NVS_NS, false);
  prefs.putString(WM_NVS_SSID, ssid);
  prefs.putString(WM_NVS_PASS, pass);
  prefs.end();
  ESP_LOGI(WM_TAG, "Credentials saved for SSID: %s", ssid.c_str());
}

void wm_clear_credentials() {
  Preferences prefs;
  prefs.begin(WM_NVS_NS, false);
  prefs.clear();
  prefs.end();
  ESP_LOGI(WM_TAG, "Credentials cleared");
}

// ── HTML pages ────────────────────────────────────────────────────────────────
static String wm_html_header(const char *title) {
  return String(
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta charset='UTF-8'>"
    "<title>") + title + "</title>"
    "<style>"
    "body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,sans-serif;"
         "background:#0D1117;color:#E6EDF3;margin:0;padding:20px;}"
    "h1{color:#4472C4;font-size:1.4em;margin-bottom:4px;}"
    "h2{color:#8B949E;font-size:0.9em;font-weight:normal;margin-top:0;}"
    ".card{background:#161B22;border:1px solid #30363D;border-radius:8px;"
          "padding:16px;margin:12px 0;}"
    ".net{display:flex;align-items:center;justify-content:space-between;"
         "padding:10px 12px;margin:6px 0;background:#0D1117;"
         "border:1px solid #30363D;border-radius:6px;cursor:pointer;"
         "transition:border-color 0.2s;}"
    ".net:hover{border-color:#4472C4;}"
    ".net-name{font-size:1em;font-weight:500;}"
    ".net-rssi{font-size:0.8em;color:#8B949E;}"
    ".net-lock{color:#8B949E;font-size:0.85em;margin-left:6px;}"
    ".signal{display:inline-block;width:14px;text-align:center;margin-right:6px;}"
    "input[type=text],input[type=password]{"
         "width:100%;box-sizing:border-box;background:#0D1117;"
         "border:1px solid #30363D;border-radius:6px;"
         "color:#E6EDF3;padding:10px 12px;font-size:1em;margin:6px 0;}"
    "input:focus{outline:none;border-color:#4472C4;}"
    "button{width:100%;background:#4472C4;color:#fff;border:none;"
           "border-radius:6px;padding:12px;font-size:1em;"
           "cursor:pointer;margin-top:8px;}"
    "button:hover{background:#5885d5;}"
    ".ok{color:#3FB950;} .err{color:#F85149;}"
    ".hint{color:#8B949E;font-size:0.8em;margin-top:4px;}"
    "</style></head><body>"
    "<h1>&#128225; NINA Display</h1>"
    "<h2>WiFi Setup</h2>";
}

static String wm_signal_icon(int rssi) {
  if (rssi >= -55) return "&#9646;&#9646;&#9646;";  // strong
  if (rssi >= -70) return "&#9646;&#9646;&#9647;";  // medium
  return "&#9646;&#9647;&#9647;";                   // weak
}

static String wm_scan_page() {
  String html = wm_html_header("WiFi Setup");
  html += "<div class='card'><b>Available Networks</b><p class='hint'>Tap a network to connect</p>";

  int n = WiFi.scanNetworks(false, false, false, 200);
  if (n <= 0) {
    html += "<p class='err'>No networks found. <a href='/'>Retry</a></p>";
  } else {
    // Sort by RSSI (simple bubble sort on indices)
    int idx[n];
    for (int i = 0; i < n; i++) idx[i] = i;
    for (int i = 0; i < n-1; i++)
      for (int j = i+1; j < n; j++)
        if (WiFi.RSSI(idx[j]) > WiFi.RSSI(idx[i])) { int t=idx[i]; idx[i]=idx[j]; idx[j]=t; }

    for (int i = 0; i < n; i++) {
      int k = idx[i];
      String ssid_esc = WiFi.SSID(k);
      ssid_esc.replace("'", "\\'");
      bool enc = (WiFi.encryptionType(k) != WIFI_AUTH_OPEN);
      html += "<div class='net' onclick=\"selectNet('" + ssid_esc + "'," + (enc?"true":"false") + ")\">";
      html += "<span><span class='signal'>" + wm_signal_icon(WiFi.RSSI(k)) + "</span>";
      html += "<span class='net-name'>" + WiFi.SSID(k) + "</span>";
      if (enc) html += "<span class='net-lock'>&#128274;</span>";
      html += "</span>";
      html += "<span class='net-rssi'>" + String(WiFi.RSSI(k)) + " dBm</span>";
      html += "</div>";
    }
  }
  WiFi.scanDelete();

  html += "</div>"
    "<div class='card' id='connectForm' style='display:none'>"
    "<b id='netName'></b>"
    "<form action='/connect' method='POST'>"
    "<input type='hidden' id='ssidField' name='ssid' value=''>"
    "<input type='password' name='pass' id='passField' placeholder='Password' autocomplete='current-password'>"
    "<p class='hint' id='openHint' style='display:none'>Open network — no password needed</p>"
    "<button type='submit'>Connect</button>"
    "</form></div>"
    "<script>"
    "function selectNet(ssid,enc){"
    "  document.getElementById('netName').textContent=ssid;"
    "  document.getElementById('ssidField').value=ssid;"
    "  document.getElementById('connectForm').style.display='block';"
    "  document.getElementById('passField').style.display=enc?'block':'none';"
    "  document.getElementById('openHint').style.display=enc?'none':'block';"
    "  document.getElementById('passField').focus();"
    "  window.scrollTo(0,document.body.scrollHeight);"
    "}"
    "</script>"
    "</body></html>";
  return html;
} 

static String wm_connecting_page(const String &ssid) {
  return wm_html_header("Connecting...") +
    "<div class='card'>"
    "<p>Connecting to <b>" + ssid + "</b>&hellip;</p>"
    "<p class='hint'>The device will reboot when connected.<br>"
    "Reconnect your phone to your normal WiFi.</p>"
    "<p class='hint' id='t'>Waiting&hellip;</p>"
    "</div>"
    "<script>var s=15;var t=setInterval(function(){"
    "s--;document.getElementById('t').textContent='Rebooting in '+s+'s...';"
    "if(s<=0)clearInterval(t);},1000);</script>"
    "</body></html>";
}

// ── Web server route handlers ─────────────────────────────────────────────────
static void wm_handle_root() {
  Serial.println("[WM] root requested");
  wm_server.send(200, "text/html", wm_scan_page());
}

static void wm_handle_captive() {
  Serial.printf("[WM] captive redirect: %s\n", wm_server.uri().c_str());
  wm_server.sendHeader("Location", "http://192.168.4.1/", true);
  wm_server.send(302, "text/plain", "");
}

static void wm_handle_connect() {
  String ssid = wm_server.arg("ssid");
  String pass = wm_server.arg("pass");

  if (ssid.length() == 0) {
    wm_server.send(400, "text/html",
      wm_html_header("Error") + "<div class='card'><p class='err'>No SSID provided.</p>"
      "<a href='/'>Back</a></div></body></html>");
    return;
  }

  wm_server.send(200, "text/html", wm_connecting_page(ssid));
  delay(500);  // let the response flush

  wm_save_credentials(ssid, pass);

  delay(1000);
  ESP_LOGI(WM_TAG, "Rebooting with new credentials...");
  ESP.restart();
}

// Captive portal redirect — any unknown URL redirects to root


// ── Start AP + web server ─────────────────────────────────────────────────────
static void wm_start_ap() {
  ESP_LOGI(WM_TAG, "Starting AP: %s", WM_AP_SSID);
  WiFi.mode(WIFI_AP_STA);

  IPAddress apIP(192, 168, 4, 1), gw(192, 168, 4, 1), mask(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gw, mask);   // ← pin the AP IP before bringing it up
  WiFi.softAP(WM_AP_SSID);
  delay(200);

  // DNS — redirect everything to our IP (captive portal magic)
  wm_dns.start(WM_DNS_PORT, "*", apIP);

  // Web server routes
  wm_server.on("/",        HTTP_GET,  wm_handle_root);
  wm_server.on("/connect", HTTP_POST, wm_handle_connect);
  wm_server.onNotFound(wm_handle_captive);
  wm_server.begin();

  wm_ap_active = true;
  ESP_LOGI(WM_TAG, "AP started. Connect to '%s' and open http://192.168.4.1", WM_AP_SSID);
}

// ── Try connecting with given credentials ─────────────────────────────────────
static bool wm_try_connect(const String &ssid, const String &pass) {
  Serial.printf("[WiFi] Connecting to '%s'...\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > WM_CONN_TIMEOUT_MS) {
      Serial.printf("[WiFi] TIMEOUT after %dms (status=%d)\n",
        WM_CONN_TIMEOUT_MS, WiFi.status());
      WiFi.disconnect();
      return false;
    }
    delay(250);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] CONNECTED! IP=%s RSSI=%ddBm\n",
    WiFi.localIP().toString().c_str(), WiFi.RSSI());
  return true;
}

// ── Public API ────────────────────────────────────────────────────────────────

/*
 * Call once in setup().
 * Returns true  = connected to WiFi (normal operation)
 * Returns false = AP mode active, web server running
 */
bool wifi_manager_begin() {
  String ssid, pass;

  if (wm_load_credentials(ssid, pass)) {
    Serial.printf("[WiFi] Found saved credentials for '%s'\n", ssid.c_str());
    if (wm_try_connect(ssid, pass)) {
      wm_connected = true;
      return true;
    }
    Serial.println("[WiFi] Saved credentials FAILED — starting setup AP");
  } else {
    Serial.println("[WiFi] No saved credentials — starting setup AP");
  }

  wm_start_ap();
  Serial.println("[WiFi] AP 'NINA-Display' active. Connect phone, open http://192.168.4.1");
  return false;
}

/*
 * Call from loop() or a FreeRTOS task when in AP mode.
 * Handles DNS and HTTP requests.
 * No-op when already connected.
 */
void wifi_manager_loop() {
  if (!wm_ap_active) return;
  wm_dns.processNextRequest();
  wm_server.handleClient();
}

/*
 * Returns true if connected to WiFi.
 */
bool wifi_manager_connected() {
  return WiFi.status() == WL_CONNECTED;
}
