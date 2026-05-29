#include "web_server.h"
#include "config.h"
#include "bt_obd.h"
#include "mqtt_handler.h"
#include "storage.h"
#include "display.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

static WebServer server(WEB_SERVER_PORT);

// ============================================================================
//  Embedded HTML page
// ============================================================================

static const char PAGE_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0" />
<title>ZUS OBD Bridge</title>
<style>
  :root {
    --bg: #0f172a; --surface: #1e293b; --border: #334155;
    --text: #e2e8f0; --muted: #94a3b8; --accent: #38bdf8;
    --green: #4ade80; --red: #f87171; --yellow: #facc15;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: system-ui, sans-serif; background: var(--bg);
         color: var(--text); min-height: 100vh; padding: 1.5rem; }
  h1 { font-size: 1.4rem; font-weight: 700; color: var(--accent);
       margin-bottom: 1.5rem; display: flex; align-items: center; gap: .5rem; }
  h2 { font-size: 1rem; font-weight: 600; color: var(--muted);
       text-transform: uppercase; letter-spacing: .05em; margin-bottom: .75rem; }
  .card { background: var(--surface); border: 1px solid var(--border);
          border-radius: .75rem; padding: 1.25rem; margin-bottom: 1rem; }
  .pill { display: inline-flex; align-items: center; gap: .35rem;
          font-size: .75rem; font-weight: 600; padding: .2rem .65rem;
          border-radius: 9999px; }
  .pill.ok  { background: #14532d; color: var(--green); }
  .pill.err { background: #450a0a; color: var(--red); }
  .pill.warn{ background: #422006; color: var(--yellow); }
  .dot { width: 7px; height: 7px; border-radius: 50%; background: currentColor; }
  .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(130px, 1fr)); gap: .75rem; }
  .metric { background: var(--bg); border: 1px solid var(--border);
            border-radius: .5rem; padding: .75rem; text-align: center; }
  .metric .val { font-size: 1.5rem; font-weight: 700; color: var(--accent); }
  .metric .lbl { font-size: .7rem; color: var(--muted); margin-top: .2rem; }
  button { cursor: pointer; border: none; border-radius: .5rem;
           font-size: .85rem; font-weight: 600; padding: .5rem 1rem;
           transition: opacity .15s; }
  button:hover { opacity: .85; }
  button:disabled { opacity: .4; cursor: not-allowed; }
  .btn-primary { background: var(--accent); color: #0f172a; }
  .btn-danger  { background: var(--red);    color: #0f172a; }
  .btn-ghost   { background: var(--border); color: var(--text); }
  .device-row  { display: flex; align-items: center; justify-content: space-between;
                 padding: .6rem .75rem; border-radius: .5rem; margin-bottom: .4rem;
                 background: var(--bg); border: 1px solid var(--border); }
  .device-name { font-weight: 600; font-size: .9rem; }
  .device-mac  { font-size: .72rem; color: var(--muted); font-family: monospace; }
  .device-rssi { font-size: .72rem; color: var(--muted); margin-right: .75rem; }
  .bar { height: 4px; border-radius: 2px; background: var(--border); margin-top: .5rem; }
  .bar-fill { height: 100%; border-radius: 2px; background: var(--accent); transition: width .4s; }
  #scan-progress { display: none; }
  #scan-progress.active { display: block; }
  .row { display: flex; align-items: center; gap: .75rem; flex-wrap: wrap; }
  .status-row { display: flex; flex-direction: column; gap: .4rem; margin-bottom: .75rem; }
  .status-item { display: flex; align-items: center; justify-content: space-between; }
  .status-label { font-size: .85rem; color: var(--muted); }
  .mac-display { font-family: monospace; font-size: .85rem;
                 color: var(--accent); background: var(--bg);
                 padding: .2rem .5rem; border-radius: .3rem; }
  footer { margin-top: 2rem; text-align: center; font-size: .72rem; color: var(--border); }
</style>
</head>
<body>

<h1>&#x1F697; ZUS OBD Bridge</h1>

<div class="card" id="status-card">
  <h2>Status</h2>
  <div class="status-row">
    <div class="status-item">
      <span class="status-label">Bluetooth</span>
      <span class="pill err" id="pill-bt"><span class="dot"></span>Disconnected</span>
    </div>
    <div class="status-item">
      <span class="status-label">MQTT</span>
      <span class="pill err" id="pill-mqtt"><span class="dot"></span>Disconnected</span>
    </div>
    <div class="status-item">
      <span class="status-label">IP Address</span>
      <span class="mac-display" id="val-ip">...</span>
    </div>
    <div class="status-item">
      <span class="status-label">Target Device</span>
      <span class="mac-display" id="val-saved-mac">none</span>
    </div>
  </div>
  <div class="row">
    <button class="btn-ghost" onclick="reconnect()">&#x21BB; Reconnect</button>
    <button class="btn-danger" id="btn-forget" onclick="forgetDevice()" style="display:none">Forget Device</button>
  </div>
</div>

<div class="card" id="obd-card" style="display:none">
  <h2>Live OBD Data <span id="obd-age" style="font-size:.7rem;font-weight:400;color:var(--muted)"></span></h2>
  <div class="grid">
    <div class="metric"><div class="val" id="v-rpm">--</div><div class="lbl">RPM</div></div>
    <div class="metric"><div class="val" id="v-speed">--</div><div class="lbl">Speed (km/h)</div></div>
    <div class="metric"><div class="val" id="v-coolant">--</div><div class="lbl">Coolant (&#xB0;C)</div></div>
    <div class="metric"><div class="val" id="v-load">--</div><div class="lbl">Engine Load (%)</div></div>
    <div class="metric"><div class="val" id="v-fuel">--</div><div class="lbl">Fuel (%)</div></div>
    <div class="metric"><div class="val" id="v-throttle">--</div><div class="lbl">Throttle (%)</div></div>
    <div class="metric"><div class="val" id="v-intake">--</div><div class="lbl">Intake (&#xB0;C)</div></div>
    <div class="metric"><div class="val" id="v-batt">--</div><div class="lbl">Battery (V)</div></div>
  </div>
</div>

<div class="card">
  <h2>Pair ZUS Device</h2>
  <p style="font-size:.85rem;color:var(--muted);margin-bottom:1rem">
    Click <strong>Scan</strong> to discover nearby Bluetooth devices, then select your ZUS OBD-II adapter.
    The MAC address is saved to flash and used on every reconnect.
  </p>
  <div class="row" style="margin-bottom:1rem">
    <button class="btn-primary" id="btn-scan" onclick="startScan()">&#x1F50D; Scan for Devices</button>
    <span id="scan-status" style="font-size:.8rem;color:var(--muted)"></span>
  </div>

  <div id="scan-progress">
    <div class="bar"><div class="bar-fill" id="scan-bar" style="width:0%"></div></div>
    <p style="font-size:.75rem;color:var(--muted);margin-top:.35rem">
      Scanning... <span id="scan-countdown"></span>s remaining
    </p>
  </div>

  <div id="device-list" style="margin-top:1rem"></div>
</div>

<div class="card">
  <h2>Display Settings</h2>
  <div class="status-item" style="margin-bottom:.75rem">
    <span class="status-label">Frame interval (ms)</span>
    <input id="inp-disp-ms" type="number" min="500" max="60000" step="100"
           value="2000"
           style="width:90px;padding:.3rem .5rem;border-radius:.4rem;
                  border:1px solid var(--border);background:var(--bg);
                  color:var(--text);font-size:.85rem;text-align:right">
  </div>
  <div class="row">
    <button class="btn-primary" onclick="saveDisplayInterval()">Save</button>
    <span id="disp-save-status" style="font-size:.8rem;color:var(--muted)"></span>
  </div>
</div>

<footer>ESP32 ZUS OBD-II Bridge</footer>

<script>
const sd = 12;
let scanTimer = null;
let scanElapsed = 0;
let pollTimer = null;

function startStatusPoll() {
  if (pollTimer) clearInterval(pollTimer);
  pollTimer = setInterval(fetchStatus, 4000);
  fetchStatus();
}

function fetchStatus() {
  fetch('/api/status').then(r => r.json()).then(d => {
    updatePill('pill-bt',   d.bt_connected,   'Connected',    'Disconnected');
    updatePill('pill-mqtt', d.mqtt_connected, 'Connected',    'Disconnected');
    setText('val-ip', d.ip || '...');
    setText('val-saved-mac', d.saved_mac || 'none');
    loadDisplayInterval(d.display_interval_ms);

    const hasMac = d.saved_mac && d.saved_mac.length === 17;
    document.getElementById('btn-forget').style.display = hasMac ? '' : 'none';

    const obd = d.obd;
    const obdCard = document.getElementById('obd-card');
    if (d.bt_connected && obd && obd.age_ms < 30000) {
      obdCard.style.display = '';
      setVal('v-rpm',     obd.rpm,         0);
      setVal('v-speed',   obd.speed_kph,   0);
      setVal('v-coolant', obd.coolant_c,   1);
      setVal('v-load',    obd.engine_load, 1);
      setVal('v-fuel',    obd.fuel_pct,    1);
      setVal('v-throttle',obd.throttle,    1);
      setVal('v-intake',  obd.intake_c,    1);
      setVal('v-batt',    obd.battery_v,   2);
      const age = Math.round(obd.age_ms / 1000);
      setText('obd-age', '(' + age + 's ago)');
    } else {
      obdCard.style.display = 'none';
    }
  }).catch(() => {});
}

function loadDisplayInterval(ms) {
  var el = document.getElementById('inp-disp-ms');
  if (el && ms != null) el.value = ms;
}

function updatePill(id, ok, okLabel, errLabel) {
  const el = document.getElementById(id);
  el.className = 'pill ' + (ok ? 'ok' : 'err');
  el.innerHTML = '<span class="dot"></span>' + (ok ? okLabel : errLabel);
}

function setText(id, val) { document.getElementById(id).textContent = val; }

function setVal(id, val, dp) {
  document.getElementById(id).textContent = (val != null && val >= -999) ? val.toFixed(dp) : '--';
}

function startScan() {
  document.getElementById('btn-scan').disabled = true;
  document.getElementById('scan-status').textContent = 'Starting...';
  document.getElementById('device-list').innerHTML = '';
  document.getElementById('scan-progress').classList.add('active');
  scanElapsed = 0;

  fetch('/api/scan/start', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (!d.ok) {
        document.getElementById('scan-status').textContent = d.error || 'Failed';
        endScan(); return;
      }
      document.getElementById('scan-status').textContent = '';
      updateCountdown();
      scanTimer = setInterval(updateCountdown, 1000);
      setTimeout(fetchResults, (sd + 2) * 1000);
    })
    .catch(() => { document.getElementById('scan-status').textContent = 'Error'; endScan(); });
}

function updateCountdown() {
  scanElapsed++;
  const remaining = Math.max(0, sd - scanElapsed);
  const pct = Math.min(100, (scanElapsed / sd) * 100);
  document.getElementById('scan-bar').style.width = pct + '%';
  document.getElementById('scan-countdown').textContent = remaining;
}

function fetchResults() {
  fetch('/api/scan/results').then(r => r.json()).then(d => {
    renderDevices(d.devices || []);
    if (!d.scanning) endScan();
    else setTimeout(fetchResults, 1500);
  }).catch(() => endScan());
}

function endScan() {
  if (scanTimer) { clearInterval(scanTimer); scanTimer = null; }
  document.getElementById('btn-scan').disabled = false;
  document.getElementById('scan-progress').classList.remove('active');
  document.getElementById('scan-status').textContent = '';
}

function renderDevices(devices) {
  const list = document.getElementById('device-list');
  if (!devices.length) {
    list.innerHTML = '<p style="font-size:.85rem;color:var(--muted)">No devices found yet...</p>';
    return;
  }
  devices.sort((a, b) => (b.rssi || -999) - (a.rssi || -999));
  list.innerHTML = devices.map(d =>
    '<div class="device-row">' +
    '<div><div class="device-name">' + escHtml(d.name) + '</div>' +
    '<div class="device-mac">' + escHtml(d.address) + '</div></div>' +
    '<div style="display:flex;align-items:center">' +
    '<span class="device-rssi">' + (d.rssi ? d.rssi + ' dBm' : '') + '</span>' +
    '<button class="btn-primary" style="font-size:.75rem;padding:.3rem .65rem" ' +
    'onclick="pairDevice(\'' + escHtml(d.address) + '\',\'' + escHtml(d.name) + '\')">Set as ZUS</button>' +
    '</div></div>'
  ).join('');
}

function pairDevice(mac, name) {
  if (!confirm('Set "' + name + '" (' + mac + ') as your ZUS OBD device?')) return;
  fetch('/api/pair', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mac: mac })
  }).then(r => r.json()).then(d => {
    if (d.ok) {
      alert('Saved! The bridge will now connect to ' + mac + '.');
      fetchStatus();
    } else {
      alert('Error: ' + (d.error || 'unknown'));
    }
  });
}

function forgetDevice() {
  if (!confirm('Forget the saved device? The bridge will stop reconnecting.')) return;
  fetch('/api/forget', { method: 'POST' }).then(() => fetchStatus());
}

function reconnect() {
  fetch('/api/reconnect', { method: 'POST' }).then(() => {
    document.getElementById('scan-status').textContent = 'Reconnecting...';
    setTimeout(fetchStatus, 3000);
  });
}

function saveDisplayInterval() {
  var ms = parseInt(document.getElementById('inp-disp-ms').value, 10);
  var st = document.getElementById('disp-save-status');
  if (!ms || ms < 500 || ms > 60000) { st.textContent = 'Must be 500-60000 ms'; return; }
  fetch('/api/display_interval', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ ms: ms })
  }).then(function(r) { return r.json(); }).then(function(d) {
    st.textContent = d.ok ? 'Saved!' : (d.error || 'Error');
    setTimeout(function() { st.textContent = ''; }, 2000);
  }).catch(function() { st.textContent = 'Error'; });
}

function escHtml(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

startStatusPoll();
</script>
</body>
</html>
)rawhtml";

// ============================================================================
//  REST endpoint helpers
// ============================================================================

static void handle_root() {
    server.sendHeader("Connection", "close");
    server.send_P(200, "text/html", PAGE_HTML);
}

static void handle_status() {
    JsonDocument doc;

    doc["ip"]             = WiFi.localIP().toString();
    doc["bt_connected"]   = bt_obd_is_connected();
    doc["mqtt_connected"] = mqtt_is_connected();
    doc["saved_mac"]      = bt_get_target_mac();
    doc["display_interval_ms"] = display_get_interval();

    OBDData d = bt_obd_get_data();
    JsonObject obd    = doc["obd"].to<JsonObject>();
    obd["rpm"]         = d.rpm;
    obd["speed_kph"]   = d.speedKph;
    obd["coolant_c"]   = d.coolantTempC;
    obd["engine_load"] = d.engineLoadPct;
    obd["fuel_pct"]    = d.fuelLevelPct;
    obd["throttle"]    = d.throttlePct;
    obd["intake_c"]    = d.intakeTempC;
    obd["battery_v"]   = d.batteryV;
    obd["running"]     = d.engineRunning;
    obd["age_ms"]      = (d.lastUpdateMs > 0) ? (long)(millis() - d.lastUpdateMs) : -1;

    String body;
    serializeJson(doc, body);
    server.send(200, "application/json", body);
}

static void handle_scan_start() {
    if (bt_scan_in_progress()) {
        server.send(409, "application/json", "{\"error\":\"scan already running\"}");
        return;
    }
    if (bt_scan_start()) {
        server.send(200, "application/json",
            "{\"ok\":true,\"duration_sec\":" + String(BT_SCAN_DURATION_SEC) + "}");
    } else {
        server.send(500, "application/json", "{\"error\":\"failed to start scan\"}");
    }
}

static void handle_scan_results() {
    JsonDocument doc;
    doc["scanning"] = bt_scan_in_progress();
    JsonArray devices = doc["devices"].to<JsonArray>();
    for (auto &r : bt_scan_get_results()) {
        JsonObject d = devices.add<JsonObject>();
        d["name"]    = r.name.length() ? r.name : "(unknown)";
        d["address"] = r.address;
        d["rssi"]    = r.rssi;
    }
    String body;
    serializeJson(doc, body);
    server.send(200, "application/json", body);
}

static void handle_pair() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"no body\"}");
        return;
    }
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err || !doc["mac"].is<const char *>()) {
        server.send(400, "application/json", "{\"error\":\"invalid JSON\"}");
        return;
    }
    String mac = doc["mac"].as<String>();
    mac.toUpperCase();
    if (mac.length() != 17) {
        server.send(400, "application/json", "{\"error\":\"bad MAC format\"}");
        return;
    }
    bt_set_target_mac(mac);
    server.send(200, "application/json", "{\"ok\":true,\"mac\":\"" + mac + "\"}");
}

static void handle_forget() {
    storage_clear_mac();
    bt_obd_force_reconnect();
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handle_reconnect() {
    bt_obd_force_reconnect();
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handle_display_interval() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"no body\"}");
        return;
    }
    JsonDocument doc;
    if (deserializeJson(doc, server.arg("plain")) || !doc["ms"].is<uint32_t>()) {
        server.send(400, "application/json", "{\"error\":\"invalid JSON\"}");
        return;
    }
    uint32_t ms = doc["ms"].as<uint32_t>();
    if (ms < 500 || ms > 60000) {
        server.send(400, "application/json", "{\"error\":\"ms must be 500-60000\"}");
        return;
    }
    display_set_interval(ms);
    server.send(200, "application/json", "{\"ok\":true,\"ms\":" + String(ms) + "}");
}

// ============================================================================
//  Init / tick
// ============================================================================

void web_server_init() {
    server.on("/",                 HTTP_GET,  handle_root);
    server.on("/api/status",       HTTP_GET,  handle_status);
    server.on("/api/scan/start",   HTTP_POST, handle_scan_start);
    server.on("/api/scan/results", HTTP_GET,  handle_scan_results);
    server.on("/api/pair",         HTTP_POST, handle_pair);
    server.on("/api/forget",       HTTP_POST, handle_forget);
    server.on("/api/reconnect",    HTTP_POST, handle_reconnect);
    server.on("/api/display_interval", HTTP_POST, handle_display_interval);

    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });

    server.begin();
    Serial.printf("[WEB] Server started at http://%s/\n",
        WiFi.localIP().toString().c_str());
}

void web_server_handle_client() {
    server.handleClient();
}
