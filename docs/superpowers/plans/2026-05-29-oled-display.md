# OLED Display Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a 1.30" SH1106 I2C OLED (128×64) that cycles through three frames — Engine, Sensors, Connectivity — at a web-configurable interval.

**Architecture:** New `display.cpp/h` module using U8g2. `display_init()` called once in setup, `display_loop()` called every loop iteration receiving OBD data and connectivity flags. Frame interval persisted in NVS, exposed via a new settings card in the web UI.

**Tech Stack:** U8g2 (olikraus/U8g2), ESP32 HW I2C (SDA=21, SCL=22), Preferences NVS, existing WebServer.

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `platformio.ini` | Modify | Add U8g2 lib_dep |
| `src/config.h` | Modify | DISPLAY_SDA, DISPLAY_SCL, DISPLAY_DEFAULT_INTERVAL_MS |
| `src/storage.h` | Modify | Declare `storage_get/set_display_interval()` |
| `src/storage.cpp` | Modify | Implement NVS helpers for disp_ms |
| `src/display.h` | Create | Public module interface |
| `src/display.cpp` | Create | U8g2 init, frame drawing, cycling logic |
| `src/main.cpp` | Modify | Call `display_init()` and `display_loop()` |
| `src/web_server.cpp` | Modify | Add `display_interval_ms` to status JSON; add POST `/api/display_interval` handler and new settings card HTML |
| `data/index.html` | Modify | Keep in sync with embedded HTML in web_server.cpp |

---

## Task 1: Add U8g2 dependency and display config constants

**Files:**
- Modify: `platformio.ini`
- Modify: `src/config.h`

- [ ] **Step 1: Add U8g2 to lib_deps in `platformio.ini`**

  Add one line to the `lib_deps` block:

  ```ini
  lib_deps =
      knolleary/PubSubClient@^2.8
      powerbroker2/ELMduino@^3.4.0
      bblanchon/ArduinoJson@^7.0.0
      olikraus/U8g2@^2.35.0
  ```

- [ ] **Step 2: Add display pin and interval constants to `src/config.h`**

  Append after the `WEB_SERVER_PORT` line:

  ```cpp
  // --- OLED Display ---------------------------------------------------------
  #define DISPLAY_SDA                  21
  #define DISPLAY_SCL                  22
  #define DISPLAY_DEFAULT_INTERVAL_MS  2000UL
  ```

- [ ] **Step 3: Verify build resolves U8g2**

  Run: `pio pkg install --environment esp32dev`

  Expected: U8g2 downloads without error.

---

## Task 2: NVS storage helpers for display interval

**Files:**
- Modify: `src/storage.h`
- Modify: `src/storage.cpp`

- [ ] **Step 1: Add declarations to `src/storage.h`**

  After the `storage_clear_mac()` declaration:

  ```cpp
  // OLED frame rotation interval (ms), default DISPLAY_DEFAULT_INTERVAL_MS.
  uint32_t storage_get_display_interval();
  void     storage_set_display_interval(uint32_t ms);
  ```

- [ ] **Step 2: Implement helpers in `src/storage.cpp`**

  Add `#include "config.h"` is already present. Append after `storage_clear_mac()`:

  ```cpp
  uint32_t storage_get_display_interval() {
      return prefs.getUInt("disp_ms", DISPLAY_DEFAULT_INTERVAL_MS);
  }

  void storage_set_display_interval(uint32_t ms) {
      prefs.putUInt("disp_ms", ms);
  }
  ```

- [ ] **Step 3: Verify build**

  Run: `pio run --environment esp32dev`

  Expected: Compiles without errors.

---

## Task 3: Create `display.h`

**Files:**
- Create: `src/display.h`

- [ ] **Step 1: Write `src/display.h`**

  ```cpp
  #pragma once
  #include <Arduino.h>
  #include "bt_obd.h"

  void     display_init();
  void     display_loop(const OBDData &data, bool bt_connected, bool mqtt_connected);
  void     display_set_interval(uint32_t ms);
  uint32_t display_get_interval();
  ```

---

## Task 4: Create `display.cpp` — init and interval management

**Files:**
- Create: `src/display.cpp`

- [ ] **Step 1: Write `src/display.cpp` scaffold**

  ```cpp
  #include "display.h"
  #include "storage.h"
  #include "config.h"
  #include <WiFi.h>
  #include <U8g2lib.h>
  #include <Wire.h>
  #include <math.h>

  static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
      U8G2_R0, U8X8_PIN_NONE, DISPLAY_SCL, DISPLAY_SDA);

  static uint8_t  s_frame       = 0;
  static uint32_t s_interval_ms = DISPLAY_DEFAULT_INTERVAL_MS;
  static uint32_t s_last_switch = 0;

  // ---- value formatting ----------------------------------------------------

  static void fmt_int(char *buf, size_t n, float v, const char *unit) {
      if (v < 0) { snprintf(buf, n, "--"); return; }
      snprintf(buf, n, "%d%s", (int)roundf(v), unit);
  }

  static void fmt_f1(char *buf, size_t n, float v, const char *unit) {
      if (v < 0) { snprintf(buf, n, "--"); return; }
      snprintf(buf, n, "%.1f%s", v, unit);
  }

  // ---- shared draw helpers -------------------------------------------------

  static void draw_title(const char *title) {
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.drawStr(0, 8, title);
      u8g2.drawHLine(0, 10, 128);
  }

  static void draw_cell(uint8_t x, uint8_t top,
                        const char *label, const char *value) {
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.drawStr(x, top + 8, label);
      u8g2.setFont(u8g2_font_logisoso16_tf);
      u8g2.drawStr(x, top + 26, value);
  }

  // ---- frame 0: Engine -----------------------------------------------------

  static void draw_frame_engine(const OBDData &d) {
      char b0[12], b1[12], b2[12], b3[12];
      fmt_int(b0, 12, d.rpm,           "");
      fmt_int(b1, 12, d.speedKph,      "");
      fmt_int(b2, 12, d.engineLoadPct, "%");
      fmt_int(b3, 12, d.throttlePct,   "%");

      draw_title("ENGINE");
      u8g2.drawVLine(63, 10, 54);
      u8g2.drawHLine(0, 38, 128);
      draw_cell(0,  11, "RPM",      b0);
      draw_cell(65, 11, "SPEED",    b1);
      draw_cell(0,  39, "LOAD",     b2);
      draw_cell(65, 39, "THROTTLE", b3);
  }

  // ---- frame 1: Sensors ----------------------------------------------------

  static void draw_frame_sensors(const OBDData &d) {
      char b0[12], b1[12], b2[12], b3[12];
      fmt_int(b0, 12, d.coolantTempC,  "\xb0\x43");  // °C
      fmt_int(b1, 12, d.intakeTempC,   "\xb0\x43");
      fmt_int(b2, 12, d.fuelLevelPct,  "%");
      fmt_f1 (b3, 12, d.batteryV,      "V");

      draw_title("SENSORS");
      u8g2.drawVLine(63, 10, 54);
      u8g2.drawHLine(0, 38, 128);
      draw_cell(0,  11, "COOLANT", b0);
      draw_cell(65, 11, "INTAKE",  b1);
      draw_cell(0,  39, "FUEL",    b2);
      draw_cell(65, 39, "BATTERY", b3);
  }

  // ---- frame 2: Connectivity -----------------------------------------------

  static void draw_conn_row(uint8_t y, const char *label,
                             bool ok, const char *detail) {
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.drawStr(0, y, label);
      if (ok) u8g2.drawDisc(22, y - 3, 3);
      else    u8g2.drawCircle(22, y - 3, 3);
      u8g2.drawStr(29, y, detail);
  }

  static void draw_frame_connectivity(bool bt_ok, bool mqtt_ok) {
      draw_title("CONNECTIVITY");
      bool wifi_ok = (WiFi.status() == WL_CONNECTED);
      draw_conn_row(21, "BT",   bt_ok,   bt_ok   ? "Connected"                         : "Scanning");
      draw_conn_row(34, "WIFI", wifi_ok, wifi_ok ? WiFi.SSID().c_str()                 : "---");
      draw_conn_row(47, "MQTT", mqtt_ok, mqtt_ok ? "Connected"                         : "---");
      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.drawStr(0, 61, "IP");
      u8g2.drawStr(29, 61, wifi_ok ? WiFi.localIP().toString().c_str() : "---");
  }

  // ---- redraw --------------------------------------------------------------

  static void redraw(const OBDData &data, bool bt, bool mqtt) {
      u8g2.clearBuffer();
      switch (s_frame) {
          case 0: draw_frame_engine(data);          break;
          case 1: draw_frame_sensors(data);         break;
          case 2: draw_frame_connectivity(bt, mqtt); break;
      }
      u8g2.sendBuffer();
  }

  // ---- public API ----------------------------------------------------------

  void display_init() {
      s_interval_ms = storage_get_display_interval();
      u8g2.begin();
      u8g2.clearDisplay();
      Serial.printf("[DISP] Init OK — interval=%ums\n", s_interval_ms);
  }

  void display_loop(const OBDData &data, bool bt_connected, bool mqtt_connected) {
      if (millis() - s_last_switch >= s_interval_ms) {
          s_frame = (s_frame + 1) % 3;
          s_last_switch = millis();
          redraw(data, bt_connected, mqtt_connected);
      }
  }

  void display_set_interval(uint32_t ms) {
      if (ms == 0) return;
      s_interval_ms = ms;
  }

  uint32_t display_get_interval() {
      return s_interval_ms;
  }
  ```

- [ ] **Step 2: Verify build**

  Run: `pio run --environment esp32dev`

  Expected: Compiles without errors. (If font names are wrong, check the U8g2 font reference at https://github.com/olikraus/u8g2/wiki/fntlistall — substitutes: `u8g2_font_6x10_tf` for labels, `u8g2_font_profont22_tr` for values.)

---

## Task 5: Wire display into `main.cpp`

**Files:**
- Modify: `src/main.cpp`

- [ ] **Step 1: Add `#include "display.h"` at the top of `src/main.cpp`**

  After `#include "web_server.h"`:

  ```cpp
  #include "display.h"
  ```

- [ ] **Step 2: Call `display_init()` in `setup()`**

  Add as step 6 (after `web_server_init()`):

  ```cpp
      // 6. OLED display
      display_init();
  ```

- [ ] **Step 3: Call `display_loop()` in `loop()`**

  After the `web_server_handle_client()` call, before `delay(10)`:

  ```cpp
      // Update OLED
      display_loop(bt_obd_get_data(), bt_obd_is_connected(), mqtt_is_connected());
  ```

- [ ] **Step 4: Verify build**

  Run: `pio run --environment esp32dev`

  Expected: Compiles without errors.

- [ ] **Step 5: Flash and smoke-test**

  Run: `pio run --environment esp32dev --target upload`

  Expected: Display lights up, shows ENGINE frame, advances to SENSORS after ~2 s, then CONNECTIVITY.

---

## Task 6: Add display interval setting to web UI

**Files:**
- Modify: `src/web_server.cpp`
- Modify: `data/index.html`

This task has two parts: (A) backend — add the API endpoint and update the status response, and (B) frontend — add the settings card to the embedded HTML and to `data/index.html`.

### Part A — Backend

- [ ] **Step 1: Add `#include "display.h"` to `src/web_server.cpp`**

  After `#include "storage.h"`:

  ```cpp
  #include "display.h"
  ```

- [ ] **Step 2: Add `display_interval_ms` field to `handle_status()`**

  In the `handle_status()` function, after the line `doc["saved_mac"] = bt_get_target_mac();`:

  ```cpp
      doc["display_interval_ms"] = display_get_interval();
  ```

- [ ] **Step 3: Add `handle_display_interval()` handler**

  Add this function after `handle_reconnect()`:

  ```cpp
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
      storage_set_display_interval(ms);
      server.send(200, "application/json", "{\"ok\":true,\"ms\":" + String(ms) + "}");
  }
  ```

- [ ] **Step 4: Register the route in `web_server_init()`**

  After `server.on("/api/reconnect", HTTP_POST, handle_reconnect);`:

  ```cpp
      server.on("/api/display_interval", HTTP_POST, handle_display_interval);
  ```

### Part B — Frontend (HTML)

The HTML lives in two places: embedded in `web_server.cpp` as `PAGE_HTML`, and in `data/index.html`. Both must receive the same change. The new card goes just before the `<footer>` tag.

- [ ] **Step 5: Add Display Settings card to `data/index.html`**

  Find this line in `data/index.html`:
  ```html
  <footer>ESP32 ZUS OBD-II Bridge &bull; MQTT: 192.168.1.55</footer>
  ```

  Insert the following block immediately before it:

  ```html
  <!-- Display settings card -->
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
  ```

- [ ] **Step 6: Add JS functions `loadDisplayInterval()` and `saveDisplayInterval()` to `data/index.html`**

  In the `<script>` block, after the `fetchStatus()` function, add:

  ```js
  function loadDisplayInterval(ms) {
    const el = document.getElementById('inp-disp-ms');
    if (el && ms) el.value = ms;
  }
  ```

  Inside `fetchStatus()`, after `setText('val-saved-mac', d.saved_mac || 'none');`, add:

  ```js
    loadDisplayInterval(d.display_interval_ms);
  ```

  After the `reconnect()` function, add:

  ```js
  function saveDisplayInterval() {
    const ms = parseInt(document.getElementById('inp-disp-ms').value, 10);
    const st = document.getElementById('disp-save-status');
    if (!ms || ms < 500 || ms > 60000) { st.textContent = 'Must be 500–60000 ms'; return; }
    fetch('/api/display_interval', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ms })
    }).then(r => r.json()).then(d => {
      st.textContent = d.ok ? 'Saved!' : (d.error || 'Error');
      setTimeout(() => { st.textContent = ''; }, 2000);
    }).catch(() => { st.textContent = 'Error'; });
  }
  ```

- [ ] **Step 7: Apply the identical HTML and JS changes to the embedded `PAGE_HTML` in `src/web_server.cpp`**

  The `PAGE_HTML` raw string in `web_server.cpp` is identical to `data/index.html` except it uses string concatenation instead of template literals in the JS. Apply the exact same HTML block (Step 5) and JS (Step 6) changes to the `PAGE_HTML` raw string. The JS additions use only single quotes and standard concatenation so they work unchanged in both files.

- [ ] **Step 8: Verify build**

  Run: `pio run --environment esp32dev`

  Expected: Compiles without errors.

- [ ] **Step 9: Flash and end-to-end test**

  Run: `pio run --environment esp32dev --target upload`

  Expected:
  - Display cycles through all 3 frames at 2 s default.
  - Web UI at `http://<device-ip>/` shows "Display Settings" card with interval field pre-populated.
  - Changing the value and clicking Save updates the cycling speed immediately and persists across reboot.

---

## Self-Review Notes

- Font `u8g2_font_logisoso16_tf` may need substitution if it causes flash overflow (try `u8g2_font_6x13_tf` for a smaller alternative). Check with `pio run` — if it links, it fits.
- The `\xb0\x43` sequence for °C assumes U8g2 full-charset font (`_tf` suffix). If the degree glyph renders as a box, fall back to just `"C"` or change the font to `u8g2_font_6x10_tf`.
- `WiFi.SSID()` returns a `String`; `.c_str()` is valid for the lifetime of the expression. Safe here since `drawStr` copies the string internally.
- The PROGMEM HTML and `data/index.html` will drift again if future changes touch only one. Consider consolidating to one source in the future (not in scope here).
