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
    u8g2.drawHLine(0, 36, 128);
    draw_cell(0,  11, "RPM",      b0);
    draw_cell(65, 11, "SPEED",    b1);
    draw_cell(0,  37, "LOAD",     b2);
    draw_cell(65, 37, "THROTTLE", b3);
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
    u8g2.drawHLine(0, 36, 128);
    draw_cell(0,  11, "COOLANT", b0);
    draw_cell(65, 11, "INTAKE",  b1);
    draw_cell(0,  37, "FUEL",    b2);
    draw_cell(65, 37, "BATTERY", b3);
}

// ---- frame 2: Connectivity -----------------------------------------------

static void draw_conn_row(uint8_t y, const char *label,
                           bool ok, const char *detail) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, y, label);
    if (ok) u8g2.drawDisc(25, y - 3, 3);
    else    u8g2.drawCircle(25, y - 3, 3);
    u8g2.drawStr(32, y, detail);
}

static void draw_frame_connectivity(bool bt_ok, bool mqtt_ok) {
    draw_title("CONNECTIVITY");
    bool wifi_ok = (WiFi.status() == WL_CONNECTED);
    String ssid = wifi_ok ? WiFi.SSID() : String("---");
    String ip   = wifi_ok ? WiFi.localIP().toString() : String("---");
    draw_conn_row(21, "BT",   bt_ok,   bt_ok   ? "Connected" : "Scanning");
    draw_conn_row(34, "WIFI", wifi_ok, ssid.c_str());
    draw_conn_row(47, "MQTT", mqtt_ok, mqtt_ok ? "Connected" : "---");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 61, "IP");
    u8g2.drawStr(29, 61, ip.c_str());
}

// ---- redraw --------------------------------------------------------------

static void redraw(const OBDData &data, bool bt, bool mqtt) {
    u8g2.clearBuffer();
    switch (s_frame) {
        case 0: draw_frame_engine(data);           break;
        case 1: draw_frame_sensors(data);          break;
        case 2: draw_frame_connectivity(bt, mqtt); break;
    }
    u8g2.sendBuffer();
}

// ---- public API ----------------------------------------------------------

void display_init() {
    s_interval_ms = storage_get_display_interval();
    u8g2.begin();
    u8g2.clearDisplay();
    s_last_switch = millis() - s_interval_ms;  // fire on first loop() call
    Serial.printf("[DISP] Init OK — interval=%ums\n", s_interval_ms);
}

void display_loop(const OBDData &data, bool bt_connected, bool mqtt_connected) {
    uint32_t now = millis();
    if (now - s_last_switch >= s_interval_ms) {
        s_last_switch = now;
        redraw(data, bt_connected, mqtt_connected);
        s_frame = (s_frame + 1) % 3;
    }
}

void display_set_interval(uint32_t ms) {
    if (ms == 0) return;
    s_interval_ms = ms;
    storage_set_display_interval(ms);
}

uint32_t display_get_interval() {
    return s_interval_ms;
}
