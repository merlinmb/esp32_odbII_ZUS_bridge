#include "display.h"
#include "storage.h"
#include "config.h"
#include <WiFi.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, U8X8_PIN_NONE, DISPLAY_SCL, DISPLAY_SDA);

static uint8_t s_frame = 0;
static uint32_t s_interval_ms = DISPLAY_DEFAULT_INTERVAL_MS;
static uint32_t s_last_switch = 0;
static bool s_boot_complete = false;

// ---- value formatting ----------------------------------------------------

static void fmt_int(char *buf, size_t n, float v, const char *unit)
{
    if (isnan(v))
    {
        snprintf(buf, n, "--");
        return;
    }
    snprintf(buf, n, "%d%s", (int)roundf(v), unit);
}

static void fmt_f1(char *buf, size_t n, float v, const char *unit)
{
    if (isnan(v))
    {
        snprintf(buf, n, "--");
        return;
    }
    snprintf(buf, n, "%.1f%s", v, unit);
}

// ---- shared draw helpers -------------------------------------------------

static void draw_title(const char *title)
{
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 8, title);
    u8g2.drawHLine(0, 10, 128);
}

static void draw_cell(uint8_t x, uint8_t top,
                      const char *label, const char *value)
{
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(x, top + 8, label);
    u8g2.setFont(u8g2_font_ncenB14_tf);
    u8g2.drawStr(x + 10, top + 23, value);
}

// ---- frame 0: Drive ------------------------------------------------------

static void draw_frame_drive(const OBDData &d)
{
    char b0[12], b1[12], b2[12], b3[12];
    fmt_int(b0, 12, d.speedKph, "");
    fmt_int(b1, 12, d.socPct, "%");
    fmt_int(b2, 12, d.hvPowerKw, "kW");
    fmt_f1(b3, 12, d.batteryV, "V");

    draw_title("DRIVE");
    draw_cell(0, 11, "km/h", b0);
    draw_cell(65, 11, "charge", b1);
    draw_cell(0, 37, "power", b2);
    draw_cell(65, 37, "12v batt", b3);
}

// ---- frame 1: HV battery -------------------------------------------------

static void draw_frame_hv_battery(const OBDData &d)
{
    char b0[12], b1[12], b2[12], b3[12];
    fmt_int(b0, 12, d.hvVoltage, "V");
    fmt_int(b1, 12, d.hvCurrentA, "A");
    fmt_int(b2, 12, d.hvBattTempC, "\xb0\x43"); // °C
    fmt_f1(b3, 12, d.socPct, "%");

    draw_title("HV BATTERY");
    draw_cell(0, 11, "voltage", b0);
    draw_cell(65, 11, "current", b1);
    draw_cell(0, 37, "temp", b2);
    draw_cell(65, 37, "charge", b3);
}

// ---- frame 2: Connectivity -----------------------------------------------

static void draw_conn_row(uint8_t y, const char *label,
                          bool ok, const char *detail)
{
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, y, label);
    if (ok)
        u8g2.drawDisc(25, y - 3, 3);
    else
        u8g2.drawCircle(25, y - 3, 3);
    u8g2.drawStr(32, y, detail);
}

static void draw_frame_connectivity(bool bt_ok, bool mqtt_ok)
{
    draw_title("CONNECTIVITY");
    bool wifi_ok = (WiFi.status() == WL_CONNECTED);
    bool ap_active = WiFi.getMode() == WIFI_AP_STA;
    String ssid = wifi_ok ? WiFi.SSID() : (ap_active ? String("AP ") + WIFI_AP_SSID : String("---"));
    if (wifi_ok)
    {
        int quality = constrain(2 * (WiFi.RSSI() + 100), 0, 100);
        ssid += " " + String(quality) + "%";
    }
    String ip = wifi_ok ? WiFi.localIP().toString() : (ap_active ? WiFi.softAPIP().toString() : String("---"));
    draw_conn_row(21, "BT", bt_ok, bt_ok ? "Connected" : "Scanning");
    draw_conn_row(34, "WIFI", wifi_ok, ssid.c_str());
    draw_conn_row(47, "MQTT", mqtt_ok, mqtt_ok ? "Connected" : "---");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 61, "IP");
    u8g2.drawStr(29, 61, ip.c_str());
}

// ---- redraw --------------------------------------------------------------

static void draw_frame_petrol_engine(const OBDData &d)
{
    char b0[12], b1[12], b2[12], b3[12];
    fmt_int(b0, sizeof(b0), d.rpm, "");
    fmt_int(b1, sizeof(b1), d.fuelPct, "%");
    fmt_int(b2, sizeof(b2), d.coolantC, "\xb0\x43");
    fmt_int(b3, sizeof(b3), d.intakeC, "\xb0\x43");

    draw_title("ENGINE");
    draw_cell(0, 11, "rpm", b0);
    draw_cell(65, 11, "fuel", b1);
    draw_cell(0, 37, "coolant", b2);
    draw_cell(65, 37, "intake", b3);
}

static void draw_frame_petrol_drive(const OBDData &d)
{
    char b0[12], b1[12], b2[12], b3[12];
    fmt_int(b0, sizeof(b0), d.speedKph, "");
    fmt_int(b1, sizeof(b1), d.throttlePct, "%");
    fmt_int(b2, sizeof(b2), d.engineLoadPct, "%");
    fmt_f1(b3, sizeof(b3), d.batteryV, "V");

    draw_title("DRIVE");
    draw_cell(0, 11, "km/h", b0);
    draw_cell(65, 11, "throttle", b1);
    draw_cell(0, 37, "load", b2);
    draw_cell(65, 37, "12v batt", b3);
}

static void redraw(const OBDData &data, bool bt, bool mqtt)
{
    u8g2.clearBuffer();
    if (!storage_get_electric_mode())
    {
        switch (s_frame)
        {
        case 0:
            draw_frame_petrol_engine(data);
            break;
        case 1:
            draw_frame_petrol_drive(data);
            break;
        default:
            draw_frame_connectivity(bt, mqtt);
            break;
        }
        u8g2.sendBuffer();
        return;
    }
    switch (s_frame)
    {
    case 0:
        draw_frame_drive(data);
        break;
    case 1:
        draw_frame_hv_battery(data);
        break;
    case 2:
        draw_frame_connectivity(bt, mqtt);
        break;
    }
    u8g2.sendBuffer();
}

// ---- public API ----------------------------------------------------------

void display_init()
{
    s_interval_ms = storage_get_display_interval();
    u8g2.begin();
    display_boot_status("Starting...");
    Serial.printf("[DISP] Init OK — interval=%ums\n", s_interval_ms);
}

void display_boot_status(const char *status)
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 9, "ZUS OBD BRIDGE");
    u8g2.drawHLine(0, 12, 128);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 31, status);
    u8g2.sendBuffer();
}

void display_loop(const OBDData &data, bool bt_connected, bool mqtt_connected)
{
    if (!s_boot_complete)
    {
        s_boot_complete = true;
        s_last_switch = millis() - s_interval_ms;
    }
    uint32_t now = millis();
    if (now - s_last_switch >= s_interval_ms)
    {
        s_last_switch = now;
        redraw(data, bt_connected, mqtt_connected);
        s_frame = (s_frame + 1) % 3;
    }
}

void display_set_interval(uint32_t ms)
{
    if (ms == 0)
        return;
    s_interval_ms = ms;
    storage_set_display_interval(ms);
}

uint32_t display_get_interval()
{
    return s_interval_ms;
}
