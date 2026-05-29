# OLED Display — Design Spec
_2026-05-29_

## Overview

Add a 1.30" monochrome I2C OLED (SH1106, 128×64) to the ZUS OBD-II Bridge. The display cycles through three frames — Engine, Sensors, Connectivity — at a user-configurable interval stored in NVS and editable via the existing web UI.

## Hardware

| Signal | GPIO |
|--------|------|
| SDA    | 21   |
| SCL    | 22   |

Controller: SH1106 (assumed from 1.30" IIC V2.2 marking). Library: **U8g2** (`olikraus/U8g2`).

## Frames

### Frame 0 — Engine
Two-column 2×2 grid. Title bar: `ENGINE`.
| Cell | OBDData field | Format |
|------|--------------|--------|
| RPM | `rpm` | `3500` (no unit) |
| SPEED | `speedKph` | `87` + `km/h` sub-label |
| LOAD | `engineLoadPct` | `45%` |
| THROTTLE | `throttlePct` | `28%` |

### Frame 1 — Sensors
Two-column 2×2 grid. Title bar: `SENSORS`.
| Cell | OBDData field | Format |
|------|--------------|--------|
| COOLANT | `coolantTempC` | `89°C` |
| INTAKE | `intakeTempC` | `35°C` |
| FUEL | `fuelLevelPct` | `64%` |
| BATTERY | `batteryV` | `12.4V` |

### Frame 2 — Connectivity
Four rows. Title bar: `CONNECTIVITY`.
| Row | Content |
|-----|---------|
| BT | filled circle if connected + "ZUS-OBD" / "Scanning…" |
| WIFI | filled circle if connected + SSID |
| MQTT | filled circle if connected + broker IP |
| IP | device IP address (or "---" if no WiFi) |

Filled circle = `\x07` (U8g2 bullet glyph) for connected; empty `o` for disconnected.

## Value rendering

- Any `float` field that equals `-1` renders as `--`.
- `engineRunning == false` → Frame 0 shows `OFF` for RPM and speed.

## Frame cycling

```cpp
static uint8_t  s_frame       = 0;   // 0, 1, 2
static uint32_t s_interval_ms = 2000;
static uint32_t s_last_switch = 0;

// in display_loop():
if (millis() - s_last_switch >= s_interval_ms) {
    s_frame = (s_frame + 1) % 3;
    s_last_switch = millis();
    redraw();
}
```

Full redraw every cycle; no partial updates needed at this refresh rate.

## Storage

NVS namespace: `zus_bridge` (existing). Key: `disp_ms` (uint32). Default: `2000`.

New helpers in `storage.h/cpp`:
```cpp
uint32_t storage_get_display_interval();
void     storage_set_display_interval(uint32_t ms);
```

## Web UI

Add to `data/index.html` settings section:

```
Frame interval: [____] ms   [Save]
```

POST handler in `web_server.cpp`:
- Param: `display_interval`
- Validates > 0, calls `display_set_interval()` + `storage_set_display_interval()`

## Module interface

```cpp
// display.h
void     display_init();
void     display_loop(const OBDData &data, bool bt_connected, bool mqtt_connected);
void     display_set_interval(uint32_t ms);
uint32_t display_get_interval();
```

`display_init()` reads interval from NVS on startup.

## Files changed

| File | Change |
|------|--------|
| `src/display.h` | new |
| `src/display.cpp` | new |
| `src/config.h` | add DISPLAY_SDA/SCL/DEFAULT_INTERVAL_MS |
| `platformio.ini` | add olikraus/U8g2 |
| `src/storage.h` | add display interval helpers |
| `src/storage.cpp` | implement helpers |
| `src/main.cpp` | call display_init() / display_loop() |
| `src/web_server.cpp` | handle display_interval POST param |
| `data/index.html` | add interval input field |
