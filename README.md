# ZUS OBD-II to MQTT Bridge

An ESP32-based bridge that connects to a ZUS OBD-II Bluetooth adapter and publishes vehicle data to an MQTT broker. Perfect for integrating your car's diagnostic data into home automation systems, logging platforms, or custom dashboards.

## Features

- **Bluetooth OBD-II Integration**: Connects to ZUS OBD-II adapters via Bluetooth Classic SPP
- **MQTT Publishing**: Automatically publishes OBD data to your MQTT broker (configurable interval)
- **Web UI Pairing**: Simple web interface for scanning and pairing with available Bluetooth devices
- **Persistent Storage**: Remembers paired device MAC address across reboots
- **OLED Display**: Real-time status display showing connection state, OBD data, and MQTT status
- **WiFi Auto-Reconnect**: Automatic WiFi reconnection with background retry
- **Configurable**: Easy setup via `config.h` for WiFi, MQTT, Bluetooth, and OBD parameters

## Hardware Requirements

- **ESP32 WROOM32** development board (or compatible ESP32 variant)
- **Micro USB cable** for power and flashing
- **OLED display** (128x64, I2C-based, e.g., SSD1306) — optional but recommended
- **SDA/SCL pins** for display (configurable, defaults to GPIO21/GPIO22)
- **USB Serial adapter** for initial Serial Monitor access (if not integrated on your devboard)

## Getting Started

### 1. Flash the Firmware

#### Prerequisites
- [PlatformIO](https://platformio.org/) installed (via VSCode extension or CLI)
- USB connection to your ESP32 board

#### Build and Upload
```bash
# Using PlatformIO CLI
pio run -e esp32dev --target upload

# Or use VSCode: PlatformIO → Upload
```

Upload port defaults to `COM18` (configured in `platformio.ini`). Adjust if your device uses a different port.

### 2. Initial Setup

1. Open the Serial Monitor at **115200 baud**
2. Wait for the device to boot and display its IP address (e.g., `192.168.1.100`)
3. Open a browser and navigate to `http://<device-ip>/`
4. Use the web interface to scan for nearby Bluetooth devices
5. Select your ZUS OBD-II adapter
6. The device will store the MAC address and attempt to connect on every boot

### 3. Configuration

Edit `src/config.h` before flashing to set:

**WiFi**
- `WIFI_SSID` — your home network SSID
- `WIFI_PASSWORD` — your WiFi password
- `WIFI_CONNECT_TIMEOUT_MS` — connection timeout (default: 20s)

**MQTT**
- `MQTT_BROKER` — IP address of your MQTT broker
- `MQTT_PORT` — MQTT port (default: 1883)
- `MQTT_CLIENT_ID` — unique identifier for this bridge
- `MQTT_TOPIC_ROOT` — base topic for published data (e.g., `car/obd`)
- `MQTT_PUBLISH_INTERVAL_MS` — publish frequency (default: 10 minutes)

**Bluetooth**
- `BT_DEVICE_NAME` — name advertised by this ESP32
- `BT_CONNECT_TIMEOUT_MS` — BT connection attempt timeout (default: 8s)
- `BT_RECONNECT_INTERVAL_MS` — pause between reconnection attempts (default: 30s)
- `BT_OBD_PIN` — PIN for the OBD adapter (commonly "0000" or "1234")
- `BT_SCAN_DURATION_SEC` — duration of web UI pairing scan (default: 12s)

**OBD**
- `OBD_TIMEOUT_MS` — timeout per PID request (default: 2s)
- `OBD_POLL_INTERVAL_MS` — polling frequency when connected (default: 5s)

**Display**
- `DISPLAY_SDA`, `DISPLAY_SCL` — I2C pins for OLED (GPIO 21 & 22 by default)
- `DISPLAY_DEFAULT_INTERVAL_MS` — refresh rate for display (default: 2s)

See `src/config.example.h` for a template with all options.

## Architecture

### Core Modules

| Module | Purpose |
|--------|---------|
| **main.cpp** | Initialization and main loop; WiFi reconnection |
| **bt_obd.cpp** | Bluetooth SPP connection & OBD-II data polling (ELMduino) |
| **mqtt_handler.cpp** | MQTT broker connection & data publishing |
| **web_server.cpp** | HTTP server for pairing UI |
| **display.cpp** | OLED status display rendering |
| **storage.cpp** | Persistent storage (NVS) for paired device MAC |

### Data Flow

```
ZUS OBD-II Adapter (Bluetooth)
    ↓
[bt_obd] — polls PID requests
    ↓
OBD Data (RPM, Speed, etc.)
    ↓
[mqtt_handler] — publishes every 10 minutes (if fresh data exists)
    ↓
MQTT Broker
    ↓
Home Automation / Logging / Dashboard
```

### WiFi & Bluetooth Behavior

- WiFi connects at startup; if unreachable within 20 seconds, continues with background retries
- Bluetooth state machine maintains connection to stored ZUS device MAC
- If Bluetooth disconnects, retries are attempted every 30 seconds
- Web UI remains accessible even if Bluetooth or MQTT are offline

## MQTT Topic Structure

Published data uses the following topic hierarchy:

```
car/obd/rpm              → engine RPM
car/obd/speed            → vehicle speed
car/obd/fuel_level       → fuel tank percentage
[other OBD PIDs...]
```

(Topic root configurable via `MQTT_TOPIC_ROOT`)

## Status Display (OLED)

The optional OLED display shows:
- WiFi connection status
- Bluetooth connection status
- Active OBD PIDs being polled
- Timestamp of last MQTT publish
- Current refresh interval (configurable via web UI)

## Troubleshooting

### Device won't boot / Serial monitor shows errors
- Ensure USB cable is connected and device is recognized on the correct COM port
- Check `platformio.ini` for correct `upload_port`
- Try a different USB cable or port

### Can't find device on web UI
- Verify ESP32 IP address from Serial Monitor
- Ensure your computer is on the same WiFi network
- Check that WiFi connected successfully in Serial Monitor

### Bluetooth pairing fails
- Verify ZUS adapter PIN (usually "0000" or "1234") in `config.h`
- Ensure ZUS adapter is powered on and within range
- Check that no other device is paired to the adapter
- Review BT SPP logs in Serial Monitor

### MQTT data not publishing
- Verify MQTT broker is reachable (ping the IP from ESP32)
- Check `MQTT_BROKER` and `MQTT_PORT` in `config.h`
- Monitor Serial output for MQTT connection logs
- Ensure firewall allows port 1883 (or your configured MQTT port)

### OBD data not updating
- Confirm vehicle is running or at least in ON position (not just ACC)
- Verify ZUS adapter is connected to car's OBD-II port
- Check OBD polling logs in Serial Monitor
- Increase `OBD_TIMEOUT_MS` if PIDs timeout frequently

### Display not showing anything
- Verify I2C address (typically 0x3C or 0x3D) — check Serial output
- Confirm SDA/SCL pins match your wiring (default: GPIO 21/22)
- Ensure pull-up resistors are in place (typically on the display module)

## Dependencies

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) (via PlatformIO/Arduino framework)
- [ArduinoJson](https://github.com/bblanchon/ArduinoJson) — JSON parsing for web requests
- [ELMduino](https://github.com/PowerBroker2/ELMduino) — OBD-II protocol & PID parsing
- [PubSubClient](https://github.com/knolleary/pubsub) — MQTT client
- [U8g2](https://github.com/olikraus/u8g2) — OLED display library

All dependencies are automatically fetched by PlatformIO.

## Building & Development

### Project Structure
```
src/
  ├── main.cpp           # Main loop & WiFi handling
  ├── bt_obd.cpp/h       # Bluetooth & OBD polling
  ├── mqtt_handler.cpp/h # MQTT publish
  ├── web_server.cpp/h   # HTTP pairing UI
  ├── display.cpp/h      # OLED rendering
  ├── storage.cpp/h      # NVS persistence
  ├── config.h           # Configuration (user-editable)
  └── config.example.h   # Configuration template
platformio.ini            # Build configuration
```

### Compilation
```bash
# Build for ESP32
pio run -e esp32dev

# Upload to device
pio run -e esp32dev --target upload

# Monitor serial output
pio device monitor -b 115200
```

## License

This project is licensed under the MIT License — see LICENSE file for details.

## Contributing

Found a bug or have a feature request? Open an issue or submit a pull request on GitHub.
