#pragma once

// =============================================================================
//  ZUS OBD-II to MQTT Bridge — Configuration
//  Edit the values in this file to match your network and preferences.
// =============================================================================

// --- Wi-Fi ----------------------------------------------------------------
#define WIFI_SSID "examplessid"
#define WIFI_PASSWORD "examplepassword"
#define WIFI_CONNECT_TIMEOUT_MS 20000UL // Give up after 20 s, retry later

// --- MQTT -----------------------------------------------------------------
#define MQTT_BROKER "192.168.1.55"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "esp32-zus-bridge"
// Root topic; individual values published as sub-topics, e.g. car/obd/rpm
#define MQTT_TOPIC_ROOT "car/obd"
// How often to push data to MQTT (only if fresh OBD data exists)
#define MQTT_PUBLISH_INTERVAL_MS (10UL * 60UL * 1000UL) // 10 minutes

// --- Bluetooth ------------------------------------------------------------
// Friendly name this ESP32 advertises (master mode, so not really visible)
#define BT_DEVICE_NAME "ESP32-ZUS-Bridge"
// How long (ms) to attempt a BT SPP connect before giving up for this cycle
#define BT_CONNECT_TIMEOUT_MS 8000UL
// Pause between reconnect attempts when the car is out of range
#define BT_RECONNECT_INTERVAL_MS 30000UL
// ELM327 PIN — most cheap adapters use "0000" or "1234"
#define BT_OBD_PIN "1234"
// Duration of a Bluetooth inquiry scan used by the web pairing page
#define BT_SCAN_DURATION_SEC 12

// --- OBD ------------------------------------------------------------------
// ELMduino response timeout per PID request
#define OBD_TIMEOUT_MS 2000
// How frequently to poll the OBD PIDs while connected (ms)
#define OBD_POLL_INTERVAL_MS 5000UL

// --- NVS (Non-Volatile Storage) ------------------------------------------
#define NVS_NAMESPACE "zus_bridge"
#define NVS_KEY_BT_MAC "bt_mac" // Stored as "AA:BB:CC:DD:EE:FF"

// --- Web server -----------------------------------------------------------
#define WEB_SERVER_PORT 80

// --- OLED Display ---------------------------------------------------------
#define DISPLAY_SDA                  21
#define DISPLAY_SCL                  22
#define DISPLAY_DEFAULT_INTERVAL_MS  2000UL
