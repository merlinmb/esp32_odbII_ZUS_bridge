// =============================================================================
//  ZUS OBD-II to MQTT Bridge
//  ESP32 WROOM32
//
//  Architecture:
//    - Wi-Fi STA mode (home network)
//    - Bluetooth Classic SPP master — connects to ZUS OBD-II adapter
//    - ELMduino — parses OBD PID responses
//    - WebServer — pairing UI at http://<device-ip>/
//    - PubSubClient — publishes OBD data to MQTT every 10 minutes
//    - Preferences (NVS) — persists target BT MAC across reboots
//
//  First-time setup:
//    1. Flash and open Serial Monitor at 115200.
//    2. Connect to http://<ip shown in Serial Monitor>/
//    3. Scan and select your ZUS device.
//    4. The bridge connects automatically and stays connected.
//
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "storage.h"
#include "bt_obd.h"
#include "mqtt_handler.h"
#include "web_server.h"

// ============================================================================
//  Wi-Fi connection
// ============================================================================
static void wifi_connect() {
    Serial.printf("[WiFi] Connecting to '%s'", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println("\n[WiFi] Timed out — will retry in background");
            return;
        }
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[WiFi] Connected — IP: %s\n",
        WiFi.localIP().toString().c_str());
}

static void wifi_ensure_connected() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.println("[WiFi] Reconnecting...");
    WiFi.reconnect();
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000UL) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Reconnected — IP: %s\n",
            WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] Still offline");
    }
}

// ============================================================================
//  setup()
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n\n=== ZUS OBD Bridge booting ===");

    // 1. Persistent storage
    storage_init();

    // 2. Wi-Fi (blocking until connected or timeout)
    wifi_connect();

    // 3. Bluetooth + OBD module
    bt_obd_init();

    // 4. MQTT client
    mqtt_init();

    // 5. Web server (needs WiFi and BT objects ready)
    web_server_init();

    Serial.println("[BOOT] Setup complete\n");
}

// ============================================================================
//  loop()
// ============================================================================
static unsigned long s_lastWifiCheck = 0;

void loop() {
    // Keep Wi-Fi alive
    if (millis() - s_lastWifiCheck > 30000UL) {
        s_lastWifiCheck = millis();
        wifi_ensure_connected();
    }

    // Bluetooth + OBD state machine
    bt_obd_loop();

    // MQTT keepalive + scheduled publish
    if (WiFi.status() == WL_CONNECTED) {
        mqtt_loop();
        // Publish if fresh data exists and 10-minute window elapsed
        if (bt_obd_is_connected()) {
            mqtt_publish_if_due(bt_obd_get_data());
        }
    }

    // Service web server requests
    web_server_handle_client();

    delay(10);
}
