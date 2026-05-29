#include "mqtt_handler.h"
#include "config.h"
#include "bt_obd.h"

#include <WiFi.h>
#include <PubSubClient.h>

static WiFiClient    s_wifiClient;
static PubSubClient  s_mqtt(s_wifiClient);

static unsigned long s_lastPublishMs    = 0;
static unsigned long s_lastDataUpdateMs = 0;   // lastUpdateMs at time of last publish
static unsigned long s_lastConnectMs    = 0;

// ---- helpers ------------------------------------------------------------

static bool do_connect() {
    Serial.print("[MQTT] Connecting to "); Serial.print(MQTT_BROKER); Serial.print("...");
    if (s_mqtt.connect(MQTT_CLIENT_ID)) {
        Serial.println(" OK");
        // Publish an online retained status message
        s_mqtt.publish(MQTT_TOPIC_ROOT "/status", "online", true);
        return true;
    }
    Serial.printf(" FAILED rc=%d\n", s_mqtt.state());
    return false;
}

static void publish_float(const char *subtopic, float val) {
    if (val < 0) return;  // -1 sentinel — skip
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_ROOT, subtopic);
    char payload[16];
    snprintf(payload, sizeof(payload), "%.2f", val);
    s_mqtt.publish(topic, payload, /*retain=*/true);
}

static bool do_publish(const OBDData &data) {
    if (!s_mqtt.connected()) return false;

    publish_float("rpm",         data.rpm);
    publish_float("speed_kph",   data.speedKph);
    publish_float("coolant_c",   data.coolantTempC);
    publish_float("engine_load", data.engineLoadPct);
    publish_float("fuel_pct",    data.fuelLevelPct);
    publish_float("throttle",    data.throttlePct);
    publish_float("intake_c",    data.intakeTempC);
    publish_float("battery_v",   data.batteryV);

    // JSON summary on root topic
    char json[256];
    snprintf(json, sizeof(json),
        "{\"rpm\":%.0f,\"speed\":%.0f,\"coolant\":%.1f,"
        "\"load\":%.1f,\"fuel\":%.1f,\"throttle\":%.1f,"
        "\"intake\":%.1f,\"battery\":%.2f,\"running\":%s}",
        data.rpm, data.speedKph, data.coolantTempC,
        data.engineLoadPct, data.fuelLevelPct, data.throttlePct,
        data.intakeTempC, data.batteryV,
        data.engineRunning ? "true" : "false");

    s_mqtt.publish(MQTT_TOPIC_ROOT "/json", json, true);
    s_lastPublishMs    = millis();
    s_lastDataUpdateMs = data.lastUpdateMs;
    Serial.println("[MQTT] Published OBD snapshot");
    return true;
}

// ============================================================================
//  Public API
// ============================================================================

void mqtt_init() {
    s_mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    s_mqtt.setKeepAlive(60);
    Serial.println("[MQTT] Configured");
}

void mqtt_loop() {
    if (!s_mqtt.connected()) {
        unsigned long now = millis();
        if (now - s_lastConnectMs > 10000UL) {  // retry every 10 s
            s_lastConnectMs = now;
            do_connect();
        }
    } else {
        s_mqtt.loop();
    }
}

bool mqtt_is_connected() {
    return s_mqtt.connected();
}

bool mqtt_publish_if_due(const OBDData &data) {
    if (!s_mqtt.connected()) return false;

    // No fresh data
    if (data.lastUpdateMs == 0) return false;

    // Data is not newer than last publish
    if (data.lastUpdateMs <= s_lastDataUpdateMs) return false;

    // Interval not yet elapsed (allow first publish immediately)
    unsigned long now = millis();
    if (s_lastPublishMs > 0 && (now - s_lastPublishMs) < MQTT_PUBLISH_INTERVAL_MS) return false;

    return do_publish(data);
}

bool mqtt_publish_now(const OBDData &data) {
    return do_publish(data);
}
