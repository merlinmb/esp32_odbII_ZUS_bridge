#include "mqtt_handler.h"
#include "config.h"
#include "bt_obd.h"
#include "storage.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

static WiFiClient s_wifiClient;
static PubSubClient s_mqtt(s_wifiClient);

static unsigned long s_lastPublishMs = 0;
static unsigned long s_lastDataUpdateMs = 0; // lastUpdateMs at time of last publish
static unsigned long s_lastConnectMs = 0;

// ---- helpers ------------------------------------------------------------

static void mqttPublishStat(String deviceName, String topicPostFix, String value, bool retain)
{
    String topic = "stat/" + deviceName + "/" + topicPostFix;
    s_mqtt.publish(topic.c_str(), value.c_str(), retain);
}

static bool do_connect()
{
    Serial.print("[MQTT] Connecting to ");
    Serial.print(MQTT_BROKER);
    Serial.print("...");
    // Last Will: broker flips the retained status to "offline" if we drop
    if (s_mqtt.connect(MQTT_CLIENT_ID, "stat/" BT_DEVICE_NAME "/status", 0, true, "offline"))
    {
        Serial.println(" OK");
        // Publish an online retained status message
        s_mqtt.publish("stat/" BT_DEVICE_NAME "/status", "online", true);
        mqttTransmitInitStat(BT_DEVICE_NAME);
        return true;
    }
    Serial.printf(" FAILED rc=%d\n", s_mqtt.state());
    return false;
}

static void publish_float(const char *subtopic, float val)
{
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_ROOT, subtopic);

    // Invalid values must not remain as retained payloads on broker.
    if (!isfinite(val))
    {
        s_mqtt.publish(topic, "", /*retain=*/true);
        return;
    }

    char payload[16];
    snprintf(payload, sizeof(payload), "%.2f", val);
    s_mqtt.publish(topic, payload, /*retain=*/true);
}

static bool do_publish(const OBDData &data)
{
    if (!s_mqtt.connected())
        return false;
    if (data.lastUpdateMs == 0)
        return false;

    const bool electric = storage_get_electric_mode();

    publish_float("speed_kph", data.speedKph);
    publish_float("battery_v", data.batteryV);
    if (electric)
    {
        publish_float("soc_pct", data.socPct);
        publish_float("hv_voltage", data.hvVoltage);
        publish_float("hv_current_a", data.hvCurrentA);
        publish_float("hv_power_kw", data.hvPowerKw);
        publish_float("hv_batt_temp_c", data.hvBattTempC);
        publish_float("odometer_km", data.odometerKm);
    }
    else
    {
        publish_float("rpm", data.rpm);
        publish_float("coolant_c", data.coolantC);
        publish_float("engine_load_pct", data.engineLoadPct);
        publish_float("fuel_pct", data.fuelPct);
        publish_float("throttle_pct", data.throttlePct);
        publish_float("intake_c", data.intakeC);
    }

    // JSON summary on root topic (ArduinoJson writes NAN as null)
    JsonDocument doc;
    doc["speed"] = data.speedKph;
    doc["battery"] = data.batteryV;
    if (electric)
    {
        doc["soc"] = data.socPct;
        doc["hv_voltage"] = data.hvVoltage;
        doc["hv_current"] = data.hvCurrentA;
        doc["hv_power"] = data.hvPowerKw;
        doc["hv_temp"] = data.hvBattTempC;
        doc["odometer"] = data.odometerKm;
    }
    else
    {
        doc["rpm"] = data.rpm;
        doc["coolant"] = data.coolantC;
        doc["engine_load"] = data.engineLoadPct;
        doc["fuel"] = data.fuelPct;
        doc["throttle"] = data.throttlePct;
        doc["intake"] = data.intakeC;
    }
    char json[256];
    serializeJson(doc, json, sizeof(json));

    s_mqtt.publish(MQTT_TOPIC_ROOT "/json", json, true);
    s_lastPublishMs = millis();
    s_lastDataUpdateMs = data.lastUpdateMs;
    Serial.println("[MQTT] Published OBD snapshot");
    return true;
}

// ============================================================================
//  Public API
// ============================================================================

void mqtt_init()
{
    if (!MQTT_ENABLED)
        return;

    s_mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    s_mqtt.setKeepAlive(60);
    // Bound how long an unreachable broker can stall loop()
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    s_wifiClient.setConnectionTimeout(1500);
#else
    s_wifiClient.setTimeout(2); // core 2.x: seconds, also bounds connect()
#endif
    s_mqtt.setSocketTimeout(2);
    Serial.println("[MQTT] Configured");
}

void mqtt_loop()
{
    if (!MQTT_ENABLED)
        return;

    if (!s_mqtt.connected())
    {
        unsigned long now = millis();
        if (now - s_lastConnectMs > 10000UL)
        { // retry every 10 s
            s_lastConnectMs = now;
            do_connect();
        }
    }
    else
    {
        s_mqtt.loop();
    }
}

bool mqtt_is_connected()
{
    return MQTT_ENABLED && s_mqtt.connected();
}

void mqttTransmitInitStat(String deviceName)
{
    if (!MQTT_ENABLED)
        return;

    mqttPublishStat(deviceName, "init", "{\"value1\":\"" + WiFi.localIP().toString() + "\",\"value2\":\"" + WiFi.macAddress() + "\",\"value3\":\"" + deviceName + "\"}", true);
}

bool mqtt_publish_if_due(const OBDData &data)
{
    if (!MQTT_ENABLED || !s_mqtt.connected())
        return false;

    // No fresh data
    if (data.lastUpdateMs == 0)
        return false;

    // Data is not newer than last publish
    if (data.lastUpdateMs <= s_lastDataUpdateMs)
        return false;

    // Interval not yet elapsed (allow first publish immediately)
    unsigned long now = millis();
    if (s_lastPublishMs > 0 && (now - s_lastPublishMs) < MQTT_PUBLISH_INTERVAL_MS)
        return false;

    return do_publish(data);
}

bool mqtt_publish_now(const OBDData &data)
{
    if (!MQTT_ENABLED)
        return false;

    return do_publish(data);
}
