#pragma once
#include <Arduino.h>

// Initialise MQTT (call once after WiFi is up).
void mqtt_init();

// Must be called every loop() iteration to maintain connection and process
// incoming messages.
void mqtt_loop();

// Returns true when connected to the broker.
bool mqtt_is_connected();

// Publish an OBD snapshot.  Only actually sends if:
//   1. The broker is connected, AND
//   2. `data.lastUpdateMs` is newer than the last publish, AND
//   3. At least MQTT_PUBLISH_INTERVAL_MS has elapsed since the last publish.
// Returns true if a publish occurred.
#include "bt_obd.h"
bool mqtt_publish_if_due(const OBDData &data);

// Force-publish regardless of the timer (useful for testing).
bool mqtt_publish_now(const OBDData &data);
