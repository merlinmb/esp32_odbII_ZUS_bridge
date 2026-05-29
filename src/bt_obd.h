#pragma once
#include <Arduino.h>
#include <vector>

// =============================================================================
//  bt_obd — Bluetooth SPP + ELMduino OBD interface
//
//  Call bt_obd_init() once in setup().
//  Call bt_obd_loop() every iteration of loop().
//
//  The module runs an internal state machine:
//    NO_MAC  -> SCANNING -> CONNECTING -> OBD_INIT -> POLLING -> DISCONNECTED
//  and loops back from DISCONNECTED -> SCANNING automatically.
// =============================================================================

// Latest OBD snapshot (updated while connected; values are -1 when unknown).
struct OBDData {
    float   rpm           = -1;
    float   speedKph      = -1;
    float   coolantTempC  = -1;
    float   engineLoadPct = -1;
    float   fuelLevelPct  = -1;
    float   throttlePct   = -1;
    float   intakeTempC   = -1;
    float   batteryV      = -1;
    bool    engineRunning = false;
    unsigned long lastUpdateMs = 0;  // millis() when last successfully polled
};

// Bluetooth scan result (used by the web pairing page).
struct BTScanResult {
    String name;
    String address;   // "AA:BB:CC:DD:EE:FF"
    int    rssi;
};

// ---- lifecycle ----------------------------------------------------------
void bt_obd_init();
void bt_obd_loop();

// ---- state queries ------------------------------------------------------
bool bt_obd_is_connected();

// Returns a copy of the latest OBD data snapshot.
OBDData bt_obd_get_data();

// ---- web pairing helpers ------------------------------------------------

// Kicks off an async BT Classic inquiry scan.
// Returns false if a scan is already running or BT is connected.
bool bt_scan_start();

// True while the inquiry scan is running.
bool bt_scan_in_progress();

// Returns the results collected so far (safe to call while scanning).
// Results are cleared when a new scan starts.
std::vector<BTScanResult> bt_scan_get_results();

// Set the target MAC from the web UI; persists to NVS automatically.
void bt_set_target_mac(const String &mac);

// Returns the currently configured target MAC (empty string if none).
String bt_get_target_mac();

// Force a disconnect so the next loop() iteration attempts to reconnect.
void bt_obd_force_reconnect();
