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

// Latest OBD snapshot (updated while connected; values are NAN when unknown,
// since current and temperatures can legitimately be negative).
struct OBDData {
    float   rpm           = NAN;
    float   speedKph      = NAN;
    float   coolantC      = NAN;
    float   engineLoadPct = NAN;
    float   fuelPct       = NAN;
    float   throttlePct   = NAN;
    float   intakeC       = NAN;
    float   socPct        = NAN;   // HV battery state of charge
    float   hvVoltage     = NAN;   // HV battery voltage
    float   hvCurrentA    = NAN;   // HV battery current (sign convention unverified)
    float   hvPowerKw     = NAN;   // hvVoltage * hvCurrentA / 1000
    float   hvBattTempC   = NAN;   // HV battery average temperature
    float   odometerKm    = NAN;
    float   batteryV      = NAN;   // 12 V, read by the adapter itself (AT RV)
    unsigned long lastUpdateMs = 0;  // millis() when last successfully polled
};

struct OBDDiagnostics {
    String state;
    String lastCommand;
    String lastResponse;
    uint32_t successfulResponses = 0;
    uint32_t failedResponses = 0;
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
OBDDiagnostics bt_obd_get_diagnostics();

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

// Forget the target MAC (NVS and RAM) and stop reconnecting.
void bt_clear_target_mac();

// Returns the currently configured target MAC (empty string if none).
String bt_get_target_mac();

// Force a disconnect so the next loop() iteration attempts to reconnect.
void bt_obd_force_reconnect();

// Queue a non-blocking UDS DID sweep (EV mode, while polling). Results are
// logged to the serial monitor as "[SWEEP] ...". Returns false if not possible.
bool bt_obd_start_sweep();

// True while a sweep is queued or running.
bool bt_obd_sweep_running();
