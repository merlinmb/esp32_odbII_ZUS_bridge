#include "bt_obd.h"
#include "config.h"
#include "storage.h"

#include <BluetoothSerial.h>
#include <ELMduino.h>
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <vector>

// ============================================================================
//  Module-level state
// ============================================================================

static BluetoothSerial SerialBT;
static ELM327 myELM327;

enum BTState
{
    BT_NO_MAC,     // No target MAC saved yet — waiting for web pairing
    BT_IDLE,       // MAC known, waiting for reconnect timer
    BT_CONNECTING, // Attempting SPP connect
    BT_OBD_INIT,   // SPP connected, initialising ELM327
    BT_POLLING,    // Normal polling loop
    BT_SCANNING    // Running BT Classic inquiry scan
};

static BTState s_state = BT_NO_MAC;
static String s_targetMac = "";
static unsigned long s_lastAttemptMs = 0;
static unsigned long s_lastPollMs = 0;
static OBDData s_data;

// ---- scan state ---------------------------------------------------------
static volatile bool s_scanRunning = false;
static volatile bool s_scanDone = false;
static std::vector<BTScanResult> s_scanResults;
static SemaphoreHandle_t s_scanMutex = nullptr;

// ============================================================================
//  Bluetooth GAP callback — collects Classic BT inquiry results
// ============================================================================
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    if (event == ESP_BT_GAP_DISC_RES_EVT)
    {
        BTScanResult r;

        // Format MAC address
        char mac[18];
        snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 param->disc_res.bda[0], param->disc_res.bda[1],
                 param->disc_res.bda[2], param->disc_res.bda[3],
                 param->disc_res.bda[4], param->disc_res.bda[5]);
        r.address = mac;
        r.name = "";
        r.rssi = 0;

        // Extract name and RSSI from device properties
        for (int i = 0; i < param->disc_res.num_prop; i++)
        {
            auto &p = param->disc_res.prop[i];
            if (p.type == ESP_BT_GAP_DEV_PROP_BDNAME && p.len > 0)
            {
                r.name = String((char *)p.val).substring(0, p.len);
            }
            else if (p.type == ESP_BT_GAP_DEV_PROP_RSSI)
            {
                r.rssi = *((int8_t *)p.val);
            }
        }

        xSemaphoreTake(s_scanMutex, portMAX_DELAY);
        // Avoid duplicates
        bool found = false;
        for (auto &existing : s_scanResults)
        {
            if (existing.address == r.address)
            {
                found = true;
                break;
            }
        }
        if (!found)
            s_scanResults.push_back(r);
        xSemaphoreGive(s_scanMutex);
    }
    else if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT)
    {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED)
        {
            s_scanRunning = false;
            s_scanDone = true;
            Serial.println("[BT] Inquiry scan complete");
        }
    }
}

// ============================================================================
//  OBD helpers — non-blocking ELMduino polling
// ============================================================================

// Reads a single float PID via ELMduino.
// Returns true and fills `out` on success; returns false if still waiting.
// On hard error, returns false and sets out = -1.
static bool poll_pid(float &out, float (*pidFn)())
{
    out = pidFn();
    if (myELM327.nb_rx_state == ELM_SUCCESS)
    {
        return true;
    }
    if (myELM327.nb_rx_state == ELM_GETTING_MSG)
    {
        return false; // still waiting
    }
    out = -1;
    return true; // finished (with error value)
}

// Sequence index for round-robin polling
static uint8_t s_pollStep = 0;

static void do_poll_step()
{
    float val;
    bool done = false;

    switch (s_pollStep)
    {
    case 0:
        done = poll_pid(val, []() -> float
                        { return myELM327.rpm(); });
        if (done)
        {
            s_data.rpm = (val > 0) ? val : -1;
            s_pollStep++;
        }
        break;
    case 1:
        done = poll_pid(val, []() -> float
                        { return (float)myELM327.kph(); });
        if (done)
        {
            s_data.speedKph = (val >= 0) ? val : -1;
            s_pollStep++;
        }
        break;
    case 2:
        done = poll_pid(val, []() -> float
                        { return myELM327.engineCoolantTemp(); });
        if (done)
        {
            s_data.coolantTempC = (val > -300) ? val : -1;
            s_pollStep++;
        }
        break;
    case 3:
        done = poll_pid(val, []() -> float
                        { return myELM327.engineLoad(); });
        if (done)
        {
            s_data.engineLoadPct = (val >= 0) ? val : -1;
            s_pollStep++;
        }
        break;
    case 4:
        done = poll_pid(val, []() -> float
                        { return myELM327.fuelLevel(); });
        if (done)
        {
            if (val < 0)
            {
                static bool s_fuelErrLogged = false;
                if (!s_fuelErrLogged)
                {
                    Serial.printf("[OBD] Fuel level unavailable (ELM state=%d) — PID 0x2F may not be supported\n",
                                  (int)myELM327.nb_rx_state);
                    s_fuelErrLogged = true;
                }
            }
            s_data.fuelLevelPct = (val >= 0) ? val : -1;
            s_pollStep++;
        }
        break;
    case 5:
        done = poll_pid(val, []() -> float
                        { return myELM327.throttle(); });
        if (done)
        {
            s_data.throttlePct = (val >= 0) ? val : -1;
            s_pollStep++;
        }
        break;
    case 6:
        done = poll_pid(val, []() -> float
                        { return myELM327.intakeAirTemp(); });
        if (done)
        {
            s_data.intakeTempC = (val > -300) ? val : -1;
            s_pollStep++;
        }
        break;
    case 7:
        // Battery voltage via AT RV (ELMduino exposes this as batteryVoltage())
        done = poll_pid(val, []() -> float
                        { return myELM327.batteryVoltage(); });
        if (done)
        {
            s_data.batteryV = (val > 0) ? val : -1;
            s_pollStep = 0;
        }
        break;
    default:
        s_pollStep = 0;
        break;
    }

    if (s_pollStep == 0)
    {
        // Completed one full round
        s_data.engineRunning = (s_data.rpm > 0);
        s_data.lastUpdateMs = millis();
        Serial.printf("[OBD] RPM:%.0f Speed:%.0f Coolant:%.1f Load:%.1f Fuel:%.1f Throttle:%.1f IAT:%.1f Batt:%.2fV\n",
                      s_data.rpm, s_data.speedKph, s_data.coolantTempC,
                      s_data.engineLoadPct, s_data.fuelLevelPct,
                      s_data.throttlePct, s_data.intakeTempC, s_data.batteryV);
    }
}

// ============================================================================
//  Convert "AA:BB:CC:DD:EE:FF" string to uint8_t[6]
// ============================================================================
static bool mac_str_to_bytes(const String &mac, uint8_t out[6])
{
    if (mac.length() != 17)
        return false;
    for (int i = 0; i < 6; i++)
    {
        out[i] = (uint8_t)strtol(mac.substring(i * 3, i * 3 + 2).c_str(), nullptr, 16);
    }
    return true;
}

// ============================================================================
//  Public API
// ============================================================================

void bt_obd_init()
{
    s_scanMutex = xSemaphoreCreateMutex();

    // Load saved MAC from NVS
    if (storage_load_mac(s_targetMac))
    {
        Serial.print("[BT] Loaded saved MAC: ");
        Serial.println(s_targetMac);
        s_state = BT_IDLE;
    }
    else
    {
        Serial.println("[BT] No MAC saved — waiting for web pairing");
        s_state = BT_NO_MAC;
    }

    // Init BT in master mode
    SerialBT.begin(BT_DEVICE_NAME, true);

    // Register our GAP callback for discovery scans
    esp_bt_gap_register_callback(gap_cb);

    // Allow the adapter time to settle
    delay(200);
    Serial.println("[BT] Bluetooth master initialised");
}

void bt_obd_loop()
{
    switch (s_state)
    {

    // -----------------------------------------------------------------
    case BT_NO_MAC:
        // Nothing to do until the web UI provides a MAC
        return;

    // -----------------------------------------------------------------
    case BT_IDLE:
    {
        unsigned long now = millis();
        if (now - s_lastAttemptMs < BT_RECONNECT_INTERVAL_MS)
            return;
        s_lastAttemptMs = now;
        Serial.print("[BT] Attempting connect to ");
        Serial.println(s_targetMac);
        s_state = BT_CONNECTING;
        break; // fall into CONNECTING on next call
    }

    // -----------------------------------------------------------------
    case BT_CONNECTING:
    {
        uint8_t mac[6];
        if (!mac_str_to_bytes(s_targetMac, mac))
        {
            Serial.println("[BT] Bad MAC format — resetting to NO_MAC");
            s_state = BT_NO_MAC;
            return;
        }

        SerialBT.setPin(BT_OBD_PIN, sizeof(BT_OBD_PIN) - 1);
        bool ok = SerialBT.connect(mac);

        if (ok && SerialBT.connected())
        {
            Serial.println("[BT] SPP connected!");
            s_state = BT_OBD_INIT;
            s_pollStep = 0;
        }
        else
        {
            Serial.println("[BT] Connect failed — will retry");
            // Don't call disconnect() — no session was established and doing so
            // can corrupt the ESP32 BT stack, preventing future connect() calls.
            s_state = BT_IDLE;
            s_lastAttemptMs = millis(); // reset timer
        }
        return;
    }

    // -----------------------------------------------------------------
    case BT_OBD_INIT:
    {
        if (myELM327.begin(SerialBT, false, OBD_TIMEOUT_MS))
        {
            Serial.println("[OBD] ELM327 handshake OK");
            s_state = BT_POLLING;
            s_lastPollMs = millis();
        }
        else
        {
            Serial.println("[OBD] ELM327 handshake FAILED — disconnecting");
            SerialBT.disconnect();
            s_state = BT_IDLE;
            s_lastAttemptMs = millis();
        }
        return;
    }

    // -----------------------------------------------------------------
    case BT_POLLING:
    {
        if (!SerialBT.connected())
        {
            Serial.println("[BT] Connection lost — will reconnect");
            SerialBT.disconnect();
            s_state = BT_IDLE;
            s_lastAttemptMs = millis();
            return;
        }

        unsigned long now = millis();
        if (now - s_lastPollMs >= OBD_POLL_INTERVAL_MS)
        {
            s_lastPollMs = now;
            s_pollStep = 0; // start a fresh round
        }
        do_poll_step();
        return;
    }

    // -----------------------------------------------------------------
    case BT_SCANNING:
        // Scan is driven by the GAP callback; just wait for it to finish.
        if (s_scanDone)
        {
            // Restore idle state; connection loop resumes on next timer tick
            s_scanDone = false;
            s_state = s_targetMac.length() ? BT_IDLE : BT_NO_MAC;
            s_lastAttemptMs = millis(); // don't immediately try to connect
        }
        return;
    }
}

bool bt_obd_is_connected()
{
    return (s_state == BT_POLLING) && SerialBT.connected();
}

OBDData bt_obd_get_data()
{
    return s_data;
}

// ---- scan ---------------------------------------------------------------

bool bt_scan_start()
{
    if (s_scanRunning)
        return false;
    if (s_state == BT_POLLING)
    {
        // Disconnect so the radio is free for inquiry
        SerialBT.disconnect();
    }

    xSemaphoreTake(s_scanMutex, portMAX_DELAY);
    s_scanResults.clear();
    xSemaphoreGive(s_scanMutex);

    s_scanRunning = true;
    s_scanDone = false;
    s_state = BT_SCANNING;

    esp_err_t err = esp_bt_gap_start_discovery(
        ESP_BT_INQ_MODE_GENERAL_INQUIRY,
        BT_SCAN_DURATION_SEC,
        0);

    if (err != ESP_OK)
    {
        Serial.printf("[BT] Scan start failed: %d\n", err);
        s_scanRunning = false;
        s_scanDone = true;
        return false;
    }
    Serial.println("[BT] Inquiry scan started");
    return true;
}

bool bt_scan_in_progress()
{
    return s_scanRunning;
}

std::vector<BTScanResult> bt_scan_get_results()
{
    std::vector<BTScanResult> copy;
    xSemaphoreTake(s_scanMutex, portMAX_DELAY);
    copy = s_scanResults;
    xSemaphoreGive(s_scanMutex);
    return copy;
}

void bt_set_target_mac(const String &mac)
{
    s_targetMac = mac;
    storage_save_mac(mac);
    // Disconnect from any current target and start fresh
    if (s_state == BT_POLLING)
        SerialBT.disconnect();
    s_state = BT_IDLE;
    s_lastAttemptMs = 0; // connect immediately on next loop
    Serial.print("[BT] Target MAC updated: ");
    Serial.println(mac);
}

String bt_get_target_mac()
{
    return s_targetMac;
}

void bt_obd_force_reconnect()
{
    if (s_state == BT_POLLING)
        SerialBT.disconnect();
    s_state = BT_IDLE;
    s_lastAttemptMs = 0;
}
