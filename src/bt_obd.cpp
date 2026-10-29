#include "bt_obd.h"
#include "config.h"
#include "storage.h"

#include <BluetoothSerial.h>
#include <ELMduino.h>
#include <esp_bt.h>
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
static String s_lastCommand = "-";
static String s_lastResponse = "-";
static uint32_t s_successfulResponses = 0;
static uint32_t s_failedResponses = 0;

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
//  Volvo EX30 polling — non-blocking ELMduino polling
//
//  The EX30 (Geely SEA platform) does not answer the standard Mode 01 PIDs
//  usefully. Instead we use Volvo's UDS "ReadDataByIdentifier" (service 0x22)
//  over 29-bit CAN, addressing individual ECUs directly:
//
//    request ID  = 1D D0 16 <ecu>    (ATCP 1D + ATSH D016<ecu>)
//    response ID = 1E C? ?E 80       (ATCRA, ELM's default filter would drop it)
//
//    ECU 0x37 (VCU)  : F40D speed, DD01 odometer
//    ECU 0x35 (BECM) : 4801 SoC, 4802 current, 4803 voltage, 491B avg temp
//
//  Source: OBDb signal sets for Polestar 2 / XC40 Recharge / Polestar 4
//  (https://github.com/OBDb). Polestar 4 is SEA-based like the EX30, and the
//  4801 SoC scaling matches a reported EX30 capture. Not yet confirmed on an
//  EX30 for every DID — check the [OBD] serial log for raw responses.
// ============================================================================

static void parse_speed(const char *p);
static void parse_odometer(const char *p);
static void parse_soc(const char *p);
static void parse_hv_current(const char *p);
static void parse_hv_voltage(const char *p);
static void parse_hv_temp(const char *p);
static void parse_12v(const char *p);
static void parse_rpm(const char *p);
static void parse_coolant(const char *p);
static void parse_engine_load(const char *p);
static void parse_fuel(const char *p);
static void parse_throttle(const char *p);
static void parse_intake(const char *p);

struct PollItem
{
    const char *cmd;                  // ELM command / UDS request to send
    void (*parse)(const char *payload); // nullptr for AT setup commands
};

static const PollItem POLL_SEQUENCE[] = {
    {"ATSHD01637", nullptr}, // VCU
    {"ATCRA1EC6EE80", nullptr},
    {"22F40D", parse_speed},
    {"22DD01", parse_odometer},
    {"ATSHD01635", nullptr}, // BECM
    {"ATCRA1EC6AE80", nullptr},
    {"224801", parse_soc},
    {"224802", parse_hv_current},
    {"224803", parse_hv_voltage},
    {"22491B", parse_hv_temp},
    {"ATRV", parse_12v},
};
static const PollItem PETROL_POLL_SEQUENCE[] = {
    {"010C", parse_rpm},
    {"010D", parse_speed},
    {"0105", parse_coolant},
    {"0104", parse_engine_load},
    {"012F", parse_fuel},
    {"0111", parse_throttle},
    {"010F", parse_intake},
    {"ATRV", parse_12v},
};

static const uint8_t EV_POLL_COUNT = sizeof(POLL_SEQUENCE) / sizeof(POLL_SEQUENCE[0]);
static const uint8_t PETROL_POLL_COUNT = sizeof(PETROL_POLL_SEQUENCE) / sizeof(PETROL_POLL_SEQUENCE[0]);

// One-time setup after the ELM327 handshake: 29-bit CAN @ 500k, priority 0x1D.
static const char *const SETUP_COMMANDS[] = {"ATSP6", "ATH0"};

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

// Finds the positive response "62<did>" for `req` ("22<did>") in the ELM
// payload and decodes the following `n` data bytes into `out`.
// Skips line breaks and ISO-TP frame markers ("0:", "1:") between bytes.
static bool did_bytes(const char *payload, const char *req, uint8_t *out, uint8_t n)
{
    char expect[8];
    snprintf(expect, sizeof(expect), "62%s", req + 2);
    const char *p = strstr(payload, expect);
    if (!p)
        return false;
    p += strlen(expect);

    uint8_t i = 0;
    while (i < n)
    {
        int hi;
        while (*p && (hi = hex_nibble(*p)) < 0)
            p++;
        if (!*p)
            return false;
        // A digit followed by ':' is a multi-frame line index — skip it.
        if (p[1] == ':')
        {
            p += 2;
            continue;
        }
        int lo = hex_nibble(p[1]);
        if (lo < 0)
            return false;
        out[i++] = (uint8_t)((hi << 4) | lo);
        p += 2;
    }
    return true;
}

static bool pid_bytes(const char *payload, const char *pid, uint8_t *out, uint8_t n)
{
    char expect[8];
    snprintf(expect, sizeof(expect), "41%s", pid);
    const char *p = strstr(payload, expect);
    if (!p)
        return false;
    p += strlen(expect);
    for (uint8_t i = 0; i < n; i++)
    {
        int hi;
        while (*p && (hi = hex_nibble(*p)) < 0)
            p++;
        if (!*p)
            return false;
        int lo = hex_nibble(p[1]);
        if (lo < 0)
            return false;
        out[i] = (uint8_t)((hi << 4) | lo);
        p += 2;
    }
    return true;
}

static void parse_speed(const char *p)
{
    uint8_t b[1];
    if (storage_get_electric_mode())
        s_data.speedKph = did_bytes(p, "22F40D", b, 1) ? b[0] : NAN;
    else
        s_data.speedKph = pid_bytes(p, "0D", b, 1) ? b[0] : NAN;
}

static void parse_rpm(const char *p)
{
    uint8_t b[2];
    s_data.rpm = pid_bytes(p, "0C", b, 2) ? ((b[0] * 256.0f) + b[1]) / 4.0f : NAN;
}

static void parse_coolant(const char *p)
{
    uint8_t b[1];
    s_data.coolantC = pid_bytes(p, "05", b, 1) ? (float)b[0] - 40.0f : NAN;
}

static void parse_engine_load(const char *p)
{
    uint8_t b[1];
    s_data.engineLoadPct = pid_bytes(p, "04", b, 1) ? b[0] * 100.0f / 255.0f : NAN;
}

static void parse_fuel(const char *p)
{
    uint8_t b[1];
    s_data.fuelPct = pid_bytes(p, "2F", b, 1) ? b[0] * 100.0f / 255.0f : NAN;
}

static void parse_throttle(const char *p)
{
    uint8_t b[1];
    s_data.throttlePct = pid_bytes(p, "11", b, 1) ? b[0] * 100.0f / 255.0f : NAN;
}

static void parse_intake(const char *p)
{
    uint8_t b[1];
    s_data.intakeC = pid_bytes(p, "0F", b, 1) ? (float)b[0] - 40.0f : NAN;
}

static void parse_odometer(const char *p)
{
    uint8_t b[3];
    s_data.odometerKm = did_bytes(p, "22DD01", b, 3)
                            ? (float)(((uint32_t)b[0] << 16) | (b[1] << 8) | b[2])
                            : NAN;
}

static void parse_soc(const char *p)
{
    uint8_t b[2];
    s_data.socPct = did_bytes(p, "224801", b, 2) ? ((b[0] << 8) | b[1]) / 500.0f : NAN;
}

static void parse_hv_current(const char *p)
{
    uint8_t b[2];
    s_data.hvCurrentA = did_bytes(p, "224802", b, 2) ? (int16_t)((b[0] << 8) | b[1]) / 10.0f : NAN;
}

static void parse_hv_voltage(const char *p)
{
    uint8_t b[2];
    s_data.hvVoltage = did_bytes(p, "224803", b, 2) ? (float)((b[0] << 8) | b[1]) : NAN;
}

static void parse_hv_temp(const char *p)
{
    uint8_t b[2];
    s_data.hvBattTempC = did_bytes(p, "22491B", b, 2) ? ((b[0] << 8) | b[1]) / 100.0f - 50.0f : NAN;
}

static void parse_12v(const char *p)
{
    // Payload looks like "14.7V" (or "ATRV14.7V" if echo is on)
    if (strncmp(p, "ATRV", 4) == 0)
        p += 4;
    float v = strtof(p, nullptr);
    s_data.batteryV = (v > 0) ? v : NAN;
}

// Sequence index for round-robin polling
static uint8_t s_pollStep = 0;
static bool s_pollRunning = false;
static bool s_cmdSent = false;

// Sends the one-time protocol setup. Blocking; called once per connection.
static void volvo_setup()
{
    for (const char *cmd : SETUP_COMMANDS)
    {
        int8_t st = myELM327.sendCommand_Blocking(cmd);
        Serial.printf("[OBD] %s -> %s (state=%d)\n", cmd, myELM327.payload, (int)st);
    }

    // One-shot DID sweep with headers on: log every reply that isn't NO DATA
    myELM327.sendCommand_Blocking("ATH1");
    myELM327.sendCommand_Blocking("ATAR");
    myELM327.sendCommand_Blocking("ATSH7E0");
    static const uint16_t RANGES[][2] = {{0x4800, 0x48FF}, {0x4900, 0x49FF}, {0xDD00, 0xDDFF}, {0xF400, 0xF4FF}};
    for (const auto &r : RANGES)
    {
        for (uint32_t did = r[0]; did <= r[1]; did++)
        {
            char cmd[8];
            snprintf(cmd, sizeof(cmd), "22%04X", (unsigned)did);
            int8_t st = myELM327.sendCommand_Blocking(cmd);
            if (st != ELM_NO_DATA)
                Serial.printf("[SWEEP] %s -> '%s' (state=%d)\n", cmd, myELM327.payload, (int)st);
            delay(1);
        }
    }
    Serial.println("[SWEEP] done");
    myELM327.sendCommand_Blocking("ATH0");
}

static void do_poll_step()
{
    if (!s_pollRunning)
        return;

    const bool electric = storage_get_electric_mode();
    const PollItem &item = electric ? POLL_SEQUENCE[s_pollStep] : PETROL_POLL_SEQUENCE[s_pollStep];
    const uint8_t pollCount = electric ? EV_POLL_COUNT : PETROL_POLL_COUNT;

    if (!s_cmdSent)
    {
        s_lastCommand = item.cmd;
        myELM327.sendCommand(item.cmd);
        s_cmdSent = true;
        return;
    }

    // Drain everything available so a response isn't throttled to 1 char per loop()
    int8_t st;
    do
    {
        st = myELM327.get_response();
    } while (st == ELM_GETTING_MSG && SerialBT.available());

    if (st == ELM_GETTING_MSG)
        return; // still waiting

    if (item.parse)
    {
        if (st != ELM_SUCCESS)
        {
            Serial.printf("[OBD] %s raw error reply: '%s' (state=%d)\n", item.cmd, myELM327.payload, (int)st);
            myELM327.payload[0] = '\0'; // parsers will set NAN
        }
        item.parse(myELM327.payload);
        s_lastResponse = myELM327.payload;
        if (s_lastResponse.length() > 96)
            s_lastResponse.remove(96);
        if (st == ELM_SUCCESS && myELM327.payload[0] != '\0')
            s_successfulResponses++;
        else
            s_failedResponses++;
        if (st != ELM_SUCCESS || !strstr(myELM327.payload, "62"))
        {
            Serial.printf("[OBD] %s -> '%s' (state=%d)\n", item.cmd, myELM327.payload, (int)st);
        }
    }

    s_cmdSent = false;
    if (++s_pollStep < pollCount)
        return;

    // Completed one full round
    s_pollStep = 0;
    s_pollRunning = false;
    s_data.hvPowerKw = s_data.hvVoltage * s_data.hvCurrentA / 1000.0f; // NAN propagates
    s_data.lastUpdateMs = millis();
    Serial.printf("[OBD] Speed:%.0f SoC:%.1f HV:%.0fV %.1fA %.1fkW BattT:%.1f Odo:%.0f 12V:%.2f\n",
                  s_data.speedKph, s_data.socPct, s_data.hvVoltage, s_data.hvCurrentA,
                  s_data.hvPowerKw, s_data.hvBattTempC, s_data.odometerKm, s_data.batteryV);
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

    // Init BT in master mode, Classic only (releases BLE controller memory)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    SerialBT.begin(BT_DEVICE_NAME, true, true);
#else
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    SerialBT.begin(BT_DEVICE_NAME, true);
#endif

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

#if ESP_ARDUINO_VERSION_MAJOR >= 3
        SerialBT.setPin(BT_OBD_PIN, sizeof(BT_OBD_PIN) - 1);
#else
        SerialBT.setPin(BT_OBD_PIN);
#endif
        bool ok = SerialBT.connect(mac);

        if (ok && SerialBT.connected())
        {
            Serial.println("[BT] SPP connected!");
            s_state = BT_OBD_INIT;
            s_pollStep = 0;
            s_pollRunning = false;
            s_cmdSent = false;
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
        // Protocol '7' = ISO 15765-4 CAN, 29-bit ID, 500 kbaud. Selecting it
        // explicitly skips the auto-search, which needs a Mode 01 reply the
        // EX30 may not give.
        const char protocol = '6';
        if (myELM327.begin(SerialBT, false, OBD_TIMEOUT_MS, protocol))
        {
            Serial.println("[OBD] ELM327 handshake OK");
            if (storage_get_electric_mode())
                volvo_setup();
            else
            {
                int8_t status = myELM327.sendCommand_Blocking("ATH0");
                Serial.printf("[OBD] ATH0 -> %s (state=%d)\n", myELM327.payload, (int)status);
            }
            s_state = BT_POLLING;
            s_lastPollMs = millis() - OBD_POLL_INTERVAL_MS; // poll immediately
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
        if (!s_pollRunning && now - s_lastPollMs >= OBD_POLL_INTERVAL_MS)
        {
            s_lastPollMs = now;
            s_pollRunning = true; // start a fresh round
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

OBDDiagnostics bt_obd_get_diagnostics()
{
    OBDDiagnostics diagnostics;
    switch (s_state)
    {
    case BT_NO_MAC: diagnostics.state = "No paired adapter"; break;
    case BT_IDLE: diagnostics.state = "Waiting to connect"; break;
    case BT_CONNECTING: diagnostics.state = "Connecting to adapter"; break;
    case BT_OBD_INIT: diagnostics.state = "Initializing ELM327"; break;
    case BT_POLLING: diagnostics.state = "Polling OBD-II"; break;
    case BT_SCANNING: diagnostics.state = "Bluetooth scan in progress"; break;
    }
    diagnostics.lastCommand = s_lastCommand;
    diagnostics.lastResponse = s_lastResponse;
    diagnostics.successfulResponses = s_successfulResponses;
    diagnostics.failedResponses = s_failedResponses;
    return diagnostics;
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

void bt_clear_target_mac()
{
    storage_clear_mac();
    s_targetMac = "";
    bt_obd_force_reconnect();
    s_state = BT_NO_MAC;
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
    s_pollRunning = false;
    s_cmdSent = false;
    s_pollStep = 0;
    s_lastAttemptMs = 0;
}
