#include "storage.h"
#include "config.h"
#include <Preferences.h>

static Preferences prefs;

void storage_init() {
    // Open in read-write mode; creates namespace if it doesn't exist.
    prefs.begin(NVS_NAMESPACE, false);
    Serial.println("[NVS] Storage initialised");
}

bool storage_load_mac(String &macOut) {
    if (!prefs.isKey(NVS_KEY_BT_MAC)) return false;
    macOut = prefs.getString(NVS_KEY_BT_MAC, "");
    return macOut.length() == 17;   // "AA:BB:CC:DD:EE:FF" is 17 chars
}

void storage_save_mac(const String &mac) {
    prefs.putString(NVS_KEY_BT_MAC, mac);
    Serial.print("[NVS] Saved BT MAC: ");
    Serial.println(mac);
}

void storage_clear_mac() {
    prefs.remove(NVS_KEY_BT_MAC);
    Serial.println("[NVS] Cleared BT MAC");
}

uint32_t storage_get_display_interval() {
    return prefs.getUInt("disp_ms", DISPLAY_DEFAULT_INTERVAL_MS);
}

void storage_set_display_interval(uint32_t ms) {
    prefs.putUInt("disp_ms", ms);
}
