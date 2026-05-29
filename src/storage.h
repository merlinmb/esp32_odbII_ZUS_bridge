#pragma once
#include <Arduino.h>

// Persist the ZUS Bluetooth MAC address in NVS so it survives reboots.
// Call storage_init() once in setup(), then use the load/save helpers.

void   storage_init();

// Returns true and populates macOut ("AA:BB:CC:DD:EE:FF") if a MAC is saved.
bool   storage_load_mac(String &macOut);

// Saves the MAC string to NVS.
void   storage_save_mac(const String &mac);

// Erase the saved MAC (used by the web UI "forget device" button).
void   storage_clear_mac();
