#pragma once
#include <Arduino.h>
#include "bt_obd.h"

void     display_init();
void     display_loop(const OBDData &data, bool bt_connected, bool mqtt_connected);
void     display_set_interval(uint32_t ms);
uint32_t display_get_interval();
