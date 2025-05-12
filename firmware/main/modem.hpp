#pragma once
#include "XPowersLib.h"
#include "sim7080g_driver_esp_idf.h"

void modem_init_power(XPowersAXP2101 &pmu);
esp_err_t modem_init(XPowersAXP2101 &pmu, sim7080g_handle_t *sim7080g_handle);
