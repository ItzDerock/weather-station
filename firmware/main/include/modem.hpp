#pragma once
#include "XPowersLib.h"
#include "include/sensors.hpp"
#include "sim7080g_driver_esp_idf.h"

void modem_init_power(XPowersAXP2101 &pmu);
esp_err_t modem_init(XPowersAXP2101 &pmu, sim7080g_handle_t *sim7080g_handle);
esp_err_t mqtt_send_data(sim7080g_handle_t *handle, sensor_data_t &data);
