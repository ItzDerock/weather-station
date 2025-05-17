#pragma once

#include "argentdata.h"
#include <optional>

struct sensor_data_t {
  std::optional<float> pressure;
  std::optional<float> temperature;
  std::optional<float> humidity;

  std::optional<float> uv_index;
  std::optional<float> ambient_light;

  // from AXP2101
  std::optional<uint16_t> battery_voltage; // mv
  std::optional<int> battery_est_percentage;

  // cellular information
  std::optional<int8_t> rssi;
  std::optional<uint8_t> ber;

  ArgentSensorData *argent;
};
