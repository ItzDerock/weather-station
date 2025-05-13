#pragma once

#include "argentdata.h"
#include <optional>

struct sensor_data_t {
  std::optional<float> pressure;
  std::optional<float> temperature;
  std::optional<float> humidity;
  std::optional<float> uv_index;

  ArgentSensorData *argent;
};
