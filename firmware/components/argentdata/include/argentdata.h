#pragma once

// 3 seconds
#define ULP_WAKEUP_PERIOD 3000

struct ArgentSensorData {
  float wind_speed;
};

void argentdata_init_gpio(void);
void argentdata_init_ulp(void);
void argentdata_read_sensors(struct ArgentSensorData *data);
