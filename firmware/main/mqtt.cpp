#include "cJSON.h"
#include "config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "include/sensors.hpp"
#include "sim7080g_driver_esp_idf.h"
#include <ctime>
#include <string.h>

static const char *TAG = "mqtt";

#define CJSON_ADD_IF_OPTION_NOT_NULL(obj, key, opt_value)                      \
  if ((opt_value).has_value()) {                                               \
    cJSON_AddNumberToObject(obj, key, (*(opt_value)));                         \
  }

static cJSON *mqtt_build_payload(sensor_data_t &data, time_t &time) {
  cJSON *root = cJSON_CreateObject();

  if (root == nullptr) {
    ESP_LOGE(TAG, "Failed to create JSON object");
    return nullptr;
  }

  cJSON_AddNumberToObject(root, "wind_speed", data.argent->wind_speed);
  cJSON_AddNumberToObject(root, "wind_gust", data.argent->wind_speed_gust);

  // TODO: send all degrees
  cJSON_AddNumberToObject(root, "wind_dir", data.argent->degrees[0]);
  cJSON_AddNumberToObject(root, "rain", data.argent->rainfall);

  CJSON_ADD_IF_OPTION_NOT_NULL(root, "temp", data.temperature);
  CJSON_ADD_IF_OPTION_NOT_NULL(root, "humidity", data.humidity);
  CJSON_ADD_IF_OPTION_NOT_NULL(root, "pressure", data.pressure);
  CJSON_ADD_IF_OPTION_NOT_NULL(root, "uv_index", data.uv_index);
  CJSON_ADD_IF_OPTION_NOT_NULL(root, "lux", data.ambient_light);
  CJSON_ADD_IF_OPTION_NOT_NULL(root, "bat_volt", data.battery_voltage);
  CJSON_ADD_IF_OPTION_NOT_NULL(root, "bat_level", data.battery_est_percentage);
  CJSON_ADD_IF_OPTION_NOT_NULL(root, "rssi", data.rssi);
  CJSON_ADD_IF_OPTION_NOT_NULL(root, "ber", data.ber);

  cJSON_AddNumberToObject(root, "dateTime", (double)time);

  return root;
}

esp_err_t mqtt_send_data(sim7080g_handle_t *handle, sensor_data_t &data) {
  // Validate arguments
  if (!handle) {
    ESP_LOGE(TAG, "mqtt_send_data: Invalid sim7080g_handle.");
    return ESP_ERR_INVALID_ARG;
  }

  // Get the current timestamp
  time_t current_epoch;
  ESP_RETURN_ON_ERROR(sim7080g_get_epoch_time_utc(handle, &current_epoch), TAG,
                      "Failed to get current epoch time");

  // Prepare the MQTT JSON payload
  cJSON *payload = mqtt_build_payload(data, current_epoch);
  if (payload == nullptr) {
    ESP_LOGE(TAG, "Failed to build JSON payload");
    return ESP_FAIL;
  }

  // Convert the payload to a string
  char *payload_str = cJSON_PrintUnformatted(payload);

  if (payload_str == nullptr) {
    ESP_LOGE(TAG, "Failed to convert JSON payload to string");
    cJSON_Delete(payload);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "MQTT Payload: %s", payload_str);

  // Publish the payload to the MQTT broker
  esp_err_t ret =
      sim7080g_mqtt_publish(handle, MQTT_TOPIC, payload_str, 0, false);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to publish MQTT message: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "MQTT message published successfully");
  }

  // Clean up
  cJSON_Delete(payload);
  cJSON_free(payload_str);

  return ret;
}
