#include "XPowersLib.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log_level.h"
#include "esp_sleep.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/uart_types.h"
#include "portmacro.h"
#include "sim7080g_driver_esp_idf.h"
#include "soc/clk_tree_defs.h"
#include <string.h>

#define BOARD_MODEM_PWR_PIN GPIO_NUM_41
#define BOARD_MODEM_DTR_PIN GPIO_NUM_42
#define BOARD_MODEM_RI_PIN GPIO_NUM_3
#define BOARD_MODEM_RXD_PIN GPIO_NUM_4
#define BOARD_MODEM_TXD_PIN GPIO_NUM_5

#define NETWORK_CONNECT_RETRY_DELAY_MS 20000
#define CFUN_CYCLE_RETRY_THRESHOLD 2
#define POST_CFUN_DELAY_MS 5000

static const char *TAG = "modem";
static const char *TAG_CONNECT = "modem:connect";

// forward declarations
static esp_err_t modem_basic_at_test(sim7080g_handle_t *handle,
                                     gpio_num_t pwr_key_pin, int test_retries,
                                     bool pwr_cycle_on_fail);

static esp_err_t try_connect_network(sim7080g_handle_t *handle, int attempts,
                                     const char *apn);

static esp_err_t wait_physical_layer(sim7080g_handle_t *&sim7080g_handle,
                                     int attempts = 10);

static esp_err_t modem_init_tls(sim7080g_handle_t *handle);

static esp_err_t modem_init_gpio();

void modem_init_power(XPowersAXP2101 &pmu) {
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    // power cycle the modem on power cycles
    pmu.disableDC3();
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  // set working voltage for modem
  pmu.setDC3Voltage(3'000);
  pmu.enableDC3();

  // power for the GPS
  pmu.setBLDO2Voltage(3'300);
  pmu.enableBLDO2();

  // disable ts pin detection to allow for charging
  pmu.disableTSPinMeasure();
}

esp_err_t modem_init(XPowersAXP2101 &pmu, sim7080g_handle_t *sim7080g_handle) {
  modem_init_power(pmu);
  modem_init_gpio();

  // Pull PWRKEY for 1 second to start modem
  ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_MODEM_PWR_PIN, 0), TAG,
                      "Failed to set pin %d low", BOARD_MODEM_PWR_PIN);
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_MODEM_PWR_PIN, 1), TAG,
                      "Failed to set pin %d high", BOARD_MODEM_PWR_PIN);
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_MODEM_PWR_PIN, 0), TAG,
                      "Failed to set pin %d low", BOARD_MODEM_PWR_PIN);

  // Modem exists via UART
  // so set up config
  sim7080g_uart_config_t uart_config = {.gpio_num_tx = BOARD_MODEM_TXD_PIN,
                                        .gpio_num_rx = BOARD_MODEM_RXD_PIN,
                                        .port_num = UART_NUM_1};

  sim7080g_mqtt_config_t mqtt_config = {.broker_url = MQTT_BROKER_URL,
                                        .username = MQTT_USERNAME,
                                        .client_id = MQTT_CLIENT_ID,
                                        .client_password = MQTT_CLIENT_PASSWORD,
                                        .port = MQTT_PORT,
                                        .use_tls = USE_MQTTS,
                                        .ssl_context_index = 0,
                                        .ca_cert_filename_on_modem =
                                            "mqtts_ca.pem"};

  ESP_LOGI(TAG, "Configuring SIM7080G");
  ESP_RETURN_ON_ERROR(
      sim7080g_config(sim7080g_handle, uart_config, mqtt_config), TAG,
      "Failed to configure modem handle");

  ESP_LOGI(TAG, "Power cycling modem until connected...");
  ESP_RETURN_ON_ERROR(
      modem_basic_at_test(sim7080g_handle, BOARD_MODEM_PWR_PIN, 5, true), TAG,
      "Failed to power cycle modem");

  ESP_LOGI(TAG, "Initializing modem handle...");
  ESP_RETURN_ON_ERROR(sim7080g_init(sim7080g_handle), TAG,
                      "Failed to initialize modem handle");

  // Set up SSL
  ESP_LOGI(TAG, "Configuring TLS for MQTTS...");
  esp_err_t tls_err = modem_init_tls(sim7080g_handle);
  if (tls_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure MQTTS SSL: %s",
             esp_err_to_name(tls_err));

    if (sim7080g_handle != nullptr)
      sim7080g_handle->mqtt_config.use_tls = false; // Disable TLS if failed
  }

  // Attempt to connect to the network
  int network_connection_attempts = 5;
  esp_err_t network_connect_status =
      try_connect_network(sim7080g_handle, network_connection_attempts, APN);

  if (network_connect_status != ESP_OK) {
    ESP_LOGE(TAG,
             "modem_init: Failed to connect to network after all attempts.");
    return network_connect_status;
  }

  // confirm physical layer connection
  ESP_RETURN_ON_ERROR(wait_physical_layer(sim7080g_handle), TAG,
                      "Failed to confirm physical layer connection");

  // confirm application layer connection
  bool is_data_link_connected = false;
  esp_err_t data_link_check = sim7080g_is_data_link_layer_connected(
      sim7080g_handle, &is_data_link_connected);
  if (data_link_check != ESP_OK || !is_data_link_connected) {
    ESP_LOGE(TAG, "modem_init: Data link layer not connected even after "
                  "successful bearer connection report.");
    return (data_link_check != ESP_OK) ? data_link_check : ESP_FAIL;
  }
  ESP_LOGI(TAG, "modem_init: Data link layer connected.");

  // connect to broker
  ESP_RETURN_ON_ERROR(sim7080g_mqtt_connect_to_broker(sim7080g_handle), TAG,
                      "Failed to connect to MQTT broker");

  return ESP_OK;
}

#define MODEM_AT_CMD_RETRY_DELAY_MS 1000
#define MODEM_POST_PWRKEY_BOOT_DELAY_MS 3000

/**
 * @brief Performs a basic AT command test to check modem responsiveness using
 * UART config from handle.
 *
 * @param handle Pointer to the sim7080g_handle_t struct, which must have
 * uart_config populated.
 * @param pwr_key_pin GPIO number for modem PWRKEY.
 * @param test_retries Number of times to retry the AT command.
 * @param pwr_cycle_on_fail If true, toggle PWRKEY on failed retry sequence.
 * @return esp_err_t ESP_OK on success, ESP_FAIL or other error on failure.
 */
static esp_err_t modem_basic_at_test(sim7080g_handle_t *handle,
                                     gpio_num_t pwr_key_pin, int test_retries,
                                     bool pwr_cycle_on_fail) {
  esp_err_t ret = ESP_OK;           // Declare ret for ESP_GOTO_ON_ERROR
  esp_err_t install_err = ESP_FAIL; // Track UART installation status

  if (!handle) {
    ESP_LOGE(TAG, "Basic AT Test: Invalid sim7080g_handle.");
    return ESP_ERR_INVALID_ARG;
  }

  // Extract UART parameters from the handle
  uart_port_t uart_num = (uart_port_t)handle->uart_config.port_num;
  gpio_num_t tx_pin =
      (gpio_num_t)handle->uart_config.gpio_num_tx; // Cast to gpio_num_t
  gpio_num_t rx_pin =
      (gpio_num_t)handle->uart_config.gpio_num_rx; // Cast to gpio_num_t
  int baud_rate = SIM7080G_UART_BAUD_RATE;

  // Declare variables before potential gotos
  char response_buffer[SIM87080G_UART_BUFF_SIZE];
  const char *at_cmd = "AT\r\n";

  ESP_LOGI(
      TAG,
      "Starting basic AT test for modem on UART%d (TX:%d, RX:%d, Baud:%d)...",
      (int)uart_num, (int)tx_pin, (int)rx_pin,
      baud_rate); // Cast back for logging if needed

  // Use designated initializers for uart_config_t
  uart_config_t uart_config_test = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  install_err = uart_driver_install(uart_num, SIM87080G_UART_BUFF_SIZE * 2, 0,
                                    0, nullptr, 0);
  if (install_err != ESP_OK) {
    if (install_err == ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG,
               "UART%d driver already installed. Using existing instance for "
               "AT test.",
               (int)uart_num);
      // If already installed, don't try to delete it later
      install_err = ESP_FAIL; // Mark so we don't delete later
    } else {
      ESP_LOGE(TAG, "Failed to install UART%d driver for AT test: %s",
               (int)uart_num, esp_err_to_name(install_err));
      return install_err; // Return the actual error
    }
  }

  ESP_GOTO_ON_ERROR(uart_param_config(uart_num, &uart_config_test),
                    basic_test_cleanup_and_fail, TAG,
                    "UART param config failed for AT test");
  ESP_GOTO_ON_ERROR(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE),
                    basic_test_cleanup_and_fail, TAG,
                    "UART set pins failed for AT test");

  for (int i = 0; i < test_retries; ++i) {
    ESP_LOGI(TAG, "Basic AT Test: Attempt %d/%d", i + 1, test_retries);
    uart_flush_input(uart_num);
    int written = uart_write_bytes(uart_num, at_cmd, strlen(at_cmd));
    if (written < (int)strlen(at_cmd)) {
      ESP_LOGE(TAG, "Basic AT Test: UART write failed or incomplete.");
      vTaskDelay(pdMS_TO_TICKS(MODEM_AT_CMD_RETRY_DELAY_MS));
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    int len = uart_read_bytes(uart_num, (uint8_t *)response_buffer,
                              SIM87080G_UART_BUFF_SIZE - 1, pdMS_TO_TICKS(500));

    if (len > 0) {
      response_buffer[len] = '\0';
      ESP_LOGD(TAG, "Basic AT Test: Raw Response: %s",
               response_buffer); // Use Debug level for raw output
      if (strstr(response_buffer, "OK") != NULL) {
        ESP_LOGI(TAG, "Basic AT Test: PASSED!");
        if (install_err == ESP_OK)
          uart_driver_delete(uart_num); // Clean up only if we installed it
        return ESP_OK;                  // Success
      } else {
        ESP_LOGW(TAG, "Basic AT Test: 'OK' not found in response: %s",
                 response_buffer);
      }
    } else if (len == 0) {
      ESP_LOGW(TAG, "Basic AT Test: No response from modem (timeout).");
    } else {
      ESP_LOGE(TAG, "Basic AT Test: UART read error (%d).", len);
    }

    if (i < test_retries - 1) {
      vTaskDelay(pdMS_TO_TICKS(MODEM_AT_CMD_RETRY_DELAY_MS));
    }
  }

  if (pwr_cycle_on_fail) {
    ESP_LOGW(TAG,
             "Basic AT Test: Failed after %d retries. Cycling PWRKEY and "
             "trying once more.",
             test_retries);
    // Use gpio_set_level with gpio_num_t
    gpio_set_level(pwr_key_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(pwr_key_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(pwr_key_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(MODEM_POST_PWRKEY_BOOT_DELAY_MS));

    uart_flush_input(uart_num);
    uart_write_bytes(uart_num, at_cmd, strlen(at_cmd));
    vTaskDelay(pdMS_TO_TICKS(500));
    int len = uart_read_bytes(uart_num, (uint8_t *)response_buffer,
                              SIM87080G_UART_BUFF_SIZE - 1, pdMS_TO_TICKS(500));
    if (len > 0) {
      response_buffer[len] = '\0';
      ESP_LOGD(TAG, "Basic AT Test (after pwr cycle): Raw Response: %s",
               response_buffer);
      if (strstr(response_buffer, "OK") != NULL) {
        ESP_LOGI(TAG, "Basic AT Test: PASSED after power cycle!");
        if (install_err == ESP_OK)
          uart_driver_delete(uart_num);
        return ESP_OK; // Success
      }
    } else {
      ESP_LOGW(TAG, "Basic AT Test (after pwr cycle): No response.");
    }
  }

basic_test_cleanup_and_fail:
  ESP_LOGE(TAG, "Basic AT Test: FAILED definitively.");
  // Use ret which should hold the error code from ESP_GOTO_ON_ERROR or set to
  // ESP_FAIL
  if (ret == ESP_OK) {
    ret = ESP_FAIL;
  } // Ensure we return an error
  if (install_err == ESP_OK) {
    esp_err_t delete_err = uart_driver_delete(uart_num);
    if (delete_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to delete test UART driver: %s",
               esp_err_to_name(delete_err));
    }
  }
  return ret;
}

/**
 * @brief Tries to connect to the network bearer with a specified number of
 * attempts.
 *
 * This function calls sim7080g_connect_to_network_bearer and retries on
 * failure. It can optionally perform a CFUN cycle (radio reset) if multiple
 * attempts fail.
 *
 * @param handle Pointer to the initialized sim7080g_handle_t.
 * @param max_attempts Maximum number of attempts to connect to the network
 * bearer.
 * @param apn Access Point Name string.
 * @return esp_err_t ESP_OK on successful connection, or the error from the last
 * attempt.
 */
static esp_err_t try_connect_network(sim7080g_handle_t *handle,
                                     int max_attempts, const char *apn) {
  if (!handle || !apn) {
    ESP_LOGE(TAG_CONNECT, "Invalid arguments (null handle or APN).");
    return ESP_ERR_INVALID_ARG;
  }
  if (max_attempts <= 0) {
    ESP_LOGE(TAG_CONNECT, "max_attempts must be greater than 0.");
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t result = ESP_FAIL;

  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    ESP_LOGI(TAG_CONNECT,
             "Attempting to connect to network bearer (Attempt %d/%d)...",
             attempt, max_attempts);

    result = sim7080g_connect_to_network_bearer(handle, apn);

    if (result == ESP_OK) {
      ESP_LOGI(TAG_CONNECT,
               "Successfully connected to network bearer on attempt %d.",
               attempt);
      return ESP_OK; // Success
    } else {
      ESP_LOGE(TAG_CONNECT,
               "Failed to connect to network bearer (Attempt %d/%d): %s (0x%x)",
               attempt, max_attempts, esp_err_to_name(result), result);

      // Optional: CFUN cycle as a more drastic recovery after a certain number
      // of failures Ensure sim7080g_cycle_cfun is available in your driver and
      // appropriate to call.
      if (attempt % CFUN_CYCLE_RETRY_THRESHOLD == 0 && attempt < max_attempts) {
        ESP_LOGW(
            TAG_CONNECT,
            "Performing a CFUN (radio reset) cycle before next attempt...");
        esp_err_t cfun_err =
            sim7080g_cycle_cfun(handle); // Assumes this function exists
        if (cfun_err != ESP_OK) {
          ESP_LOGE(TAG_CONNECT, "CFUN cycle failed: %s. Continuing with retry.",
                   esp_err_to_name(cfun_err));
        }
        ESP_LOGI(TAG_CONNECT, "Waiting %dms after CFUN cycle...",
                 POST_CFUN_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(POST_CFUN_DELAY_MS));
      } else if (attempt < max_attempts) {
        ESP_LOGI(TAG_CONNECT, "Retrying in %d ms...",
                 NETWORK_CONNECT_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(NETWORK_CONNECT_RETRY_DELAY_MS));
      }
    }
  }

  ESP_LOGE(TAG_CONNECT,
           "Failed to connect to network bearer after %d attempts.",
           max_attempts);
  return result; // Return the error from the last attempt
}

/**
 * @brief Waits for the physical layer to be connected.
 */
static esp_err_t wait_physical_layer(sim7080g_handle_t *&sim7080g_handle,
                                     int attempts) {
  bool connected = false;
  int attempt = 0;
  esp_err_t err = ESP_OK;

  ESP_LOGI(TAG, "Waiting for physical layer connection...");

  do {
    err = sim7080g_is_physical_layer_connected(sim7080g_handle, &connected);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to check physical layer connection: %s",
               esp_err_to_name(err));
    }

    if (connected) {
      ESP_LOGI(TAG, "Physical layer connected");
      return ESP_OK;
    } else {
      ESP_LOGI(TAG, "Waiting for physical layer connection...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      attempt++;
    }
  } while (!connected && attempt < attempts);

  return err == ESP_OK ? ESP_FAIL : err;
}

static esp_err_t modem_init_gpio() {
  // init gpios
  ESP_RETURN_ON_ERROR(gpio_reset_pin(BOARD_MODEM_PWR_PIN), TAG,
                      "Failed to reset pin %d", BOARD_MODEM_PWR_PIN);
  ESP_RETURN_ON_ERROR(gpio_reset_pin(BOARD_MODEM_DTR_PIN), TAG,
                      "Failed to reset pin %d", BOARD_MODEM_DTR_PIN);
  ESP_RETURN_ON_ERROR(gpio_reset_pin(BOARD_MODEM_RI_PIN), TAG,
                      "Failed to reset pin %d", BOARD_MODEM_RI_PIN);

  ESP_RETURN_ON_ERROR(gpio_set_direction(BOARD_MODEM_PWR_PIN, GPIO_MODE_OUTPUT),
                      TAG, "Failed to set pin %d as output",
                      BOARD_MODEM_PWR_PIN);
  ESP_RETURN_ON_ERROR(gpio_set_direction(BOARD_MODEM_DTR_PIN, GPIO_MODE_OUTPUT),
                      TAG, "Failed to set pin %d as output",
                      BOARD_MODEM_DTR_PIN);
  ESP_RETURN_ON_ERROR(gpio_set_direction(BOARD_MODEM_RI_PIN, GPIO_MODE_INPUT),
                      TAG, "Failed to set pin %d as input", BOARD_MODEM_RI_PIN);

  return ESP_OK;
}

static esp_err_t modem_init_tls(sim7080g_handle_t *handle) {
#if USE_MQTTS
  ESP_LOGI(TAG, "Configuring SSL for MQTTS...");
  ESP_RETURN_ON_ERROR(sim7080g_mqtts_configure_ssl(handle, CA_PEM), TAG,
                      "Failed to configure MQTTS SSL.");
#else
  ESP_LOGI(TAG, "MQTTS disabled. Using unencrypted MQTT.");
#endif

  return ESP_OK;
}
