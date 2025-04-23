#include "include/cellular.hpp"
#include "TinyGsmClientSIM70xx.h"
#include "config.h"
#include "config.h.example"
#include "include/power.hpp"
#include "include/utilities.h"
#include <stdarg.h>
#include <stdio.h>

enum {
  MODEM_CATM = 1,
  MODEM_NB_IOT,
  MODEM_CATM_NBIOT,
};

// set up the modem
#ifndef DUMP_AT_COMMANDS
#include <include/StreamDebugger.h>
StreamDebugger debugger(Serial1, Serial);
TinyGsm cellular::modem(debugger);
#else
TinyGsm cellular::modem(SerialAT);
#endif

char buffer[1024] = {0};

// "frontend" for the modem status
const char *register_info[] = {
    "Not registered, MT is not currently searching an operator to register to. "
    "The GPRS service is disabled, the UE is allowed to attach for GPRS if "
    "requested by the user.",
    "Registered, home network.",
    "Not registered, but MT is currently trying to attach or searching an "
    "operator to register to. The GPRS service is enabled, but an allowable "
    "PLMN is currently not available. The UE will start a GPRS attach as soon "
    "as an allowable PLMN is available.",
    "Registration denied, the GPRS service is disabled, the UE is not allowed "
    "to attach for GPRS if it is requested by the user.",
    "Unknown.",
    "Registered, roaming.",
};

bool cellular::isConnected() {
  cellular::modem.sendAT("+SMSTATE?");

  if (cellular::modem.waitResponse("+SMSTATE: ")) {
    String res = cellular::modem.stream.readStringUntil('\r');
    return res.toInt();
  }

  return false;
}

int cellular::initialize() {
  Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);

  pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
  pinMode(BOARD_MODEM_DTR_PIN, OUTPUT);
  pinMode(BOARD_MODEM_RI_PIN, INPUT);

  // PWRKEY needs to be pressed for 1 second to start the modem
  int retry = 0;
  while (!cellular::modem.testAT(1000)) {
    Serial.print(".");
    if (retry++ > 6) {
      // Pull down PWRKEY for more than 1 second according to manual
      // requirements
      digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
      delay(1000);
      digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
      retry = 0;
      Serial.println("Retrying modem initialization...");
    }
  }

  Serial.println();
  Serial.print("Modem started!");

  if (cellular::modem.getSimStatus() != SIM_READY) {
    Serial.print("[fatal] Sim card is not inserted!");
    return cellular::CellError::NO_SIM_CARD;
  }

  // set network mode to NB-IoT
  cellular::modem.setNetworkMode(2); // auto
  cellular::modem.setPreferredMode(MODEM_NB_IOT);

  uint8_t pre = cellular::modem.getPreferredMode();
  uint8_t mode = cellular::modem.getNetworkMode();

  Serial.printf("getNetworkMode:%u getPreferredMode:%u\n", mode, pre);
  return 0;
}

bool cellular::isRegistered() {
  SIM70xxRegStatus status = cellular::modem.getRegistrationStatus();
  if (status == REG_OK_HOME || status == REG_OK_ROAMING) {
    return true;
  } else {
    Serial.print("Network register info:");
    Serial.println(register_info[status]);
    return false;
  }
}

int cellular::initializeAPN() {
  // attempt to connect GPRS
  Serial.print("Connecting to GPRS...");
  if (!cellular::modem.isGprsConnected()) {
    cellular::modem.sendAT("+CNACT=0,1");
    if (cellular::modem.waitResponse() != 1) {
      Serial.println("Failed to activate network bearer!");
      return cellular::CellError::BEARER_NOT_ACTIVATED;
    }
  }

  // Set APN
  if (!cellular::modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    Serial.println("Failed to connect to GPRS");
    return cellular::CellError::GPRS_NOT_CONNECTED;
  }

  // Check if GPRS is connected
  if (!cellular::modem.isGprsConnected()) {
    Serial.println("GPRS not connected!");
    return cellular::CellError::GPRS_NOT_CONNECTED;
  }

  Serial.println("GPRS connected!");
  return 0;
}

int cellular::sendATCommand(TinyGsm &modem, char *buffer, size_t bufferSize,
                            uint32_t timeout, const char *format, ...) {
  va_list args;
  va_start(args, format);

  // Use vsnprintf to format the command safely into the buffer
  int ret = vsnprintf(buffer, bufferSize, format, args);

  va_end(args);

  // Check for formatting errors or buffer overflow
  if (ret < 0 || (size_t)ret >= bufferSize) {
    Serial.println("ERROR: AT command formatting failed or buffer too small.");
    // Optional: Print format string for debugging
    // Serial.print("Format: "); Serial.println(format);
    return AT_CMD_FORMAT_ERROR; // Indicate a formatting/buffer error
  }

  // Send the formatted command
  // Assuming sendAT takes const char* - adjust if it needs char*
  modem.sendAT(buffer);

  // Wait for the response with the specified timeout
  int response = modem.waitResponse(timeout);

  return response;
}

// Implementation of the simple command overload
// It simply calls the main variadic version.
int sendATCommand(TinyGsm &modem, char *buffer, size_t bufferSize,
                  uint32_t timeout, const char *command) {
  // Call the main function, passing the command as the format string
  // with no additional arguments needed for formatting.
  return sendATCommand(modem, buffer, bufferSize, timeout, command);
}

int cellular::connectMQTT() {
  // Connect to MQTT broker
  Serial.print("Connecting to MQTT...");

  // Disconnect if already connected
  cellular::sendATCommand(modem, buffer, sizeof(buffer), MODEM_DEFAULT_TIMEOUT,
                          "+SMDISC");

  // Configure MQTT parameters using the utility function
  if (cellular::sendATCommand(
          modem, buffer, sizeof(buffer), MODEM_DEFAULT_TIMEOUT,
          "+SMCONF=\"URL\",\"%s\",%d", MQTT_HOST, MQTT_PORT) != 1) {
    Serial.println("ERROR: Failed to set MQTT URL");
    return -1;
  }

  if (cellular::sendATCommand(modem, buffer, sizeof(buffer),
                              MODEM_DEFAULT_TIMEOUT,
                              "+SMCONF=\"USERNAME\",\"%s\"", MQTT_USER) != 1) {
    Serial.println("ERROR: Failed to set MQTT Username");
    return -2;
  }

  if (cellular::sendATCommand(modem, buffer, sizeof(buffer),
                              MODEM_DEFAULT_TIMEOUT,
                              "+SMCONF=\"PASSWORD\",\"%s\"", MQTT_PASS) != 1) {
    Serial.println("ERROR: Failed to set MQTT Password");
    return -3;
  }

  if (cellular::sendATCommand(
          modem, buffer, sizeof(buffer), MODEM_DEFAULT_TIMEOUT,
          "+SMCONF=\"CLIENTID\",\"%s\"", MQTT_CLIENT_ID) != 1) {
    Serial.println("ERROR: Failed to set MQTT ClientID");
    return -4;
  }

  // Connect to MQTT Broker with retry
  Serial.println("Connecting to MQTT Broker...");
  int8_t ret;
  int retryCount = 0;
  const int maxRetries = 5;
  do {
    // Use a longer timeout for connection (e.g., 30 seconds)
    ret = cellular::sendATCommand(modem, buffer, sizeof(buffer), 30000,
                                  "+SMCONN");
    if (ret != 1) {
      retryCount++;
      Serial.printf("Connect failed (attempt %d/%d), retrying in 5s...\n",
                    retryCount, maxRetries);
      // Optional: Check specific error codes from 'ret' if needed
      delay(5000); // Wait longer before retry
    }
    if (retryCount >= maxRetries) {
      Serial.println("ERROR: Max MQTT connection retries reached.");
      // TODO: Handle persistent failure (e.g., restart, sleep)
      return -5;
    }
  } while (ret != 1);

  return 0;
}
