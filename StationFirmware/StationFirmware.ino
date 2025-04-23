#include <Arduino.h>

#include "config.h"
#include "include/cellular.hpp"
#include "include/power.hpp"
#include "include/sensors/UVI.h"

UVI_Calculator uviSensor;
XPowersPMU PMU;
extern char buffer[1024];

#define randMax 35
#define randMin 18

int data_channel = 0;

void setup() {
  // Initialize with default Gain 3x, Res 18-bit
  if (!uviSensor.begin()) {
    Serial.println("Failed to initialize LTR390 sensor!");
  }

  Serial.begin(115200);

  // Start while waiting for Serial monitoring
  while (!Serial);

  delay(3000);

  power::initialize();
  cellular::initialize();

  // wait for cell service
  while (!cellular::isRegistered()) {
    delay(1000);
  }

  cellular::initializeAPN();
  cellular::connectMQTT();

  // random seed data
  randomSeed(esp_random());
}

void loop() {
  float uvi = uviSensor.readUVI();

  // Publish fake temperature dataA
  String payload = "temp,c=";
  int temp = rand() % (randMax - randMin + 1) + randMin;
  payload.concat(temp);
  // payload.concat("\r\n"); // Usually not needed for MQTT payload

  Serial.print("Preparing to publish: ");
  Serial.println(payload);

  // Format the SMPUB command itself using snprintf first
  // Note: Using the global buffer
  snprintf(buffer, sizeof(buffer), "+SMPUB=\"v1/%s/things/%s/data/%d\",%d,1,1",
           MQTT_USER, MQTT_CLIENT_ID, data_channel, payload.length());

  // Send the SMPUB command and wait specifically for the ">" prompt
  cellular::modem.sendAT(buffer);
  if (cellular::modem.waitResponse(">") == 1) {  // Wait for ">"
    // Send the payload data
    cellular::modem.stream.write(payload.c_str(), payload.length());
    cellular::modem.stream.flush();  // Ensure data is sent

    Serial.print("Payload sent. Waiting for publish confirmation...");

    // Wait for the final OK/ERROR response after sending payload (e.g., 5 sec
    // timeout)
    if (cellular::modem.waitResponse(5000) == 1) {  // Wait for OK
      Serial.println(" Publish Success!");
    } else {
      Serial.println(" Publish Failed!");
      // Consider logging the buffer content here to see if there was an error
      // message modem.streamReadAll() or similar might capture error details
    }
  } else {
    Serial.println("Failed to get '>' prompt for SMPUB!");
    // Log buffer content here too?
  }

  delay(60000);  // Wait 60 seconds
}
