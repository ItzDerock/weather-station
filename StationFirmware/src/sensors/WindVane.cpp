#include "include/sensors/WindVane.h"
#include <limits>

// Define the lookup table using resistance values from the datasheet
// Resistance (Ohms), Direction (Degrees)
const WindVane::DirectionLookup WindVane::_lookupTable[] = {
    {33000, 0.0},    {6570, 22.5},   {8200, 45.0},   {891, 67.5},
    {1000, 90.0},    {688, 112.5},   {2200, 135.0},  {1410, 157.5},
    {3900, 180.0},   {3140, 202.5},  {16000, 225.0}, {14120, 247.5},
    {120000, 270.0}, {42120, 292.5}, {64900, 315.0}, {21880, 337.5}};

// Calculate the size of the lookup table automatically
const int WindVane::_lookupTableSize =
    sizeof(WindVane::_lookupTable) / sizeof(WindVane::_lookupTable[0]);

WindVane::WindVane(uint8_t adcPin, float fixedResistorOhms,
                   float adcReferenceVoltage, uint8_t adcResolutionBits)
    : _adcPin(adcPin), _fixedResistorOhms(fixedResistorOhms),
      _adcReferenceVoltage(adcReferenceVoltage), _lastDirectionDegrees(0.0),
      _lastVoltage(0.0), _lastResistanceOhms(0.0), _lastRawADC(0) {
  // Calculate the maximum ADC reading based on resolution (2^bits - 1)
  _adcMaxReading = (1 << adcResolutionBits) - 1;
}

void WindVane::begin() {
  // Set pin mode for analog input (often optional, but good practice)
  pinMode(_adcPin, INPUT);

// For ESP32, you might want to set attenuation for full range reading
#ifdef ESP32
  // Example: Set attenuation to 11dB for 0-3.3V range on most ESP32 pins
  // Adjust if using different attenuation or ADC pin characteristics
  analogSetPinAttenuation(_adcPin, ADC_11db);
#endif
}

void WindVane::update() {
  // Read raw ADC value
  _lastRawADC = analogRead(_adcPin);

  // Convert raw ADC value to voltage
  // Ensure floating point division
  _lastVoltage =
      (float)_lastRawADC / (float)_adcMaxReading * _adcReferenceVoltage;

  // Calculate the sensor's resistance (Rs) based on the measured voltage
  _lastResistanceOhms = calculateSensorResistance(_lastVoltage);

  // Find the closest match in the lookup table based on resistance
  float minDifference = std::numeric_limits<float>::max();
  float bestDirection = 0.0;

  // Handle cases where calculated resistance might be invalid (e.g., if
  // voltage is too close to reference voltage, leading to negative resistance)
  if (_lastResistanceOhms < 0) {
    // Could indicate an open circuit or voltage near/at reference.
    // Assign a default or error value. Let's check the highest resistance.
    if (_lastVoltage >=
        _adcReferenceVoltage * 0.99) { // If voltage is very high
      bestDirection = 270.0; // Corresponds to 120k (highest resistance)
    } else {
      bestDirection = -1.0; // Indicate an error/undefined state
    }
  } else {
    // Iterate through the lookup table to find the closest resistance match
    for (int i = 0; i < _lookupTableSize; ++i) {
      float difference =
          abs(_lookupTable[i].resistanceOhms - _lastResistanceOhms);
      if (difference < minDifference) {
        minDifference = difference;
        bestDirection = _lookupTable[i].degrees;
      }
    }
  }

  _lastDirectionDegrees = bestDirection;
}

float WindVane::calculateSensorResistance(float voltage) const {
  // Prevent division by zero or negative results if voltage equals or exceeds
  // reference
  if (voltage >= _adcReferenceVoltage) {
    // This implies Rs is extremely high or infinite (open circuit)
    // Return a very large value or handle as an error/specific case
    return std::numeric_limits<float>::max(); // Indicate near infinite
                                              // resistance
  }
  if (voltage <= 0) {
    // This implies Rs is zero (short circuit)
    return 0.0;
  }

  // Calculate Rs using the voltage divider formula rearranged:
  // Rs = R_fixed * Vout / (Vref - Vout)
  return _fixedResistorOhms * (voltage / (_adcReferenceVoltage - voltage));
}

float WindVane::getDirectionDegrees() const { return _lastDirectionDegrees; }
float WindVane::getLastVoltage() const { return _lastVoltage; }
float WindVane::getLastResistanceOhms() const { return _lastResistanceOhms; }
int WindVane::getLastRawADC() const { return _lastRawADC; }
