#pragma once
#include <Arduino.h>

class WindVane {
public:
  /** Constructor
   * @param adcPin: The analog input pin connected to the voltage divider
   *                output.
   * @param fixedResistorOhms: The value (in Ohms) of the fixed resistor (R)
   *                           used in the voltage divider (e.g., 10000.0 for
   *                           10k).
   * @param adcReferenceVoltage: The voltage used by the ADC as its full-scale
   *                             reference (e.g., 3.3 for ESP32, 5.0 for Uno).
   * @param adcResolutionBits: The resolution of the ADC in bits (e.g., 12 for
   *                           ESP32 default, 10 for Arduino Uno/Nano).
   */
  WindVane(uint8_t adcPin, float fixedResistorOhms, float adcReferenceVoltage,
           uint8_t adcResolutionBits = 10 // Default to Arduino Uno/Nano 10-bit
  );

  /**
   * Sets the pinmodes
   */
  void begin();

  /**
   * Re-reads the position from the ADC
   */
  void update();

  /**
   * Returns the last read direction in degrees (0-359.9).
   */
  float getDirectionDegrees() const;

  /**
   * Returns the last measured voltage in Volts.
   */
  float getLastVoltage() const;

  /**
   * Returns the last calculated resistance in Ohms.
   */
  float getLastResistanceOhms() const;

  /**
   * Returns the raw ADC value (ADC=analog digital converter).
   */
  int getLastRawADC() const;

private:
  // Configuration
  uint8_t _adcPin;
  float _fixedResistorOhms;
  float _adcReferenceVoltage;
  int _adcMaxReading;

  // State variables
  float _lastDirectionDegrees;
  float _lastVoltage;
  float _lastResistanceOhms;
  int _lastRawADC;

  // Structure to hold lookup table entries (Resistance -> Degrees)
  struct DirectionLookup {
    float resistanceOhms;
    float degrees;
  };

  // Lookup table based on datasheet resistance values
  static const DirectionLookup _lookupTable[];
  static const int _lookupTableSize;

  // Helper to calculate resistance from voltage
  float calculateSensorResistance(float voltage) const;
};
