#pragma once
#include <Arduino.h>

class Anemometer {
public:
  Anemometer(uint8_t pin);
  ~Anemometer();

  /**
   * Sets up the pins for the anemometer and attaches the interrupt handler.
   */
  void begin();

  /**
   * Updates the wind speed based on the number of pulses detected.
   * Should be called periodically (e.g., every second) to calculate the wind
   * speed.
   */
  void update();

  float getWindSpeedMPH() const;
  float getWindSpeedKmh() const;

private:
  uint8_t _pin;

  // Counter for pulses detected by the ISR (volatile!)
  volatile unsigned long _pulseCount;

  // Timestamp of the last time wind speed was calculated
  unsigned long _lastReadTime;

  // Calculated wind speed
  float _windSpeedMPH;

  // Conversion factor: 1 Hz = 1.492 MPH
  static constexpr float ANEMOMETER_MPH_PER_HZ = 1.492;
  // Conversion factor: 1 Hz = 2.4 km/h
  static constexpr float ANEMOMETER_KMH_PER_HZ = 2.4;

  // --- Static members for ISR handling ---

  static Anemometer *_instance;

#ifdef ESP32
  static void IRAM_ATTR pulseISR();
#else
  static void pulseISR();
#endif
};
