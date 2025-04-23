#pragma once
#include <Arduino.h>

class RainGauge {
public:
  RainGauge(uint8_t pin);

  /**
   * Unregisters the interrupt handler and cleans up the instance.
   */
  ~RainGauge();

  /**
   * Attaches the interrupt handler and sets up the pin.
   */
  void begin();

  float getTotalInches() const;
  float getTotalMillimeters() const;

  /**
   * Resets the total count of tips to zero.
   */
  void resetTotal();

  /**
   * Returns the number of tips detected since the last reset.
   */
  unsigned long getRawTipCount() const;

private:
  uint8_t _pin;

  // Counter for tips detected by the ISR (volatile!)
  volatile unsigned long _tipCount;

  // Conversion factor: Tips to Inches (0.011" per tip)
  static constexpr float INCHES_PER_TIP = 0.011;
  // Conversion factor: Tips to Millimeters (0.2794 mm per tip)
  static constexpr float MM_PER_TIP = 0.2794;

  // --- Static members for ISR handling ---
  static RainGauge *_instance;

#ifdef ESP32
  static void IRAM_ATTR tipISR();
#else
  static void tipISR();
#endif
};
