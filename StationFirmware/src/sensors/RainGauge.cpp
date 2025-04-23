#include "include/sensors/RainGauge.h"

// Initialize static instance pointer
RainGauge *RainGauge::_instance = nullptr;

RainGauge::RainGauge(uint8_t pin) : _pin(pin), _tipCount(0) {
  // Store the instance pointer (assuming only one instance)
  _instance = this;
}

RainGauge::~RainGauge() {
  // Cleanup: Detach the interrupt when the object is destroyed
  if (digitalPinToInterrupt(_pin) != NOT_AN_INTERRUPT) {
    detachInterrupt(digitalPinToInterrupt(_pin));
  }
}

void RainGauge::begin() {
  // Ensure pin is valid for interrupts
  if (digitalPinToInterrupt(_pin) == NOT_AN_INTERRUPT) {
    // Handle error - perhaps log to Serial or set an error flag
    return;
  }

  // Set pin mode with internal pull-up resistor
  pinMode(_pin, INPUT_PULLUP);

  // Reset counter
  resetTotal(); // Use the reset function to clear the count

  // Attach the interrupt handler
  // Trigger on FALLING edge (pin goes from HIGH due to pull-up to LOW when
  // switch closes)
  attachInterrupt(digitalPinToInterrupt(_pin), tipISR, FALLING);
}

float RainGauge::getTotalInches() const {
  // Read volatile count safely (interrupts are already disabled in ISR)
  // For simple reads like this, direct access is often okay on AVR/ESP32
  // but for multi-byte reads or read-modify-write, disable interrupts.
  unsigned long currentCount = _tipCount;
  return (float)currentCount * INCHES_PER_TIP;
}

float RainGauge::getTotalMillimeters() const {
  unsigned long currentCount = _tipCount;
  return (float)currentCount * MM_PER_TIP;
}

void RainGauge::resetTotal() {
  // Safely reset the counter (disable interrupts briefly)
  noInterrupts();
  _tipCount = 0;
  interrupts();
}

unsigned long RainGauge::getRawTipCount() const {
  // Read volatile count
  return _tipCount;
}

// --- Static ISR Implementation ---
// This function is called by the hardware interrupt
#ifdef ESP32
void IRAM_ATTR RainGauge::tipISR() {
#else
void RainGauge::tipISR() {
#endif
  // Check if the instance exists (safety measure)
  if (_instance != nullptr) {
    // Increment the tip count of the single instance
    // This needs to be as fast as possible!
    // Note: Basic debouncing could be added here if needed, e.g., by
    // checking millis() since the last ISR call, but adds overhead.
    _instance->_tipCount++;
  }
}
