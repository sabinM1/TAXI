#pragma once
#include <Arduino.h>

/**
 * LineSensor.h — QRE1113 analog line/edge sensor driver
 * 
 * Simple wrapper for reading QRE1113 reflective sensors on analog pins.
 * Provides both raw analog values and boolean line detection with
 * configurable threshold.
 * 
 * Wiring: VCC -> 5V, GND -> GND, OUT -> Analog pin (A1, A4, A5)
 * Lower analog value = more reflective (white surface)
 * Higher analog value = less reflective (black line/edge)
 */

class LineSensor {
public:
  /**
   * Create a line sensor on the given analog pin.
   * @param pin        Analog pin (A0-A5)
   * @param threshold  Analog threshold for line detection (0-1023).
   *                   Values ABOVE threshold = line detected (black/edge).
   *                   Values BELOW threshold = no line (white surface).
   *                   Typical values: 400-600 depending on lighting/surface.
   */
  LineSensor(uint8_t pin, uint16_t threshold = 500)
    : _pin(pin), _threshold(threshold), _inverted(false) {}

  /**
   * Initialize the sensor pin.
   * Call once in setup().
   */
  void begin() {
    pinMode(_pin, INPUT);
  }

  /**
   * Read raw analog value (0-1023).
   */
  uint16_t readRaw() const {
    return analogRead(_pin);
  }

  /**
   * Read line detection state.
   * @return true  = line/edge detected (dark surface)
   *         false = no line (light surface)
   */
  bool read() const {
    return evaluate(analogRead(_pin));
  }

  /**
   * Evaluate a previously-read raw sample against the threshold.
   * Use when you need both the raw value and the boolean state from a
   * single ADC conversion (avoids sampling twice).
   */
  bool evaluate(uint16_t raw) const {
    return _inverted ? (raw < _threshold) : (raw > _threshold);
  }

  /**
   * Set the detection threshold.
   * @param threshold  New threshold value (0-1023)
   */
  void setThreshold(uint16_t threshold) {
    _threshold = threshold;
  }

  /**
   * Get current threshold.
   */
  uint16_t getThreshold() const {
    return _threshold;
  }

  /**
   * Invert the detection logic.
   * @param inverted  true = low values = line, false = high values = line
   */
  void setInverted(bool inverted) {
    _inverted = inverted;
  }

  /**
   * Calibrate by sampling min/max over a period.
   * Call while moving sensor over both white and black surfaces.
   * @param durationMs  Calibration time in milliseconds
   * @return true if calibration succeeded (min != max)
   */
  bool calibrate(uint32_t durationMs = 3000) {
    uint16_t minVal = 1023;
    uint16_t maxVal = 0;
    uint32_t start = millis();
    
    while (millis() - start < durationMs) {
      uint16_t val = analogRead(_pin);
      if (val < minVal) minVal = val;
      if (val > maxVal) maxVal = val;
      delay(10);
    }
    
    if (maxVal > minVal) {
      _threshold = (minVal + maxVal) / 2;
      return true;
    }
    return false;
  }

  /**
   * Get the pin number.
   */
  uint8_t getPin() const {
    return _pin;
  }

private:
  uint8_t _pin;
  uint16_t _threshold;
  bool _inverted;
};