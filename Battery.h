#pragma once

#include <Arduino.h>

class Battery {
public:
  enum State : uint8_t {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL
  };

  void begin(uint8_t adcPin, uint8_t samples = 8);
  void setCalibration(uint16_t rawLow, float voltageLow,
                      uint16_t rawHigh, float voltageHigh);
  void setThresholds(float warningVoltage, float criticalVoltage);
  void update();

  float voltage() const { return voltage_; }
  float percent() const { return percent_; }
  State state() const { return state_; }
  uint16_t raw() const { return raw_; }
  bool isReady() const { return started_; }

private:
  uint8_t adcPin_ = 0;
  uint8_t samples_ = 8;
  bool started_ = false;

  uint16_t raw_ = 0;
  uint16_t rawLow_ = 2800;
  uint16_t rawHigh_ = 3350;
  float voltageLow_ = 7.20f;
  float voltageHigh_ = 8.40f;
  float warningVoltage_ = 7.10f;
  float criticalVoltage_ = 6.90f;

  float voltage_ = 0.0f;
  float percent_ = 0.0f;
  State state_ = BATTERY_CRITICAL;
};
