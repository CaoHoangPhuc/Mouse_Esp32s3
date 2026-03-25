#include "Battery.h"

void Battery::begin(uint8_t adcPin, uint8_t samples) {
  adcPin_ = adcPin;
  samples_ = (samples == 0) ? 1 : samples;

  analogReadResolution(12);
  pinMode(adcPin_, INPUT);
  started_ = true;
}

void Battery::setCalibration(uint16_t rawLow, float voltageLow,
                             uint16_t rawHigh, float voltageHigh) {
  rawLow_ = rawLow;
  rawHigh_ = (rawHigh <= rawLow) ? (rawLow + 1) : rawHigh;
  voltageLow_ = voltageLow;
  voltageHigh_ = voltageHigh;
}

void Battery::setThresholds(float warningVoltage, float criticalVoltage) {
  warningVoltage_ = warningVoltage;
  criticalVoltage_ = criticalVoltage;
}

void Battery::update() {
  if (!started_) return;

  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples_; ++i) {
    acc += analogRead(adcPin_);
  }

  raw_ = (uint16_t)(acc / samples_);

  const float rawSpan = (float)(rawHigh_ - rawLow_);
  float t = rawSpan > 0.0f ? ((float)raw_ - (float)rawLow_) / rawSpan : 0.0f;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  voltage_ = voltageLow_ + t * (voltageHigh_ - voltageLow_);
  percent_ = t * 100.0f;

  if (voltage_ <= criticalVoltage_) {
    state_ = BATTERY_CRITICAL;
  } else if (voltage_ <= warningVoltage_) {
    state_ = BATTERY_WARNING;
  } else {
    state_ = BATTERY_OK;
  }
}
