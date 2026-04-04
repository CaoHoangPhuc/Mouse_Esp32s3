#include "Battery.h"

void Battery::begin(uint8_t adcPin, uint8_t samples) {
  adcPin_ = adcPin;
  samples_ = (samples == 0) ? 1 : samples;

  analogReadResolution(12);
#if defined(ESP32)
  analogSetPinAttenuation(adcPin_, ADC_11db);
#endif
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

void Battery::setDivider(float topKohm, float bottomKohm) {
  if (topKohm <= 0.0f || bottomKohm <= 0.0f) {
    dividerRatio_ = 0.0f;
    return;
  }
  dividerRatio_ = (topKohm + bottomKohm) / bottomKohm;
}

void Battery::setThresholds(float warningVoltage, float criticalVoltage) {
  warningVoltage_ = warningVoltage;
  criticalVoltage_ = criticalVoltage;
}

void Battery::update() {
  if (!started_) return;

  uint32_t acc = 0;
  uint32_t mvAcc = 0;
  for (uint8_t i = 0; i < samples_; ++i) {
    acc += analogRead(adcPin_);
#if defined(ESP32)
    mvAcc += (uint32_t)analogReadMilliVolts(adcPin_);
#endif
  }

  raw_ = (uint16_t)(acc / samples_);
#if defined(ESP32)
  adcVoltage_ = ((float)mvAcc / (float)samples_) / 1000.0f;
#else
  adcVoltage_ = 3.3f * ((float)raw_ / 4095.0f);
#endif

  if (dividerRatio_ > 0.0f) {
    dividerEstimatedVoltage_ = adcVoltage_ * dividerRatio_;
    voltage_ = 0.9 * voltage_ + 0.1 * dividerEstimatedVoltage_;
  } else {
    dividerEstimatedVoltage_ = 0.0f;
    const float rawSpan = (float)(rawHigh_ - rawLow_);
    const float tRaw = rawSpan > 0.0f ? ((float)raw_ - (float)rawLow_) / rawSpan : 0.0f;
    voltage_ = 0.9 * voltage_ + 0.1 * (voltageLow_ + tRaw * (voltageHigh_ - voltageLow_));
  }
  const float voltageSpan = voltageHigh_ - voltageLow_;
  float tPercent = voltageSpan > 0.0f ? (voltage_ - voltageLow_) / voltageSpan : 0.0f;
  if (tPercent < 0.0f) tPercent = 0.0f;
  if (tPercent > 1.0f) tPercent = 1.0f;
  percent_ = tPercent * 100.0f;

  if (voltage_ <= criticalVoltage_) {
    state_ = BATTERY_CRITICAL;
  } else if (voltage_ <= warningVoltage_) {
    state_ = BATTERY_WARNING;
  } else {
    state_ = BATTERY_OK;
  }
}
