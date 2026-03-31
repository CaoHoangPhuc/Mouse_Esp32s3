#include "Config.h"
#include "MultiVL53L0X.h"

uint8_t MultiVL53L0X::effectiveState_(uint8_t index) const {
    if (index >= _numSensors) return 255;
    if (!_initialized[index]) return 3;

    // if (_version == SENSOR_V2 && index == 3) {
    //     uint8_t s0State = _timeoutFlag[0];

    //     if (s0State == 2) return 2;  // S0 sees "far", clamp S3 to far too.
    //     if (s0State == 3) return 3;  // S0 invalid => S3 invalid.
    // }

    return _timeoutFlag[index];
}

uint16_t MultiVL53L0X::effectiveDistance_(uint8_t index) const {
    if (index >= _numSensors) return 0;
    if (!_initialized[index]) return 0;

    // if (_version == SENSOR_V2 && index == 3) {
    //     uint8_t s0State = _timeoutFlag[0];

    //     if (s0State == 2) return AppConfig::Tof::DIST_FAR_MM;  // Match S0 "far" state.
    //     if (s0State == 3) return 0;         // Match S0 invalid state.
    // }

    return _lastDistance[index];
}

void MultiVL53L0X::setCenterPid(float kp, float ki, float kd, float iLimit, float outLimit) {
    _centerKp = kp;
    _centerKi = ki;
    _centerKd = kd;
    _centerILimit = fabsf(iLimit);
    _centerOutLimit = fabsf(outLimit);
}

void MultiVL53L0X::setCenterTargets(float leftMm, float rightMm) {
    _centerTargetLeft = leftMm;
    _centerTargetRight = rightMm;
}

void MultiVL53L0X::resetCenterPid() {
    _centerIntegral = 0.0f;
    _centerPrevError = 0.0f;
    _centerRawFiltered = 0.0f;
    _lastDualWallError = 0.0f;
    _dualWallBlend = 0.0f;
    _captureCenterTargetsOnFirstSample = true;
    _straightTrackMode = TRACK_NONE;
    _centerPrevMs = 0;
    _centerPidPrimed = false;
    error = 0.0f;
}

MultiVL53L0X::MultiVL53L0X(uint8_t pcfAddress,
                           uint8_t numSensors,
                           const uint8_t* xshutPins,
                           const uint8_t* sensorAddresses,
                           uint16_t updateIntervalMs,
                           TwoWire& wire)
    : _wire(&wire),
      _pcf(pcfAddress),
      _pcfAddress(pcfAddress),
      _numSensors(numSensors),
      _intervalMs(updateIntervalMs),
      _xshutPins(xshutPins),
      _sensorAddresses(sensorAddresses)
{
    if (_numSensors > MAX_SENSORS) _numSensors = MAX_SENSORS;

    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        _initialized[i]  = false;
        _timeoutFlag[i]  = false;
        _lastDistance[i] = 0;
    }
}

bool MultiVL53L0X::begin() {

    if (!_pcf.begin()) return false;

    if (_i2cMutex == nullptr) {
        _i2cMutex = xSemaphoreCreateMutex();
    }

    i2cLock();

    for (uint8_t i = 0; i < 8; i++) {
        _pcf.write(i, HIGH);
    }

    xshutAllLow();
    vTaskDelay(pdMS_TO_TICKS(10));

    for (uint8_t i = 0; i < _numSensors; i++) {
        _pcf.write(_xshutPins[i], HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));

        _sensors[i].setBus(_wire);
        _sensors[i].setTimeout(30);

        // Retry up to 3 times
        bool initialized = false;
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            if (_sensors[i].init()) {
                _sensors[i].setAddress(_sensorAddresses[i]);
                vTaskDelay(pdMS_TO_TICKS(5));
                _sensors[i].startContinuous(_intervalMs);
                vTaskDelay(pdMS_TO_TICKS(30));

                uint16_t test = _sensors[i].readRangeContinuousMillimeters();
                bool to = _sensors[i].timeoutOccurred();

                if (!to && test != 0 && test != 65535) {
                    _lastDistance[i] = test;
                    initialized = true;
                    break; // success, exit retry loop
                }
            }

            // Failed this attempt, shutdown and wait before retry
            _pcf.write(_xshutPins[i], LOW);
            vTaskDelay(pdMS_TO_TICKS(30));
            _pcf.write(_xshutPins[i], HIGH);
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        _initialized[i] = initialized;
        if (!initialized) {
            _pcf.write(_xshutPins[i], LOW); // final shutdown if all retries fail
        }
    }

    xshutAllHigh();
    i2cUnlock();

    detectLayout();  // Auto-detect sensor layout here

    return true;
}

bool MultiVL53L0X::readTOF_fast(uint8_t addr, uint16_t &dist)
{
    _wire->beginTransmission(addr);
    _wire->write(0x14);

    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    delayMicroseconds(200);

    _wire->requestFrom(addr, (uint8_t)12);

    if (_wire->available() < 12) {
        return false;
    }

    uint8_t buf[12];
    for (int i = 0; i < 12; i++) {
        buf[i] = _wire->read();
    }

    dist = (buf[10] << 8) | buf[11];

    return true;
}

void MultiVL53L0X::update() {
    // Define the sequence pattern
    static const uint8_t sensorOrder[] = {0, 2, 1, 3, 4};
    uint8_t currentSensor = sensorOrder[_cSensor % _numSensors];

    if (_numSensors == 0) {
        return;
    }

    for (uint8_t tries = 0; tries < _numSensors; tries++) {
        if (_initialized[_cSensor]) break;
        _cSensor = (_cSensor + 1) % _numSensors;
    }

    if (!_initialized[currentSensor]) {
        return;
    }

    // uint16_t dist = _sensors[currentSensor].readRangeContinuousMillimeters();
    // bool to = _sensors[currentSensor].timeoutOccurred();
    uint16_t dist;
    i2cLock();
    bool ok = readTOF_fast(_sensorAddresses[currentSensor], dist);
    i2cUnlock();

    _raw[currentSensor] = dist;

    if (ok && dist > 0 && dist < 1000) {

        float corrected = dist * _scale[currentSensor] + _offset[currentSensor];

        if (corrected < AppConfig::Tof::DIST_MIN_VALID_MM) {
            _lastDistance[currentSensor] = AppConfig::Tof::DIST_MIN_VALID_MM;
            _timeoutFlag[currentSensor]  = 1;
        }
        else if (corrected > AppConfig::Tof::DIST_MAX_VALID_MM) {
            _lastDistance[currentSensor] = AppConfig::Tof::DIST_FAR_MM;
            _timeoutFlag[currentSensor]  = 2;
        }
        else {
            uint16_t val = (uint16_t)corrected;

            if ((_lastDistance[currentSensor] == 0) || (_lastDistance[currentSensor] > AppConfig::Tof::DIST_MAX_VALID_MM)) {
                _lastDistance[currentSensor] = val;
            } else {
                _lastDistance[currentSensor] =
                    AppConfig::Tof::DIST_LPF_PREV_WEIGHT * _lastDistance[currentSensor] +
                    AppConfig::Tof::DIST_LPF_SAMPLE_WEIGHT * val;
            }

            _timeoutFlag[currentSensor] = 0;
        }
    }
    else {
        _timeoutFlag[currentSensor] = 3;
        err_count ++;
    }

    _cSensor = (_cSensor + 1) % _numSensors;
}

// Detect layout
void MultiVL53L0X::detectLayout() {
    uint8_t active = 0;

    for (uint8_t i = 0; i < _numSensors; i++) {
        if (_initialized[i]) active++;
    }

    if (active == 4) {
        _version = SENSOR_V2;
    } else {
        _version = SENSOR_V1;
    }
}

// Unified sensor read
MultiVL53L0X::SensorState MultiVL53L0X::getSensorState() {
    SensorState s = {false, false, false, false, false, false, 0, 0, 0};
    auto isObservable = [&](uint8_t index) {
        if (index >= _numSensors) return false;
        uint8_t state = stateTimeout(index);
        return state == 0 || state == 1 || state == 2;
    };

    if (_version == SENSOR_V1) {
        uint16_t left  = getDistance(0);
        uint16_t front = getDistance(2);
        uint16_t right = getDistance(4);

        s.leftValid = isObservable(0);
        s.frontValid = isObservable(2);
        s.rightValid = isObservable(4);
        s.leftMm = left;
        s.frontMm = front;
        s.rightMm = right;
        s.leftWall  = s.leftValid && (left  < _wallThreshold);
        s.rightWall = s.rightValid && (right < _wallThreshold);
        s.frontWall = s.frontValid && (front < _wallThreshold);
    }
    else if (_version == SENSOR_V2) {
        uint16_t fl = getDistance(0);
        uint16_t l  = getDistance(1);
        uint16_t r  = getDistance(2);
        uint16_t fr = getDistance(3);

        uint8_t flState = stateTimeout(0);
        uint8_t frState = stateTimeout(3);
        bool flValid = isObservable(0);
        bool frValid = isObservable(3);
        // bool frValidRaw = isObservable(3);
        // S3 is front-right. On current hardware it is only trustworthy when
        // S0 (front-left) is also seeing the front wall region.
        // bool frValid = flValid && frValidRaw;
        bool flFar = flState == 2;
        bool frFar = frState == 2;
        s.leftValid  = isObservable(1);
        s.rightValid = isObservable(2);
        s.frontValid = flValid || frValid || (flFar && frFar);
        s.leftMm = l;
        s.rightMm = r;
        if (flValid && frValid) s.frontMm = min(fl, fr);
        else if (flValid) s.frontMm = fl;
        else if (frValid) s.frontMm = fr;
        else if (flFar && frFar) s.frontMm = AppConfig::Tof::DIST_FAR_MM;

        s.leftWall  = s.leftValid && (l < _wallThreshold);
        s.rightWall = s.rightValid && (r < _wallThreshold);
        s.frontWall = s.frontValid && (s.frontMm < _wallThreshold);
    }

    return s;
}

// ---- getters ----
bool MultiVL53L0X::isSensorOk(uint8_t index) const {
    if (index >= _numSensors) return false;
    return _initialized[index];
}

uint16_t MultiVL53L0X::getDistance(uint8_t index) const {
    return effectiveDistance_(index);
}

uint16_t MultiVL53L0X::getRaw(uint8_t index) const {
    if (index >= _numSensors) return 0;
    return _raw[index];
}

uint8_t MultiVL53L0X::stateTimeout(uint8_t index) const {
    return effectiveState_(index);
}

uint8_t MultiVL53L0X::getSensorAddress(uint8_t index) const {
    return (index < _numSensors) ? _sensorAddresses[index] : 0;
}

// ---- XSHUT ----
void MultiVL53L0X::xshutAllLow() {
    for (uint8_t i = 0; i < _numSensors; i++) {
        _pcf.write(_xshutPins[i], LOW);
    }
}

void MultiVL53L0X::xshutAllHigh() {
    for (uint8_t i = 0; i < _numSensors; i++) {
        _pcf.write(_xshutPins[i], HIGH);
    }
}

float MultiVL53L0X::computeError(float headingError) {

    auto isGood = [&](uint8_t state) {
        return state == 0 || state == 1;  // valid or clipped-min
    };

    uint16_t left = AppConfig::Tof::DIST_ERROR_MM;
    uint16_t right = AppConfig::Tof::DIST_ERROR_MM;

    static uint16_t effectiveLeft = 150;
    static uint16_t effectiveRight = 150;

    uint8_t leftState = 3;
    uint8_t rightState = 3;

    if (_version == SENSOR_V1) {
        left  = getDistance(0) < effectiveLeft ? getDistance(0) : effectiveLeft;
        right = getDistance(4) < effectiveRight ? getDistance(4) : effectiveRight;
        leftState  = stateTimeout(0);
        rightState = stateTimeout(4);
    }
    else if (_version == SENSOR_V2) {
        left  = getDistance(1) < effectiveLeft ? getDistance(1) : effectiveLeft;
        right = getDistance(2) < effectiveRight ? getDistance(2) : effectiveRight;
        leftState  = stateTimeout(1);
        rightState = stateTimeout(2);
    }

    const bool leftValid  = isGood(leftState);
    const bool rightValid = isGood(rightState);
    const bool dualWallValid = leftValid && rightValid;
    const bool forceLeft = _straightTrackMode == TRACK_LEFT;
    const bool forceRight = _straightTrackMode == TRACK_RIGHT;
    const bool forceDual = _straightTrackMode == TRACK_DUAL;
    const bool forceNone = _straightTrackMode == TRACK_NONE;
    const bool shouldCaptureTargets = _captureCenterTargetsOnFirstSample;
    _captureCenterTargetsOnFirstSample = false;

    float dualErr = 0.0f;
    if (dualWallValid) {
        if (shouldCaptureTargets &&
            fabsf((float)left - (float)right) <= AppConfig::Motion::CENTER_TARGET_CAPTURE_WINDOW_MM) {
            _centerTargetLeft = 0.8f * _centerTargetLeft + 0.2f * (float)left;
            _centerTargetRight = (float)right;
        }
        dualErr = 0.5f * ((_centerTargetLeft - (float)left) +
                          ((float)right - _centerTargetRight));
        _lastDualWallError = dualErr;
    }

    float singleErr = headingError;
    if (leftValid && !rightValid) {
        singleErr = _centerTargetLeft - (float)left;
    } else if (rightValid && !leftValid) {
        singleErr = (float)right - _centerTargetRight;
    }

    const uint32_t now = millis();
    float dt = 0.02f;
    if (_centerPidPrimed && _centerPrevMs > 0) {
        const uint32_t deltaMs = now - _centerPrevMs;
        if (deltaMs > 0) dt = max(0.001f, deltaMs / 1000.0f);
    }

    if (!_centerPidPrimed) {
        _centerIntegral = 0.0f;
        _centerPrevError = 0.0f;
        if (forceDual) {
            _centerRawFiltered = dualWallValid ? dualErr : headingError;
            _dualWallBlend = dualWallValid ? 1.0f : 0.0f;
        } else if (forceLeft) {
            _centerRawFiltered = leftValid ? (_centerTargetLeft - (float)left) : headingError;
            _dualWallBlend = 0.0f;
        } else if (forceRight) {
            _centerRawFiltered = rightValid ? ((float)right - _centerTargetRight) : headingError;
            _dualWallBlend = 0.0f;
        } else {
            _centerRawFiltered = headingError;
            _dualWallBlend = 0.0f;
        }
        _centerPidPrimed = true;
    }

    const float blendTarget = (forceDual && dualWallValid) ? 1.0f : 0.0f;
    const float blendTauSec = AppConfig::Motion::CENTER_BLEND_TAU_SEC;
    const float blendAlpha = dt / (blendTauSec + dt);
    _dualWallBlend += (blendTarget - _dualWallBlend) * blendAlpha;
    if (_dualWallBlend < 0.0f) _dualWallBlend = 0.0f;
    if (_dualWallBlend > 1.0f) _dualWallBlend = 1.0f;

    float targetRawErr = headingError;
    if (forceDual) {
        if (dualWallValid) {
            targetRawErr = dualErr;
        } else if (leftValid) {
            // Corridor transition: if dual tracking loses the right wall, keep following left.
            targetRawErr = _centerTargetLeft - (float)left;
        } else if (rightValid) {
            // Corridor transition: if dual tracking loses the left wall, keep following right.
            targetRawErr = (float)right - _centerTargetRight;
        } else {
            targetRawErr = headingError;
        }
    } else if (forceLeft) {
        targetRawErr = leftValid ? (_centerTargetLeft - (float)left) : headingError;
    } else if (forceRight) {
        targetRawErr = rightValid ? ((float)right - _centerTargetRight) : headingError;
    } else if (!forceNone) {
        if (dualWallValid) {
            targetRawErr = dualErr;
        } else if (leftValid || rightValid) {
            targetRawErr = (_dualWallBlend * _lastDualWallError) +
                           ((1.0f - _dualWallBlend) * singleErr);
        }
    }

    const float rawTauSec = AppConfig::Motion::CENTER_RAW_TAU_SEC;
    const float rawAlpha = dt / (rawTauSec + dt);
    _centerRawFiltered += (targetRawErr - _centerRawFiltered) * rawAlpha;
    const float rawErr = _centerRawFiltered;

    _centerIntegral += rawErr * dt;
    if (_centerIntegral > _centerILimit) _centerIntegral = _centerILimit;
    if (_centerIntegral < -_centerILimit) _centerIntegral = -_centerILimit;

    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (rawErr - _centerPrevError) / dt;
    }

    float out = (_centerKp * rawErr) +
                (_centerKi * _centerIntegral) +
                (_centerKd * derivative);

    // Front-corner anti-stick assist:
    // when front corner is too close, force stronger steering away from that corner.
    float escapeSign = 0.0f;  // + => steer right, - => steer left
    const float stickMm = AppConfig::Motion::FRONT_CORNER_STICK_MM;
    if (_version == SENSOR_V2) {
        const uint16_t fl = getDistance(0);
        const uint16_t fr = getDistance(3);
        const uint8_t flState = stateTimeout(0);
        const uint8_t frState = stateTimeout(3);
        const bool flClose = isGood(flState) && fl > 0 && (float)fl < stickMm;
        const bool frClose = isGood(frState) && fr > 0 && (float)fr < stickMm;

        if (flClose && !frClose) {
            escapeSign = 1.0f;
        } else if (frClose && !flClose) {
            escapeSign = -1.0f;
        } else if (flClose && frClose) {
            if (leftValid && rightValid) {
                if (left < right) escapeSign = 1.0f;
                else if (right < left) escapeSign = -1.0f;
            }
            if (escapeSign == 0.0f) {
                escapeSign = (rawErr >= 0.0f) ? 1.0f : -1.0f;
            }
        }
    } else if (_version == SENSOR_V1) {
        const uint16_t front = getDistance(2);
        const uint8_t frontState = stateTimeout(2);
        const bool frontClose = isGood(frontState) && front > 0 && (float)front < stickMm;
        if (frontClose) {
            if (leftValid && rightValid) {
                if (left < right) escapeSign = 1.0f;
                else if (right < left) escapeSign = -1.0f;
            } else if (leftValid) {
                escapeSign = 1.0f;
            } else if (rightValid) {
                escapeSign = -1.0f;
            } else {
                escapeSign = (rawErr >= 0.0f) ? 1.0f : -1.0f;
            }
        }
    }

    if (escapeSign != 0.0f) {
        const float extraOut =
            AppConfig::Motion::FRONT_CORNER_ESCAPE_EXTRA_OUT_FACTOR * _centerOutLimit;
        const float minOut =
            AppConfig::Motion::FRONT_CORNER_ESCAPE_MIN_OUT_FACTOR * _centerOutLimit;
        out += escapeSign * extraOut;
        if (escapeSign > 0.0f && out < minOut) out = minOut;
        if (escapeSign < 0.0f && out > -minOut) out = -minOut;
    }

    if (out > _centerOutLimit) out = _centerOutLimit;
    if (out < -_centerOutLimit) out = -_centerOutLimit;

    _centerPrevError = rawErr;
    _centerPrevMs = now;
    error = out;

    return out;
}

bool MultiVL53L0X::isGoodReading_(uint8_t index) const {
    if (index >= _numSensors) return false;
    uint8_t state = stateTimeout(index);
    return state == 0 || state == 1;
}

