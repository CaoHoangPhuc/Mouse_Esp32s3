#include "common/tusb_fifo.h"
#include "MultiVL53L0X.h"

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

    detectLayout();  // 🔥 AUTO DETECT HERE

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
    i2cLock();

    if (_numSensors == 0) {
        i2cUnlock();
        return;
    }

    for (uint8_t tries = 0; tries < _numSensors; tries++) {
        if (_initialized[_cSensor]) break;
        _cSensor = (_cSensor + 1) % _numSensors;
    }

    if (!_initialized[_cSensor]) {
        i2cUnlock();
        return;
    }

    // uint16_t dist = _sensors[_cSensor].readRangeContinuousMillimeters();
    // bool to = _sensors[_cSensor].timeoutOccurred();
    uint16_t dist;
    bool ok = readTOF_fast(_sensorAddresses[_cSensor], dist);

    _raw[_cSensor] = dist;

    if (ok && dist > 0 && dist < 1000) {

        float corrected = dist * _scale[_cSensor] + _offset[_cSensor];

        if (corrected < DIST_MIN_VALID) {
            _lastDistance[_cSensor] = DIST_MIN_VALID;
            _timeoutFlag[_cSensor]  = 1;
        }
        else if (corrected > DIST_MAX_VALID) {
            _lastDistance[_cSensor] = DIST_FAR;
            _timeoutFlag[_cSensor]  = 2;
        }
        else {
            uint16_t val = (uint16_t)corrected;

            if ((_lastDistance[_cSensor] == 0) || (_lastDistance[_cSensor] > DIST_MAX_VALID)) {
                _lastDistance[_cSensor] = val;
            } else {
                _lastDistance[_cSensor] = 0.8f * _lastDistance[_cSensor] + 0.2f * val;
            }

            _timeoutFlag[_cSensor] = 0;
            computeError(0);
        }
    }
    else {
        _timeoutFlag[_cSensor] = 3;
        err_count ++;
    }

    _cSensor = (_cSensor + 1) % _numSensors;
    i2cUnlock();
}

// 🔥 Detect layout
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

// 🔥 Unified sensor read
MultiVL53L0X::SensorState MultiVL53L0X::getSensorState() {
    SensorState s = {false, false, false, false, false, false, 0, 0, 0};

    if (_version == SENSOR_V1) {
        uint16_t left  = getDistance(0);
        uint16_t front = getDistance(2);
        uint16_t right = getDistance(4);

        s.leftValid = isGoodReading_(0);
        s.frontValid = isGoodReading_(2);
        s.rightValid = isGoodReading_(4);
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

        bool flValid = isGoodReading_(0);
        bool frValid = isGoodReading_(3);
        s.leftValid  = isGoodReading_(1);
        s.rightValid = isGoodReading_(2);
        s.frontValid = flValid || frValid;
        s.leftMm = l;
        s.rightMm = r;
        if (flValid && frValid) s.frontMm = min(fl, fr);
        else if (flValid) s.frontMm = fl;
        else if (frValid) s.frontMm = fr;

        s.leftWall  = s.leftValid && (l < _wallThreshold);
        s.rightWall = s.rightValid && (r < _wallThreshold);
        s.frontWall = s.frontValid && (s.frontMm < _wallThreshold);
    }

    return s;
}

// ---- getters ----
bool MultiVL53L0X::isSensorOk(uint8_t index) const {
    return (index < _numSensors) && _initialized[index];
}

uint16_t MultiVL53L0X::getDistance(uint8_t index) const {
    return (index < _numSensors) ? _lastDistance[index] : 0;
}

uint16_t MultiVL53L0X::getRaw(uint8_t index) const {
    return (index < _numSensors) ? _raw[index]: 0;
}

uint8_t MultiVL53L0X::stateTimeout(uint8_t index) const {
    return (index < _numSensors) ? _timeoutFlag[index]: 255;
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

    uint16_t left = DIST_ERROR;
    uint16_t right = DIST_ERROR;

    uint8_t leftState = 3;
    uint8_t rightState = 3;

    // 🔹 Map sensors
    if (_version == SENSOR_V1) {
        left  = getDistance(0);
        right = getDistance(4);
        leftState  = stateTimeout(0);
        rightState = stateTimeout(4);
    }
    else if (_version == SENSOR_V2) {
        left  = getDistance(1);
        right = getDistance(2);
        leftState  = stateTimeout(1);
        rightState = stateTimeout(2);
    }

    bool leftValid  = isGood(leftState);
    bool rightValid = isGood(rightState);

    float err = 0.0f;

    // ✅ BOTH WALLS
    // if (leftValid && rightValid) {
    //     err = (float)left - (float)right;
    //     // CENTER_TARGET = 0.8*CENTER_TARGET + 0.2 * (((float)left + (float)right) / 2.0);
    // }
    // ✅ ONLY LEFT
    if (leftValid) {
        err = (float)CENTER_TARGET - (float)left;
    }
    // ✅ ONLY RIGHT
    else if (rightValid) {
        err = (float)right - (float)CENTER_TARGET;
    }
    // ✅ NO WALL → fallback
    else {
        err = headingError;
    }

    // 🔥 Optional blending (VERY useful)
    // helps reduce oscillation
    err = 0.8f * error + 0.2f * err;

    // 🔥 clamp
    if (err > 50) err = 50;
    if (err < -50) err = -50;

    error = err;

    return err;;
}

bool MultiVL53L0X::isGoodReading_(uint8_t index) const {
    if (index >= _numSensors) return false;
    uint8_t state = stateTimeout(index);
    return state == 0 || state == 1;
}

