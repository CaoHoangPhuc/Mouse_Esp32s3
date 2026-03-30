#ifndef MULTIVL53L0X_H
#define MULTIVL53L0X_H

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <VL53L0X.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class MultiVL53L0X {
public:
    static const uint8_t MAX_SENSORS = 8;

    enum SensorVersion {
        SENSOR_UNKNOWN = 0,
        SENSOR_V1,   // 3 sensors: left(0), front(2), right(4)
        SENSOR_V2    // 4 sensors: FL(0), L(1), R(2), FR(3)
    };

    struct SensorState {
        bool leftWall;
        bool rightWall;
        bool frontWall;
        bool leftValid;
        bool rightValid;
        bool frontValid;
        uint16_t leftMm;
        uint16_t rightMm;
        uint16_t frontMm;
    };

    MultiVL53L0X(uint8_t pcfAddress,
                 uint8_t numSensors,
                 const uint8_t* xshutPins,
                 const uint8_t* sensorAddresses,
                 uint16_t updateIntervalMs,
                 TwoWire& wire = Wire);

    bool begin();
    void update();

    // ---- New ----
    void detectLayout();
    SensorVersion getVersion() const { return _version; }
    SensorState getSensorState();
    void setWallThreshold(uint16_t th) { _wallThreshold = th; }
    uint16_t wallThreshold() const { return _wallThreshold; }
    void setCenterPid(float kp, float ki, float kd, float iLimit, float outLimit);
    void setCenterTargets(float leftMm, float rightMm);
    void resetCenterPid();

    // ---- Sensors ----
    bool     isSensorOk(uint8_t index) const;
    uint16_t getDistance(uint8_t index) const;
    uint16_t getRaw(uint8_t index) const;
    uint8_t  stateTimeout(uint8_t index) const;
    uint8_t  getSensorAddress(uint8_t index) const;
    uint8_t  sensorCount() const { return _numSensors; };
    float computeError(float headingError = 0.0f);
    float getError(float headingError = 0.0f) {return error; };

    void setMutex(SemaphoreHandle_t m) {_i2cMutex = m;}

    bool readTOF_fast(uint8_t addr, uint16_t &dist);
    
    uint16_t err_count = 0;

private:
    // I2C
    TwoWire*  _wire;

    SemaphoreHandle_t _i2cMutex = nullptr;
    inline void i2cLock()   { if (_i2cMutex) xSemaphoreTake(_i2cMutex, portMAX_DELAY); }
    inline void i2cUnlock() { if (_i2cMutex) xSemaphoreGive(_i2cMutex); }

    // PCF8574
    PCF8574   _pcf;
    uint8_t   _pcfAddress;

    // Sensors
    uint8_t   _numSensors;
    uint16_t  _intervalMs;
    const uint8_t* _xshutPins;
    const uint8_t* _sensorAddresses;

    VL53L0X  _sensors[MAX_SENSORS];
    bool     _initialized[MAX_SENSORS];
    uint8_t  _timeoutFlag[MAX_SENSORS];
    uint16_t _raw[MAX_SENSORS];
    uint16_t _lastDistance[MAX_SENSORS];
    int      _cSensor = 0;

    SensorVersion _version = SENSOR_UNKNOWN;

    // S1 range: 62 -> 150, so range = 88, scale = 97/88, center = 97
    // S2 range: 30 -> 127, so range = 97, absolute minimum = (62 - 30) / 2 = 16

    //37 183; 129 94

    static constexpr uint16_t DIST_MIN_VALID   = 1;
    static constexpr uint16_t DIST_MAX_VALID   = 200;
    static constexpr uint16_t DIST_FAR         = DIST_MAX_VALID + 1;
    static constexpr uint16_t DIST_ERROR       = DIST_MAX_VALID + 2;

    uint16_t _wallThreshold = 150;

    float error = 0;
    float _centerKp = 1.0f;
    float _centerKi = 0.0f;
    float _centerKd = 0.0f;
    float _centerILimit = 50.0f;
    float _centerOutLimit = 50.0f;
    float _centerIntegral = 0.0f;
    float _centerPrevError = 0.0f;
    float _centerRawFiltered = 0.0f;
    float _lastDualWallError = 0.0f;
    float _dualWallBlend = 0.0f;
    float _centerTargetLeft = 100.0f;
    float _centerTargetRight = 100.0f;
    bool _captureCenterTargetsOnFirstSample = true;
    uint32_t _centerPrevMs = 0;
    bool _centerPidPrimed = false;

    float _scale[MAX_SENSORS] = {1, 97.0/88.0, 1, 1, 1};
    int _offset[MAX_SENSORS] = {0, -16, 16, 0, 0};
    
    // static constexpr uint16_t CENTER_TARGET    = 91;

    // float _scale[MAX_SENSORS] = {1, 1, 92.0/89.0, 1, 1};
    // int _offset[MAX_SENSORS] = {0, 28, -28, 0, 0};
    
    // Helpers
    void xshutAllLow();
    void xshutAllHigh();
    bool isGoodReading_(uint8_t index) const;
    uint8_t effectiveState_(uint8_t index) const;
    uint16_t effectiveDistance_(uint8_t index) const;
};

#endif
