#pragma once
#include <Arduino.h>
#include "driver/gpio.h"

class DcMotor {
public:
  using LogFn = void (*)(const String&);

  struct Pins {
    uint8_t in1;
    uint8_t in2;
    uint8_t pwm;
    uint8_t encA;
    uint8_t encB;
    bool invertDir = false;
    bool invertEnc = false; // flips tick direction
  };

  DcMotor();

  // ticksPerWheelRev: encoder ticks counted by THIS decoding method per 1 wheel revolution
  bool begin(const Pins& pins,
             uint8_t pwmChannel,
             uint32_t pwmFreq = 20000,
             uint8_t pwmResolutionBits = 10);

  // Direct power control [-1..+1]
  void setPower(float power);

  // Speed control in ticks/sec
  void setSpeedTPS(float tps);
  void enableSpeedControl(bool en);
  void coastStop();
  void brakeStop();
  void hardStop();

  // PID tuning (TPS units)
  // outLimit: max command magnitude (0..1)
  // iLimit: max integrator magnitude (0..outLimit)
  // dFilterHz: low-pass cutoff for derivative of measurement, 0 disables
  // slewRatePerSec: max output change per second, 0 disables
  void setSpeedPID(float kp, float ki, float kd,
                   float outLimit = 1.0f,
                   float iLimit = 0.6f,
                   float dFilterHz = 30.0f,
                   float slewRatePerSec = 0.0f);
  void setLog(LogFn fn) { _logFn = fn; }

  void resetPID();

  // Call periodically at the motor-task cadence (fixed-dt PID update).
  void update();

  // Encoder / speed
  int32_t getTicks() const;
  void    resetTicks(int32_t value = 0);
  float   getTicksPerSecond() const { return _tps; }

  // motor drive
  void applyDuty(int32_t duty);

  // ===== USAGE =====
  inline float ticksToMM() const {
    return _ticks * K_MM_PER_TICK;
  }

  inline float tpsToMMps() const {
    return _tps * K_MM_PER_TICK;
  }

private:
  Pins _p{};
  uint8_t _pwmCh = 0;
  uint32_t _pwmFreq = 20000;
  uint8_t _pwmResBits = 10;
  uint32_t _pwmMax = 1023;

  const int DEADZONE = 10;
  volatile int32_t _ticks = 0;

  // speed estimation
  uint32_t _lastMicros = 0;
  int32_t  _lastTicks = 0;
  float    _tps = 0.0f; // filtered ticks/sec
  int32_t  _speedAccumTicks = 0;
  uint32_t _speedAccumUs = 0;

  // control
  bool  _speedCtrlEnabled = false;
  float _targetTPS = 0.0f;

  // PID params
  float _kp = 0.0f, _ki = 0.0f, _kd = 0.0f;
  float _outLimit = 1.0f;
  float _iLimit = 0.6f;
  float _dFilterHz = 30.0f;
  float _slewRate = 0.0f; // output units/sec

  // PID internal
  float _iTerm = 0.0f;
  float _dMeas = 0.0f;       // filtered derivative of measurement (tps/s)
  float _lastTPSForD = 0.0f;
  float _lastOut = 0.0f;
  LogFn _logFn = nullptr;

  // ISR glue (encA only, max 4 motors)
  static void isrEncA0();
  static void isrEncA1();
  static void isrEncA2();
  static void isrEncA3();

  float K_MM_PER_TICK = 0.54f;  // <-- calibrated

  static DcMotor* _instances[4];
  int _slot = -1;

  // Called from ISR when encA changes
  void handleEncAChange();

  inline void pwmWriteDuty(uint32_t duty);

  static int allocSlot(DcMotor* m);

  static float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }
};
