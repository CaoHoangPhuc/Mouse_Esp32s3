#pragma once

#include <Arduino.h>

#include "Battery.h"
#include "DcMotor.h"
#include "MultiVL53L0X.h"
#include "RobotTypes.h"

class MotionController {
public:
  struct Config {
    float cellDistanceMm = 180.0f;
    int32_t turnTicks90 = 300;
    int32_t turnTicks180 = 600;
    float moveSpeedTps = 320.0f;
    float shortForwardDistanceMm = 90.0f;
    float shortForwardSpeedTps = 220.0f;
    float reverseDistanceMm = 45.0f;
    float reverseSpeedTps = 180.0f;
    uint32_t snapCenterStopHoldMs = 50;
    float turnSpeedTps = 250.0f;
    float centeringGain = 1.6f;
    float frontStopMm = 55.0f;
    uint32_t primitiveTimeoutMs = 3000;
    uint32_t stallTimeoutMs = 700;
    float stopTps = 20.0f;
    float minProgressMm = 12.0f;
    float mmPerTick = 0.54f;
  };

  void begin(DcMotor& left, DcMotor& right, MultiVL53L0X& tof, Battery* battery = nullptr);
  void setConfig(const Config& cfg) { cfg_ = cfg; }

  bool moveOneCell();
  bool moveForwardShort();
  bool moveBackwardShort();
  bool snapCenter();
  bool turnLeft90();
  bool turnRight90();
  bool turn180();
  void stop();
  void abort(const String& reason);
  void update(RobotState& state);

  bool isBusy() const { return status_ == MOTION_RUNNING_PRIMITIVE; }
  MotionStatus status() const { return status_; }
  MotionPrimitiveType primitive() const { return primitive_; }
  MotionPrimitiveType lastFinishedPrimitive() const { return lastFinishedPrimitive_; }
  const String& lastError() const { return lastError_; }

private:
  bool startPrimitive_(MotionPrimitiveType primitive);
  void markDone_(MotionStatus status, const String& reason = "");
  float averageProgressMm_() const;
  float absoluteAverageProgressMm_() const;
  int32_t differentialTicks_() const;

  DcMotor* left_ = nullptr;
  DcMotor* right_ = nullptr;
  MultiVL53L0X* tof_ = nullptr;
  Battery* battery_ = nullptr;
  Config cfg_{};

  MotionPrimitiveType primitive_ = MOTION_NONE;
  MotionPrimitiveType lastFinishedPrimitive_ = MOTION_NONE;
  MotionStatus status_ = MOTION_IDLE;
  String lastError_;

  int32_t startLeftTicks_ = 0;
  int32_t startRightTicks_ = 0;
  uint32_t startedMs_ = 0;
  uint32_t lastProgressMs_ = 0;
  uint32_t snapCenterHoldUntilMs_ = 0;
  float lastProgressMm_ = 0.0f;
  uint8_t snapCenterPhase_ = 0;
};
