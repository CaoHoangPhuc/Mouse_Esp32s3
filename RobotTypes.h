#pragma once

#include <Arduino.h>

enum RobotMode : uint8_t {
  ROBOT_MODE_IDLE = 0,
  ROBOT_MODE_MANUAL_TEST,
  ROBOT_MODE_EXPLORE,
  ROBOT_MODE_SPEED_RUN,
  ROBOT_MODE_FAULT
};

enum MotionPrimitiveType : uint8_t {
  MOTION_NONE = 0,
  MOTION_MOVE_ONE_CELL,
  MOTION_MOVE_FORWARD_SHORT,
  MOTION_MOVE_BACKWARD_SHORT,
  MOTION_SNAP_CENTER,
  MOTION_TURN_LEFT_90,
  MOTION_TURN_RIGHT_90,
  MOTION_TURN_180,
  MOTION_STOP
};

enum MotionStatus : uint8_t {
  MOTION_IDLE = 0,
  MOTION_RUNNING_PRIMITIVE,
  MOTION_COMPLETED,
  MOTION_FAILED,
  MOTION_ABORTED
};

struct RobotPose {
  uint8_t cellX = 0;
  uint8_t cellY = 15;
  uint8_t heading = 0;
  float forwardProgressMm = 0.0f;
  float turnProgressDeg = 0.0f;
};

struct WallObservation {
  bool leftWall = false;
  bool frontWall = false;
  bool rightWall = false;
  bool leftValid = false;
  bool frontValid = false;
  bool rightValid = false;
  uint16_t leftMm = 0;
  uint16_t frontMm = 0;
  uint16_t rightMm = 0;
};

struct RobotState {
  RobotMode mode = ROBOT_MODE_IDLE;
  MotionStatus motionStatus = MOTION_IDLE;
  MotionPrimitiveType activePrimitive = MOTION_NONE;

  RobotPose pose{};
  WallObservation walls{};

  float batteryVoltage = 0.0f;
  float batteryPercent = 0.0f;
  uint8_t batteryState = 0;

  int32_t leftTicks = 0;
  int32_t rightTicks = 0;
  float leftTps = 0.0f;
  float rightTps = 0.0f;

  uint16_t sensors[5] = {0, 0, 0, 0, 0};
  bool sensorHealthy = false;
  bool readyForMotion = false;
  bool goalReached = false;
  bool speedRunReady = false;
  uint32_t faultCount = 0;
  String lastFault;
};
