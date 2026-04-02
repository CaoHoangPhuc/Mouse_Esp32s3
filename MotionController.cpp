#include "MotionController.h"

#include <math.h>

namespace {
constexpr uint8_t SNAP_CENTER_PHASE_NONE = 0;
constexpr uint8_t SNAP_CENTER_PHASE_BACK = 1;
constexpr uint8_t SNAP_CENTER_PHASE_FORWARD = 2;
constexpr uint8_t SNAP_CENTER_PHASE_HOLD = 3;
}

void MotionController::begin(DcMotor& left, DcMotor& right, MultiVL53L0X& tof, Battery* battery) {
  left_ = &left;
  right_ = &right;
  tof_ = &tof;
  battery_ = battery;
}

void MotionController::resetSnapState_() {
  snapCenterHoldUntilMs_ = 0;
  snapCenterPhase_ = SNAP_CENTER_PHASE_NONE;
}

float MotionController::frontApproachSpeedTps_(float baseSpeedTps,
                                               const WallObservation& walls,
                                               float stopThresholdMm) const {
  if (!walls.frontValid || !walls.frontWall || walls.frontMm == 0) return baseSpeedTps;
  const float frontMm = (float)walls.frontMm;
  if (frontMm <= stopThresholdMm) return cfg_.frontApproachMinSpeedTps;
  if (frontMm >= cfg_.frontApproachSlowdownMm) return baseSpeedTps;
  const float span = max(1.0f, cfg_.frontApproachSlowdownMm - stopThresholdMm);
  const float t = (frontMm - stopThresholdMm) / span;  // 0..1
  return cfg_.frontApproachMinSpeedTps +
         (baseSpeedTps - cfg_.frontApproachMinSpeedTps) * t;
}

bool MotionController::frontStopReached_(const WallObservation& walls, float stopThresholdMm) {
  const bool inRange = walls.frontValid &&
                       walls.frontWall &&
                       walls.frontMm > 0 &&
                       (float)walls.frontMm <= stopThresholdMm;
  if (inRange) {
    if (frontStopStableCount_ < 255) frontStopStableCount_++;
  } else {
    frontStopStableCount_ = 0;
  }
  return frontStopStableCount_ >= max<uint8_t>(1, cfg_.frontStopConfirmSamples);
}

bool MotionController::updateProgressOrFail_(float progressMm, uint32_t now, const char* stallReason) {
  if (progressMm > lastProgressMm_ + cfg_.minProgressMm) {
    lastProgressMm_ = progressMm;
    lastProgressMs_ = now;
    return false;
  }
  if ((uint32_t)(now - lastProgressMs_) > cfg_.stallTimeoutMs) {
    markDone_(MOTION_FAILED, stallReason);
    return true;
  }
  return false;
}

bool MotionController::centerBiasActiveForState_(const RobotState& state) const {
  if (cfg_.centerBiasBackMm <= 0.0f) return false;
  if (state.mode == ROBOT_MODE_EXPLORE) return cfg_.centerBiasEnableExplore;
  if (state.mode == ROBOT_MODE_SPEED_RUN && state.speedRunPhase == 1) {
    return cfg_.centerBiasEnableSpeedRun1;
  }
  return false;
}

bool MotionController::startPrimitive_(MotionPrimitiveType primitive) {
  if (!left_ || !right_ || !tof_) return false;
  if (isBusy()) return false;

  primitive_ = primitive;
  lastFinishedPrimitive_ = MOTION_NONE;
  status_ = MOTION_RUNNING_PRIMITIVE;
  lastError_ = "";
  startLeftTicks_ = left_->getTicks();
  startRightTicks_ = right_->getTicks();
  startedMs_ = millis();
  lastProgressMs_ = startedMs_;
  resetSnapState_();
  lastProgressMm_ = 0.0f;
  frontStopStableCount_ = 0;
  moveCellTargetCount_ = 1;
  moveEndsAtKnownWall_ = false;
  straightTrackModeLatched_ = false;
  straightTrackMode_ = MultiVL53L0X::TRACK_NONE;
  tof_->setStraightTrackMode(MultiVL53L0X::TRACK_NONE);
  return true;
}

bool MotionController::moveOneCell() {
  if (!startPrimitive_(MOTION_MOVE_ONE_CELL)) return false;
  tof_->resetCenterPid();
  left_->setSpeedTPS(cfg_.moveSpeedTps);
  right_->setSpeedTPS(cfg_.moveSpeedTps);
  return true;
}

bool MotionController::moveCells(uint8_t cells, bool requireFrontStopAtEnd) {
  if (cells <= 1) return moveOneCell();
  if (cells > cfg_.corridorMaxCells) cells = cfg_.corridorMaxCells;
  if (!startPrimitive_(MOTION_MOVE_MULTI_CELL)) return false;
  moveCellTargetCount_ = cells;
  moveEndsAtKnownWall_ = requireFrontStopAtEnd;
  tof_->resetCenterPid();
  left_->setSpeedTPS(cfg_.corridorMoveSpeedTps);
  right_->setSpeedTPS(cfg_.corridorMoveSpeedTps);
  return true;
}

bool MotionController::moveLeft90() {
  if (!startPrimitive_(MOTION_MOVE_LEFT_90)) return false;
  tof_->resetCenterPid();
  const float outer = cfg_.cornerMoveSpeedTps;
  const float inner = cfg_.cornerMoveSpeedTps * cfg_.cornerInnerWheelRatio;
  left_->setSpeedTPS(inner);
  right_->setSpeedTPS(outer);
  return true;
}

bool MotionController::moveRight90() {
  if (!startPrimitive_(MOTION_MOVE_RIGHT_90)) return false;
  tof_->resetCenterPid();
  const float outer = cfg_.cornerMoveSpeedTps;
  const float inner = cfg_.cornerMoveSpeedTps * cfg_.cornerInnerWheelRatio;
  left_->setSpeedTPS(outer);
  right_->setSpeedTPS(inner);
  return true;
}

void MotionController::limitMoveCellTargetCount(uint8_t cells) {
  if (primitive_ != MOTION_MOVE_MULTI_CELL || status_ != MOTION_RUNNING_PRIMITIVE) return;
  if (cells < 1) cells = 1;
  if (cells < moveCellTargetCount_) {
    moveCellTargetCount_ = cells;
  }
}

bool MotionController::moveForwardShort() {
  if (!startPrimitive_(MOTION_MOVE_FORWARD_SHORT)) return false;
  left_->setSpeedTPS(cfg_.shortForwardSpeedTps);
  right_->setSpeedTPS(cfg_.shortForwardSpeedTps);
  return true;
}

bool MotionController::moveBackwardShort() {
  if (!startPrimitive_(MOTION_MOVE_BACKWARD_SHORT)) return false;
  left_->setSpeedTPS(-cfg_.reverseSpeedTps);
  right_->setSpeedTPS(-cfg_.reverseSpeedTps);
  return true;
}

bool MotionController::snapCenter() {
  if (!startPrimitive_(MOTION_SNAP_CENTER)) return false;
  snapCenterPhase_ = SNAP_CENTER_PHASE_BACK;
  left_->setSpeedTPS(-cfg_.reverseSpeedTps);
  right_->setSpeedTPS(-cfg_.reverseSpeedTps);
  return true;
}

bool MotionController::turnLeft90() {
  if (!startPrimitive_(MOTION_TURN_LEFT_90)) return false;
  left_->setSpeedTPS(-cfg_.turnSpeedTps);
  right_->setSpeedTPS(cfg_.turnSpeedTps);
  return true;
}

bool MotionController::turnRight90() {
  if (!startPrimitive_(MOTION_TURN_RIGHT_90)) return false;
  left_->setSpeedTPS(cfg_.turnSpeedTps);
  right_->setSpeedTPS(-cfg_.turnSpeedTps);
  return true;
}

bool MotionController::turn180() {
  if (!startPrimitive_(MOTION_TURN_180)) return false;
  left_->setSpeedTPS(cfg_.turnSpeedTps);
  right_->setSpeedTPS(-cfg_.turnSpeedTps);
  return true;
}

void MotionController::stop() {
  if (!left_ || !right_) return;
  if (tof_) tof_->setStraightTrackMode(MultiVL53L0X::TRACK_NONE);
  left_->coastStop();
  right_->coastStop();

  if (primitive_ == MOTION_NONE) {
    status_ = MOTION_IDLE;
  } else {
    primitive_ = MOTION_STOP;
    status_ = MOTION_RUNNING_PRIMITIVE;
    startedMs_ = millis();
  }
}

void MotionController::abort(const String& reason) {
  if (!left_ || !right_) return;
  if (tof_) tof_->setStraightTrackMode(MultiVL53L0X::TRACK_NONE);
  left_->hardStop();
  right_->hardStop();
  primitive_ = MOTION_NONE;
  lastFinishedPrimitive_ = MOTION_NONE;
  status_ = MOTION_ABORTED;
  lastError_ = reason;
  resetSnapState_();
}

void MotionController::clearCompletionState() {
  if (status_ == MOTION_RUNNING_PRIMITIVE) return;
  if (status_ == MOTION_IDLE) return;
  if (tof_) tof_->setStraightTrackMode(MultiVL53L0X::TRACK_NONE);
  status_ = MOTION_IDLE;
  primitive_ = MOTION_NONE;
  lastFinishedPrimitive_ = MOTION_NONE;
  lastError_ = "";
  resetSnapState_();
}

void MotionController::setUseLatchedTrackMode(bool en) {
  useLatchedTrackMode_ = en;
  straightTrackModeLatched_ = false;
  if (tof_ && !useLatchedTrackMode_) {
    tof_->setStraightTrackMode(MultiVL53L0X::TRACK_NONE);
  }
}

MultiVL53L0X::StraightTrackMode MotionController::chooseStraightTrackMode_(const WallObservation& walls) const {
  if (walls.leftValid && walls.rightValid) return MultiVL53L0X::TRACK_DUAL;
  if (walls.leftValid) return MultiVL53L0X::TRACK_LEFT;
  if (walls.rightValid) return MultiVL53L0X::TRACK_RIGHT;
  return MultiVL53L0X::TRACK_NONE;
}

void MotionController::latchStraightTrackMode(const WallObservation& walls) {
  if (!tof_) return;
  if (!useLatchedTrackMode_) return;
  straightTrackMode_ = chooseStraightTrackMode_(walls);
  straightTrackModeLatched_ = true;
  tof_->setStraightTrackMode(straightTrackMode_);
}

float MotionController::averageProgressMm_() const {
  const int32_t leftDelta = left_->getTicks() - startLeftTicks_;
  const int32_t rightDelta = right_->getTicks() - startRightTicks_;
  return 0.5f * (leftDelta + rightDelta) * cfg_.mmPerTick;
}

float MotionController::absoluteAverageProgressMm_() const {
  return fabsf(averageProgressMm_());
}

int32_t MotionController::differentialTicks_() const {
  const int32_t leftDelta = left_->getTicks() - startLeftTicks_;
  const int32_t rightDelta = right_->getTicks() - startRightTicks_;
  return rightDelta - leftDelta;
}

void MotionController::markDone_(MotionStatus status, const String& reason) {
  if (status != MOTION_COMPLETED || stopOnCompletion_) {
    left_->hardStop();
    right_->hardStop();
  }
  if (tof_) tof_->setStraightTrackMode(MultiVL53L0X::TRACK_NONE);
  status_ = status;
  lastError_ = reason;
  lastFinishedPrimitive_ = primitive_;
  primitive_ = MOTION_NONE;
  resetSnapState_();
}

void MotionController::update(RobotState& state) {
  if (!left_ || !right_) return;

  state.activePrimitive = primitive_;
  state.motionStatus = status_;

  if (status_ != MOTION_RUNNING_PRIMITIVE) return;

  const bool v3ControlAdjustmentsEnabled =
    (state.mode == ROBOT_MODE_EXPLORE) ||
    (state.mode == ROBOT_MODE_SPEED_RUN && state.speedRunPhase == 1);
  tof_->setV3CompensationEnabled(v3ControlAdjustmentsEnabled);

  const uint32_t now = millis();
  uint32_t primitiveTimeoutMs = cfg_.primitiveTimeoutMs;
  if (primitive_ == MOTION_MOVE_MULTI_CELL && moveCellTargetCount_ > 1) {
    primitiveTimeoutMs += (uint32_t)(moveCellTargetCount_ - 1) * cfg_.corridorTimeoutPerCellMs;
  }

  if ((uint32_t)(now - startedMs_) > primitiveTimeoutMs) {
    markDone_(MOTION_FAILED, "primitive timeout");
    return;
  }

  if (primitive_ == MOTION_MOVE_ONE_CELL) {
    const float progressMm = averageProgressMm_();
    state.pose.forwardProgressMm = progressMm;

    const WallObservation& walls = state.walls;
    if (useLatchedTrackMode_ && !straightTrackModeLatched_) {
      latchStraightTrackMode(walls);
    } else if (!useLatchedTrackMode_) {
      tof_->setStraightTrackMode(chooseStraightTrackMode_(walls));
    }
    const bool shouldFrontStop = frontStopReached_(walls, cfg_.frontStopMm);

    float correction = 0.0f;
    if (walls.leftValid || walls.rightValid) {
      correction = tof_->computeError(0.0f) * cfg_.centeringGain;
    }

    const float commandedSpeed = frontApproachSpeedTps_(cfg_.moveSpeedTps, walls, cfg_.frontStopMm);
    left_->setSpeedTPS(commandedSpeed + correction);
    right_->setSpeedTPS(commandedSpeed - correction);

    if (progressMm >= cfg_.cellDistanceMm || shouldFrontStop) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (updateProgressOrFail_(progressMm, now, "move stall")) return;
  } else if (primitive_ == MOTION_MOVE_MULTI_CELL) {
    const float progressMm = averageProgressMm_();
    const float targetDistanceMm = cfg_.cellDistanceMm * (float)moveCellTargetCount_;
    const float finalCellStartMm = max(0.0f, targetDistanceMm - cfg_.cellDistanceMm);
    const float frontStopThresholdMm =
      (moveCellTargetCount_ <= 1) ? cfg_.frontStopMm : cfg_.corridorFrontStopMm;
    state.pose.forwardProgressMm = progressMm;

    const WallObservation& walls = state.walls;
    if (useLatchedTrackMode_ && !straightTrackModeLatched_) {
      latchStraightTrackMode(walls);
    } else if (!useLatchedTrackMode_) {
      tof_->setStraightTrackMode(chooseStraightTrackMode_(walls));
    }
    const bool inFinalCell = progressMm >= finalCellStartMm;
    const bool shouldFrontStop = inFinalCell &&
                                 frontStopReached_(walls, frontStopThresholdMm);

    float correction = 0.0f;
    if (walls.leftValid || walls.rightValid) {
      correction = tof_->computeError(0.0f) * cfg_.corridorCenteringGain;
    }

    const float corridorBase = frontApproachSpeedTps_(cfg_.corridorMoveSpeedTps, walls, frontStopThresholdMm);
    left_->setSpeedTPS(corridorBase + correction);
    right_->setSpeedTPS(corridorBase - correction);

    const bool reachedDistanceTarget = progressMm >= targetDistanceMm;
    const bool shouldComplete = moveEndsAtKnownWall_
                                  ? shouldFrontStop
                                  : (reachedDistanceTarget || shouldFrontStop);

    if (shouldComplete) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (updateProgressOrFail_(progressMm, now, "corridor move stall")) return;
  } else if (primitive_ == MOTION_MOVE_LEFT_90 ||
             primitive_ == MOTION_MOVE_RIGHT_90) {
    const float progressMm = averageProgressMm_();
    const float turnTicks = (float)abs(differentialTicks_());
    state.pose.forwardProgressMm = progressMm;
    state.pose.turnProgressDeg = min(90.0f, (turnTicks / (float)max<int32_t>(1, cfg_.turnTicks90)) * 90.0f);

    const float outer = cfg_.cornerMoveSpeedTps;
    const float inner = cfg_.cornerMoveSpeedTps * cfg_.cornerInnerWheelRatio;
    if (primitive_ == MOTION_MOVE_LEFT_90) {
      left_->setSpeedTPS(inner);
      right_->setSpeedTPS(outer);
    } else {
      left_->setSpeedTPS(outer);
      right_->setSpeedTPS(inner);
    }

    if (progressMm >= cfg_.cellDistanceMm && turnTicks >= (float)max<int32_t>(1, cfg_.turnTicks90)) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (updateProgressOrFail_(progressMm, now, "corner move stall")) return;
  } else if (primitive_ == MOTION_MOVE_FORWARD_SHORT) {
    const float progressMm = averageProgressMm_();
    state.pose.forwardProgressMm = progressMm;

    left_->setSpeedTPS(cfg_.shortForwardSpeedTps);
    right_->setSpeedTPS(cfg_.shortForwardSpeedTps);

    if (progressMm >= cfg_.shortForwardDistanceMm) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (updateProgressOrFail_(progressMm, now, "short forward stall")) return;
  } else if (primitive_ == MOTION_MOVE_BACKWARD_SHORT) {
    const float progressMm = absoluteAverageProgressMm_();
    state.pose.forwardProgressMm = -progressMm;

    left_->setSpeedTPS(-cfg_.reverseSpeedTps);
    right_->setSpeedTPS(-cfg_.reverseSpeedTps);

    if (progressMm >= cfg_.reverseDistanceMm) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (updateProgressOrFail_(progressMm, now, "reverse stall")) return;
  } else if (primitive_ == MOTION_SNAP_CENTER) {
    const bool applyBias = centerBiasActiveForState_(state);
    const float snapReverseTargetMm = cfg_.reverseDistanceMm + (applyBias ? cfg_.centerBiasBackMm : 0.0f);
    const float snapForwardTargetMm = cfg_.shortForwardDistanceMm;
    if (snapCenterPhase_ == SNAP_CENTER_PHASE_BACK) {
      const float progressMm = absoluteAverageProgressMm_();
      state.pose.forwardProgressMm = -progressMm;

      left_->setSpeedTPS(-cfg_.reverseSpeedTps);
      right_->setSpeedTPS(-cfg_.reverseSpeedTps);

      if (progressMm >= snapReverseTargetMm) {
        left_->hardStop();
        right_->hardStop();
        snapCenterPhase_ = SNAP_CENTER_PHASE_HOLD;
        snapCenterHoldUntilMs_ = now + cfg_.snapCenterStopHoldMs;
        startLeftTicks_ = left_->getTicks();
        startRightTicks_ = right_->getTicks();
        lastProgressMm_ = 0.0f;
        lastProgressMs_ = now;
        return;
      }

      if (updateProgressOrFail_(progressMm, now, "snap center reverse stall")) return;
    } else if (snapCenterPhase_ == SNAP_CENTER_PHASE_HOLD) {
      state.pose.forwardProgressMm = 0.0f;
      left_->hardStop();
      right_->hardStop();

      if ((int32_t)(now - snapCenterHoldUntilMs_) >= 0) {
        snapCenterPhase_ = SNAP_CENTER_PHASE_FORWARD;
        startLeftTicks_ = left_->getTicks();
        startRightTicks_ = right_->getTicks();
        lastProgressMm_ = 0.0f;
        lastProgressMs_ = now;
        left_->setSpeedTPS(cfg_.shortForwardSpeedTps);
        right_->setSpeedTPS(cfg_.shortForwardSpeedTps);
      }
      return;
    } else {
      const float progressMm = averageProgressMm_();
      state.pose.forwardProgressMm = progressMm;

      left_->setSpeedTPS(cfg_.shortForwardSpeedTps);
      right_->setSpeedTPS(cfg_.shortForwardSpeedTps);

      if (progressMm >= snapForwardTargetMm) {
        markDone_(MOTION_COMPLETED);
        return;
      }

      if (updateProgressOrFail_(progressMm, now, "snap center forward stall")) return;
    }
  } else if (primitive_ == MOTION_TURN_LEFT_90 ||
             primitive_ == MOTION_TURN_RIGHT_90 ||
             primitive_ == MOTION_TURN_180) {
    const int32_t turnTarget = (primitive_ == MOTION_TURN_180)
      ? ((cfg_.turnTicks180 > 0) ? cfg_.turnTicks180 : max<int32_t>(1, cfg_.turnTicks90 * 2))
      : ((cfg_.turnTicks90 > 0) ? cfg_.turnTicks90 : 1);
    const float turnDegrees = (primitive_ == MOTION_TURN_180) ? 180.0f : 90.0f;
    const float turnRatio = (float)abs(differentialTicks_()) / (float)turnTarget;
    state.pose.turnProgressDeg = min(turnDegrees, turnRatio * turnDegrees);

    if (primitive_ == MOTION_TURN_LEFT_90) {
      left_->setSpeedTPS(-cfg_.turnSpeedTps);
      right_->setSpeedTPS(cfg_.turnSpeedTps);
    } else {
      left_->setSpeedTPS(cfg_.turnSpeedTps);
      right_->setSpeedTPS(-cfg_.turnSpeedTps);
    }

    if (abs(differentialTicks_()) >= turnTarget) {
      markDone_(MOTION_COMPLETED);
      return;
    }
  } else if (primitive_ == MOTION_STOP) {
    state.pose.forwardProgressMm = 0.0f;
    state.pose.turnProgressDeg = 0.0f;
    if (fabsf(left_->getTicksPerSecond()) <= cfg_.stopTps &&
        fabsf(right_->getTicksPerSecond()) <= cfg_.stopTps) {
      markDone_(MOTION_COMPLETED);
      return;
    }
  }

  state.motionStatus = status_;
  state.activePrimitive = primitive_;
}
