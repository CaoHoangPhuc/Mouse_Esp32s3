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
  snapCenterHoldUntilMs_ = 0;
  lastProgressMm_ = 0.0f;
  snapCenterPhase_ = SNAP_CENTER_PHASE_NONE;
  return true;
}

bool MotionController::moveOneCell() {
  if (!startPrimitive_(MOTION_MOVE_ONE_CELL)) return false;
  left_->setSpeedTPS(cfg_.moveSpeedTps);
  right_->setSpeedTPS(cfg_.moveSpeedTps);
  return true;
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
  left_->hardStop();
  right_->hardStop();

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
  left_->hardStop();
  right_->hardStop();
  primitive_ = MOTION_NONE;
  lastFinishedPrimitive_ = MOTION_NONE;
  status_ = MOTION_ABORTED;
  lastError_ = reason;
  snapCenterHoldUntilMs_ = 0;
  snapCenterPhase_ = SNAP_CENTER_PHASE_NONE;
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
  left_->hardStop();
  right_->hardStop();
  status_ = status;
  lastError_ = reason;
  lastFinishedPrimitive_ = primitive_;
  primitive_ = MOTION_NONE;
  snapCenterHoldUntilMs_ = 0;
  snapCenterPhase_ = SNAP_CENTER_PHASE_NONE;
}

void MotionController::update(RobotState& state) {
  if (!left_ || !right_) return;

  state.activePrimitive = primitive_;
  state.motionStatus = status_;

  if (status_ != MOTION_RUNNING_PRIMITIVE) return;

  if (battery_ && battery_->state() == Battery::BATTERY_CRITICAL) {
    markDone_(MOTION_ABORTED, "battery critical");
    return;
  }

  const uint32_t now = millis();
  if ((uint32_t)(now - startedMs_) > cfg_.primitiveTimeoutMs) {
    markDone_(MOTION_FAILED, "primitive timeout");
    return;
  }

  if (primitive_ == MOTION_MOVE_ONE_CELL) {
    const float progressMm = averageProgressMm_();
    state.pose.forwardProgressMm = progressMm;

    const WallObservation& walls = state.walls;
    bool shouldFrontStop = walls.frontValid && walls.frontWall && walls.frontMm > 0 &&
                           walls.frontMm <= cfg_.frontStopMm;

    float correction = 0.0f;
    if (walls.leftValid || walls.rightValid) {
      correction = tof_->computeError(0.0f) * cfg_.centeringGain;
    }

    left_->setSpeedTPS(cfg_.moveSpeedTps + correction);
    right_->setSpeedTPS(cfg_.moveSpeedTps - correction);

    if (progressMm >= cfg_.cellDistanceMm || shouldFrontStop) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (progressMm > lastProgressMm_ + cfg_.minProgressMm) {
      lastProgressMm_ = progressMm;
      lastProgressMs_ = now;
    } else if ((uint32_t)(now - lastProgressMs_) > cfg_.stallTimeoutMs) {
      markDone_(MOTION_FAILED, "move stall");
      return;
    }
  } else if (primitive_ == MOTION_MOVE_FORWARD_SHORT) {
    const float progressMm = averageProgressMm_();
    state.pose.forwardProgressMm = progressMm;

    left_->setSpeedTPS(cfg_.shortForwardSpeedTps);
    right_->setSpeedTPS(cfg_.shortForwardSpeedTps);

    if (progressMm >= cfg_.shortForwardDistanceMm) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (progressMm > lastProgressMm_ + cfg_.minProgressMm) {
      lastProgressMm_ = progressMm;
      lastProgressMs_ = now;
    } else if ((uint32_t)(now - lastProgressMs_) > cfg_.stallTimeoutMs) {
      markDone_(MOTION_FAILED, "short forward stall");
      return;
    }
  } else if (primitive_ == MOTION_MOVE_BACKWARD_SHORT) {
    const float progressMm = absoluteAverageProgressMm_();
    state.pose.forwardProgressMm = -progressMm;

    left_->setSpeedTPS(-cfg_.reverseSpeedTps);
    right_->setSpeedTPS(-cfg_.reverseSpeedTps);

    if (progressMm >= cfg_.reverseDistanceMm) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (progressMm > lastProgressMm_ + cfg_.minProgressMm) {
      lastProgressMm_ = progressMm;
      lastProgressMs_ = now;
    } else if ((uint32_t)(now - lastProgressMs_) > cfg_.stallTimeoutMs) {
      markDone_(MOTION_FAILED, "reverse stall");
      return;
    }
  } else if (primitive_ == MOTION_SNAP_CENTER) {
    if (snapCenterPhase_ == SNAP_CENTER_PHASE_BACK) {
      const float progressMm = absoluteAverageProgressMm_();
      state.pose.forwardProgressMm = -progressMm;

      left_->setSpeedTPS(-cfg_.reverseSpeedTps);
      right_->setSpeedTPS(-cfg_.reverseSpeedTps);

      if (progressMm >= cfg_.reverseDistanceMm) {
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

      if (progressMm > lastProgressMm_ + cfg_.minProgressMm) {
        lastProgressMm_ = progressMm;
        lastProgressMs_ = now;
      } else if ((uint32_t)(now - lastProgressMs_) > cfg_.stallTimeoutMs) {
        markDone_(MOTION_FAILED, "snap center reverse stall");
        return;
      }
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

      if (progressMm >= cfg_.shortForwardDistanceMm) {
        markDone_(MOTION_COMPLETED);
        return;
      }

      if (progressMm > lastProgressMm_ + cfg_.minProgressMm) {
        lastProgressMm_ = progressMm;
        lastProgressMs_ = now;
      } else if ((uint32_t)(now - lastProgressMs_) > cfg_.stallTimeoutMs) {
        markDone_(MOTION_FAILED, "snap center forward stall");
        return;
      }
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
