#pragma once

#include <Arduino.h>

#include "DcMotor.h"
#include "FloodFillExplorer.h"
#include "MotionController.h"

// Easy firmware profile switch (no build flags needed):
// 0 = FULL firmware (WiFi/OTA/web enabled by config defaults)
// 1 = LITE firmware (minimal runtime, WiFi/OTA/web disabled by default)
#ifndef APP_LITE_FIRMWARE
#define APP_LITE_FIRMWARE 0
#endif

namespace AppConfig {
namespace Build {
// Build profile selected by compile flag.
// Controlled by APP_LITE_FIRMWARE above.
extern const bool LITE_FIRMWARE;
extern const char* PROFILE_NAME;
}

namespace Battery {
// Battery ADC input pin.
// Affects: Battery.cpp sampling and motion safety gating.
extern const uint8_t ADC_PIN;

// Physical divider currently expected:
// battery+ -> 47k -> ADC node -> 18k -> GND
// Divider ratio at ADC ~= 18 / (47 + 18) = 0.2769
// Battery voltage ~= ADC voltage * 3.6111
// This keeps a 2S pack in a safe ADC input range.
extern const float DIVIDER_R_TOP_KOHM;
extern const float DIVIDER_R_BOTTOM_KOHM;

// Two-point ADC calibration.
// Measure pack voltage with a multimeter and record the matching ADC raw values.
// Affects: reported battery voltage and battery percentage.
extern const uint16_t RAW_LOW;
extern const uint16_t RAW_HIGH;
extern const float VOLTAGE_LOW;
extern const float VOLTAGE_HIGH;

// Runtime battery safety thresholds.
// WARNING: robot may still operate but should be watched closely.
// CRITICAL: motion is blocked/aborted for safety.
// Affects: readyForMotion, fault behavior, primitive aborts.
extern const float WARNING_VOLTAGE;
extern const float CRITICAL_VOLTAGE;
}

namespace Maze {
// Robot start pose in maze cell coordinates.
// Affects: initial floodfill pose, web explorer pose, reset behavior.
extern const uint8_t START_X;
extern const uint8_t START_Y;
extern const FloodFillExplorer::Dir START_HEADING;

// Home rectangle used by floodfill target toggling during explore loops.
// This can differ from the single physical start pose.
extern const uint8_t HOME_X0;
extern const uint8_t HOME_Y0;
extern const uint8_t HOME_W;
extern const uint8_t HOME_H;

// Goal rectangle for floodfill.
// Typical micromouse center goal is 2x2.
// Affects: planner target and floodfill distance field.
extern const uint8_t GOAL_X0;
extern const uint8_t GOAL_Y0;
extern const uint8_t GOAL_W;
extern const uint8_t GOAL_H;
}

namespace Wifi {
// Development Wi-Fi / OTA / web logging settings.
// Affects: WiFiOtaWebSerial startup, OTA hostname, web serial availability.
extern const char* SSID;
extern const char* PASS;
extern const char* HOSTNAME;
// Set false to disable the HTTP web log on port 80.
// OTA and the TCP debug console can still remain enabled.
extern const bool ENABLE_WEB_LOG;
// Simple firmware upload page for browser-based wireless updates.
extern const bool ENABLE_UPLOAD_WEB;
extern const uint16_t UPLOAD_WEB_PORT;
// Plain TCP debug/command console.
// Connect with telnet, PuTTY raw TCP, or `nc <ip> <port>`.
extern const uint16_t DEBUG_TCP_PORT;

// FreeRTOS task placement/settings for Wi-Fi service loop.
// Usually only change these if Wi-Fi/OTA becomes unstable.
extern const BaseType_t CORE;
extern const UBaseType_t TASK_PRIORITY;
extern const uint32_t TASK_STACK;
extern const uint32_t SERVICE_DELAY_MS;
// OTA reliability knobs. Increase connect timeout if Wi-Fi takes longer to join.
extern const uint32_t CONNECT_TIMEOUT_MS;
// Retry interval when Wi-Fi drops after boot.
extern const uint32_t RECONNECT_INTERVAL_MS;
  }

namespace Debug {
  // Global serial output switch.
  // Set false to mute Serial.print/println output across the app while still
  // allowing the serial port to be opened for input if needed.
  extern const bool ENABLE_SERIAL_OUTPUT;

// Compact status line option.
// Set false to keep status prints but hide motor TPS values.
// Affects: periodic `status`/telemetry output formatting only.
extern const bool PRINT_STATUS_TPS;
}

namespace I2C {
// ESP32 I2C pins used for the TOF bus.
// Affects: Wire.begin() and I2C recovery.
extern const uint8_t SDA;
extern const uint8_t SCL;

// Intended I2C bus speed.
// Currently documented here for clarity; use this if/when bus speed is centralized.
extern const uint32_t CLOCK_HZ;
}

namespace Tof {
// PCF8574 I/O expander address used to control XSHUT pins.
// Affects: TOF power-up and address assignment.
extern const uint8_t PCF_ADDRESS;

// Number of configured TOF sensors and update cadence.
// Affects: MultiVL53L0X initialization and polling behavior.
extern const uint8_t SENSOR_COUNT;
extern const uint16_t UPDATE_INTERVAL_MS;
// If true, heading-error PID is computed only once per full TOF sensor sweep.
// This avoids mixing old/new sensor samples in the same control update.
extern const bool COMPUTE_HEADING_FROM_FULL_SWEEP;

// Distance threshold for wall detection.
// Smaller = more conservative wall detection.
// Larger = walls detected earlier/farther away.
// Affects: left/front/right wall booleans used by motion + planner.
extern const uint16_t WALL_THRESHOLD_MM;

// Sensor distance validity window and sentinel values.
// DIST_FAR represents a valid "clear / far" reading beyond the usable range.
// DIST_ERROR represents an invalid/error sentinel for internal fusion paths.
extern const uint16_t DIST_MIN_VALID_MM;
extern const uint16_t DIST_MAX_VALID_MM;
extern const uint16_t DIST_FAR_MM;
extern const uint16_t DIST_ERROR_MM;

// XSHUT control pins on the PCF8574, one per sensor.
// Order matters because it must match SENSOR_ADDR and physical mounting order.
extern const uint8_t XSHUT_PINS[];

// Final I2C addresses assigned to each sensor during startup.
// Order matters and must match the physical sensor order expected by MultiVL53L0X.
extern const uint8_t SENSOR_ADDR[];

// Per-sensor linear calibration after distance calibration.
// Corrected(mm) = raw(mm) * SENSOR_SCALE[i] + SENSOR_OFFSET_MM[i]
// Index map:
// - V1: 0=LEFT, 2=FRONT, 4=RIGHT
// - V2: 0=FL, 1=LEFT, 2=RIGHT, 3=FR, 4=spare
// Only first SENSOR_COUNT entries are used.
extern const float SENSOR_SCALE[8];
extern const int16_t SENSOR_OFFSET_MM[8];

// Low-pass smoothing for per-sensor distance updates.
// update = prev * DIST_LPF_PREV_WEIGHT + sample * DIST_LPF_SAMPLE_WEIGHT
extern const float DIST_LPF_PREV_WEIGHT;
extern const float DIST_LPF_SAMPLE_WEIGHT;

}

namespace Motors {
// Right motor pin mapping and encoder polarity.
// Affects: low-level motor direction, encoder tick sign, and all motion control.
extern const DcMotor::Pins RIGHT_PINS;

// Left motor pin mapping and encoder polarity.
// `invertDir` or `invertEnc` are common first-upload tuning points.
extern const DcMotor::Pins LEFT_PINS;

// LEDC channels and PWM setup.
// Affects: motor drive generation on ESP32.
extern const uint8_t LEFT_PWM_CHANNEL;
extern const uint8_t RIGHT_PWM_CHANNEL;
extern const uint32_t PWM_FREQ;
extern const uint8_t PWM_RESOLUTION_BITS;

// Wheel speed PID defaults.
// Affects: how aggressively each wheel tracks target ticks/sec.
// Tune only after verifying motor direction and encoder polarity.
extern const float PID_KP;
extern const float PID_KI;
extern const float PID_KD;
extern const float PID_OUT_LIMIT;
extern const float PID_I_LIMIT;
extern const float PID_D_FILTER_HZ;
extern const float PID_SLEW_RATE;
// Minimum absolute PWM duty used by speed PID when target TPS is non-zero.
// Helps avoid weak-duty region where one motor starts later than the other.
extern const uint16_t PID_MIN_DRIVE_DUTY;

// Low-pass smoothing for encoder speed estimate in DcMotor::update().
// _tps += TPS_LPF_ALPHA * (instant - _tps)
extern const float TPS_LPF_ALPHA;
// Window for period-based TPS estimate (ticks accumulated across this time).
// Larger = smoother/noisier tradeoff.
extern const uint32_t TPS_ESTIMATE_WINDOW_MS;
}

namespace Motion {
// Estimated forward travel for one maze cell.
// Affects: when moveOneCell() decides the move is complete.
// One of the most important hardware tuning values.
extern const float CELL_DISTANCE_MM;

// Differential wheel travel (mm) needed for turn completion.
// Affects: turnLeft90() / turnRight90() / turn180() completion.
// Tune left/right independently to compensate turn asymmetry.
extern const float TURN_LEFT_90_MM;
extern const float TURN_RIGHT_90_MM;
// Keep this separate from 2x90 so you can tune U-turns independently.
extern const float TURN_180_MM;

// Nominal primitive speeds in ticks/sec.
// Affects: how fast the robot attempts straight moves and turns.
extern const float MOVE_SPEED_TPS;
extern const float CORRIDOR_MOVE_SPEED_TPS;
// Short forward settle after a snap-back. Intended for explore-only recentering.
extern const float SHORT_FORWARD_DISTANCE_MM;
extern const float SHORT_FORWARD_SPEED_TPS;
// Short reverse primitive used for manual alignment and future turn recentering work.
extern const float REVERSE_DISTANCE_MM;
extern const float REVERSE_SPEED_TPS;
// Hold time between snapcenter reverse hard-stop and forward restart.
// Affects: how long the robot pauses after backing up before returning to center.
extern const uint32_t SNAP_CENTER_STOP_HOLD_MS;
extern const float TURN_SPEED_TPS;
// Turn slowdown profile to reduce overshoot near 90/180 target ticks.
// Once progress reaches TURN_SLOWDOWN_START_RATIO of target ticks, reduce to
// TURN_MIN_SPEED_TPS for final approach.
extern const float TURN_MIN_SPEED_TPS;
extern const float TURN_SLOWDOWN_START_RATIO;

// Wall-centering correction gain while driving straight.
// Higher = stronger correction, but too high can oscillate.
// Affects: corridor following stability.
extern const float CENTERING_GAIN;
extern const float CORRIDOR_CENTERING_GAIN;
// Extra gain applied only on the side that should slow down from centering correction.
// 1.0 = symmetric behavior, 2.0 = slowing side is reduced 2x.
extern const float CENTERING_SLOW_SIDE_GAIN;
// Gain applied on the side that would speed up from centering correction.
// 0.0 = keep fast side at base speed (no speed-up).
extern const float CENTERING_FAST_SIDE_GAIN;
extern const float CENTER_TARGET_LEFT_MM;
extern const float CENTER_TARGET_RIGHT_MM;
extern const float CENTER_TARGET_CAPTURE_WINDOW_MM;
extern const float CENTER_PID_KP;
extern const float CENTER_PID_KI;
extern const float CENTER_PID_KD;
extern const float CENTER_PID_I_LIMIT;
extern const float CENTER_PID_OUT_LIMIT;
// Clamp one-wall tracking target error to reduce large steering spikes
// when the opposite side is open/far.
extern const float CENTER_PID_SINGLE_WALL_ERR_LIMIT_MM;
// Clamp derivative magnitude to avoid large D kicks on abrupt sensor transitions.
extern const float CENTER_PID_DERIV_LIMIT;
// Clamp side-wall distance used by center PID math only.
// Sensor data can remain valid farther than this, but PID error uses this max.
extern const uint16_t CENTER_PID_EFFECTIVE_SIDE_MAX_MM;
// Low-pass time constants (seconds) for wall-centering blend and raw error smoothing.
extern const float CENTER_BLEND_TAU_SEC;
extern const float CENTER_RAW_TAU_SEC;

// If a front wall is seen this close near the end of a move, stop early.
// Affects: wall approach safety and cell alignment.
extern const float FRONT_STOP_MM;
extern const float CORRIDOR_FRONT_STOP_MM;
// Distance-based slowdown near motion target distance.
// Example: 0.85 means start slowing after 85% of target distance is reached.
extern const float DISTANCE_APPROACH_START_RATIO;
extern const float DISTANCE_APPROACH_MIN_SPEED_TPS;
extern const float FRONT_APPROACH_START_FACTOR;
extern const float FRONT_APPROACH_MIN_SPEED_TPS;

// Primitive fault timing.
// Affects: when moves/turns fail due to timeout or lack of progress.
extern const uint32_t PRIMITIVE_TIMEOUT_MS;
extern const uint32_t CORRIDOR_TIMEOUT_PER_CELL_MS;
extern const uint32_t STALL_TIMEOUT_MS;
extern const uint8_t CORRIDOR_MAX_CELLS;

// Primitive completion thresholds.
// STOP_TPS: considered stopped when wheel speed falls below this.
// MIN_PROGRESS_MM: minimum progress before stall timer is refreshed.
extern const float STOP_TPS;
extern const float MIN_PROGRESS_MM;
// Stop mode used when a primitive completes in normal motion flow
// (explore / speedrun 1). Choose one of: COAST, BRAKE, HARDSTOP.
extern const MotionController::StopMode COMPLETION_STOP_MODE;

// Mechanical distance-per-tick estimate per wheel.
// Affects: forward progress estimation and one-cell completion.
// Tune left/right independently to reduce long straight drift.
extern const float LEFT_MM_PER_TICK;
extern const float RIGHT_MM_PER_TICK;
// Print known maze as ASCII after exploration updates the map.
extern const bool AUTO_PRINT_MAZE_AFTER_SENSE;
// Hold the motors in hard-stop briefly after a primitive completes.
// Affects: how long the robot fully settles before wall sensing and the next action.
extern const uint32_t POST_MOTION_HARD_STOP_HOLD_MS;
}

namespace Explorer {
// Floodfill web explorer settings.
// Affects: debug UI on the network and action ACK timeout behavior.
// Set false to disable the floodfill web UI on port 81 while keeping floodfill logic active.
extern const bool ENABLE_WEB;
extern const uint16_t PORT;
extern const uint16_t WS_PORT;
extern const bool AUTO_RUN;
extern const uint32_t ACK_TIMEOUT_MS;
extern const bool PAUSE_ON_ACK_TIMEOUT;
// In explore mode, keep looping between original goal and original start
// after each target is reached. This helps continue discovering alternate
// walls and improving the path without resetting pose.
extern const bool CONTINUE_AFTER_GOAL;
// Mark the shortest path as known after this many consecutive
// goal->home round trips report the same best-known start->goal cost.
extern const uint8_t SHORTEST_PATH_STABLE_ROUND_TRIPS;
}

namespace SpeedRun2 {
// Dedicated one-way shortest-path motion profile.
// Starts with the current stable speedrun tuning so it can be tuned independently later.
extern const float MOVE_SPEED_TPS;
extern const float CORRIDOR_MOVE_SPEED_TPS;
extern const float TURN_SPEED_TPS;
extern const float CENTERING_GAIN;
extern const float CORRIDOR_CENTERING_GAIN;
extern const float FRONT_STOP_MM;
extern const float CORRIDOR_FRONT_STOP_MM;
}

namespace Inputs {
// Built-in BOOT button multi-press launcher on ESP32-S3 GPIO0.
extern const bool ENABLE_BOOT_BUTTON_LAUNCH;
extern const uint8_t BOOT_BUTTON_PIN;
extern const bool BOOT_BUTTON_ACTIVE_LOW;
extern const uint32_t BOOT_BUTTON_DEBOUNCE_MS;
extern const uint32_t BOOT_BUTTON_MULTI_PRESS_TIMEOUT_MS;
}

namespace Tasks {
// Main periodic task cadences (milliseconds).
// Affects: scheduler pacing and loop watchdog expected periods.
extern const uint32_t USER_LOOP_PERIOD_MS;
extern const uint32_t PLANNER_LOOP_PERIOD_MS;
extern const uint32_t MOTOR_LOOP_PERIOD_MS;
extern const uint32_t EXPLORER_LOOP_PERIOD_MS;
extern const uint32_t TOF_LOOP_PERIOD_MS;
extern const uint32_t TELEMETRY_LOOP_PERIOD_MS;
}

namespace Debug {
  // Additional runtime flow logging for motion, snap, and wall application.
  // Affects: extra serial/TCP debug output only; no behavior changes.
  extern const bool DEBUG_MOTION_FLOW;
  // Dedicated gate for debugMotionEvent() start/end primitive logs.
  extern const bool DEBUG_MOTION_EVENT;
  // Gate for automatic maze ASCII dumps via maze_debug_s().
  extern const bool DEBUG_MAZE_PRINT;
  extern const bool DEBUG_WALL_APPLY;
  // High-rate center PID trace from MultiVL53L0X::computeError().
  extern const bool CENTER_PID_TRACE;
  // Print every N center-PID updates (1 = every update).
  extern const uint8_t CENTER_PID_TRACE_EVERY_N;
  // High-rate motor PID trace from DcMotor::update().
  // Prints one line per motor update with target/tps/err/P/I/D/out/duty.
  extern const bool MOTOR_PID_TRACE;
  // Print every N motor updates (1 = every update).
  extern const uint16_t MOTOR_PID_TRACE_EVERY_N;
  // Lightweight periodic-task timing watchdog.
  // Warns when a watched loop runs later than its expected cadence.
  extern const bool ENABLE_LOOP_WATCHDOG;
  extern const uint32_t LOOP_WATCHDOG_TOLERANCE_MS;
  extern const uint32_t LOOP_WATCHDOG_RATE_LIMIT_MS;
}

// Helper that converts config constants into the runtime motion controller config.
// Update this if MotionController::Config grows new fields.
inline MotionController::Config makeMotionConfig() {
  MotionController::Config cfg;
  cfg.cellDistanceMm = Motion::CELL_DISTANCE_MM;
  cfg.turnLeft90Mm = Motion::TURN_LEFT_90_MM;
  cfg.turnRight90Mm = Motion::TURN_RIGHT_90_MM;
  cfg.turn180Mm = Motion::TURN_180_MM;
  cfg.moveSpeedTps = Motion::MOVE_SPEED_TPS;
  cfg.corridorMoveSpeedTps = Motion::CORRIDOR_MOVE_SPEED_TPS;
  cfg.shortForwardDistanceMm = Motion::SHORT_FORWARD_DISTANCE_MM;
  cfg.shortForwardSpeedTps = Motion::SHORT_FORWARD_SPEED_TPS;
  cfg.reverseDistanceMm = Motion::REVERSE_DISTANCE_MM;
  cfg.reverseSpeedTps = Motion::REVERSE_SPEED_TPS;
  cfg.snapCenterStopHoldMs = Motion::SNAP_CENTER_STOP_HOLD_MS;
  cfg.turnSpeedTps = Motion::TURN_SPEED_TPS;
  cfg.turnMinSpeedTps = Motion::TURN_MIN_SPEED_TPS;
  cfg.turnSlowdownStartRatio = Motion::TURN_SLOWDOWN_START_RATIO;
  cfg.centeringGain = Motion::CENTERING_GAIN;
  cfg.corridorCenteringGain = Motion::CORRIDOR_CENTERING_GAIN;
  cfg.centeringSlowSideGain = Motion::CENTERING_SLOW_SIDE_GAIN;
  cfg.centeringFastSideGain = Motion::CENTERING_FAST_SIDE_GAIN;
  cfg.frontStopMm = Motion::FRONT_STOP_MM;
  cfg.corridorFrontStopMm = Motion::CORRIDOR_FRONT_STOP_MM;
  cfg.distanceApproachStartRatio = Motion::DISTANCE_APPROACH_START_RATIO;
  cfg.distanceApproachMinSpeedTps = Motion::DISTANCE_APPROACH_MIN_SPEED_TPS;
  cfg.frontApproachStartFactor = Motion::FRONT_APPROACH_START_FACTOR;
  cfg.frontApproachMinSpeedTps = Motion::FRONT_APPROACH_MIN_SPEED_TPS;
  cfg.primitiveTimeoutMs = Motion::PRIMITIVE_TIMEOUT_MS;
  cfg.corridorTimeoutPerCellMs = Motion::CORRIDOR_TIMEOUT_PER_CELL_MS;
  cfg.stallTimeoutMs = Motion::STALL_TIMEOUT_MS;
  cfg.stopTps = Motion::STOP_TPS;
  cfg.minProgressMm = Motion::MIN_PROGRESS_MM;
  cfg.completionStopMode = Motion::COMPLETION_STOP_MODE;
  cfg.leftMmPerTick = Motion::LEFT_MM_PER_TICK;
  cfg.rightMmPerTick = Motion::RIGHT_MM_PER_TICK;
  cfg.corridorMaxCells = Motion::CORRIDOR_MAX_CELLS;
  return cfg;
}

inline MotionController::Config makeSpeedRun2MotionConfig() {
  MotionController::Config cfg = makeMotionConfig();
  cfg.moveSpeedTps = SpeedRun2::MOVE_SPEED_TPS;
  cfg.corridorMoveSpeedTps = SpeedRun2::CORRIDOR_MOVE_SPEED_TPS;
  cfg.turnSpeedTps = SpeedRun2::TURN_SPEED_TPS;
  cfg.centeringGain = SpeedRun2::CENTERING_GAIN;
  cfg.corridorCenteringGain = SpeedRun2::CORRIDOR_CENTERING_GAIN;
  cfg.frontStopMm = SpeedRun2::FRONT_STOP_MM;
  cfg.corridorFrontStopMm = SpeedRun2::CORRIDOR_FRONT_STOP_MM;
  return cfg;
}

// Helper that converts config constants into the floodfill explorer runtime config.
inline FloodFillExplorer::Config makeExplorerConfig() {
  FloodFillExplorer::Config cfg;
  cfg.enableWeb = Explorer::ENABLE_WEB;
  cfg.port = Explorer::PORT;
  cfg.wsPort = Explorer::WS_PORT;
  cfg.autoRun = Explorer::AUTO_RUN;
  cfg.maxForwardCells = Motion::CORRIDOR_MAX_CELLS;
  cfg.ackTimeoutMs = Explorer::ACK_TIMEOUT_MS;
  cfg.pauseOnAckTimeout = Explorer::PAUSE_ON_ACK_TIMEOUT;
  return cfg;
}

}  // namespace AppConfig

