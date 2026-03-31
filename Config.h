#pragma once

#include <Arduino.h>

#include "DcMotor.h"
#include "FloodFillExplorer.h"
#include "MotionController.h"

namespace AppConfig {

namespace Battery {
// Battery ADC input pin.
// Affects: Battery.cpp sampling and motion safety gating.
static constexpr uint8_t ADC_PIN = 3;

// Physical divider currently expected:
// battery+ -> 47k -> ADC node -> 18k -> GND
// Divider ratio at ADC ~= 18 / (47 + 18) = 0.2769
// Battery voltage ~= ADC voltage * 3.6111
// This keeps a 2S pack in a safe ADC input range.
static constexpr float DIVIDER_R_TOP_KOHM = 56.0f;
static constexpr float DIVIDER_R_BOTTOM_KOHM = 18.0f;

// Two-point ADC calibration.
// Measure pack voltage with a multimeter and record the matching ADC raw values.
// Affects: reported battery voltage and battery percentage.
static constexpr uint16_t RAW_LOW = 2800;
static constexpr uint16_t RAW_HIGH = 3350;
static constexpr float VOLTAGE_LOW = 7.20f;
static constexpr float VOLTAGE_HIGH = 8.40f;

// Runtime battery safety thresholds.
// WARNING: robot may still operate but should be watched closely.
// CRITICAL: motion is blocked/aborted for safety.
// Affects: readyForMotion, fault behavior, primitive aborts.
static constexpr float WARNING_VOLTAGE = 7.10f;
static constexpr float CRITICAL_VOLTAGE = 6.90f;
}

namespace Maze {
// Robot start pose in maze cell coordinates.
// Affects: initial floodfill pose, web explorer pose, reset behavior.
static constexpr uint8_t START_X = 0;
static constexpr uint8_t START_Y = 0;
static constexpr FloodFillExplorer::Dir START_HEADING = FloodFillExplorer::SOUTH;

// Home rectangle used by floodfill target toggling during explore loops.
// This can differ from the single physical start pose.
static constexpr uint8_t HOME_X0 = 0;
static constexpr uint8_t HOME_Y0 = 0;
static constexpr uint8_t HOME_W = 1;
static constexpr uint8_t HOME_H = 1;

// Goal rectangle for floodfill.
// Typical micromouse center goal is 2x2.
// Affects: planner target and floodfill distance field.
static constexpr uint8_t GOAL_X0 = 4;
static constexpr uint8_t GOAL_Y0 = 4;
static constexpr uint8_t GOAL_W = 1;
static constexpr uint8_t GOAL_H = 1;
}

namespace Wifi {
// Development Wi-Fi / OTA / web logging settings.
// Affects: WiFiOtaWebSerial startup, OTA hostname, web serial availability.
static constexpr const char* SSID = "PhucWifi";
static constexpr const char* PASS = "000000001";
static constexpr const char* HOSTNAME = "PhucC_Esp32s3_mice";
// Set false to disable the HTTP web log on port 80.
// OTA and the TCP debug console can still remain enabled.
static constexpr bool ENABLE_WEB_LOG = true;
// Simple firmware upload page for browser-based wireless updates.
static constexpr bool ENABLE_UPLOAD_WEB = true;
static constexpr uint16_t UPLOAD_WEB_PORT = 82;
// Plain TCP debug/command console.
// Connect with telnet, PuTTY raw TCP, or `nc <ip> <port>`.
static constexpr uint16_t DEBUG_TCP_PORT = 2323;

// FreeRTOS task placement/settings for Wi-Fi service loop.
// Usually only change these if Wi-Fi/OTA becomes unstable.
static constexpr BaseType_t CORE = 1;
static constexpr UBaseType_t TASK_PRIORITY = 0;
static constexpr uint32_t TASK_STACK = 10 * 1024;
static constexpr uint32_t SERVICE_DELAY_MS = 5;
// OTA reliability knobs. Increase connect timeout if Wi-Fi takes longer to join.
static constexpr uint32_t CONNECT_TIMEOUT_MS = 15000;
// Retry interval when Wi-Fi drops after boot.
static constexpr uint32_t RECONNECT_INTERVAL_MS = 5000;
  }

namespace Debug {
  // Global serial output switch.
  // Set false to mute Serial.print/println output across the app while still
  // allowing the serial port to be opened for input if needed.
  static constexpr bool ENABLE_SERIAL_OUTPUT = true;

// Compact status line option.
// Set false to keep status prints but hide motor TPS values.
// Affects: periodic `status`/telemetry output formatting only.
static constexpr bool PRINT_STATUS_TPS = false;
}

namespace I2C {
// ESP32 I2C pins used for the TOF bus.
// Affects: Wire.begin() and I2C recovery.
static constexpr uint8_t SDA = 8;
static constexpr uint8_t SCL = 9;

// Intended I2C bus speed.
// Currently documented here for clarity; use this if/when bus speed is centralized.
static constexpr uint32_t CLOCK_HZ = 400000;
}

namespace Tof {
// PCF8574 I/O expander address used to control XSHUT pins.
// Affects: TOF power-up and address assignment.
static constexpr uint8_t PCF_ADDRESS = 0x20;

// Number of configured TOF sensors and update cadence.
// Affects: MultiVL53L0X initialization and polling behavior.
static constexpr uint8_t SENSOR_COUNT = 5;
static constexpr uint16_t UPDATE_INTERVAL_MS = 20;

// Distance threshold for wall detection.
// Smaller = more conservative wall detection.
// Larger = walls detected earlier/farther away.
// Affects: left/front/right wall booleans used by motion + planner.
static constexpr uint16_t WALL_THRESHOLD_MM = 130;

// Sensor distance validity window and sentinel values.
// DIST_FAR represents a valid "clear / far" reading beyond the usable range.
// DIST_ERROR represents an invalid/error sentinel for internal fusion paths.
static constexpr uint16_t DIST_MIN_VALID_MM = 1;
static constexpr uint16_t DIST_MAX_VALID_MM = 200;
static constexpr uint16_t DIST_FAR_MM = DIST_MAX_VALID_MM + 1;
static constexpr uint16_t DIST_ERROR_MM = DIST_MAX_VALID_MM + 2;

// XSHUT control pins on the PCF8574, one per sensor.
// Order matters because it must match SENSOR_ADDR and physical mounting order.
static constexpr uint8_t XSHUT_PINS[SENSOR_COUNT] = {0, 1, 2, 3, 4};

// Final I2C addresses assigned to each sensor during startup.
// Order matters and must match the physical sensor order expected by MultiVL53L0X.
static constexpr uint8_t SENSOR_ADDR[SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33, 0x34};

}

namespace Motors {
// Right motor pin mapping and encoder polarity.
// Affects: low-level motor direction, encoder tick sign, and all motion control.
static constexpr DcMotor::Pins RIGHT_PINS = {
  .in1 = 10,
  .in2 = 11,
  .pwm = 7,
  .encA = 12,
  .encB = 13,
  .invertDir = false,
  .invertEnc = false
};

// Left motor pin mapping and encoder polarity.
// `invertDir` or `invertEnc` are common first-upload tuning points.
static constexpr DcMotor::Pins LEFT_PINS = {
  .in1 = 5,
  .in2 = 6,
  .pwm = 4,
  .encA = 1,
  .encB = 2,
  .invertDir = true,
  .invertEnc = true
};

// LEDC channels and PWM setup.
// Affects: motor drive generation on ESP32.
static constexpr uint8_t LEFT_PWM_CHANNEL = 0;
static constexpr uint8_t RIGHT_PWM_CHANNEL = 1;
static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint8_t PWM_RESOLUTION_BITS = 10;

// Wheel speed PID defaults.
// Affects: how aggressively each wheel tracks target ticks/sec.
// Tune only after verifying motor direction and encoder polarity.
static constexpr float PID_KP = 0.0050f;
static constexpr float PID_KI = 0.0030f;
static constexpr float PID_KD = 0.0004f;
static constexpr float PID_OUT_LIMIT = 0.80f;
static constexpr float PID_I_LIMIT = 0.50f;
static constexpr float PID_D_FILTER_HZ = 25.0f;
static constexpr float PID_SLEW_RATE = 10.0f;
}

namespace Motion {
// Estimated forward travel for one maze cell.
// Affects: when moveOneCell() decides the move is complete.
// One of the most important hardware tuning values.
static constexpr float CELL_DISTANCE_MM = 180.0f;

// Encoder differential ticks needed for a 90-degree turn.
// Affects: turnLeft90() / turnRight90() completion.
// One of the most important hardware tuning values.
static constexpr int32_t TURN_TICKS_90 = 210;
// Encoder differential ticks needed for a 180-degree turn.
// Keep this separate from 2x90 so you can tune U-turns independently.
static constexpr int32_t TURN_TICKS_180 = 430;

// Nominal primitive speeds in ticks/sec.
// Affects: how fast the robot attempts straight moves and turns.
static constexpr float MOVE_SPEED_TPS = 400.0f;
static constexpr float CORRIDOR_MOVE_SPEED_TPS = 500.0f;
// Short forward settle after a snap-back. Intended for explore-only recentering.
static constexpr float SHORT_FORWARD_DISTANCE_MM = 50.0f;
static constexpr float SHORT_FORWARD_SPEED_TPS = 400.0f;
// Short reverse primitive used for manual alignment and future turn recentering work.
static constexpr float REVERSE_DISTANCE_MM = 100.0f;
static constexpr float REVERSE_SPEED_TPS = 400.0f;
// Hold time between snapcenter reverse hard-stop and forward restart.
// Affects: how long the robot pauses after backing up before returning to center.
static constexpr uint32_t SNAP_CENTER_STOP_HOLD_MS = 1;
static constexpr float TURN_SPEED_TPS = 400.0f;

// Wall-centering correction gain while driving straight.
// Higher = stronger correction, but too high can oscillate.
// Affects: corridor following stability.
static constexpr float CENTERING_GAIN = 1.0f;
static constexpr float CORRIDOR_CENTERING_GAIN = 1.0f;
static constexpr float CENTER_TARGET_LEFT_MM = 100.0f;
static constexpr float CENTER_TARGET_RIGHT_MM = 100.0f;
static constexpr float CENTER_TARGET_CAPTURE_WINDOW_MM = 0.0f;
static constexpr float CENTER_PID_KP = 2.0f;
static constexpr float CENTER_PID_KI = 0.01f;
static constexpr float CENTER_PID_KD = 1.0f;
static constexpr float CENTER_PID_I_LIMIT = 40.0f;
static constexpr float CENTER_PID_OUT_LIMIT = 50.0f;

// If a front wall is seen this close near the end of a move, stop early.
// Affects: wall approach safety and cell alignment.
static constexpr float FRONT_STOP_MM = 100.0f;
static constexpr float CORRIDOR_FRONT_STOP_MM = 120.0f;

// Primitive fault timing.
// Affects: when moves/turns fail due to timeout or lack of progress.
static constexpr uint32_t PRIMITIVE_TIMEOUT_MS = 3000;
static constexpr uint32_t CORRIDOR_TIMEOUT_PER_CELL_MS = 1000;
static constexpr uint32_t STALL_TIMEOUT_MS = 700;
static constexpr uint8_t CORRIDOR_MAX_CELLS = 4;

// Primitive completion thresholds.
// STOP_TPS: considered stopped when wheel speed falls below this.
// MIN_PROGRESS_MM: minimum progress before stall timer is refreshed.
static constexpr float STOP_TPS = 20.0f;
static constexpr float MIN_PROGRESS_MM = 12.0f;

// Mechanical distance-per-tick estimate.
// Affects: forward progress estimation and one-cell completion.
// Usually tune this before final CELL_DISTANCE_MM tuning.
static constexpr float MM_PER_TICK = 0.54f;
// Print known maze as ASCII after exploration updates the map.
static constexpr bool AUTO_PRINT_MAZE_AFTER_SENSE = true;
// Hold the motors in hard-stop briefly after a primitive completes.
// Affects: how long the robot fully settles before wall sensing and the next action.
static constexpr uint32_t POST_MOTION_HARD_STOP_HOLD_MS = 1;
}

namespace Explorer {
// Floodfill web explorer settings.
// Affects: debug UI on the network and action ACK timeout behavior.
// Set false to disable the floodfill web UI on port 81 while keeping floodfill logic active.
static constexpr bool ENABLE_WEB = true;
static constexpr uint16_t PORT = 81;
static constexpr uint16_t WS_PORT = 83;
static constexpr bool AUTO_RUN = false;
static constexpr uint32_t ACK_TIMEOUT_MS = 2000;
static constexpr bool PAUSE_ON_ACK_TIMEOUT = true;
// In explore mode, keep looping between original goal and original start
// after each target is reached. This helps continue discovering alternate
// walls and improving the path without resetting pose.
static constexpr bool CONTINUE_AFTER_GOAL = true;
// Mark the shortest path as known after this many consecutive
// goal->home round trips report the same best-known start->goal cost.
static constexpr uint8_t SHORTEST_PATH_STABLE_ROUND_TRIPS = 1;
}

namespace SpeedRun2 {
// Dedicated one-way shortest-path motion profile.
// Starts with the current stable speedrun tuning so it can be tuned independently later.
static constexpr float MOVE_SPEED_TPS = Motion::MOVE_SPEED_TPS;
static constexpr float CORRIDOR_MOVE_SPEED_TPS = Motion::CORRIDOR_MOVE_SPEED_TPS;
static constexpr float TURN_SPEED_TPS = Motion::TURN_SPEED_TPS;
static constexpr float CENTERING_GAIN = Motion::CENTERING_GAIN;
static constexpr float CORRIDOR_CENTERING_GAIN = Motion::CORRIDOR_CENTERING_GAIN;
static constexpr float FRONT_STOP_MM = Motion::FRONT_STOP_MM;
static constexpr float CORRIDOR_FRONT_STOP_MM = Motion::CORRIDOR_FRONT_STOP_MM;
}

namespace Inputs {
// Built-in BOOT button multi-press launcher on ESP32-S3 GPIO0.
static constexpr bool ENABLE_BOOT_BUTTON_LAUNCH = true;
static constexpr uint8_t BOOT_BUTTON_PIN = 0;
static constexpr bool BOOT_BUTTON_ACTIVE_LOW = true;
static constexpr uint32_t BOOT_BUTTON_DEBOUNCE_MS = 30;
static constexpr uint32_t BOOT_BUTTON_MULTI_PRESS_TIMEOUT_MS = 5000;
}

namespace Tasks {
// Main periodic task cadences (milliseconds).
// Affects: scheduler pacing and loop watchdog expected periods.
static constexpr uint32_t USER_LOOP_PERIOD_MS = 20;
static constexpr uint32_t PLANNER_LOOP_PERIOD_MS = 50;
static constexpr uint32_t MOTOR_LOOP_PERIOD_MS = 5;
static constexpr uint32_t EXPLORER_LOOP_PERIOD_MS = 10;
static constexpr uint32_t TOF_LOOP_PERIOD_MS = 5;
static constexpr uint32_t TELEMETRY_LOOP_PERIOD_MS = 1000;
}

namespace Debug {
  // Additional runtime flow logging for motion, snap, and wall application.
  // Affects: extra serial/TCP debug output only; no behavior changes.
  static constexpr bool DEBUG_MOTION_FLOW = true;
  static constexpr bool DEBUG_WALL_APPLY = true;
  // Lightweight periodic-task timing watchdog.
  // Warns when a watched loop runs later than its expected cadence.
  static constexpr bool ENABLE_LOOP_WATCHDOG = true;
  static constexpr uint32_t LOOP_WATCHDOG_TOLERANCE_MS = 3;
  static constexpr uint32_t LOOP_WATCHDOG_RATE_LIMIT_MS = 500;
}

// Helper that converts config constants into the runtime motion controller config.
// Update this if MotionController::Config grows new fields.
inline MotionController::Config makeMotionConfig() {
  MotionController::Config cfg;
  cfg.cellDistanceMm = Motion::CELL_DISTANCE_MM;
  cfg.turnTicks90 = Motion::TURN_TICKS_90;
  cfg.turnTicks180 = Motion::TURN_TICKS_180;
  cfg.moveSpeedTps = Motion::MOVE_SPEED_TPS;
  cfg.corridorMoveSpeedTps = Motion::CORRIDOR_MOVE_SPEED_TPS;
  cfg.shortForwardDistanceMm = Motion::SHORT_FORWARD_DISTANCE_MM;
  cfg.shortForwardSpeedTps = Motion::SHORT_FORWARD_SPEED_TPS;
  cfg.reverseDistanceMm = Motion::REVERSE_DISTANCE_MM;
  cfg.reverseSpeedTps = Motion::REVERSE_SPEED_TPS;
  cfg.snapCenterStopHoldMs = Motion::SNAP_CENTER_STOP_HOLD_MS;
  cfg.turnSpeedTps = Motion::TURN_SPEED_TPS;
  cfg.centeringGain = Motion::CENTERING_GAIN;
  cfg.corridorCenteringGain = Motion::CORRIDOR_CENTERING_GAIN;
  cfg.frontStopMm = Motion::FRONT_STOP_MM;
  cfg.corridorFrontStopMm = Motion::CORRIDOR_FRONT_STOP_MM;
  cfg.primitiveTimeoutMs = Motion::PRIMITIVE_TIMEOUT_MS;
  cfg.corridorTimeoutPerCellMs = Motion::CORRIDOR_TIMEOUT_PER_CELL_MS;
  cfg.stallTimeoutMs = Motion::STALL_TIMEOUT_MS;
  cfg.stopTps = Motion::STOP_TPS;
  cfg.minProgressMm = Motion::MIN_PROGRESS_MM;
  cfg.mmPerTick = Motion::MM_PER_TICK;
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
