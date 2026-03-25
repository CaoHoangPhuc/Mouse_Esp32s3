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
static constexpr uint8_t START_Y = 15;
static constexpr FloodFillExplorer::Dir START_HEADING = FloodFillExplorer::NORTH;

// Goal rectangle for floodfill.
// Typical micromouse center goal is 2x2.
// Affects: planner target and floodfill distance field.
static constexpr uint8_t GOAL_X0 = 7;
static constexpr uint8_t GOAL_Y0 = 7;
static constexpr uint8_t GOAL_W = 2;
static constexpr uint8_t GOAL_H = 2;
}

namespace Wifi {
// Development Wi-Fi / OTA / web logging settings.
// Affects: WiFiOtaWebSerial startup, OTA hostname, web serial availability.
static constexpr const char* SSID = "PhucWifi";
static constexpr const char* PASS = "000000001";
static constexpr const char* HOSTNAME = "PhucC_Esp32s3_mice";
static constexpr const char* OTA_PASSWORD = "";

// FreeRTOS task placement/settings for Wi-Fi service loop.
// Usually only change these if Wi-Fi/OTA becomes unstable.
static constexpr BaseType_t CORE = 0;
static constexpr UBaseType_t TASK_PRIORITY = 3;
static constexpr uint32_t TASK_STACK = 10 * 1024;
static constexpr uint32_t SERVICE_DELAY_MS = 5;
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
static constexpr uint16_t WALL_THRESHOLD_MM = 150;

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
static constexpr float PID_KP = 0.004f;
static constexpr float PID_KI = 0.0080f;
static constexpr float PID_KD = 0.00005f;
static constexpr float PID_OUT_LIMIT = 0.80f;
static constexpr float PID_I_LIMIT = 0.50f;
static constexpr float PID_D_FILTER_HZ = 25.0f;
static constexpr float PID_SLEW_RATE = 4.0f;
}

namespace Motion {
// Estimated forward travel for one maze cell.
// Affects: when moveOneCell() decides the move is complete.
// One of the most important hardware tuning values.
static constexpr float CELL_DISTANCE_MM = 180.0f;

// Encoder differential ticks needed for a 90-degree turn.
// Affects: turnLeft90() / turnRight90() completion.
// One of the most important hardware tuning values.
static constexpr int32_t TURN_TICKS_90 = 300;

// Nominal primitive speeds in ticks/sec.
// Affects: how fast the robot attempts straight moves and turns.
static constexpr float MOVE_SPEED_TPS = 320.0f;
static constexpr float TURN_SPEED_TPS = 250.0f;

// Wall-centering correction gain while driving straight.
// Higher = stronger correction, but too high can oscillate.
// Affects: corridor following stability.
static constexpr float CENTERING_GAIN = 1.6f;

// If a front wall is seen this close near the end of a move, stop early.
// Affects: wall approach safety and cell alignment.
static constexpr float FRONT_STOP_MM = 55.0f;

// Primitive fault timing.
// Affects: when moves/turns fail due to timeout or lack of progress.
static constexpr uint32_t PRIMITIVE_TIMEOUT_MS = 3000;
static constexpr uint32_t STALL_TIMEOUT_MS = 700;

// Primitive completion thresholds.
// STOP_TPS: considered stopped when wheel speed falls below this.
// MIN_PROGRESS_MM: minimum progress before stall timer is refreshed.
static constexpr float STOP_TPS = 20.0f;
static constexpr float MIN_PROGRESS_MM = 12.0f;

// Mechanical distance-per-tick estimate.
// Affects: forward progress estimation and one-cell completion.
// Usually tune this before final CELL_DISTANCE_MM tuning.
static constexpr float MM_PER_TICK = 0.54f;
}

namespace Explorer {
// Floodfill web explorer settings.
// Affects: debug UI on the network and action ACK timeout behavior.
static constexpr uint16_t PORT = 81;
static constexpr bool AUTO_RUN = false;
static constexpr uint32_t ACK_TIMEOUT_MS = 2000;
static constexpr bool PAUSE_ON_ACK_TIMEOUT = true;
}

// Helper that converts config constants into the runtime motion controller config.
// Update this if MotionController::Config grows new fields.
inline MotionController::Config makeMotionConfig() {
  MotionController::Config cfg;
  cfg.cellDistanceMm = Motion::CELL_DISTANCE_MM;
  cfg.turnTicks90 = Motion::TURN_TICKS_90;
  cfg.moveSpeedTps = Motion::MOVE_SPEED_TPS;
  cfg.turnSpeedTps = Motion::TURN_SPEED_TPS;
  cfg.centeringGain = Motion::CENTERING_GAIN;
  cfg.frontStopMm = Motion::FRONT_STOP_MM;
  cfg.primitiveTimeoutMs = Motion::PRIMITIVE_TIMEOUT_MS;
  cfg.stallTimeoutMs = Motion::STALL_TIMEOUT_MS;
  cfg.stopTps = Motion::STOP_TPS;
  cfg.minProgressMm = Motion::MIN_PROGRESS_MM;
  cfg.mmPerTick = Motion::MM_PER_TICK;
  return cfg;
}

// Helper that converts config constants into the floodfill explorer runtime config.
inline FloodFillExplorer::Config makeExplorerConfig() {
  FloodFillExplorer::Config cfg;
  cfg.port = Explorer::PORT;
  cfg.autoRun = Explorer::AUTO_RUN;
  cfg.ackTimeoutMs = Explorer::ACK_TIMEOUT_MS;
  cfg.pauseOnAckTimeout = Explorer::PAUSE_ON_ACK_TIMEOUT;
  return cfg;
}

}  // namespace AppConfig
