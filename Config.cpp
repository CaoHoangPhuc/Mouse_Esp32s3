#include "Config.h"

namespace AppConfig {
#if APP_LITE_FIRMWARE
static constexpr bool kLiteFirmware = true;
#else
static constexpr bool kLiteFirmware = false;
#endif

namespace Build {
const bool LITE_FIRMWARE = kLiteFirmware;
const char* PROFILE_NAME = kLiteFirmware ? "LITE" : "FULL";
}

namespace Battery {
const uint8_t ADC_PIN = 3;
const float DIVIDER_R_TOP_KOHM = 56.0f;
const float DIVIDER_R_BOTTOM_KOHM = 18.0f;
const uint16_t RAW_LOW = 2800;
const uint16_t RAW_HIGH = 3350;
const float VOLTAGE_LOW = 7.20f;
const float VOLTAGE_HIGH = 8.40f;
const float WARNING_VOLTAGE = 7.10f;
const float CRITICAL_VOLTAGE = 6.90f;
}

namespace Maze {
const uint8_t START_X = 0;
const uint8_t START_Y = 0;
const FloodFillExplorer::Dir START_HEADING = FloodFillExplorer::SOUTH;
const uint8_t HOME_X0 = 0;
const uint8_t HOME_Y0 = 0;
const uint8_t HOME_W = 1;
const uint8_t HOME_H = 1;
const uint8_t GOAL_X0 = 4;
const uint8_t GOAL_Y0 = 4;
const uint8_t GOAL_W = 1;
const uint8_t GOAL_H = 1;
}

namespace Wifi {
const char* SSID = "PhucWifi";
const char* PASS = "000000001";
const char* HOSTNAME = "PhucC_Esp32s3";
const bool ENABLE_WEB_LOG = true;
const bool ENABLE_UPLOAD_WEB = true;
const uint16_t UPLOAD_WEB_PORT = 82;
const uint16_t DEBUG_TCP_PORT = 2323;
const BaseType_t CORE = 0;
const UBaseType_t TASK_PRIORITY = 0;
const uint32_t TASK_STACK = 10 * 1024;
const uint32_t SERVICE_DELAY_MS = 50;
const uint32_t CONNECT_TIMEOUT_MS = 15000;
const uint32_t RECONNECT_INTERVAL_MS = 10000;
}

namespace Debug {
const bool ENABLE_SERIAL_OUTPUT = false;
const bool PRINT_STATUS_TPS = false;
const bool DEBUG_MOTION_FLOW = false;
const bool DEBUG_MOTION_EVENT = DEBUG_MOTION_FLOW;
const bool DEBUG_MAZE_PRINT = false;
const bool DEBUG_WALL_APPLY = false;
const bool CENTER_PID_TRACE = false;
const uint8_t CENTER_PID_TRACE_EVERY_N = 10;
const bool MOTOR_PID_TRACE = false;
const uint16_t MOTOR_PID_TRACE_EVERY_N = 100;
const bool ENABLE_LOOP_WATCHDOG = false;
const uint32_t LOOP_WATCHDOG_TOLERANCE_MS = 3;
const uint32_t LOOP_WATCHDOG_RATE_LIMIT_MS = 500;
}

namespace I2C {
const uint8_t SDA = 8;
const uint8_t SCL = 9;
const uint32_t CLOCK_HZ = 400000;
}

namespace Tof {
const uint8_t PCF_ADDRESS = 0x20;
const uint8_t SENSOR_COUNT = 5;
const uint16_t UPDATE_INTERVAL_MS = 20;
const bool COMPUTE_HEADING_FROM_FULL_SWEEP = true;
const uint16_t WALL_THRESHOLD_MM = 140;
const uint16_t DIST_MIN_VALID_MM = 1;
const uint16_t DIST_MAX_VALID_MM = 200;
const uint16_t DIST_FAR_MM = DIST_MAX_VALID_MM + 1;
const uint16_t DIST_ERROR_MM = DIST_MAX_VALID_MM + 2;
const uint8_t XSHUT_PINS[] = {0, 1, 2, 3, 4};
const uint8_t SENSOR_ADDR[] = {0x30, 0x31, 0x32, 0x33, 0x34};
const float SENSOR_SCALE[8] = {
  1.0f, (97.0f / 88.0f), 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};
const int16_t SENSOR_OFFSET_MM[8] = {
  0, -16, 16, 0, 0, 0, 0, 0
};
const float DIST_LPF_PREV_WEIGHT = 0.75f;
const float DIST_LPF_SAMPLE_WEIGHT = 1.0f - DIST_LPF_PREV_WEIGHT;
}

namespace Motors {
const DcMotor::Pins RIGHT_PINS = {
  .in1 = 10,
  .in2 = 11,
  .pwm = 7,
  .encA = 12,
  .encB = 13,
  .invertDir = false,
  .invertEnc = false
};

const DcMotor::Pins LEFT_PINS = {
  .in1 = 5,
  .in2 = 6,
  .pwm = 4,
  .encA = 1,
  .encB = 2,
  .invertDir = true,
  .invertEnc = true
};

const uint8_t LEFT_PWM_CHANNEL = 0;
const uint8_t RIGHT_PWM_CHANNEL = 1;
const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_RESOLUTION_BITS = 10;

const float PID_KP = 0.0025f;
const float PID_KI = 0.0010f;
const float PID_KD = 0.0005f;
const float PID_OUT_LIMIT = 1.00f;
const float PID_I_LIMIT = 1.0f;
const float PID_D_FILTER_HZ = 25.0f;
const float PID_SLEW_RATE = 2.0f;
const float PID_SLEW_RATE_LEFT = 1.0 * PID_SLEW_RATE;
const float PID_SLEW_RATE_RIGHT = 1.0f * PID_SLEW_RATE;
const uint16_t PID_MIN_DRIVE_DUTY = 20;
const float TPS_LPF_ALPHA = 0.2f; // apply new
const uint32_t TPS_ESTIMATE_WINDOW_MS = 20;
}

namespace Motion {
const float CELL_DISTANCE_MM = 180.0f;
const float TURN_LEFT_90_MM = 115.0f;
const float TURN_RIGHT_90_MM = 115.0f;
const float TURN_180_MM = 235.0f;
const float MOVE_SPEED_TPS = 500.0f;
const float CORRIDOR_MOVE_SPEED_TPS = 700.0f;
const float SHORT_FORWARD_DISTANCE_MM = 40.0f;
const float SHORT_FORWARD_SPEED_TPS = 350.0f;
const float REVERSE_DISTANCE_MM = 100.0f;
const float REVERSE_SPEED_TPS = 350.0f;
const uint32_t SNAP_CENTER_STOP_HOLD_MS = 1;
const float TURN_SPEED_TPS = 350.0f;
const float TURN_MIN_SPEED_TPS = 200.0f;
const float TURN_SLOWDOWN_START_RATIO = 0.8f;
const float CENTERING_GAIN = 1.0f;
const float CORRIDOR_CENTERING_GAIN = 1.0f;
const float CENTERING_SLOW_SIDE_GAIN = 2.0f;
const float CENTERING_FAST_SIDE_GAIN = 0.5f;
const float CENTER_TARGET_LEFT_MM = 99.0f;
const float CENTER_TARGET_RIGHT_MM = 99.0f;
const float CENTER_TARGET_CAPTURE_WINDOW_MM = 4.0f;
const float CENTER_PID_KP = 1.8f;
const float CENTER_PID_KI = 0.1f;
const float CENTER_PID_KD = 0.5f;
const float CENTER_PID_I_LIMIT = 500.0f;
const float CENTER_PID_OUT_LIMIT = 500.0f;
const float CENTER_PID_SINGLE_WALL_ERR_LIMIT_MM = 0.0f;
const float CENTER_PID_DERIV_LIMIT = 500.0f;
const uint16_t CENTER_PID_EFFECTIVE_SIDE_MAX_MM = 200;
const float CENTER_BLEND_TAU_SEC = 0.14f;
const float CENTER_RAW_TAU_SEC = 0.07f;
const float FRONT_STOP_MM = 110.0f;
const float CORRIDOR_FRONT_STOP_MM = 130.0f;
const float DISTANCE_APPROACH_START_RATIO = 0.80f;
const float DISTANCE_APPROACH_MIN_SPEED_TPS = 350.0f;
const float FRONT_APPROACH_START_FACTOR = 1.4f;
const float FRONT_APPROACH_MIN_SPEED_TPS = 350.0f;
const uint32_t PRIMITIVE_TIMEOUT_MS = 3000;
const uint32_t CORRIDOR_TIMEOUT_PER_CELL_MS = 1000;
const uint32_t STALL_TIMEOUT_MS = 1200;
const uint8_t CORRIDOR_MAX_CELLS = 4;
const float STOP_TPS = 20.0f;
const float MIN_PROGRESS_MM = 12.0f;
const MotionController::StopMode COMPLETION_STOP_MODE = MotionController::StopMode::BRAKE;
const MotionController::StopMode SNAP_CENTER_HOLD_STOP_MODE = MotionController::StopMode::BRAKE;
const MotionController::StopMode POST_MOTION_SETTLE_STOP_MODE = MotionController::StopMode::BRAKE;
const float LEFT_MM_PER_TICK = 0.53f;
const float RIGHT_MM_PER_TICK = 0.53f;
const bool AUTO_PRINT_MAZE_AFTER_SENSE = true;
const uint32_t POST_MOTION_HARD_STOP_HOLD_MS = 1;
}

namespace Explorer {
const bool ENABLE_WEB = true;
const uint16_t PORT = 81;
const uint16_t WS_PORT = 83;
const bool AUTO_RUN = false;
const uint32_t ACK_TIMEOUT_MS = 2000;
const bool PAUSE_ON_ACK_TIMEOUT = true;
const bool CONTINUE_AFTER_GOAL = true;
const uint8_t SHORTEST_PATH_STABLE_ROUND_TRIPS = 1;
const bool QUEUE_ENABLE_EXPLORE = false;
const bool QUEUE_ENABLE_SPEEDRUN1 = true;
const bool QUEUE_DISABLE_WALL_REGISTER_WHILE_ACTIVE = true;
const uint16_t QUEUE_CAPACITY = 128;
const bool QUEUE_DEBUG_PRINT = true;
}

namespace SpeedRun2 {
const float MOVE_SPEED_TPS = Motion::MOVE_SPEED_TPS;
const float CORRIDOR_MOVE_SPEED_TPS = Motion::CORRIDOR_MOVE_SPEED_TPS;
const float TURN_SPEED_TPS = Motion::TURN_SPEED_TPS;
const float CENTERING_GAIN = Motion::CENTERING_GAIN;
const float CORRIDOR_CENTERING_GAIN = Motion::CORRIDOR_CENTERING_GAIN;
const float FRONT_STOP_MM = Motion::FRONT_STOP_MM;
const float CORRIDOR_FRONT_STOP_MM = Motion::CORRIDOR_FRONT_STOP_MM;
}

namespace Inputs {
const bool ENABLE_BOOT_BUTTON_LAUNCH = true;
const uint8_t BOOT_BUTTON_PIN = 0;
const bool BOOT_BUTTON_ACTIVE_LOW = true;
const uint32_t BOOT_BUTTON_DEBOUNCE_MS = 30;
const uint32_t BOOT_BUTTON_MULTI_PRESS_TIMEOUT_MS = 5000;
}

namespace Tasks {
const uint32_t USER_LOOP_PERIOD_MS = 20;
const uint32_t PLANNER_LOOP_PERIOD_MS = 10;
const uint32_t MOTOR_LOOP_PERIOD_MS = 5;
const uint32_t EXPLORER_LOOP_PERIOD_MS = 10;
const uint32_t TOF_LOOP_PERIOD_MS = 5;
const uint32_t TELEMETRY_LOOP_PERIOD_MS = 1000;
}
}  // namespace AppConfig
