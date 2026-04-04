#include "Config.h"

namespace AppConfig {
namespace Tof {
const uint16_t WALL_THRESHOLD_MM = 140;
const uint16_t DIST_MAX_VALID_MM = 250;
const uint16_t DIST_FAR_MM = DIST_MAX_VALID_MM + 1;
const uint16_t DIST_ERROR_MM = DIST_MAX_VALID_MM + 2;
const float DIST_LPF_PREV_WEIGHT = 0.5f;
const float DIST_LPF_SAMPLE_WEIGHT = 1.0f - DIST_LPF_PREV_WEIGHT;
}

namespace Motors {
const float PID_KP = 0.0038f;
const float PID_KI = 0.0008f;
const float PID_KD = 0.0005f;
const float PID_OUT_LIMIT = 1.00f;
const float PID_I_LIMIT = 1.0f;
const float PID_D_FILTER_HZ = 25.0f;
const float PID_SLEW_RATE = 1.0f;
const float TPS_LPF_ALPHA = 0.2f;
const uint32_t TPS_ESTIMATE_WINDOW_MS = 20;
}

namespace Motion {
const float CELL_DISTANCE_MM = 180.0f;
const float TURN_LEFT_90_MM = 100.0f;
const float TURN_RIGHT_90_MM = 102.0f;
const float TURN_180_MM = 235.0f;
const float MOVE_SPEED_TPS = 350.0f;
const float CORRIDOR_MOVE_SPEED_TPS = 500.0f;
const float SHORT_FORWARD_DISTANCE_MM = 50.0f;
const float SHORT_FORWARD_SPEED_TPS = 350.0f;
const float REVERSE_DISTANCE_MM = 100.0f;
const float REVERSE_SPEED_TPS = 350.0f;
const uint32_t SNAP_CENTER_STOP_HOLD_MS = 1;
const float TURN_SPEED_TPS = 350.0f;
const float TURN_MIN_SPEED_TPS = 200.0f;
const float TURN_SLOWDOWN_START_RATIO = 0.8f;
const float CENTERING_GAIN = 1.0f;
const float CORRIDOR_CENTERING_GAIN = 1.0f;
const float CENTER_TARGET_LEFT_MM = 96.0f;
const float CENTER_TARGET_RIGHT_MM = 96.0f;
const float CENTER_TARGET_CAPTURE_WINDOW_MM = 4.0f;
const float CENTER_PID_KP = 2.0f;
const float CENTER_PID_KI = 0.01f;
const float CENTER_PID_KD = 0.5f;
const float CENTER_PID_I_LIMIT = 40.0f;
const float CENTER_PID_OUT_LIMIT = 50.0f;
const uint16_t CENTER_PID_EFFECTIVE_SIDE_MAX_MM = 200;
const float CENTER_BLEND_TAU_SEC = 0.14f;
const float CENTER_RAW_TAU_SEC = 0.07f;
const float FRONT_STOP_MM = 100.0f;
const float CORRIDOR_FRONT_STOP_MM = 130.0f;
const uint32_t PRIMITIVE_TIMEOUT_MS = 3000;
const uint32_t CORRIDOR_TIMEOUT_PER_CELL_MS = 1000;
const uint32_t STALL_TIMEOUT_MS = 700;
const uint8_t CORRIDOR_MAX_CELLS = 4;
const float STOP_TPS = 20.0f;
const float MIN_PROGRESS_MM = 12.0f;
const MotionController::StopMode COMPLETION_STOP_MODE = MotionController::StopMode::BRAKE;
const float LEFT_MM_PER_TICK = 0.54f;
const float RIGHT_MM_PER_TICK = 0.54f;
const uint32_t POST_MOTION_HARD_STOP_HOLD_MS = 1;
}

namespace Explorer {
const uint32_t ACK_TIMEOUT_MS = 2000;
const uint8_t SHORTEST_PATH_STABLE_ROUND_TRIPS = 1;
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
}  // namespace AppConfig
