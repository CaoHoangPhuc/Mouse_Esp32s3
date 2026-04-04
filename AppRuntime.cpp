#include <Arduino.h>
#include <WiFi.h>

#include "Battery.h"
#include "Config.h"
#include "DcMotor.h"
#include "FloodFillExplorer.h"
#include "LedController.h"
#include "MotionController.h"
#include "MultiVL53L0X.h"
#include "PersistenceStore.h"
#include "RobotTypes.h"
#include "WiFiOtaWebSerial.h"
#include "AppRuntime.h"

namespace MainApp {

WiFiOtaWebSerial dbg;
FloodFillExplorer explorer;
Battery mouseBattery;
MotionController motionController;
RobotState robotState;
LedController ledController;
WiFiServer debugServer(AppConfig::Wifi::DEBUG_TCP_PORT);
WiFiClient debugClient;

DcMotor leftMotor;
DcMotor rightMotor;

static bool motorsOk = false;
static bool tofOk = false;
static bool batteryOk = false;
static bool wifiOk = false;
static uint32_t lastStatusMs = 0;

enum TestLoopMode : uint8_t {
  TEST_LOOP_NONE = 0,
  TEST_LOOP_STATUS,
  TEST_LOOP_BATTERY,
  TEST_LOOP_SENSORS,
  TEST_LOOP_SENSORS_RAW,
  TEST_LOOP_ENCODERS,
  TEST_LOOP_MAZE
};

static TestLoopMode testLoopMode = TEST_LOOP_NONE;
static bool motorBothFlipTestEnabled = false;
static uint32_t motorFlipLastToggleMs = 0;
static float motorFlipPower = 1.0f;
static bool bootButtonRawPressed = false;
static bool bootButtonStablePressed = false;
static uint32_t bootButtonLastEdgeMs = 0;
static uint32_t bootButtonLastAcceptedPressMs = 0;
static uint8_t bootButtonPressCount = 0;

MultiVL53L0X tofArray(
  AppConfig::Tof::PCF_ADDRESS, AppConfig::Tof::SENSOR_COUNT,
  AppConfig::Tof::XSHUT_PINS, AppConfig::Tof::SENSOR_ADDR,
  AppConfig::Tof::UPDATE_INTERVAL_MS,
  Wire
);

static TaskHandle_t tofTaskHandle = nullptr;
static TaskHandle_t motorTaskHandle = nullptr;
static TaskHandle_t explorerTaskHandle = nullptr;
static TaskHandle_t plannerTaskHandle = nullptr;
static TaskHandle_t telemetryTaskHandle = nullptr;
static TaskHandle_t userTaskHandle = nullptr;

static void motorTask(void* arg);
static void tofTask(void* arg);
static void explorerTask(void* arg);
static void telemetryTask(void* arg);
static void handleSerialCommand(const String& line);
static void updateRobotState();
static void applyWallsToExplorer();
static bool executePlannerAction(FloodFillExplorer::Action act);
static void serviceActiveForwardAction();
static void handleMotionCompletion();
static void enterIdleMode(const String& reason);
static void enterFaultMode(const String& reason);
static const char* modeName(RobotMode mode);
static const char* batteryStateName(uint8_t state);
static const char* motionStatusName(MotionStatus status);
static const char* testLoopModeName(TestLoopMode mode);
static void printStartupSummary();
static void motor_debug_s();
static void tof_debug_s();
static void tof_raw_debug_s();
static void robot_debug_s();
static void battery_debug_s();
static void serviceDebugConsole();
static void serviceDebugServerState();
static void serviceMotorBothFlipTest();
static void serviceBootButtonLauncher();
static bool readBootButtonPressed();
static void dispatchBootButtonAction(uint8_t pressCount);
static void debugPrint(const String& s);
static void debugPrintln(const String& s = "");
static void debugPrompt();
static bool debugClientConnected();
static bool serialOutputEnabled();
static bool wifiReconnectAllowed();
static void updateOtaSafeMode();
static void suspendTaskIfRunning(TaskHandle_t handle);
static void resumeTaskIfSuspended(TaskHandle_t handle);
static bool handleLedCommand(const String& cmd);
static bool onWebTelnetReconnect();
static String onWebHealthJson();
static void onExplorerWebCommand(const String& cmd);
static void updateRobotLed();
static void beginExplore(bool clearMaze, int32_t stepBudget = -1);
static void beginSpeedRun(uint8_t phase = 1);
static uint8_t speedRunBasePhase(uint8_t phase);
static bool usesSpeedRun2Profile(uint8_t phase);
static bool isContinuousSpeedRunPhase(uint8_t phase);
static void resetMazeToConfiguredStart();
static void clearMazeMemoryOnly();
static void applyCurrentPoseAsHomeRect();
static FloodFillExplorer::Dir headingDir();
static FloodFillExplorer::Dir oppositeDir(FloodFillExplorer::Dir dir);
static void maze_debug_s();
static void resetCommonModeState();
static void closeDebugConsole(const String& reason = "");
static bool shouldSnapCenterFromKnownBackWall();
static bool startRunSnapSequence(const char* label);
static bool startSpeedRunPreAlignSequence();
static void finishSpeedRunStart();
static void advancePoseForwardOneCell();
static bool commitForwardActionCell(bool allowFrontStopCompletion, bool& stepBudgetReached, bool& reachedGoal);
static bool handleReachedGoal(bool& skipPostMotionHold);
static void clearForwardActionTracking();
static bool shouldStopCorridorAfterCommittedCell();
static void resetExploreLoopTracking();
static void setPose(uint8_t x, uint8_t y, FloodFillExplorer::Dir h);
static void resetLapHistory();
static void startLapTimer(const char* legLabel);
static void stopLapTimer(bool record);
static String explorerLapStateJson();
static void resetLoopWatchdogState(struct LoopWatchdogState& state);
static void serviceLoopWatchdog(struct LoopWatchdogState& state, uint32_t expectedMs);

static bool otaSafeModeActive = false;
static uint32_t lastConsoleActivityMs = 0;
static constexpr uint32_t kConsoleQuietMs = 1500;
static bool runStartSnapPending = false;
static bool deferPlannerAckUntilSnapCenter = false;
static RobotMode runStartSnapMode = ROBOT_MODE_IDLE;
static uint16_t lastStableBestCost = 0xFFFF;
static uint8_t stableRoundTripCount = 0;
static bool reachedOriginalGoalOnThisLoop = false;
static bool exploreGoalSeen = false;
static int32_t exploreStepBudget = -1;
static bool serialOutputTemporarilyMuted = false;
static bool debugServerStarted = false;
static uint8_t activeForwardActionCellsRequested = 0;
static uint8_t activeForwardActionCellsCommitted = 0;
enum SpeedRunPreAlignStage : uint8_t {
  SPEEDRUN_PREALIGN_NONE = 0,
  SPEEDRUN_PREALIGN_AFTER_SIDE_TURN,
  SPEEDRUN_PREALIGN_AFTER_SIDE_SNAP,
  SPEEDRUN_PREALIGN_AFTER_RETURN_TURN,
  SPEEDRUN_PREALIGN_AFTER_FINAL_SNAP
};
static SpeedRunPreAlignStage speedRunPreAlignStage = SPEEDRUN_PREALIGN_NONE;
static bool speedRunPreAlignTurnRight = true;
static bool speedRunPreAlignHasSideSnap = false;
static bool lapTimerRunning = false;
static uint32_t lapStartMs = 0;
static uint32_t lapCurrentMsSnapshot = 0;
static String lapCurrentLabel = "HG";
static constexpr uint8_t kLapHistoryMax = 16;
static uint32_t lapHistoryMs[kLapHistoryMax] = {};
static String lapHistoryLabels[kLapHistoryMax];
static uint8_t lapHistoryCount = 0;
struct LoopWatchdogState {
  const char* taskName = "";
  TickType_t lastTick = 0;
  TickType_t lastWarnTick = 0;
  uint32_t warningCount = 0;
};
static LoopWatchdogState motorLoopWatchdog{"motorTask"};
static LoopWatchdogState tofLoopWatchdog{"tofTask"};
static LoopWatchdogState userLoopWatchdog{"userTaskBody"};
static LoopWatchdogState plannerLoopWatchdog{"plannerTaskBody"};
static LoopWatchdogState telemetryLoopWatchdog{"telemetryTask"};
static LoopWatchdogState explorerLoopWatchdog{"explorerTask"};
static uint8_t runtimeGoalX0 = AppConfig::Maze::GOAL_X0;
static uint8_t runtimeGoalY0 = AppConfig::Maze::GOAL_Y0;
static uint8_t runtimeGoalW = AppConfig::Maze::GOAL_W;
static uint8_t runtimeGoalH = AppConfig::Maze::GOAL_H;
static const char* primitiveName(MotionPrimitiveType primitive);
static const char* headingName(FloodFillExplorer::Dir dir);
static String poseSummary(uint8_t x, uint8_t y, FloodFillExplorer::Dir dir);
static String wallFlagsSummary();
static String wallDistanceSummary();
static void debugMotionEvent(const char* tag, MotionPrimitiveType primitive, MotionStatus status,
                             uint8_t beforeX, uint8_t beforeY, FloodFillExplorer::Dir beforeH,
                             uint8_t afterX, uint8_t afterY, FloodFillExplorer::Dir afterH,
                             const String& extra = "");
static void debugWallApplyEvent(const char* tag, const char* source, const String& extra = "");

static void logToDbg(const String& s) {
  debugPrintln(s);
}

static bool debugClientConnected() {
  return debugClient && debugClient.connected();
}

static bool serialOutputEnabled() {
  return AppConfig::Debug::ENABLE_SERIAL_OUTPUT && !serialOutputTemporarilyMuted;
}

static bool wifiReconnectAllowed() {
  return robotState.mode == ROBOT_MODE_IDLE;
}

static void resetLoopWatchdogState(LoopWatchdogState& state) {
  state.lastTick = 0;
  state.lastWarnTick = 0;
  state.warningCount = 0;
}

static void serviceLoopWatchdog(LoopWatchdogState& state, uint32_t expectedMs) {
  if (!AppConfig::Debug::ENABLE_LOOP_WATCHDOG) return;
  if (xPortGetCoreID() != 0) return;

  const TickType_t now = xTaskGetTickCount();
  if (state.lastTick == 0) {
    state.lastTick = now;
    return;
  }

  const uint32_t actualMs = (uint32_t)(now - state.lastTick) * (uint32_t)portTICK_PERIOD_MS;
  state.lastTick = now;

  if (actualMs <= expectedMs + AppConfig::Debug::LOOP_WATCHDOG_TOLERANCE_MS) return;

  const uint32_t sinceLastWarnMs =
    (state.lastWarnTick == 0) ? UINT32_MAX :
    (uint32_t)(now - state.lastWarnTick) * (uint32_t)portTICK_PERIOD_MS;
  if (sinceLastWarnMs < AppConfig::Debug::LOOP_WATCHDOG_RATE_LIMIT_MS) return;

  state.lastWarnTick = now;
  state.warningCount++;

  const uint32_t lateMs = actualMs - expectedMs;
  debugPrintln("[LOOP WARN] task=" + String(state.taskName) +
               " period=" + String(expectedMs) + "ms" +
               " actual=" + String(actualMs) + "ms" +
               " late=" + String(lateMs) + "ms" +
               " core=" + String((int)xPortGetCoreID()) +
               " count=" + String(state.warningCount));
}

static void resetLapHistory() {
  lapTimerRunning = false;
  lapStartMs = 0;
  lapCurrentMsSnapshot = 0;
  lapCurrentLabel = "HG";
  lapHistoryCount = 0;
  memset(lapHistoryMs, 0, sizeof(lapHistoryMs));
  for (uint8_t i = 0; i < kLapHistoryMax; ++i) {
    lapHistoryLabels[i] = "";
  }
}

static void startLapTimer(const char* legLabel) {
  if (lapTimerRunning) {
    stopLapTimer(false);
  }
  lapTimerRunning = true;
  lapStartMs = millis();
  lapCurrentMsSnapshot = 0;
  lapCurrentLabel = legLabel ? String(legLabel) : String("HG");
  explorer.notifyStateChanged();
}

static void stopLapTimer(bool record) {
  if (!lapTimerRunning) {
    if (!record) {
      explorer.notifyStateChanged();
    }
    return;
  }
  lapCurrentMsSnapshot = millis() - lapStartMs;
  lapTimerRunning = false;
  if (record) {
    if (lapHistoryCount < kLapHistoryMax) {
      lapHistoryLabels[lapHistoryCount] = lapCurrentLabel;
      lapHistoryMs[lapHistoryCount++] = lapCurrentMsSnapshot;
    } else {
      memmove(&lapHistoryMs[0], &lapHistoryMs[1], sizeof(lapHistoryMs[0]) * (kLapHistoryMax - 1));
      lapHistoryMs[kLapHistoryMax - 1] = lapCurrentMsSnapshot;
      for (uint8_t i = 0; i < kLapHistoryMax - 1; ++i) {
        lapHistoryLabels[i] = lapHistoryLabels[i + 1];
      }
      lapHistoryLabels[kLapHistoryMax - 1] = lapCurrentLabel;
    }
  }
  explorer.notifyStateChanged();
}

static String explorerLapStateJson() {
  uint32_t currentMs = lapTimerRunning ? (millis() - lapStartMs) : lapCurrentMsSnapshot;
  String out = "\"lap\":{";
  out += "\"running\":";
  out += lapTimerRunning ? "true" : "false";
  out += ",\"currentLabel\":\"";
  out += lapCurrentLabel;
  out += "\"";
  out += ",\"currentMs\":";
  out += String(currentMs);
  out += ",\"nextLap\":";
  out += String((int)lapHistoryCount + 1);
  out += ",\"history\":[";
  for (uint8_t i = 0; i < lapHistoryCount; ++i) {
    if (i) out += ",";
    out += "{\"label\":\"";
    out += lapHistoryLabels[i];
    out += "\",\"ms\":";
    out += String(lapHistoryMs[i]);
    out += "}";
  }
  out += "]}";
  return out;
}

static uint8_t speedRunBasePhase(uint8_t phase) {
  if (phase <= 1) return 1;
  if (phase > 4) return 1;
  return phase - 1;
}

static bool usesSpeedRun2Profile(uint8_t phase) {
  return phase >= 2;
}

static bool isContinuousSpeedRunPhase(uint8_t phase) {
  return phase >= 2;
}

static void debugPrint(const String& s) {
  if (serialOutputEnabled()) {
    Serial.print(s);
  }
  if (debugClientConnected()) {
    debugClient.print(s);
  }
}

static void debugPrintln(const String& s) {
  if (serialOutputEnabled()) {
    Serial.println(s);
  }
  if (debugClientConnected()) {
    debugClient.println(s);
  }
}

static void debugPrompt() {
  if (debugClientConnected()) {
    debugClient.print("> ");
  }
}

static const char* primitiveName(MotionPrimitiveType primitive) {
  switch (primitive) {
    case MOTION_NONE: return "none";
    case MOTION_MOVE_ONE_CELL: return "move1";
    case MOTION_MOVE_MULTI_CELL: return "moveN";
    case MOTION_SNAP_CENTER: return "snapcenter";
    case MOTION_TURN_LEFT_90: return "turnL";
    case MOTION_TURN_RIGHT_90: return "turnR";
    case MOTION_TURN_180: return "turn180";
    case MOTION_MOVE_BACKWARD_SHORT: return "back";
    case MOTION_MOVE_FORWARD_SHORT: return "fwdshort";
    default: return "unknown";
  }
}

static const char* headingName(FloodFillExplorer::Dir dir) {
  switch (dir) {
    case FloodFillExplorer::NORTH: return "N";
    case FloodFillExplorer::EAST: return "E";
    case FloodFillExplorer::SOUTH: return "S";
    case FloodFillExplorer::WEST: return "W";
    default: return "?";
  }
}

static String poseSummary(uint8_t x, uint8_t y, FloodFillExplorer::Dir dir) {
  return "(" + String(x) + "," + String(y) + "," + headingName(dir) + ")";
}

static String wallFlagsSummary() {
  return String(robotState.walls.leftWall ? 1 : 0) + "/" +
         String(robotState.walls.frontWall ? 1 : 0) + "/" +
         String(robotState.walls.rightWall ? 1 : 0) +
         " valid=" +
         String(robotState.walls.leftValid ? 1 : 0) + "/" +
         String(robotState.walls.frontValid ? 1 : 0) + "/" +
         String(robotState.walls.rightValid ? 1 : 0);
}

static String wallDistanceSummary() {
  return String(robotState.walls.leftMm) + "/" +
         String(robotState.walls.frontMm) + "/" +
         String(robotState.walls.rightMm);
}

static void debugMotionEvent(const char* tag, MotionPrimitiveType primitive, MotionStatus status,
                             uint8_t beforeX, uint8_t beforeY, FloodFillExplorer::Dir beforeH,
                             uint8_t afterX, uint8_t afterY, FloodFillExplorer::Dir afterH,
                             const String& extra) {
  if (!AppConfig::Debug::DEBUG_MOTION_FLOW) return;
  String msg = String(tag) +
               " prim=" + primitiveName(primitive) +
               " status=" + motionStatusName(status) +
               " mode=" + modeName(robotState.mode) +
               " before=" + poseSummary(beforeX, beforeY, beforeH) +
               " after=" + poseSummary(afterX, afterY, afterH) +
               " walls=" + wallFlagsSummary() +
               " dist=" + wallDistanceSummary();
  if (extra.length() > 0) {
    msg += " " + extra;
  }
  debugPrintln(msg);
}

static void debugWallApplyEvent(const char* tag, const char* source, const String& extra) {
  if (!AppConfig::Debug::DEBUG_WALL_APPLY) return;
  String msg = String(tag) +
               " source=" + source +
               " pose=" + poseSummary(robotState.pose.cellX, robotState.pose.cellY, headingDir()) +
               " walls=" + wallFlagsSummary() +
               " dist=" + wallDistanceSummary();
  if (extra.length() > 0) {
    msg += " " + extra;
  }
  debugPrintln(msg);
}

static void closeDebugConsole(const String& reason) {
  if (!debugClientConnected()) return;
  if (reason.length() > 0) {
    debugClient.println(reason);
  }
  debugClient.clear();
  vTaskDelay(pdMS_TO_TICKS(20));
  debugClient.stop();
}

static void serviceDebugConsole() {
  if (!debugServerStarted) return;
  if (debugServer.hasClient()) {
    WiFiClient incoming = debugServer.accept();
    if (debugClientConnected()) {
      incoming.println("busy");
      incoming.stop();
    } else {
      debugClient = incoming;
      debugClient.setNoDelay(true);
      debugPrintln("[NET] debug console connected");
      debugPrintln("[NET] commands match serial commands");
      debugPrompt();
    }
  }

  static String netLine;
  if (!debugClientConnected()) return;

  while (debugClient.available() > 0) {
    char ch = (char)debugClient.read();
    lastConsoleActivityMs = millis();
    if (ch == '\n' || ch == '\r') {
      if (netLine.length() > 0) {
        handleSerialCommand(netLine);
        netLine = "";
        debugPrompt();
      }
    } else {
      netLine += ch;
    }
  }

  if (!debugClient.connected()) {
    debugClient.stop();
    netLine = "";
  }
}

static void serviceDebugServerState() {
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected) {
    if (!debugServerStarted) {
      debugServer.begin();
      debugServer.setNoDelay(true);
      debugServerStarted = true;
      debugPrintln(String("[NET] TCP console ready: ") + WiFi.localIP().toString() +
                   ":" + String(AppConfig::Wifi::DEBUG_TCP_PORT));
    }
    return;
  }

  if (!debugServerStarted) return;

  closeDebugConsole("[NET] disconnected (wifi offline)");
  debugServer.stop();
  debugServerStarted = false;
}

static void i2cRecover(int sda, int scl) {
  pinMode(sda, OUTPUT);
  pinMode(scl, OUTPUT);
  vTaskDelay(pdMS_TO_TICKS(10));

  if (digitalRead(sda) == LOW) {
    for (int i = 0; i < 9; i++) {
      digitalWrite(scl, HIGH);
      delayMicroseconds(5);
      digitalWrite(scl, LOW);
      delayMicroseconds(5);
    }
  }

  digitalWrite(sda, LOW);
  delayMicroseconds(5);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda, HIGH);

  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);

  Wire.end();
  Wire.begin(sda, scl, AppConfig::I2C::CLOCK_HZ);
  vTaskDelay(pdMS_TO_TICKS(10));
}

static bool onWebLedCommand(const String& cmd, String& response) {
  return ledController.handleCommand(cmd, &response);
}

static bool onWebTelnetReconnect() {
  closeDebugConsole("[NET] disconnected for web telnet reconnect");
  return true;
}

static String onWebHealthJson() {
  char batteryText[96];
  snprintf(batteryText, sizeof(batteryText), "%.2fV (%s, %.0f%%)",
           robotState.batteryVoltage,
           batteryStateName(robotState.batteryState),
           robotState.batteryPercent);

  String out = "\"batteryText\":\"";
  out += String(batteryText);
  out += "\",\"batteryVoltage\":";
  out += String(robotState.batteryVoltage, 2);
  out += ",\"batteryPercent\":";
  out += String(robotState.batteryPercent, 0);
  out += ",\"batteryState\":\"";
  out += String(batteryStateName(robotState.batteryState));
  out += "\"";
  return out;
}

static bool handleLedCommand(const String& cmd) {
  String response;
  if (!ledController.handleCommand(cmd, &response)) return false;
  if (response.length() > 0) debugPrintln(response);
  return true;
}

static void updateRobotLed() {
  if (dbg.isUpdateInProgress()) return;

  if (robotState.goalReached) {
    ledController.setState(LedController::State::WHITE);
    return;
  }

  switch (robotState.mode) {
    case ROBOT_MODE_EXPLORE:
      if (exploreGoalSeen) ledController.setState(LedController::State::BLUE);
      else ledController.setState(LedController::State::GREEN);
      return;
    case ROBOT_MODE_FAULT:
      ledController.setState(LedController::State::RED);
      return;
    default:
      if (robotState.speedRunReady) {
        ledController.setState(LedController::State::WHITE);
        return;
      }
      ledController.setState(LedController::State::OFF);
      return;
  }
}

static void serviceMotorBothFlipTest() {
  if (!motorBothFlipTestEnabled) return;

  const uint32_t now = millis();
  if ((uint32_t)(now - motorFlipLastToggleMs) < 1000) return;

  motorFlipLastToggleMs = now;
  motorFlipPower = -motorFlipPower;
  leftMotor.setPower(motorFlipPower);
  rightMotor.setPower(motorFlipPower);
  debugPrintln(String("[TEST] both motors power=") + (motorFlipPower > 0.0f ? "+100%" : "-100%"));
}

static bool readBootButtonPressed() {
  if (!AppConfig::Inputs::ENABLE_BOOT_BUTTON_LAUNCH) return false;
  const int level = digitalRead(AppConfig::Inputs::BOOT_BUTTON_PIN);
  return AppConfig::Inputs::BOOT_BUTTON_ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

static void dispatchBootButtonAction(uint8_t pressCount) {
  if (pressCount == 0) return;
  if (pressCount == 1) {
    debugPrintln("[BOOT BTN] launch explore");
    beginExplore(false, -1);
    return;
  }

  if (pressCount >= 2 && pressCount <= 5) {
    const uint8_t phase = pressCount - 1;
    debugPrintln("[BOOT BTN] launch speedrun " + String(phase));
    beginSpeedRun(phase);
    return;
  }

  debugPrintln("[BOOT BTN] ignored press count=" + String(pressCount));
}

static void serviceBootButtonLauncher() {
  if (!AppConfig::Inputs::ENABLE_BOOT_BUTTON_LAUNCH) return;

  const bool launcherReady =
      !dbg.isUpdateInProgress() &&
      robotState.mode == ROBOT_MODE_IDLE &&
      !motionController.isBusy();
  if (!launcherReady) {
    bootButtonPressCount = 0;
    bootButtonRawPressed = false;
    bootButtonStablePressed = false;
    bootButtonLastEdgeMs = millis();
    return;
  }

  const uint32_t now = millis();
  const bool rawPressed = readBootButtonPressed();
  if (rawPressed != bootButtonRawPressed) {
    bootButtonRawPressed = rawPressed;
    bootButtonLastEdgeMs = now;
  }

  if ((uint32_t)(now - bootButtonLastEdgeMs) >= AppConfig::Inputs::BOOT_BUTTON_DEBOUNCE_MS &&
      bootButtonStablePressed != bootButtonRawPressed) {
    bootButtonStablePressed = bootButtonRawPressed;
    if (bootButtonStablePressed) {
      if (bootButtonPressCount < 5) {
        bootButtonPressCount++;
      }
      bootButtonLastAcceptedPressMs = now;
      handleLedCommand("cycle");
      debugPrintln("[BOOT BTN] press count=" + String(bootButtonPressCount));
    }
  }

  if (bootButtonPressCount > 0 &&
      (uint32_t)(now - bootButtonLastAcceptedPressMs) >= AppConfig::Inputs::BOOT_BUTTON_MULTI_PRESS_TIMEOUT_MS) {
    const uint8_t actionPressCount = bootButtonPressCount;
    bootButtonPressCount = 0;
    dispatchBootButtonAction(actionPressCount);
  }
}

static void resetExploreLoopTracking() {
  lastStableBestCost = 0xFFFF;
  stableRoundTripCount = 0;
  reachedOriginalGoalOnThisLoop = false;
  exploreGoalSeen = false;
}

static void applyRuntimeGoalRect() {
  explorer.setGoalRect(runtimeGoalX0, runtimeGoalY0, runtimeGoalW, runtimeGoalH);
}

static void applyCurrentPoseAsHomeRect() {
  explorer.setHomeRect(robotState.pose.cellX, robotState.pose.cellY, 1, 1);
}

static void setPose(uint8_t x, uint8_t y, FloodFillExplorer::Dir h) {
  robotState.pose.cellX = x;
  robotState.pose.cellY = y;
  robotState.pose.heading = (uint8_t)h;
  robotState.pose.forwardProgressMm = 0.0f;
  robotState.pose.turnProgressDeg = 0.0f;
}

static FloodFillExplorer::Dir headingDir() {
  return (FloodFillExplorer::Dir)(robotState.pose.heading & 3);
}

static FloodFillExplorer::Dir oppositeDir(FloodFillExplorer::Dir dir) {
  return (FloodFillExplorer::Dir)(((uint8_t)dir + 2) & 3);
}

static void maze_debug_s() {
  String maze = explorer.buildKnownMazeAscii(robotState.pose.cellX, robotState.pose.cellY, headingDir());
  maze.replace("\n", "\r\n");
  debugPrintln("[MAZE]");
  debugPrint("\r\n");
  debugPrint(maze);
  if (!maze.endsWith("\r\n")) {
    debugPrint("\r\n");
  }
  debugPrintln("");
}

static void resetCommonModeState() {
  serialOutputTemporarilyMuted = false;
  motionController.setStopOnCompletion(true);
  motionController.setConfig(AppConfig::makeMotionConfig());
  if (lapTimerRunning) {
    stopLapTimer(false);
  }
  clearForwardActionTracking();
  testLoopMode = TEST_LOOP_NONE;
  runStartSnapPending = false;
  deferPlannerAckUntilSnapCenter = false;
  runStartSnapMode = ROBOT_MODE_IDLE;
  speedRunPreAlignStage = SPEEDRUN_PREALIGN_NONE;
  speedRunPreAlignHasSideSnap = false;
  motorBothFlipTestEnabled = false;
  explorer.setRunning(false);
}

static void enterIdleMode(const String& reason) {
  resetCommonModeState();
  robotState.mode = ROBOT_MODE_IDLE;
  robotState.motionStatus = MOTION_IDLE;
  robotState.activePrimitive = MOTION_NONE;
  motionController.stop();
  updateRobotLed();
  debugPrintln("[MODE] IDLE: " + reason);
}

static void enterFaultMode(const String& reason) {
  resetCommonModeState();
  robotState.mode = ROBOT_MODE_FAULT;
  robotState.lastFault = reason;
  robotState.faultCount++;
  motionController.abort(reason);
  updateRobotLed();
  debugPrintln("[FAULT] " + reason);
}

static void updateOtaSafeMode() {
  const bool otaNow = dbg.isUpdateInProgress();
  if (otaNow == otaSafeModeActive) return;

  otaSafeModeActive = otaNow;
  if (otaSafeModeActive) {
    runStartSnapPending = false;
    deferPlannerAckUntilSnapCenter = false;
    runStartSnapMode = ROBOT_MODE_IDLE;
    explorer.setRunning(false);
    motionController.stop();
    robotState.mode = ROBOT_MODE_IDLE;
    robotState.motionStatus = MOTION_IDLE;
    robotState.activePrimitive = MOTION_NONE;
    testLoopMode = TEST_LOOP_NONE;
    suspendTaskIfRunning(motorTaskHandle);
    suspendTaskIfRunning(tofTaskHandle);
    suspendTaskIfRunning(explorerTaskHandle);
    suspendTaskIfRunning(plannerTaskHandle);
    suspendTaskIfRunning(telemetryTaskHandle);
    debugPrintln("[OTA] App safe mode");
  } else {
    resumeTaskIfSuspended(telemetryTaskHandle);
    resumeTaskIfSuspended(plannerTaskHandle);
    resumeTaskIfSuspended(explorerTaskHandle);
    resumeTaskIfSuspended(tofTaskHandle);
    resumeTaskIfSuspended(motorTaskHandle);
    debugPrintln("[OTA] App resumed");
  }
}

static void suspendTaskIfRunning(TaskHandle_t handle) {
  if (!handle) return;
  if (eTaskGetState(handle) != eSuspended) {
    vTaskSuspend(handle);
  }
}

static void resumeTaskIfSuspended(TaskHandle_t handle) {
  if (!handle) return;
  if (eTaskGetState(handle) == eSuspended) {
    vTaskResume(handle);
  }
}

static void onExplorerWebCommand(const String& cmd) {
  if (cmd == "run") {
    beginExplore(false, -1);
    return;
  }
  if (cmd == "step") {
    beginExplore(false, 1);
    return;
  }
  if (cmd == "pause") {
    enterIdleMode("web explorer pause");
    return;
  }
  if (cmd == "reset") {
    resetMazeToConfiguredStart();
    return;
  }
  if (cmd == "clearmaze") {
    clearMazeMemoryOnly();
  }
}

static void beginExplore(bool clearMaze, int32_t stepBudget) {
  serialOutputTemporarilyMuted = false;
  startLapTimer("HG");
  motionController.setConfig(AppConfig::makeMotionConfig());
  motionController.setUseLatchedTrackMode(false);
  motionController.setStopOnCompletion(true);
  robotState.goalReached = false;
  robotState.speedRunReady = false;
  robotState.mode = ROBOT_MODE_EXPLORE;
  robotState.lastFault = "";
  clearForwardActionTracking();
  resetExploreLoopTracking();
  exploreStepBudget = stepBudget;
  runStartSnapPending = false;
  deferPlannerAckUntilSnapCenter = false;
  runStartSnapMode = ROBOT_MODE_IDLE;
  speedRunPreAlignStage = SPEEDRUN_PREALIGN_NONE;
  speedRunPreAlignHasSideSnap = false;
  explorer.setHardwareMode(true);
  explorer.setStart(robotState.pose.cellX, robotState.pose.cellY, headingDir());
  applyCurrentPoseAsHomeRect();
  applyRuntimeGoalRect();
  if (clearMaze) explorer.clearKnownMaze();
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  updateRobotLed();
  debugPrintln("[MODE] EXPLORE");
  if (exploreStepBudget >= 0) {
    debugPrintln("[EXPLORE] step budget=" + String(exploreStepBudget));
  }
  startRunSnapSequence("run-start snapcenter");
}

static void beginSpeedRun(uint8_t phase) {
  if (phase < 1 || phase > 4) phase = 1;
  robotState.speedRunPhase = phase;
  serialOutputTemporarilyMuted = false;
  stopLapTimer(false);
  motionController.setConfig(usesSpeedRun2Profile(phase)
                               ? AppConfig::makeSpeedRun2MotionConfig()
                               : AppConfig::makeMotionConfig());
  motionController.setUseLatchedTrackMode(usesSpeedRun2Profile(phase));
  motionController.setStopOnCompletion(!isContinuousSpeedRunPhase(phase));
  robotState.mode = ROBOT_MODE_SPEED_RUN;
  robotState.goalReached = false;
  robotState.lastFault = "";
  clearForwardActionTracking();
  runStartSnapPending = false;
  deferPlannerAckUntilSnapCenter = false;
  runStartSnapMode = ROBOT_MODE_IDLE;
  speedRunPreAlignStage = SPEEDRUN_PREALIGN_NONE;
  speedRunPreAlignHasSideSnap = false;
  explorer.setHardwareMode(true);
  explorer.setStart(robotState.pose.cellX, robotState.pose.cellY, headingDir());
  applyCurrentPoseAsHomeRect();
  applyRuntimeGoalRect();
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  explorer.setRunning(false);
  updateRobotLed();
  debugPrintln("[MODE] SPEED RUN " + String((int)phase));
  if (phase == 1) {
    debugPrintln("[SPEEDRUN] phase 1 is the baseline shortest-path run");
  } else if (phase == 2) {
    debugPrintln("[SPEEDRUN] phase 2 is the one-way shortest-path run with the dedicated motion profile");
  } else {
    debugPrintln("[SPEEDRUN] phase " + String((int)phase) +
                 " inherits phase " + String((int)speedRunBasePhase(phase)) +
                 " behavior until its profile is tuned");
  }
  startSpeedRunPreAlignSequence();
}

static void resetMazeToConfiguredStart() {
  enterIdleMode("maze reset to start");
  robotState.goalReached = false;
  robotState.speedRunReady = false;
  robotState.lastFault = "";
  resetExploreLoopTracking();
  exploreStepBudget = -1;

  setPose(AppConfig::Maze::START_X, AppConfig::Maze::START_Y, AppConfig::Maze::START_HEADING);
  explorer.setHardwareMode(true);
  applyCurrentPoseAsHomeRect();
  applyRuntimeGoalRect();
  explorer.setStart(AppConfig::Maze::START_X, AppConfig::Maze::START_Y, AppConfig::Maze::START_HEADING);
  explorer.clearKnownMaze();
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  applyWallsToExplorer();
  updateRobotLed();
  debugPrintln("[CMD] maze cleared and pose reset to configured start");
}

static void clearMazeMemoryOnly() {
  enterIdleMode("maze memory cleared");
  robotState.goalReached = false;
  robotState.speedRunReady = false;
  robotState.lastFault = "";
  resetExploreLoopTracking();
  exploreStepBudget = -1;

  explorer.setHardwareMode(true);
  applyRuntimeGoalRect();
  explorer.setStart(robotState.pose.cellX, robotState.pose.cellY, headingDir());
  applyCurrentPoseAsHomeRect();
  explorer.clearKnownMaze();
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  PersistenceStore::clearMaze();
  updateRobotLed();
  debugPrintln("[CMD] maze memory cleared at current pose");
}

static bool executePlannerAction(FloodFillExplorer::Action act) {
  if (robotState.mode == ROBOT_MODE_SPEED_RUN &&
      usesSpeedRun2Profile(robotState.speedRunPhase) &&
      act == FloodFillExplorer::ACT_TURN_180) {
    enterFaultMode("speedrun 2 unexpected uturn");
    return false;
  }

  bool ok = false;
  MotionPrimitiveType startedPrimitive = MOTION_NONE;
  uint8_t forwardCells = 1;
  bool endsAtKnownWall = false;
  switch (act) {
    case FloodFillExplorer::ACT_MOVE_F:
      forwardCells = explorer.lastActionForwardCells();
      endsAtKnownWall = explorer.lastActionEndsAtKnownWall();
      if (forwardCells <= 1) {
        ok = motionController.moveOneCell();
        startedPrimitive = MOTION_MOVE_ONE_CELL;
      } else {
        ok = motionController.moveCells(forwardCells, endsAtKnownWall);
        startedPrimitive = MOTION_MOVE_MULTI_CELL;
      }
      break;
    case FloodFillExplorer::ACT_TURN_L:
      ok = motionController.turnLeft90();
      startedPrimitive = MOTION_TURN_LEFT_90;
      break;
    case FloodFillExplorer::ACT_TURN_R:
      ok = motionController.turnRight90();
      startedPrimitive = MOTION_TURN_RIGHT_90;
      break;
    case FloodFillExplorer::ACT_TURN_180:
      ok = motionController.turn180();
      startedPrimitive = MOTION_TURN_180;
      break;
    default:
      return true;
  }

  if (!ok) {
    enterFaultMode("failed to start primitive");
    return false;
  }

  if (act == FloodFillExplorer::ACT_MOVE_F) {
    activeForwardActionCellsRequested = (forwardCells > 0) ? forwardCells : 1;
    activeForwardActionCellsCommitted = 0;
  } else {
    clearForwardActionTracking();
  }

  debugMotionEvent("[MOTION START]", startedPrimitive, motionController.status(),
                   robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                   robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                   "action=" + String((int)act) +
                   " cells=" + String((int)((forwardCells > 0) ? forwardCells : 1)));

  return true;
}

static void clearForwardActionTracking() {
  activeForwardActionCellsRequested = 0;
  activeForwardActionCellsCommitted = 0;
}

static void advancePoseForwardOneCell() {
  switch (headingDir()) {
    case FloodFillExplorer::NORTH: if (robotState.pose.cellY > 0) robotState.pose.cellY--; break;
    case FloodFillExplorer::EAST:  if (robotState.pose.cellX < 15) robotState.pose.cellX++; break;
    case FloodFillExplorer::SOUTH: if (robotState.pose.cellY < 15) robotState.pose.cellY++; break;
    case FloodFillExplorer::WEST:  if (robotState.pose.cellX > 0) robotState.pose.cellX--; break;
  }
}

static void advancePoseForFinishedPrimitive(MotionPrimitiveType primitive) {
  if (primitive == MOTION_MOVE_ONE_CELL) {
    advancePoseForwardOneCell();
  } else if (primitive == MOTION_TURN_LEFT_90) {
    robotState.pose.heading = (robotState.pose.heading + 3) & 3;
  } else if (primitive == MOTION_TURN_RIGHT_90) {
    robotState.pose.heading = (robotState.pose.heading + 1) & 3;
  } else if (primitive == MOTION_TURN_180) {
    robotState.pose.heading = (robotState.pose.heading + 2) & 3;
  }

  robotState.pose.forwardProgressMm = 0.0f;
  robotState.pose.turnProgressDeg = 0.0f;
}

static bool handleReachedGoal(bool& skipPostMotionHold) {
  const bool atOriginalGoal = explorer.isInOriginalGoal(robotState.pose.cellX, robotState.pose.cellY);
  const bool atOriginalStart = explorer.isInOriginalStart(robotState.pose.cellX, robotState.pose.cellY);
  if (atOriginalGoal || atOriginalStart) {
    stopLapTimer(true);
  }
  if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
    if (robotState.speedRunPhase == 1 &&
        atOriginalGoal &&
        !atOriginalStart) {
      startLapTimer("GH");
      explorer.advanceTargetAfterReach();
      explorer.setRunning(true);
      robotState.goalReached = false;
      robotState.speedRunReady = true;
      skipPostMotionHold = true;
      updateRobotLed();
      debugPrintln("[SPEEDRUN] reached goal, flip target and return home");
      clearForwardActionTracking();
      motionController.clearCompletionState();
      return false;
    }
    if (robotState.speedRunPhase == 1 && atOriginalStart) {
      explorer.advanceTargetAfterReach();
    }
    if (robotState.speedRunPhase == 1) {
      serialOutputTemporarilyMuted = false;
    } else if (usesSpeedRun2Profile(robotState.speedRunPhase)) {
      debugPrintln("[SPEEDRUN] reached goal, finish one-way run");
    }
    robotState.goalReached = true;
    updateRobotLed();
    debugPrintln("[GOAL] Goal reached");
    robotState.speedRunReady = true;
    clearForwardActionTracking();
    enterIdleMode("speed run finished");
    return false;
  }

  if (robotState.mode == ROBOT_MODE_EXPLORE &&
      AppConfig::Explorer::CONTINUE_AFTER_GOAL) {
    robotState.goalReached = true;
    updateRobotLed();
    debugPrintln("[GOAL] Goal reached");
    exploreGoalSeen = true;
    const uint16_t bestCost = explorer.bestKnownCostOriginalStartToGoal();
    if (atOriginalGoal) {
      reachedOriginalGoalOnThisLoop = true;
      debugPrintln("[EXPLORE] reached goal leg bestCost=" + String(bestCost));
      startLapTimer("GH");
    } else if (atOriginalStart &&
               reachedOriginalGoalOnThisLoop) {
      reachedOriginalGoalOnThisLoop = false;
      startLapTimer("HG");
      if (bestCost == lastStableBestCost) {
        stableRoundTripCount++;
      } else {
        lastStableBestCost = bestCost;
        stableRoundTripCount = 1;
      }

      debugPrintln("[EXPLORE] round trip bestCost=" + String(bestCost) +
                   " stable=" + String(stableRoundTripCount) + "/" +
                   String(AppConfig::Explorer::SHORTEST_PATH_STABLE_ROUND_TRIPS));

      if (bestCost != 0xFFFF &&
          stableRoundTripCount >= AppConfig::Explorer::SHORTEST_PATH_STABLE_ROUND_TRIPS) {
        robotState.speedRunReady = true;
        debugPrintln("[EXPLORE] shortest path known cost=" + String(bestCost));
        PersistenceStore::saveMaze(explorer);
        clearForwardActionTracking();
        enterIdleMode("shortest path known");
        return false;
      }
    }

    explorer.setRunning(true);
    robotState.goalReached = false;
    skipPostMotionHold = true;
    updateRobotLed();
    debugPrintln("[EXPLORE] target toggled, continue exploring");
  }

  return true;
}

static bool shouldStopCorridorAfterCommittedCell() {
  if (activeForwardActionCellsCommitted >= activeForwardActionCellsRequested) return false;
  return robotState.walls.frontValid &&
         robotState.walls.frontWall;
}

static bool commitForwardActionCell(bool allowFrontStopCompletion, bool& stepBudgetReached, bool& reachedGoal) {
  if (activeForwardActionCellsRequested == 0 ||
      activeForwardActionCellsCommitted >= activeForwardActionCellsRequested) {
    return false;
  }

  const float nextBoundaryMm =
    (float)(activeForwardActionCellsCommitted + 1) * AppConfig::Motion::CELL_DISTANCE_MM;
  bool crossedBoundary = robotState.pose.forwardProgressMm >= nextBoundaryMm;

  if (!crossedBoundary && allowFrontStopCompletion) {
    const bool finalCell = (activeForwardActionCellsCommitted + 1) == activeForwardActionCellsRequested;
    // Keep one-cell completion aligned with single-cell motion behavior.
    // Corridor threshold applies only when the active target is truly multi-cell.
    const float frontStopThresholdMm =
      (activeForwardActionCellsRequested <= 1)
        ? AppConfig::Motion::FRONT_STOP_MM
        : AppConfig::Motion::CORRIDOR_FRONT_STOP_MM;
    crossedBoundary = finalCell &&
                      robotState.walls.frontValid &&
                      robotState.walls.frontWall &&
                      robotState.walls.frontMm > 0 &&
                      robotState.walls.frontMm <= frontStopThresholdMm;
  }

  if (!crossedBoundary) return false;

  advancePoseForwardOneCell();
  activeForwardActionCellsCommitted++;
  updateRobotState();
  if (robotState.mode == ROBOT_MODE_SPEED_RUN &&
      usesSpeedRun2Profile(robotState.speedRunPhase)) {
    motionController.latchStraightTrackMode(robotState.walls);
  }

  const bool truncateHere = shouldStopCorridorAfterCommittedCell();
  if (truncateHere) {
    activeForwardActionCellsRequested = activeForwardActionCellsCommitted;
    motionController.limitMoveCellTargetCount(activeForwardActionCellsRequested);
    if (robotState.mode != ROBOT_MODE_SPEED_RUN) {
      explorer.truncatePendingForwardAction();
    }
  }

  if (robotState.mode != ROBOT_MODE_SPEED_RUN) {
    debugWallApplyEvent("[WALL APPLY]", "forward_progress",
                        "step=" + String((int)activeForwardActionCellsCommitted) + "/" +
                        String((int)activeForwardActionCellsRequested));
    applyWallsToExplorer();
    debugWallApplyEvent("[WALL APPLIED]", "forward_progress");
    reachedGoal = explorer.ackPendingActionExternal(true,
      robotState.pose.cellX,
      robotState.pose.cellY,
      headingDir());
  } else {
    explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
    reachedGoal = explorer.atGoal();
  }

  if (robotState.mode == ROBOT_MODE_EXPLORE && exploreStepBudget > 0) {
    exploreStepBudget--;
    debugPrintln("[EXPLORE] steps remaining=" + String(exploreStepBudget));
    if (exploreStepBudget == 0) {
      stepBudgetReached = true;
    }
  }

  return true;
}

static void handleMotionCompletion() {
  MotionStatus status = motionController.status();
  MotionPrimitiveType primitive = motionController.lastFinishedPrimitive();
  const uint8_t beforeX = robotState.pose.cellX;
  const uint8_t beforeY = robotState.pose.cellY;
  const FloodFillExplorer::Dir beforeH = headingDir();
  const bool isTurnPrimitive =
    primitive == MOTION_TURN_LEFT_90 ||
    primitive == MOTION_TURN_RIGHT_90 ||
    primitive == MOTION_TURN_180;
  const bool isSnapCenterPrimitive = primitive == MOTION_SNAP_CENTER;
  const bool isMultiCellForward = primitive == MOTION_MOVE_MULTI_CELL;
  const bool continuousSpeedRun =
    robotState.mode == ROBOT_MODE_SPEED_RUN &&
    isContinuousSpeedRunPhase(robotState.speedRunPhase) &&
    speedRunPreAlignStage == SPEEDRUN_PREALIGN_NONE;
  bool skipPostMotionHold = false;
  bool stepBudgetReached = false;

  if (status == MOTION_COMPLETED) {
    bool reachedGoal = false;
    bool forwardAlreadyCommitted = false;

    if (isMultiCellForward) {
      updateRobotState();
      while (commitForwardActionCell(true, stepBudgetReached, reachedGoal)) {
        forwardAlreadyCommitted = true;
        if (reachedGoal && !handleReachedGoal(skipPostMotionHold)) {
          return;
        }
        if (stepBudgetReached) {
          enterIdleMode("explore step budget reached");
          return;
        }
      }
      if (activeForwardActionCellsCommitted < activeForwardActionCellsRequested) {
        enterFaultMode("corridor move ended early");
        return;
      }
    } else {
      advancePoseForFinishedPrimitive(primitive);
    }

    debugMotionEvent("[MOTION END]", primitive, status,
                     beforeX, beforeY, beforeH,
                     robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                     isMultiCellForward ?
                       ("cells=" + String((int)activeForwardActionCellsCommitted) + "/" +
                        String((int)activeForwardActionCellsRequested)) :
                       String());
    if (!continuousSpeedRun) {
      leftMotor.hardStop();
      rightMotor.hardStop();
    }

    if (robotState.mode == ROBOT_MODE_SPEED_RUN &&
        speedRunPreAlignStage != SPEEDRUN_PREALIGN_NONE) {
      if (speedRunPreAlignStage == SPEEDRUN_PREALIGN_AFTER_SIDE_TURN) {
        updateRobotState();
        if (shouldSnapCenterFromKnownBackWall()) {
          if (!motionController.snapCenter()) {
            enterFaultMode("failed to start speedrun pre-run side snapcenter");
            return;
          }
          speedRunPreAlignStage = SPEEDRUN_PREALIGN_AFTER_SIDE_SNAP;
          debugMotionEvent("[MOTION START]", MOTION_SNAP_CENTER, motionController.status(),
                           robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                           robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                           "source=speedrun-prerun-side-snap");
          debugPrintln("[SPEEDRUN] pre-run side snapcenter");
          return;
        }

        const bool ok = speedRunPreAlignTurnRight ? motionController.turnLeft90()
                                                  : motionController.turnRight90();
        if (!ok) {
          enterFaultMode("failed to start speedrun pre-run return turn");
          return;
        }
        speedRunPreAlignStage = SPEEDRUN_PREALIGN_AFTER_RETURN_TURN;
        debugMotionEvent("[MOTION START]",
                         speedRunPreAlignTurnRight ? MOTION_TURN_LEFT_90 : MOTION_TURN_RIGHT_90,
                         motionController.status(),
                         robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                         robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                         "source=speedrun-prerun-return-turn");
        debugPrintln("[SPEEDRUN] pre-run return turn");
        return;
      }

      if (speedRunPreAlignStage == SPEEDRUN_PREALIGN_AFTER_SIDE_SNAP) {
        const bool ok = speedRunPreAlignTurnRight ? motionController.turnLeft90()
                                                  : motionController.turnRight90();
        if (!ok) {
          enterFaultMode("failed to start speedrun pre-run return turn");
          return;
        }
        speedRunPreAlignStage = SPEEDRUN_PREALIGN_AFTER_RETURN_TURN;
        debugMotionEvent("[MOTION START]",
                         speedRunPreAlignTurnRight ? MOTION_TURN_LEFT_90 : MOTION_TURN_RIGHT_90,
                         motionController.status(),
                         robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                         robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                         "source=speedrun-prerun-return-turn");
        debugPrintln("[SPEEDRUN] pre-run return turn");
        return;
      }

      if (speedRunPreAlignStage == SPEEDRUN_PREALIGN_AFTER_RETURN_TURN) {
        updateRobotState();
        if (shouldSnapCenterFromKnownBackWall()) {
          if (!motionController.snapCenter()) {
            enterFaultMode("failed to start speedrun pre-run snapcenter");
            return;
          }
          speedRunPreAlignStage = SPEEDRUN_PREALIGN_AFTER_FINAL_SNAP;
          debugMotionEvent("[MOTION START]", MOTION_SNAP_CENTER, motionController.status(),
                           robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                           robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                           "source=speedrun-prerun-final-snap");
          debugPrintln("[SPEEDRUN] pre-run snapcenter");
          return;
        }

        finishSpeedRunStart();
        clearForwardActionTracking();
        motionController.stop();
        return;
      }

      if (speedRunPreAlignStage == SPEEDRUN_PREALIGN_AFTER_FINAL_SNAP) {
        finishSpeedRunStart();
        clearForwardActionTracking();
        motionController.stop();
        return;
      }
    }

    if (isTurnPrimitive &&
        !deferPlannerAckUntilSnapCenter &&
        robotState.mode == ROBOT_MODE_EXPLORE &&
        primitive == MOTION_TURN_180 &&
        (updateRobotState(), shouldSnapCenterFromKnownBackWall())) {
      if (!motionController.snapCenter()) {
        enterFaultMode("failed to start post-turn snapcenter");
        return;
      }
      deferPlannerAckUntilSnapCenter = true;
      debugMotionEvent("[MOTION START]", MOTION_SNAP_CENTER, motionController.status(),
                       robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                       robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                       "source=post-turn");
      debugPrintln("[SNAP] post-turn snapcenter");
      return;
    }

    updateRobotState();
    if (!forwardAlreadyCommitted && robotState.mode != ROBOT_MODE_SPEED_RUN) {
      debugWallApplyEvent("[WALL APPLY]", "motion_complete");
      applyWallsToExplorer();
      debugWallApplyEvent("[WALL APPLIED]", "motion_complete");
    }

    if (!forwardAlreadyCommitted) {
      if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
        explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
        reachedGoal = explorer.atGoal();
      } else if (deferPlannerAckUntilSnapCenter) {
        deferPlannerAckUntilSnapCenter = false;
        reachedGoal = explorer.ackPendingActionExternal(true,
          robotState.pose.cellX,
          robotState.pose.cellY,
          headingDir());
      } else if (!runStartSnapPending || !isSnapCenterPrimitive) {
        reachedGoal = explorer.ackPendingActionExternal(true,
          robotState.pose.cellX,
          robotState.pose.cellY,
          headingDir());
      }
    } else if (deferPlannerAckUntilSnapCenter) {
      deferPlannerAckUntilSnapCenter = false;
    }

    if (runStartSnapPending && isSnapCenterPrimitive) {
      runStartSnapPending = false;
      explorer.setRunning(runStartSnapMode == ROBOT_MODE_EXPLORE || runStartSnapMode == ROBOT_MODE_SPEED_RUN);
      runStartSnapMode = ROBOT_MODE_IDLE;
      debugPrintln("[SNAP] run-start snapcenter complete");
    }

    if (AppConfig::Motion::AUTO_PRINT_MAZE_AFTER_SENSE && robotState.mode == ROBOT_MODE_EXPLORE) {
      maze_debug_s();
    }

    if (!forwardAlreadyCommitted &&
        primitive == MOTION_MOVE_ONE_CELL &&
        robotState.mode == ROBOT_MODE_EXPLORE &&
        exploreStepBudget > 0) {
      exploreStepBudget--;
      debugPrintln("[EXPLORE] steps remaining=" + String(exploreStepBudget));
      if (exploreStepBudget == 0) {
        stepBudgetReached = true;
      }
    }

    if (reachedGoal && !handleReachedGoal(skipPostMotionHold)) {
      return;
    }

    if (stepBudgetReached) {
      enterIdleMode("explore step budget reached");
      return;
    }

    if (AppConfig::Motion::POST_MOTION_HARD_STOP_HOLD_MS > 0 &&
        !continuousSpeedRun &&
        !skipPostMotionHold &&
        motionController.status() == MOTION_COMPLETED &&
        robotState.mode != ROBOT_MODE_IDLE &&
        robotState.mode != ROBOT_MODE_FAULT) {
      if (AppConfig::Debug::DEBUG_WALL_APPLY) {
        debugPrintln("[STOP HOLD] wait " + String(AppConfig::Motion::POST_MOTION_HARD_STOP_HOLD_MS) +
                     "ms before next motion");
      }
      vTaskDelay(pdMS_TO_TICKS(AppConfig::Motion::POST_MOTION_HARD_STOP_HOLD_MS));
    }
  } else if (status == MOTION_FAILED || status == MOTION_ABORTED) {
    debugMotionEvent("[MOTION END]", primitive, status,
                     beforeX, beforeY, beforeH,
                     robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                     "error=" + motionController.lastError());
    if (robotState.mode != ROBOT_MODE_SPEED_RUN) {
      explorer.ackPendingActionExternal(false,
        robotState.pose.cellX,
        robotState.pose.cellY,
        headingDir());
    }
    enterFaultMode(motionController.lastError());
  }

  clearForwardActionTracking();
  if (continuousSpeedRun &&
      status == MOTION_COMPLETED &&
      robotState.mode == ROBOT_MODE_SPEED_RUN) {
    motionController.clearCompletionState();
  } else {
    motionController.stop();
  }
}

static void serviceActiveForwardAction() {
  if (!motionController.isBusy()) return;
  if (motionController.primitive() != MOTION_MOVE_MULTI_CELL) return;
  if (activeForwardActionCellsRequested <= 1) return;

  bool stepBudgetReached = false;
  bool reachedGoal = false;
  bool skipPostMotionHold = false;

  while (commitForwardActionCell(false, stepBudgetReached, reachedGoal)) {
    if (reachedGoal && !handleReachedGoal(skipPostMotionHold)) {
      return;
    }
    if (stepBudgetReached) {
      enterIdleMode("explore step budget reached");
      return;
    }
  }
}

static void updateRobotState() {
  mouseBattery.update();

  robotState.batteryVoltage = mouseBattery.voltage();
  robotState.batteryPercent = mouseBattery.percent();
  robotState.batteryState = mouseBattery.state();

  robotState.leftTicks = leftMotor.getTicks();
  robotState.rightTicks = rightMotor.getTicks();
  robotState.leftTps = leftMotor.getTicksPerSecond();
  robotState.rightTps = rightMotor.getTicksPerSecond();

  for (uint8_t i = 0; i < AppConfig::Tof::SENSOR_COUNT; ++i) {
    robotState.sensors[i] = tofArray.getDistance(i);
  }

  MultiVL53L0X::SensorState sensed = tofArray.getSensorState();
  robotState.walls.leftWall = sensed.leftWall;
  robotState.walls.frontWall = sensed.frontWall;
  robotState.walls.rightWall = sensed.rightWall;
  robotState.walls.leftValid = sensed.leftValid;
  robotState.walls.frontValid = sensed.frontValid;
  robotState.walls.rightValid = sensed.rightValid;
  robotState.walls.leftMm = sensed.leftMm;
  robotState.walls.frontMm = sensed.frontMm;
  robotState.walls.rightMm = sensed.rightMm;

  robotState.sensorHealthy = sensed.leftValid || sensed.frontValid || sensed.rightValid;
  robotState.readyForMotion = motorsOk && tofOk && batteryOk;
}

static void applyWallsToExplorer() {
  explorer.observeRelativeWalls(
    robotState.pose.cellX,
    robotState.pose.cellY,
    headingDir(),
    robotState.walls.leftWall,
    robotState.walls.frontWall,
    robotState.walls.rightWall,
    robotState.walls.leftValid,
    robotState.walls.frontValid,
    robotState.walls.rightValid
  );
}

static bool shouldSnapCenterFromKnownBackWall() {
  const FloodFillExplorer::Dir backDir = oppositeDir(headingDir());
  bool known = false;
  bool wall = false;
  explorer.getKnownWall(robotState.pose.cellX, robotState.pose.cellY, backDir, known, wall);
  return known && wall;
}

static void finishSpeedRunStart() {
  speedRunPreAlignStage = SPEEDRUN_PREALIGN_NONE;
  debugPrintln("[SPEEDRUN] pre-run centering complete");
  serialOutputTemporarilyMuted = (robotState.speedRunPhase == 1);
  startLapTimer("HG");
  explorer.setRunning(true);
  updateRobotLed();
}

static bool startSpeedRunPreAlignSequence() {
  speedRunPreAlignStage = SPEEDRUN_PREALIGN_NONE;
  speedRunPreAlignHasSideSnap = false;

  if (!shouldSnapCenterFromKnownBackWall()) {
    debugPrintln("[SPEEDRUN] skip pre-run centering (need known back wall)");
    finishSpeedRunStart();
    return true;
  }

  if (!motionController.snapCenter()) {
    enterFaultMode("failed to start speedrun pre-run snapcenter");
    return false;
  }

  speedRunPreAlignStage = SPEEDRUN_PREALIGN_AFTER_FINAL_SNAP;
  debugMotionEvent("[MOTION START]", MOTION_SNAP_CENTER, motionController.status(),
                   robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                   robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                   "source=speedrun-prerun-final-snap");
  debugPrintln("[SPEEDRUN] pre-run snapcenter");
  return true;
}

static bool startRunSnapSequence(const char* label) {
  explorer.setRunning(false);
  if (!shouldSnapCenterFromKnownBackWall()) {
    explorer.setRunning(robotState.mode == ROBOT_MODE_EXPLORE || robotState.mode == ROBOT_MODE_SPEED_RUN);
    debugPrintln(String("[SNAP] skip ") + label + " (need known back wall)");
    return true;
  }
  if (!motionController.snapCenter()) {
    enterFaultMode(String("failed to start ") + label);
    return false;
  }

  runStartSnapPending = true;
  runStartSnapMode = robotState.mode;
  debugMotionEvent("[MOTION START]", MOTION_SNAP_CENTER, motionController.status(),
                   robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                   robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                   String("source=") + label);
  debugPrintln(String("[SNAP] ") + label);
  return true;
}

void setupApp(TaskFunction_t userTaskFn, TaskFunction_t plannerTaskFn) {
  ledController.begin();
  ledController.setState(LedController::State::RED);
  Serial.begin(921600);
  vTaskDelay(pdMS_TO_TICKS(200));
  i2cRecover(AppConfig::I2C::SDA, AppConfig::I2C::SCL);
  if (AppConfig::Inputs::ENABLE_BOOT_BUTTON_LAUNCH) {
    pinMode(AppConfig::Inputs::BOOT_BUTTON_PIN,
            AppConfig::Inputs::BOOT_BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  }
  setPose(AppConfig::Maze::START_X, AppConfig::Maze::START_Y, AppConfig::Maze::START_HEADING);

  WiFiOtaWebSerial::Config wifiCfg;
  wifiCfg.ssid = AppConfig::Wifi::SSID;
  wifiCfg.pass = AppConfig::Wifi::PASS;
  wifiCfg.hostname = AppConfig::Wifi::HOSTNAME;
  wifiCfg.enableWeb = AppConfig::Wifi::ENABLE_WEB_LOG;
  wifiCfg.debugTcpPort = AppConfig::Wifi::DEBUG_TCP_PORT;
  wifiCfg.enableUploadWeb = AppConfig::Wifi::ENABLE_UPLOAD_WEB;
  wifiCfg.uploadPort = AppConfig::Wifi::UPLOAD_WEB_PORT;
  wifiCfg.wifiCore = AppConfig::Wifi::CORE;
  wifiCfg.wifiTaskStack = AppConfig::Wifi::TASK_STACK;
  wifiCfg.wifiTaskPrio = AppConfig::Wifi::TASK_PRIORITY;
  wifiCfg.serviceDelayMs = AppConfig::Wifi::SERVICE_DELAY_MS;
  wifiCfg.wifiConnectTimeoutMs = AppConfig::Wifi::CONNECT_TIMEOUT_MS;
  wifiCfg.wifiReconnectIntervalMs = AppConfig::Wifi::RECONNECT_INTERVAL_MS;
  wifiCfg.uploadCore = (wifiCfg.wifiCore == 0) ? 1 : 0;
  wifiCfg.uploadTaskPrio = wifiCfg.wifiTaskPrio + 1;
  wifiCfg.uploadTaskStack = wifiCfg.wifiTaskStack;
  dbg.setLedCommandHandler(onWebLedCommand);
  dbg.setTelnetReconnectHandler(onWebTelnetReconnect);
  dbg.setHealthJsonProvider(onWebHealthJson);
  dbg.setSerialOutputAllowedHandler(serialOutputEnabled);
  dbg.setReconnectAllowedHandler(wifiReconnectAllowed);
  explorer.setStateExtrasJsonProvider(explorerLapStateJson);
  wifiOk = dbg.begin(wifiCfg);
  debugPrintln(wifiOk ? "Boot OK" : "Boot with WiFi failed");
  debugServerStarted = false;

  motorsOk = leftMotor.begin(AppConfig::Motors::LEFT_PINS,
                             AppConfig::Motors::LEFT_PWM_CHANNEL,
                             AppConfig::Motors::PWM_FREQ,
                             AppConfig::Motors::PWM_RESOLUTION_BITS) &&
             rightMotor.begin(AppConfig::Motors::RIGHT_PINS,
                              AppConfig::Motors::RIGHT_PWM_CHANNEL,
                              AppConfig::Motors::PWM_FREQ,
                              AppConfig::Motors::PWM_RESOLUTION_BITS);
  if (motorsOk) {
    leftMotor.setSpeedPID(AppConfig::Motors::PID_KP, AppConfig::Motors::PID_KI,
                          AppConfig::Motors::PID_KD, AppConfig::Motors::PID_OUT_LIMIT,
                          AppConfig::Motors::PID_I_LIMIT, AppConfig::Motors::PID_D_FILTER_HZ,
                          AppConfig::Motors::PID_SLEW_RATE);
    rightMotor.setSpeedPID(AppConfig::Motors::PID_KP, AppConfig::Motors::PID_KI,
                           AppConfig::Motors::PID_KD, AppConfig::Motors::PID_OUT_LIMIT,
                           AppConfig::Motors::PID_I_LIMIT, AppConfig::Motors::PID_D_FILTER_HZ,
                           AppConfig::Motors::PID_SLEW_RATE);
  }

  tofArray.setWallThreshold(AppConfig::Tof::WALL_THRESHOLD_MM);
  tofArray.setCenterPid(AppConfig::Motion::CENTER_PID_KP,
                        AppConfig::Motion::CENTER_PID_KI,
                        AppConfig::Motion::CENTER_PID_KD,
                        AppConfig::Motion::CENTER_PID_I_LIMIT,
                        AppConfig::Motion::CENTER_PID_OUT_LIMIT);
  tofArray.setCenterTargets(AppConfig::Motion::CENTER_TARGET_LEFT_MM,
                            AppConfig::Motion::CENTER_TARGET_RIGHT_MM);
  tofOk = tofArray.begin();

  mouseBattery.begin(AppConfig::Battery::ADC_PIN);
  mouseBattery.setCalibration(AppConfig::Battery::RAW_LOW, AppConfig::Battery::VOLTAGE_LOW,
                              AppConfig::Battery::RAW_HIGH, AppConfig::Battery::VOLTAGE_HIGH);
  mouseBattery.setDivider(AppConfig::Battery::DIVIDER_R_TOP_KOHM,
                          AppConfig::Battery::DIVIDER_R_BOTTOM_KOHM);
  mouseBattery.setThresholds(AppConfig::Battery::WARNING_VOLTAGE,
                             AppConfig::Battery::CRITICAL_VOLTAGE);
  mouseBattery.update();
  batteryOk = mouseBattery.isReady();

  PersistenceStore::setLogger(logToDbg);
  PersistenceStore::begin();

  motionController.begin(leftMotor, rightMotor, tofArray, &mouseBattery);
  motionController.setConfig(AppConfig::makeMotionConfig());

  explorer.setLog(logToDbg);
  explorer.setWebCommandHandler(onExplorerWebCommand);
  explorer.begin(AppConfig::makeExplorerConfig());
  explorer.setHardwareMode(true);
  applyCurrentPoseAsHomeRect();
  applyRuntimeGoalRect();
  explorer.setStart(robotState.pose.cellX, robotState.pose.cellY, headingDir());
  if (!PersistenceStore::loadMaze(explorer, robotState.pose.cellX, robotState.pose.cellY, headingDir())) {
    explorer.clearKnownMaze();
    explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  } else {
    robotState.speedRunReady = true;
    debugPrintln("[SPIFFS] shortest path available from saved maze");
  }

  updateRobotState();
  updateRobotLed();
  printStartupSummary();

  xTaskCreatePinnedToCore(motorTask,     "motor",     4096, nullptr, 3, &motorTaskHandle,     0);
  xTaskCreatePinnedToCore(tofTask,       "tof",       4096, nullptr, 2, &tofTaskHandle,       0);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 1, &telemetryTaskHandle, 1);
  xTaskCreatePinnedToCore(explorerTask,  "explorer",  8192, nullptr, 2, &explorerTaskHandle,  1);
  xTaskCreatePinnedToCore(plannerTaskFn, "planner",   4096, nullptr, 2, &plannerTaskHandle,   1);
  xTaskCreatePinnedToCore(userTaskFn,    "user",      6144, nullptr, 2, &userTaskHandle,      1);

  enterIdleMode("ready");
  updateRobotLed();
}

void loopApp() {
  vTaskDelay(1);
}

void userTaskBody(void* arg) {
  (void)arg;
  String line;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(AppConfig::Tasks::USER_LOOP_PERIOD_MS);

  for (;;) {
    serviceLoopWatchdog(userLoopWatchdog, AppConfig::Tasks::USER_LOOP_PERIOD_MS);
    updateOtaSafeMode();
    if (dbg.isUpdateInProgress()) {
      resetLoopWatchdogState(userLoopWatchdog);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    updateRobotState();
    motionController.update(robotState);
    serviceMotorBothFlipTest();
    serviceBootButtonLauncher();
    serviceDebugServerState();
    serviceDebugConsole();

    while (Serial.available() > 0) {
      char ch = (char)Serial.read();
      lastConsoleActivityMs = millis();
      if (ch == '\n' || ch == '\r') {
        if (line.length() > 0) {
          handleSerialCommand(line);
          line = "";
        }
      } else {
        line += ch;
      }
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

void plannerTaskBody(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(AppConfig::Tasks::PLANNER_LOOP_PERIOD_MS);

  for (;;) {
    serviceLoopWatchdog(plannerLoopWatchdog, AppConfig::Tasks::PLANNER_LOOP_PERIOD_MS);
    if (dbg.isUpdateInProgress()) {
      resetLoopWatchdogState(plannerLoopWatchdog);
      vTaskDelayUntil(&last, period);
      continue;
    }

      const MotionStatus motionStatus = motionController.status();
      if (motionController.isBusy()) {
        serviceActiveForwardAction();
      }
      if (motionStatus == MOTION_COMPLETED ||
          motionStatus == MOTION_FAILED ||
          motionStatus == MOTION_ABORTED) {
      handleMotionCompletion();
      vTaskDelayUntil(&last, period);
      continue;
    }

    if ((robotState.mode == ROBOT_MODE_EXPLORE || robotState.mode == ROBOT_MODE_SPEED_RUN) &&
        explorer.isRunning() &&
        !motionController.isBusy()) {
      if (!robotState.readyForMotion) {
        enterFaultMode("robot not ready for motion");
      } else {
        updateRobotState();
        explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
        FloodFillExplorer::Action act = FloodFillExplorer::ACT_NONE;
        if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
          act = explorer.requestNextActionNoAck();
        } else {
          debugWallApplyEvent("[WALL APPLY]", "planner_idle");
          applyWallsToExplorer();
          debugWallApplyEvent("[WALL APPLIED]", "planner_idle");
          act = explorer.requestNextAction();
        }
        if (act == FloodFillExplorer::ACT_NONE) {
          if (robotState.mode == ROBOT_MODE_EXPLORE && explorer.atGoal()) {
            robotState.goalReached = true;
            robotState.speedRunReady = true;
            updateRobotLed();
            enterIdleMode("explore finished");
          } else if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
            robotState.goalReached = true;
            updateRobotLed();
            enterIdleMode("speed run finished");
          }
        } else {
          executePlannerAction(act);
        }
      }
    }

    vTaskDelayUntil(&last, period);
  }
}

static void motorTask(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS);

  for (;;) {
    serviceLoopWatchdog(motorLoopWatchdog, AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS);
    if (dbg.isUpdateInProgress()) {
      resetLoopWatchdogState(motorLoopWatchdog);
      vTaskDelayUntil(&last, period);
      continue;
    }
    leftMotor.update();
    rightMotor.update();
    vTaskDelayUntil(&last, period);
  }
}

static void explorerTask(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(AppConfig::Tasks::EXPLORER_LOOP_PERIOD_MS);
  for (;;) {
    serviceLoopWatchdog(explorerLoopWatchdog, AppConfig::Tasks::EXPLORER_LOOP_PERIOD_MS);
    if (dbg.isUpdateInProgress()) {
      resetLoopWatchdogState(explorerLoopWatchdog);
      vTaskDelay(pdMS_TO_TICKS(50));
      last = xTaskGetTickCount();
      continue;
    }
    explorer.loop();
    vTaskDelayUntil(&last, period);
  }
}

static void tofTask(void* arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(AppConfig::Tasks::TOF_LOOP_PERIOD_MS);

  for (;;) {
    serviceLoopWatchdog(tofLoopWatchdog, AppConfig::Tasks::TOF_LOOP_PERIOD_MS);
    if (dbg.isUpdateInProgress()) {
      resetLoopWatchdogState(tofLoopWatchdog);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }
    tofArray.update();
    vTaskDelayUntil(&lastWake, period);
  }
}

static void telemetryTask(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(AppConfig::Tasks::TELEMETRY_LOOP_PERIOD_MS);

  for (;;) {
    serviceLoopWatchdog(telemetryLoopWatchdog, AppConfig::Tasks::TELEMETRY_LOOP_PERIOD_MS);
    if (dbg.isUpdateInProgress()) {
      resetLoopWatchdogState(telemetryLoopWatchdog);
      vTaskDelayUntil(&last, period);
      continue;
    }
    if ((uint32_t)(millis() - lastConsoleActivityMs) < kConsoleQuietMs) {
      vTaskDelayUntil(&last, period);
      continue;
    }
    if (robotState.mode == ROBOT_MODE_MANUAL_TEST) {
      switch (testLoopMode) {
        case TEST_LOOP_STATUS:
          robot_debug_s();
          break;
        case TEST_LOOP_BATTERY:
          battery_debug_s();
          break;
        case TEST_LOOP_SENSORS:
          tof_debug_s();
          break;
        case TEST_LOOP_SENSORS_RAW:
          tof_raw_debug_s();
          break;
        case TEST_LOOP_ENCODERS:
          motor_debug_s();
          break;
        case TEST_LOOP_MAZE:
          maze_debug_s();
          break;
        case TEST_LOOP_NONE:
        default:
          break;
      }
    }
    vTaskDelayUntil(&last, period);
  }
}

static void printStartupSummary() {
  debugPrintln(String("[BOOT] WiFi=") + (wifiOk ? "OK" : "FAIL"));
  debugPrintln(String("[BOOT] Motors=") + (motorsOk ? "OK" : "FAIL"));
  debugPrintln(String("[BOOT] TOF=") + (tofOk ? "OK" : "FAIL"));
  debugPrintln(String("[BOOT] Battery=") + (batteryOk ? "OK" : "FAIL"));
  if (WiFi.status() == WL_CONNECTED) {
    debugPrintln(String("[BOOT] TCP Console: ") + WiFi.localIP().toString() + ":" +
                 String(AppConfig::Wifi::DEBUG_TCP_PORT));
  } else {
    debugPrintln(String("[BOOT] TCP Console: waiting for WiFi on :") +
                 String(AppConfig::Wifi::DEBUG_TCP_PORT));
  }
  debugPrintln("[CMD] help | explore [n] | speedrun [1-4] | idle | stop | brake | restart | move [n] | back | left | right | uturn | testsnap | status | resetpose x y h | setgoal x y w h | clearmaze");
  debugPrintln("[SPEEDRUN] 1=baseline round trip | 2=one-way home->goal | 3-4 inherit previous until tuned");
  debugPrintln("[CMD] led cycle|rotate|off|red|green|blue|yellow|cyan|magenta|white");
  debugPrintln("[CMD] maze");
  debugPrintln("[CMD] test | test off | test loop status|battery|sensors|sensorsraw|encoders|maze|off");
  debugPrintln("[CMD] test battery|sensors|sensorsraw|motorl [tps]|motorr [tps]|motor both [tps]|encoders");
  if (AppConfig::Inputs::ENABLE_BOOT_BUTTON_LAUNCH) {
    debugPrintln("[BOOT BTN] 1=explore 2=speedrun1 3=speedrun2 4=speedrun3 5=speedrun4 (5s timeout)");
  }
}

static void handleSerialCommand(const String& rawLine) {
  String line = rawLine;
  line.trim();
  line.toLowerCase();

  if (line == "help") {
    printStartupSummary();
    return;
  }
  if (line == "status") {
    robot_debug_s();
    return;
  }
  if (line == "restart" || line == "reboot") {
    debugPrintln("[CMD] restarting...");
    closeDebugConsole("[NET] disconnecting for restart");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP.restart();
    return;
  }
  if (line == "maze") {
    maze_debug_s();
    return;
  }
  if (line == "led" || line == "led cycle" || line == "led rotate") {
    if (!handleLedCommand("cycle")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "led off") {
    if (!handleLedCommand("off")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "led red") {
    if (!handleLedCommand("red")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "led green") {
    if (!handleLedCommand("green")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "led blue") {
    if (!handleLedCommand("blue")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "led yellow") {
    if (!handleLedCommand("yellow")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "led cyan" || line == "led bluegreen") {
    if (!handleLedCommand("cyan")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "led magenta") {
    if (!handleLedCommand("magenta")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "led white") {
    if (!handleLedCommand("white")) {
      debugPrintln("[CMD] led command failed");
    }
    return;
  }
  if (line == "test") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    testLoopMode = TEST_LOOP_STATUS;
    debugPrintln("[MODE] TEST loop=" + String(testLoopModeName(testLoopMode)));
    return;
  }
  if (line == "test off") {
    enterIdleMode("test off");
    return;
  }
  if (line.startsWith("test loop ")) {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;

    if (line == "test loop status") testLoopMode = TEST_LOOP_STATUS;
    else if (line == "test loop battery") testLoopMode = TEST_LOOP_BATTERY;
    else if (line == "test loop sensors") testLoopMode = TEST_LOOP_SENSORS;
    else if (line == "test loop sensorsraw") testLoopMode = TEST_LOOP_SENSORS_RAW;
    else if (line == "test loop encoders") testLoopMode = TEST_LOOP_ENCODERS;
    else if (line == "test loop maze") testLoopMode = TEST_LOOP_MAZE;
    else if (line == "test loop off") testLoopMode = TEST_LOOP_NONE;
    else {
      debugPrintln("[CMD] usage: test loop status|battery|sensors|sensorsraw|encoders|maze|off");
      return;
    }

    debugPrintln("[MODE] TEST loop=" + String(testLoopModeName(testLoopMode)));
    return;
  }
  if (line == "explore") {
    beginExplore(false, -1);
    return;
  }
  if (line.startsWith("explore ")) {
    int steps = 0;
    if (sscanf(line.c_str(), "explore %d", &steps) == 1 && steps > 0) {
      beginExplore(false, steps);
    } else {
      debugPrintln("[CMD] usage: explore [n>0]");
    }
    return;
  }
  if (line == "clearmaze" || line == "mazereset" || line == "resetstart") {
    clearMazeMemoryOnly();
    return;
  }
  if (line == "speedrun" || line.startsWith("speedrun ")) {
    if (!robotState.speedRunReady) {
      debugPrintln("[CMD] speed run not ready yet");
      return;
    }
    uint8_t phase = 1;
    if (line.startsWith("speedrun ")) {
      const String arg = line.substring(9);
      const int parsed = arg.toInt();
      if (parsed < 1 || parsed > 4) {
        debugPrintln("[CMD] usage: speedrun [1-4]");
        return;
      }
      phase = (uint8_t)parsed;
    }
    beginSpeedRun(phase);
    return;
  }
  if (line == "idle") {
    enterIdleMode("manual idle");
    return;
  }
  if (line == "stop") {
    motionController.stop();
    enterIdleMode("manual stop");
    return;
  }
  if (line == "brake") {
    leftMotor.hardStop();
    rightMotor.hardStop();
    robotState.motionStatus = MOTION_IDLE;
    robotState.activePrimitive = MOTION_NONE;
    robotState.pose.forwardProgressMm = 0.0f;
    robotState.pose.turnProgressDeg = 0.0f;
    debugPrintln("[CMD] brake stop");
    return;
  }
  if (line == "move" || line.startsWith("move ")) {
    int cells = 1;
    if (line.startsWith("move ")) {
      if (sscanf(line.c_str(), "move %d", &cells) != 1 || cells <= 0) {
        debugPrintln("[CMD] usage: move [n>0]");
        return;
      }
    }

    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    const uint8_t requestedCells = (uint8_t)min(cells, (int)AppConfig::Motion::CORRIDOR_MAX_CELLS);
    const bool ok = (requestedCells <= 1) ? motionController.moveOneCell() : motionController.moveCells(requestedCells);
    if (!ok) {
      debugPrintln("[CMD] failed to start move");
      return;
    }

    activeForwardActionCellsRequested = requestedCells;
    activeForwardActionCellsCommitted = 0;
    const MotionPrimitiveType primitive =
      (requestedCells <= 1) ? MOTION_MOVE_ONE_CELL : MOTION_MOVE_MULTI_CELL;
    debugMotionEvent("[MOTION START]", primitive, motionController.status(),
                     robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                     robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                     "source=manual cells=" + String((int)requestedCells));
    return;
  }
  if (line == "back") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    if (!motionController.moveBackwardShort()) {
      debugPrintln("[CMD] failed to start back");
    }
    return;
  }
  if (line == "testsnap") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    if (!motionController.snapCenter()) {
      debugPrintln("[CMD] failed to start testsnap");
    } else {
      debugMotionEvent("[MOTION START]", MOTION_SNAP_CENTER, motionController.status(),
                       robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                       robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                       "source=testsnap");
    }
    return;
  }
  if (line == "left") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    executePlannerAction(FloodFillExplorer::ACT_TURN_L);
    return;
  }
  if (line == "right") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    executePlannerAction(FloodFillExplorer::ACT_TURN_R);
    return;
  }
  if (line == "uturn") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    executePlannerAction(FloodFillExplorer::ACT_TURN_180);
    return;
  }
  if (line.startsWith("resetpose")) {
    int x, y, h;
    if (sscanf(line.c_str(), "resetpose %d %d %d", &x, &y, &h) == 3) {
      x = constrain(x, 0, 15);
      y = constrain(y, 0, 15);
      h &= 3;
      setPose((uint8_t)x, (uint8_t)y, (FloodFillExplorer::Dir)h);
      explorer.setStart(robotState.pose.cellX, robotState.pose.cellY, headingDir());
      applyCurrentPoseAsHomeRect();
      explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
      debugPrintln("[CMD] pose reset");
    } else {
      debugPrintln("[CMD] usage: resetpose x y h");
    }
    return;
  }
  if (line.startsWith("setgoal")) {
    int x, y, w, h;
    if (sscanf(line.c_str(), "setgoal %d %d %d %d", &x, &y, &w, &h) == 4) {
      x = constrain(x, 0, 15);
      y = constrain(y, 0, 15);
      w = constrain(w, 1, 16 - x);
      h = constrain(h, 1, 16 - y);
      runtimeGoalX0 = (uint8_t)x;
      runtimeGoalY0 = (uint8_t)y;
      runtimeGoalW = (uint8_t)w;
      runtimeGoalH = (uint8_t)h;
      applyRuntimeGoalRect();
      robotState.goalReached = false;
      robotState.speedRunReady = false;
      resetExploreLoopTracking();
      debugPrintln("[CMD] goal rect set to (" + String(x) + "," + String(y) +
                   ") size " + String(w) + "x" + String(h));
    } else {
      debugPrintln("[CMD] usage: setgoal x y w h");
    }
    return;
  }
  if (line == "test battery") {
    battery_debug_s();
    return;
  }
  if (line == "test sensors") {
    tof_debug_s();
    return;
  }
  if (line == "test sensorsraw") {
    tof_raw_debug_s();
    return;
  }
  if (line == "test motorl" || line.startsWith("test motorl ")) {
    float tps = 220.0f;
    if (line.startsWith("test motorl ")) {
      const String arg = line.substring(12);
      const float parsed = arg.toFloat();
      if (parsed == 0.0f) {
        debugPrintln("[CMD] usage: test motorl [tps!=0]");
        return;
      }
      tps = parsed;
    }
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    motorBothFlipTestEnabled = false;
    leftMotor.setSpeedTPS(tps);
    rightMotor.coastStop();
    debugPrintln("[TEST] left motor tps=" + String(tps, 1));
    return;
  }
  if (line == "test motorr" || line.startsWith("test motorr ")) {
    float tps = 220.0f;
    if (line.startsWith("test motorr ")) {
      const String arg = line.substring(12);
      const float parsed = arg.toFloat();
      if (parsed == 0.0f) {
        debugPrintln("[CMD] usage: test motorr [tps!=0]");
        return;
      }
      tps = parsed;
    }
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    motorBothFlipTestEnabled = false;
    leftMotor.coastStop();
    rightMotor.setSpeedTPS(tps);
    debugPrintln("[TEST] right motor tps=" + String(tps, 1));
    return;
  }
  if (line == "test motor both" || line.startsWith("test motor both ")) {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    if (line.startsWith("test motor both ")) {
      const String arg = line.substring(16);
      const float tps = arg.toFloat();
      if (tps == 0.0f) {
        debugPrintln("[CMD] usage: test motor both [tps!=0]");
        return;
      }
      motorBothFlipTestEnabled = false;
      leftMotor.setSpeedTPS(tps);
      rightMotor.setSpeedTPS(tps);
      debugPrintln("[TEST] both motors tps=" + String(tps, 1));
    } else {
      motorBothFlipTestEnabled = true;
      motorFlipPower = 1.0f;
      motorFlipLastToggleMs = millis();
      leftMotor.setPower(motorFlipPower);
      rightMotor.setPower(motorFlipPower);
      debugPrintln("[TEST] both motors flip loop +100%/-100% every 1s");
    }
    return;
  }
  if (line == "test encoders") {
    motor_debug_s();
    return;
  }

  debugPrintln("[CMD] unknown: " + rawLine);
}

static const char* modeName(RobotMode mode) {
  switch (mode) {
    case ROBOT_MODE_IDLE: return "idle";
    case ROBOT_MODE_MANUAL_TEST: return "manual";
    case ROBOT_MODE_EXPLORE: return "explore";
    case ROBOT_MODE_SPEED_RUN: return "speedrun";
    case ROBOT_MODE_FAULT: return "fault";
  }
  return "unknown";
}

static const char* batteryStateName(uint8_t state) {
  switch (state) {
    case Battery::BATTERY_OK: return "ok";
    case Battery::BATTERY_WARNING: return "warning";
    case Battery::BATTERY_CRITICAL: return "critical";
  }
  return "unknown";
}

static const char* motionStatusName(MotionStatus status) {
  switch (status) {
    case MOTION_IDLE: return "idle";
    case MOTION_RUNNING_PRIMITIVE: return "running";
    case MOTION_COMPLETED: return "completed";
    case MOTION_FAILED: return "failed";
    case MOTION_ABORTED: return "aborted";
  }
  return "unknown";
}

static const char* testLoopModeName(TestLoopMode mode) {
  switch (mode) {
    case TEST_LOOP_NONE: return "off";
    case TEST_LOOP_STATUS: return "status";
    case TEST_LOOP_BATTERY: return "battery";
    case TEST_LOOP_SENSORS: return "sensors";
    case TEST_LOOP_SENSORS_RAW: return "sensorsraw";
    case TEST_LOOP_ENCODERS: return "encoders";
    case TEST_LOOP_MAZE: return "maze";
  }
  return "unknown";
}

static void motor_debug_s() {
  char buf[128];
  snprintf(buf, sizeof(buf), "MOTOR | L ticks=%ld tps=%.1f | R ticks=%ld tps=%.1f",
           (long)leftMotor.getTicks(), leftMotor.getTicksPerSecond(),
           (long)rightMotor.getTicks(), rightMotor.getTicksPerSecond());
  debugPrintln(String(buf));
}

static void tof_debug_s() {
  String out;
  for (uint8_t i = 0; i < tofArray.sensorCount(); i++) {
    out += "S" + String(i) + "=" + String(tofArray.getDistance(i)) + " ";
  }
  out += "| wall L/F/R=" + String((int)robotState.walls.leftWall) + "/" +
         String((int)robotState.walls.frontWall) + "/" + String((int)robotState.walls.rightWall);
  out += " | valid=" + String((int)robotState.walls.leftValid) + "/" +
         String((int)robotState.walls.frontValid) + "/" + String((int)robotState.walls.rightValid);
  debugPrintln(out);
}

static void tof_raw_debug_s() {
  String out = "[TOF RAW] ";

  for (uint8_t i = 0; i < tofArray.sensorCount(); i++) {
    const uint16_t raw = tofArray.getRaw(i);
    out += "S" + String(i) + "=" + String(raw) + " ";
  }
  debugPrintln(out);
}

static void robot_debug_s() {
  const uint32_t now = millis();
  if (now - lastStatusMs < 200) return;
  lastStatusMs = now;

  char buf[256];
  if (AppConfig::Debug::PRINT_STATUS_TPS) {
    snprintf(buf, sizeof(buf),
             "MODE=%s motion=%s pose=(%u,%u,%u) batt=%.2fV(%s) tps=(%.1f,%.1f) walls=%d/%d/%d dist=%u/%u/%u fault=%s",
             modeName(robotState.mode),
             motionStatusName(robotState.motionStatus),
             robotState.pose.cellX,
             robotState.pose.cellY,
             robotState.pose.heading,
             robotState.batteryVoltage,
             batteryStateName(robotState.batteryState),
             robotState.leftTps,
             robotState.rightTps,
             robotState.walls.leftWall,
             robotState.walls.frontWall,
             robotState.walls.rightWall,
             robotState.walls.leftMm,
             robotState.walls.frontMm,
             robotState.walls.rightMm,
             robotState.lastFault.c_str());
  } else {
    snprintf(buf, sizeof(buf),
             "MODE=%s motion=%s pose=(%u,%u,%u) batt=%.2fV(%s) walls=%d/%d/%d dist=%u/%u/%u fault=%s",
             modeName(robotState.mode),
             motionStatusName(robotState.motionStatus),
             robotState.pose.cellX,
             robotState.pose.cellY,
             robotState.pose.heading,
             robotState.batteryVoltage,
             batteryStateName(robotState.batteryState),
             robotState.walls.leftWall,
             robotState.walls.frontWall,
             robotState.walls.rightWall,
             robotState.walls.leftMm,
             robotState.walls.frontMm,
             robotState.walls.rightMm,
             robotState.lastFault.c_str());
  }
  debugPrintln(String(buf));
}

static void battery_debug_s() {
  const uint16_t raw = mouseBattery.raw();
  const float adcVolts = mouseBattery.adcVoltage();
  const float estimatedBatteryFromDivider = mouseBattery.dividerEstimatedVoltage();

  char buf[160];
  snprintf(buf, sizeof(buf),
           "[BAT] raw=%u adc=%.3fV batt=%.2fV est=%.2fV pct=%.0f state=%s",
           raw,
           adcVolts,
           robotState.batteryVoltage,
           estimatedBatteryFromDivider,
           robotState.batteryPercent,
           batteryStateName(robotState.batteryState));
  debugPrintln(String(buf));
}

}  // namespace MainApp
