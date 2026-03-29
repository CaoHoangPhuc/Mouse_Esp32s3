#include <Arduino.h>
#include <WiFi.h>
#include <driver/gpio.h>

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
static void debugPrint(const String& s);
static void debugPrintln(const String& s = "");
static void debugPrompt();
static bool debugClientConnected();
static void updateOtaSafeMode();
static bool handleLedCommand(const String& cmd);
static bool onWebTelnetReconnect();
static String onWebHealthJson();
static void onExplorerWebCommand(const String& cmd);
static void updateRobotLed();
static void beginExplore(bool clearMaze, int32_t stepBudget = -1);
static void resetMazeToConfiguredStart();
static void clearMazeMemoryOnly();
static void applyCurrentPoseAsHomeRect();
static FloodFillExplorer::Dir headingDir();
static FloodFillExplorer::Dir oppositeDir(FloodFillExplorer::Dir dir);
static void maze_debug_s();
static void closeDebugConsole(const String& reason = "");
static bool startRunSnapSequence(const char* label);
static bool shouldSnapCenterAfterTurn();
static void resetExploreLoopTracking();
static void setPose(uint8_t x, uint8_t y, FloodFillExplorer::Dir h);

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

static void debugPrint(const String& s) {
  Serial.print(s);
  if (debugClientConnected()) {
    debugClient.print(s);
  }
}

static void debugPrintln(const String& s) {
  Serial.println(s);
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
    ledController.setWhite();
    return;
  }

  switch (robotState.mode) {
    case ROBOT_MODE_EXPLORE:
      if (exploreGoalSeen) ledController.setBlue();
      else ledController.setGreen();
      return;
    case ROBOT_MODE_FAULT:
      ledController.setRed();
      return;
    default:
      if (robotState.speedRunReady) {
        ledController.setWhite();
        return;
      }
      ledController.off();
      return;
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

static void enterIdleMode(const String& reason) {
  robotState.mode = ROBOT_MODE_IDLE;
  robotState.motionStatus = MOTION_IDLE;
  robotState.activePrimitive = MOTION_NONE;
  testLoopMode = TEST_LOOP_NONE;
  runStartSnapPending = false;
  deferPlannerAckUntilSnapCenter = false;
  runStartSnapMode = ROBOT_MODE_IDLE;
  explorer.setRunning(false);
  motionController.stop();
  updateRobotLed();
  debugPrintln("[MODE] IDLE: " + reason);
}

static void enterFaultMode(const String& reason) {
  robotState.mode = ROBOT_MODE_FAULT;
  robotState.lastFault = reason;
  robotState.faultCount++;
  runStartSnapPending = false;
  deferPlannerAckUntilSnapCenter = false;
  runStartSnapMode = ROBOT_MODE_IDLE;
  motionController.abort(reason);
  explorer.setRunning(false);
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
    debugPrintln("[OTA] App safe mode");
  } else {
    debugPrintln("[OTA] App resumed");
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
  }
}

static void beginExplore(bool clearMaze, int32_t stepBudget) {
  robotState.goalReached = false;
  robotState.speedRunReady = false;
  robotState.mode = ROBOT_MODE_EXPLORE;
  robotState.lastFault = "";
  resetExploreLoopTracking();
  exploreStepBudget = stepBudget;
  runStartSnapPending = false;
  deferPlannerAckUntilSnapCenter = false;
  runStartSnapMode = ROBOT_MODE_IDLE;
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

static void beginSpeedRun() {
  robotState.mode = ROBOT_MODE_SPEED_RUN;
  robotState.goalReached = false;
  robotState.lastFault = "";
  runStartSnapPending = false;
  deferPlannerAckUntilSnapCenter = false;
  runStartSnapMode = ROBOT_MODE_IDLE;
  explorer.setHardwareMode(true);
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  updateRobotLed();
  debugPrintln("[MODE] SPEED RUN");
  startRunSnapSequence("run-start snapcenter");
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
  bool ok = false;
  MotionPrimitiveType startedPrimitive = MOTION_NONE;
  switch (act) {
    case FloodFillExplorer::ACT_MOVE_F:
      ok = motionController.moveOneCell();
      startedPrimitive = MOTION_MOVE_ONE_CELL;
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

  debugMotionEvent("[MOTION START]", startedPrimitive, motionController.status(),
                   robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                   robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                   "action=" + String((int)act));

  return true;
}

static void advancePoseForFinishedPrimitive(MotionPrimitiveType primitive) {
  if (primitive == MOTION_MOVE_ONE_CELL) {
    switch (headingDir()) {
      case FloodFillExplorer::NORTH: if (robotState.pose.cellY > 0) robotState.pose.cellY--; break;
      case FloodFillExplorer::EAST:  if (robotState.pose.cellX < 15) robotState.pose.cellX++; break;
      case FloodFillExplorer::SOUTH: if (robotState.pose.cellY < 15) robotState.pose.cellY++; break;
      case FloodFillExplorer::WEST:  if (robotState.pose.cellX > 0) robotState.pose.cellX--; break;
    }
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
  bool skipPostMotionHold = false;
  bool stepBudgetReached = false;

  if (status == MOTION_COMPLETED) {
    advancePoseForFinishedPrimitive(primitive);
    debugMotionEvent("[MOTION END]", primitive, status,
                     beforeX, beforeY, beforeH,
                     robotState.pose.cellX, robotState.pose.cellY, headingDir());
    leftMotor.hardStop();
    rightMotor.hardStop();

    if (isTurnPrimitive &&
        !deferPlannerAckUntilSnapCenter &&
        (robotState.mode == ROBOT_MODE_EXPLORE || robotState.mode == ROBOT_MODE_SPEED_RUN) &&
        shouldSnapCenterAfterTurn()) {
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
    debugWallApplyEvent("[WALL APPLY]", "motion_complete");
    applyWallsToExplorer();
    debugWallApplyEvent("[WALL APPLIED]", "motion_complete");

    bool reachedGoal = false;
    if (deferPlannerAckUntilSnapCenter) {
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

    if (runStartSnapPending && isSnapCenterPrimitive) {
      runStartSnapPending = false;
      explorer.setRunning(runStartSnapMode == ROBOT_MODE_EXPLORE || runStartSnapMode == ROBOT_MODE_SPEED_RUN);
      runStartSnapMode = ROBOT_MODE_IDLE;
      debugPrintln("[SNAP] run-start snapcenter complete");
    }

    if (AppConfig::Motion::AUTO_PRINT_MAZE_AFTER_SENSE && robotState.mode == ROBOT_MODE_EXPLORE) {
      maze_debug_s();
    }

    if (primitive == MOTION_MOVE_ONE_CELL &&
        robotState.mode == ROBOT_MODE_EXPLORE &&
        exploreStepBudget > 0) {
      exploreStepBudget--;
      debugPrintln("[EXPLORE] steps remaining=" + String(exploreStepBudget));
      if (exploreStepBudget == 0) {
        stepBudgetReached = true;
      }
    }

    if (reachedGoal) {
      robotState.goalReached = true;
      updateRobotLed();
      debugPrintln("[GOAL] Goal reached");
      if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
        robotState.speedRunReady = true;
        enterIdleMode("speed run finished");
      } else if (robotState.mode == ROBOT_MODE_EXPLORE &&
                 AppConfig::Explorer::CONTINUE_AFTER_GOAL) {
        exploreGoalSeen = true;
        const uint16_t bestCost = explorer.bestKnownCostOriginalStartToGoal();
        if (explorer.isInOriginalGoal(robotState.pose.cellX, robotState.pose.cellY)) {
          reachedOriginalGoalOnThisLoop = true;
          debugPrintln("[EXPLORE] reached goal leg bestCost=" + String(bestCost));
        } else if (explorer.isInOriginalStart(robotState.pose.cellX, robotState.pose.cellY) &&
                   reachedOriginalGoalOnThisLoop) {
          reachedOriginalGoalOnThisLoop = false;
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
            enterIdleMode("shortest path known");
            return;
          }
        }

        explorer.setRunning(true);
        robotState.goalReached = false;
        skipPostMotionHold = true;
        updateRobotLed();
        debugPrintln("[EXPLORE] target toggled, continue exploring");
      }
    }

    if (stepBudgetReached) {
      enterIdleMode("explore step budget reached");
      return;
    }

    if (AppConfig::Motion::POST_MOTION_HARD_STOP_HOLD_MS > 0 &&
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
    explorer.ackPendingActionExternal(false,
      robotState.pose.cellX,
      robotState.pose.cellY,
      headingDir());
    enterFaultMode(motionController.lastError());
  }

  motionController.stop();
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

static bool shouldSnapCenterAfterTurn() {
  return shouldSnapCenterFromKnownBackWall();
}

static bool startRunSnapSequence(const char* label) {
  explorer.setRunning(false);
  if (!shouldSnapCenterFromKnownBackWall()) {
    explorer.setRunning(robotState.mode == ROBOT_MODE_EXPLORE || robotState.mode == ROBOT_MODE_SPEED_RUN);
    debugPrintln(String("[SNAP] skip ") + label + " (no known back wall)");
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
  Serial.begin(921600);
  vTaskDelay(pdMS_TO_TICKS(200));
  i2cRecover(AppConfig::I2C::SDA, AppConfig::I2C::SCL);
  setPose(AppConfig::Maze::START_X, AppConfig::Maze::START_Y, AppConfig::Maze::START_HEADING);
  ledController.begin();

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
  dbg.setLedCommandHandler(onWebLedCommand);
  dbg.setTelnetReconnectHandler(onWebTelnetReconnect);
  dbg.setHealthJsonProvider(onWebHealthJson);
  wifiOk = dbg.begin(wifiCfg);
  debugPrintln(wifiOk ? "Boot OK" : "Boot with WiFi failed");
  debugServer.begin();
  debugServer.setNoDelay(true);

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
  xTaskCreatePinnedToCore(userTaskFn,    "user",      6144, nullptr, 2, &userTaskHandle,      0);

  enterIdleMode("ready");
}

void loopApp() {
  vTaskDelay(1);
}

void userTaskBody(void* arg) {
  (void)arg;
  String line;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20);

  for (;;) {
    updateOtaSafeMode();
    if (dbg.isUpdateInProgress()) {
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    updateRobotState();
    motionController.update(robotState);
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
  const TickType_t period = pdMS_TO_TICKS(50);

  for (;;) {
    if (dbg.isUpdateInProgress()) {
      vTaskDelayUntil(&last, period);
      continue;
    }

    const MotionStatus motionStatus = motionController.status();
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
        debugWallApplyEvent("[WALL APPLY]", "planner_idle");
        applyWallsToExplorer();
        debugWallApplyEvent("[WALL APPLIED]", "planner_idle");
        explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
        FloodFillExplorer::Action act = explorer.requestNextAction();
        if (act == FloodFillExplorer::ACT_NONE) {
          if (robotState.mode == ROBOT_MODE_EXPLORE && explorer.atGoal()) {
            robotState.goalReached = true;
            robotState.speedRunReady = true;
            updateRobotLed();
            enterIdleMode("explore finished");
          } else if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
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
  const TickType_t period = pdMS_TO_TICKS(5);

  for (;;) {
    if (dbg.isUpdateInProgress()) {
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
  for (;;) {
    if (dbg.isUpdateInProgress()) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    explorer.loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void tofTask(void* arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5);

  for (;;) {
    if (dbg.isUpdateInProgress()) {
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
  const TickType_t period = pdMS_TO_TICKS(1000);

  for (;;) {
    if (dbg.isUpdateInProgress()) {
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
  debugPrintln(String("[BOOT] TCP Console: ") + WiFi.localIP().toString() + ":" + String(AppConfig::Wifi::DEBUG_TCP_PORT));
  debugPrintln("[CMD] help | explore [n] | speedrun | idle | stop | brake | restart | move | back | left | right | uturn | testsnap | status | resetpose x y h | setgoal x y w h | clearmaze");
  debugPrintln("[CMD] led cycle|rotate|off|red|green|blue|cyan|white");
  debugPrintln("[CMD] maze");
  debugPrintln("[CMD] test | test off | test loop status|battery|sensors|sensorsraw|encoders|maze|off");
  debugPrintln("[CMD] test battery|sensors|sensorsraw|motorl|motorr|encoders");
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
  if (line == "led cyan" || line == "led bluegreen") {
    if (!handleLedCommand("cyan")) {
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
    beginExplore(true, -1);
    return;
  }
  if (line.startsWith("explore ")) {
    int steps = 0;
    if (sscanf(line.c_str(), "explore %d", &steps) == 1 && steps > 0) {
      beginExplore(true, steps);
    } else {
      debugPrintln("[CMD] usage: explore [n>0]");
    }
    return;
  }
  if (line == "clearmaze" || line == "mazereset" || line == "resetstart") {
    clearMazeMemoryOnly();
    return;
  }
  if (line == "speedrun") {
    if (!robotState.speedRunReady) {
      debugPrintln("[CMD] speed run not ready yet");
      return;
    }
    beginSpeedRun();
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
  if (line == "move") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    executePlannerAction(FloodFillExplorer::ACT_MOVE_F);
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
  if (line == "test motorl") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    leftMotor.setSpeedTPS(220.0f);
    rightMotor.coastStop();
    debugPrintln("[TEST] left motor spin");
    return;
  }
  if (line == "test motorr") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    leftMotor.coastStop();
    rightMotor.setSpeedTPS(220.0f);
    debugPrintln("[TEST] right motor spin");
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
