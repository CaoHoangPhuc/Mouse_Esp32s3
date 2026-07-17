# NotebookLM Project Export

Generated from the Arduino project at "C:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3" on 2026-04-26 01:07:25 +07:00.

This bundle is intended to be imported as a single source into NotebookLM. It includes the project tree and the full contents of the selected text/code files.

Included file count: 37

Excluded directories: `.git`, `logs`

## Project Tree

```text
.gitignore
AGENT.md
AppRuntime.cpp
AppRuntime.h
ARCHITECTURE_SCAN.md
Battery.cpp
Battery.h
Config.cpp
Config.h
DcMotor.cpp
DcMotor.h
esp32s3.txt
FloodFillExplorer.cpp
FloodFillExplorer.h
HARDWARE.md
LedController.cpp
LedController.h
MotionController.cpp
MotionController.h
Mouse_esp32s3.ino
MultiVL53L0X.cpp
MultiVL53L0X.h
PersistenceStore.cpp
PersistenceStore.h
README.md
RELEASE_0.3.0.md
RobotTypes.h
scripts/
  build.ps1
  build-and-upload.ps1
  export-notebooklm.ps1
  upload.ps1
SECURITY_FLASHING.md
SKILL.md
TODO.md
UPLOAD_INSTRUCTIONS.md
WiFiOtaWebSerial.cpp
WiFiOtaWebSerial.h
```

## Full File Contents

### `.gitignore`

````
# Arduino build artifacts
*.o
*.obj
*.elf
*.bin
*.map
*.hex
*.eep
*.a
*.so
*.dll
*.dylib

# Arduino IDE / CLI temporary files
build/
dist/
.tmp/
*.tmp

# OS/editor files
.DS_Store
Thumbs.db
.vscode/
````

### `AGENT.md`

````md
# AGENT.md

## Purpose

This repository is an ESP32-S3 micromouse project that is being built in layers:
1. hardware safety and telemetry
2. reliable single-cell motion
3. wall sensing and maze updates
4. floodfill exploration
5. conservative second-run optimization

When changing this code, prefer reliability and observability over aggressive optimization.

This repository is also evolving into an education-friendly robotics platform:
- teachable architecture over clever shortcuts
- reusable modules over one-off logic
- clear interfaces and documentation so students can learn and extend safely

## Coding Priorities

- Do not break hardware safety for convenience.
- Keep motion gating tied to fault handling; battery is telemetry-only in the current code unless deliberately changed again.
- Preserve the planner/executor split:
  - planner chooses actions
  - motion controller executes physical movement
  - pose updates only after motion completion
- Treat the web explorer as a debug instrument, not the source of physical truth.
- Prefer simple discrete-cell behaviors before adding faster multi-cell behaviors.
- Keep module APIs stable when possible; if a contract changes, update docs and call sites together.
- Prefer composable helpers and config-driven behavior so features can be reused in labs/demos.

## Education Platform Direction

We are developing this codebase as a reusable framework for learning embedded robotics.

### Framework goals

- Keep a clean separation between:
  - hardware drivers
  - control/motion primitives
  - planner logic
  - runtime orchestration and UI/debug
- Make each layer independently testable and understandable.
- Avoid hidden coupling between modules.

### Reusability rules

- Add new features behind existing module boundaries first; avoid bypassing `AppRuntime` orchestration.
- Put tunables and hardware variants in `Config.h/.cpp`, not scattered constants.
- Prefer small, explicit data contracts (`struct`/enums) over implicit side effects.
- Keep fallback paths for teaching and bring-up (simple mode should remain available).

### Documentation-first expectation

- For each non-trivial feature, include:
  - what it is
  - where it lives
  - how to tune/use it
  - failure/safety behavior
- When behavior changes, update `README.md`, `SKILL.md`, and `TODO.md` in the same change.

## Important Files

- `Mouse_esp32s3.ino`: thin Arduino entrypoint only
- `AppRuntime.h`: app interface between Arduino entrypoints and the main application module
- `AppRuntime.cpp`: runtime orchestration, serial command surface, startup flow, background task bodies, planner integration
- `Config.h`: centralized hardware and tuning configuration
- `RobotTypes.h`: shared state and enums
- `MotionController.*`: motion primitive executor
- `MultiVL53L0X.*`: wall observation generation
- `FloodFillExplorer.*`: floodfill map/planner and web debug UI
- `Battery.*`: battery monitor and thresholds

## Current System Contract

- `RobotState` is the shared snapshot for telemetry and coordination.
- `MotionController` owns primitive execution and timeout/stall logic.
- `FloodFillExplorer` owns maze memory and next-action choice.
- `AppRuntime.cpp` wires sensor data into the explorer and acknowledges actions only after physical motion completes and wall sensing for the new pose has been applied.
- `Mouse_esp32s3.ino` should stay minimal and only expose Arduino entrypoints plus task wrappers.
- `Config.h` is the single source of truth for pins, calibration constants, and runtime tuning defaults.

## Safe Change Guidelines

- If changing wheel geometry, update both tuning docs and `AppConfig::Motion` values.
- If changing TOF mapping, keep `left/front/right` semantics stable for the planner.
- If adding a hardware workaround or config switch, update the related `.md` files in the same change.
- If changing floodfill action names or behavior, keep the planner output limited to turn-left, turn-right, and move-forward unless the executor is extended too.
- If adding faster movement, do not remove the current primitive path; keep it as a reliable fallback.
- If adjusting battery thresholds, note the measured pack voltage and calibration source in docs or commit message.
- Keep `Mouse_esp32s3.ino` small; do not move general application logic back into it.

## Validation Expectations

Before calling a change done, try to validate these in order:
- build compiles for the target ESP32-S3 board using the documented Arduino IDE settings
- battery telemetry still reads sensible values
- sensor telemetry still reports stable left/front/right walls
- motor direction and encoder signs remain correct
- one-cell move and 90-degree turns still complete
- planner can still step through a small maze without faulting

## Repository Documentation

Keep these files current when architecture or behavior changes:
- `README.md`: public project overview and usage
- `TODO.md`: active implementation backlog
- `SKILL.md`: operator playbook / build-and-tune workflow
- `AGENT.md`: repository guidance for maintainers and coding agents

Rule:
- whenever a code change affects setup, commands, tuning, hardware behavior, or operator workflow, update the related `.md` documents in the same change instead of leaving docs as follow-up work
- whenever a code change changes behavior or tuning, bump the documented project version, then stage and commit that versioned change together
- whenever a command is added, removed, or its usage changes, update both the serial/web help text and the related documentation in the same change
- whenever any repository change is completed, stage and commit that change in the same working session unless the user explicitly says not to commit
````

### `AppRuntime.cpp`

````cpp
#include <Arduino.h>
#include <WiFi.h>
#include "driver/gptimer.h"

#include "Battery.h"
#include "Config.h"
#include "DcMotor.h"
#include "FloodFillExplorer.h"
#include "LedController.h"
#include "MotionController.h"
#include "MultiVL53L0X.h"
#include "PersistenceStore.h"
#include "RobotTypes.h"
#if !APP_LITE_FIRMWARE
#include "WiFiOtaWebSerial.h"
#endif
#include "AppRuntime.h"

namespace MainApp {

#if APP_LITE_FIRMWARE
class LiteDebugBackend {
public:
  bool isUpdateInProgress() const { return false; }
};
LiteDebugBackend dbg;
#else
WiFiOtaWebSerial dbg;
#endif
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
enum CenterTrackTestMode : uint8_t {
  CENTER_TEST_OFF = 0,
  CENTER_TEST_LEFT,
  CENTER_TEST_RIGHT,
  CENTER_TEST_DUAL
};
static CenterTrackTestMode centerTrackTestMode = CENTER_TEST_OFF;
static float centerTrackTestBaseTps = 220.0f;
static float centerTrackTestLastCorrection = 0.0f;
static uint32_t centerTrackTestLastPrintMs = 0;
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
static gptimer_handle_t motorLoopTimer = nullptr;
static gptimer_handle_t tofLoopTimer = nullptr;
static bool motorLoopTimerActive = false;
static bool tofLoopTimerActive = false;

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
static void serviceCenterTrackTest();
static void serviceBootButtonLauncher();
static bool readBootButtonPressed();
static void dispatchBootButtonAction(uint8_t pressCount);
static void debugPrint(const String& s);
static void debugPrintln(const String& s = "");
static void serviceCrossCoreDebugLogs();
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
static void applyStopModeToDriveMotors(MotionController::StopMode mode);
static const char* centerTrackTestModeName(CenterTrackTestMode mode);
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
static void maze_debug_s(bool force = false);
static void resetCommonModeState();
static void closeDebugConsole(const String& reason = "");
static bool shouldSnapCenterFromKnownBackWall();
static bool startRunSnapSequence(const char* label);
static bool startSpeedRunPreAlignSequence();
static void finishSpeedRunStart();
static void advancePoseForwardOneCell();
static bool commitForwardActionCell(bool allowFrontStopCompletion, bool& stepBudgetReached, bool& reachedGoal);
static bool handleReachedGoal();
static void clearForwardActionTracking();
static bool shouldStopCorridorAfterCommittedCell();
static bool queueModeEnabledForCurrentMode();
static bool queueIsActive();
static void clearPlannerQueue();
static bool enqueueExplorePlannerAction();
static bool buildAndEnqueueSpeedRunQueue();
static bool startNextQueuedAction();
static bool queueSuppressWallRegistration();
static const char* queueItemName(MotionPrimitiveType primitive);
static String plannerQueueSequenceString(const FloodFillExplorer::QueuedAction* actions, uint16_t count);
static void debugPrintSpeedRunQueuePreview(const char* tag);
static void servicePendingPersistence();
static void schedulePostTurnWallSettle(MotionPrimitiveType primitive);
static bool isPostTurnWallSettleReady();
static void queueTrace(const char* tag);
static void resetExploreLoopTracking();
static void setPose(uint8_t x, uint8_t y, FloodFillExplorer::Dir h);
static void resetLapHistory();
static void startLapTimer(const char* legLabel);
static void stopLapTimer(bool record);
static String explorerLapStateJson();
static void resetLoopWatchdogState(struct LoopWatchdogState& state);
static void serviceLoopWatchdog(struct LoopWatchdogState& state, uint32_t expectedMs);
static bool onRealtimeLoopTimer(gptimer_handle_t timer,
                                const gptimer_alarm_event_data_t* edata,
                                void* user_ctx);
static bool startRealtimeLoopTimer(gptimer_handle_t* outTimer,
                                   TaskHandle_t targetTask,
                                   uint32_t periodMs,
                                   const char* tag);

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
struct PlannerQueueItem {
  MotionPrimitiveType primitive = MOTION_NONE;
  uint8_t forwardCells = 1;
  bool endsAtKnownWall = false;
};
static constexpr uint16_t kPlannerQueueStorageCapacity = 256;
static PlannerQueueItem plannerQueueBuffer[kPlannerQueueStorageCapacity];
static uint16_t plannerQueueHead = 0;
static uint16_t plannerQueueTail = 0;
static uint16_t plannerQueueCount = 0;
static uint16_t plannerQueueTotalBuilt = 0;
static bool plannerQueueItemInFlight = false;
static PlannerQueueItem plannerQueueInFlightItem{};
static bool speedRunQueueNeedsBuild = false;
static bool speedRunGoalSnapPending = false;
static bool queueTraceEnabled = AppConfig::Debug::QUEUE_TRACE_DEFAULT;
static bool mazeSavePending = false;
static bool postTurnWallSettlePending = false;
static uint32_t postTurnWallSettleUntilMs = 0;
static uint8_t postTurnWallStableCount = 0;
static uint8_t postTurnWallLastSignature = 0xFF;
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
static constexpr uint16_t kCrossCoreLogQueueDepth = 64;
static constexpr size_t kCrossCoreLogLineMax = 192;
struct CrossCoreLogEntry {
  char text[kCrossCoreLogLineMax];
  bool newline;
};
static CrossCoreLogEntry crossCoreLogQueue[kCrossCoreLogQueueDepth];
static uint16_t crossCoreLogHead = 0;
static uint16_t crossCoreLogTail = 0;
static uint16_t crossCoreLogCount = 0;
static uint32_t crossCoreLogDropped = 0;
static portMUX_TYPE crossCoreLogMux = portMUX_INITIALIZER_UNLOCKED;
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

static void logMotorPidToTelnet(const String& s) {
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

static bool IRAM_ATTR onRealtimeLoopTimer(gptimer_handle_t timer,
                                          const gptimer_alarm_event_data_t* edata,
                                          void* user_ctx) {
  (void)timer;
  (void)edata;
  TaskHandle_t task = static_cast<TaskHandle_t>(user_ctx);
  BaseType_t woke = pdFALSE;
  if (task != nullptr) {
    vTaskNotifyGiveFromISR(task, &woke);
  }
  return woke == pdTRUE;
}

static bool startRealtimeLoopTimer(gptimer_handle_t* outTimer,
                                   TaskHandle_t targetTask,
                                   uint32_t periodMs,
                                   const char* tag) {
  if (outTimer == nullptr || targetTask == nullptr || periodMs == 0) {
    return false;
  }

  const uint64_t periodUs = (uint64_t)periodMs * 1000ULL;
  gptimer_config_t timerCfg = {};
  timerCfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timerCfg.direction = GPTIMER_COUNT_UP;
  timerCfg.resolution_hz = 1000000;  // 1 tick = 1us

  gptimer_handle_t timer = nullptr;
  esp_err_t err = gptimer_new_timer(&timerCfg, &timer);
  if (err != ESP_OK || timer == nullptr) {
    debugPrintln(String("[RT] timer create failed: ") + tag + " err=" + String((int)err));
    return false;
  }

  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = onRealtimeLoopTimer;
  err = gptimer_register_event_callbacks(timer, &cbs, targetTask);
  if (err != ESP_OK) {
    debugPrintln(String("[RT] timer callback failed: ") + tag + " err=" + String((int)err));
    gptimer_del_timer(timer);
    return false;
  }

  gptimer_alarm_config_t alarmCfg = {};
  alarmCfg.reload_count = 0;
  alarmCfg.alarm_count = periodUs;
  alarmCfg.flags.auto_reload_on_alarm = true;
  err = gptimer_set_alarm_action(timer, &alarmCfg);
  if (err != ESP_OK) {
    debugPrintln(String("[RT] timer alarm failed: ") + tag + " err=" + String((int)err));
    gptimer_del_timer(timer);
    return false;
  }

  err = gptimer_enable(timer);
  if (err == ESP_OK) err = gptimer_start(timer);
  if (err != ESP_OK) {
    debugPrintln(String("[RT] timer start failed: ") + tag + " err=" + String((int)err));
    gptimer_del_timer(timer);
    return false;
  }

  *outTimer = timer;
  debugPrintln(String("[RT] timer started: ") + tag + " period=" + String(periodMs) + "ms");
  return true;
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

static void emitDebugNow(const char* s, bool newline) {
  if (serialOutputEnabled()) {
    if (newline) {
      Serial.println(s);
    } else {
      Serial.print(s);
    }
  }
  if (debugClientConnected()) {
    if (newline) {
      debugClient.println(s);
    } else {
      debugClient.print(s);
    }
  }
}

static void enqueueCrossCoreDebugLog(const String& s, bool newline) {
  char text[kCrossCoreLogLineMax];
  s.toCharArray(text, sizeof(text));

  portENTER_CRITICAL(&crossCoreLogMux);
  if (crossCoreLogCount >= kCrossCoreLogQueueDepth) {
    crossCoreLogTail = (uint16_t)((crossCoreLogTail + 1U) % kCrossCoreLogQueueDepth);
    crossCoreLogCount--;
    crossCoreLogDropped++;
  }
  CrossCoreLogEntry& slot = crossCoreLogQueue[crossCoreLogHead];
  strncpy(slot.text, text, sizeof(slot.text));
  slot.text[sizeof(slot.text) - 1] = '\0';
  slot.newline = newline;
  crossCoreLogHead = (uint16_t)((crossCoreLogHead + 1U) % kCrossCoreLogQueueDepth);
  crossCoreLogCount++;
  portEXIT_CRITICAL(&crossCoreLogMux);
}

static void serviceCrossCoreDebugLogs() {
  if (xPortGetCoreID() != 0) return;

  for (;;) {
    CrossCoreLogEntry item{};
    bool hasItem = false;
    uint32_t dropped = 0;
    bool emitDrop = false;

    portENTER_CRITICAL(&crossCoreLogMux);
    if (crossCoreLogCount > 0) {
      item = crossCoreLogQueue[crossCoreLogTail];
      crossCoreLogTail = (uint16_t)((crossCoreLogTail + 1U) % kCrossCoreLogQueueDepth);
      crossCoreLogCount--;
      hasItem = true;
    } else if (crossCoreLogDropped > 0) {
      dropped = crossCoreLogDropped;
      crossCoreLogDropped = 0;
      emitDrop = true;
    }
    portEXIT_CRITICAL(&crossCoreLogMux);

    if (hasItem) {
      emitDebugNow(item.text, item.newline);
      continue;
    }
    if (emitDrop) {
      char buf[72];
      snprintf(buf, sizeof(buf), "[DBGQ] dropped %lu messages", (unsigned long)dropped);
      emitDebugNow(buf, true);
      continue;
    }
    break;
  }
}

static void debugPrint(const String& s) {
  if (xPortGetCoreID() != 0) {
    enqueueCrossCoreDebugLog(s, false);
    return;
  }
  serviceCrossCoreDebugLogs();
  emitDebugNow(s.c_str(), false);
}

static void debugPrintln(const String& s) {
  if (xPortGetCoreID() != 0) {
    enqueueCrossCoreDebugLog(s, true);
    return;
  }
  serviceCrossCoreDebugLogs();
  emitDebugNow(s.c_str(), true);
}

static void debugPrompt() {
  if (debugClientConnected()) {
    debugClient.print("> ");
  }
}

static void applyStopModeToDriveMotors(MotionController::StopMode mode) {
  switch (mode) {
    case MotionController::StopMode::COAST:
      leftMotor.coastStop();
      rightMotor.coastStop();
      break;
    case MotionController::StopMode::BRAKE:
      leftMotor.enableSpeedControl(false);
      rightMotor.enableSpeedControl(false);
      leftMotor.brakeStop();
      rightMotor.brakeStop();
      break;
    case MotionController::StopMode::HARDSTOP:
    default:
      leftMotor.hardStop();
      rightMotor.hardStop();
      break;
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
  if (!AppConfig::Debug::DEBUG_MOTION_EVENT) return;
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

static const char* centerTrackTestModeName(CenterTrackTestMode mode) {
  switch (mode) {
    case CENTER_TEST_LEFT: return "left";
    case CENTER_TEST_RIGHT: return "right";
    case CENTER_TEST_DUAL: return "dual";
    case CENTER_TEST_OFF:
    default: return "off";
  }
}

static const char* queueItemName(MotionPrimitiveType primitive) {
  return primitiveName(primitive);
}

static bool queueModeEnabledForCurrentMode() {
  if (robotState.mode == ROBOT_MODE_EXPLORE) {
    return AppConfig::Explorer::QUEUE_ENABLE_EXPLORE;
  }
  if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
    return AppConfig::Explorer::QUEUE_ENABLE_SPEEDRUN1 &&
           robotState.speedRunPhase == 1;
  }
  return false;
}

static uint16_t plannerQueueCapacity() {
  const uint16_t cfgCap = AppConfig::Explorer::QUEUE_CAPACITY;
  if (cfgCap == 0) return 1;
  return (cfgCap < kPlannerQueueStorageCapacity) ? cfgCap : kPlannerQueueStorageCapacity;
}

static void clearPlannerQueue() {
  plannerQueueHead = 0;
  plannerQueueTail = 0;
  plannerQueueCount = 0;
  plannerQueueTotalBuilt = 0;
  plannerQueueItemInFlight = false;
  plannerQueueInFlightItem = PlannerQueueItem{};
  speedRunQueueNeedsBuild = false;
  speedRunGoalSnapPending = false;
}

static bool plannerQueuePush(const PlannerQueueItem& item) {
  const uint16_t cap = plannerQueueCapacity();
  if (plannerQueueCount >= cap) return false;
  plannerQueueBuffer[plannerQueueTail] = item;
  plannerQueueTail = (uint16_t)((plannerQueueTail + 1U) % cap);
  plannerQueueCount++;
  return true;
}

static bool plannerQueuePeek(PlannerQueueItem& item) {
  if (plannerQueueCount == 0) return false;
  item = plannerQueueBuffer[plannerQueueHead];
  return true;
}

static bool plannerQueuePop() {
  if (plannerQueueCount == 0) return false;
  const uint16_t cap = plannerQueueCapacity();
  plannerQueueHead = (uint16_t)((plannerQueueHead + 1U) % cap);
  plannerQueueCount--;
  return true;
}

static bool queueIsActive() {
  return queueModeEnabledForCurrentMode() &&
         (plannerQueueCount > 0 || plannerQueueItemInFlight || speedRunGoalSnapPending);
}

static bool queueSuppressWallRegistration() {
  if (!AppConfig::Explorer::QUEUE_DISABLE_WALL_REGISTER_WHILE_ACTIVE) return false;
  // Keep explore mapping stable; suppress map writes only during speedrun queue execution.
  return queueIsActive() &&
         robotState.mode == ROBOT_MODE_SPEED_RUN &&
         robotState.speedRunPhase == 1;
}

static String plannerQueueSequenceString(const FloodFillExplorer::QueuedAction* actions, uint16_t count) {
  String out;
  for (uint16_t i = 0; i < count; ++i) {
    if (i > 0) out += ",";
    const auto& a = actions[i];
    if (a.action == FloodFillExplorer::ACT_MOVE_F) {
      out += "M";
      out += String(a.forwardCells > 0 ? a.forwardCells : 1);
    } else if (a.action == FloodFillExplorer::ACT_TURN_L) {
      out += "L";
    } else if (a.action == FloodFillExplorer::ACT_TURN_R) {
      out += "R";
    } else if (a.action == FloodFillExplorer::ACT_TURN_180) {
      out += "U";
    } else {
      out += "?";
    }
  }
  return out;
}

static void queueTrace(const char* tag) {
  if (!queueTraceEnabled) return;
  const MotionPrimitiveType prim = motionController.primitive();
  const MotionStatus st = motionController.status();
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  const bool atGoal = explorer.atGoal();

  char buf[320];
  snprintf(buf, sizeof(buf),
           "[QTRACE] %s mode=%s pose=(%u,%u,%u) prog=%.1f req=%u com=%u prim=%s st=%s busy=%d q=%u in=%d tot=%u build=%d snap=%d atGoal=%d",
           tag ? tag : "",
           modeName(robotState.mode),
           robotState.pose.cellX, robotState.pose.cellY, robotState.pose.heading,
           robotState.pose.forwardProgressMm,
           (unsigned)activeForwardActionCellsRequested,
           (unsigned)activeForwardActionCellsCommitted,
           primitiveName(prim),
           motionStatusName(st),
           motionController.isBusy() ? 1 : 0,
           (unsigned)plannerQueueCount,
           plannerQueueItemInFlight ? 1 : 0,
           (unsigned)plannerQueueTotalBuilt,
           speedRunQueueNeedsBuild ? 1 : 0,
           speedRunGoalSnapPending ? 1 : 0,
           atGoal ? 1 : 0);
  debugPrintln(String(buf));
}

static void debugPrintSpeedRunQueuePreview(const char* tag) {
  FloodFillExplorer::QueuedAction actions[kPlannerQueueStorageCapacity];
  uint16_t count = 0;
  if (!explorer.buildQueuedActionsFromCurrentPose(actions, plannerQueueCapacity(), count)) {
    debugPrintln(String(tag) + " queue build failed");
    return;
  }
  debugPrintln(String(tag) + " queue count=" + String(count));
  debugPrintln(String(tag) + " queue seq=" + plannerQueueSequenceString(actions, count));
}

static void serviceCenterTrackTest() {
  if (centerTrackTestMode == CENTER_TEST_OFF) return;
  if (robotState.mode != ROBOT_MODE_MANUAL_TEST) return;
  if (motionController.isBusy()) return;
  if (motorBothFlipTestEnabled) return;

  MultiVL53L0X::StraightTrackMode mode = MultiVL53L0X::TRACK_NONE;
  switch (centerTrackTestMode) {
    case CENTER_TEST_LEFT: mode = MultiVL53L0X::TRACK_LEFT; break;
    case CENTER_TEST_RIGHT: mode = MultiVL53L0X::TRACK_RIGHT; break;
    case CENTER_TEST_DUAL: mode = MultiVL53L0X::TRACK_DUAL; break;
    case CENTER_TEST_OFF:
    default: mode = MultiVL53L0X::TRACK_NONE; break;
  }

  tofArray.setStraightTrackMode(mode);
  const float correction = tofArray.computeError(0.0f) * AppConfig::Motion::CENTERING_GAIN;
  centerTrackTestLastCorrection = correction;

  const float slowSideGain = max(1.0f, AppConfig::Motion::CENTERING_SLOW_SIDE_GAIN);
  const float fastSideGain = max(0.0f, AppConfig::Motion::CENTERING_FAST_SIDE_GAIN);
  float lCmd = centerTrackTestBaseTps;
  float rCmd = centerTrackTestBaseTps;
  if (correction > 0.0f) {
    lCmd = centerTrackTestBaseTps + correction * fastSideGain;
    rCmd = centerTrackTestBaseTps - correction * slowSideGain;
  } else if (correction < 0.0f) {
    rCmd = centerTrackTestBaseTps - correction * fastSideGain;
    lCmd = centerTrackTestBaseTps + correction * slowSideGain;
  }
  leftMotor.setSpeedTPS(lCmd);
  rightMotor.setSpeedTPS(rCmd);

  const uint32_t now = millis();
  if ((uint32_t)(now - centerTrackTestLastPrintMs) >= 250) {
    centerTrackTestLastPrintMs = now;
    MultiVL53L0X::SensorState ss = tofArray.getSensorState();
    char buf[260];
    snprintf(buf, sizeof(buf),
             "[TEST CENTER] mode=%s base=%.1f corr=%.2f cmdL=%.1f cmdR=%.1f tpsL=%.1f tpsR=%.1f dist L/F/R=%u/%u/%u valid=%u/%u/%u",
             centerTrackTestModeName(centerTrackTestMode), centerTrackTestBaseTps,
             centerTrackTestLastCorrection, lCmd, rCmd,
             leftMotor.getTicksPerSecond(), rightMotor.getTicksPerSecond(),
             (unsigned)ss.leftMm, (unsigned)ss.frontMm, (unsigned)ss.rightMm,
             ss.leftValid ? 1 : 0, ss.frontValid ? 1 : 0, ss.rightValid ? 1 : 0);
    debugPrintln(String(buf));
  }
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

static void maze_debug_s(bool force) {
  if (!force && !AppConfig::Debug::DEBUG_MAZE_PRINT) return;
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
  postTurnWallSettlePending = false;
  postTurnWallSettleUntilMs = 0;
  postTurnWallStableCount = 0;
  postTurnWallLastSignature = 0xFF;
  motorBothFlipTestEnabled = false;
  centerTrackTestMode = CENTER_TEST_OFF;
  centerTrackTestLastCorrection = 0.0f;
  tofArray.setStraightTrackMode(MultiVL53L0X::TRACK_NONE);
  explorer.setRunning(false);
  clearPlannerQueue();
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
    postTurnWallSettlePending = false;
    postTurnWallSettleUntilMs = 0;
    postTurnWallStableCount = 0;
    postTurnWallLastSignature = 0xFF;
    clearPlannerQueue();
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
    return;
  }
  if (cmd == "srun 1" || cmd == "srun1") {
    if (!robotState.speedRunReady) {
      debugPrintln("[SPEEDRUN] not ready, explore until shortest path is known");
      return;
    }
    beginSpeedRun(1);
    return;
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
  if (AppConfig::Explorer::QUEUE_ENABLE_EXPLORE) {
    (void)explorer.requestNextActionNoAck();  // clear pending ACK state for queue mode
  }
  updateRobotLed();
  clearPlannerQueue();
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
  explorer.setHomeRect(AppConfig::Maze::HOME_X0, AppConfig::Maze::HOME_Y0,
                       AppConfig::Maze::HOME_W, AppConfig::Maze::HOME_H);
  applyRuntimeGoalRect();
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  explorer.setRunning(false);
  clearPlannerQueue();
  speedRunQueueNeedsBuild = AppConfig::Explorer::QUEUE_ENABLE_SPEEDRUN1 && phase == 1;
  updateRobotLed();
  debugPrintln("[MODE] SPEED RUN " + String((int)phase));
  if (phase == 1) {
    debugPrintln("[SPEEDRUN] phase 1 is round-trip shortest-path run (home->goal->home)");
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

static bool enqueueExplorePlannerAction() {
  if (plannerQueueCount > 0 || plannerQueueItemInFlight) return true;

  debugWallApplyEvent("[WALL APPLY]", "queue_explore_idle");
  applyWallsToExplorer();
  debugWallApplyEvent("[WALL APPLIED]", "queue_explore_idle");

  PlannerQueueItem item;
  item.primitive = MOTION_NONE;
  item.forwardCells = 1;
  item.endsAtKnownWall = false;
  const FloodFillExplorer::Action act = explorer.requestNextActionNoAck();
  if (act == FloodFillExplorer::ACT_NONE) {
    if (!explorer.atGoal()) {
      enterFaultMode("explore queue has no action (dead-end known map)");
      return false;
    }
    return true;
  }
  if (act == FloodFillExplorer::ACT_MOVE_F) {
    item.forwardCells = explorer.lastActionForwardCells();
    if (item.forwardCells == 0) item.forwardCells = 1;
    item.endsAtKnownWall = explorer.lastActionEndsAtKnownWall();
    item.primitive = (item.forwardCells <= 1) ? MOTION_MOVE_ONE_CELL : MOTION_MOVE_MULTI_CELL;
  } else if (act == FloodFillExplorer::ACT_TURN_L) {
    item.primitive = MOTION_TURN_LEFT_90;
  } else if (act == FloodFillExplorer::ACT_TURN_R) {
    item.primitive = MOTION_TURN_RIGHT_90;
  } else if (act == FloodFillExplorer::ACT_TURN_180) {
    item.primitive = MOTION_TURN_180;
  } else {
    return true;
  }

  if (!plannerQueuePush(item)) {
    enterFaultMode("planner queue overflow (explore)");
    return false;
  }
  plannerQueueTotalBuilt = plannerQueueCount;
  if (AppConfig::Explorer::QUEUE_DEBUG_PRINT) {
    debugPrintln("[QUEUE] explore enqueue " + String(queueItemName(item.primitive)) +
                 (item.primitive == MOTION_MOVE_MULTI_CELL
                    ? (" cells=" + String((int)item.forwardCells))
                    : ""));
  }
  return true;
}

static bool buildAndEnqueueSpeedRunQueue() {
  FloodFillExplorer::QueuedAction actions[kPlannerQueueStorageCapacity];
  uint16_t actionCount = 0;
  const uint16_t cap = plannerQueueCapacity();
  if (!explorer.buildQueuedActionsFromCurrentPose(actions, cap, actionCount)) {
    enterFaultMode("speedrun queue build failed");
    return false;
  }

  clearPlannerQueue();
  for (uint16_t i = 0; i < actionCount; ++i) {
    PlannerQueueItem item;
    item.forwardCells = actions[i].forwardCells > 0 ? actions[i].forwardCells : 1;
    item.endsAtKnownWall = actions[i].endsAtKnownWall;
    if (actions[i].action == FloodFillExplorer::ACT_MOVE_F) {
      item.primitive = (item.forwardCells <= 1) ? MOTION_MOVE_ONE_CELL : MOTION_MOVE_MULTI_CELL;
    } else if (actions[i].action == FloodFillExplorer::ACT_TURN_L) {
      item.primitive = MOTION_TURN_LEFT_90;
    } else if (actions[i].action == FloodFillExplorer::ACT_TURN_R) {
      item.primitive = MOTION_TURN_RIGHT_90;
    } else if (actions[i].action == FloodFillExplorer::ACT_TURN_180) {
      item.primitive = MOTION_TURN_180;
    } else {
      continue;
    }
    if (!plannerQueuePush(item)) {
      enterFaultMode("planner queue overflow (speedrun)");
      return false;
    }
  }

  plannerQueueTotalBuilt = plannerQueueCount;
  speedRunQueueNeedsBuild = false;
  if (AppConfig::Explorer::QUEUE_DEBUG_PRINT) {
    debugPrintln("[QUEUE] speedrun built count=" + String(plannerQueueCount));
    debugPrintln("[QUEUE] speedrun seq=" + plannerQueueSequenceString(actions, actionCount));
  }
  return true;
}

static bool startNextQueuedAction() {
  if (plannerQueueItemInFlight || plannerQueueCount == 0) return true;

  PlannerQueueItem item;
  if (!plannerQueuePeek(item)) {
    enterFaultMode("planner queue underflow");
    return false;
  }

  bool ok = false;
  if (item.primitive == MOTION_MOVE_ONE_CELL) {
    ok = motionController.moveOneCell();
  } else if (item.primitive == MOTION_MOVE_MULTI_CELL) {
    ok = motionController.moveCells(item.forwardCells > 0 ? item.forwardCells : 1, item.endsAtKnownWall);
  } else if (item.primitive == MOTION_TURN_LEFT_90) {
    ok = motionController.turnLeft90();
  } else if (item.primitive == MOTION_TURN_RIGHT_90) {
    ok = motionController.turnRight90();
  } else if (item.primitive == MOTION_TURN_180) {
    ok = motionController.turn180();
  } else {
    enterFaultMode("invalid queue primitive");
    return false;
  }

  if (!ok) {
    enterFaultMode("failed to start queued primitive");
    return false;
  }

  if (item.primitive == MOTION_MOVE_ONE_CELL || item.primitive == MOTION_MOVE_MULTI_CELL) {
    activeForwardActionCellsRequested = (item.forwardCells > 0) ? item.forwardCells : 1;
    activeForwardActionCellsCommitted = 0;
  } else {
    clearForwardActionTracking();
  }

  plannerQueueItemInFlight = true;
  plannerQueueInFlightItem = item;
  queueTrace("queue-start");
  if (AppConfig::Explorer::QUEUE_DEBUG_PRINT) {
    const uint16_t done = (plannerQueueTotalBuilt >= plannerQueueCount) ? (plannerQueueTotalBuilt - plannerQueueCount) : 0;
    debugPrintln("[QUEUE] start " + String(queueItemName(item.primitive)) +
                 " idx=" + String((int)(done + 1)) + "/" + String((int)plannerQueueTotalBuilt));
  }
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

static bool handleReachedGoal() {
  queueTrace("goal-handle-enter");
  const bool atOriginalGoal = explorer.isInOriginalGoal(robotState.pose.cellX, robotState.pose.cellY);
  const bool atOriginalStart = explorer.isInOriginalStart(robotState.pose.cellX, robotState.pose.cellY);
  if (atOriginalGoal || atOriginalStart) {
    stopLapTimer(true);
  }
  if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
    if (robotState.speedRunPhase == 1) {
      if (atOriginalGoal && !atOriginalStart) {
        debugPrintln("[SPEEDRUN] reached goal, swap target to home");
        if (activeForwardActionCellsRequested > activeForwardActionCellsCommitted) {
          activeForwardActionCellsRequested = activeForwardActionCellsCommitted;
          if (motionController.primitive() == MOTION_MOVE_MULTI_CELL &&
              activeForwardActionCellsRequested > 0) {
            motionController.limitMoveCellTargetCount(activeForwardActionCellsRequested);
          }
        }
        explorer.advanceTargetAfterReach();
        explorer.setRunning(true);
        clearPlannerQueue();
        speedRunQueueNeedsBuild = true;
        motionController.clearCompletionState();
        robotState.goalReached = false;
        updateRobotLed();
        startLapTimer("GH");
        queueTrace("goal-swap-home");
        return true;
      }
      debugPrintln("[SPEEDRUN] reached home, speedrun finished");
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
        debugPrintSpeedRunQueuePreview("[EXPLORE]");
        mazeSavePending = true;
        debugPrintln("[SPIFFS] save queued");
        clearForwardActionTracking();
        enterIdleMode("shortest path known");
        return false;
      }
    }

    explorer.setRunning(true);
    robotState.goalReached = false;
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

  queueTrace("cell-cross-before");
  advancePoseForwardOneCell();
  activeForwardActionCellsCommitted++;
  updateRobotState();
  queueTrace("cell-cross-after");
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
    if (queueModeEnabledForCurrentMode()) {
      explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
      if (explorer.atGoal()) {
        if (robotState.mode == ROBOT_MODE_EXPLORE) {
          explorer.advanceTargetAfterReach();
        }
        reachedGoal = true;
      }
    } else {
      reachedGoal = explorer.ackPendingActionExternal(true,
        robotState.pose.cellX,
        robotState.pose.cellY,
        headingDir());
    }
  } else {
    explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
    reachedGoal = explorer.atGoal();
    if (reachedGoal) {
      queueTrace("goal-detected-speedrun");
    }
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
  bool stepBudgetReached = false;

  if (status == MOTION_COMPLETED) {
    if (plannerQueueItemInFlight) {
      if (primitive != plannerQueueInFlightItem.primitive) {
        clearPlannerQueue();
        enterFaultMode("queue primitive mismatch");
        return;
      }
      if (!plannerQueuePop()) {
        clearPlannerQueue();
        enterFaultMode("queue pop failed");
        return;
      }
      plannerQueueItemInFlight = false;
      if (AppConfig::Explorer::QUEUE_DEBUG_PRINT) {
        const uint16_t done = (plannerQueueTotalBuilt >= plannerQueueCount)
                                ? (plannerQueueTotalBuilt - plannerQueueCount)
                                : 0;
        debugPrintln("[QUEUE] done " + String(queueItemName(primitive)) +
                     " idx=" + String((int)done) + "/" + String((int)plannerQueueTotalBuilt));
      }
    } else if (queueModeEnabledForCurrentMode() &&
               explorer.isRunning() &&
               primitive != MOTION_SNAP_CENTER &&
               !(robotState.mode == ROBOT_MODE_SPEED_RUN &&
                 robotState.speedRunPhase == 1 &&
                 speedRunQueueNeedsBuild)) {
      clearPlannerQueue();
      enterFaultMode("queued completion without inflight item");
      return;
    }
  }

  if (status == MOTION_COMPLETED) {
    bool reachedGoal = false;
    bool forwardAlreadyCommitted = false;

    if (isMultiCellForward) {
      updateRobotState();
      while (commitForwardActionCell(true, stepBudgetReached, reachedGoal)) {
        forwardAlreadyCommitted = true;
        if (reachedGoal) {
          if (!handleReachedGoal()) {
            return;
          }
          // Goal/home transition handled; do not commit additional cells in this completion tick.
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
      applyStopModeToDriveMotors(AppConfig::Motion::POST_MOTION_SETTLE_STOP_MODE);
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
        robotState.mode == ROBOT_MODE_EXPLORE &&
        queueModeEnabledForCurrentMode() &&
        primitive == MOTION_TURN_180) {
      updateRobotState();
      if (shouldSnapCenterFromKnownBackWall()) {
        if (!motionController.snapCenter()) {
          enterFaultMode("failed to start queue post-uturn snapcenter");
          return;
        }
        debugMotionEvent("[MOTION START]", MOTION_SNAP_CENTER, motionController.status(),
                         robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                         robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                         "source=queue-post-uturn");
        debugPrintln("[SNAP] queue post-uturn snapcenter");
        return;
      }
      if (AppConfig::Explorer::QUEUE_DEBUG_PRINT) {
        debugPrintln("[SNAP] skip queue post-uturn snapcenter (need known back wall)");
      }
    }

    if (isTurnPrimitive &&
        !deferPlannerAckUntilSnapCenter &&
        robotState.mode == ROBOT_MODE_EXPLORE &&
        !queueModeEnabledForCurrentMode()) {
      updateRobotState();
      const bool knownBackWall = shouldSnapCenterFromKnownBackWall();
      const bool sideWallsOpen = !robotState.walls.leftWall && !robotState.walls.rightWall;
      const bool shouldPostTurnSnap =
        (primitive == MOTION_TURN_180 && knownBackWall) ||
        ((primitive == MOTION_TURN_LEFT_90 || primitive == MOTION_TURN_RIGHT_90) &&
         knownBackWall && sideWallsOpen);

      if (shouldPostTurnSnap) {
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
    }

    updateRobotState();
    schedulePostTurnWallSettle(primitive);
    if (robotState.mode == ROBOT_MODE_SPEED_RUN &&
        robotState.speedRunPhase == 1 &&
        AppConfig::Explorer::QUEUE_ENABLE_SPEEDRUN1 &&
        explorer.isRunning()) {
      if (speedRunGoalSnapPending && primitive == MOTION_SNAP_CENTER) {
        speedRunGoalSnapPending = false;
        if (AppConfig::Explorer::QUEUE_DEBUG_PRINT) {
          debugPrintln("[QUEUE] goal snapcenter complete");
        }
      } else if (!speedRunGoalSnapPending &&
                 plannerQueueCount == 0 &&
                 !plannerQueueItemInFlight &&
                 primitive != MOTION_SNAP_CENTER) {
        explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
        const bool atOriginalGoalNow =
          explorer.isInOriginalGoal(robotState.pose.cellX, robotState.pose.cellY);
        const bool atOriginalStartNow =
          explorer.isInOriginalStart(robotState.pose.cellX, robotState.pose.cellY);
        // Speedrun1 round-trip rule:
        // at goal, swap target to home and rebuild queue; do not snap before home-leg queue starts.
        if (atOriginalGoalNow && !atOriginalStartNow) {
        if (!handleReachedGoal()) {
            return;
          }
          return;
        }
        if (!explorer.atGoal()) {
          clearPlannerQueue();
          enterFaultMode("speedrun queue exhausted before goal");
          return;
        }
        if (!motionController.snapCenter()) {
          clearPlannerQueue();
          enterFaultMode("failed to start speedrun goal snapcenter");
          return;
        }
        speedRunGoalSnapPending = true;
        debugMotionEvent("[MOTION START]", MOTION_SNAP_CENTER, motionController.status(),
                         robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                         robotState.pose.cellX, robotState.pose.cellY, headingDir(),
                         "source=speedrun-goal");
        debugPrintln("[SNAP] speedrun goal snapcenter");
        return;
      }
    }

    if (!forwardAlreadyCommitted && robotState.mode != ROBOT_MODE_SPEED_RUN) {
      debugWallApplyEvent("[WALL APPLY]", "motion_complete");
      applyWallsToExplorer();
      debugWallApplyEvent("[WALL APPLIED]", "motion_complete");
    }

    if (!forwardAlreadyCommitted) {
      if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
        explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
        reachedGoal = explorer.atGoal();
      } else if (queueModeEnabledForCurrentMode()) {
        explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
        if (explorer.atGoal()) {
          explorer.advanceTargetAfterReach();
          reachedGoal = true;
        }
      } else if (!queueModeEnabledForCurrentMode() && deferPlannerAckUntilSnapCenter) {
        deferPlannerAckUntilSnapCenter = false;
        reachedGoal = explorer.ackPendingActionExternal(true,
          robotState.pose.cellX,
          robotState.pose.cellY,
          headingDir());
      } else if (!queueModeEnabledForCurrentMode() &&
                 (!runStartSnapPending || !isSnapCenterPrimitive)) {
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

    if (reachedGoal && !handleReachedGoal()) {
      return;
    }

    if (stepBudgetReached) {
      enterIdleMode("explore step budget reached");
      return;
    }

    if (AppConfig::Motion::POST_MOTION_HARD_STOP_HOLD_MS > 0 &&
        !continuousSpeedRun &&
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
    if (robotState.mode != ROBOT_MODE_SPEED_RUN &&
        !queueModeEnabledForCurrentMode()) {
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

  while (commitForwardActionCell(false, stepBudgetReached, reachedGoal)) {
    if (reachedGoal) {
      if (!handleReachedGoal()) {
        return;
      }
      // Goal/home transition handled; do not commit additional cells in this service tick.
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
  if (queueSuppressWallRegistration()) {
    return;
  }
  if (!isPostTurnWallSettleReady()) {
    return;
  }
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

static void schedulePostTurnWallSettle(MotionPrimitiveType primitive) {
  if (robotState.mode != ROBOT_MODE_EXPLORE) return;
  if (queueModeEnabledForCurrentMode()) return;
  const bool turnOrSnap =
    primitive == MOTION_TURN_LEFT_90 ||
    primitive == MOTION_TURN_RIGHT_90 ||
    primitive == MOTION_TURN_180 ||
    primitive == MOTION_SNAP_CENTER;
  if (!turnOrSnap) return;

  const uint32_t settleMs = AppConfig::Motion::POST_TURN_WALL_SETTLE_MS;
  const uint8_t needSamples = max((uint8_t)1, AppConfig::Motion::POST_TURN_WALL_STABLE_SAMPLES);
  if (settleMs == 0 && needSamples <= 1) return;

  postTurnWallSettlePending = true;
  postTurnWallSettleUntilMs = millis() + settleMs;
  postTurnWallStableCount = 0;
  postTurnWallLastSignature = 0xFF;
  if (AppConfig::Debug::DEBUG_MOTION_FLOW) {
    debugPrintln("[WALL] settle start ms=" + String((int)settleMs) +
                 " samples=" + String((int)needSamples));
  }
}

static bool isPostTurnWallSettleReady() {
  if (!postTurnWallSettlePending) return true;

  uint8_t validMask = 0;
  if (robotState.walls.leftValid) validMask |= 0x1;
  if (robotState.walls.frontValid) validMask |= 0x2;
  if (robotState.walls.rightValid) validMask |= 0x4;

  uint8_t wallMask = 0;
  if (robotState.walls.leftWall) wallMask |= 0x1;
  if (robotState.walls.frontWall) wallMask |= 0x2;
  if (robotState.walls.rightWall) wallMask |= 0x4;

  const uint8_t signature = (uint8_t)(wallMask | (validMask << 3));
  if (validMask == 0) {
    postTurnWallStableCount = 0;
    postTurnWallLastSignature = 0xFF;
  } else if (signature == postTurnWallLastSignature) {
    if (postTurnWallStableCount < 255) postTurnWallStableCount++;
  } else {
    postTurnWallLastSignature = signature;
    postTurnWallStableCount = 1;
  }

  const uint8_t needSamples = max((uint8_t)1, AppConfig::Motion::POST_TURN_WALL_STABLE_SAMPLES);
  const bool sampleReady = postTurnWallStableCount >= needSamples;
  const bool timeReady = (int32_t)(millis() - postTurnWallSettleUntilMs) >= 0;
  if (!sampleReady || !timeReady) return false;

  postTurnWallSettlePending = false;
  if (AppConfig::Debug::DEBUG_MOTION_FLOW) {
    debugPrintln("[WALL] settle done valid=" + String((int)validMask) +
                 " walls=" + String((int)wallMask));
  }
  return true;
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
  if (AppConfig::Explorer::QUEUE_ENABLE_SPEEDRUN1 && robotState.speedRunPhase == 1) {
    clearPlannerQueue();
    speedRunQueueNeedsBuild = true;
    if (AppConfig::Explorer::QUEUE_DEBUG_PRINT) {
      debugPrintln("[QUEUE] speedrun queue build requested");
    }
  }
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
  constexpr BaseType_t kCoreRealtime = 1;
  constexpr BaseType_t kCoreApp = 0;
  ledController.begin();
  ledController.setState(LedController::State::RED);
  Serial.begin(921600);
  vTaskDelay(pdMS_TO_TICKS(200));
  i2cRecover(AppConfig::I2C::SDA, AppConfig::I2C::SCL);
  if (AppConfig::Inputs::ENABLE_BOOT_BUTTON_LAUNCH) {
    pinMode(AppConfig::Inputs::BOOT_BUTTON_PIN,
            AppConfig::Inputs::BOOT_BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  }
  debugPrintln(String("[BUILD] profile=") + AppConfig::Build::PROFILE_NAME);
  setPose(AppConfig::Maze::START_X, AppConfig::Maze::START_Y, AppConfig::Maze::START_HEADING);

  explorer.setStateExtrasJsonProvider(explorerLapStateJson);
#if APP_LITE_FIRMWARE
  wifiOk = false;
  debugPrintln("[BOOT] LITE_FIRMWARE profile: WiFi/OTA/Web disabled");
#else
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
  wifiCfg.uploadCore = wifiCfg.wifiCore;
  wifiCfg.uploadTaskPrio = wifiCfg.wifiTaskPrio + 1;
  wifiCfg.uploadTaskStack = wifiCfg.wifiTaskStack;
  dbg.setLedCommandHandler(onWebLedCommand);
  dbg.setTelnetReconnectHandler(onWebTelnetReconnect);
  dbg.setHealthJsonProvider(onWebHealthJson);
  dbg.setSerialOutputAllowedHandler(serialOutputEnabled);
  dbg.setReconnectAllowedHandler(wifiReconnectAllowed);
  wifiOk = dbg.begin(wifiCfg);
  debugPrintln(wifiOk ? "Boot OK" : "Boot with WiFi failed");
#endif
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
    leftMotor.setLog(logMotorPidToTelnet);
    rightMotor.setLog(logMotorPidToTelnet);
    leftMotor.setSpeedPID(AppConfig::Motors::PID_KP, AppConfig::Motors::PID_KI,
                          AppConfig::Motors::PID_KD, AppConfig::Motors::PID_OUT_LIMIT,
                          AppConfig::Motors::PID_I_LIMIT, AppConfig::Motors::PID_D_FILTER_HZ,
                          AppConfig::Motors::PID_SLEW_RATE_LEFT);
    rightMotor.setSpeedPID(AppConfig::Motors::PID_KP, AppConfig::Motors::PID_KI,
                           AppConfig::Motors::PID_KD, AppConfig::Motors::PID_OUT_LIMIT,
                           AppConfig::Motors::PID_I_LIMIT, AppConfig::Motors::PID_D_FILTER_HZ,
                           AppConfig::Motors::PID_SLEW_RATE_RIGHT);
  }

  tofArray.setWallThreshold(AppConfig::Tof::WALL_THRESHOLD_MM);
  tofArray.setCenterPid(AppConfig::Motion::CENTER_PID_KP,
                        AppConfig::Motion::CENTER_PID_KI,
                        AppConfig::Motion::CENTER_PID_KD,
                        AppConfig::Motion::CENTER_PID_I_LIMIT,
                        AppConfig::Motion::CENTER_PID_OUT_LIMIT);
  tofArray.setLog(logToDbg);
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

  xTaskCreatePinnedToCore(motorTask,     "motor",     4096, nullptr, 3, &motorTaskHandle,     kCoreRealtime);
  xTaskCreatePinnedToCore(tofTask,       "tof",       4096, nullptr, 2, &tofTaskHandle,       kCoreRealtime);
  xTaskCreatePinnedToCore(plannerTaskFn, "planner",   4096, nullptr, 2, &plannerTaskHandle,   kCoreApp);
  xTaskCreatePinnedToCore(explorerTask,  "explorer",  8192, nullptr, 3, &explorerTaskHandle,  kCoreApp);
  xTaskCreatePinnedToCore(userTaskFn,    "user",      6144, nullptr, 1, &userTaskHandle,      kCoreApp);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 0, &telemetryTaskHandle, kCoreApp);

  motorLoopTimerActive = startRealtimeLoopTimer(
    &motorLoopTimer, motorTaskHandle, AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS, "motor");
  tofLoopTimerActive = startRealtimeLoopTimer(
    &tofLoopTimer, tofTaskHandle, AppConfig::Tasks::TOF_LOOP_PERIOD_MS, "tof");

  enterIdleMode("ready");
  updateRobotLed();
}

void loopApp() {
  vTaskDelay(portMAX_DELAY);
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
    servicePendingPersistence();
    serviceMotorBothFlipTest();
    serviceCenterTrackTest();
    serviceBootButtonLauncher();
    serviceCrossCoreDebugLogs();
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
      updateRobotState();
      if (!robotState.readyForMotion) {
        enterFaultMode("robot not ready for motion");
      } else {
        explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
        if (queueModeEnabledForCurrentMode()) {
          if (robotState.mode == ROBOT_MODE_SPEED_RUN &&
              robotState.speedRunPhase == 1 &&
              speedRunQueueNeedsBuild &&
              plannerQueueCount == 0 &&
              !plannerQueueItemInFlight) {
            if (!buildAndEnqueueSpeedRunQueue()) {
              vTaskDelayUntil(&last, period);
              continue;
            }
          } else if (robotState.mode == ROBOT_MODE_EXPLORE &&
                     plannerQueueCount == 0 &&
                     !plannerQueueItemInFlight) {
            if (!enqueueExplorePlannerAction()) {
              vTaskDelayUntil(&last, period);
              continue;
            }
          }

          if (!plannerQueueItemInFlight && plannerQueueCount > 0) {
            if (!startNextQueuedAction()) {
              vTaskDelayUntil(&last, period);
              continue;
            }
          } else if (!plannerQueueItemInFlight &&
                     plannerQueueCount == 0 &&
                     !speedRunGoalSnapPending) {
            if (robotState.mode == ROBOT_MODE_EXPLORE && explorer.atGoal()) {
              robotState.goalReached = true;
              robotState.speedRunReady = true;
              updateRobotLed();
              enterIdleMode("explore finished");
            } else if (robotState.mode == ROBOT_MODE_SPEED_RUN &&
                       robotState.speedRunPhase == 1 &&
                       explorer.atGoal()) {
              handleReachedGoal();
            }
          }
        } else {
          FloodFillExplorer::Action act = FloodFillExplorer::ACT_NONE;
          if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
            act = explorer.requestNextActionNoAck();
          } else {
            updateRobotState();
            if (!queueModeEnabledForCurrentMode() && !isPostTurnWallSettleReady()) {
              vTaskDelayUntil(&last, period);
              continue;
            }
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
    }

    vTaskDelayUntil(&last, period);
  }
}

static void motorTask(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS);

  for (;;) {
    if (motorLoopTimerActive) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    } else {
      vTaskDelayUntil(&last, period);
    }
    serviceLoopWatchdog(motorLoopWatchdog, AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS);
    if (dbg.isUpdateInProgress()) {
      resetLoopWatchdogState(motorLoopWatchdog);
      continue;
    }
    leftMotor.update();
    rightMotor.update();
  }
}

static void servicePendingPersistence() {
  if (!mazeSavePending) return;
  mazeSavePending = false;
  if (!PersistenceStore::saveMaze(explorer)) {
    debugPrintln("[SPIFFS] save failed");
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
    if (tofLoopTimerActive) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    } else {
      vTaskDelayUntil(&lastWake, period);
    }
    serviceLoopWatchdog(tofLoopWatchdog, AppConfig::Tasks::TOF_LOOP_PERIOD_MS);
    if (dbg.isUpdateInProgress()) {
      resetLoopWatchdogState(tofLoopWatchdog);
      continue;
    }
    tofArray.update();
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
  debugPrintln("[CMD] help | explore [n] | speedrun [1-4] | qtrace on|off|status | idle | stop | brake | hardstop | restart | move [n] | back | left | right | uturn | testsnap | status | resetpose x y h | setgoal x y w h | clearmaze");
  debugPrintln("[SPEEDRUN] 1=round-trip home->goal->home | 2=one-way home->goal (dedicated profile) | 3-4 inherit previous until tuned");
  debugPrintln("[CMD] led cycle|rotate|off|red|green|blue|yellow|cyan|magenta|white");
  debugPrintln("[CMD] maze");
  debugPrintln("[CMD] test | test off | test loop status|battery|sensors|sensorsraw|encoders|maze|off");
  debugPrintln("[CMD] test battery|sensors|sensorsraw|motorl [tps]|motorr [tps]|motor both [tps]|encoders");
  debugPrintln("[CMD] test center left|right|dual [tps] | test center off");
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
    maze_debug_s(true);
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
  if (line == "qtrace" || line == "qtrace status") {
    debugPrintln(String("[CMD] qtrace=") + (queueTraceEnabled ? "on" : "off"));
    queueTrace("manual");
    return;
  }
  if (line == "qtrace on") {
    queueTraceEnabled = true;
    debugPrintln("[CMD] qtrace on");
    queueTrace("manual-on");
    return;
  }
  if (line == "qtrace off") {
    queueTrace("manual-off");
    queueTraceEnabled = false;
    debugPrintln("[CMD] qtrace off");
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
    motionController.stop();
    leftMotor.enableSpeedControl(false);
    rightMotor.enableSpeedControl(false);
    leftMotor.brakeStop();
    rightMotor.brakeStop();
    resetCommonModeState();
    robotState.mode = ROBOT_MODE_IDLE;
    robotState.motionStatus = MOTION_IDLE;
    robotState.activePrimitive = MOTION_NONE;
    robotState.pose.forwardProgressMm = 0.0f;
    robotState.pose.turnProgressDeg = 0.0f;
    updateRobotLed();
    debugPrintln("[CMD] brake stop");
    return;
  }
  if (line == "hardstop") {
    motionController.stop();
    leftMotor.hardStop();
    rightMotor.hardStop();
    resetCommonModeState();
    robotState.mode = ROBOT_MODE_IDLE;
    robotState.motionStatus = MOTION_IDLE;
    robotState.activePrimitive = MOTION_NONE;
    robotState.pose.forwardProgressMm = 0.0f;
    robotState.pose.turnProgressDeg = 0.0f;
    updateRobotLed();
    debugPrintln("[CMD] hard stop (target TPS = 0)");
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
  if (line == "test center off") {
    centerTrackTestMode = CENTER_TEST_OFF;
    centerTrackTestLastCorrection = 0.0f;
    tofArray.setStraightTrackMode(MultiVL53L0X::TRACK_NONE);
    leftMotor.coastStop();
    rightMotor.coastStop();
    debugPrintln("[TEST CENTER] off");
    return;
  }
  if (line.startsWith("test center ")) {
    String args = line.substring(12);
    args.trim();

    float tps = centerTrackTestBaseTps;
    if (args.startsWith("left")) {
      centerTrackTestMode = CENTER_TEST_LEFT;
      String rest = args.substring(4);
      rest.trim();
      if (rest.length() > 0) {
        float parsed = rest.toFloat();
        if (parsed == 0.0f) {
          debugPrintln("[CMD] usage: test center left [tps]");
          return;
        }
        tps = parsed;
      }
    } else if (args.startsWith("right")) {
      centerTrackTestMode = CENTER_TEST_RIGHT;
      String rest = args.substring(5);
      rest.trim();
      if (rest.length() > 0) {
        float parsed = rest.toFloat();
        if (parsed == 0.0f) {
          debugPrintln("[CMD] usage: test center right [tps]");
          return;
        }
        tps = parsed;
      }
    } else if (args.startsWith("dual")) {
      centerTrackTestMode = CENTER_TEST_DUAL;
      String rest = args.substring(4);
      rest.trim();
      if (rest.length() > 0) {
        float parsed = rest.toFloat();
        if (parsed == 0.0f) {
          debugPrintln("[CMD] usage: test center dual [tps]");
          return;
        }
        tps = parsed;
      }
    } else {
      debugPrintln("[CMD] usage: test center left|right|dual [tps] | test center off");
      return;
    }

    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    motorBothFlipTestEnabled = false;
    centerTrackTestBaseTps = tps;
    centerTrackTestLastCorrection = 0.0f;
    centerTrackTestLastPrintMs = 0;
    tofArray.resetCenterPid();
    debugPrintln("[TEST CENTER] mode=" + String(centerTrackTestModeName(centerTrackTestMode)) +
                 " base tps=" + String(centerTrackTestBaseTps, 1));
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
````

### `AppRuntime.h`

````cpp
#pragma once

#include <Arduino.h>

namespace MainApp {

void setupApp(TaskFunction_t userTaskFn, TaskFunction_t plannerTaskFn);
void loopApp();
void userTaskBody(void* arg);
void plannerTaskBody(void* arg);

}  // namespace MainApp
````

### `ARCHITECTURE_SCAN.md`

````md
# Mouse_esp32s3 Architecture Scan Report

## Executive Summary

`Mouse_esp32s3` is an ESP32-S3 Arduino micromouse firmware project, not a header-only library. The codebase is organized around a thin Arduino entrypoint, a central runtime/orchestration module, hardware drivers, a motion executor, a floodfill planner, and optional Wi-Fi/OTA/web tooling.

Current documented project version: `0.4.1`

## Module Map

### Entry and runtime

- `Mouse_esp32s3.ino`
  - Thin Arduino wrapper only.
  - Exposes `setup()`, `loop()`, and task wrapper functions that forward into `MainApp`.
- `AppRuntime.h`
  - Public bridge between the `.ino` wrapper and the main runtime module.
- `AppRuntime.cpp`
  - Main application runtime.
  - Owns startup, serial command parsing, mode changes, planner/executor coordination, telemetry, lap timing, Wi-Fi integration, and task creation.

### Hardware and motion

- `Config.h` / `Config.cpp`
  - Single source of truth for build profile, pins, thresholds, Wi-Fi settings, and tuning values.
- `DcMotor.h` / `DcMotor.cpp`
  - Per-motor PWM, encoder, and TPS/PID support.
- `MotionController.h` / `MotionController.cpp`
  - Primitive motion execution.
  - Supports forward motion, multi-cell moves, short reverse/forward motions, left/right turns, 180 turns, stop behavior, and snap-center.
- `Battery.h` / `Battery.cpp`
  - ADC sampling, calibration, divider compensation, voltage estimation, and warning/critical telemetry state.
  - Battery state is currently telemetry-only; motion is not blocked on warning/critical thresholds.
- `LedController.h` / `LedController.cpp`
  - Single-pixel RGB status LED control and manual LED commands.

### Sensors, planning, persistence

- `MultiVL53L0X.h` / `MultiVL53L0X.cpp`
  - Multi-sensor VL53L0X bring-up, reading, filtering, and wall interpretation.
- `FloodFillExplorer.h` / `FloodFillExplorer.cpp`
  - Maze memory, floodfill planner, ASCII maze/debug helpers, HTTP floodfill page, and WebSocket sync.
- `PersistenceStore.h` / `PersistenceStore.cpp`
  - SPIFFS-backed persistence for saved maze wall memory only.
- `RobotTypes.h`
  - Shared enums, mode identifiers, motion status values, and `RobotState`.

### Connectivity and upload

- `WiFiOtaWebSerial.h` / `WiFiOtaWebSerial.cpp`
  - Wi-Fi service loop, Arduino OTA, port `80` control page, port `82` browser upload page, debug console helpers, and upload-safe task coordination.

## Runtime Structure

### Task and core split

From the current runtime setup:

- Core `1` realtime tasks:
  - motor task
  - TOF task
- Core `0` app tasks:
  - planner task
  - explorer task
  - user task
  - telemetry task
  - Wi-Fi/OTA task from `AppConfig::Wifi::CORE`

### Control flow

1. `setup()` in `Mouse_esp32s3.ino` forwards into `MainApp::setupApp(...)`.
2. `AppRuntime.cpp` initializes battery, motors, TOF, explorer, persistence, LED state, optional Wi-Fi services, and FreeRTOS tasks.
3. Manual commands, explore flow, and speedrun flow all dispatch physical movement through `MotionController`.
4. Pose updates are committed only after motion completion handling in the runtime.
5. In explore mode, wall sensing is applied around planner ACK/commit flow so maze memory follows physical progress.
6. Saved maze wall memory is restored from SPIFFS at boot when present and cleared by `clearmaze`.

## Explore and Speedrun Behavior

### Explore

- `explore` and `explore n` keep the current known maze.
- `clearmaze` is the explicit wall-memory reset path.
- Explore can continue goal-to-home and home-to-goal until the shortest path is considered stable.
- When the shortest path becomes known, the runtime saves maze wall memory and enters idle.

### Speedrun

- `speedrun` means `speedrun 1`.
- `speedrun 1` is the round-trip shortest-path run: home -> goal -> home.
- `speedrun 2` is the dedicated one-way shortest-path run: home -> goal.
- `speedrun 3` and `speedrun 4` currently inherit the previous tuned phase behavior.
- `speedrun 2` uses the dedicated `SpeedRun2` motion profile and continuous execution path.

## Web and Service Surface

Current network surface from the code/config:

- port `80`: lightweight control/status page
- port `81`: floodfill web UI
- port `82`: browser firmware upload page
- port `2323`: debug TCP console
- mDNS hostname service for the configured device hostname

Browser upload behavior:

- Open `http://<ip>:82/`
- The page uses chunked HTTP upload endpoints:
  - `/upload/start`
  - `/upload/chunk`
  - `/upload/finish`
- Upload-safe mode pauses motion/planner/sensor-heavy activity during transfer

## Important Current Constraints

- Battery warning/critical levels are reported, but they do not currently abort motion.
- Compile/build verification is still a bring-up task and has not been established as a guaranteed known-good baseline in this repository.
- Motion and wall-centering values still require on-robot tuning in `Config.h`.
- The codebase is optimized for reliable bring-up and observability first, not final race tuning.

## Recommended Reading Order

1. `README.md`
2. `AGENT.md`
3. `Config.h`
4. `AppRuntime.cpp`
5. `MotionController.*`
6. `FloodFillExplorer.*`
7. `WiFiOtaWebSerial.*`
8. `TODO.md`
````

### `Battery.cpp`

````cpp
#include "Battery.h"

void Battery::begin(uint8_t adcPin, uint8_t samples) {
  adcPin_ = adcPin;
  samples_ = (samples == 0) ? 1 : samples;

  analogReadResolution(12);
#if defined(ESP32)
  analogSetPinAttenuation(adcPin_, ADC_11db);
#endif
  pinMode(adcPin_, INPUT);
  started_ = true;
}

void Battery::setCalibration(uint16_t rawLow, float voltageLow,
                             uint16_t rawHigh, float voltageHigh) {
  rawLow_ = rawLow;
  rawHigh_ = (rawHigh <= rawLow) ? (rawLow + 1) : rawHigh;
  voltageLow_ = voltageLow;
  voltageHigh_ = voltageHigh;
}

void Battery::setDivider(float topKohm, float bottomKohm) {
  if (topKohm <= 0.0f || bottomKohm <= 0.0f) {
    dividerRatio_ = 0.0f;
    return;
  }
  dividerRatio_ = (topKohm + bottomKohm) / bottomKohm;
}

void Battery::setThresholds(float warningVoltage, float criticalVoltage) {
  warningVoltage_ = warningVoltage;
  criticalVoltage_ = criticalVoltage;
}

void Battery::update() {
  if (!started_) return;

  uint32_t acc = 0;
  uint32_t mvAcc = 0;
  for (uint8_t i = 0; i < samples_; ++i) {
    acc += analogRead(adcPin_);
#if defined(ESP32)
    mvAcc += (uint32_t)analogReadMilliVolts(adcPin_);
#endif
  }

  raw_ = (uint16_t)(acc / samples_);
#if defined(ESP32)
  adcVoltage_ = ((float)mvAcc / (float)samples_) / 1000.0f;
#else
  adcVoltage_ = 3.3f * ((float)raw_ / 4095.0f);
#endif

  if (dividerRatio_ > 0.0f) {
    dividerEstimatedVoltage_ = adcVoltage_ * dividerRatio_;
    voltage_ = 0.9 * voltage_ + 0.1 * dividerEstimatedVoltage_;
  } else {
    dividerEstimatedVoltage_ = 0.0f;
    const float rawSpan = (float)(rawHigh_ - rawLow_);
    const float tRaw = rawSpan > 0.0f ? ((float)raw_ - (float)rawLow_) / rawSpan : 0.0f;
    voltage_ = 0.9 * voltage_ + 0.1 * (voltageLow_ + tRaw * (voltageHigh_ - voltageLow_));
  }
  const float voltageSpan = voltageHigh_ - voltageLow_;
  float tPercent = voltageSpan > 0.0f ? (voltage_ - voltageLow_) / voltageSpan : 0.0f;
  if (tPercent < 0.0f) tPercent = 0.0f;
  if (tPercent > 1.0f) tPercent = 1.0f;
  percent_ = tPercent * 100.0f;

  if (voltage_ <= criticalVoltage_) {
    state_ = BATTERY_CRITICAL;
  } else if (voltage_ <= warningVoltage_) {
    state_ = BATTERY_WARNING;
  } else {
    state_ = BATTERY_OK;
  }
}
````

### `Battery.h`

````cpp
#pragma once

#include <Arduino.h>

class Battery {
public:
  enum State : uint8_t {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL
  };

  void begin(uint8_t adcPin, uint8_t samples = 8);
  void setCalibration(uint16_t rawLow, float voltageLow,
                      uint16_t rawHigh, float voltageHigh);
  void setDivider(float topKohm, float bottomKohm);
  void setThresholds(float warningVoltage, float criticalVoltage);
  void update();

  float voltage() const { return voltage_; }
  float percent() const { return percent_; }
  float adcVoltage() const { return adcVoltage_; }
  float dividerEstimatedVoltage() const { return dividerEstimatedVoltage_; }
  State state() const { return state_; }
  uint16_t raw() const { return raw_; }
  bool isReady() const { return started_; }

private:
  uint8_t adcPin_ = 0;
  uint8_t samples_ = 8;
  bool started_ = false;

  uint16_t raw_ = 0;
  uint16_t rawLow_ = 2800;
  uint16_t rawHigh_ = 3350;
  float voltageLow_ = 7.20f;
  float voltageHigh_ = 8.40f;
  float warningVoltage_ = 7.10f;
  float criticalVoltage_ = 6.90f;
  float dividerRatio_ = 0.0f;

  float voltage_ = 0.0f;
  float percent_ = 0.0f;
  float adcVoltage_ = 0.0f;
  float dividerEstimatedVoltage_ = 0.0f;
  State state_ = BATTERY_CRITICAL;
};
````

### `Config.cpp`

````cpp
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
const uint8_t GOAL_X0 = 7;
const uint8_t GOAL_Y0 = 7;
const uint8_t GOAL_W = 2;
const uint8_t GOAL_H = 2;
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
const bool QUEUE_TRACE_DEFAULT = false;
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
const float MOVE_SPEED_TPS = 600.0f;
const float CORRIDOR_MOVE_SPEED_TPS = 700.0f;
const float SHORT_FORWARD_DISTANCE_MM = 40.0f;
const float SHORT_FORWARD_SPEED_TPS = 300.0f;
const float REVERSE_DISTANCE_MM = 100.0f;
const float REVERSE_SPEED_TPS = 300.0f;
const uint32_t SNAP_CENTER_STOP_HOLD_MS = 1;
const float TURN_SPEED_TPS = 300.0f;
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
const float DISTANCE_APPROACH_MIN_SPEED_TPS = 300.0f;
const float FRONT_APPROACH_START_FACTOR = 1.4f;
const float FRONT_APPROACH_MIN_SPEED_TPS = 300.0f;
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
const uint32_t POST_TURN_WALL_SETTLE_MS = 45;
const uint8_t POST_TURN_WALL_STABLE_SAMPLES = 2;
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
const uint16_t QUEUE_CAPACITY = 250;
const bool QUEUE_DEBUG_PRINT = false;
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
const uint32_t PLANNER_LOOP_PERIOD_MS = 50;
const uint32_t MOTOR_LOOP_PERIOD_MS = 5;
const uint32_t EXPLORER_LOOP_PERIOD_MS = 50;
const uint32_t TOF_LOOP_PERIOD_MS = 5;
const uint32_t TELEMETRY_LOOP_PERIOD_MS = 1000;
}
}  // namespace AppConfig
````

### `Config.h`

````cpp
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
// Affects: Battery.cpp sampling and battery telemetry readiness.
extern const uint8_t ADC_PIN;

// Physical divider currently expected:
// battery+ -> 56k -> ADC node -> 18k -> GND
// Divider ratio at ADC ~= 18 / (56 + 18) = 0.2432
// Battery voltage ~= ADC voltage * 4.1111
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

// Runtime battery telemetry thresholds.
// WARNING and CRITICAL are currently reported to telemetry/status output.
// They do not automatically block or abort motion in the current runtime.
// Affects: reported battery state and status/debug output.
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
extern const float PID_SLEW_RATE_LEFT;
extern const float PID_SLEW_RATE_RIGHT;
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
// Stop mode used during snap-center back/hold phases.
extern const MotionController::StopMode SNAP_CENTER_HOLD_STOP_MODE;
// Stop mode applied in AppRuntime right after each completed primitive
// before the optional settle hold delay.
extern const MotionController::StopMode POST_MOTION_SETTLE_STOP_MODE;

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
// Extra guard after a turn/snap before explore registers walls.
// Helps avoid transient TOF states right after heading changes.
extern const uint32_t POST_TURN_WALL_SETTLE_MS;
extern const uint8_t POST_TURN_WALL_STABLE_SAMPLES;
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
// Motion queue runtime behavior.
extern const bool QUEUE_ENABLE_EXPLORE;
extern const bool QUEUE_ENABLE_SPEEDRUN1;
extern const bool QUEUE_DISABLE_WALL_REGISTER_WHILE_ACTIVE;
extern const uint16_t QUEUE_CAPACITY;
extern const bool QUEUE_DEBUG_PRINT;
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
  // Queue runtime trace default (can still be toggled at runtime by `qtrace on/off`).
  extern const bool QUEUE_TRACE_DEFAULT;
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
  cfg.snapCenterHoldStopMode = Motion::SNAP_CENTER_HOLD_STOP_MODE;
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

````

### `DcMotor.cpp`

````cpp
#include "esp32-hal-gpio.h"
#include "DcMotor.h"
#include "Config.h"
#include "driver/gpio.h"

#if defined(ESP32)
  #include <esp32-hal-ledc.h>
#endif

DcMotor* DcMotor::_instances[4] = {nullptr, nullptr, nullptr, nullptr};
static portMUX_TYPE sMotorPidTraceMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t sMotorPidTraceCounterL = 0;
static uint32_t sMotorPidTraceCounterR = 0;

DcMotor::DcMotor() {}

int DcMotor::allocSlot(DcMotor* m) {
  for (int i = 0; i < 4; i++) {
    if (_instances[i] == nullptr) { _instances[i] = m; return i; }
  }
  return -1;
}

// PWM write wrapper: Arduino-ESP32 core v3 writes by pin, core v2 writes by channel
void DcMotor::pwmWriteDuty(uint32_t duty) {
  if (duty > _pwmMax) duty = _pwmMax;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(_p.pwm, duty);     // core v3
#else
  ledcWrite(_pwmCh, duty);     // core v2
#endif
}

bool DcMotor::begin(const Pins& pins,
                    uint8_t pwmChannel,
                    uint32_t pwmFreq,
                    uint8_t pwmResolutionBits)
{
  _p = pins;
  _pwmCh = pwmChannel;
  _pwmFreq = pwmFreq;
  _pwmResBits = pwmResolutionBits;

  pinMode(_p.in1, OUTPUT);
  pinMode(_p.in2, OUTPUT);
  pinMode(_p.pwm, OUTPUT);

  pinMode(_p.encA, INPUT_PULLUP);
  pinMode(_p.encB, INPUT_PULLUP);

  _pwmMax = (1UL << _pwmResBits) - 1;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Arduino-ESP32 core v3
  if (!ledcAttachChannel(_p.pwm, _pwmFreq, _pwmResBits, _pwmCh)) return false;
  pwmWriteDuty(0);
#else
  // Arduino-ESP32 core v2
  ledcSetup(_pwmCh, _pwmFreq, _pwmResBits);
  ledcAttachPin(_p.pwm, _pwmCh);
  pwmWriteDuty(0);
#endif

  // allocate instance slot for ISR
  _slot = allocSlot(this);
  if (_slot < 0) return false;

  // Attach ONLY encA interrupt (CHANGE)
  switch (_slot) {
    case 0: attachInterrupt(digitalPinToInterrupt(_p.encA), isrEncA0, RISING); break;
    case 1: attachInterrupt(digitalPinToInterrupt(_p.encA), isrEncA1, RISING); break;
    case 2: attachInterrupt(digitalPinToInterrupt(_p.encA), isrEncA2, RISING); break;
    case 3: attachInterrupt(digitalPinToInterrupt(_p.encA), isrEncA3, RISING); break;
  }

  // init runtime state
  _ticks = 0;
  _lastMicros = micros();
  _lastTicks = 0;
  _tps = 0.0f;
  _speedAccumTicks = 0;
  _speedAccumUs = 0;

  _speedCtrlEnabled = false;
  _targetTPS = 0.0f;

  // safe starter defaults (TPS-based) - tune later
  _kp = 0.00045f;
  _ki = 0.0080f;
  _kd = 0.00005f;
  _outLimit = 0.90f;
  _iLimit = 0.50f;
  _dFilterHz = 25.0f;
  _slewRate = 3.0f;

  resetPID();

  // stop motor
  coastStop();

  return true;
}

void DcMotor::applyDuty(int32_t duty) {
  // Handle inversion
  if (_p.invertDir) duty = -duty;

  // Clamp range
  if (duty > (int32_t)_pwmMax) duty = _pwmMax;
  if (duty < -(int32_t)_pwmMax) duty = -_pwmMax;

  if (duty > 0 && duty < DEADZONE) duty = 0;
  if (duty < 0 && duty > -DEADZONE) duty = 0;

  // Direction
  if (duty > 0) {
    // forward
    digitalWrite(_p.in1, HIGH);
    digitalWrite(_p.in2, LOW);
    pwmWriteDuty(duty);
  } else {
    // backward
    digitalWrite(_p.in1, LOW);
    digitalWrite(_p.in2, HIGH);
    pwmWriteDuty(-duty);  // use absolute value
  }
}

void DcMotor::setPower(float power) {
  _speedCtrlEnabled = false;
  power = clampf(power, -1.0f, 1.0f);

  int32_t duty = (int32_t)(power * (float)_pwmMax + (power >= 0 ? 0.5f : -0.5f));
  applyDuty(duty);
}

void DcMotor::enableSpeedControl(bool en) {
  _speedCtrlEnabled = en;
  if (!en) resetPID();
}

void DcMotor::setSpeedTPS(float tps) {
  _targetTPS = tps;
  _speedCtrlEnabled = true;
}

void DcMotor::coastStop() {
  digitalWrite(_p.in1, LOW);
  digitalWrite(_p.in2, LOW);
  pwmWriteDuty(0);
}

void DcMotor::brakeStop() {
  applyDuty(0);
}

void DcMotor::hardStop() {
  setSpeedTPS(0.0f);
}

void DcMotor::setSpeedPID(float kp, float ki, float kd,
                          float outLimit, float iLimit,
                          float dFilterHz, float slewRatePerSec)
{
  _kp = kp; _ki = ki; _kd = kd;
  _outLimit  = clampf(outLimit, 0.05f, 1.0f);
  _iLimit    = clampf(iLimit, 0.0f, _outLimit);
  _dFilterHz = (dFilterHz <= 0.0f) ? 0.0f : dFilterHz;
  _slewRate  = (slewRatePerSec < 0.0f) ? 0.0f : slewRatePerSec;
  resetPID();
}

void DcMotor::resetPID() {
  _iTerm = 0.0f;
  _dMeas = 0.0f;
  _lastTPSForD = _tps;
  _lastOut = 0.0f;
}

void DcMotor::update() {
  uint32_t now = micros();
  const uint32_t fallbackUs = (AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS > 0)
                              ? (AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS * 1000UL)
                              : 5000UL;
  uint32_t elapsedUs = (_lastMicros == 0) ? fallbackUs : (now - _lastMicros);
  if (elapsedUs == 0) elapsedUs = 1;
  const float dt = elapsedUs / 1e6f;

  // Always update speed estimate even if very fast
  int32_t ticksNow;
  ticksNow = _ticks;

  int32_t dTicks = ticksNow - _lastTicks;

  // Period-based speed estimate from a fixed accumulation window.
  const uint32_t windowUs = (AppConfig::Motors::TPS_ESTIMATE_WINDOW_MS > 0)
                            ? (AppConfig::Motors::TPS_ESTIMATE_WINDOW_MS * 1000UL)
                            : 5000UL;
  _speedAccumTicks += dTicks;
  _speedAccumUs += elapsedUs;
  if (_speedAccumUs >= windowUs) {
    float tpsInstant = _speedAccumTicks / (_speedAccumUs / 1e6f);
    const float alpha = AppConfig::Motors::TPS_LPF_ALPHA;
    _tps = _tps + alpha * (tpsInstant - _tps);
    _speedAccumTicks = 0;
    _speedAccumUs = 0;
  }

  _lastMicros = now;
  _lastTicks = ticksNow;

  if (!_speedCtrlEnabled) return;

  float err = _targetTPS - _tps;
  float pTerm = _kp * err;

  // D on measurement
  float dMeasRaw = (_tps - _lastTPSForD) / dt;
  _lastTPSForD = _tps;

  float dTerm = 0.0f;
  if (_kd != 0.0f) {
    if (_dFilterHz > 0.0f) {
      float rc = 1.0f / (2.0f * 3.1415926f * _dFilterHz);
      float aD = dt / (dt + rc);
      _dMeas = _dMeas + aD * (dMeasRaw - _dMeas);
      dTerm = -_kd * _dMeas;
    } else {
      dTerm = -_kd * dMeasRaw;
    }
  }

  float preSat = pTerm + dTerm;

  // Anti-windup integrator
  float outUnsat = preSat + _iTerm;
  float outSat = clampf(outUnsat, -_outLimit, _outLimit);

  bool saturated = (outUnsat != outSat);
  bool helpsUnsat =
    (!saturated) ||
    (outUnsat >  _outLimit && err < 0) ||
    (outUnsat < -_outLimit && err > 0);

  if (_ki != 0.0f && helpsUnsat) {
    _iTerm += (err * _ki * dt);
    _iTerm = clampf(_iTerm, -_iLimit, _iLimit);
  }

  float out = preSat + _iTerm;

  // Clamp output
  out = clampf(out, -_outLimit, _outLimit);

  // Slew-rate limiting (smooth torque)
  if (_slewRate > 0.0f) {
    float maxStep = _slewRate * dt;
    float delta = out - _lastOut;
    if (delta >  maxStep) out = _lastOut + maxStep;
    if (delta < -maxStep) out = _lastOut - maxStep;
  }
  _lastOut = out;

  int32_t duty = (int32_t)(out * (float)_pwmMax + (out >= 0 ? 0.5f : -0.5f));
  if (_targetTPS != 0.0f && duty != 0) {
    const int32_t minDuty = (int32_t)AppConfig::Motors::PID_MIN_DRIVE_DUTY;
    const int32_t absDuty = (duty >= 0) ? duty : -duty;
    if (absDuty < minDuty) {
      duty = (duty > 0) ? minDuty : -minDuty;
    }
  }
  if (AppConfig::Debug::MOTOR_PID_TRACE) {
    const char* side = "?";
    uint32_t* ctr = nullptr;
    if (_pwmCh == AppConfig::Motors::LEFT_PWM_CHANNEL) {
      side = "L";
      ctr = &sMotorPidTraceCounterL;
    } else if (_pwmCh == AppConfig::Motors::RIGHT_PWM_CHANNEL) {
      side = "R";
      ctr = &sMotorPidTraceCounterR;
    }
    const uint16_t everyN = (AppConfig::Debug::MOTOR_PID_TRACE_EVERY_N == 0)
                           ? 1
                           : AppConfig::Debug::MOTOR_PID_TRACE_EVERY_N;
    bool shouldPrint = true;
    if (ctr != nullptr) {
      (*ctr)++;
      shouldPrint = ((*ctr % everyN) == 0);
    }
    if (shouldPrint) {
      char buf[180];
      snprintf(buf, sizeof(buf),
               "PID %s tgt=%.1f tps=%.1f err=%.2f P=%.4f I=%.4f D=%.4f out=%.4f duty=%ld",
               side, _targetTPS, _tps, err, pTerm, _iTerm, dTerm, out, (long)duty);
      if (_logFn != nullptr) {
        _logFn(String(buf));
      } else if (AppConfig::Debug::ENABLE_SERIAL_OUTPUT) {
        portENTER_CRITICAL(&sMotorPidTraceMux);
        Serial.println(buf);
        portEXIT_CRITICAL(&sMotorPidTraceMux);
      }
    }
  }
  applyDuty(duty);
}

int32_t DcMotor::getTicks() const {
  int32_t t;
  t = _ticks;
  return t;
}

void DcMotor::resetTicks(int32_t value) {
  _ticks = value;
  _lastTicks = value;
  _speedAccumTicks = 0;
  _speedAccumUs = 0;
}

// ---------------- Encoder ISR (simple, encA only) ----------------
// Attach interrupt on encA CHANGE. When it triggers, read A and B.
// Simple rule: step = (A == B) ? +1 : -1
// If direction is reversed, set invertEnc=true for that motor.
// If still wrong, change the rule to (A != B) ? +1 : -1.
void DcMotor::handleEncAChange() { // rename ok, still called by ISR

  bool B = gpio_get_level((gpio_num_t)_p.encB);

  // For A rising, direction depends on B (might need to flip)
  int8_t step = B ? -1 : +1;

  if (_p.invertEnc) step = -step;
  _ticks += step;
}


// ISR wrappers (only these have IRAM_ATTR)
void IRAM_ATTR DcMotor::isrEncA0(){ if(_instances[0]) _instances[0]->handleEncAChange(); }
void IRAM_ATTR DcMotor::isrEncA1(){ if(_instances[1]) _instances[1]->handleEncAChange(); }
void IRAM_ATTR DcMotor::isrEncA2(){ if(_instances[2]) _instances[2]->handleEncAChange(); }
void IRAM_ATTR DcMotor::isrEncA3(){ if(_instances[3]) _instances[3]->handleEncAChange(); }
````

### `DcMotor.h`

````cpp
#pragma once
#include <Arduino.h>
#include "driver/gpio.h"

class DcMotor {
public:
  using LogFn = void (*)(const String&);

  struct Pins {
    uint8_t in1;
    uint8_t in2;
    uint8_t pwm;
    uint8_t encA;
    uint8_t encB;
    bool invertDir = false;
    bool invertEnc = false; // flips tick direction
  };

  DcMotor();

  // ticksPerWheelRev: encoder ticks counted by THIS decoding method per 1 wheel revolution
  bool begin(const Pins& pins,
             uint8_t pwmChannel,
             uint32_t pwmFreq = 20000,
             uint8_t pwmResolutionBits = 10);

  // Direct power control [-1..+1]
  void setPower(float power);

  // Speed control in ticks/sec
  void setSpeedTPS(float tps);
  void enableSpeedControl(bool en);
  void coastStop();
  void brakeStop();
  void hardStop();

  // PID tuning (TPS units)
  // outLimit: max command magnitude (0..1)
  // iLimit: max integrator magnitude (0..outLimit)
  // dFilterHz: low-pass cutoff for derivative of measurement, 0 disables
  // slewRatePerSec: max output change per second, 0 disables
  void setSpeedPID(float kp, float ki, float kd,
                   float outLimit = 1.0f,
                   float iLimit = 0.6f,
                   float dFilterHz = 30.0f,
                   float slewRatePerSec = 0.0f);
  void setLog(LogFn fn) { _logFn = fn; }

  void resetPID();

  // Call periodically; dt is measured from micros() inside update().
  void update();

  // Encoder / speed
  int32_t getTicks() const;
  void    resetTicks(int32_t value = 0);
  float   getTicksPerSecond() const { return _tps; }

  // motor drive
  void applyDuty(int32_t duty);

  // ===== USAGE =====
  inline float ticksToMM() const {
    return _ticks * K_MM_PER_TICK;
  }

  inline float tpsToMMps() const {
    return _tps * K_MM_PER_TICK;
  }

private:
  Pins _p{};
  uint8_t _pwmCh = 0;
  uint32_t _pwmFreq = 20000;
  uint8_t _pwmResBits = 10;
  uint32_t _pwmMax = 1023;

  const int DEADZONE = 10;
  volatile int32_t _ticks = 0;

  // speed estimation
  uint32_t _lastMicros = 0;
  int32_t  _lastTicks = 0;
  float    _tps = 0.0f; // filtered ticks/sec
  int32_t  _speedAccumTicks = 0;
  uint32_t _speedAccumUs = 0;

  // control
  bool  _speedCtrlEnabled = false;
  float _targetTPS = 0.0f;

  // PID params
  float _kp = 0.0f, _ki = 0.0f, _kd = 0.0f;
  float _outLimit = 1.0f;
  float _iLimit = 0.6f;
  float _dFilterHz = 30.0f;
  float _slewRate = 0.0f; // output units/sec

  // PID internal
  float _iTerm = 0.0f;
  float _dMeas = 0.0f;       // filtered derivative of measurement (tps/s)
  float _lastTPSForD = 0.0f;
  float _lastOut = 0.0f;
  LogFn _logFn = nullptr;

  // ISR glue (encA only, max 4 motors)
  static void isrEncA0();
  static void isrEncA1();
  static void isrEncA2();
  static void isrEncA3();

  float K_MM_PER_TICK = 0.54f;  // <-- calibrated

  static DcMotor* _instances[4];
  int _slot = -1;

  // Called from ISR when encA changes
  void handleEncAChange();

  inline void pwmWriteDuty(uint32_t duty);

  static int allocSlot(DcMotor* m);

  static float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }
};
````

### `esp32s3.txt`

````text
PS C:\Users\donot\OneDrive\Documents\WorkSpace> C:\Users\donot\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.1.0\espefuse.exe --port COM4 summary
>>
espefuse v5.1.0
Connecting...
Detecting chip type... ESP32-S3

=== Run "summary" command ===
EFUSE_NAME (Block) Description  = [Meaningful Value] [Readable/Writeable] (Hex Value)
----------------------------------------------------------------------------------------
Calibration fuses:
K_RTC_LDO (BLOCK1)                                 BLOCK1 K_RTC_LDO                                   = -80 R/W (0b1010100)
K_DIG_LDO (BLOCK1)                                 BLOCK1 K_DIG_LDO                                   = 48 R/W (0b0001100)
V_RTC_DBIAS20 (BLOCK1)                             BLOCK1 voltage of rtc dbias20                      = -112 R/W (0x9c)
V_DIG_DBIAS20 (BLOCK1)                             BLOCK1 voltage of digital dbias20                  = 96 R/W (0x18)
DIG_DBIAS_HVT (BLOCK1)                             BLOCK1 digital dbias when hvt                      = -24 R/W (0b10110)
ADC2_CAL_VOL_ATTEN3 (BLOCK1)                       ADC2 calibration voltage at atten3                 = 96 R/W (0b011000)
TEMP_CALIB (BLOCK2)                                Temperature calibration data                       = -16.6 R/- (0b110100110)
OCODE (BLOCK2)                                     ADC OCode                                          = 76 R/- (0x4c)
ADC1_INIT_CODE_ATTEN0 (BLOCK2)                     ADC1 init code at atten0                           = -24 R/- (0x86)
ADC1_INIT_CODE_ATTEN1 (BLOCK2)                     ADC1 init code at atten1                           = 120 R/- (0b011110)
ADC1_INIT_CODE_ATTEN2 (BLOCK2)                     ADC1 init code at atten2                           = 104 R/- (0b011010)
ADC1_INIT_CODE_ATTEN3 (BLOCK2)                     ADC1 init code at atten3                           = 112 R/- (0b011100)
ADC2_INIT_CODE_ATTEN0 (BLOCK2)                     ADC2 init code at atten0                           = -120 R/- (0x9e)
ADC2_INIT_CODE_ATTEN1 (BLOCK2)                     ADC2 init code at atten1                           = -20 R/- (0b100101)
ADC2_INIT_CODE_ATTEN2 (BLOCK2)                     ADC2 init code at atten2                           = 48 R/- (0b001100)
ADC2_INIT_CODE_ATTEN3 (BLOCK2)                     ADC2 init code at atten3                           = 120 R/- (0b011110)
ADC1_CAL_VOL_ATTEN0 (BLOCK2)                       ADC1 calibration voltage at atten0                 = 236 R/- (0x3b)
ADC1_CAL_VOL_ATTEN1 (BLOCK2)                       ADC1 calibration voltage at atten1                 = 276 R/- (0x45)
ADC1_CAL_VOL_ATTEN2 (BLOCK2)                       ADC1 calibration voltage at atten2                 = 216 R/- (0x36)
ADC1_CAL_VOL_ATTEN3 (BLOCK2)                       ADC1 calibration voltage at atten3                 = 256 R/- (0x40)
ADC2_CAL_VOL_ATTEN0 (BLOCK2)                       ADC2 calibration voltage at atten0                 = 316 R/- (0x4f)
ADC2_CAL_VOL_ATTEN1 (BLOCK2)                       ADC2 calibration voltage at atten1                 = 132 R/- (0b0100001)
ADC2_CAL_VOL_ATTEN2 (BLOCK2)                       ADC2 calibration voltage at atten2                 = 192 R/- (0b0110000)

Config fuses:
WR_DIS (BLOCK0)                                    Disable programming of individual eFuses           = 741116963 R/W (0x2c2c8c23)
RD_DIS (BLOCK0)                                    Disable reading from BlOCK4-10                     = 0 R/- (0b0000000)
DIS_ICACHE (BLOCK0)                                Set this bit to disable Icache                     = False R/W (0b0)
DIS_DCACHE (BLOCK0)                                Set this bit to disable Dcache                     = False R/W (0b0)
DIS_TWAI (BLOCK0)                                  Set this bit to disable CAN function               = False R/W (0b0)
DIS_APP_CPU (BLOCK0)                               Disable app cpu                                    = False R/W (0b0)
DIS_DIRECT_BOOT (BLOCK0)                           Disable direct boot mode                           = False R/- (0b0)
UART_PRINT_CONTROL (BLOCK0)                        Set the default UART boot message output mode      = Enable R/- (0b00)
PIN_POWER_SELECTION (BLOCK0)                       Set default power supply for GPIO33-GPIO37; set wh = VDD3P3_CPU R/- (0b0)
                                                   en SPI flash is initialized
PSRAM_CAP (BLOCK1)                                 PSRAM capacity                                     = 2M R/W (0b10)
PSRAM_TEMP (BLOCK1)                                PSRAM temperature                                  = 105C R/W (0b01)
PSRAM_VENDOR (BLOCK1)                              PSRAM vendor                                       = AP_3v3 R/W (0b01)
PSRAM_CAP_3 (BLOCK1)                               PSRAM capacity bit 3                               = False R/W (0b0)
BLOCK_USR_DATA (BLOCK3)                            User data
   = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 R/W
BLOCK_SYS_DATA2 (BLOCK10)                          System data part 2 (reserved)
   = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 R/-

Flash fuses:
FLASH_TPUW (BLOCK0)                                Configures flash waiting time after power-up; in u = 0 R/- (0x0)
                                                   nit of ms. If the value is less than 15; the waiti
                                                   ng time is the configurable value.  Otherwise; the
                                                    waiting time is twice the configurable value
FLASH_ECC_MODE (BLOCK0)                            Flash ECC mode in ROM                              = 16to18 byte R/- (0b0)
FLASH_TYPE (BLOCK0)                                SPI flash type                                     = 4 data lines R/- (0b0)
FLASH_PAGE_SIZE (BLOCK0)                           Set Flash page size                                = 0 R/- (0b00)
FLASH_ECC_EN (BLOCK0)                              Set 1 to enable ECC for flash boot                 = False R/- (0b0)
FORCE_SEND_RESUME (BLOCK0)                         Set this bit to force ROM code to send a resume co = False R/- (0b0)
                                                   mmand during SPI boot
FLASH_CAP (BLOCK1)                                 Flash capacity                                     = 4M R/W (0b010)
FLASH_TEMP (BLOCK1)                                Flash temperature                                  = 105C R/W (0b01)
FLASH_VENDOR (BLOCK1)                              Flash vendor                                       = XMC R/W (0b001)

Identity fuses:
DISABLE_WAFER_VERSION_MAJOR (BLOCK0)               Disables check of wafer version major              = False R/- (0b0)
DISABLE_BLK_VERSION_MAJOR (BLOCK0)                 Disables check of blk version major                = False R/- (0b0)
WAFER_VERSION_MINOR_LO (BLOCK1)                    WAFER_VERSION_MINOR least significant bits         = 2 R/W (0b010)
PKG_VERSION (BLOCK1)                               Package version                                    = 0 R/W (0b000)
BLK_VERSION_MINOR (BLOCK1)                         BLK_VERSION_MINOR                                  = 4 R/W (0b100)
WAFER_VERSION_MINOR_HI (BLOCK1)                    WAFER_VERSION_MINOR most significant bit           = False R/W (0b0)
WAFER_VERSION_MAJOR (BLOCK1)                       WAFER_VERSION_MAJOR                                = 0 R/W (0b00)
OPTIONAL_UNIQUE_ID (BLOCK2)                        Optional unique 128-bit ID
   = 62 d7 6c 68 f1 37 dc c9 47 61 5b 50 ca c0 4a bf R/-
BLK_VERSION_MAJOR (BLOCK2)                         BLK_VERSION_MAJOR of BLOCK2                        = ADC calib V1 R/- (0b01)
WAFER_VERSION_MINOR (BLOCK0)                       calc WAFER VERSION MINOR = WAFER_VERSION_MINOR_HI  = 2 R/W (0x2)
                                                   << 3 + WAFER_VERSION_MINOR_LO (read only)
PSRAM_CAPACITY (BLOCK0)                            calc as = PSRAM_CAP_3 << 2 + PSRAM_CAP (read only) = 2 R/W (0b010)

Jtag fuses:
SOFT_DIS_JTAG (BLOCK0)                             Set these bits to disable JTAG in the soft way (od = 0 R/W (0b000)
                                                   d number 1 means disable ). JTAG can be enabled in
                                                    HMAC module
DIS_PAD_JTAG (BLOCK0)                              Set this bit to disable JTAG in the hard way. JTAG = False R/W (0b0)
                                                    is disabled permanently
STRAP_JTAG_SEL (BLOCK0)                            Set this bit to enable selection between usb_to_jt = False R/W (0b0)
                                                   ag and pad_to_jtag through strapping gpio3 when bo
                                                   th reg_dis_usb_jtag and reg_dis_pad_jtag are equal
                                                    to 0

Mac fuses:
MAC (BLOCK1)                                       MAC address
   = 1c:db:d4:83:73:c8 (OK) R/W
CUSTOM_MAC (BLOCK3)                                Custom MAC
   = 00:00:00:00:00:00 (OK) R/W

Security fuses:
DIS_DOWNLOAD_ICACHE (BLOCK0)                       Set this bit to disable Icache in download mode (b = False R/W (0b0)
                                                   oot_mode[3:0] is 0; 1; 2; 3; 6; 7)
DIS_DOWNLOAD_DCACHE (BLOCK0)                       Set this bit to disable Dcache in download mode (  = False R/W (0b0)
                                                   boot_mode[3:0] is 0; 1; 2; 3; 6; 7)
DIS_FORCE_DOWNLOAD (BLOCK0)                        Set this bit to disable the function that forces c = False R/W (0b0)
                                                   hip into download mode
DIS_DOWNLOAD_MANUAL_ENCRYPT (BLOCK0)               Set this bit to disable flash encryption when in d = False R/W (0b0)
                                                   ownload boot modes
SPI_BOOT_CRYPT_CNT (BLOCK0)                        Enables flash encryption when 1 or 3 bits are set  = Enable R/W (0b111)
                                                   and disabled otherwise
SECURE_BOOT_KEY_REVOKE0 (BLOCK0)                   Revoke 1st secure boot key                         = False R/- (0b0)
SECURE_BOOT_KEY_REVOKE1 (BLOCK0)                   Revoke 2nd secure boot key                         = True R/W (0b1)
SECURE_BOOT_KEY_REVOKE2 (BLOCK0)                   Revoke 3rd secure boot key                         = False R/W (0b0)
KEY_PURPOSE_0 (BLOCK0)                             Purpose of Key0                                    = USER R/W (0x0)
KEY_PURPOSE_1 (BLOCK0)                             Purpose of Key1                                    = USER R/W (0x0)
KEY_PURPOSE_2 (BLOCK0)                             Purpose of Key2                                    = USER R/- (0x0)
KEY_PURPOSE_3 (BLOCK0)                             Purpose of Key3                                    = USER R/- (0x0)
KEY_PURPOSE_4 (BLOCK0)                             Purpose of Key4                                    = SECURE_BOOT_DIGEST1 R/W (0xa)
KEY_PURPOSE_5 (BLOCK0)                             Purpose of Key5                                    = USER R/W (0x0)
SECURE_BOOT_EN (BLOCK0)                            Set this bit to enable secure boot                 = False R/- (0b0)
SECURE_BOOT_AGGRESSIVE_REVOKE (BLOCK0)             Set this bit to enable revoking aggressive secure  = False R/W (0b0)
                                                   boot
DIS_DOWNLOAD_MODE (BLOCK0)                         Set this bit to disable download mode (boot_mode[3 = False R/- (0b0)
                                                   :0] = 0; 1; 2; 3; 6; 7)
ENABLE_SECURITY_DOWNLOAD (BLOCK0)                  Set this bit to enable secure UART download mode   = False R/- (0b0)
SECURE_VERSION (BLOCK0)                            Secure version (used by ESP-IDF anti-rollback feat = 0 R/- (0x0000)
                                                   ure)
BLOCK_KEY0 (BLOCK4)
  Purpose: USER
               Key0 or user data
   = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 R/W
BLOCK_KEY1 (BLOCK5)
  Purpose: USER
               Key1 or user data
   = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 R/W
BLOCK_KEY2 (BLOCK6)
  Purpose: USER
               Key2 or user data
   = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 R/W
BLOCK_KEY3 (BLOCK7)
  Purpose: USER
               Key3 or user data
   = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 R/-
BLOCK_KEY4 (BLOCK8)
  Purpose: SECURE_BOOT_DIGEST1
  Key4 or user data
   = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 R/-
BLOCK_KEY5 (BLOCK9)
  Purpose: USER
               Key5 or user data
   = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 R/W

Spi Pad fuses:
SPI_PAD_CONFIG_CLK (BLOCK1)                        SPI_PAD_configure CLK                              = 0 R/W (0b000000)
SPI_PAD_CONFIG_Q (BLOCK1)                          SPI_PAD_configure Q(D1)                            = 0 R/W (0b000000)
SPI_PAD_CONFIG_D (BLOCK1)                          SPI_PAD_configure D(D0)                            = 0 R/W (0b000000)
SPI_PAD_CONFIG_CS (BLOCK1)                         SPI_PAD_configure CS                               = 0 R/W (0b000000)
SPI_PAD_CONFIG_HD (BLOCK1)                         SPI_PAD_configure HD(D3)                           = 0 R/W (0b000000)
SPI_PAD_CONFIG_WP (BLOCK1)                         SPI_PAD_configure WP(D2)                           = 0 R/W (0b000000)
SPI_PAD_CONFIG_DQS (BLOCK1)                        SPI_PAD_configure DQS                              = 0 R/W (0b000000)
SPI_PAD_CONFIG_D4 (BLOCK1)                         SPI_PAD_configure D4                               = 0 R/W (0b000000)
SPI_PAD_CONFIG_D5 (BLOCK1)                         SPI_PAD_configure D5                               = 0 R/W (0b000000)
SPI_PAD_CONFIG_D6 (BLOCK1)                         SPI_PAD_configure D6                               = 0 R/W (0b000000)
SPI_PAD_CONFIG_D7 (BLOCK1)                         SPI_PAD_configure D7                               = 0 R/W (0b000000)

Usb fuses:
DIS_USB_OTG (BLOCK0)                               Set this bit to disable USB function               = False R/W (0b0)
USB_EXCHG_PINS (BLOCK0)                            Set this bit to exchange USB D+ and D- pins        = False R/W (0b0)
USB_EXT_PHY_ENABLE (BLOCK0)                        Set this bit to enable external PHY                = False R/W (0b0)
DIS_USB_JTAG (BLOCK0)                              Set this bit to disable function of usb switch to  = False R/W (0b0)
                                                   jtag in module of usb device
DIS_USB_SERIAL_JTAG (BLOCK0)                       Set this bit to disable usb device                 = False R/W (0b0)
USB_PHY_SEL (BLOCK0)                               This bit is used to switch internal PHY and extern
   = internal PHY is assigned to USB Device while external PHY is assigned to USB OTG R/W (0b0)
                                                   al PHY for USB OTG and USB Device
DIS_USB_SERIAL_JTAG_ROM_PRINT (BLOCK0)             USB printing                                       = Enable R/- (0b0)
DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE (BLOCK0)         Set this bit to disable UART download mode through = False R/- (0b0)
                                                    USB
DIS_USB_OTG_DOWNLOAD_MODE (BLOCK0)                 Set this bit to disable download through USB-OTG   = False R/- (0b0)

Vdd fuses:
VDD_SPI_XPD (BLOCK0)                               SPI regulator power up signal                      = True R/W (0b1)
VDD_SPI_TIEH (BLOCK0)                              If VDD_SPI_FORCE is 1; determines VDD_SPI voltage
   = VDD_SPI connects to VDD3P3_RTC_IO R/W (0b1)
VDD_SPI_FORCE (BLOCK0)                             Set this bit and force to use the configuration of = True R/W (0b1)
                                                    eFuse to configure VDD_SPI

Wdt fuses:
WDT_DELAY_SEL (BLOCK0)                             RTC watchdog timeout threshold; in unit of slow cl = 40000 R/W (0b00)
                                                   ock cycle

Flash voltage (VDD_SPI) set to 3.3V by efuse.
PS C:\Users\donot\OneDrive\Documents\WorkSpace> 
````

### `FloodFillExplorer.cpp`

````cpp
#include "FloodFillExplorer.h"
#include <WiFi.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha1.h>

// ============================ HTML UI ============================
// Improvements:
// - fetchWithTimeout() + AbortController to abort hung requests (1~2s)
// - stateInFlight prevents overlapping /state calls
// - 204 still runs doNextIfNeeded + autoAckIfEnabled (state machine keeps going)
// - seq snapshot for autoAck to avoid 409 seq mismatch
// - configurable poll/timeout/delay constants

static const char* kHtml PROGMEM = R"HTML(
<!doctype html><html>
<head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Flood Fill Explorer (ACK)</title>
<style>
  body{font-family:system-ui,Segoe UI,Arial;margin:0}
  header{background:#111;color:#eee;padding:10px 12px;position:sticky;top:0}
  .row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
  button{padding:8px 12px;cursor:pointer}
  canvas{display:block;margin:0 auto;border-top:1px solid #333}
  small{opacity:.85}
  .pill{padding:2px 8px;border-radius:999px;background:rgba(255,255,255,.12)}
  .warn{background:rgba(255,140,0,.18)}
  .statusbar{background:#f4f6f8;border-bottom:1px solid #d8dde3;padding:8px 12px;
             font:13px/1.45 ui-monospace,Consolas,monospace;white-space:normal;
             overflow-wrap:anywhere;word-break:break-word}
  .lapbar{background:#eef3f6;border-bottom:1px solid #d8dde3;padding:8px 12px;
          font:13px/1.5 ui-monospace,Consolas,monospace}
  .lapbar b{display:block;margin-bottom:4px}
  .lapbar ul{margin:6px 0 0;padding-left:20px}
  .lapbar li{margin:2px 0}
</style>
</head>
<body>
<header>
  <div class="row">
    <b>Flood Fill Explorer</b>
    <small id="conn" class="pill">loading...</small>

    <div style="margin-left:auto" class="row">
      <button onclick="cmd('step')">Step</button>
      <button onclick="cmd('run')">Run</button>
      <button onclick="cmd('pause')">Pause</button>
      <button onclick="cmd('reset')">Reset</button>
      <button onclick="hwcmd('clearmaze')">Clear Maze</button>
      <button onclick="hwcmd('srun 1')">SRun1</button>

      <span class="pill warn">
        AutoACK(sim)
        <input id="autoAck" type="checkbox" style="transform:scale(1.2);margin-left:6px"/>
      </span>
    </div>
  </div>
</header>

<div id="status" class="statusbar">Connecting to explorer...</div>
  <div id="lapbar" class="lapbar">
    <b>Run Timing</b>
  <div id="lapCurrent">Waiting to start...</div>
  <ul id="lapHistory"></ul>
</div>

<canvas id="c" width="720" height="720"></canvas>

<script>
const N=16;
const WS_PORT=%WS_PORT%;
const c=document.getElementById('c');
const ctx=c.getContext('2d');
let S=null;
let ws=null;
let busy=false;
let stopLoop=false;
let nextInFlight=false;
let ackInFlight=false;
let lapStateRxMs = 0;

const TICK_MS = 50;
const AUTO_ACK_DELAY_MS = 50;

function st(s){
  const el=document.getElementById('status');
  el.textContent=s;
}

function setConn(s){
  const el=document.getElementById('conn');
  el.textContent=s;
}

function cmd(a){
  if(busy || !wsReady()) return;
  busy = true;
  const autoAck = document.getElementById('autoAck').checked;
  sendWs((autoAck ? 'cmd|' : 'hwcmd|') + a);
}

function hwcmd(a){
  if(busy || !wsReady()) return;
  busy = true;
  sendWs('hwcmd|' + a);
}

function cellGeom(){
  const W=c.width, H=c.height;
  const pad=20;
  const grid=Math.min(W,H)-pad*2;
  const cs=grid/N;
  const ox=(W-grid)/2, oy=(H-grid)/2;
  return {W,H,pad,grid,cs,ox,oy};
}

function cellCenter(x,y,geom){
  const {cs,ox,oy}=geom;
  return {cx: ox + x*cs + cs*0.5, cy: oy + y*cs + cs*0.5};
}

function drawPlanPath(geom, plan){
  if(!plan || plan.length < 2) return;
  const {cs}=geom;

  ctx.save();
  ctx.lineWidth = Math.max(1.2, cs*0.040);
  ctx.strokeStyle = 'rgba(255, 130, 0, 0.65)';
  ctx.setLineDash([cs*0.16, cs*0.11]);

  ctx.beginPath();
  for(let i=0;i<plan.length;i++){
    const a = plan[i];
    const b = plan[i+1] || a;

    const ca = cellCenter(a.x, a.y, geom);
    const cb = cellCenter(b.x, b.y, geom);

    const sdx = Math.sign(b.x - a.x);
    const sdy = Math.sign(b.y - a.y);

    const off = cs * 0.22;
    const ax = ca.cx + sdy * off;
    const ay = ca.cy - sdx * off;
    const bx = cb.cx + sdy * off;
    const by = cb.cy - sdx * off;

    if(i === 0) ctx.moveTo(ax, ay);
    else ctx.lineTo(ax, ay);

    if(i === plan.length-1) ctx.lineTo(bx, by);
  }
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.restore();
}

function draw(){
  if(!S) return;
  const g = cellGeom();
  const {cs,ox,oy}=g;
  ctx.clearRect(0,0,c.width,c.height);

  const gx0=S.goal.x0, gy0=S.goal.y0, gw=S.goal.w, gh=S.goal.h;
  ctx.fillStyle='rgba(0,0,0,0.06)';
  ctx.fillRect(ox+gx0*cs, oy+gy0*cs, gw*cs, gh*cs);

  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      if(S.visited[y][x]){
        ctx.fillStyle='rgba(0,0,0,0.03)';
        ctx.fillRect(ox+x*cs, oy+y*cs, cs, cs);
      }
    }
  }

  ctx.strokeStyle='rgba(0,0,0,0.08)';
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      ctx.strokeRect(ox+x*cs, oy+y*cs, cs, cs);
    }
  }

  ctx.textAlign='center';
  ctx.textBaseline='middle';
  ctx.font=Math.floor(cs*0.35)+'px ui-monospace,monospace';
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      const d=S.dist[y][x];
      if(d!==65535){
        ctx.fillStyle = S.visited[y][x] ? 'rgba(0,0,0,0.55)' : 'rgba(0,0,0,0.25)';
        ctx.fillText(String(d), ox+x*cs+cs*0.5, oy+y*cs+cs*0.5);
      }
    }
  }

  ctx.strokeStyle='rgba(0,0,0,0.9)';
  ctx.lineWidth=Math.max(2, cs*0.08);
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      const w=S.knownWalls[y][x];
      const m=S.knownMask[y][x];
      const px=ox+x*cs, py=oy+y*cs;

      if((m&1) && (w&1)){ ctx.beginPath(); ctx.moveTo(px,py); ctx.lineTo(px+cs,py); ctx.stroke(); }
      if((m&2) && (w&2)){ ctx.beginPath(); ctx.moveTo(px+cs,py); ctx.lineTo(px+cs,py+cs); ctx.stroke(); }
      if((m&4) && (w&4)){ ctx.beginPath(); ctx.moveTo(px,py+cs); ctx.lineTo(px+cs,py+cs); ctx.stroke(); }
      if((m&8) && (w&8)){ ctx.beginPath(); ctx.moveTo(px,py); ctx.lineTo(px,py+cs); ctx.stroke(); }
    }
  }

  if(S.plan) drawPlanPath(g, S.plan);

  const mx=S.mouse.x, my=S.mouse.y, h=S.mouse.h;
  const cc = cellCenter(mx,my,g);
  const cx=cc.cx, cy=cc.cy;

  ctx.fillStyle = 'rgba(0, 0, 0, 0.18)';
  ctx.beginPath();
  ctx.arc(cx, cy, cs*0.30, 0, Math.PI*2);
  ctx.fill();

  ctx.strokeStyle = 'rgba(0, 0, 0, 0.45)';
  ctx.lineWidth = Math.max(1.5, cs*0.05);
  ctx.beginPath();
  ctx.arc(cx, cy, cs*0.30, 0, Math.PI*2);
  ctx.stroke();

  const ax=[0,1,0,-1], ay=[-1,0,1,0];
  ctx.strokeStyle='rgba(0,0,0,0.35)';
  ctx.lineWidth=Math.max(2.2, cs*0.065);
  ctx.beginPath();
  ctx.moveTo(cx,cy);
  ctx.lineTo(cx + ax[h]*cs*0.42, cy + ay[h]*cs*0.42);
  ctx.stroke();

  const cur = S.dist[my][mx];
  const pa = S.pendingActionName || '-';
  const home = S.home ? `home=(${S.home.x0},${S.home.y0}) ${S.home.w}x${S.home.h}` : 'home=-';
  const goal = S.goal ? `goal=(${S.goal.x0},${S.goal.y0}) ${S.goal.w}x${S.goal.h}` : 'goal=-';
  st('mouse=('+mx+','+my+') h='+h+
     ' | running='+S.running+
     ' | waitAck='+S.waitAck+
     ' | seq='+(S.pendingSeq||0)+
     ' | act='+pa+
     ' | dist='+cur+
     ' | planLen='+(S.planLen||0)+
     ' | '+home+
     ' | '+goal+
     ' | ver='+(S.ver||0));
  renderLap();
}

function wsReady(){
  return ws && ws.readyState === WebSocket.OPEN;
}

function sendWs(msg){
  if(!wsReady()) return false;
  ws.send(msg);
  return true;
}

function cmd(a){
  if(busy || !wsReady()) return;
  busy = true;
  const autoAck = document.getElementById('autoAck').checked;
  sendWs((autoAck ? 'cmd|' : 'hwcmd|') + a);
}

function hwcmd(a){
  if(busy || !wsReady()) return;
  busy = true;
  sendWs('hwcmd|' + a);
}

function cellGeom(){
  const W=c.width, H=c.height;
  const pad=20;
  const grid=Math.min(W,H)-pad*2;
  const cs=grid/N;
  const ox=(W-grid)/2, oy=(H-grid)/2;
  return {W,H,pad,grid,cs,ox,oy};
}

function cellCenter(x,y,geom){
  const {cs,ox,oy}=geom;
  return {cx: ox + x*cs + cs*0.5, cy: oy + y*cs + cs*0.5};
}

function drawPlanPath(geom, plan){
  if(!plan || plan.length < 2) return;
  const {cs}=geom;

  ctx.save();
  ctx.lineWidth = Math.max(1.2, cs*0.040);
  ctx.strokeStyle = 'rgba(255, 130, 0, 0.65)';
  ctx.setLineDash([cs*0.16, cs*0.11]);

  ctx.beginPath();
  for(let i=0;i<plan.length;i++){
    const a = plan[i];
    const b = plan[i+1] || a;

    const ca = cellCenter(a.x, a.y, geom);
    const cb = cellCenter(b.x, b.y, geom);

    const sdx = Math.sign(b.x - a.x);
    const sdy = Math.sign(b.y - a.y);

    const off = cs * 0.22;
    const ax = ca.cx + sdy * off;
    const ay = ca.cy - sdx * off;
    const bx = cb.cx + sdy * off;
    const by = cb.cy - sdx * off;

    if(i === 0) ctx.moveTo(ax, ay);
    else ctx.lineTo(ax, ay);

    if(i === plan.length-1) ctx.lineTo(bx, by);
  }
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.restore();
}

function draw(){
  if(!S) return;
  const g = cellGeom();
  const {cs,ox,oy}=g;
  ctx.clearRect(0,0,c.width,c.height);

  const gx0=S.goal.x0, gy0=S.goal.y0, gw=S.goal.w, gh=S.goal.h;
  ctx.fillStyle='rgba(0,0,0,0.06)';
  ctx.fillRect(ox+gx0*cs, oy+gy0*cs, gw*cs, gh*cs);

  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      if(S.visited[y][x]){
        ctx.fillStyle='rgba(0,0,0,0.03)';
        ctx.fillRect(ox+x*cs, oy+y*cs, cs, cs);
      }
    }
  }

  ctx.strokeStyle='rgba(0,0,0,0.08)';
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      ctx.strokeRect(ox+x*cs, oy+y*cs, cs, cs);
    }
  }

  ctx.textAlign='center';
  ctx.textBaseline='middle';
  ctx.font=Math.floor(cs*0.35)+'px ui-monospace,monospace';
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      const d=S.dist[y][x];
      if(d!==65535){
        ctx.fillStyle = S.visited[y][x] ? 'rgba(0,0,0,0.55)' : 'rgba(0,0,0,0.25)';
        ctx.fillText(String(d), ox+x*cs+cs*0.5, oy+y*cs+cs*0.5);
      }
    }
  }

  ctx.strokeStyle='rgba(0,0,0,0.9)';
  ctx.lineWidth=Math.max(2, cs*0.08);
  for(let y=0;y<N;y++){
    for(let x=0;x<N;x++){
      const w=S.knownWalls[y][x];
      const m=S.knownMask[y][x];
      const px=ox+x*cs, py=oy+y*cs;

      if((m&1) && (w&1)){ ctx.beginPath(); ctx.moveTo(px,py); ctx.lineTo(px+cs,py); ctx.stroke(); }
      if((m&2) && (w&2)){ ctx.beginPath(); ctx.moveTo(px+cs,py); ctx.lineTo(px+cs,py+cs); ctx.stroke(); }
      if((m&4) && (w&4)){ ctx.beginPath(); ctx.moveTo(px,py+cs); ctx.lineTo(px+cs,py+cs); ctx.stroke(); }
      if((m&8) && (w&8)){ ctx.beginPath(); ctx.moveTo(px,py); ctx.lineTo(px,py+cs); ctx.stroke(); }
    }
  }

  if(S.plan) drawPlanPath(g, S.plan);

  const mx=S.mouse.x, my=S.mouse.y, h=S.mouse.h;
  const cc = cellCenter(mx,my,g);
  const cx=cc.cx, cy=cc.cy;

  ctx.fillStyle = 'rgba(0, 0, 0, 0.18)';
  ctx.beginPath();
  ctx.arc(cx, cy, cs*0.30, 0, Math.PI*2);
  ctx.fill();

  ctx.strokeStyle = 'rgba(0, 0, 0, 0.45)';
  ctx.lineWidth = Math.max(1.5, cs*0.05);
  ctx.beginPath();
  ctx.arc(cx, cy, cs*0.30, 0, Math.PI*2);
  ctx.stroke();

  const ax=[0,1,0,-1], ay=[-1,0,1,0];
  ctx.strokeStyle='rgba(0,0,0,0.35)';
  ctx.lineWidth=Math.max(2.2, cs*0.065);
  ctx.beginPath();
  ctx.moveTo(cx,cy);
  ctx.lineTo(cx + ax[h]*cs*0.42, cy + ay[h]*cs*0.42);
  ctx.stroke();

  const cur = S.dist[my][mx];
  const pa = S.pendingActionName || '-';
  const home = S.home ? `home=(${S.home.x0},${S.home.y0}) ${S.home.w}x${S.home.h}` : 'home=-';
  const goal = S.goal ? `goal=(${S.goal.x0},${S.goal.y0}) ${S.goal.w}x${S.goal.h}` : 'goal=-';
  st('mouse=('+mx+','+my+') h='+h+
     ' | running='+S.running+
     ' | waitAck='+S.waitAck+
     ' | seq='+(S.pendingSeq||0)+
     ' | act='+pa+
     ' | dist='+cur+
     ' | planLen='+(S.planLen||0)+
     ' | '+home+
     ' | '+goal+
     ' | ver='+(S.ver||0));
  renderLap();
}

function fmtMs(ms){
  const total = Math.max(0, Math.floor(ms || 0));
  const min = Math.floor(total / 60000);
  const sec = Math.floor((total % 60000) / 1000);
  const rem = total % 1000;
  return `${min}:${String(sec).padStart(2,'0')}.${String(rem).padStart(3,'0')}`;
}

  function renderLap(){
    const currentEl = document.getElementById('lapCurrent');
    const listEl = document.getElementById('lapHistory');
  if(!currentEl || !listEl) return;
  listEl.innerHTML = '';

    if(!S || !S.lap){
      currentEl.textContent = 'Waiting to start...';
      return;
    }

    let currentMs = S.lap.currentMs || 0;
    const currentLabel = S.lap.currentLabel || 'HG';
    if(S.lap.running){
      currentMs += Math.max(0, Date.now() - lapStateRxMs);
      currentEl.textContent = `Leg ${S.lap.nextLap} (${currentLabel}) running: ${fmtMs(currentMs)}`;
    } else if((S.lap.history || []).length > 0){
      const last = S.lap.history[S.lap.history.length - 1];
      currentEl.textContent = `Last leg (${last.label || 'HG'}): ${fmtMs(last.ms || 0)}`;
    } else {
      currentEl.textContent = 'Waiting to start...';
    }

    (S.lap.history || []).forEach((entry, idx) => {
      const li = document.createElement('li');
      li.textContent = `Leg ${idx + 1} (${entry.label || 'HG'}): ${fmtMs(entry.ms || 0)}`;
      listEl.appendChild(li);
    });
  }

function doNextIfNeeded(){
  if(!S || !S.running || S.waitAck) return;
  if(nextInFlight || busy || ackInFlight || !wsReady()) return;
  nextInFlight = sendWs('next');
}

function autoAckIfEnabled(){
  if(!S || !S.running || !S.waitAck) return;
  if(ackInFlight || busy || !wsReady()) return;
  const autoAck = document.getElementById('autoAck').checked;
  if(!autoAck) return;
  ackInFlight = true;
  const seqSnapshot = S.pendingSeq;
  setTimeout(()=>{
    if(!wsReady()){
      ackInFlight = false;
      return;
    }
    sendWs('ack|' + String(seqSnapshot) + '|1');
  }, AUTO_ACK_DELAY_MS);
}

function tickLoop(){
  if(!stopLoop){
    doNextIfNeeded();
    autoAckIfEnabled();
    setTimeout(tickLoop, TICK_MS);
  }
}

function handleWsMessage(text){
  if(text.startsWith('state|')){
    S = JSON.parse(text.slice(6));
    lapStateRxMs = Date.now();
    nextInFlight = false;
    ackInFlight = false;
    busy = false;
    draw();
    return;
  }
  if(text.startsWith('reply|')){
    st(text.slice(6));
    nextInFlight = false;
    ackInFlight = false;
    busy = false;
    return;
  }
  if(text.startsWith('error|')){
    st(text.slice(6));
    nextInFlight = false;
    ackInFlight = false;
    busy = false;
  }
}

function connectWs(){
  const proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
  const host = location.hostname || window.location.host.split(':')[0];
  ws = new WebSocket(proto + host + ':' + WS_PORT + '/ws');
  ws.onopen = () => {
    setConn('ws connected');
    st('Explorer websocket connected.');
    sendWs('hello');
  };
  ws.onmessage = (ev) => handleWsMessage(String(ev.data || ''));
  ws.onclose = () => {
    setConn('ws disconnected');
    st('Explorer websocket disconnected. Retrying...');
    nextInFlight = false;
    ackInFlight = false;
    busy = false;
    setTimeout(connectWs, 1000);
  };
  ws.onerror = () => {
    setConn('ws error');
  };
}

connectWs();
tickLoop();
setInterval(renderLap, 200);
</script>
</body></html>
)HTML";

class FloodFillExplorer::WsServerWrapper {
public:
  explicit WsServerWrapper(uint16_t port) : server(port) {}

  WiFiServer server;
  WiFiClient client;
  bool handshaken = false;
  uint32_t lastStateVerSent = 0xFFFFFFFFu;
};

// ----------------- Core constants -----------------
static const int dx4[4] = {0, 1, 0, -1};
static const int dy4[4] = {-1, 0, 1, 0};

static inline uint8_t clampMask(uint8_t v){ return v & 0x0F; }

// ========================= Optional helpers for truth walls =========================
static void forceBoundaries16(uint8_t w[FloodFillExplorer::N][FloodFillExplorer::N]){
  const int N = FloodFillExplorer::N;
  for(int x=0;x<N;x++){
    w[0][x]   |= FloodFillExplorer::WALL_N;
    w[N-1][x] |= FloodFillExplorer::WALL_S;
  }
  for(int y=0;y<N;y++){
    w[y][0]   |= FloodFillExplorer::WALL_W;
    w[y][N-1] |= FloodFillExplorer::WALL_E;
  }
}

static void normalizePairs16(uint8_t w[FloodFillExplorer::N][FloodFillExplorer::N]){
  const int N = FloodFillExplorer::N;
  for(int y=0;y<N;y++){
    for(int x=0;x<N;x++){
      uint8_t a = w[y][x];
      if(a & FloodFillExplorer::WALL_N){ if(y>0)   w[y-1][x] |= FloodFillExplorer::WALL_S; }
      if(a & FloodFillExplorer::WALL_E){ if(x<N-1) w[y][x+1] |= FloodFillExplorer::WALL_W; }
      if(a & FloodFillExplorer::WALL_S){ if(y<N-1) w[y+1][x] |= FloodFillExplorer::WALL_N; }
      if(a & FloodFillExplorer::WALL_W){ if(x>0)   w[y][x-1] |= FloodFillExplorer::WALL_E; }
    }
  }
}

// ========================= Impl =========================

FloodFillExplorer::FloodFillExplorer() {}

FloodFillExplorer::~FloodFillExplorer(){
  if(server_){
    delete server_;
    server_ = nullptr;
  }
  if(ws_){
    delete ws_;
    ws_ = nullptr;
  }
}

void FloodFillExplorer::log_(const String& s){
  if(logFn_) logFn_(s);
}

uint8_t FloodFillExplorer::bitForDir_(Dir d) const{
  switch(d){
    case NORTH: return WALL_N;
    case EAST:  return WALL_E;
    case SOUTH: return WALL_S;
    case WEST:  return WALL_W;
  }
  return WALL_N;
}

FloodFillExplorer::Dir FloodFillExplorer::opposite_(Dir d) const{
  return (Dir)(((uint8_t)d + 2) & 3);
}

bool FloodFillExplorer::inBounds_(int x,int y) const{
  return (x>=0 && x<N && y>=0 && y<N);
}

bool FloodFillExplorer::isGoal_(int x,int y) const{
  return (x >= gx0_ && x < (int)(gx0_ + gw_) &&
          y >= gy0_ && y < (int)(gy0_ + gh_));
}

bool FloodFillExplorer::atActiveTarget_() const{
  if(!isGoal_(mx_, my_)) return false;
  if(!targetHome_) return true;
  return mh_ == origSh_;
}

bool FloodFillExplorer::atActiveTargetPose_(uint8_t x, uint8_t y, Dir h) const {
  if(!isGoal_(x, y)) return false;
  if(!targetHome_) return true;
  return h == origSh_;
}

bool FloodFillExplorer::isKnownOpen_(int x, int y, Dir d) const {
  if (!inBounds_(x, y)) return false;
  bool known = false;
  bool wall = false;
  getKnownWall((uint8_t)x, (uint8_t)y, d, known, wall);
  return known && !wall;
}

uint16_t FloodFillExplorer::computeBestKnownCost_(uint8_t startX0, uint8_t startY0,
                                                  uint8_t startW, uint8_t startH,
                                                  uint8_t goalX0, uint8_t goalY0,
                                                  uint8_t goalW, uint8_t goalH) const {
  static constexpr uint16_t INF = 0xFFFF;
  uint16_t dist[N][N];
  for (int y = 0; y < N; ++y) {
    for (int x = 0; x < N; ++x) {
      dist[y][x] = INF;
    }
  }

  int qx[N * N], qy[N * N];
  int qh = 0, qt = 0;

  for (int y = goalY0; y < (int)(goalY0 + goalH); ++y) {
    for (int x = goalX0; x < (int)(goalX0 + goalW); ++x) {
      if (!inBounds_(x, y)) continue;
      dist[y][x] = 0;
      qx[qt] = x;
      qy[qt] = y;
      qt++;
    }
  }

  while (qh < qt) {
    int x = qx[qh];
    int y = qy[qh];
    qh++;
    const uint16_t base = dist[y][x];

    for (int di = 0; di < 4; ++di) {
      Dir d = (Dir)di;
      if (knownHasWall_(x, y, d)) continue;
      int nx = x + dx4[di];
      int ny = y + dy4[di];
      if (!inBounds_(nx, ny)) continue;
      if (dist[ny][nx] > base + 1) {
        dist[ny][nx] = base + 1;
        qx[qt] = nx;
        qy[qt] = ny;
        qt++;
      }
    }
  }

  uint16_t best = INF;
  for (int y = startY0; y < (int)(startY0 + startH); ++y) {
    for (int x = startX0; x < (int)(startX0 + startW); ++x) {
      if (!inBounds_(x, y)) continue;
      if (dist[y][x] < best) best = dist[y][x];
    }
  }
  return best;
}

uint16_t FloodFillExplorer::bestKnownCostOriginalStartToGoal() const {
  return computeBestKnownCost_(origHx0_, origHy0_, origHw_, origHh_,
                               origGx0_, origGy0_, origGw_, origGh_);
}

bool FloodFillExplorer::isInOriginalStart(uint8_t x, uint8_t y) const {
  return x >= origHx0_ && x < (uint8_t)(origHx0_ + origHw_) &&
         y >= origHy0_ && y < (uint8_t)(origHy0_ + origHh_);
}

bool FloodFillExplorer::isInOriginalGoal(uint8_t x, uint8_t y) const {
  return x >= origGx0_ && x < (uint8_t)(origGx0_ + origGw_) &&
         y >= origGy0_ && y < (uint8_t)(origGy0_ + origGh_);
}

void FloodFillExplorer::exportKnownMaze(uint8_t walls[N][N], uint8_t mask[N][N], uint8_t visited[N][N]) const {
  memcpy(walls, knownWalls_, sizeof(knownWalls_));
  memcpy(mask, knownMask_, sizeof(knownMask_));
  for (int y = 0; y < N; ++y) {
    for (int x = 0; x < N; ++x) {
      visited[y][x] = visited_[y][x] ? 1 : 0;
    }
  }
}

bool FloodFillExplorer::importKnownMaze(const uint8_t walls[N][N], const uint8_t mask[N][N], const uint8_t visited[N][N],
                                        uint8_t mouseX, uint8_t mouseY, Dir mouseH) {
  if (!inBounds_(mouseX, mouseY)) return false;

  memcpy(knownWalls_, walls, sizeof(knownWalls_));
  memcpy(knownMask_, mask, sizeof(knownMask_));
  for (int y = 0; y < N; ++y) {
    for (int x = 0; x < N; ++x) {
      visited_[y][x] = visited[y][x] != 0;
    }
  }

  mx_ = mouseX;
  my_ = mouseY;
  mh_ = mouseH;
  visited_[my_][mx_] = true;
  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  running_ = false;
  targetHome_ = false;

  applyBoundaryWalls_();
  computeFloodFill_();
  computePlan_();
  markDirty_();
  return true;
}

const char* FloodFillExplorer::actionName_(Action a) const{
  switch(a){
    case ACT_NONE:   return "none";
    case ACT_TURN_L: return "turnL";
    case ACT_TURN_R: return "turnR";
    case ACT_TURN_180: return "turn180";
    case ACT_MOVE_F: return "moveF";
  }
  return "none";
}

String FloodFillExplorer::actionLabel_(Action a, uint8_t forwardCells) const {
  const char* name = actionName_(a);
  if (a == ACT_MOVE_F && forwardCells > 1) {
    return String(name) + " x" + String(forwardCells);
  }
  return String(name);
}

String FloodFillExplorer::buildKnownMazeAscii(uint8_t mouseX = 255, uint8_t mouseY = 255, Dir mouseH = NORTH) const {
  String result = "[N]E[S]W\n";
  for (uint8_t y = 0; y < N; y++) {
    for (uint8_t x = 0; x < N; x++) {
      int val = (knownMask_[y][x] & 0xF) << 3;
      if (mouseX == 255 && mouseY == 255) {
        if (mouseH == NORTH) val |= 0x40;
        else if (mouseH == SOUTH) val |= 0x20;
        else if (mouseH == EAST) val |= 0x10;
        else if (mouseH == WEST) val |= 0x08;
      }
      result += String(val) + " ";
    }
    result += "\n";
  }
  return result;
}

void FloodFillExplorer::markDirty_(){
  stateVer_++;
  buildStateJson_();
  wsStatePending_ = true;
}

void FloodFillExplorer::setHardwareMode(bool en) {
  hardwareMode_ = en;
  markDirty_();
}

void FloodFillExplorer::notifyStateChanged() {
  markDirty_();
}

void FloodFillExplorer::setStart(uint8_t x, uint8_t y, Dir h){
  if(x >= N || y >= N) return;
  sx_ = x; sy_ = y; sh_ = h;
  origSx_ = x; origSy_ = y; origSh_ = h;
  targetHome_ = false;
  markDirty_();
}

void FloodFillExplorer::setHomeRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h){
  if(w == 0 || h == 0) return;
  if(x0 >= N || y0 >= N) return;
  if(x0 + w > N) w = N - x0;
  if(y0 + h > N) h = N - y0;
  hx0_ = x0; hy0_ = y0; hw_ = w; hh_ = h;
  origHx0_ = x0; origHy0_ = y0; origHw_ = w; origHh_ = h;
  targetHome_ = false;
  markDirty_();
}

void FloodFillExplorer::setGoalRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h){
  if(w == 0 || h == 0) return;
  if(x0 >= N || y0 >= N) return;
  if(x0 + w > N) w = N - x0;
  if(y0 + h > N) h = N - y0;
  gx0_ = x0; gy0_ = y0; gw_ = w; gh_ = h;
  origGx0_ = x0; origGy0_ = y0; origGw_ = gw_; origGh_ = gh_;
  targetHome_ = false;
  markDirty_();
}

void FloodFillExplorer::setRunning(bool en) {
  running_ = en;
  markDirty_();
}

void FloodFillExplorer::clearKnownMaze() {
  memset(visited_, 0, sizeof(visited_));
  clearKnown_();
  mx_ = sx_;
  my_ = sy_;
  mh_ = sh_;
  visited_[my_][mx_] = true;
  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  pendingForwardCells_ = 0;
  lastActionForwardCells_ = 0;
  lastActionEndsAtKnownWall_ = false;
  computeFloodFill_();
  computePlan_();
  markDirty_();
}

void FloodFillExplorer::syncPose(uint8_t x, uint8_t y, Dir h, bool markVisited) {
  if (x >= N || y >= N) return;
  mx_ = x;
  my_ = y;
  mh_ = h;
  if (markVisited) visited_[my_][mx_] = true;
  computeFloodFill_();
  computePlan_();
  markDirty_();
}

void FloodFillExplorer::observeRelativeWalls(uint8_t x, uint8_t y, Dir heading,
                                             bool leftWall, bool frontWall, bool rightWall,
                                             bool leftValid, bool frontValid, bool rightValid) {
  if (!inBounds_(x, y)) return;

  auto leftDir = (Dir)(((uint8_t)heading + 3) & 3);
  auto rightDir = (Dir)(((uint8_t)heading + 1) & 3);

  if (leftValid) confirmObservedWall_(x, y, leftDir, leftWall);
  if (frontValid) confirmObservedWall_(x, y, heading, frontWall);
  if (rightValid) confirmObservedWall_(x, y, rightDir, rightWall);

  visited_[y][x] = true;
  applyBoundaryWalls_();
  computeFloodFill_();
  computePlan_();
  markDirty_();
}

FloodFillExplorer::Action FloodFillExplorer::requestNextAction() {
  if (waitAck_) return pendingAction_;

  Action act = chooseNextAction_();
  if (act == ACT_NONE) {
    running_ = false;
    markDirty_();
    return ACT_NONE;
  }

  dispatchAction_(act);
  return act;
}

FloodFillExplorer::Action FloodFillExplorer::requestNextActionNoAck() {
  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  pendingForwardCells_ = 0;

  Action act = chooseNextAction_();
  if (act == ACT_NONE) {
    running_ = false;
    markDirty_();
    return ACT_NONE;
  }

  return act;
}

void FloodFillExplorer::advanceTargetAfterReach() {
  onGoalReached_();
}

bool FloodFillExplorer::ackPendingActionExternal(bool ok, uint8_t x, uint8_t y, Dir h) {
  if (!waitAck_) return false;

  if (!ok) {
    running_ = false;
    waitAck_ = false;
    pendingAction_ = ACT_NONE;
    pendingForwardCells_ = 0;
    markDirty_();
    return false;
  }

  mx_ = x;
  my_ = y;
  mh_ = h;
  visited_[my_][mx_] = true;

  bool reachedGoal = atActiveTarget_();
  if (reachedGoal) {
    waitAck_ = false;
    pendingAction_ = ACT_NONE;
    pendingForwardCells_ = 0;
    onGoalReached_();
    return true;
  }

  if (pendingAction_ == ACT_MOVE_F && pendingForwardCells_ > 1) {
    pendingForwardCells_--;
    pendingSinceMs_ = millis();
    computeFloodFill_();
    computePlan_();
    markDirty_();
    return false;
  }

  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  pendingForwardCells_ = 0;

  computeFloodFill_();
  computePlan_();
  markDirty_();
  return false;
}

void FloodFillExplorer::truncatePendingForwardAction() {
  if (!waitAck_ || pendingAction_ != ACT_MOVE_F) return;
  if (pendingForwardCells_ > 1) {
    pendingForwardCells_ = 1;
    markDirty_();
  }
}

bool FloodFillExplorer::begin(const Config& cfg){
  cfg_ = cfg;

  if(server_){
    delete server_;
    server_ = nullptr;
  }
  if(ws_){
    delete ws_;
    ws_ = nullptr;
  }
  if (cfg_.enableWeb) {
    server_ = new WebServer(cfg_.port);
    ws_ = new WsServerWrapper(cfg_.wsPort);
  }

  clearKnown_();
  reset();

  targetHome_ = false; // start by targeting original goal

  if (cfg_.enableWeb && server_) {
    setupWeb_();
  }

  started_ = true;
  webServing_ = false;
  running_ = cfg_.autoRun;

  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  pendingForwardCells_ = 0;
  pendingSeq_ = 0;
  lastActionForwardCells_ = 0;

  if (cfg_.enableWeb) {
    log_("[Explorer] Web on port " + String(cfg_.port) + ", WS on port " + String(cfg_.wsPort));
  } else {
    log_("[Explorer] Web disabled");
  }
  return true;
}

void FloodFillExplorer::loop() {
  if (!started_) return;
  serviceWebServerState_();

  if (webServing_ && server_) {
    server_->handleClient();
  }
  if (webServing_) {
    serviceWs_();
  }

  static uint32_t lastLogMs = 0;

  if (running_ && waitAck_) {
    const uint32_t now = millis();

    if (cfg_.ackTimeoutMs > 0 && (uint32_t)(now - pendingSinceMs_) > cfg_.ackTimeoutMs) {
      const uint32_t LOG_EVERY_MS = 2000;
      if ((uint32_t)(now - lastLogMs) > LOG_EVERY_MS) {
          log_("[ACK] timeout (still waiting). seq=" + String(pendingSeq_) +
             " act=" + actionLabel_(pendingAction_, pendingForwardCells_));
        lastLogMs = now;
      }
    }
  }
}

// ========================= Truth / Known (simulator) =========================

void FloodFillExplorer::setTruthFromWalls(const uint8_t walls16[N][N],
                                         bool normalizePairs,
                                         bool forceBoundaries){
  for(int y=0;y<N;y++){
    for(int x=0;x<N;x++){
      truthWalls_[y][x] = clampMask(walls16[y][x]);
    }
  }

  if(forceBoundaries){
    forceBoundaries16(truthWalls_);
  }
  if(normalizePairs){
    normalizePairs16(truthWalls_);
    if(forceBoundaries) forceBoundaries16(truthWalls_);
  }

  reset();
  log_("[Truth] set from array");
}

void FloodFillExplorer::clearKnown_(){
  memset(knownWalls_, 0, sizeof(knownWalls_));
  memset(knownMask_,  0, sizeof(knownMask_));
  applyBoundaryWalls_();
}

void FloodFillExplorer::applyBoundaryWalls_() {
  for (int x = 0; x < N; ++x) {
    knownSetWallBoth_(x, 0, NORTH, true);
    knownSetWallBoth_(x, N - 1, SOUTH, true);
  }
  for (int y = 0; y < N; ++y) {
    knownSetWallBoth_(0, y, WEST, true);
    knownSetWallBoth_(N - 1, y, EAST, true);
  }
}

bool FloodFillExplorer::truthHasWall_(int x,int y, Dir d) const{
  return (truthWalls_[y][x] & bitForDir_(d)) != 0;
}

bool FloodFillExplorer::knownHasWall_(int x,int y, Dir d) const{
  uint8_t b = bitForDir_(d);
  if((knownMask_[y][x] & b) == 0) return false; // unknown treated open
  return (knownWalls_[y][x] & b) != 0;
}

bool FloodFillExplorer::getKnownWall(uint8_t x, uint8_t y, Dir d, bool& known, bool& wall) const {
  if (!inBounds_(x, y)) {
    known = false;
    wall = false;
    return false;
  }
  const uint8_t bit = bitForDir_(d);
  known = (knownMask_[y][x] & bit) != 0;
  wall = (knownWalls_[y][x] & bit) != 0;
  return true;
}

String FloodFillExplorer::buildKnownMazeAscii(uint8_t mouseX, uint8_t mouseY, Dir mouseH) const {
  String out;
  out.reserve(N * N * 8);

  for (int y = 0; y < N; ++y) {
    for (int x = 0; x < N; ++x) {
      out += "+";
      bool known = false;
      bool wall = false;
      getKnownWall((uint8_t)x, (uint8_t)y, NORTH, known, wall);
      out += (known && wall) ? "---" : "   ";
    }
    out += "+\n";

    for (int x = 0; x < N; ++x) {
      bool known = false;
      bool wall = false;
      getKnownWall((uint8_t)x, (uint8_t)y, WEST, known, wall);
      out += (known && wall) ? "|" : " ";

      char cell[4] = {' ', ' ', ' ', '\0'};
      if (x == mouseX && y == mouseY) {
        static const char kHeading[4] = {'^', '>', 'v', '<'};
        cell[1] = kHeading[(uint8_t)mouseH & 3];
      } else if (isGoal_(x, y)) {
        cell[1] = 'G';
      } else if (visited_[y][x]) {
        cell[1] = '.';
      }
      out += cell;
    }
    bool known = false;
    bool wall = false;
    getKnownWall((uint8_t)(N - 1), (uint8_t)y, EAST, known, wall);
    out += (known && wall) ? "|\n" : " \n";
  }

  for (int x = 0; x < N; ++x) {
    out += "+";
    bool known = false;
    bool wall = false;
    getKnownWall((uint8_t)x, (uint8_t)(N - 1), SOUTH, known, wall);
    out += (known && wall) ? "---" : "   ";
  }
  out += "+\n";
  return out;
}

void FloodFillExplorer::knownSetWallBoth_(int x,int y, Dir d, bool on){
  auto setOne = [&](int cx,int cy, Dir cd, bool v){
    if(!inBounds_(cx,cy)) return;
    uint8_t b = bitForDir_(cd);

    knownMask_[cy][cx] |= b;
    if(v) knownWalls_[cy][cx] |= b;
    else  knownWalls_[cy][cx] &= ~b;
  };

  setOne(x,y,d,on);
  int nx = x + dx4[(int)d], ny = y + dy4[(int)d];
  if(inBounds_(nx,ny)){
    setOne(nx,ny, opposite_(d), on);
  }
}

bool FloodFillExplorer::confirmObservedWall_(int x, int y, Dir d, bool on) {
  if (!inBounds_(x, y)) return false;

  const uint8_t bit = bitForDir_(d);
  const bool known = (knownMask_[y][x] & bit) != 0;
  const bool currentWall = (knownWalls_[y][x] & bit) != 0;

  if (known && currentWall == on) {
    return false;
  }
  knownSetWallBoth_(x, y, d, on);
  return true;
}

void FloodFillExplorer::senseCell_(int x,int y){
  if (hardwareMode_) return;
  // SIM: use truthWalls_
  for(int di=0; di<4; di++){
    Dir d = (Dir)di;
    bool w = truthHasWall_(x,y,d);
    knownSetWallBoth_(x,y,d,w);
  }
}

// ========================= Reset / Flood / Plan =========================

void FloodFillExplorer::reset(){
  mx_ = sx_; my_ = sy_; mh_ = sh_;

  memset(visited_, 0, sizeof(visited_));
  clearKnown_();

  senseCell_(mx_, my_);
  visited_[my_][mx_] = true;

  computeFloodFill_();
  computePlan_();

  running_ = false;
  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  pendingForwardCells_ = 0;
  lastActionForwardCells_ = 0;
  lastActionEndsAtKnownWall_ = false;

  log_("[FF] reset");
  markDirty_();
}

void FloodFillExplorer::serviceWebServerState_() {
  if (!cfg_.enableWeb || !server_) return;
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected) {
    if (!webServing_) {
      server_->begin();
      setupWs_();
      webServing_ = true;
      log_("[Explorer] Web server started (WiFi connected)");
    }
    return;
  }

  if (!webServing_) return;
  if (ws_) {
    if (ws_->client) ws_->client.stop();
    ws_->server.stop();
    ws_->handshaken = false;
    ws_->lastStateVerSent = 0xFFFFFFFFu;
  }
  webServing_ = false;
  log_("[Explorer] Web server paused (WiFi disconnected)");
}

void FloodFillExplorer::computeFloodFill_(){
  const uint16_t INF = 0xFFFF;
  for(int y=0;y<N;y++) for(int x=0;x<N;x++) dist_[y][x] = INF;

  int qx[N*N], qy[N*N];
  int qh=0, qt=0;

  for(int y=gy0_; y<(int)(gy0_+gh_); y++){
    for(int x=gx0_; x<(int)(gx0_+gw_); x++){
      dist_[y][x] = 0;
      qx[qt]=x; qy[qt]=y; qt++;
    }
  }

  while(qh<qt){
    int x=qx[qh], y=qy[qh]; qh++;
    uint16_t base = dist_[y][x];

    for(int di=0; di<4; di++){
      Dir d = (Dir)di;
      if(knownHasWall_(x,y,d)) continue;

      int nx = x + dx4[di], ny = y + dy4[di];
      if(!inBounds_(nx,ny)) continue;

      if(dist_[ny][nx] > base + 1){
        dist_[ny][nx] = base + 1;
        qx[qt]=nx; qy[qt]=ny; qt++;
      }
    }
  }
}

void FloodFillExplorer::computePlan_(){
  planLen_ = 0;

  uint8_t x = mx_, y = my_;
  Dir h = mh_;

  auto push = [&](uint8_t px, uint8_t py, Dir ph){
    if(planLen_ >= kMaxPlan) return;
    plan_[planLen_++] = PlanNode{px, py, (uint8_t)ph};
  };

  push(x,y,h);
  if(isGoal_(x,y)) return;

  static uint8_t seen[N][N][4];
  memset(seen, 0, sizeof(seen));
  seen[y][x][(uint8_t)h] = 1;

  auto orderFromHeading = [&](Dir hh, Dir out[4]){
    out[0] = hh;
    out[1] = (Dir)(((uint8_t)hh + 3) & 3);
    out[2] = (Dir)(((uint8_t)hh + 1) & 3);
    out[3] = (Dir)(((uint8_t)hh + 2) & 3);
  };

  const uint16_t maxSteps = N*N*4;
  for(uint16_t step=0; step<maxSteps; step++){
    uint16_t cur = dist_[y][x];
    if(cur == 0xFFFF) break;
    if(cur == 0) break;

    Dir order[4];
    orderFromHeading(h, order);

    bool moved=false;
    for(int i=0;i<4;i++){
      Dir d = order[i];
      int nx = (int)x + dx4[(int)d];
      int ny = (int)y + dy4[(int)d];
      if(!inBounds_(nx,ny)) continue;
      if(knownHasWall_(x,y,d)) continue;

      uint16_t nd = dist_[ny][nx];
      if(nd == 0xFFFF) continue;

      if(nd < cur){
        h = d;
        x = (uint8_t)nx;
        y = (uint8_t)ny;
        push(x,y,h);
        moved=true;
        break;
      }
    }

    if(!moved) break;
    if(isGoal_(x,y)) break;
    if(seen[y][x][(uint8_t)h]) break;
    seen[y][x][(uint8_t)h] = 1;
  }
}

// ========================= ACTION (ACK-driven) =========================

FloodFillExplorer::Action FloodFillExplorer::chooseNextAction_(){
  if (!hardwareMode_) {
    senseCell_(mx_, my_);
  }
  visited_[my_][mx_] = true;
  computeFloodFill_();
  computePlan_();

  lastActionForwardCells_ = 0;
  lastActionEndsAtKnownWall_ = false;

  Action act = chooseNextActionForPose_(mx_, my_, mh_, lastActionForwardCells_, lastActionEndsAtKnownWall_);
  return act;
}

uint8_t FloodFillExplorer::chooseForwardCells_() const {
  return chooseForwardCellsForPose_(mx_, my_, mh_);
}

FloodFillExplorer::Action FloodFillExplorer::chooseNextActionForPose_(uint8_t x, uint8_t y, Dir h,
                                                                      uint8_t& forwardCells,
                                                                      bool& endsAtKnownWall) const {
  forwardCells = 0;
  endsAtKnownWall = false;

  if (atActiveTargetPose_(x, y, h)) return ACT_NONE;

  if (targetHome_ && isGoal_(x, y) && h != origSh_) {
    const uint8_t curh = (uint8_t)h;
    const uint8_t tarh = (uint8_t)origSh_;
    const uint8_t diff = (tarh + 4 - curh) & 3;
    forwardCells = 1;
    if (diff == 1) return ACT_TURN_R;
    if (diff == 3) return ACT_TURN_L;
    return ACT_TURN_180;
  }

  const uint16_t cur = dist_[y][x];
  if (cur == 0xFFFF) return ACT_NONE;

  Dir order[4];
  order[0] = h;
  order[1] = (Dir)((h + 3) & 3);
  order[2] = (Dir)((h + 1) & 3);
  order[3] = (Dir)((h + 2) & 3);

  Dir best = h;
  bool found = false;
  for (int i = 0; i < 4; i++) {
    const Dir d = order[i];
    if (knownHasWall_(x, y, d)) continue;
    const int nx = x + dx4[(int)d];
    const int ny = y + dy4[(int)d];
    if (!inBounds_(nx, ny)) continue;
    const uint16_t nd = dist_[ny][nx];
    if (nd < cur) {
      best = d;
      found = true;
      break;
    }
  }
  if (!found) return ACT_NONE;

  if (best == h) {
    forwardCells = chooseForwardCellsForPose_(x, y, h);
    bool known = false;
    bool wall = false;
    uint8_t fx = x;
    uint8_t fy = y;
    for (uint8_t i = 0; i < forwardCells; ++i) {
      fx = (uint8_t)(fx + dx4[(int)h]);
      fy = (uint8_t)(fy + dy4[(int)h]);
    }
    getKnownWall(fx, fy, h, known, wall);
    endsAtKnownWall = known && wall;
    return ACT_MOVE_F;
  }

  forwardCells = 1;
  const uint8_t curh = (uint8_t)h;
  const uint8_t tarh = (uint8_t)best;
  const uint8_t diff = (tarh + 4 - curh) & 3;
  if (diff == 1) return ACT_TURN_R;
  if (diff == 3) return ACT_TURN_L;
  return ACT_TURN_180;
}

uint8_t FloodFillExplorer::chooseForwardCellsForPose_(uint8_t x, uint8_t y, Dir h) const {
  uint8_t maxCells = cfg_.maxForwardCells;
  if (maxCells == 0) maxCells = 1;

  uint16_t cur = dist_[y][x];
  uint8_t cells = 0;
  while (cells < maxCells) {
    if (!isKnownOpen_(x, y, h)) break;
    const int nx = x + dx4[(int)h];
    const int ny = y + dy4[(int)h];
    if (!inBounds_(nx, ny)) break;
    const uint16_t nd = dist_[ny][nx];
    if (nd >= cur) break;
    x = (uint8_t)nx;
    y = (uint8_t)ny;
    cur = nd;
    cells++;
    if (isGoal_(x, y)) break;
  }

  return cells > 0 ? cells : 1;
}

bool FloodFillExplorer::buildQueuedActionsFromCurrentPose(QueuedAction* outActions,
                                                          uint16_t capacity,
                                                          uint16_t& outCount) {
  outCount = 0;
  if (outActions == nullptr || capacity == 0) return false;

  if (!hardwareMode_) {
    senseCell_(mx_, my_);
  }
  visited_[my_][mx_] = true;
  computeFloodFill_();
  computePlan_();

  uint8_t x = mx_;
  uint8_t y = my_;
  Dir h = mh_;

  while (!atActiveTargetPose_(x, y, h)) {
    if (outCount >= capacity) {
      return false;
    }

    uint8_t forwardCells = 0;
    bool endsAtKnownWall = false;
    const Action act = chooseNextActionForPose_(x, y, h, forwardCells, endsAtKnownWall);
    if (act == ACT_NONE) {
      return false;
    }

    QueuedAction& qa = outActions[outCount++];
    qa.action = act;
    qa.forwardCells = forwardCells > 0 ? forwardCells : 1;
    qa.endsAtKnownWall = endsAtKnownWall;

    if (act == ACT_MOVE_F) {
      for (uint8_t i = 0; i < qa.forwardCells; ++i) {
        if (knownHasWall_(x, y, h)) return false;
        const int nx = x + dx4[(int)h];
        const int ny = y + dy4[(int)h];
        if (!inBounds_(nx, ny)) return false;
        x = (uint8_t)nx;
        y = (uint8_t)ny;
      }
    } else if (act == ACT_TURN_L) {
      h = (Dir)(((uint8_t)h + 3) & 3);
    } else if (act == ACT_TURN_R) {
      h = (Dir)(((uint8_t)h + 1) & 3);
    } else if (act == ACT_TURN_180) {
      h = (Dir)(((uint8_t)h + 2) & 3);
    }
  }

  return true;
}

void FloodFillExplorer::dispatchAction_(Action a){
  pendingAction_ = a;
  pendingForwardCells_ = (a == ACT_MOVE_F) ? ((lastActionForwardCells_ > 0) ? lastActionForwardCells_ : 1) : 1;
  pendingSeq_++;
  pendingSinceMs_ = millis();
  waitAck_ = true;
  markDirty_();
}

bool FloodFillExplorer::commitPendingAction_(){
  if(pendingAction_ == ACT_TURN_L){
    mh_ = (Dir)(((uint8_t)mh_ + 3) & 3);
  }else if(pendingAction_ == ACT_TURN_R){
    mh_ = (Dir)(((uint8_t)mh_ + 1) & 3);
  }else if(pendingAction_ == ACT_TURN_180){
    mh_ = (Dir)(((uint8_t)mh_ + 2) & 3);
  }else if(pendingAction_ == ACT_MOVE_F){
    if(!knownHasWall_(mx_, my_, mh_)){
      int nx = mx_ + dx4[(int)mh_];
      int ny = my_ + dy4[(int)mh_];
      if(inBounds_(nx,ny)){
        mx_ = (uint8_t)nx;
        my_ = (uint8_t)ny;
        visited_[my_][mx_] = true;
        senseCell_(mx_, my_);
      }
    }
    if (pendingForwardCells_ > 0) pendingForwardCells_--;
  }
  const bool done = pendingAction_ != ACT_MOVE_F || pendingForwardCells_ == 0;
  if (done) {
    pendingAction_ = ACT_NONE;
  }
  return done;
}

bool FloodFillExplorer::performStepMove_(String& reply){
  if(waitAck_){
    reply = "waiting ack";
    return false;
  }

  for(uint8_t guard = 0; guard < 8; ++guard){
    Action act = chooseNextAction_();
    if(act == ACT_NONE){
      running_ = false;
      markDirty_();
      reply = "done";
      return true;
    }

    dispatchAction_(act);
    commitPendingAction_();
    waitAck_ = false;
    pendingAction_ = ACT_NONE;
    pendingForwardCells_ = 0;

    if(atActiveTarget_()){
      onGoalReached_();
      reply = "step move ok (GOAL)";
      return true;
    }

    computeFloodFill_();
    computePlan_();
    markDirty_();

    if(act == ACT_MOVE_F){
      reply = "step move ok";
      return true;
    }
  }

  reply = "step move guard";
  return false;
}

void FloodFillExplorer::onGoalReached_(){
  // Stop running for safety
  running_ = false;
  waitAck_ = false;
  pendingAction_ = ACT_NONE;
  pendingForwardCells_ = 0;
  lastActionForwardCells_ = 0;

  // --- Toggle target ---
  // If we just reached the original GOAL (2x2), switch the target back to HOME (the original 1-cell start).
  // If we just reached HOME, switch the target back to the original GOAL (2x2).
  targetHome_ = !targetHome_;

  if(targetHome_){
    // target HOME rectangle
    gx0_ = origHx0_;
    gy0_ = origHy0_;
    gw_  = origHw_;
    gh_  = origHh_;
    log_("[FF] reached target -> now GO HOME (original home rect)");
  }else{
    // target original GOAL rect
    gx0_ = origGx0_;
    gy0_ = origGy0_;
    gw_  = origGw_;
    gh_  = origGh_;
    log_("[FF] reached target -> now GO TO GOAL (original rect)");
  }

  // Recompute for next trip
  computeFloodFill_();
  computePlan_();
  markDirty_();
}

// ========================= Web =========================

void FloodFillExplorer::setupWeb_(){
  server_->on("/", HTTP_GET, [this](){ handleRoot_(); });
  server_->on("/state", HTTP_GET, [this](){ handleState_(); });
  server_->on("/cmd", HTTP_GET, [this](){ handleCmd_(); });
  server_->on("/next", HTTP_GET, [this](){ handleNext_(); });
  server_->on("/ack", HTTP_GET, [this](){ handleAck_(); });
}

void FloodFillExplorer::setupWs_() {
  if (!ws_) return;
  ws_->server.begin();
  ws_->handshaken = false;
  ws_->lastStateVerSent = 0xFFFFFFFFu;
}

bool FloodFillExplorer::handleWsHandshake_() {
  if (!ws_ || !ws_->client || !ws_->client.connected()) return false;

  ws_->client.setTimeout(50);
  String key;
  while (ws_->client.connected()) {
    String line = ws_->client.readStringUntil('\n');
    if (line.length() == 0) break;
    line.trim();
    if (line.startsWith("Sec-WebSocket-Key:")) {
      key = line.substring(strlen("Sec-WebSocket-Key:"));
      key.trim();
    }
    if (line.length() == 0) break;
  }

  if (key.length() == 0) {
    ws_->client.stop();
    return false;
  }

  const String acceptSrc = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  unsigned char sha1[20];
  mbedtls_sha1((const unsigned char*)acceptSrc.c_str(), acceptSrc.length(), sha1);

  unsigned char base64Out[64];
  size_t outLen = 0;
  mbedtls_base64_encode(base64Out, sizeof(base64Out), &outLen, sha1, sizeof(sha1));
  const String acceptKey = String((const char*)base64Out).substring(0, outLen);

  ws_->client.print(
    "HTTP/1.1 101 Switching Protocols\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    "Sec-WebSocket-Accept: " + acceptKey + "\r\n\r\n");

  ws_->handshaken = true;
  ws_->lastStateVerSent = 0xFFFFFFFFu;
  return true;
}

bool FloodFillExplorer::readWsFrame_(String& payload, uint8_t& opcode) {
  if (!ws_ || !ws_->client || !ws_->client.connected() || ws_->client.available() < 2) return false;

  const uint8_t b0 = ws_->client.read();
  const uint8_t b1 = ws_->client.read();
  opcode = b0 & 0x0F;
  uint64_t len = b1 & 0x7F;

  if (len == 126) {
    while (ws_->client.available() < 2) yield();
    len = ((uint16_t)ws_->client.read() << 8) | (uint16_t)ws_->client.read();
  } else if (len == 127) {
    while (ws_->client.available() < 8) yield();
    len = 0;
    for (int i = 0; i < 8; ++i) {
      len = (len << 8) | (uint64_t)ws_->client.read();
    }
  }

  const bool masked = (b1 & 0x80) != 0;
  uint8_t mask[4] = {0, 0, 0, 0};
  if (masked) {
    while (ws_->client.available() < 4) yield();
    for (int i = 0; i < 4; ++i) mask[i] = ws_->client.read();
  }

  payload = "";
  payload.reserve((size_t)len);
  for (uint64_t i = 0; i < len; ++i) {
    while (ws_->client.available() < 1) yield();
    char ch = (char)ws_->client.read();
    if (masked) ch = (char)(ch ^ mask[i & 3]);
    payload += ch;
  }
  return true;
}

void FloodFillExplorer::sendWsText_(const String& text) {
  if (!ws_ || !ws_->client || !ws_->client.connected() || !ws_->handshaken) return;

  const size_t len = text.length();
  ws_->client.write((uint8_t)0x81);
  if (len < 126) {
    ws_->client.write((uint8_t)len);
  } else if (len < 65536) {
    ws_->client.write((uint8_t)126);
    ws_->client.write((uint8_t)((len >> 8) & 0xFF));
    ws_->client.write((uint8_t)(len & 0xFF));
  } else {
    ws_->client.write((uint8_t)127);
    for (int i = 7; i >= 0; --i) {
      ws_->client.write((uint8_t)((((uint64_t)len) >> (i * 8)) & 0xFF));
    }
  }
  ws_->client.write((const uint8_t*)text.c_str(), len);
}

void FloodFillExplorer::sendWsState_() {
  sendWsText_("state|" + stateJson_);
  if (ws_) ws_->lastStateVerSent = stateVer_;
  wsStatePending_ = false;
}

String FloodFillExplorer::jsonEscape_(const String& s) const {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i) {
    const char ch = s[i];
    if (ch == '\\' || ch == '"') out += '\\';
    if (ch == '\n') out += "\\n";
    else if (ch == '\r') out += "\\r";
    else out += ch;
  }
  return out;
}

void FloodFillExplorer::sendWsReply_(const String& kind, const String& msg) {
  sendWsText_(kind + "|" + msg);
}

void FloodFillExplorer::processWsMessage_(const String& msg) {
  if (msg == "hello") {
    sendWsState_();
    return;
  }
  if (msg == "next") {
    if (!running_) { sendWsReply_("reply", "not running"); return; }
    if (waitAck_) { sendWsReply_("reply", "waiting ack"); return; }
    Action act = chooseNextAction_();
    if (act == ACT_NONE) {
      running_ = false;
      markDirty_();
      sendWsReply_("reply", "done");
      return;
    }
    dispatchAction_(act);
    sendWsReply_("reply", "dispatched");
    return;
  }
  if (msg.startsWith("ack|")) {
    const int p1 = msg.indexOf('|', 4);
    if (p1 < 0) { sendWsReply_("error", "bad ack"); return; }
    const uint32_t seq = (uint32_t)strtoul(msg.substring(4, p1).c_str(), nullptr, 10);
    const bool ok = msg.substring(p1 + 1) != "0";
    if (!waitAck_) { sendWsReply_("error", "no pending"); return; }
    if (seq != pendingSeq_) { sendWsReply_("error", "seq mismatch"); return; }
    if (!ok) {
      running_ = false;
      waitAck_ = false;
      pendingAction_ = ACT_NONE;
      markDirty_();
      sendWsReply_("reply", "ACK FAIL");
      return;
    }
    commitPendingAction_();
    waitAck_ = false;
    if (atActiveTarget_()) {
      onGoalReached_();
      sendWsReply_("reply", "ACK OK (GOAL)");
      return;
    }
    computeFloodFill_();
    computePlan_();
    markDirty_();
    sendWsReply_("reply", "ACK OK");
    return;
  }
  if (msg.startsWith("setstart|")) {
    int p1 = msg.indexOf('|', 9);
    int p2 = p1 < 0 ? -1 : msg.indexOf('|', p1 + 1);
    if (p1 < 0 || p2 < 0) { sendWsReply_("error", "bad setstart"); return; }
    int x = msg.substring(9, p1).toInt();
    int y = msg.substring(p1 + 1, p2).toInt();
    int h = msg.substring(p2 + 1).toInt();
    if (x < 0) x = 0; if (x >= N) x = N - 1;
    if (y < 0) y = 0; if (y >= N) y = N - 1;
    h &= 3;
    running_ = false;
    waitAck_ = false;
    pendingAction_ = ACT_NONE;
    sx_ = (uint8_t)x;
    sy_ = (uint8_t)y;
    sh_ = (Dir)h;
    reset();
    sendWsReply_("reply", "OK");
    return;
  }
  if (msg.startsWith("cmd|")) {
    const String a = msg.substring(4);
    if (a == "step") {
      String reply;
      performStepMove_(reply);
      sendWsReply_("reply", reply);
      return;
    }
    if (a == "run") {
      running_ = true;
      if (!waitAck_) {
        Action act = chooseNextAction_();
        if (act != ACT_NONE) dispatchAction_(act);
      }
      markDirty_();
      sendWsReply_("reply", "OK");
      return;
    }
    if (a == "pause") {
      running_ = false;
      markDirty_();
      sendWsReply_("reply", "OK");
      return;
    }
    if (a == "reset") {
      reset();
      sendWsReply_("reply", "OK");
      return;
    }
  }
  if (msg.startsWith("hwcmd|")) {
    const String a = msg.substring(6);
    if (!hardwareMode_ || !webCommandFn_) {
      sendWsReply_("error", "hardware control unavailable");
      return;
    }
    webCommandFn_(a);
    sendWsReply_("reply", "hardware " + a + " requested");
    return;
  }
  sendWsReply_("error", "unknown");
}

void FloodFillExplorer::serviceWs_() {
  if (!ws_) return;

  if ((!ws_->client || !ws_->client.connected()) && ws_->server.hasClient()) {
    if (ws_->client) ws_->client.stop();
    ws_->client = ws_->server.accept();
    ws_->client.setNoDelay(true);
    ws_->handshaken = false;
    ws_->lastStateVerSent = 0xFFFFFFFFu;
  }

  if (!ws_->client || !ws_->client.connected()) return;

  if (!ws_->handshaken) {
    if (!handleWsHandshake_()) return;
    wsStatePending_ = true;
  }

  while (ws_->client.available() > 1) {
    String payload;
    uint8_t opcode = 0;
    if (!readWsFrame_(payload, opcode)) break;
    if (opcode == 0x8) {
      ws_->client.stop();
      ws_->handshaken = false;
      return;
    }
    if (opcode == 0x9) {
      ws_->client.write((uint8_t)0x8A);
      ws_->client.write((uint8_t)0x00);
      continue;
    }
    if (opcode == 0x1) {
      processWsMessage_(payload);
    }
  }

  if (ws_->handshaken && wsStatePending_ && ws_->lastStateVerSent != stateVer_) {
    sendWsState_();
  }
}

void FloodFillExplorer::handleRoot_(){
  String html = FPSTR(kHtml);
  html.replace("%WS_PORT%", String(cfg_.wsPort));
  server_->sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server_->sendHeader("Pragma", "no-cache");
  server_->sendHeader("Expires", "0");
  server_->sendHeader("Connection", "close");
  server_->send(200, "text/html; charset=utf-8", html);
}

void FloodFillExplorer::handleCmd_(){
  if(!server_->hasArg("a")){
    server_->sendHeader("Connection", "close");
    server_->send(400, "text/plain", "missing a=step|run|pause|reset|clearmaze|setstart");
    return;
  }

  const String a = server_->arg("a");

  // STEP (SIM): dispatch+commit immediately
  if(a == "step"){
    String reply;
    performStepMove_(reply);
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", reply);
    return;
  }

  if(a == "run"){
    running_ = true;

    if(!waitAck_){
      Action act = chooseNextAction_();
      if(act != ACT_NONE){
        dispatchAction_(act);
      }
    }

    markDirty_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  if(a == "pause"){
    running_ = false;
    markDirty_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  if(a == "reset"){
    reset();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  if (a == "clearmaze") {
    if (!hardwareMode_ || !webCommandFn_) {
      server_->sendHeader("Connection", "close");
      server_->send(503, "text/plain", "hardware control unavailable");
      return;
    }
    webCommandFn_("clearmaze");
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  if(a == "setstart"){
    if(!server_->hasArg("x") || !server_->hasArg("y") || !server_->hasArg("h")){
      server_->sendHeader("Connection", "close");
      server_->send(400, "text/plain", "missing x,y,h");
      return;
    }

    int x = server_->arg("x").toInt();
    int y = server_->arg("y").toInt();
    int h = server_->arg("h").toInt();

    if(x < 0) x = 0; if(x >= N) x = N - 1;
    if(y < 0) y = 0; if(y >= N) y = N - 1;
    h &= 3;

    running_ = false;
    waitAck_ = false;
    pendingAction_ = ACT_NONE;

    sx_ = (uint8_t)x;
    sy_ = (uint8_t)y;
    sh_ = (Dir)h;

    reset();

    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "OK");
    return;
  }

  server_->sendHeader("Connection", "close");
  server_->send(400, "text/plain", "unknown a= (step|run|pause|reset|clearmaze|setstart)");
}

void FloodFillExplorer::handleNext_(){
  if(!running_){
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "not running");
    return;
  }
  if(waitAck_){
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "waiting ack");
    return;
  }

  Action act = chooseNextAction_();
  if(act == ACT_NONE){
    running_ = false;
    markDirty_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "done");
    return;
  }

  dispatchAction_(act);
  server_->sendHeader("Connection", "close");
  server_->send(200, "text/plain", "dispatched");
}

void FloodFillExplorer::handleAck_(){
  if(!server_->hasArg("seq")){
    server_->sendHeader("Connection", "close");
    server_->send(400, "text/plain", "missing seq");
    return;
  }
  uint32_t seq = (uint32_t) strtoul(server_->arg("seq").c_str(), nullptr, 10);
  bool ok = !server_->hasArg("ok") || server_->arg("ok") != "0";

  if(!waitAck_){
    server_->sendHeader("Connection", "close");
    server_->send(409, "text/plain", "no pending");
    return;
  }
  if(seq != pendingSeq_){
    server_->sendHeader("Connection", "close");
    server_->send(409, "text/plain", "seq mismatch");
    return;
  }

  if(!ok){
    running_ = false;
    waitAck_ = false;
    pendingAction_ = ACT_NONE;
    markDirty_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "ACK FAIL");
    return;
  }

  commitPendingAction_();
  waitAck_ = false;

  if(atActiveTarget_()){
    onGoalReached_();
    server_->sendHeader("Connection", "close");
    server_->send(200, "text/plain", "ACK OK (GOAL)");
    return;
  }

  computeFloodFill_();
  computePlan_();
  markDirty_();

  server_->sendHeader("Connection", "close");
  server_->send(200, "text/plain", "ACK OK");
}

void FloodFillExplorer::handleState_() {
  uint32_t since = 0;
  if (server_->hasArg("since")) {
    since = (uint32_t)strtoul(server_->arg("since").c_str(), nullptr, 10);
  }

  // Always disable caching for state
  server_->sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server_->sendHeader("Pragma", "no-cache");
  server_->sendHeader("Expires", "0");

  // Close to avoid keep-alive stalls on ESP32 WebServer
  server_->sendHeader("Connection", "close");

  // Fast path: no changes
  if (since == stateVer_) {
    // 204 must not include body
    server_->send(204);
    return;
  }

  // Optional but helps browser + reduces weird stalls:
  server_->sendHeader("Content-Type", "application/json");
  server_->sendHeader("Content-Length", String(stateJson_.length()));

  server_->send(200, "application/json", stateJson_);
}

// ========================= JSON =========================

void FloodFillExplorer::buildStateJson_(){
  String out;
  out.reserve(12000); // Reduce slightly to lower heap pressure.

  out += "{";
  out += "\"ver\":" + String(stateVer_) + ",";

  out += "\"mouse\":{\"x\":" + String(mx_) + ",\"y\":" + String(my_) + ",\"h\":" + String((int)mh_) + "},";
  out += "\"start\":{\"x\":" + String(sx_) + ",\"y\":" + String(sy_) + ",\"h\":" + String((int)sh_) + "},";
  out += "\"home\":{\"x0\":" + String(hx0_) + ",\"y0\":" + String(hy0_) + ",\"w\":" + String(hw_) + ",\"h\":" + String(hh_) + "},";
  out += "\"goal\":{\"x0\":" + String(gx0_) + ",\"y0\":" + String(gy0_) + ",\"w\":" + String(gw_) + ",\"h\":" + String(gh_) + "},";
  out += "\"running\":" + String(running_ ? "true":"false") + ",";
  out += "\"hardwareMode\":" + String(hardwareMode_ ? "true":"false") + ",";
  out += "\"waitAck\":" + String(waitAck_ ? "true":"false") + ",";
  out += "\"pendingSeq\":" + String((uint32_t)pendingSeq_) + ",";
  out += "\"pendingActionName\":\"" + jsonEscape_(actionLabel_(pendingAction_, pendingForwardCells_)) + "\",";

  out += "\"knownWalls\":[";
  for(int y=0;y<N;y++){
    if(y) out += ",";
    out += "[";
    for(int x=0;x<N;x++){
      if(x) out += ",";
      out += String((int)knownWalls_[y][x]);
    }
    out += "]";
  }
  out += "],";

  out += "\"knownMask\":[";
  for(int y=0;y<N;y++){
    if(y) out += ",";
    out += "[";
    for(int x=0;x<N;x++){
      if(x) out += ",";
      out += String((int)knownMask_[y][x]);
    }
    out += "]";
  }
  out += "],";

  out += "\"visited\":[";
  for(int y=0;y<N;y++){
    if(y) out += ",";
    out += "[";
    for(int x=0;x<N;x++){
      if(x) out += ",";
      out += (visited_[y][x] ? "true" : "false");
    }
    out += "]";
  }
  out += "],";

  out += "\"dist\":[";
  for(int y=0;y<N;y++){
    if(y) out += ",";
    out += "[";
    for(int x=0;x<N;x++){
      if(x) out += ",";
      uint16_t v = dist_[y][x];
      out += (v == 0xFFFF) ? "65535" : String(v);
    }
    out += "]";
  }
  out += "],";

  out += "\"planLen\":" + String((int)planLen_) + ",";
  out += "\"plan\":[";
  for(uint16_t i=0;i<planLen_; i++){
    if(i) out += ",";
    out += "{";
    out += "\"x\":" + String(plan_[i].x) + ",";
    out += "\"y\":" + String(plan_[i].y) + ",";
    out += "\"h\":" + String((int)plan_[i].h);
    out += "}";
  }
  out += "]";

  if (stateExtrasJsonFn_) {
    const String extras = stateExtrasJsonFn_();
    if (extras.length() > 0) {
      out += ",";
      out += extras;
    }
  }

  out += "}";

  stateJson_ = out;
}


````

### `FloodFillExplorer.h`

````cpp
#pragma once
#include <Arduino.h>
#include <WebServer.h>

class FloodFillExplorer {
public:
  static constexpr int N = 16;

  enum Dir : uint8_t { NORTH=0, EAST=1, SOUTH=2, WEST=3 };

  enum : uint8_t {
    WALL_N = 1,
    WALL_E = 2,
    WALL_S = 4,
    WALL_W = 8
  };

  enum Action : uint8_t {
    ACT_NONE = 0,
    ACT_TURN_L,
    ACT_TURN_R,
    ACT_TURN_180,
    ACT_MOVE_F
  };

  struct QueuedAction {
    Action action = ACT_NONE;
    uint8_t forwardCells = 1;
    bool endsAtKnownWall = false;
  };

  struct PlanNode {
    uint8_t x;
    uint8_t y;
    uint8_t h;
  };

  static constexpr uint16_t kMaxPlan = 512;

  struct Config {
    uint16_t port = 80;
    uint16_t wsPort = 83;
    bool enableWeb = true;
    bool autoRun = false;
    uint8_t maxForwardCells = 1;

    // ACK timeout handling (ms). 0 = disable
    uint32_t ackTimeoutMs = 2000;

    // true: timeout -> pause (safe)
    // false: timeout -> discard pending and continue running
    bool pauseOnAckTimeout = true;
  };

  using LogFn = void(*)(const String&);
  using WebCommandFn = void(*)(const String&);
  using StateExtrasJsonFn = String(*)();

  FloodFillExplorer();
  ~FloodFillExplorer();

  bool begin(const Config& cfg);
  void loop();

  void setLog(LogFn fn) { logFn_ = fn; }
  void setWebCommandHandler(WebCommandFn fn) { webCommandFn_ = fn; }
  void setStateExtrasJsonProvider(StateExtrasJsonFn fn) { stateExtrasJsonFn_ = fn; }
  void setHardwareMode(bool en);
  void notifyStateChanged();

  void setStart(uint8_t x, uint8_t y, Dir h);
  void setHomeRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
  void setGoalRect(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
  void setRunning(bool en);
  void clearKnownMaze();
  void syncPose(uint8_t x, uint8_t y, Dir h, bool markVisited = true);
  void observeRelativeWalls(uint8_t x, uint8_t y, Dir heading,
                            bool leftWall, bool frontWall, bool rightWall,
                            bool leftValid = true, bool frontValid = true, bool rightValid = true);
  Action requestNextAction();
  Action requestNextActionNoAck();
  bool buildQueuedActionsFromCurrentPose(QueuedAction* outActions, uint16_t capacity, uint16_t& outCount);
  bool ackPendingActionExternal(bool ok, uint8_t x, uint8_t y, Dir h);
  void truncatePendingForwardAction();

  // SIM truth walls (optional). If you don't call this, truthWalls_ default 0 (no walls),
  // robot mode should override senseCell_ later by real sensors.
  void setTruthFromWalls(const uint8_t walls16[N][N],
                         bool normalizePairs = true,
                         bool forceBoundaries = true);

  // optional external controls
  bool isRunning() const { return running_; }
  bool isWaitAck() const { return waitAck_; }
  uint32_t pendingSeq() const { return pendingSeq_; }
  Action pendingAction() const { return pendingAction_; }
  uint8_t pendingForwardCells() const { return pendingForwardCells_; }
  uint8_t lastActionForwardCells() const { return lastActionForwardCells_; }
  bool lastActionEndsAtKnownWall() const { return lastActionEndsAtKnownWall_; }
  bool atGoal() const { return atActiveTarget_(); }
  void advanceTargetAfterReach();
  bool getKnownWall(uint8_t x, uint8_t y, Dir d, bool& known, bool& wall) const;
  String buildKnownMazeAscii(uint8_t mouseX = 255, uint8_t mouseY = 255, Dir mouseH = NORTH) const;
  uint16_t bestKnownCostOriginalStartToGoal() const;
  bool isInOriginalStart(uint8_t x, uint8_t y) const;
  bool isInOriginalGoal(uint8_t x, uint8_t y) const;
  void exportKnownMaze(uint8_t walls[N][N], uint8_t mask[N][N], uint8_t visited[N][N]) const;
  bool importKnownMaze(const uint8_t walls[N][N], const uint8_t mask[N][N], const uint8_t visited[N][N],
                       uint8_t mouseX, uint8_t mouseY, Dir mouseH);
  
private:
  // --- web handlers ---
  void setupWeb_();
  void setupWs_();
  void serviceWebServerState_();
  void handleRoot_();
  void handleState_();
  void handleCmd_();
  void handleNext_();
  void handleAck_();
  void serviceWs_();
  bool handleWsHandshake_();
  bool readWsFrame_(String& payload, uint8_t& opcode);
  void sendWsText_(const String& text);
  void sendWsState_();
  void processWsMessage_(const String& msg);
  String jsonEscape_(const String& s) const;
  void sendWsReply_(const String& kind, const String& msg);

  // --- state / json ---
  void buildStateJson_();
  void markDirty_();
  const char* actionName_(Action a) const;
  String actionLabel_(Action a, uint8_t forwardCells) const;

  // --- floodfill core ---
  bool inBounds_(int x,int y) const;
  bool isGoal_(int x,int y) const;
  bool atActiveTarget_() const;
  bool atActiveTargetPose_(uint8_t x, uint8_t y, Dir h) const;
  bool isKnownOpen_(int x, int y, Dir d) const;
  uint16_t computeBestKnownCost_(uint8_t startX0, uint8_t startY0,
                                 uint8_t startW, uint8_t startH,
                                 uint8_t goalX0, uint8_t goalY0,
                                 uint8_t goalW, uint8_t goalH) const;

  uint8_t bitForDir_(Dir d) const;
  Dir opposite_(Dir d) const;

  void clearKnown_();
  void applyBoundaryWalls_();
  bool truthHasWall_(int x,int y, Dir d) const;
  bool knownHasWall_(int x,int y, Dir d) const;
  void knownSetWallBoth_(int x,int y, Dir d, bool on);
  bool confirmObservedWall_(int x, int y, Dir d, bool on);

  void senseCell_(int x,int y);

  void reset();
  void computeFloodFill_();
  void computePlan_();

  // --- ack-driven action ---
  Action chooseNextAction_();
  uint8_t chooseForwardCells_() const;
  Action chooseNextActionForPose_(uint8_t x, uint8_t y, Dir h,
                                  uint8_t& forwardCells, bool& endsAtKnownWall) const;
  uint8_t chooseForwardCellsForPose_(uint8_t x, uint8_t y, Dir h) const;
  void dispatchAction_(Action a);
  bool commitPendingAction_();
  bool performStepMove_(String& reply);
  void onGoalReached_();

  void log_(const String& s);
  // --- remember original start/goal so we can toggle ---
  uint8_t origSx_ = 0, origSy_ = 0;
  Dir     origSh_ = NORTH;
  uint8_t origHx0_ = 0, origHy0_ = 15, origHw_ = 1, origHh_ = 1;
  uint8_t origGx0_ = 7, origGy0_ = 7, origGw_ = 2, origGh_ = 2;

  // true: currently targeting HOME (the original start), false: currently targeting the original GOAL (2x2)
  bool    targetHome_ = false;


private:
  WebServer* server_ = nullptr;
  class WsServerWrapper;
  WsServerWrapper* ws_ = nullptr;
  Config cfg_{};

  bool started_ = false;
  bool running_ = false;
  bool hardwareMode_ = false;
  bool webServing_ = false;

  uint8_t sx_ = 0, sy_ = 15, mx_ = 0, my_ = 15;
  Dir sh_ = NORTH, mh_ = NORTH;

  uint8_t hx0_ = 0, hy0_ = 15, hw_ = 1, hh_ = 1;
  uint8_t gx0_ = 7, gy0_ = 7, gw_ = 2, gh_ = 2;

  bool visited_[N][N]{};
  uint16_t dist_[N][N]{};

  uint8_t knownWalls_[N][N]{};
  uint8_t knownMask_[N][N]{};
  uint8_t truthWalls_[N][N]{};

  PlanNode plan_[kMaxPlan]{};
  uint16_t planLen_ = 0;

  uint32_t stateVer_ = 0;
  String stateJson_;
  volatile bool wsStatePending_ = false;

  // ACK state
  bool waitAck_ = false;
  uint32_t pendingSeq_ = 0;
  Action pendingAction_ = ACT_NONE;
  uint8_t pendingForwardCells_ = 0;
  uint32_t pendingSinceMs_ = 0;
  uint8_t lastActionForwardCells_ = 0;
  bool lastActionEndsAtKnownWall_ = false;

  LogFn logFn_ = nullptr;
  WebCommandFn webCommandFn_ = nullptr;
  StateExtrasJsonFn stateExtrasJsonFn_ = nullptr;
};

````

### `HARDWARE.md`

````md
# HARDWARE.md

## Board

- MCU: ESP32-S3
- Framework: Arduino-ESP32
- Serial: USB CDC (`Serial.begin(921600)`)
- Onboard features in use:
  - BOOT button
  - RGB LED (WS2812B-style single pixel on `GPIO48`)
  - Wi-Fi
  - SPIFFS
  - Dual-core task split (core 1 realtime, core 0 app/network)
- Optional future feature:
  - BLE (not enabled in current firmware)

## ESP32-S3 Built-in Capabilities (Not Enabled Yet)

- AI vector instructions (SIMD on Xtensa LX7):
  - Useful for DSP / quantized ML acceleration.
  - Present on ESP32-S3, but this firmware does not currently use dedicated AI kernels.
- Rich programmable GPIO/peripheral matrix:
  - Up to `45` programmable GPIOs on ESP32-S3 family variants (board breakout may expose fewer).
  - Peripheral options include `SPI`, LCD interface, camera interface, `I2C`, `I2S`, and PWM/LEDC routing.
  - Current firmware uses only a subset needed for motors, encoders, TOF, battery ADC, and UI/network.

## Motor + Driver

- Motor driver: `TB6612` dual H-bridge
- Motors: `GA12-N20` DC gear motors
- Gear ratio: `1:30`
- Encoder: `2-channel AB hall` (quadrature)
- Wheel diameter: `34mm`

## Power

- Battery pack: `2S LiPo`, `1200mAh` (cells in series)
- Battery monitor ADC pin: `GPIO3`
- Battery divider (configured):
  - `R_top = 56k`
  - `R_bottom = 18k`
- Battery thresholds:
  - Warning: `7.10V`
  - Critical: `6.90V`

## I2C Bus

- SDA: `GPIO8`
- SCL: `GPIO9`
- Clock: `400kHz`
- Bus is shared by:
  - PCF8574 I/O expander (`0x20`)
  - VL53L0X sensors (assigned addresses below)

## TOF / Wall Sensors (VL53L0X)

- Sensor count: `5`
- Address assignment:
  - Sensor 0 -> `0x30`
  - Sensor 1 -> `0x31`
  - Sensor 2 -> `0x32`
  - Sensor 3 -> `0x33`
  - Sensor 4 -> `0x34`
- XSHUT control is through PCF8574 pins:
  - Sensor 0 -> PCF P0
  - Sensor 1 -> PCF P1
  - Sensor 2 -> PCF P2
  - Sensor 3 -> PCF P3
  - Sensor 4 -> PCF P4

### Sensor layout interpretation in firmware

- Layout auto-detect:
  - V1: fallback path (typically 3 wall sensors used)
  - V2: when 4 sensors initialize (front composed from diagonals)
- V1 mapping used by control:
  - Left = sensor 0
  - Front = sensor 2
  - Right = sensor 4
- V2 mapping used by control:
  - Front-left = sensor 0
  - Left = sensor 1
  - Right = sensor 2
  - Front-right = sensor 3

## Motor Driver Wiring

### Left motor

- IN1: `GPIO5`
- IN2: `GPIO6`
- PWM: `GPIO4` (LEDC channel 0)
- Encoder A: `GPIO1`
- Encoder B: `GPIO2`
- Direction invert: `true`
- Encoder invert: `true`

### Right motor

- IN1: `GPIO10`
- IN2: `GPIO11`
- PWM: `GPIO7` (LEDC channel 1)
- Encoder A: `GPIO12`
- Encoder B: `GPIO13`
- Direction invert: `false`
- Encoder invert: `false`

### PWM setup

- Frequency: `20kHz`
- Resolution: `10-bit`

## LED

- Onboard RGB LED data pin (default): `GPIO48`
- Pixel count (default): `1`
- Source: `LedController.cpp` (`RGB_PIN`, `NUM_PIXELS` defaults)

## Buttons / Inputs

- BOOT button input pin: `GPIO0`
- Active level: LOW (`INPUT_PULLUP`)

## Network / Service Ports

- Floodfill web UI: `81`
- Floodfill WebSocket: `83`
- OTA upload web page: `82`
- Debug TCP console: `2323`

## Task/Core placement (runtime)

- Realtime core: `Core 1`
  - motor task
  - tof task
  - gptimer-triggered wakeup
- App core: `Core 0`
  - planner task
  - explorer task
  - user task
  - telemetry task
  - Wi-Fi/OTA task (from config)

## Notes

- All values above are from current `Config.cpp` and runtime initialization.
- If hardware wiring changes, update `Config.cpp` first, then this file.
- TOF quality and centering behavior also depend on per-sensor calibration values (`SENSOR_SCALE`, `SENSOR_OFFSET_MM`).
- Motor distance model (`LEFT_MM_PER_TICK`, `RIGHT_MM_PER_TICK`) should be tuned on real hardware; do not assume ideal values from wheel diameter alone.
- Planned extension (not enabled in current firmware): add a dedicated MCU path for yaw control/fusion integration.
````

### `LedController.cpp`

````cpp
#include "LedController.h"

#include <Adafruit_NeoPixel.h>

#ifndef RGB_PIN
#define RGB_PIN 48
#endif

#ifndef NUM_PIXELS
#define NUM_PIXELS 1
#endif

namespace {
Adafruit_NeoPixel gPixels(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

constexpr Rgb kCycleColors[] = {
  {255,   0,   0},
  {  0, 255,   0},
  {  0,   0, 255},
  {255, 255,   0},
  {  0, 255, 255},
  {255,   0, 255},
  {255, 255, 255},
  {  0,   0,   0},
};
}

void LedController::begin() {
  gPixels.begin();
  gPixels.setBrightness(255);
  gPixels.show();
  colorIndex_ = -1;
}

void LedController::setState(State state) {
  switch (state) {
    case State::OFF: off(); return;
    case State::RED: setColor_(255, 0, 0); return;
    case State::GREEN: setColor_(0, 255, 0); return;
    case State::BLUE: setColor_(0, 0, 255); return;
    case State::YELLOW: setColor_(255, 255, 0); return;
    case State::MAGENTA: setColor_(255, 0, 255); return;
    case State::CYAN: setColor_(0, 255, 255); return;
    case State::WHITE: setColor_(255, 255, 255); return;
  }
}

void LedController::setRed() {
  setState(State::RED);
}

void LedController::setGreen() {
  setState(State::GREEN);
}

void LedController::setBlue() {
  setState(State::BLUE);
}

void LedController::setYellow() {
  setState(State::YELLOW);
}

void LedController::setMagenta() {
  setState(State::MAGENTA);
}

void LedController::setCyan() {
  setState(State::CYAN);
}

void LedController::setWhite() {
  setState(State::WHITE);
}

bool LedController::handleCommand(const String& rawCmd, String* response) {
  String cmd = rawCmd;
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "cycle" || cmd == "rotate") {
    colorIndex_ = (colorIndex_ + 1) % (int)(sizeof(kCycleColors) / sizeof(kCycleColors[0]));
    const Rgb& c = kCycleColors[colorIndex_];
    setColor_(c.r, c.g, c.b);
    if (response) {
      *response = String("[LED] cycle idx=") + colorIndex_ +
                  " rgb=(" + c.r + "," + c.g + "," + c.b + ")";
    }
    return true;
  }

  if (cmd == "off") {
    off();
    if (response) *response = "[LED] off";
    return true;
  }

  if (cmd == "red") {
    setRed();
    if (response) *response = "[LED] red";
    return true;
  }

  if (cmd == "green") {
    setGreen();
    if (response) *response = "[LED] green";
    return true;
  }

  if (cmd == "blue") {
    setBlue();
    if (response) *response = "[LED] blue";
    return true;
  }

  if (cmd == "yellow") {
    setYellow();
    if (response) *response = "[LED] yellow";
    return true;
  }

  if (cmd == "magenta") {
    setMagenta();
    if (response) *response = "[LED] magenta";
    return true;
  }

  if (cmd == "cyan" || cmd == "bluegreen") {
    setCyan();
    if (response) *response = "[LED] cyan";
    return true;
  }

  if (cmd == "white") {
    setWhite();
    if (response) *response = "[LED] white";
    return true;
  }

  return false;
}

void LedController::off() {
  setColor_(0, 0, 0);
}

void LedController::setColor_(uint8_t r, uint8_t g, uint8_t b) {
  gPixels.setPixelColor(0, gPixels.Color(r, g, b));
  gPixels.show();
}
````

### `LedController.h`

````cpp
#pragma once

#include <Arduino.h>

class LedController {
public:
  enum class State : uint8_t {
    OFF = 0,
    WHITE,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    MAGENTA,
    CYAN
  };

  void begin();
  bool handleCommand(const String& cmd, String* response = nullptr);
  void setState(State state);
  void off();
  void setCyan();
  void setWhite();
  void setRed();
  void setGreen();
  void setBlue();
  void setYellow();
  void setMagenta();

private:
  void setColor_(uint8_t r, uint8_t g, uint8_t b);

  int colorIndex_ = -1;
};
````

### `MotionController.cpp`

````cpp
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

void MotionController::applyStopMode_(StopMode mode) {
  if (!left_ || !right_) return;
  switch (mode) {
    case StopMode::COAST:
      left_->coastStop();
      right_->coastStop();
      break;
    case StopMode::BRAKE:
      left_->enableSpeedControl(false);
      right_->enableSpeedControl(false);
      left_->brakeStop();
      right_->brakeStop();
      break;
    case StopMode::HARDSTOP:
    default:
      left_->hardStop();
      right_->hardStop();
      break;
  }
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
  right_->setSpeedTPS(cfg_.moveSpeedTps);
  left_->setSpeedTPS(cfg_.moveSpeedTps);
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
  applyStopMode_(StopMode::HARDSTOP);
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
  const float leftMm = leftDelta * cfg_.leftMmPerTick;
  const float rightMm = rightDelta * cfg_.rightMmPerTick;
  return 0.5f * (leftMm + rightMm);
}

float MotionController::absoluteAverageProgressMm_() const {
  return fabsf(averageProgressMm_());
}

float MotionController::differentialProgressMm_() const {
  const int32_t leftDelta = left_->getTicks() - startLeftTicks_;
  const int32_t rightDelta = right_->getTicks() - startRightTicks_;
  const float leftMm = leftDelta * cfg_.leftMmPerTick;
  const float rightMm = rightDelta * cfg_.rightMmPerTick;
  return rightMm - leftMm;
}

void MotionController::markDone_(MotionStatus status, const String& reason) {
  if (status != MOTION_COMPLETED) {
    applyStopMode_(StopMode::HARDSTOP);
  } else if (stopOnCompletion_) {
    applyStopMode_(cfg_.completionStopMode);
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

  const uint32_t now = millis();
  uint32_t primitiveTimeoutMs = cfg_.primitiveTimeoutMs;
  if (primitive_ == MOTION_MOVE_MULTI_CELL && moveCellTargetCount_ > 1) {
    primitiveTimeoutMs += (uint32_t)(moveCellTargetCount_ - 1) * cfg_.corridorTimeoutPerCellMs;
  }

  if ((uint32_t)(now - startedMs_) > primitiveTimeoutMs) {
    markDone_(MOTION_FAILED, "primitive timeout");
    return;
  }

  auto approachSpeedTps = [&](float baseSpeedTps, float stopMm, const WallObservation& walls) {
    if (!walls.frontValid || walls.frontMm == 0) return baseSpeedTps;
    const float startFactor = max(1.0f, cfg_.frontApproachStartFactor);
    const float startMm = stopMm * startFactor;
    const float minSpeed = max(1.0f, min(baseSpeedTps, cfg_.frontApproachMinSpeedTps));
    const float frontMm = (float)walls.frontMm;
    if (frontMm >= startMm) return baseSpeedTps;
    if (frontMm <= stopMm) return minSpeed;
    const float span = max(1.0f, startMm - stopMm);
    const float t = (frontMm - stopMm) / span;  // 1 at start, 0 at stop
    return minSpeed + t * (baseSpeedTps - minSpeed);
  };

  auto distanceSpeedTps = [&](float baseSpeedTps, float targetMm, float progressMm) {
    const float startRatio = constrain(cfg_.distanceApproachStartRatio, 0.0f, 1.0f);
    const float startMm = targetMm * startRatio;
    const float minSpeed = max(1.0f, min(baseSpeedTps, cfg_.distanceApproachMinSpeedTps));
    if (progressMm <= startMm) return baseSpeedTps;
    if (progressMm >= targetMm) return minSpeed;
    const float span = max(1.0f, targetMm - startMm);
    const float t = (progressMm - startMm) / span;  // 0 at start, 1 at target
    return baseSpeedTps - t * (baseSpeedTps - minSpeed);
  };

  auto applyCenteredSpeed = [&](float baseSpeedTps, float correction) {
    const float slowSideGain = max(1.0f, cfg_.centeringSlowSideGain);
    const float fastSideGain = max(0.0f, cfg_.centeringFastSideGain);
    float leftCmd = baseSpeedTps;
    float rightCmd = baseSpeedTps;
    if (correction > 0.0f) {
      // Right side should slow down.
      leftCmd = baseSpeedTps + correction * fastSideGain;
      rightCmd = baseSpeedTps - correction * slowSideGain;
    } else if (correction < 0.0f) {
      // Left side should slow down.
      rightCmd = baseSpeedTps - correction * fastSideGain;
      leftCmd = baseSpeedTps + correction * slowSideGain;
    }
    left_->setSpeedTPS(leftCmd);
    right_->setSpeedTPS(rightCmd);
  };

  if (primitive_ == MOTION_MOVE_ONE_CELL) {
    const float progressMm = averageProgressMm_();
    state.pose.forwardProgressMm = progressMm;

    const WallObservation& walls = state.walls;
    if (useLatchedTrackMode_ && !straightTrackModeLatched_) {
      latchStraightTrackMode(walls);
    } else if (!useLatchedTrackMode_) {
      tof_->setStraightTrackMode(chooseStraightTrackMode_(walls));
    }
    bool shouldFrontStop = walls.frontValid && walls.frontWall && walls.frontMm > 0 &&
                           walls.frontMm <= cfg_.frontStopMm;

    float correction = 0.0f;
    if (walls.leftValid || walls.rightValid) {
      correction = tof_->computeError(0.0f) * cfg_.centeringGain;
    }

    float cmdSpeed = distanceSpeedTps(cfg_.moveSpeedTps, cfg_.cellDistanceMm, progressMm);
    cmdSpeed = min(cmdSpeed, approachSpeedTps(cfg_.moveSpeedTps, cfg_.frontStopMm, walls));
    applyCenteredSpeed(cmdSpeed, correction);

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
                                 walls.frontValid &&
                                 walls.frontWall &&
                                 walls.frontMm > 0 &&
                                 walls.frontMm <= frontStopThresholdMm;

    float correction = 0.0f;
    if (walls.leftValid || walls.rightValid) {
      correction = tof_->computeError(0.0f) * cfg_.corridorCenteringGain;
    }

    float cmdSpeed = distanceSpeedTps(cfg_.corridorMoveSpeedTps, targetDistanceMm, progressMm);
    if (inFinalCell) {
      cmdSpeed = min(cmdSpeed,
                     approachSpeedTps(cfg_.corridorMoveSpeedTps, frontStopThresholdMm, walls));
    }
    applyCenteredSpeed(cmdSpeed, correction);

    const bool reachedDistanceTarget = progressMm >= targetDistanceMm;
    const bool shouldComplete = moveEndsAtKnownWall_
                                  ? shouldFrontStop
                                  : (reachedDistanceTarget || shouldFrontStop);

    if (shouldComplete) {
      markDone_(MOTION_COMPLETED);
      return;
    }

    if (updateProgressOrFail_(progressMm, now, "corridor move stall")) return;
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
    if (snapCenterPhase_ == SNAP_CENTER_PHASE_BACK) {
      const float progressMm = absoluteAverageProgressMm_();
      state.pose.forwardProgressMm = -progressMm;

      left_->setSpeedTPS(-cfg_.reverseSpeedTps);
      right_->setSpeedTPS(-cfg_.reverseSpeedTps);

      if (progressMm >= cfg_.reverseDistanceMm) {
        applyStopMode_(cfg_.snapCenterHoldStopMode);
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
      applyStopMode_(cfg_.snapCenterHoldStopMode);

      if ((int32_t)(now - snapCenterHoldUntilMs_) >= 0) {
        snapCenterPhase_ = SNAP_CENTER_PHASE_FORWARD;
        startLeftTicks_ = left_->getTicks();
        startRightTicks_ = right_->getTicks();
        lastProgressMm_ = 0.0f;
        lastProgressMs_ = now;
      }
      return;
    } else {
      const float progressMm = averageProgressMm_();
      state.pose.forwardProgressMm = progressMm;
      const WallObservation& walls = state.walls;
      tof_->setStraightTrackMode(chooseStraightTrackMode_(walls));
      float correction = 0.0f;
      if (walls.leftValid || walls.rightValid) {
        correction = tof_->computeError(0.0f) * cfg_.centeringGain;
      }
      applyCenteredSpeed(cfg_.shortForwardSpeedTps, correction);

      if (progressMm >= cfg_.shortForwardDistanceMm) {
        markDone_(MOTION_COMPLETED);
        return;
      }

      if (updateProgressOrFail_(progressMm, now, "snap center forward stall")) return;
    }
  } else if (primitive_ == MOTION_TURN_LEFT_90 ||
             primitive_ == MOTION_TURN_RIGHT_90 ||
             primitive_ == MOTION_TURN_180) {
    float turnTargetMm = 1.0f;
    if (primitive_ == MOTION_TURN_LEFT_90) {
      turnTargetMm = (cfg_.turnLeft90Mm > 0.0f) ? cfg_.turnLeft90Mm : 1.0f;
    } else if (primitive_ == MOTION_TURN_RIGHT_90) {
      turnTargetMm = (cfg_.turnRight90Mm > 0.0f) ? cfg_.turnRight90Mm : 1.0f;
    } else {
      turnTargetMm = (cfg_.turn180Mm > 0.0f) ? cfg_.turn180Mm : max(1.0f, 2.0f * cfg_.turnLeft90Mm);
    }
    const float diffMm = fabsf(differentialProgressMm_());
    const float turnDegrees = (primitive_ == MOTION_TURN_180) ? 180.0f : 90.0f;
    const float turnRatio = diffMm / turnTargetMm;
    state.pose.turnProgressDeg = min(turnDegrees, turnRatio * turnDegrees);

    const float slowdownStartRatio = constrain(cfg_.turnSlowdownStartRatio, 0.0f, 1.0f);
    const bool slowZone = turnRatio >= slowdownStartRatio;
    const float turnCmdTps = slowZone
      ? min(cfg_.turnSpeedTps, max(1.0f, cfg_.turnMinSpeedTps))
      : cfg_.turnSpeedTps;

    if (primitive_ == MOTION_TURN_LEFT_90) {
      left_->setSpeedTPS(-turnCmdTps);
      right_->setSpeedTPS(turnCmdTps);
    } else {
      left_->setSpeedTPS(turnCmdTps);
      right_->setSpeedTPS(-turnCmdTps);
    }

    if (diffMm >= turnTargetMm) {
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
````

### `MotionController.h`

````cpp
#pragma once

#include <Arduino.h>

#include "Battery.h"
#include "DcMotor.h"
#include "MultiVL53L0X.h"
#include "RobotTypes.h"

class MotionController {
public:
  enum class StopMode : uint8_t {
    COAST = 0,
    BRAKE = 1,
    HARDSTOP = 2,
  };

  struct Config {
    float cellDistanceMm = 180.0f;
    float turnLeft90Mm = 113.0f;
    float turnRight90Mm = 113.0f;
    float turn180Mm = 232.0f;
    float moveSpeedTps = 320.0f;
    float corridorMoveSpeedTps = 320.0f;
    float shortForwardDistanceMm = 90.0f;
    float shortForwardSpeedTps = 220.0f;
    float reverseDistanceMm = 45.0f;
    float reverseSpeedTps = 180.0f;
    uint32_t snapCenterStopHoldMs = 50;
    float turnSpeedTps = 250.0f;
    float turnMinSpeedTps = 120.0f;
    float turnSlowdownStartRatio = 0.75f;
    float centeringGain = 1.6f;
    float corridorCenteringGain = 1.6f;
    float centeringSlowSideGain = 1.0f;
    float centeringFastSideGain = 0.0f;
    float frontStopMm = 55.0f;
    float corridorFrontStopMm = 55.0f;
    float distanceApproachStartRatio = 0.85f;
    float distanceApproachMinSpeedTps = 180.0f;
    float frontApproachStartFactor = 1.5f;
    float frontApproachMinSpeedTps = 180.0f;
    uint32_t primitiveTimeoutMs = 3000;
    uint32_t corridorTimeoutPerCellMs = 1000;
    uint32_t stallTimeoutMs = 700;
    float stopTps = 20.0f;
    float minProgressMm = 12.0f;
    float leftMmPerTick = 0.54f;
    float rightMmPerTick = 0.54f;
    uint8_t corridorMaxCells = 1;
    StopMode completionStopMode = StopMode::HARDSTOP;
    StopMode snapCenterHoldStopMode = StopMode::HARDSTOP;
  };

  void begin(DcMotor& left, DcMotor& right, MultiVL53L0X& tof, Battery* battery = nullptr);
  void setConfig(const Config& cfg) { cfg_ = cfg; }

  bool moveOneCell();
  bool moveCells(uint8_t cells, bool requireFrontStopAtEnd = false);
  bool moveForwardShort();
  bool moveBackwardShort();
  bool snapCenter();
  bool turnLeft90();
  bool turnRight90();
  bool turn180();
  void stop();
  void abort(const String& reason);
  void update(RobotState& state);
  void setStopOnCompletion(bool en) { stopOnCompletion_ = en; }
  void clearCompletionState();
  void setUseLatchedTrackMode(bool en);

  bool isBusy() const { return status_ == MOTION_RUNNING_PRIMITIVE; }
  MotionStatus status() const { return status_; }
  MotionPrimitiveType primitive() const { return primitive_; }
  MotionPrimitiveType lastFinishedPrimitive() const { return lastFinishedPrimitive_; }
  const String& lastError() const { return lastError_; }
  uint8_t moveCellTargetCount() const { return moveCellTargetCount_; }
  void limitMoveCellTargetCount(uint8_t cells);
  void latchStraightTrackMode(const WallObservation& walls);

private:
  bool startPrimitive_(MotionPrimitiveType primitive);
  void markDone_(MotionStatus status, const String& reason = "");
  MultiVL53L0X::StraightTrackMode chooseStraightTrackMode_(const WallObservation& walls) const;
  float averageProgressMm_() const;
  float absoluteAverageProgressMm_() const;
  float differentialProgressMm_() const;
  void applyStopMode_(StopMode mode);
  void resetSnapState_();
  bool updateProgressOrFail_(float progressMm, uint32_t now, const char* stallReason);

  DcMotor* left_ = nullptr;
  DcMotor* right_ = nullptr;
  MultiVL53L0X* tof_ = nullptr;
  Battery* battery_ = nullptr;
  Config cfg_{};

  MotionPrimitiveType primitive_ = MOTION_NONE;
  MotionPrimitiveType lastFinishedPrimitive_ = MOTION_NONE;
  MotionStatus status_ = MOTION_IDLE;
  String lastError_;

  int32_t startLeftTicks_ = 0;
  int32_t startRightTicks_ = 0;
  uint32_t startedMs_ = 0;
  uint32_t lastProgressMs_ = 0;
  uint32_t snapCenterHoldUntilMs_ = 0;
  float lastProgressMm_ = 0.0f;
  uint8_t snapCenterPhase_ = 0;
  uint8_t moveCellTargetCount_ = 1;
  bool moveEndsAtKnownWall_ = false;
  bool straightTrackModeLatched_ = false;
  MultiVL53L0X::StraightTrackMode straightTrackMode_ = MultiVL53L0X::TRACK_NONE;
  bool useLatchedTrackMode_ = true;
  bool stopOnCompletion_ = true;
};
````

### `Mouse_esp32s3.ino`

````cpp
#include "AppRuntime.h"
#include <Arduino.h>

static void userTask(void* arg);
static void plannerTask(void* arg);

void setup() {
  MainApp::setupApp(userTask, plannerTask);
}

void loop() {
  MainApp::loopApp();
}

static void userTask(void* arg) {
  MainApp::userTaskBody(arg);
}

static void plannerTask(void* arg) {
  MainApp::plannerTaskBody(arg);
}
````

### `MultiVL53L0X.cpp`

````cpp
#include "Config.h"
#include "MultiVL53L0X.h"

static uint32_t sCenterPidTraceCounter = 0;

uint8_t MultiVL53L0X::effectiveState_(uint8_t index) const {
    if (index >= _numSensors) return 255;
    if (!_initialized[index]) return 3;
    return _timeoutFlag[index];
}

uint16_t MultiVL53L0X::effectiveDistance_(uint8_t index) const {
    if (index >= _numSensors) return 0;
    if (!_initialized[index]) return 0;
    return _lastDistance[index];
}

void MultiVL53L0X::setCenterPid(float kp, float ki, float kd, float iLimit, float outLimit) {
    _centerKp = kp;
    _centerKi = ki;
    _centerKd = kd;
    _centerILimit = fabsf(iLimit);
    _centerOutLimit = fabsf(outLimit);
}

void MultiVL53L0X::setCenterTargets(float leftMm, float rightMm) {
    _centerTargetLeft = leftMm;
    _centerTargetRight = rightMm;
}

void MultiVL53L0X::resetCenterPid() {
    _centerIntegral = 0.0f;
    _centerPrevError = 0.0f;
    _centerRawFiltered = 0.0f;
    _lastDualWallError = 0.0f;
    _dualWallBlend = 0.0f;
    _captureCenterTargetsOnFirstSample = true;
    _straightTrackMode = TRACK_NONE;
    _centerPrevUs = 0;
    _centerPidPrimed = false;
    _sweepReadyForCompute = false;
    error = 0.0f;
}

MultiVL53L0X::MultiVL53L0X(uint8_t pcfAddress,
                           uint8_t numSensors,
                           const uint8_t* xshutPins,
                           const uint8_t* sensorAddresses,
                           uint16_t updateIntervalMs,
                           TwoWire& wire)
    : _wire(&wire),
      _pcf(pcfAddress),
      _pcfAddress(pcfAddress),
      _numSensors(numSensors),
      _intervalMs(updateIntervalMs),
      _xshutPins(xshutPins),
      _sensorAddresses(sensorAddresses)
{
    if (_numSensors > MAX_SENSORS) _numSensors = MAX_SENSORS;

    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        _initialized[i]  = false;
        _timeoutFlag[i]  = false;
        _lastDistance[i] = 0;
    }
}

bool MultiVL53L0X::begin() {

    if (!_pcf.begin()) return false;

    if (_i2cMutex == nullptr) {
        _i2cMutex = xSemaphoreCreateMutex();
    }

    i2cLock();

    for (uint8_t i = 0; i < 8; i++) {
        _pcf.write(i, HIGH);
    }

    xshutAllLow();
    vTaskDelay(pdMS_TO_TICKS(10));

    for (uint8_t i = 0; i < _numSensors; i++) {
        _pcf.write(_xshutPins[i], HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));

        _sensors[i].setBus(_wire);
        _sensors[i].setTimeout(30);

        // Retry up to 3 times
        bool initialized = false;
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            if (_sensors[i].init()) {
                _sensors[i].setAddress(_sensorAddresses[i]);
                vTaskDelay(pdMS_TO_TICKS(5));
                _sensors[i].startContinuous(_intervalMs);
                vTaskDelay(pdMS_TO_TICKS(30));

                uint16_t test = _sensors[i].readRangeContinuousMillimeters();
                bool to = _sensors[i].timeoutOccurred();

                if (!to && test != 0 && test != 65535) {
                    _lastDistance[i] = test;
                    initialized = true;
                    break; // success, exit retry loop
                }
            }

            // Failed this attempt, shutdown and wait before retry
            _pcf.write(_xshutPins[i], LOW);
            vTaskDelay(pdMS_TO_TICKS(30));
            _pcf.write(_xshutPins[i], HIGH);
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        _initialized[i] = initialized;
        if (!initialized) {
            _pcf.write(_xshutPins[i], LOW); // final shutdown if all retries fail
        }
    }

    xshutAllHigh();
    i2cUnlock();

    detectLayout();  // Auto-detect sensor layout here

    return true;
}

bool MultiVL53L0X::readTOF_fast(uint8_t addr, uint16_t &dist)
{
    _wire->beginTransmission(addr);
    _wire->write(0x14);

    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    delayMicroseconds(200);

    _wire->requestFrom(addr, (uint8_t)12);

    if (_wire->available() < 12) {
        return false;
    }

    uint8_t buf[12];
    for (int i = 0; i < 12; i++) {
        buf[i] = _wire->read();
    }

    dist = (buf[10] << 8) | buf[11];

    return true;
}

void MultiVL53L0X::update() {
    // Define the sequence pattern
    static const uint8_t sensorOrder[] = {0, 2, 1, 3, 4};
    uint8_t currentSensor = sensorOrder[_cSensor % _numSensors];

    if (_numSensors == 0) {
        return;
    }

    for (uint8_t tries = 0; tries < _numSensors; tries++) {
        if (_initialized[_cSensor]) break;
        _cSensor = (_cSensor + 1) % _numSensors;
    }

    if (!_initialized[currentSensor]) {
        return;
    }

    // uint16_t dist = _sensors[currentSensor].readRangeContinuousMillimeters();
    // bool to = _sensors[currentSensor].timeoutOccurred();
    uint16_t dist;
    i2cLock();
    bool ok = readTOF_fast(_sensorAddresses[currentSensor], dist);
    i2cUnlock();

    _raw[currentSensor] = dist;

    if (ok && dist > 0 && dist < 1000) {

        float corrected = dist * AppConfig::Tof::SENSOR_SCALE[currentSensor] +
                          AppConfig::Tof::SENSOR_OFFSET_MM[currentSensor];

        if (corrected < AppConfig::Tof::DIST_MIN_VALID_MM) {
            _lastDistance[currentSensor] = AppConfig::Tof::DIST_MIN_VALID_MM;
            _timeoutFlag[currentSensor]  = 1;
        }
        else if (corrected > AppConfig::Tof::DIST_MAX_VALID_MM) {
            _lastDistance[currentSensor] = AppConfig::Tof::DIST_FAR_MM;
            _timeoutFlag[currentSensor]  = 2;
        }
        else {
            uint16_t val = (uint16_t)corrected;

            if ((_lastDistance[currentSensor] == 0) || (_lastDistance[currentSensor] > AppConfig::Tof::DIST_MAX_VALID_MM)) {
                _lastDistance[currentSensor] = val;
            } else {
                _lastDistance[currentSensor] =
                    AppConfig::Tof::DIST_LPF_PREV_WEIGHT * _lastDistance[currentSensor] +
                    AppConfig::Tof::DIST_LPF_SAMPLE_WEIGHT * val;
            }

            _timeoutFlag[currentSensor] = 0;
        }
    }
    else {
        _timeoutFlag[currentSensor] = 3;
        err_count ++;
    }

    _cSensor = (_cSensor + 1) % _numSensors;
    if (_cSensor == 4) {
        _sweepReadyForCompute = true;
    }
}

// Detect layout
void MultiVL53L0X::detectLayout() {
    uint8_t active = 0;

    for (uint8_t i = 0; i < _numSensors; i++) {
        if (_initialized[i]) active++;
    }

    if (active == 4) {
        _version = SENSOR_V2;
    } else {
        _version = SENSOR_V1;
    }
}

// Unified sensor read
MultiVL53L0X::SensorState MultiVL53L0X::getSensorState() {
    SensorState s = {false, false, false, false, false, false, 0, 0, 0};
    auto isObservable = [&](uint8_t index) {
        if (index >= _numSensors) return false;
        uint8_t state = stateTimeout(index);
        return state == 0 || state == 1 || state == 2;
    };

    if (_version == SENSOR_V1) {
        uint16_t left  = getDistance(0);
        uint16_t front = getDistance(2);
        uint16_t right = getDistance(4);

        s.leftValid = isObservable(0);
        s.frontValid = isObservable(2);
        s.rightValid = isObservable(4);
        s.leftMm = left;
        s.frontMm = front;
        s.rightMm = right;
        s.leftWall  = s.leftValid && (left  < _wallThreshold);
        s.rightWall = s.rightValid && (right < _wallThreshold);
        s.frontWall = s.frontValid && (front < _wallThreshold);
    }
    else if (_version == SENSOR_V2) {
        uint16_t fl = getDistance(0);
        uint16_t l  = getDistance(1);
        uint16_t r  = getDistance(2);
        uint16_t fr = getDistance(3);

        uint8_t flState = stateTimeout(0);
        uint8_t frState = stateTimeout(3);
        bool flValid = isObservable(0);
        bool frValid = isObservable(3);
        bool flFar = flState == 2;
        bool frFar = frState == 2;
        // V2 compatibility mode: S3 (front-right) is only trusted when
        // S0 (front-left) is observable. If S0 is far, force S3 to far too.
        if (!flValid) {
            frValid = false;
            frFar = false;
        } else if (flFar) {
            frValid = true;
            frFar = true;
            fr = AppConfig::Tof::DIST_FAR_MM;
        }
        s.leftValid  = isObservable(1);
        s.rightValid = isObservable(2);
        s.frontValid = flValid || frValid || (flFar && frFar);
        s.leftMm = l;
        s.rightMm = r;
        if (flValid && frValid) s.frontMm = min(fl, fr);
        else if (flValid) s.frontMm = fl;
        else if (frValid) s.frontMm = fr;
        else if (flFar && frFar) s.frontMm = AppConfig::Tof::DIST_FAR_MM;

        s.leftWall  = s.leftValid && (l < _wallThreshold);
        s.rightWall = s.rightValid && (r < _wallThreshold);
        s.frontWall = s.frontValid && (s.frontMm < _wallThreshold);
    }

    return s;
}

// ---- getters ----
bool MultiVL53L0X::isSensorOk(uint8_t index) const {
    if (index >= _numSensors) return false;
    return _initialized[index];
}

uint16_t MultiVL53L0X::getDistance(uint8_t index) const {
    return effectiveDistance_(index);
}

uint16_t MultiVL53L0X::getRaw(uint8_t index) const {
    if (index >= _numSensors) return 0;
    return _raw[index];
}

uint8_t MultiVL53L0X::stateTimeout(uint8_t index) const {
    return effectiveState_(index);
}

uint8_t MultiVL53L0X::getSensorAddress(uint8_t index) const {
    return (index < _numSensors) ? _sensorAddresses[index] : 0;
}

// ---- XSHUT ----
void MultiVL53L0X::xshutAllLow() {
    for (uint8_t i = 0; i < _numSensors; i++) {
        _pcf.write(_xshutPins[i], LOW);
    }
}

void MultiVL53L0X::xshutAllHigh() {
    for (uint8_t i = 0; i < _numSensors; i++) {
        _pcf.write(_xshutPins[i], HIGH);
    }
}

float MultiVL53L0X::computeError(float headingError) {
    if (AppConfig::Tof::COMPUTE_HEADING_FROM_FULL_SWEEP) {
        if (!_sweepReadyForCompute) {
            return error;
        }
        _sweepReadyForCompute = false;
    }

    auto isGood = [&](uint8_t state) {
        return state == 0 || state == 1;  // valid or clipped-min
    };

    uint16_t left = AppConfig::Tof::DIST_ERROR_MM;
    uint16_t right = AppConfig::Tof::DIST_ERROR_MM;

    const uint16_t effectiveSideMax = AppConfig::Motion::CENTER_PID_EFFECTIVE_SIDE_MAX_MM;

    uint8_t leftState = 3;
    uint8_t rightState = 3;

    if (_version == SENSOR_V1) {
        left  = min(getDistance(0), effectiveSideMax);
        right = min(getDistance(4), effectiveSideMax);
        leftState  = stateTimeout(0);
        rightState = stateTimeout(4);
    }
    else if (_version == SENSOR_V2) {
        left  = min(getDistance(1), effectiveSideMax);
        right = min(getDistance(2), effectiveSideMax);
        leftState  = stateTimeout(1);
        rightState = stateTimeout(2);
    }

    const bool leftValid  = isGood(leftState);
    const bool rightValid = isGood(rightState);
    const bool leftWallTrackable = leftValid && (left < _wallThreshold);
    const bool rightWallTrackable = rightValid && (right < _wallThreshold);
    const bool dualWallValid = leftWallTrackable && rightWallTrackable;
    const bool forceLeft = _straightTrackMode == TRACK_LEFT;
    const bool forceRight = _straightTrackMode == TRACK_RIGHT;
    const bool forceDual = _straightTrackMode == TRACK_DUAL;
    const bool forceNone = _straightTrackMode == TRACK_NONE;
    const bool shouldCaptureTargets = _captureCenterTargetsOnFirstSample;
    _captureCenterTargetsOnFirstSample = false;

    float dualErr = 0.0f;
    if (dualWallValid) {
        if (shouldCaptureTargets &&
            fabsf((float)left - (float)right) <= AppConfig::Motion::CENTER_TARGET_CAPTURE_WINDOW_MM) {
            _centerTargetLeft = 0.8f * _centerTargetLeft + 0.2f * (float)left;
            _centerTargetRight = 0.8f * _centerTargetRight + 0.2f * (float)right;
        }
        dualErr = 0.5f * ((_centerTargetLeft - (float)left) +
                          ((float)right - _centerTargetRight));
        _lastDualWallError = dualErr;
    }

    float singleErr = headingError;
    if (leftWallTrackable && !rightWallTrackable) {
        singleErr = _centerTargetLeft - (float)left;
    } else if (rightWallTrackable && !leftWallTrackable) {
        singleErr = (float)right - _centerTargetRight;
    }

    const uint32_t nowUs = micros();
    const uint32_t fallbackUs = max(1000UL, (uint32_t)AppConfig::Tof::UPDATE_INTERVAL_MS * 1000UL);
    uint32_t elapsedUs = (_centerPrevUs == 0) ? fallbackUs : (nowUs - _centerPrevUs);
    if (elapsedUs == 0) elapsedUs = 1;
    float dt = max(0.0005f, elapsedUs / 1e6f);

    if (!_centerPidPrimed) {
        _centerIntegral = 0.0f;
        _centerPrevError = 0.0f;
        if (forceDual) {
            _centerRawFiltered = dualWallValid ? dualErr : headingError;
            _dualWallBlend = dualWallValid ? 1.0f : 0.0f;
        } else if (forceLeft) {
            _centerRawFiltered = leftValid ? (_centerTargetLeft - (float)left) : headingError;
            _dualWallBlend = 0.0f;
        } else if (forceRight) {
            _centerRawFiltered = rightValid ? ((float)right - _centerTargetRight) : headingError;
            _dualWallBlend = 0.0f;
        } else {
            _centerRawFiltered = headingError;
            _dualWallBlend = 0.0f;
        }
        _centerPidPrimed = true;
    }

    const float blendTarget = (forceDual && dualWallValid) ? 1.0f : 0.0f;
    const float blendTauSec = AppConfig::Motion::CENTER_BLEND_TAU_SEC;
    const float blendAlpha = dt / (blendTauSec + dt);
    _dualWallBlend += (blendTarget - _dualWallBlend) * blendAlpha;
    if (_dualWallBlend < 0.0f) _dualWallBlend = 0.0f;
    if (_dualWallBlend > 1.0f) _dualWallBlend = 1.0f;

    float targetRawErr = headingError;
    if (forceDual) {
        if (dualWallValid) {
            targetRawErr = dualErr;
        } else if (leftWallTrackable) {
            // Corridor transition: if dual tracking loses the right wall, keep following left.
            targetRawErr = _centerTargetLeft - (float)left;
        } else if (rightWallTrackable) {
            // Corridor transition: if dual tracking loses the left wall, keep following right.
            targetRawErr = (float)right - _centerTargetRight;
        } else {
            targetRawErr = headingError;
        }
    } else if (forceLeft) {
        targetRawErr = leftWallTrackable ? (_centerTargetLeft - (float)left) : headingError;
    } else if (forceRight) {
        targetRawErr = rightWallTrackable ? ((float)right - _centerTargetRight) : headingError;
    } else if (!forceNone) {
        if (dualWallValid) {
            targetRawErr = dualErr;
        } else if (leftWallTrackable || rightWallTrackable) {
            targetRawErr = (_dualWallBlend * _lastDualWallError) +
                           ((1.0f - _dualWallBlend) * singleErr);
        }
    }

    // In one-wall tracking, clamp target error to avoid aggressive steering
    // when the opposite side is open/far.
    if (!dualWallValid && (leftWallTrackable || rightWallTrackable)) {
        const float lim = AppConfig::Motion::CENTER_PID_SINGLE_WALL_ERR_LIMIT_MM;
        if (lim > 0.0f) {
            if (targetRawErr > lim) targetRawErr = lim;
            if (targetRawErr < -lim) targetRawErr = -lim;
        }
    }

    const float rawTauSec = AppConfig::Motion::CENTER_RAW_TAU_SEC;
    const float rawAlpha = dt / (rawTauSec + dt);
    _centerRawFiltered += (targetRawErr - _centerRawFiltered) * rawAlpha;
    const float rawErr = _centerRawFiltered;

    _centerIntegral += rawErr * dt;
    if (_centerIntegral > _centerILimit) _centerIntegral = _centerILimit;
    if (_centerIntegral < -_centerILimit) _centerIntegral = -_centerILimit;

    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (rawErr - _centerPrevError) / dt;
        const float dLim = AppConfig::Motion::CENTER_PID_DERIV_LIMIT;
        if (derivative > dLim) derivative = dLim;
        if (derivative < -dLim) derivative = -dLim;
    }

    const float pTerm = _centerKp * rawErr;
    const float iTerm = _centerKi * _centerIntegral;
    const float dTerm = _centerKd * derivative;
    float out = pTerm + iTerm + dTerm;

    if (out > _centerOutLimit) out = _centerOutLimit;
    if (out < -_centerOutLimit) out = -_centerOutLimit;

    _centerPrevError = rawErr;
    _centerPrevUs = nowUs;
    error = out;

    if (AppConfig::Debug::CENTER_PID_TRACE && _logFn != nullptr) {
        const uint8_t everyN = (AppConfig::Debug::CENTER_PID_TRACE_EVERY_N == 0)
                             ? 1
                             : AppConfig::Debug::CENTER_PID_TRACE_EVERY_N;
        sCenterPidTraceCounter++;
        if ((sCenterPidTraceCounter % everyN) == 0) {
            const char* mode = "none";
            switch (_straightTrackMode) {
                case TRACK_LEFT: mode = "left"; break;
                case TRACK_RIGHT: mode = "right"; break;
                case TRACK_DUAL: mode = "dual"; break;
                case TRACK_NONE:
                default: mode = "none"; break;
            }
            char buf[220];
            snprintf(buf, sizeof(buf),
                     "CPID mode=%s L=%u(%u) R=%u(%u) raw=%.3f tgt=%.3f h=%.3f P=%.3f I=%.3f D=%.3f out=%.3f",
                     mode,
                     (unsigned)left, (unsigned)leftState,
                     (unsigned)right, (unsigned)rightState,
                     rawErr, targetRawErr, headingError,
                     pTerm, iTerm, dTerm, out);
            _logFn(String(buf));
        }
    }

    return out;
}

bool MultiVL53L0X::isGoodReading_(uint8_t index) const {
    if (index >= _numSensors) return false;
    uint8_t state = stateTimeout(index);
    return state == 0 || state == 1;
}

````

### `MultiVL53L0X.h`

````cpp
#ifndef MULTIVL53L0X_H
#define MULTIVL53L0X_H

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <VL53L0X.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class MultiVL53L0X {
public:
    using LogFn = void (*)(const String&);
    static const uint8_t MAX_SENSORS = 8;

    enum SensorVersion {
        SENSOR_UNKNOWN = 0,
        SENSOR_V1,   // 3 sensors: left(0), front(2), right(4)
        SENSOR_V2    // 4 sensors: FL(0), L(1), R(2), FR(3)
    };

    enum StraightTrackMode : uint8_t {
        TRACK_NONE = 0,
        TRACK_LEFT,
        TRACK_RIGHT,
        TRACK_DUAL
    };

    struct SensorState {
        bool leftWall;
        bool rightWall;
        bool frontWall;
        bool leftValid;
        bool rightValid;
        bool frontValid;
        uint16_t leftMm;
        uint16_t rightMm;
        uint16_t frontMm;
    };

    MultiVL53L0X(uint8_t pcfAddress,
                 uint8_t numSensors,
                 const uint8_t* xshutPins,
                 const uint8_t* sensorAddresses,
                 uint16_t updateIntervalMs,
                 TwoWire& wire = Wire);

    bool begin();
    void update();

    // ---- New ----
    void detectLayout();
    SensorVersion getVersion() const { return _version; }
    SensorState getSensorState();
    void setWallThreshold(uint16_t th) { _wallThreshold = th; }
    uint16_t wallThreshold() const { return _wallThreshold; }
    void setCenterPid(float kp, float ki, float kd, float iLimit, float outLimit);
    void setCenterTargets(float leftMm, float rightMm);
    void resetCenterPid();
    void setStraightTrackMode(StraightTrackMode mode) { _straightTrackMode = mode; }
    StraightTrackMode straightTrackMode() const { return _straightTrackMode; }

    // ---- Sensors ----
    bool     isSensorOk(uint8_t index) const;
    uint16_t getDistance(uint8_t index) const;
    uint16_t getRaw(uint8_t index) const;
    uint8_t  stateTimeout(uint8_t index) const;
    uint8_t  getSensorAddress(uint8_t index) const;
    uint8_t  sensorCount() const { return _numSensors; };
    float computeError(float headingError = 0.0f);
    float getError(float headingError = 0.0f) {return error; };

    void setMutex(SemaphoreHandle_t m) {_i2cMutex = m;}
    void setLog(LogFn fn) { _logFn = fn; }

    bool readTOF_fast(uint8_t addr, uint16_t &dist);
    
    uint16_t err_count = 0;

private:
    // I2C
    TwoWire*  _wire;

    SemaphoreHandle_t _i2cMutex = nullptr;
    inline void i2cLock()   { if (_i2cMutex) xSemaphoreTake(_i2cMutex, portMAX_DELAY); }
    inline void i2cUnlock() { if (_i2cMutex) xSemaphoreGive(_i2cMutex); }

    // PCF8574
    PCF8574   _pcf;
    uint8_t   _pcfAddress;

    // Sensors
    uint8_t   _numSensors;
    uint16_t  _intervalMs;
    const uint8_t* _xshutPins;
    const uint8_t* _sensorAddresses;

    VL53L0X  _sensors[MAX_SENSORS];
    bool     _initialized[MAX_SENSORS];
    uint8_t  _timeoutFlag[MAX_SENSORS];
    uint16_t _raw[MAX_SENSORS];
    uint16_t _lastDistance[MAX_SENSORS];
    int      _cSensor = 0;
    bool _sweepReadyForCompute = false;

    SensorVersion _version = SENSOR_UNKNOWN;

    // S1 range: 62 -> 150, so range = 88, scale = 97/88, center = 97
    // S2 range: 30 -> 127, so range = 97, absolute minimum = (62 - 30) / 2 = 16

    //37 183; 129 94

    uint16_t _wallThreshold = 150;

    float error = 0;
    float _centerKp = 1.0f;
    float _centerKi = 0.0f;
    float _centerKd = 0.0f;
    float _centerILimit = 50.0f;
    float _centerOutLimit = 50.0f;
    float _centerIntegral = 0.0f;
    float _centerPrevError = 0.0f;
    float _centerRawFiltered = 0.0f;
    float _lastDualWallError = 0.0f;
    float _dualWallBlend = 0.0f;
    float _centerTargetLeft = 100.0f;
    float _centerTargetRight = 100.0f;
    bool _captureCenterTargetsOnFirstSample = true;
    StraightTrackMode _straightTrackMode = TRACK_NONE;
    uint32_t _centerPrevUs = 0;
    bool _centerPidPrimed = false;
    LogFn _logFn = nullptr;
    
    // Helpers
    void xshutAllLow();
    void xshutAllHigh();
    bool isGoodReading_(uint8_t index) const;
    uint8_t effectiveState_(uint8_t index) const;
    uint16_t effectiveDistance_(uint8_t index) const;
};

#endif
````

### `PersistenceStore.cpp`

````cpp
#include "PersistenceStore.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SPIFFS.h>
#include <cstring>

namespace PersistenceStore {

namespace {

constexpr uint32_t kMazeMagic = 0x4D415A31u;  // MAZ1
constexpr const char* kMazePath = "/maze_state.bin";

struct PersistedMazeData {
  uint32_t magic;
  uint8_t walls[FloodFillExplorer::N][FloodFillExplorer::N];
  uint8_t mask[FloodFillExplorer::N][FloodFillExplorer::N];
  uint8_t visited[FloodFillExplorer::N][FloodFillExplorer::N];
};

LogFn gLogFn = nullptr;
bool gInitialized = false;
bool gReady = false;
SemaphoreHandle_t gSpiMutex = nullptr;
PersistedMazeData gScratch{};

void logLine(const String& s) {
  if (gLogFn) gLogFn(s);
}

bool ensureMutex() {
  if (gSpiMutex) return true;
  gSpiMutex = xSemaphoreCreateMutex();
  if (!gSpiMutex) {
    logLine("[SPIFFS] mutex create failed");
    return false;
  }
  return true;
}

bool lockSpi() {
  if (!ensureMutex()) return false;
  return xSemaphoreTake(gSpiMutex, pdMS_TO_TICKS(2000)) == pdTRUE;
}

void unlockSpi() {
  if (gSpiMutex) xSemaphoreGive(gSpiMutex);
}

}  // namespace

void setLogger(LogFn fn) {
  gLogFn = fn;
}

bool begin() {
  if (!ensureMutex()) return false;
  if (!lockSpi()) return false;
  if (gInitialized) {
    const bool ready = gReady;
    unlockSpi();
    return ready;
  }
  gInitialized = true;
  gReady = SPIFFS.begin(true);
  logLine(String("[SPIFFS] ") + (gReady ? "ready" : "failed"));
  unlockSpi();
  return gReady;
}

bool loadMaze(FloodFillExplorer& explorer, uint8_t mouseX, uint8_t mouseY, FloodFillExplorer::Dir mouseH) {
  if (!begin()) return false;
  if (!lockSpi()) return false;
  File f = SPIFFS.open(kMazePath, FILE_READ);
  if (!f) {
    unlockSpi();
    return false;
  }

  memset(&gScratch, 0, sizeof(gScratch));
  const size_t n = f.readBytes(reinterpret_cast<char*>(&gScratch), sizeof(gScratch));
  f.close();
  unlockSpi();
  if (n != sizeof(gScratch) || gScratch.magic != kMazeMagic) {
    logLine("[SPIFFS] maze restore skipped (invalid file)");
    return false;
  }

  if (!explorer.importKnownMaze(gScratch.walls, gScratch.mask, gScratch.visited, mouseX, mouseY, mouseH)) {
    logLine("[SPIFFS] maze restore failed");
    return false;
  }

  logLine("[SPIFFS] restored maze memory");
  return true;
}

bool saveMaze(const FloodFillExplorer& explorer) {
  if (!begin()) return false;
  if (!lockSpi()) return false;
  if (SPIFFS.exists(kMazePath)) {
    SPIFFS.remove(kMazePath);
  }

  File f = SPIFFS.open(kMazePath, FILE_WRITE);
  if (!f) {
    unlockSpi();
    logLine("[SPIFFS] failed to open maze file for write");
    return false;
  }

  memset(&gScratch, 0, sizeof(gScratch));
  gScratch.magic = kMazeMagic;
  explorer.exportKnownMaze(gScratch.walls, gScratch.mask, gScratch.visited);

  const bool ok = f.write(reinterpret_cast<const uint8_t*>(&gScratch), sizeof(gScratch)) == sizeof(gScratch);
  f.close();
  unlockSpi();
  if (!ok) {
    logLine("[SPIFFS] failed to save maze memory");
    return false;
  }

  logLine("[SPIFFS] saved maze memory");
  return true;
}

bool clearMaze() {
  if (!begin()) return false;
  if (!lockSpi()) return false;
  if (!SPIFFS.exists(kMazePath)) {
    unlockSpi();
    return true;
  }
  const bool ok = SPIFFS.remove(kMazePath);
  unlockSpi();
  logLine(ok ? "[SPIFFS] cleared saved maze memory" : "[SPIFFS] failed to clear saved maze memory");
  return ok;
}

}  // namespace PersistenceStore
````

### `PersistenceStore.h`

````cpp
#pragma once

#include <Arduino.h>

#include "FloodFillExplorer.h"

namespace PersistenceStore {

using LogFn = void(*)(const String&);

void setLogger(LogFn fn);
bool begin();
bool loadMaze(FloodFillExplorer& explorer, uint8_t mouseX, uint8_t mouseY, FloodFillExplorer::Dir mouseH);
bool saveMaze(const FloodFillExplorer& explorer);
bool clearMaze();

}  // namespace PersistenceStore
````

### `README.md`

````md
# Mouse_esp32s3

ESP32-S3 micromouse project for a floodfill-based maze runner.

Current project version: `0.4.1`

## Current Status

This repository now includes the first integrated hardware-oriented control stack:
- dual DC motor control with encoder-based speed PID
- multi-VL53L0X wall sensing
- battery monitoring with warning/critical telemetry states
- primitive motion executor for `move`, `back`, `turn 90 deg`, and `turn 180 deg`
- floodfill maze state and web visualizer
- floodfill maze web sync over WebSocket instead of browser polling
- task-based planner / executor / telemetry flow
- Wi-Fi OTA, port `80` control page, and telnet/debug tools
- centralized configuration for hardware and tuning values
- Wi-Fi boot logging now waits for a real STA IP before printing HTTP/upload URLs, and the reconnect loop no longer retries while the station is already connecting
- SPIFFS persistence for saved maze memory when the shortest path is known
- build fixes for the SPIFFS persistence integration on the current ESP32 core
- when a saved maze is restored successfully at boot, the robot marks shortest-path-ready immediately and shows white LED in idle
- explore now uses the current pose as the active home target for goal/home swapping, so `resetpose` affects the next explore loop as expected
- far/open TOF readings now count as valid maze observations, so revisits can clear stale remembered walls for recovery
- battery monitoring is now telemetry-only and no longer blocks or aborts motion primitives
- pose and goal are runtime-only again; SPIFFS now stores only maze wall memory
- SPIFFS persistence now lives in a dedicated module for easier control and future changes
- wall-centering now blends smoothly when transitioning between both-wall centering and single-wall following
- wall-centering now uses left-target and right-target references consistently for both dual-wall and single-wall follow
- wall-centering now captures left/right center targets once near the start of each straight move, only when both walls are visible and already within the configured `5 mm` balance window
- wall-centering correction is now tuned through the center-wall `KP` term and correction output limit, while leaving `CORRIDOR_CENTERING_GAIN` unchanged
- motion speed targets in `Config.h` are now standardized to `350 TPS` for move, short-forward, reverse, and turn primitives, with corridor speed kept at `450 TPS`
- the shortest-path-known rule now triggers after `1` stable goal->home round trip with the same best-known cost
- the ESP32-S3 BOOT button now supports a 5-second multi-press launcher from idle with LED-cycle feedback on each accepted press
- BOOT-button `1` press now starts `explore` without clearing the known maze first
- the serial `explore` and `explore n` commands now also keep the known maze, matching the BOOT-button and web explore entry paths; `clearmaze` is the explicit wall-memory reset command
- the browser uploader on port `82` now uses the chunked HTTP retry path again, streaming firmware through `/upload/start`, `/upload/chunk`, and `/upload/finish`
- legacy firmware-upload WebSocket transport has been removed; firmware upload now uses HTTP chunk endpoints only
- the browser uploader now uses chunked HTTP with retry, adaptive chunk-size fallback, and fixed retry backoff/pacing
- OTA/web upload status LED is now solid `blue` while receiving and forced `off` on success
- interrupted browser uploads now abort cleanly and force the LED `red` on failure/abort paths
- long straight `move N` actions that are known to end at a wall now finish on the front-wall stop distance instead of stopping only on encoder distance
- OTA safe mode now suspends the motor, TOF, explorer, planner, and telemetry tasks entirely during upload, then resumes them afterward for a quieter and more stable transfer path
- the dedicated Wi-Fi/OTA service task is currently pinned to core `0` by config
- the Wi-Fi service loop currently runs with a `50 ms` cadence during normal service
- Wi-Fi reconnect recovery now escalates from normal `WiFi.reconnect()` attempts to a full STA restart and fresh `WiFi.begin(...)` after a longer disconnect, so the robot can recover from wedged network states without a power cycle
- Wi-Fi reconnect attempts now run only while the robot is idle; active explore/speedrun/test motion leaves the link alone so reconnect churn does not destabilize motion
- manual LED commands now also support `yellow` and `magenta`
- the port `80` control page now also has an `Open Upload` button that jumps straight to the browser firmware upload page on port `82`
- the port `80` command guide now combines `explore` and `explore n` into one line: explore until the shortest path is known, or stop after `n` forward moves
- floodfill web on port `81` now has a `Clear Maze` button that triggers the same robot-side `clearmaze` behavior (runtime + saved maze memory)
- added `test motor both` for a simple full-power forward/reverse bench loop on both motors
- compact status printing can now hide `tps=(left,right)` with a config flag when motor-speed text is too noisy
- serial output can now be globally muted with a config flag while keeping the serial port open for input
- `speedrun [1-4]` is phase-aware: `speedrun 1` is the round-trip home->goal->home profile, `speedrun 2` is the one-way home->goal profile, and phases `3-4` inherit the previous phase until tuned
- `speedrun 1` now temporarily mutes serial output while the run is active, then restores it automatically on goal/idle/fault
- `speedrun` now rebuilds its start/home target and goal target from the current runtime pose and current runtime goal before the run begins
- `speedrun` still means `speedrun 1`, and `speedrun 2` runs the known shortest path one-way from home to goal with its dedicated profile selection
- fixed the `speedrun 1` serial-mute build path by wiring the Wi-Fi serial mirror code to the shared config header
- the floodfill web now shows live leg timing for both `HG` and `GH`, and keeps lap history in RAM across runs until reboot/reset
- fixed the intermediate `speedrun 1` goal-flip path so a completed move is cleared before the return-home leg begins, preventing an extra logical cell advance
- `speedrun 1` keeps the normal per-motion hard-stop and stop-hold behavior, matching `explore` while keeping the same no-ACK shortest-path planner flow
- `speedrun 2` now executes the known shortest path directly as `move N -> turn left/right -> ... -> goal`, without explore-style ACK/wall-apply steps and without per-primitive stop-hold pauses
- `speedrun 2` now treats an unexpected shortest-path `uturn` as a fault instead of executing it as a normal high-speed action
- every speedrun phase now performs at most one untimed pre-run snap-center (no pre-run side turns) before the lap starts, and skips it when the known back wall condition is not met
- periodic RTOS task loops now have a lightweight watchdog that warns when a loop misses its expected cadence, including task name, expected period, actual interval, lateness, and core id
- the RTOS loop watchdog now only warns for time-critical loops running on core `0`; core `1` loops are treated as delay-tolerant and no longer emit cadence warnings
- `explorerTask` now uses `vTaskDelayUntil(...)` in normal operation so it follows the same fixed-cadence scheduling rule as the other steady-state task loops
- global Serial output remains configurable with `AppConfig::Debug::ENABLE_SERIAL_OUTPUT`, while `speedrun 1` still temporarily mutes Serial only during the active run when Serial output is enabled
- floodfill forward planning and runtime motion now support long straight corridors as `move N`, including manual `move [n]`, planner-emitted multi-cell runs, and cell-by-cell logical commits during explore so maze updates still happen per crossed cell
- startup LED behavior now shows `red` during setup, then on ready idle shows `white` if shortest-path-ready is loaded, otherwise `off`
- low-pass smoothing constants for TOF distance updates, motor TPS estimate, and center-track blending are now configurable in `Config.h`
- HTTP firmware upload now uses a dedicated upload service task during active chunk transfer to improve upload stability under load
- HTTP firmware upload now pre-erases the OTA target partition before transfer and sets TCP no-delay on upload requests to improve throughput consistency
- latched straight-track mode is now enabled only for `speedrun 2`; `explore` and `speedrun 1` use live wall availability each cycle for safer unknown-cell transitions
- front-stop threshold now uses `FRONT_STOP_MM` when effective move target is 1 cell, and `CORRIDOR_FRONT_STOP_MM` only for true multi-cell corridor runs
- runtime forward-cell commit logic now matches that rule too, so `move 1` completion uses `FRONT_STOP_MM` consistently in both motion and planner-commit paths
- center PID side-distance clamp is now configurable via `CENTER_PID_EFFECTIVE_SIDE_MAX_MM`, and V2 front-right (S3) is now treated independently (no S0 dependency workaround)

This is a bring-up and integration version, not a race-tuned final solver yet.

Release note:
- see [RELEASE_0.3.0.md](RELEASE_0.3.0.md) for the packaged solver milestone summary and the next target after this release

## Architecture

### Entry and application split
- [Mouse_esp32s3.ino](Mouse_esp32s3.ino): thin Arduino entrypoint with `setup()`, `loop()`, `userTask()`, and `plannerTask()` wrappers only
- [AppRuntime.h](AppRuntime.h): app interface exposed to the `.ino` wrapper
- [AppRuntime.cpp](AppRuntime.cpp): application logic, globals, startup flow, command handling, background tasks, planner integration
- [Config.h](Config.h): centralized hardware pins, thresholds, Wi-Fi settings, and motion tuning constants
- Build profile: set `APP_LITE_FIRMWARE` in [Config.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Config.h) (`0` full / `1` lite) to use the minimal runtime profile (Wi-Fi/OTA/web disabled by default)
- [RobotTypes.h](RobotTypes.h): shared enums and `RobotState`

### Motion and hardware
- [DcMotor.h](DcMotor.h): low-level motor + encoder + speed PID
- [DcMotor.cpp](DcMotor.cpp): PWM / ISR / TPS estimation
- [MotionController.h](MotionController.h): primitive motion interface
- [MotionController.cpp](MotionController.cpp): move / turn / stop execution and fault detection
- [Battery.h](Battery.h): battery voltage API
- [Battery.cpp](Battery.cpp): ADC sampling and battery-state classification

### Sensors and planning
- [MultiVL53L0X.h](MultiVL53L0X.h): TOF array API and wall observation structure
- [MultiVL53L0X.cpp](MultiVL53L0X.cpp): sensor reads, correction, thresholding, wall interpretation
- [FloodFillExplorer.h](FloodFillExplorer.h): floodfill planner / map / web interface
- [FloodFillExplorer.cpp](FloodFillExplorer.cpp): planner logic and live web UI
- [PersistenceStore.h](PersistenceStore.h): SPIFFS persistence interface for saved maze memory
- [PersistenceStore.cpp](PersistenceStore.cpp): SPIFFS file format and load/save/clear implementation

### Connectivity
- [WiFiOtaWebSerial.h](WiFiOtaWebSerial.h): OTA and lightweight port `80` control page API
- [WiFiOtaWebSerial.cpp](WiFiOtaWebSerial.cpp): Wi-Fi task, control page, Arduino OTA, browser upload page, LED control

## Wi-Fi Feature Surface

With Wi-Fi enabled, this project already provides:

- HTTP control page for command input and runtime visibility
- WebSocket stream for live maze/state synchronization
- Telnet-style TCP debug console
- Arduino OTA and browser-based OTA upload path
- mDNS service discovery (`<hostname>.local`)

High-value Wi-Fi features that can be added incrementally:

1. REST API for state/config (`/api/state`, `/api/config`, `/api/motion`)
2. Live dashboard plotting (TPS, PID terms, TOF distances, battery)
3. Structured log streaming endpoint (machine-friendly JSON lines)
4. Remote tuning profile save/load and compare
5. Remote queue/mission submit API for scripted test runs
6. Run record/replay tooling for tuning and regression checks
7. Web calibration flows (TOF calibration, mm-per-tick, PID assist)
8. Metrics endpoint for external monitoring
9. Access control (viewer/operator/admin) and command authorization
10. Remote safety controls (heartbeat + kill-switch)

Design note:
- Keep Wi-Fi callbacks lightweight and move heavy work into normal app tasks.
- Realtime motor/TOF behavior should stay deterministic even when network traffic is active.

## BLE Options (Optional / Future)

BLE is not enabled in the current firmware, but can be added for short-range control and telemetry.

Useful BLE feature ideas:

1. BLE GATT telemetry service (battery, pose, walls, mode)
2. BLE command/control service (start/stop/explore/speedrun)
3. BLE tuning service for selected runtime parameters
4. BLE provisioning (set Wi-Fi credentials from phone app)
5. BLE fallback control path when Wi-Fi is unavailable
6. BLE notifications for fault/state change events

Recommendation:
- Treat BLE as a secondary control channel; keep motion safety and planner authority in the same runtime core logic used by serial/Wi-Fi commands.

## Runtime Flow

1. `setup()` in the `.ino` forwards to `MainApp::setupApp(...)`.
2. `AppRuntime.cpp` initializes Wi-Fi, motors, TOF, battery, floodfill explorer, and tasks.
3. The robot does not write start-cell walls into the maze at boot; first wall observation happens when exploration or speed-run logic begins.
4. `tofTask` continuously updates TOF readings.
5. `motorTask` continuously updates motor PID loops.
6. `userTask()` remains visible in the `.ino`, but forwards to `MainApp::userTaskBody(...)`.
7. `plannerTask()` remains visible in the `.ino`, but forwards to `MainApp::plannerTaskBody(...)`.
8. `explore` only starts with `snapCenter()` when the wall behind the robot is already known to exist; otherwise the run-start snap is skipped and the planner is allowed to continue immediately.
9. After a motion completes in explore hardware mode, the runtime refreshes robot sensor state, applies wall sensing for the new pose once, ACKs the pending planner action, and only then holds the motors in hard-stop briefly before allowing the next motion.
10. After a 180-degree turn in explore hardware mode, if the wall behind the robot is known to exist, the runtime runs `snapCenter()` before wall registration and before ACKing the turn so the next planner action starts from the re-centered pose.
11. `speedrun 1` uses the shortest known path as a round-trip run: home -> goal -> home, with no wall-map updates and no floodfill ACK handshake in the motion loop.
12. `speedrun 2` runs one-way from home to goal with its own motion profile and continuous shortest-path execution, so the runtime syncs pose and dispatches the next action directly instead of using the explore ACK loop.
13. `telemetryTask` now focuses on the selected manual-test loop output instead of always printing the compact status line every cycle.
14. `explorerTask` serves the web maze view and now runs on a fixed `vTaskDelayUntil(...)` cadence during normal operation.
15. When the robot is standing still and ready for the next planner action, the runtime refreshes wall sensing from the current cell before calling floodfill again, so valid current-cell observations can overwrite stale wall memory.
16. In explore mode, the runtime can continue after a reached target by keeping the current pose, letting `FloodFillExplorer` flip the target between the original goal rectangle and the original home rectangle, and then resuming exploration from where the robot stands.
17. Explore now stops automatically and prints that the shortest path is known once the same best-known home-to-goal cost has remained unchanged for the configured number of consecutive round trips.
18. Floodfill now distinguishes the single physical start pose from a separate home rectangle, so target toggling happens between the configured home region and goal region.
19. When explore continues immediately after a reached target, the runtime now skips the normal post-motion hold so the goal-to-home transition starts without the extra 100 ms pause.
20. On boot, the runtime restores saved maze wall memory from SPIFFS when that file exists, while pose and goal still come from the current runtime/config state.
21. `clearmaze` clears only wall memory and removes the saved maze file without changing the current pose or goal.
22. When explore decides the shortest path is known, the runtime saves maze memory to SPIFFS automatically before going idle.

Planner synchronization note:
- `plannerTaskBody()` now uses `MotionController` as the single source of truth for motion completion/busy state before dispatching the next action.
- This prevents a race where `robotState.motionStatus` could still be stale while the controller had already left `RUNNING`, which could otherwise cause repeated `move1` starts before wall sensing and ACK completed.

Loop watchdog note:
- steady-state RTOS loops now log `[LOOP WARN]` when the actual loop interval exceeds the expected period plus a configurable tolerance
- the warning includes the task name, expected period, actual interval, lateness, and current core id
- setup delays, one-shot waits, and OTA special pause branches still use plain `vTaskDelay(...)` and are not treated as periodic watchdog loops

Turn behavior note:
- Floodfill explore uses a real `ACT_TURN_180` again for dead-end reversals.
- When a completed turn will be followed by `snapCenter()`, walls are now sensed and applied only after `snapCenter()` finishes, so the maze is updated from the re-centered pose instead of the immediate post-turn pose.

Explore loop note:
- `FloodFillExplorer` now toggles its target between the original goal rectangle and the original home rectangle when a target is reached.
- In hardware explore mode, the runtime now re-enables exploring after a reached target when `AppConfig::Explorer::CONTINUE_AFTER_GOAL` is `true`.
- This keeps the robot at its current pose and lets return trips keep discovering or correcting wall memory.
- When returning to the home target, the planner now requires the robot to match the configured start heading before the home leg is considered complete, so the next run starts with the same orientation as boot.
- The runtime tracks the best-known cost from the original home region to the original goal region.
- After each completed goal->home round trip, if that best-known cost is unchanged, the stable round-trip count increases.
- When the count reaches `AppConfig::Explorer::SHORTEST_PATH_STABLE_ROUND_TRIPS`, explore stops and prints `shortest path known`.
- From idle, the ESP32-S3 BOOT button can launch runs after a `5s` multi-press timeout:
  - `1` press: `explore`
  - `2` presses: `speedrun 1`
  - `3` presses: `speedrun 2`
  - `4` presses: `speedrun 3`
  - `5` presses: `speedrun 4`
  - each accepted press cycles the LED so the user gets immediate feedback
- Reaching a target no longer rewrites the explorer's configured start marker; the original start remains fixed while only the active target toggles between goal and home.
- The original home/goal references used for toggling are now taken from `setHomeRect()` and `setGoalRect()`, so the loop uses the configured maze values instead of the class default placeholders.

## Configuration

All hard configuration now lives in [Config.h](Config.h).

Key sections:
- `AppConfig::Battery`: ADC pin, battery calibration, warning/critical thresholds
- `AppConfig::Maze`: start pose and goal rectangle
  Default config starts at `(0,0)`, heading south, with a home rectangle at `(0,0)` size `1x1` and a goal rectangle defined in `Config.h`.
- `AppConfig::Wifi`: Wi-Fi / OTA / control-page settings
- `AppConfig::Debug`: serial-output switches, runtime debug flags, and loop-watchdog timing thresholds
- `AppConfig::I2C`: SDA/SCL and bus speed
- `AppConfig::Tof`: sensor addresses, XSHUT pins, and wall threshold
- `AppConfig::Motors`: motor pins, encoder inversion, PWM and PID settings
- `AppConfig::Motion`: one-cell distance, turn ticks, speed and timeout tuning
- `AppConfig::Explorer`: web floodfill UI settings, whether explore should continue looping between goal and home after a target is reached, and how many stable round trips are required before the shortest path is considered known

## Persistence

- maze wall memory is saved in SPIFFS automatically when explore reports `shortest path known`
- on boot, the runtime restores saved maze wall memory if the SPIFFS file is present
- `clearmaze` now clears only the remembered wall map and deletes the saved maze-memory file; it no longer resets pose or goal

## Dependencies / Build Expectations

- the sketch is intended to build from the repository root as a standard Arduino sketch folder
- target platform is an ESP32-S3 board using the Arduino ESP32 core
- external libraries used by the code include `PCF8574`, `VL53L0X`, and `Adafruit_NeoPixel`
- compile/build verification is still pending in this repository, so the first build should be treated as a bring-up check rather than a guaranteed known-good baseline

## Arduino IDE Build / Upload

If the project is not already under your Arduino sketch folder, place or copy the whole folder here so the sketch keeps its Arduino layout:

- `Documents/Arduino/Mouse_esp32s3`

Then build and upload like this:

1. Open [Mouse_esp32s3.ino](Mouse_esp32s3.ino) in Arduino IDE.
2. Select your ESP32-S3 board target in `Tools`.
3. Set the ESP32-S3 board options to match this project:
   - `USB CDC On Boot`: `Enabled`
   - `Partition Scheme`: `Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)` or the closest current ESP32-core label that gives about `1.9MB code`, `1.9MB OTA`, and `128KB/190KB SPIFFS`
   - `Arduino Runs On`: `Core 1`
4. Select the correct serial `COM` port in `Tools -> Port`.
5. Click `Verify` first.
6. Click `Upload`.

Bring-up notes:

- if the board does not enumerate correctly, re-check that `USB CDC On Boot` is enabled
- if the partition scheme is wrong, OTA or SPIFFS features may fail later even if the sketch compiles
- after upload, open Serial Monitor or telnet and use `help` / `status` for first validation
- Wi-Fi OTA and browser upload on port `82` are convenient later, but first bring-up should still start from a normal USB upload

## Robot Modes

Defined in [RobotTypes.h](RobotTypes.h):
- `ROBOT_MODE_IDLE`
- `ROBOT_MODE_MANUAL_TEST`
- `ROBOT_MODE_EXPLORE`
- `ROBOT_MODE_SPEED_RUN`
- `ROBOT_MODE_FAULT`

## Motion Primitives

Implemented in [MotionController.cpp](MotionController.cpp):
- `moveOneCell()`
- `moveCells(n)`
- `moveForwardShort()`
- `moveBackwardShort()`
- `snapCenter()`
- `turnLeft90()`
- `turnRight90()`
- `turn180()`
- `stop()`

Primitive execution currently includes:
- primitive timeout
- simple stall detection
- front-wall stop support based directly on front distance threshold
- side-wall centering correction using a wall PID error term
- battery state is still reported, but it no longer aborts or blocks motion
- motor control now separates coast stop from active brake: motion-completion and transition paths use brake, while general stop/idle paths can still coast
- on the current driver setup, `coastStop()` explicitly sets `IN1=LOW`, `IN2=LOW`, `PWM=0`, while `brakeStop()` routes through the zero-duty motor drive path that holds the wheel in position
- motor commands inside the PWM dead zone now coast at zero instead of forcing a minimum forward/reverse duty
- motion start/end debug hooks in the runtime for tracing primitive flow during tuning
- a short post-motion hard-stop hold after sensing/ACK so the robot pauses only when the system is otherwise ready for the next action
- `speedrun 2` bypasses the normal per-primitive stop-hold path so one-way shortest-path execution can flow directly from one segment to the next
- `snapCenter()` runs as one primitive: reverse short, hard stop, hold briefly, then forward short
- `snapCenter()` does not change the logical maze pose; it is a physical re-centering primitive only
- task-context millisecond waits now use `vTaskDelay(...)` instead of Arduino `delay(...)` so other FreeRTOS tasks can keep running cleanly during those pauses

## Serial Commands

Available from the main sketch:
- `help`
- `status`
- `explore`
- `explore n`
- `speedrun`
- `speedrun 1`
- `speedrun 2`
- `speedrun 3`
- `speedrun 4`
- `idle`
- `stop`
- `brake`
- `restart`
- `move`
- `move n`
- `back`
- `testsnap`
- `left`
- `right`
- `uturn`
- `maze`
- `led cycle|rotate|off|red|green|blue|yellow|cyan|magenta|white`
- `test`
- `test off`
- `test loop status|battery|sensors|sensorsraw|encoders|maze|off`
- `resetpose x y h`
- `setgoal x y w h`
- `clearmaze`
- `test battery`
- `test sensors`
- `test sensorsraw`
- `test motorl [tps]`
- `test motorr [tps]`
- `test motor both [tps]`
- `test encoders`

Manual motion note:
- `testsnap` triggers the combined `snapCenter()` primitive so the back-then-forward recenter motion is executed and reported as one normal motion

Console note:
- periodic debug output pauses briefly while you type on serial or telnet, then resumes automatically
- `restart` closes the TCP debug console first, then reboots the ESP32
- `brake` applies the active motor brake immediately for bench testing and tuning, while `stop` now forces motion stop + hard-stop on both motors before entering idle
- the TCP debug console close path follows the current ESP32 `NetworkClient` API to avoid deprecated-call warnings during build
- the TCP debug console listens on port `2323`
- port `80` now serves a simple control page that shows the robot hostname and offers `Reconnect Telnet` plus `Cycle LED`
- the port `80` control page now also includes a grouped quick-reference for the main CLI commands, so users can see the accepted commands and their purpose before opening telnet
- the port `80` control page now renders the current battery voltage/state directly into the page when it loads, without background polling
- the port `80` telnet reconnect action forcibly disconnects the current TCP debug client before launching a fresh telnet connection to the robot IP and configured debug port
- the port `80` control page now also has an `Open Floodfill` button that jumps straight to the live floodfill viewer on port `81`
- `setgoal x y w h` now updates the active runtime goal rectangle used by `explore` for the current runtime

## Web Debugging

Port `80` is now a lightweight control page, and the floodfill explorer still runs on port `81` as the live maze debug tool.

Expected use:
- verify discovered walls
- verify current pose
- inspect floodfill distance field
- confirm planner action sequence
- the floodfill page now pushes state over WebSocket, so the browser no longer polls the maze state over HTTP while the mouse is running
- the floodfill page now loads with `AutoACK(sim)` turned off by default, so browser control waits for the real robot ACK unless you explicitly enable simulation
- with `AutoACK(sim)` off, the port `81` `Step` and `Run` buttons now hand control to the real app runtime: `Step` starts `explore 1`, `Run` starts real explore without clearing the current maze, and `Pause` / `Reset` map to the runtime pause/reset flow
- the floodfill web page no longer exposes a web `Set Start` editor; start is now treated as a configured/runtime app concern instead of a browser-side control
- the floodfill web page now prints planner status in a wrapped status bar so long state text does not stretch or resize the whole page while the mouse is running
- when `AutoACK(sim)` is enabled, the floodfill web `Step` button advances one full simulated cell move, consuming any required turn actions first, while `Run` continues auto-running in simulated mode

## Wireless Upload

Two wireless update paths are available:
- Arduino OTA using the configured hostname
- browser upload page on port `82`

Current auth behavior:
- Arduino OTA has no password configured
- browser upload on port `82` has no password gate

Browser upload flow:
1. compile the firmware so you have the `.bin`
2. open `http://<mouse-ip>:82/`
3. choose the firmware `.bin`
4. upload; the page now transfers the firmware over chunked HTTP requests with retry handling in the browser
5. wait for the board to reboot

During browser upload:
- the robot enters the same quiet/safe mode used for Arduino OTA
- motion, planner, telemetry, and sensor polling are paused so the upload has priority
- after a successful upload, the browser receives a success JSON reply and the ESP32 reboots shortly after
- while an update is active, the firmware no longer forces a Wi-Fi reconnect cycle; this avoids killing the upload socket during brief link wobble
- if an upload aborts mid-transfer, the firmware now avoids stacking extra reconnect requests while the STA is already reconnecting
- LED status during upload:
  - blue: upload/OTA in progress
  - off: upload finished successfully
  - red: upload/OTA error or abort
- Robot-state LED status:
  - green: explore active before first reached target
  - blue: explore active after the first reached target
  - white: shortest path known / final reached-target state
  - red: fault mode
  - off: idle / non-explore normal state

Use this file for the browser uploader:
- `Mouse_esp32s3.ino.bin`

Do not use these for the browser uploader:
- `Mouse_esp32s3.ino.merged.bin`
- bootloader or partition binaries

Related config in [Config.h](Config.h):
- `AppConfig::Wifi::ENABLE_UPLOAD_WEB`
- `AppConfig::Wifi::UPLOAD_WEB_PORT`

## Hardware / Tuning Notes

Current values are placeholders and will need on-robot tuning in [Config.h](Config.h):
- battery ADC pin and calibration values
- `cellDistanceMm`
- `turnTicks90`
- `turnTicks180`
- move and turn TPS
- wall threshold
- wall-centering PID gains (`CENTER_PID_KP/KI/KD`)
- wall-centering PID integral/output limits
- post-motion hard-stop hold before the next action
- snapcenter reverse-stop hold before forward restart
- front stop distance
- `mmPerTick`

Current front-sensor behavior:
- V2 front fusion uses `min(front-left, front-right)` when both are valid, or the valid side when only one side is valid
- V2 `frontValid` is true when either front sensor is valid, or when both front sensors are far/open
- both front sensors are now treated independently; front-right is not gated by front-left validity
- far/open and error values follow `AppConfig::Tof::DIST_FAR_MM` and `AppConfig::Tof::DIST_ERROR_MM`

Battery divider note:
- the current configured divider is `56k / 18k` into `GPIO 3`
- battery voltage now uses the measured ADC node voltage plus the configured divider ratio as the primary pack-voltage estimate
- on ESP32, battery ADC debug now uses the calibrated millivolt reading path instead of a simple `raw / 4095 * 3.3` approximation
- the current tuned divider constant uses an effective top resistor value of `56k` with `18k` bottom to better match the measured pack voltage on this board

## Known Limitations

- No compile/build verification was done in this environment.
- Speed-run mode is still conservative and discrete-cell based.
- Motion completion depends on encoder/tick calibration.
- Sensor fusion is basic; it does not yet do robust front alignment or advanced filtering.
- Floodfill uses live observed walls, but there is not yet a separate persistent optimized path module.

## Suggested Bring-Up Order

1. Verify battery readings against a multimeter.
2. Verify left and right motor direction.
3. Verify encoder polarity and ticks.
4. Verify TOF wall detection for left/front/right.
5. Tune one-cell movement.
6. Tune left/right 90-degree turns.
7. Tune `uturn` and `back`.
8. Use `maze` to confirm the robot's known walls match reality.
9. Run `explore` in a simple maze and verify wall registration and planner decisions.
10. Only after stable exploration, tune `speedrun`.

````

### `RELEASE_0.3.0.md`

````md
# Release 0.3.0

`0.3.0` is the first packaged solver milestone for this project.

This release groups together:
- reliable floodfill exploration
- shortest-path discovery with home/goal looping
- saved maze-memory restore from SPIFFS
- `speedrun 1` as the first direct known-path run mode

It is not the final race release yet, but it is the first version that feels like a complete mouse solver package instead of only bring-up pieces.

## Included In 0.3.0

### Core motion and sensing
- dual DC motor control with encoder-based speed PID
- one-cell move, reverse, 90-degree turns, 180-degree turn, and `snapCenter()`
- left / front / right TOF wall sensing
- wall-centering correction during motion
- battery telemetry path

### Explore solver
- floodfill planner with live maze memory
- hardware explore flow with planner/executor split
- wall registration after motion completion
- dead-end recovery with real `turn180`
- goal/home target toggling during explore
- shortest-path-known detection from stable repeated round trips

### Maze persistence
- shortest-path maze memory saved to SPIFFS
- maze wall memory restored at boot
- pose and runtime goal kept out of SPIFFS to avoid stale swap confusion

### Web tools
- port `80` control page
- port `81` live floodfill maze page
- WebSocket maze sync for floodfill web
- live run timing with `HG` and `GH` leg history kept in RAM across runs

### Speedrun 1
- direct shortest-path action execution from known maze
- no floodfill ACK handshake
- no wall-map updates during the run
- no snapcenter recovery during the run
- optional quiet serial output while running
- goal-to-home flip support
- smooth primitive-to-primitive transitions without the normal explore stop/hold behavior

## Operational Meaning

`0.3.0` should be treated as:
- good for solver testing
- good for maze learning and shortest-path confirmation
- good for first direct shortest-path runs
- still not the final aggressive speed profile

## Not In 0.3.0 Yet

- speedrun phase 2 tuning/profile
- single-flow snake move compression
- advanced multi-cell path compression
- final race-tuned motion profiles
- complete board-level compile verification matrix

## Next Target

The next milestone after `0.3.0` is:

- `speedrun phase 2`

Current direction for that work:
- build phase 2 on top of the tuned `speedrun 1` baseline
- add a faster single-flow snake move style
- keep `speedrun 1` as the reliable fallback path

## Release Summary

`0.3.0` is the version where this project becomes a real mouse solver package:
- explore can learn
- shortest path can be recognized
- maze memory can be restored
- `speedrun 1` can execute the known path

````

### `RobotTypes.h`

````cpp
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
  MOTION_MOVE_MULTI_CELL,
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
  uint8_t speedRunPhase = 1;
  uint32_t faultCount = 0;
  String lastFault;
};
````

### `scripts/build.ps1`

````powershell
param(
  [string]$Fqbn = "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashMode=qio120,PartitionScheme=min_spiffs,LoopCore=0,EventsCore=0",
  [string]$Sketch = "",
  [int]$Jobs = 0,
  [string]$LogFile = "",
  [switch]$VerboseOutput = $true,
  [string]$CliPath = ""
)

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

function Resolve-ArduinoCliPath {
  param([string]$ExplicitCliPath)

  if (-not [string]::IsNullOrWhiteSpace($ExplicitCliPath)) {
    $candidate = (Resolve-Path $ExplicitCliPath).Path
    if (!(Test-Path $candidate)) {
      throw "arduino-cli not found at: $candidate"
    }
    return $candidate
  }

  $default = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
  if (Test-Path $default) {
    return $default
  }

  $fromPath = Get-Command arduino-cli.exe -ErrorAction SilentlyContinue
  if ($fromPath) {
    return $fromPath.Source
  }

  throw "arduino-cli not found. Set -CliPath or install Arduino IDE / arduino-cli."
}

$cli = Resolve-ArduinoCliPath -ExplicitCliPath $CliPath

if ([string]::IsNullOrWhiteSpace($Sketch)) {
  $Sketch = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
} else {
  $Sketch = (Resolve-Path $Sketch).Path
}

if ($Jobs -le 0) {
  $Jobs = [Math]::Max(1, [Environment]::ProcessorCount * 4)
}

Write-Host "[build] Compiling $Sketch"
if ($VerboseOutput) {
  Write-Host "[build] FQBN: $Fqbn"
  Write-Host "[build] Jobs: $Jobs (4x logical processors)"
}

$cliArgs = @("compile", "--fqbn", $Fqbn, "--jobs", $Jobs)
if ($VerboseOutput) {
  $cliArgs += "--verbose"
}
$cliArgs += $Sketch

Write-Host ("[build] cmd: {0} {1}" -f $cli, ($cliArgs -join " "))

$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "SilentlyContinue"
try {
  if ($LogFile.Length -gt 0) {
    & $cli @cliArgs 2>&1 | ForEach-Object {
      $text = [string]$_
      Write-Host $text
      Add-Content -Path $LogFile -Value $text -Encoding utf8
    }
  } else {
    & $cli @cliArgs 2>&1 | ForEach-Object { Write-Host ([string]$_) }
  }
}
finally {
  $ErrorActionPreference = $previousErrorActionPreference
}

$buildExit = $LASTEXITCODE
if ($buildExit -ne 0) {
  throw "Compile failed (exit $buildExit). See log: $LogFile"
}

Write-Host "[build] Compile success"
````

### `scripts/build-and-upload.ps1`

````powershell
param(
  [string]$Fqbn = "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashMode=qio120,PartitionScheme=min_spiffs,LoopCore=0,EventsCore=0",
  [string]$Sketch = "",
  [string]$Port = "COM3",
  [int]$UploadSpeed = 921600,
  [int]$Jobs = 0,
  [switch]$RetryWithLowerBaud = $true,
  [switch]$VerboseOutput = $true,
  [switch]$ForceFreePort = $true,
  [string]$CliPath = ""
)

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

if ([string]::IsNullOrWhiteSpace($Sketch)) {
  $Sketch = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
} else {
  $Sketch = (Resolve-Path $Sketch).Path
}

Write-Host "[pipeline] Compile -> Upload"
$logsDir = Join-Path $Sketch "logs"
if (!(Test-Path $logsDir)) { New-Item -ItemType Directory -Path $logsDir | Out-Null }
$ts = Get-Date -Format "yyyyMMdd_HHmmss"
$logFile = Join-Path $logsDir "build_upload_$ts.log"
$latest = Join-Path $logsDir "build_upload_latest.log"
"[$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')] pipeline start" | Out-File -FilePath $logFile -Encoding utf8

try {
  & "$PSScriptRoot\build.ps1" -Fqbn $Fqbn -Sketch $Sketch -Jobs $Jobs -LogFile $logFile -VerboseOutput:$VerboseOutput -CliPath $CliPath

  try {
    & "$PSScriptRoot\upload.ps1" -Fqbn $Fqbn -Sketch $Sketch -Port $Port -UploadSpeed $UploadSpeed -LogFile $logFile -VerboseOutput:$VerboseOutput -ForceFreePort:$ForceFreePort -CliPath $CliPath
  }
  catch {
    if ($RetryWithLowerBaud) {
      Write-Warning "[pipeline] Upload failed at $UploadSpeed. Retrying with 460800..."
      & "$PSScriptRoot\upload.ps1" -Fqbn $Fqbn -Sketch $Sketch -Port $Port -UploadSpeed 460800 -LogFile $logFile -VerboseOutput:$VerboseOutput -ForceFreePort:$ForceFreePort -CliPath $CliPath
    }
    else {
      throw
    }
  }

  Write-Host "[pipeline] Done: compile + upload successful"
}
finally {
  "[$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')] pipeline end (last exit: $LASTEXITCODE)" | Out-File -FilePath $logFile -Encoding utf8 -Append
  if (Test-Path $logFile) { Copy-Item -Force $logFile $latest }
  Write-Host "[pipeline] Log: $logFile"
}
````

### `scripts/export-notebooklm.ps1`

````powershell
param(
    [string]$ProjectRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$OutputFile = "NOTEBOOKLM_PROJECT_EXPORT.md"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$resolvedRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
$outputPath = Join-Path $resolvedRoot $OutputFile

$excludedDirectoryNames = @(
    ".git",
    "logs"
)

$includedFileNames = @(
    ".gitignore"
)

$includedExtensions = @(
    ".ino",
    ".h",
    ".hpp",
    ".c",
    ".cpp",
    ".md",
    ".txt",
    ".ps1",
    ".json",
    ".yml",
    ".yaml",
    ".ini",
    ".cfg"
)

function Test-IsExcludedPath {
    param(
        [string]$FullName
    )

    $relativePath = Get-RelativePath -BasePath $resolvedRoot -TargetPath $FullName
    $segments = $relativePath -split "[/\\]"

    foreach ($segment in $segments) {
        if ($excludedDirectoryNames -contains $segment) {
            return $true
        }
    }

    return $false
}

function Get-RelativePath {
    param(
        [string]$BasePath,
        [string]$TargetPath
    )

    $baseUri = [System.Uri]((Resolve-Path -LiteralPath $BasePath).Path.TrimEnd('\') + '\')
    $targetUri = [System.Uri](Resolve-Path -LiteralPath $TargetPath).Path
    return [System.Uri]::UnescapeDataString($baseUri.MakeRelativeUri($targetUri).ToString()).Replace("\", "/")
}

function Test-IsIncludedFile {
    param(
        [System.IO.FileInfo]$File
    )

    if ($File.FullName -eq $outputPath) {
        return $false
    }

    if (Test-IsExcludedPath -FullName $File.FullName) {
        return $false
    }

    if ($includedFileNames -contains $File.Name) {
        return $true
    }

    return $includedExtensions -contains $File.Extension.ToLowerInvariant()
}

function Get-CodeFenceLanguage {
    param(
        [string]$RelativePath
    )

    $extension = [System.IO.Path]::GetExtension($RelativePath).ToLowerInvariant()

    switch ($extension) {
        ".ino" { return "cpp" }
        ".c" { return "c" }
        ".cpp" { return "cpp" }
        ".h" { return "cpp" }
        ".hpp" { return "cpp" }
        ".md" { return "md" }
        ".txt" { return "text" }
        ".ps1" { return "powershell" }
        ".json" { return "json" }
        ".yml" { return "yaml" }
        ".yaml" { return "yaml" }
        ".ini" { return "ini" }
        ".cfg" { return "ini" }
        default { return "" }
    }
}

function Get-TreeLines {
    param(
        [string[]]$RelativePaths
    )

    $lines = New-Object System.Collections.Generic.List[string]
    $seenDirectories = New-Object System.Collections.Generic.HashSet[string]

    foreach ($relativePath in $RelativePaths) {
        $parts = $relativePath -split "[/\\]"

        if ($parts.Length -gt 1) {
            for ($i = 0; $i -lt ($parts.Length - 1); $i++) {
                $directoryKey = ($parts[0..$i] -join "/")
                if ($seenDirectories.Add($directoryKey)) {
                    $lines.Add(("  " * $i) + $parts[$i] + "/")
                }
            }
        }

        $fileIndent = if ($parts.Length -gt 1) { "  " * ($parts.Length - 1) } else { "" }
        $lines.Add($fileIndent + $parts[-1])
    }

    return $lines
}

$files = Get-ChildItem -LiteralPath $resolvedRoot -Recurse -File |
    Where-Object { Test-IsIncludedFile -File $_ } |
    Sort-Object FullName

$relativePaths = @(
    foreach ($file in $files) {
        Get-RelativePath -BasePath $resolvedRoot -TargetPath $file.FullName
    }
)

$treeLines = Get-TreeLines -RelativePaths $relativePaths
$generatedAt = Get-Date -Format "yyyy-MM-dd HH:mm:ss zzz"

$builder = New-Object System.Text.StringBuilder

[void]$builder.AppendLine("# NotebookLM Project Export")
[void]$builder.AppendLine()
[void]$builder.AppendLine('Generated from the Arduino project at "' + $resolvedRoot + '" on ' + $generatedAt + '.')
[void]$builder.AppendLine()
[void]$builder.AppendLine("This bundle is intended to be imported as a single source into NotebookLM. It includes the project tree and the full contents of the selected text/code files.")
[void]$builder.AppendLine()
[void]$builder.AppendLine("Included file count: $($files.Count)")
[void]$builder.AppendLine()
[void]$builder.AppendLine('Excluded directories: `.git`, `logs`')
[void]$builder.AppendLine()
[void]$builder.AppendLine("## Project Tree")
[void]$builder.AppendLine()
[void]$builder.AppendLine('```text')
foreach ($line in $treeLines) {
    [void]$builder.AppendLine($line)
}
[void]$builder.AppendLine('```')
[void]$builder.AppendLine()
[void]$builder.AppendLine("## Full File Contents")
[void]$builder.AppendLine()

foreach ($file in $files) {
    $relativePath = Get-RelativePath -BasePath $resolvedRoot -TargetPath $file.FullName
    $language = Get-CodeFenceLanguage -RelativePath $relativePath
    $content = Get-Content -LiteralPath $file.FullName -Raw

    [void]$builder.AppendLine('### `' + $relativePath + '`')
    [void]$builder.AppendLine()
    [void]$builder.AppendLine('````' + $language)
    [void]$builder.Append($content)
    if (-not $content.EndsWith("`n") -and -not $content.EndsWith("`r")) {
        [void]$builder.AppendLine()
    }
    [void]$builder.AppendLine('````')
    [void]$builder.AppendLine()
}

[System.IO.File]::WriteAllText($outputPath, $builder.ToString(), [System.Text.UTF8Encoding]::new($false))
Write-Output "Created $outputPath"
````

### `scripts/upload.ps1`

````powershell
param(
  [string]$Fqbn = "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashMode=qio120,PartitionScheme=min_spiffs,LoopCore=0,EventsCore=0",
  [string]$Sketch = "",
  [string]$Port = "COM3",
  [int]$UploadSpeed = 921600,
  [string]$LogFile = "",
  [switch]$VerboseOutput = $true,
  [switch]$ForceFreePort = $true,
  [string]$CliPath = ""
)

if ($args.Count -gt 0) {
  throw "Unknown arguments for upload.ps1: $($args -join ' '). Use: -Port, -UploadSpeed, -Fqbn, -Sketch, -VerboseOutput, -ForceFreePort, -LogFile, -CliPath"
}

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

function Resolve-ArduinoCliPath {
  param([string]$ExplicitCliPath)

  if (-not [string]::IsNullOrWhiteSpace($ExplicitCliPath)) {
    $candidate = (Resolve-Path $ExplicitCliPath).Path
    if (!(Test-Path $candidate)) {
      throw "arduino-cli not found at: $candidate"
    }
    return $candidate
  }

  $default = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
  if (Test-Path $default) {
    return $default
  }

  $fromPath = Get-Command arduino-cli.exe -ErrorAction SilentlyContinue
  if ($fromPath) {
    return $fromPath.Source
  }

  throw "arduino-cli not found. Set -CliPath or install Arduino IDE / arduino-cli."
}

$cli = Resolve-ArduinoCliPath -ExplicitCliPath $CliPath

if ([string]::IsNullOrWhiteSpace($Sketch)) {
  $Sketch = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
} else {
  $Sketch = (Resolve-Path $Sketch).Path
}

function Invoke-KillSerialTools {
  param([string]$ComPort)

  $killed = New-Object System.Collections.Generic.List[string]
  $targets = @(
    "arduino-cli",
    "serial-monitor",
    "putty",
    "kitty",
    "ttermpro",
    "coolterm",
    "python"
  )

  foreach ($name in $targets) {
    $procs = Get-Process -Name $name -ErrorAction SilentlyContinue
    foreach ($p in $procs) {
      try {
        Stop-Process -Id $p.Id -Force -ErrorAction Stop
        $killed.Add("$name#$($p.Id)") | Out-Null
      } catch {
      }
    }
  }

  if ($killed.Count -gt 0) {
    Write-Warning "[upload] Closed processes blocking ${ComPort}: $($killed -join ', ')"
  } else {
    Write-Warning "[upload] No known serial tool process found to close for $ComPort"
  }
}

Write-Host "[upload] Uploading to $Port"
if ($VerboseOutput) {
  Write-Host "[upload] FQBN: $Fqbn"
  Write-Host "[upload] Speed: $UploadSpeed"
}

$cliArgs = @(
  "upload",
  "-p", $Port,
  "--fqbn", $Fqbn,
  "--upload-property", "upload.speed=$UploadSpeed"
)
if ($VerboseOutput) {
  $cliArgs += "--verbose"
}
$cliArgs += $Sketch

Write-Host ("[upload] cmd: {0} {1}" -f $cli, ($cliArgs -join " "))

function Invoke-UploadAttempt {
  $previousErrorActionPreference = $ErrorActionPreference
  $ErrorActionPreference = "SilentlyContinue"
  $lines = New-Object System.Collections.Generic.List[string]
  try {
    if ($LogFile.Length -gt 0) {
      & $cli @cliArgs 2>&1 | ForEach-Object {
        $text = [string]$_
        $lines.Add($text) | Out-Null
        Write-Host $text
        Add-Content -Path $LogFile -Value $text -Encoding utf8
      }
    } else {
      & $cli @cliArgs 2>&1 | ForEach-Object {
        $text = [string]$_
        $lines.Add($text) | Out-Null
        Write-Host $text
      }
    }
  }
  finally {
    $ErrorActionPreference = $previousErrorActionPreference
  }

  return @{
    ExitCode = $LASTEXITCODE
    Output = (($lines.ToArray()) -join [Environment]::NewLine)
  }
}

$result = Invoke-UploadAttempt
$uploadExit = $result.ExitCode
$uploadText = $result.Output

$looksLikeCliHelp = $uploadText -match "(?s)Arduino Command Line Interface \(arduino-cli\).*Usage:"
$looksLikeEsptoolSuccess = $uploadText -match "Hard resetting via RTS pin|Hash of data verified|Wrote [0-9]+ bytes"
$looksLikeUploadFailure = $uploadText -match "Failed uploading|A fatal error occurred"

if ($uploadExit -ne 0 -and $ForceFreePort) {
  $isPortBusy = $uploadText -match "Could not open .*port is busy|Access is denied|PermissionError\(13"
  if ($isPortBusy) {
    Write-Warning "[upload] Detected busy $Port. Attempting to free serial port and retry once..."
    Invoke-KillSerialTools -ComPort $Port
    Start-Sleep -Milliseconds 1200
    $result = Invoke-UploadAttempt
    $uploadExit = $result.ExitCode
    $uploadText = $result.Output
    $looksLikeCliHelp = $uploadText -match "(?s)Arduino Command Line Interface \(arduino-cli\).*Usage:"
    $looksLikeEsptoolSuccess = $uploadText -match "Hard resetting via RTS pin|Hash of data verified|Wrote [0-9]+ bytes"
    $looksLikeUploadFailure = $uploadText -match "Failed uploading|A fatal error occurred"
  }
}

if ($uploadExit -ne 0 -or $looksLikeUploadFailure -or $looksLikeCliHelp -or -not $looksLikeEsptoolSuccess) {
  throw "Upload failed or not verified (exit $uploadExit). No esptool success markers detected. Check COM port access and see log: $LogFile"
}

Write-Host "[upload] Upload success"
````

### `SECURITY_FLASHING.md`

````md
# SECURITY_FLASHING.md

## What we found (from scan)

- No flash-encryption or eFuse burn logic exists in this project source (`.ino/.cpp/.h`).
- No local build files in this repo (`sdkconfig`, `build_opt.h`, `platformio.ini`) were found that enable secure boot / flash encryption.
- Your `esp32s3.txt` shows:
  - `SPI_BOOT_CRYPT_CNT = 0b111` (flash encryption enabled)
  - `SECURE_BOOT_EN = False`

## Root-cause conclusion

From project scan alone, there is no evidence that this repository burned eFuses directly.

Most likely trigger happened outside this repo, such as:
- prior provisioning/security script on this board,
- a different project/tooling flow (ESP-IDF security flow),
- board delivered pre-configured with encryption enabled.

Because security fuses are one-way, once `SPI_BOOT_CRYPT_CNT` is enabled, normal plain Arduino images will not boot.

## Safe Arduino template (development mode)

Use this profile for normal development boards (non-secure):

- Board: ESP32-S3 Dev Module
- USB CDC On Boot: Enabled
- Flash Size: 4MB (or your real module size)
- Partition Scheme: Minimal SPIFFS with OTA (project default style)
- Flash Mode: `DIO`
- Flash Frequency: `80MHz`
- Upload Speed: `115200` (recovery-safe) or `460800` (normal)
- Erase Flash: `Only Sketch` (avoid full erase by default)

## Safe flashing SOP (recommended)

1. Before using a new board, run:
   - `espefuse --port COMx summary`
2. Confirm security state:
   - `SPI_BOOT_CRYPT_CNT` not enabled for plain-Arduino workflow.
3. Avoid:
   - `erase_flash --force`
   - `write_flash --force --encrypt`
   - any `espefuse burn_*` command unless intentional.
4. Before risky operations, backup full flash:
   - `read_flash 0x0 0x400000 backup.bin`
5. Keep one separate board for security experiments.

## Pre-flight check commands

```powershell
# 1) Check security fuses
C:\Users\donot\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.1.0\espefuse.py --port COM4 summary

# 2) Validate generated image on host
C:\Users\donot\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.1.0\esptool.exe --chip esp32s3 --port COM4 image-info C:\path\to\firmware.bin
```
````

### `SKILL.md`

````md
# SKILL.md

## Purpose

This project skill describes how to work effectively on the `Mouse_esp32s3` micromouse codebase.

Use it when you need to:
- bring up hardware safely
- tune motion and sensing
- extend floodfill exploration
- debug planner / executor interaction
- prepare the code for a second-run speed mode

## Mental Model

Think of the project as 6 layers:
1. Arduino entrypoint wrapper
2. app runtime/orchestration
3. low-level hardware drivers
4. shared robot state
5. primitive motion control
6. maze observation and floodfill planning

Changes should usually respect those boundaries.

Documentation rule:
- when a code change affects behavior, commands, tuning, setup, or hardware assumptions, update the related `.md` files in the same change
- when a code change changes behavior or tuning, bump the documented project version and commit the versioned change together

## Core Files

- `Mouse_esp32s3.ino`: keep thin; Arduino-facing entrypoints and task wrappers only
- `AppRuntime.h`: bridge between Arduino wrapper and app module
- `AppRuntime.cpp`: top-level runtime logic, serial commands, tasks, planner coordination
- `Config.h`: hardware pins, calibration values, and runtime tuning defaults
- `Battery.*`: battery health and thresholds
- `DcMotor.*`: motor/encoder speed loop
- `MotionController.*`: physical primitive execution
- `MultiVL53L0X.*`: sensor reads and wall interpretation
- `FloodFillExplorer.*`: maze model, floodfill, live web view
- `RobotTypes.h`: shared enums/state contracts

## Common Workflows

### 1. Hardware bring-up
- Open [Mouse_esp32s3.ino](Mouse_esp32s3.ino) from the Arduino sketch folder, not as an arbitrary loose file.
- Use the intended ESP32-S3 Arduino settings during bring-up:
  - `USB CDC On Boot`: `Enabled`
  - `Partition Scheme`: the minimal SPIFFS/OTA layout used by this project
  - `Arduino Runs On`: `Core 1`
- Select the correct `COM` port before upload and prefer a normal USB upload first before relying on OTA.
- Start in `status`, `test battery`, `test sensors`, `test encoders`.
- Verify motor direction with `test motorl` and `test motorr` before trying `move`.
- Use `maze` to inspect the robot's known wall map after sensor and motion tests.
- Use `test loop maze` when you want a live ASCII map stream during exploration tests.
- If Arduino OTA is flaky, use the browser upload page on port `82` as a simpler fallback.
- During OTA/web upload, expect the onboard LED to turn solid blue while receiving, turn off on success, and turn red on error/abort.
- When `ENABLE_WEB_LOG` is off, `dbg.print/println` are effectively Serial-only and should not add load to the web log path.
- OTA and the browser upload page are currently configured without a password gate.
- Only enable `explore` after one-cell moves and 90-degree turns are reliable.
- Do first-pass tuning in `Config.h`, not scattered through source files.

### 2. Motion tuning
- Tune `mmPerTick` first.
- Then tune `cellDistanceMm`.
- Then tune `turnTicks90`.
- Then tune `turnTicks180`.
- Only after distance/turn correctness, tune centering gain and stop distance.

### 3. Sensor debugging
- Inspect raw distances in serial output.
- Confirm `left/front/right` validity flags make sense.
- If planner acts strangely, first confirm sensor-relative walls before changing floodfill logic.

### 4. Planner debugging
- Check current pose in serial output.
- Check the web explorer on port `81`.
- Verify the explorer's known walls match physical surroundings.
- Verify motion completion is acknowledging actions exactly once.
- During `explore`, expect the ASCII maze to print after map updates unless that behavior is disabled in `Config.h`.

## Design Rules

- Battery state is telemetry-only in the current codebase; do not assume `critical` will stop motion unless that behavior is intentionally reintroduced.
- Planner should issue one action at a time.
- Motion executor should be the only layer deciding primitive success/failure.
- Pose should only advance when the primitive really completes.
- Prefer conservative behavior over speed when adding new capabilities.
- Keep tunables in `Config.h` unless there is a strong reason not to.
- Keep `Mouse_esp32s3.ino` minimal.

## Good Next Enhancements

- add persistent storage for tuning values
- improve front-wall alignment
- add compressed action sequences for second run
- add richer web telemetry for live tuning
- add config validation / startup sanity checks

## Anti-Patterns To Avoid

- Writing directly to motors from multiple tasks
- Updating pose before primitive completion
- Mixing simulator truth walls with hardware observations in the same run
- Adding aggressive speed-run logic before reliable one-cell behavior exists
- Hiding calibration assumptions in random source files without updating docs
- Growing the `.ino` back into the main implementation file

````

### `TODO.md`

````md
# TODO.md

## Immediate

- Tune the dedicated `speedrun 2` profile on top of the stable `speedrun 1` baseline.
- Prototype the first single-flow snake/corridor behavior for `speedrun 2` while keeping `speedrun 1` as the fallback path.

- Compile the project against the intended ESP32-S3 board and fix any board/core compile issues.
- Verify all new source files are included by the Arduino build.
- Verify browser upload on port 82 works reliably with the intended `.bin` workflow.
- If browser upload still reports `network error`, capture serial logs around `[WEB OTA] Start`, `Success`, and reboot scheduling to separate HTTP-response issues from flash-write issues.
- If browser upload still aborts, compare whether the last serial line is `[WEB OTA] Received ... bytes`, `[WEB OTA] Aborted`, or a Wi-Fi reconnect event to isolate transport vs. flash finalization.
- Current evidence points to a mid-transfer network drop after partial progress, not an OTA password or `.bin` selection issue.
- Update docs/examples if the Arduino builder needs any filename/layout adjustments.
- Confirm the battery ADC pin is correct for the real board.
- Confirm the current Wi-Fi credentials strategy is acceptable for development.

## ESP32-S3 Platform Audit

- Document current in-use ESP32-S3 features in README (`gptimer`, dual-core tasks, LEDC PWM, GPIO encoder ISR, I2C TOF, Wi-Fi/OTA/Web, mDNS, SPIFFS, ADC).
- Add a security-state check step (`espefuse summary`) to bring-up docs before any erase/update workflow.
- Define a secure update path if secure boot / flash encryption is enabled (signed/encrypted image flow).
- Evaluate moving encoder counting from GPIO ISR to `PCNT` for high-speed robustness.
- Add a crash diagnostics checklist (boot reason + panic/exception capture + fault context print).
- Define per-task watchdog policy (TWDT coverage and expected recovery behavior).
- Evaluate NVS/Preferences for persistent tuning/calibration values (keep SPIFFS for maze map only).

## Hardware Bring-Up

- Measure battery raw ADC at low and full pack voltage.
- Tune battery warning and critical thresholds.
- Verify left motor direction matches commanded forward motion.
- Verify right motor direction matches commanded forward motion.
- Verify encoder sign and tick counts for both motors.
- Tune `mmPerTick` from measured travel distance.
- Verify the configured `56k / 18k` battery divider reading matches the expected ADC range.

## Motion Tuning

- Tune `cellDistanceMm` so one cell move is repeatable.
- Tune `turnTicks90` for accurate 90-degree turns.
- Tune `turnTicks180` for reliable U-turns without overshoot.
- Tune `moveSpeedTps` and `turnSpeedTps`.
- Tune `stallTimeoutMs` so valid slow motion does not false-fail.
- Tune `frontStopMm` for safe wall approach.
- Tune `centeringGain` so corridor tracking is stable and does not oscillate.
- Mark final tuned values clearly in `Config.h`.

## Sensor Tuning

- Verify sensor layout detection for the actual hardware version.
- Confirm left/front/right sensor mapping is correct.
- Confirm whether the temporary S3 workaround should stay on `ignore`, switch to `mirror S0->S3`, or be removed after hardware fixes.
- Tune wall threshold from measured distances.
- Add filtering or confidence logic for inconsistent TOF readings.
- Add better front-wall alignment behavior near cell boundaries.

## Planner / Maze

- Confirm the start cell stays unknown at boot and is only observed when explore/speed-run begins.
- Verify pose updates after each primitive in a real maze.
- Validate discovered wall writes into the expected maze cells.
- Validate the ASCII `maze` dump against the physical maze after each test.
- Test dead-end and backtracking behavior on hardware.
- Persist or export known maze data for debugging.
- Add path compression for the second run.
- Add a clear distinction between exploration completion and optimized second-run execution.

## Observability

- Extend web UI to show battery, mode, motion status, and live wall observation validity.
- Add more compact serial status output for tuning sessions.
- Add explicit fault codes / categories instead of only free-form strings.
- Log primitive start/end timing for motion tuning.
- Consider a dedicated calibration printout grouped around values from `Config.h`.

## Nice To Have

- Add EEPROM / NVS storage for tuned values.
- Move Wi-Fi credentials out of the sketch/config for safer sharing.
- Add an optional simulator mode switch in the main app.
- Add automated test scaffolding for maze-state updates and planner decisions.
````

### `UPLOAD_INSTRUCTIONS.md`

````md
# Upload Instructions

## Overview

This project provides two upload methods:

1. **Arduino IDE Upload (USB)** - Recommended for initial flash
2. **Browser Upload (Port 82)** - Recommended for later OTA updates

## Port 82 - Browser Upload

### Features

- Chunked HTTP upload with retry handling
- Streaming firmware transfer
- Adaptive chunk-size fallback
- Fixed retry backoff/pacing
- LED status feedback:
  - **Blue** = Upload/OTA in progress
  - **Off** = Upload finished successfully
  - **Red** = Upload/OTA error or abort

### Usage

1. Open browser
2. Navigate to: `http://<ESP32_IP>:82/`
3. Select firmware file
4. Upload

### Upload Flow

1. Open the upload page on port `82`
2. The page starts the upload session with `/upload/start`
3. Firmware data is streamed in chunks through `/upload/chunk`
4. The transfer is finalized with `/upload/finish`

### Troubleshooting

If browser upload reports network error:
- Check Serial Monitor for `[WEB OTA] Start`, `Success`, and reboot scheduling
- Capture serial logs to isolate HTTP-response issues from flash-write issues
- If upload still aborts:
  - Check if the last serial line is `[WEB OTA] Received ... bytes`, `[WEB OTA] Aborted`, or a Wi-Fi reconnect event
  - Use that to separate transport problems from flash finalization problems

## Port 80 - Control Page

Port `80` is the runtime control/status page, not the main firmware upload surface.

Current use:

- shows battery/runtime status
- links to the floodfill page on port `81`
- links to the browser upload page on port `82`
- provides command guidance and telnet reconnect helpers

For firmware upload, use the dedicated browser uploader on port `82`.

## Build Configuration

### Arduino IDE Build / Upload

**File Location:** `Mouse_esp32s3.ino`

**Build Settings:**
- `USB CDC On Boot`: Enabled
- `Partition Scheme`: Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
- `Arduino Runs On`: Core 1
- `COM Port`: Select correct port before upload

**Upload Steps:**
1. Open `Mouse_esp32s3.ino` in Arduino IDE
2. Select ESP32-S3 board in Tools
3. Set Partition Scheme to match project requirements
4. Select correct COM port
5. Click Verify first
6. Click Upload

## Firmware Files

Use this compiled firmware file for the browser uploader:

- `Mouse_esp32s3.ino.bin`

Do not use these for the browser uploader:

- `Mouse_esp32s3.ino.merged.bin`
- bootloader binaries
- partition table binaries

## External Libraries

- `PCF8574` - I2C I/O expander
- `VL53L0X` - TOF distance sensors
- `Adafruit_NeoPixel` - LED control

## Additional Resources

- See `README.md` for full documentation
- See `SKILL.md` for development workflow guidelines
- See `TODO.md` for known issues and next steps
````

### `WiFiOtaWebSerial.cpp`

````cpp
#include "WiFiOtaWebSerial.h"
#include "Config.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <esp_err.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_wifi.h>

// Wrapper to keep WebServer out of header
class WiFiOtaWebSerial::WebServerWrapper {
public:
  WebServerWrapper(uint16_t port) : server(port) {}
  WebServer server;
};

static const char* kIndexHtml PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>%HOSTNAME% Control</title>
  <style>
    :root { color-scheme: light; }
    body { font-family: "Segoe UI", Arial, sans-serif; margin: 0; background: #eef3ea; color: #102017; }
    .wrap { max-width: 720px; margin: 0 auto; padding: 24px; }
    .card { background: #fbfdf9; border-radius: 18px; box-shadow: 0 10px 28px rgba(0,0,0,0.08); padding: 22px; }
    h1 { margin: 0 0 8px; font-size: 1.5rem; }
    .sub { color: #4f5f55; margin-bottom: 20px; }
    .meta { display: grid; grid-template-columns: 110px 1fr; gap: 8px 14px; margin-bottom: 20px; }
    .meta b { color: #355040; }
    .row { display:flex; gap:12px; align-items:center; flex-wrap:wrap; }
    h2 { margin: 24px 0 10px; font-size: 1.05rem; color: #1d3b2b; }
    .hint { margin: 0 0 12px; color: #4f5f55; line-height: 1.5; }
    .guide { display: grid; gap: 12px; margin-top: 8px; }
    .group { background: #f4f8f1; border: 1px solid #d9e6d7; border-radius: 14px; padding: 14px; }
    .group h3 { margin: 0 0 8px; font-size: 0.98rem; color: #284735; }
    .group ul { margin: 0; padding-left: 18px; }
    .group li { margin: 6px 0; line-height: 1.4; }
    code { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; background: #e8efe6; padding: 1px 5px; border-radius: 6px; }
    button { padding: 11px 14px; cursor: pointer; border: 0; border-radius: 12px; font-weight: 600; }
    .primary { background: #1d6b45; color: white; }
    .secondary { background: #1d4f6b; color: white; }
    #status { margin-top: 16px; min-height: 1.3em; color: #44584a; }
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>%HOSTNAME%</h1>
    <div class="sub">Simple robot control page on port 80.</div>
    <div class="meta">
      <b>Hostname</b><span>%HOSTNAME%</span>
      <b>IP</b><span id="robotIp">%IP%</span>
      <b>Telnet</b><span id="telnetTarget">%IP%:%TELNET_PORT%</span>
      <b>Battery</b><span id="batteryNow">%BATTERY_TEXT%</span>
    </div>
    <div class="row">
      <button class="primary" onclick="reconnectTelnet()">Reconnect Telnet</button>
      <button class="secondary" onclick="cycleLed()">Cycle LED</button>
      <button class="secondary" onclick="openFloodfill()">Open Floodfill</button>
      <button class="secondary" onclick="openUpload()">Open Upload</button>
    </div> 
    <div id="status">Ready.</div>
    <h2>How To Use</h2>
    <p class="hint">Use <b>Reconnect Telnet</b> to open the live CLI in your telnet app. The commands below are the main ones the robot accepts, grouped by purpose so you can quickly find the right action.</p>
    <div class="guide">
      <div class="group">
        <h3>Run Control</h3>
        <ul>
          <li><code>help</code> - print the command list in the robot console.</li>
          <li><code>status</code> - show mode, pose, battery, motion, and wall state.</li>
          <li><code>explore [n]</code> - explore the maze and learn walls until the shortest path is known, or stop after <code>n</code> forward cell moves.</li>
          <li><code>speedrun [1-4]</code> - start the speed-run mode when ready; <code>speedrun</code> means phase 1, phase 2 is the one-way home-to-goal profile, and phases 3-4 inherit the previous phase until tuned.</li>
          <li><code>idle</code> - switch the robot back to idle mode.</li>
          <li><code>restart</code> - reboot the robot after closing the debug client.</li>
        </ul>
      </div>
      <div class="group">
        <h3>Motion</h3>
        <ul>
          <li><code>move [n]</code> - move forward 1 cell by default, or move exactly <code>n</code> cells in a straight corridor.</li>
          <li><code>back</code> - move backward using the configured short reverse distance.</li>
          <li><code>left</code> - turn left 90 degrees.</li>
          <li><code>right</code> - turn right 90 degrees.</li>
          <li><code>uturn</code> - turn 180 degrees.</li>
          <li><code>testsnap</code> - run the snap-center combo for alignment tuning.</li>
          <li><code>stop</code> - stop with the normal coast-to-idle path.</li>
          <li><code>brake</code> - apply the active brake immediately for bench testing.</li>
        </ul>
      </div>
      <div class="group">
        <h3>Maze And Pose</h3>
        <ul>
          <li><code>maze</code> - print the current maze memory.</li>
          <li><code>clearmaze</code> - clear remembered maze walls only; keep the current pose and remove saved wall memory from SPIFFS.</li>
          <li><code>resetpose x y h</code> - set pose manually for the current runtime; heading is the numeric enum used by the firmware.</li>
          <li><code>setgoal x y w h</code> - set the active goal rectangle origin and size for the current runtime.</li>
        </ul>
      </div>
      <div class="group">
        <h3>LED Control</h3>
        <ul>
          <li><code>led cycle</code> - step through LED states to identify the robot.</li>
          <li><code>led rotate</code> - rotate LED colors.</li>
          <li><code>led off</code>, <code>led red</code>, <code>led green</code>, <code>led blue</code>, <code>led yellow</code>, <code>led cyan</code>, <code>led magenta</code>, <code>led white</code> - force a specific LED state.</li>
        </ul>
      </div>
      <div class="group">
        <h3>Test And Debug</h3>
        <ul>
          <li><code>test</code> - enter test mode.</li>
          <li><code>test off</code> - leave test mode.</li>
          <li><code>test battery</code> - print battery diagnostics.</li>
          <li><code>test sensors</code> - print interpreted wall-sensor readings.</li>
          <li><code>test sensorsraw</code> - print raw TOF sensor readings.</li>
          <li><code>test encoders</code> - print encoder diagnostics.</li>
          <li><code>test motorl</code>, <code>test motorr</code> - spin a single motor for bench checks.</li>
          <li><code>test motor both</code> - flip both motors between +100% and -100% every second for bench testing.</li>
          <li><code>test loop status|battery|sensors|sensorsraw|encoders|maze|off</code> - start or stop periodic debug printing.</li>
        </ul>
      </div>
    </div>
  </div>
</div>
<script>
let busy = false;
function setStatus(s){ document.getElementById('status').textContent = s; }

async function cycleLed(){
  if (busy) return;
  busy = true;
  try {
    const res = await fetch('/led', {
      method: 'POST',
      headers: { 'Content-Type':'application/x-www-form-urlencoded' },
      body: 'cmd=' + encodeURIComponent('cycle'),
      cache: 'no-store'
    });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    setStatus('LED cycle command sent.');
  } catch (e) {
    setStatus('LED command failed: ' + e);
  } finally {
    busy = false;
  }
}

async function reconnectTelnet(){
  if (busy) return;
  busy = true;
  try {
    const res = await fetch('/telnet/reconnect', { method: 'POST', cache: 'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const info = await res.json();
    document.getElementById('robotIp').textContent = info.ip;
    document.getElementById('telnetTarget').textContent = info.ip + ':' + info.port;
    if (info.batteryText) {
      document.getElementById('batteryNow').textContent = info.batteryText;
    }
    setStatus('Reconnecting telnet to ' + info.ip + ':' + info.port + ' ...');
    window.location.href = 'telnet://' + info.ip + ':' + info.port;
  } catch (e) {
    setStatus('Telnet reconnect failed: ' + e);
  } finally {
    busy = false;
  }
}

function openFloodfill(){
  const ip = document.getElementById('robotIp').textContent || location.hostname;
  setStatus('Opening floodfill viewer on ' + ip + ':81 ...');
  window.location.href = 'http://' + ip + ':81/';
}

function openUpload(){
  const ip = document.getElementById('robotIp').textContent || location.hostname;
  setStatus('Opening upload page on ' + ip + ':%UPLOAD_PORT% ...');
  window.location.href = 'http://' + ip + ':%UPLOAD_PORT%/';
}
</script>
</body>
</html>
)HTML";

static const char* kUploadHtml PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 Upload</title>
  <style>
    body { font-family: ui-sans-serif, Segoe UI, Arial, sans-serif; margin: 24px; background: #f6f3ea; color: #1b1b1b; }
    .card { max-width: 560px; padding: 20px; border-radius: 16px; background: #fffdf7; box-shadow: 0 8px 30px rgba(0,0,0,0.08); }
    h1 { margin-top: 0; font-size: 1.4rem; }
    input, button { font: inherit; }
    input[type=file] { display: block; margin: 12px 0; }
    button { padding: 10px 14px; border: 0; border-radius: 10px; background: #165d47; color: white; cursor: pointer; }
    progress { width: 100%; height: 18px; margin-top: 16px; }
    #msg { margin-top: 12px; white-space: pre-wrap; }
    .muted { color: #5d5d5d; font-size: 0.95rem; }
  </style>
</head>
<body>
    <div class="card">
    <h1>Wireless Firmware Upload</h1>
    <div class="muted">Chunked HTTP upload with retry. Upload a compiled firmware binary (`.bin`). The mouse will reboot after a successful update.</div>
    <form id="fwForm">
      <input id="fwFile" name="firmware" type="file" accept=".bin,application/octet-stream" required />
      <button type="submit">Upload Firmware</button>
    </form>
    <progress id="prog" value="0" max="100" hidden></progress>
    <div id="msg"></div>
  </div>
<script>
const form = document.getElementById('fwForm');
const fileInput = document.getElementById('fwFile');
const prog = document.getElementById('prog');
const msg = document.getElementById('msg');
const CHUNK_SIZE = 64 * 1024; 
const MAX_CHUNK_SIZE = 128 * 1024;
const MIN_CHUNK_SIZE = 8192;
const MAX_RETRIES = 5;
const RETRY_BACKOFF_MS = 1000;
const CHUNK_SUCCESS_PAUSE_MS = 1000;
const GROW_CHUNK_AFTER_SUCCESS = 4;

async function postStart(totalSize){
  const res = await fetch('/upload/start?size=' + totalSize, {
    method: 'POST',
    cache: 'no-store'
  });
  const text = await res.text();
  if (!res.ok) throw new Error(text || ('HTTP ' + res.status));
  return JSON.parse(text);
}

async function postChunk(file, offset, chunkSize){
  const end = Math.min(offset + chunkSize, file.size);
  const chunk = file.slice(offset, end);
  const form = new FormData();
  form.append('chunk', chunk, file.name + '.part');
  const res = await fetch('/upload/chunk?offset=' + offset, {
    method: 'POST',
    body: form,
    cache: 'no-store'
  });
  const text = await res.text();
  if (!res.ok) throw new Error(text || ('HTTP ' + res.status));
  return JSON.parse(text);
}

async function postFinish(){
  const res = await fetch('/upload/finish', {
    method: 'POST',
    cache: 'no-store'
  });
  const text = await res.text();
  if (!res.ok) throw new Error(text || ('HTTP ' + res.status));
  return JSON.parse(text);
}

form.addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const file = fileInput.files[0];
  if (!file) {
    msg.textContent = 'Choose a .bin file first.';
    return;
  }

  prog.hidden = false;
  prog.value = 0;
  msg.textContent = 'Starting chunked upload...';

  try {
    let info = await postStart(file.size);
    let offset = Number(info.nextOffset || 0);
    let chunkSize = CHUNK_SIZE;
    let successStreak = 0;

    while (offset < file.size) {
      let done = false;
      let lastErr = '';
      for (let attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
        try {
          info = await postChunk(file, offset, chunkSize);
          offset = Number(info.nextOffset || 0);
          prog.value = Math.round((offset / file.size) * 100);
          msg.textContent = 'Uploading... ' + prog.value + '%';
          successStreak++;
          if (successStreak >= GROW_CHUNK_AFTER_SUCCESS && chunkSize < MAX_CHUNK_SIZE) {
            chunkSize = Math.min(MAX_CHUNK_SIZE, chunkSize * 2);
            successStreak = 0;
          }
          await new Promise((resolve) => setTimeout(resolve, CHUNK_SUCCESS_PAUSE_MS));
          done = true;
          break;
        } catch (err) {
          lastErr = String(err);
          successStreak = 0;
          msg.textContent = 'Chunk retry ' + attempt + '/' + MAX_RETRIES + ' at offset ' + offset + ' failed: ' + lastErr + ' (chunk=' + chunkSize + ')';
          await new Promise((resolve) => setTimeout(resolve, RETRY_BACKOFF_MS));
        }
      }
      if (!done) {
        if (chunkSize > MIN_CHUNK_SIZE) {
          chunkSize = Math.max(MIN_CHUNK_SIZE, Math.floor(chunkSize / 2));
          msg.textContent = 'Network unstable at offset ' + offset + '. Reducing chunk size to ' + chunkSize + ' and retrying...';
          continue;
        }
        throw new Error('Chunk upload failed at offset ' + offset + ': ' + lastErr);
      }
    }

    await postFinish();
    prog.value = 100;
    msg.textContent = 'Upload complete. Device will reboot.';
  } catch (err) {
    msg.textContent = 'Upload failed: ' + err;
  }
});
</script>
</body>
</html>
)HTML";

static constexpr uint32_t kUploadStallTimeoutMs = 10000;
static constexpr bool kUploadPreEraseEnabled = true;
static constexpr size_t kFlashSectorSize = 4096;

WiFiOtaWebSerial::WiFiOtaWebSerial() {}

WiFiOtaWebSerial::~WiFiOtaWebSerial() {
  end();
}

bool WiFiOtaWebSerial::begin(const Config& cfg) {
  cfg_ = cfg;
  if (!cfg_.ssid || cfg_.ssid[0] == '\0') return false;
  otaStarted_ = false;
  webServerStarted_ = false;
  uploadWebServerStarted_ = false;

  // server
  if (web_) { delete web_; web_ = nullptr; }
  if (uploadWeb_) { delete uploadWeb_; uploadWeb_ = nullptr; }
  if (cfg_.enableWeb) {
    web_ = new WebServerWrapper(cfg_.port);
  }
  if (cfg_.enableUploadWeb) {
    uploadWeb_ = new WebServerWrapper(cfg_.uploadPort);
  }

  setupWiFi_();
  setupOta_();
  if (cfg_.enableWeb && web_) {
    setupWeb_();
  }
  if (cfg_.enableUploadWeb && uploadWeb_) {
    setupUploadWeb_();
  }

  started_ = true;

  // start service task pinned to core
  if (wifiTask_) {
    vTaskDelete(wifiTask_);
    wifiTask_ = nullptr;
  }
  if (uploadTask_) {
    vTaskDelete(uploadTask_);
    uploadTask_ = nullptr;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(
    &WiFiOtaWebSerial::wifiTaskThunk_,
    "wifiTask",
    cfg_.wifiTaskStack / sizeof(StackType_t),
    this,
    cfg_.wifiTaskPrio,
    &wifiTask_,
    cfg_.wifiCore
  );

  if (ok != pdPASS) {
    wifiTask_ = nullptr;
    started_ = false;
    return false;
  }

  if (cfg_.enableUploadWeb && uploadWeb_) {
    BaseType_t uploadOk = xTaskCreatePinnedToCore(
      &WiFiOtaWebSerial::uploadTaskThunk_,
      "uploadTask",
      cfg_.uploadTaskStack / sizeof(StackType_t),
      this,
      cfg_.uploadTaskPrio,
      &uploadTask_,
      cfg_.uploadCore
    );
    if (uploadOk != pdPASS) {
      uploadTask_ = nullptr;
      vTaskDelete(wifiTask_);
      wifiTask_ = nullptr;
      started_ = false;
      return false;
    }
  }

  println(String("OTA Hostname: ") + cfg_.hostname);
  return true;
}

void WiFiOtaWebSerial::end() {
  if (wifiTask_) {
    vTaskDelete(wifiTask_);
    wifiTask_ = nullptr;
  }
  if (uploadTask_) {
    vTaskDelete(uploadTask_);
    uploadTask_ = nullptr;
  }
  started_ = false;
  otaStarted_ = false;
  webServerStarted_ = false;
  uploadWebServerStarted_ = false;

  if (web_) {
    delete web_;
    web_ = nullptr;
  }
  if (uploadWeb_) {
    delete uploadWeb_;
    uploadWeb_ = nullptr;
  }
}

void WiFiOtaWebSerial::loopNoService() {
  // Intentionally empty: WiFi/OTA/Web runs in pinned task.
  // Keep your main app logic in Arduino loop().
  
  if (otaStarted_) {
    ArduinoOTA.handle();
  }
}

void WiFiOtaWebSerial::wifiTaskThunk_(void* arg) {
  static_cast<WiFiOtaWebSerial*>(arg)->wifiTaskLoop_();
}

void WiFiOtaWebSerial::uploadTaskThunk_(void* arg) {
  static_cast<WiFiOtaWebSerial*>(arg)->uploadTaskLoop_();
}

void WiFiOtaWebSerial::wifiTaskLoop_() {
  uint32_t lastReconnectMs = 0;
  uint32_t disconnectSinceMs = 0;
  bool urlsAnnounced = false;
  for (;;) {
    if (started_) {
      serviceUpdateLed_();
      const wl_status_t wifiStatus = WiFi.status();
      if (wifiStatus != WL_CONNECTED) {
        urlsAnnounced = false;
        otaStarted_ = false;
        if (otaInProgress || webUploadInProgress_ || rebootPending_) {
          vTaskDelay(pdMS_TO_TICKS(50));
          continue;
        }
        const bool reconnectAllowed =
          reconnectAllowedFn_ ? reconnectAllowedFn_() : true;
        if (!reconnectAllowed) {
          disconnectSinceMs = 0;
          lastReconnectMs = 0;
          vTaskDelay(pdMS_TO_TICKS(250));
          continue;
        }
        if (disconnectSinceMs == 0) disconnectSinceMs = millis();
        const uint32_t now = millis();
        const bool shouldReconnect =
          wifiStatus == WL_DISCONNECTED ||
          wifiStatus == WL_CONNECTION_LOST ||
          wifiStatus == WL_CONNECT_FAILED ||
          wifiStatus == WL_NO_SSID_AVAIL;
        if (shouldReconnect && now - lastReconnectMs >= cfg_.wifiReconnectIntervalMs) {
          lastReconnectMs = now;
          const bool serialOutputAllowed =
            serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
          if (serialOutputAllowed) Serial.println("[WiFi] reconnecting...");
          WiFi.reconnect();
        }
        if ((uint32_t)(now - disconnectSinceMs) >= cfg_.wifiConnectTimeoutMs) {
          disconnectSinceMs = now;
          restartSta_();
        }
        vTaskDelay(pdMS_TO_TICKS(250));
        continue;
      }
      if (disconnectSinceMs != 0) {
        disconnectSinceMs = 0;
        onWiFiConnected_();
      }
      if (!urlsAnnounced) {
        const String currentIp = WiFi.localIP().toString();
        if (cfg_.enableWeb) {
          println(String("HTTP: http://") + currentIp + "/  (or http://" + cfg_.hostname + ".local/ )");
        }
        if (cfg_.enableUploadWeb) {
          println(String("Upload: http://") + currentIp + ":" + String(cfg_.uploadPort) + "/");
        }
        urlsAnnounced = true;
      }
      if (otaInProgress) {
        // OTA priority mode
        if (otaStarted_) ArduinoOTA.handle();
        if (cfg_.enableUploadWeb && uploadWeb_ && uploadWebServerStarted_ && !uploadServerOwnedByUploadTask_()) {
          uploadWeb_->server.handleClient();
        }
        serviceUploadSession_();
        // Optional: skip web server to reduce load
        // if (web_) web_->server.handleClient();
        vTaskDelay(1);
        continue;
      }
      if (webUploadInProgress_) {
        if (cfg_.enableUploadWeb && uploadWeb_ && uploadWebServerStarted_ && !uploadServerOwnedByUploadTask_()) {
          uploadWeb_->server.handleClient();
        }
        serviceUploadSession_();
        vTaskDelay(1);
        continue;
      }
      // Normal mode
      if (otaStarted_) ArduinoOTA.handle();
      if (cfg_.enableWeb && web_ && webServerStarted_) web_->server.handleClient();
      if (cfg_.enableUploadWeb && uploadWeb_ && uploadWebServerStarted_ && !uploadServerOwnedByUploadTask_()) {
        uploadWeb_->server.handleClient();
      }
      serviceUploadSession_();
      if (rebootPending_ && static_cast<int32_t>(millis() - rebootAtMs_) >= 0) {
        println("[WEB OTA] Rebooting now...");
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP.restart();
      }
      vTaskDelay(pdMS_TO_TICKS(cfg_.serviceDelayMs));
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void WiFiOtaWebSerial::uploadTaskLoop_() {
  for (;;) {
    if (!started_ || !cfg_.enableUploadWeb || !uploadWeb_ || !uploadWebServerStarted_) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (!uploadServerOwnedByUploadTask_()) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    uploadWeb_->server.handleClient();
    serviceUploadSession_();
    vTaskDelay(pdMS_TO_TICKS(cfg_.uploadServiceDelayMs));
  }
}

bool WiFiOtaWebSerial::uploadServerOwnedByUploadTask_() const {
  return uploadTask_ != nullptr && (webUploadInProgress_ || chunkUploadActive_);
}

void WiFiOtaWebSerial::print(const String& s, bool mirrorToSerial) {
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (mirrorToSerial && serialOutputAllowed) Serial.print(s);
}

void WiFiOtaWebSerial::println(const String& s, bool mirrorToSerial) {
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (mirrorToSerial && serialOutputAllowed) Serial.println(s);
}

String WiFiOtaWebSerial::ip() const {
  return WiFi.isConnected() ? WiFi.localIP().toString() : String("0.0.0.0");
}

void WiFiOtaWebSerial::setupWiFi_() {
  configureSta_();
  WiFi.begin(cfg_.ssid, cfg_.pass);
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (serialOutputAllowed) Serial.println("[WiFi] start connect (non-blocking)");
}

void WiFiOtaWebSerial::configureSta_() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

void WiFiOtaWebSerial::restartSta_() {
  const bool serialOutputAllowed =
    serialOutputAllowedFn_ ? serialOutputAllowedFn_() : AppConfig::Debug::ENABLE_SERIAL_OUTPUT;
  if (serialOutputAllowed) Serial.println("[WiFi] restarting STA...");
  MDNS.end();
  WiFi.disconnect(false, true);
  vTaskDelay(pdMS_TO_TICKS(100));
  configureSta_();
  WiFi.begin(cfg_.ssid, cfg_.pass);
}

void WiFiOtaWebSerial::onWiFiConnected_() {
  MDNS.end();
  if (MDNS.begin(cfg_.hostname)) {
    if (cfg_.enableWeb) {
      MDNS.addService("http", "tcp", cfg_.port);
    }
    if (cfg_.enableUploadWeb) {
      MDNS.addService("http", "tcp", cfg_.uploadPort);
    }
  }
  startWebServersIfNeeded_();
  setupOta_();
}

void WiFiOtaWebSerial::setupOta_() {
  if (!otaHandlersReady_) {
    ArduinoOTA.setHostname(cfg_.hostname);

    ArduinoOTA
      .onStart([this]() {
        otaInProgress = true;
        ledBlinkOn_ = true;
        setUploadLedActive_();

        println("[OTA] Start");

        // Enter OTA safe mode
        WiFi.setSleep(false);     // improve stability
      })

      .onEnd([this]() {
        println("[OTA] End");
        setUploadLedSuccess_();

        // Exit OTA mode
        otaInProgress = false;

      })

      .onProgress([this](unsigned int progress, unsigned int total) {
        static uint32_t lastMs = 0;
        if (millis() - lastMs > 1000) {
          lastMs = millis();
          int pct = (total > 0) ? (int)(progress * 100 / total) : 0;
          println(String("[OTA] ") + pct + "%");
        }
      })

      .onError([this](ota_error_t error) {
        println(String("[OTA] Error: ") + (int)error);
        setUploadLedError_();

        otaInProgress = false;
      });

    otaHandlersReady_ = true;
  }

  if (WiFi.status() == WL_CONNECTED && !otaStarted_) {
    ArduinoOTA.begin();
    otaStarted_ = true;
  }
}

void WiFiOtaWebSerial::setupWeb_() {
  auto& srv = web_->server;

  srv.on("/", HTTP_GET, [this]() {
    String html = FPSTR(kIndexHtml);
    html.replace("%HOSTNAME%", String(cfg_.hostname ? cfg_.hostname : "esp32"));
    html.replace("%IP%", ip());
    html.replace("%TELNET_PORT%", String(cfg_.debugTcpPort));
    html.replace("%UPLOAD_PORT%", String(cfg_.uploadPort));
    html.replace("%BATTERY_TEXT%", healthJsonProvider_ ? extractBatteryText_(healthJsonProvider_()) : String("unavailable"));
    web_->server.send(200, "text/html; charset=utf-8", html);
  });

  srv.on("/led", HTTP_POST, [this]() {
    String cmd = web_->server.arg("cmd");
    cmd.toLowerCase();

    if (!ledCommandHandler_) {
      web_->server.send(503, "text/plain", "LED unavailable");
      return;
    }

    String response;
    if (!ledCommandHandler_(cmd, response)) {
      web_->server.send(400, "text/plain", "Bad cmd");
      return;
    }

    if (response.length() > 0) {
      println(response);
    }
    web_->server.send(200, "text/plain", "OK");
  });

  srv.on("/telnet/reconnect", HTTP_POST, [this]() {
    if (!telnetReconnectHandler_) {
      web_->server.send(503, "application/json", "{\"ok\":false,\"error\":\"telnet unavailable\"}");
      return;
    }

    const bool ok = telnetReconnectHandler_();
    const String host = cfg_.hostname ? String(cfg_.hostname) : String("esp32");
    const String extra = healthJsonProvider_ ? (String(",") + healthJsonProvider_()) : String("");
    String body = String("{\"ok\":") + (ok ? "true" : "false") +
                  ",\"ip\":\"" + ip() + "\"" +
                  ",\"hostname\":\"" + host + "\"" +
                  ",\"port\":" + String(cfg_.debugTcpPort) +
                  extra + "}";
    web_->server.send(ok ? 200 : 500, "application/json", body);
  });

  srv.onNotFound([this]() {
    web_->server.send(404, "text/plain", "Not found");
  });

}

String WiFiOtaWebSerial::extractBatteryText_(const String& json) const {
  const String key = "\"batteryText\":\"";
  const int start = json.indexOf(key);
  if (start < 0) return "unavailable";
  const int valueStart = start + key.length();
  const int valueEnd = json.indexOf('"', valueStart);
  if (valueEnd < 0) return "unavailable";
  return json.substring(valueStart, valueEnd);
}

void WiFiOtaWebSerial::setupUploadWeb_() {
  auto& srv = uploadWeb_->server;

  srv.on("/", HTTP_GET, [this]() {
    String html = FPSTR(kUploadHtml);
    uploadWeb_->server.send(200, "text/html; charset=utf-8", html);
  });

  srv.on("/upload/start", HTTP_POST, [this]() {
    uploadWeb_->server.client().setNoDelay(true);
    const size_t totalSize = (size_t)uploadWeb_->server.arg("size").toInt();
    if (totalSize == 0) {
      uploadWeb_->server.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid size\"}");
      return;
    }

    if (chunkUploadActive_) {
      Update.abort();
    }

    webUploadInProgress_ = true;
    chunkUploadActive_ = false;
    chunkExpectedOffset_ = 0;
    chunkTotalSize_ = totalSize;
    chunkRequestOffset_ = 0;
    chunkRequestBytes_ = 0;
    chunkRequestOk_ = false;
    chunkRequestSkip_ = false;
    chunkRequestError_ = "";
    uploadLastActivityMs_ = millis();
    rebootPending_ = false;
    rebootAtMs_ = 0;
    ledBlinkOn_ = true;
    setUploadLedActive_();
    println(String("[WEB OTA] Chunked start total=") + totalSize);

    if (kUploadPreEraseEnabled) {
      const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
      if (next == nullptr) {
        setUploadLedError_();
        webUploadInProgress_ = false;
        chunkUploadActive_ = false;
        uploadWeb_->server.send(500, "application/json", "{\"ok\":false,\"error\":\"no ota partition\"}");
        return;
      }
      const size_t eraseSize =
        ((totalSize + kFlashSectorSize - 1) / kFlashSectorSize) * kFlashSectorSize;
      println(String("[WEB OTA] Pre-erase start bytes=") + eraseSize);
      const esp_err_t eraseErr = esp_partition_erase_range(next, 0, eraseSize);
      if (eraseErr != ESP_OK) {
        setUploadLedError_();
        webUploadInProgress_ = false;
        chunkUploadActive_ = false;
        uploadWeb_->server.send(
          500,
          "application/json",
          String("{\"ok\":false,\"error\":\"pre-erase failed:") + esp_err_to_name(eraseErr) + "\"}"
        );
        return;
      }
      println("[WEB OTA] Pre-erase done");
    }

    if (!Update.begin(totalSize)) {
      Update.printError(Serial);
      setUploadLedError_();
      webUploadInProgress_ = false;
      uploadWeb_->server.send(500, "application/json", "{\"ok\":false,\"error\":\"update begin failed\"}");
      return;
    }

    chunkUploadActive_ = true;
    uploadWeb_->server.send(200, "application/json", "{\"ok\":true,\"nextOffset\":0}");
  });

  srv.on("/upload/chunk", HTTP_POST,
    [this]() {
      uploadWeb_->server.client().setNoDelay(true);
      if (!chunkUploadActive_) {
        uploadWeb_->server.send(409, "application/json", "{\"ok\":false,\"error\":\"no active upload session\"}");
        return;
      }
      if (!chunkRequestOk_) {
        const String err = chunkRequestError_.length() > 0 ? chunkRequestError_ : String("chunk failed");
        uploadWeb_->server.send(409, "application/json",
                                String("{\"ok\":false,\"error\":\"") + err + "\"}");
        return;
      }
      uploadWeb_->server.send(200, "application/json",
                              String("{\"ok\":true,\"nextOffset\":") + chunkExpectedOffset_ + "}");
    },
    [this]() {
      HTTPUpload& upload = uploadWeb_->server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        uploadLastActivityMs_ = millis();
        chunkRequestOffset_ = (size_t)uploadWeb_->server.arg("offset").toInt();
        chunkRequestBytes_ = 0;
        chunkRequestOk_ = false;
        chunkRequestHadWrite_ = false;
        chunkRequestSkip_ = false;
        chunkRequestError_ = "";
        if (!chunkUploadActive_) {
          chunkRequestError_ = "no active upload session";
          return;
        }
        if (chunkRequestOffset_ < chunkExpectedOffset_) {
          chunkRequestOk_ = true;
          chunkRequestSkip_ = true;
          return;
        }
        if (chunkRequestOffset_ != chunkExpectedOffset_) {
          chunkRequestError_ = String("offset mismatch expected ") + chunkExpectedOffset_ +
                               " got " + chunkRequestOffset_;
          return;
        }
        chunkRequestOk_ = true;
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (!chunkRequestOk_) return;
        if (chunkRequestSkip_) return;
        if (upload.buf == nullptr || upload.currentSize == 0) return;
        uploadLastActivityMs_ = millis();
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
          chunkRequestOk_ = false;
          chunkRequestError_ = "flash write failed";
          chunkUploadActive_ = false;
          webUploadInProgress_ = false;
          uploadLastActivityMs_ = 0;
          Update.abort();
          setUploadLedError_();
          println("[WEB OTA] Chunked session aborted (flash write failed)");
          return;
        }
        delay(0);
        chunkRequestHadWrite_ = true;
        chunkRequestBytes_ += upload.currentSize;
      } else if (upload.status == UPLOAD_FILE_END) {
        if (!chunkRequestOk_) return;
        uploadLastActivityMs_ = millis();
        if (chunkRequestSkip_) return;
        chunkExpectedOffset_ += chunkRequestBytes_;
        if (chunkExpectedOffset_ > chunkTotalSize_) {
          chunkRequestOk_ = false;
          chunkRequestError_ = "size overflow";
          setUploadLedError_();
          return;
        }
        if ((chunkExpectedOffset_ % (128 * 1024)) < chunkRequestBytes_) {
          println(String("[WEB OTA] Chunked received ") + chunkExpectedOffset_ + " bytes");
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        chunkRequestOk_ = false;
        chunkRequestError_ = "chunk aborted, restart upload";
        if (chunkUploadActive_) {
          chunkUploadActive_ = false;
          webUploadInProgress_ = false;
          uploadLastActivityMs_ = 0;
          Update.abort();
          setUploadLedError_();
          println(String("[WEB OTA] Chunked session aborted (chunk aborted, wrote=") +
                  (chunkRequestHadWrite_ ? "1" : "0") + ")");
        }
        uploadLastActivityMs_ = 0;
      }
    }
  );

  srv.on("/upload/finish", HTTP_POST, [this]() {
    if (!chunkUploadActive_) {
      uploadWeb_->server.send(409, "application/json", "{\"ok\":false,\"error\":\"no active upload session\"}");
      return;
    }
    if (chunkExpectedOffset_ != chunkTotalSize_) {
      uploadWeb_->server.send(409, "application/json",
                              String("{\"ok\":false,\"error\":\"incomplete upload at ") +
                              chunkExpectedOffset_ + "/" + chunkTotalSize_ + "\"}");
      return;
    }
    const bool ok = Update.end(true);
    chunkUploadActive_ = false;
    webUploadInProgress_ = false;
    uploadLastActivityMs_ = 0;
    if (ok) {
      println(String("[WEB OTA] Chunked success: ") + chunkTotalSize_ + " bytes");
      setUploadLedSuccess_();
      rebootPending_ = true;
      rebootAtMs_ = millis() + 1500;
      uploadWeb_->server.send(200, "application/json", "{\"ok\":true}");
    } else {
      Update.printError(Serial);
      setUploadLedError_();
      println("[WEB OTA] Chunked finish failed");
      uploadWeb_->server.send(500, "application/json", "{\"ok\":false,\"error\":\"update end failed\"}");
    }
  });

  srv.on("/update", HTTP_POST,
    [this]() {
      const bool ok = !Update.hasError();
      if (ok) {
        uploadWeb_->server.send(200, "text/plain", "OK");
        println("[WEB OTA] Update complete, reboot scheduled...");
        setUploadLedSuccess_();
        rebootPending_ = true;
        rebootAtMs_ = millis() + 1500;
      } else {
        uploadWeb_->server.send(500, "text/plain", "FAIL");
        println("[WEB OTA] Update failed");
      }
      webUploadInProgress_ = false;
    },
    [this]() {
      HTTPUpload& upload = uploadWeb_->server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        webUploadInProgress_ = true;
        rebootPending_ = false;
        rebootAtMs_ = 0;
        ledBlinkOn_ = true;
        setUploadLedActive_();
        println(String("[WEB OTA] Start: ") + upload.filename);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
          setUploadLedError_();
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (upload.buf == nullptr || upload.currentSize == 0) {
          return;
        }
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
          setUploadLedError_();
        } else {
          delay(0);
          if ((upload.totalSize % (128 * 1024)) < upload.currentSize) {
          println(String("[WEB OTA] Received ") + upload.totalSize + " bytes");
          }
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          println(String("[WEB OTA] Success: ") + upload.totalSize + " bytes");
          setUploadLedSuccess_();
        } else {
          Update.printError(Serial);
          setUploadLedError_();
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        webUploadInProgress_ = false;
        setUploadLedError_();
        println("[WEB OTA] Aborted");
      }
    }
  );

  srv.onNotFound([this]() {
    uploadWeb_->server.send(404, "text/plain", "Not found");
  });

}

void WiFiOtaWebSerial::startWebServersIfNeeded_() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (cfg_.enableWeb && web_ && !webServerStarted_) {
    web_->server.begin();
    webServerStarted_ = true;
  }
  if (cfg_.enableUploadWeb && uploadWeb_ && !uploadWebServerStarted_) {
    uploadWeb_->server.begin();
    uploadWebServerStarted_ = true;
  }
}

void WiFiOtaWebSerial::serviceUploadSession_() {
  if ((chunkUploadActive_ || webUploadInProgress_) &&
      uploadLastActivityMs_ != 0 &&
      (uint32_t)(millis() - uploadLastActivityMs_) >= kUploadStallTimeoutMs) {
    Update.abort();
    chunkUploadActive_ = false;
    webUploadInProgress_ = false;
    uploadLastActivityMs_ = 0;
    setUploadLedError_();
    println("[WEB OTA] Upload stalled");
  }
}

void WiFiOtaWebSerial::setLedState_(const String& cmd) {
  if (!ledCommandHandler_) return;
  String response;
  ledCommandHandler_(cmd, response);
}

void WiFiOtaWebSerial::setUploadLedActive_() {
  setLedState_("blue");
}

void WiFiOtaWebSerial::setUploadLedSuccess_() {
  setLedState_("off");
}

void WiFiOtaWebSerial::setUploadLedError_() {
  setLedState_("red");
}

void WiFiOtaWebSerial::serviceUpdateLed_() {
  if (!(otaInProgress || webUploadInProgress_)) return;
  if (!ledBlinkOn_) {
    ledBlinkOn_ = true;
    setUploadLedActive_();
  }
}



````

### `WiFiOtaWebSerial.h`

````cpp
#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

class WiFiOtaWebSerial {
public:
  using LedCommandHandler = bool (*)(const String& cmd, String& response);
  using TelnetReconnectHandler = bool (*)();
  using HealthJsonProvider = String (*)();
  using SerialOutputAllowedFn = bool (*)();
  using ReconnectAllowedFn = bool (*)();

  struct Config {
    const char* ssid       = nullptr;
    const char* pass       = nullptr;

    const char* hostname   = "esp32";
    uint16_t    port       = 80;
    bool        enableWeb  = true;
    uint16_t    debugTcpPort = 2323;
    uint16_t    uploadPort = 82;
    bool        enableUploadWeb = true;

    // Service task
    uint32_t    serviceDelayMs = 10;
    uint32_t    wifiTaskStack  = 6144;   // bytes
    UBaseType_t wifiTaskPrio   = 1;
    BaseType_t  wifiCore       = 0;
    // Dedicated upload service task (used during active HTTP chunk upload).
    uint32_t    uploadTaskStack = 6144;  // bytes
    UBaseType_t uploadTaskPrio  = 2;
    BaseType_t  uploadCore      = 0;
    uint32_t    uploadServiceDelayMs = 1;
    uint32_t    wifiConnectTimeoutMs = 5000;
    uint32_t    wifiReconnectIntervalMs = 1000;
  };

  WiFiOtaWebSerial();
  ~WiFiOtaWebSerial();

  bool begin(const Config& cfg);
  void end();

  // In this design, WiFi/OTA/Web runs in pinned task. Call this in loop if you want (no-op).
  void loopNoService();

  // Logging API
  void print(const String& s, bool mirrorToSerial = true);
  void println(const String& s, bool mirrorToSerial = true);
  void setLedCommandHandler(LedCommandHandler handler) { ledCommandHandler_ = handler; }
  void setTelnetReconnectHandler(TelnetReconnectHandler handler) { telnetReconnectHandler_ = handler; }
  void setHealthJsonProvider(HealthJsonProvider provider) { healthJsonProvider_ = provider; }
  void setSerialOutputAllowedHandler(SerialOutputAllowedFn fn) { serialOutputAllowedFn_ = fn; }
  void setReconnectAllowedHandler(ReconnectAllowedFn fn) { reconnectAllowedFn_ = fn; }

  // Info
  String ip() const;
  bool isOtaInProgress() const { return otaInProgress; }
  bool isUpdateInProgress() const { return otaInProgress || webUploadInProgress_ || rebootPending_; }

private:
  class WebServerWrapper;   // forward declaration
  WebServerWrapper* web_ = nullptr;
  WebServerWrapper* uploadWeb_ = nullptr;

  Config cfg_;
  bool started_ = false;

  // FreeRTOS task
  TaskHandle_t wifiTask_ = nullptr;
  TaskHandle_t uploadTask_ = nullptr;
  
  volatile bool otaInProgress = false;
  volatile bool webUploadInProgress_ = false;
  volatile bool rebootPending_ = false;
  uint32_t rebootAtMs_ = 0;
  LedCommandHandler ledCommandHandler_ = nullptr;
  TelnetReconnectHandler telnetReconnectHandler_ = nullptr;
  HealthJsonProvider healthJsonProvider_ = nullptr;
  SerialOutputAllowedFn serialOutputAllowedFn_ = nullptr;
  ReconnectAllowedFn reconnectAllowedFn_ = nullptr;

private:
  static void wifiTaskThunk_(void* arg);
  static void uploadTaskThunk_(void* arg);
  void wifiTaskLoop_();
  void uploadTaskLoop_();
  bool uploadServerOwnedByUploadTask_() const;

  void setupWiFi_();
  void configureSta_();
  void restartSta_();
  void onWiFiConnected_();
  void setupOta_();
  void setupWeb_();
  void setupUploadWeb_();
  void startWebServersIfNeeded_();
  void serviceUploadSession_();
  void setLedState_(const String& cmd);
  void setUploadLedActive_();
  void setUploadLedSuccess_();
  void setUploadLedError_();
  void serviceUpdateLed_();
  String extractBatteryText_(const String& json) const;

  bool ledBlinkOn_ = false;
  bool chunkUploadActive_ = false;
  size_t chunkExpectedOffset_ = 0;
  size_t chunkTotalSize_ = 0;
  size_t chunkRequestOffset_ = 0;
  size_t chunkRequestBytes_ = 0;
  bool chunkRequestOk_ = false;
  bool chunkRequestHadWrite_ = false;
  bool chunkRequestSkip_ = false;
  String chunkRequestError_;
  uint32_t uploadLastActivityMs_ = 0;
  bool otaHandlersReady_ = false;
  bool otaStarted_ = false;
  bool webServerStarted_ = false;
  bool uploadWebServerStarted_ = false;
};
````

