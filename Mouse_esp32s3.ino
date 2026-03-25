#include <Arduino.h>
#include <WiFi.h>
#include <driver/gpio.h>

#include "Battery.h"
#include "DcMotor.h"
#include "FloodFillExplorer.h"
#include "MotionController.h"
#include "MultiVL53L0X.h"
#include "RobotTypes.h"
#include "WiFiOtaWebSerial.h"

static const uint8_t BATTERY_ADC_PIN = 3;
static const uint8_t START_X = 0;
static const uint8_t START_Y = 15;
static const FloodFillExplorer::Dir START_H = FloodFillExplorer::NORTH;

static const uint8_t GOAL_X0 = 7;
static const uint8_t GOAL_Y0 = 7;
static const uint8_t GOAL_W = 2;
static const uint8_t GOAL_H = 2;

WiFiOtaWebSerial dbg;
FloodFillExplorer explorer;
Battery mouseBattery;
MotionController motionController;
RobotState robotState;

DcMotor leftMotor;
DcMotor rightMotor;

static bool motorsOk = false;
static bool tofOk = false;
static bool batteryOk = false;
static bool wifiOk = false;
static uint32_t lastStatusMs = 0;

static DcMotor::Pins R_PINS = {
  .in1 = 10,
  .in2 = 11,
  .pwm = 7,
  .encA = 12,
  .encB = 13,
  .invertDir = false,
  .invertEnc = false
};

static DcMotor::Pins L_PINS = {
  .in1 = 5,
  .in2 = 6,
  .pwm = 4,
  .encA = 1,
  .encB = 2,
  .invertDir = true,
  .invertEnc = true
};

#define I2C_SDA 8
#define I2C_SCL 9

static const uint8_t xshutPins[5] = {0, 1, 2, 3, 4};
static const uint8_t tofAddr[5] = {0x30, 0x31, 0x32, 0x33, 0x34};

MultiVL53L0X tofArray(
  0x20, 5,
  xshutPins, tofAddr,
  20,
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
static void plannerTask(void* arg);
static void userTask(void* arg);
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
static void printStartupSummary();
static void motor_debug_s();
static void tof_debug_s();
static void robot_debug_s();

static void logToDbg(const String& s) {
  dbg.println(s);
}

void i2cRecover(int sda, int scl) {
  pinMode(sda, OUTPUT);
  pinMode(scl, OUTPUT);
  delay(10);

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
  Wire.begin(sda, scl, 400000);
  vTaskDelay(pdMS_TO_TICKS(10));
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

static void enterIdleMode(const String& reason) {
  robotState.mode = ROBOT_MODE_IDLE;
  robotState.motionStatus = MOTION_IDLE;
  robotState.activePrimitive = MOTION_NONE;
  explorer.setRunning(false);
  motionController.stop();
  dbg.println("[MODE] IDLE: " + reason);
}

static void enterFaultMode(const String& reason) {
  robotState.mode = ROBOT_MODE_FAULT;
  robotState.lastFault = reason;
  robotState.faultCount++;
  motionController.abort(reason);
  explorer.setRunning(false);
  dbg.println("[FAULT] " + reason);
}

static void beginExplore(bool clearMaze) {
  robotState.goalReached = false;
  robotState.mode = ROBOT_MODE_EXPLORE;
  robotState.lastFault = "";
  explorer.setHardwareMode(true);
  explorer.setStart(robotState.pose.cellX, robotState.pose.cellY, headingDir());
  explorer.setGoalRect(GOAL_X0, GOAL_Y0, GOAL_W, GOAL_H);
  if (clearMaze) explorer.clearKnownMaze();
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  applyWallsToExplorer();
  explorer.setRunning(true);
  dbg.println("[MODE] EXPLORE");
}

static void beginSpeedRun() {
  robotState.mode = ROBOT_MODE_SPEED_RUN;
  robotState.goalReached = false;
  robotState.lastFault = "";
  explorer.setHardwareMode(true);
  explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
  applyWallsToExplorer();
  explorer.setRunning(true);
  dbg.println("[MODE] SPEED RUN");
}

static bool executePlannerAction(FloodFillExplorer::Action act) {
  bool ok = false;
  switch (act) {
    case FloodFillExplorer::ACT_MOVE_F:
      ok = motionController.moveOneCell();
      break;
    case FloodFillExplorer::ACT_TURN_L:
      ok = motionController.turnLeft90();
      break;
    case FloodFillExplorer::ACT_TURN_R:
      ok = motionController.turnRight90();
      break;
    default:
      return true;
  }

  if (!ok) {
    enterFaultMode("failed to start primitive");
    return false;
  }

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
  }

  robotState.pose.forwardProgressMm = 0.0f;
  robotState.pose.turnProgressDeg = 0.0f;
}

static void handleMotionCompletion() {
  MotionStatus status = motionController.status();
  MotionPrimitiveType primitive = motionController.lastFinishedPrimitive();

  if (status == MOTION_COMPLETED) {
    advancePoseForFinishedPrimitive(primitive);
    bool reachedGoal = explorer.ackPendingActionExternal(true,
      robotState.pose.cellX,
      robotState.pose.cellY,
      headingDir());
    applyWallsToExplorer();

    if (reachedGoal) {
      robotState.goalReached = true;
      robotState.speedRunReady = true;
      dbg.println("[GOAL] Goal reached");
      if (robotState.mode == ROBOT_MODE_SPEED_RUN) {
        enterIdleMode("speed run finished");
      }
    }
  } else if (status == MOTION_FAILED || status == MOTION_ABORTED) {
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

  for (uint8_t i = 0; i < 5; ++i) {
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
  robotState.readyForMotion = motorsOk && tofOk && batteryOk && mouseBattery.state() != Battery::BATTERY_CRITICAL;
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

void setup() {
  Serial.begin(921600);
  vTaskDelay(pdMS_TO_TICKS(200));
  i2cRecover(I2C_SDA, I2C_SCL);
  setPose(START_X, START_Y, START_H);

  WiFiOtaWebSerial::Config wifiCfg;
  wifiCfg.ssid = "PhucWifi";
  wifiCfg.pass = "000000001";
  wifiCfg.hostname = "PhucC_Esp32s3_mice";
  wifiCfg.otaPassword = "";
  wifiCfg.wifiCore = 0;
  wifiCfg.wifiTaskStack = 10 * 1024;
  wifiCfg.wifiTaskPrio = 3;
  wifiCfg.serviceDelayMs = 5;
  wifiOk = dbg.begin(wifiCfg);
  dbg.println(wifiOk ? "Boot OK" : "Boot with WiFi failed");

  motorsOk = leftMotor.begin(L_PINS, 0, 20000, 10) && rightMotor.begin(R_PINS, 1, 20000, 10);
  if (motorsOk) {
    leftMotor.setSpeedPID(0.004f, 0.0080f, 0.00005f, 0.80f, 0.50f, 25.0f, 4.0f);
    rightMotor.setSpeedPID(0.004f, 0.0080f, 0.00005f, 0.80f, 0.50f, 25.0f, 4.0f);
  }

  tofArray.setWallThreshold(150);
  tofOk = tofArray.begin();

  mouseBattery.begin(BATTERY_ADC_PIN);
  mouseBattery.setCalibration(2800, 7.20f, 3350, 8.40f);
  mouseBattery.setThresholds(7.10f, 6.90f);
  mouseBattery.update();
  batteryOk = mouseBattery.isReady();

  motionController.begin(leftMotor, rightMotor, tofArray, &mouseBattery);
  MotionController::Config motionCfg;
  motionCfg.cellDistanceMm = 180.0f;
  motionCfg.turnTicks90 = 300;
  motionController.setConfig(motionCfg);

  explorer.setLog(logToDbg);
  FloodFillExplorer::Config explorerCfg;
  explorerCfg.port = 81;
  explorerCfg.autoRun = false;
  explorerCfg.ackTimeoutMs = 2000;
  explorerCfg.pauseOnAckTimeout = true;
  explorer.begin(explorerCfg);
  explorer.setHardwareMode(true);
  explorer.setGoalRect(GOAL_X0, GOAL_Y0, GOAL_W, GOAL_H);
  explorer.setStart(START_X, START_Y, START_H);
  explorer.clearKnownMaze();
  explorer.syncPose(START_X, START_Y, START_H, true);

  updateRobotState();
  applyWallsToExplorer();
  printStartupSummary();

  xTaskCreatePinnedToCore(motorTask,     "motor",     4096, nullptr, 3, &motorTaskHandle,     0);
  xTaskCreatePinnedToCore(tofTask,       "tof",       4096, nullptr, 2, &tofTaskHandle,       0);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 1, &telemetryTaskHandle, 1);
  xTaskCreatePinnedToCore(explorerTask,  "explorer",  8192, nullptr, 2, &explorerTaskHandle,  1);
  xTaskCreatePinnedToCore(plannerTask,   "planner",   4096, nullptr, 2, &plannerTaskHandle,   1);
  xTaskCreatePinnedToCore(userTask,      "user",      6144, nullptr, 2, &userTaskHandle,      0);

  enterIdleMode("ready");
}

void loop() {
  vTaskDelay(100);
}

static void userTask(void* arg) {
  (void)arg;
  String line;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20);

  for (;;) {
    updateRobotState();
    motionController.update(robotState);

    while (Serial.available() > 0) {
      char ch = (char)Serial.read();
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

static void plannerTask(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(50);

  for (;;) {
    if (robotState.motionStatus == MOTION_COMPLETED ||
        robotState.motionStatus == MOTION_FAILED ||
        robotState.motionStatus == MOTION_ABORTED) {
      handleMotionCompletion();
    }

    if ((robotState.mode == ROBOT_MODE_EXPLORE || robotState.mode == ROBOT_MODE_SPEED_RUN) &&
        !motionController.isBusy()) {
      if (!robotState.readyForMotion) {
        enterFaultMode("robot not ready for motion");
      } else {
        applyWallsToExplorer();
        explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
        FloodFillExplorer::Action act = explorer.requestNextAction();
        if (act == FloodFillExplorer::ACT_NONE) {
          if (robotState.mode == ROBOT_MODE_EXPLORE && explorer.atGoal()) {
            robotState.goalReached = true;
            robotState.speedRunReady = true;
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

static void explorerTask(void* arg) {
  (void)arg;
  for (;;) {
    explorer.loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void tofTask(void* arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5);

  for (;;) {
    tofArray.update();
    vTaskDelayUntil(&lastWake, period);
  }
}

static void motorTask(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);

  for (;;) {
    leftMotor.update();
    rightMotor.update();
    vTaskDelayUntil(&last, period);
  }
}

static void telemetryTask(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(250);

  for (;;) {
    robot_debug_s();
    vTaskDelayUntil(&last, period);
  }
}

static void printStartupSummary() {
  dbg.println(String("[BOOT] WiFi=") + (wifiOk ? "OK" : "FAIL"));
  dbg.println(String("[BOOT] Motors=") + (motorsOk ? "OK" : "FAIL"));
  dbg.println(String("[BOOT] TOF=") + (tofOk ? "OK" : "FAIL"));
  dbg.println(String("[BOOT] Battery=") + (batteryOk ? "OK" : "FAIL"));
  dbg.println("[CMD] help | explore | speedrun | idle | stop | move | left | right | status | resetpose x y h | test battery|sensors|motorl|motorr|encoders");
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
  if (line == "explore") {
    beginExplore(true);
    return;
  }
  if (line == "speedrun") {
    if (!robotState.speedRunReady) {
      dbg.println("[CMD] speed run not ready yet");
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
  if (line == "move") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    executePlannerAction(FloodFillExplorer::ACT_MOVE_F);
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
  if (line.startsWith("resetpose")) {
    int x, y, h;
    if (sscanf(line.c_str(), "resetpose %d %d %d", &x, &y, &h) == 3) {
      x = constrain(x, 0, 15);
      y = constrain(y, 0, 15);
      h &= 3;
      setPose((uint8_t)x, (uint8_t)y, (FloodFillExplorer::Dir)h);
      explorer.syncPose(robotState.pose.cellX, robotState.pose.cellY, headingDir(), true);
      dbg.println("[CMD] pose reset");
    } else {
      dbg.println("[CMD] usage: resetpose x y h");
    }
    return;
  }
  if (line == "test battery") {
    dbg.println("[TEST] battery V=" + String(robotState.batteryVoltage, 2) +
                " pct=" + String(robotState.batteryPercent, 0) +
                " state=" + batteryStateName(robotState.batteryState));
    return;
  }
  if (line == "test sensors") {
    tof_debug_s();
    return;
  }
  if (line == "test motorl") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    leftMotor.setSpeedTPS(220.0f);
    rightMotor.setSpeedTPS(0.0f);
    dbg.println("[TEST] left motor spin");
    return;
  }
  if (line == "test motorr") {
    robotState.mode = ROBOT_MODE_MANUAL_TEST;
    leftMotor.setSpeedTPS(0.0f);
    rightMotor.setSpeedTPS(220.0f);
    dbg.println("[TEST] right motor spin");
    return;
  }
  if (line == "test encoders") {
    motor_debug_s();
    return;
  }

  dbg.println("[CMD] unknown: " + rawLine);
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

static void motor_debug_s() {
  Serial.print("MOTOR | L ticks="); Serial.print(leftMotor.getTicks());
  Serial.print(" tps="); Serial.print(leftMotor.getTicksPerSecond(), 1);
  Serial.print(" | R ticks="); Serial.print(rightMotor.getTicks());
  Serial.print(" tps="); Serial.println(rightMotor.getTicksPerSecond(), 1);
}

static void tof_debug_s() {
  for (uint8_t i = 0; i < tofArray.sensorCount(); i++) {
    Serial.printf("S%u=%u ", i, tofArray.getDistance(i));
  }
  Serial.printf("| wall L/F/R=%d/%d/%d | valid=%d/%d/%d\n",
                robotState.walls.leftWall,
                robotState.walls.frontWall,
                robotState.walls.rightWall,
                robotState.walls.leftValid,
                robotState.walls.frontValid,
                robotState.walls.rightValid);
}

static void robot_debug_s() {
  const uint32_t now = millis();
  if (now - lastStatusMs < 200) return;
  lastStatusMs = now;

  Serial.printf("MODE=%s motion=%s pose=(%u,%u,%u) batt=%.2fV(%s) tps=(%.1f,%.1f) walls=%d/%d/%d dist=%u/%u/%u fault=%s\n",
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
}
