#include <Arduino.h>
#include <WiFi.h>
#include "DcMotor.h"
#include "MultiVL53L0X.h"
#include "WiFiOtaWebSerial.h"
#include "FloodFillExplorer.h"
#include <driver/gpio.h>

// #include "Battery.h"

// Base truth = empty (we will add walls in code)
// Bits per cell: N=1, E=2, S=4, W=8
// You can edit this table manually.
// Tip: outer boundaries can be left 0 because setTruthFromWalls(..., forceBoundaries=true) adds them.

static const uint8_t MAZE_SRC[16][16] = {
  // y=0 (top)
  { 0, 0, 0, 0, 2, 0, 0, 4, 2, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // (x=4,y=1) has EAST wall
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // gap at y=8 -> no wall at x=4
  { 0, 0, 0, 0, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 4, 4, 4, 4, 6, 4, 4, 4, 4, 4, 4, 4, 0, 0}, // y=10 row has SOUTH walls (bit=4) to form a horizontal barrier
  { 0, 0, 0, 0, 0, 0, 0, 0,15, 0,15, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,15, 0, 0}, // y=15 (bottom) start
};

// =====================================================
// Globals
// =====================================================
WiFiOtaWebSerial dbg;

FloodFillExplorer explorer;
// ---------------- Battery pin configs -----------------
// Battery mouseBattery;

// ---------------- Motor pin configs -----------------
DcMotor leftMotor;
DcMotor rightMotor;

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

// ---------------- TOF / I2C -----------------
#define I2C_SDA 8
#define I2C_SCL 9

static const uint8_t xshutPins[5] = {0, 1, 2, 3, 4};
static const uint8_t tofAddr[5]   = {0x30, 0x31, 0x32, 0x33, 0x34};

MultiVL53L0X tofArray(
  0x20, 5,
  xshutPins, tofAddr,
  20,
  Wire
);

void i2cRecover(int sda, int scl) {
  pinMode(sda, OUTPUT);
  pinMode(scl, OUTPUT);
  delay(10);

  // If SDA stuck LOW → clock it out
  if (digitalRead(sda) == LOW) {
    pinMode(scl, OUTPUT);

    for (int i = 0; i < 9; i++) {
      digitalWrite(scl, HIGH);
      delayMicroseconds(5);
      digitalWrite(scl, LOW);
      delayMicroseconds(5);
    }
  }

  // Send STOP
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(5);
  digitalWrite(scl, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda, HIGH);

  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  
  Wire.end();
  Wire.begin(sda, scl, 400000);  // 400kHz fast mode

  vTaskDelay(pdMS_TO_TICKS(10));
}

// =====================================================
// Task handles
// =====================================================
static TaskHandle_t tofTaskHandle       = nullptr;
static TaskHandle_t motorTaskHandle     = nullptr;
static TaskHandle_t explorerTaskHandle  = nullptr;
static TaskHandle_t plannerTaskHandle   = nullptr;
static TaskHandle_t telemetryTaskHandle = nullptr;
static TaskHandle_t userTaskHandle = nullptr;

// =====================================================
// Forward declarations
// =====================================================
static void motorTask(void* arg);
static void tofTask(void* arg);
static void explorerTask(void* arg);
static void telemetryTask(void* arg);
static void plannerTask(void* arg);
static void userTask(void* arg);

static void logToDbg(const String& s) {
  dbg.println(s);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(921600);
  vTaskDelay(pdMS_TO_TICKS(200));
  i2cRecover(I2C_SDA, I2C_SCL);

  // ---------------- WiFi + OTA + web-serial ----------------
  {
    WiFiOtaWebSerial::Config cfg;
    cfg.ssid = "PhucWifi";
    cfg.pass = "000000001";
    cfg.hostname = "PhucC_Esp32s3_mice";
    cfg.otaPassword = "";          // optional
    cfg.wifiCore = 0;               // WiFi/OTA/WebSerial task on Core 1
    cfg.wifiTaskStack = 10 * 1024;
    cfg.wifiTaskPrio = 3;
    cfg.serviceDelayMs = 5;

    dbg.begin(cfg);
    dbg.println("Boot OK");
  }

  // ---------------- Motors ----------------
  {
    bool okL = leftMotor.begin(L_PINS, 0, 20000, 10);
    bool okR = rightMotor.begin(R_PINS, 1, 20000, 10);

    if (!okL || !okR) {
      Serial.println("Motor begin() failed");
      while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    leftMotor.setSpeedPID(0.004f, 0.0080f, 0.00005f, 0.80f, 0.50f, 25.0f, 4.f);
    rightMotor.setSpeedPID(0.004f, 0.0080f, 0.00005f, 0.80f, 0.50f, 25.0f, 4.f);

    dbg.println("Motors OK");
  }

  // ---------------- TOF sensors ----------------
  {
    bool ok = tofArray.begin();
    dbg.println(String("TOF begin ") + (ok ? "OK" : "FAIL"));
  }

  // Point 1: Low Battery (e.g., Raw ADC was 2800, Multimeter read 7.20V)
  // Point 2: Full Battery (e.g., Raw ADC was 3350, Multimeter read 8.40V)
  // mouseBattery.setCalibration(2800, 7.20f, 3350, 8.40f);
  
  // mouseBattery.begin(3); // Start the task on your ADC pin

  // OPTIONAL: recalibrate explicitly after everything is stable

  // ---------------- FloodFill Explorer (visual web) ----------------
  {
    explorer.setLog(logToDbg);

    FloodFillExplorer::Config mc;
    mc.port = 81;
    mc.autoRun = false;
    // mc.stepPeriodMs = 300;

    explorer.begin(mc);

    // Build the easy maze:
    // - boundaries auto-added by setTruthFromWalls(forceBoundaries=true)
    // - vertical wall between x=4 and x=5 from y=1..14 except y=8 (gap)
    uint8_t maze[16][16];
    memcpy(maze, MAZE_SRC, sizeof(maze));

    // normalizePairs=true fixes neighbor walls automatically
    // forceBoundaries=true adds the outer border walls
    explorer.setTruthFromWalls(maze, true, true);

    dbg.println("Explorer web on port 81");
  }

  // =====================================================
  // Create tasks
  // =====================================================
  xTaskCreatePinnedToCore(motorTask,     "motor",     4096, nullptr, 3, &motorTaskHandle,     0);
  xTaskCreatePinnedToCore(tofTask,       "tof",       4096, nullptr, 2, &tofTaskHandle,       0);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 1, &telemetryTaskHandle, 1);
  xTaskCreatePinnedToCore(explorerTask,  "explorer",  8192, nullptr, 2, &explorerTaskHandle,  1);
  xTaskCreatePinnedToCore(plannerTask,   "planner",   4096, nullptr, 2, &plannerTaskHandle,   1);
  xTaskCreatePinnedToCore(userTask,      "user",      4096, nullptr, 2, &userTaskHandle,      0);

  dbg.println("Ready");
}

void loop() {
  vTaskDelay(100);
}


// =====================================================
// Tasks
// =====================================================

static void userTask(void* arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    
    // float err = tofArray.getError(0); // positive lean right, negative lean left
    // leftMotor.setSpeedTPS(500 + err );
    // rightMotor.setSpeedTPS(500 - err );

    vTaskDelayUntil(&lastWake, period);
  }
}


static void plannerTask(void* arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(100);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
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
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    tofArray.update();    
    vTaskDelayUntil(&lastWake, period);
  }
}

static void motorTask(void* arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    leftMotor.update();
    rightMotor.update();
    vTaskDelayUntil(&last, period);
  }
}

static void telemetryTask(void* arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(200);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    tof_debug_s();
    // motor_debug_s();
    // wifi_debug();
    vTaskDelayUntil(&last, period);
  }
}
// =====================================================
// Debug print functions
// =====================================================

static void motor_debug_s() {
  Serial.print("MOTOR | L ticks="); Serial.print(leftMotor.getTicks());
  Serial.print(" mm=");             Serial.print(leftMotor.ticksToMM());
  Serial.print(" tps=");            Serial.print(leftMotor.getTicksPerSecond(), 1);

  Serial.print(" | R ticks=");      Serial.print(rightMotor.getTicks());
  Serial.print(" mm=");             Serial.print(rightMotor.ticksToMM());
  Serial.print(" tps=");            Serial.println(rightMotor.getTicksPerSecond(), 1);
}

static void motor_debug() {
  dbg.print("[MOTOR] L ticks="); dbg.print(String(leftMotor.getTicks()));
  dbg.print(" mm=");             dbg.print(String(leftMotor.ticksToMM()));
  dbg.print(" tps=");            dbg.print(String(leftMotor.getTicksPerSecond()), 1);

  dbg.print(" | R ticks=");      dbg.print(String(rightMotor.getTicks()));
  dbg.print(" mm=");             dbg.print(String(rightMotor.ticksToMM()));
  dbg.print(" tps=");            dbg.println(String(rightMotor.getTicksPerSecond()), 1);
}

static void tof_debug_s() {

  for (uint8_t i = 0; i < tofArray.sensorCount(); i++) {
    Serial.printf("S%u=%u ", i, tofArray.getDistance(i));
  }
  float err = tofArray.getError(0);
  Serial.printf(" | error: %.2f | %.2f |%u", err, tofArray.CENTER_TARGET, tofArray.err_count);
  Serial.println();
}

static void tof_debug() {
  dbg.print("[TOF] ");

  for (uint8_t i = 0; i < tofArray.sensorCount(); i++) {
    dbg.print("S");
    dbg.print(String(i));
    dbg.print("=");
    dbg.print(String(tofArray.getDistance(i)));
    dbg.print(" ");
  }
  
  dbg.println("");
}

static void wifi_debug() {

  static uint32_t lastMs = 0;
  static bool printedUrl = false;
  if (printedUrl) return;

  const uint32_t now = millis();
  if (now - lastMs < 2000) return; // every 2s
  lastMs = now;

  if (!WiFi.isConnected()) {
    dbg.println("[WiFi] Not connected");
    printedUrl = false;
    return;
  }

  const String ipStr = dbg.ip();
  const String gwStr = WiFi.gatewayIP().toString();
  const int rssi = WiFi.RSSI();

  dbg.println("[WiFi] Connected");
  dbg.println("[WiFi] IP: " + ipStr);
  dbg.println("[WiFi] Gateway: " + gwStr);
  dbg.println("[WiFi] RSSI: " + String(rssi) + " dBm");

  if (!printedUrl) {
    dbg.println("Open: http://" + ipStr + ":81/");
    printedUrl = true;
  }
}
