# Mouse_esp32s3

ESP32-S3 micromouse project for a floodfill-based maze runner.

## Current Status

This repository now includes the first integrated hardware-oriented control stack:
- dual DC motor control with encoder-based speed PID
- multi-VL53L0X wall sensing
- battery monitoring with safety states
- primitive motion executor for `move one cell` and `turn 90 deg`
- floodfill maze state and web visualizer
- task-based planner / executor / telemetry flow
- Wi-Fi OTA and web serial logging
- centralized configuration for hardware and tuning values

This is a bring-up and integration version, not a race-tuned final solver yet.

## Architecture

### Entry and application split
- [Mouse_esp32s3.ino](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Mouse_esp32s3.ino): thin Arduino entrypoint with `setup()`, `loop()`, `userTask()`, and `plannerTask()` wrappers only
- [main.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\main.h): app interface exposed to the `.ino` wrapper
- [main.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\main.cpp): application logic, globals, startup flow, command handling, background tasks, planner integration
- [Config.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Config.h): centralized hardware pins, thresholds, Wi-Fi settings, and motion tuning constants
- [RobotTypes.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\RobotTypes.h): shared enums and `RobotState`

### Motion and hardware
- [DcMotor.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\DcMotor.h): low-level motor + encoder + speed PID
- [DcMotor.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\DcMotor.cpp): PWM / ISR / TPS estimation
- [MotionController.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\MotionController.h): primitive motion interface
- [MotionController.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\MotionController.cpp): move / turn / stop execution and fault detection
- [Battery.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Battery.h): battery voltage API
- [Battery.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Battery.cpp): ADC sampling and battery-state classification

### Sensors and planning
- [MultiVL53L0X.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\MultiVL53L0X.h): TOF array API and wall observation structure
- [MultiVL53L0X.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\MultiVL53L0X.cpp): sensor reads, correction, thresholding, wall interpretation
- [FloodFillExplorer.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\FloodFillExplorer.h): floodfill planner / map / web interface
- [FloodFillExplorer.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\FloodFillExplorer.cpp): planner logic and live web UI

### Connectivity
- [WiFiOtaWebSerial.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\WiFiOtaWebSerial.h): OTA and web log API
- [WiFiOtaWebSerial.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\WiFiOtaWebSerial.cpp): Wi-Fi task, log page, OTA, LED control

## Runtime Flow

1. `setup()` in the `.ino` forwards to `MainApp::setupApp(...)`.
2. `main.cpp` initializes Wi-Fi, motors, TOF, battery, floodfill explorer, and tasks.
3. `tofTask` continuously updates TOF readings.
4. `motorTask` continuously updates motor PID loops.
5. `userTask()` remains visible in the `.ino`, but forwards to `MainApp::userTaskBody(...)`.
6. `plannerTask()` remains visible in the `.ino`, but forwards to `MainApp::plannerTaskBody(...)`.
7. `telemetryTask` prints compact runtime state to serial.
8. `explorerTask` serves the web maze view.

## Configuration

All hard configuration now lives in [Config.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Config.h).

Key sections:
- `AppConfig::Battery`: ADC pin, battery calibration, warning/critical thresholds
- `AppConfig::Maze`: start pose and goal rectangle
- `AppConfig::Wifi`: Wi-Fi / OTA / web logging settings
- `AppConfig::I2C`: SDA/SCL and bus speed
- `AppConfig::Tof`: sensor addresses, XSHUT pins, wall threshold
- `AppConfig::Motors`: motor pins, encoder inversion, PWM and PID settings
- `AppConfig::Motion`: one-cell distance, turn ticks, speed and timeout tuning
- `AppConfig::Explorer`: web floodfill UI settings

## Robot Modes

Defined in [RobotTypes.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\RobotTypes.h):
- `ROBOT_MODE_IDLE`
- `ROBOT_MODE_MANUAL_TEST`
- `ROBOT_MODE_EXPLORE`
- `ROBOT_MODE_SPEED_RUN`
- `ROBOT_MODE_FAULT`

## Motion Primitives

Implemented in [MotionController.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\MotionController.cpp):
- `moveOneCell()`
- `turnLeft90()`
- `turnRight90()`
- `stop()`

Primitive execution currently includes:
- primitive timeout
- simple stall detection
- front-wall stop support
- side-wall centering correction
- battery-critical abort

## Serial Commands

Available from the main sketch:
- `help`
- `status`
- `explore`
- `speedrun`
- `idle`
- `stop`
- `move`
- `left`
- `right`
- `resetpose x y h`
- `test battery`
- `test sensors`
- `test motorl`
- `test motorr`
- `test encoders`

## Web Debugging

The floodfill explorer runs on port `81` and is intended as a live debug tool.

Expected use:
- verify discovered walls
- verify current pose
- inspect floodfill distance field
- confirm planner action sequence

## Hardware / Tuning Notes

Current values are placeholders and will need on-robot tuning in [Config.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Config.h):
- battery ADC pin and calibration values
- `cellDistanceMm`
- `turnTicks90`
- move and turn TPS
- wall threshold
- front stop distance
- `mmPerTick`

Battery divider note:
- current comments assume a `47k / 18k` divider into `GPIO 3`

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
7. Run `explore` in a simple maze.
8. Only after stable exploration, tune `speedrun`.
