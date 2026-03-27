# Mouse_esp32s3

ESP32-S3 micromouse project for a floodfill-based maze runner.

Current project version: `0.0.2.2`

## Current Status

This repository now includes the first integrated hardware-oriented control stack:
- dual DC motor control with encoder-based speed PID
- multi-VL53L0X wall sensing
- battery monitoring with safety states
- primitive motion executor for `move`, `back`, `turn 90 deg`, and `turn 180 deg`
- floodfill maze state and web visualizer
- task-based planner / executor / telemetry flow
- Wi-Fi OTA and web serial logging
- centralized configuration for hardware and tuning values

This is a bring-up and integration version, not a race-tuned final solver yet.

## Architecture

### Entry and application split
- [Mouse_esp32s3.ino](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Mouse_esp32s3.ino): thin Arduino entrypoint with `setup()`, `loop()`, `userTask()`, and `plannerTask()` wrappers only
- [AppRuntime.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\AppRuntime.h): app interface exposed to the `.ino` wrapper
- [AppRuntime.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\AppRuntime.cpp): application logic, globals, startup flow, command handling, background tasks, planner integration
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
- [WiFiOtaWebSerial.cpp](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\WiFiOtaWebSerial.cpp): Wi-Fi task, log page, Arduino OTA, browser upload page, LED control

## Runtime Flow

1. `setup()` in the `.ino` forwards to `MainApp::setupApp(...)`.
2. `AppRuntime.cpp` initializes Wi-Fi, motors, TOF, battery, floodfill explorer, and tasks.
3. The robot does not write start-cell walls into the maze at boot; first wall observation happens when exploration or speed-run logic begins.
4. `tofTask` continuously updates TOF readings.
5. `motorTask` continuously updates motor PID loops.
6. `userTask()` remains visible in the `.ino`, but forwards to `MainApp::userTaskBody(...)`.
7. `plannerTask()` remains visible in the `.ino`, but forwards to `MainApp::plannerTaskBody(...)`.
8. `explore` and `speedrun` start with one `snapCenter()` alignment primitive before the planner is allowed to run.
9. After a motion completes in hardware mode, the runtime holds the motors in hard-stop briefly, waits a short settle period, refreshes robot sensor state, applies wall sensing for the new pose once, then ACKs the pending planner action so the next motion cannot start before sensing is committed.
10. After a 90-degree or 180-degree turn in hardware mode, if the wall behind the robot is known to exist, the runtime runs `snapCenter()` before ACKing the turn so the next planner action starts from the re-centered pose.
11. `telemetryTask` prints compact runtime state to serial.
12. `explorerTask` serves the web maze view.

## Configuration

All hard configuration now lives in [Config.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Config.h).

Key sections:
- `AppConfig::Battery`: ADC pin, battery calibration, warning/critical thresholds
- `AppConfig::Maze`: start pose and goal rectangle
  Default config starts at `(0,0)`, heading south, with goal cell `(4,4)`.
- `AppConfig::Wifi`: Wi-Fi / OTA / web logging settings
- `AppConfig::I2C`: SDA/SCL and bus speed
- `AppConfig::Tof`: sensor addresses, XSHUT pins, and wall threshold
- `AppConfig::Motors`: motor pins, encoder inversion, PWM and PID settings
- `AppConfig::Motion`: one-cell distance, turn ticks, speed and timeout tuning
- `AppConfig::Explorer`: web floodfill UI settings

## Dependencies / Build Expectations

- the sketch is intended to build from the repository root as a standard Arduino sketch folder
- target platform is an ESP32-S3 board using the Arduino ESP32 core
- external libraries used by the code include `PCF8574`, `VL53L0X`, and `Adafruit_NeoPixel`
- compile/build verification is still pending in this repository, so the first build should be treated as a bring-up check rather than a guaranteed known-good baseline

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
- battery-critical abort
- hard stop now disables motor speed PID and coasts with `applyDuty(0)` instead of relying on `setSpeedTPS(0)`
- motor commands inside the PWM dead zone now coast at zero instead of forcing a minimum forward/reverse duty
- motion start/end debug hooks in the runtime for tracing primitive flow during tuning
- a short post-motion hard-stop hold so the robot physically settles before sensing and the next action
- a short post-motion sensor settle before wall registration so TOF readings can catch up to the new pose
- `snapCenter()` runs as one primitive: reverse short, hard stop, hold briefly, then forward short
- `snapCenter()` does not change the logical maze pose; it is a physical re-centering primitive only

## Serial Commands

Available from the main sketch:
- `help`
- `status`
- `explore`
- `speedrun`
- `idle`
- `stop`
- `restart`
- `move`
- `back`
- `testsnap`
- `left`
- `right`
- `uturn`
- `maze`
- `led cycle|rotate|off|red|green|blue|cyan|white`
- `test`
- `test off`
- `test loop status|battery|sensors|sensorsraw|encoders|maze|off`
- `resetpose x y h`
- `clearmaze`
- `test battery`
- `test sensors`
- `test sensorsraw`
- `test motorl`
- `test motorr`
- `test encoders`

Manual motion note:
- `testsnap` triggers the combined `snapCenter()` primitive so the back-then-forward recenter motion is executed and reported as one normal motion

Console note:
- periodic debug output pauses briefly while you type on serial or telnet, then resumes automatically
- `restart` closes the TCP debug console first, then reboots the ESP32
- the TCP debug console close path follows the current ESP32 `NetworkClient` API to avoid deprecated-call warnings during build
- when `AppConfig::Wifi::ENABLE_WEB_LOG` is `false`, `dbg.print/println` no longer feed the HTTP log buffer and behave as Serial-only status output
- the TCP debug console listens on port `2323`

## Web Debugging

The floodfill explorer runs on port `81` and is intended as a live debug tool.

Expected use:
- verify discovered walls
- verify current pose
- inspect floodfill distance field
- confirm planner action sequence

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
4. upload and wait for the board to reboot

During browser upload:
- the robot enters the same quiet/safe mode used for Arduino OTA
- motion, planner, telemetry, and sensor polling are paused so the upload has priority
- after a successful upload, the browser should receive `OK` first and the ESP32 reboots shortly after
- while an update is active, the firmware no longer forces a Wi-Fi reconnect cycle; this avoids killing the upload socket during brief link wobble
- if an upload aborts mid-transfer, the firmware now avoids stacking extra reconnect requests while the STA is already reconnecting
- LED status during upload:
  - immediate blue, then blinking blue: upload/OTA in progress
  - green: upload finished successfully
  - red: upload/OTA error or abort
- Robot-state LED status:
  - cyan (blue + green): explore mode active
  - white: goal reached
  - red: fault mode
  - off: idle / non-explore normal state

Use this file for the browser uploader:
- `Mouse_esp32s3.ino.bin`

Do not use these for the browser uploader:
- `Mouse_esp32s3.ino.merged.bin`
- bootloader or partition binaries

Related config in [Config.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Config.h):
- `AppConfig::Wifi::ENABLE_UPLOAD_WEB`
- `AppConfig::Wifi::UPLOAD_WEB_PORT`

## Hardware / Tuning Notes

Current values are placeholders and will need on-robot tuning in [Config.h](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Config.h):
- battery ADC pin and calibration values
- `cellDistanceMm`
- `turnTicks90`
- `turnTicks180`
- move and turn TPS
- wall threshold
- wall-centering PID gains (`CENTER_PID_KP/KI/KD`)
- wall-centering PID integral/output limits
- post-motion hard-stop hold before sensing / next action
- post-motion sensor settle delay before wall registration
- snapcenter reverse-stop hold before forward restart
- front stop distance
- `mmPerTick`

Current front-sensor behavior:
- In the current front fusion logic, S3 (front-right) is only trusted when S0 (front-left) is also valid
- If S0 reports `201` / far, S3 is exposed as `201` too
- If S0 is invalid, S3 is exposed as invalid too
- If both front sensors are far, fused `frontMm` now reports `201` instead of `0`

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
7. Tune `uturn` and `back`.
8. Use `maze` to confirm the robot's known walls match reality.
9. Run `explore` in a simple maze and verify wall registration and planner decisions.
10. Only after stable exploration, tune `speedrun`.
