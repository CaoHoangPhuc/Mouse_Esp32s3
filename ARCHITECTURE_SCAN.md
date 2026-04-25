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
