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
- Open [Mouse_esp32s3.ino](c:\Users\donot\OneDrive\Documents\Arduino\Mouse_esp32s3\Mouse_esp32s3.ino) from the Arduino sketch folder, not as an arbitrary loose file.
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
- During OTA/web upload, expect the onboard LED to turn blue immediately, then blink blue, then turn green on success or red on error/abort.
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

- Battery critical should always stop motion.
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
