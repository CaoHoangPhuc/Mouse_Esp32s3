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

Think of the project as 5 layers:
1. low-level hardware drivers
2. shared robot state
3. primitive motion control
4. maze observation and floodfill planning
5. operator tools: serial commands, web explorer, OTA/logging

Changes should usually respect those boundaries.

## Core Files

- `Mouse_esp32s3.ino`: top-level integration and task logic
- `Battery.*`: battery health and thresholds
- `DcMotor.*`: motor/encoder speed loop
- `MotionController.*`: physical primitive execution
- `MultiVL53L0X.*`: sensor reads and wall interpretation
- `FloodFillExplorer.*`: maze model, floodfill, live web view
- `RobotTypes.h`: shared enums/state contracts

## Common Workflows

### 1. Hardware bring-up
- Start in `status`, `test battery`, `test sensors`, `test encoders`.
- Verify motor direction with `test motorl` and `test motorr` before trying `move`.
- Only enable `explore` after one-cell moves and 90-degree turns are reliable.

### 2. Motion tuning
- Tune `mmPerTick` first.
- Then tune `cellDistanceMm`.
- Then tune `turnTicks90`.
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

## Design Rules

- Battery critical should always stop motion.
- Planner should issue one action at a time.
- Motion executor should be the only layer deciding primitive success/failure.
- Pose should only advance when the primitive really completes.
- Prefer conservative behavior over speed when adding new capabilities.

## Good Next Enhancements

- separate config constants into a `Config.h`
- add persistent storage for tuning values
- improve front-wall alignment
- add compressed action sequences for second run
- add richer web telemetry for live tuning

## Anti-Patterns To Avoid

- Writing directly to motors from multiple tasks
- Updating pose before primitive completion
- Mixing simulator truth walls with hardware observations in the same run
- Adding aggressive speed-run logic before reliable one-cell behavior exists
- Hiding calibration assumptions in random source files without updating docs
