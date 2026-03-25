# AGENT.md

## Purpose

This repository is an ESP32-S3 micromouse project that is being built in layers:
1. hardware safety and telemetry
2. reliable single-cell motion
3. wall sensing and maze updates
4. floodfill exploration
5. conservative second-run optimization

When changing this code, prefer reliability and observability over aggressive optimization.

## Coding Priorities

- Do not break hardware safety for convenience.
- Keep motion gating tied to battery health and fault handling.
- Preserve the planner/executor split:
  - planner chooses actions
  - motion controller executes physical movement
  - pose updates only after motion completion
- Treat the web explorer as a debug instrument, not the source of physical truth.
- Prefer simple discrete-cell behaviors before adding faster multi-cell behaviors.

## Important Files

- `Mouse_esp32s3.ino`: thin Arduino entrypoint only
- `main.h`: app interface between Arduino entrypoints and the main application module
- `main.cpp`: runtime orchestration, serial command surface, startup flow, background task bodies, planner integration
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
- `main.cpp` wires sensor data into the explorer and acknowledges actions when physical motion finishes.
- `Mouse_esp32s3.ino` should stay minimal and only expose Arduino entrypoints plus task wrappers.
- `Config.h` is the single source of truth for pins, calibration constants, and runtime tuning defaults.

## Safe Change Guidelines

- If changing wheel geometry, update both tuning docs and `AppConfig::Motion` values.
- If changing TOF mapping, keep `left/front/right` semantics stable for the planner.
- If changing floodfill action names or behavior, keep the planner output limited to turn-left, turn-right, and move-forward unless the executor is extended too.
- If adding faster movement, do not remove the current primitive path; keep it as a reliable fallback.
- If adjusting battery thresholds, note the measured pack voltage and calibration source in docs or commit message.
- Keep `Mouse_esp32s3.ino` small; do not move general application logic back into it.

## Validation Expectations

Before calling a change done, try to validate these in order:
- build compiles for the target ESP32 board
- battery telemetry still reads sensible values
- sensor telemetry still reports stable left/front/right walls
- motor direction and encoder signs remain correct
- one-cell move and 90-degree turns still complete
- planner can still step through a small maze without faulting

## Repository Documentation

Keep these files current when architecture changes:
- `README.md`: public project overview and usage
- `TODO.md`: active implementation backlog
- `SKILL.md`: operator playbook / build-and-tune workflow
- `AGENT.md`: repository guidance for maintainers and coding agents
