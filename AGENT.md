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
