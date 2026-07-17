# AGENT.md

## Purpose

This repository is an ESP32-S3 micromouse project that is being built in layers:
1. hardware safety and telemetry
2. reliable single-cell motion
3. wall sensing and maze updates
4. floodfill exploration
5. conservative second-run optimization

When changing this code, prefer reliability and observability over aggressive optimization.

## Command Execution with RTK

All shell commands executed in this repository should be prefixed with `rtk` (Rust Token Killer) for token-optimized CLI proxying.
Examples:
- `rtk git status`
- `rtk make`
- `rtk ./build.sh`
- `rtk pytest`

See `~/.codex/RTK.md` for full RTK usage instructions.

## Education-Friendly Robotics Platform

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
