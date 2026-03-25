# TODO.md

## Immediate

- Compile the project against the intended ESP32-S3 board and fix any board/core compile issues.
- Verify all new source files are included by the Arduino build.
- Replace remaining mojibake / encoding corruption in comments and UI strings.
- Confirm the battery ADC pin is correct for the real board.
- Confirm the current Wi-Fi credentials strategy is acceptable for development.

## Hardware Bring-Up

- Measure battery raw ADC at low and full pack voltage.
- Tune battery warning and critical thresholds.
- Verify left motor direction matches commanded forward motion.
- Verify right motor direction matches commanded forward motion.
- Verify encoder sign and tick counts for both motors.
- Tune `mmPerTick` from measured travel distance.

## Motion Tuning

- Tune `cellDistanceMm` so one cell move is repeatable.
- Tune `turnTicks90` for accurate 90-degree turns.
- Tune `moveSpeedTps` and `turnSpeedTps`.
- Tune `stallTimeoutMs` so valid slow motion does not false-fail.
- Tune `frontStopMm` for safe wall approach.
- Tune `centeringGain` so corridor tracking is stable and does not oscillate.

## Sensor Tuning

- Verify sensor layout detection for the actual hardware version.
- Confirm left/front/right sensor mapping is correct.
- Tune wall threshold from measured distances.
- Add filtering or confidence logic for inconsistent TOF readings.
- Add better front-wall alignment behavior near cell boundaries.

## Planner / Maze

- Verify pose updates after each primitive in a real maze.
- Validate discovered wall writes into the expected maze cells.
- Test dead-end and backtracking behavior on hardware.
- Persist or export known maze data for debugging.
- Add path compression for the second run.
- Add a clear distinction between exploration completion and optimized second-run execution.

## Observability

- Extend web UI to show battery, mode, motion status, and live wall observation validity.
- Add more compact serial status output for tuning sessions.
- Add explicit fault codes / categories instead of only free-form strings.
- Log primitive start/end timing for motion tuning.

## Nice To Have

- Move configuration constants into a dedicated config header.
- Add EEPROM / NVS storage for tuned values.
- Move Wi-Fi credentials out of the sketch.
- Add an optional simulator mode switch in the main sketch.
- Add automated test scaffolding for maze-state updates and planner decisions.
