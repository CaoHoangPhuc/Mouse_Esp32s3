# TODO.md

## Immediate

- Compile the project against the intended ESP32-S3 board and fix any board/core compile issues.
- Verify all new source files are included by the Arduino build.
- Verify browser upload on port 82 works reliably with the intended `.bin` workflow.
- If browser upload still reports `network error`, capture serial logs around `[WEB OTA] Start`, `Success`, and reboot scheduling to separate HTTP-response issues from flash-write issues.
- If browser upload still aborts, compare whether the last serial line is `[WEB OTA] Received ... bytes`, `[WEB OTA] Aborted`, or a Wi-Fi reconnect event to isolate transport vs. flash finalization.
- Current evidence points to a mid-transfer network drop after partial progress, not an OTA password or `.bin` selection issue.
- Update docs/examples if the Arduino builder needs any filename/layout adjustments.
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
- Verify the `47k / 18k` battery divider reading matches the expected ADC range.

## Motion Tuning

- Tune `cellDistanceMm` so one cell move is repeatable.
- Tune `turnTicks90` for accurate 90-degree turns.
- Tune `turnTicks180` for reliable U-turns without overshoot.
- Tune `moveSpeedTps` and `turnSpeedTps`.
- Tune `stallTimeoutMs` so valid slow motion does not false-fail.
- Tune `frontStopMm` for safe wall approach.
- Tune `centeringGain` so corridor tracking is stable and does not oscillate.
- Mark final tuned values clearly in `Config.h`.

## Sensor Tuning

- Verify sensor layout detection for the actual hardware version.
- Confirm left/front/right sensor mapping is correct.
- Confirm whether the temporary S3 workaround should stay on `ignore`, switch to `mirror S0->S3`, or be removed after hardware fixes.
- Tune wall threshold from measured distances.
- Add filtering or confidence logic for inconsistent TOF readings.
- Add better front-wall alignment behavior near cell boundaries.

## Planner / Maze

- Confirm the start cell stays unknown at boot and is only observed when explore/speed-run begins.
- Verify pose updates after each primitive in a real maze.
- Validate discovered wall writes into the expected maze cells.
- Validate the ASCII `maze` dump against the physical maze after each test.
- Test dead-end and backtracking behavior on hardware.
- Persist or export known maze data for debugging.
- Add path compression for the second run.
- Add a clear distinction between exploration completion and optimized second-run execution.

## Observability

- Extend web UI to show battery, mode, motion status, and live wall observation validity.
- Add more compact serial status output for tuning sessions.
- Add explicit fault codes / categories instead of only free-form strings.
- Log primitive start/end timing for motion tuning.
- Consider a dedicated calibration printout grouped around values from `Config.h`.

## Nice To Have

- Add EEPROM / NVS storage for tuned values.
- Move Wi-Fi credentials out of the sketch/config for safer sharing.
- Add an optional simulator mode switch in the main app.
- Add automated test scaffolding for maze-state updates and planner decisions.
