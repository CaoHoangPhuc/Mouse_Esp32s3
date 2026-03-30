# Mouse_esp32s3

ESP32-S3 micromouse project for a floodfill-based maze runner.

Current project version: `0.3.3`

## Current Status

This repository now includes the first integrated hardware-oriented control stack:
- dual DC motor control with encoder-based speed PID
- multi-VL53L0X wall sensing
- battery monitoring with safety states
- primitive motion executor for `move`, `back`, `turn 90 deg`, and `turn 180 deg`
- floodfill maze state and web visualizer
- floodfill maze web sync over WebSocket instead of browser polling
- task-based planner / executor / telemetry flow
- Wi-Fi OTA, port `80` control page, and telnet/debug tools
- centralized configuration for hardware and tuning values
- Wi-Fi boot logging now waits for a real STA IP before printing HTTP/upload URLs, and the reconnect loop no longer retries while the station is already connecting
- SPIFFS persistence for saved maze memory when the shortest path is known
- build fixes for the SPIFFS persistence integration and WebSocket accept path on the current ESP32 core
- when a saved maze is restored successfully at boot, the robot marks shortest-path-ready immediately and shows white LED in idle
- explore now uses the current pose as the active home target for goal/home swapping, so `resetpose` affects the next explore loop as expected
- far/open TOF readings now count as valid maze observations, so revisits can clear stale remembered walls for recovery
- battery monitoring is now telemetry-only and no longer blocks or aborts motion primitives
- pose and goal are runtime-only again; SPIFFS now stores only maze wall memory
- SPIFFS persistence now lives in a dedicated module for easier control and future changes
- wall-centering now blends smoothly when transitioning between both-wall centering and single-wall following
- wall-centering now uses left-target and right-target references consistently for both dual-wall and single-wall follow
- wall-centering now captures left/right targets once at the start of a straight move, only when both walls are visible and nearly balanced
- added `test motor both` for a simple full-power forward/reverse bench loop on both motors
- compact status printing can now hide `tps=(left,right)` with a config flag when motor-speed text is too noisy
- serial output can now be globally muted with a config flag while keeping the serial port open for input
- `speedrun [1-4]` is now phase-aware, and phase 1 runs the known shortest path directly without wall updates, ACK handshakes, or snap-center recovery motions
- `speedrun 1` now temporarily mutes serial output while the run is active, then restores it automatically on goal/idle/fault
- `speedrun 1` now flips the active target at the goal and continues the shortest-path run back home before finishing
- `speedrun 1` now restores the active target back to the original goal after the return-home leg finishes, so the web/planner state is ready for the next run
- `speedrun` now rebuilds its start/home target and goal target from the current runtime pose and current runtime goal before the run begins
- `speedrun` now means `speedrun 1`, and phases 2-4 are defined as incremental layers that inherit the previous phase until tuned separately
- fixed the `speedrun 1` serial-mute build path by wiring the Wi-Fi serial mirror code to the shared config header
- the floodfill web now shows live leg timing for both `HG` and `GH`, and keeps lap history in RAM across runs until reboot/reset
- fixed the intermediate `speedrun 1` goal-flip path so a completed move is cleared before the return-home leg begins, preventing an extra logical cell advance
- `speedrun 1` now keeps successful primitive transitions smooth by skipping the normal completion brake/hold between moves and turns, while still stopping normally on finish or fault
- periodic RTOS task loops now have a lightweight watchdog that warns when a loop misses its expected cadence, including task name, expected period, actual interval, lateness, and core id
- `explorerTask` now uses `vTaskDelayUntil(...)` in normal operation so it follows the same fixed-cadence scheduling rule as the other steady-state task loops
- global Serial output is enabled again for normal boot/runtime logs, while `speedrun 1` still temporarily mutes Serial only during the active run

This is a bring-up and integration version, not a race-tuned final solver yet.

Release note:
- see [RELEASE_0.3.0.md](Documents/Arduino/Mouse_esp32s3/RELEASE_0.3.0.md) for the packaged solver milestone summary and the next target after this release

## Architecture

### Entry and application split
- [Mouse_esp32s3.ino](Documents/Arduino/Mouse_esp32s3\Mouse_esp32s3.ino): thin Arduino entrypoint with `setup()`, `loop()`, `userTask()`, and `plannerTask()` wrappers only
- [AppRuntime.h](Documents/Arduino/Mouse_esp32s3\AppRuntime.h): app interface exposed to the `.ino` wrapper
- [AppRuntime.cpp](Documents/Arduino/Mouse_esp32s3\AppRuntime.cpp): application logic, globals, startup flow, command handling, background tasks, planner integration
- [Config.h](Documents/Arduino/Mouse_esp32s3\Config.h): centralized hardware pins, thresholds, Wi-Fi settings, and motion tuning constants
- [RobotTypes.h](Documents/Arduino/Mouse_esp32s3\RobotTypes.h): shared enums and `RobotState`

### Motion and hardware
- [DcMotor.h](Documents/Arduino/Mouse_esp32s3\DcMotor.h): low-level motor + encoder + speed PID
- [DcMotor.cpp](Documents/Arduino/Mouse_esp32s3\DcMotor.cpp): PWM / ISR / TPS estimation
- [MotionController.h](Documents/Arduino/Mouse_esp32s3\MotionController.h): primitive motion interface
- [MotionController.cpp](Documents/Arduino/Mouse_esp32s3\MotionController.cpp): move / turn / stop execution and fault detection
- [Battery.h](Documents/Arduino/Mouse_esp32s3\Battery.h): battery voltage API
- [Battery.cpp](Documents/Arduino/Mouse_esp32s3\Battery.cpp): ADC sampling and battery-state classification

### Sensors and planning
- [MultiVL53L0X.h](Documents/Arduino/Mouse_esp32s3\MultiVL53L0X.h): TOF array API and wall observation structure
- [MultiVL53L0X.cpp](Documents/Arduino/Mouse_esp32s3\MultiVL53L0X.cpp): sensor reads, correction, thresholding, wall interpretation
- [FloodFillExplorer.h](Documents/Arduino/Mouse_esp32s3\FloodFillExplorer.h): floodfill planner / map / web interface
- [FloodFillExplorer.cpp](Documents/Arduino/Mouse_esp32s3\FloodFillExplorer.cpp): planner logic and live web UI
- [PersistenceStore.h](Documents/Arduino/Mouse_esp32s3\PersistenceStore.h): SPIFFS persistence interface for saved maze memory
- [PersistenceStore.cpp](Documents/Arduino/Mouse_esp32s3\PersistenceStore.cpp): SPIFFS file format and load/save/clear implementation

### Connectivity
- [WiFiOtaWebSerial.h](Documents/Arduino/Mouse_esp32s3\WiFiOtaWebSerial.h): OTA and lightweight port `80` control page API
- [WiFiOtaWebSerial.cpp](Documents/Arduino/Mouse_esp32s3\WiFiOtaWebSerial.cpp): Wi-Fi task, control page, Arduino OTA, browser upload page, LED control

## Runtime Flow

1. `setup()` in the `.ino` forwards to `MainApp::setupApp(...)`.
2. `AppRuntime.cpp` initializes Wi-Fi, motors, TOF, battery, floodfill explorer, and tasks.
3. The robot does not write start-cell walls into the maze at boot; first wall observation happens when exploration or speed-run logic begins.
4. `tofTask` continuously updates TOF readings.
5. `motorTask` continuously updates motor PID loops.
6. `userTask()` remains visible in the `.ino`, but forwards to `MainApp::userTaskBody(...)`.
7. `plannerTask()` remains visible in the `.ino`, but forwards to `MainApp::plannerTaskBody(...)`.
8. `explore` only starts with `snapCenter()` when the wall behind the robot is already known to exist; otherwise the run-start snap is skipped and the planner is allowed to continue immediately.
9. After a motion completes in explore hardware mode, the runtime refreshes robot sensor state, applies wall sensing for the new pose once, ACKs the pending planner action, and only then holds the motors in hard-stop briefly before allowing the next motion.
10. After a 90-degree or 180-degree turn in explore hardware mode, if the wall behind the robot is known to exist, the runtime runs `snapCenter()` before wall registration and before ACKing the turn so the next planner action starts from the re-centered pose.
11. `speedrun 1` uses the shortest known path directly: no wall-map updates, no floodfill ACK handshake, and no snap-center recovery steps during the run.
12. `telemetryTask` now focuses on the selected manual-test loop output instead of always printing the compact status line every cycle.
13. `explorerTask` serves the web maze view and now runs on a fixed `vTaskDelayUntil(...)` cadence during normal operation.
14. When the robot is standing still and ready for the next planner action, the runtime refreshes wall sensing from the current cell before calling floodfill again, so valid current-cell observations can overwrite stale wall memory.
15. In explore mode, the runtime can continue after a reached target by keeping the current pose, letting `FloodFillExplorer` flip the target between the original goal rectangle and the original home rectangle, and then resuming exploration from where the robot stands.
16. Explore now stops automatically and prints that the shortest path is known once the same best-known home-to-goal cost has remained unchanged for the configured number of consecutive round trips.
17. Floodfill now distinguishes the single physical start pose from a separate home rectangle, so target toggling happens between the configured home region and goal region.
18. When explore continues immediately after a reached target, the runtime now skips the normal post-motion hold so the goal-to-home transition starts without the extra 100 ms pause.
19. On boot, the runtime restores saved maze wall memory from SPIFFS when that file exists, while pose and goal still come from the current runtime/config state.
20. `clearmaze` clears only wall memory and removes the saved maze file without changing the current pose or goal.
20. When explore decides the shortest path is known, the runtime saves maze memory to SPIFFS automatically before going idle.

Planner synchronization note:
- `plannerTaskBody()` now uses `MotionController` as the single source of truth for motion completion/busy state before dispatching the next action.
- This prevents a race where `robotState.motionStatus` could still be stale while the controller had already left `RUNNING`, which could otherwise cause repeated `move1` starts before wall sensing and ACK completed.

Loop watchdog note:
- steady-state RTOS loops now log `[LOOP WARN]` when the actual loop interval exceeds the expected period plus a configurable tolerance
- the warning includes the task name, expected period, actual interval, lateness, and current core id
- setup delays, one-shot waits, and OTA special pause branches still use plain `vTaskDelay(...)` and are not treated as periodic watchdog loops

Turn behavior note:
- Floodfill explore uses a real `ACT_TURN_180` again for dead-end reversals.
- When a completed turn will be followed by `snapCenter()`, walls are now sensed and applied only after `snapCenter()` finishes, so the maze is updated from the re-centered pose instead of the immediate post-turn pose.

Explore loop note:
- `FloodFillExplorer` now toggles its target between the original goal rectangle and the original home rectangle when a target is reached.
- In hardware explore mode, the runtime now re-enables exploring after a reached target when `AppConfig::Explorer::CONTINUE_AFTER_GOAL` is `true`.
- This keeps the robot at its current pose and lets return trips keep discovering or correcting wall memory.
- When returning to the home target, the planner now requires the robot to match the configured start heading before the home leg is considered complete, so the next run starts with the same orientation as boot.
- The runtime tracks the best-known cost from the original home region to the original goal region.
- After each completed goal->home round trip, if that best-known cost is unchanged, the stable round-trip count increases.
- When the count reaches `AppConfig::Explorer::SHORTEST_PATH_STABLE_ROUND_TRIPS`, explore stops and prints `shortest path known`.
- Reaching a target no longer rewrites the explorer's configured start marker; the original start remains fixed while only the active target toggles between goal and home.
- The original home/goal references used for toggling are now taken from `setHomeRect()` and `setGoalRect()`, so the loop uses the configured maze values instead of the class default placeholders.

## Configuration

All hard configuration now lives in [Config.h](Documents/Arduino/Mouse_esp32s3\Config.h).

Key sections:
- `AppConfig::Battery`: ADC pin, battery calibration, warning/critical thresholds
- `AppConfig::Maze`: start pose and goal rectangle
  Default config starts at `(0,0)`, heading south, with a home rectangle at `(0,0)` size `1x1` and a goal rectangle defined in `Config.h`.
- `AppConfig::Wifi`: Wi-Fi / OTA / control-page settings
- `AppConfig::Debug`: serial-output switches, runtime debug flags, and loop-watchdog timing thresholds
- `AppConfig::I2C`: SDA/SCL and bus speed
- `AppConfig::Tof`: sensor addresses, XSHUT pins, and wall threshold
- `AppConfig::Motors`: motor pins, encoder inversion, PWM and PID settings
- `AppConfig::Motion`: one-cell distance, turn ticks, speed and timeout tuning
- `AppConfig::Explorer`: web floodfill UI settings, whether explore should continue looping between goal and home after a target is reached, and how many stable round trips are required before the shortest path is considered known

## Persistence

- maze wall memory is saved in SPIFFS automatically when explore reports `shortest path known`
- on boot, the runtime restores saved maze wall memory if the SPIFFS file is present
- `clearmaze` now clears only the remembered wall map and deletes the saved maze-memory file; it no longer resets pose or goal

## Dependencies / Build Expectations

- the sketch is intended to build from the repository root as a standard Arduino sketch folder
- target platform is an ESP32-S3 board using the Arduino ESP32 core
- external libraries used by the code include `PCF8574`, `VL53L0X`, and `Adafruit_NeoPixel`
- compile/build verification is still pending in this repository, so the first build should be treated as a bring-up check rather than a guaranteed known-good baseline

## Arduino IDE Build / Upload

If the project is not already under your Arduino sketch folder, place or copy the whole folder here so the sketch keeps its Arduino layout:

- `Documents/Arduino/Mouse_esp32s3`

Then build and upload like this:

1. Open [Mouse_esp32s3.ino](Documents/Arduino/Mouse_esp32s3\Mouse_esp32s3.ino) in Arduino IDE.
2. Select your ESP32-S3 board target in `Tools`.
3. Set the ESP32-S3 board options to match this project:
   - `USB CDC On Boot`: `Enabled`
   - `Partition Scheme`: `Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)` or the closest current ESP32-core label that gives about `1.9MB code`, `1.9MB OTA`, and `128KB/190KB SPIFFS`
   - `Arduino Runs On`: `Core 1`
4. Select the correct serial `COM` port in `Tools -> Port`.
5. Click `Verify` first.
6. Click `Upload`.

Bring-up notes:

- if the board does not enumerate correctly, re-check that `USB CDC On Boot` is enabled
- if the partition scheme is wrong, OTA or SPIFFS features may fail later even if the sketch compiles
- after upload, open Serial Monitor or telnet and use `help` / `status` for first validation
- Wi-Fi OTA and browser upload on port `82` are convenient later, but first bring-up should still start from a normal USB upload

## Robot Modes

Defined in [RobotTypes.h](Documents/Arduino/Mouse_esp32s3\RobotTypes.h):
- `ROBOT_MODE_IDLE`
- `ROBOT_MODE_MANUAL_TEST`
- `ROBOT_MODE_EXPLORE`
- `ROBOT_MODE_SPEED_RUN`
- `ROBOT_MODE_FAULT`

## Motion Primitives

Implemented in [MotionController.cpp](Documents/Arduino/Mouse_esp32s3\MotionController.cpp):
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
- battery state is still reported, but it no longer aborts or blocks motion
- motor control now separates coast stop from active brake: motion-completion and transition paths use brake, while general stop/idle paths can still coast
- on the current driver setup, `coastStop()` explicitly sets `IN1=LOW`, `IN2=LOW`, `PWM=0`, while `brakeStop()` routes through the zero-duty motor drive path that holds the wheel in position
- motor commands inside the PWM dead zone now coast at zero instead of forcing a minimum forward/reverse duty
- motion start/end debug hooks in the runtime for tracing primitive flow during tuning
- a short post-motion hard-stop hold after sensing/ACK so the robot pauses only when the system is otherwise ready for the next action
- `snapCenter()` runs as one primitive: reverse short, hard stop, hold briefly, then forward short
- `snapCenter()` does not change the logical maze pose; it is a physical re-centering primitive only
- task-context millisecond waits now use `vTaskDelay(...)` instead of Arduino `delay(...)` so other FreeRTOS tasks can keep running cleanly during those pauses

## Serial Commands

Available from the main sketch:
- `help`
- `status`
- `explore`
- `explore n`
- `speedrun`
- `speedrun 1`
- `speedrun 2`
- `speedrun 3`
- `speedrun 4`
- `idle`
- `stop`
- `brake`
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
- `setgoal x y w h`
- `clearmaze`
- `test battery`
- `test sensors`
- `test sensorsraw`
- `test motorl`
- `test motorr`
- `test motor both`
- `test encoders`

Manual motion note:
- `testsnap` triggers the combined `snapCenter()` primitive so the back-then-forward recenter motion is executed and reported as one normal motion

Console note:
- periodic debug output pauses briefly while you type on serial or telnet, then resumes automatically
- `restart` closes the TCP debug console first, then reboots the ESP32
- `brake` applies the active motor brake immediately for bench testing and tuning, while `stop` follows the normal coast-to-idle path
- the TCP debug console close path follows the current ESP32 `NetworkClient` API to avoid deprecated-call warnings during build
- the TCP debug console listens on port `2323`
- port `80` now serves a simple control page that shows the robot hostname and offers `Reconnect Telnet` plus `Cycle LED`
- the port `80` control page now also includes a grouped quick-reference for the main CLI commands, so users can see the accepted commands and their purpose before opening telnet
- the port `80` control page now renders the current battery voltage/state directly into the page when it loads, without background polling
- the port `80` telnet reconnect action forcibly disconnects the current TCP debug client before launching a fresh telnet connection to the robot IP and configured debug port
- the port `80` control page now also has an `Open Floodfill` button that jumps straight to the live floodfill viewer on port `81`
- `setgoal x y w h` now updates the active runtime goal rectangle used by `explore` for the current runtime

## Web Debugging

Port `80` is now a lightweight control page, and the floodfill explorer still runs on port `81` as the live maze debug tool.

Expected use:
- verify discovered walls
- verify current pose
- inspect floodfill distance field
- confirm planner action sequence
- the floodfill page now pushes state over WebSocket, so the browser no longer polls the maze state over HTTP while the mouse is running
- the floodfill page now loads with `AutoACK(sim)` turned off by default, so browser control waits for the real robot ACK unless you explicitly enable simulation
- with `AutoACK(sim)` off, the port `81` `Step` and `Run` buttons now hand control to the real app runtime: `Step` starts `explore 1`, `Run` starts real explore without clearing the current maze, and `Pause` / `Reset` map to the runtime pause/reset flow
- the floodfill web page no longer exposes a web `Set Start` editor; start is now treated as a configured/runtime app concern instead of a browser-side control
- the floodfill web page now prints planner status in a wrapped status bar so long state text does not stretch or resize the whole page while the mouse is running
- when `AutoACK(sim)` is enabled, the floodfill web `Step` button advances one full simulated cell move, consuming any required turn actions first, while `Run` continues auto-running in simulated mode

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
  - green: explore active before first reached target
  - blue: explore active after the first reached target
  - white: shortest path known / final reached-target state
  - red: fault mode
  - off: idle / non-explore normal state

Use this file for the browser uploader:
- `Mouse_esp32s3.ino.bin`

Do not use these for the browser uploader:
- `Mouse_esp32s3.ino.merged.bin`
- bootloader or partition binaries

Related config in [Config.h](Documents/Arduino/Mouse_esp32s3\Config.h):
- `AppConfig::Wifi::ENABLE_UPLOAD_WEB`
- `AppConfig::Wifi::UPLOAD_WEB_PORT`

## Hardware / Tuning Notes

Current values are placeholders and will need on-robot tuning in [Config.h](Documents/Arduino/Mouse_esp32s3\Config.h):
- battery ADC pin and calibration values
- `cellDistanceMm`
- `turnTicks90`
- `turnTicks180`
- move and turn TPS
- wall threshold
- wall-centering PID gains (`CENTER_PID_KP/KI/KD`)
- wall-centering PID integral/output limits
- post-motion hard-stop hold before the next action
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
- battery voltage now uses the measured ADC node voltage plus the configured divider ratio as the primary pack-voltage estimate
- on ESP32, battery ADC debug now uses the calibrated millivolt reading path instead of a simple `raw / 4095 * 3.3` approximation
- the current tuned divider constant uses an effective top resistor value of `56k` with `18k` bottom to better match the measured pack voltage on this board

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

