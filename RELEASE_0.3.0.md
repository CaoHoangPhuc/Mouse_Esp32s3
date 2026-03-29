# Release 0.3.0

`0.3.0` is the first packaged solver milestone for this project.

This release groups together:
- reliable floodfill exploration
- shortest-path discovery with home/goal looping
- saved maze-memory restore from SPIFFS
- `speedrun 1` as the first direct known-path run mode

It is not the final race release yet, but it is the first version that feels like a complete mouse solver package instead of only bring-up pieces.

## Included In 0.3.0

### Core motion and sensing
- dual DC motor control with encoder-based speed PID
- one-cell move, reverse, 90-degree turns, 180-degree turn, and `snapCenter()`
- left / front / right TOF wall sensing
- wall-centering correction during motion
- battery telemetry path

### Explore solver
- floodfill planner with live maze memory
- hardware explore flow with planner/executor split
- wall registration after motion completion
- dead-end recovery with real `turn180`
- goal/home target toggling during explore
- shortest-path-known detection from stable repeated round trips

### Maze persistence
- shortest-path maze memory saved to SPIFFS
- maze wall memory restored at boot
- pose and runtime goal kept out of SPIFFS to avoid stale swap confusion

### Web tools
- port `80` control page
- port `81` live floodfill maze page
- WebSocket maze sync for floodfill web
- live run timing with `HG` and `GH` leg history kept in RAM across runs

### Speedrun 1
- direct shortest-path action execution from known maze
- no floodfill ACK handshake
- no wall-map updates during the run
- no snapcenter recovery during the run
- optional quiet serial output while running
- goal-to-home flip support
- smooth primitive-to-primitive transitions without the normal explore stop/hold behavior

## Operational Meaning

`0.3.0` should be treated as:
- good for solver testing
- good for maze learning and shortest-path confirmation
- good for first direct shortest-path runs
- still not the final aggressive speed profile

## Not In 0.3.0 Yet

- speedrun phase 2 tuning/profile
- single-flow snake move compression
- advanced multi-cell path compression
- final race-tuned motion profiles
- complete board-level compile verification matrix

## Next Target

The next milestone after `0.3.0` is:

- `speedrun phase 2`

Current direction for that work:
- build phase 2 on top of the tuned `speedrun 1` baseline
- add a faster single-flow snake move style
- keep `speedrun 1` as the reliable fallback path

## Release Summary

`0.3.0` is the version where this project becomes a real mouse solver package:
- explore can learn
- shortest path can be recognized
- maze memory can be restored
- `speedrun 1` can execute the known path

