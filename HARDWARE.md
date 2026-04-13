# HARDWARE.md

## Board

- MCU: ESP32-S3
- Framework: Arduino-ESP32
- Serial: USB CDC (`Serial.begin(921600)`)
- Onboard features in use:
  - BOOT button
  - RGB LED (WS2812B-style single pixel on `GPIO48`)
  - Wi-Fi
  - SPIFFS
  - Dual-core task split (core 1 realtime, core 0 app/network)
- Optional future feature:
  - BLE (not enabled in current firmware)

## ESP32-S3 Built-in Capabilities (Not Enabled Yet)

- AI vector instructions (SIMD on Xtensa LX7):
  - Useful for DSP / quantized ML acceleration.
  - Present on ESP32-S3, but this firmware does not currently use dedicated AI kernels.
- Rich programmable GPIO/peripheral matrix:
  - Up to `45` programmable GPIOs on ESP32-S3 family variants (board breakout may expose fewer).
  - Peripheral options include `SPI`, LCD interface, camera interface, `I2C`, `I2S`, and PWM/LEDC routing.
  - Current firmware uses only a subset needed for motors, encoders, TOF, battery ADC, and UI/network.

## Motor + Driver

- Motor driver: `TB6612` dual H-bridge
- Motors: `GA12-N20` DC gear motors
- Gear ratio: `1:30`
- Encoder: `2-channel AB hall` (quadrature)
- Wheel diameter: `34mm`

## Power

- Battery pack: `2S LiPo`, `1200mAh` (cells in series)
- Battery monitor ADC pin: `GPIO3`
- Battery divider (configured):
  - `R_top = 56k`
  - `R_bottom = 18k`
- Battery thresholds:
  - Warning: `7.10V`
  - Critical: `6.90V`

## I2C Bus

- SDA: `GPIO8`
- SCL: `GPIO9`
- Clock: `400kHz`
- Bus is shared by:
  - PCF8574 I/O expander (`0x20`)
  - VL53L0X sensors (assigned addresses below)

## TOF / Wall Sensors (VL53L0X)

- Sensor count: `5`
- Address assignment:
  - Sensor 0 -> `0x30`
  - Sensor 1 -> `0x31`
  - Sensor 2 -> `0x32`
  - Sensor 3 -> `0x33`
  - Sensor 4 -> `0x34`
- XSHUT control is through PCF8574 pins:
  - Sensor 0 -> PCF P0
  - Sensor 1 -> PCF P1
  - Sensor 2 -> PCF P2
  - Sensor 3 -> PCF P3
  - Sensor 4 -> PCF P4

### Sensor layout interpretation in firmware

- Layout auto-detect:
  - V1: fallback path (typically 3 wall sensors used)
  - V2: when 4 sensors initialize (front composed from diagonals)
- V1 mapping used by control:
  - Left = sensor 0
  - Front = sensor 2
  - Right = sensor 4
- V2 mapping used by control:
  - Front-left = sensor 0
  - Left = sensor 1
  - Right = sensor 2
  - Front-right = sensor 3

## Motor Driver Wiring

### Left motor

- IN1: `GPIO5`
- IN2: `GPIO6`
- PWM: `GPIO4` (LEDC channel 0)
- Encoder A: `GPIO1`
- Encoder B: `GPIO2`
- Direction invert: `true`
- Encoder invert: `true`

### Right motor

- IN1: `GPIO10`
- IN2: `GPIO11`
- PWM: `GPIO7` (LEDC channel 1)
- Encoder A: `GPIO12`
- Encoder B: `GPIO13`
- Direction invert: `false`
- Encoder invert: `false`

### PWM setup

- Frequency: `20kHz`
- Resolution: `10-bit`

## LED

- Onboard RGB LED data pin (default): `GPIO48`
- Pixel count (default): `1`
- Source: `LedController.cpp` (`RGB_PIN`, `NUM_PIXELS` defaults)

## Buttons / Inputs

- BOOT button input pin: `GPIO0`
- Active level: LOW (`INPUT_PULLUP`)

## Network / Service Ports

- Floodfill web UI: `81`
- Floodfill WebSocket: `83`
- OTA upload web page: `82`
- Debug TCP console: `2323`

## Task/Core placement (runtime)

- Realtime core: `Core 1`
  - motor task
  - tof task
  - gptimer-triggered wakeup
- App core: `Core 0`
  - planner task
  - explorer task
  - user task
  - telemetry task
  - Wi-Fi/OTA task (from config)

## Notes

- All values above are from current `Config.cpp` and runtime initialization.
- If hardware wiring changes, update `Config.cpp` first, then this file.
- TOF quality and centering behavior also depend on per-sensor calibration values (`SENSOR_SCALE`, `SENSOR_OFFSET_MM`).
- Motor distance model (`LEFT_MM_PER_TICK`, `RIGHT_MM_PER_TICK`) should be tuned on real hardware; do not assume ideal values from wheel diameter alone.
- Planned extension (not enabled in current firmware): add a dedicated MCU path for yaw control/fusion integration.
