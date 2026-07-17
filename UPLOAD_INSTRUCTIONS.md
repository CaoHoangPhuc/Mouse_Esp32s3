# Upload Instructions

## Overview

This project provides two upload methods:

1. **Arduino IDE Upload (USB)** - Recommended for initial flash
2. **Browser Upload (Port 82)** - Recommended for later OTA updates

## Port 82 - Browser Upload

### Features

- Chunked HTTP upload with retry handling
- Streaming firmware transfer
- Adaptive chunk-size fallback
- Fixed retry backoff/pacing
- LED status feedback:
  - **Blue** = Upload/OTA in progress
  - **Off** = Upload finished successfully
  - **Red** = Upload/OTA error or abort

### Usage

1. Open browser
2. Navigate to: `http://<ESP32_IP>:82/`
3. Select firmware file
4. Upload

### Upload Flow

1. Open the upload page on port `82`
2. The page starts the upload session with `/upload/start`
3. Firmware data is streamed in chunks through `/upload/chunk`
4. The transfer is finalized with `/upload/finish`

### Troubleshooting

If browser upload reports network error:
- Check Serial Monitor for `[WEB OTA] Start`, `Success`, and reboot scheduling
- Capture serial logs to isolate HTTP-response issues from flash-write issues
- If upload still aborts:
  - Check if the last serial line is `[WEB OTA] Received ... bytes`, `[WEB OTA] Aborted`, or a Wi-Fi reconnect event
  - Use that to separate transport problems from flash finalization problems

## Port 80 - Control Page

Port `80` is the runtime control/status page, not the main firmware upload surface.

Current use:

- shows battery/runtime status
- links to the floodfill page on port `81`
- links to the browser upload page on port `82`
- provides command guidance and telnet reconnect helpers

For firmware upload, use the dedicated browser uploader on port `82`.

## Build Configuration

### Arduino IDE Build / Upload

**File Location:** `Mouse_esp32s3.ino`

**Build Settings:**
- `USB CDC On Boot`: Enabled
- `Partition Scheme`: Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
- `Arduino Runs On`: Core 1
- `COM Port`: Select correct port before upload

**Upload Steps:**
1. Open `Mouse_esp32s3.ino` in Arduino IDE
2. Select ESP32-S3 board in Tools
3. Set Partition Scheme to match project requirements
4. Select correct COM port
5. Click Verify first
6. Click Upload

## Firmware Files

Use this compiled firmware file for the browser uploader:

- `Mouse_esp32s3.ino.bin`

Do not use these for the browser uploader:

- `Mouse_esp32s3.ino.merged.bin`
- bootloader binaries
- partition table binaries

## External Libraries

- `PCF8574` - I2C I/O expander
- `VL53L0X` - TOF distance sensors
- `Adafruit_NeoPixel` - LED control

## Additional Resources

- See `README.md` for full documentation
- See `SKILL.md` for development workflow guidelines
- See `TODO.md` for known issues and next steps
