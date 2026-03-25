# Mouse_esp32s3

ESP32-S3 micromouse project with:
- dual DC motor control with encoder-based PID
- multi-VL53L0X distance sensing
- Wi-Fi OTA and web serial logging
- flood-fill maze exploration UI

## Files
- `Mouse_esp32s_v0.1.ino`: main Arduino sketch
- `DcMotor.*`: motor driver and encoder/PID control
- `MultiVL53L0X.*`: TOF sensor array management
- `WiFiOtaWebSerial.*`: Wi-Fi, OTA, and web log interface
- `FloodFillExplorer.*`: maze explorer and web visualization

## Notes
- Wi-Fi credentials are currently hardcoded in the sketch.
- This repo lives in the `Mouse_esp32s3` folder.
