# SECURITY_FLASHING.md

## What we found (from scan)

- No flash-encryption or eFuse burn logic exists in this project source (`.ino/.cpp/.h`).
- No local build files in this repo (`sdkconfig`, `build_opt.h`, `platformio.ini`) were found that enable secure boot / flash encryption.
- Your `esp32s3.txt` shows:
  - `SPI_BOOT_CRYPT_CNT = 0b111` (flash encryption enabled)
  - `SECURE_BOOT_EN = False`

## Root-cause conclusion

From project scan alone, there is no evidence that this repository burned eFuses directly.

Most likely trigger happened outside this repo, such as:
- prior provisioning/security script on this board,
- a different project/tooling flow (ESP-IDF security flow),
- board delivered pre-configured with encryption enabled.

Because security fuses are one-way, once `SPI_BOOT_CRYPT_CNT` is enabled, normal plain Arduino images will not boot.

## Safe Arduino template (development mode)

Use this profile for normal development boards (non-secure):

- Board: ESP32-S3 Dev Module
- USB CDC On Boot: Enabled
- Flash Size: 4MB (or your real module size)
- Partition Scheme: Minimal SPIFFS with OTA (project default style)
- Flash Mode: `DIO`
- Flash Frequency: `80MHz`
- Upload Speed: `115200` (recovery-safe) or `460800` (normal)
- Erase Flash: `Only Sketch` (avoid full erase by default)

## Safe flashing SOP (recommended)

1. Before using a new board, run:
   - `espefuse --port COMx summary`
2. Confirm security state:
   - `SPI_BOOT_CRYPT_CNT` not enabled for plain-Arduino workflow.
3. Avoid:
   - `erase_flash --force`
   - `write_flash --force --encrypt`
   - any `espefuse burn_*` command unless intentional.
4. Before risky operations, backup full flash:
   - `read_flash 0x0 0x400000 backup.bin`
5. Keep one separate board for security experiments.

## Pre-flight check commands

```powershell
# 1) Check security fuses
C:\Users\donot\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.1.0\espefuse.py --port COM4 summary

# 2) Validate generated image on host
C:\Users\donot\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.1.0\esptool.exe --chip esp32s3 --port COM4 image-info C:\path\to\firmware.bin
```
