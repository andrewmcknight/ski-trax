# Ski-Trax

Ski-Trax is an embedded tracking node built around the Heltec WiFi LoRa 32 V3. It's a work in progress - the goal is a rugged device that skiers can use to find their friends/family on the slopes.

## Hardware Stack
- **Main MCU:** Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
- **GNSS:** SparkFun MAX-M10S breakout (I2C)
- **Display:** Adafruit 2.2" TFT (ILI9341 driver) connected through the Adafruit EYESPI breakout (SPI)
- **IMU:** Adafruit BNO055 9-DOF sensor (I2C)
- **Barometer:** Adafruit BMP390 (I2C)
- **Inputs/Outputs:** Three external buttons, piezo buzzer, TFT backlight PWM

## Current Pin Assignments
| Function | Heltec GPIO | Notes |
|----------|-------------|-------|
| I2C SDA | 47 | Shared by MAX-M10S, BNO055, BMP390 |
| I2C SCL | 48 | Shared by MAX-M10S, BNO055, BMP390 |
| TFT SCK | 3 | Hardware SPI via EyeSPI ribbon |
| TFT MOSI | 4 | Hardware SPI via EyeSPI ribbon |
| TFT CS | 7 | |
| TFT D/C | 5 | |
| TFT RST | 6 | |
| TFT Backlight PWM | 2 | Driven with LEDC PWM |
| Button – Dim | 40 | Input with pull-up |
| Button – Buzzer | 41 | Input with pull-up |
| Button – Brighten | 42 | Input with pull-up |
| Piezo Buzzer | 39 | LEDC PWM output |

> **Note:** LoRa radio is built into the Heltec board!

## Project Status
- I/O test sketches complete for full hardware stack
- PCB design finalized
- In progress: enclosure design, user-facing app, and field testing

## Main Firmware Overview
- `code/main/src/main.cpp` now implements the end-to-end Ski-Trax experience: splash/pairing UI, nickname entry, host/join workflows, session beacons, and the tracking carousel.
- Pairing uses LoRa beacons + join-request/accept handshakes so guests can browse nearby sessions before entering tracking mode.
- The tracking screen keeps the top status bar (GPS time, battery icon, sensor indicators) visible while the layered compass renderer shows peer direction, distance, and altitude deltas using imperial units.

## Compass Demo Assets
- `code/examples/compass_heading` now renders a static 176×176 compass background (`compass_bg.h`) plus transparent 170×170 needle overlays (`needle_XXX.h`).
- Transparent pixels are encoded with RGB565 magenta (`0xF81F`) so the background shows through; only the rotating needle region is redrawn each update to minimize SPI traffic and flicker.
