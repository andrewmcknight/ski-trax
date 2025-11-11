# Ski-Trax (Work in Progress)

Ski-Trax is an embedded tracking node built around the Heltec WiFi LoRa 32 V3. The goal is a rugged, skier-friendly device that can capture GNSS data, sense motion, render a live dashboard on a TFT screen, and share status over a LoRa mesh network. This repository currently contains early hardware bring-up sketches, wiring notes, and reference assets while the full application is still under active development.

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
| I2C SDA | 48 | Shared by MAX-M10S, BNO055, BMP390 |
| I2C SCL | 47 | Shared by MAX-M10S, BNO055, BMP390 |
| TFT SCK | 3 | Hardware SPI via EyeSPI ribbon (shares Heltec VEXT control pad) |
| TFT MOSI | 4 | Hardware SPI via EyeSPI ribbon |
| TFT CS | 7 | |
| TFT D/C | 5 | |
| TFT RST | 6 | |
| TFT Backlight PWM | 2 | Driven with LEDC PWM |
| GNSS Power Enable (VGNSS_CTRL) | 3 | Drives Heltec VEXT rail for MAX-M10S power (may be revisited) |
| Button – Dim | 40 | Input with pull-up |
| Button – Buzzer | 41 | Input with pull-up |
| Button – Brighten | 42 | Input with pull-up |
| Piezo Buzzer | 39 | LEDC PWM output |

> **Note:** Additional peripherals (LoRa radio, onboard OLED, etc.) are handled by the Heltec board but not yet integrated into the main application sketch.

## Project Status
- ✅ Peripheral test sketches for GNSS, TFT, IMU, pressure sensor, and buzzer
- 🚧 System integration and user-facing application logic
- 🚧 Enclosure, power budgeting, and field testing

Expect rapid iteration: wiring, firmware structure, and documentation will change as we move from bench testing to field-ready prototypes. Contributions should treat this README as a living document.
