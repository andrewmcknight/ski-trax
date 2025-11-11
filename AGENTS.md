# Project Mission
Multi-device GPS tracker with LoRa mesh networking. Clean, interactive UI displayed on TFT screen. Built for skiers in rugged outdoor environments.

## Hardware Stack

### Heltec WiFi LoRa 32 V3 (Main Board)
- **MCU**: ESP32-S3FN8 (dual-core 240MHz, 8MB Flash, 512KB SRAM)
- **Radio**: SX1262 LoRa + WiFi/BT 5.0
- **Display**: Built-in 0.96" 128x96 OLED (unused)
- **Power**: USB-C, SH1.25-2 battery, charge management
- **Critical Pins**: GPIO1 (battery ADC), GPIO3 (Vext control), GPIO17 (internal GPS UART)

### External Modules
| Component | Interface | Address/Notes |
|-----------|-----------|---------------|
| SparkFun MAX-M10S GNSS | I2C | 0x42 (fixed), needs SMA antenna |
| Adafruit 2.2" TFT (ILI9341 driver) | SPI | 240×320 RGB, use HW SPI! |
| Adafruit BNO-055 IMU | I2C | 0x28/0x29, 9-DOF fusion |
| Adafruit BMP-390 | I2C | 0x76/0x77, pressure/temp |
| Buttons + Buzzer | GPIO | TBD assignments |

## Project Structure
```
code/
├── main/                   # ← Target application
    ├── config, etc.        # Architecture tbd
    └── ...
├── ...
└── io_tests/               # Individual component tests
    ├── sparkfun_max_m10s_test/
    ├── tft_fast/           # HW SPI demo
    ├── bmp390_test/
    ├── ...
    └── imu_10dof_test/
docs/        # All datasheets/guides (check here first!)
diagrams/    # Pinouts and schematics
3d-models/   # Board CAD files
pcb/         # Custom PCB designs
```

*Note: some example code may be outdated*

Consult the documentation when uncertain. Be aware that datasheets may pertain to individual components, not the breakout-style modules we are using.