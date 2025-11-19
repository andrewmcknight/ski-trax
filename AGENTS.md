# Project Mission
Multi-device GPS tracker with LoRa mesh networking. Clean, interactive UI displayed on TFT screen. Built for skiers in rugged outdoor environments.

## Hardware Stack

### Heltec WiFi LoRa 32 V3 (Main Board)
- **MCU**: ESP32-S3FN8 (dual-core 240MHz, 8MB Flash, 512KB SRAM)
- **Radio**: SX1262 LoRa + WiFi/BT 5.0
- **Display**: Built-in 0.96" 128x96 OLED (unused)
- **Power**: USB-C, SH1.25-2 battery, charge management
- **Notable Pins**: GPIO1 (battery ADC), GPIO36 (Vext control), see more at `docs/ht_wifi_lora_32_v3_pin_table.json`

### External Modules

- SparkFun MAX-M10S GNSS
- Adafruit 2.2" TFT (ILI9341 driver, 240×320 RGB)
- Adafruit BNO-055 IMU
- Adafruit BMP-390
- Buttons + Buzzer

> Note: we now connect to the Adafruit TFT via the Adafruit EYESPI breakout board.

## Project Structure
```
code/
├── main/                   # Target application
    ├── src                 # Architecture tbd
    │   └── config, assets, etc.
    └── platformio.ini
├── ...
└── io_tests/               # Individual component tests
    ├── sparkfun_max_m10s_test/
    ├── tft_fast/           # HW SPI demo
    ├── bmp390_test/
    └── ...
docs/        # All datasheets/guides
diagrams/    # Pinouts and schematics
3d-models/   # Enclosure CAD files
pcb/         # PCB design files
```

*Note: component selection, pin assignments, project structure, etc. subject to change; example code may be outdated*

Consult the documentation when uncertain. Be aware that datasheets may pertain to individual components, not the breakout-style modules we are using.

Do not write code for the Heltec Wireless Tracker! Only write code for the Heltec WiFi LoRa 32 v3.