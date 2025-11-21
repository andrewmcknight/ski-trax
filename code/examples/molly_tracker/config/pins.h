// SPDX-License-Identifier: MIT
// Centralized pin definitions for the Heltec WiFi LoRa 32 V3 hardware stack.

#pragma once

#include <Arduino.h>

namespace Pins {
constexpr uint8_t I2C_SDA = 47;
constexpr uint8_t I2C_SCL = 48;

constexpr uint8_t TFT_CS = 7;
constexpr uint8_t TFT_DC = 5;
constexpr uint8_t TFT_RST = 6;
constexpr uint8_t TFT_SCK = 3;
constexpr uint8_t TFT_MOSI = 4;
constexpr int8_t TFT_MISO = -1;

constexpr uint8_t TFT_BACKLIGHT = 2;

constexpr uint8_t BUTTON_LEFT = 40;
constexpr uint8_t BUTTON_CENTER = 41;
constexpr uint8_t BUTTON_RIGHT = 42;

constexpr uint8_t BUZZER = 39;

constexpr uint8_t VEXT_CTRL = 36;      // Active-low enable for sensors/display rail

constexpr uint8_t BATTERY_ADC = 1;     // Battery sense via GPIO1 (ADC1_CH0)
}  // namespace Pins
