/*
 * Compass Heading Demo
 * --------------------
 * Renders a 176x176 compass card on the Adafruit 2.2" ILI9341 TFT with a
 * transparent 170x170 needle overlay selected from 36 bitmaps. The heading
 * is provided by the Adafruit BNO055 IMU's geomagnetic sensor.
 *
 * Hardware: Heltec WiFi LoRa 32 v3 + EyeSPI TFT breakout + Adafruit BNO055
 * Wiring:
 *   - I2C SDA/SCL -> GPIO47/GPIO48
 *   - EyeSPI TFT  -> GPIO3/4/5/6/7 (see pin defines below)
 *   - Vext enable -> GPIO36 (pull LOW to power sensors + TFT)
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <pgmspace.h>

// ---------------------------- Pin Definitions ----------------------------
constexpr uint8_t I2C_SDA_PIN = 47;
constexpr uint8_t I2C_SCL_PIN = 48;
constexpr uint8_t VEXT_CTRL_PIN = 36;   // Active-low enable for the Vext rail

constexpr uint8_t TFT_CS = 7;
constexpr uint8_t TFT_RST = 6;
constexpr uint8_t TFT_DC = 5;
constexpr uint8_t TFT_MOSI = 4;
constexpr uint8_t TFT_SCK = 3;
constexpr int8_t  TFT_MISO = -1;        // Display is write-only
constexpr uint32_t TFT_SPI_FREQUENCY = 40000000UL;  // 40 MHz hardware SPI

constexpr uint16_t COMPASS_BG_SIZE = 176;
constexpr uint16_t NEEDLE_SIZE = 170;
constexpr uint16_t NEEDLE_OFFSET_PX = (COMPASS_BG_SIZE - NEEDLE_SIZE) / 2;
constexpr uint16_t NEEDLE_TRANSPARENT_KEY = 0xF81F;
constexpr uint16_t STATUS_PANEL_HEIGHT = 56;
constexpr uint32_t COMPASS_REFRESH_INTERVAL_MS = 125;
constexpr float HEADING_SMOOTHING = 0.18f;  // 0=raw, 1=very sluggish

// ---------------------------- Assets -------------------------------------
#include "assets/compass_bg.h"
#include "assets/needle_000.h"
#include "assets/needle_010.h"
#include "assets/needle_020.h"
#include "assets/needle_030.h"
#include "assets/needle_040.h"
#include "assets/needle_050.h"
#include "assets/needle_060.h"
#include "assets/needle_070.h"
#include "assets/needle_080.h"
#include "assets/needle_090.h"
#include "assets/needle_100.h"
#include "assets/needle_110.h"
#include "assets/needle_120.h"
#include "assets/needle_130.h"
#include "assets/needle_140.h"
#include "assets/needle_150.h"
#include "assets/needle_160.h"
#include "assets/needle_170.h"
#include "assets/needle_180.h"
#include "assets/needle_190.h"
#include "assets/needle_200.h"
#include "assets/needle_210.h"
#include "assets/needle_220.h"
#include "assets/needle_230.h"
#include "assets/needle_240.h"
#include "assets/needle_250.h"
#include "assets/needle_260.h"
#include "assets/needle_270.h"
#include "assets/needle_280.h"
#include "assets/needle_290.h"
#include "assets/needle_300.h"
#include "assets/needle_310.h"
#include "assets/needle_320.h"
#include "assets/needle_330.h"
#include "assets/needle_340.h"
#include "assets/needle_350.h"

struct NeedleFrame {
  const uint16_t* bitmap;
  uint16_t bearingDeg;
};

static const NeedleFrame kNeedleFrames[] = {
  {needle_000,   0},
  {needle_010,  10},
  {needle_020,  20},
  {needle_030,  30},
  {needle_040,  40},
  {needle_050,  50},
  {needle_060,  60},
  {needle_070,  70},
  {needle_080,  80},
  {needle_090,  90},
  {needle_100, 100},
  {needle_110, 110},
  {needle_120, 120},
  {needle_130, 130},
  {needle_140, 140},
  {needle_150, 150},
  {needle_160, 160},
  {needle_170, 170},
  {needle_180, 180},
  {needle_190, 190},
  {needle_200, 200},
  {needle_210, 210},
  {needle_220, 220},
  {needle_230, 230},
  {needle_240, 240},
  {needle_250, 250},
  {needle_260, 260},
  {needle_270, 270},
  {needle_280, 280},
  {needle_290, 290},
  {needle_300, 300},
  {needle_310, 310},
  {needle_320, 320},
  {needle_330, 330},
  {needle_340, 340},
  {needle_350, 350},
};

constexpr size_t kFrameCount = sizeof(kNeedleFrames) / sizeof(kNeedleFrames[0]);

// -------------------------- Peripherals ----------------------------------
SPIClass tftSpi(HSPI);               // Dedicated HSPI bus for the TFT
Adafruit_ILI9341 tft(&tftSpi, TFT_DC, TFT_CS, TFT_RST);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

bool bnoReady = false;
float filteredHeadingDeg = NAN;
unsigned long lastCompassUpdate = 0;
size_t displayedNeedleIndex = kFrameCount;  // kFrameCount == invalid
bool compassBackgroundDrawn = false;
int16_t compassOriginX = 0;
int16_t compassOriginY = 0;
int16_t needleOriginX = 0;
int16_t needleOriginY = 0;

// -------------------------- Helper Functions ------------------------------
static void enablePeripheralPower() {
  pinMode(VEXT_CTRL_PIN, OUTPUT);
  digitalWrite(VEXT_CTRL_PIN, LOW);  // LOW = enable on Heltec WiFi LoRa 32 v3
  delay(20);
}

static void initDisplay() {
  tftSpi.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin(TFT_SPI_FREQUENCY);
  tft.setRotation(1);  // Landscape (320x240)
  tft.fillScreen(ILI9341_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height() - STATUS_PANEL_HEIGHT, ILI9341_DARKGREY);
  tft.drawFastHLine(0, tft.height() - STATUS_PANEL_HEIGHT, tft.width(), ILI9341_DARKGREY);
  drawCompassBackground();
}

static void initBno() {
  bnoReady = bno.begin();
  if (bnoReady) {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 ready");
  } else {
    Serial.println("ERROR: BNO055 not detected. Check wiring and I2C power.");
  }
}

static float normalizeHeading(float headingDeg) {
  if (isnan(headingDeg)) {
    return NAN;
  }
  while (headingDeg < 0.0f) {
    headingDeg += 360.0f;
  }
  while (headingDeg >= 360.0f) {
    headingDeg -= 360.0f;
  }
  return headingDeg;
}

static float applyHeadingFilter(float newHeadingDeg) {
  if (isnan(newHeadingDeg)) {
    return filteredHeadingDeg;
  }
  if (isnan(filteredHeadingDeg)) {
    filteredHeadingDeg = newHeadingDeg;
    return filteredHeadingDeg;
  }
  float diff = newHeadingDeg - filteredHeadingDeg;
  if (diff > 180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  filteredHeadingDeg += diff * HEADING_SMOOTHING;
  return normalizeHeading(filteredHeadingDeg);
}

static size_t frameIndexForHeading(float headingDeg) {
  if (isnan(headingDeg)) {
    return displayedNeedleIndex;
  }
  uint16_t bucket = static_cast<uint16_t>((headingDeg + 5.0f) / 10.0f) % kFrameCount;
  return bucket;
}

static void computeCompassLayout() {
  const int16_t availableHeight = tft.height() - STATUS_PANEL_HEIGHT;
  compassOriginX = (tft.width() - COMPASS_BG_SIZE) / 2;
  compassOriginY = (availableHeight - COMPASS_BG_SIZE) / 2;
  if (compassOriginX < 0) compassOriginX = 0;
  if (compassOriginY < 0) compassOriginY = 0;
  needleOriginX = compassOriginX + NEEDLE_OFFSET_PX;
  needleOriginY = compassOriginY + NEEDLE_OFFSET_PX;
}

static void drawCompassBackground() {
  if (!compassBackgroundDrawn) {
    computeCompassLayout();
    tft.drawRGBBitmap(compassOriginX, compassOriginY, compass_bg, COMPASS_BG_SIZE, COMPASS_BG_SIZE);
    tft.drawRect(compassOriginX - 2, compassOriginY - 2, COMPASS_BG_SIZE + 4, COMPASS_BG_SIZE + 4, ILI9341_WHITE);
    compassBackgroundDrawn = true;
  }
}

static void restorePreviousNeedle() {
  if (displayedNeedleIndex >= kFrameCount) {
    return;
  }
  // Repaint the compass background only where the last needle drew pixels.
  tft.startWrite();
  for (uint16_t row = 0; row < NEEDLE_SIZE; ++row) {
    for (uint16_t col = 0; col < NEEDLE_SIZE; ++col) {
      const uint32_t idx = static_cast<uint32_t>(row) * NEEDLE_SIZE + col;
      uint16_t pixel = pgm_read_word(&(kNeedleFrames[displayedNeedleIndex].bitmap[idx]));
      if (pixel != NEEDLE_TRANSPARENT_KEY) {
        const uint16_t bg = pgm_read_word(&(compass_bg[(row + NEEDLE_OFFSET_PX) * COMPASS_BG_SIZE + (col + NEEDLE_OFFSET_PX)]));
        tft.writePixel(needleOriginX + col, needleOriginY + row, bg);
      }
    }
  }
  tft.endWrite();
}

static void drawNeedleTransparent(size_t index) {
  if (index >= kFrameCount) {
    return;
  }
  // Overlay the needle bitmap while ignoring magenta key pixels.
  tft.startWrite();
  for (uint16_t row = 0; row < NEEDLE_SIZE; ++row) {
    for (uint16_t col = 0; col < NEEDLE_SIZE; ++col) {
      const uint32_t idx = static_cast<uint32_t>(row) * NEEDLE_SIZE + col;
      uint16_t pixel = pgm_read_word(&(kNeedleFrames[index].bitmap[idx]));
      if (pixel != NEEDLE_TRANSPARENT_KEY) {
        tft.writePixel(needleOriginX + col, needleOriginY + row, pixel);
      }
    }
  }
  tft.endWrite();
  displayedNeedleIndex = index;
}

static void drawStatusPanel(float rawHeading, float smoothHeading, uint8_t calSys, uint8_t calMag) {
  const int16_t panelY = tft.height() - STATUS_PANEL_HEIGHT;
  tft.fillRect(0, panelY, tft.width(), STATUS_PANEL_HEIGHT, ILI9341_BLACK);

  tft.setCursor(10, panelY + 6);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("Heading: ");
  if (isnan(smoothHeading)) {
    tft.print("---");
  } else {
    tft.print(smoothHeading, 1);
    tft.print(" deg");
  }

  tft.setTextSize(1);
  tft.setCursor(10, panelY + 32);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.print("Raw: ");
  if (isnan(rawHeading)) {
    tft.print("---");
  } else {
    tft.print(rawHeading, 1);
    tft.print(" deg");
  }

  tft.setCursor(150, panelY + 32);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.print("Cal SYS:");
  tft.print(calSys);
  tft.print(" MAG:");
  tft.print(calMag);

  tft.setCursor(10, panelY + 44);
  tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
  tft.print("Wave the board in a figure-eight to finish calibration.");
}

// ------------------------------ Arduino API -------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Compass Heading Demo ===");

  enablePeripheralPower();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initDisplay();
  initBno();
}

void loop() {
  const unsigned long now = millis();
  if (!bnoReady) {
    static unsigned long lastRetry = 0;
    if (now - lastRetry > 2000) {
      lastRetry = now;
      initBno();
    }
    delay(50);
    return;
  }

  if (now - lastCompassUpdate < COMPASS_REFRESH_INTERVAL_MS) {
    delay(5);
    return;
  }
  lastCompassUpdate = now;

  sensors_event_t orientationEvent;
  bno.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
  float rawHeading = normalizeHeading(orientationEvent.orientation.x);
  float smoothHeading = applyHeadingFilter(rawHeading);

  size_t newFrameIndex = frameIndexForHeading(smoothHeading);
  if (newFrameIndex != displayedNeedleIndex && newFrameIndex < kFrameCount) {
    restorePreviousNeedle();
    drawNeedleTransparent(newFrameIndex);
  }

  uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;
  bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
  drawStatusPanel(rawHeading, smoothHeading, calSys, calMag);

  static unsigned long lastSerialReport = 0;
  if (now - lastSerialReport > 1000) {
    lastSerialReport = now;
    Serial.print("Heading raw/smooth: ");
    Serial.print(rawHeading, 1);
    Serial.print(" deg / ");
    Serial.print(smoothHeading, 1);
    Serial.print(" deg  |  Frame ");
    Serial.print(displayedNeedleIndex < kFrameCount ? displayedNeedleIndex : 999);
    Serial.print("  |  Cal SYS:");
    Serial.print(calSys);
    Serial.print(" MAG:");
    Serial.println(calMag);
  }
}
