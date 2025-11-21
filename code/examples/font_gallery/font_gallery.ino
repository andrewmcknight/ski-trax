/*
 * Adafruit GFX Font Gallery
 * -------------------------
 * Rotates through several Adafruit_GFX fonts on the Adafruit 2.2" ILI9341 TFT
 * so you can quickly pick a style for the main UI.
 *
 * Hardware: Heltec WiFi LoRa 32 v3 + Adafruit EyeSPI ILI9341 breakout
 * Wiring:
 *   - EyeSPI (TFT) -> GPIO3/4/5/6/7 (SPI on HSPI, MISO unused)
 *   - Vext enable  -> GPIO36 (pull LOW to power the TFT)
 */
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansOblique12pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
// Pin definitions for Heltec WiFi LoRa 32 v3 + EyeSPI TFT breakout
constexpr uint8_t TFT_CS = 7;
constexpr uint8_t TFT_RST = 6;
constexpr uint8_t TFT_DC = 5;
constexpr uint8_t TFT_MOSI = 4;
constexpr uint8_t TFT_SCK = 3;
constexpr int8_t  TFT_MISO = -1;  // Display is write-only
constexpr uint8_t VEXT_CTRL_PIN = 36;  // Active-low enable for Vext rail
constexpr uint32_t TFT_SPI_FREQUENCY = 40000000UL;  // 40 MHz
constexpr uint16_t SCREEN_WIDTH = 320;
constexpr uint16_t SCREEN_HEIGHT = 240;
constexpr int16_t LEFT_MARGIN = 10;
constexpr int16_t TOP_MARGIN = 12;
constexpr int16_t LINE_PADDING = 6;
constexpr uint32_t PAGE_HOLD_MS = 4000;
SPIClass tftSpi(HSPI);
Adafruit_ILI9341 tft(&tftSpi, TFT_DC, TFT_CS, TFT_RST);
struct FontEntry {
  const char* label;
  const GFXfont* font;
};
static const FontEntry kFonts[] = {
  {"System 5x7 (default)", nullptr},
  {"FreeSans 9pt", &FreeSans9pt7b},
  {"FreeSans Bold 12pt", &FreeSansBold12pt7b},
  {"FreeSans Oblique 12pt", &FreeSansOblique12pt7b},
  {"FreeSerif 9pt", &FreeSerif9pt7b},
  {"FreeSerif Bold 12pt", &FreeSerifBold12pt7b},
  {"FreeMono Bold 12pt", &FreeMonoBold12pt7b},
};
constexpr size_t kFontCount = sizeof(kFonts) / sizeof(kFonts[0]);
static const char* kSampleText = "SkiTrax 123 ABC xyz";
static void enableVext() {
  pinMode(VEXT_CTRL_PIN, OUTPUT);
  digitalWrite(VEXT_CTRL_PIN, LOW);  // LOW = enable on Heltec WiFi LoRa 32 v3
  delay(20);
}
static void initDisplay() {
  tftSpi.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin(TFT_SPI_FREQUENCY);
  tft.setRotation(1);  // Landscape
  tft.fillScreen(ILI9341_BLACK);
}
// Returns the cursor Y position needed to place the top of `text` at yTop using the current font.
static int16_t baselineForTop(int16_t yTop, const char* text) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  return yTop - y1;
}
// Measures the height of `text` with the current font and size.
static uint16_t measureTextHeight(const char* text) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  return h;
}
static size_t renderFontPage(size_t startIndex) {
  tft.fillScreen(ILI9341_BLACK);
  tft.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, ILI9341_DARKGREY);
  // Header
  tft.setFont(nullptr);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  const char* header = "Adafruit_GFX Font Gallery";
  tft.setCursor(LEFT_MARGIN, baselineForTop(TOP_MARGIN, header));
  tft.print(header);
  int16_t y = TOP_MARGIN + measureTextHeight(header) + 4;
  // Render as many fonts as will fit on the screen from startIndex.
  for (size_t i = startIndex; i < kFontCount; ++i) {
    // Measure label height using default font.
    tft.setFont(nullptr);
    tft.setTextSize(1);
    uint16_t labelHeight = measureTextHeight(kFonts[i].label);
    // Measure sample height using target font.
    tft.setFont(kFonts[i].font);
    tft.setTextSize(1);
    uint16_t sampleHeight = measureTextHeight(kSampleText);
    // Check if both lines fit on this page.
    if (y + labelHeight + sampleHeight + LINE_PADDING > SCREEN_HEIGHT - TOP_MARGIN) {
      return i;  // Rendered up to i - 1
    }
    // Label
    tft.setFont(nullptr);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
    tft.setCursor(LEFT_MARGIN, baselineForTop(y, kFonts[i].label));
    tft.print(kFonts[i].label);
    y += labelHeight + 2;
    // Sample
    tft.setFont(kFonts[i].font);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(LEFT_MARGIN, baselineForTop(y, kSampleText));
    tft.print(kSampleText);
    y += sampleHeight + LINE_PADDING;
  }
  return kFontCount;  // Showed the final page
}
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\nFont gallery demo starting...");
  enableVext();
  initDisplay();
  Serial.println("TFT ready. Cycling through font pages every few seconds.");
}
void loop() {
  static size_t nextStartIndex = 0;
  static unsigned long lastPageTimestamp = 0;
  static bool firstPageDrawn = false;
  if (!firstPageDrawn || millis() - lastPageTimestamp >= PAGE_HOLD_MS) {
    nextStartIndex = renderFontPage(nextStartIndex);
    if (nextStartIndex >= kFontCount) {
      nextStartIndex = 0;  // Loop back to the first page
    }
    lastPageTimestamp = millis();
    firstPageDrawn = true;
  }
}