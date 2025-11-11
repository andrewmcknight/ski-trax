/*
 * ILI9341 Display Test for EyeSPI Breakout
 * ----------------------------------------
 * Minimal diagnostic sketch that exercises the Adafruit 2.2" ILI9341 TFT
 * when it is connected through the EyeSPI breakout.  The test uses the
 * GPIO assignments requested in the issue and configures hardware SPI on
 * the Heltec Wireless Tracker (ESP32-S3) accordingly.
 *
 * Connections (EyeSPI ribbon -> Wireless Tracker J3 header):
 *   - VCC  -> 3V3 (make sure VEXT is enabled on the Heltec board)
 *   - GND  -> GND
 *   - SCK  -> GPIO3  (EyeSPI SCK)
 *   - MOSI -> GPIO4
 *   - CS   -> GPIO7
 *   - D/C  -> GPIO5
 *   - RST  -> GPIO6
 *   - MISO -> not used
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// Pin definitions supplied in the request
#define TFT_CS   7
#define TFT_RST  6
#define TFT_DC   5
#define TFT_MOSI 4
#define TFT_SCK  3
#define TFT_MISO -1

// Drive the panel at a conservative SPI clock until wiring is confirmed
#define TFT_SPI_FREQUENCY  20000000UL  // 20 MHz is well within the ILI9341 spec

// ILI9341 object using the hardware SPI interface
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Helper to initialise the SPI peripheral with custom pins on ESP32-class MCUs
static void configureSpiPins() {
#if defined(ARDUINO_ARCH_ESP32)
  // Route FSPI signals to the EyeSPI pins provided in the request
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
#else
  SPI.begin();
#endif
}

// Draw a simple focus/colour test pattern once the display is initialised
static void runScreenTest() {
  // Fill the frame with a few colours to confirm that pixels are being driven
  tft.fillScreen(ILI9341_BLACK);
  delay(250);
  tft.fillScreen(ILI9341_RED);
  delay(250);
  tft.fillScreen(ILI9341_GREEN);
  delay(250);
  tft.fillScreen(ILI9341_BLUE);
  delay(250);
  tft.fillScreen(ILI9341_BLACK);

  // Draw reference geometry and text so orientation is obvious
  tft.drawRect(0, 0, tft.width(), tft.height(), ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(8, 8);
  tft.print("EyeSPI TFT Test");

  tft.setTextSize(1);
  tft.setCursor(8, 32);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.print("CS="), tft.print(TFT_CS);
  tft.print("  DC="), tft.print(TFT_DC);
  tft.print("  RST="), tft.print(TFT_RST);

  tft.setCursor(8, 48);
  tft.print("MOSI="), tft.print(TFT_MOSI);
  tft.print("  SCK="), tft.print(TFT_SCK);

  // Diagonal lines exercise both orientations of memory writes
  tft.drawLine(0, 0, tft.width(), tft.height(), ILI9341_MAGENTA);
  tft.drawLine(0, tft.height(), tft.width(), 0, ILI9341_MAGENTA);

  // Checkerboard region near the centre for gamma/colour sanity
  const int boxSize = 20;
  const int startX = (tft.width()  - (boxSize * 6)) / 2;
  const int startY = (tft.height() - (boxSize * 6)) / 2;
  for (int y = 0; y < 6; ++y) {
    for (int x = 0; x < 6; ++x) {
      uint16_t colour = ((x + y) & 0x01) ? ILI9341_WHITE : ILI9341_NAVY;
      tft.fillRect(startX + x * boxSize, startY + y * boxSize, boxSize, boxSize, colour);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("====================================");
  Serial.println("EyeSPI ILI9341 Bring-up Test");
  Serial.println("====================================");

  Serial.println("\nConfigured pins:");
  Serial.print("  CS   -> GPIO"); Serial.println(TFT_CS);
  Serial.print("  RST  -> GPIO"); Serial.println(TFT_RST);
  Serial.print("  DC   -> GPIO"); Serial.println(TFT_DC);
  Serial.print("  MOSI -> GPIO"); Serial.println(TFT_MOSI);
  Serial.print("  SCK  -> GPIO"); Serial.println(TFT_SCK);

  Serial.println("\nInitialising SPI bus...");
  configureSpiPins();

  Serial.println("Initialising ILI9341 controller...");
  tft.begin(TFT_SPI_FREQUENCY);
  tft.setRotation(1);  // Landscape view matches the ribbon orientation on the tracker

  Serial.println("Display ready! Running drawing test.");
  runScreenTest();

  Serial.println("\nIf the above pattern appears the EyeSPI wiring is correct.");
}

void loop() {
  // Blink a small pixel in the corner so we have a heartbeat while debugging
  static uint32_t lastToggle = 0;
  static bool pixelOn = false;

  if (millis() - lastToggle >= 500) {
    lastToggle = millis();
    pixelOn = !pixelOn;
    tft.drawPixel(4, 4, pixelOn ? ILI9341_GREEN : ILI9341_BLACK);
    Serial.print(pixelOn ? "[TFT]" : ".");
  }
}
