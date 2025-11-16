/*
 * ILI9341 Display Test for EyeSPI Breakout (Heltec WiFi LoRa 32 V3)
 * -----------------------------------------------------------------
 * Core TFT pin mapping (EyeSPI -> Heltec):
 *   - VCC  -> 3V3
 *   - GND  -> GND
 *   - SCK  -> GPIO3
 *   - MOSI -> GPIO4
 *   - CS   -> GPIO7
 *   - D/C  -> GPIO5
 *   - RST  -> GPIO6
 *   - MISO -> not connected
 */

// **Working as of 11/11/25**

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// EyeSPI -> Heltec WiFi LoRa 32 V3 pin assignments
#define TFT_CS   7
#define TFT_RST  6
#define TFT_DC   5
#define TFT_MOSI 4
#define TFT_SCK  3
#define TFT_MISO -1  // Not used (write-only TFT)

#define VEXT_CTRL 36

// Safe SPI clock (ILI9341 supports up to 40 MHz)
#define TFT_SPI_FREQUENCY  20000000UL

SPIClass tftSpi(HSPI);  // Dedicated HSPI bus so we don't disturb the SX1262 SPI wiring
Adafruit_ILI9341 tft(&tftSpi, TFT_DC, TFT_CS, TFT_RST);

static void configureSpiPins() {
  tftSpi.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
}

static void runScreenTest() {
  tft.fillScreen(ILI9341_BLACK);
  delay(250);
  tft.fillScreen(ILI9341_RED);
  delay(250);
  tft.fillScreen(ILI9341_GREEN);
  delay(250);
  tft.fillScreen(ILI9341_BLUE);
  delay(250);
  tft.fillScreen(ILI9341_BLACK);

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

  tft.drawLine(0, 0, tft.width(), tft.height(), ILI9341_MAGENTA);
  tft.drawLine(0, tft.height(), tft.width(), 0, ILI9341_MAGENTA);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // === ENABLE VEXT POWER RAIL ===
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);   // Turn on Vext 3.3V output
  delay(50); // give sensors time to power up
  // ===============================

  Serial.println("\n=== EyeSPI ILI9341 Test (Core TFT Pins) ===");
  Serial.printf("CS=%d  DC=%d  RST=%d  MOSI=%d  SCK=%d\n",
                TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK);

  configureSpiPins();

  Serial.println("Initialising display...");
  tft.begin(TFT_SPI_FREQUENCY);
  tft.setRotation(1);
  runScreenTest();
  Serial.println("Display initialised!");
}

void loop() {
  static uint32_t lastBlink = 0;
  static bool on = false;

  if (millis() - lastBlink > 500) {
    lastBlink = millis();
    on = !on;
    tft.drawPixel(4, 4, on ? ILI9341_GREEN : ILI9341_BLACK);
    Serial.print(on ? "*" : ".");
  }
}
