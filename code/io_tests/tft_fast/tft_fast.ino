/* ILI9341 Display Test - FAST VERSION
 * 
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// Pin definitions
#define TFT_CS 7
#define TFT_RST 6
#define TFT_DC 5
#define TFT_MOSI 4
#define TFT_SCK 3
#define TFT_MISO -1

// SPI Speed - ILI9341 can handle up to 40MHz, some can do 80MHz!
#define SPI_FREQUENCY  40000000  // 40 MHz - try 80000000 if this works well!

// Create display object using hardware SPI
// This constructor only needs CS, DC, and RST pins
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n========================================");
  Serial.println("ILI9341 FAST Test - Hardware SPI");
  Serial.println("========================================");
  Serial.println("\nPin Configuration:");
  Serial.print("  CS:    GPIO"); Serial.print(TFT_CS);   Serial.println(" (Header J3, Pin 5)");
  Serial.print("  DC:    GPIO"); Serial.print(TFT_DC);   Serial.println(" (Header J3, Pin 7)");
  Serial.print("  RST:   GPIO"); Serial.print(TFT_RST);  Serial.println(" (Header J3, Pin 6)");
  Serial.print("  MOSI:  GPIO"); Serial.print(TFT_MOSI); Serial.println(" (Header J3, Pin 8)");
  Serial.print("  SCK:   GPIO"); Serial.print(TFT_SCK);  Serial.println(" (Header J3, Pin 2)");
  
  Serial.print("\nUsing HARDWARE SPI at ");
  Serial.print(SPI_FREQUENCY / 1000000);
  Serial.println(" MHz!");
  
  // Initialize SPI with custom pins
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  
  Serial.println("\nInitializing display...");
  
  // Initialize display with fast SPI speed
  tft.begin(SPI_FREQUENCY);
  
  Serial.println("Display initialized!");
  
  // Test speed
  unsigned long startTime = millis();
  
  Serial.println("Drawing test pattern...");
  
  // Test 1: Fill screen with colors - should be INSTANT now!
  Serial.println("\nTest 1: Color fills (watch the speed!)");
  
  tft.fillScreen(ILI9341_BLACK);
  Serial.println("  Black - done");
  delay(300);
  
  tft.fillScreen(ILI9341_RED);
  Serial.println("  Red - done");
  delay(300);
  
  tft.fillScreen(ILI9341_GREEN);
  Serial.println("  Green - done");
  delay(300);
  
  tft.fillScreen(ILI9341_BLUE);
  Serial.println("  Blue - done");
  delay(300);
  
  tft.fillScreen(ILI9341_WHITE);
  Serial.println("  White - done");
  delay(300);
  
  unsigned long colorTime = millis() - startTime;
  
  // Test 2: Draw interface
  Serial.println("\nTest 2: Drawing interface");
  startTime = millis();
  
  tft.fillScreen(ILI9341_BLACK);
  
  // Large text
  tft.setTextSize(4);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 40);
  tft.println("ILI9341");
  
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(30, 100);
  tft.println("HARDWARE SPI");
  
  // Speed indicator
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(55, 140);
  tft.print(SPI_FREQUENCY / 1000000);
  tft.println(" MHz");
  
  // Draw border
  tft.drawRect(5, 5, 230, 310, ILI9341_GREEN);
  tft.drawRect(7, 7, 226, 306, ILI9341_GREEN);
  
  // Pin information
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(20, 190);
  tft.println("Pin Connections:");
  
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 210);
  tft.println("CS   -> GPIO7  (J3-5)");
  tft.setCursor(20, 225);
  tft.println("DC   -> GPIO5  (J3-7)");
  tft.setCursor(20, 240);
  tft.println("RST  -> GPIO6  (J3-6)");
  tft.setCursor(20, 255);
  tft.println("MOSI -> GPIO4  (J3-8)");
  tft.setCursor(20, 270);
  tft.println("SCK  -> GPIO17 (J3-2)");
  
  unsigned long drawTime = millis() - startTime;
  
  // Status
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(35, 295);
  tft.println("SUPER FAST!");
  
  // Performance stats
  Serial.println("\n========================================");
  Serial.println("PERFORMANCE RESULTS:");
  Serial.print("  Color fills: "); Serial.print(colorTime); Serial.println(" ms");
  Serial.print("  Draw interface: "); Serial.print(drawTime); Serial.println(" ms");
  Serial.println("  Compare to software SPI (typically 5000+ ms!)");
  Serial.println("========================================\n");
  
  Serial.println("Test complete!");
  Serial.println("Display should be SUPER responsive now!");
  Serial.println("========================================\n");
}

void loop() {
  // Animated corner indicators - even faster now!
  static unsigned long lastUpdate = 0;
  static int phase = 0;
  
  if (millis() - lastUpdate > 200) {  // Faster animation
    lastUpdate = millis();
    phase = (phase + 1) % 4;
    
    // Clear all corners
    tft.fillCircle(15, 15, 5, ILI9341_BLACK);
    tft.fillCircle(225, 15, 5, ILI9341_BLACK);
    tft.fillCircle(15, 305, 5, ILI9341_BLACK);
    tft.fillCircle(225, 305, 5, ILI9341_BLACK);
    
    // Draw new indicator
    switch(phase) {
      case 0: tft.fillCircle(15, 15, 5, ILI9341_GREEN); break;
      case 1: tft.fillCircle(225, 15, 5, ILI9341_GREEN); break;
      case 2: tft.fillCircle(225, 305, 5, ILI9341_GREEN); break;
      case 3: tft.fillCircle(15, 305, 5, ILI9341_GREEN); break;
    }
    
    Serial.print(".");
    if (phase == 0) Serial.println(" Blazing fast loop!");
  }
}
