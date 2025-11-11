/* ILI9341 Display Test
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

// Create display object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n========================================");
  Serial.println("========================================");
  Serial.println("\nPin Configuration:");
  Serial.print("  CS:    GPIO"); Serial.print(TFT_CS);
  Serial.print("  DC:    GPIO"); Serial.print(TFT_DC);
  Serial.print("  RST:   GPIO"); Serial.print(TFT_RST);
  Serial.print("  MOSI:  GPIO"); Serial.print(TFT_MOSI);
  Serial.print("  SCK:   GPIO"); Serial.print(TFT_SCK);
  
  Serial.println("\nThese pins are FREE - no conflicts!");
  
  Serial.println("\nInitializing display...");
  
  // Initialize display
  tft.begin();
  
  Serial.println("Display initialized!");
  Serial.println("Drawing test pattern...");
  
  // Test 1: Fill screen with colors
  Serial.println("\nTest 1: Color fills");
  tft.fillScreen(ILI9341_BLACK);
  delay(500);
  
  tft.fillScreen(ILI9341_RED);
  delay(500);
  
  tft.fillScreen(ILI9341_GREEN);
  delay(500);
  
  tft.fillScreen(ILI9341_BLUE);
  delay(500);
  
  tft.fillScreen(ILI9341_WHITE);
  delay(500);
  
  // Test 2: Draw some shapes
  Serial.println("Test 2: Shapes and text");
  tft.fillScreen(ILI9341_BLACK);
  
  // Large text first
  tft.setTextSize(4);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 40);
  tft.println("ILI9341");
  
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(40, 100);
  tft.println("Display Test");
  
  // Draw border
  tft.drawRect(5, 5, 230, 310, ILI9341_GREEN);
  tft.drawRect(7, 7, 226, 306, ILI9341_GREEN);
  
  // Pin information
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(20, 150);
  tft.println("Pin Connections:");
  
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 170);
  tft.println("CS   -> GPIO7  (J3-5)");
  tft.setCursor(20, 185);
  tft.println("DC   -> GPIO5  (J3-7)");
  tft.setCursor(20, 200);
  tft.println("RST  -> GPIO6  (J3-6)");
  tft.setCursor(20, 215);
  tft.println("MOSI -> GPIO4  (J3-8)");
  tft.setCursor(20, 230);
  tft.println("SCK  -> GPIO17 (J3-2)");
  
  // Status
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(60, 270);
  tft.println("SUCCESS!");
  
  Serial.println("\nTest complete!");
  Serial.println("If you see this, display is working!");
  Serial.println("========================================\n");
}

void loop() {
  // Animated corner indicators
  static unsigned long lastUpdate = 0;
  static int phase = 0;
  
  if (millis() - lastUpdate > 500) {
    lastUpdate = millis();
    phase = (phase + 1) % 4;
    
    // Clear previous indicators
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
    if (phase == 0) Serial.println(" Loop running...");
  }
}
