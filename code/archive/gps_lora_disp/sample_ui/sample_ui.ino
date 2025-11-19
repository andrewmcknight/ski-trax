/* Party Tracker UI - Clean Modern Design
 * 
 * Hardware Connections:
 * ILI9341 TFT Display to Wireless Tracker:
 * - VCC   -> 3V3
 * - GND   -> GND
 * - CS    -> GPIO7
 * - RESET -> GPIO6
 * - DC    -> GPIO5
 * - MOSI  -> GPIO4
 * - SCK   -> GPIO17
 * - LED   -> 3V3
 */

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// Pin definitions
#define TFT_CS      7
#define TFT_DC      5
#define TFT_RST     6
#define TFT_MOSI    4
#define TFT_SCK     17
#define TFT_MISO    -1

#define SPI_FREQUENCY  40000000

// Color scheme - clean and modern
#define COLOR_BG        0x2945  // Dark blue-gray
#define COLOR_PANEL     0x4A49  // Light gray panel
#define COLOR_TEXT      0xFFFF  // White
#define COLOR_TEXT_DIM  0x8410  // Gray
#define COLOR_ACCENT    0xFDA0  // Salmon/pink
#define COLOR_GREEN     0x07E0
#define COLOR_YELLOW    0xFFE0
#define COLOR_ORANGE    0xFD20
#define COLOR_RED       0xF800

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Party member structure
struct PartyMember {
  const char* name;
  int relativeAlt;      // feet, positive = above you
  int distance;         // feet
  float bearing;        // degrees, 0=N, 90=E, 180=S, 270=W
  int lastUpdate;       // seconds ago
};

// Sample data
PartyMember members[2] = {
  {"JAKE", 50, 150, 315, 3},      // NW, above you
  {"JEN", -30, 200, 85, 8}        // E, below you
};

int yourAltitude = 12650;
int batteryPercent = 87;

// Helper functions
void drawStatusBar() {
  // Time
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(10, 8);
  tft.print("3:45 PM");
  
  // Battery
  tft.setCursor(175, 8);
  tft.print(batteryPercent);
  tft.print("%");
  
  // Battery icon
  tft.drawRect(153, 10, 18, 10, COLOR_TEXT);
  tft.fillRect(154, 11, 16, 8, COLOR_GREEN);
  tft.fillRect(171, 13, 2, 6, COLOR_TEXT);
}

void drawYourAltitude() {
  // Panel background
  tft.fillRoundRect(10, 35, 220, 50, 8, COLOR_PANEL);
  
  // Label - size 3 to match party member names (title size)
  tft.setTextSize(3);
  tft.setTextColor(COLOR_TEXT_DIM);
  tft.setCursor(20, 42);
  tft.print("YOUR ALT");
  
  // Altitude value - size 2 to match body text
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, 63);
  tft.print(yourAltitude);
  tft.print(" ft");
}

void drawAltitudeArrow(int x, int y, int relativeAlt, uint16_t color) {
  // Draw up or down arrow based on relative altitude
  // Made slightly larger for better visibility
  if (relativeAlt >= 0) {
    // Up arrow (triangle pointing up)
    tft.fillTriangle(x, y - 6, x - 6, y + 4, x + 6, y + 4, color);
  } else {
    // Down arrow (triangle pointing down)
    tft.fillTriangle(x, y + 6, x - 6, y - 4, x + 6, y - 4, color);
  }
}

void drawCompassDial(int centerX, int centerY, int radius, float bearing, uint16_t fillColor) {
  // Outer ring
  tft.fillCircle(centerX, centerY, radius, 0x2104);  // Dark border
  tft.fillCircle(centerX, centerY, radius - 2, 0x4208);  // Medium gray
  
  // Inner circle background
  tft.fillCircle(centerX, centerY, radius - 6, COLOR_BG);
  
  // Draw N, E, S, W labels - size 2
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  
  // North
  tft.setCursor(centerX - 6, centerY - radius + 10);
  tft.print("N");
  
  // East
  tft.setCursor(centerX + radius - 20, centerY - 8);
  tft.print("E");
  
  // South
  tft.setCursor(centerX - 5, centerY + radius - 24);
  tft.print("S");
  
  // West
  tft.setCursor(centerX - radius + 6, centerY - 8);
  tft.print("W");
  
  // Draw direction arrow pointing to bearing
  float rad = bearing * 3.14159 / 180.0;
  
  // Arrow tip - scaled for current compass size
  int tipX = centerX + (radius - 10) * sin(rad);
  int tipY = centerY - (radius - 10) * cos(rad);
  
  // Arrow base corners (make a proper arrow shape)
  int baseLen = 10;
  int baseX1 = centerX + baseLen * sin(rad - 3.14159/2);
  int baseY1 = centerY - baseLen * cos(rad - 3.14159/2);
  
  int baseX2 = centerX + baseLen * sin(rad + 3.14159/2);
  int baseY2 = centerY - baseLen * cos(rad + 3.14159/2);
  
  // Draw filled arrow
  tft.fillTriangle(tipX, tipY, baseX1, baseY1, baseX2, baseY2, fillColor);
  
  // Draw line from center to base for arrow shaft
  int shaftX1 = centerX + 5 * sin(rad - 3.14159/2);
  int shaftY1 = centerY - 5 * cos(rad - 3.14159/2);
  int shaftX2 = centerX + 5 * sin(rad + 3.14159/2);
  int shaftY2 = centerY - 5 * cos(rad + 3.14159/2);
  
  tft.fillTriangle(shaftX1, shaftY1, shaftX2, shaftY2, baseX1, baseY1, fillColor);
  tft.fillTriangle(shaftX2, shaftY2, baseX1, baseY1, baseX2, baseY2, fillColor);
  
  // Center dot
  tft.fillCircle(centerX, centerY, 3, COLOR_TEXT);
}

uint16_t getDistanceColor(int distance) {
  if (distance < 100) return COLOR_GREEN;
  else if (distance < 300) return COLOR_YELLOW;
  else if (distance < 600) return COLOR_ORANGE;
  else return COLOR_RED;
}

void drawPartyMember(int yPos, PartyMember &member) {
  // Panel background - adjusted height for proper compass fit
  tft.fillRoundRect(10, yPos, 220, 95, 8, COLOR_PANEL);
  
  // Name - size 3 (title)
  tft.setTextSize(3);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(20, yPos + 8);
  tft.print(member.name);
  
  // Relative altitude with directional arrow
  // Draw arrow BEFORE the number (in place of +/-)
  drawAltitudeArrow(20, yPos + 44, member.relativeAlt, COLOR_ACCENT);
  
  tft.setTextSize(2);
  tft.setTextColor(COLOR_ACCENT);
  tft.setCursor(32, yPos + 36);  // Aligned with arrow
  tft.print(abs(member.relativeAlt));
  tft.print(" ft");
  
  // Distance - size 2 (body)
  tft.setTextSize(2);
  tft.setTextColor(getDistanceColor(member.distance));
  tft.setCursor(20, yPos + 58);
  tft.print(member.distance);
  tft.print(" ft");
  
  // Compass dial - radius 39, positioned to fit in panel
  int dialX = 170;
  int dialY = yPos + 55;
  drawCompassDial(dialX, dialY, 39, member.bearing, COLOR_RED);
}

void drawUI() {
  // Clear screen
  tft.fillScreen(COLOR_BG);
  
  // Status bar
  drawStatusBar();
  tft.drawFastHLine(0, 30, 240, COLOR_TEXT_DIM);
  
  // Your altitude
  drawYourAltitude();
  
  // Party members - adjusted spacing for properly sized panels (95px tall each)
  drawPartyMember(95, members[0]);    // JAKE
  drawPartyMember(200, members[1]);   // JEN (95 + 10px gap from previous panel)
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("Party Tracker UI");
  Serial.println("========================================\n");
  
  // Initialize SPI
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  
  // Initialize display
  tft.begin(SPI_FREQUENCY);
  tft.setRotation(0);  // Portrait mode
  
  Serial.println("Drawing UI...");
  drawUI();
  
  Serial.println("UI complete!");
  Serial.println("========================================\n");
}

void loop() {
  static unsigned long lastUpdate = 0;
  static int counter = 0;
  
  // Update every 2 seconds to simulate live updates
  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();
    counter++;
    
    // Simulate updates
    members[0].lastUpdate++;
    members[1].lastUpdate++;
    
    // Simulate movement
    if (counter % 3 == 0) {
      members[0].bearing += 15;
      if (members[0].bearing >= 360) members[0].bearing -= 360;
      members[0].lastUpdate = 0;
    }
    
    if (counter % 5 == 0) {
      members[1].bearing -= 20;
      if (members[1].bearing < 0) members[1].bearing += 360;
      members[1].distance += 10;
      members[1].lastUpdate = 0;
    }
    
    // Redraw (in production, only redraw changed sections)
    drawUI();
    
    Serial.print("Update ");
    Serial.println(counter);
  }
}
