/* Heltec Wireless Tracker - GPS Position Sharing via LoRa with ILI9341 Display
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
 * - LED   -> 3V3 (with 100-150Ω resistor if needed)
 * 
 */

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// ILI9341 Pin Configuration
#define TFT_CS      7
#define TFT_DC      5
#define TFT_RST     6
#define TFT_MOSI    4
#define TFT_SCK     19

// LoRa Configuration
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // 125 kHz
#define LORA_SPREADING_FACTOR                       7         // SF7
#define LORA_CODINGRATE                             1         // 4/5
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 50

// GPS Configuration
#define VGNSS_CTRL 3

// Color Definitions (Modern UI Palette)
#define COLOR_BG           0x0000  // Black background
#define COLOR_PRIMARY      0x07FF  // Cyan
#define COLOR_SECONDARY    0xFD20  // Orange
#define COLOR_SUCCESS      0x07E0  // Green
#define COLOR_WARNING      0xFFE0  // Yellow
#define COLOR_ERROR        0xF800  // Red
#define COLOR_TEXT         0xFFFF  // White
#define COLOR_TEXT_DIM     0x7BEF  // Gray
#define COLOR_PANEL_BG     0x2104  // Dark gray panel

// Objects
TinyGPSPlus GPS;
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

// LoRa Variables
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

typedef enum {
    LOWPOWER,
    STATE_RX,
    STATE_TX
} States_t;

States_t state;
int16_t Rssi = 0;

// GPS Position Storage
struct Position {
    double lat;
    double lon;
    bool valid;
    unsigned long lastUpdate;
};

Position localPos = {0, 0, false, 0};
Position remotePos = {0, 0, false, 0};

// Function Prototypes
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void updateGPS();
void updateDisplay();
void drawStatusBar();
void drawPositionPanel(const char* title, Position& pos, int y, uint16_t color);
void drawRelativePosition();
void drawCompass(int centerX, int centerY, int radius, double bearing);
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
String formatCoordinate(double coord, bool isLat);
String formatDistance(double meters);
String getDirectionString(double bearing);

void setup() {
    Serial.begin(115200);
    
    // Initialize ILI9341 Display
    tft.begin();
    tft.setRotation(0);  // Portrait mode (240x320)
    tft.fillScreen(COLOR_BG);
    
    // Show splash screen
    tft.setTextColor(COLOR_PRIMARY);
    tft.setTextSize(3);
    tft.setCursor(30, 100);
    tft.println("GPS TRACKER");
    
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setTextSize(1);
    tft.setCursor(60, 140);
    tft.println("LoRa Position");
    tft.setCursor(75, 150);
    tft.println("Sharing");
    
    tft.setTextColor(COLOR_SECONDARY);
    tft.setTextSize(1);
    tft.setCursor(70, 180);
    tft.println("Initializing...");
    
    delay(2000);
    
    // Initialize GPS
    pinMode(VGNSS_CTRL, OUTPUT);
    digitalWrite(VGNSS_CTRL, HIGH);
    Serial1.begin(115200, SERIAL_8N1, 33, 34);
    
    // Initialize LoRa
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    
    state = STATE_RX;
    
    // Initial display setup
    tft.fillScreen(COLOR_BG);
    delay(500);
}

void loop() {
    // Update GPS position
    updateGPS();
    
    // State machine for LoRa communication
    switch(state) {
        case STATE_TX:
            // Always send a packet - either coordinates or "NOFIX"
            if (localPos.valid) {
                sprintf(txpacket, "%.6f,%.6f", localPos.lat, localPos.lon);
                Serial.printf("Sending: %s\r\n", txpacket);
            } else {
                sprintf(txpacket, "NOFIX");
                Serial.println("Sending: NOFIX");
            }
            Radio.Send((uint8_t *)txpacket, strlen(txpacket));
            state = LOWPOWER;
            break;
            
        case STATE_RX:
            Serial.println("RX mode");
            Radio.Rx(0);
            state = LOWPOWER;
            break;
            
        case LOWPOWER:
            Radio.IrqProcess();
            break;
    }
    
    // Update display periodically
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 1000) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }
    
    // Transmit periodically (every 5 seconds)
    static unsigned long lastTx = 0;
    if (millis() - lastTx > 5000 && state == LOWPOWER) {
        state = STATE_TX;
        lastTx = millis();
    }
}

void updateGPS() {
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        if (GPS.encode(c)) {
            if (GPS.location.isValid()) {
                localPos.lat = GPS.location.lat();
                localPos.lon = GPS.location.lng();
                localPos.valid = true;
                localPos.lastUpdate = millis();
            }
        }
    }
}

void updateDisplay() {
    tft.fillScreen(COLOR_BG);
    
    // Draw status bar
    drawStatusBar();
    
    // Draw local position panel
    drawPositionPanel("LOCAL", localPos, 25, COLOR_PRIMARY);
    
    // Draw remote position panel
    drawPositionPanel("REMOTE", remotePos, 130, COLOR_SECONDARY);
    
    // Draw relative position (distance & bearing)
    if (localPos.valid && remotePos.valid && 
        (millis() - remotePos.lastUpdate < 30000)) {
        drawRelativePosition();
    }
}

void drawStatusBar() {
    // Status bar background
    tft.fillRect(0, 0, 240, 20, COLOR_PANEL_BG);
    
    // GPS Status
    tft.setTextSize(1);
    if (localPos.valid) {
        tft.setTextColor(COLOR_SUCCESS);
        tft.setCursor(5, 6);
        tft.print("GPS");
        tft.fillCircle(28, 10, 3, COLOR_SUCCESS);
        
        // Satellite count if available
        if (GPS.satellites.isValid()) {
            tft.setTextColor(COLOR_TEXT_DIM);
            tft.setCursor(35, 6);
            tft.print("SAT:");
            tft.print(GPS.satellites.value());
        }
    } else {
        tft.setTextColor(COLOR_ERROR);
        tft.setCursor(5, 6);
        tft.print("NO GPS");
    }
    
    // LoRa RSSI
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(150, 6);
    tft.print("RSSI:");
    if (Rssi != 0) {
        tft.setTextColor(COLOR_TEXT);
        tft.print(Rssi);
    } else {
        tft.print("--");
    }
}

void drawPositionPanel(const char* title, Position& pos, int y, uint16_t color) {
    // Panel background
    tft.drawRect(5, y, 230, 95, color);
    tft.drawRect(6, y+1, 228, 93, color);
    
    // Title
    tft.setTextSize(1);
    tft.setTextColor(color);
    tft.setCursor(10, y + 5);
    tft.print(title);
    
    // Check remote signal timeout
    bool isRemote = (strcmp(title, "REMOTE") == 0);
    if (isRemote && (millis() - pos.lastUpdate > 30000)) {
        tft.setTextSize(2);
        tft.setTextColor(COLOR_TEXT_DIM);
        tft.setCursor(50, y + 45);
        tft.print("NO SIGNAL");
        return;
    }
    
    // Position data
    if (pos.valid) {
        // Latitude
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT_DIM);
        tft.setCursor(10, y + 25);
        tft.print("LAT:");
        
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(10, y + 40);
        tft.setTextSize(2);
        String latStr = formatCoordinate(pos.lat, true);
        tft.print(latStr);
        
        // Longitude
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT_DIM);
        tft.setCursor(10, y + 60);
        tft.print("LON:");
        
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(10, y + 75);
        tft.setTextSize(2);
        String lonStr = formatCoordinate(pos.lon, false);
        tft.print(lonStr);
        
    } else {
        tft.setTextSize(2);
        tft.setTextColor(COLOR_WARNING);
        tft.setCursor(55, y + 45);
        tft.print("NO FIX");
    }
}

void drawRelativePosition() {
    int panelY = 235;
    
    // Panel background
    tft.fillRect(5, panelY, 230, 80, COLOR_PANEL_BG);
    
    // Calculate distance and bearing
    double distance = calculateDistance(localPos.lat, localPos.lon, 
                                       remotePos.lat, remotePos.lon);
    double bearing = calculateBearing(localPos.lat, localPos.lon, 
                                     remotePos.lat, remotePos.lon);
    
    // Distance display
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(15, panelY + 10);
    tft.print("DISTANCE");
    
    tft.setTextSize(3);
    tft.setTextColor(COLOR_SUCCESS);
    tft.setCursor(15, panelY + 25);
    tft.print(formatDistance(distance));
    
    // Direction display
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(15, panelY + 55);
    tft.print("DIRECTION: ");
    tft.setTextColor(COLOR_TEXT);
    tft.print(getDirectionString(bearing));
    
    // Compass
    drawCompass(180, panelY + 40, 25, bearing);
}

void drawCompass(int centerX, int centerY, int radius, double bearing) {
    // Outer circle
    tft.drawCircle(centerX, centerY, radius, COLOR_PRIMARY);
    tft.drawCircle(centerX, centerY, radius - 1, COLOR_PRIMARY);
    
    // Cardinal directions
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_DIM);
    tft.setCursor(centerX - 3, centerY - radius - 8);
    tft.print("N");
    tft.setCursor(centerX + radius + 2, centerY - 3);
    tft.print("E");
    tft.setCursor(centerX - 3, centerY + radius + 2);
    tft.print("S");
    tft.setCursor(centerX - radius - 8, centerY - 3);
    tft.print("W");
    
    // Bearing needle
    float bearingRad = bearing * PI / 180.0;
    int x2 = centerX + (radius - 5) * sin(bearingRad);
    int y2 = centerY - (radius - 5) * cos(bearingRad);
    
    // Draw arrow
    tft.drawLine(centerX, centerY, x2, y2, COLOR_SECONDARY);
    tft.drawLine(centerX, centerY, x2, y2, COLOR_SECONDARY);
    tft.fillCircle(x2, y2, 4, COLOR_SECONDARY);
    tft.fillCircle(centerX, centerY, 3, COLOR_TEXT);
}

String formatCoordinate(double coord, bool isLat) {
    char dir;
    if (isLat) {
        dir = (coord >= 0) ? 'N' : 'S';
    } else {
        dir = (coord >= 0) ? 'E' : 'W';
    }
    
    coord = abs(coord);
    int degrees = (int)coord;
    double minutes = (coord - degrees) * 60.0;
    
    char buffer[20];
    sprintf(buffer, "%d°%.2f'%c", degrees, minutes, dir);
    return String(buffer);
}

String formatDistance(double meters) {
    if (meters < 1000) {
        return String((int)meters) + "m";
    } else {
        return String(meters / 1000.0, 1) + "km";
    }
}

String getDirectionString(double bearing) {
    const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int index = (int)((bearing + 22.5) / 45.0) % 8;
    return String(directions[index]) + " " + String((int)bearing) + "°";
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Haversine formula for distance calculation
    const double R = 6371000; // Earth radius in meters
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLon = (lon2 - lon1) * PI / 180.0;
    
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return R * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    // Calculate bearing from point 1 to point 2
    double dLon = (lon2 - lon1) * PI / 180.0;
    double lat1Rad = lat1 * PI / 180.0;
    double lat2Rad = lat2 * PI / 180.0;
    
    double y = sin(dLon) * cos(lat2Rad);
    double x = cos(lat1Rad) * sin(lat2Rad) -
               sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
    double bearing = atan2(y, x) * 180.0 / PI;
    
    // Normalize to 0-360
    bearing = fmod((bearing + 360.0), 360.0);
    
    return bearing;
}

void OnTxDone(void) {
    Serial.println("TX done");
    state = STATE_RX;
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("TX Timeout");
    state = STATE_RX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    Rssi = rssi;
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Radio.Sleep();
    
    Serial.printf("Received: %s (RSSI: %d)\r\n", rxpacket, Rssi);
    
    // Check if remote has no fix
    if (strcmp(rxpacket, "NOFIX") == 0) {
        remotePos.valid = false;
        remotePos.lastUpdate = millis();
        Serial.println("Remote has no GPS fix");
    } else {
        // Parse received coordinates
        double lat, lon;
        if (sscanf(rxpacket, "%lf,%lf", &lat, &lon) == 2) {
            remotePos.lat = lat;
            remotePos.lon = lon;
            remotePos.valid = true;
            remotePos.lastUpdate = millis();
            Serial.printf("Remote position updated: %.6f, %.6f\r\n", lat, lon);
        }
    }
    
    state = STATE_RX;
}
