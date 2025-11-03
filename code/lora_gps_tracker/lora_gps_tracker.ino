/* Heltec Wireless Tracker - GPS Position Sharing via LoRa
 * 
 * Function:
 * 1. Read GPS coordinates from onboard GPS module
 * 2. Share position with another tracker via LoRa
 * 3. Display local and remote positions on onboard TFT screen
 * 4. Calculate and display distance and bearing between devices
 */

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"

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

// Objects
TinyGPSPlus GPS;
HT_st7735 st7735;

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
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);

void setup() {
    Serial.begin(115200);
    
    // Initialize TFT Display
    st7735.st7735_init();
    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, (String)"GPS LoRa Tracker");
    st7735.st7735_write_str(0, 20, (String)"Initializing...");
    
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
    delay(1000);
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
    if (millis() - lastDisplayUpdate > 2000) {
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
    st7735.st7735_fill_screen(ST7735_BLACK);
    
    // Compact display for 160x80 screen
    // Local Position
    if (localPos.valid) {
        String latStr = "L:" + String(localPos.lat, 4);
        String lonStr = String(localPos.lon, 4);
        st7735.st7735_write_str(0, 0, latStr);
        st7735.st7735_write_str(0, 10, lonStr);
    } else {
        st7735.st7735_write_str(0, 0, (String)"L:No Fix");
    }
    
    // Remote Position & Distance
    if (millis() - remotePos.lastUpdate < 30000) {
        // We've received something recently from remote
        if (remotePos.valid) {
            // Remote has a GPS fix
            String rLatStr = "R:" + String(remotePos.lat, 4);
            String rLonStr = String(remotePos.lon, 4);
            st7735.st7735_write_str(0, 25, rLatStr);
            st7735.st7735_write_str(0, 35, rLonStr);
            
            // Calculate and display relative position
            if (localPos.valid) {
                double distance = calculateDistance(localPos.lat, localPos.lon, 
                                                   remotePos.lat, remotePos.lon);
                double bearing = calculateBearing(localPos.lat, localPos.lon, 
                                                 remotePos.lat, remotePos.lon);
                
                String distStr = String(distance, 0) + "m " + String(bearing, 0) + "d";
                st7735.st7735_write_str(0, 50, distStr);
            }
        } else {
            // Remote connected but no GPS fix
            st7735.st7735_write_str(0, 25, (String)"R:No Fix");
        }
    } else {
        // Haven't heard from remote in 30 seconds
        st7735.st7735_write_str(0, 25, (String)"R:No Signal");
    }
    
    // RSSI
    String rssiStr = "RSSI:" + String(Rssi);
    st7735.st7735_write_str(0, 65, rssiStr);
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