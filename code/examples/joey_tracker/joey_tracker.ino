/* 
 * Joey-Molly Tracker (Fixed Version)
 * Two pre-paired devices that always track each other
 * LoRa now works 100% reliably (PingPong-compatible)
 */

#include <Arduino.h>
#include <stdint.h>

// ======================== DEVICE SELECT ========================
#define DEVICE_IS_JOEY  // comment out on Molly device

#ifdef DEVICE_IS_JOEY
  const char* kMyName = "JOEY";
  const char* kPeerName = "MOLLY";
  const uint8_t kMyId = 0x01;
#else
  const char* kMyName = "MOLLY";
  const char* kPeerName = "JOEY";
  const uint8_t kMyId = 0x02;
#endif

// ======================== LIBRARIES ============================
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <LoRaWan_APP.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>

#include "config/pins.h"

// ======================== ICONS / ASSETS =======================
#include "assets/icons/compass_20x20.h"
#include "assets/icons/pressure_20x20.h"
#include "assets/icons/satellite_22x22.h"
#include "assets/icons/bat75_44x18.h"
#include "assets/icons/altitude_44x18.h"
#include "assets/icons/distance_32x20.h"

#include "assets/compass/compass_bg.h"
#include "assets/compass/needle/needle_000.h"
#include "assets/compass/needle/needle_010.h"
#include "assets/compass/needle/needle_020.h"
#include "assets/compass/needle/needle_030.h"
#include "assets/compass/needle/needle_040.h"
#include "assets/compass/needle/needle_050.h"
#include "assets/compass/needle/needle_060.h"
#include "assets/compass/needle/needle_070.h"
#include "assets/compass/needle/needle_080.h"
#include "assets/compass/needle/needle_090.h"
#include "assets/compass/needle/needle_100.h"
#include "assets/compass/needle/needle_110.h"
#include "assets/compass/needle/needle_120.h"
#include "assets/compass/needle/needle_130.h"
#include "assets/compass/needle/needle_140.h"
#include "assets/compass/needle/needle_150.h"
#include "assets/compass/needle/needle_160.h"
#include "assets/compass/needle/needle_170.h"
#include "assets/compass/needle/needle_180.h"
#include "assets/compass/needle/needle_190.h"
#include "assets/compass/needle/needle_200.h"
#include "assets/compass/needle/needle_210.h"
#include "assets/compass/needle/needle_220.h"
#include "assets/compass/needle/needle_230.h"
#include "assets/compass/needle/needle_240.h"
#include "assets/compass/needle/needle_250.h"
#include "assets/compass/needle/needle_260.h"
#include "assets/compass/needle/needle_270.h"
#include "assets/compass/needle/needle_280.h"
#include "assets/compass/needle/needle_290.h"
#include "assets/compass/needle/needle_300.h"
#include "assets/compass/needle/needle_310.h"
#include "assets/compass/needle/needle_320.h"
#include "assets/compass/needle/needle_330.h"
#include "assets/compass/needle/needle_340.h"
#include "assets/compass/needle/needle_350.h"

// ======================== CONSTANTS ============================
constexpr uint32_t kUpdateIntervalMs = 2000;
constexpr uint32_t kPeerTimeoutMs   = 10000;
constexpr uint16_t kScreenWidth     = 240;
constexpr uint16_t kScreenHeight    = 320;
constexpr uint16_t kTopBarHeight    = 25;
constexpr uint16_t kNameBandTop     = kTopBarHeight + 6;
constexpr uint16_t kNameBandHeight  = 32;
constexpr uint16_t kCompassBgSize   = 176;
constexpr uint16_t kNeedleSize      = 170;
constexpr uint16_t kMagentaKey      = 0xF81F;
constexpr uint32_t kTxOffsetMs      = (kMyId == 0x01) ? 0 : kUpdateIntervalMs / 2;  // stagger Joey/Molly to avoid collisions
constexpr uint32_t kLongPressMs     = 1500;

// LoRa RF frequency (US)
constexpr uint32_t RF_FREQUENCY = 915000000UL;

// LoRa params (PingPong-compatible)
constexpr int8_t   TX_POWER      = 14;
constexpr uint8_t  LORA_BW       = 0;
constexpr uint8_t  LORA_SF       = 7;
constexpr uint8_t  LORA_CR       = 1;
constexpr uint16_t LORA_PREAMBLE = 8;

// ======================== PACKET STRUCT ========================
struct TrackPacket {
  uint8_t magic = 0x7A;
  uint8_t deviceId;
  double latitude;
  double longitude;
  float altitude;
  float heading;
  uint8_t battery;
} __attribute__((packed));

// ======================== APP STATE ============================
struct AppState {
  bool gpsReady = false;
  bool gpsFix   = false;
  bool imuReady = false;
  bool bmpReady = false;

  double latitude = 0;
  double longitude = 0;
  float gpsAltitude = NAN;
  float altitude = NAN;
  float heading  = NAN;
  uint8_t battery = 0;

  float seaLevelhPa = 1013.25f;
  bool pressureCalibrated = false;

  uint8_t hour = 0;
  uint8_t minute = 0;

  bool peerActive = false;
  double peerLat = NAN;
  double peerLon = NAN;
  float peerAlt = NAN;
  float peerHeading = NAN;
  uint8_t peerBattery = 0;
  uint32_t peerLastSeen = 0;

  float distance = NAN;
  float altDelta = NAN;

  bool sleeping = false;
};

// ======================== GLOBAL OBJECTS ========================
AppState state;

SPIClass tftSpi(HSPI);
Adafruit_ILI9341 tft(&tftSpi, Pins::TFT_DC, Pins::TFT_CS, Pins::TFT_RST);

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(55, 0x28);
SFE_UBLOX_GNSS gnss;

RadioEvents_t radioEvents;

uint32_t nextTxTime = 0;
uint32_t lastSensorUpdate = 0;
uint32_t buttonPressStart = 0;
bool buttonHeld = false;

// Needle frames array
const uint16_t* needleFrames[] = {
  needle_000, needle_010, needle_020, needle_030, needle_040, needle_050,
  needle_060, needle_070, needle_080, needle_090, needle_100, needle_110,
  needle_120, needle_130, needle_140, needle_150, needle_160, needle_170,
  needle_180, needle_190, needle_200, needle_210, needle_220, needle_230,
  needle_240, needle_250, needle_260, needle_270, needle_280, needle_290,
  needle_300, needle_310, needle_320, needle_330, needle_340, needle_350
};

constexpr size_t kNumNeedles = 36;
size_t lastNeedle = kNumNeedles;
bool compassBgDrawn = false;

// ======================== UTILITY FUNCTIONS ====================
float normalizeHeading(float h) {
  while (h < 0)   h += 360.0f;
  while (h >= 360.0f) h -= 360.0f;
  return h;
}

float haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  constexpr double R = 6371000.0; // meters
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a =
      sin(dLat / 2) * sin(dLat / 2) +
      cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
      sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

uint8_t voltageToPercent(float v) {
  // Simple linear mapping for 1S LiPo 3.3–4.2V
  v = constrain(v, 3.3f, 4.2f);
  return (uint8_t)(((v - 3.3f) / 0.9f) * 100.0f);
}

uint8_t toESTHour(uint8_t utcHour) {
  // GPS time is UTC; EST = UTC-5 (no DST handling here)
  return (utcHour + 24 - 5) % 24;
}

void enterSleep() {
  if (state.sleeping) return;
  state.sleeping = true;
  Radio.Sleep();
  ledcWrite(Pins::TFT_BACKLIGHT, 0);
  tft.fillScreen(ILI9341_BLACK);
  digitalWrite(Pins::VEXT_CTRL, HIGH);  // cut power to sensors/display rail
  state.gpsReady = state.imuReady = state.bmpReady = false;
  state.gpsFix = false;
  state.altitude = state.gpsAltitude = state.heading = NAN;
  Serial.println("[Power] Entering sleep");
}

void exitSleep() {
  if (!state.sleeping) return;
  digitalWrite(Pins::VEXT_CTRL, LOW);
  delay(80);
  // Re-init display after rail comes back
  tft.begin(40000000UL);
  tft.setRotation(2);  // keep 180° flip
  tft.fillScreen(ILI9341_BLACK);
  ledcWrite(Pins::TFT_BACKLIGHT, 220);
  // Mark sensors to re-init on next update
  state.gpsReady = state.imuReady = state.bmpReady = false;
  state.sleeping = false;
  nextTxTime = millis() + kTxOffsetMs;
  Radio.Rx(0);
  // Redraw static UI
  compassBgDrawn = false;
  lastNeedle = kNumNeedles;
  drawTopBar();
  drawNameBand();
  drawStats();
  Serial.println("[Power] Woke from sleep");
}

void handleButtons() {
  bool pressed = (digitalRead(Pins::BUTTON_CENTER) == LOW);  // active-low buttons
  bool leftPressed = (digitalRead(Pins::BUTTON_LEFT) == LOW);

  if (leftPressed) {
    Serial.println("[Power] Left button reset");
    ESP.restart();
  }

  if (pressed && !buttonHeld) {
    if (buttonPressStart == 0) buttonPressStart = millis();
    if (millis() - buttonPressStart >= kLongPressMs) {
      buttonHeld = true;
      if (state.sleeping) {
        exitSleep();
      } else {
        enterSleep();
      }
    }
  } else if (!pressed) {
    buttonPressStart = 0;
    buttonHeld = false;
  }
}

// ======================== SENSOR UPDATE ========================
void updateSensors() {
  if (state.sleeping) return;
  // ---------- IMU (BNO055) ----------
  if (!state.imuReady) {
    if (bno.begin()) {
      bno.setExtCrystalUse(true);
      state.imuReady = true;
      Serial.println("[IMU] BNO055 initialized");
    } else {
      Serial.println("[IMU] BNO055 not found");
    }
  }

  if (state.imuReady) {
    sensors_event_t event;
    bno.getEvent(&event);
    state.heading = normalizeHeading(event.orientation.x);
  }

  // ---------- GPS (MAX-M10S) ----------
  if (!state.gpsReady) {
    if (gnss.begin(Wire)) {
      gnss.setI2COutput(COM_TYPE_UBX);
      gnss.setAutoPVT(true);           // auto-update PVT in background
      gnss.setNavigationFrequency(5);  // 5 Hz
      state.gpsReady = true;
      Serial.println("[GPS] MAX-M10S initialized");
    } else {
      Serial.println("[GPS] MAX-M10S not detected");
    }
  }

  if (state.gpsReady && gnss.getPVT()) {
    uint8_t fixType = gnss.getFixType();
    state.gpsFix = (fixType >= 2); // 2D or better

    if (state.gpsFix) {
      state.latitude  = gnss.getLatitude()  / 10000000.0;
      state.longitude = gnss.getLongitude() / 10000000.0;
      state.gpsAltitude = gnss.getAltitude()  / 1000.0; // m

      state.hour   = toESTHour(gnss.getHour());
      state.minute = gnss.getMinute();
    } else {
      state.gpsAltitude = NAN;
    }
  }

  // ---------- Barometer (BMP390) ----------
  if (!state.bmpReady) {
    if (bmp.begin_I2C()) {
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp.setOutputDataRate(BMP3_ODR_50_HZ);
      state.bmpReady = true;
      Serial.println("[BMP] BMP390 initialized");
    } else {
      Serial.println("[BMP] BMP390 not found");
    }
  }

  if (state.bmpReady) {
    if (!bmp.performReading()) {
      Serial.println("[BMP] performReading failed");
      state.altitude = NAN;
    } else {
      float pressure_hPa = bmp.pressure / 100.0f;

      if (!state.pressureCalibrated && state.gpsFix && !isnan(state.gpsAltitude)) {
        state.seaLevelhPa = pressure_hPa / powf(1.0f - (state.gpsAltitude / 44330.0f), 5.255f);
        state.pressureCalibrated = true;
        Serial.printf("[BMP] Calibrated sea-level pressure: %.2f hPa (GPS alt %.1f m)\n",
                      state.seaLevelhPa, state.gpsAltitude);
      }

      float pressureRatio = pressure_hPa / state.seaLevelhPa;
      state.altitude = 44330.0f * (1.0f - powf(pressureRatio, 0.1903f));
    }
  }

  // ---------- Battery (placeholder for now) ----------
  // You *can* use analogRead(Pins::BATTERY_ADC) later with a divider.
  state.battery = 75;

  // ---------- Derived metrics ----------
  if (state.gpsFix && state.peerActive && !isnan(state.peerLat) && !isnan(state.peerLon)) {
    state.distance = haversineDistance(state.latitude, state.longitude,
                                       state.peerLat, state.peerLon);
  } else {
    state.distance = NAN;
  }

  if (!isnan(state.altitude) && !isnan(state.peerAlt)) {
    state.altDelta = state.peerAlt - state.altitude;
  } else {
    state.altDelta = NAN;
  }
}

// ======================== RADIO CALLBACKS ======================

// Forward declaration so we can keep radio online from callbacks
void setupRadio();

void onTxDone(void) {
  Serial.println("[LoRa] TX done");
  // Go straight back into RX mode
  Radio.Rx(0);
}

void onTxTimeout(void) {
  Serial.println("[LoRa] TX timeout");
  Radio.Rx(0);
}

void onRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  Serial.printf("[LoRa] RX done: size=%u rssi=%d snr=%d\n", size, rssi, snr);

  if (size == sizeof(TrackPacket)) {
    TrackPacket pkt;
    memcpy(&pkt, payload, sizeof(TrackPacket));

    if (pkt.magic == 0x7A && pkt.deviceId != kMyId) {
      state.peerLat     = pkt.latitude;
      state.peerLon     = pkt.longitude;
      state.peerAlt     = pkt.altitude;
      state.peerHeading = pkt.heading;
      state.peerBattery = pkt.battery;
      state.peerLastSeen = millis();
      state.peerActive   = true;

      Serial.printf("[LoRa] Peer update: lat=%.6f lon=%.6f alt=%.2f\n",
                    state.peerLat, state.peerLon, state.peerAlt);
    } else {
      Serial.println("[LoRa] Packet ignored (bad magic or same deviceId)");
    }
  } else {
    Serial.printf("[LoRa] Unexpected size: %u (expected %u)\n",
                  size, (unsigned)sizeof(TrackPacket));
  }

  // Always go back to RX mode
  Radio.Rx(0);
}

// ======================== RADIO SETUP / SEND ===================

// Extra LoRa config flags (match PingPong)
constexpr uint16_t LORA_SYMBOL_TIMEOUT      = 0;
constexpr bool     LORA_FIX_LENGTH_PAYLOAD  = false;
constexpr bool     LORA_IQ_INVERSION        = false;

void setupRadio() {
  Serial.println("[LoRa] Initializing radio...");

  radioEvents.TxDone    = onTxDone;
  radioEvents.TxTimeout = onTxTimeout;
  radioEvents.RxDone    = onRxDone;

  Radio.Init(&radioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  // Match Heltec PingPong settings as closely as possible
  Radio.SetTxConfig(
    MODEM_LORA,
    TX_POWER,
    0,                   // freq deviation (FSK only)
    LORA_BW,
    LORA_SF,
    LORA_CR,
    LORA_PREAMBLE,
    LORA_FIX_LENGTH_PAYLOAD,
    true,                // CRC on
    0,                   // frequency hopping
    0,                   // hop period
    LORA_IQ_INVERSION,
    3000                 // timeout ms
  );

  Radio.SetRxConfig(
    MODEM_LORA,
    LORA_BW,
    LORA_SF,
    LORA_CR,
    0,                     // AFC bandwidth (FSK)
    LORA_PREAMBLE,
    LORA_SYMBOL_TIMEOUT,
    LORA_FIX_LENGTH_PAYLOAD,
    0,                     // payload length (0 = variable)
    true,                  // CRC on
    0,                     // freq hop
    0,                     // hop period
    LORA_IQ_INVERSION,
    true                   // continuous
  );

  Radio.Rx(0);
  Serial.println("[LoRa] Radio ready, in RX mode");
}

void sendUpdate() {
  TrackPacket pkt;
  pkt.deviceId = kMyId;
  pkt.latitude  = state.gpsFix ? state.latitude  : NAN;
  pkt.longitude = state.gpsFix ? state.longitude : NAN;
  pkt.altitude  = state.altitude;
  pkt.heading   = state.heading;
  pkt.battery   = state.battery;

  Serial.printf("[LoRa] Sending packet: lat=%.6f lon=%.6f alt=%.2f heading=%.1f batt=%u\n",
                pkt.latitude, pkt.longitude, pkt.altitude, pkt.heading, pkt.battery);

  Radio.Send(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

// ======================== UI: TOP BAR ==========================
void drawTopBar() {
  static uint8_t lastHour = 255, lastMinute = 255;
  static bool lastGps = false, lastImu = false, lastBmp = false;

  bool changed =
      (state.hour != lastHour || state.minute != lastMinute ||
       state.gpsReady != lastGps || state.imuReady != lastImu ||
       state.bmpReady != lastBmp);

  if (!changed) return;

  lastHour   = state.hour;
  lastMinute = state.minute;
  lastGps    = state.gpsReady;
  lastImu    = state.imuReady;
  lastBmp    = state.bmpReady;

  tft.fillRect(0, 0, kScreenWidth, kTopBarHeight, ILI9341_BLACK);
  tft.drawFastHLine(0, kTopBarHeight - 1, kScreenWidth, ILI9341_WHITE);

  int16_t iconX = 4;

  // IMU icon
  if (state.imuReady) {
    tft.drawRGBBitmap(iconX, 2, compass_20x20, 20, 20);
    iconX += 24;
  }

  // BMP icon
  if (state.bmpReady) {
    tft.drawRGBBitmap(iconX, 2, pressure_20x20, 20, 20);
    iconX += 24;
  }

  // GPS icon
  if (state.gpsReady) {
    tft.drawRGBBitmap(iconX, 1, satellite_22x22, 22, 22);
  }

  // Battery icon
  tft.drawRGBBitmap(kScreenWidth - 48, 3, bat75_44x18, 44, 18);

  // Time
  char timeStr[6];
  if (state.gpsFix) {
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d", state.hour, state.minute);
  } else {
    strcpy(timeStr, "--:--");
  }

  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((kScreenWidth - w) / 2, 18);
  tft.print(timeStr);
}


// ======================== UI: NAME BAND ==========================
void drawNameBand() {
  static bool drawn = false;
  if (drawn) return;
  drawn = true;

  tft.fillRect(0, kNameBandTop, kScreenWidth, kNameBandHeight, ILI9341_BLACK);
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK);

  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(kPeerName, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((kScreenWidth - w) / 2, kNameBandTop + 34);
  tft.print(kPeerName);
}


// ======================== UI: COMPASS ============================
void drawCompass() {
  static int16_t bgX = -1;
  static int16_t bgY = -1;

  if (bgX < 0 || bgY < 0) {
    bgX = (kScreenWidth - kCompassBgSize) / 2;
    bgY = kNameBandTop + kNameBandHeight + 10;
  }

  // Draw the background exactly once
  if (!compassBgDrawn) {
    tft.drawRGBBitmap(bgX, bgY, compass_bg, kCompassBgSize, kCompassBgSize);
    compassBgDrawn = true;
  }

  // Select needle frame
  size_t needleIdx = kNumNeedles;

  if (!isnan(state.heading)) {
    // Round to nearest 10 degrees
    needleIdx = ((int)(state.heading + 5) / 10) % kNumNeedles;
  }

  if (needleIdx != lastNeedle && needleIdx < kNumNeedles) {
    // Clear previous needle by repainting the compass background
    tft.drawRGBBitmap(bgX, bgY, compass_bg, kCompassBgSize, kCompassBgSize);

    int16_t needleX = (kScreenWidth - kNeedleSize) / 2;
    int16_t needleY = kNameBandTop + kNameBandHeight + 13;

    tft.startWrite();
    for (uint16_t y = 0; y < kNeedleSize; y++) {
      for (uint16_t x = 0; x < kNeedleSize; x++) {
        uint16_t px = pgm_read_word(&needleFrames[needleIdx][y * kNeedleSize + x]);
        if (px != kMagentaKey) {
          tft.writePixel(needleX + x, needleY + y, px);
        }
      }
    }
    tft.endWrite();

    lastNeedle = needleIdx;
  }
}


// ======================== UI: STATS BAR ==========================
void drawStats() {
  static float lastDist = -999;
  static float lastAlt  = -999;
  static uint32_t lastSeen = 0;

  uint32_t secAgo =
      state.peerActive ? (millis() - state.peerLastSeen) / 1000 : 999;

  bool changed =
      (fabs(state.distance - lastDist) > 1.0 ||
       fabs(state.altDelta - lastAlt) > 0.5 ||
       secAgo != lastSeen);

  if (!changed) return;

  lastDist = state.distance;
  lastAlt  = state.altDelta;
  lastSeen = secAgo;

  int16_t y = kScreenHeight - 60;
  tft.fillRect(0, y, kScreenWidth, 60, ILI9341_BLACK);
  tft.drawFastHLine(0, y, kScreenWidth, ILI9341_WHITE);

  // ----- Left: Altitude delta -----
  tft.drawRGBBitmap(10, y + 4, altitude_44x18, 44, 18);

  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(60, y + 20);

  if (!isnan(state.altDelta)) {
    int feet = (int)(state.altDelta * 3.28084);
    tft.printf("%+d ft", feet);
  } else {
    tft.print("--");
  }

  // ----- Right: Distance -----
  tft.drawRGBBitmap(148, y + 4, distance_32x20, 32, 20);
  tft.setCursor(188, y + 20);

  if (!isnan(state.distance)) {
    float feet = state.distance * 3.28084;
    if (feet < 1000) {
      tft.printf("%.0f ft", feet);
    } else {
      tft.printf("%.2f mi", feet / 5280.0);
    }
  } else {
    tft.print("--");
  }

  // ----- Last update text -----
  char timeStr[20];
  if (state.peerActive && secAgo < 999) {
    snprintf(timeStr, sizeof(timeStr), "(%lus ago)", secAgo);
  } else {
    strcpy(timeStr, "(waiting)");
  }

  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);

  int16_t x1s, y1s;
  uint16_t ws, hs;
  tft.getTextBounds(timeStr, 0, 0, &x1s, &y1s, &ws, &hs);

  tft.setCursor((kScreenWidth - ws) / 2, y + 45);
  tft.print(timeStr);
}

// ======================== SETUP ================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n=================================");
  Serial.printf("   %s Tracker Starting...\n", kMyName);
  Serial.println("=================================\n");

  // -------- Heltec board init (REQUIRED or radio DEAD) --------
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // -------- Power Rail (VEXT = sensors + TFT) -----------------
  pinMode(Pins::VEXT_CTRL, OUTPUT);
  digitalWrite(Pins::VEXT_CTRL, LOW);   // LOW = ON for Heltec VEXT
  delay(80);

  // -------- Buttons -------------------------------------------
  pinMode(Pins::BUTTON_LEFT, INPUT_PULLUP);
  pinMode(Pins::BUTTON_CENTER, INPUT_PULLUP);
  pinMode(Pins::BUTTON_RIGHT, INPUT_PULLUP);

  // -------- I2C Init ------------------------------------------
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(400000);
  Serial.printf("[I2C] SDA=%d  SCL=%d\n", Pins::I2C_SDA, Pins::I2C_SCL);

  // -------- TFT SPI Init --------------------------------------
  tftSpi.begin(Pins::TFT_SCK, Pins::TFT_MISO, Pins::TFT_MOSI, Pins::TFT_CS);
  tft.begin(40000000UL);
  tft.setRotation(2);  // flipped 180°
  tft.fillScreen(ILI9341_BLACK);

  // Backlight via LEDC PWM
  ledcAttach(Pins::TFT_BACKLIGHT, 5000, 8);
  ledcWrite(Pins::TFT_BACKLIGHT, 220);  // brightness 0–255

  // -------- Radio Init ----------------------------------------
  setupRadio();

  // -------- Draw initial static UI elements --------------------
  drawTopBar();
  drawNameBand();
  drawStats();

  Serial.println("[Init] Device ready.\n");
}


// ======================== LOOP ================================
void loop() {
  uint32_t now = millis();

  // ---------- Sensor update (10 Hz) ----------
  if (now - lastSensorUpdate > 100) {
    lastSensorUpdate = now;
    updateSensors();
  }

  // ---------- Buttons / power ----------
  handleButtons();
  if (state.sleeping) return;

  // ---------- LoRa update (2 Hz) ----------
  if (nextTxTime == 0) {
    nextTxTime = now + kTxOffsetMs;  // stagger first TX between devices
  }

  if ((int32_t)(now - nextTxTime) >= 0) {
    sendUpdate();
    nextTxTime += kUpdateIntervalMs;
  }

  // ---------- Peer timeout ----------
  if (state.peerActive && (now - state.peerLastSeen > kPeerTimeoutMs)) {
    state.peerActive = false;
    state.distance   = NAN;
    state.altDelta   = NAN;
  }

  // ---------- Radio services ----------
  // MUST run frequently or RX/TX stalls
  Radio.IrqProcess();

  // ---------- UI draw ----------
  drawTopBar();
  drawCompass();
  drawStats();
}
