/*
 * Full Stack I/O Validation Sketch
 * --------------------------------
 * Drives all peripherals on the Heltec WiFi LoRa 32 V3 simultaneously:
 *   - LoRa radio (SX1262) send/receive heartbeat packets
 *   - SparkFun MAX-M10S GNSS over I2C (position + satellites)
 *   - Adafruit BMP390 pressure/temperature sensor over I2C
 *   - Adafruit BNO055 IMU over I2C (orientation + calibration)
 *   - ILI9341 TFT display over hardware SPI (status dashboard)
 *   - Three external buttons on GPIO40-42 (dim, buzzer pulse, brighten)
 *   - TFT backlight PWM control on GPIO2 with on-screen brightness level
 *   - Piezo buzzer on GPIO39 for audible confirmation
 *
 * Use this sketch to confirm that all subsystems can run concurrently
 * without blocking each other. Pair it with the companion sketch located
 * in `code/io_tests/full_stack_remote/full_stack_remote.ino` on a second
 * Heltec board to validate the LoRa link.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "LoRaWan_APP.h"

// ---------------------- Pin & Interface Definitions ----------------------
#define I2C_SDA_PIN 48
#define I2C_SCL_PIN 47

#define TFT_CS 7
#define TFT_RST 6
#define TFT_DC 5
#define TFT_MOSI 4
#define TFT_SCK 3
#define TFT_MISO -1
#define TFT_SPI_FREQUENCY 40000000UL

#define BUTTON_DIM 40     // External button: dim TFT backlight
#define BUTTON_BUZZ 41    // External button: trigger buzzer pulse
#define BUTTON_BRIGHT 42  // External button: brighten TFT backlight

#define TFT_BACKLIGHT_PIN 2  // PWM-capable pin connected to TFT backlight
#define BUZZER_PIN 39        // Piezo buzzer control line

#define VGNSS_CTRL 3    // Power control for GNSS module

const uint8_t BACKLIGHT_CHANNEL = 0;
const uint8_t BUZZER_CHANNEL = 1;
const uint32_t BACKLIGHT_PWM_FREQ = 5000;
const uint8_t BACKLIGHT_PWM_RESOLUTION = 8;   // 0-255 duty cycle
const uint32_t BUZZER_PWM_FREQ = 2400;
const uint8_t BUZZER_PWM_RESOLUTION = 10;     // 0-1023 duty cycle
const uint32_t BUZZER_ACTIVE_DUTY = 1 << (BUZZER_PWM_RESOLUTION - 1);
const unsigned long BUZZER_PULSE_MS = 200;

const uint8_t kBrightnessSteps[] = {12, 32, 56, 80, 112, 144, 184, 224, 255};
const size_t kBrightnessStepCount = sizeof(kBrightnessSteps) / sizeof(kBrightnessSteps[0]);

// --------------------------- LoRa Configuration --------------------------
#define RF_FREQUENCY 915000000UL
#define TX_OUTPUT_POWER 14
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 160

const unsigned long LORA_TX_INTERVAL_MS = 5000;  // Periodic heartbeat interval

// ------------------------------ Sensor Objects ---------------------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SFE_UBLOX_GNSS myGNSS;

bool bmpReady = false;
bool bnoReady = false;
bool gpsReady = false;

// ------------------------------ Data Models ------------------------------
struct SensorSnapshot {
  bool gpsFix = false;
  uint8_t gpsFixType = 0;
  uint8_t satellites = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  float altitudeM = 0.0f;
  float groundSpeedMps = 0.0f;
  unsigned long lastFixMs = 0;

  float temperatureC = NAN;
  float pressureHpa = NAN;

  float imuHeadingDeg = NAN;
  float imuRollDeg = NAN;
  float imuPitchDeg = NAN;
  uint8_t imuCalSys = 0;
  uint8_t imuCalGyro = 0;
  uint8_t imuCalAccel = 0;
  uint8_t imuCalMag = 0;

  bool buttonDimPressed = false;
  bool buttonBuzzPressed = false;
  bool buttonBrightPressed = false;
  uint8_t displayBrightnessPercent = 0;
  bool buzzerPlaying = false;
};

SensorSnapshot snapshot;

struct LoRaStatus {
  uint32_t txCount = 0;
  uint32_t rxCount = 0;
  unsigned long lastTxMillis = 0;
  unsigned long lastRxMillis = 0;
  int16_t lastRssi = 0;
  bool lastMessageValid = false;
  char lastMessage[BUFFER_SIZE] = {0};
};

LoRaStatus loraStatus;

struct LayoutGeometry {
  uint16_t marginLeft = 0;
  uint16_t contentWidth = 0;
  uint16_t screenWidth = 0;
  uint16_t screenHeight = 0;
  uint16_t gnssTop = 0;
  uint16_t gnssHeight = 0;
  uint16_t imuTop = 0;
  uint16_t imuHeight = 0;
  uint16_t buttonsTop = 0;
  uint16_t buttonsHeight = 0;
  uint16_t loraTop = 0;
  uint16_t loraHeight = 0;
};

LayoutGeometry layout;

size_t brightnessIndex = kBrightnessStepCount > 0 ? kBrightnessStepCount - 2 : 0;
uint8_t currentBacklightDuty = 0;
bool buzzerPlaying = false;
unsigned long buzzerOffDeadline = 0;

// ------------------------------- LoRa State -------------------------------
static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

typedef enum {
  LOWPOWER,
  STATE_RX,
  STATE_TX
} States_t;

States_t state = STATE_RX;
bool pendingTx = true;  // Kick off an initial transmission after startup
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

// --------------------------- Timing Bookkeeping --------------------------
unsigned long lastGpsPoll = 0;
unsigned long lastImuPoll = 0;
unsigned long lastPressurePoll = 0;
unsigned long lastButtonPoll = 0;
unsigned long lastDisplayUpdate = 0;

const unsigned long GPS_POLL_INTERVAL_MS = 250;
const unsigned long IMU_POLL_INTERVAL_MS = 100;
const unsigned long PRESSURE_POLL_INTERVAL_MS = 500;
const unsigned long BUTTON_POLL_INTERVAL_MS = 50;
const unsigned long DISPLAY_REFRESH_MS = 500;

// ------------------------------- Prototypes ------------------------------
void initDisplay();
void drawStaticLayout();
void updateDisplay();
void initBacklight();
void initBuzzer();
void applyBacklightDuty();
bool adjustBrightness(int8_t delta);
void triggerBuzzerPulse();
void serviceBuzzer();
void updateGps();
void updateImu();
void updatePressure();
void updateButtons();
void processLoRa();
void scheduleLoRaTx();
void buildLoRaPacket();
void configureGNSS();

// -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Full Stack I/O Validation ===\n");

  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);  // Enable GNSS power rail

  pinMode(BUTTON_DIM, INPUT_PULLUP);
  pinMode(BUTTON_BUZZ, INPUT_PULLUP);
  pinMode(BUTTON_BRIGHT, INPUT_PULLUP);

  initBacklight();
  initBuzzer();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  initDisplay();

  Serial.println("Initializing BMP390...");
  if (bmp.begin_I2C()) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    bmpReady = true;
    Serial.println("  ✓ BMP390 ready\n");
  } else {
    Serial.println("  ✗ BMP390 not detected\n");
  }

  Serial.println("Initializing BNO055...");
  if (bno.begin()) {
    delay(1000);
    bno.setExtCrystalUse(true);
    bnoReady = true;
    Serial.println("  ✓ BNO055 ready\n");
  } else {
    Serial.println("  ✗ BNO055 not detected\n");
  }

  Serial.println("Initializing MAX-M10S GNSS...");
  if (myGNSS.begin(Wire)) {
    configureGNSS();
    gpsReady = true;
    Serial.println("  ✓ GNSS ready\n");
  } else {
    Serial.println("  ✗ GNSS not detected\n");
  }

  Serial.println("Initializing LoRa radio...\n");
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

  state = STATE_RX;  // Start in RX so we can hear the remote board immediately
  drawStaticLayout();
}

void loop() {
  const unsigned long now = millis();

  if (now - lastGpsPoll >= GPS_POLL_INTERVAL_MS) {
    updateGps();
    lastGpsPoll = now;
  }

  if (now - lastImuPoll >= IMU_POLL_INTERVAL_MS) {
    updateImu();
    lastImuPoll = now;
  }

  if (now - lastPressurePoll >= PRESSURE_POLL_INTERVAL_MS) {
    updatePressure();
    lastPressurePoll = now;
  }

  if (now - lastButtonPoll >= BUTTON_POLL_INTERVAL_MS) {
    updateButtons();
    lastButtonPoll = now;
  }

  if (now - lastDisplayUpdate >= DISPLAY_REFRESH_MS) {
    updateDisplay();
    lastDisplayUpdate = now;
  }

  serviceBuzzer();

  scheduleLoRaTx();
  processLoRa();
}

// ------------------------------ Initialization ---------------------------
void initDisplay() {
  tft.begin(TFT_SPI_FREQUENCY);
  tft.setRotation(0);  // Portrait orientation for 240x360 layout
  tft.setTextWrap(false);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(12, 24);
  tft.println("Heltec Full I/O Test");
  tft.setTextSize(1);
  tft.setCursor(12, 52);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("Initializing peripherals...");
}

void drawStaticLayout() {
  layout.screenWidth = tft.width();
  layout.screenHeight = tft.height();
  layout.marginLeft = 10;
  layout.contentWidth = layout.screenWidth > 2 * layout.marginLeft
                             ? layout.screenWidth - (2 * layout.marginLeft)
                             : layout.screenWidth;

  const uint16_t topMargin = 8;
  const uint16_t bottomMargin = 8;
  const uint16_t headerHeight = 32;
  const uint16_t sectionGap = 6;

  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(layout.marginLeft, topMargin);
  tft.println("FULL STACK");
  tft.setCursor(layout.marginLeft, topMargin + 18);
  tft.println("I/O TEST");
  tft.setTextSize(1);
  tft.drawFastHLine(layout.marginLeft, topMargin + headerHeight, layout.contentWidth,
                    ILI9341_DARKGREY);

  uint16_t availableHeight = 0;
  if (layout.screenHeight > (topMargin + headerHeight + bottomMargin + sectionGap * 3)) {
    availableHeight = layout.screenHeight -
                      (topMargin + headerHeight + bottomMargin + sectionGap * 3);
  }

  uint16_t sectionHeights[4] = {0, 0, 0, 0};
  const uint16_t minHeights[4] = {72, 80, 54, 60};
  const uint16_t minSum = minHeights[0] + minHeights[1] + minHeights[2] + minHeights[3];

  if (availableHeight == 0) {
    // No vertical real estate beyond the header; leave sections collapsed.
  } else if (availableHeight < minSum) {
    float scale = minSum > 0 ? static_cast<float>(availableHeight) / static_cast<float>(minSum)
                             : 0.0f;
    if (scale < 0.2f) {
      scale = 0.2f;
    }
    uint16_t total = 0;
    for (size_t i = 0; i < 4; ++i) {
      sectionHeights[i] = static_cast<uint16_t>(minHeights[i] * scale);
      if (sectionHeights[i] < 18) {
        sectionHeights[i] = 18;
      }
      total += sectionHeights[i];
    }
    if (total > availableHeight) {
      uint16_t over = total - availableHeight;
      while (over > 0) {
        bool reduced = false;
        for (size_t i = 0; i < 4 && over > 0; ++i) {
          if (sectionHeights[i] > 18) {
            sectionHeights[i]--;
            --over;
            reduced = true;
          }
        }
        if (!reduced) {
          break;
        }
      }
    }
    total = sectionHeights[0] + sectionHeights[1] + sectionHeights[2] + sectionHeights[3];
    if (total < availableHeight) {
      sectionHeights[3] += availableHeight - total;
    }
  } else {
    sectionHeights[0] = (availableHeight * 30) / 100;
    sectionHeights[1] = (availableHeight * 32) / 100;
    sectionHeights[2] = (availableHeight * 18) / 100;
    sectionHeights[3] = availableHeight -
                        (sectionHeights[0] + sectionHeights[1] + sectionHeights[2]);

    uint16_t total = 0;
    for (size_t i = 0; i < 4; ++i) {
      if (sectionHeights[i] < minHeights[i]) {
        sectionHeights[i] = minHeights[i];
      }
      total += sectionHeights[i];
    }

    if (total > availableHeight) {
      uint16_t over = total - availableHeight;
      while (over > 0) {
        size_t largestIndex = 0;
        uint16_t largestSpare = 0;
        for (size_t i = 0; i < 4; ++i) {
          if (sectionHeights[i] > minHeights[i]) {
            uint16_t spare = sectionHeights[i] - minHeights[i];
            if (spare > largestSpare) {
              largestSpare = spare;
              largestIndex = i;
            }
          }
        }
        if (largestSpare == 0) {
          break;
        }
        sectionHeights[largestIndex]--;
        --over;
      }
    }

    uint16_t recalculatedTotal = sectionHeights[0] + sectionHeights[1] + sectionHeights[2] +
                                 sectionHeights[3];
    if (recalculatedTotal < availableHeight) {
      sectionHeights[3] += availableHeight - recalculatedTotal;
    }
  }

  layout.gnssTop = topMargin + headerHeight + 4;
  layout.gnssHeight = sectionHeights[0];
  layout.imuTop = layout.gnssTop + layout.gnssHeight + sectionGap;
  layout.imuHeight = sectionHeights[1];
  layout.buttonsTop = layout.imuTop + layout.imuHeight + sectionGap;
  layout.buttonsHeight = sectionHeights[2];
  layout.loraTop = layout.buttonsTop + layout.buttonsHeight + sectionGap;
  layout.loraHeight = sectionHeights[3];

  const uint16_t boxLeft = layout.marginLeft - 2;
  const uint16_t boxWidth = layout.contentWidth + 4;

  tft.setTextColor(ILI9341_YELLOW);
  if (layout.gnssHeight > 8) {
    tft.drawRoundRect(boxLeft, layout.gnssTop - 4, boxWidth, layout.gnssHeight, 6,
                      ILI9341_DARKGREY);
    tft.setCursor(layout.marginLeft, layout.gnssTop + 4);
    tft.print("GNSS");
  }

  if (layout.imuHeight > 8) {
    tft.drawRoundRect(boxLeft, layout.imuTop - 4, boxWidth, layout.imuHeight, 6,
                      ILI9341_DARKGREY);
    tft.setCursor(layout.marginLeft, layout.imuTop + 4);
    tft.print("IMU & ENV");
  }

  if (layout.buttonsHeight > 8) {
    tft.drawRoundRect(boxLeft, layout.buttonsTop - 4, boxWidth, layout.buttonsHeight, 6,
                      ILI9341_DARKGREY);
    tft.setCursor(layout.marginLeft, layout.buttonsTop + 4);
    tft.print("CONTROLS");
  }

  if (layout.loraHeight > 8) {
    tft.drawRoundRect(boxLeft, layout.loraTop - 4, boxWidth, layout.loraHeight, 6,
                      ILI9341_DARKGREY);
    tft.setCursor(layout.marginLeft, layout.loraTop + 4);
    tft.print("LORA LINK");
  }

  tft.setTextColor(ILI9341_WHITE);
}

void initBacklight() {
  pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
  ledcSetup(BACKLIGHT_CHANNEL, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_RESOLUTION);
  ledcAttachPin(TFT_BACKLIGHT_PIN, BACKLIGHT_CHANNEL);

  if (kBrightnessStepCount == 0) {
    currentBacklightDuty = 0;
    ledcWrite(BACKLIGHT_CHANNEL, currentBacklightDuty);
    snapshot.displayBrightnessPercent = 0;
    return;
  }

  if (brightnessIndex >= kBrightnessStepCount) {
    brightnessIndex = kBrightnessStepCount - 1;
  }

  applyBacklightDuty();
}

void initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, BUZZER_PWM_FREQ, BUZZER_PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0);
  buzzerPlaying = false;
  snapshot.buzzerPlaying = false;
}

void applyBacklightDuty() {
  if (kBrightnessStepCount == 0) {
    currentBacklightDuty = 0;
    ledcWrite(BACKLIGHT_CHANNEL, 0);
    snapshot.displayBrightnessPercent = 0;
    return;
  }

  if (brightnessIndex >= kBrightnessStepCount) {
    brightnessIndex = kBrightnessStepCount - 1;
  }

  currentBacklightDuty = kBrightnessSteps[brightnessIndex];
  ledcWrite(BACKLIGHT_CHANNEL, currentBacklightDuty);

  snapshot.displayBrightnessPercent =
      static_cast<uint8_t>((static_cast<uint32_t>(currentBacklightDuty) * 100U + 127U) / 255U);
}

bool adjustBrightness(int8_t delta) {
  if (kBrightnessStepCount == 0) {
    return false;
  }

  int32_t candidate = static_cast<int32_t>(brightnessIndex) + delta;
  if (candidate < 0) {
    candidate = 0;
  } else if (candidate >= static_cast<int32_t>(kBrightnessStepCount)) {
    candidate = static_cast<int32_t>(kBrightnessStepCount) - 1;
  }

  if (static_cast<size_t>(candidate) != brightnessIndex) {
    brightnessIndex = static_cast<size_t>(candidate);
    applyBacklightDuty();
    return true;
  }

  return false;
}

void triggerBuzzerPulse() {
  const unsigned long now = millis();
  buzzerOffDeadline = now + BUZZER_PULSE_MS;
  ledcWrite(BUZZER_CHANNEL, BUZZER_ACTIVE_DUTY);
  buzzerPlaying = true;
  snapshot.buzzerPlaying = true;
}

void serviceBuzzer() {
  if (buzzerPlaying && millis() >= buzzerOffDeadline) {
    ledcWrite(BUZZER_CHANNEL, 0);
    buzzerPlaying = false;
    snapshot.buzzerPlaying = false;
  }

  if (!buzzerPlaying) {
    snapshot.buzzerPlaying = false;
  }
}

// ------------------------------ Sensor Polling ---------------------------
void updateGps() {
  if (!gpsReady) {
    snapshot.gpsFix = false;
    return;
  }

  if (!myGNSS.getPVT()) {
    return;  // Keep previous readings if the module didn't provide new data
  }

  snapshot.gpsFixType = myGNSS.getFixType();
  snapshot.satellites = myGNSS.getSIV();
  snapshot.gpsFix = snapshot.gpsFixType >= 2;

  if (snapshot.gpsFix) {
    snapshot.latitude = myGNSS.getLatitude() / 10000000.0;
    snapshot.longitude = myGNSS.getLongitude() / 10000000.0;
    snapshot.altitudeM = myGNSS.getAltitudeMSL() / 1000.0f;
    snapshot.groundSpeedMps = myGNSS.getGroundSpeed() / 1000.0f;
    snapshot.lastFixMs = millis();
  }
}

void updateImu() {
  if (!bnoReady) {
    snapshot.imuHeadingDeg = snapshot.imuRollDeg = snapshot.imuPitchDeg = NAN;
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);
  snapshot.imuHeadingDeg = event.orientation.x;
  snapshot.imuRollDeg = event.orientation.y;
  snapshot.imuPitchDeg = event.orientation.z;

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  snapshot.imuCalSys = sys;
  snapshot.imuCalGyro = gyro;
  snapshot.imuCalAccel = accel;
  snapshot.imuCalMag = mag;
}

void updatePressure() {
  if (!bmpReady) {
    snapshot.temperatureC = snapshot.pressureHpa = NAN;
    return;
  }

  if (bmp.performReading()) {
    snapshot.temperatureC = bmp.temperature;
    snapshot.pressureHpa = bmp.pressure / 100.0f;
  }
}

void updateButtons() {
  static bool lastDim = false;
  static bool lastBuzz = false;
  static bool lastBright = false;

  const bool dimPressed = digitalRead(BUTTON_DIM) == LOW;
  const bool buzzPressed = digitalRead(BUTTON_BUZZ) == LOW;
  const bool brightPressed = digitalRead(BUTTON_BRIGHT) == LOW;

  snapshot.buttonDimPressed = dimPressed;
  snapshot.buttonBuzzPressed = buzzPressed;
  snapshot.buttonBrightPressed = brightPressed;

  if (dimPressed != lastDim) {
    Serial.printf("DIM button %s\n", dimPressed ? "pressed" : "released");
    if (dimPressed) {
      const bool changed = adjustBrightness(-1);
      if (changed) {
        Serial.printf("  -> Brightness reduced to %u%%\n", snapshot.displayBrightnessPercent);
      } else {
        Serial.println("  -> Backlight already at minimum");
      }
    }
    lastDim = dimPressed;
  }

  if (brightPressed != lastBright) {
    Serial.printf("BRIGHT button %s\n", brightPressed ? "pressed" : "released");
    if (brightPressed) {
      const bool changed = adjustBrightness(1);
      if (changed) {
        Serial.printf("  -> Brightness increased to %u%%\n", snapshot.displayBrightnessPercent);
      } else {
        Serial.println("  -> Backlight already at maximum");
      }
    }
    lastBright = brightPressed;
  }

  if (buzzPressed != lastBuzz) {
    Serial.printf("BUZZ button %s\n", buzzPressed ? "pressed" : "released");
    if (buzzPressed) {
      triggerBuzzerPulse();
      Serial.println("  -> Buzzer pulse triggered");
    }
    lastBuzz = buzzPressed;
  }
}

// ----------------------------- Display Output ----------------------------
void updateDisplay() {
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  const uint16_t contentLeft = layout.marginLeft + 2;
  const uint16_t contentWidth = layout.contentWidth > 4 ? layout.contentWidth - 4 : layout.contentWidth;
  const uint16_t contentTopOffset = 16;
  const uint16_t contentBottomPadding = 6;
  const uint8_t lineHeight = 12;
  const uint32_t now = millis();
  char line[96];

  if (layout.gnssHeight > contentTopOffset + contentBottomPadding) {
    const uint16_t dataTop = layout.gnssTop + contentTopOffset;
    const uint16_t dataHeight = layout.gnssHeight - (contentTopOffset + contentBottomPadding);
    tft.fillRect(contentLeft, dataTop, contentWidth, dataHeight, ILI9341_BLACK);

    uint16_t cursorY = dataTop;
    tft.setCursor(contentLeft, cursorY);
    if (snapshot.gpsFix) {
      snprintf(line, sizeof(line), "Fix:%u Sat:%u Age:%lus", snapshot.gpsFixType, snapshot.satellites,
               (now - snapshot.lastFixMs) / 1000UL);
      tft.print(line);
      cursorY += lineHeight;

      tft.setCursor(contentLeft, cursorY);
      snprintf(line, sizeof(line), "Lat: %.5f", snapshot.latitude);
      tft.print(line);
      cursorY += lineHeight;

      tft.setCursor(contentLeft, cursorY);
      snprintf(line, sizeof(line), "Lon: %.5f", snapshot.longitude);
      tft.print(line);
      cursorY += lineHeight;

      if (cursorY < dataTop + dataHeight) {
        tft.setCursor(contentLeft, cursorY);
        snprintf(line, sizeof(line), "Alt: %.1fm Spd: %.1fm/s", snapshot.altitudeM, snapshot.groundSpeedMps);
        tft.print(line);
      }
    } else {
      snprintf(line, sizeof(line), "No fix (type %u) Sat:%u", snapshot.gpsFixType, snapshot.satellites);
      tft.print(line);
      cursorY += lineHeight;
      if (cursorY < dataTop + dataHeight) {
        tft.setCursor(contentLeft, cursorY);
        tft.print("Waiting for valid position...");
      }
    }
  }

  if (layout.imuHeight > contentTopOffset + contentBottomPadding) {
    const uint16_t dataTop = layout.imuTop + contentTopOffset;
    const uint16_t dataHeight = layout.imuHeight - (contentTopOffset + contentBottomPadding);
    tft.fillRect(contentLeft, dataTop, contentWidth, dataHeight, ILI9341_BLACK);

    uint16_t cursorY = dataTop;
    tft.setCursor(contentLeft, cursorY);
    if (bnoReady && !isnan(snapshot.imuHeadingDeg)) {
      snprintf(line, sizeof(line), "Head:%5.1f Roll:%5.1f", snapshot.imuHeadingDeg, snapshot.imuRollDeg);
      tft.print(line);
      cursorY += lineHeight;

      if (cursorY < dataTop + dataHeight) {
        tft.setCursor(contentLeft, cursorY);
        snprintf(line, sizeof(line), "Pitch:%5.1f", snapshot.imuPitchDeg);
        tft.print(line);
        cursorY += lineHeight;
      }
    } else {
      tft.print("IMU unavailable");
      cursorY += lineHeight;
    }

    if (cursorY < dataTop + dataHeight) {
      tft.setCursor(contentLeft, cursorY);
      if (bnoReady) {
        snprintf(line, sizeof(line), "Cal S:%u G:%u A:%u M:%u", snapshot.imuCalSys, snapshot.imuCalGyro,
                 snapshot.imuCalAccel, snapshot.imuCalMag);
        tft.print(line);
      } else {
        tft.print("Calibration data N/A");
      }
      cursorY += lineHeight;
    }

    if (cursorY < dataTop + dataHeight) {
      tft.setCursor(contentLeft, cursorY);
      if (bmpReady && !isnan(snapshot.temperatureC)) {
        snprintf(line, sizeof(line), "Temp:%5.1fC Pres:%7.2fhPa", snapshot.temperatureC, snapshot.pressureHpa);
        tft.print(line);
      } else {
        tft.print("BMP390 unavailable");
      }
    }
  }

  if (layout.buttonsHeight > contentTopOffset + contentBottomPadding) {
    const uint16_t dataTop = layout.buttonsTop + contentTopOffset;
    const uint16_t dataHeight = layout.buttonsHeight - (contentTopOffset + contentBottomPadding);
    tft.fillRect(contentLeft, dataTop, contentWidth, dataHeight, ILI9341_BLACK);

    uint16_t cursorY = dataTop;
    tft.setCursor(contentLeft, cursorY);
    snprintf(line, sizeof(line), "Dim:%s Buzz:%s Bright:%s",
             snapshot.buttonDimPressed ? "DOWN" : "UP  ",
             snapshot.buttonBuzzPressed ? "DOWN" : "UP  ",
             snapshot.buttonBrightPressed ? "DOWN" : "UP  ");
    tft.print(line);
    cursorY += lineHeight;

    if (cursorY < dataTop + dataHeight) {
      tft.setCursor(contentLeft, cursorY);
      const unsigned int brightnessLevel =
          kBrightnessStepCount > 0 ? static_cast<unsigned int>(brightnessIndex + 1) : 0;
      const unsigned int brightnessTotal = static_cast<unsigned int>(kBrightnessStepCount);
      snprintf(line, sizeof(line), "Brightness: %3u%% (%u/%u)", snapshot.displayBrightnessPercent,
               brightnessLevel, brightnessTotal);
      tft.print(line);
      cursorY += lineHeight;
    }

    if (cursorY < dataTop + dataHeight) {
      tft.setCursor(contentLeft, cursorY);
      tft.print(snapshot.buzzerPlaying ? "Buzzer: PLAYING" : "Buzzer: idle");
    }
  }

  if (layout.loraHeight > contentTopOffset + contentBottomPadding) {
    const uint16_t dataTop = layout.loraTop + contentTopOffset;
    const uint16_t dataHeight = layout.loraHeight - (contentTopOffset + contentBottomPadding);
    tft.fillRect(contentLeft, dataTop, contentWidth, dataHeight, ILI9341_BLACK);

    uint16_t cursorY = dataTop;
    tft.setCursor(contentLeft, cursorY);
    snprintf(line, sizeof(line), "TX:%lu RX:%lu RSSI:%d", static_cast<unsigned long>(loraStatus.txCount),
             static_cast<unsigned long>(loraStatus.rxCount), loraStatus.lastRssi);
    tft.print(line);
    cursorY += lineHeight;

    if (cursorY < dataTop + dataHeight) {
      tft.setCursor(contentLeft, cursorY);
      if (loraStatus.lastRxMillis > 0) {
        snprintf(line, sizeof(line), "Last RX: %lus ago", (now - loraStatus.lastRxMillis) / 1000UL);
      } else {
        snprintf(line, sizeof(line), "Last RX: --");
      }
      tft.print(line);
      cursorY += lineHeight;
    }

    if (cursorY < dataTop + dataHeight) {
      tft.setCursor(contentLeft, cursorY);
      snprintf(line, sizeof(line), "Last TX: %lus ago", (now - loraStatus.lastTxMillis) / 1000UL);
      tft.print(line);
      cursorY += lineHeight;
    }

    if (cursorY < dataTop + dataHeight) {
      const uint16_t msgTop = cursorY;
      const uint16_t msgAreaHeight = dataTop + dataHeight - msgTop;
      const uint8_t availableLines = msgAreaHeight / lineHeight;

      if (availableLines > 0) {
        if (loraStatus.lastMessageValid && loraStatus.lastMessage[0] != '\0') {
          const char *message = loraStatus.lastMessage;
          size_t remaining = strlen(message);
          size_t offset = 0;
          const uint8_t maxLineChars = contentWidth > 0 ? contentWidth / 6 : 0;

          for (uint8_t lineIndex = 0; lineIndex < availableLines; ++lineIndex) {
            const bool firstLine = lineIndex == 0;
            const uint16_t yPos = msgTop + lineIndex * lineHeight;
            tft.setCursor(contentLeft, yPos);

            uint8_t charsThisLine = maxLineChars > 4 ? maxLineChars : 24;
            if (charsThisLine > sizeof(line) - 1) {
              charsThisLine = sizeof(line) - 1;
            }

            if (remaining <= charsThisLine) {
              strncpy(line, message + offset, remaining);
              line[remaining] = '\0';
              if (firstLine) {
                tft.print("Msg: ");
                tft.print(line);
              } else {
                tft.print("      ");
                tft.print(line);
              }
              break;
            }

            if (lineIndex == availableLines - 1) {
              if (firstLine) {
                tft.print("Msg: ...");
              } else {
                tft.print("      ...");
              }
              break;
            }

            strncpy(line, message + offset, charsThisLine);
            line[charsThisLine] = '\0';
            if (firstLine) {
              tft.print("Msg: ");
              tft.print(line);
            } else {
              tft.print("      ");
              tft.print(line);
            }
            offset += charsThisLine;
            remaining -= charsThisLine;
          }
        } else {
          tft.setCursor(contentLeft, msgTop);
          tft.print("Msg: Waiting for remote heartbeat...");
        }
      }
    }
  }
}

// --------------------------- LoRa Infrastructure -------------------------
void scheduleLoRaTx() {
  if (millis() - loraStatus.lastTxMillis >= LORA_TX_INTERVAL_MS) {
    pendingTx = true;
  }
}

void processLoRa() {
  if (state == LOWPOWER && pendingTx) {
    state = STATE_TX;
  }

  switch (state) {
    case STATE_TX:
      buildLoRaPacket();
      Radio.Send(reinterpret_cast<uint8_t *>(txpacket), strlen(txpacket));
      loraStatus.lastTxMillis = millis();
      loraStatus.txCount++;
      Serial.printf("LoRa TX (%lu): %s\n", static_cast<unsigned long>(loraStatus.txCount), txpacket);
      pendingTx = false;
      state = LOWPOWER;
      break;

    case STATE_RX:
      Radio.Rx(0);
      state = LOWPOWER;
      break;

    case LOWPOWER:
    default:
      Radio.IrqProcess();
      break;
  }
}

void buildLoRaPacket() {
  snprintf(txpacket, sizeof(txpacket),
           "FIX:%d,FIXTYPE:%u,SAT:%u,LAT:%.6f,LON:%.6f,ALT:%.1f,SPD:%.1f,TEMP:%.1f,PRES:%.1f,HEAD:%.1f,ROLL:%.1f,PITCH:%.1f,BTN:%d%d%d,BRI:%u,BUZ:%d",
           snapshot.gpsFix ? 1 : 0,
           snapshot.gpsFixType,
           snapshot.satellites,
           snapshot.latitude,
           snapshot.longitude,
           snapshot.altitudeM,
           snapshot.groundSpeedMps,
           snapshot.temperatureC,
           snapshot.pressureHpa,
           snapshot.imuHeadingDeg,
           snapshot.imuRollDeg,
           snapshot.imuPitchDeg,
           snapshot.buttonDimPressed ? 1 : 0,
           snapshot.buttonBuzzPressed ? 1 : 0,
           snapshot.buttonBrightPressed ? 1 : 0,
           snapshot.displayBrightnessPercent,
           snapshot.buzzerPlaying ? 1 : 0);
}

void OnTxDone(void) {
  Serial.println("LoRa TX complete");
  state = STATE_RX;
}

void OnTxTimeout(void) {
  Serial.println("LoRa TX timeout - will retry");
  pendingTx = true;
  state = STATE_TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();

  loraStatus.rxCount++;
  loraStatus.lastRssi = rssi;
  loraStatus.lastRxMillis = millis();
  loraStatus.lastMessageValid = true;
  strncpy(loraStatus.lastMessage, rxpacket, sizeof(loraStatus.lastMessage) - 1);
  loraStatus.lastMessage[sizeof(loraStatus.lastMessage) - 1] = '\0';

  Serial.printf("LoRa RX (%lu) RSSI:%d -> %s\n", static_cast<unsigned long>(loraStatus.rxCount), rssi, rxpacket);

  // Schedule a prompt response so the remote board sees activity
  pendingTx = true;
  state = STATE_RX;
}

// ------------------------------- GNSS Setup ------------------------------
void configureGNSS() {
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setAutoPVT(false);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
  myGNSS.setNavigationFrequency(1);
  myGNSS.setDynamicModel(DYN_MODEL_PORTABLE);
}
