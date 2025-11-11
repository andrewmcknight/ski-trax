/*
 * Full Stack I/O Validation Sketch
 * --------------------------------
 * Drives all peripherals on the Heltec WiFi LoRa 32 V3 simultaneously:
 *   - LoRa radio (SX1262) send/receive heartbeat packets
 *   - SparkFun MAX-M10S GNSS over I2C (position + satellites)
 *   - Adafruit BMP390 pressure/temperature sensor over I2C
 *   - Adafruit BNO055 IMU over I2C (orientation + calibration)
 *   - ILI9341 TFT display over hardware SPI (status dashboard)
 *   - Two onboard buttons (BOOT and USER) with live state reporting
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

#define BUTTON_BOOT 0   // BOOT button on Heltec WiFi LoRa 32 V3
#define BUTTON_USER 14  // User button (GPIO14) on Heltec WiFi LoRa 32 V3

#define VGNSS_CTRL 3    // Power control for GNSS module

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

  bool buttonBootPressed = false;
  bool buttonUserPressed = false;
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

  pinMode(BUTTON_BOOT, INPUT_PULLUP);
  pinMode(BUTTON_USER, INPUT_PULLUP);

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

  scheduleLoRaTx();
  processLoRa();
}

// ------------------------------ Initialization ---------------------------
void initDisplay() {
  tft.begin(TFT_SPI_FREQUENCY);
  tft.setRotation(1);  // Landscape orientation
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(20, 40);
  tft.println("Heltec Full I/O Test");
  tft.setTextSize(1);
  tft.setCursor(20, 70);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("Initializing peripherals...");
}

void drawStaticLayout() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.println("FULL STACK I/O TEST");

  tft.drawFastHLine(0, 30, 320, ILI9341_DARKGREY);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(8, 38);
  tft.println("GNSS");

  tft.setCursor(8, 98);
  tft.println("IMU / ENV");

  tft.setCursor(8, 168);
  tft.println("BUTTONS");

  tft.setCursor(8, 208);
  tft.println("LORA LINK");

  tft.drawRoundRect(4, 50, 312, 44, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(4, 110, 312, 52, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(4, 178, 312, 24, 4, ILI9341_DARKGREY);
  tft.drawRoundRect(4, 214, 312, 102, 4, ILI9341_DARKGREY);
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
  static bool lastBoot = false;
  static bool lastUser = false;

  snapshot.buttonBootPressed = digitalRead(BUTTON_BOOT) == LOW;
  snapshot.buttonUserPressed = digitalRead(BUTTON_USER) == LOW;

  if (snapshot.buttonBootPressed != lastBoot) {
    Serial.printf("BOOT button %s\n", snapshot.buttonBootPressed ? "pressed" : "released");
    lastBoot = snapshot.buttonBootPressed;
  }

  if (snapshot.buttonUserPressed != lastUser) {
    Serial.printf("USER button %s\n", snapshot.buttonUserPressed ? "pressed" : "released");
    lastUser = snapshot.buttonUserPressed;
  }
}

// ----------------------------- Display Output ----------------------------
void updateDisplay() {
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  // GNSS Section
  char line[64];
  tft.setCursor(12, 58);
  if (snapshot.gpsFix) {
    snprintf(line, sizeof(line), "Fix:%u Sat:%u Age:%lus", snapshot.gpsFixType, snapshot.satellites,
             (millis() - snapshot.lastFixMs) / 1000UL);
    tft.print(line);

    tft.setCursor(12, 70);
    snprintf(line, sizeof(line), "Lat: %.5f", snapshot.latitude);
    tft.print(line);

    tft.setCursor(160, 70);
    snprintf(line, sizeof(line), "Lon: %.5f", snapshot.longitude);
    tft.print(line);

    tft.setCursor(12, 82);
    snprintf(line, sizeof(line), "Alt: %.1fm  Spd: %.1fm/s", snapshot.altitudeM, snapshot.groundSpeedMps);
    tft.print(line);
  } else {
    snprintf(line, sizeof(line), "No fix (type %u) Sat:%u", snapshot.gpsFixType, snapshot.satellites);
    tft.print(line);
    tft.setCursor(12, 70);
    tft.print("Waiting for valid position...");
    tft.fillRect(12, 82, 296, 10, ILI9341_BLACK);
  }

  // IMU / ENV Section
  tft.setCursor(12, 120);
  if (bnoReady && !isnan(snapshot.imuHeadingDeg)) {
    snprintf(line, sizeof(line), "Head: %5.1f  Roll: %5.1f  Pitch: %5.1f", snapshot.imuHeadingDeg,
             snapshot.imuRollDeg, snapshot.imuPitchDeg);
    tft.print(line);
  } else {
    tft.print("IMU unavailable");
  }

  tft.setCursor(12, 132);
  if (bnoReady) {
    snprintf(line, sizeof(line), "Cal S:%u G:%u A:%u M:%u", snapshot.imuCalSys,
             snapshot.imuCalGyro, snapshot.imuCalAccel, snapshot.imuCalMag);
    tft.print(line);
  } else {
    tft.print("Calibration data N/A");
  }

  tft.setCursor(12, 144);
  if (bmpReady && !isnan(snapshot.temperatureC)) {
    snprintf(line, sizeof(line), "Temp: %5.1fC  Pressure: %7.2fhPa", snapshot.temperatureC, snapshot.pressureHpa);
    tft.print(line);
  } else {
    tft.print("BMP390 unavailable");
  }

  // Button Section
  tft.setCursor(12, 186);
  snprintf(line, sizeof(line), "BOOT: %s    USER: %s",
           snapshot.buttonBootPressed ? "DOWN" : "UP  ",
           snapshot.buttonUserPressed ? "DOWN" : "UP  ");
  tft.print(line);

  // LoRa Section
  tft.setCursor(12, 226);
  snprintf(line, sizeof(line), "TX:%lu RX:%lu", static_cast<unsigned long>(loraStatus.txCount),
           static_cast<unsigned long>(loraStatus.rxCount));
  tft.print(line);

  tft.setCursor(160, 226);
  snprintf(line, sizeof(line), "Last RSSI: %d", loraStatus.lastRssi);
  tft.print(line);

  tft.setCursor(12, 238);
  snprintf(line, sizeof(line), "Last TX: %lus ago", (millis() - loraStatus.lastTxMillis) / 1000UL);
  tft.print(line);

  tft.setCursor(160, 238);
  if (loraStatus.lastRxMillis > 0) {
    snprintf(line, sizeof(line), "Last RX: %lus ago", (millis() - loraStatus.lastRxMillis) / 1000UL);
  } else {
    snprintf(line, sizeof(line), "Last RX: --");
  }
  tft.print(line);

  tft.setCursor(12, 252);
  if (loraStatus.lastMessageValid) {
    snprintf(line, sizeof(line), "%s", loraStatus.lastMessage);
  } else {
    snprintf(line, sizeof(line), "Waiting for remote heartbeat...");
  }
  tft.print(line);

  tft.fillRect(12, 264, 296, 44, ILI9341_BLACK);
  tft.setCursor(12, 264);
  tft.print("Serial monitor shows full sensor log.");
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
           "FIX:%d,FIXTYPE:%u,SAT:%u,LAT:%.6f,LON:%.6f,ALT:%.1f,SPD:%.1f,TEMP:%.1f,PRES:%.1f,HEAD:%.1f,ROLL:%.1f,PITCH:%.1f,BTN:%d%d",
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
           snapshot.buttonBootPressed ? 1 : 0,
           snapshot.buttonUserPressed ? 1 : 0);
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
