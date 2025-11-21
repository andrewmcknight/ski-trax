#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <esp32-hal-ledc.h>
#include <LoRaWan_APP.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include "assets/icons/compass_20x20.h"
#include "assets/icons/pressure_20x20.h"
#include "assets/icons/satellite_22x22.h"
#include "assets/icons/bat0_44x18.h"
#include "assets/icons/bat25_44x18.h"
#include "assets/icons/bat50_44x18.h"
#include "assets/icons/bat75_44x18.h"
#include "assets/icons/bat100_44x18.h"
#include "assets/icons/altitude_44x18.h"
#include "assets/icons/distance_32x20.h"

#include "config/pins.h"
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

// --------------------------- Display & SPI ---------------------------------
constexpr uint32_t kTftSpiFrequency = 40000000UL;
constexpr uint16_t kScreenWidth = 240;
constexpr uint16_t kScreenHeight = 320;
constexpr uint16_t kTopBarHeight = 25;
constexpr uint16_t kNameBandTop = kTopBarHeight + 6;
constexpr uint16_t kNameBandHeight = 32;
constexpr uint16_t kCompassTopMargin = 8;
constexpr uint16_t kCompassBottomMargin = 4;
constexpr uint16_t kStatsRowHeight = 60;
constexpr uint16_t kCompassBgSize = 176;
constexpr uint16_t kNeedleSize = 170;
constexpr uint16_t kNeedleOffset = (kCompassBgSize - kNeedleSize) / 2;
constexpr uint16_t kMagentaKey = 0xF81F;

SPIClass tftSpi(HSPI);
Adafruit_ILI9341 tft(&tftSpi, Pins::TFT_DC, Pins::TFT_CS, Pins::TFT_RST);

// --------------------------- Sensors ---------------------------------------
Adafruit_BMP3XX bmp390;
Adafruit_BNO055 bno055(55, 0x28);
SFE_UBLOX_GNSS gnss;

// Live snapshot of sensors and derived values used by the UI and radio payloads.
struct SensorSnapshot {
  bool gpsReady = false;
  bool gpsFix = false;
  bool imuReady = false;
  bool bmpReady = false;
  double latitude = 0.0;
  double longitude = 0.0;
  float altitudeMeters = NAN;
  float groundSpeedMps = 0.0f;
  float headingDegrees = NAN;
  float imuHeading = NAN;
  uint8_t imuCalSys = 0;
  uint8_t imuCalMag = 0;
  float temperatureC = NAN;
  float pressurePa = NAN;
  uint32_t lastGpsFixMs = 0;
  uint8_t batteryPercent = 0;
};

SensorSnapshot sensorSnapshot;

// --------------------------- Compass Assets --------------------------------
struct NeedleFrame {
  const uint16_t* bitmap;
  uint16_t bearingDeg;
};

static const NeedleFrame kNeedleFrames[] = {
  {needle_000,   0},
  {needle_010,  10},
  {needle_020,  20},
  {needle_030,  30},
  {needle_040,  40},
  {needle_050,  50},
  {needle_060,  60},
  {needle_070,  70},
  {needle_080,  80},
  {needle_090,  90},
  {needle_100, 100},
  {needle_110, 110},
  {needle_120, 120},
  {needle_130, 130},
  {needle_140, 140},
  {needle_150, 150},
  {needle_160, 160},
  {needle_170, 170},
  {needle_180, 180},
  {needle_190, 190},
  {needle_200, 200},
  {needle_210, 210},
  {needle_220, 220},
  {needle_230, 230},
  {needle_240, 240},
  {needle_250, 250},
  {needle_260, 260},
  {needle_270, 270},
  {needle_280, 280},
  {needle_290, 290},
  {needle_300, 300},
  {needle_310, 310},
  {needle_320, 320},
  {needle_330, 330},
  {needle_340, 340},
  {needle_350, 350},
};
constexpr size_t kNeedleFrameCount = sizeof(kNeedleFrames) / sizeof(kNeedleFrames[0]);

// Handles positioning and partial redraw of the compass needle without repainting the background every frame.
struct CompassRenderer {
  bool backgroundDrawn = false;
  size_t activeNeedle = kNeedleFrameCount;
  int16_t bgX = 0;
  int16_t bgY = 0;
  int16_t needleX = 0;
  int16_t needleY = 0;

  void initLayout() {
    const int16_t areaTop = kNameBandTop + kNameBandHeight + kCompassTopMargin;
    const int16_t areaBottom = kScreenHeight - kStatsRowHeight - kCompassBottomMargin;
    const int16_t availHeight = areaBottom - areaTop;
    bgX = (kScreenWidth - kCompassBgSize) / 2;
    bgY = areaTop + (availHeight - kCompassBgSize) / 2;
    if (bgY < areaTop) {
      bgY = areaTop;
    }
    needleX = bgX + kNeedleOffset;
    needleY = bgY + kNeedleOffset;
  }

  void drawBackgroundOnce() {
    if (backgroundDrawn) {
      return;
    }
    initLayout();
    tft.drawRGBBitmap(bgX, bgY, compass_bg, kCompassBgSize, kCompassBgSize);
    backgroundDrawn = true;
  }

  void eraseNeedlePixels(size_t index) {
    if (index >= kNeedleFrameCount) {
      return;
    }
    tft.startWrite();
    for (uint16_t row = 0; row < kNeedleSize; ++row) {
      for (uint16_t col = 0; col < kNeedleSize; ++col) {
        const uint32_t idx = static_cast<uint32_t>(row) * kNeedleSize + col;
        uint16_t px = pgm_read_word(&(kNeedleFrames[index].bitmap[idx]));
        if (px == kMagentaKey) {
          continue;
        }
        const uint16_t bg = pgm_read_word(&(compass_bg[(row + kNeedleOffset) * kCompassBgSize + (col + kNeedleOffset)]));
        tft.writePixel(needleX + col, needleY + row, bg);
      }
    }
    tft.endWrite();
  }

  void drawNeedle(size_t index) {
    if (index >= kNeedleFrameCount) {
      return;
    }
    if (activeNeedle != index) {
      eraseNeedlePixels(activeNeedle);
    }
    tft.startWrite();
    for (uint16_t row = 0; row < kNeedleSize; ++row) {
      for (uint16_t col = 0; col < kNeedleSize; ++col) {
        const uint32_t idx = static_cast<uint32_t>(row) * kNeedleSize + col;
        uint16_t px = pgm_read_word(&(kNeedleFrames[index].bitmap[idx]));
        if (px == kMagentaKey) {
          continue;
        }
        tft.writePixel(needleX + col, needleY + row, px);
      }
    }
    tft.endWrite();
    activeNeedle = index;
  }

  void reset() {
    backgroundDrawn = false;
    activeNeedle = kNeedleFrameCount;
  }
};

CompassRenderer compassRenderer;

// --------------------------- Buttons ---------------------------------------
struct ButtonState {
  uint8_t pin = 0;
  bool pressed = false;
  bool lastReported = false;
  bool longReported = false;
  uint32_t changeMs = 0;
};

enum class ButtonId : uint8_t { Left, Center, Right };

struct ButtonEvent {
  ButtonId id;
  bool longPress;
};

// Polled button handler with debouncing, long-press detection, and a small event queue.
class ButtonInput {
 public:
  void begin() {
    initButton(btnLeft_, Pins::BUTTON_LEFT);
    initButton(btnCenter_, Pins::BUTTON_CENTER);
    initButton(btnRight_, Pins::BUTTON_RIGHT);
  }

  void poll() {
    pollButton(btnLeft_);
    pollButton(btnCenter_);
    pollButton(btnRight_);
  }

  bool popEvent(ButtonEvent* evt) {
    if (queueSize_ == 0) {
      return false;
    }
    *evt = queue_[0];
    for (size_t i = 1; i < queueSize_; ++i) {
      queue_[i - 1] = queue_[i];
    }
    --queueSize_;
    return true;
  }

 private:
  static constexpr uint32_t kLongPressMs = 600;
  static constexpr size_t kMaxQueue = 8;

  ButtonState btnLeft_{};
  ButtonState btnCenter_{};
  ButtonState btnRight_{};
  ButtonEvent queue_[kMaxQueue];
  size_t queueSize_ = 0;

  void initButton(ButtonState& state, uint8_t pin) {
    state.pin = pin;
    pinMode(pin, INPUT_PULLUP);
    state.pressed = !digitalRead(pin);
    state.lastReported = state.pressed;
    state.longReported = false;
    state.changeMs = millis();
  }

  void enqueue(ButtonId id, bool longPress) {
    if (queueSize_ >= kMaxQueue) {
      return;
    }
    queue_[queueSize_++] = ButtonEvent{id, longPress};
  }

  void pollButton(ButtonState& state) {
    bool pressed = !digitalRead(state.pin);
    if (pressed != state.pressed) {
      state.pressed = pressed;
      state.changeMs = millis();
      state.longReported = false;
    }
    const uint32_t now = millis();
    if (state.pressed && !state.longReported && (now - state.changeMs) >= kLongPressMs) {
      state.longReported = true;
      enqueue(buttonIdForPin(state.pin), true);
    }
    if (state.pressed != state.lastReported && (now - state.changeMs) > 20) {
      state.lastReported = state.pressed;
      if (!state.pressed && !state.longReported) {
        ButtonId id = buttonIdForPin(state.pin);
        enqueue(id, false);
      }
    }
  }

  static ButtonId buttonIdForPin(uint8_t pin) {
    if (pin == Pins::BUTTON_LEFT) {
      return ButtonId::Left;
    }
    if (pin == Pins::BUTTON_RIGHT) {
      return ButtonId::Right;
    }
    return ButtonId::Center;
  }
};

ButtonInput buttons;

// --------------------------- Battery Helpers -------------------------------
float batteryVoltage = 0.0f;
uint8_t batteryPercent = 0;

float readBatteryVoltage() {
  // Based on divider: 100k / (100k + 390k)
  // VBAT = Vadc * ((100 + 390) / 100) = Vadc * 4.9
  constexpr float kVoltageDividerRatio = 4.9f;

  const uint16_t raw = analogRead(Pins::BATTERY_ADC);
  const float adcVoltage = (static_cast<float>(raw) / 4095.0f) * 3.3f;
  return adcVoltage * kVoltageDividerRatio;
}

uint8_t voltageToPercent(float volts) {
  // Clamp typical Li-ion usable range
  const float clamped = constrain(volts, 3.3f, 4.2f);
  const float pct = (clamped - 3.3f) / (4.2f - 3.3f);
  return static_cast<uint8_t>(pct * 100.0f);
}

// --------------------------- UI State Machine ------------------------------
enum class AppScreen {
  Splash,
  PairChoice,
  NicknameEntry,
  HostLobby,
  JoinDiscover,
  Tracking,
};

AppScreen currentScreen = AppScreen::Splash;
bool screenDirty = true;
uint32_t splashStartMs = 0;
bool hostSelected = true;
size_t selectedSessionIndex = 0;
size_t selectedPeerIndex = 0;

// --------------------------- LoRa Networking -------------------------------
constexpr uint8_t kMaxPeers = 8;
constexpr size_t kNicknameMax = 12;
constexpr uint32_t kBeaconIntervalMs = 1000;
constexpr uint32_t kDiscoveryExpiryMs = 5000;
constexpr uint32_t kTrackUpdateIntervalMs = 2000;
constexpr uint8_t kProtocolVersion = 1;

enum class PacketType : uint8_t {
  Beacon = 1,       // Host advertises session availability and member count.
  JoinRequest = 2,  // Client asks to join a specific session.
  JoinAccept = 3,   // Host approves a join request and informs the requester.
  TrackUpdate = 4,  // Position/headings broadcast to peers during tracking.
  Heartbeat = 5,    // Reserved keepalive/health message for future use.
};

// Small fixed header shared by all packet types for validation and routing.
struct PacketHeader {
  uint8_t magic0 = 'S';
  uint8_t magic1 = 'T';
  uint8_t version = kProtocolVersion;
  PacketType type = PacketType::Beacon;
  uint32_t sessionId = 0;
  uint32_t senderId = 0;
};
static_assert(sizeof(PacketHeader) == 12, "PacketHeader must stay compact");

// Broadcast advertisement from a host so others can discover active sessions.
struct BeaconInfo {
  uint32_t sessionId = 0;
  uint32_t hostId = 0;
  char hostName[kNicknameMax + 1] = {0};
  uint8_t memberCount = 0;
  int16_t rssi = 0;
  uint32_t lastSeenMs = 0;
};

// State we track for each peer we are following in a session.
struct PeerState {
  uint32_t deviceId = 0;
  char nickname[kNicknameMax + 1] = {0};
  double latitude = 0.0;
  double longitude = 0.0;
  float altitudeMeters = 0.0f;
  float headingDeg = NAN;
  uint8_t batteryPercent = 0;
  uint32_t lastUpdateMs = 0;
};

// Overall session membership and role tracking.
struct SessionState {
  bool isHost = false;
  bool active = false;
  uint32_t sessionId = 0;
  char hostName[kNicknameMax + 1] = {0};
  PeerState peers[kMaxPeers];
  size_t peerCount = 0;
};

SessionState sessionState;
uint32_t pendingAcceptDeviceId = 0;
uint8_t pendingAcceptRetries = 0;
uint32_t lastAcceptSendMs = 0;
BeaconInfo discoveredSessions[kMaxPeers];
size_t discoveredCount = 0;

uint32_t deviceId = 0;
char nickname[kNicknameMax + 1] = "SKIER";
bool joinRequestPending = false;
uint32_t pendingJoinSessionId = 0;

enum class AppScreen;
void setScreen(AppScreen next);
void sendJoinAccept(uint32_t targetDeviceId);
extern bool screenDirty;

static RadioEvents_t RadioEvents;
typedef enum {
  LOWPOWER,
  STATE_RX,
  STATE_TX,
} States_t;
States_t radioState = STATE_RX;
bool radioPendingTx = false;
uint8_t txBuffer[160];
uint16_t txLength = 0;
uint32_t lastBeaconSend = 0;
uint32_t lastTrackSend = 0;
uint32_t lastRadioRxMs = 0;

// -------------- LoRa config ----------------
constexpr uint32_t RF_FREQUENCY = 915000000UL; 
// Carrier frequency in Hz. 915 MHz is the ISM band used in North America.

constexpr int8_t TX_OUTPUT_POWER = 14;
// Transmit power in dBm. Higher = longer range, more battery use.
// 14 dBm (~25 mW) is typical and legal for US LoRa modules.

constexpr uint8_t LORA_BANDWIDTH = 0;
// LoRa bandwidth index. 0 = 125 kHz, 1 = 250 kHz, 2 = 500 kHz.
// Lower bandwidth = better range, slower data rate.

constexpr uint8_t LORA_SPREADING_FACTOR = 7;
// Spreading factor (SF7–SF12). Higher SF = more range & robustness, but slower.
// SF7 is fast and commonly used for mobile devices.

constexpr uint8_t LORA_CODINGRATE = 1;
// Error coding rate. 1 = 4/5, 2 = 4/6, 3 = 4/7, 4 = 4/8.
// Higher CR = more redundancy, better reliability, slower throughput.

constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;
// Number of preamble symbols before the payload. Affects detection reliability.
// 8 is standard; longer improves detection in noisy environments at cost of airtime.

constexpr uint16_t LORA_SYMBOL_TIMEOUT = 0;
// Symbol timeout for Rx window. 0 = no timeout (continuous receive).
// Used mostly in LoRaWAN; usually left at 0 for point-to-point.

constexpr bool LORA_FIX_LENGTH_PAYLOAD_ON = false;
// false = variable-length packets (normal).
// true = fixed-length packets (rarely used unless protocol demands it).

constexpr bool LORA_IQ_INVERSION_ON = false;
// IQ inversion toggles the LoRa modulation baseband phase.
// false = standard LoRa packets.
// true = used only for special networks (e.g., LoRaWAN downlinks or sniffing).

void onTxDone();
void onTxTimeout();
void onRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr);

void setupRadio() {
  RadioEvents.TxDone = onTxDone;
  RadioEvents.TxTimeout = onTxTimeout;
  RadioEvents.RxDone = onRxDone;

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
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
  radioState = STATE_RX;
  Radio.Rx(0);
}

// Write a common packet header into the provided buffer.
bool encodeHeader(uint8_t* buffer, size_t capacity, PacketType type, uint32_t sessionId) {
  if (capacity < sizeof(PacketHeader)) {
    return false;
  }
  PacketHeader hdr;
  hdr.type = type;
  hdr.sessionId = sessionId;
  hdr.senderId = deviceId;
  memcpy(buffer, &hdr, sizeof(PacketHeader));
  return true;
}

// Validate header contents and unpack it into the provided struct.
bool decodeHeader(const uint8_t* data, size_t size, PacketHeader* out) {
  if (size < sizeof(PacketHeader)) {
    return false;
  }
  memcpy(out, data, sizeof(PacketHeader));
  if (out->magic0 != 'S' || out->magic1 != 'T') {
    return false;
  }
  if (out->version != kProtocolVersion) {
    return false;
  }
  return true;
}

// Stage bytes for transmission the next time the radio is idle.
void queueTx(const uint8_t* data, uint16_t len) {
  if (len > sizeof(txBuffer)) {
    return;
  }
  memcpy(txBuffer, data, len);
  txLength = len;
  radioPendingTx = true;
}

// Processes pending TX/RX requests and advances the radio state machine.
void serviceRadio() {
  // Always process IRQs so state transitions complete even if a TX is queued.
  Radio.IrqProcess();

  if (radioPendingTx && radioState != STATE_TX) {
    Radio.Send(txBuffer, txLength);
    radioState = STATE_TX;
    radioPendingTx = false;
    return;
  }
  if (radioState == STATE_RX) {
    Radio.Rx(0);
  }
}

void onTxDone() {
  radioState = STATE_RX;
  Radio.Rx(0);
}

void onTxTimeout() {
  radioState = STATE_RX;
  Radio.Rx(0);
}

// Record a host advertisement so the join UI can display it.
void handleBeacon(const PacketHeader& header, const uint8_t* payload, uint16_t size, int16_t rssi) {
  if (size < sizeof(PacketHeader) + 1 + kNicknameMax) {
    return;
  }
  BeaconInfo info{};
  info.sessionId = header.sessionId;
  info.hostId = header.senderId;
  info.memberCount = payload[sizeof(PacketHeader)];
  strncpy(info.hostName, reinterpret_cast<const char*>(payload + sizeof(PacketHeader) + 1), kNicknameMax);
  info.hostName[kNicknameMax] = '\0';
  info.rssi = rssi;
  info.lastSeenMs = millis();

  bool updated = false;
  bool changed = false;
  for (size_t i = 0; i < discoveredCount; ++i) {
    if (discoveredSessions[i].sessionId == info.sessionId) {
      changed = (discoveredSessions[i].memberCount != info.memberCount) ||
                (discoveredSessions[i].rssi != info.rssi) ||
                (strncmp(discoveredSessions[i].hostName, info.hostName, kNicknameMax) != 0);
      discoveredSessions[i] = info;
      updated = true;
      break;
    }
  }
  if (!updated && discoveredCount < kMaxPeers) {
    discoveredSessions[discoveredCount++] = info;
    changed = true;
  }
  if (changed && currentScreen == AppScreen::JoinDiscover) {
    screenDirty = true;
  }
}

void handleJoinAccept(const PacketHeader& header, const uint8_t* payload, uint16_t size) {
  if (sessionState.active) {
    return;
  }
  if (size < sizeof(PacketHeader) + sizeof(uint32_t) + kNicknameMax) {
    return;
  }
  const uint8_t* ptr = payload + sizeof(PacketHeader);
  uint32_t targetId = 0;
  memcpy(&targetId, ptr, sizeof(uint32_t));
  ptr += sizeof(uint32_t);
  if (targetId != deviceId) {
    return;
  }
  sessionState.active = true;
  sessionState.isHost = false;
  sessionState.sessionId = header.sessionId;
  strncpy(sessionState.hostName, reinterpret_cast<const char*>(ptr), kNicknameMax);
  sessionState.hostName[kNicknameMax] = '\0';

  // Seed peer list with host so the tracking screen shows it immediately.
  sessionState.peerCount = 0;
  if (sessionState.peerCount < kMaxPeers) {
    PeerState& hostPeer = sessionState.peers[sessionState.peerCount++];
    hostPeer.deviceId = header.senderId;
    strncpy(hostPeer.nickname, sessionState.hostName, kNicknameMax);
    hostPeer.nickname[kNicknameMax] = '\0';
    hostPeer.lastUpdateMs = millis();
  }

  joinRequestPending = false;
  lastTrackSend = 0;  // Force an immediate track update from the joiner.
  setScreen(AppScreen::Tracking);
}

// Update or create a peer record when their latest GPS/heading arrives.
void handleTrackUpdate(const PacketHeader& header, const uint8_t* payload, uint16_t size) {
  if (!sessionState.active) {
    return;
  }
  // Header + lat/lon + altitude + heading + battery + nameLen
  const size_t kFixedPayload = sizeof(PacketHeader) + sizeof(double) * 2 + sizeof(float) * 2 + 2;
  if (size < kFixedPayload) {
    return;
  }
  const uint8_t* ptr = payload + sizeof(PacketHeader);
  double lat, lon;
  float alt, heading;
  memcpy(&lat, ptr, sizeof(double)); ptr += sizeof(double);
  memcpy(&lon, ptr, sizeof(double)); ptr += sizeof(double);
  memcpy(&alt, ptr, sizeof(float)); ptr += sizeof(float);
  memcpy(&heading, ptr, sizeof(float)); ptr += sizeof(float);
  uint8_t battery = *ptr++;
  uint8_t nameLen = *ptr++;
  if (ptr + nameLen > payload + size) {
    return;
  }
  char remoteName[kNicknameMax + 1] = {0};
  if (nameLen > kNicknameMax) nameLen = kNicknameMax;
  memcpy(remoteName, ptr, nameLen);

  bool found = false;
  for (size_t i = 0; i < sessionState.peerCount; ++i) {
    if (sessionState.peers[i].deviceId == header.senderId) {
      PeerState& peer = sessionState.peers[i];
      peer.latitude = lat;
      peer.longitude = lon;
      peer.altitudeMeters = alt;
      peer.headingDeg = heading;
      peer.batteryPercent = battery;
      strncpy(peer.nickname, remoteName, kNicknameMax);
      peer.nickname[kNicknameMax] = '\0';
      peer.lastUpdateMs = millis();
      found = true;
      break;
    }
  }
  if (!found && sessionState.peerCount < kMaxPeers) {
    PeerState& peer = sessionState.peers[sessionState.peerCount++];
    peer.deviceId = header.senderId;
    strncpy(peer.nickname, remoteName, kNicknameMax);
    peer.nickname[kNicknameMax] = '\0';
    peer.latitude = lat;
    peer.longitude = lon;
    peer.altitudeMeters = alt;
    peer.headingDeg = heading;
    peer.batteryPercent = battery;
    peer.lastUpdateMs = millis();
  }
}

// Hosts accept join requests and seed the peer list with the nicknames sent.
void handleJoinRequest(const PacketHeader& header, const uint8_t* payload, uint16_t size) {
  if (!sessionState.isHost) {
    return;
  }
  if (header.sessionId != sessionState.sessionId) {
    return;
  }
  const uint8_t* ptr = payload + sizeof(PacketHeader);
  if (ptr >= payload + size) {
    return;
  }
  uint8_t nameLen = *ptr++;
  if (nameLen > kNicknameMax) {
    nameLen = kNicknameMax;
  }
  if (ptr + nameLen > payload + size) {
    return;
  }
  char joinerName[kNicknameMax + 1] = {0};
  memcpy(joinerName, ptr, nameLen);
  joinerName[nameLen] = '\0';

  bool exists = false;
  for (size_t i = 0; i < sessionState.peerCount; ++i) {
    if (sessionState.peers[i].deviceId == header.senderId) {
      exists = true;
      strncpy(sessionState.peers[i].nickname, joinerName, kNicknameMax);
      sessionState.peers[i].nickname[kNicknameMax] = '\0';
      break;
    }
  }
  if (!exists && sessionState.peerCount < kMaxPeers) {
    PeerState& peer = sessionState.peers[sessionState.peerCount++];
    peer.deviceId = header.senderId;
    strncpy(peer.nickname, joinerName, kNicknameMax);
    peer.nickname[kNicknameMax] = '\0';
    peer.lastUpdateMs = millis();
  }
  pendingAcceptDeviceId = header.senderId;
  pendingAcceptRetries = 3;
  lastAcceptSendMs = 0;
  screenDirty = true;
}

// Packet dispatcher for inbound LoRa messages.
void onRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  Radio.Rx(0);
  radioState = STATE_RX;
  lastRadioRxMs = millis();
  PacketHeader header;
  if (!decodeHeader(payload, size, &header)) {
    return;
  }
  switch (header.type) {
    case PacketType::Beacon:
      handleBeacon(header, payload, size, rssi);
      break;
    case PacketType::JoinRequest:
      handleJoinRequest(header, payload, size);
      break;
    case PacketType::JoinAccept:
      handleJoinAccept(header, payload, size);
      break;
    case PacketType::TrackUpdate:
      handleTrackUpdate(header, payload, size);
      break;
    default:
      break;
  }
}

// --------------------------- Utility Functions -----------------------------
float normalizeHeading(float heading) {
  while (heading < 0.0f) {
    heading += 360.0f;
  }
  while (heading >= 360.0f) {
    heading -= 360.0f;
  }
  return heading;
}

size_t frameIndexForHeading(float heading) {
  if (std::isnan(heading)) {
    return kNeedleFrameCount;
  }
  uint16_t bucket = static_cast<uint16_t>((heading + 5.0f) / 10.0f) % kNeedleFrameCount;
  return bucket;
}

float haversineDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadiusM = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return static_cast<float>(kEarthRadiusM * c);
}

float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  double brng = atan2(y, x);
  brng = degrees(brng);
  return normalizeHeading(static_cast<float>(brng));
}

String formatDistanceImperial(float meters) {
  float feet = meters * 3.28084f;
  if (feet < 1000.0f) {
    return String(static_cast<int>(feet)) + " ft";
  }
  float miles = feet / 5280.0f;
  return String(miles, 2) + " mi";
}

String formatAltitudeDelta(float deltaMeters) {
  float feet = deltaMeters * 3.28084f;
  if (feet > 0) {
    return String("+") + String(static_cast<int>(feet)) + " ft";
  }
  return String(static_cast<int>(feet)) + " ft";
}

String formatAltitudeFeet(float meters) {
  float feet = meters * 3.28084f;
  return String(static_cast<int>(feet)) + " ft";
}

// --------------------------- Rendering Helpers -----------------------------
static int16_t baselineForTop(int16_t yTop, const char* text) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  return yTop - y1;
}

static uint16_t measureTextHeight(const char* text) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  return h;
}

static int16_t baselineForCenteredBox(int16_t boxTop, int16_t boxHeight, const char* text) {
  uint16_t textHeight = measureTextHeight(text);
  int16_t top = boxTop + (boxHeight - textHeight) / 2;
  return baselineForTop(top, text);
}

static uint8_t fitTextSize(const char* text, const GFXfont* font, uint16_t maxWidth, uint16_t maxHeight, uint8_t maxSize = 4) {
  uint8_t best = 1;
  for (uint8_t sz = 1; sz <= maxSize; ++sz) {
    tft.setFont(font);
    tft.setTextSize(sz);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    if (w <= maxWidth && h <= maxHeight) {
      best = sz;
    } else {
      break;
    }
  }
  return best;
}

static void drawTextInBoxCentered(int16_t x, int16_t y, uint16_t w, uint16_t h, const char* text,
                                  const GFXfont* font, uint16_t color, uint8_t size) {
  tft.setFont(font);
  tft.setTextSize(size);
  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);
  int16_t cx = x + (static_cast<int16_t>(w) - static_cast<int16_t>(tw)) / 2;
  int16_t baseline = baselineForCenteredBox(y, h, text);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.setCursor(cx, baseline);
  tft.print(text);
}

static void drawBitmapKeyed(const uint16_t* bitmap, uint16_t w, uint16_t h, int16_t x, int16_t y) {
  tft.startWrite();
  for (uint16_t row = 0; row < h; ++row) {
    for (uint16_t col = 0; col < w; ++col) {
      uint16_t px = pgm_read_word(&(bitmap[row * w + col]));
      if (px == kMagentaKey) continue;
      tft.writePixel(x + col, y + row, px);
    }
  }
  tft.endWrite();
}

static void drawBitmapKeyedScaled(const uint16_t* bitmap, uint16_t srcW, uint16_t srcH,
                                  int16_t dstX, int16_t dstY, uint16_t dstW, uint16_t dstH) {
  float scaleX = static_cast<float>(srcW) / static_cast<float>(dstW);
  float scaleY = static_cast<float>(srcH) / static_cast<float>(dstH);
  tft.startWrite();
  for (uint16_t dy = 0; dy < dstH; ++dy) {
    uint16_t sy = static_cast<uint16_t>(dy * scaleY);
    if (sy >= srcH) sy = srcH - 1;
    for (uint16_t dx = 0; dx < dstW; ++dx) {
      uint16_t sx = static_cast<uint16_t>(dx * scaleX);
      if (sx >= srcW) sx = srcW - 1;
      uint16_t px = pgm_read_word(&(bitmap[sy * srcW + sx]));
      if (px == kMagentaKey) continue;
      tft.writePixel(dstX + dx, dstY + dy, px);
    }
  }
  tft.endWrite();
}

struct TopBarSnapshot {
  bool showCompass = false;
  bool showPressure = false;
  bool showGps = false;
  uint8_t hour = 255;
  uint8_t minute = 255;
  uint8_t batteryBucket = 0;
};

TopBarSnapshot lastTopBar{};
bool topBarValid = false;

void drawTopBar(bool force = false) {
  TopBarSnapshot next{};
  next.showCompass = sensorSnapshot.imuReady;
  next.showPressure = sensorSnapshot.bmpReady;
  next.showGps = sensorSnapshot.gpsReady;
  next.batteryBucket = 75;  // Force 75% icon while ADC is unreliable.
  if (sensorSnapshot.gpsFix) {
    next.hour = gnss.getHour();
    next.minute = gnss.getMinute();
  }

  if (!force && topBarValid && memcmp(&next, &lastTopBar, sizeof(next)) == 0) {
    return;
  }
  lastTopBar = next;
  topBarValid = true;

  tft.fillRect(0, 0, kScreenWidth, kTopBarHeight, ILI9341_BLACK);
  tft.drawFastHLine(0, kTopBarHeight - 1, kScreenWidth, ILI9341_WHITE);

  const int16_t iconY = (kTopBarHeight - 20) / 2;
  int16_t iconX = 4;
  if (next.showCompass) {
    for (uint16_t y = 0; y < 20; ++y) {
      for (uint16_t x = 0; x < 20; ++x) {
        uint16_t px = pgm_read_word(&(compass_20x20[y * 20 + x]));
        if (px != kMagentaKey) tft.drawPixel(iconX + x, iconY + y, px);
      }
    }
    iconX += 22;
  }
  if (next.showPressure) {
    for (uint16_t y = 0; y < 20; ++y) {
      for (uint16_t x = 0; x < 20; ++x) {
        uint16_t px = pgm_read_word(&(pressure_20x20[y * 20 + x]));
        if (px != kMagentaKey) tft.drawPixel(iconX + x, iconY + y, px);
      }
    }
    iconX += 22;
  }
  if (next.showGps) {
    const uint16_t satW = 22;
    const uint16_t satH = 22;
    const int16_t satY = (kTopBarHeight - satH) / 2;
    for (uint16_t y = 0; y < satH; ++y) {
      for (uint16_t x = 0; x < satW; ++x) {
        uint16_t px = pgm_read_word(&(satellite_22x22[y * satW + x]));
        if (px != kMagentaKey) tft.drawPixel(iconX + x, satY + y, px);
      }
    }
    iconX += satW + 2;
  }

  const uint16_t batteryW = 44;
  const uint16_t batteryH = 18;
  const int16_t batteryX = kScreenWidth - batteryW - 4;
  const int16_t batteryY = (kTopBarHeight - batteryH) / 2;
  const uint16_t* batIcon = bat75_44x18;
  drawBitmapKeyed(batIcon, batteryW, batteryH, batteryX, batteryY);

  char timeText[6];
  if (next.hour != 255) {
    snprintf(timeText, sizeof(timeText), "%02u:%02u", next.hour, next.minute);
  } else {
    strncpy(timeText, "--:--", sizeof(timeText));
  }
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(timeText, 0, 0, &x1, &y1, &w, &h);
  int16_t timeX = (kScreenWidth - static_cast<int16_t>(w)) / 2;
  int16_t timeY = baselineForCenteredBox(1, kTopBarHeight - 2, timeText);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(timeX, timeY);
  tft.print(timeText);
}

void drawPairChoice() {
  tft.fillRect(0, kTopBarHeight, kScreenWidth, kScreenHeight - kTopBarHeight, ILI9341_BLACK);
  const int16_t margin = 12;
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);

  const char* title = "Choose Mode";
  uint16_t titleBoxW = kScreenWidth - margin * 2;
  uint16_t titleBoxH = 28;
  drawTextInBoxCentered(margin, kTopBarHeight + 8, titleBoxW, titleBoxH, title, &FreeSansBold12pt7b, ILI9341_YELLOW, 1);

  const uint16_t optionBoxW = kScreenWidth - margin * 2;
  const uint16_t optionBoxH = 40;
  const int16_t hostBoxY = kTopBarHeight + 56;
  const int16_t joinBoxY = hostBoxY + optionBoxH + 16;

  tft.drawRect(margin, hostBoxY, optionBoxW, optionBoxH, ILI9341_WHITE);
  tft.drawRect(margin, joinBoxY, optionBoxW, optionBoxH, ILI9341_WHITE);

  const uint16_t activeColor = ILI9341_WHITE;
  const uint16_t inactiveColor = ILI9341_LIGHTGREY;
  const bool hostActive = hostSelected;

  uint16_t hostBoxColor = hostActive ? activeColor : inactiveColor;
  uint16_t joinBoxColor = hostActive ? inactiveColor : activeColor;

  if (hostActive) {
    tft.drawRect(margin - 2, hostBoxY - 2, optionBoxW + 4, optionBoxH + 4, ILI9341_WHITE);
  } else {
    tft.drawRect(margin - 2, joinBoxY - 2, optionBoxW + 4, optionBoxH + 4, ILI9341_WHITE);
  }

  drawTextInBoxCentered(margin, hostBoxY, optionBoxW, optionBoxH, "HOST Session", &FreeSansBold12pt7b, hostBoxColor, 1);
  drawTextInBoxCentered(margin, joinBoxY, optionBoxW, optionBoxH, "JOIN Session", &FreeSansBold12pt7b, joinBoxColor, 1);
}

static constexpr size_t kInitialCount = 3;
char initials[kInitialCount + 1] = {'A', 'A', 'A', '\0'};
size_t initialCursor = 0;

void drawInitialsEntry() {
  tft.fillRect(0, kTopBarHeight, kScreenWidth, kScreenHeight - kTopBarHeight, ILI9341_BLACK);
  const int16_t margin = 12;
  drawTextInBoxCentered(margin, kTopBarHeight + 12, kScreenWidth - margin * 2, 24,
                        "Enter Initials", &FreeSansBold12pt7b, ILI9341_YELLOW, 1);

  const uint16_t boxW = 50;
  const uint16_t boxH = 60;
  const uint16_t gap = 14;
  const uint16_t totalWidth = boxW * kInitialCount + gap * (kInitialCount - 1);
  const int16_t startX = (kScreenWidth - totalWidth) / 2;
  const int16_t boxY = kTopBarHeight + 56;
  // Use a single computed size so every glyph matches scale.
  uint8_t letterSize = fitTextSize("W", &FreeSansBold18pt7b, boxW - 8, boxH - 8, 3);

  for (size_t i = 0; i < kInitialCount; ++i) {
    int16_t x = startX + i * (boxW + gap);
    tft.drawRect(x, boxY, boxW, boxH, ILI9341_WHITE);
    if (i == initialCursor) {
      tft.drawRect(x - 2, boxY - 2, boxW + 4, boxH + 4, ILI9341_WHITE);
    }
    char letter[2] = {initials[i], '\0'};
    drawTextInBoxCentered(x, boxY, boxW, boxH, letter, &FreeSansBold18pt7b, ILI9341_WHITE, letterSize);
  }

  const int16_t instructionsTop = boxY + boxH + 26;
  drawTextInBoxCentered(margin, instructionsTop, kScreenWidth - margin * 2, 16,
                        "L/R: change letter", &FreeSans9pt7b, ILI9341_LIGHTGREY, 1);
  drawTextInBoxCentered(margin, instructionsTop + 16, kScreenWidth - margin * 2, 16,
                        "HOLD L/R: move cursor", &FreeSans9pt7b, ILI9341_LIGHTGREY, 1);
  drawTextInBoxCentered(margin, instructionsTop + 32, kScreenWidth - margin * 2, 16,
                        "OK: confirm", &FreeSans9pt7b, ILI9341_LIGHTGREY, 1);
}

void drawHostLobby() {
  tft.fillRect(0, kTopBarHeight, kScreenWidth, kScreenHeight - kTopBarHeight, ILI9341_BLACK);
  tft.setTextSize(1);

  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  int16_t titleBaseline = baselineForTop(kTopBarHeight + 30, "Hosting");
  tft.setCursor(20, titleBaseline);
  tft.print("Hosting as ");
  tft.print(nickname);

  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
  int16_t sessionBaseline = baselineForTop(kTopBarHeight + 70, "Session");
  tft.setCursor(20, sessionBaseline);
  tft.print("Session ID: ");
  tft.print(sessionState.sessionId, HEX);

  int16_t waitingBaseline = baselineForTop(kTopBarHeight + 95, "Waiting");
  tft.setCursor(20, waitingBaseline);
  tft.print("Waiting for friends...");

  int16_t listY = kTopBarHeight + 125;
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  for (size_t i = 0; i < sessionState.peerCount; ++i) {
    tft.setCursor(24, listY);
    tft.print("- ");
    tft.print(sessionState.peers[i].nickname);
    listY += 18;
  }

  tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
  int16_t hintBaseline = baselineForTop(kScreenHeight - 30, "CENTER to start tracking");
  tft.setCursor(20, hintBaseline);
  tft.print("CENTER to start tracking");
}

void drawDiscovery() {
  tft.fillRect(0, kTopBarHeight, kScreenWidth, kScreenHeight - kTopBarHeight, ILI9341_BLACK);
  tft.setTextSize(1);

  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  int16_t titleBaseline = baselineForTop(kTopBarHeight + 30, "Join Session");
  tft.setCursor(20, titleBaseline);
  tft.print("Join Session");

  tft.setFont(&FreeSans9pt7b);
  if (joinRequestPending) {
    tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
    int16_t pendingBaseline = baselineForTop(kTopBarHeight + 90, "Waiting");
    tft.setCursor(20, pendingBaseline);
    tft.print("Waiting for host to accept...");
    return;
  }

  if (discoveredCount == 0) {
    tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
    int16_t scanningBaseline = baselineForTop(kTopBarHeight + 90, "Scanning");
    tft.setCursor(20, scanningBaseline);
    tft.print("Scanning...");
    return;
  }

  if (selectedSessionIndex >= discoveredCount) {
    selectedSessionIndex = 0;
  }
  const BeaconInfo& info = discoveredSessions[selectedSessionIndex];

  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  int16_t hostBaseline = baselineForTop(kTopBarHeight + 90, info.hostName);
  tft.setCursor(20, hostBaseline);
  tft.print(info.hostName);
  tft.print(" (");
  tft.print(info.memberCount + 1);
  tft.print(")");

  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
  int16_t rssiBaseline = baselineForTop(kTopBarHeight + 120, "RSSI");
  tft.setCursor(20, rssiBaseline);
  tft.print("RSSI: ");
  tft.print(info.rssi);
  tft.print(" dBm");

  int16_t hintBaseline = baselineForTop(kScreenHeight - 40, "LEFT/RIGHT cycle, CENTER join");
  tft.setCursor(20, hintBaseline);
  tft.print("LEFT/RIGHT cycle, CENTER join");
}

struct TrackingViewState {
  char mainName[kNicknameMax + 1] = {0};
  char prevName[kNicknameMax + 1] = {0};
  char nextName[kNicknameMax + 1] = {0};
  bool hasPrev = false;
  bool hasNext = false;
  bool hasPeer = false;
  bool metricsReady = false;
  bool peerAltitudeAvailable = false;
  bool altitudeDeltaAvailable = false;
  uint32_t peerId = 0;
  size_t peerIndex = 0;
  size_t peerCount = 0;
  float distanceMeters = NAN;
  float altitudeDelta = NAN;
  float peerAltitudeMeters = NAN;
  uint32_t lastUpdateSec = 0;
};

TrackingViewState lastTrackingView{};
bool trackingViewValid = false;
bool trackingLayoutDrawn = false;

static bool trackingStateChanged(const TrackingViewState& state) {
  if (!trackingViewValid) return true;
  if (state.hasPeer != lastTrackingView.hasPeer ||
      state.metricsReady != lastTrackingView.metricsReady ||
      state.peerAltitudeAvailable != lastTrackingView.peerAltitudeAvailable ||
      state.altitudeDeltaAvailable != lastTrackingView.altitudeDeltaAvailable ||
      state.peerId != lastTrackingView.peerId ||
      state.peerIndex != lastTrackingView.peerIndex ||
      state.peerCount != lastTrackingView.peerCount ||
      state.lastUpdateSec != lastTrackingView.lastUpdateSec ||
      (std::isnan(state.distanceMeters) != std::isnan(lastTrackingView.distanceMeters)) ||
      (std::isnan(state.altitudeDelta) != std::isnan(lastTrackingView.altitudeDelta)) ||
      (!std::isnan(state.distanceMeters) && fabs(state.distanceMeters - lastTrackingView.distanceMeters) > 1.0f) ||
      (!std::isnan(state.altitudeDelta) && fabs(state.altitudeDelta - lastTrackingView.altitudeDelta) > 1.0f) ||
      (!std::isnan(state.peerAltitudeMeters) && fabs(state.peerAltitudeMeters - lastTrackingView.peerAltitudeMeters) > 1.0f) ||
      strcmp(state.mainName, lastTrackingView.mainName) != 0 ||
      strcmp(state.prevName, lastTrackingView.prevName) != 0 ||
      strcmp(state.nextName, lastTrackingView.nextName) != 0) {
    return true;
  }
  return false;
}

static void drawSideArrow(bool leftSide, int16_t centerY) {
  const int16_t arrowW = 18;
  const int16_t arrowH = 12;
  if (leftSide) {
    tft.fillTriangle(6, centerY, 6 + arrowW, centerY - arrowH / 2, 6 + arrowW, centerY + arrowH / 2, ILI9341_WHITE);
  } else {
    int16_t x = kScreenWidth - 6;
    tft.fillTriangle(x, centerY, x - arrowW, centerY - arrowH / 2, x - arrowW, centerY + arrowH / 2, ILI9341_WHITE);
  }
}

static void drawStatBox(int16_t boxX, int16_t boxWidth, const uint16_t* icon, uint16_t iconW, uint16_t iconH,
                        const char* valueText, const char* subtitle) {
  const int16_t statsTop = kScreenHeight - kStatsRowHeight;
  const uint16_t maxIconH = 28;
  const uint16_t maxIconW = boxWidth / 2;
  float aspect = static_cast<float>(iconW) / static_cast<float>(iconH);
  uint16_t targetIconH = maxIconH;
  uint16_t targetIconW = static_cast<uint16_t>(targetIconH * aspect);
  if (targetIconW > maxIconW) {
    targetIconW = maxIconW;
    targetIconH = static_cast<uint16_t>(targetIconW / aspect);
  }
  drawBitmapKeyedScaled(icon, iconW, iconH, boxX, statsTop + 2, targetIconW, targetIconH);

  const int16_t textX = boxX + targetIconW + 6;
  const uint16_t textWidth = boxWidth - targetIconW - 6;
  uint8_t valueSize = fitTextSize(valueText, &FreeSansBold12pt7b, textWidth, 24, 3);
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(valueSize);
  int16_t valueBaseline = statsTop + targetIconH / 2 + 12;
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(textX, valueBaseline);
  tft.print(valueText);

  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
  int16_t subtitleBaseline = statsTop + targetIconH + 26;
  drawTextInBoxCentered(boxX, subtitleBaseline - 14, boxWidth, 20, subtitle, &FreeSans9pt7b, ILI9341_LIGHTGREY, 1);
}

void drawTracking(bool force) {
  const uint32_t nowMs = millis();
  TrackingViewState state{};
  strncpy(state.mainName, "WAITING", kNicknameMax);
  state.mainName[kNicknameMax] = '\0';
  state.peerCount = sessionState.peerCount;

  if (sessionState.peerCount > 0) {
    if (selectedPeerIndex >= sessionState.peerCount) {
      selectedPeerIndex = 0;
    }
    PeerState& peer = sessionState.peers[selectedPeerIndex];
    state.hasPeer = true;
    state.peerId = peer.deviceId;
    state.peerIndex = selectedPeerIndex;
    strncpy(state.mainName, peer.nickname, kNicknameMax);
    state.mainName[kNicknameMax] = '\0';

    if (!std::isnan(peer.altitudeMeters)) {
      state.peerAltitudeAvailable = true;
      state.peerAltitudeMeters = peer.altitudeMeters;
    }

    if (sessionState.peerCount > 1) {
      size_t prevIdx = (selectedPeerIndex + sessionState.peerCount - 1) % sessionState.peerCount;
      size_t nextIdx = (selectedPeerIndex + 1) % sessionState.peerCount;
      if (prevIdx != selectedPeerIndex) {
        state.hasPrev = true;
        strncpy(state.prevName, sessionState.peers[prevIdx].nickname, kNicknameMax);
        state.prevName[kNicknameMax] = '\0';
      }
      if (nextIdx != selectedPeerIndex) {
        state.hasNext = true;
        strncpy(state.nextName, sessionState.peers[nextIdx].nickname, kNicknameMax);
        state.nextName[kNicknameMax] = '\0';
      }
    }

    if (sensorSnapshot.gpsFix && !std::isnan(peer.latitude) && !std::isnan(peer.longitude)) {
      state.metricsReady = true;
      state.distanceMeters = haversineDistanceMeters(sensorSnapshot.latitude, sensorSnapshot.longitude,
                                                     peer.latitude, peer.longitude);
    }
    if (!std::isnan(sensorSnapshot.altitudeMeters) && state.peerAltitudeAvailable) {
      state.altitudeDeltaAvailable = true;
      state.altitudeDelta = peer.altitudeMeters - sensorSnapshot.altitudeMeters;
    }
    if (peer.lastUpdateMs > 0) {
      state.lastUpdateSec = (nowMs - peer.lastUpdateMs) / 1000;
    }
  }

  bool layoutChange = !trackingViewValid ||
                      state.hasPeer != lastTrackingView.hasPeer ||
                      state.peerId != lastTrackingView.peerId ||
                      state.peerIndex != lastTrackingView.peerIndex ||
                      state.peerCount != lastTrackingView.peerCount ||
                      strcmp(state.mainName, lastTrackingView.mainName) != 0 ||
                      strcmp(state.prevName, lastTrackingView.prevName) != 0 ||
                      strcmp(state.nextName, lastTrackingView.nextName) != 0;

  bool statsChange = trackingStateChanged(state);

  if (force || !trackingLayoutDrawn || layoutChange) {
    tft.fillRect(0, kTopBarHeight, kScreenWidth, kScreenHeight - kTopBarHeight, ILI9341_BLACK);
    compassRenderer.reset();
    trackingLayoutDrawn = true;

    // Name band
    constexpr uint16_t kSideTouchWidth = 56;
    const uint16_t nameCenterBoxW = kScreenWidth - 2 * kSideTouchWidth;
    const int16_t nameCenterBoxX = kSideTouchWidth;
    const int16_t nameCenterBoxY = kNameBandTop;
    uint8_t nameSize = fitTextSize(state.mainName, &FreeSansBold12pt7b, nameCenterBoxW, kNameBandHeight, 4);
    drawTextInBoxCentered(nameCenterBoxX, nameCenterBoxY, nameCenterBoxW, kNameBandHeight,
                          state.mainName, &FreeSansBold12pt7b, ILI9341_WHITE, nameSize);

    const int16_t sideCenterY = kNameBandTop + kNameBandHeight / 2;
    if (state.hasPrev) {
      drawSideArrow(true, sideCenterY);
      const int16_t textBoxX = 6 + 18 + 2;
      const uint16_t textBoxW = kSideTouchWidth - 18 - 8;
      uint8_t sideSize = fitTextSize(state.prevName, &FreeSansBold9pt7b, textBoxW, kNameBandHeight, 3);
      drawTextInBoxCentered(textBoxX, kNameBandTop, textBoxW, kNameBandHeight,
                            state.prevName, &FreeSansBold9pt7b, ILI9341_WHITE, sideSize);
    }
    if (state.hasNext) {
      drawSideArrow(false, sideCenterY);
      const int16_t textBoxX = kScreenWidth - kSideTouchWidth + 6;
      const uint16_t textBoxW = kSideTouchWidth - 18 - 8;
      uint8_t sideSize = fitTextSize(state.nextName, &FreeSansBold9pt7b, textBoxW, kNameBandHeight, 3);
      drawTextInBoxCentered(textBoxX, kNameBandTop, textBoxW, kNameBandHeight,
                            state.nextName, &FreeSansBold9pt7b, ILI9341_WHITE, sideSize);
    }

    // Stats row divider
    tft.drawFastHLine(0, kScreenHeight - kStatsRowHeight - 2, kScreenWidth, ILI9341_WHITE);
  }

  compassRenderer.drawBackgroundOnce();
  float heading = sensorSnapshot.imuHeading;
  size_t frameIdx = frameIndexForHeading(heading);
  if (frameIdx < kNeedleFrameCount) {
    compassRenderer.drawNeedle(frameIdx);
  }

  if (force || statsChange) {
    char distanceText[20] = "--";
    char altitudeText[20] = "--";
    char subtitleLeft[32] = "(--s ago)";
    char subtitleRight[32] = "(--s ago)";

    if (state.metricsReady && !std::isnan(state.distanceMeters)) {
      String distStr = formatDistanceImperial(state.distanceMeters);
      distStr.toCharArray(distanceText, sizeof(distanceText));
    }
    if (state.altitudeDeltaAvailable && !std::isnan(state.altitudeDelta)) {
      String altStr = formatAltitudeDelta(state.altitudeDelta);
      altStr.toCharArray(altitudeText, sizeof(altitudeText));
    } else if (state.peerAltitudeAvailable && !std::isnan(state.peerAltitudeMeters)) {
      String altStr = formatAltitudeFeet(state.peerAltitudeMeters);
      altStr.toCharArray(altitudeText, sizeof(altitudeText));
    }
    if (state.lastUpdateSec > 0) {
      snprintf(subtitleLeft, sizeof(subtitleLeft), "(%lus ago)", static_cast<unsigned long>(state.lastUpdateSec));
      snprintf(subtitleRight, sizeof(subtitleRight), "(%lus ago)", static_cast<unsigned long>(state.lastUpdateSec));
    }

    const int16_t statsTop = kScreenHeight - kStatsRowHeight;
    const int16_t statPadding = 10;
    const uint16_t statBoxWidth = (kScreenWidth - statPadding * 3) / 2;
    const int16_t leftBoxX = statPadding;
    const int16_t rightBoxX = leftBoxX + statBoxWidth + statPadding;

    tft.fillRect(0, statsTop, kScreenWidth, kStatsRowHeight, ILI9341_BLACK);
    tft.drawFastHLine(0, statsTop, kScreenWidth, ILI9341_WHITE);

    drawStatBox(leftBoxX, statBoxWidth, altitude_44x18, 44, 18, altitudeText, subtitleLeft);
    drawStatBox(rightBoxX, statBoxWidth, distance_32x20, 32, 20, distanceText, subtitleRight);

    lastTrackingView = state;
    trackingViewValid = true;
  } else if (!trackingViewValid) {
    lastTrackingView = state;
    trackingViewValid = true;
  }
}

// --------------------------- Nickname Editing ------------------------------
void rotateInitialChar(int delta) {
  char c = initials[initialCursor];
  int ascii = static_cast<int>(c) + delta;
  if (ascii > 'Z') ascii = 'A';
  if (ascii < 'A') ascii = 'Z';
  initials[initialCursor] = static_cast<char>(ascii);
  screenDirty = true;
}

// --------------------------- Screen Management -----------------------------
void renderCurrentScreen() {
  tft.fillScreen(ILI9341_BLACK);
  topBarValid = false;
  drawTopBar(true);
  switch (currentScreen) {
    case AppScreen::Splash:
      tft.setTextSize(1);
      tft.setFont(&FreeSansBold12pt7b);
      tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
      {
        const char* splash = "Ski-Trax";
        int16_t x1, y1;
        uint16_t w, h;
        tft.getTextBounds(splash, 0, 0, &x1, &y1, &w, &h);
        int16_t splashX = (kScreenWidth - static_cast<int16_t>(w)) / 2;
        int16_t splashY = baselineForTop(kTopBarHeight + 120, splash);
        tft.setCursor(splashX, splashY);
        tft.print(splash);
      }
      tft.setFont(&FreeSans9pt7b);
      tft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
      {
        const char* tagline = "Outdoor LoRa tracker";
        int16_t taglineY = baselineForTop(kTopBarHeight + 160, tagline);
        tft.setCursor(30, taglineY);
        tft.print(tagline);
      }
      break;
    case AppScreen::PairChoice:
      drawPairChoice();
      break;
    case AppScreen::NicknameEntry:
      drawInitialsEntry();
      break;
    case AppScreen::HostLobby:
      drawHostLobby();
      break;
    case AppScreen::JoinDiscover:
      drawDiscovery();
      break;
    case AppScreen::Tracking:
      drawTracking(true);
      break;
  }
  screenDirty = false;
}

void setScreen(AppScreen next) {
  if (currentScreen == next) {
    return;
  }
  currentScreen = next;
  topBarValid = false;
  trackingLayoutDrawn = false;
  trackingViewValid = false;
  compassRenderer.reset();
  screenDirty = true;
}

// --------------------------- Sensor Polling --------------------------------
void updateBno() {
  if (!sensorSnapshot.imuReady) {
    sensorSnapshot.imuReady = bno055.begin();
    if (!sensorSnapshot.imuReady) {
      return;
    }
    bno055.setExtCrystalUse(true);
  }
  sensors_event_t event;
  bno055.getEvent(&event);
  sensorSnapshot.imuHeading = normalizeHeading(event.orientation.x);
  uint8_t calSys, calGyro, calAccel, calMag;
  bno055.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
  sensorSnapshot.imuCalSys = calSys;
  sensorSnapshot.imuCalMag = calMag;
}

void updateBmp() {
  if (!sensorSnapshot.bmpReady) {
    sensorSnapshot.bmpReady = bmp390.begin_I2C();
    if (sensorSnapshot.bmpReady) {
      bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
      bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp390.setOutputDataRate(BMP3_ODR_50_HZ);
    } else {
      return;
    }
  }
  if (bmp390.performReading()) {
    sensorSnapshot.temperatureC = bmp390.temperature;
    sensorSnapshot.pressurePa = bmp390.pressure;
  }
}

void updateGnss() {
  if (!sensorSnapshot.gpsReady) {
    sensorSnapshot.gpsReady = gnss.begin(Wire);
    if (sensorSnapshot.gpsReady) {
      gnss.setI2COutput(COM_TYPE_UBX);
      gnss.setNavigationFrequency(5);
      gnss.setAutoPVT(true);
    } else {
      return;
    }
  }
  if (gnss.getPVT()) {
    sensorSnapshot.gpsFix = gnss.getFixType() >= 2;
    if (sensorSnapshot.gpsFix) {
      sensorSnapshot.latitude = gnss.getLatitude() / 10000000.0;
      sensorSnapshot.longitude = gnss.getLongitude() / 10000000.0;
      sensorSnapshot.altitudeMeters = gnss.getAltitude() / 1000.0f;
      sensorSnapshot.groundSpeedMps = gnss.getGroundSpeed() / 1000.0f;
      sensorSnapshot.headingDegrees = gnss.getHeading() / 100000.0f;
      sensorSnapshot.lastGpsFixMs = millis();
    }
  }
}

void updateBattery() {
  batteryVoltage = readBatteryVoltage();
  batteryPercent = voltageToPercent(batteryVoltage);
  sensorSnapshot.batteryPercent = batteryPercent;
}

// --------------------------- Session Helpers -------------------------------
uint32_t allocateSessionId() {
  return (deviceId ^ millis()) | 0x01010101;
}

void startHosting() {
  sessionState = SessionState{};
  sessionState.isHost = true;
  sessionState.sessionId = allocateSessionId();
  strncpy(sessionState.hostName, nickname, kNicknameMax);
  sessionState.active = false;
}

void startJoining() {
  sessionState = SessionState{};
  sessionState.isHost = false;
  sessionState.active = false;
  discoveredCount = 0;
}

void sendBeacon() {
  if (!sessionState.isHost) {
    return;
  }
  uint8_t buffer[64] = {0};
  if (!encodeHeader(buffer, sizeof(buffer), PacketType::Beacon, sessionState.sessionId)) {
    return;
  }
  size_t offset = sizeof(PacketHeader);
  buffer[offset++] = static_cast<uint8_t>(sessionState.peerCount + 1);
  strncpy(reinterpret_cast<char*>(buffer + offset), nickname, kNicknameMax);
  offset += kNicknameMax;
  queueTx(buffer, offset);
}

void sendJoinRequest(uint32_t sessionId) {
  uint8_t buffer[64];
  if (!encodeHeader(buffer, sizeof(buffer), PacketType::JoinRequest, sessionId)) {
    return;
  }
  size_t offset = sizeof(PacketHeader);
  uint8_t nameLen = strlen(nickname);
  buffer[offset++] = nameLen;
  memcpy(buffer + offset, nickname, nameLen);
  offset += nameLen;
  queueTx(buffer, offset);
}

void sendJoinAccept(uint32_t targetDeviceId) {
  uint8_t buffer[64] = {0};
  if (!encodeHeader(buffer, sizeof(buffer), PacketType::JoinAccept, sessionState.sessionId)) {
    return;
  }
  size_t offset = sizeof(PacketHeader);
  memcpy(buffer + offset, &targetDeviceId, sizeof(uint32_t));
  offset += sizeof(uint32_t);
  strncpy(reinterpret_cast<char*>(buffer + offset), sessionState.hostName, kNicknameMax);
  offset += kNicknameMax;
  queueTx(buffer, offset);
}

void sendTrackUpdate() {
  if (!sessionState.active) {
    return;
  }
  uint8_t buffer[96] = {0};
  if (!encodeHeader(buffer, sizeof(buffer), PacketType::TrackUpdate, sessionState.sessionId)) {
    return;
  }
  size_t offset = sizeof(PacketHeader);
  double lat = sensorSnapshot.latitude;
  double lon = sensorSnapshot.longitude;
  if (!sensorSnapshot.gpsFix) {
    lat = NAN;
    lon = NAN;
  }
  memcpy(buffer + offset, &lat, sizeof(double));
  offset += sizeof(double);
  memcpy(buffer + offset, &lon, sizeof(double));
  offset += sizeof(double);
  float altitude = sensorSnapshot.altitudeMeters;
  memcpy(buffer + offset, &altitude, sizeof(float));
  offset += sizeof(float);
  float imuHeading = sensorSnapshot.imuHeading;
  memcpy(buffer + offset, &imuHeading, sizeof(float));
  offset += sizeof(float);
  buffer[offset++] = sensorSnapshot.batteryPercent;
  uint8_t nameLen = strlen(nickname);
  buffer[offset++] = nameLen;
  memcpy(buffer + offset, nickname, nameLen);
  offset += nameLen;
  queueTx(buffer, offset);
}

void processHostLogic() {
  const uint32_t now = millis();
  if (now - lastBeaconSend > kBeaconIntervalMs) {
    lastBeaconSend = now;
    sendBeacon();
  }
  if (pendingAcceptRetries > 0) {
    if (lastAcceptSendMs == 0 || (now - lastAcceptSendMs) > 800) {
      sendJoinAccept(pendingAcceptDeviceId);
      lastAcceptSendMs = now;
      --pendingAcceptRetries;
    }
  }
}

// Client/host tracking loop for transmitting our position regularly.
void processTrackingLogic() {
  const uint32_t now = millis();
  if (lastTrackSend == 0 || (now - lastTrackSend) > kTrackUpdateIntervalMs) {
    lastTrackSend = now;
    sendTrackUpdate();
  }
}

// Drop discovery entries that haven't been heard from recently.
void cleanupExpiredSessions() {
  const uint32_t now = millis();
  bool removed = false;
  for (size_t i = 0; i < discoveredCount; ) {
    if (now - discoveredSessions[i].lastSeenMs > kDiscoveryExpiryMs) {
      for (size_t j = i + 1; j < discoveredCount; ++j) {
        discoveredSessions[j - 1] = discoveredSessions[j];
      }
      --discoveredCount;
      removed = true;
    } else {
      ++i;
    }
  }
  if (removed && currentScreen == AppScreen::JoinDiscover) {
    screenDirty = true;
  }
}

// --------------------------- Input Handling --------------------------------
void handleButtonEvent(const ButtonEvent& evt) {
  switch (currentScreen) {
    case AppScreen::Splash:
      break;
    case AppScreen::PairChoice:
      if (evt.id == ButtonId::Left || evt.id == ButtonId::Right) {
        hostSelected = evt.id == ButtonId::Left ? true : (evt.id == ButtonId::Right ? false : hostSelected);
        screenDirty = true;
      } else if (evt.id == ButtonId::Center) {
        if (hostSelected) {
          startHosting();
        } else {
          startJoining();
        }
        memset(initials, 'A', kInitialCount);
        initials[kInitialCount] = '\0';
        initialCursor = 0;
        setScreen(AppScreen::NicknameEntry);
      }
      break;
    case AppScreen::NicknameEntry:
      if (evt.id == ButtonId::Left) {
        if (evt.longPress) {
          initialCursor = (initialCursor == 0) ? kInitialCount - 1 : initialCursor - 1;
          screenDirty = true;
        } else {
          rotateInitialChar(-1);
        }
      } else if (evt.id == ButtonId::Right) {
        if (evt.longPress) {
          initialCursor = (initialCursor + 1) % kInitialCount;
          screenDirty = true;
        } else {
          rotateInitialChar(1);
        }
      } else if (evt.id == ButtonId::Center) {
        strncpy(nickname, initials, kInitialCount);
        nickname[kInitialCount] = '\0';
        if (sessionState.isHost) {
          setScreen(AppScreen::HostLobby);
        } else {
          setScreen(AppScreen::JoinDiscover);
        }
      }
      break;
    case AppScreen::HostLobby:
      if (evt.id == ButtonId::Center) {
        sessionState.active = true;
        lastTrackSend = 0;  // Send immediately when tracking starts.
        setScreen(AppScreen::Tracking);
      }
      break;
    case AppScreen::JoinDiscover:
      if (discoveredCount == 0) {
        break;
      }
      if (evt.id == ButtonId::Left) {
        if (selectedSessionIndex == 0) selectedSessionIndex = discoveredCount - 1;
        else --selectedSessionIndex;
        screenDirty = true;
      } else if (evt.id == ButtonId::Right) {
        selectedSessionIndex = (selectedSessionIndex + 1) % discoveredCount;
        screenDirty = true;
      } else if (evt.id == ButtonId::Center) {
        if (selectedSessionIndex < discoveredCount && !joinRequestPending) {
          const BeaconInfo& info = discoveredSessions[selectedSessionIndex];
          pendingJoinSessionId = info.sessionId;
          sessionState.sessionId = info.sessionId;
          strncpy(sessionState.hostName, info.hostName, kNicknameMax);
          joinRequestPending = true;
          sendJoinRequest(info.sessionId);
          screenDirty = true;
        }
      }
      break;
    case AppScreen::Tracking:
      if (evt.id == ButtonId::Left) {
        if (sessionState.peerCount == 0) break;
        if (selectedPeerIndex == 0) selectedPeerIndex = sessionState.peerCount - 1;
        else --selectedPeerIndex;
      } else if (evt.id == ButtonId::Right) {
        if (sessionState.peerCount == 0) break;
        selectedPeerIndex = (selectedPeerIndex + 1) % sessionState.peerCount;
      }
      break;
  }
}

// --------------------------- Setup & Loop ----------------------------------
void enablePeripherals() {
  pinMode(Pins::VEXT_CTRL, OUTPUT);
  digitalWrite(Pins::VEXT_CTRL, LOW);
  delay(50);
}

void setupBacklight() {
  ledcSetup(0, 5000, 8);
  ledcAttachPin(Pins::TFT_BACKLIGHT, 0);
  ledcWrite(0, 220);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  deviceId = (uint32_t)(esp_random());
  enablePeripherals();
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  tftSpi.begin(Pins::TFT_SCK, Pins::TFT_MISO, Pins::TFT_MOSI, Pins::TFT_CS);
  tft.begin(kTftSpiFrequency);
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);
  setupBacklight();
  buttons.begin();
  setupRadio();
  splashStartMs = millis();
  screenDirty = true;
}

void loop() {
  buttons.poll();
  ButtonEvent evt;
  while (buttons.popEvent(&evt)) {
    handleButtonEvent(evt);
  }

  const uint32_t now = millis();
  if (currentScreen == AppScreen::Splash && now - splashStartMs > 2000) {
    setScreen(AppScreen::PairChoice);
  }

  static uint32_t lastSensorMs = 0;
  if (now - lastSensorMs > 100) {
    lastSensorMs = now;
    updateBno();
    updateBmp();
    updateGnss();
    updateBattery();
  }

  cleanupExpiredSessions();

  if (sessionState.isHost && (currentScreen == AppScreen::HostLobby || currentScreen == AppScreen::Tracking)) {
    processHostLogic();
  }
  if (sessionState.active && currentScreen == AppScreen::Tracking) {
    processTrackingLogic();
  }

  serviceRadio();

  if (screenDirty) {
    renderCurrentScreen();
  }

  // Incremental UI updates without full-screen redraws.
  drawTopBar();
  if (currentScreen == AppScreen::Tracking) {
    drawTracking(false);
  }
}
