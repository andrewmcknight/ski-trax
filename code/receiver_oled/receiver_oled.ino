// Receiver (OLED version) for Heltec WiFi LoRa 32 (V3)
// Prints received payloads and RSSI/SNR to the onboard OLED.
//
// Based on Heltec LoRa examples.

#include <Arduino.h>
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"

// ---------------- LoRa radio params (must match sender) ----------------
#define RF_FREQUENCY            915000000   // Hz (US915). Use 868e6 for EU, etc.
#define LORA_BANDWIDTH          0           // [0:125kHz, 1:250kHz, 2:500kHz]
#define LORA_SPREADING_FACTOR   7           // [SF7..SF12]
#define LORA_CODINGRATE         1           // [1:4/5, 2:4/6, 3:4/7, 4:4/8]
#define LORA_PREAMBLE_LENGTH    8
#define LORA_FIX_LENGTH_PAYLOAD false
#define LORA_IQ_INVERSION       false
#define LORA_SYMBOL_TIMEOUT     0

#define BUFFER_SIZE             64
static char rxpacket[BUFFER_SIZE];

// ---------------- OLED ----------------
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
static inline void VextON()  { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }
static inline void VextOFF() { pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }

static void oledPrint(const String& l1, const String& l2 = "", const String& l3 = "") {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, l1);
  if (!l2.isEmpty()) display.drawString(0, 22, l2);
  if (!l3.isEmpty()) {
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 44, l3);
  }
  display.display();
}

// ---------------- Radio state ----------------
static RadioEvents_t RadioEvents;
static volatile bool lora_idle = true;
static volatile int16_t lastRssi = 0;
static volatile int8_t  lastSnr  = 0;
static volatile uint16_t lastLen = 0;

// ---------------- Callbacks ----------------
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Radio.Sleep();
  lastRssi = rssi;
  lastSnr  = snr;
  lastLen  = size > (BUFFER_SIZE - 1) ? (BUFFER_SIZE - 1) : size;
  memcpy(rxpacket, payload, lastLen);
  rxpacket[lastLen] = '\0';

  oledPrint("RX", String(rxpacket), "RSSI " + String(lastRssi) + "  SNR " + String(lastSnr));
  Serial.println("Received");

  lora_idle = true;
}

static void OnRxTimeout(void) {
  // Optional: indicate timeout
  oledPrint("RX timeout", "listening…");
  lora_idle = true;
}

// ---------------- Setup/Loop ----------------
void setup() {
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  VextON();
  delay(100);
  display.init();
  display.setContrast(255);
  display.flipScreenVertically();
  oledPrint("Receiver booting", "LoRa init…");

  RadioEvents.RxDone    = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
                    LORA_FIX_LENGTH_PAYLOAD, 0, true, 0, 0, LORA_IQ_INVERSION, true);

  oledPrint("Receiver ready", "freq " + String(RF_FREQUENCY/1e6,0) + " MHz");
}

void loop() {
  if (lora_idle) {
    lora_idle = false;
    Radio.Rx(0); // continuous
    // oledPrint("Listening…");
  }
  Radio.IrqProcess();
}
