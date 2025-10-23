// Sender (OLED version) for Heltec WiFi LoRa 32 (V3)
// Sends a random number each time and shows it on the OLED.

#include <Arduino.h>
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "esp_system.h"       // <-- for esp_random()

// ---------------- LoRa radio params (adjust to your region) ----------------
#define RF_FREQUENCY            915e6       // Hz (US915)
#define TX_OUTPUT_POWER         14          // dBm

#define LORA_BANDWIDTH          0           // [0:125kHz, 1:250kHz, 2:500kHz]
#define LORA_SPREADING_FACTOR   7           // [SF7..SF12]
#define LORA_CODINGRATE         1           // [1:4/5, 2:4/6, 3:4/7, 4:4/8]
#define LORA_PREAMBLE_LENGTH    8           // symbols
#define LORA_FIX_LENGTH_PAYLOAD false
#define LORA_IQ_INVERSION       false
#define LORA_SYMBOL_TIMEOUT     0

#define BUFFER_SIZE             64
static char txpacket[BUFFER_SIZE];

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
  if (!l3.isEmpty()) display.drawString(0, 44, l3);
  display.display();
}

// ---------------- Radio state ----------------
static RadioEvents_t RadioEvents;
static bool lora_idle = true;
static uint32_t txCounter = 0;

// ---------------- Callbacks ----------------
static void OnTxDone(void) {
  lora_idle = true;
  // oledPrint("LoRa: TX done", "ok");
}

static void OnTxTimeout(void) {
  lora_idle = true;
  oledPrint("LoRa: TX timeout", "retrying…");
}

// ---------------- Setup/Loop ----------------
void setup() {
  // Hardware RNG (esp_random) doesn’t need seeding, so no randomSeed() needed.

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  VextON();
  delay(100);
  display.init();
  display.setContrast(255);
  display.flipScreenVertically();

  oledPrint("Sender booting", "LoRa init…");

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel((uint32_t)RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD,
                    true, 0, 0, LORA_IQ_INVERSION, 3000);

  oledPrint("Sender ready", "freq " + String(RF_FREQUENCY/1e6,0) + " MHz");
}

void loop() {
  // Send a packet every 3s (when radio is idle)
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs >= 3000 && lora_idle) {
    lastMs = now;
    lora_idle = false;

    // --- Random number each send ---
    uint32_t r = esp_random();            // full 32-bit randomness

    // Build payload (include seq for debugging)
    snprintf(txpacket, sizeof(txpacket), "%lu", r);

    // Show on OLED and send
    oledPrint("TX →", String(txpacket));
    Radio.Send((uint8_t*)txpacket, strlen(txpacket));
  }

  // Must be called regularly for radio ISR handling
  Radio.IrqProcess();
}
