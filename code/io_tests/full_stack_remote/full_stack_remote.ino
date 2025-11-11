/*
 * Companion LoRa Heartbeat Sketch
 * --------------------------------
 * Flash this on a second Heltec WiFi LoRa 32 V3 board to validate
 * end-to-end communication with the full_stack_test sketch. The remote
 * node periodically transmits its own heartbeat, listens for packets
 * from the primary node, and prints everything to the serial monitor.
 */

#include <Arduino.h>
#include "LoRaWan_APP.h"

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

#define BUTTON_PIN 0  // BOOT button doubles as user input on the Heltec board

const unsigned long TX_INTERVAL_MS = 5000;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

typedef enum {
  LOWPOWER,
  STATE_RX,
  STATE_TX
} States_t;

States_t state = STATE_TX;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

uint32_t heartbeatCounter = 0;
unsigned long lastTx = 0;
bool pendingTx = true;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Full Stack Remote LoRa Test ===\n");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

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

  state = STATE_TX;
  pendingTx = true;
}

void loop() {
  const unsigned long now = millis();

  if (now - lastTx >= TX_INTERVAL_MS) {
    pendingTx = true;
  }

  if (state == LOWPOWER && pendingTx) {
    state = STATE_TX;
  }

  switch (state) {
    case STATE_TX:
      snprintf(txpacket, sizeof(txpacket), "REMOTE,%lu,BTN:%d", static_cast<unsigned long>(heartbeatCounter++),
               digitalRead(BUTTON_PIN) == LOW ? 1 : 0);
      Radio.Send(reinterpret_cast<uint8_t *>(txpacket), strlen(txpacket));
      Serial.printf("LoRa TX -> %s\n", txpacket);
      lastTx = now;
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

void OnTxDone(void) {
  Serial.println("TX complete");
  state = STATE_RX;
}

void OnTxTimeout(void) {
  Serial.println("TX timeout, retrying soon");
  pendingTx = true;
  state = STATE_TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();

  Serial.printf("RX (RSSI %d): %s\n", rssi, rxpacket);

  // Answer quickly to keep the handshake active
  pendingTx = true;
  state = STATE_RX;
}
