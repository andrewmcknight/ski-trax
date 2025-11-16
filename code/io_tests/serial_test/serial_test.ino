/*
 * Heltec WiFi LoRa 32 V3 – Serial + LED Test
 */

void setup() {
  // Force USB CDC Serial
  Serial.begin(115200);
  delay(200);

  // Onboard LED (GPIO35)
  pinMode(35, OUTPUT);

  Serial.println("\n--- Serial Test Running ---");
}

void loop() {
  // Blink LED
  digitalWrite(35, HIGH);
  delay(500);
  digitalWrite(35, LOW);
  delay(500);

  // Serial output
  Serial.println("Hello from Heltec!");
}
