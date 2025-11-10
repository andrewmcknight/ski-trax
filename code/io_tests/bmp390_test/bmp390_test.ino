/*
for HT Wireless Tracker - OLD!!!
*/

#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#define SDA_PIN 47
#define SCL_PIN 48

Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bmp.begin_I2C()) {  // Default I2C addr is 0x77 (0x76 also possible)
    Serial.println("BMP390 not found!");
    while (1)
      ;
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to read BMP390");
    return;
  }
  Serial.print("Temp: ");
  Serial.print(bmp.temperature);
  Serial.println(" °C");
  Serial.print("Pressure: ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");
  float altitude_m = bmp.readAltitude(1013.25); // ref sea-level pressure in hPa
  float altitude_ft = altitude_m * 3.28084;
  Serial.print("Altitude: ");
  Serial.print(altitude_ft);
  Serial.println(" ft");
  delay(500);
}
