/*
 * Simple BMP390 + BNO055 Test
 * For Heltec WiFi LoRa 32 V3 (ESP32-S3)
 * 
 * GPIO 47 = SDA
 * GPIO 48 = SCL
 * GPIO 36 = Vext control (active HIGH)
 */

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SDA_PIN 47
#define SCL_PIN 48
#define VEXT_CTRL 36   // <-- NEW

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

bool bmp_found = false;
bool bno_found = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(500);

  // === ENABLE VEXT POWER RAIL ===
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);   // Turn on Vext 3.3V output
  delay(50); // give sensors time to power up
  // ===============================

  Serial.println("\n=== BMP390 + BNO055 Test ===\n");

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("I2C initialized on GPIO 48 (SDA) and GPIO 47 (SCL)");

  // Scan I2C bus
  Serial.println("\nScanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  Serial.print("Found ");
  Serial.print(nDevices);
  Serial.println(" device(s)\n");

  // Initialize BMP390
  Serial.println("Initializing BMP390...");
  if (bmp.begin_I2C()) {
    Serial.println("✓ BMP390 found!");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    bmp_found = true;
  } else {
    Serial.println("✗ BMP390 not found!");
  }

  // Initialize BNO055
  Serial.println("\nInitializing BNO055...");
  if (bno.begin()) {
    Serial.println("✓ BNO055 found!");
    delay(1000);
    bno.setExtCrystalUse(true);
    bno_found = true;
  } else {
    Serial.println("✗ BNO055 not found!");
    Serial.println("  Make sure PS0=LOW and PS1=LOW for I2C mode");
    Serial.println("  Default address is 0x28 (COM3=LOW)");
  }

  if (!bmp_found && !bno_found) {
    Serial.println("\n⚠ No sensors found! Check wiring.");
    while(1) delay(100);
  }

  Serial.println("\n=== Starting readings ===\n");
}

void loop() {
  Serial.println("------ Reading Sensors ------");

  // Read BMP390
  if (bmp_found) {
    if (bmp.performReading()) {
      Serial.println("\nBMP390:");
      Serial.print("  Temp:     ");
      Serial.print(bmp.temperature);
      Serial.println(" °C");
      Serial.print("  Pressure: ");
      Serial.print(bmp.pressure / 100.0);
      Serial.println(" hPa");
      float altitude_m = bmp.readAltitude(1013.25);
      float altitude_ft = altitude_m * 3.28084;
      Serial.print("  Altitude: ");
      Serial.print(altitude_ft);
      Serial.print(" ft (");
      Serial.print(altitude_m);
      Serial.println(" m)");
    } else {
      Serial.println("\nBMP390: Read failed!");
    }
  }

  // Read BNO055
  if (bno_found) {
    Serial.println("\nBNO055:");

    sensors_event_t event;
    bno.getEvent(&event);
    Serial.print("  Heading: ");
    Serial.print(event.orientation.x);
    Serial.println(" °");
    Serial.print("  Roll:    ");
    Serial.print(event.orientation.y);
    Serial.println(" °");
    Serial.print("  Pitch:   ");
    Serial.print(event.orientation.z);
    Serial.println(" °");

    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("  Cal: Sys=");
    Serial.print(system);
    Serial.print(" Gyr=");
    Serial.print(gyro);
    Serial.print(" Acc=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);

    int8_t temp = bno.getTemp();
    Serial.print("  Temp:    ");
    Serial.print(temp);
    Serial.println(" °C");
  }

  Serial.println("-----------------------------\n");
  delay(1000);
}
