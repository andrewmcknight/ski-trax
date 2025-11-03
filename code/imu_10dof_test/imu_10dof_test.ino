// --- DFRobot 10DOF IMU Test ---

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085_U.h>
#include <DFRobot_ITG3200.h>

// === Instantiate sensors ===
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(54321);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
DFRobot_ITG3200 gyro;

// GPIO pin numbers
#define SDA_PIN   45
#define SCL_PIN   46

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing 10DOF sensors...\n");

  if (!accel.begin()) Serial.println("ADXL345 not detected!");
  else Serial.println("ADXL345 ready.");

  if (!gyro.begin()) Serial.println("ITG3200 not detected!");
  else Serial.println("ITG3200 ready.");

  if (!mag.begin()) Serial.println("HMC5883L not detected!");
  else Serial.println("HMC5883L ready.");

  if (!bmp.begin()) Serial.println("BMP085 not detected!");
  else Serial.println("BMP085 ready.");

  delay(500);
}

void loop() {
  sensors_event_t accelEvent, magEvent, bmpEvent;
  accel.getEvent(&accelEvent);
  mag.getEvent(&magEvent);
  bmp.getEvent(&bmpEvent);

  float gx, gy, gz;
  gyro.readGyro(&gx, &gy, &gz); // returns °/s already

  Serial.println("---- SENSOR DATA ----");
  Serial.printf("Accel: %.2f  %.2f  %.2f  m/s²\n",
                accelEvent.acceleration.x,
                accelEvent.acceleration.y,
                accelEvent.acceleration.z);

  Serial.printf("Gyro : %.2f  %.2f  %.2f  °/s\n", gx, gy, gz);

  Serial.printf("Mag  : %.2f  %.2f  %.2f  µT\n",
                magEvent.magnetic.x,
                magEvent.magnetic.y,
                magEvent.magnetic.z);

  Serial.printf("Press: %.2f hPa | Alt: %.2f m\n\n",
                bmpEvent.pressure,
                bmp.pressureToAltitude(1013.25, bmpEvent.pressure));

  delay(500);
}
